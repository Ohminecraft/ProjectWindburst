// Blackbox: record format, the queue between the flight loop and the link task, the flash log, the service.
// Expected values are worked out by hand or decoded by the small reader in this file, which knows the format only from
// the text of the header - like the app's reader.
// Build: g++ -std=c++17 -O2 -pthread test/test_blackbox.cpp -o build/test_blackbox
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <map>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include "sim_flash.h"
#include "sim_card.h"
#include "wblog_reader.h"
#include "../src/blackbox/bb_service.h"
#include "../src/blackbox/bb_header.h"

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, ...) do { if (cond) g_pass++; else { g_fail++; printf("  FAIL line %d: %s   ", __LINE__, #cond); printf(__VA_ARGS__); printf("\n"); } } while (0)

using namespace bb;

// ------------------------------------------------------------------------------------------------------------------------------
static void t_format() {
    printf("[format] record sizes, packing, saturation, header text\n");
    Plan p;
    p.build(1u << G_GYRO, 4); CHECK(p.fast_size == 4 + 6 && p.slow_size == 4 + 23, "gyro only: 10-byte fast record; slow = time + always-on state (%u, %u)", p.fast_size, p.slow_size);
    const uint32_t dflt = (1u << G_GYRO) | (1u << G_SETPOINT) | (1u << G_PID) | (1u << G_RC) | (1u << G_MOTORS) | (1u << G_ATTITUDE) | (1u << G_TIMING) | (1u << G_ALT) | (1u << G_BATTERY) | (1u << G_GPS) | (1u << G_NAV);
    { wbp::S_blackbox d; wbp::defaults(d); CHECK(d.fields == dflt, "the settings' default groups: gyro, setpoint, PID, RC, motors, attitude, timing, altitude, battery, GPS, navigation (0x%x)", (unsigned)d.fields); }
    p.build(dflt, 4); CHECK(p.fast_size == 4 + 6 + 6 + 24 + 10 + 8 + 6 + 9 && p.slow_size == 4 + 23 + 12 + 8 + 22 + 33 + 27, "default groups, quad: %u-byte fast record, %u-byte slow record (battery: voltage, cell, current, mAh)", p.fast_size, p.slow_size);
    p.build(0xFFFFFFFFu, 8); CHECK(p.fast_size == 4 + 18 + 24 + 10 + 16 + 6 + 6 + 16 + 9 + 38 && p.slow_size == 4 + 23 + 12 + 8 + 22 + 6 + 33 + 27, "everything, 8 motors: %u / %u bytes", p.fast_size, p.slow_size);
    p.build(0xFFFFFFFFu, 20); CHECK(p.motors == 8, "motor count capped at 8");
    CHECK(p.fast_size + 1 <= 192 && p.slow_size + 1 <= 160, "the largest records fit the recorder's buffers");

    // ---- fast packing: [t][gyro x3][motor x4], little-endian, value / scale rounded
    { p.build((1u << G_GYRO) | (1u << G_MOTORS), 4);
      const float gyro[3] = {12.34f, -123.4f, 5000.0f}, motor[8] = {0.0f, 0.5f, 1.0f, -0.2f};
      FastSample s; s.t_us = 0x12345678u; s.gyro = gyro; s.motor = motor;
      uint8_t b[64]; const uint16_t n = pack_fast(p, s, b);
      int16_t g[3]; uint16_t m[4]; uint32_t t; memcpy(&t, b, 4); memcpy(g, b + 4, 6); memcpy(m, b + 10, 8);
      CHECK(n == 18 && t == 0x12345678u && g[0] == 123 && g[1] == -1234 && g[2] == 32767, "gyro 12.34 / -123.4 / 5000 deg/s at 0.1: %d %d %d (saturated)", g[0], g[1], g[2]);
      CHECK(m[0] == 0 && m[1] == 5000 && m[2] == 10000 && m[3] == 0, "motor 0 / 0.5 / 1.0 / -0.2 at 0.0001: %u %u %u %u (negative clipped to 0)", m[0], m[1], m[2], m[3]);
      const float bad[3] = {NAN, INFINITY, -INFINITY}; s.gyro = bad; pack_fast(p, s, b); memcpy(g, b + 4, 6);
      CHECK(g[0] == 0 && g[1] == 32767 && g[2] == -32768, "NaN -> 0, infinities saturate: %d %d %d", g[0], g[1], g[2]); }
    { uint8_t b[8]; uint8_t* q = b; put_int(q, T_U8, 300); put_int(q, T_U8, -3); put_int(q, T_I16, -40000); put_int(q, T_U16, 70000);
      int16_t i16; uint16_t u16; memcpy(&i16, b + 2, 2); memcpy(&u16, b + 4, 2);
      CHECK(b[0] == 255 && b[1] == 0 && i16 == -32768 && u16 == 65535 && q == b + 6, "integer types saturate at their limits");
      q = b; put_int(q, T_I32, 473977419.4); put_int(q, T_U32, 4294967295.0); int32_t i32; uint32_t u32; memcpy(&i32, b, 4); memcpy(&u32, b + 4, 4);
      CHECK(i32 == 473977419 && u32 == 4294967295u, "32-bit fields keep every digit");
      q = b; put_int(q, T_I16, 2.5); put_int(q, T_I16, -2.5); int16_t a, c; memcpy(&a, b, 2); memcpy(&c, b + 2, 2); CHECK(a == 3 && c == -3, "halves round away from zero"); }

    // ---- every fast field carries its own value: a distinct number per field, read back by NAME through the header text
    { p.build(0xFFFFFFFFu, 8);
      const float gyro[3] = {1, 2, 3}, raw[3] = {4, 5, 6}, sp[3] = {7, 8, 9}, pp[3] = {144.5f, -1.5f, 576.0f}, pi[3] = {72.4f, -236.1f, 320.0f}, pd[3] = {-31.2f, 8.8f, 0.0f}, pf[3] = {0.0f, 0.3f, -400.0f};    // PID sum units: hundreds, not fractions
      const float stick[4] = {0.1f, -0.2f, 0.3f, 0.4f}, motor[8] = {0.11f, 0.12f, 0.13f, 0.14f, 0.15f, 0.16f, 0.17f, 0.18f}, rpm[8] = {1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000}, acc[3] = {0.1f, -0.2f, -0.9f};
      FastSample s; s.t_us = 123456; s.gyro = gyro; s.gyro_raw = raw; s.setpoint = sp; s.pid_p = pp; s.pid_i = pi; s.pid_d = pd; s.pid_f = pf; s.stick = stick; s.throttle = 0.55f; s.motor = motor; s.rpm = rpm; s.acc = acc;
      s.roll = 10.5f; s.pitch = -20.25f; s.yaw = 170.75f; s.loop_us = 77; s.gap_us = 999;
      s.t_lock_us = 152; s.t_imu_us = 39; s.t_core_us = 96; s.t_est_us = 300; s.t_motor_us = 5000;
      fc::nav::Log nl; nl.acc_ned[0] = 0.25f; nl.acc_ned[1] = -1.5f; nl.acc_ned[2] = 9.0f; nl.vib = 3.21f; nl.clip = 7; nl.baro_alt = -123.45f;
      nl.gps_pos[0] = 15000.5f; nl.gps_pos[1] = -20.25f; nl.gps_pos[2] = -55.5f; nl.gps_vel[0] = 1.5f; nl.gps_vel[1] = -2.5f; nl.gps_vel[2] = 0.75f;
      nl.gps_hacc = 1.23f; nl.gps_sacc = 0.45f; nl.gps_sats = 17; nl.gps_flags = 5; nl.gps_seq = 201; s.nav = &nl;
      uint8_t b[192]; const uint16_t n = pack_fast(p, s, b);
      char txt[4096]; TextOut to(txt, sizeof(txt)); describe_fields(to, "fast", p, kFast, kFastDefs); int size = 0;
      std::string line(txt); line.pop_back(); const std::vector<RField> f = parse_fields(line, size);
      auto val = [&](const std::string& name) { for (const RField& x : f) if (x.name == name) return rd(b + x.offset, x); return (double)NAN; };
      auto vec3 = [&](const char* name, const float* want, double tol) { for (int i = 0; i < 3; i++) if (!(fabs(val(std::string(name) + "[" + char('0' + i) + "]") - want[i]) <= tol)) return false; return true; };
      CHECK(n == p.fast_size && size == n && val("t") == 123456, "all groups, 8 motors: %u bytes, layout described = layout packed", n);
      CHECK(vec3("gyro", gyro, 1e-6) && vec3("gyro_raw", raw, 1e-6) && vec3("setpoint", sp, 1e-6), "gyro, raw gyro and setpoint each land in their own columns");
      CHECK(vec3("pid_p", pp, 1e-4) && vec3("pid_i", pi, 1e-4) && vec3("pid_d", pd, 1e-4) && vec3("pid_f", pf, 1e-4), "P, I, D and F terms are not mixed up, and values of several hundred are stored as they are (0.1 steps; until 0.6.1 everything beyond 3.28 was clipped)");
      CHECK(val("t_lock") == 152 && val("t_imu") == 40 && val("t_core") == 96 && val("t_est") == 300 && val("t_motor") == 1020, "the stage times: one byte each in 4 us steps, rounded (39 -> %g), saturating at 1020 us", val("t_imu"));
      bool st = true; for (int i = 0; i < 4; i++) if (fabs(val(std::string("stick[") + char('0' + i) + "]") - stick[i]) > 1e-6) st = false;
      bool mo = true; for (int i = 0; i < 8; i++) if (fabs(val(std::string("motor[") + char('0' + i) + "]") - motor[i]) > 1e-6 || fabs(val(std::string("rpm[") + char('0' + i) + "]") - rpm[i]) > 1e-6) mo = false;
      CHECK(st && fabs(val("throttle") - 0.55) < 1e-6 && mo, "sticks, throttle, 8 motors, 8 rpm values");
      CHECK(fabs(val("roll") - 10.5) < 1e-6 && fabs(val("pitch") + 20.25) < 1e-6 && fabs(val("yaw") - 170.75) < 1e-6 && vec3("acc", acc, 1e-6) && val("loop_us") == 77 && val("gap_us") == 999, "attitude, accelerometer, timing");
      CHECK(vec3("nav_acc", nl.acc_ned, 1e-4) && fabs(val("nav_vib") - 3.21) < 1e-4 && val("nav_clip") == 7 && fabs(val("nav_baro") + 123.45) < 1e-4, "NAV_DEBUG: acceleration N/E/D, vibration, clipping, baro (%g)", val("nav_baro"));
      CHECK(vec3("nav_gps_pos", nl.gps_pos, 1e-3) && vec3("nav_gps_vel", nl.gps_vel, 1e-4) && fabs(val("nav_gps_hacc") - 1.23) < 1e-4 && fabs(val("nav_gps_sacc") - 0.45) < 1e-4
            && val("nav_gps_sats") == 17 && val("nav_gps_flags") == 5 && val("nav_gps_seq") == 201, "NAV_DEBUG: GPS position (15 km keeps its cm), velocity, accuracies, satellites, flags, fix counter");
      s.nav = nullptr; pack_fast(p, s, b); CHECK(val("nav_acc[2]") == 0 && val("nav_gps_seq") == 0, "no nav log given: the nav fields read 0"); }

    // ---- slow packing from the telemetry snapshot
    { p.build((1u << G_GPS) | (1u << G_ALT) | (1u << G_BATTERY) | (1u << G_MAG) | (1u << G_NAV), 4);
      app::Telemetry t; t.armed = true; t.failsafe = 2; t.ah_state = 4; t.batt_state = 2; t.modes = 0x80000005u; t.blockers = 0x1004; t.health = 0x0ABC; t.loop_max_us = 190; t.gap_max_us = 1250; t.loop_hz = 3987;
      t.rssi = 97; t.rc_rate = 150; t.baro_alt = -12.34f; t.z = 101.5f; t.vz = -1.27f; t.range_m = 2.345f; t.batt_v = 15.87f; t.cell_v = 3.968f;
      t.gps_fix = 3; t.gps_sats = 14; t.lat = 473977419; t.lon = -1225419876; t.alt_cm = 48820; t.gs_cms = 1234; t.course_ddeg = 2700; t.hdop = 95; t.pos_n = 12.5f; t.pos_e = -7.25f; t.vel_n = 1.5f; t.vel_e = -0.5f; t.home_dist_m = 77;
      t.mag[0] = 0.301f; t.mag[1] = -0.002f; t.mag[2] = 0.38f;
      t.alt_valid = true; t.vel_valid = true; t.flow_valid = true; t.nav_flags = 0x15; t.flow_quality = 78; t.flow_reject = 3; t.flow_fwd = -1.25f; t.flow_right = 0.37f; t.nav_roll = -12.3f; t.nav_pitch = 4.5f;
      t.nav_target_n = -1234.56f; t.nav_target_e = 87.5f; t.rescue_phase = 4;
      uint8_t b[160]; const uint16_t n = pack_slow(p, 1000000u, t, b);
      CHECK(n == p.slow_size, "slow record size %u", n);
      // read it back through the header text, like the app does
      char txt[2048]; TextOut to(txt, sizeof(txt)); describe_fields(to, "slow", p, kSlow, kSlowDefs); int size = 0;
      std::string line(txt); line.pop_back(); const std::vector<RField> f = parse_fields(line, size);
      auto val = [&](const char* name) { for (const RField& x : f) if (x.name == name) return rd(b + x.offset, x); return (double)NAN; };
      CHECK(size == n && !to.overflow, "the described layout has the packed size (%d)", size);
      CHECK(val("armed") == 1 && val("failsafe") == 2 && val("ah_state") == 4 && val("batt_state") == 2 && val("modes") == 2147483653.0 && val("blockers") == 0x1004 && val("health") == 0x0ABC, "state fields");
      CHECK(val("loop_max_us") == 190 && val("gap_max_us") == 1250 && val("loop_hz") == 3987 && val("rssi") == 97 && val("rc_rate") == 150, "timing and receiver fields");
      CHECK(fabs(val("baro_alt") + 12.34) < 1e-9 && fabs(val("est_z") - 101.5) < 1e-9 && fabs(val("est_vz") + 1.27) < 1e-9 && fabs(val("range") - 2.345) < 1e-9, "altitude fields: %.3f %.3f %.3f %.3f", val("baro_alt"), val("est_z"), val("est_vz"), val("range"));
      CHECK(fabs(val("vbat") - 15.87) < 1e-9 && fabs(val("cell") - 3.968) < 1e-9, "battery fields");
      CHECK(fabs(val("lat") - 47.3977419) < 1e-9 && fabs(val("lon") + 122.5419876) < 1e-9, "latitude / longitude to the last digit of the receiver: %.7f %.7f", val("lat"), val("lon"));
      CHECK(fabs(val("gps_alt") - 488.20) < 1e-9 && fabs(val("gps_speed") - 12.34) < 1e-9 && fabs(val("gps_course") - 270.0) < 1e-9 && fabs(val("hdop") - 0.95) < 1e-9 && val("gps_fix") == 3 && val("gps_sats") == 14, "GPS fields");
      CHECK(fabs(val("pos_n") - 12.5) < 1e-9 && fabs(val("pos_e") + 7.25) < 1e-9 && fabs(val("vel_n") - 1.5) < 1e-9 && fabs(val("vel_e") + 0.5) < 1e-9 && val("home_dist") == 77, "position estimate fields");
      CHECK(val("est_valid") == (wbp::EST_VALID_ALT | wbp::EST_VALID_VEL | wbp::EST_VALID_FLOW) && val("nav_flags") == 0x15 && val("flow_quality") == 78 && val("flow_reject") == 3 && val("rescue_phase") == 4, "navigation state fields");
      CHECK(fabs(val("flow_fwd") + 1.25) < 1e-9 && fabs(val("flow_right") - 0.37) < 1e-9 && fabs(val("nav_roll") + 12.3) < 1e-9 && fabs(val("nav_pitch") - 4.5) < 1e-9 && fabs(val("nav_target_n") + 1234.56) < 1e-9 && fabs(val("nav_target_e") - 87.5) < 1e-9,
            "flow speed, the autopilot's lean and its target: %.2f %.2f %.1f %.1f %.2f %.2f", val("flow_fwd"), val("flow_right"), val("nav_roll"), val("nav_pitch"), val("nav_target_n"), val("nav_target_e"));
      { Plan g; g.build(1u << G_GPS, 4); char t2[2048]; TextOut o2(t2, sizeof(t2)); describe_fields(o2, "slow", g, kSlow, kSlowDefs);
        CHECK(strstr(t2, " lat:") && strstr(t2, " home_dist:") && !strstr(t2, " pos_n:") && !strstr(t2, " flow_fwd:") && !strstr(t2, " nav_flags:"), "the GPS group alone: what the receiver says, not the estimate");
        g.build(1u << G_NAV, 4); TextOut o3(t2, sizeof(t2)); describe_fields(o3, "slow", g, kSlow, kSlowDefs);
        CHECK(strstr(t2, " pos_n:") && strstr(t2, " vel_e:") && strstr(t2, " flow_fwd:") && strstr(t2, " nav_target_e:") && !strstr(t2, " lat:"), "the navigation group alone: the position estimate, flow, Position Hold / Rescue"); }
      CHECK(fabs(val("mag[0]") - 0.301) < 1e-9 && fabs(val("mag[1]") + 0.002) < 1e-9 && fabs(val("mag[2]") - 0.38) < 1e-9, "compass fields"); }

    // ---- header text
    { p.build((1u << G_GYRO) | (1u << G_MOTORS) | (1u << G_ATTITUDE), 6);
      char txt[2048]; TextOut to(txt, sizeof(txt)); describe_fields(to, "fast", p, kFast, kFastDefs);
      CHECK(std::string(txt) == "fast t:u32:1e-06:s gyro[0]:i16:0.1:deg/s gyro[1]:i16:0.1:deg/s gyro[2]:i16:0.1:deg/s motor[0]:u16:0.0001: motor[1]:u16:0.0001: motor[2]:u16:0.0001: motor[3]:u16:0.0001: motor[4]:u16:0.0001: motor[5]:u16:0.0001: roll:i16:0.01:deg pitch:i16:0.01:deg yaw:u16:0.01:deg\n",
            "field list: name:type:scale:unit, six motors for a hexa: %s", txt);
      char tiny[40]; TextOut small(tiny, sizeof(tiny)); describe_fields(small, "fast", p, kFast, kFastDefs);
      CHECK(small.overflow && strlen(tiny) < sizeof(tiny), "a text buffer that is too small is reported, never overrun"); }
    CHECK(rate_divider(4000, 250) == 16 && rate_divider(4000, 500) == 8 && rate_divider(4000, 4000) == 1 && rate_divider(4000, 9000) == 1 && rate_divider(1000, 300) == 3 && rate_divider(4000, 1500) == 3 && rate_divider(8000, 10) == 255 && rate_divider(4000, 0) == 1,
          "records per second -> every N-th control cycle");
}

// ------------------------------------------------------------------------------------------------------------------------------
static void t_ring() {
    printf("[queue] order, wrap-around, overflow, two threads\n");
    { static Ring<256> r; uint8_t a[5] = {1, 2, 3, 4, 5}, out[255];
      CHECK(r.peek() == 0 && r.pop(out) == 0 && r.used() == 0, "empty");
      CHECK(r.push(a, 5) && r.push(a, 3) && r.used() == 10 && r.peek() == 5, "two records queued");
      CHECK(r.pop(out) == 5 && out[4] == 5 && r.peek() == 3 && r.pop(out) == 3 && out[2] == 3 && r.peek() == 0, "they come out in order, whole");
      // records of every length through the wrap point, many times
      bool ok = true; uint32_t seed = 1; uint8_t rec[255];
      for (int k = 0; k < 20000 && ok; k++) {
          seed = seed * 1664525u + 1013904223u; const uint8_t n = (uint8_t)(1 + (seed >> 24) % 60);
          for (int i = 0; i < n; i++) rec[i] = (uint8_t)(k + i * 7);
          if (!r.push(rec, n)) { ok = false; break; }
          if (r.peek() != n || r.pop(out) != n) { ok = false; break; }
          for (int i = 0; i < n; i++) if (out[i] != (uint8_t)(k + i * 7)) ok = false;
      }
      CHECK(ok && r.dropped() == 0, "20000 records of 1..60 bytes through a 256-byte queue arrive intact");
      int pushed = 0; while (r.push(a, 5)) pushed++;
      CHECK(pushed == 256 / 6 && r.dropped() == 1 && r.used() == (uint32_t)pushed * 6, "a full queue refuses the record and counts it (%d fitted)", pushed);
      CHECK(!r.push(a, 5) && r.dropped() == 2, "and keeps counting");
      r.pop(out); CHECK(r.push(a, 5), "room again after one was taken");
      // exactly as many bytes free as the record is long: one short (its length byte needs a place too)
      r.clear(); uint8_t big[250]; for (int i = 0; i < 250; i++) big[i] = (uint8_t)(i * 3); r.push(big, 250);
      CHECK(r.used() == 251 && !r.push(a, 5), "5 bytes free, a 5-byte record needs 6: refused");
      CHECK(r.pop(out) == 250 && memcmp(out, big, 250) == 0 && r.push(a, 5), "and the record before it is untouched");
      r.clear(); CHECK(r.used() == 0 && r.peek() == 0, "clear"); }
    { // the real situation: one thread writes (flight loop), another reads (link task)
      static Ring<4096> r; const uint32_t N = 300000; std::atomic<bool> done{false}; uint32_t got = 0, bad = 0, next = 0;
      std::thread reader([&]() {
          uint8_t out[255];
          for (;;) {
              const uint8_t n = r.pop(out);
              if (!n) { if (done.load() && !r.peek()) break; std::this_thread::yield(); continue; }
              uint32_t id; memcpy(&id, out, 4);
              if (id < next) bad++;                                    // out of order or repeated
              next = id + 1;
              if (n != 4 + id % 40) bad++;
              for (uint8_t i = 4; i < n; i++) if (out[i] != (uint8_t)(id + i)) { bad++; break; }
              got++;
          }
      });
      uint8_t rec[64];
      for (uint32_t id = 0; id < N; id++) { const uint8_t n = (uint8_t)(4 + id % 40); memcpy(rec, &id, 4); for (uint8_t i = 4; i < n; i++) rec[i] = (uint8_t)(id + i); r.push(rec, n);
          if ((id & 63u) == 63u) std::this_thread::yield(); }            // the writer is a paced loop, not a tight one: let the reader run (also on a busy PC)
      done.store(true); reader.join();
      CHECK(bad == 0 && got + r.dropped() == N && got > 300, "two threads: every record is either delivered intact and in order or counted as dropped (%u delivered, %u dropped, %u damaged)", got, r.dropped(), bad); }
}

// ------------------------------------------------------------------------------------------------------------------------------
static void t_recorder() {
    printf("[recorder] decimation, loop gap, events\n");
    static Recorder rec; Plan p; p.build((1u << G_GYRO) | (1u << G_TIMING), 4);
    const float gyro[3] = {1, 2, 3}; FastSample s; s.gyro = gyro; uint8_t out[255];
    CHECK(!rec.fast_due(250) && !rec.active(), "no session: nothing is recorded");
    rec.log_slow(0, app::Telemetry()); rec.event(0, EV_ARM); rec.note_state(0, true, 0, 0, 0, 0);
    CHECK(rec.ring().peek() == 0, "... not even events");
    rec.start(p, 8);
    int due = 0; for (int i = 0; i < 80; i++) if (rec.fast_due(250)) { due++; s.t_us = (uint32_t)i; rec.log_fast(s); }
    CHECK(due == 10, "divider 8: 10 records in 80 cycles (%d)", due);
    { int n = 0; uint32_t first = 0; while (rec.ring().pop(out)) { if (n == 0) memcpy(&first, out + 1, 4); n++; } CHECK(n == 10 && first == 7 && out[0] == 'F', "the 8th cycle is the first one logged"); }
    // the longest start-to-start time inside one decimation window
    const uint32_t gaps[8] = {250, 250, 1180, 250, 250, 250, 250, 250}; for (int i = 0; i < 8; i++) if (rec.fast_due(gaps[i])) { s.loop_us = 91; rec.log_fast(s); }
    { const uint8_t n = rec.ring().pop(out); uint16_t loop, gap; memcpy(&loop, out + 1 + 4 + 6, 2); memcpy(&gap, out + 1 + 4 + 8, 2);
      CHECK(n == 1 + p.fast_size && loop == 91 && gap == 1180, "a stalled cycle inside the window shows in gap_us (%u), the cycle's own run time in loop_us (%u)", gap, loop); }
    for (int i = 0; i < 8; i++) if (rec.fast_due(250)) rec.log_fast(s);
    { rec.ring().pop(out); uint16_t gap; memcpy(&gap, out + 1 + 4 + 8, 2); CHECK(gap == 250, "the next window starts clean (%u)", gap); }
    // events
    rec.note_state(1000, false, 0, 0x02, 1, 0);
    { int n = 0; int codes[8] = {}; while (uint8_t k = rec.ring().pop(out)) { if (n < 8) codes[n] = out[5]; n++; (void)k; }
      CHECK(n == 5 && codes[0] == EV_DISARM && codes[1] == EV_FAILSAFE && codes[2] == EV_MODES && codes[3] == EV_BATTERY && codes[4] == EV_AH_STATE, "the first state of a session is recorded in full (%d events)", n); }
    rec.note_state(2000, false, 0, 0x02, 1, 0); CHECK(rec.ring().peek() == 0, "no change, no event");
    rec.note_state(3000, true, 0, 0x03, 1, 0);
    { const uint8_t n1 = rec.ring().pop(out); uint32_t t; memcpy(&t, out + 1, 4); const bool arm = n1 == 7 && out[0] == 'E' && out[5] == EV_ARM && out[6] == 0 && t == 3000;
      const uint8_t n2 = rec.ring().pop(out); uint32_t m; memcpy(&m, out + 7, 4);
      CHECK(arm && n2 == 11 && out[5] == EV_MODES && out[6] == 4 && m == 3 && rec.ring().peek() == 0, "arming + a mode change: two events with their time and data"); }
    rec.note_state(4000, true, 2, 0x03, 3, 4);
    { int n = 0; while (rec.ring().pop(out)) n++; CHECK(n == 3, "failsafe stage, battery state, altitude-hold state: one event each (%d)", n); }
    rec.stop(); CHECK(!rec.fast_due(250), "stopped");
    rec.set_busy(true); CHECK(rec.busy(), "busy flag set"); rec.set_busy(false); rec.post_erase_done(); CHECK(rec.take_erase_done() && !rec.take_erase_done(), "the erase-done notice is delivered once");
}

// ------------------------------------------------------------------------------------------------------------------------------
// A frame as the service would write it: BB_HEADER seq 0 "WBLOG..." / BB_DATA with `n` payload bytes
static std::vector<uint8_t> header_frame(const char* text = "WBLOG 1\nsession 1\nend\n") { uint8_t b[300]; const size_t n = wbp::encode(b, sizeof(b), kMsgHeader, 0, (const uint8_t*)text, (uint16_t)strlen(text)); return std::vector<uint8_t>(b, b + n); }
static std::vector<uint8_t> data_frame(uint8_t seq, uint16_t n, uint8_t fill) { uint8_t pay[240], b[300]; for (uint16_t i = 0; i < n; i++) pay[i] = (uint8_t)(fill + i); const size_t k = wbp::encode(b, sizeof(b), kMsgData, seq, pay, n); return std::vector<uint8_t>(b, b + k); }
// writes one session of `frames` data frames, pumping like the service does. Returns the bytes written.
static std::vector<uint8_t> write_session(FlashLog& log, int frames, uint8_t fill = 1, uint16_t payload = 240) {
    std::vector<uint8_t> all;
    if (!log.start_session()) return all;
    auto put = [&](const std::vector<uint8_t>& f) { while (!log.append(f.data(), (uint32_t)f.size())) { if (!log.pump(true)) return false; } all.insert(all.end(), f.begin(), f.end()); log.pump(false); return true; };
    if (!put(header_frame())) return all;
    for (int i = 0; i < frames; i++) if (!put(data_frame((uint8_t)i, payload, (uint8_t)(fill + i)))) break;
    log.end_session();
    return all;
}
static void verify_all(FlashLog& log) { while (log.verify_step()) {} }

static void t_flash() {
    printf("[flash log] sessions on raw NOR flash: scan, verify, write, power cut, foreign data, erase\n");
    { MemFlash m(256 * 1024); FlashLog log;
      CHECK(log.begin(&m) && log.ok() && log.sessions() == 0 && log.used() == 0 && !log.needs_erase() && !log.full(), "blank region: empty log");
      CHECK(log.writable() == 0 && !log.start_session(), "nothing is written before the free space has been checked");
      int steps = 1; while (log.verify_step()) steps++;
      CHECK(steps == 64 && log.verified_end() == m.size() && log.writable() == m.size(), "the check reads sector by sector (%d steps for 256 kB)", steps);
      const std::vector<uint8_t> s1 = write_session(log, 40, 10);
      const uint32_t pages1 = (uint32_t)((s1.size() + kPage - 1) / kPage * kPage);
      CHECK(log.sessions() == 1 && log.session_start(0) == 0 && log.session_size(0) == pages1 && log.used() == pages1, "one session: %u bytes (whole pages) at the start of the region", log.session_size(0));
      CHECK(m.bad_writes == 0 && m.longest_write <= kPage && m.erases == 0, "only erased bytes were programmed, never more than a page at once, nothing erased (bad %u, longest %u)", m.bad_writes, m.longest_write);
      std::vector<uint8_t> back(s1.size()); CHECK(log.read(0, back.data(), (uint32_t)back.size()) && back == s1, "it reads back byte for byte");
      CHECK(!log.read(0, back.data(), pages1 + 1) && !log.read(pages1, back.data(), 1), "nothing can be read past the end of the data");
      const std::vector<uint8_t> s2 = write_session(log, 7, 99);
      const uint32_t start2 = (uint32_t)((s1.size() + kSector - 1) / kSector * kSector);
      CHECK(log.sessions() == 2 && log.session_start(1) == start2 && log.session_size(1) == (s2.size() + kPage - 1) / kPage * kPage && log.session_size(0) == start2, "the second session starts on the next 4 kB boundary (%u)", log.session_start(1));
      // power cycle: everything is found again from the flash contents alone
      FlashLog again; CHECK(again.begin(&m) && again.sessions() == 2 && again.session_start(1) == start2 && again.used() == log.used() && !again.needs_erase(), "after a reset the same two sessions are found (used %u)", again.used());
      verify_all(again);
      const std::vector<uint8_t> s3 = write_session(again, 3, 50);
      CHECK(again.sessions() == 3 && again.session_start(2) % kSector == 0 && again.session_start(2) > start2 && m.bad_writes == 0, "and a third one is appended behind them");
      // a damaged byte costs one frame; the reader below finds the others
      const std::vector<RSession> ss = read_log(m.mem.data(), again.used());
      CHECK(ss.size() == 3 && ss[0].data_frames == 40 && ss[1].data_frames == 7 && ss[2].data_frames == 3, "reading the raw region gives the three sessions (%zu)", ss.size()); }

    { // power cut in the middle of a page program
      MemFlash m(256 * 1024); FlashLog log; log.begin(&m); verify_all(log);
      write_session(log, 20, 3);
      m.cut_after_writes = 6;                                   // the 7th page of the next session is half-written, then the power is gone
      write_session(log, 30, 4);
      m.cut_after_writes = -1;
      FlashLog again; CHECK(again.begin(&m) && again.sessions() == 2 && !again.needs_erase(), "power cut during a session: both sessions are listed after the reset");
      CHECK(again.used() % kPage == 0 && again.used() >= again.session_start(1) + 6 * kPage && again.used() <= again.session_start(1) + 8 * kPage, "the end of the data is the page after the torn one (%u)", again.used() - again.session_start(1));
      const std::vector<RSession> ss = read_log(m.mem.data(), again.used());
      CHECK(ss.size() == 2 && ss[0].data_frames == 20 && ss[1].data_frames >= 5 && ss[1].data_frames <= 7, "the torn session keeps its frames up to the cut (%d of 30)", ss.size() == 2 ? ss[1].data_frames : -1);
      verify_all(again); const uint32_t bad0 = m.bad_writes;
      write_session(again, 5, 9);
      CHECK(again.sessions() == 3 && again.session_start(2) % kSector == 0 && again.session_start(2) >= again.session_start(1) + 7 * kPage && m.bad_writes == bad0, "the next session starts on a clean sector behind it, no byte is programmed twice"); }

    { // a partition that held something else (an old SPIFFS, another firmware's data)
      MemFlash m(256 * 1024); m.fill_garbage(0, 9000); FlashLog log;
      CHECK(log.begin(&m) && log.needs_erase() && log.sessions() == 0 && log.writable() == 0 && !log.start_session() && !log.verify_step(), "foreign data at the start: 'needs erase', nothing is written");
      CHECK(log.erase_start() && log.erasing(), "erase");
      int steps = 0; uint8_t pct_mid = 0; while (log.erase_step()) { steps++; if (steps == 2) pct_mid = log.erase_percent(); }
      CHECK(steps == 3 && pct_mid == 50 && m.erases == 4 && log.erase_percent() == 100, "256 kB = four 64 kB blocks, progress reported (%d steps, %u %% half way)", steps, pct_mid);
      bool blank = true; for (uint8_t b : m.mem) if (b != 0xFF) blank = false;
      CHECK(blank && !log.needs_erase() && !log.erasing() && log.writable() == m.size() && log.sessions() == 0, "blank and usable afterwards");
      write_session(log, 4); CHECK(log.sessions() == 1 && m.bad_writes == 0, "a session can be written"); }
    { // garbage somewhere in the free space, found by the check before it can be written over
      MemFlash m(256 * 1024); FlashLog log; log.begin(&m); verify_all(log); write_session(log, 10);
      m.fill_garbage(150000, 300);
      FlashLog again; again.begin(&m);
      CHECK(!again.needs_erase() && again.sessions() == 1, "not visible at start-up");
      int steps = 0; while (again.verify_step()) steps++;
      CHECK(again.needs_erase() && again.verified_end() <= 150000 && again.writable() == 0 && !again.start_session() && m.bad_writes == 0, "the free-space check finds it (after %d sectors): the log asks for an erase instead of writing a corrupt session", steps); }
    { // a session that starts before the check is complete may only use what has been checked
      MemFlash m(256 * 1024); FlashLog log; log.begin(&m);
      for (int i = 0; i < 3; i++) log.verify_step();
      CHECK(log.writable() == 3 * kSector && log.start_session(), "three sectors checked: a session may start");
      std::vector<uint8_t> f = data_frame(0, 240, 1); uint32_t taken = 0;
      for (int i = 0; i < 200; i++) { if (!log.append(f.data(), (uint32_t)f.size())) { if (!log.pump(true)) break; if (!log.append(f.data(), (uint32_t)f.size())) break; } taken += (uint32_t)f.size(); }
      log.end_session();
      CHECK(taken <= 3 * kSector && taken > 3 * kSector - 300 && log.used() <= 3 * kSector && m.bad_writes == 0, "it stops at the checked limit (%u of %u bytes)", taken, 3 * kSector); }

    { // erase interrupted by a reset: what is left is "sessions, then free space", never free space with data behind it
      MemFlash m(512 * 1024); FlashLog log; log.begin(&m); verify_all(log);
      for (int i = 0; i < 6; i++) write_session(log, 250, (uint8_t)i);            // six sessions of ~62 kB
      CHECK(log.sessions() == 6 && log.used() > 5 * 64000, "six sessions stored");
      log.erase_start(); for (int i = 0; i < 4; i++) log.erase_step();             // erases the upper 256 kB, then the power fails
      FlashLog again; CHECK(again.begin(&m) && !again.needs_erase() && again.sessions() >= 3 && again.sessions() <= 4 && again.used() <= 256 * 1024, "half-erased region after a reset: the sessions in the untouched half are still there (%d), the rest is free", again.sessions());
      verify_all(again);
      CHECK(!again.needs_erase() && again.writable() >= 256 * 1024 - kSector, "and the free space really is erased");
      const uint32_t bad0 = m.bad_writes; write_session(again, 10); CHECK(m.bad_writes == bad0 && !again.needs_erase(), "logging continues without touching old data"); }

    { // full
      MemFlash m(16 * 1024); FlashLog log; log.begin(&m); verify_all(log);
      const std::vector<uint8_t> s = write_session(log, 1000);
      CHECK(s.size() > 15 * 1024 && s.size() <= 16 * 1024 && log.used() <= m.size() && m.bad_writes == 0, "a session stops when the region is full (%zu of 16384 bytes)", s.size());
      CHECK(log.full() && !log.start_session() && log.writable() == 0 && log.sessions() == 1, "full: no new session");
      FlashLog again; again.begin(&m); CHECK(again.full() && again.sessions() == 1 && !again.needs_erase(), "also after a reset"); }
    { // more sessions than the list holds
      MemFlash m(1024 * 1024); FlashLog log; log.begin(&m); verify_all(log);
      for (int i = 0; i < 140; i++) write_session(log, 1, (uint8_t)i, 100);
      CHECK(log.sessions() == 140 && log.listed() == FlashLog::kMaxSessions && log.session_start(0) == 12u * kSector && log.session_start(127) == 139u * kSector, "140 sessions: all counted, the newest 128 listed");
      FlashLog again; again.begin(&m); CHECK(again.sessions() == 140 && again.listed() == 128 && again.session_start(127) == 139u * kSector, "the same after a reset"); }
    { // flash faults
      MemFlash m(64 * 1024); FlashLog log; log.begin(&m); verify_all(log);
      log.start_session(); std::vector<uint8_t> f = data_frame(0, 240, 1); for (int i = 0; i < 3; i++) log.append(f.data(), (uint32_t)f.size());
      m.fail_writes = true; CHECK(log.pump(true) == 0 && log.io_error() && !log.writing(), "a failed program operation ends the session and is reported");
      m.fail_writes = false; MemFlash e(64 * 1024); e.fill_garbage(0, 100); FlashLog l2; l2.begin(&e); e.fail_erase = true; l2.erase_start();
      CHECK(!l2.erase_step() && l2.io_error() && l2.needs_erase(), "a failed erase is reported, the region stays 'needs erase'");
      MemFlash r(64 * 1024); r.fail_reads = true; FlashLog l3; CHECK(!l3.begin(&r) && l3.io_error(), "a region that cannot be read");
      FlashLog l4; MemFlash tiny(8 * 1024); CHECK(!l4.begin(&tiny) && !l4.ok() && !l4.begin(nullptr), "too small / missing region: not usable"); }
    { uint8_t b[12] = {0xA5, 0x5A, 20, 0, kMsgHeader, 0, 'W', 'B', 'L', 'O', 'G', ' '};
      CHECK(FlashLog::is_session_start(b), "session start mark");
      b[5] = 1; CHECK(!FlashLog::is_session_start(b), "a later header piece is not a start"); b[5] = 0;
      b[4] = kMsgData; CHECK(!FlashLog::is_session_start(b), "a data frame is not a start"); b[4] = kMsgHeader;
      b[8] = 'X'; CHECK(!FlashLog::is_session_start(b), "other text is not a start"); }
}

// ------------------------------------------------------------------------------------------------------------------------------
struct FakeTx : wbp::Transport {                             // a transmit buffer of `room` bytes that the "USB host" empties once per millisecond
    std::vector<uint8_t> out; size_t room = 1000000, pending = 0; uint32_t bps = 0;
    size_t write(const uint8_t* d, size_t n) override { out.insert(out.end(), d, d + n); pending += n; return n; }
    size_t writable() override { return room > pending ? room - pending : 0; }
    uint32_t bytes_per_second() override { return bps; }
};
struct Bench {
    Recorder rec; FakeTx tx; MemFlash mem; Service svc; wbp::Settings settings; FirmwareHeader hdr;
    Inputs in; uint32_t us = 0; uint32_t cycles = 0; float gyro[3] = {0, 0, 0}, zero[4] = {0, 0, 0, 0}, motor[8] = {0.1f, 0.2f, 0.3f, 0.4f}; app::Telemetry tel;
    int max_writes_per_poll = 0; bool flight_loop = true;
    explicit Bench(uint32_t bytes = 512 * 1024, bool erase_switch_on = false) : mem(bytes), svc(rec, tx), hdr{&settings, "Test FW", 1, 2, 3} {
        wbp::defaults(settings); in.erase_switch = erase_switch_on;
        svc.attach_store(&mem); svc.set_header_source(&FirmwareHeader::write, &hdr); svc.set_loop(4000.0f, 4);
        Config c; c.device = DEV_FLASH; c.trigger = TRIG_ARMED; c.rate_hz = 500; c.groups = (1u << G_GYRO) | (1u << G_MOTORS) | (1u << G_TIMING); svc.set_config(c);
        in.now_ms = 5000; us = 5000000;
        ms(200);                                               // on the ground after power-on: the free space gets checked
        max_writes_per_poll = 0;
    }
    void cfg(uint8_t device, uint8_t trigger, uint16_t rate = 500) { Config c = svc.config(); c.device = device; c.trigger = trigger; c.rate_hz = rate; svc.set_config(c); }
    // one millisecond: four control cycles of the "flight loop", then the link task's poll
    void ms(int n = 1) {
        for (int k = 0; k < n; k++) {
            for (int c = 0; c < 4 && flight_loop; c++) {
                us += 250; cycles++;
                gyro[0] = (float)(cycles % 1000) * 0.1f;
                rec.note_state(us, in.armed, 0, in.switch_on ? 0x80u : 0u, 1, 0);
                if (rec.fast_due(250)) { FastSample s; s.t_us = us; s.gyro = gyro; s.motor = motor; s.loop_us = 80; rec.log_fast(s); }
                if (cycles % 400 == 0) { tel.armed = in.armed; rec.log_slow(us, tel); }
            }
            in.now_ms++; tx.pending = 0;
            const uint32_t w0 = mem.writes; svc.poll(in); const int w = (int)(mem.writes - w0); if (w > max_writes_per_poll) max_writes_per_poll = w;
        }
    }
    wbp::Err cmd(uint8_t c, std::initializer_list<uint8_t> a = {}, std::vector<uint8_t>* res = nullptr) {
        uint8_t args[8] = {}; int i = 0; for (uint8_t x : a) args[i++] = x;
        uint8_t r[64]; uint16_t rl = 0; const wbp::Err e = svc.command(c, args, r, rl); if (res) res->assign(r, r + rl); return e;
    }
    std::vector<RSession> stored() { return read_log(mem.mem.data(), svc.log().used()); }
    std::vector<RSession> streamed() { return read_log(tx.out.data(), tx.out.size()); }
};

// The app link is a UART: a transmit buffer that the wire empties at baud / 10 bytes per second.
struct UartTx : wbp::Transport {
    std::vector<uint8_t> out; uint32_t baud; size_t cap, queued = 0; double credit = 0;
    explicit UartTx(uint32_t b, size_t buffer = 8192) : baud(b), cap(buffer) {}
    size_t write(const uint8_t* d, size_t n) override { out.insert(out.end(), d, d + n); queued += n; return n; }
    size_t writable() override { return cap > queued ? cap - queued : 0; }
    uint32_t bytes_per_second() override { return baud / 10; }
    void ms() { credit += baud / 10 / 1000.0; const size_t k = (size_t)credit; credit -= (double)k; queued = queued > k ? queued - k : 0; }
};
static void t_link_budget() {
    printf("[link budget] the live log on a thin link, a download while a session is being written\n");
    CHECK(stream_rate_limit(0, 60) == 0, "a fast link (USB): no limit");
    CHECK(stream_rate_limit(23040, 60) == 185 && stream_rate_limit(23040, 30) == 371 && stream_rate_limit(11520, 60) == 92, "UART: half of baud / 10, records + 3 %% of framing (%u %u %u)", stream_rate_limit(23040, 60), stream_rate_limit(23040, 30), stream_rate_limit(11520, 60));
    CHECK(stream_rate_limit(100, 250) == 1 && stream_rate_limit(23040, 0) == 0, "never 0 records per second on a link that exists");
    // ---- the same stream request on three links
    auto run = [](uint32_t baud, uint16_t want, double& hz, uint32_t& dropped, int& gaps, int& hdr_rate) {
        Recorder rec; UartTx tx(baud ? baud : 1000000); MemFlash mem(256 * 1024); wbp::Settings settings; wbp::defaults(settings); FirmwareHeader hdr{&settings, "Test FW", 1, 2, 3};
        struct Fast : wbp::Transport { UartTx& u; explicit Fast(UartTx& x) : u(x) {} size_t write(const uint8_t* d, size_t n) override { return u.write(d, n); } size_t writable() override { return u.writable(); } } fast(tx);
        Service svc(rec, baud ? static_cast<wbp::Transport&>(tx) : static_cast<wbp::Transport&>(fast));
        svc.attach_store(&mem); svc.set_header_source(&FirmwareHeader::write, &hdr); svc.set_loop(4000.0f, 4);
        Config c; c.device = DEV_NONE; c.rate_hz = 500; c.groups = 0xFFFFFFFFu; svc.set_config(c);
        Inputs in; in.now_ms = 1000; in.app_active = true; uint32_t us = 1000000, cycles = 0; float g[3] = {1, 2, 3}, m[8] = {0.1f, 0.2f, 0.3f, 0.4f};
        uint8_t a[8] = {1, (uint8_t)want, (uint8_t)(want >> 8)}, r[16]; uint16_t rl = 0; svc.command(wbp::CMD_BLACKBOX_STREAM, a, r, rl);
        for (int k = 0; k < 5000; k++) {
            for (int i = 0; i < 4; i++) { us += 250; cycles++; rec.note_state(us, false, 0, 0, 1, 0); if (rec.fast_due(250)) { FastSample s; s.t_us = us; s.gyro = g; s.gyro_raw = g; s.setpoint = g; s.pid_p = g; s.pid_i = g; s.pid_d = g; s.pid_f = g; s.stick = m; s.motor = m; s.rpm = m; s.acc = g; s.loop_us = 80; rec.log_fast(s); } }
            in.now_ms++; tx.ms(); svc.poll(in);
        }
        const std::vector<RSession> ss = read_log(tx.out.data(), tx.out.size());
        hz = ss.empty() ? 0 : ss[0].F.size() / 5.0; dropped = svc.stream_frames_dropped(); gaps = ss.empty() ? -1 : ss[0].seq_gaps; hdr_rate = ss.empty() || !ss[0].meta.count("rate_hz") ? -1 : (int)lround(atof(ss[0].meta.at("rate_hz").c_str()));
    };
    double hz; uint32_t dropped; int gaps, hdr;
    run(0, 1000, hz, dropped, gaps, hdr);
    CHECK(hz > 990 && hz < 1010 && dropped == 0 && gaps == 0 && hdr == 1000, "USB, 1000 records/s asked: all of them arrive (%.0f /s, header says %d)", hz, hdr);
    run(230400, 1000, hz, dropped, gaps, hdr);
    CHECK(hz > 60 && hz < 200 && dropped == 0 && gaps == 0 && fabs(hz - hdr) < 0.03 * hdr, "UART 230400, 1000 asked: the firmware records %.0f /s (header %d) and none is lost on the way - a slower chart, not a torn one", hz, hdr);
    run(921600, 1000, hz, dropped, gaps, hdr);
    CHECK(hz > 300 && hz < 800 && dropped == 0 && gaps == 0, "UART 921600: %.0f /s", hz);
    run(230400, 50, hz, dropped, gaps, hdr);
    CHECK(hz > 49 && hz < 51 && hdr == 50, "a request below the limit is left alone (%.0f /s)", hz);

    // ---- download while a session is being written
    { Bench b; b.cfg(DEV_FLASH, TRIG_ALWAYS); b.ms(1500);
      CHECK(b.svc.state() == ST_LOGGING && b.svc.log().writing(), "trigger ALWAYS: a session is being written from the start");
      std::vector<uint8_t> res; uint32_t st, sz, sz2;
      CHECK(b.cmd(wbp::CMD_BLACKBOX_SESSION, {0, 0}, &res) == wbp::ERR_OK && res.size() == 8 && b.svc.state() == ST_LOGGING,
            "0.8.8: the app asks for the list - it gets it, and the recording goes on (it used to be closed for a mere look at the list)");
      memcpy(&sz, res.data() + 4, 4); b.ms(500); b.cmd(wbp::CMD_BLACKBOX_SESSION, {0, 0}, &res); memcpy(&sz2, res.data() + 4, 4);
      CHECK(sz2 > sz && b.svc.state() == ST_LOGGING && b.stored().size() == 1, "the session being written is in the list, growing (%u -> %u bytes)", sz, sz2);
      CHECK(b.cmd(wbp::CMD_BLACKBOX_READ, {0, 0, 0, 0, 16, 0, 0, 0}) == wbp::ERR_BUSY, "a DOWNLOAD needs it closed: BUSY for the moment ...");
      b.ms(50);
      CHECK(b.svc.state() == ST_IDLE && b.cmd(wbp::CMD_BLACKBOX_SESSION, {0, 0}, &res) == wbp::ERR_OK && res.size() == 8, "... closed for it, 50 ms later the list is final");
      memcpy(&st, res.data(), 4); memcpy(&sz, res.data() + 4, 4);
      const std::vector<RSession> ss = b.stored();
      const bool ended = ss.size() == 1 && ss[0].count_events(EV_END) == 1 && ss[0].events.back().code == EV_END && ss[0].events.back().data.size() == 1 && ss[0].events.back().data[0] == END_APP_TOOK_OVER;
      CHECK(ended && sz > 20000, "closed properly, with its reason (%u bytes)", sz);
      b.ms(1000); CHECK(b.svc.state() == ST_IDLE && b.svc.log().sessions() == 1, "no new session starts while the app is busy with the memory");
      uint8_t a[8]; memcpy(a, &st, 4); memcpy(a + 4, &sz, 4); b.tx.out.clear();
      CHECK(b.cmd(wbp::CMD_BLACKBOX_READ, {a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7]}) == wbp::ERR_OK, "READ");
      uint32_t received = 0; std::vector<uint8_t> got(sz);
      for (int polls = 0; polls < 3000 && received < sz; polls++) { b.tx.out.clear(); b.ms(1); wbp::Parser p; wbp::Frame f; for (uint8_t c : b.tx.out) if (p.feed(c, f) && f.type == kMsgBlob) { uint32_t off; memcpy(&off, f.payload, 4); memcpy(got.data() + (off - st), f.payload + 4, f.len - 4u); received += f.len - 4u; } }
      CHECK(received == sz && memcmp(got.data(), b.mem.mem.data() + st, sz) == 0 && b.svc.log().sessions() == 1, "downloaded, still no new session in the way");
      b.ms(2000); CHECK(b.svc.state() == ST_IDLE, "2 s after the download: still held back");
      b.ms(1500); CHECK(b.svc.state() == ST_LOGGING, "3 s after the app's last request the logging goes on by itself (trigger ALWAYS)");
      // in the air the flight log wins
      b.in.armed = true; b.ms(300);
      CHECK(b.cmd(wbp::CMD_BLACKBOX_SESSION, {0, 0}) == wbp::ERR_OK && b.cmd(wbp::CMD_BLACKBOX_READ, {0, 0, 0, 0, 16, 0, 0, 0}) == wbp::ERR_BUSY, "armed: the list yes, a download BUSY");
      b.ms(300); CHECK(b.svc.state() == ST_LOGGING && b.stored().size() == 2 && b.stored()[1].count_events(EV_END) == 0, "and the session is NOT cut"); }
    // ---- the BLACKBOX switch is on and the app opens its blackbox tab (the bug of 0.8.7: closed, a new one 3 s later, closed again ...)
    { Bench b; b.cfg(DEV_FLASH, TRIG_SWITCH); b.in.switch_on = true; b.ms(1000);
      CHECK(b.svc.state() == ST_LOGGING, "the switch records on the ground");
      std::vector<uint8_t> res;
      for (int i = 0; i < 5; i++) { b.cmd(wbp::CMD_BLACKBOX_SESSION, {0, 0}, &res); b.ms(800); }
      CHECK(b.svc.state() == ST_LOGGING && b.stored().size() == 1 && b.stored()[0].count_events(EV_END) == 0, "the tab looks at the list every 0.8 s for 4 s: one session, still recording");
      CHECK(b.cmd(wbp::CMD_BLACKBOX_READ, {0, 0, 0, 0, 16, 0, 0, 0}) == wbp::ERR_BUSY, "a download is asked for"); b.ms(50);
      CHECK(b.svc.state() == ST_IDLE && b.stored().size() == 1, "closed for it");
      b.ms(5000); CHECK(b.svc.state() == ST_IDLE && b.svc.log().sessions() == 1, "the switch is still on, 5 s later: no new session by itself (it would be closed by the next look again)");
      b.in.switch_on = false; b.ms(100); b.in.switch_on = true; b.ms(300);
      CHECK(b.svc.state() == ST_LOGGING && b.svc.log().sessions() == 2, "switch off and on again: a new session"); }
    { Bench b; b.ms(100);                                    // trigger ARMED, on the ground, the app reads; then the pilot arms
      b.in.armed = true; b.ms(500); b.in.armed = false; b.ms(300);
      std::vector<uint8_t> res; CHECK(b.cmd(wbp::CMD_BLACKBOX_SESSION, {0, 0}, &res) == wbp::ERR_OK, "list");
      b.ms(100); b.in.armed = true; b.ms(300);
      CHECK(b.svc.state() == ST_LOGGING, "arming 100 ms after the app asked: the flight is logged at once (the hold is only for the ground)"); }
    { Bench b; b.cfg(DEV_FLASH, TRIG_SWITCH); b.ms(100);     // nothing is being written; the app lists the memory, then somebody flips the BLACKBOX switch
      b.in.armed = true; b.in.switch_on = true; b.ms(500); b.in.armed = false; b.in.switch_on = false; b.ms(300);
      CHECK(b.svc.state() == ST_IDLE && b.cmd(wbp::CMD_BLACKBOX_SESSION, {0, 0}) == wbp::ERR_OK, "list (nothing had to be closed)");
      b.in.switch_on = true; b.ms(1000);
      CHECK(b.svc.state() == ST_IDLE && b.svc.log().sessions() == 1, "the switch 1 s after the app's request: the list it got stays true for now");
      b.ms(2500); CHECK(b.svc.state() == ST_LOGGING, "3 s after the request the switch is obeyed"); }
}

static void t_service() {
    printf("[service] sessions on storage: trigger, header, records, end, one page per millisecond\n");
    { Bench b; b.ms(300);
      CHECK(b.svc.state() == ST_IDLE && b.mem.writes == 0 && !b.rec.active(), "disarmed with trigger ARMED: idle, nothing written");
      b.in.armed = true; b.ms(2000);
      CHECK(b.svc.state() == ST_LOGGING && b.rec.active(), "armed: logging");
      b.in.armed = false; b.ms(300);
      CHECK(b.svc.state() == ST_IDLE && !b.rec.active() && b.svc.log().sessions() == 1, "disarmed: session closed");
      CHECK(b.max_writes_per_poll == 1 && b.mem.bad_writes == 0 && b.mem.longest_write <= kPage, "never more than one flash page per millisecond (%d), clean NOR writes", b.max_writes_per_poll);
      // every write stops the flight loop for a moment: full pages only, plus a flush of the open page twice a second
      CHECK(b.mem.writes >= b.mem.bytes_written / kPage && b.mem.writes <= b.mem.bytes_written / kPage + 8, "%u bytes went out in %u page programs (full pages + a few flushes)", b.mem.bytes_written, b.mem.writes);
      const std::vector<RSession> ss = b.stored();
      CHECK(ss.size() == 1 && ss[0].bad_records == 0 && ss[0].seq_gaps == 0, "the stored bytes parse as one session without a gap");
      const RSession& s = ss[0];
      CHECK(s.text.rfind("WBLOG 1\n", 0) == 0 && s.text.find("\nend\n") != std::string::npos && s.meta.at("session") == "1" && s.meta.at("to") == "storage", "header: format version, session number, destination");
      CHECK(s.meta.at("firmware") == "Test FW 1.2.3" && s.meta.at("loop_hz") == "4000" && s.meta.at("rate_div") == "8" && s.meta.at("rate_hz") == "500" && s.meta.at("motors") == "4" && s.meta.at("slow_hz") == "10", "header: firmware, loop rate, record rate");
      char want[16]; snprintf(want, sizeof(want), "0x%08lX", (unsigned long)wbp::kSchemaHash); CHECK(s.meta.at("schema") == want, "header: schema hash %s", s.meta.at("schema").c_str());
      { // the settings are in the header, section by section
        const std::string& hex = s.meta.at("set.pid"); wbp::S_pid pid; bool ok = hex.size() == 2 * sizeof(pid);
        for (size_t i = 0; ok && i < sizeof(pid); i++) ((uint8_t*)&pid)[i] = (uint8_t)strtol(hex.substr(2 * i, 2).c_str(), nullptr, 16);
        CHECK(ok && memcmp(&pid, &b.settings.pid, sizeof(pid)) == 0 && s.meta.count("set.general") && s.meta.count("set.blackbox") && s.meta.count("set.calibration"), "header: every settings section as hex (PID gains readable from the log)"); }
      CHECK(s.fast_size == 4 + 6 + 8 + 9 && s.fast.size() == 1 + 3 + 4 + 7 && s.fast[1].name == "gyro[0]" && s.fast[8].name == "loop_us" && s.fast[14].name == "t_motor", "header describes the fast record (%d bytes, %zu fields)", s.fast_size, s.fast.size());
      CHECK(s.F.size() >= 995 && s.F.size() <= 1001, "2 s at 500 records/s: %zu fast records", s.F.size());
      bool spacing = true, values = true; for (size_t i = 1; i < s.F.size(); i++) { if (s.F[i][0] - s.F[i - 1][0] != 2000) spacing = false; }
      for (size_t i = 0; i < s.F.size(); i++) { if (fabs(s.fv(i, "motor[2]") - 0.3) > 1e-9 || s.fv(i, "loop_us") != 80 || s.fv(i, "gap_us") != 250) values = false; }
      CHECK(spacing && values, "every record is there: 2000 us apart, values intact");
      CHECK(s.S.size() >= 19 && s.S.size() <= 21 && s.sv(5, "armed") == 1, "slow records at 10 Hz (%zu)", s.S.size());
      CHECK(s.count_events(EV_ARM) == 1 && s.count_events(EV_DISARM) == 1 && s.events.back().code == EV_END && s.events.back().data.size() == 1 && s.events.back().data[0] == END_TRIGGER_OFF, "events: arm, disarm, and END (trigger off) as the last record");
      // second flight
      b.in.armed = true; b.ms(500); b.in.armed = false; b.ms(300);
      const std::vector<RSession> two = b.stored();
      CHECK(two.size() == 2 && two[1].meta.at("session") == "2" && b.svc.log().session_start(1) % kSector == 0 && b.mem.bad_writes == 0, "a second flight is a second session on its own sector");
      // status for the app
      uint8_t sb[64]; wbp::T_blackbox st; CHECK(b.svc.pack_status(sb, sizeof(sb)) == sizeof(st), "status struct"); memcpy(&st, sb, sizeof(st));
      CHECK(st.state == ST_IDLE && st.capacity == 512 * 1024 && st.used == b.svc.log().used() && st.sessions == 2 && st.record_bytes == 28 && st.dropped == 0 && st.erase_pct == 100, "status: capacity, used, sessions, record size");
      CHECK(b.svc.pack_status(sb, 10) == 0, "status into a buffer that is too small: nothing"); }

    { Bench b; b.cfg(DEV_FLASH, TRIG_SWITCH); b.in.armed = true; b.ms(500);
      CHECK(b.svc.state() == ST_IDLE, "trigger SWITCH: arming alone does not log");
      b.in.armed = false; b.in.switch_on = true; b.ms(500); CHECK(b.svc.state() == ST_LOGGING, "the BLACKBOX switch logs, also on the ground");
      CHECK(b.cmd(wbp::CMD_BLACKBOX_ERASE) == wbp::ERR_BUSY && b.svc.state() == ST_LOGGING, "no erase while a session is being written");
      b.in.switch_on = false; b.ms(300); CHECK(b.svc.state() == ST_IDLE && b.svc.log().sessions() == 1, "switch off: closed"); }
    { Bench b; b.cfg(DEV_FLASH, TRIG_ALWAYS); b.ms(600); CHECK(b.svc.state() == ST_LOGGING, "trigger ALWAYS: logs from power-on");
      b.cfg(DEV_NONE, TRIG_ALWAYS); b.ms(300); CHECK(b.svc.state() == ST_OFF && b.svc.log().sessions() == 1 && b.svc.device() == DEV_NONE, "device set to NONE: the session ends, state OFF; the stored session can still be read"); }
    { Bench b; b.cfg(DEV_NONE, TRIG_ARMED); b.in.armed = true; b.ms(500); CHECK(b.svc.state() == ST_OFF && b.mem.writes == 0, "device NONE: never writes"); }
    { // logging switched off: the flash is not even checked (nothing will be written there; reads cost flight-loop time on the real chip)
      Bench b; b.cfg(DEV_NONE, TRIG_ARMED); b.ms(20); MemFlash fresh(512 * 1024); b.svc.attach_store(&fresh); const uint32_t r0 = fresh.reads; b.ms(400);
      CHECK(b.svc.state() == ST_OFF && fresh.reads == r0, "device NONE: the free space of the flash is not verified (%u reads)", fresh.reads - r0);
      b.cfg(DEV_FLASH, TRIG_ARMED); b.ms(400); CHECK(b.svc.state() == ST_IDLE && fresh.reads > r0 + 100, "switched to FLASH: now it is (%u reads)", fresh.reads - r0); }
    { Bench b; b.cfg(DEV_FLASH, TRIG_ARMED, 2000); b.in.armed = true; b.ms(1000); b.in.armed = false; b.ms(300);
      const std::vector<RSession> ss = b.stored();
      CHECK(ss.size() == 1 && ss[0].meta.at("rate_div") == "2" && ss[0].F.size() >= 1990 && ss[0].seq_gaps == 0 && ss[0].count_events(EV_DROPPED) == 0, "2000 records/s (46 kB/s): everything arrives, nothing dropped (%zu records)", ss.empty() ? 0 : ss[0].F.size()); }

    printf("[service] full, erase, foreign data, no storage\n");
    { Bench b(24 * 1024); b.in.armed = true; b.ms(3000);
      CHECK(b.svc.state() == ST_FULL && !b.rec.active() && b.mem.bad_writes == 0, "24 kB of storage: full after about two seconds, recording stops");
      const std::vector<RSession> ss = b.stored();
      CHECK(ss.size() == 1 && ss[0].bad_records == 0 && !ss[0].events.empty() && ss[0].events.back().code == EV_END && ss[0].events.back().data[0] == END_FULL, "the session ends with END (full), readable to the last frame");
      b.in.armed = false; b.ms(200); b.in.armed = true; b.ms(500); CHECK(b.svc.state() == ST_FULL && b.svc.log().sessions() == 1, "no new session while full");
      b.in.armed = false; b.ms(10);
      CHECK(b.cmd(wbp::CMD_BLACKBOX_ERASE) == wbp::ERR_OK && b.svc.state() == ST_ERASING && b.rec.busy(), "ERASE: started, the craft is kept on the ground meanwhile");
      CHECK(b.cmd(wbp::CMD_BLACKBOX_ERASE) == wbp::ERR_BUSY && b.cmd(wbp::CMD_BLACKBOX_READ, {0, 0, 0, 0, 16, 0, 0, 0}) == wbp::ERR_BUSY, "no second erase, no download during the erase");
      b.ms(20);
      CHECK(b.svc.state() == ST_IDLE && !b.rec.busy() && b.rec.take_erase_done() && b.svc.log().sessions() == 0 && b.svc.log().used() == 0, "erased: idle, empty, 'done' posted for the beeper");
      b.in.armed = true; b.ms(500); CHECK(b.svc.state() == ST_LOGGING && b.mem.bad_writes == 0, "logging works again"); }
    { // the BLACKBOX_ERASE switch (0.23)
      Bench b(512 * 1024, true); b.in.armed = true; b.ms(500); b.in.armed = false; b.ms(300);
      CHECK(b.svc.state() == ST_IDLE && b.svc.log().sessions() == 1 && b.svc.switch_erases() == 0, "BLACKBOX_ERASE on since power-up: a flight logged, nothing erased (a switch left on is not a flip)");
      b.in.erase_switch = false; b.ms(20); b.in.erase_switch = true; b.ms(1);
      CHECK(b.svc.state() == ST_ERASING && b.rec.busy() && b.svc.switch_erases() == 1, "flipped off and on, disarmed: erasing, the craft kept on the ground");
      b.ms(50); CHECK(b.svc.state() == ST_IDLE && b.svc.log().sessions() == 0 && b.rec.take_erase_done(), "erased; 'done' posted for the beeper");
      b.ms(300); CHECK(b.svc.switch_erases() == 1, "held on: once only");
      b.in.armed = true; b.ms(500); b.in.erase_switch = false; b.ms(20); b.in.erase_switch = true; b.ms(50);
      CHECK(b.svc.state() == ST_LOGGING && b.svc.switch_erases() == 1, "flipped while armed: ignored, the flight is logged");
      b.in.armed = false; b.ms(300); CHECK(b.svc.log().sessions() == 1 && b.svc.switch_erases() == 1, "disarmed with the switch still on: nothing (it takes a new flip)");
      b.cfg(DEV_FLASH, TRIG_ALWAYS); b.ms(200); b.in.erase_switch = false; b.ms(20); b.in.erase_switch = true; b.ms(50);
      CHECK(b.svc.state() == ST_LOGGING && b.svc.switch_erases() == 0 + 1, "logging on the ground (trigger ALWAYS): the flip is refused, the log goes on");
      b.cfg(DEV_FLASH, TRIG_ARMED); b.ms(300); CHECK(b.svc.state() == ST_IDLE && b.svc.switch_erases() == 1, "a refused flip is not retried later by itself");
      b.cfg(DEV_FLASH, TRIG_SWITCH); b.in.armed = true; b.ms(100); b.in.erase_switch = false; b.ms(20); b.in.erase_switch = true; b.ms(50);
      CHECK(b.svc.state() == ST_IDLE && b.svc.switch_erases() == 1 && b.svc.log().sessions() > 0, "armed but not logging (trigger SWITCH, its switch off): the flip is ignored all the same"); }
    { Bench b; b.mem.fill_garbage(0, 5000); b.svc.attach_store(&b.mem); b.in.armed = true; b.ms(500);
      CHECK(b.svc.state() == ST_NEEDS_ERASE && b.mem.writes == 0, "a region with foreign data: NEEDS_ERASE, not a byte written");
      b.in.armed = false; b.ms(5); CHECK(b.cmd(wbp::CMD_BLACKBOX_ERASE) == wbp::ERR_OK, "erase"); b.ms(50);
      b.in.armed = true; b.ms(300); CHECK(b.svc.state() == ST_LOGGING, "then it logs"); }
    { Bench b; b.svc.attach_store(nullptr); b.in.armed = true; b.ms(300);
      CHECK(b.svc.state() == ST_NO_STORAGE && b.cmd(wbp::CMD_BLACKBOX_ERASE) == wbp::ERR_NO_SENSOR && b.cmd(wbp::CMD_BLACKBOX_READ, {0, 0, 0, 0, 1, 0, 0, 0}) == wbp::ERR_NO_SENSOR && b.cmd(wbp::CMD_BLACKBOX_SESSION, {0, 0}) == wbp::ERR_NO_SENSOR,
            "no log partition: NO_STORAGE, commands say so"); }
    { // the queue overflows when the link task is held up (a long flash operation, a blocked USB write)
      Bench b; b.in.armed = true; b.ms(500);
      for (int i = 0; i < 12000; i++) { b.us += 250; if (b.rec.fast_due(250)) { FastSample s; s.t_us = b.us; s.gyro = b.gyro; s.motor = b.motor; b.rec.log_fast(s); } }   // three seconds of flight without a poll: 1500 records, 43 kB, into a 16 kB queue
      b.in.now_ms += 3000; b.ms(500); b.in.armed = false; b.ms(300);
      const std::vector<RSession> ss = b.stored(); uint32_t lost = 0;
      if (!ss.empty()) for (const RSession::Ev& e : ss[0].events) if (e.code == EV_DROPPED && e.data.size() == 4) { uint32_t n; memcpy(&n, e.data.data(), 4); lost += n; }
      CHECK(ss.size() == 1 && lost > 0 && lost == b.rec.ring().dropped() && ss[0].bad_records == 0, "records lost in the queue are written down as DROPPED events with their count (%u)", lost);
      const size_t kept = ss.empty() ? 0 : ss[0].F.size();
      CHECK(kept + lost >= 1990 && kept + lost <= 2010 && lost > 800 && lost < 1000, "stored + dropped = the 2000 records produced in the 4 s of the session (%zu + %u)", kept, lost); }

    printf("[service] stream to the app\n");
    { Bench b; b.cfg(DEV_NONE, TRIG_ARMED); b.in.app_active = true; b.ms(100);
      CHECK(b.cmd(wbp::CMD_BLACKBOX_STREAM, {1, 0, 0}) == wbp::ERR_OK, "STREAM on (works without any storage)"); b.ms(1000);
      CHECK(b.svc.state() == ST_STREAMING && b.mem.writes == 0, "streaming, nothing stored");
      CHECK(b.cmd(wbp::CMD_BLACKBOX_STREAM, {0, 0, 0}) == wbp::ERR_OK, "STREAM off"); b.ms(100);
      const std::vector<RSession> ss = b.streamed();
      CHECK(b.svc.state() == ST_OFF && ss.size() == 1 && ss[0].meta.at("to") == "stream" && ss[0].meta.at("session") == "0" && ss[0].bad_records == 0 && ss[0].seq_gaps == 0, "the bytes on the link are a complete log: header, records");
      CHECK(ss[0].F.size() >= 495 && ss[0].F.size() <= 501 && ss[0].events.back().code == EV_END && ss[0].events.back().data[0] == END_STREAM_STOPPED, "1 s at the configured 500 records/s (%zu), END (stream stopped)", ss[0].F.size());
      b.tx.out.clear(); b.cmd(wbp::CMD_BLACKBOX_STREAM, {1, 100, 0}); b.ms(1000); b.cmd(wbp::CMD_BLACKBOX_STREAM, {0, 0, 0}); b.ms(100);
      const std::vector<RSession> s2 = b.streamed();
      CHECK(s2.size() == 1 && s2[0].meta.at("rate_hz") == "100" && s2[0].F.size() >= 98 && s2[0].F.size() <= 101, "the app can ask for its own rate: 100 records/s for a thin link (%zu)", s2.empty() ? 0 : s2[0].F.size()); }
    { // a slow stream must not sit in a half-filled frame: the app shows it live
      Bench b; b.cfg(DEV_NONE, TRIG_ARMED); Config c = b.svc.config(); c.groups = 1u << G_GYRO; b.svc.set_config(c); b.in.app_active = true;
      b.cmd(wbp::CMD_BLACKBOX_STREAM, {1, 50, 0}); b.ms(200);
      const std::vector<RSession> ss = b.streamed();
      CHECK(ss.size() == 1 && ss[0].F.size() >= 7 && ss[0].F.size() <= 10, "50 records/s of 11 bytes: after 0.2 s the app already has %zu of them (a frame leaves after 20 ms even when it is not full)", ss.empty() ? 0 : ss[0].F.size()); }
    { // the app goes silent (cable pulled): the stream ends by itself, storage takes over again
      Bench b; b.in.app_active = true; b.ms(50); b.cmd(wbp::CMD_BLACKBOX_STREAM, {1, 0, 0}); b.ms(300);
      b.in.armed = true; b.ms(300);
      CHECK(b.svc.state() == ST_STREAMING && b.mem.writes == 0, "while the app streams, arming does not start a storage session");
      b.in.app_active = false; b.ms(300);
      CHECK(b.svc.state() == ST_LOGGING, "the app is gone: the flight is logged to storage from here on");
      b.in.app_active = true; b.ms(10);
      CHECK(b.cmd(wbp::CMD_BLACKBOX_STREAM, {1, 0, 0}) == wbp::ERR_BUSY && b.svc.state() == ST_LOGGING, "the app comes back in flight: the storage session is NOT cut, the stream request is refused");
      b.in.armed = false; b.ms(300);
      CHECK(b.svc.state() == ST_IDLE && b.stored().size() == 1 && b.stored()[0].events.back().data[0] == END_TRIGGER_OFF, "landed and disarmed: session closed normally"); }
    { // on the ground the app takes over from a running storage session
      Bench b; b.cfg(DEV_FLASH, TRIG_SWITCH); b.in.switch_on = true; b.in.app_active = true; b.ms(500);
      CHECK(b.svc.state() == ST_LOGGING, "logging on the ground (switch)");
      CHECK(b.cmd(wbp::CMD_BLACKBOX_STREAM, {1, 0, 0}) == wbp::ERR_OK, "STREAM accepted"); b.ms(500);
      const std::vector<RSession> st = b.stored();
      CHECK(b.svc.state() == ST_STREAMING && st.size() == 1 && st[0].events.back().code == EV_END && st[0].events.back().data[0] == END_APP_TOOK_OVER, "storage session closed with END (app took over), stream running");
      const uint32_t w = b.mem.writes; b.ms(500); CHECK(b.mem.writes == w, "nothing more is stored while streaming");
      b.cmd(wbp::CMD_BLACKBOX_STREAM, {0, 0, 0}); b.ms(600);
      CHECK(b.svc.state() == ST_LOGGING && b.svc.log().sessions() == 2, "stream stopped, switch still on: a new storage session"); }
    { // a transmit buffer that is full: frames are dropped, never waited for; the gap is visible
      Bench b; b.cfg(DEV_NONE, TRIG_ARMED); b.in.app_active = true; b.cmd(wbp::CMD_BLACKBOX_STREAM, {1, 0, 0}); b.ms(300);
      b.tx.room = 100; b.ms(300); b.tx.room = 1000000; b.ms(300); b.cmd(wbp::CMD_BLACKBOX_STREAM, {0, 0, 0}); b.ms(100);
      const std::vector<RSession> ss = b.streamed();
      CHECK(ss.size() == 1 && b.svc.stream_frames_dropped() > 5 && ss[0].seq_gaps >= 1 && ss[0].bad_records == 0 && ss[0].F.size() > 250 && ss[0].F.size() < 330, "0.3 s without room on the link: %u frames dropped, the reader sees the gap, the rest is intact (%zu records)", b.svc.stream_frames_dropped(), ss[0].F.size());
      // the header is never dropped: with no room at the start it simply comes later
      Bench c; c.cfg(DEV_NONE, TRIG_ARMED); c.in.app_active = true; c.tx.room = 50; c.cmd(wbp::CMD_BLACKBOX_STREAM, {1, 0, 0}); c.ms(200);
      CHECK(c.tx.out.empty(), "no room: nothing sent yet");
      c.tx.room = 1000000; c.ms(300); c.cmd(wbp::CMD_BLACKBOX_STREAM, {0, 0, 0}); c.ms(100);
      const std::vector<RSession> cs = c.streamed();
      CHECK(cs.size() == 1 && cs[0].meta.count("firmware") && cs[0].text.find("\nend\n") != std::string::npos && !cs[0].F.empty(), "the complete header arrives first once there is room"); }

    printf("[service] download\n");
    { Bench b; b.in.armed = true; b.ms(1500); b.in.armed = false; b.ms(300); b.in.armed = true; b.ms(700);
      std::vector<uint8_t> res;
      CHECK(b.cmd(wbp::CMD_BLACKBOX_READ, {0, 0, 0, 0, 0, 1, 0, 0}) == wbp::ERR_BUSY, "no download while a session is being written");
      b.in.armed = false; b.ms(300);
      CHECK(b.cmd(wbp::CMD_BLACKBOX_SESSION, {0, 0}, &res) == wbp::ERR_OK && res.size() == 8, "SESSION 0");
      uint32_t st0, sz0, st1, sz1; memcpy(&st0, res.data(), 4); memcpy(&sz0, res.data() + 4, 4);
      CHECK(b.cmd(wbp::CMD_BLACKBOX_SESSION, {1, 0}, &res) == wbp::ERR_OK, "SESSION 1"); memcpy(&st1, res.data(), 4); memcpy(&sz1, res.data() + 4, 4);
      CHECK(st0 == 0 && sz0 == st1 && st1 % kSector == 0 && st1 + sz1 == b.svc.log().used() && b.cmd(wbp::CMD_BLACKBOX_SESSION, {2, 0}) == wbp::ERR_BAD_ARG, "the session table: start and size of each, BAD_ARG past the end");
      // read session 1 as blob frames and put it together again
      uint8_t a[8]; memcpy(a, &st1, 4); memcpy(a + 4, &sz1, 4); b.tx.out.clear();
      CHECK(b.cmd(wbp::CMD_BLACKBOX_READ, {a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7]}) == wbp::ERR_OK, "READ session 1");
      b.tx.room = 2000; int polls = 0; std::vector<uint8_t> got(sz1, 0xEE); uint32_t received = 0, next = st1; bool in_order = true; size_t most = 0;
      for (; polls < 2000 && received < sz1; polls++) {
          b.tx.out.clear(); b.ms(1); if (b.tx.out.size() > most) most = b.tx.out.size();
          wbp::Parser p; wbp::Frame f;
          for (uint8_t c : b.tx.out) if (p.feed(c, f) && f.type == kMsgBlob) { uint32_t off; memcpy(&off, f.payload, 4); if (off != next) in_order = false; memcpy(got.data() + (off - st1), f.payload + 4, f.len - 4u); received += f.len - 4u; next = off + f.len - 4u; }
      }
      CHECK(received == sz1 && in_order && memcmp(got.data(), b.mem.mem.data() + st1, sz1) == 0, "the blob frames are the stored bytes, in order (%u bytes in %d ms)", received, polls);
      CHECK(most <= 2000, "the transfer respects the room in the transmit buffer (%zu bytes at most per millisecond)", most);
      const std::vector<RSession> ss = read_log(got.data(), got.size());
      CHECK(ss.size() == 1 && ss[0].meta.at("session") == "2" && ss[0].F.size() > 300, "the downloaded bytes are session 2");
      const uint32_t used = b.svc.log().used(); uint8_t z[8]; memcpy(z, &used, 4); uint32_t one = 1; memcpy(z + 4, &one, 4);
      CHECK(b.cmd(wbp::CMD_BLACKBOX_READ, {z[0], z[1], z[2], z[3], z[4], z[5], z[6], z[7]}) == wbp::ERR_BAD_ARG, "reading past the end of the data: refused");
      // a transfer is cancelled by length 0, and by arming
      b.tx.room = 600; b.cmd(wbp::CMD_BLACKBOX_READ, {0, 0, 0, 0, a[4], a[5], a[6], a[7]}); b.ms(3); b.tx.out.clear();
      CHECK(b.cmd(wbp::CMD_BLACKBOX_READ, {0, 0, 0, 0, 0, 0, 0, 0}) == wbp::ERR_OK, "READ length 0"); b.ms(20); CHECK(b.tx.out.empty(), "cancelled: no more blob frames");
      const uint32_t rb = b.mem.reads; b.cmd(wbp::CMD_BLACKBOX_READ, {0, 0, 0, 0, a[4], a[5], a[6], a[7]}); b.ms(3); const uint32_t r0 = b.mem.reads; b.cfg(DEV_FLASH, TRIG_SWITCH); b.in.armed = true; b.ms(5); b.tx.out.clear(); const uint32_t r1 = b.mem.reads; b.ms(50);
      CHECK(r0 > rb && b.svc.state() == ST_IDLE && b.tx.out.empty() && b.mem.reads == r1, "arming stops a running download (the device stays selected, no session starts): no flash reads in flight (%u before, %u after, state %d)", r0 - rb, b.mem.reads - r1, (int)b.svc.state()); }
}

// ------------------------------------------------------------------------------------------------------------------------------
// SD card
// ------------------------------------------------------------------------------------------------------------------------------
static std::vector<uint8_t> pattern(size_t n, uint8_t seed) { std::vector<uint8_t> v(n); for (size_t i = 0; i < n; i++) v[i] = (uint8_t)(seed + i * 7u); return v; }
// append in frame-sized pieces, like the service does
static bool feed(CardLog& c, const std::vector<uint8_t>& d, size_t piece = 200) { for (size_t i = 0; i < d.size(); i += piece) if (!c.append(d.data() + i, (uint32_t)(d.size() - i < piece ? d.size() - i : piece))) return false; return true; }

static void t_card() {
    printf("[card] file names, byte queue\n");
    { char n[32]; card_file_name(42, n, sizeof(n)); CHECK(!strcmp(n, "wb_00042.wblog"), "file 42 is '%s'", n);
      card_file_name(123456, n, sizeof(n)); CHECK(!strcmp(n, "wb_123456.wblog") && card_file_number(n) == 123456, "numbers above 99999 simply get longer");
      CHECK(card_file_number("wb_00042.wblog") == 42 && card_file_number("/wblog/wb_00007.wblog") == 7 && card_file_number("WB_00009.WBLOG") == 9 && card_file_number("wb_1.wblog") == 1, "names are recognised with a path, in upper case, without leading zeros");
      CHECK(card_file_number("wb_.wblog") == 0 && card_file_number("wb_12.wblo") == 0 && card_file_number("wb_12.wblogx") == 0 && card_file_number("wb_12.wblok") == 0 && card_file_number("wb_12.txtxxx") == 0 && card_file_number("xb_12.wblog") == 0 && card_file_number("wb_12a.wblog") == 0 &&
            card_file_number("notes.txt") == 0 && card_file_number("wb-12.wblog") == 0 && card_file_number("wb_1234567890.wblog") == 0 && card_file_number("") == 0 && card_file_number(nullptr) == 0 && card_file_number("wb_00000.wblog") == 0,
            "anything else is not a log file (and is never deleted)"); }
    { uint8_t mem[5000]; ByteFifo f; f.init(mem, sizeof(mem));
      CHECK(f.capacity() == 4096 && f.used() == 0 && f.room() == 4096, "5000 bytes of memory give a 4096-byte queue");
      const std::vector<uint8_t> a = pattern(3000, 1), b = pattern(2000, 100); uint8_t out[4096];     // (seeds chosen so that b's wrapped part differs from what a left at the start of the memory)
      CHECK(f.push(a.data(), 3000) && !f.push(b.data(), 2000) && f.used() == 3000, "a piece that does not fit is refused whole");
      CHECK(f.peek(out, 1000) == 1000 && !memcmp(out, a.data(), 1000) && f.used() == 3000, "peek does not remove");
      f.consume(2500); CHECK(f.push(b.data(), 2000) && f.used() == 2500, "room again after consume");
      CHECK(memcmp(mem, b.data() + 1096, 904) == 0 && memcmp(a.data(), b.data() + 1096, 904) != 0, "the second part of a piece that passes the end of the memory is stored at its start");
      CHECK(f.peek(out, 4096) == 2500 && !memcmp(out, a.data() + 2500, 500) && !memcmp(out + 500, b.data(), 2000), "the bytes come out in order across the wrap of the memory");
      f.clear(); CHECK(f.used() == 0 && f.peek(out, 10) == 0, "clear");
      ByteFifo none; none.init(nullptr, 0); CHECK(none.capacity() == 0 && !none.push(a.data(), 1) && none.peek(out, 4) == 0, "a queue without memory takes nothing"); }

    { // a 32 GB card: bytes do not fit 32 bits, the kB figures do
      MemCard m; m.size_bytes = 32ull * 1024 * 1024 * 1024; Lock lk; CardLog c(m, lk); std::vector<uint8_t> buf(8 * 1024); c.set_buffer(buf.data(), (uint32_t)buf.size());
      CHECK(c.begin() && c.capacity() == 0xFFFFFFFFu && c.capacity_kb() == 32u * 1024 * 1024 && c.used() == 0, "32 GB card: capacity in bytes saturates at 4 GB - 1, in kB it is exact (%u kB)", c.capacity_kb());
      CHECK(c.writable() == 0xFFF00000u, "one session can take at most the 4 GB address span (%u)", c.writable()); }
    printf("[card] sessions as files: chunks, sync, the table\n");
    { MemCard m; Lock lk; CardLog c(m, lk); std::vector<uint8_t> buf(64 * 1024); c.set_buffer(buf.data(), (uint32_t)buf.size());
      CHECK(c.begin() && c.ok() && c.sessions() == 0 && c.listed() == 0 && c.used() == 0 && c.capacity_kb() == 65536 && c.used_kb() == 0 && !c.needs_erase() && !c.full(), "an empty 64 MB card");
      CHECK(c.writable() == 63u * 1024 * 1024, "writable: everything but the 1 MB reserve (%u)", c.writable());
      CHECK(c.start_session() && c.writing() && m.opens == 1 && m.open_number == 1, "session 1 = file number 1");
      const std::vector<uint8_t> d1 = pattern(2000, 3);
      CHECK(feed(c, d1) && c.staged() == 2000 && m.writes == 0, "append only queues: nothing has touched the card");
      c.work(1000); CHECK(m.writes == 1 && m.syncs == 1 && c.staged() == 0 && m.file(1) == d1, "the first bytes of a session are written and synced at once (the header is safe)");
      const std::vector<uint8_t> d2 = pattern(5000, 77); feed(c, d2);
      c.work(1100); CHECK(m.writes == 2 && m.largest_write == 4096 && c.staged() == 904 && m.syncs == 1, "full 4 kB chunks are written as they come; the rest waits (%u staged)", c.staged());
      c.work(1249); CHECK(m.writes == 2 && m.syncs == 1, "249 ms after the last sync: nothing yet");
      c.work(1250); CHECK(m.writes == 3 && m.syncs == 2 && c.staged() == 0, "250 ms: the partial chunk and a sync - at most a quarter of a second is ever unsynced");
      c.work(1600); CHECK(m.syncs == 2 && m.writes == 3, "nothing new: no write, no sync");
      CHECK(c.used_kb() == 6 && c.writable() == 63u * 1024 * 1024 - 7000u, "used / writable follow the open file");
      const std::vector<uint8_t> d3 = pattern(300, 5); feed(c, d3);
      c.end_session();
      std::vector<uint8_t> all = d1; all.insert(all.end(), d2.begin(), d2.end()); all.insert(all.end(), d3.begin(), d3.end());
      CHECK(!c.writing() && m.open_number == -1 && m.file(1) == all && m.files[1].synced == 7300, "end: the queued rest is written, synced, the file closed (%zu bytes)", m.file(1).size());
      CHECK(c.sessions() == 1 && c.listed() == 1 && c.session_start(0) == 0 && c.session_size(0) == 7300 && c.used() == 7300 && c.session_number(0) == 1, "the session table: one file of 7300 bytes at address 0");
      // a second session
      const std::vector<uint8_t> e = pattern(9000, 200); c.start_session(); feed(c, e); c.work(3000); c.work(3300); c.end_session();
      CHECK(c.sessions() == 2 && c.session_start(1) == 7300 && c.session_size(1) == 9000 && c.used() == 16300 && m.file(2) == e, "session 2 = file 2, placed after session 1 in the download addresses");
      // reading back, across the file boundary
      uint8_t rd[400]; CHECK(c.read(7200, rd, 300) && !memcmp(rd, all.data() + 7200, 100) && !memcmp(rd + 100, e.data(), 200), "a read across two sessions returns the end of one file and the start of the next");
      CHECK(!c.read(16300, rd, 1) && !c.read(16200, rd, 101) && c.read(16200, rd, 100) && !memcmp(rd, e.data() + 8900, 100), "reads stop exactly at the end of the data");
      c.start_session(); CHECK(!c.read(0, rd, 10), "nothing is read back while a session is being written"); c.end_session();
      CHECK(c.sessions() == 2 && m.files.count(3) == 0 && c.next_number() == 4, "a session that logged nothing leaves no empty file (but its number is used up)");
      // after a reset: the same table, found from the directory (which lists the files in no particular order)
      m.put(7, 1234); m.put(5, 4321);
      CardLog again(m, lk); again.set_buffer(buf.data(), (uint32_t)buf.size());
      CHECK(again.begin() && again.sessions() == 4 && again.listed() == 4 && again.session_number(0) == 1 && again.session_number(1) == 2 && again.session_number(2) == 5 && again.session_number(3) == 7 &&
            again.session_start(2) == 16300 && again.session_size(2) == 4321 && again.session_start(3) == 20621 && again.used() == 21855 && again.next_number() == 8, "after a reset: files sorted by number, next file is 8"); }

    printf("[card] what can go wrong with a card\n");
    { // power cut: what was synced survives
      MemCard m; Lock lk; std::vector<uint8_t> buf(64 * 1024);
      { CardLog c(m, lk); c.set_buffer(buf.data(), (uint32_t)buf.size()); c.begin(); c.start_session();
        feed(c, pattern(6000, 1)); c.work(1000);                               // 6000 bytes written and synced at t = 1000
        feed(c, pattern(4096, 2)); c.work(1100);                               // one more chunk written; the next sync is not due before t = 1250
        feed(c, pattern(100, 3));                                              // still in the queue
        CHECK(m.file(1).size() == 10096 && m.files[1].synced == 6000, "before the cut: 10096 bytes written, 6000 synced");
        m.power_cut(); }
      { CardLog c(m, lk); c.set_buffer(buf.data(), (uint32_t)buf.size());
        CHECK(c.begin() && c.sessions() == 1 && c.session_size(0) == 6000 && m.file(1) == pattern(6000, 1), "after the power cut the file holds exactly what had been synced: 6000 bytes");
        c.start_session(); m.power_cut(); }                                     // cut right after the file was created
      { CardLog c(m, lk); c.set_buffer(buf.data(), (uint32_t)buf.size());
        CHECK(c.begin() && c.sessions() == 1 && c.listed() == 1 && c.next_number() == 3, "an empty file left by a power cut is not a session, and its number is not used again"); } }
    { // the card is pulled out in flight, and pushed back in later
      MemCard m; Lock lk; CardLog c(m, lk); std::vector<uint8_t> buf(64 * 1024); c.set_buffer(buf.data(), (uint32_t)buf.size()); c.configure_device(true, 0, 20); c.begin(); c.start_session();
      feed(c, pattern(5000, 1)); c.work(1000); m.pull(); feed(c, pattern(5000, 2)); c.work(1010);
      CHECK(c.io_error() && !c.writing() && !c.ok() && !c.append(buf.data(), 10) && c.writable() == 0 && !m.mounted, "a failed write ends the session at once: error, nothing more is accepted");
      const int mounts = m.mounts; c.work(2000); c.work(3000); CHECK(m.mounts == mounts, "no card: not hammered - the next look is 3 s later");
      c.work(4010); CHECK(m.mounts == mounts + 1 && !c.ok(), "3 s later: looked again, still no card");
      m.push(); c.work(5000); c.work(7011);
      CHECK(c.ok() && !c.io_error() && c.sessions() == 1 && c.session_size(0) == 5000 && c.next_number() == 2, "card back: mounted again by itself, the interrupted session is listed with what reached the card");
      CHECK(c.start_session() && m.open_number == 2, "and logging goes on in the next file"); c.end_session();
      // pulled while idle: noticed within 3 s, without a session having to fail first
      c.work(8000); c.work(11000); CHECK(c.ok(), "idle and fine"); m.pull(); c.work(11500); CHECK(c.ok(), "probe not due yet"); c.work(14001);
      CHECK(!c.ok(), "pulled while idle: noticed by the periodic check"); }
    { MemCard m; m.formatted = false; Lock lk; CardLog c(m, lk); std::vector<uint8_t> buf(16 * 1024); c.set_buffer(buf.data(), (uint32_t)buf.size());
      CHECK(!c.begin() && !c.ok() && !strcmp(c.why(), "card is not FAT formatted") && !c.start_session() && c.capacity() == 0 && c.sessions() == 0, "an unformatted card: not usable, with the reason");
      MemCard none; none.inserted = false; CardLog d(none, lk); d.set_buffer(buf.data(), (uint32_t)buf.size()); CHECK(!d.begin() && !strcmp(d.why(), "no SD card"), "no card: the reason says so"); }
    { MemCard m; m.read_only = true; Lock lk; CardLog c(m, lk); std::vector<uint8_t> buf(16 * 1024); c.set_buffer(buf.data(), (uint32_t)buf.size());
      CHECK(c.begin() && !c.start_session() && !c.ok() && c.io_error(), "write-protected card: the file cannot be created - reported as an error, no session"); }
    { MemCard m; Lock lk; CardLog c(m, lk); CHECK(c.begin() && !c.start_session(), "no write buffer (out of memory): no session is started"); }
    { // the queue is full because the card is slow: the caller is told, nothing is half-written
      MemCard m; Lock lk; CardLog c(m, lk); std::vector<uint8_t> buf(8 * 1024); c.set_buffer(buf.data(), (uint32_t)buf.size()); c.begin(); c.start_session();
      const std::vector<uint8_t> f = pattern(240, 1); int taken = 0; while (taken < 100 && c.append(f.data(), 240)) taken++;
      CHECK(taken == 34 && c.staged() == 34u * 240u && c.take_buffer_peak() == 100 && c.take_buffer_peak() == 0, "8 kB queue, card not answering: 34 frames of 240 bytes fit, the 35th is refused; peak level 100 %% (read once)");
      c.work(1000); CHECK(c.append(f.data(), 240) && c.take_buffer_peak() < 10, "the writer catches up: room again"); c.end_session();
      CHECK(m.file(1).size() == 35u * 240u, "all 35 accepted frames are in the file, complete"); }
    { // a full card
      MemCard m; m.size_bytes = 2 * 1024 * 1024; Lock lk; CardLog c(m, lk); std::vector<uint8_t> buf(64 * 1024); c.set_buffer(buf.data(), (uint32_t)buf.size()); c.begin();
      CHECK(c.writable() == 1024u * 1024u && c.start_session(), "2 MB card: 1 MB can be logged");
      const std::vector<uint8_t> f = pattern(1000, 1); uint32_t total = 0; uint32_t t = 1000;
      for (int i = 0; i < 2000; i++) { if (!c.append(f.data(), 1000)) break; total += 1000; if (i % 8 == 0) c.work(t += 5); }
      CHECK(total == 1048000u && c.writable() < 1000 && !c.io_error(), "it stops taking data when 1 MB is reached (%u bytes), with the reserve untouched and without an error", total);
      c.end_session(); CHECK(c.full() && !c.start_session() && c.sessions() == 1 && c.session_size(0) == total && m.free_bytes() >= 1024u * 1024u - 576u, "full: no new session; the file is complete; the card still has its reserve"); }
    { // more files than the table holds, and more bytes than the download addresses can span
      MemCard m; for (uint32_t i = 1; i <= 140; i++) m.put(i, 100 + i); Lock lk; CardLog c(m, lk); c.begin();
      CHECK(c.sessions() == 140 && c.listed() == 128 && c.session_number(0) == 13 && c.session_number(127) == 140 && c.session_size(0) == 113 && c.next_number() == 141, "140 files: all counted, the newest 128 listed (13 .. 140)");
      struct Huge : MemCard { bool list(ListFn fn, void* ctx) override { fn(ctx, 1, 2000u * 1024u * 1024u); fn(ctx, 2, 2000u * 1024u * 1024u); fn(ctx, 3, 2000u * 1024u * 1024u); return true; } } h;
      CardLog big(h, lk); big.begin();
      CHECK(big.sessions() == 3 && big.listed() == 2 && big.session_number(0) == 2 && big.session_start(1) == 2000u * 1024u * 1024u && big.used() == 4000u * 1024u * 1024u, "three 2 GB logs: the download range (4 GB) lists the newest two"); }

    printf("[card] erase: only our files\n");
    { MemCard m; for (uint32_t i = 1; i <= 9; i++) m.put(i * 3, 1000); m.other["notes.txt"] = 500; m.other["wb_holiday.wblog"] = 700;
      Lock lk; CardLog c(m, lk); std::vector<uint8_t> buf(16 * 1024); c.set_buffer(buf.data(), (uint32_t)buf.size()); c.begin();
      CHECK(c.sessions() == 9 && c.erase_start() && c.erase_percent() == 0, "9 log files, erase started");
      CHECK(c.erase_step() && m.files.size() == 5 && c.erase_percent() == 44, "a step deletes 4 files (44 %%)");
      CHECK(c.erase_step() && m.files.size() == 1 && c.erase_percent() == 88, "4 more (88 %%)");
      CHECK(c.erase_step() && m.files.empty() && c.erase_percent() == 99, "the last one; 99 %% until the directory has been looked at once more");
      CHECK(!c.erase_step() && c.erase_percent() == 100 && c.sessions() == 0 && c.listed() == 0 && c.used() == 0 && c.next_number() == 1, "done: empty table, numbering starts at 1 again");
      CHECK(m.other.size() == 2 && m.removes == 9, "other files in the folder were not touched");
      c.start_session(); CHECK(!c.erase_start(), "no erase while a session is open"); c.end_session();
      m.put(4, 10); m.read_only = true; c.erase_start(); CHECK(!c.erase_step() && c.io_error() && !c.ok(), "a card that refuses to delete: error instead of a loop"); }

    printf("[card] selected or not, rewired in the settings\n");
    { MemCard m; Lock lk; CardLog c(m, lk); std::vector<uint8_t> buf(16 * 1024); c.set_buffer(buf.data(), (uint32_t)buf.size());
      for (uint32_t t = 0; t < 20000; t += 100) c.work(t);
      CHECK(m.mounts == 0 && !c.ok(), "another log device is selected: the card is never touched");
      c.configure_device(true, 0, 20); c.work(20100); CHECK(m.mounts == 1 && c.ok(), "selected in the app: mounted by the writer task, no reboot needed");
      c.configure_device(true, 1, 20); c.work(20200); c.work(20201);
      CHECK(m.mounts == 2 && c.ok(), "wiring changed to SDMMC in the settings: unmounted and mounted again with the new setting");
      c.start_session(); c.configure_device(true, 0, 10); c.work(20300); CHECK(m.mounts == 2 && c.writing(), "not in the middle of a session"); c.end_session();
      c.work(20400); c.work(20401); CHECK(m.mounts == 3, "afterwards it is applied");
      c.start_session(); const uint32_t num = (uint32_t)m.open_number; feed(c, pattern(5000, 1)); c.configure_device(false, 0, 10); c.work(20500); c.end_session();
      CHECK(m.file(num).size() == 5000, "deselected while a session runs: that session is still finished properly"); }
}

static bool ended(const RSession& s) { return !s.events.empty() && s.events.back().code == EV_END; }
static int end_reason(const RSession& s) { return ended(s) && s.events.back().data.size() == 1 ? s.events.back().data[0] : -1; }
static uint32_t lost_records(const RSession& s) { uint32_t lost = 0; for (const RSession::Ev& e : s.events) if (e.code == EV_DROPPED && e.data.size() == 4) { uint32_t n; memcpy(&n, e.data.data(), 4); lost += n; } return lost; }

// The service with an SD card. The writer task is played here: it runs once a millisecond unless the card is keeping it busy.
struct CardBench {
    Recorder rec; FakeTx tx; MemFlash mem; MemCard card; Lock lk; CardLog log; std::vector<uint8_t> buf; Service svc; wbp::Settings settings; FirmwareHeader hdr;
    Inputs in; uint32_t us = 0, cycles = 0, busy_until_ms = 0; float gyro[3] = {0, 0, 0}, motor[8] = {0.1f, 0.2f, 0.3f, 0.4f}; app::Telemetry tel;
    int card_ops_in_poll = 0;
    explicit CardBench(uint32_t buffer = 512 * 1024, bool inserted = true) : mem(256 * 1024), log(card, lk), buf(buffer), svc(rec, tx), hdr{&settings, "Test FW", 1, 2, 3} {
        wbp::defaults(settings); card.inserted = inserted; const bool mount = inserted;
        log.set_buffer(buf.data(), (uint32_t)buf.size());
        card.busy = [this](uint32_t t_us) { busy_until_ms = (busy_until_ms > in.now_ms ? busy_until_ms : in.now_ms) + (t_us + 999) / 1000; };
        svc.attach_store(&mem); svc.attach_card(&log); svc.set_header_source(&FirmwareHeader::write, &hdr); svc.set_loop(4000.0f, 4);
        Config c; c.device = DEV_SDCARD; c.trigger = TRIG_ARMED; c.rate_hz = 500; c.groups = (1u << G_GYRO) | (1u << G_MOTORS) | (1u << G_TIMING); svc.set_config(c);
        in.now_ms = 5000; us = 5000000;
        if (mount) log.begin();
        ms(200);
    }
    void cfg(uint8_t device, uint8_t trigger, uint16_t rate = 500) { Config c = svc.config(); c.device = device; c.trigger = trigger; c.rate_hz = rate; svc.set_config(c); }
    void ms(int n = 1) {
        for (int k = 0; k < n; k++) {
            for (int c = 0; c < 4; c++) {
                us += 250; cycles++; gyro[0] = (float)(cycles % 1000) * 0.1f;
                rec.note_state(us, in.armed, 0, 0, 1, 0);
                if (rec.fast_due(250)) { FastSample s; s.t_us = us; s.gyro = gyro; s.motor = motor; s.loop_us = 80; rec.log_fast(s); }
                if (cycles % 400 == 0) { tel.armed = in.armed; rec.log_slow(us, tel); }
            }
            in.now_ms++; tx.pending = 0;
            const int w0 = card.writes + card.syncs; svc.poll(in); if (svc.state() == ST_LOGGING && !closing_now()) card_ops_in_poll += card.writes + card.syncs - w0;
            if (in.now_ms >= busy_until_ms) log.work(in.now_ms);
        }
    }
    bool closing_now() const { return !rec.active(); }       // the service has stopped the recorder: it is closing the session (on the ground)
    wbp::Err cmd(uint8_t c, std::initializer_list<uint8_t> a = {}, std::vector<uint8_t>* res = nullptr) {
        uint8_t args[8] = {}; int i = 0; for (uint8_t x : a) args[i++] = x;
        uint8_t r[64]; uint16_t rl = 0; const wbp::Err e = svc.command(c, args, r, rl); if (res) res->assign(r, r + rl); return e;
    }
    std::vector<RSession> file(uint32_t number) { const std::vector<uint8_t>& d = card.file(number); return read_log(d.data(), d.size()); }
    wbp::T_blackbox status() { wbp::T_blackbox st{}; uint8_t b[64]; svc.pack_status(b, sizeof(b)); memcpy(&st, b, sizeof(st)); return st; }
};

static void t_service_card() {
    printf("[service + card] a flight in a file\n");
    { CardBench b;
      CHECK(b.svc.state() == ST_IDLE && b.svc.device() == DEV_SDCARD && b.card.writes == 0, "card selected, disarmed: idle");
      b.in.armed = true; b.ms(2000);
      CHECK(b.svc.state() == ST_LOGGING && b.rec.active() && b.card.open_number == 1 && b.mem.writes == 0, "armed: logging to file 1, nothing goes to the flash");
      CHECK(b.card_ops_in_poll == 0, "the link task never waits for the card while a session runs (card operations inside poll: %d)", b.card_ops_in_poll);
      const wbp::T_blackbox st = b.status();
      CHECK(st.device == DEV_SDCARD && st.state == ST_LOGGING && st.capacity_kb == 65536 && st.used_kb >= 18 && st.used_kb <= 30 && st.rate_hz >= 499 && st.rate_hz <= 501, "status: device SDCARD, 64 MB, %u kB used, %u records/s", st.used_kb, st.rate_hz);
      b.in.armed = false; b.ms(400);
      const std::vector<RSession> ss = b.file(1);
      CHECK(b.svc.state() == ST_IDLE && b.card.open_number == -1 && ss.size() == 1 && b.svc.log().sessions() == 1, "disarmed: file closed, one session");
      // 2.0 s at 500 Hz = 1000 fast records, 20 slow; timestamps 2 ms apart; the END event carries the reason
      bool spacing = ss.size() == 1 && ss[0].F.size() >= 998; for (size_t i = 1; spacing && i < ss[0].F.size(); i++) if (ss[0].F[i][0] - ss[0].F[i - 1][0] != 2000.0) spacing = false;
      CHECK(ss.size() == 1 && ss[0].F.size() >= 998 && ss[0].F.size() <= 1002 && spacing && ss[0].S.size() >= 19 && ss[0].bad_records == 0 && ss[0].seq_gaps == 0 && lost_records(ss[0]) == 0 && ended(ss[0]) && end_reason(ss[0]) == END_TRIGGER_OFF,
            "the file decodes: %zu fast records 2 ms apart, complete, ended 'trigger off'", ss.empty() ? (size_t)0 : ss[0].F.size());
      CHECK(ss[0].meta.at("session") == "1" && ss[0].meta.at("to") == "storage" && ss[0].meta.count("firmware"), "with the normal header");
      CHECK(b.card.largest_write <= 4096 && b.card.syncs >= 7 && b.card.syncs <= 12, "written in chunks of at most 4 kB, synced about 4 times a second (%d syncs in 2 s)", b.card.syncs);
      // the second flight is the second file
      b.in.armed = true; b.ms(500); b.in.armed = false; b.ms(400);
      CHECK(b.card.files.size() == 2 && b.file(2).size() == 1 && b.file(2)[0].meta.at("session") == "2" && b.svc.log().sessions() == 2, "second flight: wb_00002"); }

    printf("[service + card] a slow card\n");
    { // 2 kHz logging on a card that freezes for 180 ms every 20th write: the buffer takes it
      CardBench b; b.card.stall_every = 20; b.cfg(DEV_SDCARD, TRIG_ARMED, 2000); b.in.armed = true; b.ms(6000);
      const wbp::T_blackbox st = b.status(); b.in.armed = false; b.ms(800);
      const std::vector<RSession> ss = b.file(1);
      CHECK(ss.size() == 1 && ss[0].F.size() >= 11990 && lost_records(ss[0]) == 0 && ss[0].seq_gaps == 0 && b.rec.ring().dropped() == 0, "6 s at 2 kHz: all %zu records in the file, none dropped", ss.empty() ? (size_t)0 : ss[0].F.size());
      CHECK(b.card_ops_in_poll == 0 && st.buffer_pct >= 1 && st.buffer_pct < 20, "the stalls were absorbed by the buffer (peak fill %u %%), never by the link task", st.buffer_pct); }
    { // the same card with a buffer that is too small: records are lost, counted, and the log says where
      CardBench b(8 * 1024); b.card.stall_every = 20; b.card.stall_us = 600000; b.cfg(DEV_SDCARD, TRIG_ARMED, 2000); b.in.armed = true; b.ms(6000);
      const wbp::T_blackbox st = b.status(); b.in.armed = false; b.ms(1500);
      const std::vector<RSession> ss = b.file(1);
      CHECK(ss.size() == 1 && b.rec.ring().dropped() > 100 && lost_records(ss[0]) == b.rec.ring().dropped() && ss[0].bad_records == 0 && ended(ss[0]), "8 kB buffer and 0.6 s stalls: %u records lost, every one of them counted in the log, the rest intact", b.rec.ring().dropped());
      CHECK(st.dropped > 0, "and the status stream shows it (%u dropped)", st.dropped); }

    printf("[service + card] download, erase\n");
    { CardBench b; b.in.armed = true; b.ms(1000); b.in.armed = false; b.ms(400); b.in.armed = true; b.ms(700);
      std::vector<uint8_t> res;
      CHECK(b.cmd(wbp::CMD_BLACKBOX_READ, {0, 0, 0, 0, 0, 1, 0, 0}) == wbp::ERR_BUSY, "no download while a session is being written");
      b.in.armed = false; b.ms(400);
      CHECK(b.cmd(wbp::CMD_BLACKBOX_SESSION, {1, 0}, &res) == wbp::ERR_OK && res.size() == 8, "SESSION 1");
      uint32_t st1, sz1; memcpy(&st1, res.data(), 4); memcpy(&sz1, res.data() + 4, 4);
      CHECK(st1 == b.card.file(1).size() && sz1 == b.card.file(2).size() && b.cmd(wbp::CMD_BLACKBOX_SESSION, {2, 0}) == wbp::ERR_BAD_ARG, "its address is right behind file 1, its size is the size of file 2");
      uint8_t a[8]; memcpy(a, &st1, 4); memcpy(a + 4, &sz1, 4); b.tx.out.clear();
      CHECK(b.cmd(wbp::CMD_BLACKBOX_READ, {a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7]}) == wbp::ERR_OK, "READ session 1");
      std::vector<uint8_t> got(sz1, 0xEE); uint32_t received = 0;
      for (int polls = 0; polls < 3000 && received < sz1; polls++) {
          b.tx.out.clear(); b.ms(1); wbp::Parser p; wbp::Frame f;
          for (uint8_t c : b.tx.out) if (p.feed(c, f) && f.type == kMsgBlob) { uint32_t off; memcpy(&off, f.payload, 4); memcpy(got.data() + (off - st1), f.payload + 4, f.len - 4u); received += f.len - 4u; }
      }
      CHECK(received == sz1 && got == b.card.file(2), "the downloaded bytes are the file (%u bytes)", received);
      // erase
      b.card.other["flight notes.txt"] = 100;
      CHECK(b.cmd(wbp::CMD_BLACKBOX_ERASE) == wbp::ERR_OK && b.svc.state() == ST_ERASING && b.rec.busy(), "ERASE: erasing, the craft is kept on the ground");
      b.ms(50);
      CHECK(b.svc.state() == ST_IDLE && !b.rec.busy() && b.rec.take_erase_done() && b.card.files.empty() && b.card.other.size() == 1 && b.svc.log().sessions() == 0, "erased: the log files are gone, the other file is still there");
      b.in.armed = true; b.ms(500); b.in.armed = false; b.ms(400); CHECK(b.card.files.count(1) == 1 && b.file(1).size() == 1, "the next flight is wb_00001 again"); }

    printf("[service + card] no card, card pulled, device switched\n");
    { CardBench b(64 * 1024, false); b.ms(100);
      CHECK(b.svc.state() == ST_NO_STORAGE, "no card: NO_STORAGE"); b.in.armed = true; b.ms(1000); CHECK(!b.rec.active() && b.svc.state() == ST_NO_STORAGE, "arming without a card: no session, the recorder stays off");
      b.in.armed = false; b.card.push(); b.ms(3200);
      CHECK(b.svc.state() == ST_IDLE, "card pushed in: found within 3 s, without a reboot"); b.in.armed = true; b.ms(600);
      CHECK(b.svc.state() == ST_LOGGING, "and the next flight is logged");
      b.card.pull(); b.ms(600);
      CHECK(b.svc.state() == ST_NO_STORAGE && !b.rec.active() && b.rec.ring().used() == 0, "card pulled in flight: the session ends, the recorder stops (nothing piles up)");
      b.card.push(); b.ms(3500);
      CHECK(b.svc.state() == ST_LOGGING && b.card.open_number == 2 && b.svc.log().sessions() == 1, "card back while still armed: a new session in the next file; the interrupted one is listed");
      b.in.armed = false; b.ms(500);
      const std::vector<RSession> cut = b.file(1), next = b.file(2);
      CHECK(cut.size() == 1 && !ended(cut[0]) && cut[0].bad_records == 0 && !cut[0].F.empty() && next.size() == 1 && ended(next[0]), "the interrupted file reads cleanly up to where the card went away; the new one is complete"); }
    { // the device is changed in the app (on the ground: settings cannot be changed while armed)
      CardBench b; b.cfg(DEV_SDCARD, TRIG_ALWAYS); b.ms(500);
      CHECK(b.svc.state() == ST_LOGGING && b.card.open_number == 1 && b.mem.reads < 200, "trigger ALWAYS on the card: logging; the flash was only looked at once at boot (%u reads)", b.mem.reads);
      b.cfg(DEV_FLASH, TRIG_ALWAYS); b.ms(600);
      CHECK(b.svc.state() == ST_LOGGING && b.svc.device() == DEV_FLASH && b.card.open_number == -1 && b.mem.writes > 0, "device changed card -> flash: the card session is closed, the next one goes to the flash");
      const std::vector<RSession> c1 = b.file(1); CHECK(c1.size() == 1 && ended(c1[0]) && end_reason(c1[0]) == END_TRIGGER_OFF, "the card file was finished properly");
      const int card_ops = b.card.writes + b.card.syncs + b.card.lists; b.ms(4000); CHECK(b.card.writes + b.card.syncs + b.card.lists == card_ops, "while the flash is selected the card is left alone");
      b.cfg(DEV_SDCARD, TRIG_ALWAYS); b.ms(600);
      CHECK(b.svc.state() == ST_LOGGING && b.svc.device() == DEV_SDCARD && b.card.open_number == 2 && b.svc.flash().sessions() == 1 && b.svc.log().sessions() == 1, "and back: flash session closed, file 2 on the card");
      b.cfg(DEV_SDCARD, TRIG_ARMED); b.ms(400); const wbp::T_blackbox st = b.status(); CHECK(st.device == DEV_SDCARD && st.sessions == 2, "status describes the selected device (%u sessions on the card)", st.sessions);
      b.cfg(DEV_NONE, TRIG_ARMED); b.ms(10); const wbp::T_blackbox s0 = b.status(); CHECK(s0.state == ST_OFF && s0.device == DEV_NONE && s0.sessions == 1 && s0.capacity == 256 * 1024, "device NONE: off; the flash stays readable"); }
    { // SDCARD selected on a firmware without a card interface
      Recorder rec; FakeTx tx; Service svc(rec, tx); MemFlash mf(256 * 1024); svc.attach_store(&mf); Config c; c.device = DEV_FLASH; c.trigger = TRIG_SWITCH; svc.set_config(c); Inputs in; for (int i = 0; i < 600; i++) { in.now_ms++; svc.poll(in); }
      const bool flash_ready = svc.state() == ST_IDLE; const uint32_t w0 = mf.writes;
      c.device = DEV_SDCARD; c.trigger = TRIG_ALWAYS; svc.set_config(c); for (int i = 0; i < 600; i++) { in.now_ms++; svc.poll(in); }
      CHECK(flash_ready && svc.state() == ST_NO_STORAGE && !rec.active() && !svc.log().ok() && mf.writes == w0, "SDCARD without a card interface: NO_STORAGE - the log does not quietly go to the flash instead (%u writes)", mf.writes - w0); }
}

int main() {
    setvbuf(stdout, nullptr, _IOLBF, 0);
    t_format();
    t_ring();
    t_recorder();
    t_flash();
    t_service();
    t_link_budget();
    t_card();
    t_service_card();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
