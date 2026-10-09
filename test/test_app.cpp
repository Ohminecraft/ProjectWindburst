// Portable app-layer pieces: telemetry packing, the flight-loop <-> link-task exchange (with real threads), the request gate,
// accelerometer / gyro calibration maths, level trim, settings validation.
// Build: g++ -std=c++17 -O2 -pthread test/test_app.cpp -o build/test_app
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <atomic>
#include <chrono>
#include <random>
#include <thread>
#include <vector>
#include <functional>
#include "../src/app/link_host.h"
#include "../src/app/sync.h"
#include "../src/app/tx_queue.h"
#include "../src/proto/settings_apply.h"
#include "../src/config/hardware_config.h"
#include "../src/config/pin_check.h"
#include "../src/app/sysload.h"

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, ...) do { const bool ok_ = (cond); if (getenv("V")) { printf("    "); printf(__VA_ARGS__); printf("\n"); } if (ok_) g_pass++; else { g_fail++; printf("  FAIL line %d: %s   ", __LINE__, #cond); printf(__VA_ARGS__); printf("\n"); } } while (0)
using namespace wbp;

static void t_pack() {
    printf("[telemetry] wire structs: scaling, saturation, NaN\n");
    app::Telemetry t; uint8_t b[96];
    t.roll = 12.345f; t.pitch = -7.891f; t.yaw = -179.99f; t.gyro_f[0] = 123.44f; t.gyro_f[2] = -2000.06f; t.sp[1] = 360.0f;
    CHECK(app::pack_stream(STR_ATTITUDE, t, 0, b, sizeof(b)) == sizeof(T_attitude), "attitude size");
    { T_attitude a; memcpy(&a, b, sizeof(a)); CHECK(a.roll == 1235 && a.pitch == -789 && a.yaw == -17999 && a.gyro[0] == 1234 && a.gyro[2] == -20001 && a.sp[1] == 3600, "0.01 deg / 0.1 deg/s, rounded to nearest: %d %d %d %d %d %d", a.roll, a.pitch, a.yaw, a.gyro[0], a.gyro[2], a.sp[1]); }
    t.gyro_f[0] = 5000.0f; t.gyro_f[1] = -5000.0f; t.roll = NAN; t.pitch = INFINITY;
    app::pack_stream(STR_ATTITUDE, t, 0, b, sizeof(b));
    { T_attitude a; memcpy(&a, b, sizeof(a)); CHECK(a.gyro[0] == 32767 && a.gyro[1] == -32768, "out of range saturates instead of wrapping: %d %d", a.gyro[0], a.gyro[1]); CHECK(a.roll == 0 && a.pitch == 0, "NaN / infinity become 0: %d %d", a.roll, a.pitch); }
    t.armed = true; t.blockers = 0x1005; t.failsafe = 2; t.modes = 0x0B; t.health = 0x7FF; t.ah_state = 4; t.loop_us = 87; t.loop_max_us = 190; t.loop_hz = 3987; t.cell_v = 3.912f; t.throttle = 0.456f;
    CHECK(app::pack_stream(STR_STATUS, t, 3, b, sizeof(b)) == sizeof(T_status), "status size");
    t.t_imu_us = 31; t.t_core_us = 62; t.t_motor_us = 13; t.t_rest_us = 9; t.t_slow_max_us = 412; t.overruns = 17; t.loop_target_hz = 4000;
    CHECK(app::pack_stream(STR_STATUS, t, 3, b, sizeof(b)) == sizeof(T_status), "status size");
    { T_status s; memcpy(&s, b, sizeof(s)); CHECK(s.armed == 1 && s.blockers == 0x1005 && s.failsafe == 2 && s.modes == 0x0B && s.health == 0x7FF && s.ah_state == 4 && s.loop_us == 87 && s.loop_max_us == 190 && s.loop_hz == 3987 && s.cell_mv == 3912 && s.settings_dirty == 3 && s.throttle == 456, "status fields");
      CHECK(s.t_imu == 31 && s.t_core == 62 && s.t_motor == 13 && s.t_rest == 9 && s.t_slow_max == 412 && s.overruns == 17 && s.loop_target == 4000, "status: time per stage, lost cycles, configured rate"); }
    for (int i = 0; i < 8; i++) { t.motor_cmd[i] = 0.1f * i; t.rpm[i] = 1000.0f * i; t.esc_err_pct[i] = (uint8_t)i; t.esc_temp[i] = (uint8_t)(30 + i); }
    t.rpm[7] = 900000.0f; t.esc_volt = 16.42f; t.dyn_idle = 0.031f; t.bidir_state = BIDIR_STATE_UNAVAILABLE; t.esc_answers = 0x0B; t.hw_skipped = 1234; t.hw_late = 56;
    app::pack_stream(STR_MOTORS, t, 0, b, sizeof(b));
    { T_motors m; memcpy(&m, b, sizeof(m)); CHECK(m.bidir == BIDIR_STATE_UNAVAILABLE && m.answers == 0x0B && m.skipped == 1234 && m.late == 56, "motors: output driver state"); }
    { T_motors m; memcpy(&m, b, sizeof(m)); CHECK(m.cmd[3] == 300 && m.rpm[5] == 500 && m.rpm[7] == 65535 && m.err[6] == 6 && m.temp[2] == 32 && m.volt[0] == 1642 && m.dyn_idle == 31, "motors: cmd x1000, rpm/10 (saturating), volt x100: %u %u %u", m.cmd[3], m.rpm[5], m.rpm[7]); }
    t.gyro_raw[1] = -45.67f; t.acc[2] = -0.9987f; t.mag[0] = 0.2504f; t.baro_alt = -12.345f; t.baro_pa = 101325.4f; t.temp_c = 24.56f; t.range_m = 1.2345f;
    app::pack_stream(STR_SENSORS, t, 0, b, sizeof(b));
    { T_sensors s; memcpy(&s, b, sizeof(s)); CHECK(s.gyro[1] == -457 && s.acc[2] == -999 && s.mag[0] == 250 && s.baro_alt == -1235 && s.baro_pa == 101325 && s.temp == 2456 && s.range == 1235, "sensors: %d %d %d %d %u %d %d", s.gyro[1], s.acc[2], s.mag[0], s.baro_alt, s.baro_pa, s.temp, s.range); }
    t.gps_fix = 3; t.gps_sats = 17; t.lat = 473977419; t.lon = -85455938; t.alt_cm = 48820; t.gs_cms = 1234; t.course_ddeg = 3599; t.hdop = 85; t.hacc_mm = 70000; t.home_dist_m = 123456; t.home_dir_ddeg = 1800; t.gps_state = 5; t.gps_baud = 115200;
    app::pack_stream(STR_GPS, t, 0, b, sizeof(b));
    { T_gps g; memcpy(&g, b, sizeof(g)); CHECK(g.fix == 3 && g.sats == 17 && g.lat == 473977419 && g.lon == -85455938 && g.alt == 48820 && g.gs == 1234 && g.course == 3599 && g.hdop == 85 && g.hacc == 65535 && g.home_dist == 65535 && g.home_dir == 1800 && g.state == 5 && g.baud == 115200, "gps passes through, 32-bit values saturate into their 16-bit fields"); }
    t.z = 12.345f; t.vz = -1.239f; t.z_sigma = 0.0456f; t.alt_valid = true; t.pos_valid = false; t.yaw_from_mag = true; t.pos_n = -20.04f; t.pos_e = 3.96f; t.vel_n = 0.126f; t.vel_e = -9.994f;
    app::pack_stream(STR_ESTIMATE, t, 0, b, sizeof(b));
    { T_estimate e; memcpy(&e, b, sizeof(e)); CHECK(e.z == 1235 && e.vz == -124 && e.z_sigma == 46 && e.valid == (EST_VALID_ALT | EST_VALID_YAW_FROM_MAG) && e.pos_n == -200 && e.pos_e == 40 && e.vel_n == 13 && e.vel_e == -999, "estimate: %d %d %u %u %d %d %d %d", e.z, e.vz, e.z_sigma, e.valid, e.pos_n, e.pos_e, e.vel_n, e.vel_e); }
    t.vel_valid = true; t.flow_valid = true; t.tilt_aided = true; t.pos_valid = true; app::pack_stream(STR_ESTIMATE, t, 0, b, sizeof(b));
    { T_estimate e; memcpy(&e, b, sizeof(e)); CHECK(e.valid == (EST_VALID_ALT | EST_VALID_POS | EST_VALID_YAW_FROM_MAG | EST_VALID_VEL | EST_VALID_FLOW | EST_VALID_TILT_AIDED) && app::est_valid_bits(t) == e.valid, "estimate: every valid bit (0x%x)", e.valid); }
    t.nav_flags = NAV_FLAGS_POSHOLD | NAV_FLAGS_NAV_LEAN; t.flow_quality = 78; t.flow_reject = FLOW_REJECT_TILT; t.flow_rate = 99; t.flow_fwd = -1.234f; t.flow_right = 400.0f; t.ph_state[0] = PH_AXIS_BRAKE; t.ph_state[1] = PH_AXIS_LOITER;
    t.nav_roll = -12.34f; t.nav_pitch = 7.06f; t.nav_target_n = -123.44f; t.nav_target_e = 5000.0f; t.nav_wind_n = 1.236f; t.nav_wind_e = -0.004f; t.rescue_phase = RESCUE_PHASE_FLY_HOME; t.rescue_fail = RESCUE_FAIL_LOW_SATS;
    t.nav_home_dist = 432.16f; t.rescue_return_alt = 35.55f;
    { const uint16_t n = app::pack_stream(STR_NAV, t, 0, b, sizeof(b)); T_nav v; memcpy(&v, b, sizeof(v));
      CHECK(n == sizeof(T_nav) && v.flags == (NAV_FLAGS_POSHOLD | NAV_FLAGS_NAV_LEAN) && v.flow_quality == 78 && v.flow_reject == FLOW_REJECT_TILT && v.flow_rate == 99 && v.ph_roll == PH_AXIS_BRAKE && v.ph_pitch == PH_AXIS_LOITER && v.rescue_phase == RESCUE_PHASE_FLY_HOME && v.rescue_fail == RESCUE_FAIL_LOW_SATS, "nav: flags and states");
      CHECK(v.flow_fwd == -123 && v.flow_right == 32767 && v.lean_roll == -123 && v.lean_pitch == 71 && v.target_n == -1234 && v.target_e == 32767 && v.wind_n == 124 && v.wind_e == 0 && v.home_dist == 4322 && v.return_alt == 356,
            "nav: flow speed x100 (saturating), lean x10, target x10, wind x100, distances x10: %d %d %d %d %d %d %d %d %u %u", v.flow_fwd, v.flow_right, v.lean_roll, v.lean_pitch, v.target_n, v.target_e, v.wind_n, v.wind_e, v.home_dist, v.return_alt);
      CHECK(app::pack_stream(STR_NAV, t, 0, b, sizeof(T_nav) - 1) == 0, "nav: a buffer one byte short is refused"); }
    for (int i = 0; i < 16; i++) { t.rc[i] = (uint16_t)(1000 + i); }
    t.rc_link = true; t.rssi = 97; t.rc_rate = 250;
    app::pack_stream(STR_RC, t, 0, b, sizeof(b));
    { T_rc r; memcpy(&r, b, sizeof(r)); CHECK(r.ch[0] == 1000 && r.ch[15] == 1015 && r.link == 1 && r.rssi == 97 && r.rate == 250, "rc"); }
    t.cal_kind = CAL_KIND_ACCEL; t.cal_progress = 42; t.cal_state = CAL_STATE_RUNNING; t.cal_mask = 0x15; t.cal_raw[0] = 0.0124f; t.cal_raw[2] = -1.0046f;
    app::pack_stream(STR_CALIBRATION, t, 0, b, sizeof(b));
    { T_calibration c; memcpy(&c, b, sizeof(c)); CHECK(c.kind == CAL_KIND_ACCEL && c.progress == 42 && c.state == CAL_STATE_RUNNING && c.mask == 0x15 && c.raw[0] == 12 && c.raw[2] == -1005, "calibration"); }
    app::pack_stream(STR_GYRO_FAST, t, 0, b, sizeof(b));
    { T_gyro_fast g; memcpy(&g, b, sizeof(g)); CHECK(g.raw[1] == -457 && g.filt[0] == 32767, "gyro_fast"); }
    CHECK(app::pack_stream(77, t, 0, b, sizeof(b)) == 0, "unknown stream id => 0");
    CHECK(app::pack_stream(STR_MOTORS, t, 0, b, sizeof(T_motors) - 1) == 0, "buffer too small => 0, nothing written past it");
    // every stream comes from the flight loop's telemetry, except "blackbox": the log lives in the link task (bb::Service::pack_status)
    for (const StreamInfo& si : kStreams) CHECK(app::pack_stream(si.id, t, 0, b, sizeof(b)) == (si.id == STR_BLACKBOX ? 0 : si.size), "stream %s packs to its schema size", si.name);
    { app::Telemetry bt; bt.batt_v = 15.87f; bt.cell_v = 3.968f; bt.batt_cells = 4; bt.batt_state = BATT_STATE_WARNING; bt.batt_source = BATT_SOURCE_ADC; bt.batt_pin_mv = 1443; bt.beeping = 9;
      app::pack_stream(STR_BATTERY, bt, 0, b, sizeof(b)); T_battery o; memcpy(&o, b, sizeof(o));
      CHECK(o.voltage == 15870 && o.cell == 3968 && o.cells == 4 && o.state == BATT_STATE_WARNING && o.source == BATT_SOURCE_ADC && o.pin_mv == 1443 && o.beeping == 9, "battery stream: mV, state, pin voltage"); }
    { app::Telemetry tt; tt.gyro_lpf1_hz = 406.25f; tt.dterm_lpf1_hz = 121.9f; for (int i = 0; i < 9; i++) tt.dyn_notch_hz[i] = 100.4f + 10 * i; for (int i = 0; i < 8; i++) tt.rpm_notch_hz[i] = 200.0f + i; tt.tpa = 0.675f; tt.anti_gravity = 12.345f;
      tt.rc_smoothed_hz = 149.6f; tt.rc_cutoff_hz = 56.2f; tt.air_state = 2; tt.hover_throttle = 0.234f; tt.pid_sum[0] = 123.44f; tt.pid_sum[1] = -500.0f; tt.pid_sum[2] = 5000.0f; tt.trim_progress = 60;
      app::pack_stream(STR_TUNING, tt, 0, b, sizeof(b)); T_tuning o; memcpy(&o, b, sizeof(o));
      CHECK(o.gyro_lpf1_hz == 406 && o.dterm_lpf1_hz == 122 && o.dyn_notch_hz[0] == 100 && o.dyn_notch_hz[8] == 180 && o.rpm_hz[0] == 200 && o.rpm_hz[7] == 207, "tuning stream: the filter frequencies in whole Hz");
      CHECK(o.tpa == 68 && o.anti_gravity == 1235 && o.rc_rate == 150 && o.rc_cutoff == 56 && o.air_state == 2 && o.hover == 23 && o.pid_sum[0] == 1234 && o.pid_sum[1] == -5000 && o.pid_sum[2] == 32767, "TPA in %%, anti-gravity x 100, frame rate, cutoff, PID sums x 10 (saturated)");
      tt.tpa = NAN; tt.gyro_lpf1_hz = -5; app::pack_stream(STR_TUNING, tt, 0, b, sizeof(b)); memcpy(&o, b, sizeof(o)); CHECK(o.tpa == 0 && o.gyro_lpf1_hz == 0, "nonsense in, zero out");
      app::pack_stream(STR_CALIBRATION, tt, 0, b, sizeof(b)); T_calibration c; memcpy(&c, b, sizeof(c)); CHECK(c.inflight_trim == 60, "calibration stream: progress of the in-flight level trim"); }
}

static void t_sync() {
    printf("[sync] loop statistics\n");
    app::StageTime st;
    CHECK(st.take_avg() == 0 && st.take_max() == 0, "empty: 0");
    st.add(10); st.add(20); st.add(32); CHECK(st.take_avg() == 21, "average, rounded to nearest (20.67 -> 21)"); CHECK(st.take_avg() == 0, "taking clears");
    st.add(5); st.add(900); st.add(7); CHECK(st.take_max() == 900, "maximum"); st.add(3); CHECK(st.take_max() == 3, "of the new period only");
    st.add(70000); CHECK(st.take_avg() == 65535, "saturates"); st.add(70000); CHECK(st.take_max() == 65535, "saturates");
    CHECK(app::cycles_lost(4000, 1000, 4000.0f) == 0 && app::cycles_lost(3993, 1000, 4000.0f) == 7 && app::cycles_lost(1600, 1000, 4000.0f) == 2400, "lost cycles = configured - done");
    CHECK(app::cycles_lost(4008, 1002, 4000.0f) == 0 && app::cycles_lost(4003, 1002, 4000.0f) == 5 && app::cycles_lost(4100, 1000, 4000.0f) == 0 && app::cycles_lost(4001, 1000, 4000.0f) == 0, "over the real length of the period; a cycle too many is not a negative loss");
    CHECK(app::cycles_lost(0, 60000, 8000.0f) == 65535, "saturates");
    CHECK(app::loop_rate_held(4000, 4000.0f) && app::loop_rate_held(3200, 4000.0f) && !app::loop_rate_held(3199, 4000.0f) && !app::loop_rate_held(1500, 4000.0f) && app::loop_rate_held(1600, 2000.0f), "rate held = at least 80 %% of the configured cycles");
    // ---- the flight task and the timer ticks: a budget (the other thread is owed a quarter of every cycle as idle time)
    { app::TickBudget b;
      CHECK(b.after_cycle(250, 100, 250) == 0 && b.balance() == 125, "a short cycle after a pause: nothing owed, and the credit is half a period at most (%d us)", b.balance());
      CHECK(b.after_cycle(0, 290, 250) == 0 && b.balance() == 52, "a 290 us cycle on that credit: 73 us of it are used (a quarter, rounded up), the next cycle may follow at once (%d left)", b.balance());
      CHECK(b.after_cycle(0, 290, 250) == 21 && b.balance() == -21, "another one, back to back: now 21 us are owed");
      CHECK(b.after_cycle(231, 100, 250) == 0 && b.balance() == 125, "paid by the pause before the next cycle (231 us idle), and credit again");
      app::TickBudget c; CHECK(c.after_cycle(200, 800, 250) == 0 && c.balance() == 0 && c.after_cycle(199, 800, 250) == 1, "800 us cycles need 200 us of pause each: 200 is enough, 199 is 1 us short");
      app::TickBudget d; CHECK(d.after_cycle(0, 0xFFFFFFFFu, 250) == 0x40000000u && d.after_cycle(0, 0xFFFFFFFFu, 250) == 0x40000000u && d.after_cycle(0xFFFFFFFFu, 0, 250) == 0 && d.balance() == 125, "nonsense times do not overflow");
      app::TickBudget e; CHECK(e.after_cycle(1000, 100, 0) == 0 && e.balance() == 0 && e.after_cycle(0, 100, 0) == 25, "no period: no credit, the debt still counts"); }
    { // the task's loop as flight_task.h runs it (the same code: app::TickLoop), against a simulated timer
      struct SimPort {
          uint32_t period; std::function<uint32_t(uint32_t)> cost;
          uint64_t t = 0, next_tick = 0, idle = 0, busy_since = 0, longest_busy = 0; uint32_t pending = 0, cycles = 0, ticks = 0;
          void advance(uint64_t to) { while (next_tick <= to) { pending++; ticks++; next_tick += period; } t = to; }
          void wait_tick() { if (!pending) { if (t - busy_since > longest_busy) longest_busy = t - busy_since; idle += next_tick - t; advance(next_tick); busy_since = t; } pending = 0; }
          uint32_t now() { return (uint32_t)t; }
          void cycle(uint32_t) { advance(t + cost(cycles)); cycles++; }
      };
      struct Result { double idle, rate; uint32_t longest_busy, cycles, ticks; };
      auto run = [](uint32_t period, std::function<uint32_t(uint32_t)> cost, double seconds = 2.0) {
          SimPort p; p.period = period; p.cost = cost; p.advance(0); app::TickLoop<SimPort> loop;
          while (p.t < (uint64_t)(seconds * 1e6)) loop.turn(p, period);
          return Result{(double)p.idle / (double)p.t, (double)p.cycles / (double)p.ticks, (uint32_t)p.longest_busy, p.cycles, p.ticks}; };
      for (uint32_t period : {125u, 250u, 500u}) {
          double worst = 1, least_rate = 9; uint32_t worst_at = 0, rate_at = 0, longest = 0, longest_at = 0; bool full = true;
          for (uint32_t cost = 1; cost <= 12 * period; cost += (cost < 2 * period ? 1 : 7)) {
              const Result r = run(period, [cost](uint32_t) { return cost; });
              if (r.idle < worst) { worst = r.idle; worst_at = cost; }
              if (cost * 5 <= period * 4 && r.rate < 0.999) full = false;
              const double best = fmin(1.0, period / (1.25 * cost));            // a cycle and a quarter of it, on average
              if (r.rate / best < least_rate) { least_rate = r.rate / best; rate_at = cost; }
              if (r.longest_busy > 2 * period + 2 * cost && r.longest_busy > longest) { longest = r.longest_busy; longest_at = cost; }
          }
          CHECK(worst > 0.199 && full, "%u us timer, cycles of 1 us .. 12 periods: the other thread never gets less than a fifth of the core (least: %.1f %% with %u us cycles), and cycles up to four fifths of a period use every tick", period, worst * 100, worst_at);
          CHECK(least_rate > 0.93, "... and hardly a tick is given away that this does not take: at least %.0f %% of the rate a fifth of idle time allows (with %u us cycles)", least_rate * 100, rate_at);
          CHECK(longest == 0, "... and the other thread never waits longer for the core than two periods and two cycles (%u us with %u us cycles)", longest, longest_at);
      }
      // what the first board measured (log of 2026-10-07): 120 - 145 us, one cycle in eight with the estimator 240 - 300 us, one in sixteen writes a log record
      const auto board = [](uint32_t n) { const uint32_t k = n % 16; return k == 3 || k == 11 ? 290u : k == 7 ? 200u : 135u; };
      Result r = run(250, board);
      CHECK(r.cycles + 1 >= r.ticks && r.idle > 0.30 && r.longest_busy < 500, "the first board's cycles (135 us, one in eight 290 us): every tick gets its cycle (%u of %u), %.0f %% of the core left, the other thread waits %u us at most", r.cycles, r.ticks, r.idle * 100, r.longest_busy);
      r = run(250, [](uint32_t n) { return n % 8 == 3 ? 400u : 135u; });
      CHECK(r.cycles + 1 >= r.ticks && r.idle > 0.25, "one cycle in eight of 400 us: still every tick (%u of %u) - the two cycles after it start late and catch up", r.cycles, r.ticks);
      r = run(250, [](uint32_t n) { return n % 8 == 3 ? 600u : 135u; });
      CHECK(fabs(r.rate - 8.0 / 9) < 0.005, "one in eight of 600 us: two ticks pass during it, one cycle is made up, one is lost - %.1f %% of the rate", r.rate * 100);
      r = run(250, [](uint32_t) { return 240u; }); CHECK(fabs(r.rate - 0.8) < 0.005 && r.idle > 0.2, "240 us cycles: four in five ticks (%.3f; with 'give the next tick away after a long cycle' it was one in two), %.0f %% of the core left", r.rate, r.idle * 100);
      r = run(250, [](uint32_t) { return 800u; }); CHECK(fabs(r.rate - 0.25) < 0.005 && fabs(r.idle - 0.2) < 0.005, "the 800 us cycles of firmware 0.5: one cycle per four ticks, a fifth of the core left - not a loop that never lets go");
      r = run(250, [](uint32_t) { return 3000u; }); CHECK(fabs(r.idle - 0.2) < 0.01, "3000 us cycles: 750 us of pause after each (%.1f %% idle), not the 0 .. 250 us that happen to be left before the next tick", r.idle * 100);
      // credit does not pile up: after a long quiet time a burst of long cycles is still stopped after about two periods
      r = run(250, [](uint32_t n) { return n < 4000 ? 60u : 300u; });
      CHECK(r.longest_busy == 900, "a second of light cycles, then 300 us cycles: three of them run back to back (%u us), not as many as the idle time before would pay for", r.longest_busy);
      // one stall (the flash stops both cores for 20 ms): 5 ms of pause after it, then every tick again
      r = run(250, [](uint32_t n) { return n == 2000 ? 20000u : 100u; });
      CHECK(r.ticks - r.cycles >= 98 && r.ticks - r.cycles <= 101, "a single stall of 20 ms costs the 80 ticks it lasted and 20 more of pause (%u lost)", r.ticks - r.cycles); }
    struct Count : app::Sync { int l = 0, u = 0; void lock() override { l++; } void unlock() override { u++; } } c;
    { app::Guard g(c); CHECK(c.l == 1 && c.u == 0, "guard locks"); } CHECK(c.l == 1 && c.u == 1, "and unlocks when it goes");
    app::Sync none; { app::Guard g(none); } CHECK(true, "the default lock does nothing (one thread)");
}

// A serial port as the queue sees it: `cap` bytes of buffer that the wire / the USB host empties by `drain` bytes per millisecond.
struct FakePort : app::RawPort {
    size_t cap, drain, held = 0; std::vector<uint8_t> wire; int overfilled = 0, puts = 0;
    FakePort(size_t c, size_t d) : cap(c), drain(d) {}
    size_t room() override { return cap - held; }
    size_t put(const uint8_t* d, size_t n) override { puts++; if (n > cap - held) { overfilled++; n = cap - held; } wire.insert(wire.end(), d, d + n); held += n; return n; }
    void ms() { held = held > drain ? held - drain : 0; }
};
static int g_waits = 0; static FakePort* g_wait_port = nullptr;
static void wait_hook(void*) { g_waits++; if (g_wait_port) g_wait_port->ms(); }
static void t_tx_queue() {
    printf("[tx queue] one transmit queue in front of UART / USB Serial-JTAG / native USB CDC\n");
    std::mt19937 rng(5);
    auto frames_through = [&](FakePort& port, uint32_t bps, int n_frames, int max_len, uint32_t& lost) {
        app::QueuedTransport q(port, bps, &wait_hook, nullptr); g_wait_port = &port; g_waits = 0;
        std::vector<uint8_t> sent; int dropped = 0;
        for (int i = 0; i < n_frames; i++) {
            const size_t len = 1 + rng() % max_len; std::vector<uint8_t> f(len); for (auto& b : f) b = (uint8_t)rng();
            if (q.writable() >= len) { if (q.write(f.data(), len) == len) sent.insert(sent.end(), f.begin(), f.end()); } else dropped++;      // the protocol's rule for telemetry
            if (i % 3 == 0) { port.ms(); q.pump(); }
        }
        for (int i = 0; i < 100000 && q.queued(); i++) { port.ms(); q.pump(); }
        lost = q.frames_lost(); g_wait_port = nullptr;
        return sent == port.wire && q.queued() == 0 && q.bytes_sent() == sent.size() ? dropped : -1;
    };
    uint32_t lost;
    { FakePort usb(64, 64); const int d = frames_through(usb, 60000, 3000, 500, lost);          // native USB CDC: never more than one 64-byte packet
      CHECK(d >= 0 && usb.overfilled == 0 && lost == 0, "native USB CDC (64 bytes of room): frames of up to 500 bytes arrive whole and in order, the port is never over-filled (%d dropped for lack of room, %zu bytes)", d, usb.wire.size());
      CHECK(d > 0 && d < 3000, "what the port cannot carry is dropped by the protocol's rule, not half-sent"); }
    { FakePort jtag(2048, 2048); const int d = frames_through(jtag, 0, 3000, 500, lost);
      CHECK(d == 0 && jtag.overfilled == 0 && lost == 0, "USB Serial/JTAG (2 kB ring, fast host): nothing dropped"); }
    { FakePort uart(2048, 23); const int d = frames_through(uart, 23040, 600, 300, lost);
      CHECK(d > 0 && uart.overfilled == 0 && lost == 0, "UART 230400: carried in order, the surplus dropped (%d)", d); }
    { FakePort p(64, 64); app::QueuedTransport q(p, 0);
      CHECK(q.writable() == app::QueuedTransport::kSize && q.bytes_per_second() == 0, "empty: the whole queue is free");
      std::vector<uint8_t> big(app::QueuedTransport::kSize + 1, 7);
      CHECK(q.write(big.data(), big.size()) == 0 && q.queued() == 0 && q.write(big.data(), 0) == 0 && q.frames_lost() == 0 && p.puts == 0, "a frame larger than the queue is refused whole (that is not a loss on the link)");
      CHECK(q.write(big.data(), 1000) == 1000 && q.queued() == 1000 - 64 && q.writable() == app::QueuedTransport::kSize - (1000 - 64), "write() already hands the port what it takes"); }
    { // a frame that must not be lost waits for room; a dead port costs kWaitMs once, never a hang
      FakePort slow(64, 64); app::QueuedTransport q(slow, 0, &wait_hook, nullptr); g_wait_port = &slow; g_waits = 0;
      std::vector<uint8_t> a(app::QueuedTransport::kSize, 1), b(500, 2);
      CHECK(q.write(a.data(), a.size()) == a.size() && q.writable() < 500, "queue full");
      CHECK(q.write(b.data(), b.size()) == b.size() && g_waits >= 5 && g_waits <= 10 && q.frames_lost() == 0, "a reply waits until the port has taken enough (%d ms here)", g_waits);
      FakePort dead(64, 0); app::QueuedTransport qd(dead, 0, &wait_hook, nullptr); g_wait_port = &dead; g_waits = 0;
      CHECK(qd.write(a.data(), a.size()) == a.size(), "dead port: the queue fills");
      CHECK(qd.write(b.data(), b.size()) == 0 && g_waits == app::QueuedTransport::kWaitMs && qd.frames_lost() == 1, "then a reply is given up after %d ms (%d waits), counted", app::QueuedTransport::kWaitMs, g_waits);
      app::QueuedTransport qn(dead, 0); g_waits = 0; qn.write(a.data(), a.size());
      CHECK(qn.write(b.data(), b.size()) == 0 && g_waits == 0 && qn.frames_lost() == 1, "without a way to wait: given up at once");
      g_wait_port = nullptr; }
    { FakePort p(100, 100); app::QueuedTransport q(p, 23040); CHECK(q.bytes_per_second() == 23040, "the link's capacity is passed on (for the live log's rate)"); }
}

static void t_gate() {
    printf("[request gate]\n");
    app::Request r; r.cmd = CMD_MOTOR_TEST; r.n = 3;
    CHECK(app::check_request(r, false) == ERR_OK && app::check_request(r, true) == ERR_ARMED, "motor test: disarmed only");
    r.n = 2; CHECK(app::check_request(r, false) == ERR_BAD_ARG, "wrong argument size");
    r.cmd = 99; CHECK(app::check_request(r, false) == ERR_UNKNOWN, "unknown command");
    r.cmd = CMD_MOTOR_STOP; r.n = 0; CHECK(app::check_request(r, true) == ERR_OK, "MOTOR_STOP is allowed while armed (it can only make things safer)");
    r.cmd = CMD_SET_HOME; CHECK(app::check_request(r, true) == ERR_OK, "SET_HOME allowed while armed");
    for (const CommandInfo& c : kCommands) { app::Request q; q.cmd = c.id; q.n = c.arg_bytes; CHECK(app::check_request(q, false) == ERR_OK && (app::check_request(q, true) == ERR_OK) == c.armed_ok, "%s follows its schema flags", c.name); }
    r.cmd = app::REQ_APPLY_SECTION; r.n = 1; r.args[0] = SEC_PID; CHECK(app::check_request(r, false) == ERR_OK && app::check_request(r, true) == ERR_ARMED, "apply section: disarmed only");
    r.args[0] = 99; CHECK(app::check_request(r, false) == ERR_BAD_SECTION, "apply unknown section");
    r.cmd = app::REQ_REBOOT; r.n = 0; CHECK(app::check_request(r, true) == ERR_ARMED && app::check_request(r, false) == ERR_OK, "reboot: disarmed only");
}

static int g_wait_calls = 0;
// A flight loop that executes everything it is given: lets us see what LinkHost itself refuses before asking it.
struct YesFlight { app::Exchange x; int ran = 0; uint8_t last_cmd = 0;
    static void wait(void* p) { YesFlight* f = (YesFlight*)p; f->x.service([f](const app::Request& r) { f->ran++; f->last_cmd = r.cmd; return app::Reply(ERR_OK, 7); }); } };
static void t_host() {
    printf("[link host] what the link task decides by itself\n");
    YesFlight fl; app::LinkHost h(fl.x, &YesFlight::wait, &fl, []() -> uint32_t { return 1234; });
    uint8_t res[16]; uint16_t rlen = 9;
    CHECK(h.now_ms() == 1234, "clock");
    CHECK(h.apply_live(SEC_PID) && fl.ran == 1 && fl.last_cmd == app::REQ_APPLY_SECTION, "a live section is handed to the flight loop");
    CHECK(!h.apply_live(SEC_MOTORS) && !h.apply_live(SEC_PINS) && !h.apply_live(99) && fl.ran == 1, "a section that needs a reboot is NOT applied (and the flight loop is not even asked)");
    const uint8_t a3[3] = {0, 100, 0};
    CHECK(h.command(CMD_MOTOR_TEST, a3, 2, res, rlen) == ERR_BAD_ARG && fl.ran == 1, "wrong argument size never reaches the flight loop");
    CHECK(h.command(123, a3, 0, res, rlen) == ERR_UNKNOWN && fl.ran == 1, "unknown command never reaches the flight loop");
    CHECK(h.command(CMD_MOTOR_TEST, a3, 3, res, rlen) == ERR_OK && fl.ran == 2 && fl.last_cmd == CMD_MOTOR_TEST && rlen == 1 && res[0] == 7, "a valid command is executed and its result byte returned");
    CHECK(!h.take_settings_changed(), "a command that does not write settings leaves them clean");
    CHECK(h.command(CMD_LEVEL_TRIM, a3, 0, res, rlen) == ERR_OK && h.take_settings_changed() && !h.take_settings_changed(), "a command that writes the calibration section marks settings changed, once");
    h.reboot(); CHECK(fl.last_cmd == app::REQ_REBOOT, "reboot goes through the flight loop");
    app::Telemetry t; t.armed = true; t.roll = 12.0f; fl.x.publish(t);
    CHECK(h.armed(), "armed state comes from the published snapshot");
    // a section the flight loop changed by itself (in-flight level trim) is handed to the server for the flash - once, and not while armed
    h.tick(1); CHECK(h.take_autosave() == 0, "nothing to save by default");
    t.fw_saved_section = SEC_CALIBRATION; t.fw_saved_seq = 1; fl.x.publish(t); h.tick(2); CHECK(h.take_autosave() == 0, "the flight loop changed a section while armed: no flash write yet");
    t.armed = false; fl.x.publish(t); h.tick(3); CHECK(h.take_autosave() == SEC_CALIBRATION && h.take_autosave() == 0, "disarmed: the section is handed over, once");
    h.tick(4); h.tick(5); CHECK(h.take_autosave() == 0, "the same change is not saved again");
    t.fw_saved_seq = 2; fl.x.publish(t); h.tick(6); CHECK(h.take_autosave() == SEC_CALIBRATION, "the next change is");
    t.armed = true; fl.x.publish(t);
    uint8_t b[64]; CHECK(h.fill_stream(STR_ATTITUDE, b, sizeof(b)) == sizeof(T_attitude), "streams are packed from the snapshot"); { T_attitude at; memcpy(&at, b, sizeof(at)); CHECK(at.roll == 1200, "snapshot value on the wire"); }
}

static void t_exchange() {
    printf("[exchange] mailbox semantics, then real threads\n");
    { app::Exchange x; int ran = 0;
      CHECK(!x.service([&](const app::Request&) { ran++; return app::Reply(ERR_OK); }) && ran == 0, "nothing pending => nothing runs");
      // the flight loop answers on the 3rd wait
      struct Ctx { app::Exchange* x; int waits = 0; int ran = 0; uint8_t seen = 0; } c{&x};
      auto wait = [](void* p) { Ctx* c = (Ctx*)p; if (++c->waits == 3) c->x->service([c](const app::Request& r) { c->ran++; c->seen = r.args[1]; return app::Reply(ERR_NO_SENSOR, 0x5A); }); };
      app::Request q; q.cmd = CMD_MOTOR_TEST; q.n = 3; q.args[1] = 0x77;
      const app::Reply rep = x.call(q, wait, &c);
      CHECK(rep.err == ERR_NO_SENSOR && rep.n == 1 && rep.data[0] == 0x5A && c.ran == 1 && c.seen == 0x77 && c.waits == 3, "request delivered once, reply (error + data byte) returned");
      CHECK(!x.service([&](const app::Request&) { ran++; return app::Reply(ERR_OK); }), "a finished request does not run again"); }
    { // the flight loop is stuck (never services): the request is withdrawn and must NOT run later
      app::Exchange x; g_wait_calls = 0; app::Request q; q.cmd = CMD_MOTOR_TEST;
      const app::Reply rep = x.call(q, [](void*) { g_wait_calls++; }, nullptr, 50);
      int ran = 0; const bool late = x.service([&](const app::Request&) { ran++; return app::Reply(ERR_OK); });
      CHECK(rep.err == ERR_BUSY && g_wait_calls == 50 && !late && ran == 0 && x.timeouts() == 1, "timeout => ERR_BUSY, and the stale motor test can never fire later");
      CHECK(x.call(q, [](void* p) { ((app::Exchange*)p)->service([](const app::Request&) { return app::Reply(ERR_OK); }); }, &x).err == ERR_OK, "the mailbox is usable again"); }
    { // a second caller while one is in flight (re-entrancy from the same task) gets BUSY
      app::Exchange x; struct C { app::Exchange* x; wbp::Err inner = ERR_OK; } c{&x};
      auto wait = [](void* p) { C* c = (C*)p; app::Request q2; c->inner = c->x->call(q2, [](void*) {}, nullptr, 1).err; c->x->service([](const app::Request&) { return app::Reply(ERR_OK); }); };
      app::Request q; CHECK(x.call(q, wait, &c).err == ERR_OK && c.inner == ERR_BUSY, "one request at a time"); }

    // ---- threads: the flight loop publishes at full speed and services requests; the link thread reads snapshots and calls
    app::Exchange x; std::atomic<bool> stop{false}; std::atomic<uint32_t> executed{0}; std::atomic<uint32_t> published{0};
    std::thread flight([&] {
        app::Telemetry t; uint32_t n = 0;
        while (!stop.load()) {
            n++;
            const float v = (float)(n % 100000);                                 // every field of one snapshot carries the same number
            t.roll = v; t.pitch = v; t.yaw = v; for (int i = 0; i < 3; i++) { t.gyro_f[i] = v; t.acc[i] = v; t.mag[i] = v; } for (int i = 0; i < 8; i++) { t.motor_cmd[i] = v; t.rpm[i] = v; }
            t.z = v; t.vel_e = v; t.cal_raw[2] = v; t.blockers = n; t.gps_baud = n; t.hacc_mm = n;
            x.publish(t); published++;
            x.service([&](const app::Request& r) { executed++; return app::Reply(ERR_OK, (uint8_t)(r.args[0] + 1)); });
        }
    });
    uint32_t torn = 0, snaps = 0, bad_reply = 0, calls = 0, busy = 0; app::Telemetry s;
    const auto t0 = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - t0 < std::chrono::milliseconds(600)) {
        for (int k = 0; k < 2000; k++) {
            x.snapshot(s); snaps++;
            const float v = s.roll;
            bool same = s.pitch == v && s.yaw == v && s.z == v && s.vel_e == v && s.cal_raw[2] == v && s.blockers == s.gps_baud && s.blockers == s.hacc_mm;
            for (int i = 0; i < 3; i++) same = same && s.gyro_f[i] == v && s.acc[i] == v && s.mag[i] == v;
            for (int i = 0; i < 8; i++) same = same && s.motor_cmd[i] == v && s.rpm[i] == v;
            if (!same) torn++;
        }
        app::Request q; q.cmd = CMD_GYRO_CAL; q.args[0] = (uint8_t)calls;
        const app::Reply rep = x.call(q, [](void*) { std::this_thread::yield(); }, nullptr, 2000000);
        calls++; if (rep.err == ERR_BUSY) busy++; else if (rep.err != ERR_OK || rep.n != 1 || rep.data[0] != (uint8_t)((uint8_t)(calls - 1) + 1)) bad_reply++;
    }
    stop = true; flight.join();
    CHECK(torn == 0, "no torn snapshot in %u reads while the flight thread published %u times", snaps, published.load());
    CHECK(published.load() > 1000 && snaps > 1000, "both threads really ran (%u publishes, %u snapshots)", published.load(), snaps);
    CHECK(bad_reply == 0 && busy == 0 && executed.load() == calls && calls > 0, "%u requests: each executed exactly once with its own reply (executed %u)", calls, executed.load());
}

static void t_acc() {
    printf("[accelerometer calibration] 6-face fit, face detection, stillness\n");
    std::mt19937 rng(3); std::normal_distribution<float> nz(0.0f, 0.003f);
    const float off[3] = {0.061f, -0.043f, 0.095f}, sc[3] = {1.021f, 0.984f, 1.037f};
    auto reading = [&](const float g[3], float out[3]) { for (int k = 0; k < 3; k++) out[k] = g[k] / sc[k] + off[k] + nz(rng); };
    auto unit = [](float a, float b, float c, float out[3]) { const float n = sqrtf(a * a + b * b + c * c); out[0] = a / n; out[1] = b / n; out[2] = c / n; };
    AccCalibrator cal; AccCal res;
    CHECK(!cal.fit(res) && !res.valid && cal.mask() == 0, "no faces => no fit");
    // faces deliberately up to ~12 degrees off square, in a scrambled order
    const float dirs[6][3] = {{0.10f, -0.15f, -1}, {1, 0.20f, 0.05f}, {-0.12f, 1, 0.10f}, {0.08f, 0.10f, 1}, {-1, -0.05f, 0.18f}, {0.15f, -1, -0.10f}};
    const int expect_face[6] = {5, 0, 2, 4, 1, 3};
    for (int f = 0; f < 6; f++) {
        float g[3]; unit(dirs[f][0], dirs[f][1], dirs[f][2], g);
        cal.begin_point(800); bool done = false; float r[3];
        for (int i = 0; i < 800; i++) { reading(g, r); done = cal.feed(r); if (i == 399) CHECK(cal.running() && cal.progress() == 50, "progress mid-way: %u", cal.progress()); }
        CHECK(done && cal.state() == CalState::DONE && cal.last_face() == expect_face[f], "face %d recognised as slot %d (got %d)", f, expect_face[f], cal.last_face());
        if (f == 2) CHECK(!cal.complete() && !cal.fit(res), "3 faces are not enough");
    }
    CHECK(cal.complete() && cal.mask() == 0x3F, "six faces");
    CHECK(cal.fit(res) && res.valid, "fit succeeds");
    for (int k = 0; k < 3; k++) CHECK(fabsf(res.offset[k] - off[k]) < 0.0015f && fabsf(res.scale[k] - sc[k]) < 0.0015f, "axis %d: offset %.4f (%.4f), scale %.4f (%.4f) - although no face was square to gravity", k, res.offset[k], off[k], res.scale[k], sc[k]);
    { // the usual (max+min)/2 method on the same tilted faces is visibly worse - this is why the fit is used
      float worst = 0; for (int k = 0; k < 3; k++) { /* dominant components */ float hi = 0, lo = 0; for (int f = 0; f < 6; f++) { float g[3], r[3]; unit(dirs[f][0], dirs[f][1], dirs[f][2], g); for (int j = 0; j < 3; j++) r[j] = g[j] / sc[j] + off[j]; if (expect_face[f] == k * 2) hi = r[k]; if (expect_face[f] == k * 2 + 1) lo = r[k]; } const float s = 2.0f / (hi - lo); if (fabsf(s - sc[k]) > worst) worst = fabsf(s - sc[k]); }
      CHECK(worst > 0.004f, "naive min/max scale error on these faces: %.4f (the fit: < 0.0015)", worst); }
    { float v[3] = {0.3f, -0.2f, -0.93f}; float n0 = sqrtf(0.3f * 0.3f + 0.2f * 0.2f + 0.93f * 0.93f); float g[3] = {v[0] / n0, v[1] / n0, v[2] / n0}, r[3]; for (int k = 0; k < 3; k++) r[k] = g[k] / sc[k] + off[k];
      acc_apply_cal(r, res); CHECK(fabsf(r[0] - g[0]) < 0.003f && fabsf(r[1] - g[1]) < 0.003f && fabsf(r[2] - g[2]) < 0.003f, "applying it restores an arbitrary reading"); }
    { AccCal none; float r[3] = {0.1f, 0.2f, 0.3f}; acc_apply_cal(r, none); CHECK(r[0] == 0.1f && r[2] == 0.3f, "an invalid calibration changes nothing"); }
    { AccCalibrator c; std::normal_distribution<float> big(0.0f, 0.08f); c.begin_point(500); bool done = false; for (int i = 0; i < 500; i++) { float r[3] = {big(rng), big(rng), -1 + big(rng)}; done = c.feed(r); }
      CHECK(!done && c.state() == CalState::FAILED && c.mask() == 0, "hand-held (0.08 g of shake): rejected"); }
    { AccCalibrator c; c.begin_point(100); for (int i = 0; i < 100; i++) { float r[3] = {0.70f, 0.02f, -0.71f}; c.feed(r); } CHECK(c.state() == CalState::FAILED, "resting on an edge: no dominant axis, rejected"); }
    { AccCalibrator c; c.begin_point(100); for (int i = 0; i < 100; i++) { float r[3] = {0, 0, -0.4f}; c.feed(r); } CHECK(c.state() == CalState::FAILED, "0.4 g is not gravity (free fall / wrong range): rejected"); }
    { // a chip that is simply broken (scale 1.5) must not produce a 'valid' calibration
      AccCalibrator c; const float d[6][3] = {{1.5f, 0, 0}, {-1.5f, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
      for (int f = 0; f < 6; f++) { c.max_std_g = 1; c.begin_point(10); for (int i = 0; i < 10; i++) { float r[3] = {d[f][0], d[f][1], d[f][2]}; c.feed(r); } }
      AccCal out; CHECK(!c.complete() || !c.fit(out), "implausible result refused"); }
    { AccCalibrator c; c.begin_point(10); float r[3] = {0, 0, -1}; for (int i = 0; i < 10; i++) c.feed(r); c.reset(); CHECK(c.mask() == 0 && c.state() == CalState::IDLE, "reset clears the session"); }
    { // six clean faces from a chip whose X axis reads 22 % low: every face is accepted, but the RESULT is not a believable chip
      AccCalibrator c; const float d[6][3] = {{0.78f, 0, 0}, {-0.78f, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
      for (int f = 0; f < 6; f++) { c.begin_point(10); for (int i = 0; i < 10; i++) { float r[3] = {d[f][0], d[f][1], d[f][2]}; c.feed(r); } }
      AccCal out; CHECK(c.complete() && !c.fit(out) && !out.valid, "all faces captured but scale 1.28: fit refused"); }
    { // five faces (one of a pair missing) is not a calibration
      AccCalibrator c; const float d[5][3] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, -1}};
      for (int f = 0; f < 5; f++) { c.begin_point(10); for (int i = 0; i < 10; i++) { float r[3] = {d[f][0], d[f][1], d[f][2]}; c.feed(r); } }
      AccCal out; CHECK(!c.complete() && c.mask() == 0x2F && !c.fit(out), "five faces: no fit"); }
    { // a NEW session after a finished one must not be completed by the faces of the old session (the points are still in memory)
      AccCalibrator c; const float d[6][3] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
      for (int f = 0; f < 6; f++) { c.begin_point(10); for (int i = 0; i < 10; i++) { float r[3] = {d[f][0], d[f][1], d[f][2]}; c.feed(r); } }
      AccCal first; CHECK(c.fit(first) && first.valid, "six faces: fit accepted");
      c.reset(); c.begin_point(10); for (int i = 0; i < 10; i++) { float r[3] = {0, 0, -1}; c.feed(r); }
      AccCal out; CHECK(c.mask() == 0x20 && !c.fit(out) && !out.valid, "new session with one face: the five old faces do not count"); }

    printf("[gyro calibration] non-blocking\n");
    { GyroCalibration g; float bias[3] = {9, 9, 9}; std::normal_distribution<float> n(0.0f, 0.05f); bool done = false; g.begin(1000);
      for (int i = 0; i < 1000; i++) { float r[3] = {1.20f + n(rng), -0.70f + n(rng), 0.33f + n(rng)}; done = g.feed(r, bias); }
      CHECK(done && g.state() == CalState::DONE && fabsf(bias[0] - 1.2f) < 0.01f && fabsf(bias[1] + 0.7f) < 0.01f && fabsf(bias[2] - 0.33f) < 0.01f, "bias %.3f %.3f %.3f", bias[0], bias[1], bias[2]); }
    { GyroCalibration g; float bias[3] = {9, 9, 9}; std::normal_distribution<float> n(0.0f, 6.0f); g.begin(500); bool done = false; for (int i = 0; i < 500; i++) { float r[3] = {n(rng), n(rng), n(rng)}; done = done || g.feed(r, bias); }
      CHECK(!done && g.state() == CalState::FAILED && bias[0] == 9, "moving craft: failed, old bias untouched"); }
    { GyroCalibration g; float bias[3] = {9, 9, 9}; g.begin(100); for (int i = 0; i < 100; i++) { float r[3] = {45.0f, 0, 0}; g.feed(r, bias); } CHECK(g.state() == CalState::FAILED && bias[0] == 9, "constant 45 deg/s is rotation, not bias"); }
}

static void t_level() {
    printf("[level trim] the returned alignment really levels the reading\n");
    const float cases[][5] = {{0, 0, 0, 3.0f, -2.0f}, {0, 0, 0, -6.5f, 4.2f}, {1.5f, -0.8f, 0, 2.0f, 2.0f}, {0, 0, 37.0f, 4.0f, -3.0f}, {-2.0f, 3.0f, -120.0f, 5.5f, 1.0f}, {0, 0, 0, 0, 0}};
    for (const auto& c : cases) {
        // sensor-frame gravity for a board that needs (true_r, true_p) of fine alignment to read level
        float mt[3][3]; accgyro::build_rotation(mt, c[3], c[4], c[2]);
        const float g_sensor[3] = {-mt[0][2], -mt[1][2], -mt[2][2]};             // = M * (0,0,-1)
        // what the firmware sees right now, with its current alignment applied
        float cur[3][3]; accgyro::build_rotation(cur, c[0], c[1], c[2]);
        float board[3] = {g_sensor[0], g_sensor[1], g_sensor[2]}; accgyro::apply_rotation(board, cur);
        float r = 99, p = 99;
        CHECK(level_trim(board, c[0], c[1], c[2], r, p), "trim found (cur %.1f/%.1f yaw %.0f)", c[0], c[1], c[2]);
        CHECK(fabsf(r - c[3]) < 0.02f && fabsf(p - c[4]) < 0.02f, "total alignment %.3f / %.3f, expected %.1f / %.1f", r, p, c[3], c[4]);
        float nm[3][3]; accgyro::build_rotation(nm, r, p, c[2]); float v[3] = {g_sensor[0], g_sensor[1], g_sensor[2]}; accgyro::apply_rotation(v, nm);
        CHECK(fabsf(v[0]) < 1e-3f && fabsf(v[1]) < 1e-3f && fabsf(v[2] + 1) < 1e-3f, "with it applied gravity reads (0,0,-1): %.4f %.4f %.4f", v[0], v[1], v[2]);
    }
    { float r = 7, p = 7; const float tilted[3] = {0.5f, 0, -0.866f}; CHECK(!level_trim(tilted, 0, 0, 0, r, p) && r == 7, "30 degrees: refused, outputs untouched"); }
    { float r, p; const float zero[3] = {0, 0, 0}; const float two_g[3] = {0, 0, -2}; CHECK(!level_trim(zero, 0, 0, 0, r, p) && !level_trim(two_g, 0, 0, 0, r, p), "not a gravity reading: refused"); }
    { float r, p; const float scaled[3] = {0.0f, 0.0349f * 0.95f, -0.9994f * 0.95f}; CHECK(level_trim(scaled, 0, 0, 0, r, p) && fabsf(r - 2.0f) < 0.05f, "the length of the vector does not matter (0.95 g): %.2f", r); }
}

static void t_settings() {
    printf("[settings] validation that protects the flight code\n");
    Settings s; defaults(s);
    CHECK(validate(s.modes) == 0 && s.modes.mode[0] == 255 && s.modes.mode[15] == 255, "defaults are valid; unused mode slots stay 255");
    s.modes.mode[2] = 10; CHECK(validate(s.modes) == 1 && s.modes.mode[2] == 255, "an unknown mode id (from a newer firmware, or a damaged record) empties its slot - it does not become some other mode");
    for (const SectionInfo& si : kSections) { Settings d; defaults(d); CHECK(validate_section(d, si.id) == 0, "section %s: defaults pass their own validation", si.name); }
    const float nan = NAN;
    s.pid.stab_p[1] = nan; s.pid.stab_p[2] = nan; s.calibration.acc_scale[2] = nan; s.calibration.mag_matrix[4] = nan;
    Settings d; defaults(d);
    CHECK(validate(s.pid) == 2 && s.pid.stab_p[1] == d.pid.stab_p[1] && s.pid.stab_p[2] == 4.5f, "NaN gains are replaced by the defaults");
    CHECK(validate(s.calibration) == 2 && s.calibration.acc_scale[2] == 1.0f && s.calibration.mag_matrix[4] == 1.0f, "NaN anywhere else too");
    s.pid.stab_p[0] = INFINITY; s.pid.stab_p[2] = -INFINITY; validate(s.pid); CHECK(s.pid.stab_p[0] == 12.0f && s.pid.stab_p[2] == 1.0f, "infinities are clamped into range");
    { AccCal a; a.valid = true; a.offset[0] = 0.02f; a.offset[2] = -0.03f; a.scale[1] = 1.01f; S_calibration c; defaults(c); from_acccal(a, c); AccCal b; to_acccal(c, b);
      CHECK(b.valid && b.offset[0] == 0.02f && b.offset[2] == -0.03f && b.scale[1] == 1.01f && c.mag_valid == 0, "accelerometer calibration <-> settings round trip, compass part untouched"); }
    { Settings q; defaults(q); q.general.board_trim_roll = 15; q.general.board_trim_yaw = -900; q.calibration.acc_trim_roll = -7; q.calibration.acc_trim_pitch = 22; float r, p, y; board_fine_alignment(q, r, p, y);
      CHECK(fabsf(r - 0.8f) < 1e-5f && fabsf(p - 2.2f) < 1e-5f && fabsf(y + 90.0f) < 1e-5f, "fine alignment = board trim + level trim: %.2f %.2f %.2f", r, p, y); }
    for (const SectionInfo& si : kSections) CHECK(si.live == (si.id == SEC_MODES || si.id == SEC_RATES || si.id == SEC_PID || si.id == SEC_CALIBRATION || si.id == SEC_BEEPER || si.id == SEC_BATTERY || si.id == SEC_BLACKBOX || si.id == SEC_FILTERS || si.id == SEC_NAV || si.id == SEC_SERVO), "section %s live flag", si.name);
}

static void t_hwconfig() {
    printf("[hardware config] board defaults + pins from the app\n");
    HardwareConfig::clearAppPins();
    HardwareConfig hw, other; uint8_t p[8];
    hw.getSPIAccGyroSensorPins(p); CHECK(p[0] == GPIO_UNUSED && p[3] == GPIO_UNUSED, "nothing from the app: board default (unused)");
    hw.getGpsPins(p); CHECK(p[0] == GPIO_UNUSED && p[2] == 1, "GPS default: no pins, UART1");
    HardwarePins a; a.imu_cs = 10; a.imu_miso = 13; a.imu_mosi = 11; a.imu_sck = 12; a.i2c_sda = 8; a.i2c_scl = 9; a.gps_rx = 17; a.rx_rx = 16; a.motor[0] = 4; a.motor[3] = 7;
    HardwareConfig::setAppPins(a);
    other.getSPIAccGyroSensorPins(p); CHECK(p[0] == 10 && p[1] == 13 && p[2] == 11 && p[3] == 12, "IMU SPI pins from the app, seen by every HardwareConfig object");
    hw.getI2CAccGyroSensorPins(p); CHECK(p[0] == 8 && p[1] == 9, "IMU I2C = shared bus");
    hw.getI2CBaroSensorPins(p); CHECK(p[0] == 8 && p[1] == 9, "baro on the shared bus");
    hw.getI2CRangeSensorPins(p); CHECK(p[0] == 8 && p[1] == 9, "lidar on the shared bus");
    hw.getI2CMagSensorPins(p); CHECK(p[0] == 8 && p[1] == 9, "compass on the shared bus");
    hw.getGpsPins(p); CHECK(p[0] == 17 && p[1] == GPIO_UNUSED, "a pin the app left empty keeps the board default");
    hw.getSerialRxPins(p); CHECK(p[0] == 16 && p[1] == GPIO_UNUSED && p[2] == 2, "receiver UART pins");
    hw.getMotorPins(p); CHECK(p[0] == 4 && p[1] == GPIO_UNUSED && p[3] == 7 && p[7] == GPIO_UNUSED, "motor slots");
    hw.getSPIBaroSensorPins(p); CHECK(p[0] == GPIO_UNUSED, "baro SPI is not in the app's table");
    CHECK(HardwareConfig::appOr(5, GPIO_UNUSED) == 5 && HardwareConfig::appOr(5, 21) == 21 && HardwareConfig::appOr(GPIO_UNUSED, GPIO_UNUSED) == GPIO_UNUSED, "merge rule: a fixed board pin survives an empty app field, an app pin wins");
    hw.getSdCardPins(p); CHECK(p[0] == GPIO_UNUSED && p[3] == GPIO_UNUSED, "SD card: no pins until the app sets them");
    a.sd_cs = 39; a.sd_clk = 40; a.sd_cmd = 41; a.sd_d0 = 42; a.batt_adc = 1; HardwareConfig::setAppPins(a);
    hw.getSdCardPins(p); CHECK(p[0] == 39 && p[1] == 40 && p[2] == 41 && p[3] == 42, "SD card pins in the order CS, CLK, CMD (MOSI), D0 (MISO)");
    hw.getBatteryAdcPin(p); CHECK(p[0] == 1, "battery ADC pin");
    HardwareConfig::clearAppPins(); hw.getMotorPins(p); CHECK(p[0] == GPIO_UNUSED, "cleared");
}

static void t_pins_load() {
    printf("[pin check] one GPIO, two functions; module pins\n");
    { const pins::Use ok[] = {{10, "imu_cs"}, {255, "buzzer"}, {4, "motor 1"}, {255, "batt_adc"}, {5, "motor 2"}};
      CHECK(!pins::find_conflict(ok, 5).found, "different pins, and unused entries never clash with each other");
      const pins::Use bad[] = {{10, "imu_cs"}, {4, "motor 1"}, {5, "motor 2"}, {255, "x"}, {4, "batt_adc"}, {5, "buzzer"}};
      const pins::Conflict c = pins::find_conflict(bad, 6);
      CHECK(c.found && c.pin == 4 && !strcmp(c.a, "motor 1") && !strcmp(c.b, "batt_adc"), "the first clash is reported with both names: GPIO%u %s / %s", c.pin, c.a, c.b);
      CHECK(!pins::find_conflict(bad, 4).found && !pins::find_conflict(bad, 0).found, "only the entries given are looked at"); }
    { int n = 0; for (int p = 0; p <= 48; p++) n += pins::s3_reserved((uint8_t)p, false);
      int m = 0; for (int p = 0; p <= 48; p++) m += pins::s3_reserved((uint8_t)p, true);
      CHECK(n == 7 && pins::s3_reserved(26, false) && pins::s3_reserved(32, false) && !pins::s3_reserved(25, false) && !pins::s3_reserved(33, false) && !pins::s3_reserved(35, false), "ESP32-S3: GPIO26..32 are the flash (%d pins)", n);
      CHECK(m == 10 && pins::s3_reserved(35, true) && pins::s3_reserved(37, true) && !pins::s3_reserved(34, true) && !pins::s3_reserved(38, true), "with octal PSRAM (N16R8) also GPIO35..37 (%d pins)", m);
      const pins::Use u[] = {{255, "a"}, {36, "sd_clk"}, {27, "gps_tx"}};
      CHECK(pins::find_reserved(u, 3, false) == &u[2] && pins::find_reserved(u, 3, true) == &u[1] && pins::find_reserved(u, 1, true) == nullptr, "the first function on such a pin is found");
      const pins::Use w[] = {{255, "a"}, {18, "gps_tx"}, {20, "buzzer"}, {19, "sd_cs"}};
      CHECK(pins::s3_usb_pin(19) && pins::s3_usb_pin(20) && !pins::s3_usb_pin(18) && !pins::s3_usb_pin(21) && pins::find_on_usb(w, 4) == &w[2] && pins::find_on_usb(w, 2) == nullptr, "GPIO19 / 20 are the chip's USB socket"); }

    printf("[load] CPU percent, memory\n");
    { sys::Load l; l.add(500); CHECK(l.take_percent(1000000) == 0, "the first reading only sets the mark");
      for (int i = 0; i < 4000; i++) l.add(100);          // 4000 cycles of 100 us in one second
      CHECK(l.take_percent(2000000) == 40, "4000 x 100 us in 1 s = 40 %%");
      CHECK(l.take_percent(3000000) == 0, "nothing since: 0 %% (the sum starts again)");
      l.add(999999); CHECK(l.take_percent(4000000) == 100, "99.9999 %% rounds to 100");
      l.add(5000000); CHECK(l.take_percent(5000000) == 100, "more busy time than wall time (a stall inside a cycle) is capped at 100");
      l.add(1234); CHECK(l.take_percent(5000000) == 0, "no time passed: 0, no division by zero");
      sys::Load w; w.take_percent(0xFFFFFF00u); w.add(128); CHECK(w.take_percent(0x00000100u) == 25, "across the micros() wrap: 128 us busy of 512 us"); }
    { sys::Memory m; CHECK(m.used_percent() == 0, "unknown heap: 0 %%");
      m.heap_total = 320 * 1024; m.heap_free = 240 * 1024; CHECK(m.used_percent() == 25, "240 of 320 kB free: 25 %% used");
      m.heap_free = 1; CHECK(m.used_percent() == 100, "full"); m.heap_free = 400 * 1024; CHECK(m.used_percent() == 0, "free > total (a reporting glitch) does not wrap");
      CHECK(sys::kb16(8u * 1024 * 1024) == 8192 && sys::kb16(1023) == 0 && sys::kb16(0xFFFFFFFFu) == 65535, "bytes -> kB in 16 bits, saturated"); }
}

// ---- which UART serves the GPS, the serial receiver, an MSP flow sensor
static void t_uart_plan() {
    printf("[uart plan] AUTO and the function table\n");
    using namespace app;
    const uint8_t none[3] = {PF_NONE, PF_NONE, PF_NONE};
    { UartPlan p = plan_uarts(false, none, true, true, true, false, true, true);
      CHECK(p.gps == 1 && p.rx == 2 && p.flow == -1 && !*p.why, "AUTO: GPS UART1, receiver UART2, nothing left for the flow sensor (UART0 = app)"); }
    { UartPlan p = plan_uarts(false, none, true, true, true, true, true, true); CHECK(p.flow == 0, "AUTO, app on native USB: the flow sensor gets UART0"); }
    { UartPlan p = plan_uarts(false, none, false, true, true, true, false, true); CHECK(p.gps == -1 && p.flow == 1, "AUTO, GPS off: UART1 for the flow sensor"); }
    { UartPlan p = plan_uarts(false, none, true, false, true, false, true, false); CHECK(p.rx == -1 && p.flow == 2, "AUTO, PPM receiver: UART2 for the flow sensor"); }
    const uint8_t t1[3] = {PF_FLOW_MSP, PF_SERIAL_RX, PF_GPS};
    { UartPlan p = plan_uarts(true, t1, true, true, true, true, false, false);
      CHECK(p.gps == 2 && p.rx == 1 && p.flow == 0 && !*p.why, "TABLE: GPS on UART2, receiver on UART1, flow sensor on UART0 (app on USB)"); }
    { UartPlan p = plan_uarts(true, t1, true, true, true, false, false, false);
      CHECK(p.flow == -1 && p.gps == 2 && *p.why, "TABLE: UART0 asked for while the app uses it: not taken, said why"); }
    { UartPlan p = plan_uarts(true, t1, false, true, false, true, false, false);
      CHECK(p.gps == -1 && p.flow == -1 && p.rx == 1, "TABLE: devices that are not wanted get no UART"); }
    const uint8_t t2[3] = {PF_NONE, PF_GPS, PF_GPS};
    { UartPlan p = plan_uarts(true, t2, true, true, false, true, false, false); CHECK(p.gps == 1 && p.rx == -1, "TABLE: GPS twice: the first UART; no receiver in the table: none"); }
    const uint8_t t3[3] = {PF_TELEMETRY, PF_TELEMETRY, PF_SERIAL_RX};
    { UartPlan p = plan_uarts(true, t3, true, true, false, true, false, false); CHECK(p.gps == -1 && p.rx == 2 && !*p.why, "TABLE: TELEMETRY is not a device (unused)"); }
}

int main() {
    t_pack();
    t_sync();
    t_tx_queue(); t_gate(); t_host(); t_exchange(); t_acc(); t_level(); t_settings(); t_hwconfig(); t_pins_load(); t_uart_plan();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
