// MAVLink telemetry (src/telemetry/mavlink.h, mavlink_telemetry.h): frames byte for byte against pymavlink (test/mavlink_vectors.h),
// the frame parser, the scheduler (rates, room on the UART, STATUSTEXT chunks), the HEARTBEAT modes, the ring to the WiFi side.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <string>
#include <vector>
#include "../src/telemetry/mavlink.h"
#include "../src/telemetry/mavlink_telemetry.h"
#include "mavlink_vectors.h"
using namespace mav;

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, ...) do { const bool ok_ = (cond); if (getenv("V")) { printf("    "); printf(__VA_ARGS__); printf("\n"); } if (ok_) g_pass++; else { g_fail++; printf("  FAIL line %d: %s   ", __LINE__, #cond); printf(__VA_ARGS__); printf("\n"); } } while (0)

static const MavVector& vec(const char* name) { for (const MavVector& v : kMavVectors) if (!strcmp(v.name, name)) return v; printf("no vector %s\n", name); exit(2); }
static bool same(const char* name, const uint8_t* f, size_t n) {
    const MavVector& v = vec(name);
    if ((int)n != v.n) { printf("    %s: %d bytes, pymavlink %d\n", name, (int)n, v.n); return false; }
    for (size_t i = 0; i < n; i++) if (f[i] != v.bytes[i]) { printf("    %s: byte %d is 0x%02X, pymavlink 0x%02X\n", name, (int)i, f[i], v.bytes[i]); return false; }
    return true;
}
static Encoder enc_at(const char* name, uint8_t sysid = 1) { Encoder e; e.sysid = sysid; e.compid = 1; e.set_seq(vec(name).seq); return e; }

// the same values as tools/mavlink_vectors.py
static void t_vectors() {
    printf("[frames] every message byte for byte as pymavlink makes it\n");
    uint8_t f[kMaxFrame]; size_t n;
    { Heartbeat h; h.type = 2; h.base_mode = 1 | 64 | 128 | 16; h.custom_mode = 2; h.system_status = 4; Encoder e = enc_at("heartbeat"); n = pack(e, h, f);
      CHECK(same("heartbeat", f, n) && e.seq() == 1, "HEARTBEAT, armed in Altitude Hold"); }
    { Heartbeat h; h.type = 13; h.base_mode = 1 | 64; h.custom_mode = 1; h.system_status = 3; Encoder e = enc_at("heartbeat_disarmed_acro", 7); n = pack(e, h, f);
      CHECK(same("heartbeat_disarmed_acro", f, n) && e.seq() == 0, "HEARTBEAT of a hexa, sysid 7, sequence 255 wraps to 0"); }
    { SysStatus s; s.present = s.enabled = 0x0221802B; s.health = 0x02218023; s.load = 473; s.voltage_battery = 15872; s.current_battery = 1234; s.battery_remaining = 61; s.errors_comm = 3;
      Encoder e = enc_at("sys_status"); n = pack(e, s, f); CHECK(same("sys_status", f, n), "SYS_STATUS (the extension fields zero: cut off)"); }
    { GpsRawInt g; g.time_usec = 123456789000ull; g.fix_type = 3; g.lat = 473977418; g.lon = 85455939; g.alt = 488123; g.eph = 87; g.epv = 65535; g.vel = 1234; g.cog = 27050; g.satellites_visible = 17; g.h_acc = 1250;
      Encoder e = enc_at("gps_raw_int"); n = pack(e, g, f); CHECK(same("gps_raw_int", f, n), "GPS_RAW_INT with h_acc (an extension field)"); }
    { Attitude a; a.time_boot_ms = 654321; a.roll = 0.125f; a.pitch = -0.25f; a.yaw = 2.5f; a.rollspeed = 1.5f; a.pitchspeed = -0.75f; a.yawspeed = 0.0625f;
      Encoder e = enc_at("attitude"); n = pack(e, a, f); CHECK(same("attitude", f, n), "ATTITUDE"); }
    { GlobalPositionInt g; g.time_boot_ms = 654321; g.lat = -337000000; g.lon = -1223950000; g.alt = 12345; g.relative_alt = -2500; g.vx = -150; g.vy = 275; g.vz = -40; g.hdg = 35999;
      Encoder e = enc_at("global_position_int"); n = pack(e, g, f); CHECK(same("global_position_int", f, n), "GLOBAL_POSITION_INT, south-west, negative values"); }
    { RcChannels c; c.time_boot_ms = 1000; c.chancount = 16; c.rssi = 203; const uint16_t v[18] = {1500, 1000, 2000, 1234, 1800, 1100, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1999, 65535, 65535};
      for (int i = 0; i < 18; i++) c.chan[i] = v[i];
      Encoder e = enc_at("rc_channels"); n = pack(e, c, f); CHECK(same("rc_channels", f, n), "RC_CHANNELS"); }
    { RcChannels c; c.rssi = 0; for (uint16_t& x : c.chan) x = 0; Encoder e = enc_at("rc_channels_no_link"); n = pack(e, c, f);
      CHECK(same("rc_channels_no_link", f, n) && f[1] == 1, "a payload of zeros: one byte stays (MAVLink 2 truncation)"); }
    { VfrHud v; v.airspeed = 3.5f; v.groundspeed = 3.5f; v.heading = 271; v.throttle = 42; v.alt = 12.25f; v.climb = -0.5f;
      Encoder e = enc_at("vfr_hud"); n = pack(e, v, f); CHECK(same("vfr_hud", f, n), "VFR_HUD"); }
    { BatteryStatus b; for (int i = 0; i < 4; i++) b.voltages[i] = 3968; b.current_battery = 1234; b.current_consumed = 567; b.battery_remaining = 61;
      Encoder e = enc_at("battery_status"); n = pack(e, b, f); CHECK(same("battery_status", f, n), "BATTERY_STATUS, four cells"); }
    { StatusText t; t.severity = 6; const char* s = "imu: SPI ICM42688P, gyro 4000 Hz"; memcpy(t.text, s, strlen(s));
      Encoder e = enc_at("statustext"); n = pack(e, t, f); CHECK(same("statustext", f, n), "STATUSTEXT"); }
    { const char* s = "motors: FAILED - DShot / ProShot drive at most 4 motors on the ESP32-S3 (4 RMT transmitters)";
      StatusText a; a.severity = 4; a.id = 1; a.chunk_seq = 0; memcpy(a.text, s, 50); Encoder e = enc_at("statustext_chunk0"); n = pack(e, a, f); const bool c0 = same("statustext_chunk0", f, n);
      StatusText b; b.severity = 4; b.id = 1; b.chunk_seq = 1; memcpy(b.text, s + 50, strlen(s) - 50); n = pack(e, b, f);
      CHECK(c0 && same("statustext_chunk1", f, n), "STATUSTEXT in two chunks (id 1, chunk_seq 0 and 1)"); }
}

static void t_parser() {
    printf("[parser] frames out of a byte stream, CRC checked\n");
    std::vector<uint8_t> s = {0x00, 0x55, 0xFE};                                   // garbage first (a MAVLink 1 start byte among it)
    for (const MavVector& v : kMavVectors) s.insert(s.end(), v.bytes, v.bytes + v.n);
    Parser p; int got = 0; bool ids = true; const MavVector* exp = kMavVectors;
    for (uint8_t c : s) if (p.feed(c)) { if (p.frame().seq != exp->seq || p.frame().sysid != exp->bytes[5]) ids = false; got++; exp++; }
    const int n = (int)(sizeof(kMavVectors) / sizeof(kMavVectors[0]));
    CHECK(got == n && ids && p.bad() == 0, "all %d vectors read back (%d), sequence and system id right", n, got);
    { Parser q; const MavVector& v = vec("attitude"); for (int i = 0; i < v.n; i++) q.feed(v.bytes[i]);
      const Parser::Frame& f = q.frame();
      CHECK(f.msgid == MSG_ATTITUDE && Parser::get<uint32_t>(f, 0) == 654321u && Parser::get<float>(f, 4) == 0.125f && Parser::get<float>(f, 24) == 0.0625f, "fields of ATTITUDE read back"); }
    { Parser q; const MavVector& v = vec("rc_channels_no_link"); bool ok = false; for (int i = 0; i < v.n; i++) ok = q.feed(v.bytes[i]);
      CHECK(ok && Parser::get<uint16_t>(q.frame(), 2) == 0 && q.frame().payload[41] == 0, "a truncated payload is zero-filled"); }
    { Parser q; const MavVector& v = vec("vfr_hud"); std::vector<uint8_t> b(v.bytes, v.bytes + v.n); b[14] ^= 0x01; bool any = false; for (uint8_t c : b) any |= q.feed(c);
      CHECK(!any && q.bad() == 1, "one bit wrong: dropped (CRC)"); }
}

static State base_state() {
    State s; s.time_ms = 1000; s.have_gps = true; s.gps_fix = 3; s.lat = 473977418; s.lon = 85455939; s.alt_msl_mm = 488123; s.sats = 12; s.hdop_x100 = 90; s.gs_cms = 250; s.cog_cdeg = 9000;
    s.alt_valid = true; s.rel_alt_m = 3.5f; s.climb_ms = 0.5f; s.vel_valid = true; s.vn = 1.0f; s.ve = -2.0f; s.heading_deg = 90.0f; s.throttle = 0.42f;
    s.batt_present = true; s.volts = 15.87f; s.cells = 4; s.have_current = true; s.amps = 12.34f; s.mah = 567; s.remaining = 61;
    s.rc_link = true; for (int i = 0; i < 16; i++) s.rc[i] = (uint16_t)(1000 + 50 * i); s.rssi_pct = 80;
    s.roll = 0.1f; s.pitch = -0.2f; s.yaw = 1.5708f;
    return s;
}
struct Seen { int count[256] = {}; std::vector<Parser::Frame> frames; };
static Seen run(Scheduler& sch, const State& s0, uint32_t ms, size_t cap = 512, uint32_t start = 0) {
    Seen seen; Parser p; uint8_t buf[2048]; State s = s0;
    for (uint32_t t = start; t < start + ms; t++) {
        s.time_ms = t;
        const size_t n = sch.poll(t, s, buf, cap);
        for (size_t i = 0; i < n; i++) if (p.feed(buf[i])) { seen.count[p.frame().msgid & 0xFF]++; seen.frames.push_back(p.frame()); }
    }
    return seen;
}

static void t_scheduler() {
    printf("[scheduler] rates, room on the UART, sequence numbers\n");
    {   Scheduler sch; Rates r; sch.configure(r, 1); const Seen s = run(sch, base_state(), 10000);
        CHECK(s.count[MSG_HEARTBEAT] == 10 && s.count[MSG_ATTITUDE] == 100 && s.count[MSG_GLOBAL_POSITION_INT] == 50 && s.count[MSG_GPS_RAW_INT] == 50 && s.count[MSG_VFR_HUD] == 40
              && s.count[MSG_SYS_STATUS] == 20 && s.count[MSG_BATTERY_STATUS] == 20 && s.count[MSG_RC_CHANNELS] == 20,
              "10 s at the default rates: HEARTBEAT %d, ATTITUDE %d, POSITION %d + GPS_RAW %d, HUD %d, SYS_STATUS %d + BATTERY %d, RC %d",
              s.count[MSG_HEARTBEAT], s.count[MSG_ATTITUDE], s.count[MSG_GLOBAL_POSITION_INT], s.count[MSG_GPS_RAW_INT], s.count[MSG_VFR_HUD], s.count[MSG_SYS_STATUS], s.count[MSG_BATTERY_STATUS], s.count[MSG_RC_CHANNELS]);
        bool seq = true; for (size_t i = 1; i < s.frames.size(); i++) if (s.frames[i].seq != (uint8_t)(s.frames[i - 1].seq + 1)) seq = false;
        CHECK(seq && s.frames.size() > 300, "one sequence number per frame, no gaps over %d frames (wraps at 255)", (int)s.frames.size()); }
    {   Scheduler sch; Rates r; r.attitude = 50; r.position = 0; r.hud = 0; r.status = 0; r.rc = 0; sch.configure(r, 42); const Seen s = run(sch, base_state(), 2000);
        CHECK(s.count[MSG_ATTITUDE] == 100 && s.count[MSG_GLOBAL_POSITION_INT] == 0 && s.count[MSG_GPS_RAW_INT] == 0 && s.count[MSG_VFR_HUD] == 0 && s.count[MSG_SYS_STATUS] == 0 && s.count[MSG_RC_CHANNELS] == 0 && s.count[MSG_HEARTBEAT] == 2,
              "ATTITUDE at 50 Hz, the others at 0: off (HEARTBEAT always)");
        CHECK(!s.frames.empty() && s.frames[0].sysid == 42 && s.frames[0].compid == 1, "system id from the settings, component AUTOPILOT1"); }
    {   State st = base_state(); st.have_gps = false; Scheduler sch; sch.configure(Rates(), 1); const Seen s = run(sch, st, 2000);
        CHECK(s.count[MSG_GPS_RAW_INT] == 0 && s.count[MSG_GLOBAL_POSITION_INT] == 10, "no GPS fitted: no GPS_RAW_INT, the position (height, velocity) still"); }
    {   State st = base_state(); st.batt_present = false; Scheduler sch; sch.configure(Rates(), 1); const Seen s = run(sch, st, 2000);
        bool unknown = true; for (const Parser::Frame& f : s.frames) if (f.msgid == MSG_SYS_STATUS && (Parser::get<uint16_t>(f, 14) != 0xFFFF || (int8_t)f.payload[30] != -1)) unknown = false;
        CHECK(s.count[MSG_BATTERY_STATUS] == 0 && s.count[MSG_SYS_STATUS] == 4 && unknown, "no battery: no BATTERY_STATUS, SYS_STATUS says voltage and remaining unknown"); }
    {   // a slow UART: 64 bytes of room per millisecond (every frame fits, two rarely do): nothing cut, nothing lost
        Scheduler fast; fast.configure(Rates(), 1); const Seen all = run(fast, base_state(), 5000);
        Scheduler sch; sch.configure(Rates(), 1); const Seen slow = run(sch, base_state(), 5000, 64);
        bool same_counts = true; for (int i = 0; i < 256; i++) if (all.count[i] != slow.count[i]) same_counts = false;
        CHECK(same_counts && slow.frames.size() == all.frames.size(), "64 bytes of room per call: the same frames as with room enough (%d / %d; ATTITUDE %d / %d, GPS_RAW %d / %d, BATTERY %d / %d), none cut", (int)slow.frames.size(), (int)all.frames.size(), slow.count[MSG_ATTITUDE], all.count[MSG_ATTITUDE], slow.count[MSG_GPS_RAW_INT], all.count[MSG_GPS_RAW_INT], slow.count[MSG_BATTERY_STATUS], all.count[MSG_BATTERY_STATUS]); }
    {   // a frame bigger than the room never goes out cut: it waits, the smaller ones go
        Scheduler sch; sch.configure(Rates(), 1); Parser p; uint8_t buf[512]; State s = base_state(); bool within = true;
        for (uint32_t t = 0; t < 5000; t++) { s.time_ms = t; const size_t n = sch.poll(t, s, buf, 45); if (n > 45) within = false; for (size_t i = 0; i < n; i++) p.feed(buf[i]); }
        CHECK(within && p.bad() == 0 && p.good() > 100, "45 bytes of room: %u whole frames (the bigger ones wait), never a cut one", p.good()); }
    {   // no room at all for 3 s: nothing piles up (afterwards one of each, then the normal rate - no burst)
        Scheduler sch; sch.configure(Rates(), 1); uint8_t buf[2048]; State s = base_state();
        for (uint32_t t = 0; t < 3000; t++) sch.poll(t, s, buf, 0);
        const Seen after = run(sch, s, 200, 2048, 3000);
        CHECK(after.count[MSG_ATTITUDE] == 2 && after.count[MSG_GLOBAL_POSITION_INT] == 1 && after.count[MSG_HEARTBEAT] == 1, "after 3 s without room, the next 200 ms: ATTITUDE %d, POSITION %d, HEARTBEAT %d (no backlog)",
              after.count[MSG_ATTITUDE], after.count[MSG_GLOBAL_POSITION_INT], after.count[MSG_HEARTBEAT]); }
    {   // the values: units and conventions
        Scheduler sch; sch.configure(Rates(), 1); const Seen s = run(sch, base_state(), 1000, 512, 5000);
        const Parser::Frame* gp = nullptr; const Parser::Frame* hud = nullptr; const Parser::Frame* bat = nullptr; const Parser::Frame* rc = nullptr; const Parser::Frame* raw = nullptr;
        for (const Parser::Frame& f : s.frames) { if (f.msgid == MSG_GLOBAL_POSITION_INT) gp = &f; if (f.msgid == MSG_VFR_HUD) hud = &f; if (f.msgid == MSG_BATTERY_STATUS) bat = &f; if (f.msgid == MSG_RC_CHANNELS) rc = &f; if (f.msgid == MSG_GPS_RAW_INT) raw = &f; }
        CHECK(gp && Parser::get<int32_t>(*gp, 4) == 473977418 && Parser::get<int32_t>(*gp, 12) == 488123 && Parser::get<int32_t>(*gp, 16) == 3500 && Parser::get<int16_t>(*gp, 20) == 100
              && Parser::get<int16_t>(*gp, 22) == -200 && Parser::get<int16_t>(*gp, 24) == -50 && Parser::get<uint16_t>(*gp, 26) == 9000,
              "GLOBAL_POSITION_INT: lat, MSL mm, relative 3500 mm, vx 100 vy -200 cm/s, climbing = vz negative, heading 90.00 deg");
        CHECK(hud && Parser::get<float>(*hud, 4) == 2.5f && Parser::get<int16_t>(*hud, 16) == 90 && Parser::get<uint16_t>(*hud, 18) == 42 && Parser::get<float>(*hud, 12) == 0.5f, "VFR_HUD: 2.5 m/s, 90 deg, 42 %% throttle, climb 0.5");
        CHECK(bat && Parser::get<uint16_t>(*bat, 10) == 3968 && Parser::get<uint16_t>(*bat, 16) == 3968 && Parser::get<uint16_t>(*bat, 18) == 0xFFFF && Parser::get<int16_t>(*bat, 30) == 1234
              && Parser::get<int32_t>(*bat, 0) == 567 && (int8_t)bat->payload[35] == 61, "BATTERY_STATUS: 4 cells of 3.968 V, the 5th unused, 12.34 A, 567 mAh, 61 %%");
        CHECK(rc && Parser::get<uint16_t>(*rc, 4) == 1000 && Parser::get<uint16_t>(*rc, 34) == 1750 && Parser::get<uint16_t>(*rc, 36) == 0xFFFF && rc->payload[40] == 16 && rc->payload[41] == 203,
              "RC_CHANNELS: 16 channels, 17 and 18 unused, rssi 80 %% = 203 of 254");
        CHECK(raw && raw->payload[28] == GPS_FIX_3D && raw->payload[29] == 12 && Parser::get<uint16_t>(*raw, 20) == 90 && Parser::get<uint16_t>(*raw, 26) == 9000, "GPS_RAW_INT: 3D fix, 12 satellites, HDOP 0.90, course 90 deg"); }
}

static void t_text() {
    printf("[statustext] queue and chunks\n");
    auto texts = [](const Seen& s) {
        std::vector<std::pair<uint16_t, std::string>> out; std::string cur;
        for (const Parser::Frame& f : s.frames) {
            if (f.msgid != MSG_STATUSTEXT) continue;
            char t[51] = {}; memcpy(t, f.payload + 1, 50);
            const uint16_t id = Parser::get<uint16_t>(f, 51); const uint8_t chunk = f.payload[53];
            if (chunk == 0) cur.clear();
            cur += t;
            if (id == 0 || strlen(t) < 50) out.push_back({id, cur});
        }
        return out;
    };
    Rates off; off.attitude = off.position = off.hud = off.status = off.rc = 0;
    {   Scheduler sch; sch.configure(off, 1);
        const std::string s120(120, 'x'), s100 = std::string(99, 'y') + "!";
        sch.text(MAV_SEVERITY_INFO, "short one");
        sch.text(MAV_SEVERITY_WARNING, (std::string("pins: FAILED - ") + s120).c_str());
        sch.text(MAV_SEVERITY_INFO, s100.c_str());
        const Seen s = run(sch, State(), 2000); const auto t = texts(s);
        int n100 = 0; for (const Parser::Frame& f : s.frames) if (f.msgid == MSG_STATUSTEXT && f.payload[1] == 'y') n100++;
        CHECK(t.size() == 3 && t[0].first == 0 && t[0].second == "short one", "a short text: one frame, id 0");
        CHECK(t.size() == 3 && t[1].first != 0 && t[1].second == (std::string("pins: FAILED - ") + s120).substr(0, Scheduler::kTextLen) && s.count[MSG_STATUSTEXT] >= 6, "135 characters: three chunks of one id, put together again");
        CHECK(t.size() == 3 && t[2].second == s100 && n100 == 2, "exactly 100 characters: two full chunks and an empty one that marks the end");
        int sev4 = 0; for (const Parser::Frame& f : s.frames) if (f.msgid == MSG_STATUSTEXT && f.payload[0] == MAV_SEVERITY_WARNING) sev4++;
        CHECK(sev4 == 3, "severity carried in every chunk"); }
    {   Scheduler sch; sch.configure(off, 1); uint8_t buf[512]; State st;
        for (int i = 0; i < 10; i++) { char b[16]; snprintf(b, sizeof(b), "line %d", i); sch.text(MAV_SEVERITY_INFO, b); }
        CHECK(sch.queued_texts() == Scheduler::kTexts && sch.dropped_texts() == 4, "10 texts into a queue of 6: the 4 oldest dropped");
        uint32_t first = 0, second = 0; for (uint32_t t = 0; t < 300; t++) { const size_t n = sch.poll(t, st, buf, sizeof(buf)); Parser p; for (size_t i = 0; i < n; i++) if (p.feed(buf[i]) && p.frame().msgid == MSG_STATUSTEXT) { if (!first) first = t + 1; else if (!second) second = t + 1; } }
        CHECK(second - first == Scheduler::kTextGapMs, "one text every %u ms (%u)", (unsigned)Scheduler::kTextGapMs, second - first); }
}

static void t_heartbeat() {
    printf("[heartbeat] mode and state\n");
    State s; Heartbeat h = heartbeat_of(s);
    CHECK(h.custom_mode == COPTER_ACRO && h.system_status == MAV_STATE_STANDBY && !(h.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) && (h.base_mode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED), "disarmed Acro: ACRO, STANDBY");
    s.armed = true; s.angle = true; h = heartbeat_of(s);
    CHECK(h.custom_mode == COPTER_STABILIZE && h.system_status == MAV_STATE_ACTIVE && (h.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) && (h.base_mode & MAV_MODE_FLAG_STABILIZE_ENABLED), "armed Angle: STABILIZE, ACTIVE, armed flag");
    s.angle = false; s.stabilize = true; CHECK(heartbeat_of(s).custom_mode == COPTER_STABILIZE, "Stabilize: STABILIZE");
    s.althold = true; CHECK(heartbeat_of(s).custom_mode == COPTER_ALT_HOLD, "Altitude Hold: ALT_HOLD");
    s.poshold = true; CHECK(heartbeat_of(s).custom_mode == COPTER_POSHOLD, "Position Hold: POSHOLD");
    s.rescue = true; CHECK(heartbeat_of(s).custom_mode == COPTER_RTL, "GPS Rescue: RTL");
    s.rescue = false; s.failsafe = true; h = heartbeat_of(s); CHECK(h.custom_mode == COPTER_LAND && h.system_status == MAV_STATE_CRITICAL, "failsafe landing: LAND, CRITICAL");
    s = State(); s.calibrating = true; CHECK(heartbeat_of(s).system_status == MAV_STATE_CALIBRATING, "calibrating: CALIBRATING");
    State g; CHECK(gps_fix_of(g) == GPS_FIX_NO_GPS, "no GPS: NO_GPS"); g.have_gps = true; g.gps_fix = 0; CHECK(gps_fix_of(g) == GPS_FIX_NONE, "GPS without a fix: NO_FIX");
    g.gps_fix = 2; CHECK(gps_fix_of(g) == GPS_FIX_2D, "2D"); g.gps_fix = 4; CHECK(gps_fix_of(g) == GPS_FIX_3D, "GNSS + dead reckoning counts as 3D");
    CHECK(cdeg360(-90.0f) == 27000 && cdeg360(359.999f) == 0 && cdeg360(720.5f) == 50, "heading in cdeg, 0 .. 35999");
}

static void t_ring() {
    printf("[ring] frames to the WiFi side\n");
    Ring r; uint8_t f[100], out[600]; for (int i = 0; i < 100; i++) f[i] = (uint8_t)i;
    int pushed = 0; while (r.push(f, 100)) pushed++;
    CHECK(pushed == (int)(Ring::kSize / 102) && r.dropped() == 1, "full: whole frames only (%d of 100 bytes), the next one dropped and counted", pushed);
    const size_t n = r.pop(out, 350);
    CHECK(n == 300 && out[0] == 0 && out[99] == 99 && out[100] == 0, "a datagram of 350 bytes takes three whole frames, never half of one");
    size_t rest = 0, k; while ((k = r.pop(out, sizeof(out))) > 0) rest += k;
    CHECK(rest == (size_t)(pushed - 3) * 100 && r.pop(out, sizeof(out)) == 0, "the rest comes out, then nothing");
    for (int i = 0; i < 100; i++) { r.push(f, 37); r.pop(out, sizeof(out)); }
    CHECK(r.push(f, 100) && r.pop(out, sizeof(out)) == 100 && out[42] == 42, "across the end of the buffer many times: still whole");
}

int main() {
    t_vectors(); t_parser(); t_scheduler(); t_text(); t_heartbeat(); t_ring();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
