// WBP: framing, schema, persistent settings, server behaviour, and settings -> config mapping.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include <map>
#include <string>
#include <vector>
#include "../src/proto/wbp_server.h"
#include "../src/proto/settings_apply.h"
#include "../src/port/chip.h"

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, ...) do { const bool ok_ = (cond); if (getenv("V")) { printf("    "); printf(__VA_ARGS__); printf("\n"); } if (ok_) g_pass++; else { g_fail++; printf("  FAIL line %d: %s   ", __LINE__, #cond); printf(__VA_ARGS__); printf("\n"); } } while (0)
using namespace wbp;

struct MemStorage : Storage {
    std::map<std::string, std::vector<uint8_t>> kv; bool fail_write = false;
    int read(const char* k, uint8_t* b, size_t cap) override { auto it = kv.find(k); if (it == kv.end()) return -1; const size_t n = it->second.size() < cap ? it->second.size() : cap; memcpy(b, it->second.data(), n); return (int)n; }
    bool write(const char* k, const uint8_t* d, size_t n) override { if (fail_write) return false; kv[k] = std::vector<uint8_t>(d, d + n); return true; }
};
struct MemTransport : Transport {
    std::vector<uint8_t> out; size_t room = 100000;
    size_t write(const uint8_t* d, size_t n) override { out.insert(out.end(), d, d + n); return n; }
    size_t writable() override { return room; }
    std::vector<std::pair<Frame, std::vector<uint8_t>>> frames() { std::vector<std::pair<Frame, std::vector<uint8_t>>> r; Parser p; Frame f; for (uint8_t c : out) if (p.feed(c, f)) r.push_back({f, std::vector<uint8_t>(f.payload, f.payload + f.len)}); return r; }
};
struct FakeHost : Host {
    bool is_armed = false; uint32_t ms = 0; int cmds = 0; bool live = true; uint8_t last_cmd = 0; bool rebooted = false; int live_calls = 0; Err next_err = ERR_OK;
    bool armed() override { return is_armed; }
    uint32_t now_ms() override { return ms; }
    uint16_t fill_stream(uint8_t id, uint8_t* out, uint16_t cap) override { const StreamInfo* si = find_stream(id); if (!si || si->size > cap) return 0; memset(out, id, si->size); return si->size; }
    Err command(uint8_t cmd, const uint8_t* a, uint16_t n, uint8_t* res, uint16_t& rlen) override { cmds++; last_cmd = cmd; rlen = n; memcpy(res, a, n); return next_err; }
    bool apply_live(uint8_t) override { live_calls++; return live; }
    void reboot() override { rebooted = true; }
    uint8_t autosave = 0; uint8_t take_autosave() override { const uint8_t a = autosave; autosave = 0; return a; }
    bool wrote = false; bool take_settings_changed() override { const bool w = wrote; wrote = false; return w; }
};
static std::vector<uint8_t> frame(uint8_t type, uint8_t seq, std::vector<uint8_t> p = {}) { std::vector<uint8_t> o(p.size() + kOverhead); const size_t n = encode(o.data(), o.size(), type, seq, p.data(), (uint16_t)p.size()); o.resize(n); return o; }

static void t_codec() {
    printf("[codec]\n");
    CHECK(crc16((const uint8_t*)"123456789", 9) == 0x29B1, "CRC-16/CCITT-FALSE check value 0x29B1 (got 0x%04X)", crc16((const uint8_t*)"123456789", 9));
    uint8_t buf[64]; const uint8_t pay[3] = {1, 2, 3}; const size_t n = encode(buf, sizeof(buf), 0x42, 7, pay, 3);
    CHECK(n == 11 && buf[0] == 0xA5 && buf[1] == 0x5A && buf[2] == 3 && buf[3] == 0 && buf[4] == 0x42 && buf[5] == 7, "layout");
    Parser p; Frame f; bool got = false; for (size_t i = 0; i < n; i++) got = p.feed(buf[i], f);
    CHECK(got && f.type == 0x42 && f.seq == 7 && f.len == 3 && f.payload[2] == 3, "round trip");
    // garbage, split, corrupted, back-to-back, partial sync bytes
    std::vector<uint8_t> stream = {0x00, 0xA5, 0xA5, 0x13, 0x77}; auto a = frame(0x10, 1, {9}); auto b = frame(0x11, 2, {8, 8}); auto bad = frame(0x12, 3, {5, 5, 5}); bad[7] ^= 0x40;
    stream.insert(stream.end(), a.begin(), a.end()); stream.insert(stream.end(), bad.begin(), bad.end()); stream.insert(stream.end(), b.begin(), b.end());
    Parser q; int cnt = 0; uint8_t types[4] = {}; for (uint8_t c : stream) { if (q.feed(c, f)) { types[cnt++] = f.type; while (q.pump(f)) types[cnt++] = f.type; } }
    CHECK(cnt == 2 && types[0] == 0x10 && types[1] == 0x11 && q.errors() == 1, "garbage skipped, corrupted frame dropped, next frame still received (%d frames, %u errors)", cnt, q.errors());
    std::vector<uint8_t> big(kMaxPayload + 1); CHECK(encode(buf, sizeof(buf), 1, 0, big.data(), (uint16_t)big.size()) == 0, "oversize payload refused");
    uint8_t hdr[8] = {0xA5, 0x5A, 0xFF, 0xFF, 1, 0, 0, 0}; Parser r; int m = 0; for (uint8_t c : hdr) m += r.feed(c, f); CHECK(m == 0 && r.errors() == 1, "absurd length rejected without waiting for 65535 bytes");
    std::vector<uint8_t> two = frame(0x01, 1); auto three = frame(0x02, 2); two.insert(two.end(), three.begin(), three.end()); Parser s2; int c2 = 0; for (uint8_t c : two) { if (s2.feed(c, f)) { c2++; while (s2.pump(f)) c2++; } } CHECK(c2 == 2, "empty-payload frames back to back");
}

static void t_schema() {
    printf("[schema]\n");
    Settings s; defaults(s);
    CHECK(s.general.loop_hz == 4000 && s.general.est_hz == 500 && s.general.max_arm_angle == 25 && s.rates.max_rate[0] == 670 && s.rates.max_rate[2] == 400, "defaults");
    CHECK(s.motors.protocol == motor::PROTO_DSHOT300 && s.motors.idle_x10 == 55 && s.motors.order[7] == 7 && s.gps.baud == 115200 && s.pins.imu_cs == 255 && s.pins.motor[3] == 255, "defaults (motors, gps, pins unset = 255)");
    CHECK(s.pid.p[0] == 45 && s.pid.i[1] == 84 && s.pid.d[1] == 34 && s.pid.d_max[1] == 46 && s.pid.f[1] == 125 && s.pid.d[2] == 0 && s.pid.slider_mode == SLIDER_MODE_RPY && s.pid.slider_master == 100 && s.pid.stab_p[2] == 4.5f, "pid defaults = Betaflight's numbers, sliders on");
    CHECK(s.filters.gyro_lpf1_dyn_min_hz == 250 && s.filters.gyro_lpf1_dyn_max_hz == 500 && s.filters.gyro_lpf2_hz == 500 && s.filters.dterm_lpf1_dyn_min_hz == 75 && s.filters.dterm_lpf2_hz == 150 && s.filters.dyn_notch_count == 3 &&
          s.filters.rpm_harmonics == 3 && s.filters.yaw_lowpass_hz == 100 && s.filters.slider_gyro == 1 && s.filters.slider_dterm_mult == 100, "filter defaults = Betaflight's");
    CHECK(s.beeper.conditions == 0x5FFFFu && s.general.fs_guard_ms == 500 && s.general.airmode_start_throttle == 25 && s.receiver.rc_smoothing == 1 && s.receiver.rc_smoothing_factor == 30, "u32 default, general, receiver");
    size_t total = 0; for (const SectionInfo& si : kSections) total += si.size; CHECK(kSectionCount == 27 && total == sizeof(Settings), "section table covers the Settings struct exactly (%zu vs %zu)", total, sizeof(Settings));
    CHECK(find_section(SEC_PID)->size == sizeof(S_pid) && find_section(99) == nullptr, "lookup");
    S_general g; defaults(g); g.loop_hz = 60000; g.max_arm_angle = 0; g.board_align = 99; g.airmode = 7; g.board_trim_yaw = -9999;
    Settings t; defaults(t); t.general = g; const int fixed = validate_section(t, SEC_GENERAL);
    CHECK(fixed == 5 && t.general.loop_hz == 8000 && t.general.max_arm_angle == 5 && t.general.board_align == 7 && t.general.airmode == 2 && t.general.board_trim_yaw == -1800, "validation clamps ranges, enums and bools (%d fixed)", fixed);
    t.motors.protocol = 200; t.motors.order[3] = 99; t.pid.p[1] = 255; t.pid.f[0] = 5000; t.pid.ff_smooth_factor = 200; t.pid.iterm_relax = 9; t.pid.slider_d = 201; t.pid.stab_p[1] = 50.0f; t.pid.stab_p[2] = NAN; t.pid.iterm_windup = 5;
    validate_section(t, SEC_MOTORS); validate_section(t, SEC_PID);
    CHECK(t.motors.protocol == ESC_PROTOCOL_DRONECAN && t.motors.order[3] == 7 && t.pid.p[1] == 250 && t.pid.f[0] == 1000 && t.pid.ff_smooth_factor == 95 && t.pid.iterm_relax == ITERM_RELAX_RPY_INC && t.pid.slider_d == 200 && t.pid.stab_p[1] == 12.0f &&
          t.pid.stab_p[2] == 4.5f && t.pid.iterm_windup == 30, "motors / pid clamps (a NaN gain becomes the default)");
    t.filters.gyro_lpf1_type = 9; t.filters.gyro_lpf2_hz = 4000; t.filters.dyn_notch_count = 20; t.filters.rpm_harmonics = 5; t.filters.rpm_q = 10; t.filters.slider_gyro_mult = 3; t.filters.rpm_weights[2] = 180; validate_section(t, SEC_FILTERS);
    CHECK(t.filters.gyro_lpf1_type == LPF_TYPE_PT3 && t.filters.gyro_lpf2_hz == 1000 && t.filters.dyn_notch_count == 7 && t.filters.rpm_harmonics == 3 && t.filters.rpm_q == 250 && t.filters.slider_gyro_mult == 10 && t.filters.rpm_weights[2] == 100, "filter clamps");
    CHECK(kProtocolVersion == 1 && kStreamCount == 17 && find_stream(STR_SATELLITES)->size == 257 && find_stream(STR_CAN)->size == sizeof(T_can) && find_stream(STR_LINK)->size == sizeof(T_link) && find_stream(STR_TUNING)->size == sizeof(T_tuning) && find_stream(STR_ATTITUDE)->size == sizeof(T_attitude), "streams");
}

static void t_store() {
    printf("[settings store]\n");
    MemStorage m; SettingsStore st(m); Settings a; defaults(a); a.general.max_arm_angle = 33; a.pid.anti_gravity_gain = 123; a.motors.reversed_mask = 0x05; a.modes.mode[0] = 0; a.modes.aux[0] = 4; a.modes.lo[0] = 1700; a.modes.hi[0] = 2100;
    int rx_secs = 0; for (const SectionInfo& si : kSections) rx_secs += si.role == ROLE_RX;
    CHECK(st.save(a) && m.kv.size() == (size_t)rx_secs && rx_secs == 25, "one record per section of the flight controller written (%zu)", m.kv.size());
    Settings b; auto r = st.load(b);
    CHECK(r.defaulted_mask == 0 && r.loaded_mask != 0 && b.general.max_arm_angle == 33 && b.pid.anti_gravity_gain == 123 && b.motors.reversed_mask == 5 && b.modes.hi[0] == 2100, "round trip");
    m.kv["wbs05"][6] ^= 0xFF; Settings c; r = st.load(c);
    CHECK((r.defaulted_mask == (1u << SEC_PID)) && c.pid.anti_gravity_gain == 80 && c.general.max_arm_angle == 33, "one corrupted record -> only that section returns to defaults");
    m.kv["wbs01"].push_back(0); Settings d; r = st.load(d); CHECK((r.defaulted_mask & (1u << SEC_GENERAL)) && d.general.max_arm_angle == 25, "a record of another size (older/newer firmware) -> defaults for that section only");
    MemStorage empty; SettingsStore se(empty); Settings e; r = se.load(e); CHECK(r.defaulted_mask != 0 && r.loaded_mask == 0 && e.general.loop_hz == 4000, "blank flash -> all defaults");
    MemStorage evil; SettingsStore sv(evil); Settings bad; defaults(bad); bad.general.loop_hz = 500; sv.save(bad);
    auto& rec = evil.kv["wbs01"]; rec[3] = 0xFF; rec[4] = 0xFF; const uint16_t cc = crc16(rec.data(), rec.size() - 2); rec[rec.size() - 2] = (uint8_t)cc; rec[rec.size() - 1] = (uint8_t)(cc >> 8);
    Settings g; r = sv.load(g); CHECK(g.general.loop_hz == 8000 && r.clamped_fields >= 1, "a validly-CRC'd but out-of-range value is clamped on load (%u)", g.general.loop_hz);
    m.fail_write = true; CHECK(!st.save(a), "storage failure reported");

    // ---- a record written by an older firmware, before fields were appended to its section: loaded as far as it goes
    auto old_record = [](MemStorage& ms, const char* key, const void* data, uint16_t size) {
        std::vector<uint8_t> rec(size + 5u); rec[0] = 1; rec[1] = (uint8_t)size; rec[2] = (uint8_t)(size >> 8); memcpy(rec.data() + 3, data, size);
        const uint16_t c = crc16(rec.data(), size + 3u); rec[size + 3u] = (uint8_t)c; rec[size + 4u] = (uint8_t)(c >> 8); ms.kv[key] = rec;
    };
    CHECK(find_section(SEC_PINS)->compat == 52 && find_section(SEC_PINS)->compat2 == 50 && find_section(SEC_PINS)->compat3 == 48 && find_section(SEC_PINS)->compat4 == 47 && offsetof(S_pins, nrf_sck) == 52 && offsetof(S_pins, baro_cs) == 50 && offsetof(S_pins, mag_cs) == 51 && offsetof(S_pins, can_tx) == 48 && offsetof(S_pins, imu2_cs) == 47 && offsetof(S_pins, esc_tlm) == 46 && find_section(SEC_SENSORS)->compat == 20 && find_section(SEC_SENSORS)->compat2 == 19 && find_section(SEC_SENSORS)->compat3 == 18 && find_section(SEC_SENSORS)->compat4 == 16 && offsetof(S_sensors, esc_boot_wait) == 20 && offsetof(S_sensors, baro_primary) == 19 && offsetof(S_sensors, baro_model) == 18 && offsetof(S_sensors, gyro_use) == 16 && offsetof(S_pins, flow_cs) == 27 && offsetof(S_pins, curr_adc) == 37 && offsetof(S_pins, servo) == 33 && find_section(SEC_BATTERY)->compat == 13 && offsetof(S_battery, current_source) == 13 && find_section(SEC_BLACKBOX)->compat == 8 && find_section(SEC_MOTORS)->compat == 44 && find_section(SEC_MOTORS)->compat2 == 37 && find_section(SEC_MOTORS)->compat3 == 36 && find_section(SEC_MOTORS)->compat4 == 35 && offsetof(S_motors, crashflip_rate) == 44 && offsetof(S_motors, stall_disarm) == 37 && offsetof(S_motors, esc_sensor) == 36 && offsetof(S_motors, ext_pullup) == 35 && find_section(SEC_NAV)->compat == 43 && find_section(SEC_NAV)->compat2 == 38 && find_section(SEC_NAV)->compat3 == 34 && find_section(SEC_NAV)->compat4 == 31 && offsetof(S_nav, loiter_speed) == 43 && offsetof(S_nav, rescue_cone) == 38 && offsetof(S_nav, fence_alt_max) == 34 && offsetof(S_nav, fence_radius) == 36 && offsetof(S_nav, fs_hold_s) == 31 && offsetof(S_nav, batt_crit_action) == 33 && offsetof(S_nav, gps_alt) == 27 && offsetof(S_nav, ph_min_gps_hz) == 29 && offsetof(S_nav, ekf) == 30 && find_section(SEC_GENERAL)->compat == 57 && find_section(SEC_GENERAL)->compat2 == 56 && find_section(SEC_GENERAL)->compat3 == 24 && find_section(SEC_GENERAL)->compat4 == 22 && offsetof(S_general, fly_style) == 57 && offsetof(S_general, flycam_hold) == 58 && offsetof(S_general, reserved_g) == 22 && offsetof(S_general, pid_denom) == 23 && find_section(SEC_GPS)->compat == 0 && offsetof(S_blackbox, sd_mode) == 8,
          "pins (three times: the flow sensor's, the servos', the current sensor's and PWM receiver's), battery and blackbox grew at their end: their old sizes are known, and the old layout really is the start of the new one");
    { MemStorage ms; SettingsStore so(ms); Settings w; defaults(w); so.save(w);
      S_pins op; defaults(op); op.imu_cs = 10; op.motor[3] = 7; op.batt_adc = 2; op.sd_cs = 42; op.flow_cs = 38; op.servo[0] = 99; op.esc_tlm = 44; op.imu2_cs = 34;   // (imu2_cs lies beyond the 47 bytes that are stored)
      old_record(ms, "wbs11", &op, 47);
      S_blackbox ob; defaults(ob); ob.device = BB_DEVICE_FLASH; ob.rate_hz = 1000; ob.sd_mhz = 3;
      old_record(ms, "wbs14", &ob, 8);
      Settings got; const auto rr = so.load(got);
      CHECK(got.pins.imu_cs == 10 && got.pins.motor[3] == 7 && got.pins.batt_adc == 2, "an old pin map is kept: nobody has to type his pins again after an update");
      CHECK(got.pins.sd_cs == 42 && got.pins.flow_cs == 38 && got.pins.servo[0] == 99 && got.pins.esc_tlm == 44 && got.pins.imu2_cs == 255, "the pin map of 0.11 (47 bytes): the pin the old firmware did not know (the second IMU's) stays unset (%u)", got.pins.imu2_cs);
      CHECK(got.blackbox.device == BB_DEVICE_FLASH && got.blackbox.rate_hz == 1000 && got.blackbox.sd_mode == SD_MODE_SPI && got.blackbox.sd_mhz == 20, "the same for the blackbox section");
      CHECK(rr.migrated_mask == ((1u << SEC_PINS) | (1u << SEC_BLACKBOX)) && rr.defaulted_mask == 0 && (rr.loaded_mask & (1u << SEC_PINS)), "and the boot report can say which sections came from an older firmware (0x%x)", (unsigned)rr.migrated_mask);
      so.save(got); Settings again; const auto r2 = so.load(again);
      CHECK(r2.migrated_mask == 0 && again.pins.imu_cs == 10 && ms.kv["wbs11"].size() == sizeof(S_pins) + 5, "after the next save the record has today's size");
      // ... and the record of the firmware in between (0.7.1: with the servo pins, without the current sensor's)
      S_pins p7; defaults(p7); p7.imu_cs = 7; p7.motor[0] = 15; p7.sd_cs = 42; p7.flow_cs = 38; p7.servo[1] = 44; p7.curr_adc = 3;       // (curr_adc lies beyond the 37 bytes that are stored)
      old_record(ms, "wbs11", &p7, 37);
      Settings g7; const auto rr7 = so.load(g7);
      CHECK(g7.pins.imu_cs == 255 && (rr7.defaulted_mask & (1u << SEC_PINS)),
            "a pin map of firmware 0.7.1 (37 bytes): no longer known since 0.28 (four older sizes are kept): defaults");
      S_battery obt; defaults(obt); obt.cells = 4; obt.current_source = CURR_SOURCE_NONE; obt.current_scale = 123; obt.capacity_mah = 999;    // (the current fields lie beyond the 13 bytes that are stored)
      old_record(ms, "wbs13", &obt, 13); Settings gb; const auto rb = so.load(gb);
      CHECK(gb.battery.cells == 4 && gb.battery.current_source == CURR_SOURCE_AUTO && gb.battery.current_scale == 400 && gb.battery.capacity_mah == 0 && (rb.migrated_mask & (1u << SEC_BATTERY)),
            "the battery settings of firmware 0.7 are kept, the current sensor's get their defaults");
      // an old record that is damaged, or of a size that never existed, is not guessed at
      old_record(ms, "wbs11", &op, 33); ms.kv["wbs11"][5] ^= 0x10; Settings bad1; const auto r3 = so.load(bad1);
      CHECK((r3.defaulted_mask & (1u << SEC_PINS)) && bad1.pins.imu_cs == 255, "a damaged old record: defaults");
      { S_pins p6; defaults(p6); p6.imu_cs = 9; p6.sd_cs = 40; p6.flow_cs = 38; old_record(ms, "wbs11", &p6, 27); Settings g6; const auto rr6 = so.load(g6);     // (flow_cs lies beyond the 27 bytes)
        CHECK(g6.pins.imu_cs == 255 && (rr6.defaulted_mask & (1u << SEC_PINS)), "a pin map of firmware 0.6 (27 bytes): no longer known (three older sizes are kept): defaults"); }
      { S_pins p70; defaults(p70); p70.imu_cs = 9; old_record(ms, "wbs11", &p70, 33); Settings g70; const auto rr70 = so.load(g70);
        CHECK(g70.pins.imu_cs == 255 && (rr70.defaulted_mask & (1u << SEC_PINS)), "a pin map of firmware 0.7.0 (33 bytes): no longer known (four older sizes are kept): defaults"); }
      { S_pins p9; defaults(p9); p9.imu_cs = 9; p9.servo[0] = 4; old_record(ms, "wbs11", &p9, 46); Settings g9; const auto rr9 = so.load(g9);
        CHECK(g9.pins.imu_cs == 255 && (rr9.defaulted_mask & (1u << SEC_PINS)), "the pin map of 0.8 - 0.10 (46 bytes): no longer known since the nRF24 pins (0.38: four older sizes are kept): defaults"); }
      { S_pins p47; defaults(p47); p47.imu_cs = 9; p47.servo[0] = 4; old_record(ms, "wbs11", &p47, 47); Settings g47; const auto rr47 = so.load(g47);
        CHECK(g47.pins.imu_cs == 9 && g47.pins.servo[0] == 4 && g47.pins.imu2_cs == 255 && g47.pins.nrf_cs == 255 && (rr47.migrated_mask & (1u << SEC_PINS)), "the pin map of 47 bytes: kept, the pins after it unset"); }
      old_record(ms, "wbs11", &op, 23); Settings bad2; const auto r4 = so.load(bad2);
      CHECK((r4.defaulted_mask & (1u << SEC_PINS)) && bad2.pins.imu_cs == 255, "a size that is neither today's nor a known old one (23 bytes: firmware 0.5, four layouts ago): defaults");
      S_gps og; defaults(og); og.rate_hz = 5; old_record(ms, "wbs08", &og, (uint16_t)(sizeof(S_gps) - 1)); Settings bad3; const auto r5 = so.load(bad3);
      CHECK((r5.defaulted_mask & (1u << SEC_GPS)) && bad3.gps.rate_hz == 10, "a section without a known old size (gps) never takes a shorter record");
      { S_general og2; defaults(og2); og2.loop_hz = 2000; uint8_t rec[23]; memcpy(rec, &og2, 22); rec[22] = 1;
        old_record(ms, "wbs01", rec, 23); Settings g8; const auto r8 = so.load(g8);
        CHECK(sizeof(S_general) == 59 && g8.general.loop_hz != 2000 && (r8.defaulted_mask & (1u << SEC_GENERAL)), "the general section of 0.9.0 - 0.9.4 (23 bytes): since 0.43 (Flycam) not one of the four sizes kept - defaults"); }
      { S_general og3; defaults(og3); og3.loop_hz = 8000; og3.est_hz = 250; old_record(ms, "wbs01", &og3, 22); Settings g9; const auto r9 = so.load(g9);
        CHECK(g9.general.loop_hz == 8000 && g9.general.est_hz == 250 && g9.general.pid_denom == 1 && (r9.migrated_mask & (1u << SEC_GENERAL)), "the general section of 0.9.5 - 0.11 (22 bytes): kept, the control rate = the gyro's (pid_denom 1)"); }
      { S_sensors os; defaults(os); os.mag_enabled = 1; os.range_l1x_mode = 2; old_record(ms, "wbs07", &os, 16); Settings g16; const auto r16 = so.load(g16);
        CHECK(sizeof(S_sensors) == 21 && g16.sensors.esc_boot_wait == 20 && g16.sensors.baro_primary == 0 && g16.sensors.baro_model == BARO_MODEL_AUTO && g16.sensors.mag_enabled == 1 && g16.sensors.range_l1x_mode == 2 && g16.sensors.gyro_use == GYRO_USE_FIRST && g16.sensors.imu2_align == ALIGN_CW0 && (r16.migrated_mask & (1u << SEC_SENSORS)),
              "the sensors section before 0.13 (16 bytes): kept, the second IMU's settings at their defaults (FIRST, CW0)"); }
      { S_sensors os; defaults(os); os.gyro_use = GYRO_USE_BOTH; os.mag_model = MAG_MODEL_DRONECAN; old_record(ms, "wbs07", &os, 18); Settings g18; const auto r18 = so.load(g18);
        CHECK(g18.sensors.gyro_use == GYRO_USE_BOTH && g18.sensors.mag_model == MAG_MODEL_DRONECAN && g18.sensors.baro_model == BARO_MODEL_AUTO && (r18.migrated_mask & (1u << SEC_SENSORS)),
              "the sensors section of 0.13 - 0.16 (18 bytes): kept, the barometer AUTO (the one on the sensor bus)"); }
      { S_osd oo; defaults(oo); oo.type = OSD_TYPE_MSP_DISPLAYPORT; oo.pos[23] = 0x0805; oo.name[0] = 'Q'; old_record(ms, "wbs24", &oo, 73); Settings g33; const auto r33 = so.load(g33);
        CHECK(sizeof(S_osd) == 90 && find_section(SEC_OSD)->compat == 73 && g33.osd.type == OSD_TYPE_MSP_DISPLAYPORT && g33.osd.pos[23] == 0x0805 && g33.osd.name[0] == 'Q' && g33.osd.pos[24] == 172 && !(g33.osd.pos[31] & 0x800) && g33.osd.stats == 1 && (r33.migrated_mask & (1u << SEC_OSD)),
              "the osd section before 0.33 (73 bytes, 24 elements): kept, the 8 new elements at their (hidden) defaults, statistics on"); }
      { S_can c; defaults(c); CHECK(c.mode == CAN_MODE_OFF && c.bitrate == CAN_BITRATE_M1 && c.node_id == 10 && c.dna_server == 1 && !find_section(SEC_CAN)->live, "can: off by default, 1 Mbit/s, node 10, hands out ids, needs a reboot");
        c.node_id = 0; validate(c); CHECK(c.node_id == 1, "node id 0 is not a node id: 1"); }
      { S_pins p27; defaults(p27); p27.imu_cs = 9; p27.can_tx = 33; p27.baro_cs = 35; p27.mag_cs = 36; old_record(ms, "wbs11", &p27, 50); Settings g27; const auto r27 = so.load(g27);
        CHECK(g27.pins.imu_cs == 9 && g27.pins.can_tx == 33 && g27.pins.baro_cs == 255 && g27.pins.mag_cs == 255 && (r27.migrated_mask & (1u << SEC_PINS)), "the pin map of 0.15 - 0.27 (50 bytes): kept, the barometer's and the compass's chip selects unset"); }
      { S_nav on; defaults(on); on.fs_procedure = FS_PROCEDURE_RESCUE; on.batt_low_action = FS_BATT_LOW_LAND; const std::string nk = std::string("wbs") + (SEC_NAV < 10 ? "0" : "") + std::to_string(SEC_NAV); old_record(ms, nk.c_str(), &on, 31);
        Settings gn; const auto rn = so.load(gn);
        CHECK(gn.nav.fs_procedure == FS_PROCEDURE_RESCUE && gn.nav.batt_low_action == FS_BATT_LOW_WARN && gn.nav.fs_hold_s == 30 && gn.nav.batt_crit_action == FS_BATT_CRIT_LAND && (rn.migrated_mask & (1u << SEC_NAV)),
              "the nav section of 0.18 - 0.31 (31 bytes): kept, the failsafe actions of 0.32 at their defaults (WARN, LAND, hold 30 s)"); }
      { S_althold oa; defaults(oa); oa.climb_rate = 33; oa.acc_p = 7; old_record(ms, "wbs19", &oa, 6); Settings g6a; const auto r6a = so.load(g6a);
        CHECK(find_section(SEC_ALTHOLD)->compat == 6 && offsetof(S_althold, accel_z) == 6 && g6a.althold.climb_rate == 33 && g6a.althold.acc_p == 50 && g6a.althold.accel_z == 25 && (r6a.migrated_mask & (1u << SEC_ALTHOLD)),
              "the althold section before 0.29 (6 bytes): kept, the new controller's settings at their defaults"); }
      { S_sensors os; defaults(os); os.baro_model = BARO_MODEL_DRONECAN; os.baro_primary = 1; old_record(ms, "wbs07", &os, 19); Settings g19; const auto r19 = so.load(g19);
        CHECK(g19.sensors.baro_model == BARO_MODEL_DRONECAN && g19.sensors.baro_primary == 0 && (r19.migrated_mask & (1u << SEC_SENSORS)), "the sensors section of 0.15 - 0.27 (19 bytes): kept, baro_primary 0"); }
      { S_pins p11; defaults(p11); p11.imu_cs = 9; p11.imu2_cs = 34; old_record(ms, "wbs11", &p11, 48); Settings g11; const auto r11 = so.load(g11);
        CHECK(g11.pins.imu2_cs == 34 && g11.pins.can_tx == 255 && g11.pins.can_rx == 255 && (r11.migrated_mask & (1u << SEC_PINS)), "the pin map of 0.14 (48 bytes): kept, the CAN pins unset"); }
      {   // 0.20: receiver.telemetry at the end of the receiver section (12 bytes before)
        CHECK(sizeof(S_receiver) == 22 && offsetof(S_receiver, telemetry) == 12 && offsetof(S_receiver, nrf_addr) == 13 && find_section(SEC_RECEIVER)->compat == 13 && find_section(SEC_RECEIVER)->compat2 == 12 && find_section(SEC_RECEIVER)->compat3 == 9, "receiver: telemetry switch, then the nRF24 fields at the end; the old sizes 13, 12 and 9 known");
        S_receiver d; defaults(d); CHECK(d.telemetry == 1, "default: telemetry on (as before)");
        MemStorage ms; SettingsStore so(ms); Settings w; defaults(w); so.save(w);
        S_receiver o; defaults(o); o.protocol = RX_PROTOCOL_SBUS; o.ch_throttle = 0; o.rc_smoothing_throttle = 77; o.telemetry = 0;     // (not stored: beyond 12 bytes)
        std::vector<uint8_t> rec(12 + 5u); rec[0] = 1; rec[1] = 12; rec[2] = 0; memcpy(rec.data() + 3, &o, 12);
        const uint16_t c = crc16(rec.data(), 15); rec[15] = (uint8_t)c; rec[16] = (uint8_t)(c >> 8);
        char key[8]; snprintf(key, sizeof(key), "wbs%02u", (unsigned)SEC_RECEIVER); ms.kv[key] = rec;
        Settings got; const auto rr = so.load(got);
        CHECK(got.receiver.protocol == RX_PROTOCOL_SBUS && got.receiver.rc_smoothing_throttle == 77 && got.receiver.telemetry == 1 && (rr.migrated_mask & (1u << SEC_RECEIVER)), "a receiver section of 0.19 is kept, telemetry stays on"); }
      {   // 0.22: led MANUAL (a function and a color per LED, warnings, speed) after the 3 bytes of 0.16
        S_led d; defaults(d);
        CHECK(sizeof(S_led) == 165 && offsetof(S_led, fn) == 3 && find_section(SEC_LED)->compat == 3 && d.fn[0] == LED_FUNCTION_STATUS && d.fn[31] == LED_FUNCTION_STATUS
              && d.color[0] == 0xFFFFFF && d.color[31] == 0xFFFFFF && d.warnings == 1 && d.speed == 5, "led: MANUAL fields at the end, defaults: every LED the state picture, white, warnings on, speed 5");
        d.speed = 0; d.fn[3] = 99; d.color[2] = 0x1000000; validate(d); CHECK(d.speed == 1 && d.fn[3] == LED_FUNCTION_THROTTLE && d.color[2] == 0xFFFFFF, "out of range: clamped");
        MemStorage ms; SettingsStore so(ms); Settings w; defaults(w); so.save(w);
        S_led o; defaults(o); o.count = 12; o.brightness = 70; o.layout = LED_LAYOUT_NAV;
        std::vector<uint8_t> rec(3 + 5u); rec[0] = 1; rec[1] = 3; rec[2] = 0; memcpy(rec.data() + 3, &o, 3);
        const uint16_t c = crc16(rec.data(), 6); rec[6] = (uint8_t)c; rec[7] = (uint8_t)(c >> 8);
        char key[8]; snprintf(key, sizeof(key), "wbs%02u", (unsigned)SEC_LED); ms.kv[key] = rec;
        Settings got; const auto rr = so.load(got);
        CHECK(got.led.count == 12 && got.led.brightness == 70 && got.led.layout == LED_LAYOUT_NAV && got.led.fn[5] == LED_FUNCTION_STATUS && got.led.speed == 5 && (rr.migrated_mask & (1u << SEC_LED)), "an led section of 0.21 is kept, the manual fields at their defaults"); }
      {   // 0.24: general.stick_arm at the end (56 bytes before)
        S_general d; defaults(d);
        CHECK(offsetof(S_general, stick_arm) == 56 && find_section(SEC_GENERAL)->compat2 == 56 && d.stick_arm == 0 && d.fly_style == FLY_STYLE_FPV && d.flycam_hold == FLYCAM_HOLD_LOITER, "general: stick_arm, then (0.43) fly_style FPV / flycam_hold LOITER at the end; the old sizes 57 and 56 known");
        MemStorage ms; SettingsStore so(ms); Settings w; defaults(w); so.save(w);
        S_general o; defaults(o); o.loop_hz = 2000; o.airmode = 2;
        std::vector<uint8_t> rec(56 + 5u); rec[0] = 1; rec[1] = 56; rec[2] = 0; memcpy(rec.data() + 3, &o, 56);
        const uint16_t c = crc16(rec.data(), 59); rec[59] = (uint8_t)c; rec[60] = (uint8_t)(c >> 8);
        char key[8]; snprintf(key, sizeof(key), "wbs%02u", (unsigned)SEC_GENERAL); ms.kv[key] = rec;
        Settings got; const auto rr = so.load(got);
        CHECK(got.general.loop_hz == 2000 && got.general.airmode == 2 && got.general.stick_arm == 0 && (rr.migrated_mask & (1u << SEC_GENERAL)), "a general section of 0.23 is kept, stick arming off"); }
      {   // 0.23: the acro trainer's settings at the end of the pid section (75 bytes before)
        S_pid d; defaults(d);
        CHECK(sizeof(S_pid) == 79 && offsetof(S_pid, acro_trainer_angle_limit) == 75 && find_section(SEC_PID)->compat == 75 && d.acro_trainer_angle_limit == 20 && d.acro_trainer_lookahead_ms == 50 && d.acro_trainer_gain == 75,
              "pid: acro trainer limit / lookahead / gain at the end, Betaflight's defaults, the old size known");
        d.acro_trainer_angle_limit = 5; d.acro_trainer_lookahead_ms = 900; d.acro_trainer_gain = 3; validate(d);
        CHECK(d.acro_trainer_angle_limit == 10 && d.acro_trainer_lookahead_ms == 200 && d.acro_trainer_gain == 25, "Betaflight's ranges: 10 - 80 deg, 10 - 200 ms, 25 - 255");
        MemStorage ms; SettingsStore so(ms); Settings w; defaults(w); so.save(w);
        S_pid o; defaults(o); o.iterm_relax_cutoff = 31; o.anti_gravity_gain = 120;
        std::vector<uint8_t> rec(75 + 5u); rec[0] = 1; rec[1] = 75; rec[2] = 0; memcpy(rec.data() + 3, &o, 75);
        const uint16_t c = crc16(rec.data(), 78); rec[78] = (uint8_t)c; rec[79] = (uint8_t)(c >> 8);
        char key[8]; snprintf(key, sizeof(key), "wbs%02u", (unsigned)SEC_PID); ms.kv[key] = rec;
        Settings got; const auto rr = so.load(got);
        CHECK(got.pid.iterm_relax_cutoff == 31 && got.pid.anti_gravity_gain == 120 && got.pid.acro_trainer_gain == 75 && (rr.migrated_mask & (1u << SEC_PID)), "a pid section of 0.22 is kept, the trainer at its defaults"); }
      CHECK(MODE_BLACKBOX_ERASE == 13 && MODE_ACRO_TRAINER == 10, "modes appended after RESCUE: ACRO_TRAINER 10, AIRMODE 11, ANTI_GRAVITY 12, BLACKBOX_ERASE 13");
    { S_telemetry t; defaults(t);
        CHECK(sizeof(S_telemetry) == 16 && t.remote_arm == 0 && t.mp_home_item == 0 && find_section(SEC_TELEMETRY)->compat == 15 && find_section(SEC_TELEMETRY)->compat2 == 14 && t.protocol == TELEMETRY_PROTOCOL_NONE && t.baud == 57600 && t.tx_pin == 255 && t.rx_pin == 255 && t.sysid == 1 && t.reserved_t == 0 && t.rate_attitude == 10
              && t.rate_position == 5 && t.rate_hud == 4 && t.rate_status == 2 && t.rate_rc == 2 && !find_section(SEC_TELEMETRY)->live,
              "telemetry (0.19): off, 57600 baud, no pins, system id 1, 10 / 5 / 4 / 2 / 2 per second, needs a reboot; 0.40: remote_arm off; 0.40.1: mp_home_item off; the 15- and 14-byte records of before still read");
        t.rate_attitude = 200; t.sysid = 0; t.baud = 300; t.protocol = 9; validate(t);
        CHECK(t.rate_attitude == 50 && t.sysid == 1 && t.baud == 9600 && t.protocol == TELEMETRY_PROTOCOL_MAVLINK, "out of range: clamped (ATTITUDE 50 per second at most, system id 1, 9600 baud)");
        MemStorage m18; SettingsStore s18(m18); Settings o; defaults(o); o.gps.baud = 57600; s18.save(o); m18.kv.erase("wbs23");      // what 0.18 left in the flash: no telemetry record
        Settings n; const auto r18 = s18.load(n);
        CHECK(n.telemetry.protocol == TELEMETRY_PROTOCOL_NONE && n.gps.baud == 57600 && r18.defaulted_mask == (1u << SEC_TELEMETRY), "settings of 0.18: kept, telemetry at its defaults (off)"); }
      // the motors section of firmware 0.5 (34 bytes, before the DShot driver choice was added): since 0.42 (crash flip) no longer
      // one of the four sizes kept - defaults
      S_motors om; defaults(om); om.pole_count = 12; om.protocol = ESC_PROTOCOL_DSHOT600; om.bidirectional = 1; om.reserved1 = 1;
      old_record(ms, "wbs06", &om, 34); Settings got2; const auto r6 = so.load(got2);
      CHECK(got2.motors.pole_count != 12 && (r6.defaulted_mask & (1u << SEC_MOTORS)), "the motor settings of firmware 0.5 (34 bytes): defaults since 0.42");
      // the 44-byte record of 0.41 (before crash flip): kept, the crash flip settings at their defaults
      S_motors m41; defaults(m41); m41.pole_count = 12; m41.stall_ms = 300; m41.crashflip_rate = 35; m41.crashflip_auto_rearm = 1;
      old_record(ms, "wbs06", &m41, 44); Settings got3; const auto r41 = so.load(got3);
      CHECK(got3.motors.pole_count == 12 && got3.motors.stall_ms == 300 && got3.motors.crashflip_rate == 0 && got3.motors.crashflip_motor_pct == 0 && got3.motors.crashflip_auto_rearm == 0
            && (r41.migrated_mask & (1u << SEC_MOTORS)), "the motor settings of 0.41 (44 bytes): kept, crash flip at its defaults (rate 0, 0 %%, no auto re-arm)"); }
}

struct Rig {
    Settings s; MemStorage mem; SettingsStore store{mem}; FakeHost host; MemTransport tx; Server srv{s, &store, host, tx, ROLE_RX, 0, 1, 0, "Windburst RX", FEAT_STREAMS | FEAT_GPS};
    Rig() { defaults(s); srv.settings_loaded(); }
    void send(const std::vector<uint8_t>& f) { tx.out.clear(); srv.on_bytes(f.data(), f.size()); }
    template <class T> void set(uint8_t id, const T& v) { std::vector<uint8_t> p(1 + sizeof(T)); p[0] = id; memcpy(p.data() + 1, &v, sizeof(T)); send(frame(MSG_SECTION_SET, 7, p)); }
};

static void t_server() {
    printf("[server]\n");
    { Rig r; r.send(frame(MSG_HELLO, 9)); auto fr = r.tx.frames();
      CHECK(fr.size() == 1 && fr[0].first.type == MSG_DEVICE_INFO && fr[0].first.seq == 9, "HELLO -> DEVICE_INFO echoing the seq");
      const auto& p = fr[0].second; uint32_t hash = p[5] | (p[6] << 8) | (p[7] << 16) | ((uint32_t)p[8] << 24);
      CHECK(p[0] == kProtocolVersion && p[1] == ROLE_RX && hash == kSchemaHash && std::string(p.begin() + 13, p.end()) == "Windburst RX", "info: version, role, schema hash, name"); }
    { // 0.25: the chip by name and its pins
      Rig r; uint8_t mask[32] = {}; mask[0] = 0x05; mask[31] = 0x80; r.srv.set_chip("STM32F405", mask); r.send(frame(MSG_HELLO, 3)); auto fr = r.tx.frames();
      const auto& p = fr[0].second; const std::string rest(p.begin() + 13, p.end());
      const size_t z1 = rest.find('\0'), z2 = rest.find('\0', z1 + 1);
      CHECK(z1 == 12 && rest.substr(0, z1) == "Windburst RX" && rest.substr(z1 + 1, z2 - z1 - 1) == "STM32F405" && rest.size() - z2 - 1 == 32
            && (uint8_t)rest[z2 + 1] == 0x05 && (uint8_t)rest[z2 + 32] == 0x80, "set_chip: name, 0, MCU, 0, the 32-byte pin mask (pins 0, 2, 255 here)"); }
    { uint8_t m[32] = {}; port::esp32s3_pins(m, true);
      CHECK(port::pin_mask_has(m, 0) && port::pin_mask_has(m, 21) && !port::pin_mask_has(m, 22) && !port::pin_mask_has(m, 32) && !port::pin_mask_has(m, 35) && port::pin_mask_has(m, 38) && port::pin_mask_has(m, 48) && !port::pin_mask_has(m, 49),
            "ESP32-S3 N16R8: GPIO 0 - 21, 38 - 48 (no flash 26 - 32, no octal PSRAM 33 - 37)");
      memset(m, 0, 32); port::esp32s3_pins(m, false); CHECK(port::pin_mask_has(m, 33) && port::pin_mask_has(m, 37), "without octal PSRAM 33 - 37 are there");
      memset(m, 0, 32); port::esp32_pins(m);
      int n = 0; for (int i = 0; i < 256; i++) n += port::pin_mask_has(m, i);
      CHECK(n == 28 && port::pin_mask_has(m, 5) && !port::pin_mask_has(m, 6) && !port::pin_mask_has(m, 11) && port::pin_mask_has(m, 12) && !port::pin_mask_has(m, 20) && !port::pin_mask_has(m, 24) && port::pin_mask_has(m, 39) && !port::pin_mask_has(m, 40),
            "plain ESP32: 28 GPIOs (0 - 39 without the flash 6 - 11 and the missing 20, 24, 28 - 31) (%d)", n);
      memset(m, 0, 32); port::stm32_port_pins(m, 7); CHECK(port::pin_mask_has(m, 0) && port::pin_mask_has(m, 111) && !port::pin_mask_has(m, 112), "the simulator's STM32: PA0 .. PG15"); }
    { Rig r; r.send(frame(MSG_SECTION_GET, 1, {SEC_RATES})); auto fr = r.tx.frames(); CHECK(fr.size() == 1 && fr[0].first.type == MSG_SECTION && fr[0].second[0] == SEC_RATES && fr[0].second.size() == 3 + sizeof(S_rates), "SECTION_GET");
      CHECK(memcmp(fr[0].second.data() + 3, &r.s.rates, sizeof(S_rates)) == 0, "data = the live struct");
      r.send(frame(MSG_SECTION_GET, 2, {99})); fr = r.tx.frames(); CHECK(fr[0].first.type == MSG_ACK && fr[0].second[2] == ERR_BAD_SECTION, "unknown section"); }
    { Rig r; S_rates nr = r.s.rates; nr.max_rate[0] = 800; nr.angle_limit = 200; std::vector<uint8_t> p(1 + sizeof(S_rates)); p[0] = SEC_RATES; memcpy(p.data() + 1, &nr, sizeof(nr));
      r.send(frame(MSG_SECTION_SET, 3, p)); auto fr = r.tx.frames();
      S_rates got; memcpy(&got, fr[0].second.data() + 3, sizeof(got));
      CHECK(fr[0].first.type == MSG_SECTION && got.max_rate[0] == 800 && got.angle_limit == 80 && r.s.rates.angle_limit == 80, "SET stores range-validated values and returns what was ACTUALLY stored (200 -> 80)");
      CHECK((r.srv.flags() & SF_DIRTY) && !(r.srv.flags() & SF_REBOOT_REQUIRED) && r.host.live_calls == 1, "unsaved flag set, applied live");
      r.host.live = false; r.send(frame(MSG_SECTION_SET, 4, p)); CHECK(r.srv.flags() & SF_REBOOT_REQUIRED, "a section the firmware cannot apply live => reboot required");
      r.host.is_armed = true; nr.max_rate[0] = 100; memcpy(p.data() + 1, &nr, sizeof(nr)); r.send(frame(MSG_SECTION_SET, 5, p)); fr = r.tx.frames();
      CHECK(fr[0].first.type == MSG_ACK && fr[0].second[2] == ERR_ARMED && r.s.rates.max_rate[0] == 800, "while ARMED nothing is changed");
      r.host.is_armed = false; p.pop_back(); r.send(frame(MSG_SECTION_SET, 6, p)); fr = r.tx.frames(); CHECK(fr[0].second[2] == ERR_SIZE_MISMATCH, "wrong size refused"); }
    { Rig r; r.s.general.max_arm_angle = 30; r.send(frame(MSG_SETTINGS_SAVE, 1)); auto fr = r.tx.frames(); CHECK(fr[0].second[2] == ERR_OK && r.mem.kv.size() == (size_t)kSectionCount - 2 && !(r.srv.flags() & SF_DIRTY), "save writes flash (the flight controller's sections) and clears the unsaved flag");
      r.mem.fail_write = true; r.send(frame(MSG_SETTINGS_SAVE, 2)); fr = r.tx.frames(); CHECK(fr[0].second[2] == ERR_STORAGE, "flash failure reported");
      r.host.is_armed = true; r.send(frame(MSG_SETTINGS_SAVE, 3)); fr = r.tx.frames(); CHECK(fr[0].second[2] == ERR_ARMED, "no flash writes while armed (they stall the loop)"); }
    { Rig r; r.s.pid.tpa_rate = 10; r.s.general.loop_hz = 2000; r.send(frame(MSG_SETTINGS_DEFAULTS, 1, {SEC_PID})); CHECK(r.s.pid.tpa_rate == 65 && r.s.general.loop_hz == 2000, "defaults for one section");
      r.send(frame(MSG_SETTINGS_DEFAULTS, 2, {0xFF})); CHECK(r.s.general.loop_hz == 4000, "defaults for everything"); r.send(frame(MSG_SETTINGS_DEFAULTS, 3, {77})); CHECK(r.tx.frames()[0].second[2] == ERR_BAD_SECTION, "unknown section"); }
    // a section the firmware changed by itself (in-flight level trim) and wants kept: written without a Save from the app
    { Rig r; r.store.save(r.s); r.s.calibration.acc_trim_roll = 17; r.s.general.max_arm_angle = 44; r.tx.out.clear(); r.host.autosave = SEC_CALIBRATION; r.srv.tick(1);
      Settings back; r.store.load(back); auto fr = r.tx.frames();
      CHECK(back.calibration.acc_trim_roll == 17 && back.general.max_arm_angle == 25 && r.srv.autosaved() == 1 && !(r.srv.flags() & SF_DIRTY), "the firmware's own change: that ONE section goes to flash (the rest of RAM does not), nothing shows as unsaved");
      CHECK(fr.size() == 1 && fr[0].first.type == MSG_LOG && std::string(fr[0].second.begin() + 1, fr[0].second.end()).find("calibration") != std::string::npos, "and the app is told");
      r.srv.tick(2); CHECK(r.srv.autosaved() == 1, "once");
      r.mem.fail_write = true; r.host.autosave = SEC_CALIBRATION; r.srv.tick(3); CHECK(r.srv.autosaved() == 1 && (r.srv.flags() & SF_DIRTY), "the flash refuses: the settings show as unsaved instead");
      Rig q; q.host.autosave = 99; q.srv.tick(1); CHECK(q.srv.autosaved() == 0 && !(q.srv.flags() & SF_DIRTY) && q.mem.kv.empty(), "an unknown section is ignored");
      q.tx.room = 0; q.host.autosave = SEC_CALIBRATION; q.srv.tick(2); CHECK(q.srv.autosaved() == 1 && q.tx.out.empty(), "no room in the transmit buffer (no app listening): saved all the same, the message is dropped - never blocked on"); }
    { Settings s2; defaults(s2); FakeHost h2; MemTransport t2; Server nos{s2, nullptr, h2, t2, ROLE_RX, 0, 1, 0, "x", 0}; h2.autosave = SEC_CALIBRATION; nos.tick(1);
      CHECK(nos.autosaved() == 0 && (nos.flags() & SF_DIRTY), "a firmware without a settings store: nothing is written, the change shows as unsaved"); }
    // 0.37.1: unsaved / reboot required = differs from flash / from what the firmware started with, per section - a value changed
    // and set back as it was asks for nothing (the app showed "Save" until the next Save)
    { Rig r; const S_rates rates0 = r.s.rates; S_rates nr = rates0; nr.max_rate[0] = 800;
      CHECK(r.srv.flags() == 0, "as loaded: nothing to save");
      r.set(SEC_RATES, nr); CHECK(r.srv.flags() == SF_DIRTY, "changed: unsaved");
      r.set(SEC_RATES, rates0); CHECK(r.srv.flags() == 0, "set back as it was: nothing to save (the Save button goes away)");
      r.host.live = false; const S_general g0 = r.s.general; S_general ng = g0; ng.loop_hz = 2000;
      r.set(SEC_GENERAL, ng); CHECK(r.srv.flags() == (SF_DIRTY | SF_REBOOT_REQUIRED), "a section the firmware cannot take live: unsaved, waits for a reboot");
      r.set(SEC_GENERAL, g0); CHECK(r.srv.flags() == 0, "set back: the running firmware has that already - no reboot either");
      r.host.live = true; r.set(SEC_RATES, nr); r.host.live = false; r.set(SEC_GENERAL, ng); r.set(SEC_RATES, rates0);
      CHECK(r.srv.flags() == (SF_DIRTY | SF_REBOOT_REQUIRED), "per section: one set back, the other still changed");
      r.send(frame(MSG_SETTINGS_SAVE, 8)); CHECK(r.srv.flags() == SF_REBOOT_REQUIRED, "saved: the reboot is still owed");
      r.set(SEC_GENERAL, g0); CHECK(r.srv.flags() == SF_DIRTY, "set back after the save: flash has the new value (unsaved), the firmware runs the old one (no reboot)"); }
    { Rig r; r.host.live = false; S_pid np = r.s.pid; np.tpa_rate = 10; r.set(SEC_PID, np); CHECK(r.srv.flags() == (SF_DIRTY | SF_REBOOT_REQUIRED), "pid changed");
      r.send(frame(MSG_SETTINGS_DEFAULTS, 1, {SEC_PID})); CHECK(r.srv.flags() == 0, "defaults of a section = what was loaded: nothing to save");
      r.set(SEC_PID, np); r.send(frame(MSG_SETTINGS_DEFAULTS, 2, {0xFF})); CHECK(r.srv.flags() == 0, "defaults for everything, the same");
      r.send(frame(MSG_SETTINGS_DEFAULTS, 3, {SEC_PID})); CHECK(r.srv.flags() == 0, "defaults of a section that has them: nothing changed"); }
    { Rig r; r.host.live = false; S_general ng = r.s.general; ng.loop_hz = 2000; r.set(SEC_GENERAL, ng);
      r.s.calibration.acc_trim_roll = 9; r.host.wrote = true; r.send(frame(MSG_COMMAND, 1, {CMD_GYRO_CAL}));
      r.send(frame(MSG_SETTINGS_SAVE, 2));
      r.s.calibration.acc_trim_roll = 0; r.host.wrote = true; r.send(frame(MSG_COMMAND, 3, {CMD_GYRO_CAL}));
      CHECK(r.srv.flags() == (SF_DIRTY | SF_REBOOT_REQUIRED), "a calibration the firmware wrote itself: unsaved, and the reboot still owed for the other section");
      r.s.calibration.acc_trim_roll = 9; r.host.wrote = true; r.send(frame(MSG_COMMAND, 4, {CMD_GYRO_CAL})); CHECK(r.srv.flags() == SF_REBOOT_REQUIRED, "calibrated back to what flash holds: nothing to save");
      S_calibration nc = r.s.calibration; nc.acc_trim_pitch = 5; r.host.live = true; r.set(SEC_CALIBRATION, nc); r.host.autosave = SEC_CALIBRATION; r.srv.tick(1);
      CHECK(r.srv.flags() == SF_REBOOT_REQUIRED, "a section the firmware wrote to flash itself is saved, the app's change in it too"); }
    { Rig r; r.mem.fail_write = true; r.send(frame(MSG_SETTINGS_SAVE, 1)); CHECK(r.srv.flags() == SF_DIRTY, "a flash write that failed: flash holds who knows what - unsaved, nothing changed or not");
      r.mem.fail_write = false; r.send(frame(MSG_SETTINGS_SAVE, 2)); CHECK(r.srv.flags() == 0, "until a Save goes through"); }
    { Rig r; r.send(frame(MSG_REBOOT, 1)); CHECK(r.host.rebooted && r.tx.frames()[0].second[2] == ERR_OK, "reboot acknowledged first"); r.host.rebooted = false; r.host.is_armed = true; r.send(frame(MSG_REBOOT, 2)); CHECK(!r.host.rebooted, "no reboot while armed"); }
    { Rig r; r.send(frame(MSG_COMMAND, 4, {CMD_MOTOR_TEST, 2, 0xE8, 0x03})); auto fr = r.tx.frames();
      CHECK(r.host.last_cmd == CMD_MOTOR_TEST && fr[0].first.type == MSG_CMD_RESULT && fr[0].second[0] == CMD_MOTOR_TEST && fr[0].second[1] == ERR_OK && fr[0].second.size() == 2 + 3, "COMMAND is forwarded with its args and answered with CMD_RESULT");
      r.host.next_err = ERR_ARMED; r.send(frame(MSG_COMMAND, 5, {CMD_GYRO_CAL})); CHECK(r.tx.frames()[0].second[1] == ERR_ARMED, "the host's error is passed through"); }
    { Rig r; r.send(frame(0x6E, 1)); CHECK(r.tx.frames()[0].second[2] == ERR_UNKNOWN, "unknown message type answered, not ignored"); }
    { Rig r; auto h = frame(MSG_HELLO, 1); std::vector<uint8_t> n = {0xFF, 0x00, 0xA5, 0xA5, 0x5A}; n.insert(n.end(), h.begin(), h.end()); r.tx.out.clear(); r.srv.on_bytes(n.data(), n.size()); CHECK(r.tx.frames().size() == 1, "noise before a request does not hide it (boot log on the same USB port)"); }
    { Parser p; Frame f; auto one = frame(MSG_HELLO, 1), two = frame(MSG_SECTION_GET, 2, {1}); std::vector<uint8_t> hdrfake = {0xA5, 0x5A, 0x20, 0x00, 0x01, 0x00}; hdrfake.insert(hdrfake.end(), one.begin(), one.end()); hdrfake.insert(hdrfake.end(), two.begin(), two.end());
      int got = 0; for (uint8_t c : hdrfake) { if (p.feed(c, f)) { got++; while (p.pump(f)) got++; } }
      CHECK(got == 0 && p.mid_frame(), "while bytes keep coming the false header waits for its 32 bytes");
      if (p.idle(f)) { got++; while (p.pump(f)) got++; } CHECK(got == 2, "a false header that swallows two real frames: both are recovered by the re-scan (%d)", got); }
}

static void t_streams() {
    printf("[streams]\n");
    Rig r; auto cfg = [&](uint8_t id, uint16_t rate) { r.send(frame(MSG_STREAM_CFG, 1, {id, (uint8_t)rate, (uint8_t)(rate >> 8)})); return r.tx.frames()[0].second[2]; };
    CHECK(cfg(STR_ATTITUDE, 50) == ERR_OK && r.srv.stream_rate(STR_ATTITUDE) == 50, "enable at 50 Hz");
    CHECK(cfg(STR_STATUS, 10) == ERR_OK && cfg(99, 10) == ERR_BAD_ARG, "bad stream id refused");
    r.tx.out.clear(); int att = 0, st = 0; uint32_t last_t = 0; bool ordered = true;
    for (uint32_t t = 0; t <= 1000; t += 2) { r.host.ms = t; r.srv.tick(t); }
    for (auto& f : r.tx.frames()) if (f.first.type == MSG_STREAM) { const uint8_t id = f.second[0]; const uint32_t ts = f.second[1] | (f.second[2] << 8) | (f.second[3] << 16) | ((uint32_t)f.second[4] << 24);
        if (id == STR_ATTITUDE) { att++; if (ts < last_t) ordered = false; last_t = ts; } if (id == STR_STATUS) st++;
        CHECK(f.second.size() == 5 + find_stream(id)->size, "stream payload = id + time + struct"); if (att + st > 3) break; }
    att = st = 0; for (auto& f : r.tx.frames()) if (f.first.type == MSG_STREAM) { if (f.second[0] == STR_ATTITUDE) att++; else if (f.second[0] == STR_STATUS) st++; }
    CHECK(att >= 49 && att <= 52 && st >= 9 && st <= 11 && ordered, "1 s at 50 Hz + 10 Hz => %d + %d frames, timestamps in order", att, st);
    CHECK(cfg(STR_ATTITUDE, 5000) == ERR_OK && r.srv.stream_rate(STR_ATTITUDE) == 200, "rate clamped to the stream's maximum (200)");
    r.send(frame(MSG_STREAM_CFG, 2, {0xFF, 0, 0})); CHECK(r.srv.stream_rate(STR_ATTITUDE) == 0 && r.srv.stream_rate(STR_STATUS) == 0, "0xFF switches every stream off");
    cfg(STR_RC, 100); r.tx.out.clear(); r.tx.room = 10; r.host.ms = 5000; r.srv.tick(5000); r.srv.tick(5011); CHECK(r.tx.out.empty() && r.srv.dropped_streams() >= 1, "a full TX buffer skips stream frames instead of blocking");
    r.tx.room = 100000; r.tx.out.clear(); for (uint32_t t = 6000; t < 6100; t++) r.srv.tick(t); const size_t n100 = r.tx.frames().size(); CHECK(n100 >= 9 && n100 <= 12, "after the stall it resumes at the normal rate without a burst (%zu frames in 100 ms)", n100);
    r.tx.out.clear(); r.srv.log(2, "hello"); auto fr = r.tx.frames(); CHECK(fr.size() == 1 && fr[0].first.type == MSG_LOG && fr[0].second[0] == 2 && fr[0].second[1] == 'h', "log message");
}

static void t_apply() {
    printf("[apply] settings -> typed configs\n");
    Settings s; defaults(s); fc::CoreConfig def; fc::CoreConfig c; fc::RcInterpreter rc; apply_core(s, c, rc);
    { const fc::nav::AltConfig ad; CHECK(fabsf(c.alt.mid_stick - ad.mid_stick) < 1e-6f && fabsf(c.alt.deadband - ad.deadband) < 1e-6f && fabsf(c.alt.low_stick - ad.low_stick) < 1e-6f && fabsf(c.alt.max_stick - ad.max_stick) < 1e-6f
            && fabsf(c.alt.climb_max_mps - ad.climb_max_mps) < 1e-5f && fabsf(c.alt.descend_max_mps - ad.descend_max_mps) < 1e-5f
            && fabsf(c.alt.accel_max_mps2 - ad.accel_max_mps2) < 1e-5f && fabsf(c.alt.jerk_max_mps3 - ad.jerk_max_mps3) < 1e-5f && fabsf(c.alt.acc_p - ad.acc_p) < 1e-5f && fabsf(c.alt.acc_i - ad.acc_i) < 1e-5f,
            "althold defaults = the controller's own defaults (nothing changes for a craft that never touches the section)");
      { Settings u; defaults(u); u.althold.accel_z = 15; u.althold.jerk_z = 30; u.althold.acc_p = 0; u.althold.acc_i = 150; fc::nav::AltConfig a; apply_althold(u, a);
        CHECK(fabsf(a.accel_max_mps2 - 1.5f) < 1e-5f && fabsf(a.jerk_max_mps3 - 3.0f) < 1e-5f && a.acc_p == 0.0f && fabsf(a.acc_i - 1.5f) < 1e-5f, "althold 0.28: accel 1.5 m/s^2, jerk 3 m/s^3, acceleration loop P 0 / I 1.5"); }
      Settings t; defaults(t); t.althold.mid_stick = 45; t.althold.climb_rate = 35; t.althold.low_stick = 25; t.althold.deadband = 10; fc::nav::AltConfig a; apply_althold(t, a);
      CHECK(fabsf(a.mid_stick - 0.45f) < 1e-6f && fabsf(a.climb_max_mps - 3.5f) < 1e-5f && a.low_stick <= 0.30f + 1e-6f, "althold: 45 %% holds, 3.5 m/s; a 'low' that would leave no descent travel is pulled down (%.2f)", a.low_stick); }
    CHECK(c.loop_hz == def.loop_hz && c.est_hz == def.est_hz && c.max_arm_angle_deg == def.max_arm_angle_deg && c.airmode == def.airmode && c.airmode == fc::AM_AUTO && fabsf(c.airmode_threshold - def.airmode_threshold) < 1e-6f, "default settings reproduce the core's defaults (loop, arming, airmode)");
    { const fc::PidConfig &a = c.pid, &d = def.pid; bool same = true;
      for (int x = 0; x < 3; x++) same &= a.p[x] == d.p[x] && a.i[x] == d.i[x] && a.d[x] == d.d[x] && a.d_max[x] == d.d_max[x] && a.f[x] == d.f[x] && c.rates.max_rate_dps[x] == def.rates.max_rate_dps[x] && c.rates.center_sens_dps[x] == def.rates.center_sens_dps[x];
      CHECK(same, "default PID gains / rates = the values the closed-loop tests fly with");
      CHECK(a.iterm_relax == d.iterm_relax && a.iterm_relax_type == d.iterm_relax_type && a.iterm_relax_cutoff == d.iterm_relax_cutoff && a.iterm_windup == d.iterm_windup && a.anti_gravity == d.anti_gravity &&
            a.anti_gravity_gain == d.anti_gravity_gain && a.anti_gravity_p_gain == d.anti_gravity_p_gain && a.anti_gravity_cutoff_hz == d.anti_gravity_cutoff_hz && a.d_max_gain == d.d_max_gain && a.d_max_advance == d.d_max_advance,
            "I-term relax, anti-gravity, D max defaults");
      CHECK(a.tpa_mode == d.tpa_mode && a.tpa_rate == d.tpa_rate && fabsf(a.tpa_breakpoint - d.tpa_breakpoint) < 1e-6f && a.tpa_low_rate == d.tpa_low_rate && fabsf(a.tpa_low_breakpoint - d.tpa_low_breakpoint) < 1e-6f && a.tpa_low_always == d.tpa_low_always &&
            c.throttle_boost == def.throttle_boost && c.throttle_boost_cutoff == def.throttle_boost_cutoff, "TPA and throttle boost defaults");
      CHECK(a.dterm_lpf1_type == d.dterm_lpf1_type && a.dterm_lpf1_static_hz == d.dterm_lpf1_static_hz && a.dterm_lpf1_dyn_min_hz == d.dterm_lpf1_dyn_min_hz && a.dterm_lpf1_dyn_max_hz == d.dterm_lpf1_dyn_max_hz && a.dterm_lpf1_dyn_expo == d.dterm_lpf1_dyn_expo &&
            a.dterm_lpf2_type == d.dterm_lpf2_type && a.dterm_lpf2_hz == d.dterm_lpf2_hz && a.dterm_notch_hz == d.dterm_notch_hz && a.dterm_notch_cutoff == d.dterm_notch_cutoff && a.yaw_lowpass_hz == d.yaw_lowpass_hz, "D-term filter and yaw low-pass defaults"); }
    { const fc::GyroFilterConfig &a = c.gyro, &d = def.gyro;
      CHECK(a.lpf1_type == d.lpf1_type && a.lpf1_static_hz == d.lpf1_static_hz && a.lpf1_dyn_min_hz == d.lpf1_dyn_min_hz && a.lpf1_dyn_max_hz == d.lpf1_dyn_max_hz && a.lpf1_dyn_expo == d.lpf1_dyn_expo && a.lpf2_type == d.lpf2_type && a.lpf2_hz == d.lpf2_hz &&
            a.notch1_hz == d.notch1_hz && a.notch1_cutoff == d.notch1_cutoff && a.notch2_hz == d.notch2_hz && a.notch2_cutoff == d.notch2_cutoff, "gyro low-pass and notch defaults");
      CHECK(a.dyn.count == d.dyn.count && fabsf(a.dyn.q - d.dyn.q) < 1e-6f && a.dyn.min_hz == d.dyn.min_hz && a.dyn.max_hz == d.dyn.max_hz && a.rpm.harmonics == d.rpm.harmonics && a.rpm.min_hz == d.rpm.min_hz && fabsf(a.rpm.q - d.rpm.q) < 1e-6f &&
            a.rpm.fade_range_hz == d.rpm.fade_range_hz && a.rpm.lpf_hz == d.rpm.lpf_hz && a.rpm.weights[0] == d.rpm.weights[0] && a.rpm.weights[1] == d.rpm.weights[1] && a.rpm.weights[2] == d.rpm.weights[2], "dynamic notch and RPM filter defaults (q %.2f / %.2f)", a.dyn.q, a.rpm.q); }
    { const fc::FeedforwardConfig &a = c.ff, &d = def.ff;
      CHECK(a.transition == d.transition && a.averaging == d.averaging && a.smooth_factor == d.smooth_factor && a.jitter_factor == d.jitter_factor && a.boost == d.boost && a.max_rate_limit == d.max_rate_limit && a.yaw_hold_gain == d.yaw_hold_gain &&
            a.yaw_hold_time == d.yaw_hold_time, "feedforward defaults");
      CHECK(!a.interpolate && a.interpolate == d.interpolate && c.rc_smoothing.enabled == def.rc_smoothing.enabled && c.rc_smoothing.auto_factor == def.rc_smoothing.auto_factor && c.rc_smoothing.auto_factor_throttle == def.rc_smoothing.auto_factor_throttle,
            "RC smoothing defaults; the default receiver is CRSF, which never repeats a frame"); }
    CHECK(c.angle.limit_deg == def.angle.limit_deg && c.angle.strength == def.angle.strength && c.angle.feedforward == def.angle.feedforward && c.angle.ff_smoothing_ms == def.angle.ff_smoothing_ms && c.angle.earth_ref == def.angle.earth_ref, "Angle defaults");
    CHECK(c.stab.limit_deg == def.stab.limit_deg && c.stab.kp[0] == def.stab.kp[0] && c.stab.kp[1] == def.stab.kp[1] && c.stab.kp[2] == def.stab.kp[2] && c.stab.accel_rp_dss == def.stab.accel_rp_dss && c.stab.accel_yaw_dss == def.stab.accel_yaw_dss &&
          fabsf(c.stab.input_tc - def.stab.input_tc) < 1e-6f && c.stab.rate_ff == def.stab.rate_ff, "Stabilize defaults (ArduPilot's)");
    CHECK(fabsf(c.fs.guard_s - def.fs.guard_s) < 1e-6f && fabsf(c.fs.fallback_throttle_scale - def.fs.fallback_throttle_scale) < 1e-6f, "failsafe defaults");
    { S_pid sp; defaults(sp); bool same = true; for (int x = 0; x < 3; x++) same &= sp.p[x] == fc::kPidDefaultP[x] && sp.i[x] == fc::kPidDefaultI[x] && sp.d[x] == fc::kPidDefaultD[x] && sp.d_max[x] == fc::kPidDefaultDMax[x] && sp.f[x] == fc::kPidDefaultF[x];
      S_filters sf; defaults(sf);
      CHECK(same && sf.gyro_lpf1_dyn_min_hz == fc::kGyroLpf1DynMinDefault && sf.gyro_lpf1_dyn_max_hz == fc::kGyroLpf1DynMaxDefault && sf.gyro_lpf2_hz == fc::kGyroLpf2Default && sf.dterm_lpf1_dyn_min_hz == fc::kDtermLpf1DynMinDefault &&
            sf.dterm_lpf1_dyn_max_hz == fc::kDtermLpf1DynMaxDefault && sf.dterm_lpf2_hz == fc::kDtermLpf2Default, "the sliders at 100 give exactly the stored defaults (one set of numbers, checked)"); }

    // ---- every setting lands in its own place, in the right unit
    s.general.fs_guard_ms = 750; s.rates.throttle_mid = 30; s.rates.expo[1] = 25; s.sensors.mag_declination = -35; s.receiver.deadband = 5; s.motors.layout = LAYOUT_HEX_X; s.general.airmode_start_throttle = 40; s.rates.angle_limit = 55;
    s.receiver.protocol = RX_PROTOCOL_SBUS; s.receiver.rc_smoothing = 0; s.receiver.rc_smoothing_factor = 41; s.receiver.rc_smoothing_throttle = 42;
    for (int a = 0; a < 3; a++) { s.rates.center_sens[a] = (uint16_t)(111 + a); s.rates.max_rate[a] = (uint16_t)(555 + a); }
    s.rates.throttle_expo = 35; s.rates.throttle_limit = 85; s.general.fs_fallback_throttle = 66;
    s.receiver.ch_roll = 1; s.receiver.ch_pitch = 0; s.receiver.ch_throttle = 3; s.receiver.ch_yaw = 2; s.receiver.ch_aux0 = 6;
    { S_pid& p = s.pid; p.slider_mode = SLIDER_MODE_OFF; uint8_t v = 101;
      for (int a = 0; a < 3; a++) { p.p[a] = v++; p.i[a] = v++; p.d[a] = v++; p.d_max[a] = v++; p.f[a] = (uint16_t)(300 + a); p.stab_p[a] = 2.0f + a; }
      p.iterm_relax = ITERM_RELAX_RPY; p.iterm_relax_type = ITERM_RELAX_TYPE_GYRO; p.iterm_relax_cutoff = 21; p.iterm_windup = 61; p.anti_gravity = 0; p.anti_gravity_gain = 71; p.anti_gravity_p_gain = 72; p.anti_gravity_cutoff = 9;
      p.d_max_gain = 37; p.d_max_advance = 38; p.ff_transition = 11; p.ff_averaging = FF_AVERAGING_3_POINT; p.ff_smooth_factor = 12; p.ff_jitter_factor = 13; p.ff_boost = 14; p.ff_max_rate_limit = 16; p.ff_yaw_hold_gain = 17; p.ff_yaw_hold_time = 18;
      p.tpa_mode = TPA_MODE_PD; p.tpa_rate = 40; p.tpa_breakpoint = 45; p.tpa_low_rate = 23; p.tpa_low_breakpoint = 8; p.tpa_low_always = 1; p.throttle_boost = 9; p.throttle_boost_cutoff = 22;
      p.angle_strength = 77; p.angle_feedforward = 78; p.angle_ff_smoothing = 79; p.angle_earth_ref = 60; p.stab_accel_rp = 900; p.stab_accel_yaw = 200; p.stab_input_tc = 25; p.stab_rate_ff = 0; }
    { S_filters& f = s.filters; f.slider_gyro = 0; f.slider_dterm = 0; f.gyro_lpf1_type = LPF_TYPE_PT2; f.gyro_lpf1_static_hz = 201; f.gyro_lpf1_dyn_min_hz = 202; f.gyro_lpf1_dyn_max_hz = 403; f.gyro_lpf1_dyn_expo = 4; f.gyro_lpf2_type = LPF_TYPE_BIQUAD; f.gyro_lpf2_hz = 405;
      f.gyro_notch1_hz = 306; f.gyro_notch1_cutoff = 207; f.gyro_notch2_hz = 308; f.gyro_notch2_cutoff = 209; f.dyn_notch_count = 5; f.dyn_notch_q = 250; f.dyn_notch_min_hz = 110; f.dyn_notch_max_hz = 510;
      f.rpm_harmonics = 2; f.rpm_min_hz = 90; f.rpm_q = 700; f.rpm_fade_range_hz = 60; f.rpm_lpf_hz = 170; f.rpm_weights[0] = 91; f.rpm_weights[1] = 92; f.rpm_weights[2] = 93;
      f.dterm_lpf1_type = LPF_TYPE_PT3; f.dterm_lpf1_static_hz = 81; f.dterm_lpf1_dyn_min_hz = 82; f.dterm_lpf1_dyn_max_hz = 163; f.dterm_lpf1_dyn_expo = 6; f.dterm_lpf2_type = LPF_TYPE_PT2; f.dterm_lpf2_hz = 164; f.dterm_notch_hz = 265; f.dterm_notch_cutoff = 166; f.yaw_lowpass_hz = 67; }
    apply_core(s, c, rc);
    CHECK(fabsf(c.fs.guard_s - 0.75f) < 1e-6f && fabsf(c.rc.throttle_mid - 0.3f) < 1e-6f && fabsf(c.rates.expo[1] - 0.25f) < 1e-6f && fabsf(c.declination_deg + 3.5f) < 1e-5f && fabsf(rc.cfg.deadband - 0.05f) < 1e-6f &&
          fabsf(c.airmode_threshold - 0.4f) < 1e-6f && c.angle.limit_deg == 55 && c.stab.limit_deg == 55, "unit conversions (ms, %%, 0.1 deg); one tilt limit for Angle and Stabilize");
    CHECK(c.ff.interpolate && !c.rc_smoothing.enabled && c.rc_smoothing.auto_factor == 41 && c.rc_smoothing.auto_factor_throttle == 42, "receiver: RC smoothing, and a link that may repeat frames (SBUS) gets the duplicate handling");
    { bool r = true; for (int a = 0; a < 3; a++) r &= c.rates.center_sens_dps[a] == 111 + a && c.rates.max_rate_dps[a] == 555 + a;
      CHECK(r && fabsf(c.rc.throttle_expo - 0.35f) < 1e-6f && fabsf(c.rc.throttle_limit - 0.85f) < 1e-6f && fabsf(c.fs.fallback_throttle_scale - 0.66f) < 1e-6f, "rates per axis, throttle expo and limit, the failsafe's share of the hover thrust (66 %%)");
      CHECK(c.rc.ch_roll == 1 && c.rc.ch_pitch == 0 && c.rc.ch_throttle == 3 && c.rc.ch_yaw == 2 && c.rc.ch_aux0 == 6 && rc.cfg.ch_roll == 1 && rc.cfg.ch_yaw == 2 && rc.cfg.ch_aux0 == 6 && fabsf(rc.cfg.throttle_limit - 0.85f) < 1e-6f,
            "the channel map (here: pitch, roll, yaw, throttle, switches from channel 7) - in the configuration and in the interpreter that reads the sticks"); }
    { const fc::PidConfig& a = c.pid; bool g = true; uint8_t v = 101;
      for (int x = 0; x < 3; x++) { g &= a.p[x] == v && a.i[x] == v + 1 && a.d[x] == v + 2 && a.d_max[x] == v + 3 && a.f[x] == 300 + x && c.stab.kp[x] == 2.0f + x; v = (uint8_t)(v + 4); }
      CHECK(g, "gains per axis: P, I, D, D max, F and the Stabilize P");
      CHECK(a.iterm_relax == fc::RELAX_RPY && a.iterm_relax_type == fc::RELAX_GYRO && a.iterm_relax_cutoff == 21 && a.iterm_windup == 61 && !a.anti_gravity && a.anti_gravity_gain == 71 && a.anti_gravity_p_gain == 72 && a.anti_gravity_cutoff_hz == 9 &&
            a.d_max_gain == 37 && a.d_max_advance == 38, "I-term relax, anti-gravity, D max");
      CHECK(a.tpa_mode == fc::TPA_PD && a.tpa_rate == 40 && fabsf(a.tpa_breakpoint - 0.45f) < 1e-6f && a.tpa_low_rate == 23 && fabsf(a.tpa_low_breakpoint - 0.08f) < 1e-6f && a.tpa_low_always && c.throttle_boost == 9 && c.throttle_boost_cutoff == 22, "TPA (breakpoints in %% of throttle), throttle boost");
      CHECK(c.ff.transition == 11 && c.ff.averaging == 2 && c.ff.smooth_factor == 12 && c.ff.jitter_factor == 13 && c.ff.boost == 14 && c.ff.max_rate_limit == 16 && c.ff.yaw_hold_gain == 17 && c.ff.yaw_hold_time == 18, "feedforward");
      CHECK(c.angle.strength == 77 && c.angle.feedforward == 78 && c.angle.ff_smoothing_ms == 79 && c.angle.earth_ref == 60 && c.stab.accel_rp_dss == 900 && c.stab.accel_yaw_dss == 200 && fabsf(c.stab.input_tc - 0.25f) < 1e-6f && !c.stab.rate_ff, "Angle and Stabilize");
      CHECK(a.dterm_lpf1_type == fc::LPF_PT3 && a.dterm_lpf1_static_hz == 81 && a.dterm_lpf1_dyn_min_hz == 82 && a.dterm_lpf1_dyn_max_hz == 163 && a.dterm_lpf1_dyn_expo == 6 && a.dterm_lpf2_type == fc::LPF_PT2 && a.dterm_lpf2_hz == 164 &&
            a.dterm_notch_hz == 265 && a.dterm_notch_cutoff == 166 && a.yaw_lowpass_hz == 67, "D-term filters and yaw low-pass"); }
    { const fc::GyroFilterConfig& a = c.gyro;
      CHECK(a.lpf1_type == fc::LPF_PT2 && a.lpf1_static_hz == 201 && a.lpf1_dyn_min_hz == 202 && a.lpf1_dyn_max_hz == 403 && a.lpf1_dyn_expo == 4 && a.lpf2_type == fc::LPF_BIQUAD && a.lpf2_hz == 405 && a.notch1_hz == 306 && a.notch1_cutoff == 207 &&
            a.notch2_hz == 308 && a.notch2_cutoff == 209, "gyro low-passes and notches");
      CHECK(a.dyn.count == 5 && fabsf(a.dyn.q - 2.5f) < 1e-6f && a.dyn.min_hz == 110 && a.dyn.max_hz == 510 && a.rpm.harmonics == 2 && a.rpm.min_hz == 90 && fabsf(a.rpm.q - 7.0f) < 1e-6f && a.rpm.fade_range_hz == 60 && a.rpm.lpf_hz == 170 &&
            a.rpm.weights[0] == 91 && a.rpm.weights[1] == 92 && a.rpm.weights[2] == 93, "dynamic notch and RPM filter (Q is stored x 100)"); }
    { fc::CoreConfig c2; fc::RcInterpreter rc2; Settings s2 = s; s2.pid.tpa_rate = 12; s2.filters.yaw_lowpass_hz = 33; apply_pid(s2, c2);
      CHECK(c2.pid.tpa_rate == 12 && c2.pid.yaw_lowpass_hz == def.pid.yaw_lowpass_hz, "the PID section alone does not touch the filters"); apply_filters(s2, c2); CHECK(c2.pid.yaw_lowpass_hz == 33 && c2.pid.tpa_rate == 12, "and the other way round"); }
    { batt::Config bd, b; apply_battery(s, b);
      CHECK(b.cells == bd.cells && fabsf(b.cell_max_v - bd.cell_max_v) < 1e-6f && fabsf(b.cell_warning_v - bd.cell_warning_v) < 1e-6f && fabsf(b.cell_min_v - bd.cell_min_v) < 1e-6f &&
            fabsf(b.warning_delay_s - bd.warning_delay_s) < 1e-6f && fabsf(b.critical_delay_s - bd.critical_delay_s) < 1e-6f, "default battery settings = the monitor's defaults");
      s.battery.cells = 6; s.battery.cell_max_mv = 4350; s.battery.cell_warning_mv = 3600; s.battery.cell_min_mv = 3400; s.battery.warning_delay = 12; s.battery.critical_delay = 35; s.battery.divider = 2150;
      apply_battery(s, b);
      CHECK(b.cells == 6 && fabsf(b.cell_max_v - 4.35f) < 1e-6f && fabsf(b.cell_warning_v - 3.6f) < 1e-6f && fabsf(b.cell_min_v - 3.4f) < 1e-6f && fabsf(b.warning_delay_s - 1.2f) < 1e-6f &&
            fabsf(b.critical_delay_s - 3.5f) < 1e-6f && fabsf(battery_divider(s) - 21.5f) < 1e-5f, "battery: mV -> V, 0.1 s -> s, divider x100");
      beep::Beeper bp, mp;
      CHECK(bp.enabled == s.beeper.conditions, "a beeper that never saw the settings behaves like the default settings (0x%x vs 0x%x)", (unsigned)bp.enabled, (unsigned)s.beeper.conditions);
      apply_beeper(s, bp, mp);
      CHECK(bp.enabled == s.beeper.conditions && !((bp.enabled >> beep::ON_USB) & 1u) && ((bp.enabled >> beep::RX_LOST) & 1u) && bp.slot_ms == 0, "buzzer defaults: every sound on, quiet on the bench, patterns as written");
      // by hand: GYRO_CALIBRATED 0, RX_LOST 1, DISARMING 3, BAT_CRIT_LOW 7, BAT_LOW 8, RX_SET 9, ACC_CALIBRATION 10, ACC_CALIBRATION_FAIL 11, READY_BEEP 12, ARM_REFUSED 15, BLACKBOX_ERASE 16
      CHECK(mp.enabled == 0x59F8Bu && beeper_motor_default_matches() && mp.slot_ms == 450, "motor defaults: what a stopped motor can play, without the power-on chirp; one 450 ms tone per beep (0x%x)", (unsigned)mp.enabled);
      s.beeper.conditions = 1u << beep::BAT_LOW; s.beeper.motor_conditions = 0xFFFFFFFFu; apply_beeper(s, bp, mp);
      CHECK(bp.enabled == ((1u << beep::BAT_LOW) | (1u << beep::MISSION_WARN)), "buzzer conditions reach the buzzer (the mission's take-off warning always on)");
      CHECK(!((mp.enabled >> beep::ARMING) & 1u) && !((mp.enabled >> beep::ARMED) & 1u) && !((mp.enabled >> beep::RX_LOST_LANDING) & 1u) && !((mp.enabled >> beep::ARMING_GPS_FIX) & 1u) && !((mp.enabled >> beep::ARMING_GPS_NO_FIX) & 1u) &&
            !((mp.enabled >> beep::ON_USB) & 1u) && ((mp.enabled >> beep::SYSTEM_INIT) & 1u) && ((mp.enabled >> beep::RX_LOST) & 1u), "motors: conditions of an armed craft are removed whatever the settings say");
      s.beeper.motor_conditions = 1u << beep::RX_LOST; apply_beeper(s, bp, mp); CHECK(mp.enabled == ((1u << beep::RX_LOST) | (1u << beep::MISSION_WARN)), "motor conditions reach the motor sounder (the mission's take-off warning always on)"); }
    { bb::Config bc; apply_blackbox(s, bc);
      CHECK(bc.device == bb::DEV_NONE && bc.trigger == bb::TRIG_ARMED && bc.rate_hz == 250 && bc.sd_mode == SD_MODE_SPI && bc.sd_mhz == 20, "blackbox defaults: off, while armed, 250 records/s; SD card on SPI at 20 MHz");
      s.blackbox.device = BB_DEVICE_SDCARD; s.blackbox.sd_mode = SD_MODE_SDMMC_1BIT; s.blackbox.sd_mhz = 8; s.blackbox.rate_hz = 1000; apply_blackbox(s, bc);
      CHECK(bc.device == bb::DEV_SDCARD && bc.sd_mode == 1 && bc.sd_mhz == 8 && bc.rate_hz == 1000, "blackbox settings reach the service, the SD wiring included"); }
    CHECK(c.mixer.layout == fc::HEX_X && rc.cfg.throttle_mid == c.rc.throttle_mid, "layout + the RC interpreter receives the same curve");
    s.modes.mode[0] = fc::M_ARM; s.modes.aux[0] = 0; s.modes.lo[0] = 1700; s.modes.hi[0] = 2100; s.modes.mode[1] = fc::M_ALTHOLD; s.modes.aux[1] = 3; s.modes.lo[1] = 1300; s.modes.hi[1] = 1700;
    s.modes.mode[2] = fc::M_ANGLE; s.modes.aux[2] = 1; s.modes.lo[2] = 1700; s.modes.hi[2] = 1300;                       // lo >= hi: invalid, dropped
    apply_modes(s, rc); uint16_t aux[8] = {2000, 1000, 1000, 1500, 1000, 1000, 1000, 1000}; const uint32_t m = rc.modes.resolve(aux);
    CHECK((m & (1u << fc::M_ARM)) && (m & (1u << fc::M_ALTHOLD)) && rc.modes.n == 2, "mode ranges from the settings (255 = unused, inverted range dropped): %d entries", rc.modes.n);
    motor::MotorConfig mc; apply_motors(s, mc);
    CHECK(mc.count == 6 && mc.protocol == motor::PROTO_DSHOT300 && fabsf(mc.idle - 0.055f) < 1e-6f && fabsf(mc.output_limit - 1.0f) < 1e-6f, "motors: hex => 6 motors, idle 5.5 %%");
    s.motors.reversed_mask = 0x0A; s.motors.order[0] = 3; s.motors.order[3] = 0; s.motors.dyn_idle_min_rpm = 120; s.motors.bidirectional = 1; s.motors.protocol = motor::PROTO_ONESHOT125; apply_motors(s, mc);
    CHECK(!mc.reversed[0] && mc.reversed[1] && !mc.reversed[2] && mc.reversed[3] && mc.order[0] == 3 && mc.order[3] == 0 && mc.dyn.min_rpm_100 == 120 && mc.bidirectional && mc.protocol == motor::PROTO_ONESHOT125, "reversed mask, reorder, dynamic idle, protocol");
    { wbp::Settings t = s; t.motors.idle_x10 = 55; t.motors.protocol = motor::PROTO_ONESHOT125; motor::MotorConfig a; apply_motors(t, a);
      t.motors.protocol = motor::PROTO_DSHOT600; motor::MotorConfig b; apply_motors(t, b);
      CHECK(a.idle == 0.0f && fabsf(b.idle - 0.055f) < 1e-6f, "motor idle only with DShot: Oneshot idles at its 'armed' pulse (Betaflight's min_throttle), DShot at 5.5 %% (%.3f / %.3f)", a.idle, b.idle); }
    gps::Config gc; s.gps.baud = 115200; s.gps.sbas = gps::SBAS_EGNOS; s.gps.use_galileo = 1; s.gps.flight_model = gps::MODEL_AIRBORNE_1G; s.gps.min_sats = 10; apply_gps(s, gc);
    CHECK(gc.baud == 115200 && gc.sbas == gps::SBAS_EGNOS && gc.use_galileo && gc.flight_model == gps::MODEL_AIRBORNE_1G && gc.min_sats == 10 && gc.provider == gps::PROV_UBLOX && gc.update_rate_hz == 10, "gps config");
    MagCal mcal; mcal.valid = true; mcal.radius = 0.47f; mcal.offset[0] = 0.1f; mcal.offset[2] = -0.2f; mcal.m[0][1] = 0.03f; mcal.m[2][2] = 1.1f; S_calibration cal; defaults(cal); from_magcal(mcal, cal);
    MagCal back; to_magcal(cal, back); CHECK(back.valid && back.radius == 0.47f && back.offset[2] == -0.2f && back.m[0][1] == 0.03f && back.m[2][2] == 1.1f && back.m[1][1] == 1.0f, "mag calibration survives the settings round trip");
    // ---- navigation: optical flow, Position Hold, GPS Rescue
    { Settings n; defaults(n); fc::CoreConfig d0, c2; fc::RcInterpreter rc2; apply_core(n, c2, rc2);
      CHECK(c2.flow.yaw_deg == d0.flow.yaw_deg && c2.flow.flip == d0.flow.flip && fabsf(c2.flow.scale - d0.flow.scale) < 1e-6f && c2.flow.quality_min == d0.flow.quality_min && fabsf(c2.flow.max_height_m - d0.flow.max_height_m) < 1e-6f && c2.use_tilt_aid == d0.use_tilt_aid && c2.use_tilt_aid,
            "default settings reproduce the core's flow defaults (quality %u, up to %.1f m), tilt aid on", c2.flow.quality_min, c2.flow.max_height_m);
      CHECK(c2.poshold.angle_max_deg == d0.poshold.angle_max_deg && c2.poshold.brake_rate_dps == d0.poshold.brake_rate_dps && c2.poshold.brake_angle_max_deg == d0.poshold.brake_angle_max_deg && c2.pos.angle_max_deg == d0.pos.angle_max_deg &&
            fabsf(c2.pos.pos_p - d0.pos.pos_p) < 1e-6f && fabsf(c2.pos.vel_p - d0.pos.vel_p) < 1e-6f && fabsf(c2.pos.vel_i - d0.pos.vel_i) < 1e-6f && fabsf(c2.pos.vel_d - d0.pos.vel_d) < 1e-6f, "Position Hold defaults = the values the closed-loop tests fly with (ArduCopter's)");
      CHECK(c2.rescue.alt_mode == d0.rescue.alt_mode && c2.rescue.initial_climb_m == d0.rescue.initial_climb_m && c2.rescue.return_alt_m == d0.rescue.return_alt_m && fabsf(c2.rescue.ascend_mps - d0.rescue.ascend_mps) < 1e-6f && fabsf(c2.rescue.descend_mps - d0.rescue.descend_mps) < 1e-6f &&
            fabsf(c2.rescue.ground_speed_mps - d0.rescue.ground_speed_mps) < 1e-6f && c2.rescue.min_start_dist_m == d0.rescue.min_start_dist_m && c2.rescue.landing_alt_m == d0.rescue.landing_alt_m && c2.rescue.min_sats == d0.rescue.min_sats &&
            c2.rescue.alt_mode == fc::RESCUE_ALT_CONE && c2.rescue.cone_deg == d0.rescue.cone_deg && c2.rescue.descend_alt_m == d0.rescue.descend_alt_m && c2.rescue.loiter_s == d0.rescue.loiter_s && fabsf(c2.rescue.land_speed_mps - d0.rescue.land_speed_mps) < 1e-6f && c2.rescue.mode == fc::RESCUE_MODE_RTL,
            "GPS Rescue defaults (0.36: the PX4 cone, 10 m over home, 5 s wait, 0.5 m/s, straight home)");
      CHECK(c2.fs.procedure == fc::FSP_LAND && c2.fs.procedure == d0.fs.procedure && !c2.rescue_configured && !c2.rescue_allow_no_fix, "the failsafe lands by default; no rescue is set up, so arming does not ask for a GPS fix");
      S_nav& v = n.nav; v.flow_align = FLOW_ALIGN_CW270; v.flow_flip = 1; v.flow_scale = 135; v.flow_quality_min = 41; v.flow_max_height = 250; v.tilt_aid = 0; v.ph_angle_max = 22; v.ph_brake_rate = 6; v.ph_brake_angle = 17;
      v.pos_p = 13; v.vel_p = 31; v.vel_i = 7; v.vel_d = 44; v.fs_procedure = FS_PROCEDURE_RESCUE; v.rescue_alt_mode = RESCUE_ALT_FIXED; v.rescue_initial_climb = 12; v.rescue_return_alt = 45; v.rescue_ascend_rate = 33; v.rescue_descend_rate = 21;
      v.rescue_ground_speed = 95; v.rescue_descent_dist = 35; v.rescue_min_start_dist = 9; v.rescue_landing_alt = 6; v.rescue_min_sats = 8; v.rescue_allow_no_fix = 1;
      v.rescue_cone = 30; v.rescue_descend_alt = 15; v.rescue_loiter = 12; v.rescue_land_speed = 7; v.rescue_mode = RESCUE_MODE_SMART_RTL;
      apply_core(n, c2, rc2);
      CHECK(c2.flow.yaw_deg == 270.0f && c2.flow.flip && fabsf(c2.flow.scale - 1.35f) < 1e-6f && c2.flow.quality_min == 41 && fabsf(c2.flow.max_height_m - 2.5f) < 1e-6f && !c2.use_tilt_aid, "flow: CW270 = 270 deg, flip, scale 135 %%, quality 41, 2.5 m; tilt aid off");
      CHECK(c2.poshold.angle_max_deg == 22 && c2.pos.angle_max_deg == 22 && c2.poshold.brake_rate_dps == 6 && c2.poshold.brake_angle_max_deg == 17 && fabsf(c2.pos.pos_p - 1.3f) < 1e-6f && fabsf(c2.pos.vel_p - 3.1f) < 1e-6f && fabsf(c2.pos.vel_i - 0.7f) < 1e-6f && fabsf(c2.pos.vel_d - 0.44f) < 1e-6f,
            "Position Hold: one lean limit for the mode and its controller, brake rate / angle, gains (x0.1, D x0.01)");
      CHECK(c2.rescue.alt_mode == fc::RESCUE_ALT_FIXED && c2.rescue.initial_climb_m == 12 && c2.rescue.return_alt_m == 45 && fabsf(c2.rescue.ascend_mps - 3.3f) < 1e-6f && fabsf(c2.rescue.descend_mps - 2.1f) < 1e-6f && fabsf(c2.rescue.ground_speed_mps - 9.5f) < 1e-6f &&
            c2.rescue.min_start_dist_m == 9 && c2.rescue.landing_alt_m == 6 && c2.rescue.min_sats == 8 && c2.rescue.cone_deg == 30.0f && c2.rescue.descend_alt_m == 15.0f && c2.rescue.loiter_s == 12.0f &&
            fabsf(c2.rescue.land_speed_mps - 0.7f) < 1e-6f && c2.rescue.mode == fc::RESCUE_MODE_SMART_RTL, "GPS Rescue: heights and distances in metres, rates x0.1 m/s, cone in degrees, SmartRTL");
      v.rescue_loiter = 255; apply_nav(n, c2); CHECK(c2.rescue.loiter_s < 0.0f, "rescue_loiter 255: it waits over home for good (never lands by itself)");
      CHECK(c2.fs.procedure == fc::FSP_RESCUE && c2.rescue_configured && c2.rescue_allow_no_fix, "failsafe procedure RESCUE counts as 'a rescue is set up'");
      for (int a = 0; a < 4; a++) { v.flow_align = (uint8_t)a; apply_nav(n, c2); CHECK(c2.flow.yaw_deg == 90.0f * a, "flow_align %d = %.0f deg", a, c2.flow.yaw_deg); }
      // RESCUE on a switch
      v.fs_procedure = FS_PROCEDURE_LAND; apply_nav(n, c2); CHECK(!c2.rescue_configured && c2.fs.procedure == fc::FSP_LAND, "back to LAND: no rescue set up");
      n.modes.mode[5] = MODE_RESCUE; n.modes.aux[5] = 2; n.modes.lo[5] = 1700; n.modes.hi[5] = 2100; apply_nav(n, c2); CHECK(c2.rescue_configured, "mode RESCUE on a switch: a rescue is set up");
      n.modes.hi[5] = 1700; apply_nav(n, c2); CHECK(!c2.rescue_configured, "... not with an empty range (the switch can never be on)");
      n.modes.hi[5] = 2100; n.modes.aux[5] = 200; apply_nav(n, c2); CHECK(!c2.rescue_configured, "... nor on a channel that does not exist"); }
}

// The sliders: what the app writes into the gain boxes of an axis under the sliders does not matter - the firmware computes them.
static void t_rules() {
    printf("[rules] sliders -> gains and cutoffs\n");
    { S_pid p; defaults(p); p.slider_d = 150; p.p[0] = 9; p.d[2] = 77; apply_rules(p);
      CHECK(p.p[0] == 45 && p.d[0] == 45 && p.d[1] == 51 && p.d_max[0] == 60 && p.d_max[1] == 69 && p.d[2] == 0 && p.i[0] == 80 && p.f[0] == 120, "Damping 1.5: D 30 / 34 -> 45 / 51, D max 40 / 46 -> 60 / 69; the rest as it was (yaw D stays 0: 0 x 1.5)"); }
    { S_pid p; defaults(p); p.slider_pi = 120; apply_rules(p); CHECK(p.p[0] == 54 && p.p[1] == 56 && p.p[2] == 54 && p.i[0] == 96 && p.i[1] == 100 && p.i[2] == 96 && p.d[0] == 30 && p.f[0] == 120, "Tracking 1.2: P and I (%u %u %u / %u %u %u)", p.p[0], p.p[1], p.p[2], p.i[0], p.i[1], p.i[2]); }
    { S_pid p; defaults(p); p.slider_ff = 50; apply_rules(p); CHECK(p.f[0] == 60 && p.f[1] == 62 && p.f[2] == 60 && p.p[0] == 45, "Stick response 0.5: feedforward only (62 = 125 / 2 cut, as Betaflight does)"); }
    { S_pid p; defaults(p); p.slider_i = 50; apply_rules(p); CHECK(p.i[0] == 40 && p.i[1] == 42 && p.i[2] == 40 && p.p[1] == 47, "Drift - wobble 0.5: I only"); }
    { S_pid p; defaults(p); p.slider_d_max = 0; apply_rules(p); CHECK(p.d_max[0] == 30 && p.d_max[1] == 34 && p.d[0] == 30, "Dynamic damping 0: D max = D (no boost)");
      p.slider_d_max = 200; apply_rules(p); CHECK(p.d_max[0] == 50 && p.d_max[1] == 57, "Dynamic damping 2: twice as far above D (%u %u; 57 not 58: the product is cut, not rounded, exactly as Betaflight does)", p.d_max[0], p.d_max[1]); }
    { S_pid p; defaults(p); p.slider_pitch_d = 150; p.slider_pitch_pi = 80; apply_rules(p);
      CHECK(p.d[1] == 51 && p.d_max[1] == 69 && p.d[0] == 30 && p.p[1] == 37 && p.i[1] == 67 && p.f[1] == 100 && p.p[0] == 45 && p.f[0] == 120 && p.p[2] == 45, "Pitch damping / pitch tracking act on pitch alone (P %u I %u F %u)", p.p[1], p.i[1], p.f[1]); }
    { S_pid p; defaults(p); p.slider_master = 200; p.slider_pi = 200; p.slider_i = 200; apply_rules(p);
      CHECK(p.p[0] == 180 && p.i[0] == 250 && p.d[0] == 60 && p.f[0] == 240, "Master multiplier scales everything; a gain stops at 250 (%u %u %u %u)", p.p[0], p.i[0], p.d[0], p.f[0]); }
    { S_pid p; defaults(p); p.slider_mode = SLIDER_MODE_RP; p.slider_master = 50; p.p[2] = 99; p.i[2] = 98; p.f[2] = 97; apply_rules(p);
      CHECK(p.p[0] == 22 && p.p[1] == 23 && p.p[2] == 99 && p.i[2] == 98 && p.f[2] == 97, "sliders for roll + pitch: yaw keeps what was typed");
      p.slider_mode = SLIDER_MODE_OFF; p.p[0] = 11; p.d_max[1] = 12; apply_rules(p); CHECK(p.p[0] == 11 && p.d_max[1] == 12 && p.p[1] == 23, "sliders off: every box is the user's"); }
    { S_filters f; defaults(f); f.slider_gyro_mult = 50; f.slider_dterm_mult = 150; apply_rules(f);
      CHECK(f.gyro_lpf1_dyn_min_hz == 125 && f.gyro_lpf1_dyn_max_hz == 250 && f.gyro_lpf1_static_hz == 125 && f.gyro_lpf2_hz == 250, "gyro filter slider 0.5: half the cutoffs = more filtering");
      CHECK(f.dterm_lpf1_dyn_min_hz == 112 && f.dterm_lpf1_dyn_max_hz == 225 && f.dterm_lpf1_static_hz == 112 && f.dterm_lpf2_hz == 225, "D-term filter slider 1.5 (%u %u %u)", f.dterm_lpf1_dyn_min_hz, f.dterm_lpf1_dyn_max_hz, f.dterm_lpf2_hz);
      f.gyro_lpf2_hz = 0; f.gyro_lpf1_dyn_min_hz = 0; f.slider_gyro_mult = 100; apply_rules(f);
      CHECK(f.gyro_lpf2_hz == 0 && f.gyro_lpf1_dyn_min_hz == 0 && f.gyro_lpf1_dyn_max_hz == 250 && f.gyro_lpf1_static_hz == 250, "a filter that is switched off (0 Hz) stays off; dynamic off -> the static cutoff is the one that follows the slider");
      f.slider_gyro = 0; f.slider_dterm = 0; f.gyro_lpf1_static_hz = 333; f.dterm_lpf2_hz = 99; f.slider_gyro_mult = 20; f.slider_dterm_mult = 20; apply_rules(f);
      CHECK(f.gyro_lpf1_static_hz == 333 && f.dterm_lpf2_hz == 99, "filter sliders off: the typed cutoffs stand");
      f.slider_gyro = 1; apply_rules(f); CHECK(f.gyro_lpf1_static_hz == 50 && f.dterm_lpf2_hz == 99, "each slider has its own switch: gyro on, D-term off (%u, %u)", f.gyro_lpf1_static_hz, f.dterm_lpf2_hz);
      f.slider_gyro = 0; f.gyro_lpf1_static_hz = 333; f.slider_dterm = 1; apply_rules(f); CHECK(f.gyro_lpf1_static_hz == 333 && f.dterm_lpf2_hz == 30, "and D-term on, gyro off (%u, %u)", f.gyro_lpf1_static_hz, f.dterm_lpf2_hz);
      S_filters g; defaults(g); g.slider_gyro_mult = 200; g.slider_dterm_mult = 200; g.yaw_lowpass_hz = 77; g.gyro_notch1_hz = 300; apply_rules(g);
      CHECK(g.gyro_lpf1_dyn_max_hz == 1000 && g.gyro_lpf2_hz == 1000 && g.yaw_lowpass_hz == 77 && g.gyro_notch1_hz == 300 && g.dyn_notch_count == 3, "a cutoff never passes 1000 Hz; notches and the yaw low-pass are not the sliders' business"); }
    // through the server and through the store: the RAM copy is what flies, and the app reads back what is in effect
    { Rig r; S_pid np = r.s.pid; np.slider_d = 150; np.p[0] = 1; std::vector<uint8_t> p(1 + sizeof(S_pid)); p[0] = SEC_PID; memcpy(p.data() + 1, &np, sizeof(np));
      r.send(frame(MSG_SECTION_SET, 3, p)); auto fr = r.tx.frames(); S_pid got; memcpy(&got, fr[0].second.data() + 3, sizeof(got));
      CHECK(fr[0].first.type == MSG_SECTION && got.d[0] == 45 && got.p[0] == 45 && r.s.pid.d[1] == 51, "SET pid with a slider moved: the reply carries the computed gains");
      S_filters nf = r.s.filters; nf.slider_gyro_mult = 80; p.assign(1 + sizeof(S_filters), 0); p[0] = SEC_FILTERS; memcpy(p.data() + 1, &nf, sizeof(nf)); r.send(frame(MSG_SECTION_SET, 4, p)); fr = r.tx.frames();
      S_filters gf; memcpy(&gf, fr[0].second.data() + 3, sizeof(gf)); CHECK(gf.gyro_lpf1_dyn_min_hz == 200 && gf.gyro_lpf1_dyn_max_hz == 400 && r.s.filters.gyro_lpf2_hz == 400, "SET filters with the slider moved: computed cutoffs"); }
    { MemStorage m; SettingsStore st(m); Settings a; defaults(a); a.pid.slider_pi = 150; a.pid.p[0] = 3; a.filters.slider_dterm_mult = 80; a.filters.dterm_lpf2_hz = 7; st.save(a); Settings b; st.load(b);
      CHECK(b.pid.p[0] == 67 && b.pid.i[1] == 126 && b.filters.dterm_lpf2_hz == 120, "loaded from flash: gains / cutoffs are recomputed from the stored sliders (%u %u %u)", b.pid.p[0], b.pid.i[1], b.filters.dterm_lpf2_hz); }
}

static void t_more() {
    printf("[codec/server] limits\n");
    { uint8_t big[kMaxPayload + 16] = {}, out[kMaxPayload + 64];
      CHECK(encode(out, sizeof(out), 1, 1, big, kMaxPayload) == (size_t)kMaxPayload + kOverhead && encode(out, sizeof(out), 1, 1, big, kMaxPayload + 1) == 0, "encode refuses a payload longer than kMaxPayload");
      CHECK(encode(out, 10, 1, 1, big, 4) == 0, "encode refuses an output buffer that is too small"); }
    { // every stream is due in the same millisecond: one tick may send at most 4 frames (the rest follow in the next ticks)
      Rig r; for (const StreamInfo& si : kStreams) r.send(frame(MSG_STREAM_CFG, 1, {si.id, (uint8_t)si.max_rate, (uint8_t)(si.max_rate >> 8)}));
      r.tx.out.clear(); r.host.ms = 5000; r.srv.tick(5000);
      int n = 0; for (auto& f : r.tx.frames()) if (f.first.type == MSG_STREAM) n++;
      CHECK(n == 4, "one tick sends at most 4 stream frames (%d)", n);
      r.srv.tick(5001); n = 0; for (auto& f : r.tx.frames()) if (f.first.type == MSG_STREAM) n++;
      CHECK(n == 8, "the others follow on the next tick (%d)", n); }
}

static void t_mission_schema() {
    printf("[schema] mission (0.37)\n");
    { const SectionInfo* m = find_section(SEC_MISSION); S_mission d; defaults(d);
      CHECK(m && m->id == 26 && m->size == sizeof(S_mission) && sizeof(S_mission) == 1 + 1 + 32 * (1 + 1 + 2 + 4 + 4 + 2) && sizeof(S_mission) <= kMaxPayload - 8, "section mission: id 26, count + rc_lost + 32 x (cmd, arg, alt, lat, lon, p) = %zu bytes, fits one frame", sizeof(S_mission));
      CHECK(d.count == 0 && d.cmd[0] == MISSION_CMD_NONE && d.rc_lost == MISSION_RC_LOST_CONTINUE, "an empty plan by default; a lost link carries on (ArduPilot FS_OPTIONS continue in AUTO)");
      CHECK(MISSION_CMD_TAKEOFF == 1 && MISSION_CMD_WAYPOINT == 2 && MISSION_CMD_LOITER_TIME == 3 && MISSION_CMD_LAND == 4 && MISSION_CMD_DELAY == 5 && MISSION_CMD_RTL == 6 && MISSION_CMD_SERVO == 7 && MISSION_CMD_SPEED == 8, "mission commands in the order the firmware's MissionCmd uses"); }
    { S_servo s; defaults(s); const SectionInfo* si = find_section(SEC_SERVO);
      CHECK(s.angle_min[3] == 0 && s.angle_max[0] == 180 && s.init_us[2] == 1500 && si->compat == 26 && offsetof(S_servo, angle_min) == 26, "servo: the angles of min_us / max_us (0 / 180 deg) and the pulse before a mission command (1500 us) at the end, the old size 26 known");
      CHECK(SERVO_SOURCE_MISSION == 17 && MODE_MISSION == 14 && BEEPER_COND_MISSION_WARN == 18, "MISSION appended to servo_source and mode, MISSION_WARN to the beeper conditions"); }
    { S_beeper b; defaults(b); CHECK((b.conditions >> BEEPER_COND_MISSION_WARN) & 1u && (b.motor_conditions >> BEEPER_COND_MISSION_WARN) & 1u, "the take-off warning beeps by default on the buzzer and the motors"); }
    { Settings s; defaults(s); beep::Beeper bp, mp; s.beeper.conditions = 0; s.beeper.motor_conditions = 0; apply_beeper(s, bp, mp);
      CHECK(bp.enabled == (1u << beep::MISSION_WARN) && mp.enabled == (1u << beep::MISSION_WARN), "every condition switched off: the take-off warning still sounds (people stand next to the craft)"); }
    { Settings s; defaults(s); fc::CoreConfig c; s.mission.rc_lost = MISSION_RC_LOST_RTL; apply_nav(s, c); const bool rtl = !c.fs.mission_rc_continue;
      s.mission.rc_lost = MISSION_RC_LOST_CONTINUE; apply_nav(s, c); CHECK(rtl && c.fs.mission_rc_continue, "mission.rc_lost: RTL = the failsafe as usual, CONTINUE = it flies on"); }
    { // the plan: lat / lon -> metres from home (the GPS's own offset), heights in dm, MAV meanings
      S_mission m; defaults(m); gps::Location home; home.lat = 473977419; home.lon = 85455938; home.alt_cm = 48820;
      m.count = 4; m.cmd[0] = MISSION_CMD_TAKEOFF; m.alt[0] = 85;
      m.cmd[1] = MISSION_CMD_WAYPOINT; m.lat[1] = 473977419 + 1797; m.lon[1] = 85455938 - 1332; m.alt[1] = 120; m.p[1] = 4;     // ~20 m north, ~10 m west
      m.cmd[2] = MISSION_CMD_SERVO; m.arg[2] = 2; m.p[2] = 1750;
      m.cmd[3] = MISSION_CMD_LOITER_TIME; m.p[3] = 6; m.cmd[4] = MISSION_CMD_WAYPOINT;
      fc::MissionPlan p; mission_plan_from(m, home, p);
      CHECK(p.count == 4 && p.item[0].cmd == fc::MissionCmd::TAKEOFF && fabsf(p.item[0].alt_m - 8.5f) < 1e-4f && p.item[0].has_alt && !p.item[0].has_pos, "TAKEOFF 8.5 m, no position");
      CHECK(p.item[1].has_pos && fabsf(p.item[1].n - 20.0f) < 0.1f && fabsf(p.item[1].e + 10.0f) < 0.1f && fabsf(p.item[1].alt_m - 12.0f) < 1e-4f && p.item[1].p == 4, "WAYPOINT 20.0 m N, 10.0 m W (%.2f / %.2f), 12 m, hold 4 s", p.item[1].n, p.item[1].e);
      CHECK(p.item[2].cmd == fc::MissionCmd::SERVO && p.item[2].arg == 2 && p.item[2].p == 1750 && p.item[3].cmd == fc::MissionCmd::LOITER_TIME && !p.item[3].has_pos && !p.item[3].has_alt, "SERVO 2 -> 1750 us; LOITER_TIME without a point or a height (lat = lon = 0, alt 0)");
      m.count = 40; mission_plan_from(m, home, p); CHECK(p.count == 32, "a count beyond 32 is cut to 32"); }
}

static void t_radio_schema() {
    printf("[schema] transmitter + nRF24 (0.38)\n");
    { const SectionInfo* tr = find_section(SEC_TX_RADIO); const SectionInfo* ts = find_section(SEC_TX_STICKS); const SectionInfo* rc = find_section(SEC_RECEIVER);
      CHECK(tr && ts && tr->id == 27 && ts->id == 28 && tr->role == ROLE_TX && ts->role == ROLE_TX && rc->role == ROLE_RX && find_section(SEC_PID)->role == ROLE_RX, "tx_radio (27) and tx_sticks (28) belong to the transmitter, the rest to the flight controller");
      const StreamInfo* tx = find_stream(STR_TX); CHECK(tx && tx->id == 17 && tx->role == ROLE_TX && find_stream(STR_RC)->role == ROLE_RX, "stream tx (17) is the transmitter's"); }
    { Settings s; defaults(s);
      CHECK(s.tx_radio.rf_channel == 76 && s.tx_radio.rate == NRF_RATE_R250K && s.tx_radio.power == NRF_POWER_MAX && s.tx_radio.pps == NRF_PPS_HZ100 && s.tx_radio.bound == 0, "transmitter radio: RF channel 76, 250 kbps, full power, 100 packets/s, not bound");
      CHECK(s.tx_radio.pin_sck == 7 && s.tx_radio.pin_miso == 6 && s.tx_radio.pin_mosi == 8 && s.tx_radio.pin_cs == 24 && s.tx_radio.pin_ce == 23, "the old project's nRF24 pins on the ESP32-C5");
      CHECK(s.tx_sticks.adc_pin[0] == 4 && s.tx_sticks.adc_pin[1] == 5 && s.tx_sticks.adc_pin[2] == 3 && s.tx_sticks.adc_pin[3] == 2 && s.tx_sticks.cal_mid[1] == 2048 && s.tx_sticks.cal_max[3] == 4095, "joysticks on GPIO 4 5 3 2, a 12-bit ADC centred at 2048");
      CHECK(s.tx_sticks.map[0] == 0 && s.tx_sticks.map[1] == 1 && s.tx_sticks.map[2] == 3 && s.tx_sticks.map[3] == 2, "roll = joy1 X, pitch = joy1 Y, throttle = joy2 Y, yaw = joy2 X");
      CHECK(s.tx_sticks.throttle_mode == TX_THROTTLE_MANUAL && s.tx_sticks.deadband == 3 && s.tx_sticks.throttle_rate == 50, "throttle MANUAL (the safe one) by default");
      CHECK(s.receiver.nrf_channel == 76 && s.receiver.nrf_rate == NRF_RATE_R250K && s.receiver.nrf_pps == NRF_PPS_HZ100 && s.receiver.nrf_bound == 0 && s.pins.nrf_ce == 255 && s.pins.nrf_cs == 255, "flight controller: nRF24 receiver settings, its pins unset");
      CHECK(BIND_STATE_BOUND == 3 && FLY_MODE_MISSION == 6 && find_command(CMD_BIND) && find_command(CMD_STICK_CAL), "bind states, flight modes, commands BIND and STICK_CAL"); }
    { const SectionInfo* rc = find_section(SEC_RECEIVER); const SectionInfo* pn = find_section(SEC_PINS);
      CHECK(rc->compat == 13 && pn->compat == 52, "receiver and pins remember their 0.37 sizes (%u, %u)", rc->compat, pn->compat);
      MemStorage ms; SettingsStore so(ms); S_receiver old; defaults(old); old.protocol = RX_PROTOCOL_SBUS;
      uint8_t buf[64]; buf[0] = 1; buf[1] = 13; buf[2] = 0; memcpy(buf + 3, &old, 13); const uint16_t c = crc16(buf, 16); buf[16] = (uint8_t)c; buf[17] = (uint8_t)(c >> 8); ms.kv["wbs02"] = std::vector<uint8_t>(buf, buf + 18);
      Settings got; so.load(got); CHECK(got.receiver.protocol == RX_PROTOCOL_SBUS && got.receiver.nrf_channel == 76, "a 0.37 receiver record loads, the nRF24 fields get their defaults"); }
    { // the flight controller's server does not serve the transmitter's sections, and the other way round
      Rig r; r.send(frame(MSG_SECTION_GET, 1, {SEC_TX_RADIO})); auto fr = r.tx.frames();
      CHECK(fr.size() == 1 && fr[0].first.type == MSG_ACK && fr[0].second[2] == ERR_BAD_SECTION, "RX: GET tx_radio -> BAD_SECTION");
      std::vector<uint8_t> p(1 + sizeof(S_tx_sticks)); p[0] = SEC_TX_STICKS; r.send(frame(MSG_SECTION_SET, 2, p)); fr = r.tx.frames();
      CHECK(fr.size() == 1 && fr[0].first.type == MSG_ACK && fr[0].second[2] == ERR_BAD_SECTION, "RX: SET tx_sticks -> BAD_SECTION");
      r.send(frame(MSG_STREAM_CFG, 3, {STR_TX, 10, 0})); fr = r.tx.frames(); CHECK(fr.size() == 1 && fr[0].second[2] == ERR_BAD_ARG, "RX: the tx stream cannot be subscribed");
      r.send(frame(MSG_SETTINGS_SAVE, 4)); CHECK(r.mem.kv.count("wbs02") && !r.mem.kv.count("wbs27") && !r.mem.kv.count("wbs28"), "RX: SAVE writes its own sections only"); }
    { Settings s; defaults(s); MemStorage mem; SettingsStore store{mem}; FakeHost host; MemTransport t;
      Server tx{s, &store, host, t, ROLE_TX, 0, 38, 0, "Windburst TX", 0}; tx.settings_loaded();
      auto get = [&](uint8_t id) { t.out.clear(); const auto f = frame(MSG_SECTION_GET, 5, {id}); tx.on_bytes(f.data(), f.size()); return t.frames(); };
      auto a = get(SEC_TX_RADIO); auto b = get(SEC_PID);
      CHECK(a.size() == 1 && a[0].first.type == MSG_SECTION && b.size() == 1 && b[0].first.type == MSG_ACK && b[0].second[2] == ERR_BAD_SECTION, "TX: GET tx_radio served, GET pid -> BAD_SECTION");
      t.out.clear(); const auto sv = frame(MSG_SETTINGS_SAVE, 6); tx.on_bytes(sv.data(), sv.size());
      CHECK(mem.kv.size() == 2 && mem.kv.count("wbs27") && mem.kv.count("wbs28"), "TX: SAVE writes tx_radio + tx_sticks only (%zu records)", mem.kv.size());
      s.tx_sticks.deadband = 9; mem.kv.clear(); Settings back; SettingsStore st2{mem}; st2.set_role(ROLE_TX); store.save(s); st2.load(back);
      CHECK(back.tx_sticks.deadband == 9 && !mem.kv.count("wbs05"), "TX store: its sections round-trip, the flight controller's are not written"); }
}

int main() { t_codec(); t_schema(); t_store(); t_server(); t_streams(); t_apply(); t_rules(); t_more(); t_mission_schema(); t_radio_schema(); printf("\n%d passed, %d failed\n", g_pass, g_fail); return g_fail ? 1 : 0; }
