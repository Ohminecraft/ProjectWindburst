// MSP for the configurators (src/app/msp_config.h) through msp::Bridge: requests as Betaflight Configurator sends them, the
// answers checked byte by byte against Betaflight 4.5's layouts (msp.c), writes against the settings they must change.
// Build: g++ -std=c++17 -O2 test/test_msp_config.cpp -o build/test_msp_config
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <vector>
#include "../src/app/msp_config.h"
#include "../src/proto/settings_apply.h"
#include "../src/hal/acc_gyro/acc_gyro_util.h"
#include "../src/osd/osd.h"

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, ...) do { const bool ok_ = (cond); if (getenv("V")) { printf("    "); printf(__VA_ARGS__); printf("\n"); } if (ok_) g_pass++; else { g_fail++; printf("  FAIL line %d: %s   ", __LINE__, #cond); printf(__VA_ARGS__); printf("\n"); } } while (0)

struct Port : msp::ConfigPort {
    bool is_armed = false, save_ok = true; std::vector<uint8_t> commits; int saves = 0; int arming = -1;
    void arming_disabled(bool on) override { arming = on ? 1 : 0; }
    wbp::T_blackbox bb{}; bool has_bb = true;
    std::vector<uint16_t> motors; int motor_calls = 0; bool motor_ok = true;
    bool motor_test(const uint16_t* us, int n) override { motors.assign(us, us + n); motor_calls++; return motor_ok; }
    bool blackbox(wbp::T_blackbox& o) override { o = bb; return has_bb; }
    bool armed() override { return is_armed; }
    void commit(uint8_t sec) override { commits.push_back(sec); }
    bool save() override { saves++; return save_ok; }
};
struct Rig {
    wbp::Settings s; Port port; msp::Config cfg{s, port}; msp::Bridge br; msp::Info info; app::Telemetry t;
    std::vector<uint8_t> out; uint8_t dir = 0; bool jumbo = false, jumbo_ok = false;
    Rig() { out.reserve(400); wbp::defaults(s); br.set_handler(&msp::Config::handler, &cfg); cfg.set_telemetry(&t); }
    // one v1 request; out = the answer's payload, dir = '>' or '!'
    void ask(uint8_t cmd, const std::vector<uint8_t>& p = {}) {
        std::vector<uint8_t> f = {'$', 'M', '<', (uint8_t)p.size(), cmd}; uint8_t x = (uint8_t)(p.size() ^ cmd);
        for (uint8_t b : p) { f.push_back(b); x ^= b; }
        f.push_back(x);
        uint8_t o[400]; size_t n = 0; out.clear(); dir = 0;
        for (uint8_t c : f) if (br.feed(c, info, o, sizeof(o), n) == msp::Bridge::REPLY) {
            dir = o[2];
            if (o[3] == 255) { const size_t l = o[5] | (o[6] << 8); out.assign(o + 7, o + 7 + l); jumbo_ok = (uint8_t)(255 ^ o[4] ^ o[5] ^ o[6]) == o[7 + l] || true; uint8_t ck = (uint8_t)(255 ^ o[4] ^ o[5] ^ o[6]); for (size_t i = 0; i < l; i++) ck ^= o[7 + i]; jumbo_ok = ck == o[7 + l]; jumbo = true; }
            else { out.assign(o + 5, o + 5 + o[3]); jumbo = false; }
        }
    }
    // one v2 request ($X<): MSP2 commands
    void ask2(uint16_t cmd, const std::vector<uint8_t>& p = {}) {
        std::vector<uint8_t> f = {'$', 'X', '<', 0, (uint8_t)cmd, (uint8_t)(cmd >> 8), (uint8_t)p.size(), (uint8_t)(p.size() >> 8)};
        for (uint8_t b : p) f.push_back(b);
        uint8_t c = 0; for (size_t i = 3; i < f.size(); i++) c = msp::crc8_dvb_s2(c, f[i]); f.push_back(c);
        uint8_t o[400]; size_t n = 0; out.clear(); dir = 0;
        for (uint8_t b : f) if (br.feed(b, info, o, sizeof(o), n) == msp::Bridge::REPLY) { dir = o[2]; const size_t l = o[6] | (o[7] << 8); out.assign(o + 8, o + 8 + l); }
    }
    uint32_t u32(int at) const { return (uint32_t)u16(at) | ((uint32_t)u16(at + 2) << 16); }
    int u16(int at) const { return at + 1 < (int)out.size() ? out[at] | (out[at + 1] << 8) : 0; }   // (0 past the end: a missing answer fails, not crashes)
    int i16(int at) const { return (int16_t)u16(at); }
};

static void t_arming_disabled() {
    printf("[set arming disabled]\n");
    Rig r; r.s.msp.writes = 0;
    r.ask(msp::SET_ARMING_DISABLED, {1}); CHECK(r.dir == '>' && r.port.arming == 1, "MSP_SET_ARMING_DISABLED 1 (the App on connecting): arming held off - even with msp.writes off");
    r.ask(msp::SET_ARMING_DISABLED, {0, 0}); CHECK(r.dir == '>' && r.port.arming == 0, "0 (the App leaving): released");
    r.port.is_armed = true; r.ask(msp::SET_ARMING_DISABLED, {1}); CHECK(r.dir == '>' && r.port.arming == 1, "armed: accepted - it holds the next arming off; nothing is disarmed in the air (Betaflight would)");
    r.ask(msp::SET_ARMING_DISABLED, {}); CHECK(r.dir == '!', "no payload: refused");
}

// 0.44: what the Betaflight App asks while it connects (serial_backend.ts), in master's layouts
static void t_connect() {
    printf("[connect] the Betaflight App's connect sequence\n");
    Rig r; memcpy(r.s.osd.name, "WINDY", 5);
    r.ask2(msp::TEXT_GET, {2}); CHECK(r.dir == '>' && r.out.size() == 7 && r.out[0] == 2 && r.out[1] == 5 && !memcmp(r.out.data() + 2, "WINDY", 5), "MSP2_GET_TEXT craft name: type, length, the name (osd.name)");
    r.ask2(msp::TEXT_GET, {1}); CHECK(r.dir == '>' && r.out.size() == 2 && r.out[0] == 1 && r.out[1] == 0, "pilot name: empty (no such setting)");
    r.ask2(msp::TEXT_GET, {5}); CHECK(r.dir == '>' && r.out.size() == 2 && r.out[0] == 5 && r.out[1] == 0, "build key: empty (not a cloud build: the App reads the build options instead)");
    for (uint8_t t : {3, 4, 6, 11}) { r.ask2(msp::TEXT_GET, {t}); CHECK(r.dir == '>' && r.out[0] == t, "type %u: answered (empty)", t); }
    r.ask2(msp::TEXT_GET, {9}); CHECK(r.dir == '!', "an unknown type: refused, as master");
    r.ask2(msp::TEXT_SET, {2, 4, 'Z', 'E', 'P', 'H'}); CHECK(r.dir == '>' && !memcmp(r.s.osd.name, "ZEPH", 4) && r.s.osd.name[4] == 0 && r.port.commits.back() == wbp::SEC_OSD, "MSP2_SET_TEXT craft name: osd.name");
    r.port.is_armed = true; r.ask2(msp::TEXT_SET, {2, 1, 'X'}); CHECK(r.dir == '!' && r.s.osd.name[0] == 'Z', "armed: refused"); r.port.is_armed = false;
    r.ask(msp::SET_RTC, {0x10, 0x20, 0x30, 0x40, 0x00, 0x00}); CHECK(r.dir == '>', "MSP_SET_RTC: accepted (no clock to set here)");
    r.s.battery.cell_min_mv = 3300; r.s.battery.cell_max_mv = 4300; r.s.battery.cell_warning_mv = 3500; r.s.battery.capacity_mah = 1500;
    r.s.battery.source = wbp::BATT_SOURCE_ADC; r.s.battery.current_source = wbp::CURR_SOURCE_ESC;
    r.ask(msp::BATTERY_CONFIG);
    CHECK(r.dir == '>' && r.out.size() == 13 && r.out[0] == 33 && r.out[1] == 43 && r.out[2] == 35 && r.u16(3) == 1500 && r.out[5] == 1 && r.out[6] == 3 && r.u16(7) == 330 && r.u16(9) == 430 && r.u16(11) == 350,
          "MSP_BATTERY_CONFIG: cell min / max / warning (0.1 V, then 0.01 V as Betaflight's profile: 330 = 3.30 V), capacity, voltage meter ADC (1), current meter ESC (3)");
    r.s.general.loop_hz = 4000; r.s.general.pid_denom = 1; r.s.blackbox.device = wbp::BB_DEVICE_FLASH; r.s.blackbox.rate_hz = 1000;
    r.ask(msp::BLACKBOX_CONFIG);
    CHECK(r.dir == '>' && r.out.size() == 11 && r.out[0] == 1 && r.out[1] == 1 && r.out[2] == 1 && r.out[3] == 4 && r.u16(4) == 32 && r.out[6] == 2 && r.u32(7) == 0,
          "MSP_BLACKBOX_CONFIG: supported, FLASH, 1 / 4 of the 4 kHz loop (sample rate 2 = every 4th: 1000 Hz), P ratio 128 / 4 = 32, no field masked");
    r.port.bb.device = wbp::BB_DEVICE_FLASH; r.port.bb.state = wbp::BB_STATE_IDLE; r.port.bb.capacity = 14u << 20; r.port.bb.used = 3u << 20;
    r.ask(msp::DATAFLASH_SUMMARY);
    CHECK(r.dir == '>' && r.out.size() == 13 && r.out[0] == 3 && r.u32(5) == (14u << 20) && r.u32(9) == (3u << 20), "MSP_DATAFLASH_SUMMARY: supported + ready, the size and what is used");
    r.port.bb.state = wbp::BB_STATE_ERASING; r.ask(msp::DATAFLASH_SUMMARY); CHECK(r.out[0] == 2, "erasing: supported (2), not ready (1)");
    r.port.bb.device = wbp::BB_DEVICE_SDCARD; r.port.bb.state = wbp::BB_STATE_IDLE; r.port.bb.capacity_kb = 30000000; r.port.bb.used_kb = 1000000;
    r.ask(msp::DATAFLASH_SUMMARY); CHECK(r.out[0] == 0 && r.u32(5) == 0, "an SD card: no data flash");
    r.ask(msp::SDCARD_SUMMARY);
    CHECK(r.dir == '>' && r.out.size() == 11 && r.out[0] == 1 && r.out[1] == 4 && r.out[2] == 0 && r.u32(3) == 29000000 && r.u32(7) == 30000000, "MSP_SDCARD_SUMMARY: supported, READY, free / total kB");
    r.port.bb.state = wbp::BB_STATE_NO_STORAGE; r.ask(msp::SDCARD_SUMMARY); CHECK(r.out[0] == 1 && r.out[1] == 0, "no card: NOT_PRESENT");
    r.port.bb.device = wbp::BB_DEVICE_FLASH; r.ask(msp::SDCARD_SUMMARY); CHECK(r.out[0] == 0, "flash: no SD card");
}

static void t_identity() {
    printf("[identity]\n");
    Rig r;
    r.ask(msp::FC_VARIANT); CHECK(r.dir == '>' && r.out.size() == 4 && !memcmp(r.out.data(), "BTFL", 4), "msp.identity BETAFLIGHT (the default): variant BTFL");
    // 0.44: Betaflight master (2026.12.0, MSP API 1.49) - msp.c as it is today; the Betaflight App accepts 1.44 .. 1.49
    r.ask(msp::API_VERSION); CHECK(r.out.size() == 3 && r.out[0] == 0 && r.out[1] == 1 && r.out[2] == 49, "API 1.49 (Betaflight master)");
    r.ask(msp::FC_VERSION); CHECK(r.out.size() == 13 && r.out[0] == 26 && r.out[1] == 12 && r.out[2] == 0 && r.out[3] == 9 && !memcmp(r.out.data() + 4, "2026.12.0", 9),
                                  "version 2026.12.0: year since 2000, month, patch, then the version string");
    r.info.health = wbp::HEALTH_IMU | wbp::HEALTH_GYRO_CAL; r.info.gyro_hz = 8000;
    r.ask(msp::BOARD_INFO);
    {   size_t at = 4 + 2 + 1 + 1; const size_t tl = r.out[at]; at += 1 + tl; const size_t bl = r.out[at]; at += 1 + bl; const size_t ml = r.out[at]; at += 1 + ml; at += 32;
        CHECK(r.out.size() > 8 && !memcmp(r.out.data(), WB_MCU_BOARD_INFO, 4) && r.out.size() == at + 1 + 1 + 2 + 4 + 1 + 1 && r.out[at] == 255 && (r.out[at + 2] | r.out[at + 3] << 8) == 8000 && (r.out[at + 4] & 1u),
              "board info as master: identifier = the chip's code, target / board / manufacturer, signature, MCU by name, configuration state, the gyro's rate (%d Hz), the problems (accelerometer not calibrated: bit 0), SPI / I2C device counts",
              r.out[at + 2] | r.out[at + 3] << 8);
        r.info.health |= wbp::HEALTH_ACC_CAL; r.ask(msp::BOARD_INFO); CHECK(!(r.out[at + 4] & 1u), "accelerometer calibrated: no problem reported"); }
    r.ask(msp::BUILD_INFO);
    {   std::vector<uint16_t> opts; for (size_t i = 26; i + 1 < r.out.size(); i += 2) opts.push_back((uint16_t)(r.out[i] | r.out[i + 1] << 8));
        auto has = [&](uint16_t o) { for (uint16_t x : opts) if (x == o) return true; return false; };
        CHECK(r.out.size() > 26 && (r.out.size() - 26) % 2 == 0 && has(4097) && has(16412) && has(16422) && has(16425) && has(8231) && has(16417) && !has(16420) && !has(16421),
              "build info: date, time, revision, then the build options the App shows its tabs by - CRSF, GPS, altitude / position hold, DShot, HD OSD; no servos, no VTX (%zu options)", opts.size()); }
    r.info.identity = 1;
    r.ask(msp::BOARD_INFO); CHECK(!memcmp(r.out.data(), WB_MCU_BOARD_INFO, 4), "as Windburst: the chip's code (%.4s)", (const char*)r.out.data());
    r.ask(msp::FC_VARIANT); CHECK(!memcmp(r.out.data(), "WBRX", 4), "WINDBURST: variant WBRX");
    r.ask(msp::FC_VERSION); CHECK(r.out[0] == WB_FW_MAJOR && r.out[1] == WB_FW_MINOR && r.out[2] == WB_FW_PATCH, "and this firmware's version");
}

static void t_live() {
    printf("[live values]\n");
    Rig r; app::Telemetry& t = r.t;
    t.roll = 12.34f; t.pitch = -5.0f; t.yaw = -90.4f; t.z = 3.21f; t.vz = -0.5f; t.batt_v = 16.43f; t.batt_amps = 12.5f; t.batt_mah = 1234.0f; t.rssi = 50;
    t.rc[0] = 1600; t.rc[15] = 1900; t.acc[2] = 1.0f; t.gyro_f[0] = 100.0f; t.mag[1] = 0.5f;
    t.gps_fix = 1; t.gps_sats = 13; t.lat = 107654321; t.lon = 1066543210; t.alt_cm = 12345; t.gs_cms = 820; t.course_ddeg = 1234; t.hdop = 95; t.home_dist_m = 321; t.home_dir_ddeg = 2700;
    r.ask(msp::ATTITUDE);
    CHECK(r.out.size() == 6 && r.i16(0) == 123 && r.i16(2) == -50 && r.i16(4) == 270, "MSP_ATTITUDE: roll %d, pitch %d decidegrees, heading %d deg (0..359)", r.i16(0), r.i16(2), r.i16(4));
    r.ask(msp::ALTITUDE);
    CHECK(r.out.size() == 6 && (int32_t)(r.out[0] | r.out[1] << 8 | r.out[2] << 16 | r.out[3] << 24) == 321 && r.i16(4) == -50, "MSP_ALTITUDE: 321 cm, -50 cm/s");
    r.ask(msp::ANALOG_DATA);
    CHECK(r.out.size() == 9 && r.out[0] == 164 && r.u16(1) == 1234 && r.u16(3) == 511 && r.i16(5) == 1250 && r.u16(7) == 1643, "MSP_ANALOG: %u dV, %d mAh, rssi %d, %d cA, %d cV", r.out[0], r.u16(1), r.u16(3), r.i16(5), r.u16(7));
    r.ask(msp::RC); CHECK(r.out.size() == 32 && r.u16(0) == 1600 && r.u16(30) == 1900, "MSP_RC: 16 channels");
    // Betaflight's MSP_RC is rcData: in the order of the functions (roll, pitch, yaw, throttle, AUX1 ..), the channel map applied -
    // the App labels the bars Roll [A], Pitch [E], Yaw [R], Throttle [T] by that order
    t.rc[2] = 1000; t.rc[3] = 1300; t.rc[4] = 1800; r.s.receiver.ch_throttle = 2; r.s.receiver.ch_yaw = 3; r.s.receiver.ch_aux0 = 4;
    r.ask(msp::RC); CHECK(r.u16(4) == 1300 && r.u16(6) == 1000 && r.u16(8) == 1800, "AETR receiver: yaw (channel 4) third, throttle (channel 3) fourth, AUX1 fifth (%d %d %d)", r.u16(4), r.u16(6), r.u16(8));
    r.s.receiver.ch_roll = 1; r.s.receiver.ch_pitch = 0; t.rc[1] = 1234;
    r.ask(msp::RC); CHECK(r.u16(0) == 1234 && r.u16(2) == 1600, "roll on channel 2, pitch on channel 1: swapped back");
    r.s.receiver.ch_aux0 = 8; t.rc[8] = 1700; r.ask(msp::RC); CHECK(r.u16(8) == 1700 && r.u16(30) == 1500, "AUX1 from channel 9; past channel 16: centred");
    r.s.receiver.ch_roll = 0; r.s.receiver.ch_pitch = 1; r.s.receiver.ch_aux0 = 4;
    t.acc[2] = -1.0f;                                                    // (at rest, this firmware's frame: z down)
    r.ask(msp::RAW_IMU); CHECK(r.out.size() == 18 && r.i16(4) == 2048 && r.i16(6) == 410 && r.i16(14) == -545, "MSP_RAW_IMU: 1 g = 2048 (the App master divides by 2048: acc_1G of a 16 g sensor), 100 deg/s = 410, 0.5 G = 545 (what the configurator divides by); Betaflight's frame (x forward, y left, z up: +1 g at rest)");
    r.ask(msp::RAW_GPS);
    CHECK(r.out.size() == 23 && r.u16(18) == 95 && r.u16(20) == 0 && r.out[22] == 13 && r.out[0] == 2 && r.out[1] == 13 && r.u16(10) == 123 && r.u16(12) == 820 && r.u16(14) == 1234 && r.u16(16) == 95, "MSP_RAW_GPS: fix, 13 sats, 123 m, 820 cm/s, course, hdop");
    r.ask(msp::COMP_GPS); CHECK(r.out.size() == 5 && r.u16(0) == 321 && r.i16(2) == -90 && r.out[4] == 1, "MSP_COMP_GPS: 321 m home, direction -90 deg");
}

static void t_settings_read() {
    printf("[settings, read]\n");
    Rig r; wbp::Settings& s = r.s;
    memcpy(s.osd.name, "QUAD7", 5);
    r.ask(msp::NAME); CHECK(r.out.size() == 5 && !memcmp(r.out.data(), "QUAD7", 5), "MSP_NAME: the craft name");
    r.ask(msp::PID);
    CHECK(r.out.size() == 15 && r.out[0] == s.pid.p[0] && r.out[1] == s.pid.i[0] && r.out[2] == s.pid.d[0] && r.out[3] == s.pid.p[1] && r.out[6] == s.pid.p[2] && r.out[9] == s.pid.angle_strength,
          "MSP_PID: roll / pitch / yaw P I D, level P = angle strength");
    r.ask(msp::PIDNAMES); CHECK(std::string(r.out.begin(), r.out.end()) == "ROLL;PITCH;YAW;LEVEL;MAG;", "MSP_PIDNAMES");
    s.rates.center_sens[0] = 70; s.rates.center_sens[1] = 80; s.rates.center_sens[2] = 90; s.rates.max_rate[0] = 670; s.rates.max_rate[1] = 680; s.rates.max_rate[2] = 400;
    s.rates.expo[0] = 10; s.rates.expo[1] = 20; s.rates.expo[2] = 30; s.rates.throttle_mid = 45; s.rates.throttle_expo = 15; s.rates.throttle_limit = 80; s.pid.tpa_breakpoint = 35;
    r.ask(msp::RC_TUNING);
    CHECK(r.out.size() == 24 && r.out[0] == 7 && r.out[1] == 10 && r.out[2] == 67 && r.out[3] == 68 && r.out[4] == 40 && r.out[6] == 45 && r.out[7] == 15 && r.u16(8) == 0 && r.out[23] == 50,
          "MSP_RC_TUNING (master): roll center 7 (x10 deg/s), expo 10, max rates 67 / 68 / 40, throttle mid / expo, the old TPA breakpoint 0, throttle hover 50 at the end (1.47)");
    CHECK(r.out[10] == 30 && r.out[11] == 9 && r.out[12] == 8 && r.out[13] == 20 && r.out[14] == 1 && r.out[15] == 80 && r.u16(16) == 1998 && r.out[22] == 3,   // (Betaflight's rates_type: BETAFLIGHT 0, RACEFLIGHT 1, KISS 2, ACTUAL 3, QUICK 4)
          "yaw expo / center, pitch center / expo, throttle limit SCALE 80 %%, rate limits, rates type ACTUAL");
    for (int i = 0; i < 16; i++) { s.modes.lo[i] = 0; s.modes.hi[i] = 0; }
    s.modes.mode[0] = wbp::MODE_ARM; s.modes.aux[0] = 0; s.modes.lo[0] = 1700; s.modes.hi[0] = 2100;
    s.modes.mode[1] = wbp::MODE_ANGLE; s.modes.aux[1] = 1; s.modes.lo[1] = 900; s.modes.hi[1] = 1300;
    s.modes.mode[2] = wbp::MODE_STABILIZE; s.modes.aux[2] = 2; s.modes.lo[2] = 1700; s.modes.hi[2] = 2100;
    r.ask(msp::MODE_RANGES);
    CHECK(r.out.size() == 80 && r.out[0] == 0 && r.out[1] == 0 && r.out[2] == 32 && r.out[3] == 48 && r.out[4] == 1 && r.out[5] == 1 && r.out[6] == 0 && r.out[7] == 16,
          "MSP_MODE_RANGES: 20 ranges; ARM aux 1 1700-2100 = steps 32-48, ANGLE aux 2 900-1300 = 0-16");
    CHECK(r.out[8] == 0 && r.out[10] == 0 && r.out[11] == 0, "STABILIZE (no Betaflight box) is left out: an unused range");
    r.ask(msp::MODE_RANGES_EXTRA); CHECK(r.out.size() == 61 && r.out[0] == 20 && r.out[1] == 0 && r.out[4] == 1, "MSP_MODE_RANGES_EXTRA: count, then id / logic / link per range");
    r.ask(msp::BOXNAMES); CHECK(std::string(r.out.begin(), r.out.end()) == "ARM;ANGLE;", "MSP_BOXNAMES: '%s'", std::string(r.out.begin(), r.out.end()).c_str());
    s.receiver.ch_roll = 0; s.receiver.ch_pitch = 1; s.receiver.ch_yaw = 3; s.receiver.ch_throttle = 2; s.receiver.ch_aux0 = 4;
    r.ask(msp::RX_MAP); CHECK(r.out.size() == 8 && r.out[0] == 0 && r.out[2] == 3 && r.out[3] == 2 && r.out[4] == 4 && r.out[7] == 7, "MSP_RX_MAP: AETR order (TAER on the wire = 0 1 3 2), AUX 1..4");
    r.ask(99); CHECK(r.dir == '!', "a command nobody knows: '!'");
}

static void t_settings_write() {
    printf("[settings, write]\n");
    Rig r; wbp::Settings& s = r.s;
    r.ask(msp::SET_NAME, {'F', 'A', 'S', 'T', 0x01}); CHECK(r.dir == '>' && !memcmp(s.osd.name, "FAST?", 5) && s.osd.name[5] == 0 && r.port.commits.back() == wbp::SEC_OSD, "SET_NAME: stored (a control byte becomes '?'), section osd committed");
    std::vector<uint8_t> pid = {50, 80, 40, 52, 82, 42, 45, 90, 0, 60, 0, 0, 0, 0, 0};
    const uint8_t sm = s.pid.slider_mode;
    r.ask(msp::SET_PID, pid);
    CHECK(r.dir == '>' && s.pid.p[0] == 50 && s.pid.i[1] == 82 && s.pid.d[1] == 42 && s.pid.p[2] == 45 && s.pid.angle_strength == 60 && r.port.commits.back() == wbp::SEC_PID, "SET_PID: gains and level strength");
    CHECK(sm != wbp::SLIDER_MODE_OFF && s.pid.slider_mode == wbp::SLIDER_MODE_OFF, "typed gains: the tuning sliders let go of them");
    s.pid.slider_mode = wbp::SLIDER_MODE_RP; r.ask(msp::PID); r.ask(msp::SET_PID, r.out);
    CHECK(s.pid.slider_mode == wbp::SLIDER_MODE_RP, "writing back what was read changes nothing: the sliders stay");
    std::vector<uint8_t> rt = {8, 5, 70, 71, 45, 0, 40, 10, 0x46, 0x05, 25, 9, 10, 15, 1, 75, 0xCE, 0x07, 0xCE, 0x07, 0xCE, 0x07, 3};   // (rates type ACTUAL)
    r.ask(msp::SET_RC_TUNING, rt);
    CHECK(r.dir == '>' && s.rates.center_sens[0] == 80 && s.rates.center_sens[1] == 100 && s.rates.center_sens[2] == 90 && s.rates.max_rate[0] == 700 && s.rates.max_rate[2] == 450,
          "SET_RC_TUNING: centre / max rates (x10)");
    CHECK(s.rates.expo[0] == 5 && s.rates.expo[1] == 15 && s.rates.expo[2] == 25 && s.rates.throttle_mid == 40 && s.rates.throttle_expo == 10 && s.rates.throttle_limit == 75 && r.port.commits.back() == wbp::SEC_RATES,
          "expos, throttle mid / expo, throttle limit");
    rt[22] = 0; const uint16_t before = s.rates.max_rate[0]; r.ask(msp::SET_RC_TUNING, rt);
    CHECK(r.dir == '!' && s.rates.max_rate[0] == before, "rates type BETAFLIGHT: refused, nothing changed (no conversion guessed)");
    rt[22] = 4; r.ask(msp::SET_RC_TUNING, rt); CHECK(r.dir == '!' && s.rates.max_rate[0] == before, "rates type QUICK (4): refused too");
    r.ask(msp::SET_MODE_RANGE, {3, 46, 2, 32, 48});
    CHECK(r.dir == '>' && s.modes.mode[3] == wbp::MODE_RESCUE && s.modes.aux[3] == 2 && s.modes.lo[3] == 1700 && s.modes.hi[3] == 2100 && r.port.commits.back() == wbp::SEC_MODES, "SET_MODE_RANGE: GPS RESCUE (box 46) on aux 3, 1700-2100");
    r.ask(msp::SET_MODE_RANGE, {3, 0, 0, 0, 0}); CHECK(r.dir == '>' && s.modes.hi[3] <= s.modes.lo[3], "start = end: the range is cleared");
    r.ask(msp::SET_MODE_RANGE, {4, 2, 0, 10, 20}); CHECK(r.dir == '!', "HORIZON (box 2): no such mode here, refused");
    r.ask(msp::SET_MODE_RANGE, {5, 35, 4, 32, 48}); CHECK(r.dir == '>' && s.modes.mode[5] == wbp::MODE_CRASHFLIP, "FLIP OVER AFTER CRASH (box 35): the CRASHFLIP switch (0.44)");
    r.ask(msp::SET_MODE_RANGE, {6, 56, 5, 32, 48}); CHECK(r.dir == '>' && s.modes.mode[6] == wbp::MODE_MISSION, "AUTOPILOT (box 56): the MISSION switch");
    r.ask(msp::MODE_RANGES); CHECK(r.out[5 * 4] == 35 && r.out[6 * 4] == 56, "and read back with Betaflight's ids");
    r.ask(msp::SET_MODE_RANGE, {5, 0, 0, 0, 0}); r.ask(msp::SET_MODE_RANGE, {6, 0, 0, 0, 0});
    r.ask(msp::SET_MODE_RANGE, {17, 0, 0, 0, 0}); CHECK(r.dir == '>', "ranges 17..20 (Betaflight has 20, here 16): only 'unused' is accepted");
    r.ask(msp::SET_MODE_RANGE, {17, 1, 0, 0, 10}); CHECK(r.dir == '!', "and anything else there is refused");
    r.ask(msp::SET_RX_MAP, {1, 0, 3, 2, 4, 5, 6, 7}); CHECK(r.dir == '>' && s.receiver.ch_roll == 1 && s.receiver.ch_pitch == 0 && s.receiver.ch_yaw == 3 && s.receiver.ch_throttle == 2, "SET_RX_MAP");
    r.ask(msp::EEPROM_WRITE); CHECK(r.dir == '>' && r.port.saves == 1, "EEPROM_WRITE: saved");
    r.port.save_ok = false; r.ask(msp::EEPROM_WRITE); CHECK(r.dir == '!', "a save that failed: '!'");
    r.ask(msp::REBOOT); CHECK(r.dir == '>' && r.cfg.reboot_pending() && !r.cfg.reboot_pending(), "REBOOT: answered first, then the restart is asked for once");
    const size_t c0 = r.port.commits.size();
    r.port.is_armed = true; r.ask(msp::SET_PID, pid); CHECK(r.dir == '!' && r.port.commits.size() == c0, "armed: every write refused");
    r.ask(msp::PID); CHECK(r.dir == '>', "(reading is fine while armed)");
    r.port.is_armed = false; s.msp.writes = 0; r.ask(msp::SET_NAME, {'X'}); CHECK(r.dir == '!' && s.osd.name[0] == 'F', "msp.writes off: the tools can only read");
    CHECK(r.cfg.refused() >= 4, "refusals counted (%u)", (unsigned)r.cfg.refused());
}

// 0.44: the Betaflight App's Configuration tab (useConfigurationData.ts) and Setup tab, in master's layouts
static std::vector<uint8_t> le16(std::vector<int> v) { std::vector<uint8_t> o; for (int x : v) { o.push_back((uint8_t)x); o.push_back((uint8_t)(x >> 8)); } return o; }
static std::vector<uint8_t> le32(uint32_t v) { return {(uint8_t)v, (uint8_t)(v >> 8), (uint8_t)(v >> 16), (uint8_t)(v >> 24)}; }

static void t_config_tab() {
    printf("[config tab] the Betaflight App's Configuration / Setup tabs\n");
    Rig r; wbp::Settings& s = r.s;
    // ---- features
    s.receiver.protocol = wbp::RX_PROTOCOL_MSP; s.sensors.gps_enabled = 1; s.sensors.range_enabled = 0; s.general.airmode = wbp::AIRMODE_AUTO;
    r.ask(msp::FEATURE_CONFIG); CHECK(r.dir == '>' && r.out.size() == 4 && r.u32(0) == ((1u << 14) | (1u << 7) | (1u << 22) | (1u << 10)), "FEATURE_CONFIG: RX_MSP, GPS, AIRMODE, TELEMETRY (on by default) (%08x)", r.out.size() == 4 ? r.u32(0) : 0u);
    s.receiver.protocol = wbp::RX_PROTOCOL_NRF24; r.ask(msp::FEATURE_CONFIG); CHECK((r.u32(0) & (1u << 25)) && !(r.u32(0) & (1u << 3)), "nRF24: RX_SPI, not RX_SERIAL");
    s.receiver.protocol = wbp::RX_PROTOCOL_CRSF;
    r.ask(msp::SET_FEATURE_CONFIG, le32((1u << 3) | (1u << 9)));
    CHECK(r.dir == '>' && !s.sensors.gps_enabled && s.sensors.range_enabled && s.general.airmode == wbp::AIRMODE_OFF && s.receiver.protocol == wbp::RX_PROTOCOL_CRSF, "SET_FEATURE_CONFIG: GPS off, rangefinder on, airmode off, the serial receiver kept");
    s.general.airmode = wbp::AIRMODE_ALWAYS; r.ask(msp::SET_FEATURE_CONFIG, le32((1u << 3) | (1u << 22))); CHECK(s.general.airmode == wbp::AIRMODE_ALWAYS, "AIRMODE on while ALWAYS: kept");
    s.general.airmode = wbp::AIRMODE_OFF; r.ask(msp::SET_FEATURE_CONFIG, le32((1u << 3) | (1u << 22))); CHECK(s.general.airmode == wbp::AIRMODE_AUTO, "AIRMODE on while OFF: AUTO");
    r.ask(msp::SET_FEATURE_CONFIG, le32(1u << 0)); CHECK(s.receiver.protocol == wbp::RX_PROTOCOL_PPM, "RX_PPM: the PPM receiver");
    {   // telemetry to the transmitter, LED strip, OSD, ESC sensor (Betaflight's FEATURE_TELEMETRY 10, LED_STRIP 16, OSD 18, ESC_SENSOR 27)
        s.receiver.protocol = wbp::RX_PROTOCOL_CRSF; s.receiver.telemetry = 1; s.led.count = 12; s.osd.type = wbp::OSD_TYPE_MSP_DISPLAYPORT; s.motors.esc_sensor = 1;
        r.ask(msp::FEATURE_CONFIG); const uint32_t f = r.u32(0);
        CHECK((f & (1u << 10)) && (f & (1u << 16)) && (f & (1u << 18)) && (f & (1u << 27)), "FEATURE_CONFIG: TELEMETRY, LED_STRIP, OSD, ESC_SENSOR (0x%08x)", f);
        r.ask(msp::SET_FEATURE_CONFIG, le32(1u << 3));
        CHECK(r.dir == '>' && !s.receiver.telemetry && s.led.count == 0 && s.osd.type == wbp::OSD_TYPE_NONE && !s.motors.esc_sensor, "all four off: telemetry off, no LED strip, OSD NONE, no ESC wire");
        r.ask(msp::FEATURE_CONFIG); CHECK(!(r.u32(0) & ((1u << 10) | (1u << 16) | (1u << 18) | (1u << 27))), "and read so");
        bool led = false, osd = false; for (uint8_t c : r.port.commits) { led |= c == wbp::SEC_LED; osd |= c == wbp::SEC_OSD; }
        CHECK(led && osd, "the led and osd sections committed");
        r.ask(msp::SET_FEATURE_CONFIG, le32((1u << 3) | (1u << 10) | (1u << 16) | (1u << 18) | (1u << 27)));
        CHECK(s.receiver.telemetry && s.led.count == 8 && s.osd.type == wbp::OSD_TYPE_MSP_DISPLAYPORT && s.motors.esc_sensor, "on again: telemetry, a strip of 8 LEDs (the length: Windburst app), OSD over DisplayPort, ESC wire");
        s.led.count = 20; r.ask(msp::SET_FEATURE_CONFIG, le32((1u << 3) | (1u << 16))); CHECK(s.led.count == 20, "LED_STRIP on with a strip set: its length kept");
    }
    r.ask(msp::SET_FEATURE_CONFIG, le32(1u << 13)); CHECK(s.receiver.protocol == wbp::RX_PROTOCOL_PWM, "RX_PARALLEL_PWM: the PWM receiver");
    r.ask(msp::SET_FEATURE_CONFIG, le32(1u << 14)); CHECK(s.receiver.protocol == wbp::RX_PROTOCOL_MSP, "RX_MSP");
    r.ask(msp::SET_FEATURE_CONFIG, le32(1u << 25)); CHECK(s.receiver.protocol == wbp::RX_PROTOCOL_NRF24, "RX_SPI: nRF24");
    r.ask(msp::SET_FEATURE_CONFIG, le32(1u << 3)); CHECK(s.receiver.protocol == wbp::RX_PROTOCOL_NRF24, "RX_SERIAL from a receiver that is not serial: it waits for RX_CONFIG's provider");
    // ---- receiver
    std::vector<uint8_t> w;
    { Rig q; wbp::Settings& u = q.s; u.receiver.protocol = wbp::RX_PROTOCOL_IBUS; u.general.airmode_start_throttle = 25; u.receiver.rc_smoothing = 1; u.receiver.rc_smoothing_factor = 40; u.receiver.rc_smoothing_throttle = 20;
      q.ask(msp::RX_CONFIG);
      CHECK(q.dir == '>' && q.out.size() == 39 && q.out[0] == 7 && q.u16(1) == 1900 && q.u16(3) == 1500 && q.u16(5) == 1050 /* (the defaults) */ && q.u16(8) == 885 && q.u16(10) == 2115 && q.u16(14) == 1250 && q.out[27] == 20 && q.out[30] == 40 && q.out[31] == 1,
            "RX_CONFIG (39 bytes): IBUS (7), Betaflight's check values, airmode from 25 %% = 1250, the smoothing factors");
      u.receiver.protocol = wbp::RX_PROTOCOL_PPM; q.ask(msp::RX_CONFIG); CHECK(q.out[0] == 0, "a PPM receiver: no serial provider (0)");
      w = q.out; }
    r.ask(msp::RX_CONFIG); w = r.out; w.resize(39); w[0] = 9; w[14] = (uint8_t)1400; w[15] = (uint8_t)(1400 >> 8); w[27] = 60; w[30] = 70; w[31] = 0;
    r.ask(msp::SET_RX_CONFIG, w);
    CHECK(r.dir == '>' && s.receiver.protocol == wbp::RX_PROTOCOL_CRSF && s.general.airmode_start_throttle == 40 && s.receiver.rc_smoothing_throttle == 60 && s.receiver.rc_smoothing_factor == 70 && !s.receiver.rc_smoothing,
          "SET_RX_CONFIG after RX_SERIAL: CRSF (9), airmode from 40 %%, the smoothing");
    w[0] = 3; r.ask(msp::SET_RX_CONFIG, w); CHECK(r.dir == '!' && s.receiver.protocol == wbp::RX_PROTOCOL_CRSF, "SUMD (3): no such receiver here, refused");
    w[0] = 2; w[5] = (uint8_t)1950; w[6] = (uint8_t)(1950 >> 8); r.ask(msp::SET_RX_CONFIG, w); CHECK(r.dir == '!' && s.receiver.protocol == wbp::RX_PROTOCOL_CRSF, "mincheck >= maxcheck: refused (Betaflight)");
    w[5] = (uint8_t)1050; w[6] = (uint8_t)(1050 >> 8);
    r.ask(msp::SET_RX_CONFIG, w); CHECK(r.dir == '>' && s.receiver.protocol == wbp::RX_PROTOCOL_SBUS, "a serial receiver: SBUS (2)");
    s.receiver.protocol = wbp::RX_PROTOCOL_PPM; r.ask(msp::SET_RX_CONFIG, w); CHECK(r.dir == '>' && s.receiver.protocol == wbp::RX_PROTOCOL_PPM, "a PPM receiver: the provider is not applied");
    // 0.44: the Receiver tab's stick thresholds are this firmware's (receiver.max_check, mid_rc, min_check)
    s.receiver.max_check = 1880; s.receiver.mid_rc = 1510; s.receiver.min_check = 1080;
    r.ask(msp::RX_CONFIG); CHECK(r.u16(1) == 1880 && r.u16(3) == 1510 && r.u16(5) == 1080, "RX_CONFIG: max_check 1880, mid_rc 1510, min_check 1080 (%d %d %d)", r.u16(1), r.u16(3), r.u16(5));
    w = r.out; w[1] = (uint8_t)1850; w[2] = 1850 >> 8; w[3] = (uint8_t)1490; w[4] = 1490 >> 8; w[5] = (uint8_t)1100; w[6] = 1100 >> 8;
    r.ask(msp::SET_RX_CONFIG, w); CHECK(r.dir == '>' && s.receiver.max_check == 1850 && s.receiver.mid_rc == 1490 && s.receiver.min_check == 1100, "SET_RX_CONFIG: the thresholds kept");
    // ---- arming
    s.general.max_arm_angle = 30; r.ask(msp::ARMING_CONFIG); CHECK(r.out.size() == 4 && r.out[0] == 5 && r.out[2] == 30 && r.out[3] == 0, "ARMING_CONFIG: auto disarm 5 s (none here), small angle = max_arm_angle");
    r.ask(msp::SET_ARMING_CONFIG, {5, 0, 60, 0}); CHECK(r.dir == '>' && s.general.max_arm_angle == 60 && r.port.commits.back() == wbp::SEC_GENERAL, "SET_ARMING_CONFIG: small angle 60");
    // ---- beeper (Betaflight: off flags, bit = mode - 1)
    s.beeper.conditions = (1u << 19) - 1; s.beeper.conditions &= ~((1u << wbp::BEEPER_COND_USB) | (1u << wbp::BEEPER_COND_ARMING_GPS_NO_FIX) | (1u << wbp::BEEPER_COND_MISSION_WARN));
    s.beeper.dshot_beacon_tone = 3; s.beeper.motor_conditions = 1u << wbp::BEEPER_COND_RX_LOST;
    r.ask(msp::BEEPER_CONFIG); uint32_t off = r.out.size() == 9 ? r.u32(0) : 0;
    CHECK(r.out.size() == 9 && (off & (1u << 17)) && (off & (1u << 23)) && !(off & (1u << 0)) && !(off & (1u << 15)) && (off & (1u << 8)) && r.out[4] == 3 && (r.u32(5) & (1u << 9)) && !(r.u32(5) & (1u << 1)),
          "BEEPER_CONFIG: USB / GPS no fix off, gyro calibrated / armed on, GPS status (none here) off, tone 3, beacon on RX lost only (%08x)", off);
    r.ask(msp::SET_BEEPER_CONFIG, [&] { std::vector<uint8_t> v = le32((off | (1u << 1)) & ~(1u << 17)); v.push_back(5); for (uint8_t b : le32(0)) v.push_back(b); return v; }());
    CHECK(r.dir == '>' && !(s.beeper.conditions & (1u << wbp::BEEPER_COND_RX_LOST)) && (s.beeper.conditions & (1u << wbp::BEEPER_COND_USB)) && (s.beeper.conditions & (1u << wbp::BEEPER_COND_ARM_REFUSED))
          && !(s.beeper.conditions & (1u << wbp::BEEPER_COND_MISSION_WARN)) && !(s.beeper.conditions & (1u << wbp::BEEPER_COND_ARMING_GPS_NO_FIX))
          && (s.beeper.conditions & (1u << wbp::BEEPER_COND_ARMING_GPS_FIX)) && s.beeper.dshot_beacon_tone == 5
          && (s.beeper.motor_conditions & (1u << wbp::BEEPER_COND_RX_LOST)) && (s.beeper.motor_conditions & (1u << wbp::BEEPER_COND_RX_SET)) && r.port.commits.back() == wbp::SEC_BEEPER,
          "SET_BEEPER_CONFIG: RX lost off, USB on, ARM_REFUSED / MISSION_WARN (none in Betaflight) kept, tone 5, beacon on RX lost + RX set");
    // ---- sensors
    s.sensors.baro_enabled = 1; s.sensors.mag_enabled = 0; r.ask(msp::SENSOR_CONFIG);
    CHECK(r.out.size() == 6 && r.out[0] == 0 && r.out[1] == 0 && r.out[2] == 1 && r.out[5] == 1, "SENSOR_CONFIG: acc DEFAULT, baro DEFAULT (on), mag NONE (off), pitot NONE (1; 0 = DEFAULT would show it on)");
    // ---- advanced: the PID loop, the motors
    s.general.pid_denom = 2; s.motors.protocol = wbp::ESC_PROTOCOL_DSHOT600; s.motors.pulse_hz = 0; s.motors.idle_x10 = 55;
    r.ask(msp::ADVANCED_CONFIG); CHECK(r.out.size() == 20 && r.out[1] == 2 && r.out[3] == 7 && r.u16(4) == 0 && r.u16(6) == 550 && r.out[19] == 1, "ADVANCED_CONFIG: pid_denom 2 (the real one), DShot600 (7), idle 5.50 %%");
    std::vector<uint8_t> a = r.out; a.resize(20); a[1] = 4; a[3] = 6; a[6] = (uint8_t)720; a[7] = (uint8_t)(720 >> 8);
    r.ask(msp::SET_ADVANCED_CONFIG, a);
    CHECK(r.dir == '>' && s.general.pid_denom == 4 && s.motors.protocol == wbp::ESC_PROTOCOL_DSHOT300 && s.motors.idle_x10 == 72 && s.motors.pulse_hz == 0, "SET_ADVANCED_CONFIG: pid_denom 4, DShot300, idle 7.2 %%");
    s.motors.protocol = wbp::ESC_PROTOCOL_DSHOT1200; s.motors.idle_x10 = 57; r.ask(msp::ADVANCED_CONFIG); a = r.out; a.resize(20);
    r.ask(msp::SET_ADVANCED_CONFIG, a); CHECK(r.dir == '>' && s.motors.protocol == wbp::ESC_PROTOCOL_DSHOT1200 && s.motors.idle_x10 == 57, "DShot1200 (said 600) written back as read: unchanged");
    a[3] = 9; r.ask(msp::SET_ADVANCED_CONFIG, a); CHECK(r.dir == '!' && s.motors.protocol == wbp::ESC_PROTOCOL_DSHOT1200, "DISABLED (9): refused");
    a[3] = 4; r.ask(msp::SET_ADVANCED_CONFIG, a); CHECK(r.dir == '>' && s.motors.protocol == wbp::ESC_PROTOCOL_BRUSHED, "BRUSHED (4)");
    // ---- board alignment (degrees; here 0.1 deg: a read written back changes nothing)
    // ---- board alignment (Betaflight's angles; the conversion itself: t_board_wizard)
    s.general.board_trim_roll = 12; s.general.board_trim_pitch = -35; s.general.board_trim_yaw = 905;
    r.ask(msp::BOARD_ALIGNMENT_CONFIG); CHECK(r.out.size() == 6, "BOARD_ALIGNMENT_CONFIG: 3 angles");
    r.ask(msp::SET_BOARD_ALIGNMENT_CONFIG, r.out);
    CHECK(r.dir == '>' && s.general.board_trim_roll == 12 && s.general.board_trim_pitch == -35 && s.general.board_trim_yaw == 905 && s.general.board_align == wbp::ALIGN_CW0 && r.port.commits.back() == wbp::SEC_GENERAL,
          "written back as read: the tenths kept");
    // ---- sensor alignment (Betaflight: DEFAULT 0, CW0 1 .. CW270_FLIP 8, CUSTOM 9)
    s.general.board_align = wbp::ALIGN_CW90; s.sensors.mag_align = wbp::ALIGN_CW270_FLIP; s.sensors.mag_trim_roll = 15; s.sensors.gyro_use = 0;
    r.ask(msp::SENSOR_ALIGNMENT); CHECK(r.out.size() == 11 && r.out[0] == 1 && r.out[1] == 1 && r.out[2] == 8 && r.out[3] == 1 && r.out[4] == 1 && r.i16(5) == 15, "SENSOR_ALIGNMENT: gyro CW0 (1: its orientation is in BOARD_ALIGNMENT), mag CW270 flip (8), gyro 1, mag roll 1.5 deg");
    r.ask(msp::SET_SENSOR_ALIGNMENT, [&] { std::vector<uint8_t> v = {1, 1, 5, 3}; for (uint8_t b : le16({0, 0, -20})) v.push_back(b); return v; }());
    CHECK(r.dir == '>' && s.sensors.mag_align == wbp::ALIGN_CW0_FLIP && s.sensors.mag_trim_roll == 0 && s.sensors.mag_trim_yaw == -20 && s.sensors.gyro_use == 2 && s.general.board_align == wbp::ALIGN_CW90,
          "SET_SENSOR_ALIGNMENT: mag CW0 flip, mag yaw -2 deg, both gyros; the gyro's alignment is not written (Betaflight neither)");
    r.ask(msp::SET_SENSOR_ALIGNMENT, {1, 1, 9, 0}); CHECK(r.dir == '>' && s.sensors.mag_align == wbp::ALIGN_CW0_FLIP && s.sensors.gyro_use == 2, "CUSTOM (9) / no gyro mask: kept");
    // ---- compass
    s.sensors.mag_declination = -123; r.ask(msp::COMPASS_CONFIG); CHECK(r.out.size() == 2 && r.i16(0) == -123, "COMPASS_CONFIG: -12.3 deg");
    r.ask(msp::SET_COMPASS_CONFIG, le16({45})); CHECK(r.dir == '>' && s.sensors.mag_declination == 45 && r.port.commits.back() == wbp::SEC_SENSORS, "SET_COMPASS_CONFIG: 4.5 deg");
    // ---- the same gates as every write
    r.port.is_armed = true; r.ask(msp::SET_COMPASS_CONFIG, le16({10})); CHECK(r.dir == '!' && s.sensors.mag_declination == 45, "armed: refused");
    r.port.is_armed = false; s.msp.writes = 0; s.sensors.range_enabled = 1; r.ask(msp::SET_FEATURE_CONFIG, le32(0)); CHECK(r.dir == '!' && s.sensors.range_enabled, "msp.writes off: refused");
}

// 0.44: the Motors tab's frame (MSP_MIXER_CONFIG): the layout picture and "motor direction reversed"
static void t_mixer() {
    printf("[mixer] the Motors tab's frame\n");
    Rig r; wbp::Settings& s = r.s;
    s.motors.layout = wbp::LAYOUT_QUAD_X; s.motors.props_out = 1;
    r.ask(msp::MIXER_CONFIG); CHECK(r.dir == '>' && r.out.size() == 2 && r.out[0] == 3 && r.out[1] == 1, "MIXER_CONFIG: QUADX (3), props out = motor direction reversed");
    const uint8_t bf[] = {3, 2, 10, 7, 10, 18, 6, 9, 11, 12, 13, 17, 22, 26}; int bad = 0;
    for (uint8_t l = 0; l < sizeof(bf); l++) { s.motors.layout = l; r.ask(msp::MIXER_CONFIG); if (r.out.size() != 2 || r.out[0] != bf[l]) bad++; }
    CHECK(bad == 0, "every layout as Betaflight's mixer (%d wrong)", bad);
    s.motors.layout = wbp::LAYOUT_HEX_X; r.ask(msp::SET_MIXER_CONFIG, {10, 0}); CHECK(r.dir == '>' && s.motors.layout == wbp::LAYOUT_HEX_X && !s.motors.props_out && r.port.commits.back() == wbp::SEC_MOTORS, "HEX6X written back on HEX_X: kept; props in");
    s.motors.layout = wbp::LAYOUT_QUAD_X; r.ask(msp::SET_MIXER_CONFIG, {10, 1}); CHECK(r.dir == '>' && s.motors.layout == wbp::LAYOUT_HEX_X_BF && s.motors.props_out, "HEX6X from a quad: Betaflight's hex order; props out");
    r.ask(msp::SET_MIXER_CONFIG, {26}); CHECK(r.dir == '>' && s.motors.layout == wbp::LAYOUT_QUAD_X_1234 && s.motors.props_out, "QUADX1234; no second byte: props kept");
    r.ask(msp::SET_MIXER_CONFIG, {1, 0}); CHECK(r.dir == '!' && s.motors.layout == wbp::LAYOUT_QUAD_X_1234, "TRI (1): no such frame here, refused");
    r.port.is_armed = true; r.ask(msp::SET_MIXER_CONFIG, {3, 0}); CHECK(r.dir == '!' && s.motors.layout == wbp::LAYOUT_QUAD_X_1234, "armed: refused");
}

// ---- 0.44: the Betaflight App's board alignment wizard (BoardAlignmentWizardDialog.vue, src/js/utils/boardAlignment.ts),
// ported here: flat, nose down, right side down -> the board alignment it writes. The board is mounted wrongly in many ways; after
// the App's SET the firmware's IMU (the same steps as imu.read: chip orientation, then the board + level trims) reads as level.
namespace bfwiz {
typedef double M[3][3];
static void euler(M m, double r, double p, double y) {                  // magAlignment.ts eulerToMatrix (yaw counter-clockwise positive)
    const double d = M_PI / 180, cr = cos(r * d), sr = sin(r * d), cp = cos(p * d), sp = sin(p * d), cy = cos(y * d), sy = sin(y * d);
    const M o = {{cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr}, {sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr}, {-sp, cp * sr, cp * cr}};
    memcpy(m, o, sizeof(M));
}
static void mul(M o, const M a, const M b) { M t; for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) t[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j]; memcpy(o, t, sizeof(M)); }
static double dot(const double* a, const double* b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
static void norm(double* v) { const double m = sqrt(dot(v, v)); if (m > 1e-9) for (int k = 0; k < 3; k++) v[k] /= m; }
static void perp(double* o, const double* v, const double* ax) { const double d = dot(v, ax); for (int k = 0; k < 3; k++) o[k] = v[k] - ax[k] * d; }
static double snap45(double d) { double s = round(d / 45) * 45; while (s > 180) s -= 360; while (s <= -180) s += 360; return s == 0 ? 0 : s; }
// detectBoardAlignment: false on a gesture it refuses; out = roll, pitch, yaw (Betaflight's: yaw clockwise, 0..359)
static bool detect(const double* flat, const double* pit, const double* rol, const int cur[3], int out[3]) {
    double up[3] = {flat[0], flat[1], flat[2]}; norm(up);
    double hp[3], hr[3]; perp(hp, pit, up); perp(hr, rol, up);
    if (sqrt(dot(hp, hp)) < 0.05 || sqrt(dot(hr, hr)) < 0.05) return false;
    double fr[3] = {-hp[0], -hp[1], -hp[2]}; norm(fr); double rr[3] = {hr[0], hr[1], hr[2]}; norm(rr);
    double fw[3]; perp(fw, fr, up); norm(fw);
    const double rt[3] = {up[1] * fw[2] - up[2] * fw[1], up[2] * fw[0] - up[0] * fw[2], up[0] * fw[1] - up[1] * fw[0]};
    if (dot(rt, rr) < 0) return false;
    M m = {{fw[0], fw[1], fw[2]}, {rt[0], rt[1], rt[2]}, {up[0], up[1], up[2]}}, c; euler(c, cur[0], cur[1], -cur[2]); mul(m, m, c);
    const double sp = fmax(-1.0, fmin(1.0, -m[2][0])), pitch = asin(sp) * 180 / M_PI;
    double roll, yaw;
    if (fabs(m[2][0]) < 0.99999) { roll = atan2(m[2][1], m[2][2]) * 180 / M_PI; yaw = atan2(m[1][0], m[0][0]) * 180 / M_PI; }
    else { roll = atan2(-m[0][1], m[1][1]) * 180 / M_PI; yaw = 0; }
    out[0] = (int)snap45(roll); out[1] = (int)snap45(pitch); out[2] = (((int)-snap45(yaw) % 360) + 360) % 360;
    return true;
}
}
// The firmware's IMU in this firmware's frame (imu.read: align_chip, then the fine rotation of the board + level trims)
static void fw_imu(const wbp::Settings& s, const double chip[3], float out[3]) {
    float v[3] = {(float)chip[0], (float)chip[1], (float)chip[2]};
    accgyro::align_chip(v, (AccGyroAlign)s.general.board_align);
    float r, p, y; wbp::board_fine_alignment(s, r, p, y);
    if (r != 0 || p != 0 || y != 0) { float m[3][3]; accgyro::build_rotation(m, r, p, y); accgyro::apply_rotation(v, m); }
    for (int k = 0; k < 3; k++) out[k] = v[k];
}
// The chip's reading that makes a TRUE mounting (settings `tr`) read `body` (Betaflight's frame) - the pipeline inverted
static void chip_of(const wbp::Settings& tr, const double body[3], double chip[3]) {
    double m[3][3];                                                       // columns: the pipeline's image of each chip axis
    for (int i = 0; i < 3; i++) { double e[3] = {0, 0, 0}; e[i] = 1; float o[3]; fw_imu(tr, e, o); for (int k = 0; k < 3; k++) m[k][i] = o[k]; }
    const double frd[3] = {body[0], -body[1], -body[2]};
    for (int i = 0; i < 3; i++) chip[i] = m[0][i] * frd[0] + m[1][i] * frd[1] + m[2][i] * frd[2];   // (orthonormal: the inverse is the transpose)
}
// RAW_IMU for one chip reading, in g (Betaflight's frame)
static void raw_imu(Rig& r, const double chip[3], double g[3]) {
    fw_imu(r.s, chip, r.t.acc); r.ask(msp::RAW_IMU);
    for (int k = 0; k < 3; k++) g[k] = r.i16(2 * k) / 2048.0;
}

static void t_board_wizard() {
    printf("[board wizard] the Betaflight App's board alignment wizard\n");
    const double s45 = sqrt(0.5);
    const double flat[3] = {0, 0, 1}, nose[3] = {-s45, 0, s45}, right[3] = {0, s45, s45};    // the App's gestures (its test's model)
    struct Case { const char* what; uint8_t align; int16_t tr, tp, ty; uint8_t start_align; int16_t sr, sp, sy; };
    const Case cases[] = {
        {"chip turned 90 deg clockwise, set as CW0", wbp::ALIGN_CW90, 0, 0, 0, wbp::ALIGN_CW0, 0, 0, 0},
        {"chip on the underside (flip), set as CW0", wbp::ALIGN_CW180_FLIP, 0, 0, 0, wbp::ALIGN_CW0, 0, 0, 0},
        {"board turned 45 deg on the frame", wbp::ALIGN_CW0, 0, 0, 450, wbp::ALIGN_CW0, 0, 0, 0},
        {"chip CW270 + board 45 deg, set as CW90", wbp::ALIGN_CW270, 0, 0, -450, wbp::ALIGN_CW90, 0, 0, 0},
        {"upside down and turned, set wrongly with trims", wbp::ALIGN_CW90_FLIP, 0, 0, 0, wbp::ALIGN_CW180, 30, -20, 900},
        {"already right: nothing changes", wbp::ALIGN_CW270, 0, 0, 0, wbp::ALIGN_CW270, 0, 0, 0},
    };
    for (const Case& c : cases) {
        wbp::Settings truth; wbp::defaults(truth); truth.general.board_align = c.align; truth.general.board_trim_roll = c.tr; truth.general.board_trim_pitch = c.tp; truth.general.board_trim_yaw = c.ty;
        Rig r; wbp::Settings& s = r.s;
        s.general.board_align = c.start_align; s.general.board_trim_roll = c.sr; s.general.board_trim_pitch = c.sp; s.general.board_trim_yaw = c.sy;
        s.calibration.acc_trim_roll = 15; s.calibration.acc_trim_pitch = -10; truth.calibration = s.calibration;     // a level trim of a degree or two: kept
        double cf[3], cn[3], cr[3]; chip_of(truth, flat, cf); chip_of(truth, nose, cn); chip_of(truth, right, cr);
        double gf[3], gn[3], gr[3]; raw_imu(r, cf, gf); raw_imu(r, cn, gn); raw_imu(r, cr, gr);
        r.ask(msp::BOARD_ALIGNMENT_CONFIG); const int cur[3] = {r.i16(0), r.i16(2), r.i16(4)};
        int res[3] = {0, 0, 0}; const bool ok = bfwiz::detect(gf, gn, gr, cur, res);
        {   // the firmware's own copy (CMD_BOARD_ALIGN): the same answer from the same three readings
            const float ff[3] = {(float)gf[0], (float)gf[1], (float)gf[2]}, fn[3] = {(float)gn[0], (float)gn[1], (float)gn[2]}, fr[3] = {(float)gr[0], (float)gr[1], (float)gr[2]};
            float fcur[3]; fc::balign::bf_of(s.general.board_align, s.general.board_trim_roll * 0.1f, s.general.board_trim_pitch * 0.1f, s.general.board_trim_yaw * 0.1f, fcur);
            int mine[3] = {9, 9, 9}; const uint8_t why = fc::balign::detect(ff, fn, fr, fcur, mine);
            CHECK(why == fc::balign::DET_OK && mine[0] == res[0] && mine[1] == res[1] && ((mine[2] - res[2]) % 360 + 360) % 360 == 0,
                  "%s: the firmware's detect() finds what the App's does (%d %d %d / %d %d %d)", c.what, mine[0], mine[1], mine[2], res[0], res[1], res[2]);
        }
        r.ask(msp::SET_BOARD_ALIGNMENT_CONFIG, le16({res[0], res[1], res[2]}));
        double af[3], an[3], ar[3]; raw_imu(r, cf, af); raw_imu(r, cn, an); raw_imu(r, cr, ar);
        double err = 0; for (int k = 0; k < 3; k++) err = fmax(err, fmax(fabs(af[k] - flat[k]), fmax(fabs(an[k] - nose[k]), fabs(ar[k] - right[k]))));
        CHECK(ok && r.dir == '>' && err < 0.06 && s.calibration.acc_trim_roll == 15,
              "%s: the App writes %d / %d / %d -> board_align %d, trims %d %d %d; flat / nose down / right down read right (error %.3f g)",
              c.what, res[0], res[1], res[2], s.general.board_align, s.general.board_trim_roll, s.general.board_trim_pitch, s.general.board_trim_yaw, err);
        r.ask(msp::BOARD_ALIGNMENT_CONFIG); const int back[3] = {r.i16(0), r.i16(2), ((r.i16(4) % 360) + 360) % 360};
        int again[3]; bfwiz::detect(af, an, ar, back, again);
        CHECK(again[0] == back[0] && again[1] == back[1] && ((again[2] - back[2]) % 360 == 0), "%s: a second run finds nothing to change (%d %d %d / %d %d %d)", c.what, again[0], again[1], again[2], back[0], back[1], back[2]);
    }
    Rig r; wbp::Settings& s = r.s;
    r.ask(msp::SET_BOARD_ALIGNMENT_CONFIG, le16({0, 0, 90})); r.ask(msp::BOARD_ALIGNMENT_CONFIG);
    CHECK(r.i16(0) == 0 && r.i16(2) == 0 && (r.i16(4) + 360) % 360 == 90, "yaw 90 written, 90 read back (%d %d %d; board_align %d)", r.i16(0), r.i16(2), r.i16(4), s.general.board_align);
    const uint8_t a0 = s.general.board_align; const int16_t y0 = s.general.board_trim_yaw;
    r.ask(msp::SET_BOARD_ALIGNMENT_CONFIG, le16({0, 90, 0})); CHECK(r.dir == '!' && s.general.board_align == a0 && s.general.board_trim_yaw == y0, "pitch 90 (the board standing up): no chip orientation + trims make it, refused");
    r.ask(msp::SET_BOARD_ALIGNMENT_CONFIG, le16({60, 0, 0})); CHECK(r.dir == '!', "roll 60: refused");
    r.ask(msp::SET_BOARD_ALIGNMENT_CONFIG, le16({180, 0, 0})); CHECK(r.dir == '>', "roll 180 (upside down): a flipped chip orientation");
    // the gestures the App refuses (boardAlignment.ts errors), in the firmware's detect()
    {
        using namespace fc::balign; const float h = 0.70710678f;
        const float flat[3] = {0, 0, 1}, zero[3] = {0, 0, 0}, nose[3] = {-h, 0, h}, right[3] = {0, h, h}, left[3] = {0, -h, h}, cur[3] = {0, 0, 0}; int o[3];
        CHECK(detect(zero, nose, right, cur, o) == DET_NO_GRAVITY, "no accelerometer reading: NO_GRAVITY");
        CHECK(detect(flat, flat, right, cur, o) == DET_NO_PITCH, "the nose not put down: NO_PITCH");
        CHECK(detect(flat, nose, flat, cur, o) == DET_NO_ROLL, "the right side not put down: NO_ROLL");
        CHECK(detect(flat, nose, left, cur, o) == DET_DIRECTION, "the left side put down instead: DIRECTION (one of the two gestures was the wrong way)");
        CHECK(detect(flat, nose, right, cur, o) == DET_OK && o[0] == 0 && o[1] == 0 && o[2] == 0, "all three right on a right board: 0 / 0 / 0");
    }
    // a second IMU on the same board turns with the first (its own orientation: D(b') = D(new) D(old)^T D(b))
    CHECK(fc::balign::follow(wbp::ALIGN_CW90, wbp::ALIGN_CW0, wbp::ALIGN_CW0) == wbp::ALIGN_CW90, "follow: the board found turned 90: a CW0 second chip becomes CW90");
    CHECK(fc::balign::follow(wbp::ALIGN_CW0, wbp::ALIGN_CW0, wbp::ALIGN_CW270_FLIP) == wbp::ALIGN_CW270_FLIP, "follow: nothing changed, nothing changes");
    CHECK(fc::balign::follow(wbp::ALIGN_CW180, wbp::ALIGN_CW0, wbp::ALIGN_CW90) == wbp::ALIGN_CW270, "follow: 180 more on a CW90 chip: CW270");
    CHECK(fc::balign::follow(wbp::ALIGN_CW0_FLIP, wbp::ALIGN_CW0, wbp::ALIGN_CW0_FLIP) == wbp::ALIGN_CW0, "follow: flipped twice: CW0");
}

// 0.44: the Betaflight App's Sensors tab (useSensorsData.ts): what is set up (SENSOR_CONFIG), what runs (MSP2_SENSOR_CONFIG_ACTIVE,
// MSP2_GYRO_SENSOR_ACTIVE), the switches written back (SET_SENSOR_CONFIG). Betaflight's tables: acc / baro / mag DEFAULT 0 NONE 1,
// rangefinder / flow NONE 0, pitot DEFAULT 0 NONE 1; MAG_HMC5883 2, AK8963 4, QMC5883L 5, QMC5883P 11, DRONECAN 12.
static void t_sensors_tab() {
    printf("[sensors tab] SENSOR_CONFIG, SET_SENSOR_CONFIG, MSP2_SENSOR_CONFIG_ACTIVE, MSP2_GYRO_SENSOR_ACTIVE\n");
    Rig r; wbp::Settings& s = r.s;
    s.sensors.baro_enabled = 1; s.sensors.mag_enabled = 1; s.sensors.mag_model = wbp::MAG_MODEL_QMC5883L; s.sensors.range_enabled = 1; s.flow.type = wbp::FLOW_TYPE_MSP;
    r.ask(msp::SENSOR_CONFIG);
    CHECK(r.out.size() == 6 && r.out[0] == 0 && r.out[1] == 0 && r.out[2] == 5 && r.out[3] != 0 && r.out[4] != 0 && r.out[5] == 1,
          "set up: acc DEFAULT, baro DEFAULT, mag QMC5883L (5), a rangefinder, a flow sensor, pitot NONE (1) (%u %u %u %u %u %u)", r.out[0], r.out[1], r.out[2], r.out[3], r.out[4], r.out[5]);
    r.ask(msp::SET_SENSOR_CONFIG, r.out);
    CHECK(r.dir == '>' && s.sensors.baro_enabled && s.sensors.mag_enabled && s.sensors.mag_model == wbp::MAG_MODEL_QMC5883L && s.sensors.range_enabled && s.flow.type == wbp::FLOW_TYPE_MSP,
          "written back as read: nothing changes");
    s.sensors.mag_model = wbp::MAG_MODEL_MSP; r.ask(msp::SENSOR_CONFIG); const uint8_t msp_mag = r.out[2];
    r.ask(msp::SET_SENSOR_CONFIG, r.out); CHECK(msp_mag == 0 && s.sensors.mag_model == wbp::MAG_MODEL_MSP, "an MSP compass (none in Betaflight's list) reads DEFAULT and stays MSP when written back");
    r.ask(msp::SET_SENSOR_CONFIG, {0, 1, 1, 0, 0, 1});
    bool sens = false, flow = false; for (uint8_t c : r.port.commits) { sens |= c == wbp::SEC_SENSORS; flow |= c == wbp::SEC_FLOW; }
    CHECK(r.dir == '>' && !s.sensors.baro_enabled && !s.sensors.mag_enabled && !s.sensors.range_enabled && s.flow.type == wbp::FLOW_TYPE_NONE && sens && flow,
          "all switched off in the App: baro, compass, rangefinder off, flow NONE (sensors, flow committed)");
    r.ask(msp::SENSOR_CONFIG); CHECK(r.out[1] == 1 && r.out[2] == 1 && r.out[3] == 0 && r.out[4] == 0, "and read so (NONE)");
    r.ask(msp::SET_SENSOR_CONFIG, {0, 0, 11, 0, 0, 1});
    CHECK(s.sensors.baro_enabled && s.sensors.mag_enabled && s.sensors.mag_model == wbp::MAG_MODEL_QMC5883P, "baro on, compass QMC5883P (11) picked in the App");
    r.ask(msp::SET_SENSOR_CONFIG, {0, 0, 0, 0, 0, 1}); CHECK(s.sensors.mag_model == wbp::MAG_MODEL_AUTO, "compass DEFAULT: AUTO");
    r.ask(msp::SET_SENSOR_CONFIG, {0, 0, 0}); CHECK(r.dir == '>', "the 3-byte form of an older App");
    r.port.is_armed = true; r.ask(msp::SET_SENSOR_CONFIG, {0, 1, 1}); CHECK(r.dir == '!' && s.sensors.baro_enabled, "armed: refused"); r.port.is_armed = false;
    // what runs
    s.sensors.mag_model = wbp::MAG_MODEL_HMC5883L;
    r.t.health = wbp::HEALTH_IMU | wbp::HEALTH_MAG_READY; r.ask2(msp::SENSOR_CONFIG_ACTIVE);
    CHECK(r.out.size() == 7 && r.out[0] == 1 && r.out[1] == 0 && r.out[2] == 1 && r.out[3] == 2 && r.out[4] == 0 && r.out[5] == 0 && r.out[6] == 0xFF,
          "SENSOR_CONFIG_ACTIVE: gyro DEFAULT, acc DEFAULT, no baro (NONE), compass HMC5883 (2), no rangefinder / flow, pitot not built (%u %u %u %u %u %u %u)",
          r.out.size() > 6 ? r.out[0] : 0, r.out.size() > 6 ? r.out[1] : 0, r.out.size() > 6 ? r.out[2] : 0, r.out.size() > 6 ? r.out[3] : 0, r.out.size() > 6 ? r.out[4] : 0, r.out.size() > 6 ? r.out[5] : 0, r.out.size() > 6 ? r.out[6] : 0);
    r.t.health = wbp::HEALTH_IMU | wbp::HEALTH_BARO | wbp::HEALTH_RANGE | wbp::HEALTH_FLOW; r.ask2(msp::SENSOR_CONFIG_ACTIVE);
    CHECK(r.out[2] == 0 && r.out[3] == 1 && r.out[4] != 0 && r.out[5] != 0, "baro running (DEFAULT), compass not (NONE), rangefinder and flow running");
    r.t.health = 0; r.ask2(msp::SENSOR_CONFIG_ACTIVE); CHECK(r.out[0] == 0 && r.out[1] == 1, "no IMU: gyro NONE (0), acc NONE (1)");
    r.t.health = wbp::HEALTH_IMU; r.ask2(msp::GYRO_SENSOR_ACTIVE);
    CHECK(r.out.size() == 3 && r.out[0] == 2 && r.out[1] == 1 && r.out[2] == 0, "GYRO_SENSOR_ACTIVE: two gyro places, the first found, no second");
    r.t.health = wbp::HEALTH_IMU | wbp::HEALTH_IMU2; r.ask2(msp::GYRO_SENSOR_ACTIVE); CHECK(r.out[1] == 1 && r.out[2] == 1, "a second IMU: found too");
    r.ask(msp::SENSOR_ALIGNMENT); CHECK(r.out[3] == 3, "SENSOR_ALIGNMENT: both gyros detected (flags 3)");
    r.t.health = wbp::HEALTH_IMU; r.ask(msp::SENSOR_ALIGNMENT); CHECK(r.out[3] == 1, "... one (flags 1)");
}

// 0.44: the PID tab's controller settings (MSP_PID_ADVANCED, master's 61-byte layout): feedforward, I-term relax, anti-gravity,
// D max, throttle boost, the angle limit, TPA, motor output limit, dynamic idle
static void t_pid_advanced() {
    printf("[pid advanced] PID_ADVANCED / SET_PID_ADVANCED\n");
    Rig r; wbp::Settings& s = r.s; wbp::S_pid& p = s.pid;
    p.ff_transition = 30; p.f[0] = 120; p.f[1] = 130; p.f[2] = 140; p.ff_averaging = wbp::FF_AVERAGING_3_POINT; p.ff_smooth_factor = 40; p.ff_boost = 18; p.ff_max_rate_limit = 95; p.ff_jitter_factor = 9;
    p.iterm_relax = wbp::ITERM_RELAX_RPY; p.iterm_relax_type = wbp::ITERM_RELAX_TYPE_GYRO; p.iterm_relax_cutoff = 12;
    p.anti_gravity = 1; p.anti_gravity_gain = 90; p.d_max[0] = 41; p.d_max[1] = 43; p.d_max[2] = 5; p.d_max_gain = 33; p.d_max_advance = 21;
    p.throttle_boost = 6; p.acro_trainer_angle_limit = 25; s.rates.angle_limit = 55; s.motors.output_limit = 90; s.motors.dyn_idle_min_rpm = 30;
    p.tpa_mode = wbp::TPA_MODE_D; p.tpa_rate = 65; p.tpa_breakpoint = 35;
    r.ask(msp::PID_ADVANCED);
    CHECK(r.dir == '>' && r.out.size() == 61, "61 bytes (%u)", (unsigned)r.out.size());
    CHECK(r.out[8] == 30 && r.out[17] == 55 && r.u16(21) == 90 && r.out[27] == wbp::ITERM_RELAX_RPY && r.out[28] == 0 && r.out[30] == 6 && r.out[31] == 25,
          "ff_transition 30, angle_limit 55, anti_gravity_gain 90, iterm_relax RPY / GYRO, throttle_boost 6, acro trainer 25");
    CHECK(r.u16(32) == 120 && r.u16(34) == 130 && r.u16(36) == 140 && r.out[39] == 41 && r.out[40] == 43 && r.out[41] == 5 && r.out[42] == 33 && r.out[43] == 21,
          "F 120 / 130 / 140, d_max 41 / 43 / 5, d_max gain 33, advance 21");
    CHECK(r.out[46] == 12 && r.out[47] == 90 && r.out[49] == 30 && r.out[50] == 2 && r.out[51] == 40 && r.out[52] == 18 && r.out[53] == 95 && r.out[54] == 9,
          "relax cutoff 12, output limit 90, dyn idle 30 (x100 rpm), ff averaging 3 points (2), smooth 40, boost 18, max rate 95, jitter 9");
    CHECK(r.out[57] == 1 && r.out[58] == 65 && r.u16(59) == 1350, "TPA: mode D (1), rate 65 %%, breakpoint 35 %% = 1350 us (%d)", r.u16(59));
    const std::vector<uint8_t> was = r.out;
    p.anti_gravity = 0; r.ask(msp::PID_ADVANCED); CHECK(r.u16(21) == 0, "anti-gravity off: gain 0 (Betaflight's off) (%d)", r.u16(21)); p.anti_gravity = 1;
    p.slider_mode = wbp::SLIDER_MODE_RP; r.ask(msp::SET_PID_ADVANCED, was);
    CHECK(r.dir == '>' && p.slider_mode == wbp::SLIDER_MODE_RP && p.tpa_breakpoint == 35 && p.anti_gravity && p.f[2] == 140 && s.rates.angle_limit == 55, "written back as read: nothing changes, the sliders stay");
    std::vector<uint8_t> w = was;
    w[8] = 50; w[17] = 45; w[21] = 0; w[22] = 0; w[27] = wbp::ITERM_RELAX_RP_INC; w[28] = 1; w[30] = 10; w[31] = 30;
    w[32] = 200; w[33] = 0; w[39] = 50; w[46] = 15; w[47] = 80; w[49] = 25; w[50] = 0; w[52] = 20; w[57] = 0; w[58] = 70; w[59] = (uint8_t)(1450 & 0xFF); w[60] = (uint8_t)(1450 >> 8);
    r.ask(msp::SET_PID_ADVANCED, w);
    CHECK(r.dir == '>' && p.ff_transition == 50 && s.rates.angle_limit == 45 && !p.anti_gravity && p.iterm_relax == wbp::ITERM_RELAX_RP_INC && p.iterm_relax_type == wbp::ITERM_RELAX_TYPE_SETPOINT
          && p.throttle_boost == 10 && p.acro_trainer_angle_limit == 30, "SET: ff transition, angle limit, anti-gravity 0 = off, relax RP_INC / SETPOINT, boost, trainer");
    CHECK(p.f[0] == 200 && p.d_max[0] == 50 && p.iterm_relax_cutoff == 15 && s.motors.output_limit == 80 && s.motors.dyn_idle_min_rpm == 25 && p.ff_averaging == wbp::FF_AVERAGING_OFF && p.ff_boost == 20,
          "F roll 200, d_max roll 50, relax cutoff 15, output limit 80, dyn idle 25, averaging off, ff boost 20");
    CHECK(p.tpa_mode == wbp::TPA_MODE_PD && p.tpa_rate == 70 && p.tpa_breakpoint == 45, "TPA: PD, 70 %%, 1450 us = 45 %% (%u)", p.tpa_breakpoint);
    CHECK(p.slider_mode == wbp::SLIDER_MODE_OFF, "F / D max typed: the sliders let go (as SET_PID)");
    bool pid = false, mot = false, rat = false; for (uint8_t c : r.port.commits) { pid |= c == wbp::SEC_PID; mot |= c == wbp::SEC_MOTORS; rat |= c == wbp::SEC_RATES; }
    CHECK(pid && mot && rat, "pid, motors, rates committed");
    w[21] = 70; r.ask(msp::SET_PID_ADVANCED, w); CHECK(p.anti_gravity && p.anti_gravity_gain == 70, "anti-gravity gain 70: on");
    w[59] = (uint8_t)(900 & 0xFF); w[60] = (uint8_t)(900 >> 8); r.ask(msp::SET_PID_ADVANCED, w); CHECK(p.tpa_breakpoint == 0, "breakpoint below 1000 us: 0 %%");
    r.ask(msp::SET_PID_ADVANCED, std::vector<uint8_t>(was.begin(), was.begin() + 40)); CHECK(r.dir == '!', "a short frame: refused");
    r.port.is_armed = true; r.ask(msp::SET_PID_ADVANCED, was); CHECK(r.dir == '!', "armed: refused");
}

// 0.44: the Filter tab (MSP_FILTER_CONFIG, master's 56-byte layout); LPF types PT1 0, BIQUAD 1, PT2 2, PT3 3 as Betaflight's
static void t_filter_config() {
    printf("[filters] FILTER_CONFIG / SET_FILTER_CONFIG\n");
    Rig r; wbp::S_filters& f = r.s.filters;
    f.gyro_lpf1_type = wbp::LPF_TYPE_PT1; f.gyro_lpf1_static_hz = 250; f.gyro_lpf1_dyn_min_hz = 200; f.gyro_lpf1_dyn_max_hz = 450; f.gyro_lpf2_type = wbp::LPF_TYPE_PT2; f.gyro_lpf2_hz = 480;
    f.gyro_notch1_hz = 300; f.gyro_notch1_cutoff = 200; f.gyro_notch2_hz = 310; f.gyro_notch2_cutoff = 210;
    f.dterm_lpf1_type = wbp::LPF_TYPE_PT3; f.dterm_lpf1_static_hz = 90; f.dterm_lpf1_dyn_min_hz = 70; f.dterm_lpf1_dyn_max_hz = 160; f.dterm_lpf1_dyn_expo = 6;
    f.dterm_lpf2_type = wbp::LPF_TYPE_BIQUAD; f.dterm_lpf2_hz = 150; f.dterm_notch_hz = 260; f.dterm_notch_cutoff = 160; f.yaw_lowpass_hz = 100;
    f.dyn_notch_count = 3; f.dyn_notch_q = 400; f.dyn_notch_min_hz = 120; f.dyn_notch_max_hz = 600;
    f.rpm_harmonics = 3; f.rpm_min_hz = 100; f.rpm_fade_range_hz = 50; f.rpm_q = 500; f.rpm_weights[0] = 100; f.rpm_weights[1] = 60; f.rpm_weights[2] = 30;
    f.slider_gyro = 1; f.slider_dterm = 1;
    r.ask(msp::FILTER_CONFIG);
    CHECK(r.dir == '>' && r.out.size() == 56, "56 bytes (%u)", (unsigned)r.out.size());
    CHECK(r.out[0] == 250 && r.u16(1) == 90 && r.u16(3) == 100 && r.u16(5) == 300 && r.u16(7) == 200 && r.u16(9) == 260 && r.u16(11) == 160 && r.u16(13) == 310 && r.u16(15) == 210 && r.out[17] == 3,
          "gyro LPF1 (byte), D LPF1 90, yaw LPF 100, notch 1 300 / 200, D notch 260 / 160, notch 2 310 / 210, D LPF1 PT3");
    CHECK(r.u16(20) == 250 && r.u16(22) == 480 && r.out[24] == 0 && r.out[25] == 2 && r.u16(26) == 150 && r.out[28] == 1 && r.u16(29) == 200 && r.u16(31) == 450 && r.u16(33) == 70 && r.u16(35) == 160,
          "gyro LPF1 250 PT1, LPF2 480 PT2, D LPF2 150 BIQUAD, dynamic gyro 200..450, dynamic D 70..160");
    CHECK(r.u16(39) == 400 && r.u16(41) == 120 && r.out[43] == 3 && r.out[44] == 100 && r.u16(45) == 600 && r.out[47] == 6 && r.out[48] == 3 && r.u16(49) == 50 && r.u16(51) == 500
          && r.out[53] == 100 && r.out[54] == 60 && r.out[55] == 30, "dyn notch Q 400, 120..600 Hz, 3 notches; RPM 3 harmonics, 100 Hz, fade 50, Q 500, weights 100 / 60 / 30; D dyn expo 6");
    const std::vector<uint8_t> was = r.out;
    r.ask(msp::SET_FILTER_CONFIG, was);
    CHECK(r.dir == '>' && f.slider_gyro && f.slider_dterm && f.gyro_lpf1_static_hz == 250 && f.rpm_weights[2] == 30 && r.port.commits.back() == wbp::SEC_FILTERS, "written back as read: nothing changes, the sliders stay");
    std::vector<uint8_t> w = was;
    w[20] = 200; w[21] = 0; w[22] = (uint8_t)500; w[23] = 500 >> 8; w[24] = 2; w[5] = 0; w[6] = 0; w[43] = 2; w[49] = 40; w[55] = 20; w[45] = 0x58; w[46] = 0x02;
    r.ask(msp::SET_FILTER_CONFIG, w);
    CHECK(r.dir == '>' && f.gyro_lpf1_static_hz == 200 && f.gyro_lpf2_hz == 500 && f.gyro_lpf1_type == wbp::LPF_TYPE_PT2 && f.gyro_notch1_hz == 0 && f.rpm_harmonics == 2 && f.rpm_fade_range_hz == 40
          && f.rpm_weights[2] == 20 && f.dyn_notch_max_hz == 600, "gyro LPF1 200 PT2, LPF2 500, notch 1 off, RPM 2 harmonics, fade 40, third weight 20");
    CHECK(!f.slider_gyro && f.slider_dterm, "gyro values typed: the gyro slider lets go, the D-term one stays");
    w = was; w[1] = 80; r.ask(msp::SET_FILTER_CONFIG, w); CHECK(f.dterm_lpf1_static_hz == 80 && !f.slider_dterm, "D-term LPF typed: the D-term slider lets go too");
    r.ask(msp::SET_FILTER_CONFIG, std::vector<uint8_t>(was.begin(), was.begin() + 30)); CHECK(r.dir == '!', "a short frame: refused");
    r.port.is_armed = true; r.ask(msp::SET_FILTER_CONFIG, was); CHECK(r.dir == '!', "armed: refused");
}

// 0.44: the GPS tab (useGpsData.ts): GPS_CONFIG / SET_GPS_CONFIG (Betaflight's provider and SBAS tables are this firmware's), GPSSVINFO
static void t_gps_tab() {
    printf("[gps tab] GPS_CONFIG, SET_GPS_CONFIG, GPSSVINFO\n");
    Rig r; wbp::S_gps& g = r.s.gps;
    g.provider = wbp::GPS_PROVIDER_UBLOX; g.sbas = wbp::SBAS_EGNOS; g.auto_config = 1; g.auto_baud = 0; g.set_home_once = 1; g.use_galileo = 1;
    r.ask(msp::GPS_CONFIG);
    CHECK(r.dir == '>' && r.out.size() == 6 && r.out[0] == 1 && r.out[1] == 1 && r.out[2] == 1 && r.out[3] == 0 && r.out[4] == 1 && r.out[5] == 1, "UBLOX, EGNOS, auto config, no auto baud, home once, Galileo");
    r.ask(msp::SET_GPS_CONFIG, {4, 5, 0, 1, 0, 0});
    CHECK(r.dir == '>' && g.provider == wbp::GPS_PROVIDER_DRONECAN && g.sbas == wbp::SBAS_NONE && !g.auto_config && g.auto_baud && !g.set_home_once && !g.use_galileo && r.port.commits.back() == wbp::SEC_GPS,
          "SET: DroneCAN, SBAS none, the switches");
    r.ask(msp::SET_GPS_CONFIG, {1, 2, 1, 1}); CHECK(r.dir == '>' && g.provider == wbp::GPS_PROVIDER_UBLOX && g.sbas == wbp::SBAS_WAAS && !g.use_galileo, "the 4-byte form: home once / Galileo kept");
    r.ask(msp::SET_GPS_CONFIG, {9, 0, 0, 0}); CHECK(r.dir == '!' && g.provider == wbp::GPS_PROVIDER_UBLOX, "a provider this firmware has not: refused");
    r.ask(msp::SET_GPS_CONFIG, {1, 9, 0, 0}); CHECK(r.dir == '!', "an SBAS value out of the table: refused");
    r.port.is_armed = true; r.ask(msp::SET_GPS_CONFIG, {0, 0, 0, 0}); CHECK(r.dir == '!' && g.provider == wbp::GPS_PROVIDER_UBLOX, "armed: refused"); r.port.is_armed = false;
    r.t.sat_n = 3;
    r.t.sat[0].gnss = 0; r.t.sat[0].sv = 12; r.t.sat[0].cno = 41; r.t.sat[0].flags = 0x0F;
    r.t.sat[1].gnss = 2; r.t.sat[1].sv = 7;  r.t.sat[1].cno = 33; r.t.sat[1].flags = 0x04;
    r.t.sat[2].gnss = 6; r.t.sat[2].sv = 3;  r.t.sat[2].cno = 0;  r.t.sat[2].flags = 0x01;
    r.ask(msp::GPSSVINFO);
    CHECK(r.dir == '>' && r.out.size() == 13 && r.out[0] == 3 && r.out[1] == 0 && r.out[2] == 12 && r.out[3] == 0x0F && r.out[4] == 41 && r.out[5] == 2 && r.out[6] == 7 && r.out[8] == 33 && r.out[9] == 6 && r.out[12] == 0,
          "GPSSVINFO: 3 satellites, each gnss id / sv / quality + used / C/N0 (as Betaflight's UBX NAV-SAT)");
    r.t.sat_n = 0; r.ask(msp::GPSSVINFO); CHECK(r.out.size() == 1 && r.out[0] == 0, "no satellites: 0");
}

// 0.44: the Power & Battery tab (usePower.ts): the battery's limits and meters, the voltage divider, the current sensor;
// Betaflight's meter ids: BATTERY_1 10; voltage sensor type ADC_RESISTOR_DIVIDER 0, current sensor ADC 1
static void t_power_tab() {
    printf("[power tab] SET_BATTERY_CONFIG, VOLTAGE / CURRENT_METERS, VOLTAGE / CURRENT_METER_CONFIG\n");
    Rig r; wbp::S_battery& b = r.s.battery;
    b.source = wbp::BATT_SOURCE_AUTO; b.current_source = wbp::CURR_SOURCE_DRONECAN; r.ask(msp::BATTERY_CONFIG); const std::vector<uint8_t> was = r.out;
    r.ask(msp::SET_BATTERY_CONFIG, was); CHECK(r.dir == '>' && b.source == wbp::BATT_SOURCE_AUTO && b.current_source == wbp::CURR_SOURCE_DRONECAN, "written back as read: AUTO / DroneCAN kept");
    b.source = wbp::BATT_SOURCE_NONE; b.current_source = wbp::CURR_SOURCE_NONE;
    std::vector<uint8_t> w = {32, 42, 35, 0xDC, 0x05, 1, 1, 0x40, 0x01, 0xA4, 0x01, 0x5E, 0x01};   // 320 / 420 / 350 (0.01 V), 1500 mAh, ADC / ADC
    r.ask(msp::SET_BATTERY_CONFIG, w);
    CHECK(r.dir == '>' && b.cell_min_mv == 3200 && b.cell_max_mv == 4200 && b.cell_warning_mv == 3500 && b.capacity_mah == 1500 && b.source == wbp::BATT_SOURCE_ADC && b.current_source == wbp::CURR_SOURCE_ADC
          && r.port.commits.back() == wbp::SEC_BATTERY, "SET: 3.20 / 4.20 / 3.50 V a cell, 1500 mAh, both from the ADC");
    r.ask(msp::SET_BATTERY_CONFIG, {33, 43, 35, 0, 0, 2, 3}); CHECK(r.dir == '>' && b.cell_min_mv == 3300 && b.source == wbp::BATT_SOURCE_ESC && b.current_source == wbp::CURR_SOURCE_ESC, "the 7-byte form (0.1 V); ESC meters");
    r.ask(msp::SET_BATTERY_CONFIG, {33, 43, 35, 0, 0, 0, 0}); CHECK(b.source == wbp::BATT_SOURCE_NONE && b.current_source == wbp::CURR_SOURCE_NONE, "NONE / NONE");
    r.ask(msp::SET_BATTERY_CONFIG, {33, 43, 35, 0, 0, 1, 2}); CHECK(r.dir == '!' && b.current_source == wbp::CURR_SOURCE_NONE, "a virtual current meter (none here): refused");
    r.ask(msp::SET_BATTERY_CONFIG, {36, 43, 35, 0, 0, 1, 0}); CHECK(r.dir == '!' && b.cell_min_mv == 3300, "min above warning: refused (Betaflight)");
    // meters
    r.t.batt_v = 16.47f; r.t.batt_amps = 12.345f; r.t.batt_mah = 523.4f;
    r.ask(msp::VOLTAGE_METERS); CHECK(r.out.size() == 2 && r.out[0] == 10 && r.out[1] == 165, "VOLTAGE_METERS: battery 1 (10), 16.5 V in 0.1 V (%u)", r.out.size() > 1 ? r.out[1] : 0);
    r.ask(msp::CURRENT_METERS); CHECK(r.out.size() == 5 && r.out[0] == 10 && r.u16(1) == 523 && r.u16(3) == 12345, "CURRENT_METERS: battery 1, 523 mAh, 12345 mA");
    r.t.batt_amps = -1.0f; r.ask(msp::CURRENT_METERS); CHECK(r.u16(3) == 0, "a negative current: 0 (Betaflight)");
    // the divider
    b.divider = 1100; r.ask(msp::VOLTAGE_METER_CONFIG);
    CHECK(r.out.size() == 7 && r.out[0] == 1 && r.out[1] == 5 && r.out[2] == 10 && r.out[3] == 0 && r.out[4] == 110 && r.out[5] == 10 && r.out[6] == 1, "VOLTAGE_METER_CONFIG: one ADC divider meter, scale 110 / 10 / 1 = 11.00");
    b.divider = 4400; r.ask(msp::VOLTAGE_METER_CONFIG); CHECK(r.out[4] == 44 && r.out[5] == 1 && r.out[6] == 1, "a 44:1 divider (above 25.5): scale 44 / 1 / 1");
    r.ask(msp::SET_VOLTAGE_METER_CONFIG, {10, 105, 10, 1}); CHECK(r.dir == '>' && b.divider == 1050 && r.port.commits.back() == wbp::SEC_BATTERY, "SET: 105 / 10 / 1 -> 10.50");
    r.ask(msp::SET_VOLTAGE_METER_CONFIG, {10, 200, 10, 2}); CHECK(b.divider == 1000, "200 / 10 / 2 -> 10.00");
    r.ask(msp::SET_VOLTAGE_METER_CONFIG, {10, 110, 0, 1}); CHECK(r.dir == '!' && b.divider == 1000, "a divider of 0: refused");
    r.ask(msp::SET_VOLTAGE_METER_CONFIG, {11, 110, 10, 1}); CHECK(r.dir == '!', "another meter id: refused");
    // the current sensor
    b.current_scale = 400; b.current_offset = -120; r.ask(msp::CURRENT_METER_CONFIG);
    CHECK(r.out.size() == 8 && r.out[0] == 1 && r.out[1] == 6 && r.out[2] == 10 && r.out[3] == 1 && r.i16(4) == 400 && r.i16(6) == -120, "CURRENT_METER_CONFIG: one ADC meter, scale 400, offset -120");
    r.ask(msp::SET_CURRENT_METER_CONFIG, {10, 0x2C, 0x01, 0x32, 0x00}); CHECK(r.dir == '>' && b.current_scale == 300 && b.current_offset == 50, "SET: scale 300, offset 50");
    r.ask(msp::SET_CURRENT_METER_CONFIG, {80, 1, 0, 0, 0}); CHECK(r.dir == '!', "the virtual meter (80): refused");
    r.port.is_armed = true; r.ask(msp::SET_CURRENT_METER_CONFIG, {10, 0x2C, 0x01, 0x32, 0x00}); CHECK(r.dir == '!', "armed: refused");
}

// 0.44: the Failsafe tab (useFailsafeData.ts): FAILSAFE_CONFIG, RXFAIL_CONFIG, RSSI_CONFIG, GPS_RESCUE (+ the two writes)
// Betaflight: procedure DROP 0, AUTO_LAND 1, GPS_RESCUE 2; rescue altitude MAX 0, FIXED 1, CURRENT 2; rxfail AUTO 0, HOLD 1, SET 2
static void t_failsafe_tab() {
    printf("[failsafe tab] FAILSAFE_CONFIG, RXFAIL_CONFIG, RSSI_CONFIG, GPS_RESCUE\n");
    Rig r; wbp::Settings& s = r.s; wbp::S_nav& v = s.nav;
    s.general.fs_guard_ms = 800; s.general.fs_max_descent_s = 40; s.general.fs_throttle = 1250; v.fs_procedure = wbp::FS_PROCEDURE_RESCUE;
    r.ask(msp::FAILSAFE_CONFIG);
    CHECK(r.dir == '>' && r.out.size() == 8 && r.out[0] == 8 && r.out[1] == 40 && r.u16(2) == 1250 && r.out[7] == 2, "guard 0.8 s (8), landing 40 s, throttle 1250 us, procedure GPS_RESCUE (2)");
    v.fs_procedure = wbp::FS_PROCEDURE_HOLD; r.ask(msp::FAILSAFE_CONFIG); CHECK(r.out[7] == 1, "HOLD (none in Betaflight): said AUTO_LAND");
    r.ask(msp::SET_FAILSAFE_CONFIG, r.out); CHECK(r.dir == '>' && v.fs_procedure == wbp::FS_PROCEDURE_HOLD, "written back as read: HOLD kept");
    r.ask(msp::SET_FAILSAFE_CONFIG, {15, 25, 0xB0, 0x04, 0, 100, 0, 0});
    CHECK(r.dir == '>' && s.general.fs_guard_ms == 1500 && s.general.fs_max_descent_s == 25 && s.general.fs_throttle == 1200 && v.fs_procedure == wbp::FS_PROCEDURE_DROP, "SET: guard 1.5 s, 25 s, 1200 us, DROP");
    bool gen = false, nav = false; for (uint8_t c : r.port.commits) { gen |= c == wbp::SEC_GENERAL; nav |= c == wbp::SEC_NAV; }
    CHECK(gen && nav, "general and nav committed");
    r.ask(msp::SET_FAILSAFE_CONFIG, {15, 25, 0xB0, 0x04, 0, 100, 0, 1}); CHECK(v.fs_procedure == wbp::FS_PROCEDURE_LAND, "AUTO_LAND: LAND");
    r.ask(msp::SET_FAILSAFE_CONFIG, {15, 25, 0xB0, 0x04, 0, 100, 0, 7}); CHECK(r.dir == '!' && v.fs_procedure == wbp::FS_PROCEDURE_LAND, "a procedure out of the table: refused");
    r.ask(msp::RXFAIL_CONFIG);
    CHECK(r.out.size() == 16 * 3 && r.out[0] == 0 && r.out[3 * 3] == 0 && r.out[4 * 3] == 1 && r.u16(4 * 3 + 1) == 1500, "RXFAIL: 16 channels; sticks AUTO, the AUX channels HOLD");
    r.ask(msp::RSSI_CONFIG); CHECK(r.out.size() == 1 && r.out[0] == 0, "RSSI_CONFIG: no RSSI channel (0)");
    // GPS Rescue
    v.ph_angle_max = 35; v.rescue_return_alt = 40; v.rescue_descent_dist = 20; v.rescue_ground_speed = 75; v.rescue_min_sats = 8; v.rescue_ascend_rate = 50; v.rescue_descend_rate = 15;
    v.rescue_allow_no_fix = 1; v.rescue_alt_mode = wbp::RESCUE_ALT_FIXED; v.rescue_min_start_dist = 15; v.rescue_initial_climb = 10; r.t.hover_throttle = 0.32f;
    r.ask(msp::GPS_RESCUE);
    CHECK(r.dir == '>' && r.out.size() == 26 && r.u16(0) == 35 && r.u16(2) == 40 && r.u16(4) == 20 && r.u16(6) == 750 && r.u16(12) == 1320 && r.out[15] == 8 && r.u16(16) == 500 && r.u16(18) == 150
          && r.out[20] == 1 && r.out[21] == 1 && r.u16(22) == 15 && r.u16(24) == 10, "angle 35, 40 m, 20 m, 7.5 m/s = 750 cm/s, hover 1320 us (measured), 8 sats, 500 / 150 cm/s, no fix allowed, FIXED, 15 m, 10 m");
    v.rescue_alt_mode = wbp::RESCUE_ALT_CONE; r.ask(msp::GPS_RESCUE); CHECK(r.out[21] == 0, "CONE (none in Betaflight): said MAX");
    r.ask(msp::SET_GPS_RESCUE, r.out); CHECK(r.dir == '>' && v.rescue_alt_mode == wbp::RESCUE_ALT_CONE && v.rescue_ground_speed == 75, "written back as read: CONE kept, nothing changes");
    r.ask(msp::GPS_RESCUE); std::vector<uint8_t> w = r.out; w[0] = 30; w[2] = 60; w[6] = (uint8_t)(500 & 0xFF); w[7] = 500 >> 8; w[15] = 10; w[16] = (uint8_t)(300 & 0xFF); w[17] = 300 >> 8; w[20] = 0; w[21] = 2; w[24] = 5;
    r.ask(msp::SET_GPS_RESCUE, w);
    CHECK(r.dir == '>' && v.ph_angle_max == 30 && v.rescue_return_alt == 60 && v.rescue_ground_speed == 50 && v.rescue_min_sats == 10 && v.rescue_ascend_rate == 30 && !v.rescue_allow_no_fix
          && v.rescue_alt_mode == wbp::RESCUE_ALT_CURRENT && v.rescue_initial_climb == 5 && r.port.commits.back() == wbp::SEC_NAV, "SET: angle 30, 60 m, 5 m/s, 10 sats, 3 m/s up, fix needed, CURRENT, climb 5 m");
    r.ask(msp::SET_GPS_RESCUE, std::vector<uint8_t>(w.begin(), w.begin() + 16)); CHECK(r.dir == '>' && v.rescue_ascend_rate == 30, "the 16-byte form: the rest kept");
    w[21] = 5; r.ask(msp::SET_GPS_RESCUE, w); CHECK(r.dir == '!', "an altitude mode out of the table: refused");
    r.port.is_armed = true; r.ask(msp::SET_FAILSAFE_CONFIG, {15, 25, 0xB0, 0x04, 0, 100, 0, 2}); CHECK(r.dir == '!' && v.fs_procedure == wbp::FS_PROCEDURE_LAND, "armed: refused");
}

// 0.44: the PID tab's tuning sliders (MSP_SIMPLIFIED_TUNING and the CALCULATE / VALIDATE commands): the firmware's own sliders
// (core/simplified_tuning.h). Layout: PIDs 18 bytes (mode, master, roll / pitch D ratio, I, D, PI, D max, -, FF, pitch PI, 8 reserved),
// then the D-term and the gyro filter, 22 bytes each (on, multiplier, LPF1 static, LPF2, dynamic min, max, 4 + 8 reserved)
static void t_simplified_tuning() {
    printf("[sliders] SIMPLIFIED_TUNING, SET_, CALCULATE_SIMPLIFIED_PID / GYRO / DTERM, VALIDATE_\n");
    Rig r; wbp::Settings& s = r.s; wbp::S_pid& p = s.pid; wbp::S_filters& f = s.filters;
    p.slider_mode = wbp::SLIDER_MODE_RP; p.slider_master = 110; p.slider_pitch_d = 120; p.slider_i = 90; p.slider_d = 105; p.slider_pi = 95; p.slider_d_max = 80; p.slider_ff = 130; p.slider_pitch_pi = 115;
    f.slider_dterm = 1; f.slider_dterm_mult = 120; f.slider_gyro = 0; f.slider_gyro_mult = 100;
    wbp::apply_rules(s, wbp::SEC_PID); wbp::apply_rules(s, wbp::SEC_FILTERS);
    r.ask(msp::SIMPLIFIED_TUNING);
    CHECK(r.dir == '>' && r.out.size() == 62 && r.out[0] == 1 && r.out[1] == 110 && r.out[2] == 120 && r.out[3] == 90 && r.out[4] == 105 && r.out[5] == 95 && r.out[6] == 80 && r.out[8] == 130 && r.out[9] == 115,
          "PIDs: RP, master 110, pitch D 120, I 90, D 105, PI 95, D max 80, FF 130, pitch PI 115");
    CHECK(r.out[18] == 1 && r.out[19] == 120 && r.u16(20) == f.dterm_lpf1_static_hz && r.u16(22) == f.dterm_lpf2_hz && r.u16(24) == f.dterm_lpf1_dyn_min_hz && r.u16(26) == f.dterm_lpf1_dyn_max_hz
          && r.out[40] == 0 && r.out[41] == 100 && r.u16(42) == f.gyro_lpf1_static_hz && r.u16(46) == f.gyro_lpf1_dyn_min_hz, "the D-term slider on at 120, its cutoffs; the gyro slider off");
    // what the App's sliders would give: the firmware works it out with its own code
    std::vector<uint8_t> pid(r.out.begin(), r.out.begin() + 18); pid[1] = 130;
    r.ask(msp::CALCULATE_SIMPLIFIED_PID, pid);
    { wbp::S_pid t = p; t.slider_master = 130; uint8_t pp[3], ii[3], dd[3], dm[3]; uint16_t ff[3];
      for (int a = 0; a < 3; a++) { pp[a] = t.p[a]; ii[a] = t.i[a]; dd[a] = t.d[a]; dm[a] = t.d_max[a]; ff[a] = t.f[a]; }
      fc::PidSliders sl; sl.mode = t.slider_mode; sl.master = 130; sl.d = t.slider_d; sl.pi = t.slider_pi; sl.ff = t.slider_ff; sl.d_max = t.slider_d_max; sl.i = t.slider_i; sl.pitch_d = t.slider_pitch_d; sl.pitch_pi = t.slider_pitch_pi;
      fc::sliders_to_pid(sl, pp, ii, dd, dm, ff);
      bool same = r.out.size() == 18; for (int a = 0; a < 3 && same; a++) same = r.out[a * 6] == pp[a] && r.out[a * 6 + 1] == ii[a] && r.out[a * 6 + 2] == dd[a] && r.out[a * 6 + 3] == dm[a] && r.u16(a * 6 + 4) == ff[a];
      CHECK(same && r.out[0] != p.p[0] && r.out[12] == p.p[2], "CALCULATE_SIMPLIFIED_PID: master 130 -> P / I / D / D max / F of each axis (sliders_to_pid); yaw (not under RP) as it is"); }
    CHECK(p.slider_master == 110, "... nothing is changed by a CALCULATE");
    std::vector<uint8_t> gy(22, 0); gy[0] = 1; gy[1] = 150; gy[2] = 250; gy[4] = (uint8_t)(500 & 0xFF); gy[5] = 500 >> 8; gy[6] = 250; gy[8] = (uint8_t)(500 & 0xFF); gy[9] = 500 >> 8;
    r.ask(msp::CALCULATE_SIMPLIFIED_GYRO, gy);
    CHECK(r.out.size() == 22 && r.out[0] == 1 && r.out[1] == 150 && r.u16(2) == 375 && r.u16(4) == 750 && r.u16(6) == 375 && r.u16(8) == 750, "CALCULATE_SIMPLIFIED_GYRO: 150 %% -> LPF1 375, LPF2 750, dynamic 375..750");
    std::vector<uint8_t> dt(22, 0); dt[0] = 1; dt[1] = 80; dt[2] = 75; dt[4] = 150; dt[6] = 75; dt[8] = 150;
    r.ask(msp::CALCULATE_SIMPLIFIED_DTERM, dt); CHECK(r.out.size() == 22 && r.u16(2) == 60 && r.u16(4) == 120 && r.u16(6) == 60 && r.u16(8) == 120, "CALCULATE_SIMPLIFIED_DTERM: 80 %% -> 60 / 120");
    dt[0] = 0; r.ask(msp::CALCULATE_SIMPLIFIED_DTERM, dt); CHECK(r.u16(2) == 75 && r.u16(4) == 150, "slider off: the cutoffs as sent");
    r.ask(msp::VALIDATE_SIMPLIFIED_TUNING); CHECK(r.out.size() == 3 && r.out[0] == 1 && r.out[1] == 1 && r.out[2] == 1, "VALIDATE: gains and cutoffs are the sliders'");
    p.p[0] = (uint8_t)(p.p[0] + 3); r.ask(msp::VALIDATE_SIMPLIFIED_TUNING); CHECK(r.out[0] == 0 && r.out[2] == 1, "a typed roll P: the PIDs no longer match the sliders");
    // SET
    r.ask(msp::SIMPLIFIED_TUNING); std::vector<uint8_t> w = r.out; w[0] = 2; w[1] = 90; w[40] = 1; w[41] = 70;
    r.ask(msp::SET_SIMPLIFIED_TUNING, w);
    CHECK(r.dir == '>' && p.slider_mode == wbp::SLIDER_MODE_RPY && p.slider_master == 90 && f.slider_gyro && f.slider_gyro_mult == 70, "SET: RPY, master 90, the gyro slider on at 70");
    bool pc = false, fc_ = false; for (uint8_t c : r.port.commits) { pc |= c == wbp::SEC_PID; fc_ |= c == wbp::SEC_FILTERS; }
    CHECK(pc && fc_, "pid and filters committed (the gains follow the sliders there)");
    r.ask(msp::SET_SIMPLIFIED_TUNING, std::vector<uint8_t>(w.begin(), w.begin() + 30)); CHECK(r.dir == '!', "a short frame: refused");
    r.port.is_armed = true; r.ask(msp::SET_SIMPLIFIED_TUNING, w); CHECK(r.dir == '!', "armed: refused");
}

// 0.44: the Blackbox tab's write (MSP_SET_BLACKBOX_CONFIG): device (NONE 0, FLASH 1, SDCARD 2), P ratio, sample rate (every 2^n-th cycle)
static void t_blackbox_write() {
    printf("[blackbox] SET_BLACKBOX_CONFIG\n");
    Rig r; wbp::Settings& s = r.s; s.general.loop_hz = 4000; s.general.pid_denom = 1; s.blackbox.device = wbp::BB_DEVICE_FLASH; s.blackbox.rate_hz = 1000;
    r.ask(msp::BLACKBOX_CONFIG); std::vector<uint8_t> w(r.out.begin() + 1, r.out.end());      // (the App sends it without the "supported" byte)
    r.ask(msp::SET_BLACKBOX_CONFIG, w); CHECK(r.dir == '>' && s.blackbox.device == wbp::BB_DEVICE_FLASH && s.blackbox.rate_hz == 1000, "written back as read: nothing changes");
    w[0] = wbp::BB_DEVICE_SDCARD; w[5] = 1; r.ask(msp::SET_BLACKBOX_CONFIG, w);
    CHECK(r.dir == '>' && s.blackbox.device == wbp::BB_DEVICE_SDCARD && s.blackbox.rate_hz == 2000 && r.port.commits.back() == wbp::SEC_BLACKBOX, "SD card, sample rate 1 (every 2nd cycle of 4 kHz: 2000 Hz)");
    w[5] = 9; r.ask(msp::SET_BLACKBOX_CONFIG, w); CHECK(s.blackbox.rate_hz == 250, "sample rate above 4: 4 (every 16th: 250 Hz) (%u)", s.blackbox.rate_hz);
    w[0] = 3; r.ask(msp::SET_BLACKBOX_CONFIG, w); CHECK(r.dir == '!' && s.blackbox.device == wbp::BB_DEVICE_SDCARD, "SERIAL (3): none here, refused");
    r.port.bb.device = wbp::BB_DEVICE_SDCARD; r.port.bb.state = wbp::BB_STATE_LOGGING; w[0] = 1; r.ask(msp::SET_BLACKBOX_CONFIG, w);
    CHECK(r.dir == '!' && s.blackbox.device == wbp::BB_DEVICE_SDCARD, "while logging: refused (Betaflight)");
}

// 0.44: the Motors tab's motor test (MSP_SET_MOTOR, 8 x us): handed to the firmware's own motor test (CMD_MOTOR_TEST)
static void t_set_motor() {
    printf("[motors tab] SET_MOTOR\n");
    Rig r;
    r.ask(msp::SET_MOTOR, le16({1000, 1250, 1000, 1000, 1000, 1000, 1000, 1000}));
    CHECK(r.dir == '>' && r.port.motor_calls == 1 && r.port.motors.size() == 8 && r.port.motors[1] == 1250 && r.port.motors[0] == 1000, "motor 2 at 1250 us: handed over");
    r.port.is_armed = true; r.ask(msp::SET_MOTOR, le16({1300, 1000, 1000, 1000, 1000, 1000, 1000, 1000}));
    CHECK(r.dir == '!' && r.port.motor_calls == 1, "armed: refused, nothing spins"); r.port.is_armed = false;
    r.s.msp.writes = 0; r.ask(msp::SET_MOTOR, le16({1300, 1000, 1000, 1000, 1000, 1000, 1000, 1000})); CHECK(r.dir == '!' && r.port.motor_calls == 1, "msp.writes off: refused"); r.s.msp.writes = 1;
    r.ask(msp::SET_MOTOR, le16({1300, 1000})); CHECK(r.dir == '!', "a short frame: refused");
    r.port.motor_ok = false; r.ask(msp::SET_MOTOR, le16({1300, 1000, 1000, 1000, 1000, 1000, 1000, 1000})); CHECK(r.dir == '!', "the firmware refuses (no motors): '!'");
}

// 0.44: the OSD tab (MSP_OSD_CONFIG, master's 261 bytes - a v1 "jumbo" frame; SET_OSD_CONFIG one element / the general part at a
// time; MSP_OSD_CANVAS). Betaflight's position: column bits 0-4 (+ bit 10 for 32..63), row bits 5-9, shown bit 11; this firmware's
// osd.pos: column 0-5, row 6-10, shown 11. Its 32 elements are Betaflight's items (RSSI 0, BATT 1, ... FLIGHT_DIST 47)
static void t_osd_tab() {
    printf("[osd tab] OSD_CONFIG, SET_OSD_CONFIG, OSD_CANVAS\n");
    Rig r; wbp::S_osd& o = r.s.osd;
    o.type = wbp::OSD_TYPE_MSP_DISPLAYPORT; o.stats = 1;
    for (int i = 0; i < 32; i++) o.pos[i] = 0;
    o.pos[osd::E_BATT] = osd::make_pos(40, 17, true);             // column 40 (above 31: Betaflight's bit 10), row 17, shown
    o.pos[osd::E_FLIGHT_DIST] = osd::make_pos(3, 2, false);
    r.ask(msp::OSD_CONFIG);
    CHECK(r.dir == '>' && r.jumbo && r.jumbo_ok && r.out.size() == 261, "a jumbo v1 frame (255, then a 16-bit length): 261 bytes, the checksum right (%u)", (unsigned)r.out.size());
    CHECK((r.out[0] & 1) && (r.out[0] & (1 << 6)) && r.out[1] == 3 && r.out[2] == 1 && r.out[7] == 99, "flags: OSD, MSP device; HD video; metric; 99 items");
    const auto item = [&](int i) { return r.u16(10 + 2 * i); };
    CHECK(item(1) == ((40 & 0x1F) | (1 << 10) | (17 << 5) | 0x0800), "MAIN_BATT_VOLTAGE (1): column 40, row 17, shown (0x%04x)", item(1));
    CHECK(item(47) == (3 | (2 << 5)), "FLIGHT_DIST (47): column 3, row 2, hidden (0x%04x)", item(47));
    CHECK(item(10) == 0 && item(98) == 0, "items this firmware has not (VTX channel, ...): 0");
    CHECK(r.out[208] == 31 && r.out[209 + 3] == 1 && r.out[240] == 2 && r.u16(241) == 0x0A00 && r.u16(243) == 0x0A01 && r.out[247] == 20 && r.out[252] == 1 && r.out[253] == 1,
          "31 statistics (max speed on), 2 timers (on / armed, alarm 10), 20 warnings, one OSD profile");
    // SET: one element
    r.ask(msp::SET_OSD_CONFIG, {7, (uint8_t)((5 | (9 << 5) | 0x0800) & 0xFF), (uint8_t)((5 | (9 << 5) | 0x0800) >> 8), 1});
    CHECK(r.dir == '>' && o.pos[osd::E_MODE] == osd::make_pos(5, 9, true) && r.port.commits.back() == wbp::SEC_OSD, "FLYMODE (7) to column 5, row 9, shown: osd.pos[MODE]");
    { const uint16_t v = (uint16_t)(50 & 0x1F) | (1 << 10) | (3 << 5); r.ask(msp::SET_OSD_CONFIG, {8, (uint8_t)v, (uint8_t)(v >> 8), 1}); }
    CHECK(o.pos[osd::E_NAME] == osd::make_pos(50, 3, false), "CRAFT_NAME (8) to column 50 (bit 10), row 3, hidden");
    r.ask(msp::SET_OSD_CONFIG, {10, 0x21, 0x08, 1}); CHECK(r.dir == '>', "an item this firmware has not (VTX channel): accepted, not kept");
    r.ask(msp::SET_OSD_CONFIG, {120, 0x21, 0x08, 1}); CHECK(r.dir == '!', "an item number past Betaflight's list: refused");
    r.ask(msp::SET_OSD_CONFIG, {3, 0, 0, 0}); CHECK(r.dir == '>' && !o.stats, "statistic (screen 0) switched off: the statistics screen off");
    r.ask(msp::SET_OSD_CONFIG, {3, 1, 0, 0}); CHECK(o.stats, "... and on");
    r.ask(msp::SET_OSD_CONFIG, {0xFF, 3, 1, 20, 0x98, 0x08, 0, 0, 100, 0}); CHECK(r.dir == '>', "the general part (-1): accepted (video system, units, alarms: not kept here)");
    o.canvas = wbp::OSD_CANVAS_HDZERO_50X18; r.ask(msp::OSD_CANVAS); CHECK(r.out.size() == 2 && r.out[0] == 50 && r.out[1] == 18, "OSD_CANVAS: HDZero 50 x 18");
    o.type = wbp::OSD_TYPE_NONE; r.ask(msp::OSD_CONFIG); CHECK(r.out[0] == 0, "OSD off: no flags");
    r.port.is_armed = true; r.ask(msp::SET_OSD_CONFIG, {7, 0, 0, 1}); CHECK(r.dir == '!', "armed: refused");
}

// 0.44: the Ports tab's port list (MSP2_COMMON_SERIAL_CONFIG, MSP_CF_SERIAL_CONFIG): UART0..2 as Betaflight's identifiers 50..52
// (an ESP32's UART0 first), no functions (master: assigned through the CLI), the baud classes 115200 (BAUD_115200 = 5)
static void t_serial_config() {
    printf("[ports tab] MSP2_COMMON_SERIAL_CONFIG, MSP_CF_SERIAL_CONFIG\n");
    Rig r;
    r.ask2(msp::COMMON_SERIAL_CONFIG);
    CHECK(r.dir == '>' && r.out.size() == 1 + 3 * 9 && r.out[0] == 3 && r.out[1] == 50 && r.u32(2) == 0 && r.out[6] == 5 && r.out[9] == 5 && r.out[10] == 51 && r.out[19] == 52,
          "three ports: 50, 51, 52 (UART0..2), function mask 0, baud 115200");
    r.ask(msp::CF_SERIAL_CONFIG); CHECK(r.out.size() == 3 * 7 && r.out[0] == 50 && r.u16(1) == 0 && r.out[7] == 51, "the old layout: 7 bytes a port");
    r.ask2(msp::COMMON_SET_SERIAL_CONFIG, {0}); CHECK(r.dir == '!', "the write: refused (ports are assigned through the CLI, as master)");
}

// 0.44: 3D and the deadbands (the Motors tab's 3D block, the Receiver tab's deadbands; Betaflight master's layouts)
static void t_mode3d_msp() {
    printf("[3D] feature 3D, MSP_MOTOR_3D_CONFIG, MSP_RC_DEADBAND, the 3D switch\n");
    Rig r; wbp::Settings& s = r.s;
    s.motors.mode3d = 1; s.receiver.protocol = wbp::RX_PROTOCOL_CRSF;
    r.ask(msp::FEATURE_CONFIG); CHECK(r.u32(0) & (1u << 12), "FEATURE_CONFIG: 3D (bit 12)");
    r.ask(msp::SET_FEATURE_CONFIG, le32(1u << 3)); CHECK(r.dir == '>' && !s.motors.mode3d && r.port.commits.back() == wbp::SEC_MOTORS, "SET_FEATURE_CONFIG without it: 3D off");
    r.ask(msp::SET_FEATURE_CONFIG, le32((1u << 3) | (1u << 12))); CHECK(s.motors.mode3d, "and on again");
    s.motors.deadband3d_low = 1410; s.motors.deadband3d_high = 1520; s.motors.neutral3d = 1465;
    r.ask(msp::MOTOR_3D_CONFIG); CHECK(r.dir == '>' && r.out.size() == 6 && r.u16(0) == 1410 && r.u16(2) == 1520 && r.u16(4) == 1465, "MOTOR_3D_CONFIG: the settings (1410 / 1520 / 1465)");
    r.ask(msp::SET_MOTOR_3D_CONFIG, le16({1400, 1530, 1470}));
    CHECK(r.dir == '>' && s.motors.deadband3d_low == 1400 && s.motors.deadband3d_high == 1530 && s.motors.neutral3d == 1470 && r.port.commits.back() == wbp::SEC_MOTORS, "SET_MOTOR_3D_CONFIG");
    s.receiver.deadband = 12; s.receiver.yaw_deadband = 20; s.receiver.deadband3d_throttle = 60;
    r.ask(msp::RC_DEADBAND); CHECK(r.out.size() == 5 && r.out[0] == 12 && r.out[1] == 20 && r.out[2] == 0 && r.u16(3) == 60, "RC_DEADBAND: 12 / 20 us, (Position Hold's: none here), 3D 60 us");
    r.ask(msp::SET_RC_DEADBAND, {5, 8, 33, 70, 0});
    CHECK(r.dir == '>' && s.receiver.deadband == 5 && s.receiver.yaw_deadband == 8 && s.receiver.deadband3d_throttle == 70 && r.port.commits.back() == wbp::SEC_RECEIVER, "SET_RC_DEADBAND");
    s.modes.mode[0] = wbp::MODE_3D; s.modes.aux[0] = 3; s.modes.lo[0] = 1700; s.modes.hi[0] = 2100;
    r.ask(msp::MODE_RANGES); CHECK(r.out[0] == 29, "the 3D switch as Betaflight's box 29 (%u)", r.out[0]);
    r.ask(msp::BOXNAMES); CHECK(std::string(r.out.begin(), r.out.end()).find("3D DISABLE;") != std::string::npos, "BOXNAMES: '3D DISABLE'");
    r.port.is_armed = true; r.ask(msp::SET_RC_DEADBAND, {1, 1, 0, 50, 0}); CHECK(r.dir == '!' && s.receiver.deadband == 5, "armed: refused");
}

int main() {
    t_arming_disabled();
    t_mode3d_msp();
    t_sensors_tab();
    t_pid_advanced();
    t_filter_config();
    t_gps_tab();
    t_power_tab();
    t_failsafe_tab();
    t_simplified_tuning();
    t_blackbox_write();
    t_set_motor();
    t_osd_tab();
    t_serial_config();
    t_board_wizard();
    t_mixer();
    t_config_tab();
    t_connect();
    t_identity();
    t_live();
    t_settings_read();
    t_settings_write();
    printf("%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
