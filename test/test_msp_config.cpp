// MSP for the configurators (src/app/msp_config.h) through msp::Bridge: requests as Betaflight Configurator sends them, the
// answers checked byte by byte against Betaflight 4.5's layouts (msp.c), writes against the settings they must change.
// Build: g++ -std=c++17 -O2 test/test_msp_config.cpp -o build/test_msp_config
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <vector>
#include "../src/app/msp_config.h"

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, ...) do { const bool ok_ = (cond); if (getenv("V")) { printf("    "); printf(__VA_ARGS__); printf("\n"); } if (ok_) g_pass++; else { g_fail++; printf("  FAIL line %d: %s   ", __LINE__, #cond); printf(__VA_ARGS__); printf("\n"); } } while (0)

struct Port : msp::ConfigPort {
    bool is_armed = false, save_ok = true; std::vector<uint8_t> commits; int saves = 0;
    bool armed() override { return is_armed; }
    void commit(uint8_t sec) override { commits.push_back(sec); }
    bool save() override { saves++; return save_ok; }
};
struct Rig {
    wbp::Settings s; Port port; msp::Config cfg{s, port}; msp::Bridge br; msp::Info info; app::Telemetry t;
    std::vector<uint8_t> out; uint8_t dir = 0;
    Rig() { wbp::defaults(s); br.set_handler(&msp::Config::handler, &cfg); cfg.set_telemetry(&t); }
    // one v1 request; out = the answer's payload, dir = '>' or '!'
    void ask(uint8_t cmd, const std::vector<uint8_t>& p = {}) {
        std::vector<uint8_t> f = {'$', 'M', '<', (uint8_t)p.size(), cmd}; uint8_t x = (uint8_t)(p.size() ^ cmd);
        for (uint8_t b : p) { f.push_back(b); x ^= b; }
        f.push_back(x);
        uint8_t o[400]; size_t n = 0; out.clear(); dir = 0;
        for (uint8_t c : f) if (br.feed(c, info, o, sizeof(o), n) == msp::Bridge::REPLY) { dir = o[2]; out.assign(o + 5, o + 5 + o[3]); }
    }
    int u16(int at) const { return out[at] | (out[at + 1] << 8); }
    int i16(int at) const { return (int16_t)u16(at); }
};

static void t_identity() {
    printf("[identity]\n");
    Rig r;
    r.ask(msp::FC_VARIANT); CHECK(r.dir == '>' && r.out.size() == 4 && !memcmp(r.out.data(), "BTFL", 4), "msp.identity BETAFLIGHT (the default): variant BTFL");
    r.ask(msp::FC_VERSION); CHECK(r.out.size() == 3 && r.out[0] == 4 && r.out[1] == 5 && r.out[2] == 0, "version 4.5.0 (what the configurators check)");
    r.ask(msp::BOARD_INFO); CHECK(r.out.size() > 8 && !memcmp(r.out.data(), "WBRX", 4), "board identifier WBRX (what Betaflight Configurator shows as the board)");
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
    r.ask(msp::RAW_IMU); CHECK(r.out.size() == 18 && r.i16(4) == 512 && r.i16(6) == 410 && r.i16(14) == 545, "MSP_RAW_IMU: 1 g = 512, 100 deg/s = 410, 0.5 G = 545 (what the configurator divides by)");
    r.ask(msp::RAW_GPS);
    CHECK(r.out.size() == 18 && r.out[0] == 2 && r.out[1] == 13 && r.u16(10) == 123 && r.u16(12) == 820 && r.u16(14) == 1234 && r.u16(16) == 95, "MSP_RAW_GPS: fix, 13 sats, 123 m, 820 cm/s, course, hdop");
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
    CHECK(r.out.size() == 23 && r.out[0] == 7 && r.out[1] == 10 && r.out[2] == 67 && r.out[3] == 68 && r.out[4] == 40 && r.out[6] == 45 && r.out[7] == 15 && r.u16(8) == 1350,
          "MSP_RC_TUNING: roll center 7 (x10 deg/s), expo 10, max rates 67 / 68 / 40, throttle mid / expo, TPA breakpoint 1350 us");
    CHECK(r.out[10] == 30 && r.out[11] == 9 && r.out[12] == 8 && r.out[13] == 20 && r.out[14] == 1 && r.out[15] == 80 && r.u16(16) == 1998 && r.out[22] == 4,
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
    std::vector<uint8_t> rt = {8, 5, 70, 71, 45, 0, 40, 10, 0x46, 0x05, 25, 9, 10, 15, 1, 75, 0xCE, 0x07, 0xCE, 0x07, 0xCE, 0x07, 4};
    r.ask(msp::SET_RC_TUNING, rt);
    CHECK(r.dir == '>' && s.rates.center_sens[0] == 80 && s.rates.center_sens[1] == 100 && s.rates.center_sens[2] == 90 && s.rates.max_rate[0] == 700 && s.rates.max_rate[2] == 450,
          "SET_RC_TUNING: centre / max rates (x10)");
    CHECK(s.rates.expo[0] == 5 && s.rates.expo[1] == 15 && s.rates.expo[2] == 25 && s.rates.throttle_mid == 40 && s.rates.throttle_expo == 10 && s.rates.throttle_limit == 75 && r.port.commits.back() == wbp::SEC_RATES,
          "expos, throttle mid / expo, throttle limit");
    rt[22] = 0; const uint16_t before = s.rates.max_rate[0]; r.ask(msp::SET_RC_TUNING, rt);
    CHECK(r.dir == '!' && s.rates.max_rate[0] == before, "rates type BETAFLIGHT: refused, nothing changed (no conversion guessed)");
    r.ask(msp::SET_MODE_RANGE, {3, 46, 2, 32, 48});
    CHECK(r.dir == '>' && s.modes.mode[3] == wbp::MODE_RESCUE && s.modes.aux[3] == 2 && s.modes.lo[3] == 1700 && s.modes.hi[3] == 2100 && r.port.commits.back() == wbp::SEC_MODES, "SET_MODE_RANGE: GPS RESCUE (box 46) on aux 3, 1700-2100");
    r.ask(msp::SET_MODE_RANGE, {3, 0, 0, 0, 0}); CHECK(r.dir == '>' && s.modes.hi[3] <= s.modes.lo[3], "start = end: the range is cleared");
    r.ask(msp::SET_MODE_RANGE, {4, 2, 0, 10, 20}); CHECK(r.dir == '!', "HORIZON (box 2): no such mode here, refused");
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

int main() {
    t_identity();
    t_live();
    t_settings_read();
    t_settings_write();
    printf("%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
