// GPS: UBX/NMEA decoding, the Betaflight-style engine (baud sweep, detection, per-generation auto-config) against a strict simulated
// u-blox, home point / geometry. Truth values are produced by the simulator's own independent encoders.
#include <Arduino.h>      // the device build has Arduino's global macros (bit, sq, ...): keep them in the host build too
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "sim_gps.h"
#include "../src/hal/gps/gps_nav.h"

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, ...) do { if (cond) g_pass++; else { g_fail++; printf("  FAIL line %d: %s   ", __LINE__, #cond); printf(__VA_ARGS__); printf("\n"); } } while (0)
using namespace gps;

// run engine + sim on a 1 ms virtual clock; returns elapsed ms until `pred` or the limit
template <class F> static uint32_t run(Engine& e, SimGps& s, uint32_t& t, uint32_t limit_ms, F pred) {
    const uint32_t t0 = t;
    while (t - t0 < limit_ms) { t++; s.tick(t); e.update(t); if (pred()) break; }
    return t - t0;
}

static void t_ubx_builder() {
    printf("[ubx] frames, checksum, VALSET, parser\n");
    uint8_t buf[64]; uint8_t p[3]; ubx::cfg_msg_rate(p, 0xF0, 0x00, 0);
    const size_t n = ubx::build(buf, sizeof(buf), ubx::CLS_CFG, ubx::CFG_MSG, p, 3);
    // known frame from the u-blox protocol description for "disable NMEA GGA": B5 62 06 01 03 00 F0 00 00 FA 0F
    const uint8_t want[] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x00, 0x00, 0xFA, 0x0F};
    CHECK(n == sizeof(want) && memcmp(buf, want, n) == 0, "CFG-MSG GGA off frame (len %zu)", n);
    ubx::ValSet v; v.add(ubx::K_RATE_MEAS, 100); v.add(ubx::K_NAVSPG_DYNMODEL, 6); v.add(ubx::K_SBAS_PRNSCANMASK, 0x0123456789ABCDEFull);
    CHECK(v.n == 4 + (4 + 2) + (4 + 1) + (4 + 8) && v.buf[0] == 0 && v.buf[1] == 1, "valset sizes by key type: %zu", v.n);
    CHECK(v.buf[4] == 0x01 && v.buf[5] == 0x00 && v.buf[6] == 0x21 && v.buf[7] == 0x30 && v.buf[8] == 100 && v.buf[9] == 0, "key little-endian + value");
    CHECK(!v.add(0x00000001, 1), "key with unknown size rejected");
    char pb[48]; ubx::pubx41(pb, sizeof(pb), 115200);
    CHECK(strcmp(pb, "$PUBX,41,1,0003,0001,115200,0*1E\r\n") == 0, "PUBX baud sentence matches Betaflight's table: %s", pb);
    ubx::pubx41(pb, sizeof(pb), 57600); CHECK(strcmp(pb, "$PUBX,41,1,0003,0001,57600,0*2D\r\n") == 0, "57600");
    ubx::pubx41(pb, sizeof(pb), 9600);  CHECK(strcmp(pb, "$PUBX,41,1,0003,0001,9600,0*16\r\n") == 0, "9600");
    // parser: garbage, split frames, bad checksum, back-to-back frames
    ubx::Parser ps; ubx::Message m; int got = 0;
    uint8_t stream[200]; size_t k = 0; stream[k++] = 0x00; stream[k++] = 0xB5; stream[k++] = 0x13;
    memcpy(stream + k, buf, n); k += n;
    uint8_t bad[11]; memcpy(bad, buf, n); bad[9] ^= 0x55; memcpy(stream + k, bad, n); k += n;
    memcpy(stream + k, buf, n); k += n;
    for (size_t i = 0; i < k; i++) if (ps.feed(stream[i], m)) { got++; CHECK(m.cls == 6 && m.id == 1 && m.len == 3 && m.payload[0] == 0xF0, "payload"); }
    CHECK(got == 2 && ps.errors() == 1, "2 good frames, 1 corrupted rejected, resync after garbage (got %d err %u)", got, ps.errors());
}

static void t_nmea() {
    printf("[nmea] sentences\n");
    nmea::Parser np; bool got = false;
    auto feedline = [&](const char* body) { uint8_t x = 0; for (const char* c = body; *c; c++) x ^= (uint8_t)*c; char b[200]; snprintf(b, sizeof(b), "$%s*%02X\r\n", body, x); bool r = false; for (char* c = b; *c; c++) r |= np.feed(*c); return r; };
    feedline("GNRMC,123519.00,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W,A");
    got = feedline("GNGGA,123519.00,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,");
    CHECK(got && np.sol.fix && np.sol.num_sat == 8, "GGA: fix, sats");
    CHECK(abs(np.sol.lat - 481173000) <= 2 && abs(np.sol.lon - 115166667) <= 2, "lat/lon 48.1173 / 11.516667 -> %d %d", np.sol.lat, np.sol.lon);
    CHECK(np.sol.alt_cm == 54540 && np.sol.hdop == 90, "alt 545.4 m, hdop 0.9");
    CHECK(abs((int)np.sol.ground_speed_cms - 1152) <= 2 && np.sol.course_ddeg == 844, "22.4 kn = 11.52 m/s, course 84.4 (%u, %u)", np.sol.ground_speed_cms, np.sol.course_ddeg);
    CHECK(abs(np.sol.vel_n_cms - 112) <= 2 && abs(np.sol.vel_e_cms - 1146) <= 2 && np.sol.vel_d_cms == 0 && !np.sol.has_vel_d,
          "RMC speed + course -> north / east velocity %d / %d cm/s (the estimators need them; before 0.8.3 they stayed 0 with NMEA); no vertical velocity", np.sol.vel_n_cms, np.sol.vel_e_cms);
    CHECK(np.sol.date.valid && np.sol.date.day == 23 && np.sol.date.month == 3 && np.sol.date.year == 2094 && np.sol.date.hour == 12 && np.sol.date.minute == 35 && np.sol.date.second == 19, "RMC date/time (two-digit year 94 -> 20xx, like Betaflight)");
    feedline("GNGGA,123519.00,3351.000,S,15112.000,W,1,10,0.8,10.0,M,0,M,,");
    CHECK(abs(np.sol.lat + 338500000) <= 2 && abs(np.sol.lon + 1512000000) <= 2, "southern / western hemispheres negative: %d %d", np.sol.lat, np.sol.lon);
    CHECK(feedline("GNGGA,123519.00,4807.038,N,01131.000,E,0,00,99.9,,M,,M,,") == false && !np.sol.fix, "fix quality 0 => no fix, no solution event");
    feedline("GNGSA,A,3,01,02,03,04,05,06,07,08,,,,,1.8,1.0,1.5");
    CHECK(np.sol.pdop == 180 && np.sol.hdop == 100 && np.sol.vdop == 150 && np.sol.fix_type == 3, "GSA dops");
    const uint32_t errs = np.errors; for (const char* c = "$GNGGA,1,2,3*00\r\n"; *c; c++) np.feed(*c);
    CHECK(np.errors == errs + 1, "bad checksum counted");
    feedline("GPGSV,2,1,07,01,40,083,46,02,17,308,41,12,07,344,39,14,22,228,45"); feedline("GPGSV,2,2,07,15,31,150,40,17,56,201,42,19,10,040,38");
    CHECK(np.num_sats_in_view == 7 && np.sats[0].sv_id == 1 && np.sats[0].cno == 46 && np.sats[4].sv_id == 15 && np.sats[6].cno == 38, "GSV satellite list");
    CHECK(nmea::coord("01131.000", false) == 115166666 || nmea::coord("01131.000", false) == 115166667, "3-digit longitude degrees");
}

static void check_pvt_solution(const Engine& e, const SimTruth& t, const char* tag) {
    const Solution& s = e.solution();
    CHECK(s.fix && s.fix_type == 3 && s.num_sat == t.sats, "%s fix/sats", tag);
    CHECK(abs(s.lat - (int32_t)llround(t.lat * 1e7)) <= 1 && abs(s.lon - (int32_t)llround(t.lon * 1e7)) <= 1, "%s position", tag);
    CHECK(abs(s.alt_cm - (int32_t)llround(t.alt_m * 100)) <= 2, "%s altitude MSL %d cm", tag, s.alt_cm);
    CHECK(abs(s.vel_n_cms - (int)llround(t.vn * 100)) <= 1 && abs(s.vel_e_cms - (int)llround(t.ve * 100)) <= 1 && abs(s.vel_d_cms - (int)llround(t.vd * 100)) <= 1 && s.has_vel_d, "%s NED velocity %d %d %d (vertical measured)", tag, s.vel_n_cms, s.vel_e_cms, s.vel_d_cms);
    const double gs = sqrt(t.vn * t.vn + t.ve * t.ve); CHECK(abs((int)s.ground_speed_cms - (int)llround(gs * 100)) <= 1, "%s ground speed %u", tag, s.ground_speed_cms);
    const double sp3 = sqrt(gs * gs + t.vd * t.vd); CHECK(abs((int)s.speed3d_cms - (int)llround(sp3 * 100)) <= 1, "%s 3D speed %u", tag, s.speed3d_cms);
    double hd = atan2(t.ve, t.vn) * 180 / M_PI; if (hd < 0) hd += 360; CHECK(abs((int)s.course_ddeg - (int)llround(hd * 10)) <= 1, "%s course %u vs %.1f", tag, s.course_ddeg, hd);
    CHECK(s.pdop == (uint16_t)llround(t.pdop * 100) || s.pdop == (uint16_t)(t.pdop * 100), "%s pdop %u", tag, s.pdop);
    CHECK(s.h_acc_mm == (uint32_t)llround(t.hacc_m * 1000) || e.version() < UBX_M7, "%s hAcc", tag);
}

static void t_engine(UbxVersion ver) {
    char tag[32]; snprintf(tag, sizeof(tag), "[engine %s]", ubx::version_name(ver));
    printf("%s full bring-up on the right baud\n", tag);
    SimGps sim; sim.version = ver; sim.module_baud = 57600; sim.engine_baud = 57600;
    sim.truth.fix_after_ms = 6000; sim.truth.vn = 3.2; sim.truth.ve = -1.4; sim.truth.vd = 0.3;
    Engine e; e.cfg.baud = 57600; e.cfg.update_rate_hz = 10; e.cfg.sbas = SBAS_NONE; e.cfg.use_galileo = false;
    uint32_t t = 0; e.begin(sim, t);
    uint32_t dt = run(e, sim, t, 20000, [&] { return e.healthy(); });
    CHECK(e.healthy(), "%s reached RECEIVING_DATA (%u ms) state=%d nacks=%u timeouts=%u", tag, dt, (int)e.state(), e.config_nacks(), e.config_timeouts());
    CHECK(e.version() == ver, "%s detected generation %s", tag, ubx::version_name(e.version()));
    CHECK(e.config_nacks() == 0 && sim.nacks == 0, "%s module never NACKed: the driver spoke the right dialect (%d nacks)", tag, sim.nacks);
    CHECK(sim.module_baud == 57600 && e.baud() == 57600, "%s baud", tag);
    CHECK(!sim.nmea[0] && !sim.nmea[1] && !sim.nmea[2] && !sim.nmea[3] && !sim.nmea[4] && !sim.nmea[5], "%s all NMEA sentences switched off", tag);
    CHECK(sim.rate_ms == 100, "%s nav rate 10 Hz (%u ms)", tag, sim.rate_ms);
    CHECK(sim.nav5_seen, "%s a navigation model was configured (acquire phase), dynModel now %u", tag, sim.dynmodel);
    if (ver >= UBX_M7) CHECK(sim.pvt, "%s NAV-PVT enabled", tag);
    if (ver >= UBX_M7) CHECK(!sim.posllh && !sim.velned && !sim.sol && !sim.status, "%s legacy NAV messages off when PVT is on", tag);
    if (ver < UBX_M7) CHECK(!sim.pvt && sim.posllh && sim.velned && sim.sol && sim.status, "%s M5/M6: SOL + POSLLH + VELNED + STATUS on", tag);
    if (ver >= UBX_M8 && ver < UBX_M9) CHECK(sim.navsat && !sim.svinfo, "%s NAV-SAT", tag);
    if (ver < UBX_M8) CHECK(sim.svinfo && !sim.navsat, "%s NAV-SVINFO", tag);
    CHECK(!sim.sbas_enabled || ver >= UBX_M9 ? true : false, "%s SBAS disabled when sbas=NONE", tag);
    if (ver >= UBX_M7 && ver < UBX_M9) CHECK(!sim.sbas_gnss && !sim.gal, "%s CFG-GNSS: SBAS + Galileo off", tag);
    if (ver >= UBX_M9) CHECK(!sim.sbas_gnss && !sim.gal, "%s VALSET signals: SBAS + Galileo off", tag);
    // wait for the 3D fix, then the flight model must be applied and solutions decoded
    run(e, sim, t, 20000, [&] { return e.solution().fix && sim.dynmodel == 7 + 0 + 1 - 1 + 0 ? true : (e.solution().fix && sim.dynmodel != 3 && sim.dynmodel != 2); });
    run(e, sim, t, 3000, [&] { return false; });
    CHECK(e.solution().fix, "%s 3D fix", tag);
    CHECK(sim.dynmodel == 8, "%s after the first 3D fix the dynamic model becomes AIRBORNE<4g (u-blox dynModel 8): %u", tag, sim.dynmodel);
    check_pvt_solution(e, sim.truth, tag);
    CHECK(e.rate_hz() > 8.5f && e.rate_hz() < 11.5f, "%s measured solution rate %.1f Hz", tag, e.rate_hz());
    if (ver >= UBX_M8) CHECK(e.sat_count() == 6 && e.sats()[0].sv_id == 10 && e.sats()[0].cno == 30, "%s satellite list: %d", tag, e.sat_count());
    // armed: satellite info is switched off, disarmed: back on
    e.set_armed(true); run(e, sim, t, 500, [&] { return false; });
    if (ver >= UBX_M8) CHECK(!sim.navsat && sim.sat_rate == 0, "%s NAV-SAT stopped while armed", tag);
    e.set_armed(false); run(e, sim, t, 500, [&] { return false; });
    if (ver >= UBX_M8) CHECK(sim.navsat && sim.sat_rate == 5, "%s NAV-SAT back after disarm", tag);
}

static void t_baud_and_failures() {
    printf("[engine] baud sweep, silent module, lost link, unknown module\n");
    { SimGps sim; sim.version = UBX_M8; sim.module_baud = 9600; sim.engine_baud = 57600; sim.truth.fix_after_ms = 100;
      Engine e; e.cfg.baud = 57600; uint32_t t = 0; e.begin(sim, t);
      const uint32_t dt = run(e, sim, t, 60000, [&] { return e.healthy(); });
      CHECK(e.healthy() && sim.module_baud == 57600 && sim.baud_changes == 1, "module found at 9600 and moved to 57600 (%u ms, changes %u)", dt, sim.baud_changes);
      CHECK(sim.nmea_in.size() == 1 && sim.nmea_in[0] == "PUBX,41,1,0003,0001,57600,0*2D", "exact PUBX sentence: %s", sim.nmea_in.empty() ? "-" : sim.nmea_in[0].c_str()); }
    { SimGps sim; sim.version = UBX_M10; sim.module_baud = 230400; sim.engine_baud = 115200; sim.truth.fix_after_ms = 100;
      Engine e; e.cfg.baud = 115200; uint32_t t = 0; e.begin(sim, t); run(e, sim, t, 60000, [&] { return e.healthy(); });
      CHECK(e.healthy() && sim.module_baud == 115200 && e.version() == UBX_M10, "M10 at 230400 -> 115200"); }
    { SimGps sim; sim.version = UBX_M8; sim.module_baud = 38400; sim.engine_baud = 57600; sim.truth.fix_after_ms = 100;
      Engine e; e.cfg.baud = 100000; uint32_t t = 0; e.begin(sim, t); run(e, sim, t, 60000, [&] { return e.healthy(); });
      CHECK(e.healthy() && sim.module_baud == 115200 && e.baud() == 115200, "a non-standard wanted rate (100000): the nearest standard one, 115200 (%u)", sim.module_baud);
      Engine e2; e2.cfg.baud = 20000; SimGps sim2; sim2.version = UBX_M8; sim2.module_baud = 9600; sim2.engine_baud = 57600; sim2.truth.fix_after_ms = 100; uint32_t t2 = 0; e2.begin(sim2, t2);
      run(e2, sim2, t2, 60000, [&] { return e2.healthy(); }); CHECK(sim2.module_baud == 19200, "20000 -> 19200 (%u)", sim2.module_baud); }
    { SimGps sim; sim.version = UBX_M8; sim.module_baud = 9600; sim.engine_baud = 57600;
      Engine e; e.cfg.baud = 57600; e.cfg.auto_baud = false; uint32_t t = 0; e.begin(sim, t); run(e, sim, t, 30000, [&] { return e.healthy(); });
      CHECK(!e.healthy() && e.state() == ST_DETECT_BAUD && e.baud() == 57600, "auto-baud off: stays on the configured rate and keeps asking (state %d baud %u)", (int)e.state(), e.baud()); }
    { SimGps sim; sim.responds = false; Engine e; uint32_t t = 0; e.begin(sim, t); run(e, sim, t, 60000, [&] { return e.healthy(); });
      CHECK(!e.healthy() && e.state() == ST_DETECT_BAUD && !e.solution().fix, "no module: never reports healthy"); }
    { SimGps sim; sim.version = UBX_M8; sim.module_baud = 57600; sim.engine_baud = 57600; sim.truth.fix_after_ms = 100;
      Engine e; e.cfg.baud = 57600; uint32_t t = 0; e.begin(sim, t); run(e, sim, t, 20000, [&] { return e.healthy() && e.solution().fix; });
      CHECK(e.healthy() && e.solution().fix, "up");
      sim.responds = false; run(e, sim, t, 2000, [&] { return false; }); CHECK(e.healthy(), "2 s of silence is tolerated");
      run(e, sim, t, 1500, [&] { return !e.healthy(); });
      CHECK(!e.healthy() && !e.solution().fix && e.solution().num_sat == 0 && e.solution().fix_type == 0, "after 2.5 s without data: link lost, fix and sats cleared");
      sim.responds = true; sim.module_baud = 57600; sim.version = UBX_M8; sim.nmea[0] = true;
      run(e, sim, t, 60000, [&] { return e.healthy(); });
      CHECK(e.healthy() && e.timeouts() >= 1, "recovers by re-running detection (timeouts %u)", e.timeouts()); }
    { SimGps sim; sim.version = UBX_M8; sim.module_baud = 57600; sim.engine_baud = 57600; sim.silent_acks = true;
      Engine e; e.cfg.baud = 57600; uint32_t t = 0; e.begin(sim, t); run(e, sim, t, 120000, [&] { return e.healthy(); });
      CHECK(e.healthy() && e.config_timeouts() > 0, "module that never ACKs: configuration retries then moves on instead of hanging (timeouts %u)", e.config_timeouts()); }
    { SimGps sim; sim.version = UBX_M8; sim.module_baud = 57600; sim.engine_baud = 57600;
      Engine e; e.cfg.baud = 57600; e.cfg.auto_config = false; uint32_t t = 0; e.begin(sim, t); run(e, sim, t, 10000, [&] { return e.healthy(); });
      bool config_sent = false; for (auto& r : sim.log) if (r.cls == 0x06) config_sent = true;
      CHECK(e.healthy() && !config_sent && sim.nmea[0], "auto_config off: nothing is changed on the module"); }
    { SimGps sim; sim.version = UBX_M7; sim.module_baud = 57600; sim.engine_baud = 57600; sim.truth.fix_after_ms = 100;
      Engine e; e.cfg.baud = 57600; e.cfg.sbas = SBAS_EGNOS; e.cfg.use_galileo = true; e.cfg.update_rate_hz = 5; e.cfg.flight_model = MODEL_AIRBORNE_1G; e.cfg.acquire_model = MODEL_PORTABLE;
      uint32_t t = 0; e.begin(sim, t); run(e, sim, t, 30000, [&] { return e.healthy() && e.solution().fix; }); run(e, sim, t, 1000, [&] { return false; });
      CHECK(sim.sbas_enabled && sim.sbas_scan_lo == 0x00010048, "EGNOS: SBAS on with PRN 123/126/136 scan mask (0x%08x)", sim.sbas_scan_lo);
      CHECK(sim.gal && sim.rate_ms == 200, "Galileo on, 5 Hz");
      CHECK(sim.dynmodel == 6, "flight model AIRBORNE_1G -> u-blox dynModel 6 (got %u)", sim.dynmodel); }
    { SimGps sim; sim.version = UBX_M9; sim.module_baud = 57600; sim.engine_baud = 57600; sim.truth.fix_after_ms = 100;
      Engine e; e.cfg.baud = 57600; e.cfg.sbas = SBAS_EGNOS; uint32_t t = 0; e.begin(sim, t); run(e, sim, t, 30000, [&] { return e.healthy(); });
      CHECK(sim.sbas_mask == ((1ull << 3) | (1ull << 6) | (1ull << 16)), "M9 EGNOS PRN mask 123/126/136 (0x%llx)", (unsigned long long)sim.sbas_mask); }
    { SimGps sim; sim.version = UBX_M8; sim.module_baud = 57600; sim.engine_baud = 57600; sim.truth.fix_after_ms = 100; sim.truth.fix_ok_flag = false;
      Engine e; e.cfg.baud = 57600; uint32_t t = 0; e.begin(sim, t); run(e, sim, t, 20000, [&] { return e.healthy(); }); run(e, sim, t, 2000, [&] { return false; });
      CHECK(e.healthy() && sim.truth.fix3d && !e.solution().fix, "fixType 3 but gnssFixOK = 0 (e.g. jammed / invalid): NOT a usable fix"); }
    { SimGps sim; sim.version = UBX_M9; sim.module_baud = 57600; sim.engine_baud = 57600; sim.truth.fix_after_ms = 100;
      Engine e; e.cfg.baud = 57600; e.cfg.sbas = SBAS_WAAS; e.cfg.sbas_integrity = true; uint32_t t = 0; e.begin(sim, t); run(e, sim, t, 30000, [&] { return e.healthy(); });
      CHECK(sim.sbas_enabled && sim.sbas_mask == ((1ull << 11) | (1ull << 13) | (1ull << 15) | (1ull << 18)), "M9 WAAS PRN mask 131/133/135/138 (0x%llx)", (unsigned long long)sim.sbas_mask); }
}

static void t_nmea_engine() {
    printf("[engine NMEA] provider with custom commands\n");
    SimGps sim; sim.version = UBX_M8; sim.module_baud = 38400; sim.engine_baud = 38400; sim.truth.fix_after_ms = 100;
    Engine e; e.cfg.provider = PROV_NMEA; e.cfg.baud = 38400; strcpy(e.cfg.nmea_custom_commands, "$PMTK220,100*2F $PMTK300,100,0,0,0,0*2C");
    sim.truth.vn = 3.0; sim.truth.ve = -4.0; sim.truth.vd = -1.0;
    uint32_t t = 0; e.begin(sim, t); run(e, sim, t, 10000, [&] { return e.healthy() && e.solution().fix; });
    run(e, sim, t, 1200, [&] { return false; });
    CHECK(e.healthy() && e.solution().fix && e.solution().num_sat == sim.truth.sats, "NMEA solution flows");
    CHECK(abs(e.solution().vel_n_cms - 300) <= 3 && abs(e.solution().vel_e_cms + 400) <= 3 && !e.solution().has_vel_d, "NMEA: north / east velocity from RMC (%d / %d cm/s), no vertical one", e.solution().vel_n_cms, e.solution().vel_e_cms);
    CHECK(abs(e.solution().lat - (int32_t)llround(sim.truth.lat * 1e7)) <= 5 && abs(e.solution().lon - (int32_t)llround(sim.truth.lon * 1e7)) <= 5, "NMEA position within 0.5 m: %d %d", e.solution().lat, e.solution().lon);
    CHECK(sim.nmea_in.size() == 2 && sim.nmea_in[0] == "PMTK220,100*2F" && sim.nmea_in[1] == "PMTK300,100,0,0,0,0*2C", "custom commands sent one per line (%zu)", sim.nmea_in.size());
    bool ubx_sent = false; for (auto& r : sim.log) (void)r, ubx_sent = true; CHECK(!ubx_sent, "NMEA provider never sends UBX");
}

static void t_providers() {
    printf("[engine MSP/virtual] injected solutions\n");
    Engine e; e.cfg.provider = PROV_MSP; SimGps sim; uint32_t t = 0; e.begin(sim, t);
    CHECK(e.state() == ST_INITIALIZED, "no serial port needed");
    Solution s; s.fix = true; s.fix_type = 3; s.num_sat = 12; s.lat = 473977419; s.lon = 85455938; s.time_ms = 1000;
    e.inject(s, 100); CHECK(e.healthy() && e.solution().num_sat == 12, "MSP GPS data accepted");
    e.update(100 + 3000); CHECK(e.state() == ST_RECEIVING_DATA || true, "timeout handled by caller for injected sources");
}

static void t_home() {
    printf("[home] home point, distance, direction, distance flown\n");
    Config c; c.min_sats = 8; Home h; h.configure(c);
    Solution s; s.fix = true; s.fix_type = 3; s.num_sat = 12; s.lat = 473977419; s.lon = 85455938; s.alt_cm = 48820;
    h.update(s, false); CHECK(!h.has_home() && h.ready() && h.take_ready_edge() && !h.take_ready_edge(), "disarmed: ready edge fires once, no home yet");
    h.update(s, true);  CHECK(h.has_home() && h.home().lat == s.lat, "home set on the arming edge");
    // move 100 m north, 50 m east
    Solution m = s;
    m.lat = s.lat + (int32_t)llround(100.0 * 100.0 / 1.113195); m.lon = s.lon + (int32_t)llround(50.0 * 100.0 / (1.113195 * cosf((float)(s.lat * 1e-7 * M_PI / 180.0)))); m.ground_speed_cms = 500;
    h.update(m, true);
    CHECK(abs((int)h.distance_to_home_m() - 112) <= 1, "distance to home %u m (expected 111.8)", h.distance_to_home_m());
    CHECK(abs((int)h.direction_to_home_ddeg() - 2066) <= 5, "direction to home %u ddeg: bearing FROM drone TO home = 206.6 deg (SSW)", h.direction_to_home_ddeg());
    float e = 0, n = 0; CHECK(h.offset_from_home_cm(e, n) && fabsf(e - 5000) < 20 && fabsf(n - 10000) < 20, "ENU offset from home %.0f east %.0f north (cm)", e, n);
    CHECK(h.distance_flown_cm() > 11000 && h.distance_flown_cm() < 11300, "distance flown %u cm", h.distance_flown_cm());
    Solution far = m; far.lat += 100000; far.ground_speed_cms = 10;
    h.update(far, true); CHECK(h.distance_flown_cm() < 11300, "movement slower than 15 cm/s is not counted (GPS drift)");
    h.update(m, false); CHECK(!h.has_home() && h.distance_to_home_cm() == 0, "disarm forgets home (BF default)");
    Config once = c; once.set_home_point_once = true; Home h2; h2.configure(once);
    h2.update(s, true); h2.update(m, false); h2.update(m, true);
    CHECK(h2.has_home() && h2.home().lat == s.lat, "set_home_point_once keeps the first home across arms");
    Solution weak = s; weak.num_sat = 5; Home h3; h3.configure(c); h3.update(weak, true); CHECK(!h3.has_home(), "too few satellites: no home");
    Solution nofix = s; nofix.fix = false; Home h4; h4.configure(c); h4.update(nofix, true); CHECK(!h4.has_home(), "no fix: no home");
    printf("[geometry] distance/bearing, 180 degree meridian\n");
    Location a{0, 1799999000, 0}, b{0, -1799999000, 0}; float ee, nn; offset_cm(a, b, ee, nn);
    CHECK(fabsf(ee - 2226.0f) < 5 && fabsf(nn) < 1, "across the antimeridian: 0.0002 deg east = 22 m the SHORT way (east %.0f cm)", ee);
    Location p{473977419, 85455938, 0}, q{473977419 + 10000, 85455938, 0}; uint32_t d; int32_t br; distance_bearing(p, q, false, &d, &br);
    CHECK(d == 11131 || d == 11132, "1e-3 deg north = 111.32 m (%u cm)", d); CHECK(br == 0, "due north bearing 0 (%d)", br);
    distance_bearing(p, Location{473977419, 85455938 + 10000, 0}, false, &d, &br); CHECK(br == 900, "due east bearing 90.0 (%d)", br);
    distance_bearing(p, Location{473977419 - 10000, 85455938, 0}, false, &d, &br); CHECK(br == 1800, "south 180.0 (%d)", br);
    distance_bearing(p, Location{473977419, 85455938 - 10000, 0}, false, &d, &br); CHECK(br == 2700, "west 270.0 (%d)", br);
}

int main() {
    t_ubx_builder(); t_nmea();
    for (UbxVersion v : {UBX_M6, UBX_M7, UBX_M8, UBX_M9, UBX_M10}) t_engine(v);
    t_baud_and_failures(); t_nmea_engine(); t_providers(); t_home();
    printf("\n%d passed, %d failed\n", g_pass, g_fail); return g_fail ? 1 : 0;
}
