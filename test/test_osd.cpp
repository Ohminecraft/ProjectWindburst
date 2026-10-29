// OSD over MSP DisplayPort (src/osd/osd.h): the elements as Betaflight draws them, the bytes on the wire, the video transmitter's
// requests. The character codes are Betaflight's font (drivers/osd_symbols.h), which the goggles carry.
#include <stdio.h>
#include <string.h>
#include <string>
#include <vector>
#include "../src/osd/osd.h"

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, ...) do { if (cond) g_pass++; else { g_fail++; printf("  FAIL line %d: %s   ", __LINE__, #cond); printf(__VA_ARGS__); printf("\n"); } } while (0)

using namespace osd;

static std::string text(Element e, const State& s, uint8_t* attr = nullptr) {
    Layout l; strcpy(l.name, "WINDBURST"); char b[32]; uint8_t a = 0;
    if (!element_text(e, s, l, b, sizeof(b), a)) return "<none>";
    if (attr) *attr = a;
    return b;
}
static std::string sym(std::initializer_list<int> parts) { std::string r; for (int p : parts) r += (char)p; return r; }

static void t_elements() {
    printf("[elements] Betaflight's look, Betaflight's font\n");
    State s; s.batt = true; s.volts = 16.42f; s.cells = 4; s.cell_min = 3.3f; s.cell_max = 4.3f;
    CHECK(text(E_BATT, s) == sym({SYM_BATT_EMPTY - 6}) + "16.4" + sym({SYM_VOLT}), "16.42 V on 4S: full battery symbol, one decimal from 10 V up");
    s.volts = 4 * 3.55f; CHECK(text(E_BATT, s) == sym({SYM_BATT_EMPTY - 2}) + "14.2" + sym({SYM_VOLT}), "3.55 V a cell: two of seven (osdGetBatterySymbol)");
    s.volts = 8.4f; s.cells = 2; CHECK(text(E_BATT, s) == sym({SYM_BATT_EMPTY - 6}) + "8.40" + sym({SYM_VOLT}), "below 10 V: two decimals");
    s.batt_level = 2; uint8_t a = 0;
    CHECK(text(E_BATT, s, &a)[0] == (char)SYM_MAIN_BATT && a == kAttrBlink, "critical: the BAT- symbol, blinking");
    s.batt_level = 0; CHECK(text(E_CELL, s) == sym({SYM_BATT_EMPTY - 6}) + "4.20" + sym({SYM_VOLT}), "average cell");
    s.batt = false; CHECK(text(E_BATT, s) == "<none>" && text(E_CELL, s) == "<none>", "no battery measured: nothing");
    s.have_current = true; s.amps = 12.345f; s.mah = 1234.4f;
    CHECK(text(E_CURRENT, s) == "12.35" + sym({SYM_AMP}) && text(E_MAH, s) == "1234" + sym({SYM_MAH}), "current, mAh");
    s.mode = MODE_ALTHOLD; CHECK(text(E_MODE, s) == "ALTH", "flight mode");
    s.failsafe = true; CHECK(text(E_MODE, s) == "!FS!", "failsafe wins"); s.failsafe = false;
    // 0.36: a rescue that flies - ArduPilot's mode names, and on the warning line the way still to fly home
    { State r = s; char w[24];
      r.rescue = RESC_RTL; r.rescue_left_m = 123.4f; rescue_text(r, w, sizeof(w)); CHECK(text(E_MODE, r) == "RTL" && std::string(w) == "RTL 123M", "RTL on the way: mode RTL, '%s'", w);
      r.rescue = RESC_SMART_RTL; rescue_text(r, w, sizeof(w)); CHECK(text(E_MODE, r) == "SRTL" && std::string(w) == "SRTL 123M", "SmartRTL: SRTL, '%s' (along the way it came)", w);
      r.rescue_stage = RSTAGE_HOME; rescue_text(r, w, sizeof(w)); const std::string home = w; r.rescue_stage = RSTAGE_LANDING; rescue_text(r, w, sizeof(w));
      CHECK(home == "SRTL HOME" && std::string(w) == "SRTL LAND", "over home: '%s', then '%s'", home.c_str(), w);
      r.rescue = RESC_LAND; r.alt_valid = true; r.alt_m = 12.4f; rescue_text(r, w, sizeof(w)); const std::string land = w; r.alt_valid = false; rescue_text(r, w, sizeof(w));
      CHECK(text(E_MODE, r) == "LAND" && land == "LAND 12M" && std::string(w) == "LAND", "LAND: the height still to come down ('%s'; without a height '%s')", land.c_str(), w);
      r.rescue = RESC_RTL; r.rescue_stage = RSTAGE_WAY; r.rescue_left_m = 1e6f; rescue_text(r, w, sizeof(w)); CHECK(std::string(w) == "RTL 65000M", "never more digits than fit (%s)", w);
      r.failsafe = true; CHECK(text(E_MODE, r) == "!FS!", "a failsafe rescue: the mode says !FS! (the warning line says RTL)"); }
    CHECK(text(E_NAME, s) == "WINDBURST", "craft name");
    s.fly_s = 125; s.on_s = 3599; CHECK(text(E_FLY_TIME, s) == sym({SYM_FLY_M}) + "02:05" && text(E_ON_TIME, s) == sym({SYM_ON_M}) + "59:59", "timers mm:ss");
    s.rc_link = true; s.rssi_pct = 100; CHECK(text(E_RSSI, s) == sym({SYM_LINK_QUALITY}) + "99", "link quality, at most 99");
    s.rc_link = false; CHECK(text(E_RSSI, s, &a) == sym({SYM_LINK_QUALITY}) + "0" && a == kAttrBlink, "no link: 0, blinking");
    CHECK(text(E_SATS, s) == "<none>" && text(E_SPEED, s) == "<none>" && text(E_HOME_DIR, s) == "<none>", "no GPS fitted: no GPS elements");
    s.gps = true; s.sats = 12; s.fix = 3; s.speed_ms = 10.0f;
    CHECK(text(E_SATS, s, &a) == sym({SYM_SAT_L, SYM_SAT_R}) + "12" && a == 0, "satellites"); s.fix = 1; text(E_SATS, s, &a); CHECK(a == kAttrBlink, "no 3D fix: blinking"); s.fix = 3;
    CHECK(text(E_SPEED, s) == sym({SYM_SPEED}) + "36" + sym({SYM_KPH}), "10 m/s = 36 km/h");
    CHECK(text(E_HOME_DIR, s) == sym({SYM_HYPHEN}) && text(E_HOME_DIST, s) == sym({SYM_HOMEFLAG, SYM_HYPHEN, SYM_HYPHEN, SYM_M}), "no home yet: dashes");
    s.home = true; s.home_dist_m = 123.4f; s.heading_deg = 0;
    const struct { float dir; int sym; } dirs[] = {{0, 0x68}, {90, 0x64}, {180, 0x60}, {270, 0x6C}, {45, 0x66}, {359, 0x68}};
    bool ok = true; for (auto& d : dirs) { s.home_dir_deg = d.dir; ok = ok && text(E_HOME_DIR, s) == sym({d.sym}); }
    CHECK(ok, "home arrow: ahead = up (0x68), right = 0x64, behind = 0x60, left = 0x6C (osdGetDirectionSymbolFromHeading)");
    s.home_dir_deg = 90; s.heading_deg = 90; CHECK(text(E_HOME_DIR, s) == sym({0x68}), "relative to the nose: home east, flying east = ahead");
    CHECK(text(E_HOME_DIST, s) == sym({SYM_HOMEFLAG}) + "123" + sym({SYM_M}), "home distance");
    s.home_dist_m = 0.5f; CHECK(text(E_HOME_DIR, s) == sym({SYM_OVER_HOME}), "over home");
    CHECK(text(E_ALT, s) == sym({SYM_ALTITUDE, SYM_HYPHEN, SYM_HYPHEN, SYM_M}) && text(E_VARIO, s) == "<none>", "no height estimate: dashes, no vario");
    s.alt_valid = true; s.alt_m = 12.34f; s.vz_ms = -1.26f;
    CHECK(text(E_ALT, s) == sym({SYM_ALTITUDE}) + "12.3" + sym({SYM_M}) && text(E_VARIO, s) == sym({SYM_ARROW_SMALL_DOWN}) + "1.3" + sym({SYM_MPS}), "altitude, vario (sinking)");
    s.throttle = 0.456f; CHECK(text(E_THROTTLE, s) == sym({SYM_THR}) + "46", "throttle %%");
    CHECK(text(E_CROSSHAIRS, s) == sym({SYM_AH_CENTER_LINE, SYM_AH_CENTER, SYM_AH_CENTER_LINE_RIGHT}), "crosshairs");
    CHECK(text(E_WARNINGS, s) == "<none>", "no warning: nothing"); strcpy(s.warning, "RXLOSS"); CHECK(text(E_WARNINGS, s, &a) == "RXLOSS" && a == kAttrBlink, "a warning blinks");
    s.heading_deg = 7.4f; CHECK(text(E_HEADING, s) == sym({0x68}) + "007", "heading, three digits");
    s.pitch_deg = -12.4f; s.roll_deg = 5.0f; CHECK(text(E_PITCH, s) == sym({SYM_PITCH}) + "-12" && text(E_ROLL, s) == sym({SYM_ROLL}) + " 05", "pitch / roll angles");
    CHECK(text(E_ESC_TEMP, s) == "<none>", "no ESC temperature: nothing"); s.esc_temp_c = 45; CHECK(text(E_ESC_TEMP, s) == "45" + sym({SYM_C}), "ESC temperature");
    CHECK(text(E_DISARMED, s) == "DISARMED", "disarmed"); s.armed = true; CHECK(text(E_DISARMED, s) == "<none>", "armed: gone");
}

static void t_horizon() {
    printf("[artificial horizon] osdElementArtificialHorizon, 20 / 40 degree limits\n");
    std::vector<std::vector<int>> got;
    auto run = [&](float roll, float pitch) { State s; s.roll_deg = roll; s.pitch_deg = pitch; got.clear(); horizon(s, [&](int dx, int dy, uint8_t c) { got.push_back({dx, dy, c}); }); };
    run(0, 0); bool ok = got.size() == 9; for (auto& g : got) ok = ok && g[1] == 4 && g[2] == SYM_AH_BAR9_0 + 5;
    CHECK(ok, "level: 9 columns on the middle row (offset 4), bar 5");
    run(20, 0); CHECK(got.size() == 9 && got[0][0] == -4 && got[0][1] == 5 && got[0][2] == SYM_AH_BAR9_0 + 8 && got[8][0] == 4 && got[8][1] == 3 && got[8][2] == SYM_AH_BAR9_0 + 2,
                      "right wing down 20 deg: left end lower (row 5 bar 8), right end higher (row 3 bar 2) - Betaflight's integer maths");
    run(0, 20); ok = true; for (auto& g : got) ok = ok && g[1] == 1 && g[2] == SYM_AH_BAR9_0 + 7;
    CHECK(ok, "nose down 20 deg (the limit): the horizon moves up the screen (row 1, bar 7)");
    run(0, 90); std::vector<std::vector<int>> at_limit = got; run(0, 20); CHECK(at_limit == got, "beyond the pitch limit: as at the limit");
}

static void t_render() {
    printf("[render] shown elements only, inside the canvas\n");
    State s; s.batt = true; s.volts = 16.0f; s.cells = 4;
    Layout l; l.cols = 30; l.rows = 16;
    for (auto& p : l.pos) p = 0;
    l.pos[E_BATT] = make_pos(1, 14, true); l.pos[E_MODE] = make_pos(40, 1, true); l.pos[E_THROTTLE] = make_pos(29, 2, true); l.pos[E_DISARMED] = make_pos(5, 5, false);
    std::vector<std::string> w;
    render(s, l, [&](uint8_t row, uint8_t col, uint8_t attr, const char* t) { char h[16]; snprintf(h, sizeof(h), "%u,%u:", row, col); w.push_back(h + std::string(t)); });
    CHECK(w.size() == 2 && w[0].rfind("14,1:", 0) == 0 && w[1] == "2,29:" + sym({SYM_THR}), "battery at its place; flight mode off the 30 column canvas: left out; throttle cut at the right edge; hidden DISARMED left out (%zu)", w.size());
    CHECK(pos_col(make_pos(52, 19, true)) == 52 && pos_row(make_pos(52, 19, true)) == 19 && (make_pos(52, 19, true) & kVisible) && make_pos(1, 1, true) == (1 | 1 << 6 | 0x800), "Betaflight's position encoding");
}

// MSP v1 frames out of a byte stream
struct Fr { char dir; uint8_t cmd; std::vector<uint8_t> p; };
static std::vector<Fr> frames(const std::vector<uint8_t>& b, int& bad) {
    std::vector<Fr> out; bad = 0;
    for (size_t i = 0; i + 6 <= b.size();) {
        if (b[i] != '$' || b[i + 1] != 'M') { bad++; i++; continue; }
        const uint8_t n = b[i + 3], c = b[i + 4]; if (i + 6 + n > b.size()) { bad++; break; }
        uint8_t ck = n ^ c; Fr f; f.dir = (char)b[i + 2]; f.cmd = c;
        for (int k = 0; k < n; k++) { f.p.push_back(b[i + 5 + k]); ck ^= b[i + 5 + k]; }
        if (ck != b[i + 5 + n]) bad++; else out.push_back(f);
        i += 6 + n;
    }
    return out;
}
static std::vector<uint8_t> request(uint8_t cmd, std::vector<uint8_t> p) {
    std::vector<uint8_t> r = {'$', 'M', '<', (uint8_t)p.size(), cmd}; uint8_t ck = (uint8_t)(p.size() ^ cmd);
    for (uint8_t x : p) { r.push_back(x); ck ^= x; }
    r.push_back(ck); return r;
}

static void t_wire() {
    printf("[wire] screens as MSP DisplayPort, never more than the room\n");
    DisplayPort dp; dp.configure(DisplayPort::CANVAS_AUTO, 10);
    State s; s.batt = true; s.volts = 16.0f; s.cells = 4; Layout l;
    for (auto& p : l.pos) p = 0;
    l.pos[E_BATT] = make_pos(1, 18, true); l.pos[E_MODE] = make_pos(47, 18, true); l.pos[E_CROSSHAIRS] = make_pos(25, 10, true);
    uint8_t buf[4096]; size_t n = dp.poll(0, s, l, buf, sizeof(buf));
    const uint8_t hb[7] = {'$', 'M', '>', 1, 182, 0, 1 ^ 182 ^ 0};
    CHECK(n > 7 && memcmp(buf, hb, 7) == 0, "first a heartbeat: $M> 1 182 [0] checksum (Betaflight pushes 'replies')");
    int bad = 0; auto f = frames(std::vector<uint8_t>(buf, buf + n), bad);
    CHECK(bad == 0 && f.size() == 6 && f[1].p == std::vector<uint8_t>{DP_CLEAR} && f[5].p == std::vector<uint8_t>{DP_DRAW}, "heartbeat, clear, 3 strings, draw (%zu frames, %d bad)", f.size(), bad);
    CHECK(f[2].p[0] == DP_WRITE && f[2].p[1] == 18 && f[2].p[2] == 1 && f[2].p[3] == 0 && f[2].p.size() == 4 + 6, "write: row, column, attribute, then the characters");
    CHECK(f[3].p[1] == 18 && f[3].p[2] == 47 && std::string(f[3].p.begin() + 4, f[3].p.end()) == "ACRO", "flight mode at column 47 of the 53 column HD canvas");
    CHECK(dp.poll(50, s, l, buf, sizeof(buf)) == 0, "the next screen waits for its time (10 a second)");
    n = dp.poll(100, s, l, buf, sizeof(buf)); f = frames(std::vector<uint8_t>(buf, buf + n), bad);
    CHECK(f.size() == 5 && f[0].p[0] == DP_CLEAR, "100 ms later: the next screen, no heartbeat (once a second)");
    n = dp.poll(1000, s, l, buf, sizeof(buf)); f = frames(std::vector<uint8_t>(buf, buf + n), bad);
    CHECK(f.size() == 6 && f[0].p[0] == DP_HEARTBEAT, "a heartbeat every second");
    // a slow UART: 16 bytes of room per call - the bytes go out in order, a new screen only once the last is out
    std::vector<uint8_t> all; uint32_t t = 2000; size_t screens0 = dp.screens(); bool over = false;
    for (int k = 0; k < 400; k++, t += 5) { const size_t m = dp.poll(t, s, l, buf, 16); if (m > 16) over = true; all.insert(all.end(), buf, buf + m); }
    CHECK(!over, "never more than the 16 bytes of room");
    f = frames(all, bad);
    int draws = 0; for (auto& x : f) if (x.p[0] == DP_DRAW) draws++;
    CHECK(bad == 0 && draws >= 3 && (int)(dp.screens() - screens0) <= draws + 1, "16 bytes a call: %d whole screens, every frame intact, none started over an unfinished one", draws);
}

static void t_requests() {
    printf("[requests] what the video transmitter asks\n");
    DisplayPort dp; dp.configure(DisplayPort::CANVAS_AUTO, 10);
    msp::Info info; info.armed = true;
    State s; Layout l; for (auto& p : l.pos) p = 0;
    uint8_t buf[4096];
    dp.poll(0, s, l, buf, sizeof(buf));                                      // (the first screen is out)
    for (uint8_t c : request(101, {})) dp.feed(c, info, "MYQUAD");             // MSP_STATUS
    for (uint8_t c : request(188, {50, 18})) dp.feed(c, info, "MYQUAD");       // MSP_SET_OSD_CANVAS: HDZero's 50 x 18
    for (uint8_t c : request(10, {})) dp.feed(c, info, "MYQUAD");              // MSP_NAME
    for (uint8_t c : request(245, {0xFF})) dp.feed(c, info, "MYQUAD");         // ESC passthrough: never on this port
    size_t n = dp.poll(10, s, l, buf, sizeof(buf)); int bad = 0; auto f = frames(std::vector<uint8_t>(buf, buf + n), bad);
    CHECK(bad == 0 && f.size() == 4 && dp.requests() == 4, "four answers (%zu)", f.size());
    CHECK(f.size() == 4 && f[0].cmd == 101 && f[0].dir == '>' && f[0].p.size() >= 10 && (f[0].p[6] & 1), "MSP_STATUS: the ARM box is on (the goggles start recording)");
    CHECK(f.size() == 4 && f[1].cmd == 188 && f[1].p.empty() && dp.cols() == 50 && dp.rows() == 18 && dp.canvas_from_vtx(), "MSP_SET_OSD_CANVAS: acknowledged, the canvas is now 50 x 18");
    CHECK(f.size() == 4 && f[2].cmd == 10 && std::string(f[2].p.begin(), f[2].p.end()) == "MYQUAD", "MSP_NAME: the craft name");
    CHECK(f.size() == 4 && f[3].cmd == 245 && f[3].p.size() == 1 && f[3].p[0] == 0, "ESC passthrough refused (0 ESCs)");
    { DisplayPort fixed; fixed.configure(DisplayPort::CANVAS_SD, 10); for (uint8_t c : request(188, {50, 18})) fixed.feed(c, info, "");
      CHECK(fixed.cols() == 30 && fixed.rows() == 16, "a fixed canvas setting is kept whatever the transmitter asks"); }
    { DisplayPort d2; d2.configure(DisplayPort::CANVAS_AUTO, 10); for (uint8_t c : request(188, {200, 3})) d2.feed(c, info, "");
      CHECK(d2.cols() == 53 && d2.rows() == 20, "a canvas that makes no sense is ignored"); }
    { DisplayPort d3; d3.configure(DisplayPort::CANVAS_HDZERO, 25); CHECK(d3.cols() == 50 && d3.rows() == 18, "HDZero canvas"); }
}

static void t_elements_033() {
    printf("[elements 0.33] power, distance, coordinates, compass bar, sidebars, G, battery usage\n");
    State s; uint8_t a = 0;
    CHECK(text(E_POWER, s) == "<none>", "power: needs a battery and a current sensor");
    s.batt = true; s.volts = 16.0f; s.cells = 4; s.have_current = true; s.amps = -12.4f;
    CHECK(text(E_POWER, s) == "198W", "power: |A| x V, whole watts (%s)", text(E_POWER, s).c_str());
    s.gps = true; s.dist_m = 1234.6f; CHECK(text(E_FLIGHT_DIST, s) == sym({SYM_TOTAL_DISTANCE}) + "1235" + sym({SYM_M}), "distance flown");
    s.lat = 107654321; s.lon = -1066543210;
    CHECK(text(E_GPS_LAT, s) == sym({SYM_LAT}) + "10.7654321" && text(E_GPS_LON, s) == sym({SYM_LON}) + "-106.6543210", "coordinates: symbol, degrees, 7 decimals, sign (osdFormatCoordinate)");
    s.lat = 5; CHECK(text(E_GPS_LAT, s) == sym({SYM_LAT}) + "0.0000005", "a small one keeps its zeros");
    s.heading_deg = 0;
    CHECK(text(E_COMPASS_BAR, s) == sym({SYM_HEADING_W, SYM_HEADING_LINE, SYM_HEADING_DIVIDED_LINE, SYM_HEADING_LINE, SYM_HEADING_N, SYM_HEADING_LINE, SYM_HEADING_DIVIDED_LINE, SYM_HEADING_LINE, SYM_HEADING_E}),
          "compass bar heading north: N in the middle, W and E at the ends");
    s.heading_deg = 90; CHECK(text(E_COMPASS_BAR, s)[4] == (char)SYM_HEADING_E && text(E_COMPASS_BAR, s).size() == 9, "heading east: E in the middle");
    s.heading_deg = 200; CHECK(text(E_COMPASS_BAR, s)[4] == (char)SYM_HEADING_LINE && text(E_COMPASS_BAR, s)[3] == (char)SYM_HEADING_S, "heading 200: one sixteenth past S");
    s.heading_deg = -90; CHECK(text(E_COMPASS_BAR, s)[4] == (char)SYM_HEADING_W, "-90 = west");
    s.g_force = 1.04f; CHECK(text(E_G_FORCE, s) == "1.0G", "G force"); s.g_force = 2.46f; CHECK(text(E_G_FORCE, s) == "2.5G", "2.5G");
    CHECK(text(E_BATT_USAGE, s) == "<none>", "battery usage: nothing while what is left is unknown");
    s.batt_remaining = 75;
    CHECK(text(E_BATT_USAGE, s) == sym({SYM_PB_START, SYM_PB_FULL, SYM_PB_FULL, SYM_PB_FULL, SYM_PB_FULL, SYM_PB_FULL, SYM_PB_FULL, SYM_PB_FULL, SYM_PB_HALF, SYM_PB_EMPTY, SYM_PB_EMPTY, SYM_PB_CLOSE}),
          "75 %% left: seven full cells, a half one, two empty");
    s.batt_level = 1; text(E_BATT_USAGE, s, &a); CHECK(a == kAttrBlink, "low battery: the bar blinks");
    {   // the sidebars, drawn cell by cell
        Layout l; for (auto& p : l.pos) p = 0; l.pos[E_SIDEBARS] = make_pos(26, 10, true);
        std::vector<std::string> w; render(s, l, [&](uint8_t r, uint8_t c, uint8_t, const char* t) { w.push_back(std::to_string(r) + "," + std::to_string(c) + "," + std::to_string((uint8_t)t[0])); });
        CHECK(w.size() == 16 && w[0] == "7,19," + std::to_string(SYM_AH_DECORATION) && w[1] == "7,33," + std::to_string(SYM_AH_DECORATION) && w[14] == "10,20," + std::to_string(SYM_AH_LEFT) && w[15] == "10,32," + std::to_string(SYM_AH_RIGHT),
              "sidebars: 7 rows either side at +-7 columns, level marks at +-6 (%zu cells)", w.size());
        l.pos[E_SIDEBARS] = make_pos(2, 1, true); w.clear(); render(s, l, [&](uint8_t r, uint8_t c, uint8_t, const char* t) { w.push_back(std::to_string(r) + "," + std::to_string(c)); });
        CHECK(w.size() == 6, "near the corner: only what is on the canvas (%zu)", w.size());
        l.pos[E_SIDEBARS] = make_pos(50, 18, true); w.clear(); render(s, l, [&](uint8_t r, uint8_t c, uint8_t, const char* t) { w.push_back(std::to_string(r) + "," + std::to_string(c)); });
        CHECK(w.size() == 6, "near the bottom right corner: the left bar (5 rows) and its mark, nothing past the last column / row (%zu)", w.size());
    }
}

static void t_stats() {
    printf("[statistics] Betaflight's screen after a flight\n");
    DisplayPort dp; dp.configure(DisplayPort::CANVAS_HD, 10); Layout l; uint8_t out[4096];
    State s; s.batt = true; s.volts = 16.4f; s.cells = 4; s.have_current = true; s.gps = true; s.home = true; s.alt_valid = true;
    uint32_t t = 0;
    auto run = [&](int ms) { for (int i = 0; i < ms; i += 100) { t += 100; dp.poll(t, s, l, out, sizeof(out)); } };
    run(500); CHECK(!dp.showing_stats(), "no flight yet: no statistics");
    s.armed = true;
    for (int i = 0; i < 50; i++) { s.fly_s = (uint32_t)i / 10; s.speed_ms = i == 20 ? 25.0f : 5.0f; s.home_dist_m = (float)i * 2; s.alt_m = i == 30 ? 42.0f : 10.0f; s.volts = 16.4f - i * 0.02f; s.amps = i == 10 ? -60.0f : 20.0f; s.mah = (float)i * 10; s.dist_m = (float)i * 3; run(100); }
    const Stats& st = dp.stats();
    CHECK(st.max_speed_ms == 25.0f && st.max_dist_m == 98.0f && st.max_alt_m == 42.0f && fabsf(st.min_volts - (16.4f - 49 * 0.02f)) < 1e-4f && st.max_amps == 60.0f && st.mah == 490.0f && st.dist_m == 147.0f,
          "kept while armed: max speed, distance, altitude; min battery; max current; used mAh; distance flown");
    s.armed = false; s.volts = 16.0f;
    std::string screen; t += 100; size_t n = dp.poll(t, s, l, out, sizeof(out)); screen.assign((const char*)out, n);
    CHECK(dp.showing_stats() && screen.find("--- STATS ---") != std::string::npos && screen.find("MAX SPEED      : 90") != std::string::npos && screen.find("MAX ALTITUDE   : 42.0") != std::string::npos,
          "disarmed: the statistics screen (90 km/h, 42.0 m)");
    CHECK(screen.find("DISARMED") == std::string::npos, "instead of the elements");
    run(19000); CHECK(dp.showing_stats(), "still after 19 s");
    run(1200); CHECK(!dp.showing_stats(), "gone after 20 s");
    s.armed = true; run(300); s.armed = false; run(200); CHECK(dp.showing_stats(), "a new flight: shown again");
    CHECK(dp.stats().max_speed_ms == 5.0f && dp.stats().max_alt_m == 10.0f, "with that flight's own numbers (max speed %.1f m/s)", dp.stats().max_speed_ms);
    s.armed = true; run(200); CHECK(!dp.showing_stats(), "arming ends it at once");
    s.armed = false; l.stats = false; run(300); CHECK(!dp.showing_stats(), "osd.stats off: never shown");
}

int main() {
    t_elements();
    t_horizon();
    t_render();
    t_wire();
    t_requests();
    t_elements_033();
    t_stats();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
