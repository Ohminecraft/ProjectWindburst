// Prints, as JSON, what the FIRMWARE computes for the things the configurator previews: mixer rows per frame layout,
// the rate curve, the throttle curve and the stick deadband. app/tests/test_gui_math.py compares these numbers with
// app/wbclient/web/js/gfx/fwmath.js, so the pictures in the GUI can not drift away from the flight code.
//   g++ -std=c++17 tools/ui_reference.cpp -o build/ui_reference && build/ui_reference
#include <stdio.h>
#include "../src/proto/settings_apply.h"
#include "../src/port/pin_names.h"
#include "../src/app/led_logic.h"

int main() {
    using namespace fc;
    printf("{\n \"layouts\": {");
    const char* names[] = {"QUAD_X", "QUAD_PLUS", "HEX_X", "HEX_PLUS", "HEX_X_BF", "HEX_H", "Y6", "Y4", "OCTO_X8", "OCTO_FLAT_PLUS", "OCTO_FLAT_X", "VTAIL4", "ATAIL4", "QUAD_X_1234"};
    static_assert(sizeof(names) / sizeof(names[0]) == LAYOUT_COUNT, "one name per layout");
    for (int l = 0; l < LAYOUT_COUNT; l++) {
        printf("%s\n  \"%s\": [", l ? "," : "", names[l]);
        for (int m = 0; m < mixer_motor_count((Layout)l); m++) { MixRow r = mixer_row((Layout)l, m); printf("%s[%.6f, %.6f, %.6f]", m ? ", " : "", (double)r.roll, (double)r.pitch, (double)r.yaw); }
        printf("]");
    }
    // props_out is applied inside Mixer::mix(): measure it through the mixer itself (yaw command only)
    printf("\n },\n \"yaw_response\": {");
    for (int po = 0; po < 2; po++) {
        Mixer mx; mx.cfg.layout = QUAD_X; mx.cfg.props_out = po != 0; mx.configure();
        float lo[kMaxMotors], hi[kMaxMotors]; MotorLimits lim{0.0f, 1.0f};
        mx.mix(lim, 0.5f, 0, 0, -0.1f, lo); mx.mix(lim, 0.5f, 0, 0, 0.1f, hi);
        printf("%s\"%s\": [", po ? ", " : "", po ? "props_out" : "normal");
        for (int m = 0; m < 4; m++) printf("%s%.6f", m ? ", " : "", (double)((hi[m] - lo[m]) / 0.2f));
        printf("]");
    }
    printf("},\n \"rate\": [");
    bool first = true;
    const int cs[] = {70, 10, 200, 500}, mr[] = {670, 100, 1998, 400}, ex[] = {0, 35, 100};
    for (int a = 0; a < 4; a++) for (int e = 0; e < 3; e++) for (int k = -10; k <= 10; k++) {
        wbp::Settings s; wbp::defaults(s); CoreConfig cfg; RcInterpreter rc;
        s.rates.center_sens[1] = (uint16_t)cs[a]; s.rates.max_rate[1] = (uint16_t)mr[a]; s.rates.expo[1] = (uint8_t)ex[e];
        wbp::apply_core(s, cfg, rc);
        const float st = k / 10.0f;
        printf("%s\n  [%d, %d, %d, %.3f, %.5f]", first ? "" : ",", cs[a], mr[a], ex[e], (double)st, (double)apply_actual_rate(cfg.rates, PITCH, st)); first = false;
    }
    printf("\n ],\n \"throttle\": ["); first = true;
    const int mid[] = {50, 30, 70}, tex[] = {0, 40, 100}, lim[] = {100, 60};
    for (int a = 0; a < 3; a++) for (int e = 0; e < 3; e++) for (int l = 0; l < 2; l++) for (int k = 0; k <= 10; k++) {
        wbp::Settings s; wbp::defaults(s); CoreConfig cfg; RcInterpreter rc;
        s.rates.throttle_mid = (uint8_t)mid[a]; s.rates.throttle_expo = (uint8_t)tex[e]; s.rates.throttle_limit = (uint8_t)lim[l];
        wbp::apply_core(s, cfg, rc);
        printf("%s\n  [%d, %d, %d, %.2f, %.6f]", first ? "" : ",", mid[a], tex[e], lim[l], k / 10.0, (double)rc.curve(k / 10.0f)); first = false;
    }
    printf("\n ],\n \"stick\": ["); first = true;
    const int db[] = {0, 2, 10, 20};
    for (int d = 0; d < 4; d++) for (int us = 900; us <= 2100; us += 37) {
        wbp::Settings s; wbp::defaults(s); CoreConfig cfg; RcInterpreter rc;
        s.receiver.deadband = (uint8_t)db[d];
        wbp::apply_core(s, cfg, rc);
        uint16_t ch[16]; for (int i = 0; i < 16; i++) ch[i] = 1500; ch[2] = 1000; ch[0] = (uint16_t)us; ch[2] = (uint16_t)us;
        const RcCommand c = rc.process(ch, true);
        printf("%s\n  [%d, %d, %.6f, %.6f]", first ? "" : ",", us, db[d], (double)c.roll, (double)c.throttle_raw); first = false;
    }
    // the stick centre (receiver.mid_rc, Betaflight's mid_rc), deadband 0
    printf("\n ],\n \"stick_mid\": ["); first = true;
    const int mids[] = {1480, 1500, 1530};
    for (int m = 0; m < 3; m++) for (int us = 900; us <= 2100; us += 53) {
        wbp::Settings s; wbp::defaults(s); CoreConfig cfg; RcInterpreter rc;
        s.receiver.deadband = 0; s.receiver.mid_rc = (uint16_t)mids[m];
        wbp::apply_core(s, cfg, rc);
        uint16_t ch[16]; for (int i = 0; i < 16; i++) ch[i] = 1500; ch[0] = (uint16_t)us;
        printf("%s\n  [%d, %d, %.6f]", first ? "" : ",", us, mids[m], (double)rc.process(ch, true).roll); first = false;
    }
    // attitude: for a spread of orientations, the angles the telemetry would report and the true body->world rotation (rows: north, east, down)
    printf("\n ],\n \"attitude\": ["); first = true;
    uint32_t seed = 12345;
    auto rnd = [&]() { seed = seed * 1664525u + 1013904223u; return (float)((seed >> 8) & 0xFFFF) / 32768.0f - 1.0f; };
    for (int k = 0; k < 60; k++) {
        float q[4] = {rnd(), rnd(), rnd(), rnd()}; const float n = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        if (n < 0.2f) continue;
        Ahrs a; a.q0 = q[0] / n; a.q1 = q[1] / n; a.q2 = q[2] / n; a.q3 = q[3] / n;
        const EulerDeg e = a.euler_user();
        if (fabsf(e.pitch) > 80.0f) continue;                       // the angles are not unique at the poles
        const float q0 = a.q0, q1 = a.q1, q2 = a.q2, q3 = a.q3;
        const float R[9] = {1 - 2 * (q2 * q2 + q3 * q3), 2 * (q1 * q2 - q0 * q3), 2 * (q1 * q3 + q0 * q2),
                            2 * (q1 * q2 + q0 * q3), 1 - 2 * (q1 * q1 + q3 * q3), 2 * (q2 * q3 - q0 * q1),
                            2 * (q1 * q3 - q0 * q2), 2 * (q2 * q3 + q0 * q1), 1 - 2 * (q1 * q1 + q2 * q2)};
        // the same matrix must agree with what the estimator itself uses: tilt from vertical = R[2][2]
        if (fabsf(R[8] - a.up_cos()) > 1e-5f) { fprintf(stderr, "rotation convention mismatch\n"); return 1; }
        printf("%s\n  [%.5f, %.5f, %.5f", first ? "" : ",", (double)e.roll, (double)e.pitch, (double)e.yaw); first = false;
        for (int i = 0; i < 9; i++) printf(", %.6f", (double)R[i]);
        printf("]");
    }
    // beeper patterns: name -> on/off durations (unit 10 ms), exactly the table the buzzer plays
    printf("\n ],\n \"beeper\": {"); first = true;
    for (int m = 0; m < beep::MODE_COUNT; m++) {
        if (!beep::has_sound((uint8_t)m)) continue;
        printf("%s\n  \"%s\": [", first ? "" : ",", beep::kTable[m].name); first = false;
        for (const uint8_t* s = beep::kTable[m].sequence; *s != beep::kStop; s++) printf("%s%u", s == beep::kTable[m].sequence ? "" : ", ", (unsigned)*s);
        printf("]");
    }
    printf("\n },\n \"beeper_order\": [");
    for (int m = 0; m < beep::MODE_COUNT; m++) printf("%s\"%s\"", m ? ", " : "", beep::kTable[m].name);
    // the motors as the sounder: which conditions they can never play, the default set, and how many ESC tones each pattern becomes
    // (counted by playing it on a Beeper set up like the firmware's motor sounder)
    printf("],\n \"motor_never\": ["); first = true;
    for (int m = 0; m < beep::MODE_COUNT; m++) if ((beep::kMotorNever >> m) & 1u) { printf("%s\"%s\"", first ? "" : ", ", beep::kTable[m].name); first = false; }
    printf("],\n \"motor_default\": ["); first = true;
    { wbp::S_beeper sb; wbp::defaults(sb); for (int m = 0; m < beep::MODE_COUNT; m++) if ((sb.motor_conditions >> m) & 1u) { printf("%s\"%s\"", first ? "" : ", ", beep::kTable[m].name); first = false; } }
    printf("],\n \"motor_tones\": {"); first = true;
    for (int m = 0; m < beep::MODE_COUNT; m++) {
        if (!beep::has_sound((uint8_t)m)) continue;
        beep::Beeper b; wbp::Settings s; wbp::defaults(s); beep::Beeper buzzer; wbp::apply_beeper(s, buzzer, b);
        b.force((uint8_t)m); int tones = 0; uint32_t ms = 0, first_ms = 0, last_ms = 0;
        for (; ms < 20000 && (ms == 0 || b.current() != beep::NO_MODE); ms++) { b.update(ms); if (b.take_start()) { if (!tones) first_ms = ms; last_ms = ms; tones++; } }
        printf("%s\n  \"%s\": [%d, %u]", first ? "" : ",", beep::kTable[m].name, tones, (unsigned)(last_ms - first_ms)); first = false;
    }
    printf("\n },\n \"beacon_slot_ms\": %u,\n", (unsigned)beep::kBeaconIntervalMs);
    // the LED strip, layout MANUAL (src/app/led_logic.h): the GUI's preview (fwmath.js ledColors) must show the same colors
    {   static const char* const fnames[] = {"OFF", "STATUS", "COLOR", "MODE", "ARMED", "BLINK", "STROBE", "BREATHE", "RAINBOW", "LARSON", "BATTERY", "THROTTLE"};
        static_assert(sizeof(fnames) / sizeof(fnames[0]) == app::LedLogic::F_COUNT, "one name per function");
        static const char* const modes[] = {"ACRO", "ANGLE", "STABILIZE", "ALTHOLD", "POSHOLD", "RESCUE"};
        printf(" \"led_functions\": ["); for (int f = 0; f < app::LedLogic::F_COUNT; f++) printf("%s\"%s\"", f ? ", " : "", fnames[f]);
        printf("],\n \"led\": ["); first = true;
        uint32_t seed2 = 777; auto rnd2 = [&](uint32_t m) { seed2 = seed2 * 1664525u + 1013904223u; return (seed2 >> 8) % m; };
        const uint32_t times[] = {0, 37, 99, 450, 999, 1000, 1499, 2600, 4321, 7777, 9999, 123457};
        for (int c = 0; c < 24; c++) {
            const int count = c == 0 ? 32 : 1 + (int)rnd2(32); const uint8_t bright = (uint8_t)(c % 3 == 0 ? 100 : 5 + rnd2(96)), speed = (uint8_t)(c % 4 == 0 ? 5 : rnd2(12));
            const bool warn = c % 5 != 1; uint8_t fn[32]; uint32_t col[32];
            for (int i = 0; i < 32; i++) { fn[i] = (uint8_t)(c < 12 ? (i + c) % app::LedLogic::F_COUNT : rnd2(app::LedLogic::F_COUNT)); col[i] = (uint32_t)rnd2(0x1000000); }
            app::LedLogic L; L.configure(count, bright, app::LedLogic::MANUAL); L.manual(fn, col, warn, speed);
            app::LedInput in; const int st = c % 6;
            in.armed = st >= 2; in.ready = st != 1; in.failsafe = st == 4; in.batt_critical = st == 5 && c % 2; in.stall = st == 5 && !(c % 2); in.batt_warning = st == 3;
            in.mode = (app::LedInput::Mode)rnd2(6); in.batt_pct = (uint8_t)(c % 7 == 0 ? 255 : rnd2(101)); in.throttle = (uint8_t)rnd2(101);
            for (uint32_t t : times) {
                app::Rgb out[32]; const int n = L.render(in, t, out);
                printf("%s\n  {\"count\": %d, \"brightness\": %u, \"speed\": %u, \"warnings\": %s, \"fn\": [", first ? "" : ",", count, (unsigned)bright, (unsigned)speed, warn ? "true" : "false"); first = false;
                for (int i = 0; i < 32; i++) printf("%s\"%s\"", i ? ", " : "", fnames[fn[i]]);
                printf("], \"color\": ["); for (int i = 0; i < 32; i++) printf("%s%u", i ? ", " : "", (unsigned)col[i]);
                printf("], \"st\": {\"armed\": %s, \"ready\": %s, \"failsafe\": %s, \"batt_critical\": %s, \"stall\": %s, \"batt_warning\": %s, \"mode\": \"%s\", \"batt\": %u, \"thr\": %u}, \"t\": %u, \"out\": [",
                       in.armed ? "true" : "false", in.ready ? "true" : "false", in.failsafe ? "true" : "false", in.batt_critical ? "true" : "false", in.stall ? "true" : "false", in.batt_warning ? "true" : "false",
                       modes[in.mode], (unsigned)in.batt_pct, (unsigned)in.throttle, (unsigned)t);
                for (int i = 0; i < n; i++) printf("%s[%u, %u, %u]", i ? ", " : "", out[i].r, out[i].g, out[i].b);
                printf("]}");
            }
        }
        printf("\n ],\n");
    }
    // the stick preview (js/gfx/stickpreview.js): Acro = the gyro alone through the Ahrs; Angle = AngleController's target
    {   printf(" \"acro\": ["); first = true;
        // segments of constant user rates (roll right, pitch nose down, yaw nose right, deg/s) and how long each lasts
        static const float segs[][4] = {{200, 0, 0, 0.5f}, {0, 300, 0, 0.4f}, {0, 0, 400, 0.3f}, {150, -250, 300, 0.6f}, {-500, 120, -80, 0.35f}, {0, 0, 0, 0.2f}};
        Ahrs a; const float dt = 0.002f;
        for (const auto& sg : segs) {
            const int n = (int)(sg[3] / dt + 0.5f);
            for (int k = 0; k < n; k++) { Vec3 g; g.x = sg[0] * kDeg2Rad; g.y = -sg[1] * kDeg2Rad; g.z = sg[2] * kDeg2Rad; a.update(g, Vec3(), dt); }
            const float q0 = a.q0, q1 = a.q1, q2 = a.q2, q3 = a.q3;
            const float R[9] = {1 - 2 * (q2 * q2 + q3 * q3), 2 * (q1 * q2 - q0 * q3), 2 * (q1 * q3 + q0 * q2),
                                2 * (q1 * q2 + q0 * q3), 1 - 2 * (q1 * q1 + q3 * q3), 2 * (q2 * q3 - q0 * q1),
                                2 * (q1 * q3 - q0 * q2), 2 * (q2 * q3 + q0 * q1), 1 - 2 * (q1 * q1 + q2 * q2)};
            printf("%s\n  [%.1f, %.1f, %.1f, %.3f, %d", first ? "" : ",", (double)sg[0], (double)sg[1], (double)sg[2], (double)dt, n); first = false;
            for (int i = 0; i < 9; i++) printf(", %.6f", (double)R[i]);
            printf("]");
        }
        printf("\n ],\n \"angle\": ["); first = true;
        const int lims[] = {55, 30, 80}, cs[] = {70, 200, 700}, mr[] = {670, 400, 400}, ex[] = {0, 50};      // (700 > 400: a full stick asks for more than the limit)
        for (int l : lims) for (int c = 0; c < 3; c++) for (int e = 0; e < 2; e++) for (int k = -10; k <= 10; k += 2) {
            wbp::Settings s; wbp::defaults(s); CoreConfig cfg; RcInterpreter rc;
            s.rates.angle_limit = (uint8_t)l; s.rates.center_sens[0] = (uint16_t)cs[c]; s.rates.max_rate[0] = (uint16_t)mr[c]; s.rates.expo[0] = (uint8_t)ex[e];
            wbp::apply_core(s, cfg, rc);
            AngleController ac; ac.configure(cfg.angle, 1000.0f);
            const float st = k / 10.0f; float sp[3] = {apply_actual_rate(cfg.rates, ROLL, st), 0, 0}; const float ff[2] = {0, 0};
            ac.update(sp, ff, 0, 0, cfg.rates.max_rate_dps);
            printf("%s\n  [%d, %d, %d, %d, %.2f, %.4f]", first ? "" : ",", l, cs[c], mr[c], ex[e], (double)st, (double)ac.target_deg(ROLL)); first = false;
        }
        printf("\n ],\n");
    }
    // the pin names (src/port/pin_names.h): the GUI's js/pinname.js must name and read pins the same way
    printf(" \"pin_names\": ["); first = true;
    for (int c = 0; c < 256; c++) { char a[12], b[12]; port::pin_name((uint8_t)c, true, a, sizeof(a)); port::pin_name((uint8_t)c, false, b, sizeof(b)); printf("%s[%d, \"%s\", \"%s\"]", first ? "" : ", ", c, a, b); first = false; }
    static const char* const texts[] = {"PB3", "pb03", "B3", " PC13 ", "PK15", "PB16", "PL0", "P", "PB3x", "12", "GPIO12", "gpio48", "255", "254", "", "PA", "A0", "p0"};
    printf("],\n \"pin_parse\": ["); first = true;
    for (const char* t : texts) { printf("%s[\"%s\", %d, %d]", first ? "" : ", ", t, port::pin_parse(t, true), port::pin_parse(t, false)); first = false; }
    printf("]\n}\n");
    return 0;
}
