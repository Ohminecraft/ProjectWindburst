// Host tests for motor protocol + manager. Reference values are computed independently in the test from the Betaflight
// formulas (dshot.c / dshot_command.c / mixer.c) or from first principles, not by calling the code under test.
#include <stdio.h>
#include <math.h>
#include <vector>
#include "../src/hal/motor/motor_protocol.h"
#include "../src/hal/motor/motor_output.h"
using namespace motor;

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, ...) do { if (cond) g_pass++; else { g_fail++; printf("  FAIL line %d: %s   ", __LINE__, #cond); printf(__VA_ARGS__); printf("\n"); } } while (0)

// independent BF-style checksum
static unsigned bf_csum(unsigned packet, bool inv) { unsigned c = 0, d = packet; for (int i = 0; i < 3; i++) { c ^= d; d >>= 4; } if (inv) c = ~c; return c & 0xF; }

static void t_frame() {
    printf("[dshot] frame + waveform\n");
    // known DShot vector: throttle 1046, no telemetry -> 0x082C (widely used example)   value<<1 = 2092 = 0x82C ; csum = 8^2^C = 0x2 ... check vs BF formula
    for (unsigned v : {0u, 1u, 47u, 48u, 1046u, 1047u, 2047u}) for (int t = 0; t < 2; t++) for (int inv = 0; inv < 2; inv++) {
        uint16_t pk = dshot_packet((uint16_t)v, t, inv);
        unsigned raw = (v << 1) | t;
        CHECK((pk >> 4) == raw && (pk & 0xF) == bf_csum(raw, inv), "v=%u t=%d inv=%d pk=0x%04x", v, t, inv, pk);
        CHECK(dshot_packet_valid(pk, inv) && !dshot_packet_valid(pk ^ 0x0010, inv), "valid/invalid");
    }
    CHECK(dshot_packet(5000, 0, false) == dshot_packet(2047, 0, false), "value clamps to 2047");
    Sym s[16]; uint16_t pk = dshot_packet(1046, 0, false);
    dshot_symbols(pk, PROTO_DSHOT600, 80000000, false, s);
    int ones = 0; for (int b = 0; b < 16; b++) {
        const bool one = (pk >> (15 - b)) & 1; ones += one;
        const int total = s[b].dur0 + s[b].dur1;
        CHECK(abs(total - 133) <= 1, "DShot600 bit = 1667 ns = 133 ticks @80MHz, got %d", total);
        CHECK(abs(s[b].dur0 - (one ? 100 : 50)) <= 1 && s[b].lvl0 == 1 && s[b].lvl1 == 0, "bit %d high %d one=%d", b, s[b].dur0, (int)one);
    }
    dshot_symbols(pk, PROTO_DSHOT300, 80000000, true, s);
    CHECK(s[0].lvl0 == 0 && s[0].lvl1 == 1 && abs(s[0].dur0 + s[0].dur1 - 267) <= 1, "bidirectional = inverted, DShot300 bit 267 ticks");
    dshot_symbols(pk, PROTO_DSHOT1200, 80000000, false, s);
    CHECK(abs(s[0].dur0 + s[0].dur1 - 67) <= 1, "DShot1200 bit 833 ns");
}

static void t_gcr() {
    printf("[bidir] GCR / eRPM / EDT decode\n");
    // every 12-bit value survives encode->decode
    bool all = true; for (uint32_t v = 0; v < 4096; v++) { if (gcr_decode(gcr_encode((uint16_t)v)) != v) { all = false; break; } }
    CHECK(all, "gcr round trip for all 4096 values");
    // any single flipped bit is caught
    int missed = 0; for (int b = 0; b < 20; b++) { uint32_t g = gcr_encode(0x2A5) ^ (1u << b); if (gcr_decode(g) == 0x2A5) missed++; }
    CHECK(missed == 0, "single-bit errors never decode to the original (%d missed)", missed);
    // waveform: levels <-> gcr, and through run lengths with a truncated idle tail
    uint32_t g = gcr_encode(0x5C3); uint8_t lv[21]; gcr_to_levels(g, lv);
    CHECK(levels_to_gcr(lv) == g, "levels round trip");
    for (int cut = 0; cut < 3; cut++) {
        std::vector<Run> runs; const uint32_t bn = 1333;   // DShot600 reply: 1667 * 4/5 ns
        int i = 0; while (i < 21) { int j = i; while (j < 21 && lv[j] == lv[i]) j++; runs.push_back({lv[i], (uint32_t)((j - i) * bn + (cut == 1 ? 60 : (cut == 2 ? -60 : 0)))}); i = j; }
        if (runs.back().level == 1) runs.pop_back();       // trailing high run merges with the idle line
        CHECK(runs_to_gcr(runs.data(), (int)runs.size(), bn) == g, "runs -> gcr with timing jitter case %d", cut);
    }
    { Run one[2] = {{0, 100}, {1, 100}}; CHECK(runs_to_gcr(one, 2, 0) == kGcrInvalid, "a reply bit of length 0 (a timer nobody set up) is 'invalid', not a division by zero"); }
    CHECK(reply_bit_ns(PROTO_DSHOT600) == 1333 && reply_bit_ns(PROTO_DSHOT300) == 2666, "reply bit time = 4/5 of DShot bit");
    // eRPM: period 1000 us -> 60000 erpm -> 600 (x100). value = e<<9|m ; 1000 = 500<<1 -> e=1,m=500
    CHECK(erpm_from_reply((1 << 9) | 500) == 600, "erpm 60000 -> %u", erpm_from_reply((1 << 9) | 500));
    CHECK(erpm_from_reply(0x0FFF) == 0, "stopped");
    CHECK(erpm_from_reply(0x0000) == kTelemInvalid, "zero period invalid");
    CHECK(fabsf(erpm100_to_rpm(600, 14) - 60000.0f / 7.0f) < 1.0f, "60000 erpm / 7 pole pairs = 8571 rpm");
    CHECK(fabsf(erpm100_to_rpm(600, 12) - 10000.0f) < 1.0f, "12 poles -> 10000 rpm");
    Telem t = telem_decode(0x0200 | 42, true);   CHECK(t.type == TT_TEMP && t.value == 42, "EDT temperature");
    t = telem_decode(0x0400 | 200, true);        CHECK(t.type == TT_VOLT && t.value == 200, "EDT voltage 200*0.25 = 50 V");
    t = telem_decode(0x0600 | 17, true);         CHECK(t.type == TT_CURRENT && t.value == 17, "EDT current");
    t = telem_decode(0x0200 | 42, false);        CHECK(t.type == TT_ERPM, "without EDT every frame is eRPM");
    t = telem_decode((1 << 9) | 500, true);      CHECK(t.type == TT_ERPM && t.value == 600, "EDT on, odd exponent still eRPM");
}

static void t_pulse() {
    printf("[pulse] protocol widths and endpoints\n");
    CHECK(pulse_us(PROTO_PWM, 1500) == 1500 && pulse_us(PROTO_PWM, 0) == 0, "PWM");
    CHECK(fabsf(pulse_us(PROTO_ONESHOT125, 1000) - 125) < 1e-3f && fabsf(pulse_us(PROTO_ONESHOT125, 2000) - 250) < 1e-3f, "Oneshot125 125..250");
    CHECK(fabsf(pulse_us(PROTO_ONESHOT42, 1000) - 41.667f) < 0.01f && fabsf(pulse_us(PROTO_ONESHOT42, 2000) - 83.333f) < 0.01f, "Oneshot42");
    CHECK(fabsf(pulse_us(PROTO_MULTISHOT, 1000) - 5) < 1e-3f && fabsf(pulse_us(PROTO_MULTISHOT, 2000) - 25) < 1e-3f, "Multishot 5..25");
    for (Protocol p : {PROTO_PWM, PROTO_ONESHOT125, PROTO_ONESHOT42, PROTO_MULTISHOT})
        CHECK(1e6f / default_pulse_hz(p) >= pulse_period_min_us(p), "default frame rate leaves room for the longest pulse (%d)", (int)p);
    PulseEndpoints pe;    // app defaults 0 / 1000 / 2000
    Endpoints e = endpoints(PROTO_PWM, pe, 0.055f, 1.0f);
    CHECK(e.disarm == 0 && fabsf(e.low - 1055) < 0.01f && fabsf(e.high - 2000) < 0.01f, "PWM idle 5.5%%: low=%f", e.low);
    e = endpoints(PROTO_PWM, pe, 0.055f, 0.8f);
    CHECK(fabsf(e.high - (2000 - (2000 - 1055) * 0.2f)) < 0.01f, "PWM output limit 80%%: %f", e.high);
    e = endpoints(PROTO_DSHOT300, pe, 0.055f, 1.0f);
    CHECK(e.disarm == 0 && fabsf(e.low - (48 + 0.055f * 1999)) < 0.01f && e.high == 2047, "DShot endpoints = BF dshotInitEndpoints: low=%f", e.low);
    e = endpoints(PROTO_DSHOT300, pe, 0.0f, 0.5f);
    CHECK(fabsf(e.high - (2047 - 1999 * 0.5f)) < 0.01f, "DShot limit 50%%");
}

static Frame run_once(MotorOutputManager& m, uint32_t us, std::vector<float> f, bool armed) { Frame o; float c[8] = {}; for (size_t i = 0; i < f.size(); i++) c[i] = f[i]; m.update(us, c, armed, o); return o; }

static void t_manager() {
    printf("[manager] arm/disarm, idle, limit, reorder, test sliders\n");
    MotorOutputManager m; m.cfg.protocol = PROTO_DSHOT600; m.begin();
    Frame f = run_once(m, 0, {1, 1, 1, 1}, false);
    CHECK(f.send && f.value[0] == 0 && f.value[3] == 0, "disarmed => DShot 0 even if the mixer shouts full throttle");
    f = run_once(m, 100, {0, 0.5f, 1.0f, 0.25f}, true);
    const uint16_t low = (uint16_t)lroundf(48 + 0.055f * 1999);
    CHECK(f.value[0] == low, "armed & mixer 0 => idle, never 0/stop: %u vs %u", f.value[0], low);
    CHECK(f.value[1] == (uint16_t)lroundf(48 + 0.5f * 1999) && f.value[2] == 2047, "mid/full scale");
    m.cfg.output_limit = 0.8f;
    f = run_once(m, 200, {1, 1, 1, 1}, true);
    CHECK(f.value[0] == (uint16_t)lroundf(2047 - 1999 * 0.2f), "output limit caps the top: %u", f.value[0]);
    m.cfg.output_limit = 1.0f;
    // reorder: logical motor i goes to hw slot order[i]
    m.cfg.order[0] = 2; m.cfg.order[1] = 3; m.cfg.order[2] = 0; m.cfg.order[3] = 1;
    CHECK(m.order_valid(), "permutation ok");
    f = run_once(m, 300, {1.0f, 0.0f, 0.0f, 0.0f}, true);
    CHECK(f.value[2] == 2047 && f.value[0] == low, "motor 1 -> slot 2");
    m.cfg.order[1] = 2; CHECK(!m.order_valid(), "duplicate slot rejected"); m.fix_order(); CHECK(m.cfg.order[1] == 1 && m.order_valid(), "invalid order falls back to identity");
    // test sliders
    f = run_once(m, 1000000, {0, 0, 0, 0}, false); CHECK(f.value[1] == 0, "no test => 0");
    m.test_motor(1, 0.3f, 1000000);
    f = run_once(m, 1100000, {0, 0, 0, 0}, false);
    CHECK(f.value[1] == (uint16_t)lroundf(48 + 0.3f * 1999) && f.value[0] == 0, "slider spins only that motor while disarmed");
    f = run_once(m, 1700000, {0, 0, 0, 0}, false); CHECK(f.value[1] == 0, "slider times out 0.5 s after the app stops refreshing");
    m.test_motor(1, 0.3f, 2000000); f = run_once(m, 2000100, {0, 0, 0, 0}, true);
    f = run_once(m, 2000200, {0, 0, 0, 0}, false); CHECK(f.value[1] == 0, "arming cancels any test");
    // pulse protocol
    MotorOutputManager p; p.cfg.protocol = PROTO_ONESHOT125; p.cfg.pulse.disarmed = 0; p.begin();
    f = run_once(p, 0, {1, 1, 1, 1}, false); CHECK(f.value[0] == 0, "pulse disarmed = 0 (no pulse)");
    f = run_once(p, 10, {0, 0.5f, 1, 0}, true);
    CHECK(f.value[0] == 1055 && f.value[1] == 1500 && f.value[2] == 2000, "pulse scale values idle/mid/max: %u %u %u", f.value[0], f.value[1], f.value[2]);
    p.cfg.pulse.armed = 1100; p.cfg.pulse.max = 1900; p.begin();
    f = run_once(p, 10, {0, 1, 0, 0}, true); CHECK(f.value[1] == 1900 && f.value[0] == (uint16_t)lroundf(1100 + 0.055f * 800), "custom arm/max endpoints");
    CHECK(!p.request_direction(0, true) && !p.apply_directions(), "pulse ESCs can't be reversed by command");
}

static void t_commands() {
    printf("[commands] DShot command pacing (dshot_command.c)\n");
    MotorOutputManager m; m.cfg.protocol = PROTO_DSHOT300; m.cfg.order[0] = 1; m.cfg.order[1] = 0; m.begin();
    CHECK(m.request_direction(0, true), "queued");
    std::vector<std::pair<uint32_t, uint16_t>> sent; std::vector<uint8_t> slots; bool other_zero = true; bool telem = true;
    uint32_t t = 0;
    for (; t < 400000; t += 250) {              // 4 kHz loop
        Frame f = run_once(m, t, {0, 0, 0, 0}, false);
        for (int s = 0; s < 4; s++) if (f.value[s] > 0 && f.send) { sent.push_back({t, f.value[s]}); slots.push_back((uint8_t)s); telem &= f.telem_request[s]; }
        if (f.send) for (int s = 0; s < 4; s++) if (f.value[s] && s != 1) other_zero = false;
    }
    int dirs = 0, saves = 0; for (auto& x : sent) { if (x.second == 8) dirs++; if (x.second == 12) saves++; }
    CHECK(dirs == 10 && saves == 10, "direction x10 then save x10 (got %d / %d)", dirs, saves);
    CHECK(!sent.empty() && sent.front().first >= 10000, "10 ms initial delay before the first command frame (%u us)", sent.front().first);
    bool spacing = true; for (size_t i = 1; i < 10; i++) if (sent[i].first - sent[i - 1].first < 1000) spacing = false;
    CHECK(spacing, "repeats at least 1 ms apart");
    CHECK(other_zero && telem, "only the addressed hw slot carries the command, telemetry bit set");
    CHECK(slots[0] == 1, "logical motor 0 -> hw slot 1");
    CHECK(!m.commands_pending(), "sequence finished");
    // extended direction commands and arming cancels
    m.cfg.dir_cmd_extended = true; m.request_direction(1, false);
    Frame f; for (uint32_t u = t; u < t + 40000; u += 250) { f = run_once(m, u, {0, 0, 0, 0}, false); }
    bool saw21 = false, saw20 = false; { MotorOutputManager q; q.cfg.protocol = PROTO_DSHOT300; q.cfg.dir_cmd_extended = true; q.begin(); q.request_direction(0, true); q.request_direction(1, false);
        for (uint32_t u = 0; u < 300000; u += 250) { Frame g = run_once(q, u, {0, 0, 0, 0}, false); for (int s = 0; s < 4; s++) if (g.send) { if (g.value[s] == 21) saw21 = true; if (g.value[s] == 20) saw20 = true; } } }
    CHECK(saw21 && saw20, "extended 20/21 direction commands");
    MotorOutputManager a; a.cfg.protocol = PROTO_DSHOT300; a.begin(); a.beacon(2, 3); run_once(a, 0, {0, 0, 0, 0}, false);
    run_once(a, 20000, {0, 0, 0, 0}, true);
    CHECK(!a.commands_pending(), "arming discards queued commands (motors must never see a command in flight)");
    f = run_once(a, 30000, {0, 0, 0, 0}, true); CHECK(f.send && f.value[2] >= 48, "and normal throttle frames continue");
    volatile int bad_motor = 9; CHECK(!a.beacon(bad_motor, 1) && !a.beacon(0, 7), "range checks");

    // the beeper's tone: every motor in the SAME frame, hardware order irrelevant, nothing queued behind other work
    { MotorOutputManager b; b.cfg.protocol = PROTO_DSHOT600; b.cfg.count = 4; b.cfg.order[0] = 2; b.cfg.order[2] = 0; b.begin();
      CHECK(b.beacon_all(3, 0) && b.commands_pending(), "beacon_all queued");
      int frames = 0; uint32_t first = 0, last_cmd = 0, done = 0; bool all4 = true;
      for (uint32_t u = 0; u < 300000; u += 250) {
          Frame g = run_once(b, u, {0, 0, 0, 0}, false);
          int with = 0; for (int s = 0; s < 4; s++) if (g.send && g.value[s] == 3 && g.telem_request[s]) with++;
          if (with) { if (!frames) first = u; frames++; last_cmd = u; if (with != 4) all4 = false; }
          if (!done && frames && !b.commands_pending()) done = u;
      }
      CHECK(frames == 1 && all4 && first >= 10000 && first < 10500, "one frame, all four motors, after the 10 ms initial delay (%d frames at %u us)", frames, first);
      CHECK(done - last_cmd >= 100000 && done - last_cmd < 100500, "the line then stays quiet 100 ms for the ESCs to start the tone (%u us)", done - last_cmd);
      b.cfg.count = 6; b.cfg.order[4] = 4; b.cfg.order[5] = 5; b.beacon_all(1, 400000); int six = 0;
      for (uint32_t u = 400000; u < 600000; u += 250) { Frame g = run_once(b, u, {0, 0, 0, 0, 0, 0}, false); int with = 0; for (int s = 0; s < 8; s++) if (g.send && g.value[s] == 1) with++; if (with) six = with; }
      CHECK(six == 6, "a hexa: six motors (%d)", six);
      CHECK(!b.beacon_all(0, 700000) && !b.beacon_all(6, 700000), "tones 1..5 only");
      b.request_direction(0, true); CHECK(!b.beacon_all(2, 700000), "another command is on its way: the tone is not queued behind it (it would come late)");
      for (uint32_t u = 700000; u < 1100000; u += 250) run_once(b, u, {0, 0, 0, 0, 0, 0}, false);
      CHECK(!b.commands_pending() && b.beacon_all(2, 1100000) && !b.beacon_all(2, 1100000), "free again: accepted once, not twice at the same time");
      MotorOutputManager p; p.cfg.protocol = PROTO_ONESHOT125; p.begin(); CHECK(!p.beacon_all(1, 0), "a pulse protocol has no commands"); }
    { MotorOutputManager t; t.cfg.protocol = PROTO_DSHOT300; t.begin();
      CHECK(!t.testing(1000), "no test running");
      t.test_motor(1, 0.2f, 1000000); CHECK(t.testing(1000100) && !t.beacon_all(1, 1000100), "a motor is being turned by the test slider: no tone");
      CHECK(!t.testing(1600000), "500 ms without a refresh: the test is over"); CHECK(t.beacon_all(1, 1600000), "and a tone is accepted");
      t.test_motor(1, 0.0f, 2000000); CHECK(!t.testing(2000100), "a slider at zero is not a running test");
      MotorOutputManager w; w.cfg.protocol = PROTO_DSHOT300; w.begin(); run_once(w, 0x90000000u, {0, 0, 0, 0}, false);
      CHECK(!w.testing(0x90000000u) && w.beacon_all(1, 0x90000000u), "36 minutes after power-on (micros() past 2^31): still 'no test', tones still work"); }
}

// ESC simulator: builds the reply a real ESC sends for a given rpm
static uint32_t esc_reply(float rpm, int poles) {
    const float erpm = rpm * (poles / 2);
    const float period_us = 60e6f / erpm;
    int e = 0; uint32_t m = (uint32_t)lroundf(period_us);
    while (m > 0x1FF) { m >>= 1; e++; }
    return gcr_encode((uint16_t)((e << 9) | m));
}

static void t_telemetry() {
    printf("[telemetry] rpm, errors, timeouts, dynamic idle\n");
    MotorOutputManager m; m.cfg.protocol = PROTO_DSHOT600; m.cfg.bidirectional = true; m.cfg.pole_count = 14; m.begin();
    for (int i = 0; i < 4; i++) m.feed_reply(i, esc_reply(12000.0f + 500 * i, 14), 1000);
    for (int i = 0; i < 4; i++) { auto t = m.telemetry(i, 1010); CHECK(t.rpm_valid && fabsf(t.rpm - (12000 + 500 * i)) < 0.02f * (12000 + 500 * i), "motor %d rpm %.0f", i, t.rpm); }
    CHECK(m.rpm_all_valid(1010), "all valid");
    CHECK(!m.rpm_all_valid(1200), "rpm expires after 100 ms => the RPM filter and dynamic idle must fall back");
    { MotorOutputManager one; one.cfg.protocol = PROTO_DSHOT600; one.cfg.bidirectional = true; one.cfg.pole_count = 14; one.begin(); CHECK(!one.rpm_any_valid(1000), "no reply yet: no rpm at all");
      one.feed_reply(2, esc_reply(15000, 14), 1000); float a[kMaxMotors]; one.rpm_array(a, 1010);
      CHECK(one.rpm_any_valid(1010) && !one.rpm_all_valid(1010) && a[0] == 0 && fabsf(a[2] - 15000) < 300 && !one.rpm_any_valid(1200), "one ESC of four answers: its rpm is there (%.0f), the others read 0 - the RPM filter works per motor; after 100 ms of silence nothing is valid", a[2]);
      one.cfg.bidirectional = false; one.feed_reply(2, esc_reply(15000, 14), 2000); CHECK(!one.rpm_any_valid(2001), "bidirectional DShot off in the settings: never valid"); }
    float arr[kMaxMotors]; m.rpm_array(arr, 1200); CHECK(arr[0] == 0 && arr[3] == 0, "expired rpm reported as 0 (notch bypass)");
    { MotorOutputManager h; h.cfg.protocol = PROTO_DSHOT600; h.cfg.bidirectional = true; h.cfg.pole_count = 14; h.cfg.count = 6; h.begin(); for (int i = 0; i < 6; i++) h.feed_reply(i, esc_reply(9000.0f + 1000 * i, 14), 1000);
      float a6[kMaxMotors]; for (float& x : a6) x = -1; h.rpm_array(a6, 1001); bool ok = h.rpm_all_valid(1001); for (int i = 0; i < 6; i++) ok &= fabsf(a6[i] - (9000.0f + 1000 * i)) < 0.02f * a6[i];
      CHECK(ok && a6[6] == 0 && a6[7] == 0, "a hex: all six motor speeds are handed to the RPM filter (%.0f .. %.0f), the unused slots are 0", a6[0], a6[5]); }
    for (int k = 0; k < 20; k++) m.feed_reply(0, k % 4 == 0 ? kGcrInvalid : esc_reply(12000, 14), 2000 + k);
    auto t = m.telemetry(0, 2020); CHECK(t.error_pct >= 20 && t.error_pct <= 30, "error %% ~ 25%% (%u)", t.error_pct);
    CHECK(fabsf(m.telemetry(1, 1005).error_pct) < 1, "clean motor 0 %%");
    MotorOutputManager e; e.cfg.protocol = PROTO_DSHOT600; e.cfg.bidirectional = true; e.cfg.edt = true; e.begin();
    e.feed_reply(0, gcr_encode(0x0400 | 66), 5000); e.feed_reply(1, gcr_encode(0x0400 | 68), 5000); e.feed_reply(0, gcr_encode(0x0200 | 55), 5001); e.feed_reply(0, gcr_encode(0x0600 | 12), 5002);
    CHECK(fabsf(e.esc_voltage(5100) - 17.0f) < 0.01f, "ESC pack voltage = highest reading 68*0.25 = 17 V (%f)", e.esc_voltage(5100));
    CHECK(e.telemetry(0, 5100).temp_c == 55 && e.telemetry(0, 5100).amps == 12, "temp/current");
    CHECK(e.esc_voltage(9000) == 0, "stale voltage disappears");

    // dynamic idle closed loop: motors spin at rpm = k * (thr) ; floor must rise until slowest motor reaches the target rpm
    MotorOutputManager d; d.cfg.protocol = PROTO_DSHOT600; d.cfg.bidirectional = true; d.cfg.idle = 0.03f; d.cfg.dyn.min_rpm_100 = 100; d.begin(0.00025f);   // 10000 rpm floor
    float floor_ = d.idle_fraction(0, false); CHECK(floor_ == 0.03f, "no rpm yet: static idle only");
    float rpm = 0; uint32_t ms = 0; float end_idle = 0;
    for (int i = 0; i < 40000; i++) {                       // 10 s at 4 kHz
        ms = i / 4;
        const float thr = floor_;
        rpm += (thr * 320000.0f - rpm) * 0.002f;             // motor: 32000 rpm per 10 % throttle-ish, first order lag
        if (i % 4 == 0) for (int k = 0; k < 4; k++) d.feed_reply(k, esc_reply(rpm < 300 ? 300 : rpm, 14), ms);
        floor_ = d.idle_fraction(ms, false);
        end_idle = floor_;
    }
    CHECK(fabsf(rpm - 10000.0f) < 1500.0f, "dynamic idle settles the slowest motor near the 10000 rpm target: %.0f", rpm);
    CHECK(end_idle > 0.03f && end_idle <= 0.03f + 0.03f * 1.0f + 1e-3f, "increase limited to start increase (= idle) before throttle was raised: %.4f", end_idle);
    // once throttle has been raised the ceiling is max_increase (15 %)
    d.begin(0.00025f); rpm = 0; floor_ = 0.03f;
    for (int i = 0; i < 40000; i++) { ms = 100000 + i / 4; rpm += (floor_ * 150000.0f - rpm) * 0.002f;
        if (i % 4 == 0) for (int k = 0; k < 4; k++) d.feed_reply(k, esc_reply(rpm < 300 ? 300 : rpm, 14), ms);
        floor_ = d.idle_fraction(ms, true); }
    CHECK(floor_ > 0.06f && floor_ <= 0.03f + 0.15f + 1e-3f, "weak motors: idle rises above the start ceiling but never past max_increase: %.4f", floor_);
    // telemetry lost => back to static idle and I-term cleared
    floor_ = d.idle_fraction(ms + 1000, true); CHECK(floor_ == 0.03f, "rpm timeout => static idle (%f)", floor_);
}

int main() {
    t_frame(); t_gcr(); t_pulse(); t_manager(); t_commands(); t_telemetry();
    printf("\n%d passed, %d failed\n", g_pass, g_fail); return g_fail ? 1 : 0;
}
