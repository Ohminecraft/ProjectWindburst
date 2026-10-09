// Battery monitor + beeper: the portable logic, tested against numbers worked out by hand (not by calling the code under test).
// Build: g++ -std=c++17 -O2 test/test_power.cpp -o build/test_power
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <vector>
#include "../src/hal/battery/battery_monitor.h"
#include "../src/hal/beeper/beeper_core.h"

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, ...) do { if (cond) g_pass++; else { g_fail++; printf("  FAIL line %d: %s   ", __LINE__, #cond); printf(__VA_ARGS__); printf("\n"); } } while (0)

// ------------------------------------------------------------------------------------------------------------------------------
// Battery. The monitor runs at 50 Hz. Its low-pass is 1/3 Hz: time constant 1 / (2 pi / 3) = 0.477 s.
// ------------------------------------------------------------------------------------------------------------------------------
struct Bench {
    batt::Monitor m; uint32_t ms = 1000;
    void run(float volts, double seconds, bool valid = true) { const int n = (int)lround(seconds * 50.0); for (int i = 0; i < n; i++) { ms += 20; m.update(ms, volts, valid); } }
    // seconds until the state becomes `want` at this voltage (-1 = not within `limit` seconds)
    double until(float volts, batt::State want, double limit) { for (double t = 0; t < limit; t += 0.02) { if (m.state() == want) return t; ms += 20; m.update(ms, volts, true); } return m.state() == want ? limit : -1.0; }
};

static void t_battery() {
    printf("[battery] presence, cell count\n");
    { Bench b; b.run(0.0f, 3.0);
      CHECK(b.m.state() == batt::State::NOT_PRESENT && b.m.cells() == 0 && b.m.cell_voltage() == 0.0f && !b.m.low() && !b.m.critical(), "0 V: no battery, nothing to warn about"); }
    { Bench b; b.run(16.0f, 0.9);                             // powered up with the pack connected: the filter starts at the reading
      CHECK(!b.m.present(), "a battery is accepted only after the voltage was steady for about a second (0.9 s: not yet)");
      b.run(16.0f, 0.4);
      CHECK(b.m.present() && b.m.state() == batt::State::OK && b.m.cells() == 4 && fabsf(b.m.cell_voltage() - 4.0f) < 0.005f && fabsf(b.m.voltage() - 16.0f) < 0.01f, "16.0 V after 1.3 s: 4 cells, 4.00 V per cell (%u cells %.3f V)", b.m.cells(), b.m.cell_voltage()); }
    { Bench b; b.run(0.0f, 1.0); const double t = b.until(16.0f, batt::State::OK, 8.0);
      CHECK(t > 2.0 && t < 5.0, "plugged in later: accepted once the filtered value has settled (%.2f s)", t);
      CHECK(b.m.cells() == 4, "and the cell count comes from the settled voltage, not from the ramp: %u", b.m.cells()); }
    { const struct { float v; int cells; } table[] = {{3.7f, 1}, {4.2f, 1}, {7.4f, 2}, {8.4f, 2}, {11.1f, 3}, {12.6f, 3}, {14.8f, 4}, {16.8f, 4}, {21.0f, 5}, {22.2f, 6}, {25.2f, 6}, {33.6f, 8}, {50.0f, 8}};
      bool ok = true; int bad_cells = 0; float bad_v = 0;
      for (const auto& e : table) { Bench b; b.run(e.v, 1.5); if (!b.m.present() || b.m.cells() != e.cells) { ok = false; bad_cells = b.m.cells(); bad_v = e.v; } }
      CHECK(ok, "cell count from the voltage, 1S..8S, nominal and full packs (%.1f V gave %d)", bad_v, bad_cells); }
    { Bench b; b.run(12.8f, 1.5);
      CHECK(b.m.cells() == 3 && b.m.state() == batt::State::OK, "KNOWN LIMIT: an empty 4S (12.8 V) is taken for a full 3S when the cell count is detected");
      Bench c; c.m.cfg.cells = 4; c.run(12.8f, 1.5);
      CHECK(c.m.cells() == 4 && !c.m.cells_detected(), "with the cell count set to 4 it is a 4S at 3.2 V per cell");
      c.run(12.8f, 3.0);
      CHECK(c.m.state() == batt::State::CRITICAL && c.m.low() && c.m.critical(), "and it ends up CRITICAL"); }
    { Bench b; b.run(5.0f, 3.0); CHECK(!b.m.present(), "5.0 V is the USB supply, not a battery");
      Bench c; c.run(4.6f, 3.0); CHECK(!c.m.present(), "4.6 V: above one full cell, below two empty ones - not a battery");
      Bench d; d.run(6.5f, 3.0); CHECK(d.m.present() && d.m.cells() == 2, "6.5 V: a (low) 2S");
      Bench e; e.run(2.5f, 3.0); CHECK(!e.m.present(), "2.5 V: below any cell");
      Bench f; f.run(16.0f, 3.0, false); CHECK(!f.m.present() && f.m.voltage() == 0.0f, "a source without a reading counts as 0 V");
      Bench g; g.run(NAN, 3.0); CHECK(!g.m.present() && g.m.voltage() == 0.0f, "NaN from a broken source counts as 0 V"); }

    printf("[battery] warning / critical: thresholds, filter, delays, hysteresis\n");
    // 4S: warning below 4 * 3.50 = 14.00 V, critical below 4 * 3.30 = 13.20 V (10 mV hysteresis), delays 0.5 s and 2.0 s.
    { Bench b; b.run(16.0f, 1.5);
      // step to 13.8 V: the filtered value passes 13.99 V after 0.477 * ln(2.2 / 0.19) = 1.17 s, then the 0.5 s delay -> ~1.7 s
      const double t = b.until(13.8f, batt::State::WARNING, 5.0);
      CHECK(t > 1.5 && t < 2.0, "16.0 -> 13.8 V: WARNING after filter lag + 0.5 s delay (%.2f s, expected about 1.7)", t);
      CHECK(b.m.low() && !b.m.critical(), "warning: do not take off, no landing yet"); }
    { Bench b; b.run(16.0f, 1.5); b.run(12.0f, 0.3); const bool during = b.m.state() == batt::State::OK; b.run(16.0f, 1.0);
      CHECK(during && b.m.state() == batt::State::OK, "a 0.3 s throttle punch down to 3.0 V per cell does not raise anything"); }
    { Bench b; b.run(16.0f, 1.5); b.run(14.0f, 6.0);
      CHECK(b.m.state() == batt::State::OK, "exactly at the warning level (14.00 V) stays OK: it has to go 10 mV below");
      b.run(13.995f, 6.0);
      CHECK(b.m.state() == batt::State::OK, "13.995 V: still OK, inside the 10 mV hysteresis");
      b.run(13.98f, 6.0);
      CHECK(b.m.state() == batt::State::WARNING, "13.98 V: WARNING");
      b.run(14.0f, 6.0);
      CHECK(b.m.state() == batt::State::WARNING, "back at 14.00 V it stays WARNING (must rise ABOVE the level)");
      b.run(14.03f, 6.0);
      CHECK(b.m.state() == batt::State::OK, "14.03 V: OK again"); }
    { Bench b; b.run(16.0f, 1.5); b.run(13.6f, 4.0);
      CHECK(b.m.state() == batt::State::WARNING, "13.6 V: WARNING, not CRITICAL");
      // step 13.6 -> 13.0: passes 13.19 V after 0.477 * ln(0.6 / 0.19) = 0.55 s, then 2.0 s below it -> ~2.55 s
      const double t = b.until(13.0f, batt::State::CRITICAL, 6.0);
      CHECK(t > 2.3 && t < 2.9, "13.6 -> 13.0 V: CRITICAL after filter lag + 2.0 s delay (%.2f s, expected about 2.55)", t);
      CHECK(b.m.critical(), "critical: land");
      b.run(13.2f, 4.0); CHECK(b.m.state() == batt::State::CRITICAL, "13.20 V: still CRITICAL");
      b.run(13.3f, 4.0); CHECK(b.m.state() == batt::State::WARNING, "13.30 V: back to WARNING"); }
    { // the critical delay counts CONTINUOUS time below the level: a short recovery starts it again
      Bench b; b.run(16.0f, 1.5); b.run(13.6f, 4.0); b.run(13.0f, 2.0);       // 0.55 s lag + 1.45 s below: not yet
      const bool before = b.m.state() == batt::State::WARNING;
      b.run(13.6f, 1.0);                                                       // recovers
      b.run(13.0f, 2.0);                                                       // 0.48 s lag + 1.5 s below: not yet again
      CHECK(before && b.m.state() == batt::State::WARNING, "two dips of 1.5 s with a recovery in between do not add up to the 2 s delay");
      b.run(13.0f, 1.0);
      CHECK(b.m.state() == batt::State::CRITICAL, "one uninterrupted 2 s does"); }
    { Bench b; b.m.cfg.warning_delay_s = 0.0f; b.m.cfg.critical_delay_s = 0.0f; b.run(16.0f, 1.5);
      const double t = b.until(13.8f, batt::State::WARNING, 5.0);
      CHECK(t > 1.0 && t < 1.35, "delay 0: only the filter lag remains (%.2f s, expected about 1.17)", t); }
    { // thresholds follow the settings while the battery is connected (the app's live edit)
      Bench b; b.run(15.2f, 2.0); CHECK(b.m.state() == batt::State::OK, "15.2 V on 4S (3.80 V per cell): OK");
      b.m.cfg.cell_warning_v = 3.9f; b.run(15.2f, 1.0); CHECK(b.m.state() == batt::State::WARNING, "warning level raised to 3.90 V: WARNING");
      b.m.cfg.cell_warning_v = 3.5f; b.run(15.2f, 0.1); CHECK(b.m.state() == batt::State::OK, "lowered again: OK");
      b.m.cfg.cells = 5; b.run(15.2f, 3.5);
      CHECK(b.m.cells() == 5 && !b.m.cells_detected() && b.m.state() == batt::State::CRITICAL && fabsf(b.m.cell_voltage() - 3.04f) < 0.01f, "cell count typed as 5: 3.04 V per cell, CRITICAL (state %d)", (int)b.m.state());
      b.m.cfg.cells = 0; b.run(15.2f, 0.5);
      CHECK(b.m.cells() == 4 && b.m.cells_detected() && b.m.state() == batt::State::OK, "back to 'detect': the count found when the pack was plugged in (4) is used again"); }
    { Bench b; b.run(16.0f, 1.5); const double t = b.until(0.0f, batt::State::NOT_PRESENT, 10.0);
      CHECK(t > 1.0 && t < 6.0 && b.m.cells() == 0, "unplugged: NOT_PRESENT once the filter has run down (%.2f s), cell count forgotten", t);
      b.run(11.1f, 6.0); CHECK(b.m.present() && b.m.cells() == 3, "a different pack afterwards is detected afresh (3S)"); }

    printf("[battery] source selection, ADC scaling\n");
    CHECK(batt::pick_source(batt::SRC_AUTO, true) == batt::SRC_ADC && batt::pick_source(batt::SRC_AUTO, false) == batt::SRC_ESC, "AUTO: the ADC pin when there is one, otherwise the ESCs");
    CHECK(batt::pick_source(batt::SRC_ESC, true) == batt::SRC_ESC && batt::pick_source(batt::SRC_ADC, false) == batt::SRC_ADC && batt::pick_source(batt::SRC_NONE, true) == batt::SRC_NONE, "an explicit choice is kept");
    CHECK(fabsf(batt::adc_to_volts(1455, 11.0f) - 16.005f) < 1e-3f && fabsf(batt::adc_to_volts(3000, 8.5f) - 25.5f) < 1e-3f && batt::adc_to_volts(0, 11.0f) == 0.0f, "pin mV x divider = battery volts");
}

// ------------------------------------------------------------------------------------------------------------------------------
// Beeper. Expected patterns are written here in milliseconds, straight from Betaflight's tables (unit 10 ms).
// ------------------------------------------------------------------------------------------------------------------------------
struct Trace {                                               // the buzzer state sampled every millisecond
    std::vector<bool> on;
    int on_ms(int from, int to) const { int n = 0; for (int i = from; i < to && i < (int)on.size(); i++) n += on[(size_t)i]; return n; }
    // lengths of the on / off runs starting at t = 0 (first entry = an "on" run; 0 if the trace starts with silence)
    std::vector<int> runs() const {
        std::vector<int> r; bool cur = true; int len = 0;
        for (bool b : on) { if (b == cur) len++; else { r.push_back(len); cur = b; len = 1; } }
        r.push_back(len); return r;
    }
};
static Trace play(beep::Beeper& b, uint32_t& ms, int duration_ms) { Trace t; for (int i = 0; i < duration_ms; i++) { t.on.push_back(b.update(ms)); ms++; } return t; }
static bool starts_with(const std::vector<int>& runs, std::initializer_list<int> want) {
    size_t i = 0; for (int w : want) { if (i >= runs.size() || abs(runs[i] - w) > 1) return false; i++; } return true;
}

// ------------------------------------------------------------------------------------------------------------------------------
// Current and consumption. Numbers worked out by hand from Betaflight's current.c (ibata_scale in 0.1 mV/A, ibata_offset in mA).
// ------------------------------------------------------------------------------------------------------------------------------
static void t_current() {
    printf("[current] ADC scaling, source choice, filter, mAh, what is left\n");
    CHECK(fabsf(batt::adc_to_amps(40, 400, 0) - 1.0f) < 1e-4f && fabsf(batt::adc_to_amps(1650, 400, 0) - 41.25f) < 1e-3f, "scale 400 (40 mV/A): 40 mV = 1 A, 1650 mV = 41.25 A");
    CHECK(fabsf(batt::adc_to_amps(330, 110, 0) - 30.0f) < 1e-3f, "scale 110 (11 mV/A): 330 mV = 30 A (%.3f)", batt::adc_to_amps(330, 110, 0));
    CHECK(fabsf(batt::adc_to_amps(0, 400, -500) + 0.5f) < 1e-4f && fabsf(batt::adc_to_amps(400, 400, 250) - 10.25f) < 1e-3f, "offset in mA: -500 at 0 mV = -0.5 A; +250 at 400 mV = 10.25 A");
    CHECK(fabsf(batt::adc_to_amps(40, -400, 0) + 1.0f) < 1e-4f && batt::adc_to_amps(500, 0, 100) == 0.0f, "a negative scale (sensor the other way round) and scale 0 (no division by zero)");
    CHECK(batt::pick_current_source(batt::CUR_AUTO, true) == batt::CUR_ADC && batt::pick_current_source(batt::CUR_AUTO, false) == batt::CUR_ESC && batt::pick_current_source(batt::CUR_ESC, true) == batt::CUR_ESC
          && batt::pick_current_source(batt::CUR_ADC, false) == batt::CUR_ADC && batt::pick_current_source(batt::CUR_NONE, true) == batt::CUR_NONE, "AUTO: the ADC sensor when its pin works, otherwise the ESCs; an explicit choice is kept");
    { batt::CurrentMeter c; for (int i = 0; i < 50 * 36; i++) c.update(10.0f, true, 0.02f);
      CHECK(fabsf(c.mah_drawn() - 100.0f) < 0.05f && fabsf(c.amps() - 10.0f) < 1e-3f, "10 A for 36 s = 100 mAh (%.3f)", c.mah_drawn()); }
    { batt::CurrentMeter c; c.update(0.0f, true, 0.0f); for (int i = 0; i < 8; i++) c.update(10.0f, true, 0.02f);
      const float a8 = c.amps();                              // 1 Hz low-pass at 50 Hz: gain 0.02 / (0.159 + 0.02) = 0.1117, after 8 steps 1 - 0.8883^8 = 61 %
      for (int i = 0; i < 42; i++) c.update(10.0f, true, 0.02f);
      CHECK(a8 > 5.8f && a8 < 6.5f && c.amps() > 9.9f, "the current shown is low-passed at 1 Hz: 0 -> 10 A, %.2f A after 0.16 s, %.2f A after 1 s", a8, c.amps());
      const float m = c.mah_drawn();
      CHECK(fabsf(m - 10.0f * 1.0f / 3.6f) < 0.01f, "the consumption adds up the readings, not the filtered value: 10 A for 1.0 s = 2.78 mAh (%.3f)", m);
      c.update(10.0f, true, 1.5f); CHECK(c.mah_drawn() == m, "a gap of 1.5 s (the sensor task stood still) is not added: no jump");
      c.update(55.0f, false, 0.02f); CHECK(!c.valid() && c.amps() == 0.0f && c.mah_drawn() == m, "no reading: shows 0 A, keeps the consumption");
      c.update(NAN, true, 0.02f); CHECK(!c.valid() && c.mah_drawn() == m, "a NaN reading counts as none");
      c.update(10.0f, true, 0.02f); CHECK(c.valid() && c.amps() > 9.9f, "the next good reading: the filter goes on where it was (%.2f)", c.amps());
      c.reset(); CHECK(c.mah_drawn() == 0.0f && !c.valid(), "reset (a new current pin): starts from 0"); }
    CHECK(batt::remaining_percent(375.0f, 1500, 15.0f, 4, 3.3f, 4.3f) == 75 && batt::remaining_percent(2000.0f, 1500, 15.0f, 4, 3.3f, 4.3f) == 0, "capacity known: 375 of 1500 mAh used = 75 %%, more than the capacity = 0 %%");
    CHECK(batt::remaining_percent(0.0f, 0, 15.2f, 4, 3.3f, 4.3f) == 50 && batt::remaining_percent(0.0f, 0, 17.5f, 4, 3.3f, 4.3f) == 100 && batt::remaining_percent(0.0f, 0, 12.0f, 4, 3.3f, 4.3f) == 0,
          "no capacity: from the voltage between 4 x 3.3 and 4 x 4.3 V (15.2 V = 50 %%, above full = 100, below empty = 0) - got %u", batt::remaining_percent(0.0f, 0, 15.2f, 4, 3.3f, 4.3f));
    CHECK(batt::remaining_percent(0.0f, 1500, 0.0f, 0, 3.3f, 4.3f) == 0, "no battery (0 cells): 0 %%");
}

static void t_beeper() {
    printf("[beeper] patterns, priority, repeat\n");
    uint32_t ms = 5000;
    { beep::Beeper b; b.request(beep::ARMING); const Trace t = play(b, ms, 800);
      CHECK(starts_with(t.runs(), {300, 50, 50}) && t.on_ms(0, 800) == 350 && b.current() == beep::NO_MODE, "ARMING: 300 ms on, 50 off, 50 on, then silent (on %d ms)", t.on_ms(0, 800)); }
    { beep::Beeper b; b.request(beep::GYRO_CALIBRATED); const Trace t = play(b, ms, 1500);
      CHECK(starts_with(t.runs(), {200, 100, 200, 100, 200}) && t.on_ms(0, 1500) == 600, "GYRO_CALIBRATED: three 200 ms beeps"); }
    { beep::Beeper b; b.request(beep::DISARMING); const Trace t = play(b, ms, 800);
      CHECK(starts_with(t.runs(), {150, 50, 150}) && t.on_ms(0, 800) == 300, "DISARMING: two 150 ms beeps"); }
    { beep::Beeper b; b.request(beep::ARMING_GPS_FIX); const Trace t = play(b, ms, 1500);
      CHECK(starts_with(t.runs(), {50, 50, 150, 50, 50, 50, 150}) && t.on_ms(0, 1500) == 400, "ARMING_GPS_FIX: short long short long"); }
    { beep::Beeper b; b.request(beep::ARMING_GPS_NO_FIX); const Trace t = play(b, ms, 1500);
      CHECK(starts_with(t.runs(), {300, 50, 300, 50, 300}) && t.on_ms(0, 1500) == 900, "ARMING_GPS_NO_FIX: three long beeps"); }
    { beep::Beeper b; b.request(beep::RX_LOST_LANDING); const Trace t = play(b, ms, 4000);
      // S O S: . . .  - - -  . . .
      CHECK(starts_with(t.runs(), {100, 100, 100, 100, 100, 400, 400, 100, 400, 100, 400, 400, 100, 100, 100, 100, 100}) && t.on_ms(0, 4000) == 1800, "RX_LOST_LANDING: SOS in morse"); }
    { beep::Beeper b; b.request(beep::ARMED); const Trace t = play(b, ms, 3000);
      CHECK(t.on_ms(0, 2449) == 0 && t.on_ms(2450, 2550) >= 99 && t.on_ms(2551, 3000) == 0, "ARMED: 2.45 s pause first, then one 100 ms beep"); }
    { beep::Beeper b; Trace t;
      for (int i = 0; i < 2000; i++) { b.request(beep::BAT_CRIT_LOW); t.on.push_back(b.update(ms)); ms++; }
      CHECK(starts_with(t.runs(), {500, 20, 500, 20}), "BAT_CRIT_LOW held: 500 ms on, 20 ms off"); }
    { beep::Beeper b; Trace t;                              // a condition that stays true is requested every loop: the pattern repeats, it does not restart
      for (int i = 0; i < 10000; i++) { b.request(beep::RX_LOST); t.on.push_back(b.update(ms)); ms++; }
      CHECK(starts_with(t.runs(), {500, 500, 500, 500, 500}) && abs(t.on_ms(0, 10000) - 5000) <= 10, "RX_LOST held: 500 ms on / 500 ms off, over and over (%d ms on in 10 s)", t.on_ms(0, 10000)); }
    { beep::Beeper b; Trace t;
      for (int i = 0; i < 7500; i++) { b.request(beep::BAT_LOW); t.on.push_back(b.update(ms)); ms++; }
      CHECK(starts_with(t.runs(), {250, 500, 250, 500}) && abs(t.on_ms(0, 7500) - 2500) <= 5, "BAT_LOW held: 250 ms beep every 750 ms"); }
    { beep::Beeper b; b.request(beep::BAT_LOW); play(b, ms, 100);
      b.request(beep::ARMING); const Trace t = play(b, ms, 500);
      CHECK(b.current() == beep::NO_MODE && starts_with(t.runs(), {300, 50, 50}), "a more important sound (ARMING) replaces a running one (BAT_LOW) at once"); }
    { beep::Beeper b; b.request(beep::ARMING); play(b, ms, 100);
      b.request(beep::RX_SET); b.request(beep::BAT_LOW); CHECK(b.current() == beep::ARMING, "less important requests wait while ARMING plays");
      b.request(beep::ARMING); const Trace t = play(b, ms, 300);
      CHECK(starts_with(t.runs(), {200, 50, 50}), "asking for the running sound again does not restart it (200 ms of the first beep were left)"); }
    { beep::Beeper b; b.request(beep::ON_USB); b.request(beep::MODE_COUNT); b.request(200); b.force(beep::ON_USB);
      CHECK(b.current() == beep::NO_MODE && !b.update(ms), "USB is a switch, not a sound; out-of-range modes are ignored"); }

    printf("[beeper] conditions switched off, forced test sound\n");
    { beep::Beeper b; b.enabled = ~(1u << beep::RX_LOST); Trace t;
      for (int i = 0; i < 3000; i++) { b.request(beep::RX_LOST); b.request(beep::BAT_LOW); t.on.push_back(b.update(ms)); ms++; }
      CHECK(starts_with(t.runs(), {250, 500, 250}), "a switched-off condition (RX_LOST) is silent and does NOT hold back the battery warning behind it"); }
    { beep::Beeper b; b.enabled = 0; b.request(beep::ARMING);
      CHECK(b.current() == beep::NO_MODE && !b.update(ms), "everything off: silence");
      b.force(beep::ARMING); const Trace t = play(b, ms, 500);
      CHECK(starts_with(t.runs(), {300, 50, 50}), "the app's test button plays a switched-off sound"); }
    { beep::Beeper b; b.request(beep::GYRO_CALIBRATED); play(b, ms, 50); b.force(beep::RX_SET); const Trace t = play(b, ms, 300);
      CHECK(starts_with(t.runs(), {100, 200}) && b.forced() == false, "the test button replaces even a more important running sound"); }
    { beep::Beeper b; b.request(beep::RX_LOST); play(b, ms, 100); b.silence();
      CHECK(!b.update(ms) && b.current() == beep::NO_MODE, "silence() stops at once"); }
    { bool ok = true; for (int m = 0; m < beep::MODE_COUNT; m++) { if (m == beep::ON_USB) continue; beep::Beeper b; b.request((uint8_t)m); const Trace t = play(b, ms, 6000); if (b.current() != beep::NO_MODE || t.on_ms(0, 6000) < 60) ok = false; }
      CHECK(ok, "every sound has a pattern that plays and ends"); }
}

// ------------------------------------------------------------------------------------------------------------------------------
struct Craft {
    beep::Beeper b, m; beep::Conditions c; beep::Input in; std::vector<bool> on; int beacons = 0; std::vector<uint32_t> beacon_at; bool motors_ok = true;
    Craft() { in.now_ms = 10000; in.rx_ok = true; in.batt_state = 1; in.gyro_calibrated = true; m.slot_ms = 450; m.enabled = beep::kMotorDefault; }
    // one millisecond the way the firmware does it: conditions, then the buzzer pin, then the motors (one beacon per started tone)
    void run(int ms) {
        for (int i = 0; i < ms; i++) {
            in.now_ms++; c.update(in, b, &m, motors_ok); on.push_back(b.update(in.now_ms));
            m.update(in.now_ms); if (m.take_start()) { beacons++; beacon_at.push_back(in.now_ms); }
        }
    }
    int on_ms(int last_ms) const { int n = 0; for (size_t i = on.size() - (size_t)last_ms; i < on.size(); i++) n += on[i]; return n; }
    std::vector<int> runs_since(size_t from) const { Trace t; t.on.assign(on.begin() + (long)from, on.end()); return t.runs(); }
    // gaps between the beacons sent after beacon number `from`
    std::vector<int> gaps(size_t from = 0) const { std::vector<int> g; for (size_t i = from + 1; i < beacon_at.size(); i++) g.push_back((int)(beacon_at[i] - beacon_at[i - 1])); return g; }
    bool spaced() const { for (int g : gaps()) if (g < 450) return false; return true; }
};

static void t_conditions() {
    printf("[beeper] what makes it beep\n");
    { Craft k; k.run(500); CHECK(k.on_ms(500) == 0, "a healthy, disarmed craft is quiet");
      size_t m = k.on.size(); k.in.armed = true; k.in.throttle_low = false; k.run(800);
      CHECK(starts_with(k.runs_since(m), {300, 50, 50}), "arming without GPS: the ARMING beep");
      m = k.on.size(); k.in.armed = false; k.in.throttle_low = true; k.run(800);
      CHECK(starts_with(k.runs_since(m), {150, 50, 150}), "disarming: two beeps"); }
    { Craft k; k.in.gps_enabled = true; k.in.gps_ready = false; k.run(100);
      size_t m = k.on.size(); k.in.armed = true; k.in.throttle_low = false; k.run(1500);
      CHECK(starts_with(k.runs_since(m), {300, 50, 300, 50, 300}), "arming with GPS but no fix: three long beeps (no home point!)");
      k.in.armed = false; k.run(1000); k.in.gps_ready = true;
      m = k.on.size(); k.run(2000);
      CHECK(starts_with(k.runs_since(m), {40, 50, 40, 50, 80}), "the fix arrives while disarmed: the READY tune, once");
      m = k.on.size(); k.run(3000); CHECK(k.on_ms(3000) == 0, "... only once");
      m = k.on.size(); k.in.armed = true; k.in.throttle_low = false; k.run(1500);
      CHECK(starts_with(k.runs_since(m), {50, 50, 150, 50, 50}), "arming with a fix: the GPS-fix tune"); }
    { Craft k; k.in.rx_ok = false; k.c = beep::Conditions(); k.run(3000);
      CHECK(k.on_ms(3000) == 0, "no receiver signal since power-on: not 'lost', nothing beeps");
      k.in.rx_ok = true; k.run(500); const size_t m = k.on.size(); k.in.rx_ok = false; k.run(4000);
      CHECK(starts_with(k.runs_since(m), {500, 500, 500, 500}), "signal was there and is gone: RX_LOST, repeating");
      k.in.app_active = true; k.run(1200); CHECK(k.on_ms(600) == 0, "with the configurator connected the lost-signal beep stops (it is a bench, not a lost model)");
      k.in.app_active = false; k.in.rx_ok = true; k.run(1200); CHECK(k.on_ms(600) == 0, "signal back: quiet"); }
    { Craft k; k.in.armed = true; k.in.throttle_low = false; k.run(1000);
      const size_t m = k.on.size(); k.in.rx_ok = false; k.in.failsafe_landing = true; k.run(2500);
      CHECK(starts_with(k.runs_since(m), {100, 100, 100, 100, 100, 400, 400}), "failsafe landing: SOS, not the plain lost-signal beep"); }
    { Craft k; k.run(200); size_t m = k.on.size(); k.in.batt_state = 2; k.run(2000);
      CHECK(starts_with(k.runs_since(m), {250, 500, 250}), "battery warning: 250 ms beeps");
      m = k.on.size(); k.in.batt_state = 3; k.run(2000);
      CHECK(k.on_ms(1000) > 900, "battery critical: nearly continuous tone (%d of 1000 ms)", k.on_ms(1000));
      k.in.batt_state = 1; k.run(1500); CHECK(k.on_ms(600) == 0, "battery fine again: quiet"); }
    { Craft k; k.run(200); const size_t m = k.on.size(); k.in.beeper_switch = true; k.run(1000);
      CHECK(starts_with(k.runs_since(m), {100, 100, 100, 100}), "BEEPER switch: short beeps for as long as it is on");
      k.in.beeper_switch = false; k.run(600); CHECK(k.on_ms(300) == 0, "switch off: quiet"); }
    { Craft k; k.in.gyro_calibrated = false; k.in.gyro_cal_running = true; k.run(1000); CHECK(k.on_ms(1000) == 0, "calibrating: quiet");
      const size_t m = k.on.size(); k.in.gyro_cal_running = false; k.in.gyro_calibrated = true; k.run(1200);
      CHECK(starts_with(k.runs_since(m), {200, 100, 200, 100, 200}), "gyro calibration finished: 3 beeps");
      k.in.gyro_cal_running = true; k.run(500); k.in.gyro_cal_running = false; k.in.gyro_calibrated = false; k.run(1200);
      CHECK(k.on_ms(1200) == 0, "a calibration that FAILED (craft moved) does not announce success"); }
    { Craft k; k.run(200); const size_t m = k.on.size(); k.in.arm_refused = 1; k.run(1000);
      CHECK(starts_with(k.runs_since(m), {400, 100, 100, 100, 100}), "arm switch flipped but a blocker says no: one long + two short");
      k.run(2000); CHECK(k.on_ms(1000) == 0, "once per attempt"); }
    { Craft k; k.in.armed = true; k.in.throttle_low = false; k.run(1000);
      k.in.throttle_low = true; k.run(6000);
      // pattern: 2.45 s pause, 100 ms beep, 50 ms pause -> a beep every 2.6 s
      CHECK(k.on_ms(5200) >= 190 && k.on_ms(5200) <= 210, "armed with the throttle down: a short reminder beep every 2.6 s (%d ms in 5.2 s)", k.on_ms(5200));
      k.in.throttle_low = false; k.run(3000); CHECK(k.on_ms(2900) == 0, "throttle up: the reminder stops"); }

    printf("[beeper] quiet on the bench\n");
    { Craft k; k.in.batt_state = 0; k.in.app_active = true; k.run(200);
      k.in.armed = true; k.in.throttle_low = false; k.run(800); k.in.armed = false; k.run(800); k.in.beeper_switch = true; k.run(800);
      CHECK(k.on_ms(2400) == 0 && k.c.muted(), "app connected, no battery (USB power): arming, disarming, beeper switch - all silent");
      k.b.force(beep::ARMING); const size_t m = k.on.size(); k.run(500);
      CHECK(starts_with(k.runs_since(m), {300, 50, 50}), "but the app's own test button is heard");
      k.in.batt_state = 1; k.run(400); CHECK(k.on_ms(200) > 50 && !k.c.muted(), "battery connected: it is not a bench any more, the beeper switch beeps");
      k.in.batt_state = 0; k.in.app_active = false; k.run(400); CHECK(k.on_ms(200) > 50, "USB power bank without the app: beeps too"); }
    { Craft k; k.b.enabled |= 1u << beep::ON_USB; k.in.batt_state = 0; k.in.app_active = true; k.run(200); const size_t m = k.on.size();
      k.in.armed = true; k.in.throttle_low = false; k.run(800);
      CHECK(starts_with(k.runs_since(m), {300, 50, 50}), "condition USB switched on: beeps on the bench as well"); }
    { Craft k; k.in.batt_state = 0; k.run(200); k.in.beeper_switch = true; k.run(150); const bool was_on = k.b.current() == beep::RX_SET;
      k.in.app_active = true; k.run(5);
      CHECK(was_on && k.b.current() == beep::NO_MODE && !k.on.back(), "the app connects in the middle of a beep: cut at once"); }

    printf("[beeper] the motors as the sounder (DShot beacon)\n");
    // one "on" part of a pattern = one tone = one 450 ms slot; the pauses stay as written
    { Craft k; k.in.gyro_calibrated = false; k.in.gyro_cal_running = true; k.run(500); const uint32_t t0 = k.in.now_ms;
      k.in.gyro_cal_running = false; k.in.gyro_calibrated = true; k.run(3000);
      // GYRO_CALIBRATED = {200, 100, 200, 100, 200, 100} ms -> tones at 0, 450 + 100, 2 * 550
      CHECK(k.beacons == 3 && k.beacon_at[0] - t0 <= 2 && k.gaps() == std::vector<int>({550, 550}), "gyro calibrated: the motors beep 3 times, 550 ms apart (%d tones)", k.beacons); }
    { Craft k; k.run(2000); CHECK(k.beacons == 0, "healthy and quiet: no tone");
      k.in.rx_ok = false; k.run(4800);
      // RX_LOST = {500 on, 500 off}: tone 450 + pause 500 + 1 ms in which the finished pattern is asked for again = 951 ms
      CHECK(k.beacons == 6 && k.gaps() == std::vector<int>({951, 951, 951, 951, 951}), "signal lost while disarmed: a tone every 951 ms (%d in 4.8 s)", k.beacons);
      int n = k.beacons; k.in.app_active = true; k.run(2000); CHECK(k.beacons == n && k.m.current() == beep::NO_MODE, "never with the configurator connected (it is a bench, not a lost model)");
      k.in.app_active = false; k.in.armed = true; k.in.throttle_low = false; k.run(2000); CHECK(k.beacons == n, "never while armed");
      const uint32_t t_dis = k.in.now_ms; k.in.armed = false; k.run(1190); CHECK(k.beacons == n, "and not in the first 1.2 s after a disarm (the motors are still spinning down)");
      k.run(3000); CHECK(k.beacons > n && k.beacon_at[(size_t)n] - t_dis >= 1200 && k.beacon_at[(size_t)n] - t_dis <= 1203, "then it starts (%u ms after the disarm)", (unsigned)(k.beacon_at[(size_t)n] - t_dis));
      CHECK(k.spaced(), "never two tones closer than 450 ms: the ESC needs that long for one"); }
    { Craft k; k.run(500); k.in.armed = true; k.in.throttle_low = false; k.run(1500); CHECK(k.beacons == 0, "arming: the buzzer's business, a motor that is about to spin does not beep");
      const size_t m0 = k.on.size(); const uint32_t t_dis = k.in.now_ms; k.in.armed = false; k.in.throttle_low = true; k.run(4000);
      CHECK(starts_with(k.runs_since(m0), {150, 50, 150}), "disarm: the buzzer says so at once");
      // DISARMING = {150, 50, 150, 50}: two tones, 450 + 50 apart, after the guard time
      CHECK(k.beacons == 2 && k.beacon_at[0] - t_dis >= 1200 && k.beacon_at[0] - t_dis <= 1203 && k.gaps() == std::vector<int>({500}), "the motors say it 1.2 s later: 2 tones (%d)", k.beacons); }
    { Craft k; k.run(500); k.in.armed = true; k.in.throttle_low = false; k.in.batt_state = 3; k.run(3000); CHECK(k.beacons == 0, "battery critical in flight: nothing from the motors (they are flying)");
      k.in.armed = false; k.run(1150); const bool quiet = k.beacons == 0; k.run(3000);
      // after the guard: DISARMING (2 tones) wins over BAT_CRIT_LOW, which then repeats every 450 + 20 + 1 ms
      const std::vector<int> g = k.gaps();
      CHECK(quiet && k.beacons >= 6 && g.size() >= 4 && g[0] == 500 && g[2] == 471 && g[3] == 471, "landed with an empty pack: 'disarmed', then the critical-battery tone every 471 ms (%d tones)", k.beacons); }
    { Craft k; k.run(200); k.in.batt_state = 2; k.run(3000);
      // BAT_LOW = {250 on, 500 off}: 450 + 500 + 1
      CHECK(k.beacons == 4 && k.gaps()[0] == 951, "battery warning on the ground: a tone every 951 ms (%d in 3 s)", k.beacons); }
    { Craft k; k.run(200); k.in.arm_refused = 1; k.run(3000);
      CHECK(k.beacons == 3 && k.gaps() == std::vector<int>({550, 550}), "arming refused: 3 tones, once (%d)", k.beacons); }
    { Craft k; k.run(200); k.in.beeper_switch = true; k.run(2000);
      // RX_SET = {100, 100}: 450 + 100 + 1
      CHECK(k.beacons == 4 && k.gaps()[0] == 551, "BEEPER switch: the motors beep for as long as it is on (%d in 2 s)", k.beacons);
      int n = k.beacons; k.m.enabled &= ~(1u << beep::RX_LOST); k.in.rx_ok = false; k.run(1500);
      CHECK(k.beacons <= n + 1 && k.on_ms(500) > 0, "a dead link reports stale switches: the motors stop, the buzzer keeps its own rule (%d more)", k.beacons - n); }
    { Craft k; k.run(200); k.c.event(beep::ACC_CALIBRATION, k.b); k.run(2000);
      CHECK(k.beacons == 2 && k.gaps() == std::vector<int>({500}) && k.on_ms(2000) == 100, "a one-shot event (accelerometer point done) goes to both sounders: 2 tones, 2 short beeps"); }
    { Craft k; k.motors_ok = false; k.run(200); k.in.rx_ok = false; k.run(3000); k.in.rx_ok = true; k.in.arm_refused = 3; k.run(2000);
      CHECK(k.beacons == 0 && !k.c.motors_ready(), "ESCs that cannot beep (PWM / Oneshot, outputs failed, motor test running): no tone is ever requested"); }
    { Craft k; k.m.enabled = 0; k.run(200); const size_t m0 = k.on.size(); k.in.rx_ok = false; k.run(3000);
      CHECK(k.beacons == 0 && starts_with(k.runs_since(m0), {500, 500, 500}), "all motor conditions switched off: silent motors, the buzzer is not affected"); }
    { Craft k; k.b.enabled = 0; k.run(200); k.in.rx_ok = false; k.run(3000);
      CHECK(k.beacons == 4 && k.on_ms(3000) == 0, "all buzzer conditions switched off: silent buzzer, the motors are not affected (%d tones)", k.beacons); }
    { Craft k; k.m.enabled = 0xFFFFFFFFu; k.run(200); k.in.armed = true; k.in.throttle_low = true; k.run(6000); k.in.failsafe_landing = true; k.run(3000);
      CHECK(k.beacons == 0, "even with every bit set the motors never play ARMING / ARMED / the landing SOS (%d)", k.beacons); }
    { // the 3-tone "gyro calibrated" is running (1 tone sent) when the craft is armed: the other two must never reach the ESCs
      Craft k; k.in.gyro_calibrated = false; k.in.gyro_cal_running = true; k.run(300);
      k.in.gyro_cal_running = false; k.in.gyro_calibrated = true; k.run(100); const int n = k.beacons; const beep::Mode playing = (beep::Mode)k.m.current();
      k.in.armed = true; k.in.throttle_low = false; k.run(1); const bool cut = k.m.current() == beep::NO_MODE; k.run(3000);
      CHECK(n == 1 && playing == beep::GYRO_CALIBRATED && cut && k.beacons == 1, "arming in the middle of a motor pattern stops it at once (%d tones)", k.beacons); }
    { // a tone that was started but not yet sent is dropped by silence(): nothing reaches the ESC afterwards
      beep::Beeper m; m.slot_ms = 450; m.enabled = beep::kMotorDefault; m.request(beep::RX_SET); m.update(1000); m.silence();
      const bool stale = m.take_start(); m.update(1001);
      CHECK(!stale && !m.take_start() && m.current() == beep::NO_MODE, "silence() also drops the tone that was about to be sent");
      m.request(beep::RX_SET); m.update(1002); CHECK(m.take_start() && !m.take_start(), "a started tone is handed out exactly once"); }
    { // the bench rule (app connected, no battery measured) mutes the buzzer only: an ESC that can beep has power, so a battery is
      // there even when nothing measures it (no ADC pin) - the motors keep beeping
      Craft k; k.in.batt_state = 0; k.in.app_active = true; k.run(200); k.in.arm_refused = 1; k.run(1500);
      CHECK(k.c.muted() && k.on_ms(1500) == 0 && k.beacons == 3, "on the bench: arming refused - the buzzer is quiet, the motors play it (3 tones: %d)", k.beacons);
      const int n0 = k.beacons; k.c.event(beep::ACC_CALIBRATION, k.b); k.run(1500);
      CHECK(k.on_ms(1500) == 0 && k.beacons > n0, "a one-shot event on the bench: on the motors only (%d tones)", k.beacons - n0);
      const int n1 = k.beacons; k.m.force(beep::GYRO_CALIBRATED); k.run(2000); CHECK(k.beacons - n1 == 3, "the app's test button: played (%d tones)", k.beacons - n1); }
    { // the app connects in the middle of a motor pattern: it goes on (the bench rule is the buzzer's)
      Craft k; k.in.batt_state = 0; k.in.gyro_calibrated = false; k.in.gyro_cal_running = true; k.run(300);
      k.in.gyro_cal_running = false; k.in.gyro_calibrated = true; k.run(100); const int n = k.beacons; k.in.app_active = true; k.run(3000);
      CHECK(n == 1 && k.beacons == 3, "the app connects during the 3-tone 'gyro calibrated': the motors finish it (%d)", k.beacons); }
    { // millis() wrapping (49 days) must not start or stop anything
      Craft k; k.in.now_ms = 0xFFFFFFFFu - 3000u; k.run(1000); k.in.rx_ok = false; k.run(5700);
      CHECK(k.beacons == 6 && k.gaps() == std::vector<int>({951, 951, 951, 951, 951}) && abs(k.on_ms(4000) - 2000) < 20, "through the millis() wrap: tones and RX_LOST keep their rhythm (%d tones, %d ms on)", k.beacons, k.on_ms(4000));
      k.in.rx_ok = true; k.in.armed = true; k.in.throttle_low = false; k.run(100);
      Craft w; w.in.now_ms = 0xFFFFFFFFu - 600u; w.run(100); w.in.armed = true; w.in.throttle_low = false; w.run(200); const uint32_t t_dis = w.in.now_ms; w.in.armed = false; w.run(3000);
      CHECK(w.beacons == 2 && w.beacon_at[0] - t_dis >= 1200 && w.beacon_at[0] - t_dis <= 1203, "the 1.2 s guard also holds across the wrap"); }
    { // a craft that idles for weeks: the guard stamp must not wrap into "just disarmed"
      Craft k; k.run(100); k.in.now_ms += 0x80000000u; k.run(10); k.in.now_ms += 0x7FFFF000u; k.run(10); k.in.rx_ok = false; k.run(100);
      CHECK(k.beacons == 1, "after 49 days on the shelf a lost signal beeps at once (%d)", k.beacons); }
}

int main() {
    t_battery();
    t_current();
    t_beeper();
    t_conditions();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
