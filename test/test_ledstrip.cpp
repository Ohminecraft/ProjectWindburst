// The LED strip as Betaflight's (src/app/ledstrip.h): every expected value worked out by hand from master's io/ledstrip.c,
// common/colorconversion.c and drivers/light_ws2811strip.c, not taken from the code under test.
// Build: g++ -std=c++17 -O2 test/test_ledstrip.cpp -o build/test_ledstrip
#include <stdio.h>
#include <stdlib.h>
#include <initializer_list>
#include "../src/app/ledstrip.h"

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, ...) do { const bool ok_ = (cond); if (getenv("V")) { printf("    "); printf(__VA_ARGS__); printf("\n"); } if (ok_) g_pass++; else { g_fail++; printf("  FAIL line %d: %s   ", __LINE__, #cond); printf(__VA_ARGS__); printf("\n"); } } while (0)

using namespace led;

static bool rgb_is(const Rgb& c, int r, int g, int b) { return c.r == r && c.g == g && c.b == b; }

static void t_encoding() {
    printf("[encoding] the 32-bit led config, the palette, HSV -> RGB\n");
    // Betaflight's CLI "led 0 1,2:NE:FW:3": x 1, y 2, directions N E, function FLIGHT_MODE, overlay WARNING, color 3
    const uint32_t c = define_led(1, 2, 3, (1 << DIR_NORTH) | (1 << DIR_EAST), FN_FLIGHT_MODE, 1 << OL_WARNING);
    const uint32_t want = 0x12u | (1u << 8) | (0x40u << 12) | (3u << 22) | (3u << 26);
    CHECK(c == want, "1,2:NE:FW:3 = 0x%08x (0x%08x)", (unsigned)c, (unsigned)want);
    CHECK(led_x(c) == 1 && led_y(c) == 2 && led_function(c) == FN_FLIGHT_MODE && led_overlay(c, OL_WARNING) && !led_overlay(c, OL_BLINK) && led_color(c) == 3 && led_directions(c) == 3,
          "and read back field by field");
    CHECK(rgb_is(hsv_to_rgb(default_color(RED)), 255, 0, 0) && rgb_is(hsv_to_rgb(default_color(WHITE)), 255, 255, 255) && rgb_is(hsv_to_rgb(default_color(BLACK)), 0, 0, 0),
          "red, white (s 255 in Betaflight's HSV), black");
    { const Rgb o = hsv_to_rgb(default_color(ORANGE)); CHECK(rgb_is(o, 255, 63, 0), "orange (15, 0, 255): 255, 255 x 15 / 60 = 63, 0 (%u %u %u)", o.r, o.g, o.b); }
    { Hsv h; h.h = 120; h.s = 0; h.v = 255; const Rgb g = hsv_to_rgb(h); CHECK(rgb_is(g, 0, 255, 0), "hue 120: green"); }
    { Hsv h; h.h = 200; h.s = 128; h.v = 200; const Rgb x = hsv_to_rgb(h);
      // sat = 255 - 128 = 127; base = (255 - 127) x 200 >> 8 = 100; hue / 60 = 3: r = base, g = (100 x (60 - 20)) / 60 + 100 = 166, b = 200
      CHECK(rgb_is(x, 100, 166, 200), "a pale color: 100, 166, 200 (%u %u %u)", x.r, x.g, x.b); }
    Config d; d.defaults();
    CHECK(d.profile == PROFILE_STATUS && d.race_color == ORANGE && d.beacon_period_ms == 500 && d.beacon_percent == 50 && d.brightness == 100 && d.rainbow_freq == 120 && d.aux_channel == 3,
          "Betaflight's defaults: STATUS, orange race color, 500 ms / 50 %% beacon, 100 %%, rainbow 120, aux = throttle");
    CHECK(d.colors[ORANGE].h == 15 && d.colors[WHITE].s == 255 && d.mode_colors[MODE_ANGLE][DIR_NORTH] == CYAN && d.mode_colors[MODE_ORIENTATION][DIR_SOUTH] == RED
          && d.special[SC_ARMED] == BLUE && d.special[SC_DISARMED] == GREEN && d.special[SC_GPSNOLOCK] == ORANGE && d.mode_colors[MODE_BARO][0] == BLACK, "the default palette, mode and special colors");
}

struct Rig {
    Strip s; Input in; Rgb out[kMax]; uint32_t t = 0;
    Rig() { s.cfg.defaults(); }
    void layout(std::initializer_list<uint32_t> leds) { int i = 0; for (uint32_t l : leds) s.cfg.leds[i++] = l; s.configure(); }
    int run(double seconds, double step = 0.005) { int n = 0; for (double e = 0; e < seconds; e += step) { t += (uint32_t)(step * 1e6); s.update(t, in); n = s.render(out); } return n; }
};

static void t_fixed() {
    printf("[fixed layers] COLOR, ARM_STATE, FLIGHT_MODE, BATTERY_BAR, GPS_BAR, THRUST_RING, brightness\n");
    { Rig r; r.layout({define_led(0, 0, RED, 0, FN_COLOR, 0), define_led(1, 0, 0, 0, FN_ARM_STATE, 0)});
      CHECK(r.run(0.1) == 2 && rgb_is(r.out[0], 255, 0, 0), "COLOR red: red; two LEDs configured, two driven");
      { const Rgb a = hsv_to_rgb(default_color(GREEN)); CHECK(rgb_is(r.out[1], a.r, a.g, a.b), "ARM_STATE disarmed: green (special DISARMED)"); }
      r.in.armed = true; r.run(1.1); { const Rgb a = hsv_to_rgb(default_color(BLUE)); CHECK(rgb_is(r.out[1], a.r, a.g, a.b), "armed: blue"); }
      r.s.cfg.brightness = 50; r.run(1.1); CHECK(rgb_is(r.out[0], 127, 0, 0), "brightness 50 %%: v 255 x 50 / 100 = 127 (%u)", r.out[0].r); }
    { Rig r; r.layout({define_led(5, 0, 0, 1 << DIR_NORTH, FN_FLIGHT_MODE, 0), define_led(5, 10, 0, 1 << DIR_SOUTH, FN_FLIGHT_MODE, 0)});
      r.run(0.1); CHECK(rgb_is(r.out[0], 255, 255, 255) && rgb_is(r.out[1], 255, 0, 0), "FLIGHT_MODE, no mode (ORIENTATION): north white, south red");
      r.in.mode_angle = true; r.run(1.1); { const Rgb c = hsv_to_rgb(default_color(CYAN)); CHECK(rgb_is(r.out[0], c.r, c.g, c.b), "ANGLE: north cyan"); } }
    { Rig r; r.layout({define_led(0, 0, 0, 0, FN_BATTERY_BAR, 0), define_led(1, 0, 0, 0, FN_BATTERY_BAR, 0), define_led(2, 0, 0, 0, FN_BATTERY_BAR, 0), define_led(3, 0, 0, 0, FN_BATTERY_BAR, 0)});
      r.in.batt_percent = 50; r.run(0.1);
      // hue: red 0 + scaleRange(50, 0, 100, -30, 120) = 45; lit while the counter < 50 x 4 / 100 = 2
      Hsv h; h.h = 45; h.s = 0; h.v = 255; const Rgb lit = hsv_to_rgb(h);
      CHECK(rgb_is(r.out[0], lit.r, lit.g, lit.b) && rgb_is(r.out[1], lit.r, lit.g, lit.b) && rgb_is(r.out[2], 0, 0, 0) && rgb_is(r.out[3], 0, 0, 0), "BATTERY_BAR 50 %% of 4: two lit (hue 45), two background"); }
    { Rig r; r.layout({define_led(0, 0, 0, 0, FN_GPS_BAR, 0), define_led(1, 0, 0, 0, FN_GPS_BAR, 0), define_led(2, 0, 0, 0, FN_GPS_BAR, 0), define_led(3, 0, 0, 0, FN_GPS_BAR, 0)});
      // Betaflight picks the GPS layer's LEDs by bit mask: function GPS (5) is inside GPS_BAR (7), so the GPS layer paints these
      // too - no fix: GPSNOLOCK (orange) / GPSNOSATS (red) flashes; a fix: GPSLOCKED (green) / background
      r.in.gps_sensor = true; r.in.gps_sats = 3; r.in.gps_min_sats = 8;
      const Rgb o = hsv_to_rgb(default_color(ORANGE)), g = hsv_to_rgb(default_color(GREEN)); bool orange = false, red = false, green = false;
      for (int k = 0; k < 400; k++) { r.run(0.01); orange |= rgb_is(r.out[3], o.r, o.g, o.b); red |= rgb_is(r.out[3], 255, 0, 0); }
      CHECK(orange && red, "GPS_BAR, 3 sats, no fix: the GPS layer's orange / red flashes (Betaflight's mask)");
      r.in.gps_sats = 9; r.in.gps_fix = true; for (int k = 0; k < 400; k++) { r.run(0.01); green |= rgb_is(r.out[3], g.r, g.g, g.b); }
      CHECK(green, "9 sats and a fix: green"); }
    { Rig r; r.layout({define_led(0, 0, BLUE, 0, FN_THRUST_RING, 0), define_led(1, 0, BLUE, 0, FN_THRUST_RING, 0), define_led(2, 0, BLUE, 0, FN_THRUST_RING, 0), define_led(3, 0, BLUE, 0, FN_THRUST_RING, 0)});
      r.run(0.1); const Rgb b = hsv_to_rgb(default_color(BLUE));
      CHECK(rgb_is(r.out[0], b.r, b.g, b.b) && rgb_is(r.out[1], 0, 0, 0) && rgb_is(r.out[2], b.r, b.g, b.b) && rgb_is(r.out[3], 0, 0, 0), "THRUST_RING disarmed: every other one (alternating)");
      r.in.armed = true; int lit_max = 0; for (int k = 0; k < 40; k++) { r.run(0.02); int lit = 0; for (int i = 0; i < 4; i++) lit += r.out[i].b > 0; lit_max = lit > lit_max ? lit : lit_max; }
      CHECK(lit_max == 2, "armed: 2 of the ring's sequence of 4 turn round (%d lit)", lit_max); }
    { Rig r; r.layout({define_led(0, 0, GREEN, 0, FN_COLOR, 1 << OL_THROTTLE)});
      r.in.channel[3] = -1.0f; r.run(0.1); { const Rgb y = hsv_to_rgb(default_color(LIME_GREEN)); CHECK(rgb_is(r.out[0], y.r, y.g, y.b), "COLOR + THROTTLE, aux -1: the previous palette color (lime green)"); }
      r.in.channel[3] = 1.0f; r.run(1.1); { const Rgb m = hsv_to_rgb(default_color(MINT_GREEN)); CHECK(rgb_is(r.out[0], m.r, m.g, m.b), "aux +1: the next one (mint green)"); } }
}

static void t_overlays() {
    printf("[overlays] WARNING, INDICATOR, BLINK, RAINBOW, LARSON\n");
    { Rig r; r.layout({define_led(0, 0, RED, 0, FN_COLOR, 1 << OL_WARNING)});
      r.in.failsafe = true; bool yellow = false, blue = false, red = false;
      const Rgb y = hsv_to_rgb(default_color(YELLOW)), b = hsv_to_rgb(default_color(BLUE));
      for (int k = 0; k < 400; k++) { r.run(0.01); yellow |= rgb_is(r.out[0], y.r, y.g, y.b); blue |= rgb_is(r.out[0], b.r, b.g, b.b); red |= rgb_is(r.out[0], 255, 0, 0); }
      CHECK(yellow && blue && red, "failsafe: yellow / blue flashes (the failsafe slot of the 1.6 s cycle), its own red in the other slots");
      r.in.failsafe = false; r.in.beeper_on = true; r.run(2.0); CHECK(rgb_is(r.out[0], 255, 255, 255), "no warning, the beeper sounds: the visual beeper color (white)"); }
    { Rig r; r.layout({define_led(0, 5, BLUE, 0, FN_COLOR, 1 << OL_INDICATOR), define_led(10, 5, BLUE, 0, FN_COLOR, 1 << OL_INDICATOR)});
      r.in.rx_signal = true; r.in.roll = 0.5f; bool east_orange = false, west_orange = false; const Rgb o = hsv_to_rgb(default_color(ORANGE));
      for (int k = 0; k < 200; k++) { r.run(0.005); east_orange |= rgb_is(r.out[1], o.r, o.g, o.b); west_orange |= rgb_is(r.out[0], o.r, o.g, o.b); }
      CHECK(east_orange && !west_orange, "INDICATOR, roll right: the east LED flashes orange, the west one not"); }
    { Rig r; r.layout({define_led(0, 0, RED, 0, FN_COLOR, 1 << OL_BLINK)});
      int on = 0, off = 0; for (int k = 0; k < 160; k++) { r.run(0.01); if (r.out[0].r) on++; else off++; }
      // pattern 0b1000000000000101 at 10 Hz: 2 of every 15 steps on
      CHECK(on > 0 && off > on * 4, "BLINK: blinks twice, then waits (%d on, %d off)", on, off); }
    { Rig r; r.layout({define_led(0, 0, RED, 0, FN_COLOR, 1 << OL_RAINBOW)}); r.s.cfg.rainbow_freq = 600;
      r.run(0.02); const Rgb a = r.out[0]; r.run(0.5); const Rgb b = r.out[0];
      CHECK(!rgb_is(a, b.r, b.g, b.b), "RAINBOW: the hue moves (%u %u %u -> %u %u %u)", a.r, a.g, a.b, b.r, b.g, b.b); }
    { Rig r; r.layout({define_led(0, 0, RED, 0, FN_COLOR, 1 << OL_LARSON_SCANNER), define_led(1, 0, RED, 0, FN_COLOR, 1 << OL_LARSON_SCANNER), define_led(2, 0, RED, 0, FN_COLOR, 1 << OL_LARSON_SCANNER), define_led(3, 0, RED, 0, FN_COLOR, 1 << OL_LARSON_SCANNER)});
      int brightest_at[4] = {}; for (int k = 0; k < 300; k++) { r.run(0.01); int m = 0; for (int i = 1; i < 4; i++) if (r.out[i].r > r.out[m].r) m = i; brightest_at[m]++; }
      CHECK(brightest_at[0] && brightest_at[3], "LARSON: the bright spot runs from one end to the other (%d %d %d %d)", brightest_at[0], brightest_at[1], brightest_at[2], brightest_at[3]); }
}

static void t_profiles() {
    printf("[profiles] RACE, BEACON\n");
    { Rig r; r.layout({define_led(1, 0, 0, 0, FN_COLOR, 0), define_led(2, 0, 0, 0, FN_COLOR, 0)}); r.s.cfg.profile = PROFILE_RACE;   // (0,0 COLOR 0 is 0: the end of the list)
      const Rgb o = hsv_to_rgb(default_color(ORANGE));
      CHECK(r.run(0.1) == 2 && rgb_is(r.out[0], o.r, o.g, o.b) && rgb_is(r.out[1], o.r, o.g, o.b), "RACE: the configured LEDs all orange");
      r.in.beeper_on = true; r.run(0.1); CHECK(rgb_is(r.out[1], 255, 255, 255), "the beeper: the visual beeper color");
      r.in.beeper_on = false; r.in.failsafe = true; int w = 0, n = 0; for (int k = 0; k < 100; k++) { r.run(0.01); w += r.out[0].b == 255; n++; }
      CHECK(w > 30 && w < 70, "failsafe: 4 Hz, 50 %% white flashes (%d of %d)", w, n); }
    { Rig r; r.layout({define_led(1, 0, 0, 0, FN_COLOR, 0)}); r.s.cfg.profile = PROFILE_BEACON;
      int w = 0; for (int k = 0; k < 200; k++) { r.run(0.01); w += r.out[0].r == 255; }
      CHECK(w > 80 && w < 120, "BEACON: white 50 %% of the 500 ms period (%d of 200)", w); }
    { Rig r; r.layout({}); CHECK(r.run(0.1) == 0, "no LED configured: nothing driven (Betaflight)"); }
    { Rig r; r.layout({define_led(1, 0, 0, 0, FN_COLOR, 0), 0u, define_led(3, 0, 0, 0, FN_COLOR, 0)}); CHECK(r.run(0.1) == 1, "a config of 0 ends the list (Betaflight's updateLedCount)"); }
}

int main() {
    t_encoding();
    t_fixed();
    t_overlays();
    t_profiles();
    printf("%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
