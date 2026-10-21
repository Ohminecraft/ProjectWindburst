/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>
#include <string.h>

namespace app {

// ============================================================================
// What the LED strip (WS2812 / SK6812, pins.led, section led) shows - one picture per state of the craft, the most urgent
// first. Portable: the state goes in, a color per LED comes out (hal/led/ws2812.h sends it).
//
//   failsafe                              red, fast flash (5 Hz)
//   battery critical                      red, double blink every second
//   motor stall / runaway (disarmed by)   red / amber alternating
//   calibrating                           cyan, breathing
//   disarmed, something blocks arming     amber, slow blink (1 Hz)
//   disarmed, ready to arm                green
//   armed                                 the flight mode's color (Acro white, Angle blue, Stabilize cyan, Altitude Hold
//                                         purple, Position Hold teal, GPS Rescue orange, flashing); battery warning: amber
//                                         blink every 2 s on top
//   layout NAV, armed                     the strip's first half red (left), second half green (right), the last LED a
//                                         white strobe - aircraft navigation lights, the mode shows on the middle LED
//   layout MANUAL                         every LED its own function (led.fn[i]) and color (led.color[i]): the state picture
//                                         above, a fixed color, the mode's color, on when armed, blink, strobe, breathe,
//                                         rainbow, scanner, battery gauge, throttle. led.warnings: failsafe, battery
//                                         critical and a motor stall still take the whole strip. led.speed: 1 (slow) .. 10
// Brightness scales everything (the strip's current: 60 mA per LED at full white).
// ============================================================================
struct Rgb { uint8_t r = 0, g = 0, b = 0; };

struct LedInput {
    bool armed = false, ready = false, failsafe = false, batt_warning = false, batt_critical = false, calibrating = false, stall = false;
    enum Mode : uint8_t { ACRO, ANGLE, STABILIZE, ALTHOLD, POSHOLD, RESCUE } mode = ACRO;
    uint8_t batt_pct = 255;                          // what is left of the battery (%), 255 = no battery
    uint8_t throttle = 0;                            // the throttle stick (%)
};

class LedLogic {
public:
    enum Layout : uint8_t { ALL = 0, NAV = 1, MANUAL = 2 };      // == the schema's enum led_layout
    // == the schema's enum led_function (append only)
    enum Fn : uint8_t { F_OFF, F_STATUS, F_COLOR, F_MODE, F_ARMED, F_BLINK, F_STROBE, F_BREATHE, F_RAINBOW, F_LARSON, F_BATTERY, F_THROTTLE, F_COUNT };
    void configure(int count, uint8_t brightness_pct, Layout layout) { n_ = count < 0 ? 0 : count > kMax ? kMax : count; bright_ = brightness_pct > 100 ? 100 : brightness_pct; layout_ = layout; }
    // MANUAL: a function and a color (0xRRGGBB) for each of the kMax LEDs; an unknown function is OFF
    void manual(const uint8_t* fn, const uint32_t* color, bool warnings, uint8_t speed) {
        for (int i = 0; i < kMax; i++) { fn_[i] = fn[i] < F_COUNT ? fn[i] : (uint8_t)F_OFF; col_[i] = rgb(color[i]); }
        warn_ = warnings; speed_ = speed < 1 ? 1 : speed > 10 ? 10 : speed;
    }
    static constexpr int kMax = 32;

    // The colors at now_ms; returns the number of LEDs.
    int render(const LedInput& in, uint32_t now_ms, Rgb* out) const {
        const uint32_t t = now_ms;
        Rgb c; bool lit = true;
        if (in.failsafe) { c = kRed; lit = (t / 100) % 2 == 0; }
        else if (in.batt_critical) { c = kRed; const uint32_t p = t % 1000; lit = p < 100 || (p >= 200 && p < 300); }
        else if (in.stall) { c = (t / 250) % 2 ? kAmber : kRed; }
        else if (in.calibrating) { const uint32_t p = t % 1500; const uint32_t k = p < 750 ? p : 1500 - p; c = scale(kCyan, (uint8_t)(40 + k * 215 / 750)); }
        else if (!in.armed) { if (in.ready) c = kGreen; else { c = kAmber; lit = (t / 500) % 2 == 0; } }
        else {
            c = mode_color(in.mode);
            if (in.mode == LedInput::RESCUE) lit = (t / 250) % 2 == 0;
            if (in.batt_warning && t % 2000 < 200) c = kAmber;
        }
        if (!lit) c = Rgb();
        for (int i = 0; i < n_; i++) out[i] = c;
        if (layout_ == NAV && in.armed && !in.failsafe && !in.batt_critical && n_ >= 3) {
            const int half = n_ / 2;
            for (int i = 0; i < n_; i++) out[i] = i < half ? kRed : kGreen;
            out[half] = c;                                                     // the mode in the middle
            out[n_ - 1] = (t % 1000) < 60 ? kWhite : Rgb();                    // the strobe
        }
        if (layout_ == MANUAL && !(warn_ && (in.failsafe || in.batt_critical || in.stall))) manual_render(in, t, c, out);
        for (int i = 0; i < n_; i++) out[i] = scale(out[i], (uint8_t)(bright_ * 255 / 100));
        return n_;
    }
    static Rgb mode_color(LedInput::Mode m) {
        switch (m) { case LedInput::ANGLE: return kBlue; case LedInput::STABILIZE: return kCyan; case LedInput::ALTHOLD: return kPurple; case LedInput::POSHOLD: return kTeal; case LedInput::RESCUE: return kOrange; default: return kWhite; }
    }
    // Full saturation, full value; hue in degrees (0 red, 120 green, 240 blue).
    static Rgb hue(uint32_t h) {
        h %= 360; const uint8_t x = (uint8_t)((h % 60) * 255 / 60); Rgb o;
        switch (h / 60) { case 0: o.r = 255; o.g = x; break; case 1: o.r = (uint8_t)(255 - x); o.g = 255; break; case 2: o.g = 255; o.b = x; break;
                          case 3: o.g = (uint8_t)(255 - x); o.b = 255; break; case 4: o.r = x; o.b = 255; break; default: o.r = 255; o.b = (uint8_t)(255 - x); }
        return o;
    }
    static Rgb rgb(uint32_t v) { Rgb o; o.r = (uint8_t)(v >> 16); o.g = (uint8_t)(v >> 8); o.b = (uint8_t)v; return o; }
    static constexpr Rgb kRed{255, 0, 0}, kGreen{0, 255, 0}, kBlue{0, 0, 255}, kAmber{255, 110, 0}, kCyan{0, 200, 255}, kPurple{170, 0, 255}, kTeal{0, 255, 140}, kOrange{255, 60, 0}, kWhite{255, 255, 255};
private:
    static Rgb scale(Rgb c, uint8_t k) { Rgb o; o.r = (uint8_t)(c.r * k / 255); o.g = (uint8_t)(c.g * k / 255); o.b = (uint8_t)(c.b * k / 255); return o; }
    // One effect cycle: 10 s at speed 1, 2 s at 5, 1 s at 10.
    uint32_t cycle_ms() const { return 10000u / speed_; }
    void manual_render(const LedInput& in, uint32_t t, Rgb status, Rgb* out) const {
        const uint32_t cy = cycle_ms(), ph = t % cy;
        int n_rainbow = 0, n_larson = 0;                  // rainbow and scanner run across the LEDs that have them, in strip order
        for (int i = 0; i < n_; i++) { if (fn_[i] == F_RAINBOW) n_rainbow++; if (fn_[i] == F_LARSON) n_larson++; }
        int k_rainbow = 0, k_larson = 0;
        int head = 0;                                     // the scanner goes there and back once per cycle
        if (n_larson > 1) { const uint32_t span = 2u * (uint32_t)(n_larson - 1), x = ph * span / cy; head = (int)(x <= (uint32_t)(n_larson - 1) ? x : span - x); }
        for (int i = 0; i < n_; i++) {
            const Rgb col = col_[i]; Rgb o;
            switch (fn_[i]) {
            case F_STATUS: o = status; break;
            case F_COLOR: o = col; break;
            case F_MODE: o = mode_color(in.mode); break;
            case F_ARMED: if (in.armed) o = col; break;
            case F_BLINK: if (ph < cy / 2) o = col; break;
            case F_STROBE: if (t % (cy / 2) < 60) o = col; break;
            case F_BREATHE: { const uint32_t k = ph < cy / 2 ? ph * 2 * 255 / cy : (cy - ph) * 2 * 255 / cy; o = scale(col, (uint8_t)(20 + k * 235 / 255)); break; }
            case F_RAINBOW: o = hue(ph * 360 / cy + (uint32_t)(k_rainbow++ * 360 / n_rainbow)); break;
            case F_LARSON: { const int d = k_larson > head ? k_larson - head : head - k_larson; k_larson++; if (d == 0) o = col; else if (d == 1) o = scale(col, 50); break; }
            case F_BATTERY: if (in.batt_pct <= 100) { o.r = (uint8_t)(255 * (100 - in.batt_pct) / 100); o.g = (uint8_t)(255 * in.batt_pct / 100); } break;
            case F_THROTTLE: { const uint8_t p = in.throttle > 100 ? 100 : in.throttle; o.r = (uint8_t)(255 * p / 100); o.b = (uint8_t)(255 - o.r); break; }
            default: break;                               // F_OFF
            }
            out[i] = o;
        }
    }
    int n_ = 0; uint8_t bright_ = 50; Layout layout_ = ALL;
    uint8_t fn_[kMax] = {}; Rgb col_[kMax]; bool warn_ = true; uint8_t speed_ = 5;
};

} // namespace app
