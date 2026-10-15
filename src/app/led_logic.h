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
// Brightness scales everything (the strip's current: 60 mA per LED at full white).
// ============================================================================
struct Rgb { uint8_t r = 0, g = 0, b = 0; };

struct LedInput {
    bool armed = false, ready = false, failsafe = false, batt_warning = false, batt_critical = false, calibrating = false, stall = false;
    enum Mode : uint8_t { ACRO, ANGLE, STABILIZE, ALTHOLD, POSHOLD, RESCUE } mode = ACRO;
};

class LedLogic {
public:
    enum Layout : uint8_t { ALL = 0, NAV = 1 };      // == the schema's enum led_layout
    void configure(int count, uint8_t brightness_pct, Layout layout) { n_ = count < 0 ? 0 : count > kMax ? kMax : count; bright_ = brightness_pct > 100 ? 100 : brightness_pct; layout_ = layout; }
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
        for (int i = 0; i < n_; i++) out[i] = scale(out[i], (uint8_t)(bright_ * 255 / 100));
        return n_;
    }
    static Rgb mode_color(LedInput::Mode m) {
        switch (m) { case LedInput::ANGLE: return kBlue; case LedInput::STABILIZE: return kCyan; case LedInput::ALTHOLD: return kPurple; case LedInput::POSHOLD: return kTeal; case LedInput::RESCUE: return kOrange; default: return kWhite; }
    }
    static constexpr Rgb kRed{255, 0, 0}, kGreen{0, 255, 0}, kBlue{0, 0, 255}, kAmber{255, 110, 0}, kCyan{0, 200, 255}, kPurple{170, 0, 255}, kTeal{0, 255, 140}, kOrange{255, 60, 0}, kWhite{255, 255, 255};
private:
    static Rgb scale(Rgb c, uint8_t k) { Rgb o; o.r = (uint8_t)(c.r * k / 255); o.g = (uint8_t)(c.g * k / 255); o.b = (uint8_t)(c.b * k / 255); return o; }
    int n_ = 0; uint8_t bright_ = 50; Layout layout_ = ALL;
};

} // namespace app
