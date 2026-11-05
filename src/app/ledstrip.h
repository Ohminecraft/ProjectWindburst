/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// ============================================================================
// The LED strip as Betaflight's (0.44; written from master's io/ledstrip.c, common/colorconversion.c, drivers/light_ws2811strip.c):
// the Betaflight App's LED Strip tab configures it over MSP (MSP_LED_STRIP_CONFIG, MSP_LED_COLORS, MSP_LED_STRIP_MODECOLOR,
// MSP2_*_LED_STRIP_CONFIG_VALUES) - the same 32-bit led configs, 16 HSV colors, mode / special colors, profiles.
//
//   led config (u32): position x,y (4 + 4 bits, a 16 x 16 grid) | function (4) | overlays (10) | color (4) | directions (6)
//   profiles:  RACE    the whole strip one color (race_color; black: Betaflight's VTX color - none here: black), the visual beeper
//                      color while the beeper sounds; failsafe / the BEEPER switch: 4 Hz 50 % flashes of the visual beeper color
//              BEACON  the beacon color flashing (beacon_period_ms, beacon_percent)
//              STATUS  each LED its function (COLOR, FLIGHT_MODE, ARM_STATE, BATTERY, RSSI, GPS, THRUST_RING, GPS_BAR, BATTERY_BAR,
//                      ALTITUDE) and overlays (THROTTLE, RAINBOW, LARSON_SCANNER, BLINK, INDICATOR, WARNING; VTX: none here),
//                      the timed layers at Betaflight's rates
//   brightness: v x brightness / 100, then Betaflight's HSV -> RGB (hsvToRgb24)
// Betaflight's HSV: s = 0 is full color and s = 255 white (it is 255 - the usual saturation).
// As Betaflight, the timed layers pick their LEDs by bit mask ((config & mask) == mask): the GPS layer (function 5) also takes
// the GPS_BAR LEDs (7), the RSSI layer (4) the GPS, THRUST_RING and GPS_BAR ones while it flashes - kept, so a strip looks as it
// does on Betaflight.
// Not here: the LEDLOW switch, the VTX overlay / VTX race color, the GRBW strips. Portable: test/test_ledstrip.cpp.
// ============================================================================
namespace led {

constexpr int kMax = 32;                       // LED_STRIP_MAX_LENGTH
constexpr int kColors = 16, kModes = 6, kDirections = 6, kSpecial = 11, kFunctions = 10, kOverlays = 7;
constexpr int kHueMax = 359;

struct Hsv { uint16_t h = 0; uint8_t s = 0, v = 0; };
struct Rgb { uint8_t r = 0, g = 0, b = 0; };

enum Color : uint8_t { BLACK, WHITE, RED, ORANGE, YELLOW, LIME_GREEN, GREEN, MINT_GREEN, CYAN, LIGHT_BLUE, BLUE, DARK_VIOLET, MAGENTA, DEEP_PINK, COLOR_COUNT };
enum Mode : uint8_t { MODE_ORIENTATION, MODE_HEADFREE, MODE_HORIZON, MODE_ANGLE, MODE_MAG, MODE_BARO, MODE_SPECIAL, MODE_AUX_CHANNEL };
enum Special : uint8_t { SC_DISARMED, SC_ARMED, SC_ANIMATION, SC_BACKGROUND, SC_BLINKBACKGROUND, SC_GPSNOSATS, SC_GPSNOLOCK, SC_GPSLOCKED };
enum Direction : uint8_t { DIR_NORTH, DIR_EAST, DIR_SOUTH, DIR_WEST, DIR_UP, DIR_DOWN };
enum Function : uint8_t { FN_COLOR, FN_FLIGHT_MODE, FN_ARM_STATE, FN_BATTERY, FN_RSSI, FN_GPS, FN_THRUST_RING, FN_GPS_BAR, FN_BATTERY_BAR, FN_ALTITUDE };
enum Overlay : uint8_t { OL_THROTTLE, OL_RAINBOW, OL_LARSON_SCANNER, OL_BLINK, OL_VTX, OL_INDICATOR, OL_WARNING };
enum Profile : uint8_t { PROFILE_RACE, PROFILE_BEACON, PROFILE_STATUS };

// the 32-bit led config (Betaflight's ledstrip.h, API 1.46 layout)
constexpr int kPosOffset = 0, kFunctionOffset = 8, kOverlayOffset = 12, kColorOffset = 22, kDirectionOffset = 26;
inline uint32_t define_led(int x, int y, int color, int dirs, int fn, int overlays) {
    return (uint32_t)((((x & 0x0F) << 4) | (y & 0x0F)) << kPosOffset) | ((uint32_t)(color & 0x0F) << kColorOffset) | ((uint32_t)(dirs & 0x3F) << kDirectionOffset)
         | ((uint32_t)(fn & 0x0F) << kFunctionOffset) | ((uint32_t)(overlays & 0x3FF) << kOverlayOffset);
}
inline uint8_t led_x(uint32_t c) { return (uint8_t)((c >> (kPosOffset + 4)) & 0x0F); }
inline uint8_t led_y(uint32_t c) { return (uint8_t)((c >> kPosOffset) & 0x0F); }
inline uint8_t led_function(uint32_t c) { return (uint8_t)((c >> kFunctionOffset) & 0x0F); }
inline uint16_t led_overlays(uint32_t c) { return (uint16_t)((c >> kOverlayOffset) & 0x3FF); }
inline uint8_t led_color(uint32_t c) { return (uint8_t)((c >> kColorOffset) & 0x0F); }
inline uint8_t led_directions(uint32_t c) { return (uint8_t)((c >> kDirectionOffset) & 0x3F); }
inline bool led_overlay(uint32_t c, int ol) { return (led_overlays(c) >> ol) & 1; }

// Betaflight's default palette (ledstrip.c hsv[])
inline Hsv default_color(int i) {
    static const uint16_t h[COLOR_COUNT] = {0, 0, 0, 15, 50, 100, 115, 125, 180, 210, 240, 270, 300, 330};
    Hsv c; if (i <= BLACK || i >= COLOR_COUNT) return c;
    c.h = h[i]; c.s = i == WHITE ? 255 : 0; c.v = 255; return c;
}

// common/colorconversion.c hsvToRgb24
inline Rgb hsv_to_rgb(const Hsv& c) {
    Rgb r; const uint16_t val = c.v, sat = (uint16_t)(255 - c.s), hue = c.h;
    if (sat == 0) { r.r = r.g = r.b = (uint8_t)val; return r; }
    const uint32_t base = ((255u - sat) * val) >> 8;
    switch (hue / 60) {
    case 0: r.r = (uint8_t)val; r.g = (uint8_t)((((val - base) * hue) / 60) + base); r.b = (uint8_t)base; break;
    case 1: r.r = (uint8_t)((((val - base) * (60 - (hue % 60))) / 60) + base); r.g = (uint8_t)val; r.b = (uint8_t)base; break;
    case 2: r.r = (uint8_t)base; r.g = (uint8_t)val; r.b = (uint8_t)((((val - base) * (hue % 60)) / 60) + base); break;
    case 3: r.r = (uint8_t)base; r.g = (uint8_t)((((val - base) * (60 - (hue % 60))) / 60) + base); r.b = (uint8_t)val; break;
    case 4: r.r = (uint8_t)((((val - base) * (hue % 60)) / 60) + base); r.g = (uint8_t)base; r.b = (uint8_t)val; break;
    case 5: r.r = (uint8_t)val; r.g = (uint8_t)base; r.b = (uint8_t)((((val - base) * (60 - (hue % 60))) / 60) + base); break;
    default: break;
    }
    return r;
}

// Betaflight's settings (ledStripConfig_t + ledStripStatusModeConfig_t), defaults as pgResetFn_*
struct Config {
    uint32_t leds[kMax] = {};
    Hsv colors[kColors];
    uint8_t mode_colors[kModes][kDirections] = {};
    uint8_t special[kSpecial] = {};
    uint8_t aux_channel = 3;                   // THROTTLE (Betaflight's channel order roll, pitch, yaw, throttle, aux 1..)
    uint8_t profile = PROFILE_STATUS, race_color = ORANGE, beacon_color = WHITE, visual_beeper = 1, visual_beeper_color = WHITE;
    uint8_t brightness = 100, beacon_percent = 50, beacon_armed_only = 0;
    uint16_t beacon_period_ms = 500, rainbow_delta = 0, rainbow_freq = 120;
    void defaults() {
        *this = Config();
        for (int i = 0; i < kColors; i++) colors[i] = default_color(i);
        static const uint8_t mc[5][kDirections] = {
            {WHITE, DARK_VIOLET, RED, DEEP_PINK, BLUE, ORANGE},          // ORIENTATION
            {LIME_GREEN, DARK_VIOLET, ORANGE, DEEP_PINK, BLUE, ORANGE},  // HEADFREE
            {BLUE, DARK_VIOLET, YELLOW, DEEP_PINK, BLUE, ORANGE},        // HORIZON
            {CYAN, DARK_VIOLET, YELLOW, DEEP_PINK, BLUE, ORANGE},        // ANGLE
            {MINT_GREEN, DARK_VIOLET, ORANGE, DEEP_PINK, BLUE, ORANGE}}; // MAG
        memcpy(mode_colors, mc, sizeof(mc));                             // (BARO: all black, as Betaflight's)
        const uint8_t sc[8] = {GREEN, BLUE, WHITE, BLACK, BLACK, RED, ORANGE, GREEN};
        memcpy(special, sc, sizeof(sc));
    }
    Config() { for (int i = 0; i < kColors; i++) colors[i] = Hsv(); }
};

// What the layers look at (filled by FlightApp from the telemetry; channel values -1..1 as Betaflight's rcGetChannel)
struct Input {
    bool armed = false, arming_disabled = false, failsafe = false, crashflip = false, beeper_on = false, beeper_switch = false;
    bool batt_meter = false; uint8_t batt_state = 0;   // 0 OK, 1 WARNING, 2 CRITICAL (Betaflight's BATTERY_OK / WARNING / CRITICAL)
    uint8_t batt_percent = 0, rssi_percent = 0;
    bool gps_sensor = false, gps_fix = false; uint8_t gps_sats = 0, gps_min_sats = 8;
    int32_t alt_cm = 0;
    bool mode_headfree = false, mode_mag = false, mode_horizon = false, mode_angle = false;
    bool rx_signal = false; float roll = 0, pitch = 0;  // rcCommand roll / pitch, -1..1
    float channel[16] = {};                            // in Betaflight's order (roll, pitch, yaw, throttle, aux 1..), -1..1
};

class Strip {
public:
    Config cfg;
    // after a change of cfg (Betaflight's reevaluateLedConfig)
    void configure() {
        count_ = ring_ = larson_ = 0;
        for (int i = 0; i < kMax; i++) {
            const uint32_t c = cfg.leds[i]; if (!c) break;
            count_++;
            if (led_function(c) == FN_THRUST_RING) ring_++;
            if (led_overlay(c, OL_LARSON_SCANNER)) larson_++;
        }
        update_dimensions(); update_ring(); update_overlays(); update_bars();
    }
    int count() const { return count_; }
    // the LEDs' colors at now_us (Betaflight's ledStripUpdate, a whole pass at a time); true when they changed
    bool update(uint32_t now_us, const Input& in) {
        in_ = &in;
        bool changed = false;
        if (cfg.profile == PROFILE_STATUS) changed = status_profile(now_us);
        else changed = simple_profile(now_us);
        in_ = nullptr;
        return changed;
    }
    // the RGB the strip gets (brightness applied as ws2811UpdateStrip does); returns the number of LEDs: the configured ones, in
    // every profile (Betaflight's setUsedLedCount - a RACE / BEACON strip also needs its LEDs in the layout)
    int render(Rgb* out) const {
        const int n = count_;
        for (int i = 0; i < n; i++) { Hsv s = hsv_[i]; s.v = (uint8_t)(s.v * cfg.brightness / 100); out[i] = hsv_to_rgb(s); }
        return n;
    }
    const Hsv& hsv(int i) const { return hsv_[i]; }

private:
    // ---- the layout (ledstrip.c updateDimensions / updateLedRingCounts / updateRequiredOverlay / updateLedBars)
    void update_dimensions() {
        int maxX = 0, minX = 15, maxY = 0, minY = 15;
        for (int i = 0; i < count_; i++) { const int x = led_x(cfg.leds[i]), y = led_y(cfg.leds[i]); maxX = x > maxX ? x : maxX; minX = x < minX ? x : minX; maxY = y > maxY ? y : maxY; minY = y < minY ? y : minY; }
        if (minX < maxX) { eastX_ = (minX + maxX) / 2 + 1; westX_ = (minX + maxX - 1) / 2; } else { eastX_ = 15 / 2; westX_ = eastX_ - 1; }
        if (minY < maxY) { southY_ = (minY + maxY) / 2 + 1; northY_ = (minY + maxY - 1) / 2; } else { southY_ = 15 / 2; northY_ = southY_ - 1; }
    }
    void update_ring() {
        int seq;
        if (ring_ % 6 == 0) seq = 6;
        else { seq = ring_; while (seq > 6 && (seq % 2) == 0) seq /= 2; }
        ring_seq_ = seq;
    }
    bool overlay_used(int ol) const { for (int i = 0; i < count_; i++) if (led_overlay(cfg.leds[i], ol)) return true; return false; }
    void update_overlays() {
        disabled_ = 0;
        if (!overlay_used(OL_RAINBOW)) disabled_ |= 1u << T_RAINBOW;
        if (!overlay_used(OL_BLINK)) disabled_ |= 1u << T_BLINK;
        if (!overlay_used(OL_LARSON_SCANNER)) disabled_ |= 1u << T_LARSON;
        if (!overlay_used(OL_WARNING)) disabled_ |= 1u << T_WARNING;
        if (!overlay_used(OL_INDICATOR)) disabled_ |= 1u << T_INDICATOR;
    }
    void update_bars() { bar_gps_ = bar_batt_ = 0; for (int i = 0; i < count_; i++) { const int fn = led_function(cfg.leds[i]); if (fn == FN_GPS_BAR) bar_gps_++; else if (fn == FN_BATTERY_BAR) bar_batt_++; } }

    // ---- helpers
    const Hsv& color(int i) const { return cfg.colors[i & 0x0F]; }
    const Hsv& sc(int s) const { return cfg.colors[cfg.special[s] & 0x0F]; }
    static Hsv basic(int c) { return default_color(c); }
    static int scale_range(int x, int srcFrom, int srcTo, int destFrom, int destTo) {         // common/maths.c scaleRange
        const long a = ((long)destTo - destFrom) * ((long)x - srcFrom); const long b = srcTo - srcFrom; return (int)((a / b) + destFrom);
    }
    static float scale_rangef(float x, float srcFrom, float srcTo, float destFrom, float destTo) { return (destTo - destFrom) * (x - srcFrom) / (srcTo - srcFrom) + destFrom; }
    void set(int i, const Hsv& c) { hsv_[i] = c; }
    void apply_mask(uint32_t mask, const Hsv& c) { for (int i = 0; i < count_; i++) if ((cfg.leds[i] & mask) == mask) hsv_[i] = c; }
    enum Quadrant : uint8_t { Q_NORTH = 1, Q_SOUTH = 2, Q_EAST = 4, Q_WEST = 8 };
    int quadrant(int i) const {
        const int x = led_x(cfg.leds[i]), y = led_y(cfg.leds[i]); int q = 0;
        if (y <= northY_) q |= Q_NORTH; else if (y >= southY_) q |= Q_SOUTH;
        if (x >= eastX_) q |= Q_EAST; else if (x <= westX_) q |= Q_WEST;
        return q;
    }
    const Hsv* directional(int i, int mode) const {
        const int d = led_directions(cfg.leds[i]);
        for (int k = 0; k < kDirections; k++) if (d & (1 << k)) return &cfg.colors[cfg.mode_colors[mode][k] & 0x0F];
        return nullptr;
    }
    float channel(int ch) const { return ch >= 0 && ch < 16 ? in_->channel[ch] : 0.0f; }

    // ---- applyLedFixedLayers
    void fixed_layers() {
        const Input& in = *in_;
        int bar_gps = 0, bar_batt = 0;
        for (int i = 0; i < count_; i++) {
            const uint32_t c = cfg.leds[i];
            Hsv col = sc(SC_BACKGROUND);
            const int fn = led_function(c); int hOffset = kHueMax + 1;
            switch (fn) {
            case FN_COLOR: {
                col = color(led_color(c));
                const Hsv next = color((led_color(c) + 1 + kColors) % kColors), prev = color((led_color(c) - 1 + kColors) % kColors);
                if (led_overlay(c, OL_THROTTLE)) {                 // a smooth fade with the aux channel from the previous color through this one to the next
                    const float a = channel(cfg.aux_channel);
                    if (a < 0.0f) { col.h = (uint16_t)scale_rangef(a, -1.0f, 0.0f, prev.h, col.h); col.s = (uint8_t)scale_rangef(a, -1.0f, 0.0f, prev.s, col.s); col.v = (uint8_t)scale_rangef(a, -1.0f, 0.0f, prev.v, col.v); }
                    else { col.h = (uint16_t)scale_rangef(a, 0.0f, 1.0f, col.h, next.h); col.s = (uint8_t)scale_rangef(a, 0.0f, 1.0f, col.s, next.s); col.v = (uint8_t)scale_rangef(a, 0.0f, 1.0f, col.v, next.v); }
                }
                break; }
            case FN_FLIGHT_MODE: {                                 // flightModeToLed, in order of priority
                const int m = in.mode_headfree ? MODE_HEADFREE : in.mode_mag ? MODE_MAG : in.mode_horizon ? MODE_HORIZON : in.mode_angle ? MODE_ANGLE : MODE_ORIENTATION;
                if (const Hsv* d = directional(i, m)) col = *d;
                break; }
            case FN_ARM_STATE: col = in.armed ? sc(SC_ARMED) : sc(SC_DISARMED); break;
            case FN_BATTERY: case FN_BATTERY_BAR: { col = basic(RED); const int o = scale_range(in.batt_percent, 0, 100, -30, 120); hOffset += o > 0 ? o : 0; break; }
            case FN_GPS_BAR:
                if (in.gps_sats == 0 || !in.gps_sensor) col = basic(RED);
                else if (in.gps_sats >= in.gps_min_sats) col = basic(GREEN);
                else { col = basic(RED); const int o = scale_range(in.gps_sats, 0, in.gps_min_sats, -30, 120); hOffset += o > 0 ? o : 0; }
                break;
            case FN_ALTITUDE: { col = color(led_color(c)); const int o = scale_range(in.alt_cm, 0, 500, -30, 120); hOffset += o > 0 ? o : 0; break; }
            case FN_RSSI: { col = basic(RED); const int o = scale_range(in.rssi_percent, 0, 100, -30, 120); hOffset += o > 0 ? o : 0; break; }
            default: break;
            }
            if (fn != FN_COLOR && led_overlay(c, OL_THROTTLE)) hOffset += (int)scale_rangef(channel(cfg.aux_channel), -1.0f, 1.0f, 0.0f, (float)(kHueMax + 1));
            col.h = (uint16_t)((col.h + hOffset) % (kHueMax + 1));
            if (fn == FN_GPS_BAR) { if (bar_gps < in.gps_sats || bar_gps == 0) { bar_gps++; set(i, col); } else set(i, sc(SC_BACKGROUND)); }
            else if (fn == FN_BATTERY_BAR) { if (bar_batt < (in.batt_percent * bar_batt_) / 100 || bar_batt == 0) { bar_batt++; set(i, col); } else set(i, sc(SC_BACKGROUND)); }
            else set(i, col);
        }
    }

    // ---- the timed layers (in reverse order of priority, as Betaflight's timId_e; no VTX)
    enum Timer : uint8_t { T_RAINBOW, T_BLINK, T_LARSON, T_RING, T_INDICATOR, T_GPS, T_BATTERY, T_RSSI, T_WARNING, T_COUNT };
    static int32_t hz_us(float hz) { return (int32_t)(1000000.0f / hz); }
    void layer(int t, bool now, uint32_t& timer) {
        switch (t) {
        case T_RAINBOW: rainbow(now, timer); break;
        case T_BLINK: blink(now, timer); break;
        case T_LARSON: larson(now, timer); break;
        case T_RING: thrust_ring(now, timer); break;
        case T_INDICATOR: indicator(now, timer); break;
        case T_GPS: gps(now, timer); break;
        case T_BATTERY: battery(now, timer); break;
        case T_RSSI: rssi(now, timer); break;
        case T_WARNING: warning(now, timer); break;
        default: break;
        }
    }
    void warning(bool now, uint32_t& timer) {
        const Input& in = *in_;
        if (now) {
            warn_counter_ = (uint8_t)((warn_counter_ + 1) & 0x0F);
            if (warn_counter_ == 0) {
                warn_flags_ = 0;
                if (in.batt_meter && in.batt_state != 0) warn_flags_ |= 1u << W_LOW_BATTERY;
                if (in.failsafe) warn_flags_ |= 1u << W_FAILSAFE;
                if (!in.armed && in.arming_disabled) warn_flags_ |= 1u << W_ARMING_DISABLED;
                if (in.crashflip) warn_flags_ |= 1u << W_CRASHFLIP;
            }
            timer += hz_us(10);
        }
        bool have = false; Hsv wc;
        if (warn_flags_) {
            const bool on = (warn_counter_ % 2) == 0; const int id = warn_counter_ / 4;
            if (warn_flags_ & (1u << id)) {
                have = true;
                switch (id) {
                case W_ARMING_DISABLED: wc = on ? basic(GREEN) : basic(BLACK); break;
                case W_CRASHFLIP: wc = on ? basic(MAGENTA) : basic(BLACK); break;
                case W_LOW_BATTERY: wc = on ? basic(RED) : basic(BLACK); break;
                case W_FAILSAFE: wc = on ? basic(YELLOW) : basic(BLUE); break;
                default: have = false; break;
                }
            }
        } else if (in.beeper_on) { have = true; wc = basic(cfg.visual_beeper_color < COLOR_COUNT ? cfg.visual_beeper_color : (uint8_t)WHITE); }
        if (have) apply_mask((uint32_t)(1u << OL_WARNING) << kOverlayOffset, wc);
    }
    void battery(bool now, uint32_t& timer) {
        if (now) {
            switch (in_->batt_state) { case 0: batt_flash_ = true; timer += hz_us(1); break; case 1: batt_flash_ = !batt_flash_; timer += hz_us(2); break; default: batt_flash_ = !batt_flash_; timer += hz_us(8); break; }
        }
        if (!batt_flash_) apply_mask((uint32_t)FN_BATTERY << kFunctionOffset, sc(SC_BACKGROUND));
    }
    void rssi(bool now, uint32_t& timer) {
        if (now) {
            const int s = in_->rssi_percent;
            if (s > 50) { rssi_flash_ = true; timer += hz_us(1); } else if (s > 20) { rssi_flash_ = !rssi_flash_; timer += hz_us(2); } else { rssi_flash_ = !rssi_flash_; timer += hz_us(8); }
        }
        if (!rssi_flash_) apply_mask((uint32_t)FN_RSSI << kFunctionOffset, sc(SC_BACKGROUND));
    }
    void gps(bool now, uint32_t& timer) {
        const Input& in = *in_;
        if (now) {
            if (gps_pause_ > 0) gps_pause_--;
            else if (gps_flash_ >= in.gps_sats) { gps_flash_ = 0; gps_pause_ = 4; }
            else { gps_flash_++; gps_pause_ = 1; }
            timer += hz_us(2.5f);
        }
        Hsv c;
        if (in.gps_sats == 0 || !in.gps_sensor) c = sc(SC_GPSNOSATS);
        else { const bool on = gps_pause_ == 0; c = in.gps_fix ? (on ? sc(SC_GPSLOCKED) : sc(SC_BACKGROUND)) : (on ? sc(SC_GPSNOLOCK) : sc(SC_GPSNOSATS)); }
        apply_mask((uint32_t)FN_GPS << kFunctionOffset, c);
    }
    void indicator(bool now, uint32_t& timer) {
        const Input& in = *in_; const float db = 0.05f;
        if (now) {
            if (in.rx_signal) {
                const float defl = fabsf(in.roll) > fabsf(in.pitch) ? fabsf(in.roll) : fabsf(in.pitch);
                const int scale = (int)(45 * (defl - db) / (1.0f - db));
                timer += hz_us((float)(5 + scale));
                ind_flash_ = !ind_flash_;
            } else timer += hz_us(5);
        }
        if (!ind_flash_) return;
        int q = 0;
        if (in.roll > db) q |= Q_EAST; else if (in.roll < -db) q |= Q_WEST;
        if (in.pitch > db) q |= Q_NORTH; else if (in.pitch < -db) q |= Q_SOUTH;
        for (int i = 0; i < count_; i++) if (led_overlay(cfg.leds[i], OL_INDICATOR) && (quadrant(i) & q)) set(i, basic(ORANGE));
    }
    void thrust_ring(bool now, uint32_t& timer) {
        const Input& in = *in_; int ring_i = 0;
        if (now) {
            ring_phase_ = ring_phase_ > 0 ? ring_phase_ - 1 : (uint8_t)(ring_seq_ > 0 ? ring_seq_ - 1 : 0);
            const int thr = in.armed ? (int)scale_rangef(channel(3), -1.0f, 1.0f, 0.0f, 100.0f) : 0;
            timer += hz_us((float)(5 + (45 * thr) / 100));
        }
        for (int i = 0; i < count_; i++) {
            const uint32_t c = cfg.leds[i];
            if (led_function(c) != FN_THRUST_RING) continue;
            const bool on = in.armed ? (ring_seq_ > 0 && (ring_i + ring_phase_) % ring_seq_ < 2) : !(ring_i % 2);
            if (on) set(i, color(led_color(c)));
            ring_i++;
        }
    }
    void rainbow(bool now, uint32_t& timer) {
        if (now) { rainbow_offset_ += cfg.rainbow_freq; timer += hz_us(60); }
        int k = 0;
        for (int i = 0; i < count_; i++) {
            if (!led_overlay(cfg.leds[i], OL_RAINBOW)) continue;
            Hsv c; c.h = (uint16_t)((rainbow_offset_ / 60 + k * cfg.rainbow_delta) % (kHueMax + 1)); c.s = 0; c.v = 255; set(i, c); k++;
        }
    }
    int larson_brightness(int idx) const {
        const int off = idx - lar_index_;
        if (abs(off) > 1) return 8;
        if (off == 0) return lar_bright_;
        if (lar_dir_ == off) return lar_bright_ - 127;
        return 255 - lar_bright_;
    }
    void larson(bool now, uint32_t& timer) {
        if (now) {
            const int delta = 15;
            if (lar_bright_ > 255 - delta) { lar_bright_ = 127; if (lar_index_ >= larson_ || lar_index_ < 0) lar_dir_ = -lar_dir_; lar_index_ += lar_dir_; }
            else lar_bright_ += delta;
            timer += hz_us(60);
        }
        int k = 0;
        for (int i = 0; i < count_; i++) {
            if (!led_overlay(cfg.leds[i], OL_LARSON_SCANNER)) continue;
            Hsv c = hsv_[i]; c.v = (uint8_t)larson_brightness(k); set(i, c); k++;
        }
    }
    void blink(bool now, uint32_t& timer) {
        if (now) { blink_mask_ = (uint16_t)(blink_mask_ >> 1); if (blink_mask_ <= 1) blink_mask_ = 0x8005; timer += hz_us(10); }
        if (!(blink_mask_ & 1)) for (int i = 0; i < count_; i++) if (led_overlay(cfg.leds[i], OL_BLINK)) set(i, sc(SC_BLINKBACKGROUND));
    }

    // ---- applyStatusProfile: the layers whose timers are due, after the fixed ones (whole passes here)
    bool status_profile(uint32_t now) {
        uint32_t active = 0;
        for (int t = 0; t < T_COUNT; t++) {
            if (disabled_ & (1u << t)) continue;
            const int32_t delta = (int32_t)(now - timer_[t]);
            if (delta > 5 * 1000 * 1000) timer_[t] = now;                  // (the start, and a timer left behind: restart its interval)
            if ((int32_t)(now - timer_[t]) >= 0) active |= 1u << t;
        }
        if (!active) return false;
        fixed_layers();
        for (int t = 0; t < T_COUNT; t++) { if (disabled_ & (1u << t)) continue; layer(t, (active >> t) & 1, timer_[t]); }
        return true;
    }
    // ---- applySimpleProfile: RACE / BEACON, the whole strip one color (forced again every second)
    bool simple_profile(uint32_t now_us) {
        const Input& in = *in_; const uint32_t now_ms = now_us / 1000;
        Hsv cur;                                                              // black
        const Hsv beacon = basic(cfg.beacon_color < COLOR_COUNT ? cfg.beacon_color : (uint8_t)WHITE);
        const Hsv beeper = basic(cfg.visual_beeper_color < COLOR_COUNT ? cfg.visual_beeper_color : (uint8_t)WHITE);
        if (cfg.profile == PROFILE_RACE) {
            if (cfg.race_color != BLACK) cur = color(cfg.race_color);         // (black: Betaflight's VTX band color - no VTX here)
            if (in.beeper_switch || in.failsafe) { if (now_ms % 250 < 125) cur = beeper; }
            else if (cfg.visual_beeper && in.beeper_on) cur = beeper;
        } else if (cfg.profile == PROFILE_BEACON) {
            const uint32_t period = cfg.beacon_period_ms ? cfg.beacon_period_ms : 1, on = period * cfg.beacon_percent / 100;
            if (now_ms % period < on) cur = beacon;
        }
        const bool changed = cur.h != prev_.h || cur.s != prev_.s || cur.v != prev_.v || (int32_t)(now_us - color_update_us_) >= 0;
        if (!changed) return false;
        for (int i = 0; i < kMax; i++) hsv_[i] = cur;
        prev_ = cur; color_update_us_ = now_us + 1000000u;
        return true;
    }

    enum Warning : uint8_t { W_ARMING_DISABLED, W_LOW_BATTERY, W_FAILSAFE, W_CRASHFLIP };
    const Input* in_ = nullptr;
    Hsv hsv_[kMax], prev_;
    int count_ = 0, ring_ = 0, larson_ = 0, ring_seq_ = 0, bar_gps_ = 0, bar_batt_ = 0;
    int northY_ = 0, southY_ = 0, westX_ = 0, eastX_ = 0;
    uint32_t timer_[T_COUNT] = {}, disabled_ = 0, color_update_us_ = 0;
    uint8_t warn_counter_ = 0, warn_flags_ = 0, gps_pause_ = 0, gps_flash_ = 0, ring_phase_ = 0;
    bool batt_flash_ = false, rssi_flash_ = false, ind_flash_ = false;
    int rainbow_offset_ = 0, lar_bright_ = 0, lar_index_ = 0, lar_dir_ = 1;
    uint16_t blink_mask_ = 0;
};

} // namespace led
