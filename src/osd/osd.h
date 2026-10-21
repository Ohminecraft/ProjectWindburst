/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "../app/msp_bridge.h"

// ============================================================================
// On-screen display for digital FPV goggles (DJI O3 / O4, Walksnail, HDZero) over MSP DisplayPort, as Betaflight does it
// (io/displayport_msp.c, osd/osd_elements.c):
//
//   the flight controller pushes MSP v1 "replies" ($M>) of command 182 (MSP_DISPLAYPORT) to the video transmitter:
//     [0] heartbeat   [2] clear screen   [3] write string: row, column, attribute, characters (30 at most)   [4] draw screen
//   one screen = clear, every element, draw; a heartbeat every second keeps the goggles from showing "disconnected".
//   The goggles draw with their own font: Betaflight's (drivers/osd_symbols.h), so the same character codes are used here.
//
//   The video transmitter asks as well (MSP requests on the same wire): MSP_SET_OSD_CANVAS (HDZero, Walksnail: the size of its
//   grid), MSP_NAME, and the "who are you / armed?" requests that msp::Bridge answers like Betaflight 4.5 (MSP_STATUS: the goggles
//   start recording on arming).
//
// Elements: Betaflight's osd_*_pos encoding: column bits 0-5, row bits 6-10, shown = bit 11 (OSD_PROFILE_1_FLAG).
// Portable (test/test_osd.cpp). The sensor thread fills a State, poll() hands it the bytes to write, never more than the UART takes.
// ============================================================================
namespace osd {

// ---- Betaflight's font (drivers/osd_symbols.h)
enum : uint8_t {
    SYM_RSSI = 0x01, SYM_THR = 0x04, SYM_OVER_HOME = 0x05, SYM_VOLT = 0x06, SYM_MAH = 0x07, SYM_M = 0x0C, SYM_C = 0x0E,
    SYM_HOMEFLAG = 0x11, SYM_ROLL = 0x14, SYM_PITCH = 0x15, SYM_SAT_L = 0x1E, SYM_SAT_R = 0x1F, SYM_HYPHEN = 0x2D,
    SYM_ARROW_SOUTH = 0x60, SYM_SPEED = 0x70, SYM_AH_CENTER_LINE = 0x72, SYM_AH_CENTER = 0x73, SYM_AH_CENTER_LINE_RIGHT = 0x74,
    SYM_ARROW_SMALL_UP = 0x75, SYM_ARROW_SMALL_DOWN = 0x76, SYM_LINK_QUALITY = 0x7B, SYM_ALTITUDE = 0x7F, SYM_AH_BAR9_0 = 0x80,
    SYM_BATT_EMPTY = 0x96, SYM_MAIN_BATT = 0x97, SYM_AMP = 0x9A, SYM_ON_M = 0x9B, SYM_FLY_M = 0x9C, SYM_KPH = 0x9E, SYM_MPS = 0x9F,
};

// The elements, in the order of osd.pos[] (append only: the settings store them by index)
enum Element : uint8_t {
    E_RSSI, E_BATT, E_CELL, E_CURRENT, E_MAH, E_MODE, E_NAME, E_FLY_TIME, E_ON_TIME, E_SATS, E_SPEED, E_HOME_DIR, E_HOME_DIST,
    E_ALT, E_VARIO, E_THROTTLE, E_CROSSHAIRS, E_HORIZON, E_WARNINGS, E_HEADING, E_PITCH, E_ROLL, E_ESC_TEMP, E_DISARMED, E_COUNT
};
constexpr uint16_t kVisible = 0x0800;
inline uint8_t pos_col(uint16_t p) { return (uint8_t)(p & 0x3F); }
inline uint8_t pos_row(uint16_t p) { return (uint8_t)((p >> 6) & 0x1F); }
inline uint16_t make_pos(uint8_t col, uint8_t row, bool shown) { return (uint16_t)((col & 0x3F) | ((row & 0x1F) << 6) | (shown ? kVisible : 0)); }

// MSP DisplayPort
constexpr uint8_t kMspDisplayPort = 182, kMspSetCanvas = 188, kMspName = 10;
enum : uint8_t { DP_HEARTBEAT = 0, DP_RELEASE = 1, DP_CLEAR = 2, DP_WRITE = 3, DP_DRAW = 4 };
constexpr uint8_t kAttrBlink = 0x40;                  // DISPLAYPORT_MSP_ATTR_BLINK: the goggles blink it
constexpr int kMaxString = 30;                        // MSP_OSD_MAX_STRING_LENGTH

enum Mode : uint8_t { MODE_ACRO, MODE_ANGLE, MODE_STAB, MODE_ALTHOLD, MODE_POSHOLD, MODE_RESCUE };

// What is shown, in the units it is shown in (metric)
struct State {
    bool armed = false, failsafe = false;
    uint32_t fly_s = 0, on_s = 0;                     // armed time so far, time since power on
    uint8_t mode = MODE_ACRO;
    bool rc_link = false; uint8_t rssi_pct = 0;       // link quality (ELRS / CRSF) or RSSI, %
    bool batt = false; float volts = 0; uint8_t cells = 0; uint8_t batt_level = 0;     // batt_level: 0 ok, 1 warning, 2 critical
    float cell_min = 3.3f, cell_max = 4.3f;           // the battery section's cell range (the battery symbol's fill)
    bool have_current = false; float amps = 0, mah = 0;
    bool gps = false; uint8_t sats = 0, fix = 0; float speed_ms = 0;
    bool home = false; float home_dist_m = 0, home_dir_deg = 0;      // home direction: true bearing, degrees
    bool alt_valid = false; float alt_m = 0, vz_ms = 0;
    float throttle = 0;                               // 0..1
    float roll_deg = 0, pitch_deg = 0, heading_deg = 0;   // Betaflight's conventions: roll right +, pitch nose DOWN +, heading 0..360
    uint8_t esc_temp_c = 0;                           // the hottest ESC, 0 = none reported
    char warning[24] = {};                            // the most important warning, upper case; empty = none
};

// Where the elements go
struct Layout {
    uint16_t pos[E_COUNT] = {};
    char name[17] = {};
    uint8_t cols = 53, rows = 20;
};

// ---- the elements as text (Betaflight's look). Returns false when the element shows nothing now.
inline uint8_t battery_symbol(const State& s) {        // osdGetBatterySymbol: 7 fill levels over the cell range, "BAT-" when critical
    if (s.batt_level >= 2) return SYM_MAIN_BATT;
    const int cell = s.cells ? (int)lroundf(s.volts * 100.0f / s.cells) : 0;               // centivolts, as Betaflight's scaleRange()
    const int lo = (int)lroundf(s.cell_min * 100.0f), hi = (int)lroundf(s.cell_max * 100.0f);
    int k = hi > lo ? (cell - lo) * 8 / (hi - lo) : 0;
    k = k < 0 ? 0 : k > 6 ? 6 : k;
    return (uint8_t)(SYM_BATT_EMPTY - k);
}
inline uint8_t direction_symbol(float deg) {           // osdGetDirectionSymbolFromHeading: 16 arrows, up = ahead
    int h = (int)lroundf(deg);
    h = ((h % 360) + 360) % 360;
    int d = (h * 16 + 180) / 360 % 16;                 // 0 = up, 4 = right ...
    d = (16 - d + 8) % 16;                             // the font: down = 0, right = 4, up = 8, left = 12
    return (uint8_t)(SYM_ARROW_SOUTH + d);
}
inline const char* mode_name(const State& s) {
    if (s.failsafe) return "!FS!";
    static const char* const n[] = {"ACRO", "ANGL", "STAB", "ALTH", "POSH", "RESC"};
    return s.mode < 6 ? n[s.mode] : "ACRO";
}
inline void mmss(char* b, size_t cap, uint8_t sym, uint32_t sec) { snprintf(b, cap, "%c%02u:%02u", sym, (unsigned)(sec / 60u % 100u), (unsigned)(sec % 60u)); }

inline bool element_text(Element e, const State& s, const Layout& l, char* b, size_t cap, uint8_t& attr) {
    attr = 0; b[0] = 0;
    switch (e) {
    case E_RSSI: snprintf(b, cap, "%c%u", SYM_LINK_QUALITY, s.rc_link ? (unsigned)(s.rssi_pct > 99 ? 99 : s.rssi_pct) : 0u); if (!s.rc_link) attr = kAttrBlink; return true;
    case E_BATT: if (!s.batt) return false;
        snprintf(b, cap, s.volts >= 10.0f ? "%c%.1f%c" : "%c%.2f%c", battery_symbol(s), (double)s.volts, SYM_VOLT); if (s.batt_level >= 2) attr = kAttrBlink; return true;
    case E_CELL: if (!s.batt || !s.cells) return false;
        snprintf(b, cap, "%c%.2f%c", battery_symbol(s), (double)(s.volts / s.cells), SYM_VOLT); if (s.batt_level >= 2) attr = kAttrBlink; return true;
    case E_CURRENT: if (!s.have_current) return false; snprintf(b, cap, "%.2f%c", (double)fabsf(s.amps), SYM_AMP); return true;
    case E_MAH: if (!s.have_current) return false; snprintf(b, cap, "%u%c", (unsigned)(s.mah > 0 ? s.mah + 0.5f : 0.0f), SYM_MAH); return true;
    case E_MODE: snprintf(b, cap, "%s", mode_name(s)); return true;
    case E_NAME: if (!l.name[0]) return false; snprintf(b, cap, "%s", l.name); return true;
    case E_FLY_TIME: mmss(b, cap, SYM_FLY_M, s.fly_s); return true;
    case E_ON_TIME: mmss(b, cap, SYM_ON_M, s.on_s); return true;
    case E_SATS: if (!s.gps) return false; snprintf(b, cap, "%c%c%u", SYM_SAT_L, SYM_SAT_R, (unsigned)s.sats); if (s.fix < 3) attr = kAttrBlink; return true;
    case E_SPEED: if (!s.gps) return false; snprintf(b, cap, "%c%u%c", SYM_SPEED, (unsigned)lroundf(s.speed_ms > 0 ? s.speed_ms * 3.6f : 0.0f), SYM_KPH); return true;
    case E_HOME_DIR: if (!s.gps) return false;
        b[0] = !(s.home && s.fix >= 2) ? (char)SYM_HYPHEN : s.home_dist_m < 1.0f ? (char)SYM_OVER_HOME : (char)direction_symbol(s.home_dir_deg - s.heading_deg); b[1] = 0; return true;
    case E_HOME_DIST: if (!s.gps) return false;
        if (!(s.home && s.fix >= 2)) snprintf(b, cap, "%c%c%c%c", SYM_HOMEFLAG, SYM_HYPHEN, SYM_HYPHEN, SYM_M);
        else snprintf(b, cap, "%c%u%c", SYM_HOMEFLAG, (unsigned)lroundf(s.home_dist_m), SYM_M);
        return true;
    case E_ALT: if (!s.alt_valid) { snprintf(b, cap, "%c%c%c%c", SYM_ALTITUDE, SYM_HYPHEN, SYM_HYPHEN, SYM_M); return true; }
        snprintf(b, cap, "%c%.1f%c", SYM_ALTITUDE, (double)s.alt_m, SYM_M); return true;
    case E_VARIO: if (!s.alt_valid) return false;
        snprintf(b, cap, "%c%.1f%c", s.vz_ms >= 0 ? SYM_ARROW_SMALL_UP : SYM_ARROW_SMALL_DOWN, (double)fabsf(s.vz_ms), SYM_MPS); return true;
    case E_THROTTLE: snprintf(b, cap, "%c%u", SYM_THR, (unsigned)lroundf((s.throttle < 0 ? 0 : s.throttle > 1 ? 1 : s.throttle) * 100.0f)); return true;
    case E_CROSSHAIRS: b[0] = (char)SYM_AH_CENTER_LINE; b[1] = (char)SYM_AH_CENTER; b[2] = (char)SYM_AH_CENTER_LINE_RIGHT; b[3] = 0; return true;
    case E_WARNINGS: if (!s.warning[0]) return false; snprintf(b, cap, "%s", s.warning); attr = kAttrBlink; return true;
    case E_HEADING: { int h = (int)lroundf(s.heading_deg); h = ((h % 360) + 360) % 360; snprintf(b, cap, "%c%03d", direction_symbol(0), h); return true; }
    case E_PITCH: case E_ROLL: { const float a = e == E_PITCH ? s.pitch_deg : s.roll_deg; snprintf(b, cap, "%c%c%02u", e == E_PITCH ? SYM_PITCH : SYM_ROLL, a < 0 ? '-' : ' ', (unsigned)lroundf(fabsf(a))); return true; }
    case E_ESC_TEMP: if (!s.esc_temp_c) return false; snprintf(b, cap, "%u%c", (unsigned)s.esc_temp_c, SYM_C); return true;
    case E_DISARMED: if (s.armed) return false; snprintf(b, cap, "DISARMED"); return true;
    default: return false;            // (E_HORIZON: drawn character by character, below)
    }
}

// The artificial horizon (osdElementArtificialHorizon, ahMaxPitch 20, ahMaxRoll 40): 9 columns around the element's column, each
// one character of 9 levels on the rows pos .. pos + 9. Calls put(column offset, row offset, character) for each column drawn.
template <class Put> void horizon(const State& s, Put put) {
    const int max_pitch = 200, max_roll = 400;                       // tenths of a degree
    int roll = (int)lroundf(s.roll_deg * 10.0f), pitch = (int)lroundf(s.pitch_deg * 10.0f);
    roll = roll < -max_roll ? -max_roll : roll > max_roll ? max_roll : roll;
    pitch = pitch < -max_pitch ? -max_pitch : pitch > max_pitch ? max_pitch : pitch;
    pitch = pitch * 25 / max_pitch - 41;                             // 41 = 4 * 9 + 5
    for (int x = -4; x <= 4; x++) {
        const int y = (-roll * x) / 64 - pitch;
        if (y >= 0 && y <= 81) put(x, y / 9, (uint8_t)(SYM_AH_BAR9_0 + y % 9));
    }
}

// Every element that is shown and fits the canvas: write(row, column, attribute, text).
template <class Write> void render(const State& s, const Layout& l, Write write) {
    char b[kMaxString + 1];
    for (int e = 0; e < E_COUNT; e++) {
        const uint16_t p = l.pos[e];
        if (!(p & kVisible)) continue;
        const int col = pos_col(p), row = pos_row(p);
        if (e == E_HORIZON) {
            horizon(s, [&](int dx, int dy, uint8_t c) { const int x = col + dx, y = row + dy; if (x >= 0 && x < l.cols && y < l.rows) { char t[2] = {(char)c, 0}; write((uint8_t)y, (uint8_t)x, 0, t); } });
            continue;
        }
        uint8_t attr = 0;
        if (col >= l.cols || row >= l.rows || !element_text((Element)e, s, l, b, sizeof(b), attr)) continue;
        size_t n = strlen(b); if (n > (size_t)(l.cols - col)) { n = (size_t)(l.cols - col); b[n] = 0; }   // (cut at the right edge of the canvas)
        write((uint8_t)row, (uint8_t)col, attr, b);
    }
}

// ---- the wire: screens out, the video transmitter's requests in
class DisplayPort {
public:
    static constexpr size_t kBuf = 2048;
    enum Canvas : uint8_t { CANVAS_AUTO, CANVAS_HD, CANVAS_HDZERO, CANVAS_SD };     // == schema enum osd_canvas

    void configure(uint8_t canvas, uint8_t rate_hz) {
        canvas_ = canvas; period_ms_ = 1000u / (rate_hz ? rate_hz : 10);
        if (canvas == CANVAS_HDZERO) { cols_ = 50; rows_ = 18; } else if (canvas == CANVAS_SD) { cols_ = 30; rows_ = 16; } else { cols_ = 53; rows_ = 20; }
        n_ = sent_ = 0; started_ = false;
    }
    uint8_t cols() const { return cols_; }
    uint8_t rows() const { return rows_; }

    // One byte from the video transmitter; a reply (if any) is queued behind the screen being sent. `name`: the craft name.
    void feed(uint8_t c, const msp::Info& info, const char* name) {
        msp::Frame f;
        if (!parser_.feed(c, f)) return;
        requests_++;
        uint8_t r[80]; size_t rn = 0;
        if (f.cmd == kMspSetCanvas) {                                          // HDZero / Walksnail tell their grid: used in AUTO
            if (f.len >= 2 && canvas_ == CANVAS_AUTO && f.payload[0] >= 20 && f.payload[0] <= 63 && f.payload[1] >= 10 && f.payload[1] <= 31) { cols_ = f.payload[0]; rows_ = f.payload[1]; canvas_set_ = true; }
            rn = msp::encode(f.version, f.cmd, nullptr, 0, false, r, sizeof(r));
        } else if (f.cmd == kMspName) {
            rn = msp::encode(f.version, f.cmd, (const uint8_t*)name, (uint16_t)strnlen(name, 16), false, r, sizeof(r));
        } else rn = bridge_.reply(f, info, r, sizeof(r));
        append(r, rn);
    }

    // The bytes to write now, at most `room` (the UART's free space): a screen every period, cut nowhere but at the room's end.
    size_t poll(uint32_t now_ms, const State& s, const Layout& layout, uint8_t* out, size_t room) {
        if (sent_ >= n_) { n_ = sent_ = 0; }
        if (n_ == 0 && (!started_ || (int32_t)(now_ms - next_ms_) >= 0)) {
            started_ = true; next_ms_ = now_ms + period_ms_;
            if (!hb_ms_set_ || (int32_t)(now_ms - hb_ms_) >= 1000) { hb_ms_ = now_ms; hb_ms_set_ = true; push_dp(DP_HEARTBEAT, nullptr, 0); }
            Layout l = layout; l.cols = cols_; l.rows = rows_;
            push_dp(DP_CLEAR, nullptr, 0);
            render(s, l, [&](uint8_t row, uint8_t col, uint8_t attr, const char* t) {
                uint8_t p[kMaxString + 3]; const size_t k = strlen(t);
                p[0] = row; p[1] = col; p[2] = attr; memcpy(p + 3, t, k);
                push_dp(DP_WRITE, p, k + 3);
            });
            push_dp(DP_DRAW, nullptr, 0);
            screens_++;
        }
        size_t k = n_ - sent_; if (k > room) k = room;
        memcpy(out, buf_ + sent_, k); sent_ += k;
        return k;
    }
    uint32_t screens() const { return screens_; }
    uint32_t requests() const { return requests_; }
    uint32_t overflows() const { return overflows_; }
    bool canvas_from_vtx() const { return canvas_set_; }

private:
    void append(const uint8_t* d, size_t k) {
        if (!k) return;
        if (sent_ && sent_ == n_) { n_ = sent_ = 0; }
        if (n_ + k > kBuf) { overflows_++; return; }
        memcpy(buf_ + n_, d, k); n_ += k;
    }
    void push_dp(uint8_t sub, const uint8_t* p, size_t k) {
        uint8_t pay[kMaxString + 4]; pay[0] = sub; if (k) memcpy(pay + 1, p, k);
        uint8_t f[kMaxString + 10]; append(f, msp::encode(1, kMspDisplayPort, pay, (uint16_t)(k + 1), false, f, sizeof(f)));
    }
    msp::Parser parser_; msp::Bridge bridge_;
    uint8_t buf_[kBuf] = {}; size_t n_ = 0, sent_ = 0;
    uint8_t canvas_ = CANVAS_AUTO, cols_ = 53, rows_ = 20; bool canvas_set_ = false;
    uint32_t period_ms_ = 100, next_ms_ = 0, hb_ms_ = 0; bool started_ = false, hb_ms_set_ = false;
    uint32_t screens_ = 0, requests_ = 0, overflows_ = 0;
};

}  // namespace osd
