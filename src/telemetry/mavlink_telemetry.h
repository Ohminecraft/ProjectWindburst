/*
 * This file is part of Windburst Project
 */

#pragma once

#include "mavlink.h"

// ============================================================================
// MAVLink telemetry: what the craft is doing, as a ground station (Mission Planner, QGroundControl, a radio with an ELRS backpack)
// wants it. Send only. The sensor thread fills a State from app::Telemetry (FlightApp::mav_in) and calls poll() about once a
// millisecond; poll() appends the frames that are due and fit to a buffer, the caller writes it to the UART / the WiFi ring.
//
//   HEARTBEAT            1 Hz always
//   ATTITUDE             rates.attitude
//   GLOBAL_POSITION_INT  rates.position   (GPS_RAW_INT as well, when a GPS is fitted)
//   VFR_HUD              rates.hud
//   SYS_STATUS           rates.status     (BATTERY_STATUS as well, when a battery is measured)
//   RC_CHANNELS          rates.rc
//   STATUSTEXT           queued texts (boot report, why the craft cannot arm), at most one chunk every 100 ms
// A rate of 0 switches that group off. A frame that does not fit in the room the UART has left waits for the next call: nothing
// is cut, nothing ever blocks the sensor thread.
// Portable.
// ============================================================================
namespace mav {

struct Rates { uint8_t attitude = 10, position = 5, hud = 4, status = 2, rc = 2; };        // Hz, 0 = off

// What is sent, in MAVLink's units and conventions (FRD body, NED world; roll right +, pitch nose up +, yaw clockwise from north)
struct State {
    uint32_t time_ms = 0;
    // HEARTBEAT
    uint8_t vehicle_type = MAV_TYPE_QUADROTOR;
    bool armed = false, failsafe = false, calibrating = false;
    bool angle = false, stabilize = false, althold = false, poshold = false, rescue = false, mission = false;      // mission: it flies the waypoint mission
    // attitude, rad and rad/s
    float roll = 0, pitch = 0, yaw = 0, p = 0, q = 0, r = 0;
    // position
    bool have_gps = false; uint8_t gps_fix = 0, sats = 0;             // gps_fix: the receiver's (0 none, 1 dead reckoning, 2 2D, 3 3D, 4 GNSS + DR)
    int32_t lat = 0, lon = 0, alt_msl_mm = 0; uint16_t hdop_x100 = 0, gs_cms = 0, cog_cdeg = 0; uint32_t hacc_mm = 0;
    bool alt_valid = false; float rel_alt_m = 0, climb_ms = 0;        // the height estimate (above the take-off point)
    bool vel_valid = false; float vn = 0, ve = 0;                     // m/s
    float heading_deg = 0;                                            // 0 .. 360
    float throttle = 0;                                               // 0 .. 1
    // SYS_STATUS, BATTERY_STATUS
    uint32_t sensors_present = 0, sensors_enabled = 0, sensors_health = 0; uint8_t load_pct = 0;
    bool batt_present = false, have_current = false; float volts = 0, amps = 0, mah = 0; uint8_t cells = 0; int8_t remaining = -1;
    uint16_t rc_errors = 0;                                           // receiver frames lost / damaged in the last second
    // RC_CHANNELS
    bool rc_link = false; uint16_t rc[16] = {}; uint8_t rssi_pct = 0;
};

// HEARTBEAT fields for a state (ArduCopter mode numbers, like Betaflight's MAVLink telemetry)
inline Heartbeat heartbeat_of(const State& s) {
    Heartbeat h; h.type = s.vehicle_type; h.autopilot = MAV_AUTOPILOT_GENERIC;
    h.base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
    if (s.armed) h.base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    if (s.angle || s.stabilize || s.althold || s.poshold || s.rescue || s.mission) h.base_mode |= MAV_MODE_FLAG_STABILIZE_ENABLED;
    h.custom_mode = s.failsafe ? (s.rescue ? COPTER_RTL : COPTER_LAND) : s.rescue ? COPTER_RTL : s.mission ? COPTER_AUTO : s.poshold ? COPTER_POSHOLD : s.althold ? COPTER_ALT_HOLD
                  : (s.angle || s.stabilize) ? COPTER_STABILIZE : COPTER_ACRO;
    h.system_status = s.failsafe ? MAV_STATE_CRITICAL : s.calibrating && !s.armed ? MAV_STATE_CALIBRATING : s.armed ? MAV_STATE_ACTIVE : MAV_STATE_STANDBY;
    return h;
}
inline uint8_t gps_fix_of(const State& s) {
    if (!s.have_gps) return GPS_FIX_NO_GPS;
    return s.gps_fix >= 3 ? GPS_FIX_3D : s.gps_fix == 2 ? GPS_FIX_2D : GPS_FIX_NONE;
}
inline int16_t sat16(float v) { return v >= 32767.0f ? (int16_t)32767 : v <= -32768.0f ? (int16_t)-32768 : (int16_t)(v < 0 ? v - 0.5f : v + 0.5f); }
inline int32_t sat32(float v) { return !(v == v) ? 0 : v >= 2.0e9f ? (int32_t)2000000000 : v <= -2.0e9f ? (int32_t)-2000000000 : (int32_t)(v < 0 ? v - 0.5f : v + 0.5f); }
inline uint16_t cdeg360(float deg) { float d = deg; while (d < 0.0f) d += 360.0f; while (d >= 360.0f) d -= 360.0f; const uint32_t c = (uint32_t)(d * 100.0f + 0.5f); return (uint16_t)(c >= 36000u ? 0u : c); }

class Scheduler {
public:
    static constexpr int kTexts = 6;                  // queued texts (a full queue drops the oldest)
    static constexpr size_t kTextLen = 150;           // longer texts are cut (3 STATUSTEXT chunks)
    static constexpr uint32_t kTextGapMs = 100;

    void configure(const Rates& r, uint8_t sysid) { rates_ = r; enc_.sysid = sysid ? sysid : 1; reset(); }
    void reset() { for (uint32_t& t : next_) t = 0; started_ = false; }
    Encoder& encoder() { return enc_; }

    // queues a text (STATUSTEXT, several chunks when longer than 50 characters)
    void text(uint8_t severity, const char* s) {
        if (!s || !*s) return;
        if (n_text_ == kTexts) { head_ = (head_ + 1) % kTexts; n_text_--; dropped_texts_++; }
        Text& t = texts_[(head_ + n_text_) % kTexts];
        size_t n = 0; for (; s[n] && n < kTextLen; n++) t.s[n] = s[n];
        t.s[n] = 0; t.len = (uint16_t)n; t.severity = severity; t.chunk = 0; t.id = n > 50 ? next_id() : 0;
        n_text_++;
    }
    int queued_texts() const { return n_text_; }
    uint32_t dropped_texts() const { return dropped_texts_; }

    // Appends the frames due at now_ms that fit into cap bytes; returns how many bytes it wrote.
    size_t poll(uint32_t now_ms, const State& s, uint8_t* out, size_t cap) {
        if (!started_) { for (uint32_t& t : next_) t = now_ms; started_ = true; }
        size_t n = 0; uint8_t f[kMaxFrame];
        auto emit = [&](int k, uint32_t period_ms, size_t len) {                 // the frame in f is sent when it fits; then the next is due
            if (n + len > cap) return false;
            memcpy(out + n, f, len); n += len;
            next_[k] = (uint32_t)(now_ms - next_[k]) < period_ms ? next_[k] + period_ms : now_ms + period_ms;     // (fell far behind: no burst)
            return true;
        };
        auto due = [&](int k, uint8_t hz) { return hz && (int32_t)(now_ms - next_[k]) >= 0; };
        auto undo = [&](size_t len) { if (n + len > cap) enc_.set_seq((uint8_t)(enc_.seq() - 1)); };      // (a frame that did not fit: its number is used again)
        if (due(K_HEARTBEAT, 1)) { const size_t l = pack(enc_, heartbeat_of(s), f); undo(l); emit(K_HEARTBEAT, 1000, l); }
        if (due(K_ATTITUDE, rates_.attitude)) {
            Attitude a; a.time_boot_ms = s.time_ms; a.roll = s.roll; a.pitch = s.pitch; a.yaw = s.yaw; a.rollspeed = s.p; a.pitchspeed = s.q; a.yawspeed = s.r;
            const size_t l = pack(enc_, a, f); undo(l); emit(K_ATTITUDE, 1000u / rates_.attitude, l);
        }
        if (due(K_POSITION, rates_.position)) {
            GlobalPositionInt g; g.time_boot_ms = s.time_ms; g.hdg = cdeg360(s.heading_deg);
            if (s.have_gps && s.gps_fix >= 2) { g.lat = s.lat; g.lon = s.lon; g.alt = s.alt_msl_mm; }
            g.relative_alt = s.alt_valid ? sat32(s.rel_alt_m * 1000.0f) : 0;
            if (s.vel_valid) { g.vx = sat16(s.vn * 100.0f); g.vy = sat16(s.ve * 100.0f); }
            g.vz = s.alt_valid ? sat16(-s.climb_ms * 100.0f) : 0;
            const size_t l = pack(enc_, g, f); undo(l); emit(K_POSITION, 1000u / rates_.position, l);
        }
        if (s.have_gps && due(K_GPS, rates_.position)) {                         // (its own slot at the same rate: it waits for room by itself)
            GpsRawInt r; r.time_usec = (uint64_t)s.time_ms * 1000u; r.fix_type = gps_fix_of(s); r.lat = s.lat; r.lon = s.lon; r.alt = s.alt_msl_mm;
            r.eph = s.hdop_x100 ? s.hdop_x100 : 0xFFFF; r.vel = s.gs_cms; r.cog = s.cog_cdeg; r.satellites_visible = s.sats; r.h_acc = s.hacc_mm;
            const size_t l = pack(enc_, r, f); undo(l); emit(K_GPS, 1000u / rates_.position, l);
        }
        if (due(K_HUD, rates_.hud)) {
            VfrHud v; v.groundspeed = s.have_gps && s.gps_fix >= 2 ? s.gs_cms * 0.01f : 0.0f; v.airspeed = v.groundspeed;     // (no airspeed sensor)
            v.heading = (int16_t)(cdeg360(s.heading_deg) / 100u); v.throttle = (uint16_t)(s.throttle <= 0.0f ? 0 : s.throttle >= 1.0f ? 100 : (int)(s.throttle * 100.0f + 0.5f));
            v.alt = s.have_gps && s.gps_fix >= 3 ? s.alt_msl_mm * 0.001f : s.rel_alt_m; v.climb = s.alt_valid ? s.climb_ms : 0.0f;
            const size_t l = pack(enc_, v, f); undo(l); emit(K_HUD, 1000u / rates_.hud, l);
        }
        if (due(K_STATUS, rates_.status)) {
            SysStatus st; st.present = s.sensors_present; st.enabled = s.sensors_enabled; st.health = s.sensors_health;
            st.load = (uint16_t)(s.load_pct > 100 ? 1000 : s.load_pct * 10); st.drop_rate_comm = 0; st.errors_comm = s.rc_errors;
            if (s.batt_present) { st.voltage_battery = (uint16_t)(s.volts * 1000.0f + 0.5f); st.battery_remaining = s.remaining; }
            if (s.have_current) st.current_battery = sat16(s.amps * 100.0f);
            const size_t l = pack(enc_, st, f); undo(l); emit(K_STATUS, 1000u / rates_.status, l);
        }
        if (s.batt_present && due(K_BATTERY, rates_.status)) {
            BatteryStatus b; b.battery_remaining = s.remaining;
            if (s.cells) { const uint16_t cell = (uint16_t)(s.volts / s.cells * 1000.0f + 0.5f); for (int i = 0; i < s.cells && i < 10; i++) b.voltages[i] = cell; }
            else b.voltages[0] = (uint16_t)(s.volts * 1000.0f + 0.5f);
            if (s.have_current) { b.current_battery = sat16(s.amps * 100.0f); b.current_consumed = sat32(s.mah); }
            const size_t l = pack(enc_, b, f); undo(l); emit(K_BATTERY, 1000u / rates_.status, l);
        }
        if (due(K_RC, rates_.rc)) {
            RcChannels c; c.time_boot_ms = s.time_ms; c.chancount = s.rc_link ? 16 : 0; c.rssi = s.rc_link ? (uint8_t)(s.rssi_pct >= 100 ? 254 : s.rssi_pct * 254 / 100) : 0;
            for (int i = 0; i < 18; i++) c.chan[i] = i < 16 && s.rc_link ? s.rc[i] : 0xFFFF;
            const size_t l = pack(enc_, c, f); undo(l); emit(K_RC, 1000u / rates_.rc, l);
        }
        if (n_text_ && (int32_t)(now_ms - next_[K_TEXT]) >= 0) {
            Text& t = texts_[head_];
            StatusText m; m.severity = t.severity; m.id = t.id; m.chunk_seq = t.chunk;
            const size_t from = (size_t)t.chunk * 50u;
            for (size_t i = 0; i < 50 && from + i < t.len; i++) m.text[i] = t.s[from + i];
            const size_t l = pack(enc_, m, f);
            if (n + l <= cap) {
                memcpy(out + n, f, l); n += l; next_[K_TEXT] = now_ms + kTextGapMs;
                // the last chunk is the one that ends inside its 50 characters (a text of exactly 100 needs an empty third one: the end mark)
                if (t.id == 0 || from + 50 > t.len) { head_ = (head_ + 1) % kTexts; n_text_--; } else t.chunk++;
            } else enc_.set_seq((uint8_t)(enc_.seq() - 1));
        }
        return n;
    }

private:
    enum { K_HEARTBEAT, K_ATTITUDE, K_POSITION, K_GPS, K_HUD, K_STATUS, K_BATTERY, K_RC, K_TEXT, K_COUNT };
    struct Text { char s[kTextLen + 1] = {}; uint16_t len = 0, id = 0; uint8_t severity = 0, chunk = 0; };
    uint16_t next_id() { if (++text_id_ == 0) text_id_ = 1; return text_id_; }
    Encoder enc_; Rates rates_;
    uint32_t next_[K_COUNT] = {}; bool started_ = false;
    Text texts_[kTexts]; int head_ = 0, n_text_ = 0; uint16_t text_id_ = 0; uint32_t dropped_texts_ = 0;
};

} // namespace mav
