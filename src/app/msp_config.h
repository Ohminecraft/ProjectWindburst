/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>
#include <string.h>
#include <math.h>
#include "msp_bridge.h"
#include "telemetry.h"
#include "../proto/wbp_schema.h"

// ============================================================================
// MSP for the configurators (Betaflight Configurator, the SpeedyBee app): the live values they show and the settings they edit,
// in Betaflight 4.5's layouts (msp.c, API 1.46), turned into this firmware's settings. msp::Bridge asks this first
// (Bridge::set_handler); what is not here it answers itself, or with '!' (Betaflight: not supported - the configurators go on).
//
//   live      MSP_RC 105, MSP_RAW_IMU 102, MSP_RAW_GPS 106, MSP_COMP_GPS 107, MSP_ATTITUDE 108, MSP_ALTITUDE 109, MSP_ANALOG 110
//   settings  MSP_NAME 10 / SET 11, MSP_MODE_RANGES 34 / SET 35, MSP_MODE_RANGES_EXTRA 238, MSP_RX_MAP 64 / SET 65,
//             MSP_RC_TUNING 111 / SET 204, MSP_PID 112 / SET 202, MSP_BOXNAMES 116, MSP_PIDNAMES 117, MSP_ACC_TRIM 240
//   actions   MSP_EEPROM_WRITE 250 (save), MSP_REBOOT 68 (after the answer is out: reboot_pending())
//
// Units: the settings already are Betaflight's (PID gains, "Actual" rates, mode ranges in microseconds); only the packing differs.
// Rates go out as rates_type ACTUAL (4); a SET_RC_TUNING with another rates type is refused (no conversion is guessed).
// PIDs written while the tuning sliders own an axis turn the sliders off (msp_config.h follows Betaflight Configurator: typing a
// gain leaves the sliders) - only when a gain really changes, so a tool that writes back what it read changes nothing.
// Writes: refused while armed, and when msp.writes is off. Every write goes through Port::commit(section) - the same steps as
// a SECTION_SET from the app (clamped, rules applied, applied live or "reboot required"); EEPROM_WRITE saves like the app's Save.
// Portable: unit-tested in test/test_msp_config.cpp.
// ============================================================================
namespace msp {

enum ConfigCmd : uint16_t {
    NAME = 10, SET_NAME = 11, MODE_RANGES = 34, SET_MODE_RANGE = 35, RX_MAP = 64, SET_RX_MAP = 65, REBOOT = 68,
    RAW_IMU = 102, RC = 105, RAW_GPS = 106, COMP_GPS = 107, ATTITUDE = 108, ALTITUDE = 109, ANALOG_DATA = 110,   // (MSP_ANALOG: "ANALOG" is an Arduino-ESP32 macro)
    RC_TUNING = 111, PID = 112,
    BOXNAMES = 116, PIDNAMES = 117, SET_PID = 202, SET_RC_TUNING = 204, MODE_RANGES_EXTRA = 238, ACC_TRIM = 240, EEPROM_WRITE = 250,
};
constexpr int kBfModeRanges = 20;                     // MAX_MODE_ACTIVATION_CONDITION_COUNT
constexpr uint8_t kRatesActual = 4;                   // Betaflight RATES_TYPE_ACTUAL

// What the handler needs from the firmware for writes (LinkHost + wbp::Server on the board, a fake in the tests)
class ConfigPort {
public:
    virtual ~ConfigPort() = default;
    virtual bool armed() = 0;
    virtual void commit(uint8_t section) = 0;         // a section changed in RAM: clamp, rules, apply live or mark "reboot required"
    virtual bool save() = 0;                          // write the settings to flash
};

// Betaflight's box names for the permanent ids this firmware has (msp_box.c)
inline const char* bf_box_name(uint8_t id) {
    switch (id) {
    case 0: return "ARM"; case 1: return "ANGLE"; case 3: return "ALTHOLD"; case 4: return "ANTI GRAVITY"; case 11: return "POS HOLD";
    case 13: return "BEEPER"; case 17: return "CALIB"; case 26: return "BLACKBOX"; case 27: return "FAILSAFE"; case 28: return "AIR MODE";
    case 31: return "BLACKBOX ERASE"; case 46: return "GPS RESCUE"; case 47: return "ACRO TRAINER";
    default: return "?";
    }
}

class Config {
public:
    Config(wbp::Settings& s, ConfigPort& port) : s_(s), port_(port) {}
    // The telemetry the live answers come from (the link task's snapshot); set before each request.
    void set_telemetry(const app::Telemetry* t) { t_ = t; }
    // Bridge::Handler
    static bool handler(void* ctx, const Frame& f, uint8_t* p, uint16_t& n, bool& error) { return ((Config*)ctx)->handle(f, p, n, error); }

    bool handle(const Frame& f, uint8_t* p, uint16_t& n, bool& err) {
        n = 0; err = false;
        switch (f.cmd) {
        // ---- live values
        case RC: { const int k = 16; for (int i = 0; i < k; i++) u16(p, n, t_ ? t_->rc[i] : 1500); return true; }
        case ATTITUDE: {
            if (!t_) { err = true; return true; }
            int yaw = (int)lroundf(t_->yaw); yaw = ((yaw % 360) + 360) % 360;
            u16(p, n, (uint16_t)(int16_t)lroundf(t_->roll * 10.0f)); u16(p, n, (uint16_t)(int16_t)lroundf(t_->pitch * 10.0f)); u16(p, n, (uint16_t)(int16_t)yaw);
            return true; }
        case ALTITUDE: { const float z = t_ ? t_->z : 0.0f, vz = t_ ? t_->vz : 0.0f; u32(p, n, (uint32_t)(int32_t)lroundf(z * 100.0f)); u16(p, n, (uint16_t)(int16_t)clampf(vz * 100.0f, -32767, 32767)); return true; }
        case ANALOG_DATA: {
            const float v = t_ ? t_->batt_v : 0.0f, a = t_ ? t_->batt_amps : 0.0f, mah = t_ ? t_->batt_mah : 0.0f;
            p[n++] = (uint8_t)clampf(v * 10.0f + 0.5f, 0, 255); u16(p, n, (uint16_t)clampf(mah, 0, 65535));
            u16(p, n, (uint16_t)(t_ ? (uint32_t)t_->rssi * 1023u / 100u : 0u));                                   // (rssi 0..100 % -> 0..1023)
            u16(p, n, (uint16_t)(int16_t)clampf(a * 100.0f, -32767, 32767)); u16(p, n, (uint16_t)clampf(v * 100.0f + 0.5f, 0, 65535));
            return true; }
        case RAW_IMU: {                                                    // the configurator divides: acc / 512 (g), gyro x 4 / 16.4 (deg/s), mag / 1090 (gauss)
            for (int k = 0; k < 3; k++) u16(p, n, (uint16_t)(int16_t)clampf((t_ ? t_->acc[k] : 0.0f) * 512.0f, -32767, 32767));
            for (int k = 0; k < 3; k++) u16(p, n, (uint16_t)(int16_t)clampf((t_ ? t_->gyro_f[k] : 0.0f) * 4.1f, -32767, 32767));
            for (int k = 0; k < 3; k++) u16(p, n, (uint16_t)(int16_t)clampf((t_ ? t_->mag[k] : 0.0f) * 1090.0f, -32767, 32767));
            return true; }
        case RAW_GPS: {
            p[n++] = t_ && t_->gps_fix ? 2 : 0; p[n++] = t_ ? t_->gps_sats : 0;
            u32(p, n, (uint32_t)(t_ ? t_->lat : 0)); u32(p, n, (uint32_t)(t_ ? t_->lon : 0));
            u16(p, n, (uint16_t)(int16_t)clampf(t_ ? t_->alt_cm / 100.0f : 0.0f, -32767, 32767));                     // metres
            u16(p, n, t_ ? t_->gs_cms : 0); u16(p, n, t_ ? t_->course_ddeg : 0); u16(p, n, t_ ? t_->hdop : 0);
            return true; }
        case COMP_GPS: {
            const uint32_t d = t_ ? t_->home_dist_m : 0; u16(p, n, (uint16_t)(d > 65535u ? 65535u : d));
            int dir = t_ ? (t_->home_dir_ddeg + 5) / 10 : 0; if (dir > 180) dir -= 360;
            u16(p, n, (uint16_t)(int16_t)dir); p[n++] = t_ && t_->gps_fix ? 1 : 0;
            return true; }
        // ---- settings, read
        case NAME: { const size_t l = strnlen((const char*)s_.osd.name, sizeof(s_.osd.name)); memcpy(p, s_.osd.name, l); n = (uint16_t)l; return true; }
        case MODE_RANGES:
            for (int i = 0; i < kBfModeRanges; i++) {
                uint8_t id = 0, aux = 0, lo = 0, hi = 0;
                if (i < 16 && used(i)) { id = kBfBox[s_.modes.mode[i]]; aux = s_.modes.aux[i]; lo = step(s_.modes.lo[i]); hi = step(s_.modes.hi[i]); }
                p[n++] = id; p[n++] = aux; p[n++] = lo; p[n++] = hi;
            }
            return true;
        case MODE_RANGES_EXTRA:                                            // per range: box id, logic (0 OR), linked to (0 none)
            p[n++] = kBfModeRanges;
            for (int i = 0; i < kBfModeRanges; i++) { p[n++] = i < 16 && used(i) ? kBfBox[s_.modes.mode[i]] : 0; p[n++] = 0; p[n++] = 0; }
            return true;
        case RX_MAP: {                                                     // the input channel of roll, pitch, yaw, throttle, AUX1 .. AUX4
            p[n++] = s_.receiver.ch_roll; p[n++] = s_.receiver.ch_pitch; p[n++] = s_.receiver.ch_yaw; p[n++] = s_.receiver.ch_throttle;
            for (int k = 0; k < 4; k++) p[n++] = (uint8_t)(s_.receiver.ch_aux0 + k);
            return true; }
        case RC_TUNING: {                                                  // Betaflight 4.5's layout, rates type ACTUAL
            const wbp::S_rates& r = s_.rates;
            p[n++] = c10(r.center_sens[0]); p[n++] = r.expo[0]; p[n++] = c10(r.max_rate[0]); p[n++] = c10(r.max_rate[1]); p[n++] = c10(r.max_rate[2]);
            p[n++] = 0;                                                    // (the old TPA rate: in MSP_PID_ADVANCED since 4.3)
            p[n++] = r.throttle_mid; p[n++] = r.throttle_expo;
            u16(p, n, (uint16_t)(1000u + s_.pid.tpa_breakpoint * 10u));
            p[n++] = r.expo[2]; p[n++] = c10(r.center_sens[2]); p[n++] = c10(r.center_sens[1]); p[n++] = r.expo[1];
            p[n++] = r.throttle_limit < 100 ? 1 : 0; p[n++] = r.throttle_limit;                                     // throttle limit: SCALE / OFF, percent
            for (int k = 0; k < 3; k++) u16(p, n, 1998);                                                            // rate limits
            p[n++] = kRatesActual;
            return true; }
        case PID: {                                                        // ROLL, PITCH, YAW, LEVEL, MAG: P I D each
            for (int a = 0; a < 3; a++) { p[n++] = s_.pid.p[a]; p[n++] = s_.pid.i[a]; p[n++] = s_.pid.d[a]; }
            p[n++] = s_.pid.angle_strength; p[n++] = 0; p[n++] = 0; p[n++] = 0; p[n++] = 0; p[n++] = 0;
            return true; }
        case PIDNAMES: { const char* t = "ROLL;PITCH;YAW;LEVEL;MAG;"; n = (uint16_t)strlen(t); memcpy(p, t, n); return true; }
        case BOXNAMES: {                                                   // the names, in MSP_BOXIDS' order
            Info in; in.modes_set = modes_set(); uint8_t ids[kModeBoxes]; const int k = Bridge::boxes(in, ids);
            for (int i = 0; i < k; i++) { const char* nm = bf_box_name(ids[i]); const uint16_t l = (uint16_t)strlen(nm); if (n + l + 1 > kMaxReply) break; memcpy(p + n, nm, l); n += l; p[n++] = ';'; }
            return true; }
        case ACC_TRIM: u16(p, n, (uint16_t)s_.calibration.acc_trim_pitch); u16(p, n, (uint16_t)s_.calibration.acc_trim_roll); return true;
        // ---- settings, write
        case SET_NAME: case SET_MODE_RANGE: case SET_RX_MAP: case SET_RC_TUNING: case SET_PID: case EEPROM_WRITE: case REBOOT:
            if (!s_.msp.writes || port_.armed()) { err = true; refused_++; return true; }
            err = !write(f);
            if (err) refused_++;
            return true;
        default: return false;
        }
    }
    bool reboot_pending() { const bool r = reboot_; reboot_ = false; return r; }   // MSP_REBOOT was answered: restart now
    uint32_t refused() const { return refused_; }

private:
    bool write(const Frame& f) {
        const uint8_t* q = f.payload; const uint16_t len = f.len;
        switch (f.cmd) {
        case SET_NAME: {
            if (len > sizeof(s_.osd.name)) return false;
            memset(s_.osd.name, 0, sizeof(s_.osd.name));
            for (uint16_t i = 0; i < len; i++) { const uint8_t c = q[i]; s_.osd.name[i] = c >= 32 && c < 127 ? c : '?'; }
            port_.commit(wbp::SEC_OSD); return true; }
        case SET_MODE_RANGE: {
            if (len < 5 || q[0] >= kBfModeRanges) return false;
            const int i = q[0];
            if (i >= 16) return q[3] == q[4];                              // (ranges 17 .. 20 do not exist here: only "unused" is accepted)
            if (q[3] >= q[4]) { s_.modes.lo[i] = 0; s_.modes.hi[i] = 0; s_.modes.aux[i] = 0; s_.modes.mode[i] = 0; port_.commit(wbp::SEC_MODES); return true; }   // start = end: the range is cleared
            int mode = -1; for (int m = 0; m < kModeBoxes; m++) if (kBfBox[m] == q[1]) mode = m;
            if (mode < 0 || q[2] > 15 || q[4] > 48) return false;
            s_.modes.mode[i] = (uint8_t)mode; s_.modes.aux[i] = q[2]; s_.modes.lo[i] = (uint16_t)(900 + q[3] * 25); s_.modes.hi[i] = (uint16_t)(900 + q[4] * 25);
            port_.commit(wbp::SEC_MODES); return true; }
        case SET_RX_MAP: {
            if (len < 4) return false;
            for (int k = 0; k < 4; k++) if (q[k] > 15) return false;
            s_.receiver.ch_roll = q[0]; s_.receiver.ch_pitch = q[1]; s_.receiver.ch_yaw = q[2]; s_.receiver.ch_throttle = q[3];
            port_.commit(wbp::SEC_RECEIVER); return true; }
        case SET_RC_TUNING: {
            if (len < 23 || q[22] != kRatesActual) return false;          // (only "Actual" rates: no guessing at a conversion)
            wbp::S_rates& r = s_.rates;
            r.center_sens[0] = (uint16_t)(q[0] * 10u); r.expo[0] = q[1]; r.max_rate[0] = (uint16_t)(q[2] * 10u); r.max_rate[1] = (uint16_t)(q[3] * 10u); r.max_rate[2] = (uint16_t)(q[4] * 10u);
            r.throttle_mid = q[6]; r.throttle_expo = q[7];
            r.expo[2] = q[10]; r.center_sens[2] = (uint16_t)(q[11] * 10u); r.center_sens[1] = (uint16_t)(q[12] * 10u); r.expo[1] = q[13];
            r.throttle_limit = q[14] == 0 ? 100 : q[15];
            port_.commit(wbp::SEC_RATES); return true; }
        case SET_PID: {
            if (len < 12) return false;
            bool changed = false;
            for (int a = 0; a < 3; a++) {
                changed |= s_.pid.p[a] != q[a * 3] || s_.pid.i[a] != q[a * 3 + 1] || s_.pid.d[a] != q[a * 3 + 2];
                s_.pid.p[a] = q[a * 3]; s_.pid.i[a] = q[a * 3 + 1]; s_.pid.d[a] = q[a * 3 + 2];
            }
            s_.pid.angle_strength = q[9];
            if (changed) s_.pid.slider_mode = wbp::SLIDER_MODE_OFF;       // typed gains: the sliders no longer own them
            port_.commit(wbp::SEC_PID); return true; }
        case EEPROM_WRITE: return port_.save();
        case REBOOT: reboot_ = true; return true;
        }
        return false;
    }
    bool used(int i) const { return s_.modes.hi[i] > s_.modes.lo[i] && s_.modes.mode[i] < kModeBoxes && kBfBox[s_.modes.mode[i]] != 255; }
    uint32_t modes_set() const { uint32_t m = 0; for (int i = 0; i < 16; i++) if (s_.modes.hi[i] > s_.modes.lo[i] && s_.modes.mode[i] < 32) m |= 1u << s_.modes.mode[i]; return m; }
    static uint8_t step(uint16_t us) { const int v = ((int)us - 900) / 25; return (uint8_t)(v < 0 ? 0 : v > 48 ? 48 : v); }
    static uint8_t c10(uint16_t v) { const unsigned x = (v + 5u) / 10u; return (uint8_t)(x > 255u ? 255u : x); }
    static float clampf(float v, float lo, float hi) { return v < lo ? lo : v > hi ? hi : v; }
    static void u16(uint8_t* p, uint16_t& n, uint16_t v) { p[n++] = (uint8_t)v; p[n++] = (uint8_t)(v >> 8); }
    static void u32(uint8_t* p, uint16_t& n, uint32_t v) { u16(p, n, (uint16_t)v); u16(p, n, (uint16_t)(v >> 16)); }

    wbp::Settings& s_; ConfigPort& port_; const app::Telemetry* t_ = nullptr; bool reboot_ = false; uint32_t refused_ = 0;
};

} // namespace msp
