/*
 * This file is part of Windburst Project
 */

#pragma once
#include "../config/fast_code.h"

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>

// ============================================================================
// Blackbox log format (".wblog"), portable.
//
// A log is a sequence of WBP frames (src/proto/wbp_codec.h: sync, length, type, sequence, payload, CRC16) - the same bytes
// whether they are written to flash or sent to the app, so one reader handles both and a damaged spot costs one frame.
//
//   BB_HEADER frames   text, cut into pieces (sequence 0, 1, 2 ...). Sequence 0 starts a new session and begins with "WBLOG 1".
//                      The text lists the fields of the two record kinds with type, scale and unit, so a reader needs no
//                      knowledge of the firmware version, plus the complete settings as hex.
//   BB_DATA frames     records, one after the other, each starting with its kind:
//                        'F' fast record   [t_us u32][enabled fast fields]      every rate_div-th control cycle
//                        'S' slow record   [t_us u32][enabled slow fields]      10 times a second
//                        'E' event         [t_us u32][code u8][n u8][n bytes]   arm, disarm, failsafe stage, mode switches ...
//                      The frame sequence number counts up; a gap means frames were lost (serial) or damaged (flash).
//
// Everything is little-endian. t_us is micros() of the control cycle (wraps after 71.6 minutes; the reader unwraps it).
// Field values: stored integer * scale = value in the unit given in the header.
// ============================================================================
namespace bb {

constexpr uint8_t kMsgHeader = 0x60, kMsgData = 0x61, kMsgBlob = 0x62;      // WBP message types used by the blackbox
constexpr int kFormatVersion = 1;
constexpr int kMaxMotors = 8;
constexpr uint16_t kFramePayload = 240;          // a frame (payload + 8) always fits one 256-byte flash page
constexpr uint16_t kSlowHz = 10;

enum Type : uint8_t { T_U8, T_I16, T_U16, T_I32, T_U32 };
inline uint8_t type_size(uint8_t t) { return t == T_U8 ? 1 : (t == T_I16 || t == T_U16) ? 2 : 4; }
inline const char* type_name(uint8_t t) { static const char* n[] = {"u8", "i16", "u16", "i32", "u32"}; return t <= T_U32 ? n[t] : "?"; }

// Field groups = bits of the settings field blackbox.fields (schema enum bb_field)
enum Group : uint8_t { G_GYRO, G_GYRO_RAW, G_SETPOINT, G_PID, G_RC, G_MOTORS, G_ATTITUDE, G_ACC, G_RPM, G_TIMING, G_ALT, G_BATTERY, G_GPS, G_MAG, G_NAV, G_COUNT, G_ALWAYS = 255 };

// ---- fast record: what changes every control cycle
enum FastId : uint8_t {
    F_GYRO, F_GYRO_RAW, F_SETPOINT, F_PID_P, F_PID_I, F_PID_D, F_PID_F, F_STICK, F_THROTTLE, F_MOTOR, F_ROLL, F_PITCH, F_YAW, F_ACC, F_RPM, F_LOOP_US, F_GAP_US, F_T_LOCK, F_T_IMU, F_T_CORE, F_T_EST, F_T_MOTOR, F_COUNT
};
struct FieldDef { uint8_t id; const char* name; uint8_t count; uint8_t type; double scale; float inv; const char* unit; uint8_t group; bool per_motor; };
#define BB_FIELD(id, name, count, type, scale, unit, group, per_motor) {id, name, count, type, scale, (float)(1.0 / (scale)), unit, group, per_motor}

constexpr FieldDef kFast[] = {
    BB_FIELD(F_GYRO, "gyro", 3, T_I16, 0.1, "deg/s", G_GYRO, false),                 // filtered, pilot convention (roll right +, nose down +, yaw right +)
    BB_FIELD(F_GYRO_RAW, "gyro_raw", 3, T_I16, 0.1, "deg/s", G_GYRO_RAW, false),     // before the filters
    BB_FIELD(F_SETPOINT, "setpoint", 3, T_I16, 0.1, "deg/s", G_SETPOINT, false),     // rate setpoint
    BB_FIELD(F_PID_P, "pid_p", 3, T_I16, 0.1, "", G_PID, false),                     // PID terms in PID sum units (Betaflight's: 1000 = the whole motor range;
    BB_FIELD(F_PID_I, "pid_i", 3, T_I16, 0.1, "", G_PID, false),                     //  the sum is limited to +-500, yaw +-400). Until 0.6.1 the scale was 0.0001,
    BB_FIELD(F_PID_D, "pid_d", 3, T_I16, 0.1, "", G_PID, false),                     //  which clipped every term at +-3.28.
    BB_FIELD(F_PID_F, "pid_f", 3, T_I16, 0.1, "", G_PID, false),
    BB_FIELD(F_STICK, "stick", 4, T_I16, 0.001, "", G_RC, false),                    // roll, pitch, yaw -1..1, throttle stick 0..1
    BB_FIELD(F_THROTTLE, "throttle", 1, T_U16, 0.0001, "", G_RC, false),             // throttle after AltHold / failsafe, 0..1
    BB_FIELD(F_MOTOR, "motor", kMaxMotors, T_U16, 0.0001, "", G_MOTORS, true),       // what goes to each ESC, 0..1 (logical motor order)
    BB_FIELD(F_ROLL, "roll", 1, T_I16, 0.01, "deg", G_ATTITUDE, false),
    BB_FIELD(F_PITCH, "pitch", 1, T_I16, 0.01, "deg", G_ATTITUDE, false),
    BB_FIELD(F_YAW, "yaw", 1, T_U16, 0.01, "deg", G_ATTITUDE, false),                // heading 0 .. 360 (0.8.8; before: -180 .. 180)
    BB_FIELD(F_ACC, "acc", 3, T_I16, 0.001, "g", G_ACC, false),
    BB_FIELD(F_RPM, "rpm", kMaxMotors, T_U16, 10.0, "rpm", G_RPM, true),
    BB_FIELD(F_LOOP_US, "loop_us", 1, T_U16, 1.0, "us", G_TIMING, false),            // execution time of this control cycle
    BB_FIELD(F_GAP_US, "gap_us", 1, T_U16, 1.0, "us", G_TIMING, false),              // longest start-to-start time of the cycles since the previous record
    // where the time of a cycle goes: the LONGEST of each stage in the cycles since the previous record (one byte each, 4 us steps, up to 1020 us)
    BB_FIELD(F_T_LOCK, "t_lock", 1, T_U8, 4.0, "us", G_TIMING, false),               // waiting for the sensor thread to release the lock
    BB_FIELD(F_T_IMU, "t_imu", 1, T_U8, 4.0, "us", G_TIMING, false),                 // reading the IMU
    BB_FIELD(F_T_CORE, "t_core", 1, T_U8, 4.0, "us", G_TIMING, false),               // filters, PID, mixer - in the cycles that did not step the estimators
    BB_FIELD(F_T_EST, "t_est", 1, T_U8, 4.0, "us", G_TIMING, false),                 // the same in the cycles that did (0: none in this interval)
    BB_FIELD(F_T_MOTOR, "t_motor", 1, T_U8, 4.0, "us", G_TIMING, false),             // motor output and ESC replies
};
constexpr int kFastDefs = sizeof(kFast) / sizeof(kFast[0]);

struct FastSample {                         // filled by the flight loop for one control cycle; pointers stay valid during log_fast() only
    uint32_t t_us = 0;
    const float* gyro = nullptr; const float* gyro_raw = nullptr; const float* setpoint = nullptr;
    const float* pid_p = nullptr; const float* pid_i = nullptr; const float* pid_d = nullptr; const float* pid_f = nullptr;
    const float* stick = nullptr; float throttle = 0;
    const float* motor = nullptr; const float* rpm = nullptr; const float* acc = nullptr;
    float roll = 0, pitch = 0, yaw = 0;
    uint32_t loop_us = 0, gap_us = 0;
    uint16_t t_lock_us = 0, t_imu_us = 0, t_core_us = 0, t_est_us = 0, t_motor_us = 0;
};
WB_FAST inline float fast_value(const FastSample& s, uint8_t id, int i) {
    switch (id) {
    case F_GYRO: return s.gyro[i];
    case F_GYRO_RAW: return s.gyro_raw[i];
    case F_SETPOINT: return s.setpoint[i];
    case F_PID_P: return s.pid_p[i];
    case F_PID_I: return s.pid_i[i];
    case F_PID_D: return s.pid_d[i];
    case F_PID_F: return s.pid_f[i];
    case F_STICK: return s.stick[i];
    case F_THROTTLE: return s.throttle;
    case F_MOTOR: return s.motor[i];
    case F_ROLL: return s.roll;
    case F_PITCH: return s.pitch;
    case F_YAW: return s.yaw;
    case F_ACC: return s.acc[i];
    case F_RPM: return s.rpm[i];
    case F_LOOP_US: return (float)s.loop_us;
    case F_GAP_US: return (float)s.gap_us;
    case F_T_LOCK: return (float)s.t_lock_us;
    case F_T_IMU: return (float)s.t_imu_us;
    case F_T_CORE: return (float)s.t_core_us;
    case F_T_EST: return (float)s.t_est_us;
    case F_T_MOTOR: return (float)s.t_motor_us;
    default: return 0.0f;
    }
}

// ---- slow record: state and slow sensors
enum SlowId : uint8_t {
    S_ARMED, S_FAILSAFE, S_AH_STATE, S_BATT_STATE, S_MODES, S_BLOCKERS, S_HEALTH, S_LOOP_MAX_US, S_GAP_MAX_US, S_LOOP_HZ, S_RSSI, S_RC_RATE,
    S_BARO_ALT, S_EST_Z, S_EST_VZ, S_RANGE, S_VBAT, S_CELL, S_CURRENT, S_MAH, S_GPS_FIX, S_GPS_SATS, S_LAT, S_LON, S_GPS_ALT, S_GPS_SPEED, S_GPS_COURSE, S_HDOP,
    S_POS_N, S_POS_E, S_VEL_N, S_VEL_E, S_HOME_DIST, S_MAG,
    S_EST_VALID, S_NAV_FLAGS, S_FLOW_QUALITY, S_FLOW_REJECT, S_FLOW_FWD, S_FLOW_RIGHT, S_NAV_ROLL, S_NAV_PITCH, S_NAV_TARGET_N, S_NAV_TARGET_E, S_RESCUE_PHASE,
    S_EKF_ROLL, S_EKF_PITCH, S_EKF_YAW, S_EKF_PN, S_EKF_PE, S_EKF_VN, S_EKF_VE, S_EKF_Z, S_EKF_YAW_SIGMA, S_EKF_TEST_VEL, S_EKF_TEST_MAG, S_EKF_FLAGS, S_COUNT
};
constexpr FieldDef kSlow[] = {
    BB_FIELD(S_ARMED, "armed", 1, T_U8, 1.0, "", G_ALWAYS, false),
    BB_FIELD(S_FAILSAFE, "failsafe", 1, T_U8, 1.0, "", G_ALWAYS, false),             // schema enum failsafe
    BB_FIELD(S_AH_STATE, "ah_state", 1, T_U8, 1.0, "", G_ALWAYS, false),             // schema enum ah_state
    BB_FIELD(S_BATT_STATE, "batt_state", 1, T_U8, 1.0, "", G_ALWAYS, false),         // schema enum batt_state
    BB_FIELD(S_MODES, "modes", 1, T_U32, 1.0, "", G_ALWAYS, false),                  // bit per schema enum mode
    BB_FIELD(S_BLOCKERS, "blockers", 1, T_U32, 1.0, "", G_ALWAYS, false),
    BB_FIELD(S_HEALTH, "health", 1, T_U16, 1.0, "", G_ALWAYS, false),
    BB_FIELD(S_LOOP_MAX_US, "loop_max_us", 1, T_U16, 1.0, "us", G_ALWAYS, false),
    BB_FIELD(S_GAP_MAX_US, "gap_max_us", 1, T_U16, 1.0, "us", G_ALWAYS, false),
    BB_FIELD(S_LOOP_HZ, "loop_hz", 1, T_U16, 1.0, "Hz", G_ALWAYS, false),
    BB_FIELD(S_RSSI, "rssi", 1, T_U8, 1.0, "%", G_ALWAYS, false),
    BB_FIELD(S_RC_RATE, "rc_rate", 1, T_U16, 1.0, "Hz", G_ALWAYS, false),
    BB_FIELD(S_BARO_ALT, "baro_alt", 1, T_I32, 0.01, "m", G_ALT, false),
    BB_FIELD(S_EST_Z, "est_z", 1, T_I32, 0.01, "m", G_ALT, false),
    BB_FIELD(S_EST_VZ, "est_vz", 1, T_I16, 0.01, "m/s", G_ALT, false),
    BB_FIELD(S_RANGE, "range", 1, T_U16, 0.001, "m", G_ALT, false),
    BB_FIELD(S_VBAT, "vbat", 1, T_U16, 0.001, "V", G_BATTERY, false),
    BB_FIELD(S_CELL, "cell", 1, T_U16, 0.001, "V", G_BATTERY, false),
    BB_FIELD(S_CURRENT, "current", 1, T_I16, 0.01, "A", G_BATTERY, false),            // 0 without a current source
    BB_FIELD(S_MAH, "mah", 1, T_U16, 1.0, "mAh", G_BATTERY, false),
    BB_FIELD(S_GPS_FIX, "gps_fix", 1, T_U8, 1.0, "", G_GPS, false),
    BB_FIELD(S_GPS_SATS, "gps_sats", 1, T_U8, 1.0, "", G_GPS, false),
    BB_FIELD(S_LAT, "lat", 1, T_I32, 1e-7, "deg", G_GPS, false),
    BB_FIELD(S_LON, "lon", 1, T_I32, 1e-7, "deg", G_GPS, false),
    BB_FIELD(S_GPS_ALT, "gps_alt", 1, T_I32, 0.01, "m", G_GPS, false),
    BB_FIELD(S_GPS_SPEED, "gps_speed", 1, T_U16, 0.01, "m/s", G_GPS, false),
    BB_FIELD(S_GPS_COURSE, "gps_course", 1, T_U16, 0.1, "deg", G_GPS, false),
    BB_FIELD(S_HDOP, "hdop", 1, T_U16, 0.01, "", G_GPS, false),
    BB_FIELD(S_HOME_DIST, "home_dist", 1, T_U16, 1.0, "m", G_GPS, false),
    BB_FIELD(S_MAG, "mag", 3, T_I16, 0.001, "gauss", G_MAG, false),
    // navigation: the position estimate (from GPS and / or optical flow) and what Position Hold / GPS Rescue do with it
    BB_FIELD(S_POS_N, "pos_n", 1, T_I32, 0.01, "m", G_NAV, false),
    BB_FIELD(S_POS_E, "pos_e", 1, T_I32, 0.01, "m", G_NAV, false),
    BB_FIELD(S_VEL_N, "vel_n", 1, T_I16, 0.01, "m/s", G_NAV, false),
    BB_FIELD(S_VEL_E, "vel_e", 1, T_I16, 0.01, "m/s", G_NAV, false),
    BB_FIELD(S_EST_VALID, "est_valid", 1, T_U8, 1.0, "", G_NAV, false),               // schema bits est_valid
    BB_FIELD(S_NAV_FLAGS, "nav_flags", 1, T_U8, 1.0, "", G_NAV, false),               // schema bits nav_flags
    BB_FIELD(S_FLOW_QUALITY, "flow_quality", 1, T_U8, 1.0, "%", G_NAV, false),
    BB_FIELD(S_FLOW_REJECT, "flow_reject", 1, T_U8, 1.0, "", G_NAV, false),           // schema enum flow_reject
    BB_FIELD(S_FLOW_FWD, "flow_fwd", 1, T_I16, 0.01, "m/s", G_NAV, false),
    BB_FIELD(S_FLOW_RIGHT, "flow_right", 1, T_I16, 0.01, "m/s", G_NAV, false),
    BB_FIELD(S_NAV_ROLL, "nav_roll", 1, T_I16, 0.1, "deg", G_NAV, false),             // the lean the autopilot asks for (0 while the pilot flies)
    BB_FIELD(S_NAV_PITCH, "nav_pitch", 1, T_I16, 0.1, "deg", G_NAV, false),
    BB_FIELD(S_NAV_TARGET_N, "nav_target_n", 1, T_I32, 0.01, "m", G_NAV, false),
    BB_FIELD(S_NAV_TARGET_E, "nav_target_e", 1, T_I32, 0.01, "m", G_NAV, false),
    BB_FIELD(S_RESCUE_PHASE, "rescue_phase", 1, T_U8, 1.0, "", G_NAV, false),         // schema enum rescue_phase
    // the EKF (0.9.0, nav.ekf SHADOW / NAV): its own attitude (pilot convention, like roll / pitch / yaw), position, velocity, height
    BB_FIELD(S_EKF_ROLL, "ekf_roll", 1, T_I16, 0.01, "deg", G_NAV, false),
    BB_FIELD(S_EKF_PITCH, "ekf_pitch", 1, T_I16, 0.01, "deg", G_NAV, false),
    BB_FIELD(S_EKF_YAW, "ekf_yaw", 1, T_U16, 0.01, "deg", G_NAV, false),               // heading 0 .. 360
    BB_FIELD(S_EKF_PN, "ekf_pn", 1, T_I32, 0.01, "m", G_NAV, false),
    BB_FIELD(S_EKF_PE, "ekf_pe", 1, T_I32, 0.01, "m", G_NAV, false),
    BB_FIELD(S_EKF_VN, "ekf_vn", 1, T_I16, 0.01, "m/s", G_NAV, false),
    BB_FIELD(S_EKF_VE, "ekf_ve", 1, T_I16, 0.01, "m/s", G_NAV, false),
    BB_FIELD(S_EKF_Z, "ekf_z", 1, T_I32, 0.01, "m", G_NAV, false),
    BB_FIELD(S_EKF_YAW_SIGMA, "ekf_yaw_sigma", 1, T_U16, 0.01, "deg", G_NAV, false),
    BB_FIELD(S_EKF_TEST_VEL, "ekf_test_vel", 1, T_U8, 0.1, "", G_NAV, false),       // largest GPS velocity innovation / sigma of the last fix
    BB_FIELD(S_EKF_TEST_MAG, "ekf_test_mag", 1, T_U8, 0.1, "", G_NAV, false),
    BB_FIELD(S_EKF_FLAGS, "ekf_flags", 1, T_U8, 1.0, "", G_NAV, false),             // schema bits ekf_flags
};
constexpr int kSlowDefs = sizeof(kSlow) / sizeof(kSlow[0]);

// ---- events
enum Event : uint8_t { EV_ARM = 1, EV_DISARM = 2, EV_FAILSAFE = 3, EV_MODES = 4, EV_DROPPED = 5, EV_BATTERY = 6, EV_END = 7, EV_AH_STATE = 8 };
constexpr const char* kEventNames = "1:ARM 2:DISARM 3:FAILSAFE 4:MODES 5:DROPPED 6:BATTERY 7:END 8:AH_STATE";
enum EndReason : uint8_t { END_TRIGGER_OFF = 0, END_FULL = 1, END_STREAM_STOPPED = 2, END_APP_TOOK_OVER = 3 };

// ---- what is recorded: built once when a session starts (the "plan"), then read by the flight loop
struct Plan {
    uint32_t groups = 0;                          // bit per Group
    uint8_t motors = 4;
    uint16_t fast_size = 0, slow_size = 0;        // record body sizes including t_us
    WB_FAST bool has(const FieldDef& f) const { return f.group == G_ALWAYS || ((groups >> f.group) & 1u); }
    int count(const FieldDef& f) const { return f.per_motor ? motors : f.count; }
    void build(uint32_t group_bits, uint8_t motor_count) {
        groups = group_bits; motors = motor_count > kMaxMotors ? (uint8_t)kMaxMotors : motor_count;
        fast_size = 4; for (const FieldDef& f : kFast) if (has(f)) fast_size = (uint16_t)(fast_size + count(f) * type_size(f.type));
        slow_size = 4; for (const FieldDef& f : kSlow) if (has(f)) slow_size = (uint16_t)(slow_size + count(f) * type_size(f.type));
    }
};

// value / scale, rounded and saturated to the field's integer type; NaN / infinity -> 0
WB_FAST inline void put_int(uint8_t*& p, uint8_t type, double x) {
    if (!(x == x) || x > 1e18 || x < -1e18) x = 0;
    double lo = 0, hi = 0;
    switch (type) { case T_U8: hi = 255; break; case T_I16: lo = -32768; hi = 32767; break; case T_U16: hi = 65535; break; case T_I32: lo = -2147483648.0; hi = 2147483647.0; break; default: hi = 4294967295.0; break; }
    x = x < 0 ? x - 0.5 : x + 0.5;                 // round half away from zero
    if (x < lo) x = lo; else if (x > hi) x = hi;
    if (type == T_U32) { const uint32_t v = (uint32_t)x; memcpy(p, &v, 4); p += 4; return; }
    const int32_t v = (int32_t)x;
    const uint8_t n = type_size(type); memcpy(p, &v, n); p += n;     // little-endian host (ESP32, PC)
}
WB_FAST inline void put_u32(uint8_t*& p, uint32_t v) { memcpy(p, &v, 4); p += 4; }
// the flight-loop version: single precision only (no software double maths at 500 Hz), 8 / 16 bit fields
WB_FAST inline void put_fast(uint8_t*& p, uint8_t type, float x) {
    if (!(x == x)) x = 0;
    if (type == T_U16) { const uint16_t v = x <= 0.0f ? (uint16_t)0 : x >= 65535.0f ? (uint16_t)65535 : (uint16_t)(x + 0.5f); memcpy(p, &v, 2); p += 2; }
    else if (type == T_I16) { const int16_t v = x <= -32768.0f ? (int16_t)-32768 : x >= 32767.0f ? (int16_t)32767 : (int16_t)(x < 0 ? x - 0.5f : x + 0.5f); memcpy(p, &v, 2); p += 2; }
    else if (type == T_U8) { *p++ = x <= 0.0f ? (uint8_t)0 : x >= 255.0f ? (uint8_t)255 : (uint8_t)(x + 0.5f); }
    else put_int(p, type, (double)x);
}

// Fast record body ('F' is added by the recorder). Returns the number of bytes written (plan.fast_size).
WB_FAST inline uint16_t pack_fast(const Plan& plan, const FastSample& s, uint8_t* out) {
    uint8_t* p = out; put_u32(p, s.t_us);
    for (const FieldDef& f : kFast) {
        if (!plan.has(f)) continue;
        const int n = plan.count(f);
        for (int i = 0; i < n; i++) put_fast(p, f.type, fast_value(s, f.id, i) * f.inv);
    }
    return (uint16_t)(p - out);
}

// ---- header text. Appends to `out` (capacity cap), returns the new length. Lines end with '\n'.
struct TextOut {
    char* buf; size_t cap; size_t n = 0; bool overflow = false;
    TextOut(char* b, size_t c) : buf(b), cap(c) {}
    void add(const char* fmt, ...) __attribute__((format(printf, 2, 3))) {
        if (overflow) return;
        va_list ap; va_start(ap, fmt); const int k = vsnprintf(buf + n, cap - n, fmt, ap); va_end(ap);
        if (k < 0 || (size_t)k >= cap - n) { overflow = true; return; }
        n += (size_t)k;
    }
    void hex(const uint8_t* d, size_t len) { static const char* x = "0123456789abcdef"; if (overflow || n + 2 * len + 1 >= cap) { overflow = true; return; } for (size_t i = 0; i < len; i++) { buf[n++] = x[d[i] >> 4]; buf[n++] = x[d[i] & 15]; } buf[n] = 0; }
};
inline void describe_fields(TextOut& t, const char* kind, const Plan& plan, const FieldDef* defs, int n_defs) {
    t.add("%s t:u32:1e-06:s", kind);
    for (int d = 0; d < n_defs; d++) {
        const FieldDef& f = defs[d];
        if (!plan.has(f)) continue;
        const int n = plan.count(f);
        for (int i = 0; i < n; i++) {
            if (f.count > 1) t.add(" %s[%d]:%s:%g:%s", f.name, i, type_name(f.type), (double)f.scale, f.unit);
            else t.add(" %s:%s:%g:%s", f.name, type_name(f.type), (double)f.scale, f.unit);
        }
    }
    t.add("\n");
}

} // namespace bb
