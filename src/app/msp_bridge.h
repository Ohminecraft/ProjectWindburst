/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <string.h>

// ============================================================================
// MSP on the app link: just enough of Betaflight's MultiWii Serial Protocol for the ESC tools to find the flight controller and
// ask it for ESC passthrough - BLHeliSuite / BLHeliSuite32, ESC Configurator (esc-configurator.com) and the BLHeli_32 / AM32
// configurators all start that way: a handful of "who are you" requests, then MSP_SET_PASSTHROUGH, then the 4-way interface
// (hal/motor/esc4way.h) on the same port.
//
// The link carries WBP (the app's own protocol) as well. Both parsers see every byte; each one ignores what is not its own
// frame (MSP frames start with "$M<" or "$X<", WBP frames with its own sync and CRC). The MSP side never changes a setting.
//
//   v1  '$' 'M' '<' len cmd payload[len] xor(len, cmd, payload)          reply '$' 'M' '>' ..., unknown / refused '$' 'M' '!' ...
//   v2  '$' 'X' '<' flag cmd_lo cmd_hi len_lo len_hi payload crc8_dvb_s2(flag .. payload)
//
// Identification: the tools accept only flight controllers they know, so the MSP side answers as Betaflight 4.5 does
// (variant "BTFL", API 1.46) - for compatibility, like other non-Betaflight firmwares (esp-fc, INAV's tools mode) do; the board
// name says what it really is. Only read-only requests are answered; MSP_SET_MOTOR (spinning a motor from the ESC tool) is
// refused: the app's motor test, with its safety checks, is the place for that.
// Portable: unit-tested in test/test_esc4way.cpp.
// ============================================================================
namespace msp {

enum Cmd : uint16_t {
    API_VERSION = 1, FC_VARIANT = 2, FC_VERSION = 3, BOARD_INFO = 4, BUILD_INFO = 5, FEATURE_CONFIG = 36, ADVANCED_CONFIG = 90, STATUS = 101, MOTOR = 104, BOXIDS = 119,
    MOTOR_3D_CONFIG = 124, BATTERY_STATE = 130, MOTOR_CONFIG = 131, STATUS_EX = 150, UID = 160, SET_MOTOR = 214, SET_PASSTHROUGH = 245,
};
constexpr uint8_t kPassthroughEsc4Way = 0xFF;

inline uint8_t crc8_dvb_s2(uint8_t crc, uint8_t b) {
    crc ^= b;
    for (int i = 0; i < 8; i++) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
    return crc;
}

struct Frame { uint8_t version = 1; uint16_t cmd = 0; uint16_t len = 0; uint8_t payload[64]; };

// One byte at a time; true when a whole frame with a good checksum is in `f`. A request longer than the buffer is dropped.
class Parser {
public:
    bool mid_frame() const { return st_ != IDLE; }
    bool feed(uint8_t c, Frame& f) {
        switch (st_) {
        case IDLE: if (c == '$') st_ = HDR; break;
        case HDR: st_ = c == 'M' ? DIR1 : c == 'X' ? DIR2 : (c == '$' ? HDR : IDLE); break;
        case DIR1: st_ = c == '<' ? V1_LEN : IDLE; break;
        case DIR2: st_ = c == '<' ? V2_FLAG : IDLE; break;
        case V1_LEN: len_ = c; ck_ = c; st_ = V1_CMD; break;
        case V1_CMD: cmd_ = c; ck_ ^= c; n_ = 0; if (len_ > sizeof(f.payload)) { st_ = IDLE; break; } st_ = len_ ? V1_DATA : V1_CK; break;
        case V1_DATA: buf_[n_++] = c; ck_ ^= c; if (n_ >= len_) st_ = V1_CK; break;
        case V1_CK: st_ = IDLE; if (c == ck_) return out(1, f); bad_++; break;
        case V2_FLAG: crc_ = crc8_dvb_s2(0, c); st_ = V2_C0; break;
        case V2_C0: cmd_ = c; crc_ = crc8_dvb_s2(crc_, c); st_ = V2_C1; break;
        case V2_C1: cmd_ |= (uint16_t)(c << 8); crc_ = crc8_dvb_s2(crc_, c); st_ = V2_L0; break;
        case V2_L0: len_ = c; crc_ = crc8_dvb_s2(crc_, c); st_ = V2_L1; break;
        case V2_L1: len_ |= (uint16_t)(c << 8); crc_ = crc8_dvb_s2(crc_, c); n_ = 0;
                    if (len_ > sizeof(f.payload)) { st_ = IDLE; break; } st_ = len_ ? V2_DATA : V2_CK; break;
        case V2_DATA: buf_[n_++] = c; crc_ = crc8_dvb_s2(crc_, c); if (n_ >= len_) st_ = V2_CK; break;
        case V2_CK: st_ = IDLE; if (c == crc_) return out(2, f); bad_++; break;
        }
        return false;
    }
    uint32_t bad() const { return bad_; }
    uint32_t frames() const { return frames_; }
private:
    enum St : uint8_t { IDLE, HDR, DIR1, DIR2, V1_LEN, V1_CMD, V1_DATA, V1_CK, V2_FLAG, V2_C0, V2_C1, V2_L0, V2_L1, V2_DATA, V2_CK };
    bool out(uint8_t v, Frame& f) { f.version = v; f.cmd = cmd_; f.len = len_; memcpy(f.payload, buf_, len_); frames_++; return true; }
    St st_ = IDLE; uint16_t cmd_ = 0, len_ = 0, n_ = 0; uint8_t ck_ = 0, crc_ = 0; uint8_t buf_[64]; uint32_t bad_ = 0, frames_ = 0;
};

// A reply in the request's own version. error: '!' instead of '>' (Betaflight: unknown command, or refused). Returns its length.
inline size_t encode(uint8_t version, uint16_t cmd, const uint8_t* p, uint16_t n, bool error, uint8_t* out, size_t cap) {
    if (version == 1) {
        if (n > 255 || cap < (size_t)n + 6) return 0;
        out[0] = '$'; out[1] = 'M'; out[2] = error ? '!' : '>'; out[3] = (uint8_t)n; out[4] = (uint8_t)cmd;
        uint8_t ck = (uint8_t)(n ^ (uint8_t)cmd); for (uint16_t i = 0; i < n; i++) { out[5 + i] = p[i]; ck ^= p[i]; }
        out[5 + n] = ck; return (size_t)n + 6;
    }
    if (cap < (size_t)n + 9) return 0;
    out[0] = '$'; out[1] = 'X'; out[2] = error ? '!' : '>'; out[3] = 0; out[4] = (uint8_t)cmd; out[5] = (uint8_t)(cmd >> 8); out[6] = (uint8_t)n; out[7] = (uint8_t)(n >> 8);
    uint8_t crc = 0; for (int i = 3; i < 8; i++) crc = crc8_dvb_s2(crc, out[i]);
    for (uint16_t i = 0; i < n; i++) { out[8 + i] = p[i]; crc = crc8_dvb_s2(crc, p[i]); }
    out[8 + n] = crc; return (size_t)n + 9;
}

// What the answers are made of (the link task fills it from the telemetry snapshot)
struct Info {
    bool armed = false; uint8_t motors = 0; uint16_t cycle_us = 250;
    float batt_v = 0, amps = 0; uint8_t cells = 0; uint16_t mah = 0;
    uint8_t uid[12] = {};
    uint8_t bf_motor_protocol = 6;                      // Betaflight's numbering (PWM 0, Oneshot125 1, Oneshot42 2, Multishot 3, brushed 4,
                                                         // DShot150 5, DShot300 6, DShot600 7, ProShot1000 8)
    uint16_t pwm_rate = 480, idle_x100 = 550;           // motor PWM rate (Hz); idle in 0.01 % (Betaflight motorIdle)
    uint8_t fw_major = 0, fw_minor = 0, fw_patch = 0;
    const char* build_date = "Jan  1 2026";              // __DATE__ (11 characters)
    const char* build_time = "00:00:00";                 // __TIME__ (8)
    // ---- the firmware's own state, in its own terms (the Bridge turns it into Betaflight's): telemetry and settings
    const char* target = "ESP32S3";                      // the chip
    uint16_t health = 0;                                 // HEALTH_* bits (wbp_schema.h)
    uint32_t modes_on = 0, modes_set = 0;                // fc::Mode bits (core/rc.h): active now / given a switch in the settings
    uint32_t blockers = 0;                               // ARM_BLOCKER_* bits: why it will not arm
    uint8_t cpu_pct = 0;                                 // load of the flight-loop core
    float motor_out[8] = {};                             // 0..1, logical order
    uint16_t pulse_disarmed = 1000, pulse_armed = 1070, pulse_max = 2000;   // motors.* (PWM-type outputs; for DShot the tools only show them)
    uint8_t pole_count = 14; bool bidir = false;
    bool airmode = false;                                // general.airmode is not OFF
    uint8_t rx_kind = 0;                                 // 0 serial, 1 PPM, 2 PWM (receiver.protocol)
    bool gps = false, range = false;                     // sensors.* enabled
};

// Our modes (core/rc.h order: ARM, ANGLE, STABILIZE, ALTHOLD, POSHOLD, FAILSAFE, BEEPER, BLACKBOX, CALIB, RESCUE) as Betaflight's
// permanent box ids (msp_box.c); 255: Betaflight has none (STABILIZE).
constexpr uint8_t kBfBox[10] = {0, 1, 255, 3, 11, 27, 13, 26, 17, 46};
// Our ARM_BLOCKER_* bits (wbp_schema.h order) as Betaflight's armingDisableFlags_e bits (runtime_config.h); 255: none there.
constexpr uint8_t kBfArmBit[18] = {0, 12, 2, 7, 8, 1, 26, 255, 29, 255, 12, 27, 255, 255, 11, 5, 18, 18};
constexpr uint8_t kBfArmFlagsCount = 30;             // ARMING_DISABLE_FLAGS_COUNT (ARM_SWITCH = bit 29)

class Bridge {
public:
    bool mid_frame() const { return parser_.mid_frame(); }
    enum Result : uint8_t { NONE, REPLY, PASSTHROUGH };
    // One byte from the link. REPLY: out / out_n hold the answer. PASSTHROUGH: the PC tool asks for the 4-way interface - the caller
    // asks the flight side to let go of the motors and calls passthrough_reply() with the number of ESCs (0 = refused).
    Result feed(uint8_t c, const Info& info, uint8_t* out, size_t cap, size_t& out_n) {
        out_n = 0;
        if (!parser_.feed(c, f_)) return NONE;
        cmds_[cmd_n_ % kCmdRing] = f_.cmd; cmd_n_++;
        if (f_.cmd == SET_PASSTHROUGH) {
            const uint8_t mode = f_.len >= 1 ? f_.payload[0] : kPassthroughEsc4Way;      // (no payload: the legacy request, always 4-way)
            if (mode == kPassthroughEsc4Way && !info.armed) { pending_version_ = f_.version; return PASSTHROUGH; }
            const uint8_t zero = 0; out_n = encode(f_.version, SET_PASSTHROUGH, &zero, 1, false, out, cap); refused_++; return REPLY;
        }
        uint8_t p[64]; uint16_t n = 0; bool ok = answer(f_.cmd, info, p, n);
        out_n = encode(f_.version, f_.cmd, p, ok ? n : 0, !ok, out, cap);
        if (!ok) unknown_++;
        answered_++;
        return out_n ? REPLY : NONE;
    }
    size_t passthrough_reply(uint8_t esc_count, uint8_t* out, size_t cap) { return encode(pending_version_, SET_PASSTHROUGH, &esc_count, 1, false, out, cap); }
    static constexpr int kCmdRing = 12;
    // The last request numbers, oldest first (port_diag.h); returns how many.
    int last_cmds(uint16_t* out) const { const int n = cmd_n_ < (uint32_t)kCmdRing ? (int)cmd_n_ : kCmdRing; for (int i = 0; i < n; i++) out[i] = cmds_[(cmd_n_ - n + i) % kCmdRing]; return n; }
    uint32_t answered() const { return answered_; }
    uint32_t unknown() const { return unknown_; }                 // answered '!': not a request this firmware knows
    uint32_t refused() const { return refused_; }
    const Parser& parser() const { return parser_; }

private:
    static void u16(uint8_t* p, uint16_t& n, uint16_t v) { p[n++] = (uint8_t)v; p[n++] = (uint8_t)(v >> 8); }
    static void u32(uint8_t* p, uint16_t& n, uint32_t v) { u16(p, n, (uint16_t)v); u16(p, n, (uint16_t)(v >> 16)); }
    static void pstr(uint8_t* p, uint16_t& n, const char* s) { const uint8_t l = (uint8_t)strlen(s); p[n++] = l; memcpy(p + n, s, l); n += l; }
    static uint16_t clamp16(float v, float lo, float hi) { return (uint16_t)(v < lo ? lo : v > hi ? hi : v); }
    bool answer(uint16_t cmd, const Info& in, uint8_t* p, uint16_t& n) {
        switch (cmd) {
        case API_VERSION: p[n++] = 0; p[n++] = 1; p[n++] = 46; return true;
        case FC_VARIANT: memcpy(p, "BTFL", 4); n = 4; return true;
        case FC_VERSION: p[n++] = 4; p[n++] = 5; p[n++] = 0; return true;
        case BOARD_INFO:
            memcpy(p, "WBRX", 4); n = 4; u16(p, n, 0); p[n++] = 0; p[n++] = 0;            // identifier, hardware revision, "FC", capabilities
            pstr(p, n, in.target); pstr(p, n, "WINDBURST"); pstr(p, n, "");              // target (the chip), board, manufacturer
            memset(p + n, 0, 32); n += 32;                                                // signature
            p[n++] = 0xFF; p[n++] = 0;                                                     // MCU type (by name), configuration state
            return true;
        case BUILD_INFO: memcpy(p, in.build_date, 11); memcpy(p + 11, in.build_time, 8); n = 19;
            { char rev[8]; rev[0] = 'w'; rev[1] = (char)('0' + in.fw_major % 10); rev[2] = (char)('0' + in.fw_minor / 10 % 10); rev[3] = (char)('0' + in.fw_minor % 10); rev[4] = (char)('0' + in.fw_patch / 10 % 10); rev[5] = (char)('0' + in.fw_patch % 10); rev[6] = 'x';   // 0.10.0: w01000x
              memcpy(p + n, rev, 7); n += 7; }
            return true;
        case FEATURE_CONFIG: {                        // Betaflight's feature bits for what this firmware has / is set to (never 3D: the tools ask)
            uint32_t f = 0;
            if (in.rx_kind == 1) f |= 1u << 0;                                             // RX_PPM
            if (in.rx_kind == 0) f |= 1u << 3;                                             // RX_SERIAL
            if (in.gps) f |= 1u << 7;                                                      // GPS
            if (in.range) f |= 1u << 9;                                                    // RANGEFINDER
            if (in.rx_kind == 2) f |= 1u << 13;                                            // RX_PARALLEL_PWM
            if (in.airmode) f |= 1u << 22;                                                 // AIRMODE
            u32(p, n, f); return true; }
        case ADVANCED_CONFIG:                         // Betaflight 4.5's layout (msp.c); BLHeliSuite32 asks it for the motor protocol
            p[n++] = 1; p[n++] = 1; p[n++] = 0; p[n++] = in.bf_motor_protocol; u16(p, n, in.pwm_rate); u16(p, n, in.idle_x100);
            p[n++] = 0; p[n++] = 0; p[n++] = 0; p[n++] = 0; p[n++] = 48; u16(p, n, 125); u16(p, n, 0); p[n++] = 0;
            p[n++] = 0; p[n++] = 1;                                                       // debug mode NONE, debug mode count
            return true;
        case STATUS: case STATUS_EX: {                // Betaflight 4.5's layout (msp.c)
            u16(p, n, in.cycle_us); u16(p, n, 0);                                          // cycle time, I2C errors
            u16(p, n, sensor_bits(in)); u32(p, n, mode_flags(in)); p[n++] = 0;            // sensors, flight modes (bit = place in BOXIDS), PID profile
            u16(p, n, in.cpu_pct > 100 ? 100 : in.cpu_pct);                               // load, %
            if (cmd == STATUS) u16(p, n, in.cycle_us);                                     // gyro cycle time
            else { p[n++] = 1; p[n++] = 0; }                                               // PID profiles, rate profile
            p[n++] = 0;                                                                    // more mode flag bytes: none
            p[n++] = kBfArmFlagsCount; u32(p, n, arm_flags(in)); p[n++] = 0;               // why it will not arm; reboot needed
            return true; }
        case BOXIDS: { uint8_t ids[10]; const int k = boxes(in, ids); for (int i = 0; i < k; i++) p[n++] = ids[i]; return true; }
        case MOTOR:                                   // 1000 .. 2000, as Betaflight shows them (disarmed: 1000)
            for (int i = 0; i < 8; i++) { const float o = in.motor_out[i] < 0 ? 0.0f : in.motor_out[i] > 1 ? 1.0f : in.motor_out[i];
                u16(p, n, i < in.motors ? (uint16_t)(1000.0f + (in.armed ? o : 0.0f) * 1000.0f + 0.5f) : 0); }
            return true;
        case MOTOR_3D_CONFIG: u16(p, n, 1406); u16(p, n, 1514); u16(p, n, 1460); return true;
        case MOTOR_CONFIG:                            // min throttle, max throttle, min command, motors, poles, DShot telemetry, ESC sensor
            u16(p, n, in.pulse_armed); u16(p, n, in.pulse_max); u16(p, n, in.pulse_disarmed); p[n++] = in.motors; p[n++] = in.pole_count;
            p[n++] = in.bidir ? 1 : 0; p[n++] = 0; return true;
        case BATTERY_STATE:
            p[n++] = in.cells; u16(p, n, 0); p[n++] = (uint8_t)clamp16(in.batt_v * 10.0f + 0.5f, 0, 255); u16(p, n, in.mah);
            u16(p, n, (uint16_t)(int16_t)(in.amps * 100.0f)); p[n++] = 0; u16(p, n, clamp16(in.batt_v * 100.0f + 0.5f, 0, 65535));
            return true;
        case UID: memcpy(p, in.uid, 12); n = 12; return true;
        default: return false;                                                              // (MSP_SET_MOTOR among them: refused)
        }
    }
public:
    // (public for the tests) Betaflight's sensor bits: ACC 0, BARO 1, MAG 2, GPS 3, RANGEFINDER 4, GYRO 5
    static uint16_t sensor_bits(const Info& in) {
        uint16_t b = 0;
        if (in.health & 1u) b |= 1u << 0 | 1u << 5;                                        // HEALTH_IMU: accelerometer and gyro
        if (in.health & (1u << 2)) b |= 1u << 1;                                           // HEALTH_BARO
        if (in.health & (1u << 5)) b |= 1u << 2;                                           // HEALTH_MAG_READY
        if (in.health & (1u << 4)) b |= 1u << 3;                                           // HEALTH_GPS
        if (in.health & (1u << 3)) b |= 1u << 4;                                           // HEALTH_RANGE
        return b;
    }
    // The boxes there are, Betaflight permanent ids in increasing order: ARM always, the others when a switch is set for them.
    static int boxes(const Info& in, uint8_t* ids, uint8_t* ours = nullptr) {
        int k = 0;
        for (uint8_t id = 0; id < 64; id++)
            for (int m = 0; m < 10; m++)
                if (kBfBox[m] == id && (m == 0 || ((in.modes_set >> m) & 1u))) { if (ours) ours[k] = (uint8_t)m; ids[k++] = id; }
        return k;
    }
    static uint32_t mode_flags(const Info& in) {
        uint8_t ids[10], ours[10]; const int k = boxes(in, ids, ours); uint32_t f = 0;
        for (int i = 0; i < k; i++) if (ours[i] == 0 ? in.armed : ((in.modes_on >> ours[i]) & 1u)) f |= 1u << i;
        return f;
    }
    static uint32_t arm_flags(const Info& in) {
        uint32_t f = 0;
        for (int b = 0; b < 18; b++) if (((in.blockers >> b) & 1u) && kBfArmBit[b] != 255) f |= 1u << kBfArmBit[b];
        return in.armed ? 0u : f;
    }
private:
    Parser parser_; Frame f_; uint8_t pending_version_ = 1; uint32_t answered_ = 0, refused_ = 0;
    uint16_t cmds_[kCmdRing] = {}; uint32_t cmd_n_ = 0, unknown_ = 0;
};

} // namespace msp
