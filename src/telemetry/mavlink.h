/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <string.h>

// ============================================================================
// MAVLink 2, the few messages a ground station needs to show the craft (sent): HEARTBEAT, SYS_STATUS, GPS_RAW_INT, ATTITUDE,
// GLOBAL_POSITION_INT, RC_CHANNELS, VFR_HUD, BATTERY_STATUS, STATUSTEXT (common.xml); and the two an ExpressLRS receiver in MAVLink
// mode sends (read, hal/rx/rx_mavlink.h): RC_CHANNELS_OVERRIDE (the sticks), RADIO_STATUS (its buffer: flow control). From 0.40 the
// ground station's commands and the mission protocol, both ways (below; telemetry/mavlink_gcs.h).
//
//   frame:  0xFD len incompat compat seq sysid compid msgid(3, LE) payload(len) crc(2, LE)
//   crc:    CRC-16/MCRF4XX ("X.25" in MAVLink's words) over everything after 0xFD, then the message's CRC_EXTRA byte
//   payload: the fields sorted by size (8, 4, 2, 1 byte types; arrays by their element), extension fields after them, little
//            endian; trailing zero bytes are cut (at least one byte stays) - the receiver fills them in again
//
// Checked byte for byte against pymavlink (tools/mavlink_vectors.py -> test/mavlink_vectors.h, test/test_mavlink.cpp).
// Portable: no Arduino, little-endian hosts (ESP32, STM32, x86).
// ============================================================================
namespace mav {

constexpr uint8_t kStx = 0xFD;
constexpr size_t kHeaderLen = 10, kMaxPayload = 255, kMaxFrame = kHeaderLen + kMaxPayload + 2;

enum : uint32_t {
    MSG_HEARTBEAT = 0, MSG_SYS_STATUS = 1, MSG_GPS_RAW_INT = 24, MSG_ATTITUDE = 30, MSG_GLOBAL_POSITION_INT = 33,
    MSG_RC_CHANNELS = 65, MSG_RC_CHANNELS_OVERRIDE = 70, MSG_VFR_HUD = 74, MSG_RADIO_STATUS = 109, MSG_BATTERY_STATUS = 147, MSG_STATUSTEXT = 253,
    // 0.40: the ground station's commands and the mission protocol (mavlink_gcs.h)
    MSG_MISSION_SET_CURRENT = 41, MSG_MISSION_CURRENT = 42, MSG_MISSION_REQUEST_LIST = 43, MSG_MISSION_COUNT = 44, MSG_MISSION_CLEAR_ALL = 45,
    MSG_MISSION_ITEM_REACHED = 46, MSG_MISSION_ACK = 47, MSG_GPS_GLOBAL_ORIGIN = 49, MSG_MISSION_REQUEST_INT = 51, MSG_MISSION_ITEM_INT = 73,
    MSG_PARAM_REQUEST_READ = 20, MSG_PARAM_REQUEST_LIST = 21, MSG_PARAM_VALUE = 22, MSG_PARAM_SET = 23,
    MSG_COMMAND_INT = 75, MSG_COMMAND_LONG = 76, MSG_COMMAND_ACK = 77, MSG_TIMESYNC = 111, MSG_AUTOPILOT_VERSION = 148, MSG_HOME_POSITION = 242,
    MSG_AVAILABLE_MODES = 435, MSG_AVAILABLE_MODES_MONITOR = 437,
    MSG_MISSION_ITEM = 39, MSG_MISSION_REQUEST = 40,          // (0.40.1: the float forms Mission Planner falls back to)
};
// CRC_EXTRA of each message (from its definition; pymavlink: MAVLink_<name>_message.crc_extra)
inline uint8_t crc_extra(uint32_t id) {
    switch (id) {
    case MSG_HEARTBEAT: return 50; case MSG_SYS_STATUS: return 124; case MSG_GPS_RAW_INT: return 24; case MSG_ATTITUDE: return 39;
    case MSG_GLOBAL_POSITION_INT: return 104; case MSG_RC_CHANNELS: return 118; case MSG_VFR_HUD: return 20; case MSG_BATTERY_STATUS: return 154;
    case MSG_STATUSTEXT: return 83; case MSG_RC_CHANNELS_OVERRIDE: return 124; case MSG_RADIO_STATUS: return 185;
    case MSG_MISSION_SET_CURRENT: return 28; case MSG_MISSION_CURRENT: return 28; case MSG_MISSION_REQUEST_LIST: return 132; case MSG_MISSION_COUNT: return 221;
    case MSG_MISSION_CLEAR_ALL: return 232; case MSG_MISSION_ITEM_REACHED: return 11; case MSG_MISSION_ACK: return 153; case MSG_GPS_GLOBAL_ORIGIN: return 39;
    case MSG_MISSION_REQUEST_INT: return 196; case MSG_MISSION_ITEM_INT: return 38; case MSG_COMMAND_INT: return 158; case MSG_COMMAND_LONG: return 152;
    case MSG_COMMAND_ACK: return 143; case MSG_TIMESYNC: return 34; case MSG_PARAM_REQUEST_READ: return 214; case MSG_PARAM_REQUEST_LIST: return 159; case MSG_PARAM_VALUE: return 220; case MSG_PARAM_SET: return 168; case MSG_AUTOPILOT_VERSION: return 178; case MSG_HOME_POSITION: return 104;
    case MSG_AVAILABLE_MODES: return 134; case MSG_AVAILABLE_MODES_MONITOR: return 30; case MSG_MISSION_ITEM: return 254; case MSG_MISSION_REQUEST: return 230;
    default: return 0;
    }
}

// The enum values of common.xml that are used here
enum : uint8_t { MAV_TYPE_GENERIC = 0, MAV_TYPE_QUADROTOR = 2, MAV_TYPE_HEXAROTOR = 13, MAV_TYPE_OCTOROTOR = 14 };
enum : uint8_t { MAV_AUTOPILOT_GENERIC = 0 };
enum : uint8_t { MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1, MAV_MODE_FLAG_STABILIZE_ENABLED = 16, MAV_MODE_FLAG_MANUAL_INPUT_ENABLED = 64, MAV_MODE_FLAG_SAFETY_ARMED = 128 };
enum : uint8_t { MAV_STATE_BOOT = 1, MAV_STATE_CALIBRATING = 2, MAV_STATE_STANDBY = 3, MAV_STATE_ACTIVE = 4, MAV_STATE_CRITICAL = 5 };
enum : uint8_t { MAV_SEVERITY_CRITICAL = 2, MAV_SEVERITY_ERROR = 3, MAV_SEVERITY_WARNING = 4, MAV_SEVERITY_NOTICE = 5, MAV_SEVERITY_INFO = 6 };
enum : uint32_t {                                   // MAV_SYS_STATUS_SENSOR
    SENSOR_GYRO = 1u << 0, SENSOR_ACCEL = 1u << 1, SENSOR_MAG = 1u << 2, SENSOR_BARO = 1u << 3, SENSOR_GPS = 1u << 5,
    SENSOR_FLOW = 1u << 6, SENSOR_LASER = 1u << 8, SENSOR_MOTORS = 1u << 15, SENSOR_RC = 1u << 16, SENSOR_AHRS = 1u << 21,
    SENSOR_BATTERY = 1u << 25,
};
enum : uint8_t { GPS_FIX_NO_GPS = 0, GPS_FIX_NONE = 1, GPS_FIX_2D = 2, GPS_FIX_3D = 3 };
// ArduCopter's flight mode numbers in HEARTBEAT.custom_mode (what Betaflight sends as well): ground stations name them
enum : uint32_t { COPTER_STABILIZE = 0, COPTER_ACRO = 1, COPTER_ALT_HOLD = 2, COPTER_AUTO = 3, COPTER_RTL = 6, COPTER_LOITER = 5, COPTER_LAND = 9, COPTER_POSHOLD = 16, COPTER_SMART_RTL = 21, COPTER_AUTOTUNE = 15 };

// CRC-16/MCRF4XX step (MAVLink's crc_accumulate)
inline uint16_t crc_add(uint16_t crc, uint8_t b) {
    uint8_t t = (uint8_t)(b ^ (uint8_t)(crc & 0xFF));
    t = (uint8_t)(t ^ (uint8_t)(t << 4));
    return (uint16_t)((crc >> 8) ^ ((uint16_t)t << 8) ^ ((uint16_t)t << 3) ^ (uint16_t)(t >> 4));
}

// A payload in wire order
struct Payload {
    uint8_t b[kMaxPayload] = {}; uint8_t n = 0;
    void raw(const void* p, size_t k) { if (n + k > kMaxPayload) return; memcpy(b + n, p, k); n = (uint8_t)(n + k); }
    void u8(uint8_t v) { raw(&v, 1); }
    void i8(int8_t v) { raw(&v, 1); }
    void u16(uint16_t v) { raw(&v, 2); }
    void i16(int16_t v) { raw(&v, 2); }
    void u32(uint32_t v) { raw(&v, 4); }
    void i32(int32_t v) { raw(&v, 4); }
    void u64(uint64_t v) { raw(&v, 8); }
    void f32(float v) { raw(&v, 4); }
};

// Frames payloads: one sequence number for everything this system sends
class Encoder {
public:
    uint8_t sysid = 1, compid = 1;                  // MAV_COMP_ID_AUTOPILOT1
    // the frame into out (kMaxFrame bytes at most); returns its length
    size_t frame(uint32_t msgid, const Payload& p, uint8_t* out) {
        uint8_t len = p.n;
        while (len > 1 && p.b[len - 1] == 0) len--;          // MAVLink 2 payload truncation
        out[0] = kStx; out[1] = len; out[2] = 0; out[3] = 0; out[4] = seq_++; out[5] = sysid; out[6] = compid;
        out[7] = (uint8_t)(msgid & 0xFF); out[8] = (uint8_t)((msgid >> 8) & 0xFF); out[9] = (uint8_t)((msgid >> 16) & 0xFF);
        memcpy(out + kHeaderLen, p.b, len);
        uint16_t crc = 0xFFFF;
        for (size_t i = 1; i < kHeaderLen + len; i++) crc = crc_add(crc, out[i]);
        crc = crc_add(crc, crc_extra(msgid));
        out[kHeaderLen + len] = (uint8_t)(crc & 0xFF); out[kHeaderLen + len + 1] = (uint8_t)(crc >> 8);
        return kHeaderLen + len + 2u;
    }
    uint8_t seq() const { return seq_; }
    void set_seq(uint8_t s) { seq_ = s; }
private:
    uint8_t seq_ = 0;
};

// ---- the messages (field names and units as common.xml)
struct Heartbeat { uint8_t type = MAV_TYPE_QUADROTOR, autopilot = MAV_AUTOPILOT_GENERIC, base_mode = 0; uint32_t custom_mode = 0; uint8_t system_status = MAV_STATE_STANDBY, mavlink_version = 3; };
struct SysStatus {
    uint32_t present = 0, enabled = 0, health = 0;      // onboard_control_sensors_*
    uint16_t load = 0;                                  // d% (0.1 %)
    uint16_t voltage_battery = 0xFFFF;                  // mV, UINT16_MAX = unknown
    int16_t current_battery = -1;                       // cA, -1 = unknown
    int8_t battery_remaining = -1;                      // %, -1 = unknown
    uint16_t drop_rate_comm = 0, errors_comm = 0, errors_count[4] = {};
    uint32_t present_ext = 0, enabled_ext = 0, health_ext = 0;
};
struct GpsRawInt {
    uint64_t time_usec = 0; uint8_t fix_type = GPS_FIX_NO_GPS; int32_t lat = 0, lon = 0, alt = 0;     // degE7, mm MSL
    uint16_t eph = 0xFFFF, epv = 0xFFFF, vel = 0xFFFF, cog = 0xFFFF; uint8_t satellites_visible = 255;      // HDOP x100, cm/s, cdeg
    int32_t alt_ellipsoid = 0; uint32_t h_acc = 0, v_acc = 0, vel_acc = 0, hdg_acc = 0; uint16_t yaw = 0;   // mm, mm/s, degE5, cdeg
};
struct Attitude { uint32_t time_boot_ms = 0; float roll = 0, pitch = 0, yaw = 0, rollspeed = 0, pitchspeed = 0, yawspeed = 0; };   // rad, rad/s
struct GlobalPositionInt { uint32_t time_boot_ms = 0; int32_t lat = 0, lon = 0, alt = 0, relative_alt = 0; int16_t vx = 0, vy = 0, vz = 0; uint16_t hdg = 0xFFFF; };  // degE7, mm, cm/s NED, cdeg
struct RcChannels { uint32_t time_boot_ms = 0; uint8_t chancount = 0; uint16_t chan[18] = {}; uint8_t rssi = 255; };              // us (UINT16_MAX = unused), rssi 0..254, 255 = unknown
struct VfrHud { float airspeed = 0, groundspeed = 0; int16_t heading = 0; uint16_t throttle = 0; float alt = 0, climb = 0; };     // m/s, deg, %, m MSL, m/s
struct BatteryStatus {
    uint8_t id = 0, battery_function = 0, type = 0; int16_t temperature = 0x7FFF;                  // cdegC, INT16_MAX = unknown
    uint16_t voltages[10]; int16_t current_battery = -1; int32_t current_consumed = -1, energy_consumed = -1;    // mV (cells; UINT16_MAX = none), cA, mAh, hJ
    int8_t battery_remaining = -1; int32_t time_remaining = 0; uint8_t charge_state = 0; uint16_t voltages_ext[4] = {}; uint8_t mode = 0; uint32_t fault_bitmask = 0;
    BatteryStatus() { for (uint16_t& v : voltages) v = 0xFFFF; }
};
struct StatusText { uint8_t severity = MAV_SEVERITY_INFO; char text[50] = {}; uint16_t id = 0; uint8_t chunk_seq = 0; };
struct RcChannelsOverride { uint8_t target_system = 0, target_component = 0; uint16_t chan[18] = {}; };          // us; 0 or UINT16_MAX = leave the channel as it is
struct RadioStatus { uint16_t rxerrors = 0, fixed = 0; uint8_t rssi = 0, remrssi = 0, txbuf = 100, noise = 0, remnoise = 0; };   // txbuf: % of the radio's buffer free

inline size_t pack(Encoder& e, const Heartbeat& m, uint8_t* out) {
    Payload p; p.u32(m.custom_mode); p.u8(m.type); p.u8(m.autopilot); p.u8(m.base_mode); p.u8(m.system_status); p.u8(m.mavlink_version);
    return e.frame(MSG_HEARTBEAT, p, out);
}
inline size_t pack(Encoder& e, const SysStatus& m, uint8_t* out) {
    Payload p; p.u32(m.present); p.u32(m.enabled); p.u32(m.health); p.u16(m.load); p.u16(m.voltage_battery); p.i16(m.current_battery);
    p.u16(m.drop_rate_comm); p.u16(m.errors_comm); for (uint16_t c : m.errors_count) p.u16(c); p.i8(m.battery_remaining);
    p.u32(m.present_ext); p.u32(m.enabled_ext); p.u32(m.health_ext);
    return e.frame(MSG_SYS_STATUS, p, out);
}
inline size_t pack(Encoder& e, const GpsRawInt& m, uint8_t* out) {
    Payload p; p.u64(m.time_usec); p.i32(m.lat); p.i32(m.lon); p.i32(m.alt); p.u16(m.eph); p.u16(m.epv); p.u16(m.vel); p.u16(m.cog);
    p.u8(m.fix_type); p.u8(m.satellites_visible);
    p.i32(m.alt_ellipsoid); p.u32(m.h_acc); p.u32(m.v_acc); p.u32(m.vel_acc); p.u32(m.hdg_acc); p.u16(m.yaw);
    return e.frame(MSG_GPS_RAW_INT, p, out);
}
inline size_t pack(Encoder& e, const Attitude& m, uint8_t* out) {
    Payload p; p.u32(m.time_boot_ms); p.f32(m.roll); p.f32(m.pitch); p.f32(m.yaw); p.f32(m.rollspeed); p.f32(m.pitchspeed); p.f32(m.yawspeed);
    return e.frame(MSG_ATTITUDE, p, out);
}
inline size_t pack(Encoder& e, const GlobalPositionInt& m, uint8_t* out) {
    Payload p; p.u32(m.time_boot_ms); p.i32(m.lat); p.i32(m.lon); p.i32(m.alt); p.i32(m.relative_alt); p.i16(m.vx); p.i16(m.vy); p.i16(m.vz); p.u16(m.hdg);
    return e.frame(MSG_GLOBAL_POSITION_INT, p, out);
}
inline size_t pack(Encoder& e, const RcChannels& m, uint8_t* out) {
    Payload p; p.u32(m.time_boot_ms); for (uint16_t c : m.chan) p.u16(c); p.u8(m.chancount); p.u8(m.rssi);
    return e.frame(MSG_RC_CHANNELS, p, out);
}
inline size_t pack(Encoder& e, const VfrHud& m, uint8_t* out) {
    Payload p; p.f32(m.airspeed); p.f32(m.groundspeed); p.f32(m.alt); p.f32(m.climb); p.i16(m.heading); p.u16(m.throttle);
    return e.frame(MSG_VFR_HUD, p, out);
}
inline size_t pack(Encoder& e, const BatteryStatus& m, uint8_t* out) {
    Payload p; p.i32(m.current_consumed); p.i32(m.energy_consumed); p.i16(m.temperature); for (uint16_t v : m.voltages) p.u16(v); p.i16(m.current_battery);
    p.u8(m.id); p.u8(m.battery_function); p.u8(m.type); p.i8(m.battery_remaining);
    p.i32(m.time_remaining); p.u8(m.charge_state); for (uint16_t v : m.voltages_ext) p.u16(v); p.u8(m.mode); p.u32(m.fault_bitmask);
    return e.frame(MSG_BATTERY_STATUS, p, out);
}
inline size_t pack(Encoder& e, const StatusText& m, uint8_t* out) {
    Payload p; p.u8(m.severity); p.raw(m.text, sizeof(m.text)); p.u16(m.id); p.u8(m.chunk_seq);
    return e.frame(MSG_STATUSTEXT, p, out);
}
inline size_t pack(Encoder& e, const RcChannelsOverride& m, uint8_t* out) {
    Payload p; for (int i = 0; i < 8; i++) p.u16(m.chan[i]);
    p.u8(m.target_system); p.u8(m.target_component); for (int i = 8; i < 18; i++) p.u16(m.chan[i]);     // (chan9 .. chan18: extension fields)
    return e.frame(MSG_RC_CHANNELS_OVERRIDE, p, out);
}
inline size_t pack(Encoder& e, const RadioStatus& m, uint8_t* out) {
    Payload p; p.u16(m.rxerrors); p.u16(m.fixed); p.u8(m.rssi); p.u8(m.remrssi); p.u8(m.txbuf); p.u8(m.noise); p.u8(m.remnoise);
    return e.frame(MSG_RADIO_STATUS, p, out);
}

// ---- receiving: whole frames out of a byte stream (CRC checked, payload zero-filled to its full size again). The MAVLink receiver
// (hal/rx/rx_mavlink.h) reads with it, the tests and the simulator read what the firmware sends. Like MAVLink's own mavlink_parse_char
// it does not search a broken frame for another start byte: it waits for the next one.
class Parser {
public:
    struct Frame { uint32_t msgid = 0; uint8_t seq = 0, sysid = 0, compid = 0, len = 0; uint8_t payload[kMaxPayload] = {}; };
    // true when c completed a valid frame (in frame()); frames with a bad CRC or an unknown message are counted and dropped
    bool feed(uint8_t c) {
        if (n_ == 0 && c != kStx) return false;
        buf_[n_++] = c;
        if (n_ < 2) return false;
        const size_t want = kHeaderLen + buf_[1] + 2u;
        if (n_ < want) return false;
        n_ = 0;
        if (buf_[2] != 0) { bad_++; return false; }                          // (incompat flags: signing is not supported)
        const uint32_t id = buf_[7] | ((uint32_t)buf_[8] << 8) | ((uint32_t)buf_[9] << 16);
        const uint8_t extra = crc_extra(id);
        if (!extra) { unknown_++; return false; }
        const uint8_t len = buf_[1];
        uint16_t crc = 0xFFFF;
        for (size_t i = 1; i < kHeaderLen + len; i++) crc = crc_add(crc, buf_[i]);
        crc = crc_add(crc, extra);
        if (buf_[kHeaderLen + len] != (crc & 0xFF) || buf_[kHeaderLen + len + 1] != (crc >> 8)) { bad_++; return false; }
        f_ = Frame(); f_.msgid = id; f_.seq = buf_[4]; f_.sysid = buf_[5]; f_.compid = buf_[6]; f_.len = len;
        memcpy(f_.payload, buf_ + kHeaderLen, len);
        good_++;
        return true;
    }
    const Frame& frame() const { return f_; }
    uint32_t good() const { return good_; }
    uint32_t bad() const { return bad_; }
    uint32_t unknown() const { return unknown_; }
    // little-endian field of the (zero-filled) payload
    template <typename T> static T get(const Frame& f, size_t off) { T v; memcpy(&v, f.payload + off, sizeof(T)); return v; }
private:
    uint8_t buf_[kMaxFrame] = {}; size_t n_ = 0; Frame f_; uint32_t good_ = 0, bad_ = 0, unknown_ = 0;
};

// the two messages read, out of a parsed frame (false: the frame is another message)
inline bool unpack(const Parser::Frame& f, RcChannelsOverride& m) {
    if (f.msgid != MSG_RC_CHANNELS_OVERRIDE) return false;
    for (int i = 0; i < 8; i++) m.chan[i] = Parser::get<uint16_t>(f, 2 * i);
    m.target_system = f.payload[16]; m.target_component = f.payload[17];
    for (int i = 8; i < 18; i++) m.chan[i] = Parser::get<uint16_t>(f, 18 + 2 * (i - 8));
    return true;
}
inline bool unpack(const Parser::Frame& f, RadioStatus& m) {
    if (f.msgid != MSG_RADIO_STATUS) return false;
    m.rxerrors = Parser::get<uint16_t>(f, 0); m.fixed = Parser::get<uint16_t>(f, 2);
    m.rssi = f.payload[4]; m.remrssi = f.payload[5]; m.txbuf = f.payload[6]; m.noise = f.payload[7]; m.remnoise = f.payload[8];
    return true;
}

// ---- 0.40: the messages of the ground station's commands and of the mission protocol. Each lists its fields once, in wire order
// (wire()): payload_of() writes them, unpack() reads them back - one list, so the two cannot disagree. Field names as common.xml.
struct PutField { Payload& p; template <class T> void operator()(T& v) { p.raw(&v, sizeof(v)); } };
struct GetField { const uint8_t* b; size_t o; template <class T> void operator()(T& v) { memcpy(&v, b + o, sizeof(v)); o += sizeof(v); } };
template <class M> Payload payload_of(const M& m) { M c = m; Payload p; PutField w{p}; c.wire(w); return p; }
template <class M> size_t pack(Encoder& e, const M& m, uint8_t* out) { return e.frame(M::kId, payload_of(m), out); }
template <class M> bool unpack(const Parser::Frame& f, M& m) { if (f.msgid != M::kId) return false; GetField r{f.payload, 0}; m.wire(r); return true; }

// MAV_CMD, MAV_RESULT, MAV_MISSION_RESULT, MAV_FRAME, MISSION_STATE, ... (common.xml) as far as they are used
enum : uint16_t { CMD_NAV_WAYPOINT = 16, CMD_NAV_LOITER_TIME = 19, CMD_NAV_RETURN_TO_LAUNCH = 20, CMD_NAV_LAND = 21, CMD_NAV_TAKEOFF = 22, CMD_NAV_DELAY = 93,
                  CMD_DO_SET_MODE = 176, CMD_DO_CHANGE_SPEED = 178, CMD_DO_SET_HOME = 179, CMD_DO_SET_SERVO = 183, CMD_PREFLIGHT_REBOOT_SHUTDOWN = 246,
                  CMD_MISSION_START = 300, CMD_COMPONENT_ARM_DISARM = 400, CMD_REQUEST_MESSAGE = 512, CMD_REQUEST_AUTOPILOT_CAPABILITIES = 520 };
constexpr uint8_t RESULT_ACCEPTED = 0, RESULT_TEMPORARILY_REJECTED = 1, RESULT_DENIED = 2, RESULT_UNSUPPORTED = 3, RESULT_FAILED = 4, RESULT_IN_PROGRESS = 5;   // MAV_RESULT
enum : uint8_t { MISSION_ACCEPTED = 0, MISSION_ERROR = 1, MISSION_UNSUPPORTED_FRAME = 2, MISSION_UNSUPPORTED = 3, MISSION_NO_SPACE = 4, MISSION_INVALID = 5,
                 MISSION_INVALID_PARAM1 = 6, MISSION_INVALID_PARAM2 = 7, MISSION_INVALID_PARAM5_X = 10, MISSION_INVALID_PARAM6_Y = 11, MISSION_INVALID_PARAM7 = 12,
                 MISSION_INVALID_SEQUENCE = 13, MISSION_DENIED = 14, MISSION_OPERATION_CANCELLED = 15 };
enum : uint8_t { MISSION_TYPE_MISSION = 0, MISSION_TYPE_FENCE = 1, MISSION_TYPE_RALLY = 2, MISSION_TYPE_ALL = 255 };
enum : uint8_t { FRAME_GLOBAL = 0, FRAME_MISSION = 2, FRAME_GLOBAL_RELATIVE_ALT = 3, FRAME_GLOBAL_INT = 5, FRAME_GLOBAL_RELATIVE_ALT_INT = 6 };
enum : uint8_t { MISSION_STATE_NO_MISSION = 1, MISSION_STATE_NOT_STARTED = 2, MISSION_STATE_ACTIVE = 3, MISSION_STATE_PAUSED = 4, MISSION_STATE_COMPLETE = 5 };
enum : uint8_t { STANDARD_MODE_NON_STANDARD = 0, STANDARD_MODE_POSITION_HOLD = 1, STANDARD_MODE_ALTITUDE_HOLD = 4, STANDARD_MODE_SAFE_RECOVERY = 5,
                 STANDARD_MODE_MISSION = 6, STANDARD_MODE_LAND = 7 };
enum : uint32_t { MODE_PROPERTY_NOT_USER_SELECTABLE = 2 };
enum : uint64_t { CAP_MISSION_INT = 4, CAP_COMMAND_INT = 8, CAP_MAVLINK2 = 8192 };

struct CommandLong {
    static constexpr uint32_t kId = MSG_COMMAND_LONG;
    float param[7] = {}; uint16_t command = 0; uint8_t target_system = 0, target_component = 0, confirmation = 0;
    template <class V> void wire(V& v) { v(param); v(command); v(target_system); v(target_component); v(confirmation); }
};
struct CommandInt {
    static constexpr uint32_t kId = MSG_COMMAND_INT;
    float param[4] = {}; int32_t x = 0, y = 0; float z = 0; uint16_t command = 0; uint8_t target_system = 0, target_component = 0, frame = 0, current = 0, autocontinue = 0;
    template <class V> void wire(V& v) { v(param); v(x); v(y); v(z); v(command); v(target_system); v(target_component); v(frame); v(current); v(autocontinue); }
};
struct CommandAck {
    static constexpr uint32_t kId = MSG_COMMAND_ACK;
    uint16_t command = 0; uint8_t result = 0, progress = 0; int32_t result_param2 = 0; uint8_t target_system = 0, target_component = 0;
    template <class V> void wire(V& v) { v(command); v(result); v(progress); v(result_param2); v(target_system); v(target_component); }
};
struct AutopilotVersion {
    static constexpr uint32_t kId = MSG_AUTOPILOT_VERSION;
    uint64_t capabilities = 0, uid = 0; uint32_t flight_sw_version = 0, middleware_sw_version = 0, os_sw_version = 0, board_version = 0;
    uint16_t vendor_id = 0, product_id = 0; uint8_t flight_custom_version[8] = {}, middleware_custom_version[8] = {}, os_custom_version[8] = {}, uid2[18] = {};
    template <class V> void wire(V& v) { v(capabilities); v(uid); v(flight_sw_version); v(middleware_sw_version); v(os_sw_version); v(board_version); v(vendor_id); v(product_id);
                                         v(flight_custom_version); v(middleware_custom_version); v(os_custom_version); v(uid2); }
};
struct HomePosition {                                   // degE7, mm MSL; the local frame fields stay 0 (q = no rotation)
    static constexpr uint32_t kId = MSG_HOME_POSITION;
    int32_t latitude = 0, longitude = 0, altitude = 0; float x = 0, y = 0, z = 0, q[4] = {1, 0, 0, 0}, approach_x = 0, approach_y = 0, approach_z = 0; uint64_t time_usec = 0;
    template <class V> void wire(V& v) { v(latitude); v(longitude); v(altitude); v(x); v(y); v(z); v(q); v(approach_x); v(approach_y); v(approach_z); v(time_usec); }
};
struct GpsGlobalOrigin {
    static constexpr uint32_t kId = MSG_GPS_GLOBAL_ORIGIN;
    int32_t latitude = 0, longitude = 0, altitude = 0; uint64_t time_usec = 0;
    template <class V> void wire(V& v) { v(latitude); v(longitude); v(altitude); v(time_usec); }
};
struct AvailableModes {                                 // (development.xml in pymavlink 2.4.50; common.xml in newer MAVLink)
    static constexpr uint32_t kId = MSG_AVAILABLE_MODES;
    uint32_t custom_mode = 0, properties = 0; uint8_t number_modes = 0, mode_index = 0, standard_mode = 0; char mode_name[35] = {};
    template <class V> void wire(V& v) { v(custom_mode); v(properties); v(number_modes); v(mode_index); v(standard_mode); v(mode_name); }
};
struct AvailableModesMonitor {
    static constexpr uint32_t kId = MSG_AVAILABLE_MODES_MONITOR;
    uint8_t seq = 0;
    template <class V> void wire(V& v) { v(seq); }
};
struct MissionRequestList {
    static constexpr uint32_t kId = MSG_MISSION_REQUEST_LIST;
    uint8_t target_system = 0, target_component = 0, mission_type = 0;
    template <class V> void wire(V& v) { v(target_system); v(target_component); v(mission_type); }
};
struct MissionCount {
    static constexpr uint32_t kId = MSG_MISSION_COUNT;
    uint16_t count = 0; uint8_t target_system = 0, target_component = 0, mission_type = 0;
    template <class V> void wire(V& v) { v(count); v(target_system); v(target_component); v(mission_type); }
};
struct MissionRequestInt {
    static constexpr uint32_t kId = MSG_MISSION_REQUEST_INT;
    uint16_t seq = 0; uint8_t target_system = 0, target_component = 0, mission_type = 0;
    template <class V> void wire(V& v) { v(seq); v(target_system); v(target_component); v(mission_type); }
};
struct MissionItemInt {                                 // x, y: degE7; z: m in the frame's sense
    static constexpr uint32_t kId = MSG_MISSION_ITEM_INT;
    float param[4] = {}; int32_t x = 0, y = 0; float z = 0; uint16_t seq = 0, command = 0;
    uint8_t target_system = 0, target_component = 0, frame = 0, current = 0, autocontinue = 1, mission_type = 0;
    template <class V> void wire(V& v) { v(param); v(x); v(y); v(z); v(seq); v(command); v(target_system); v(target_component); v(frame); v(current); v(autocontinue); v(mission_type); }
};
struct MissionItem {                                    // the float form: x, y in degrees (deprecated, still what a ground station sends to a craft
    static constexpr uint32_t kId = MSG_MISSION_ITEM;    // that has not said MAV_PROTOCOL_CAPABILITY_MISSION_INT)
    float param[4] = {}; float x = 0, y = 0, z = 0; uint16_t seq = 0, command = 0;
    uint8_t target_system = 0, target_component = 0, frame = 0, current = 0, autocontinue = 1, mission_type = 0;
    template <class V> void wire(V& v) { v(param); v(x); v(y); v(z); v(seq); v(command); v(target_system); v(target_component); v(frame); v(current); v(autocontinue); v(mission_type); }
};
struct MissionRequest {
    static constexpr uint32_t kId = MSG_MISSION_REQUEST;
    uint16_t seq = 0; uint8_t target_system = 0, target_component = 0, mission_type = 0;
    template <class V> void wire(V& v) { v(seq); v(target_system); v(target_component); v(mission_type); }
};
struct MissionAck {
    static constexpr uint32_t kId = MSG_MISSION_ACK;
    uint8_t target_system = 0, target_component = 0, type = 0, mission_type = 0;
    template <class V> void wire(V& v) { v(target_system); v(target_component); v(type); v(mission_type); }
};
struct MissionClearAll {
    static constexpr uint32_t kId = MSG_MISSION_CLEAR_ALL;
    uint8_t target_system = 0, target_component = 0, mission_type = 0;
    template <class V> void wire(V& v) { v(target_system); v(target_component); v(mission_type); }
};
struct MissionSetCurrent {
    static constexpr uint32_t kId = MSG_MISSION_SET_CURRENT;
    uint16_t seq = 0; uint8_t target_system = 0, target_component = 0;
    template <class V> void wire(V& v) { v(seq); v(target_system); v(target_component); }
};
struct MissionCurrent {
    static constexpr uint32_t kId = MSG_MISSION_CURRENT;
    uint16_t seq = 0, total = 0; uint8_t mission_state = 0, mission_mode = 0;
    template <class V> void wire(V& v) { v(seq); v(total); v(mission_state); v(mission_mode); }
};
struct MissionItemReached {
    static constexpr uint32_t kId = MSG_MISSION_ITEM_REACHED;
    uint16_t seq = 0;
    template <class V> void wire(V& v) { v(seq); }
};
// 0.43: the parameter protocol (pymavlink 2.4.50: CRC_EXTRA 214 / 159 / 220 / 168). param_id: 16 characters, NUL-padded (no NUL when
// all 16 are used). The values travel as REAL32 (MAV_PARAM_TYPE 9) in the units the app shows.
constexpr uint8_t PARAM_TYPE_REAL32 = 9;
struct ParamRequestRead {
    static constexpr uint32_t kId = MSG_PARAM_REQUEST_READ;
    int16_t param_index = -1; uint8_t target_system = 0, target_component = 0; char param_id[16] = {};
    template <class V> void wire(V& v) { v(param_index); v(target_system); v(target_component); v(param_id); }
};
struct ParamRequestList {
    static constexpr uint32_t kId = MSG_PARAM_REQUEST_LIST;
    uint8_t target_system = 0, target_component = 0;
    template <class V> void wire(V& v) { v(target_system); v(target_component); }
};
struct ParamValue {
    static constexpr uint32_t kId = MSG_PARAM_VALUE;
    float param_value = 0; uint16_t param_count = 0, param_index = 0; char param_id[16] = {}; uint8_t param_type = PARAM_TYPE_REAL32;
    template <class V> void wire(V& v) { v(param_value); v(param_count); v(param_index); v(param_id); v(param_type); }
};
struct ParamSet {
    static constexpr uint32_t kId = MSG_PARAM_SET;
    float param_value = 0; uint8_t target_system = 0, target_component = 0; char param_id[16] = {}; uint8_t param_type = PARAM_TYPE_REAL32;
    template <class V> void wire(V& v) { v(param_value); v(target_system); v(target_component); v(param_id); v(param_type); }
};
struct Timesync {                                       // ns
    static constexpr uint32_t kId = MSG_TIMESYNC;
    int64_t tc1 = 0, ts1 = 0;
    template <class V> void wire(V& v) { v(tc1); v(ts1); }
};

} // namespace mav
