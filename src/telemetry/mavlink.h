/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <atomic>

// ============================================================================
// MAVLink 2, the few messages a ground station needs to show the craft (send only): HEARTBEAT, SYS_STATUS, GPS_RAW_INT, ATTITUDE,
// GLOBAL_POSITION_INT, RC_CHANNELS, VFR_HUD, BATTERY_STATUS, STATUSTEXT (common.xml).
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
    MSG_RC_CHANNELS = 65, MSG_VFR_HUD = 74, MSG_BATTERY_STATUS = 147, MSG_STATUSTEXT = 253,
};
// CRC_EXTRA of each message (from its definition; pymavlink: MAVLink_<name>_message.crc_extra)
inline uint8_t crc_extra(uint32_t id) {
    switch (id) {
    case MSG_HEARTBEAT: return 50; case MSG_SYS_STATUS: return 124; case MSG_GPS_RAW_INT: return 24; case MSG_ATTITUDE: return 39;
    case MSG_GLOBAL_POSITION_INT: return 104; case MSG_RC_CHANNELS: return 118; case MSG_VFR_HUD: return 20; case MSG_BATTERY_STATUS: return 154;
    case MSG_STATUSTEXT: return 83; default: return 0;
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
enum : uint32_t { COPTER_STABILIZE = 0, COPTER_ACRO = 1, COPTER_ALT_HOLD = 2, COPTER_AUTO = 3, COPTER_RTL = 6, COPTER_LAND = 9, COPTER_POSHOLD = 16 };

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

// ---- receiving: whole frames out of a byte stream (CRC checked, payload zero-filled to its full size again). Used by the tests
// and the simulator to read what the firmware sends; ready for a later "receive" side.
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

// ---- frames from the sensor thread to the WiFi side (link task, UDP): single producer, single consumer, whole frames only
class Ring {
public:
    static constexpr size_t kSize = 2048;
    // false: no room for the whole frame (it is dropped and counted: the radio is slower than the telemetry)
    bool push(const uint8_t* d, size_t n) {
        if (n == 0 || n > kMaxFrame) return false;
        const size_t h = head_.load(std::memory_order_relaxed), t = tail_.load(std::memory_order_acquire);
        if (kSize - (h - t) < n + 2) { dropped_++; return false; }
        put(h, (uint8_t)(n & 0xFF)); put(h + 1, (uint8_t)(n >> 8));
        for (size_t i = 0; i < n; i++) put(h + 2 + i, d[i]);
        head_.store(h + 2 + n, std::memory_order_release);
        return true;
    }
    // as many whole frames as fit into cap bytes (one UDP datagram); 0 = nothing waiting
    size_t pop(uint8_t* out, size_t cap) {
        size_t t = tail_.load(std::memory_order_relaxed), got = 0;
        const size_t h = head_.load(std::memory_order_acquire);
        while (h - t >= 2) {
            const size_t n = (size_t)at(t) | ((size_t)at(t + 1) << 8);
            if (got + n > cap) break;
            for (size_t i = 0; i < n; i++) out[got + i] = at(t + 2 + i);
            got += n; t += 2 + n;
        }
        tail_.store(t, std::memory_order_release);
        return got;
    }
    uint32_t dropped() const { return dropped_; }
private:
    void put(size_t i, uint8_t v) { buf_[i % kSize] = v; }
    uint8_t at(size_t i) const { return buf_[i % kSize]; }
    uint8_t buf_[kSize] = {};
    std::atomic<size_t> head_{0}, tail_{0};
    uint32_t dropped_ = 0;
};

} // namespace mav
