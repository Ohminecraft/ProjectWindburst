/*
 * This file is part of Windburst Project
 */

#pragma once

#include "dronecan.h"

// ============================================================================
// The DroneCAN data types this firmware speaks, field by field as their DSDL definitions (pydronecan's dsdl_specs) say.
// Signatures and default ids from the same definitions. Encoders exist for what the flight controller sends and for what the
// simulated nodes of the tests send (a GPS / compass module).
// ============================================================================
namespace dcan {

// ---- uavcan.protocol.NodeStatus (message 341), 7 bytes
constexpr uint16_t kNodeStatusId = 341; constexpr uint64_t kNodeStatusSig = 0x0F0868D0C1A7C6F1ull;
enum Health : uint8_t { HEALTH_OK = 0, HEALTH_WARNING = 1, HEALTH_ERROR = 2, HEALTH_CRITICAL = 3 };
enum Mode : uint8_t { MODE_OPERATIONAL = 0, MODE_INITIALIZATION = 1, MODE_MAINTENANCE = 2, MODE_SOFTWARE_UPDATE = 3, MODE_OFFLINE = 7 };
struct NodeStatus { uint32_t uptime_sec = 0; uint8_t health = 0, mode = 0, sub_mode = 0; uint16_t vendor = 0; };
inline void put(Writer& w, const NodeStatus& s) { w.u(s.uptime_sec, 32); w.u(s.health, 2); w.u(s.mode, 3); w.u(s.sub_mode, 3); w.u(s.vendor, 16); }
inline NodeStatus get_node_status(Reader& r) { NodeStatus s; s.uptime_sec = (uint32_t)r.u(32); s.health = (uint8_t)r.u(2); s.mode = (uint8_t)r.u(3); s.sub_mode = (uint8_t)r.u(3); s.vendor = (uint16_t)r.u(16); return s; }
inline size_t encode(const NodeStatus& s, uint8_t* buf, size_t cap) { Writer w(buf, cap); put(w, s); return w.overflow() ? 0 : w.bytes(); }

// ---- uavcan.protocol.GetNodeInfo (service 1): empty request; the response
constexpr uint8_t kGetNodeInfoId = 1; constexpr uint64_t kGetNodeInfoSig = 0xEE468A8121C46A9Eull;
struct NodeInfo {
    NodeStatus status; uint8_t sw_major = 0, sw_minor = 0, sw_flags = 0; uint32_t vcs = 0; uint64_t image_crc = 0;
    uint8_t hw_major = 0, hw_minor = 0; uint8_t uid[16] = {}; char name[81] = {};
};
inline size_t encode(const NodeInfo& n, uint8_t* buf, size_t cap) {
    Writer w(buf, cap); put(w, n.status);
    w.u(n.sw_major, 8); w.u(n.sw_minor, 8); w.u(n.sw_flags, 8); w.u(n.vcs, 32); w.u(n.image_crc, 64);
    w.u(n.hw_major, 8); w.u(n.hw_minor, 8); for (int i = 0; i < 16; i++) w.u(n.uid[i], 8);
    w.u(0, 8);                                                       // certificate_of_authenticity: none (a length prefix: not the last field)
    for (size_t i = 0; i < 80 && n.name[i]; i++) w.u((uint8_t)n.name[i], 8);     // the name: the last field, no length prefix
    return w.overflow() ? 0 : w.bytes();
}
inline bool decode(const uint8_t* p, size_t n, NodeInfo& o) {
    Reader r(p, n); o.status = get_node_status(r);
    o.sw_major = (uint8_t)r.u(8); o.sw_minor = (uint8_t)r.u(8); o.sw_flags = (uint8_t)r.u(8); o.vcs = (uint32_t)r.u(32); o.image_crc = r.u(64);
    o.hw_major = (uint8_t)r.u(8); o.hw_minor = (uint8_t)r.u(8); for (int i = 0; i < 16; i++) o.uid[i] = (uint8_t)r.u(8);
    const int cert = (int)r.u(8); r.skip(cert * 8);
    if (r.too_short()) return false;
    size_t k = 0; while (k < 80 && r.bits_left() >= 8) o.name[k++] = (char)r.u(8);
    o.name[k] = 0;
    return true;
}

// ---- uavcan.protocol.dynamic_node_id.Allocation (message 1)
constexpr uint16_t kAllocationId = 1; constexpr uint64_t kAllocationSig = 0x0B2A812620A11D40ull;
struct Allocation { uint8_t node_id = 0; bool first_part = false; uint8_t uid_len = 0; uint8_t uid[16] = {}; };
inline size_t encode(const Allocation& a, uint8_t* buf, size_t cap) {
    Writer w(buf, cap); w.u(a.node_id, 7); w.boolean(a.first_part); for (int i = 0; i < a.uid_len && i < 16; i++) w.u(a.uid[i], 8);    // (unique_id: the last field)
    return w.overflow() ? 0 : w.bytes();
}
inline bool decode(const uint8_t* p, size_t n, Allocation& a) {
    Reader r(p, n); a.node_id = (uint8_t)r.u(7); a.first_part = r.boolean(); a.uid_len = 0;
    while (a.uid_len < 16 && r.bits_left() >= 8) a.uid[a.uid_len++] = (uint8_t)r.u(8);
    return !r.too_short();
}

// ---- uavcan.equipment.gnss.Fix2 (message 1063)
constexpr uint16_t kFix2Id = 1063; constexpr uint64_t kFix2Sig = 0xCA41E7000F37435Full;
enum GnssStatus : uint8_t { GNSS_NO_FIX = 0, GNSS_TIME_ONLY = 1, GNSS_2D = 2, GNSS_3D = 3 };
struct Fix2 {
    uint64_t timestamp_us = 0, gnss_timestamp_us = 0; uint8_t time_standard = 0, leap_seconds = 0;
    int64_t lon_1e8 = 0, lat_1e8 = 0; int32_t height_ellipsoid_mm = 0, height_msl_mm = 0;
    float ned_velocity[3] = {}; uint8_t sats_used = 0, status = 0, mode = 0, sub_mode = 0;
    uint8_t cov_len = 0; float covariance[36] = {}; float pdop = 0;
};
inline size_t encode(const Fix2& f, uint8_t* buf, size_t cap) {
    Writer w(buf, cap);
    w.u(f.timestamp_us, 56); w.u(f.gnss_timestamp_us, 56); w.u(f.time_standard, 3); w.skip(13); w.u(f.leap_seconds, 8);
    w.i(f.lon_1e8, 37); w.i(f.lat_1e8, 37); w.i(f.height_ellipsoid_mm, 27); w.i(f.height_msl_mm, 27);
    for (int k = 0; k < 3; k++) w.f32(f.ned_velocity[k]);
    w.u(f.sats_used, 6); w.u(f.status, 2); w.u(f.mode, 4); w.u(f.sub_mode, 6);
    w.u(f.cov_len, 6); for (int k = 0; k < f.cov_len && k < 36; k++) w.f16(f.covariance[k]);
    w.f16(f.pdop);                                                   // (ecef_position_velocity: none - the empty tail array)
    return w.overflow() ? 0 : w.bytes();
}
inline bool decode(const uint8_t* p, size_t n, Fix2& f) {
    Reader r(p, n);
    f.timestamp_us = r.u(56); f.gnss_timestamp_us = r.u(56); f.time_standard = (uint8_t)r.u(3); r.skip(13); f.leap_seconds = (uint8_t)r.u(8);
    f.lon_1e8 = r.i(37); f.lat_1e8 = r.i(37); f.height_ellipsoid_mm = (int32_t)r.i(27); f.height_msl_mm = (int32_t)r.i(27);
    for (int k = 0; k < 3; k++) f.ned_velocity[k] = r.f32();
    f.sats_used = (uint8_t)r.u(6); f.status = (uint8_t)r.u(2); f.mode = (uint8_t)r.u(4); f.sub_mode = (uint8_t)r.u(6);
    f.cov_len = (uint8_t)r.u(6); if (f.cov_len > 36) return false;
    for (int k = 0; k < f.cov_len; k++) f.covariance[k] = r.f16();
    f.pdop = r.f16();
    return !r.too_short();
}

// ---- uavcan.equipment.gnss.Auxiliary (message 1061): the dilutions of precision
constexpr uint16_t kGnssAuxId = 1061; constexpr uint64_t kGnssAuxSig = 0x9BE8BDC4C3DBBFD2ull;
struct GnssAux { float gdop = 0, pdop = 0, hdop = 0, vdop = 0, tdop = 0, ndop = 0, edop = 0; uint8_t sats_visible = 0, sats_used = 0; };
inline size_t encode(const GnssAux& a, uint8_t* buf, size_t cap) {
    Writer w(buf, cap); w.f16(a.gdop); w.f16(a.pdop); w.f16(a.hdop); w.f16(a.vdop); w.f16(a.tdop); w.f16(a.ndop); w.f16(a.edop); w.u(a.sats_visible, 7); w.u(a.sats_used, 6);
    return w.overflow() ? 0 : w.bytes();
}
inline bool decode(const uint8_t* p, size_t n, GnssAux& a) {
    Reader r(p, n); a.gdop = r.f16(); a.pdop = r.f16(); a.hdop = r.f16(); a.vdop = r.f16(); a.tdop = r.f16(); a.ndop = r.f16(); a.edop = r.f16(); a.sats_visible = (uint8_t)r.u(7); a.sats_used = (uint8_t)r.u(6);
    return !r.too_short();
}

// ---- uavcan.equipment.ahrs.MagneticFieldStrength2 (message 1002): gauss, the module's frame (x forward, y right, z down)
constexpr uint16_t kMag2Id = 1002; constexpr uint64_t kMag2Sig = 0xB6AC0C442430297Eull;
struct Mag2 { uint8_t sensor_id = 0; float field_ga[3] = {}; };
inline size_t encode(const Mag2& m, uint8_t* buf, size_t cap) {
    Writer w(buf, cap); w.u(m.sensor_id, 8); for (int k = 0; k < 3; k++) w.f16(m.field_ga[k]);         // (covariance: none - the empty tail array)
    return w.overflow() ? 0 : w.bytes();
}
inline bool decode(const uint8_t* p, size_t n, Mag2& m) {
    Reader r(p, n); m.sensor_id = (uint8_t)r.u(8); for (int k = 0; k < 3; k++) m.field_ga[k] = r.f16();
    return !r.too_short();
}


// ---- uavcan.equipment.air_data.StaticPressure (1028) / StaticTemperature (1029)
constexpr uint16_t kPressureId = 1028; constexpr uint64_t kPressureSig = 0xCDC7C43412BDC89Aull;
constexpr uint16_t kTemperatureId = 1029; constexpr uint64_t kTemperatureSig = 0x49272A6477D96271ull;
struct Pressure { float pa = 0, variance = 0; };
struct Temperature { float kelvin = 0, variance = 0; };
inline size_t encode(const Pressure& v, uint8_t* buf, size_t cap) { Writer w(buf, cap); w.f32(v.pa); w.f16(v.variance); return w.overflow() ? 0 : w.bytes(); }
inline bool decode(const uint8_t* p, size_t n, Pressure& v) { Reader r(p, n); v.pa = r.f32(); v.variance = r.f16(); return !r.too_short(); }
inline size_t encode(const Temperature& v, uint8_t* buf, size_t cap) { Writer w(buf, cap); w.f16(v.kelvin); w.f16(v.variance); return w.overflow() ? 0 : w.bytes(); }
inline bool decode(const uint8_t* p, size_t n, Temperature& v) { Reader r(p, n); v.kelvin = r.f16(); v.variance = r.f16(); return !r.too_short(); }

// ---- uavcan.equipment.power.BatteryInfo (1092)
constexpr uint16_t kBatteryId = 1092; constexpr uint64_t kBatterySig = 0x249C26548A711966ull;
struct Battery {
    float temperature_k = 0, voltage = 0, current = 0, power_10s = 0, remaining_wh = 0, full_wh = 0, hours_to_full = 0;
    uint16_t status_flags = 0; uint8_t soh_pct = 0, soc_pct = 0, soc_stdev = 0, battery_id = 0; uint32_t model_instance = 0; char model[32] = {};
};
inline size_t encode(const Battery& b, uint8_t* buf, size_t cap) {
    Writer w(buf, cap); w.f16(b.temperature_k); w.f16(b.voltage); w.f16(b.current); w.f16(b.power_10s); w.f16(b.remaining_wh); w.f16(b.full_wh); w.f16(b.hours_to_full);
    w.u(b.status_flags, 11); w.u(b.soh_pct, 7); w.u(b.soc_pct, 7); w.u(b.soc_stdev, 7); w.u(b.battery_id, 8); w.u(b.model_instance, 32);
    for (int i = 0; i < 31 && b.model[i]; i++) w.u((uint8_t)b.model[i], 8);          // (the last field: no length prefix)
    return w.overflow() ? 0 : w.bytes();
}
inline bool decode(const uint8_t* p, size_t n, Battery& b) {
    Reader r(p, n); b.temperature_k = r.f16(); b.voltage = r.f16(); b.current = r.f16(); b.power_10s = r.f16(); b.remaining_wh = r.f16(); b.full_wh = r.f16(); b.hours_to_full = r.f16();
    b.status_flags = (uint16_t)r.u(11); b.soh_pct = (uint8_t)r.u(7); b.soc_pct = (uint8_t)r.u(7); b.soc_stdev = (uint8_t)r.u(7); b.battery_id = (uint8_t)r.u(8); b.model_instance = (uint32_t)r.u(32);
    if (r.too_short()) return false;
    int k = 0; while (k < 31 && r.bits_left() >= 8) b.model[k++] = (char)r.u(8);
    b.model[k] = 0; return true;
}

// ---- uavcan.equipment.esc.RawCommand (1030): -8191 .. 8191 per ESC (negative = reverse, 0 = stop); Status (1034)
constexpr uint16_t kRawCommandId = 1030; constexpr uint64_t kRawCommandSig = 0x217F5C87D7EC951Dull;
constexpr uint16_t kEscStatusId = 1034; constexpr uint64_t kEscStatusSig = 0xA9AF28AEA2FBB254ull;
struct RawCommand { uint8_t n = 0; int16_t cmd[20] = {}; };
inline size_t encode(const RawCommand& c, uint8_t* buf, size_t cap) { Writer w(buf, cap); for (int i = 0; i < c.n && i < 20; i++) w.i(c.cmd[i], 14); return w.overflow() ? 0 : w.bytes(); }   // (the last field: no length)
inline bool decode(const uint8_t* p, size_t n, RawCommand& c) { Reader r(p, n); c.n = 0; while (c.n < 20 && r.bits_left() >= 14) c.cmd[c.n++] = (int16_t)r.i(14); return true; }
struct EscStatus { uint32_t error_count = 0; float voltage = 0, current = 0, temperature_k = 0; int32_t rpm = 0; uint8_t power_pct = 0, index = 0; };
inline size_t encode(const EscStatus& e, uint8_t* buf, size_t cap) {
    Writer w(buf, cap); w.u(e.error_count, 32); w.f16(e.voltage); w.f16(e.current); w.f16(e.temperature_k); w.i(e.rpm, 18); w.u(e.power_pct, 7); w.u(e.index, 5);
    return w.overflow() ? 0 : w.bytes();
}
inline bool decode(const uint8_t* p, size_t n, EscStatus& e) {
    Reader r(p, n); e.error_count = (uint32_t)r.u(32); e.voltage = r.f16(); e.current = r.f16(); e.temperature_k = r.f16(); e.rpm = (int32_t)r.i(18); e.power_pct = (uint8_t)r.u(7); e.index = (uint8_t)r.u(5);
    return !r.too_short();
}

} // namespace dcan
