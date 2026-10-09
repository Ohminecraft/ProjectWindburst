/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>
#include <string.h>
#include <math.h>
#include "rx_proto.h"

// ============================================================================
// CRSF telemetry (portable): the frames a flight controller sends back to a CRSF / ExpressLRS receiver, which passes them on to
// the transmitter (EdgeTX / OpenTX "Discover sensors": RxBt, Curr, Capa, Bat%, Ptch / Roll / Yaw, FM, GPS, Alt, Sats, VSpd).
// Layout and units as Betaflight's telemetry/crsf.c, so the transmitter shows the same numbers:
//
//   [0xC8][len][type][payload][crc8 DVB-S2 over type + payload], all multi-byte values big-endian
//   0x08 battery      voltage u16 (0.1 V), current u16 (0.1 A), used u24 (mAh), remaining u8 (%)
//   0x1E attitude     pitch, roll, yaw i16 (rad x 10000; yaw sent as -pi..pi)
//   0x21 flight mode  text + 0: "!FS!" failsafe, "RTH" rescue, "POSH", "ALTH", "ANGL", "STAB", "ACRO"; disarmed: "*" appended,
//                     or "!ERR" when something blocks arming
//   0x02 GPS          lat, lon i32 (deg x 1e7), ground speed u16 (km/h x 10), course u16 (deg x 100), altitude u16 (m + 1000),
//                     satellites u8 - only with a GPS module
//   0x07 vario        vertical speed i16 (cm/s) - only with an altitude estimate
// The whole list goes out ten times a second, one frame at a time (Betaflight: CRSF_CYCLETIME_US / number of frames).
// The receiver's UART is full duplex (ELRS, Crossfire): the frames go out on its TX pin whenever they are due.
// ============================================================================
namespace crsf {

enum FrameType : uint8_t { FT_GPS = 0x02, FT_VARIO = 0x07, FT_BATTERY = 0x08, FT_ATTITUDE = 0x1E, FT_FLIGHT_MODE = 0x21 };
constexpr uint8_t kSync = 0xC8;
constexpr uint32_t kCycleUs = 100000;
constexpr size_t kMaxFrame = 64;

struct TelemetryIn {
    float volts = 0, amps = 0, mah = 0; uint8_t remaining = 0;
    float roll_deg = 0, pitch_deg = 0, yaw_deg = 0;
    bool have_gps = false; bool gps_fix = false; int32_t lat = 0, lon = 0; uint16_t gs_cms = 0, course_ddeg = 0; float alt_m = 0; uint8_t sats = 0;
    bool have_vario = false; float vz_ms = 0;
    bool armed = false, arming_blocked = false, failsafe = false, rescue = false, poshold = false, althold = false, angle = false, stabilize = false;
};

inline const char* flight_mode(const TelemetryIn& t) {
    if (t.failsafe) return "!FS!";
    if (t.rescue) return "RTH";
    if (t.poshold) return "POSH";
    if (t.althold) return "ALTH";
    if (t.angle) return "ANGL";
    if (t.stabilize) return "STAB";
    return "ACRO";
}

class Writer {
public:
    explicit Writer(uint8_t* out) : p_(out) {}
    void u8(uint8_t v) { p_[n_++] = v; }
    void u16(uint16_t v) { u8((uint8_t)(v >> 8)); u8((uint8_t)v); }
    void u24(uint32_t v) { u8((uint8_t)(v >> 16)); u16((uint16_t)v); }
    void u32(uint32_t v) { u16((uint16_t)(v >> 16)); u16((uint16_t)v); }
    size_t n() const { return n_; }
    uint8_t* data() { return p_; }
private:
    uint8_t* p_; size_t n_ = 0;
};

inline int32_t clampi(float v, int32_t lo, int32_t hi) { return v != v ? 0 : v <= (float)lo ? lo : v >= (float)hi ? hi : (int32_t)lroundf(v); }

// Builds one frame of the given type into out (kMaxFrame bytes). Returns its length.
inline size_t build(uint8_t type, const TelemetryIn& t, uint8_t* out) {
    Writer w(out); w.u8(kSync); w.u8(0); w.u8(type);
    switch (type) {
    case FT_BATTERY:
        w.u16((uint16_t)clampi(t.volts * 10.0f, 0, 65535)); w.u16((uint16_t)clampi(t.amps * 10.0f, 0, 65535));
        w.u24((uint32_t)clampi(t.mah, 0, 0xFFFFFF)); w.u8(t.remaining > 100 ? 100 : t.remaining); break;
    case FT_ATTITUDE: {
        float yaw = fmodf(t.yaw_deg, 360.0f); if (yaw > 180.0f) yaw -= 360.0f; else if (yaw <= -180.0f) yaw += 360.0f;
        const float k = 10000.0f * 3.14159265f / 180.0f;
        w.u16((uint16_t)(int16_t)clampi(t.pitch_deg * k, -32768, 32767)); w.u16((uint16_t)(int16_t)clampi(t.roll_deg * k, -32768, 32767)); w.u16((uint16_t)(int16_t)clampi(yaw * k, -32768, 32767)); break; }
    case FT_FLIGHT_MODE: {
        const char* m = t.armed ? flight_mode(t) : t.arming_blocked ? "!ERR" : flight_mode(t);
        for (const char* c = m; *c; c++) w.u8((uint8_t)*c);
        if (!t.armed && !t.arming_blocked) w.u8('*');
        w.u8(0); break; }
    case FT_GPS:
        w.u32((uint32_t)(t.gps_fix ? t.lat : 0)); w.u32((uint32_t)(t.gps_fix ? t.lon : 0));
        w.u16((uint16_t)((t.gs_cms * 36u + 50u) / 100u)); w.u16((uint16_t)(t.course_ddeg * 10u));
        w.u16((uint16_t)clampi(t.alt_m + 1000.0f, 0, 65535)); w.u8(t.sats); break;
    case FT_VARIO: w.u16((uint16_t)(int16_t)clampi(t.vz_ms * 100.0f, -32768, 32767)); break;
    default: return 0;
    }
    out[1] = (uint8_t)(w.n() - 2 + 1);                       // type + payload + crc
    uint8_t crc = 0; for (size_t i = 2; i < w.n(); i++) crc = rx::crc8_dvb_s2(crc, out[i]);
    w.u8(crc);
    return w.n();
}

// One frame at a time, the whole list ten times a second.
class Scheduler {
public:
    void reset() { next_us_ = 0; index_ = 0; started_ = false; }
    // the frame due now (0 = nothing yet)
    size_t next(uint32_t now_us, const TelemetryIn& t, uint8_t* out) {
        uint8_t list[5]; int n = 0;
        list[n++] = FT_ATTITUDE; list[n++] = FT_BATTERY; list[n++] = FT_FLIGHT_MODE;
        if (t.have_gps) list[n++] = FT_GPS;
        if (t.have_vario) list[n++] = FT_VARIO;
        if (started_ && (int32_t)(now_us - next_us_) < 0) return 0;
        const uint32_t step = kCycleUs / (uint32_t)n;
        next_us_ = started_ && (uint32_t)(now_us - next_us_) < step ? next_us_ + step : now_us + step;     // (fell far behind: no burst)
        started_ = true;
        if (index_ >= n) index_ = 0;
        return build(list[index_++], t, out);
    }
private:
    uint32_t next_us_ = 0; int index_ = 0; bool started_ = false;
};

} // namespace crsf
