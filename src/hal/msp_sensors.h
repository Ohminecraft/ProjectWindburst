/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>
#include <string.h>
#include <math.h>
#include "flow/flow_msp.h"
#include "gps/gps_types.h"
#include "rx/rx_proto.h"

// ============================================================================
// The MSP port: one UART that MSP devices send to by themselves - ArduPilot's SERIALn_PROTOCOL = MSP (AP_MSP), INAV's MSP
// sensors. One module may carry several sensors (a Matek M10Q-5883 "MSP" GPS: GPS + compass; an MTF-01: flow + lidar), so the
// port is read in one place and each frame goes where it belongs:
//
//   0x1F01  MSP2_SENSOR_RANGEFINDER    the flow sensor's lidar        (flow sensor, hal/flow/flow_msp.h)
//   0x1F02  MSP2_SENSOR_OPTIC_FLOW     optical flow                   (flow sensor)
//   0x1F03  MSP2_SENSOR_GPS            a fix, 52 bytes                -> take_gps()      (gps.provider MSP)
//   0x1F04  MSP2_SENSOR_COMPASS        field, milligauss, FRD         -> take_mag()      (sensors.mag_model MSP)
//   0x1F05  MSP2_SENSOR_BARO           pressure Pa (float), 0.01 C    -> take_baro()     (sensors.baro_model MSP)
//   200     MSP_SET_RAW_RC (v1 or v2)  channels, microseconds         -> rc()            (receiver.protocol MSP)
//
// Layouts: INAV msp_protocol_v2_sensor_msg.h (the senders' own definition), read by ArduPilot AP_MSP the same way. A GPS fix
// type of 2 is 2D, 3 and above 3D (INAV gpsMapFixType, ArduPilot AP_GPS_MSP). MSP_SET_RAW_RC as Betaflight rx_msp.c: up to 18
// channels of u16; a receiver that stops sending is a lost link after rx::Link's timeout, like any other receiver.
// Nothing is ever sent back: these devices do not ask.
// Portable: unit-tested in test/test_msp_sensors.cpp, on the virtual board in test_fw (group msp).
// ============================================================================
namespace mspdev {

constexpr uint16_t kRange = 0x1F01, kFlow = 0x1F02, kGps = 0x1F03, kCompass = 0x1F04, kBaro = 0x1F05, kSetRawRc = 200;

struct MagSample { float frd_ga[3] = {0, 0, 0}; };
struct BaroSample { float pa = 0, temp_c = 0; };

class Port {
public:
    // A frame for the flow sensor (true = it took it); nullptr when there is none on this port.
    using FlowFn = bool (*)(void* ctx, uint16_t cmd, const uint8_t* p, uint16_t n, uint32_t now_us);
    static constexpr int kMaxBytesPerCall = 512;                       // (the sensor thread has other sensors to serve)

    void attach(flow::BytePort* port) { port_ = port; }
    void setFlow(FlowFn fn, void* ctx) { flow_fn_ = fn; flow_ctx_ = ctx; }

    // Sensor thread, about once a millisecond: works through what came in.
    void poll(uint32_t now_us, uint32_t now_ms) {
        rc_link_.tick(now_ms);
        for (int k = 0; port_ && k < kMaxBytesPerCall; k++) {
            const int c = port_->read();
            if (c < 0) break;
            if (parser_.feed((uint8_t)c)) handle(parser_.command(), parser_.payload(), parser_.size(), now_us, now_ms);
        }
    }
    // One whole frame (poll() calls it; the tests too).
    void handle(uint16_t cmd, const uint8_t* p, uint16_t n, uint32_t now_us, uint32_t now_ms) {
        frames_++;
        if (cmd == kRange || cmd == kFlow) { if (flow_fn_ && flow_fn_(flow_ctx_, cmd, p, n, now_us)) flow_frames_++; else unused_++; return; }
        if (cmd == kGps && n >= 52) { gps_ = decode_gps(p); gps_fresh_ = true; gps_frames_++; return; }
        if (cmd == kCompass && n >= 11) {
            for (int k = 0; k < 3; k++) { int16_t v; memcpy(&v, p + 5 + 2 * k, 2); mag_.frd_ga[k] = v * 0.001f; }
            mag_fresh_ = true; mag_frames_++; return;
        }
        if (cmd == kBaro && n >= 11) {
            float pa; int16_t t; memcpy(&pa, p + 5, 4); memcpy(&t, p + 9, 2);
            if (!(pa > 10000.0f && pa < 120000.0f)) { unused_++; return; }          // (not a pressure an aircraft sees: a broken frame)
            baro_.pa = pa; baro_.temp_c = t * 0.01f; baro_fresh_ = true; baro_frames_++; return;
        }
        if (cmd == kSetRawRc && n >= 2) {
            rx::Frame f; const int ch = n / 2 < rx::kChannels ? n / 2 : rx::kChannels;
            for (int i = 0; i < ch; i++) { uint16_t v; memcpy(&v, p + 2 * i, 2); f.ch[i] = v < 750 ? 750 : v > 2250 ? 2250 : v; }
            for (int i = ch; i < rx::kChannels; i++) f.ch[i] = rc_link_.channels()[i];   // (channels it does not send keep their value)
            rc_link_.on_frame(f, now_ms); rc_fresh_ = true; rc_frames_++; return;
        }
        unused_++;
    }

    bool take_gps(gps::Solution& s) { if (!gps_fresh_) return false; gps_fresh_ = false; s = gps_; return true; }
    bool take_mag(MagSample& m) { if (!mag_fresh_) return false; mag_fresh_ = false; m = mag_; return true; }
    bool take_baro(BaroSample& b) { if (!baro_fresh_) return false; baro_fresh_ = false; b = baro_; return true; }
    bool take_rc() { const bool f = rc_fresh_; rc_fresh_ = false; return f; }       // a new MSP_SET_RAW_RC since the last call
    const rx::Link& rc() const { return rc_link_; }

    // A fix as INAV / ArduPilot send it (mspSensorGpsDataMessage_t, packed, little endian).
    static gps::Solution decode_gps(const uint8_t* p) {
        auto u16 = [p](int o) { uint16_t v; memcpy(&v, p + o, 2); return v; };
        auto i32 = [p](int o) { int32_t v; memcpy(&v, p + o, 4); return v; };
        auto cl16 = [](int32_t v) { return (int16_t)(v < -32767 ? -32767 : v > 32767 ? 32767 : v); };
        gps::Solution s;
        uint32_t tow; memcpy(&tow, p + 3, 4); s.time_ms = tow;
        const uint8_t ft = p[7];
        s.fix_type = ft >= 3 ? 3 : ft == 2 ? 2 : 0; s.fix = ft >= 3;
        s.num_sat = p[8];
        s.h_acc_mm = (uint32_t)u16(9) * 10u; s.v_acc_mm = (uint32_t)u16(11) * 10u; s.s_acc_mm = (uint32_t)u16(13) * 10u;
        s.hdop = u16(15);
        s.lon = i32(17); s.lat = i32(21); s.alt_cm = i32(25);
        const int32_t vn = i32(29), ve = i32(33), vd = i32(37);
        s.vel_n_cms = cl16(vn); s.vel_e_cms = cl16(ve); s.vel_d_cms = cl16(vd); s.has_vel_d = true;
        const float gs = sqrtf((float)vn * vn + (float)ve * ve), s3 = sqrtf(gs * gs + (float)vd * vd);
        s.ground_speed_cms = (uint16_t)(gs > 65535.0f ? 65535.0f : gs); s.speed3d_cms = (uint16_t)(s3 > 65535.0f ? 65535.0f : s3);
        s.course_ddeg = (uint16_t)((u16(41) / 10u) % 3600u);
        const uint16_t yr = u16(45);
        s.date.year = yr; s.date.month = p[47]; s.date.day = p[48]; s.date.hour = p[49]; s.date.minute = p[50]; s.date.second = p[51];
        s.date.valid = yr >= 2000 && p[47] >= 1 && p[47] <= 12 && p[48] >= 1;
        return s;
    }

    uint32_t frames() const { return frames_; }
    uint32_t gps_frames() const { return gps_frames_; }
    uint32_t mag_frames() const { return mag_frames_; }
    uint32_t baro_frames() const { return baro_frames_; }
    uint32_t rc_frames() const { return rc_frames_; }
    uint32_t flow_frames() const { return flow_frames_; }
    uint32_t unused() const { return unused_; }                         // good frames nobody here takes (an MSP sensor this firmware has no use for)
    uint32_t crc_errors() const { return parser_.crc_errors(); }

private:
    flow::BytePort* port_ = nullptr; flow::MspV2Parser parser_;
    FlowFn flow_fn_ = nullptr; void* flow_ctx_ = nullptr;
    gps::Solution gps_; MagSample mag_; BaroSample baro_; rx::Link rc_link_;
    bool gps_fresh_ = false, mag_fresh_ = false, baro_fresh_ = false, rc_fresh_ = false;
    uint32_t frames_ = 0, gps_frames_ = 0, mag_frames_ = 0, baro_frames_ = 0, rc_frames_ = 0, flow_frames_ = 0, unused_ = 0;
};

} // namespace mspdev
