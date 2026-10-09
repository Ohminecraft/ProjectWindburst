/*
 * This file is part of Windburst Project
 */

#pragma once

#include <string.h>
#include "flow_driver.h"

// ============================================================================
// Optical flow sensors that talk MSP v2 on a UART: MicoAir MTF-01 / MTF-02 (and the Matek 3901-L0X family). They send by
// themselves, 50 .. 100 times a second, and carry their own lidar:
//
//   frame    '$' 'X' '<'  flags(1)  command(2, LE)  size(2, LE)  payload  crc8        crc8 = DVB-S2 (poly 0xD5) over flags .. payload
//   0x1F01   range:  quality u8, distance_mm i32 (negative = out of range)                          5 bytes
//   0x1F02   flow:   quality u8 (0..255), motion_x i32, motion_y i32                                9 bytes
//
// Scaling as in Betaflight's driver for these sensors (drivers/rangefinder/rangefinder_lidarmt.c): rate = motion / 200 / dt
// rad/s, dt = the time between two flow frames as they arrive; quality x 100 / 255. The numbers the sensor sends describe a
// window of about 40 ms that ended about 45 ms before the frame arrives (Betaflight: measured, "gyro delay 65 ms").
// One count of `motion` is 5 mrad: the frames of about 50 ms are summed into one reading (flow_driver.h, Accumulator).
// A flow frame after a pause (dt above 1 s) only restarts the timing.
// ============================================================================
namespace flow {

inline uint8_t crc8_dvb_s2(uint8_t crc, uint8_t v) {
    crc ^= v;
    for (int i = 0; i < 8; i++) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
    return crc;
}

// MSP v2 frames out of a byte stream. One byte at a time; true when a frame with a good checksum is complete.
class MspV2Parser {
public:
    static constexpr uint16_t kMaxPayload = 64;                // (the two messages we know are 5 and 9 bytes long)
    bool feed(uint8_t c) {
        switch (st_) {
        case 0: if (c == '$') st_ = 1; return false;
        case 1: st_ = c == 'X' ? 2 : (c == '$' ? 1 : 0); return false;
        case 2: st_ = (c == '<' || c == '>') ? 3 : (c == '$' ? 1 : 0); n_ = 0; crc_ = 0; return false;
        case 3:                                                // flags, command (2), size (2)
            crc_ = crc8_dvb_s2(crc_, c); hdr_[n_++] = c;
            if (n_ == 5) {
                cmd_ = (uint16_t)(hdr_[1] | (hdr_[2] << 8)); size_ = (uint16_t)(hdr_[3] | (hdr_[4] << 8)); n_ = 0;
                if (size_ > kMaxPayload) { st_ = 0; too_long_++; return false; }
                st_ = size_ ? 4 : 5;
            }
            return false;
        case 4: crc_ = crc8_dvb_s2(crc_, c); payload_[n_++] = c; if (n_ >= size_) st_ = 5; return false;
        default:
            st_ = 0;
            if (c != crc_) { crc_errors_++; return false; }
            frames_++;
            return true;
        }
    }
    uint16_t command() const { return cmd_; }
    uint16_t size() const { return size_; }
    const uint8_t* payload() const { return payload_; }
    uint32_t frames() const { return frames_; }
    uint32_t crc_errors() const { return crc_errors_; }
    uint32_t too_long() const { return too_long_; }
private:
    uint8_t st_ = 0, hdr_[5] = {}, payload_[kMaxPayload] = {}, crc_ = 0; uint16_t n_ = 0, cmd_ = 0, size_ = 0;
    uint32_t frames_ = 0, crc_errors_ = 0, too_long_ = 0;
};

class BytePort {                                               // the UART, as far as this driver needs it
public:
    virtual ~BytePort() = default;
    virtual int read() = 0;                                    // -1: nothing waiting
};

class MspFlow : public Driver {
public:
    static constexpr uint16_t kCmdRange = 0x1F01, kCmdFlow = 0x1F02;
    static constexpr float kWindowS = 0.040f, kAgeS = 0.045f;
    static constexpr int kMaxBytesPerCall = 256;               // (a call never works through more than this: the thread has other sensors)

    explicit MspFlow(BytePort& port) : port_(port) {}
    const char* name() const override { return "MSP flow"; }
    float nominal_rate_hz() const override { return 20.0f; }
    uint32_t samples() const override { return flow_frames_; }
    bool init() override { have_prev_ = false; range_ = RangeSample(); sum_.reset(); return true; }      // (the sensor is not asked anything: whether it is there shows when frames arrive)

    // Returns true when a new FLOW reading is in `out`. A new range, if any, waits in take_range().
    bool update(uint32_t now_us, Reading& out) override {
        bool got = false;
        for (int k = 0; k < kMaxBytesPerCall; k++) {
            const int c = port_.read();
            if (c < 0) break;
            if (!parser_.feed((uint8_t)c)) continue;
            const uint8_t* p = parser_.payload();
            if (parser_.command() == kCmdRange && parser_.size() >= 5) {
                int32_t mm; memcpy(&mm, p + 1, 4);
                range_.fresh = mm >= 0; range_.distance_m = mm >= 0 ? mm * 0.001f : 0.0f; range_.quality_pct = (uint8_t)((uint32_t)p[0] * 100u / 255u);
                range_frames_++;
            } else if (parser_.command() == kCmdFlow && parser_.size() >= 9) {
                int32_t mx, my; memcpy(&mx, p + 1, 4); memcpy(&my, p + 5, 4);
                // Frames that were waiting together (the thread came late) share one arrival time: the first carries the whole
                // pause, the others none - summed, their counts and that time still belong together.
                const uint32_t dt_us = now_us - prev_us_;
                const bool timed = have_prev_ && dt_us < 1000000u;
                prev_us_ = now_us; have_prev_ = true; flow_frames_++;
                if (!timed) { sum_.reset(); continue; }
                sum_.add((float)mx / 200.0f, (float)my / 200.0f, dt_us, (uint8_t)((uint32_t)p[0] * 100u / 255u));
                if (!sum_.ready()) continue;
                const float per_frame = sum_.seconds() / (float)sum_.count();
                sum_.take(out);
                out.dt += kWindowS - per_frame;                // the first frame's window began that much before its own share of the sum
                out.age_s = kAgeS;
                got = true;
            }
        }
        return got;
    }
    // The lidar of the same sensor: true once per new distance.
    bool take_range(RangeSample& r) { if (!range_.fresh) return false; r = range_; range_.fresh = false; return true; }
    const MspV2Parser& parser() const { return parser_; }
    uint32_t flow_frames() const { return flow_frames_; }
    uint32_t range_frames() const { return range_frames_; }

private:
    BytePort& port_; MspV2Parser parser_; Accumulator sum_;
    bool have_prev_ = false; uint32_t prev_us_ = 0, flow_frames_ = 0, range_frames_ = 0;
    RangeSample range_;
};

} // namespace flow
