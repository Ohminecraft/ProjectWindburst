/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>
#include <stddef.h>

// ============================================================================
// Optical flow sensors - what every driver delivers (portable, no Arduino).
//
// A reading is the angular rate of the picture about the SENSOR's X and Y axes (rad/s, averaged over `dt`), in ArduPilot's
// convention: a right-handed rotation of the sensor about an axis reads positive. For a sensor in its normal mounting direction
// that means   x = roll rate - v_right / height,   y = pitch rate (nose up +) + v_forward / height.
// core/flow.h takes the gyro's part away and turns the rest into a velocity; the orientation settings (flow.align / flip) are
// applied there as well.
//
// Drivers are NON-BLOCKING: update() returns true only when a NEW reading is ready.
// NOT yet confirmed on hardware: scale and axis directions of the real sensors (the app shows the resulting velocity live so
// that the mounting direction can be checked by moving the craft by hand).
// ============================================================================
namespace flow {

struct Reading {
    float x = 0, y = 0;          // rad/s about the sensor's X / Y axis
    float dt = 0.02f;            // the stretch of time the reading covers (s)
    float age_s = 0.0f;          // how long ago that stretch ended when the reading was delivered
    uint8_t quality = 0;         // 0..100 (0 = nothing to see)
};

class Driver {
public:
    virtual ~Driver() = default;
    virtual const char* name() const = 0;
    virtual bool init() = 0;                                   // false = the sensor does not answer / is not the expected chip
    virtual bool update(uint32_t now_us, Reading& out) = 0;
    virtual float nominal_rate_hz() const = 0;                 // readings handed over per second
    virtual uint32_t samples() const = 0;                      // what the sensor itself delivered so far (frames, polls): several make one reading
};

// A reading is the sum of what the sensor delivered over about 50 ms, not one sample. Single samples of these sensors are coarse -
// an MSP sensor counts in steps of 5 mrad and a PMW3901 in steps of 1.26 mrad, which at 100 samples a second is 0.5 and
// 0.13 rad/s (half a metre per second, one metre up) - and the time between two samples is only known to the millisecond the
// sensor thread happened to look. Summed over five samples both errors are five times smaller, because the counts that one
// sample rounds away are in the next. (ArduPilot sums its PixArt sensors over 100 ms for the same reason.)
class Accumulator {
public:
    static constexpr uint32_t kWindowUs = 45000;               // handed over once the samples cover this much (5 samples of a 100 Hz sensor, to the millisecond)
    void reset() { ax_ = ay_ = 0; t_us_ = 0; n_ = 0; q_ = 255; }
    // the picture moved by ax / ay (rad) during dt_us; quality 0..100 (the worst sample of a reading counts)
    void add(float ax, float ay, uint32_t dt_us, uint8_t quality) { ax_ += ax; ay_ += ay; t_us_ += dt_us; n_++; if (quality < q_) q_ = quality; }
    bool ready() const { return t_us_ >= kWindowUs; }
    int count() const { return n_; }
    float seconds() const { return t_us_ * 1e-6f; }
    // the mean rate over the summed time into `out` (x, y, dt, quality); starts the next sum
    void take(Reading& out) { const float t = seconds(); out.x = ax_ / t; out.y = ay_ / t; out.dt = t; out.quality = q_; reset(); }
private:
    float ax_ = 0, ay_ = 0; uint32_t t_us_ = 0; int n_ = 0; uint8_t q_ = 255;
};

// Sensors that carry their own lidar (MSP) hand the distance over through this.
struct RangeSample { bool fresh = false; float distance_m = 0; uint8_t quality_pct = 0; };

} // namespace flow
