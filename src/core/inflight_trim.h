#pragma once
#include <stdint.h>
#include <math.h>

// ============================================================================
// In-flight level trim ("in-flight accelerometer calibration").
//
// A craft whose "level" is a little off drifts in Angle / Stabilize, and the pilot holds the stick against it. What is measured is
// the ACCELEROMETER in a hover: a flying craft's accelerometer feels its thrust and nothing else (drag is small at hover speeds),
// so its reading shows where the thrust points relative to the board - exactly the level error - whatever the attitude estimate
// believes at that moment and however well the pilot holds the craft still. (Betaflight's / MultiWii's in-flight ACC calibration
// averages the same thing.) Wind spoils it: holding still against wind means leaning into it. Do it in calm air.
//
//   1. hover steadily, holding the craft still with the sticks, in any mode
//   2. switch the CALIB mode on for at least 2 seconds, then off        -> MEASURED (or FAILED when it was too short)
//   3. land and disarm                                                   -> APPLY: the caller stores the mean attitude as the level trim
//
// Nothing is changed in the air: the horizon must not jump under a flying craft. Only time in a steady hover counts (armed, off
// the ground, turning slower than max_rate, tilted less than max_tilt). Switching it on again measures afresh; a measurement that
// is too short never replaces a good one.
// ============================================================================
namespace fc {

class InflightTrim {
public:
    enum Event : uint8_t { NONE, MEASURED, FAILED, APPLY };
    struct Config { float need_s = 2.0f, max_rate_dps = 15.0f, max_tilt_deg = 10.0f; } cfg;
    struct Input {
        bool armed = false, flying = false, switch_on = false;
        float roll_deg = 0, pitch_deg = 0;      // "down" by the low-passed accelerometer (Ahrs::accel_euler_aero; the convention is the caller's: it gets the same back)
        float gyro_dps = 0;                     // the largest of the three rates, absolute
    };

    Event update(const Input& in, float dt) {
        Event ev = NONE;
        const bool on = in.switch_on && in.armed;
        if (on && in.flying && in.gyro_dps < cfg.max_rate_dps && fabsf(in.roll_deg) < cfg.max_tilt_deg && fabsf(in.pitch_deg) < cfg.max_tilt_deg) {
            sum_r_ += (double)in.roll_deg * dt; sum_p_ += (double)in.pitch_deg * dt; time_ += dt;
        }
        if (was_on_ && !on) {                                                         // switched off, or disarmed with it on
            if (time_ >= cfg.need_s) { roll_ = (float)(sum_r_ / time_); pitch_ = (float)(sum_p_ / time_); pending_ = true; if (in.armed) ev = MEASURED; }
            else if (in.armed || !pending_) ev = FAILED;
            time_ = 0; sum_r_ = sum_p_ = 0;
        }
        if (was_armed_ && !in.armed && pending_) { pending_ = false; ev = APPLY; }
        was_on_ = on; was_armed_ = in.armed;
        return ev;
    }
    bool collecting() const { return was_on_; }
    bool pending() const { return pending_; }                 // a measurement waits for the disarm
    float progress() const { const float p = time_ / cfg.need_s; return p > 1.0f ? 1.0f : p; }
    float roll_deg() const { return roll_; }                  // the measured hover attitude (valid from MEASURED / APPLY on)
    float pitch_deg() const { return pitch_; }

private:
    bool was_on_ = false, was_armed_ = false, pending_ = false;
    float time_ = 0, roll_ = 0, pitch_ = 0; double sum_r_ = 0, sum_p_ = 0;
};
} // namespace fc
