/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>
#include "../proto/wbp_schema.h"

// ============================================================================
// The DIY transmitter's sticks: four 12-bit ADC readings (joy1 X / Y, joy2 X / Y) -> eight channels in microseconds.
//   per axis: calibrated against min / centre / max (each side on its own), reversed if asked, a dead band around the centre
//   (the output still reaches the ends), expo on roll / pitch / yaw (Betaflight's rcExpo shape: v (1 - e) + v^3 e)
//   ch 1..4 = roll, pitch, throttle, yaw from the axes tx_sticks.map names; ch 5..8 = 1000 (no switches on this transmitter yet)
// Throttle on a self-centring stick, two ways (tx_sticks.throttle_mode):
//   ALTPOS  the stick as it stands: centre = 1500, which ALT HOLD / POS HOLD read as "hold the height"
//   MANUAL  the stick moves the throttle up / down at throttle_rate % of the range per second and a released stick leaves it
//           where it was (ANGLE / ACRO); within 15 % of the centre it does not move. 1000 while the drone says it is disarmed
//           (or was never heard): arming with the stick gesture and letting go never lifts the craft. Armed and then not heard
//           (a dropout in flight): held where it was until the drone is heard again.
// Calibration: cal_start(), move every stick to its ends, let them rest, cal_finish(): the ends seen, and where they rest = the
// centre. An axis that moved less than 500 counts keeps its old calibration.
// ============================================================================
namespace tx {

struct StickInput { uint16_t raw[4] = {2048, 2048, 2048, 2048}; };
// what the transmitter knows about the drone: it said it is disarmed / armed, or it said armed and has not been heard since
enum Armed : uint8_t { DISARMED = 0, ARMED = 1, UNKNOWN = 2 };

class Sticks {
public:
    void configure(const wbp::S_tx_sticks& c) { c_ = c; }

    void update(const StickInput& in, uint8_t armed, uint32_t now_ms, uint16_t out_us[8]) {
        last_ = in;
        if (cal_) for (int i = 0; i < 4; i++) { if (in.raw[i] < lo_[i]) lo_[i] = in.raw[i]; if (in.raw[i] > hi_[i]) hi_[i] = in.raw[i]; }
        float v[4];
        for (int ch = 0; ch < 4; ch++) { const uint8_t a = c_.map[ch] > 3 ? 0 : c_.map[ch]; v[ch] = axis(a, in.raw[a]); }
        const float e = c_.expo / 100.0f;
        for (int ch : {0, 1, 3}) v[ch] = v[ch] * (1.0f - e) + v[ch] * v[ch] * v[ch] * e;
        out_us[0] = us(v[0]); out_us[1] = us(v[1]); out_us[3] = us(v[3]);
        if (c_.throttle_mode == wbp::TX_THROTTLE_ALTPOS) out_us[2] = us(v[2]);
        else {
            uint32_t dt = started_ ? now_ms - last_ms_ : 0; if (dt > 100) dt = 100;      // (a pause in the calls is not a long push)
            // a dead zone of its own (kPushDead) on top of the stick's: a cheap thumbstick rests a few % off its centre, and that
            // must not creep the throttle; only a drone heard to be armed moves it (UNKNOWN - armed, then not heard: held as it is)
            const float m = v[2] < 0 ? -v[2] : v[2], push = m <= kPushDead ? 0.0f : (v[2] < 0 ? -1.0f : 1.0f) * (m - kPushDead) / (1.0f - kPushDead);
            if (armed == ARMED) thr_ += push * (float)c_.throttle_rate * 10.0f * (float)dt / 1000.0f;   // rate % of 1000 us per second
            thr_ = armed == DISARMED || thr_ < 1000.0f ? 1000.0f : thr_ > 2000.0f ? 2000.0f : thr_;
            out_us[2] = (uint16_t)(thr_ + 0.5f);
        }
        started_ = true; last_ms_ = now_ms;
        for (int ch = 4; ch < 8; ch++) out_us[ch] = 1000;
    }

    static constexpr float kPushDead = 0.15f;
    void cal_start() { cal_ = true; for (int i = 0; i < 4; i++) { lo_[i] = 4095; hi_[i] = 0; } }
    void cal_cancel() { cal_ = false; }
    bool calibrating() const { return cal_; }
    // the ends seen and the centre where the sticks rest now, into `into` (and used from now on); returns the axes calibrated
    int cal_finish(wbp::S_tx_sticks& into) {
        if (!cal_) return 0;
        cal_ = false; int n = 0;
        for (int i = 0; i < 4; i++) {
            const uint16_t mid = last_.raw[i];
            if (hi_[i] < lo_[i] + 500 || mid <= lo_[i] || mid >= hi_[i]) continue;
            into.cal_min[i] = lo_[i]; into.cal_max[i] = hi_[i]; into.cal_mid[i] = mid; n++;
        }
        c_ = into; return n;
    }

    // one axis, -1 .. 1: calibrated, reversed, dead band
    float axis(int a, uint16_t raw) const {
        const float mid = c_.cal_mid[a], lo = c_.cal_min[a], hi = c_.cal_max[a];
        float v = raw >= mid ? (hi > mid ? (raw - mid) / (hi - mid) : 0.0f) : (mid > lo ? (raw - mid) / (mid - lo) : 0.0f);
        v = v > 1.0f ? 1.0f : v < -1.0f ? -1.0f : v;
        if (c_.invert[a]) v = -v;
        const float d = c_.deadband / 500.0f, m = v < 0 ? -v : v;                     // (us of the half travel)
        if (m <= d) return 0.0f;
        return (v < 0 ? -1.0f : 1.0f) * (m - d) / (1.0f - d);
    }

private:
    static uint16_t us(float v) { const float u = 1500.0f + 500.0f * v; return (uint16_t)(u < 1000.0f ? 1000 : u > 2000.0f ? 2000 : (int)(u + 0.5f)); }
    wbp::S_tx_sticks c_{}; StickInput last_; bool cal_ = false, started_ = false; uint16_t lo_[4] = {}, hi_[4] = {}; float thr_ = 1000.0f; uint32_t last_ms_ = 0;
};

} // namespace tx
