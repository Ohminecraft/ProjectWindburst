#pragma once
#include "types.h"

// ============================================================================
// Which way is down while the craft accelerates - from a velocity that was MEASURED.
//
// In flight the accelerometer shows the thrust, not gravity (ahrs.h): an estimate that is corrected by it alone gives way a few
// degrees whenever the craft gathers speed or brakes. With a velocity over the ground from GPS or optical flow the missing piece
// is known: the craft's real acceleration a = dv/dt. The accelerometer must then read  f = a - g  (specific force), and where
// the estimate puts that reading in the world can be compared with where it has to be:
//
//      measured  : the mean specific force of the last stretch, turned into the world frame BY THE ESTIMATE          f_m
//      expected  : (dv_north / dt, dv_east / dt, -g)                                                                  f_e
//      error     : f_m x f_e  (unit vectors)   = the rotation that is missing, as a vector in the world frame
//
// ArduPilot's DCM (AP_AHRS_DCM::drift_correction) and PX4's attitude_estimator_q do the same. Only the TILT is corrected (the
// vertical part of the error is a heading error; the compass owns the heading). The velocity itself is never used, only its
// change: a GPS that is off by a metre per second for minutes does no harm, wind does not matter.
//
// Timing: velocity measurements are late (GPS 0.1 .. 0.2 s) and noisy, so they are gathered in blocks of 0.2 s; what the
// specific force added up to is kept for the last second in 5 ms pieces, from which exactly the stretch between two blocks
// is cut out.
// ============================================================================
namespace fc {

class TiltAid {
public:
    struct Config {
        float block_s = 0.2f;          // velocity samples are averaged over this long; the acceleration is taken between two such averages
        float max_gap_s = 0.6f;        // two averages further apart than this are not compared (measurements were missing)
        float timeout_s = 0.6f;        // no new comparison for this long: not valid any more
        float max_accel = 15.0f;       // m/s^2: a measured change of velocity beyond this is a glitch of the sensor, not a manoeuvre
    } cfg;

    void reset() { head_ = 0; count_ = 0; t_us_ = 0; slot_us_ = 0; acc_[0] = acc_[1] = acc_[2] = 0; blk_n_ = 0; have_prev_ = false; valid_ = false; age_ = 1e9f; err_ = Vec3(); }

    // Every estimator step: the specific force (g) in the world frame (north, east, down) as the CURRENT attitude estimate turns it.
    WB_FAST void push_force(float fn, float fe, float fd, float dt) {
        const uint32_t d = (uint32_t)(dt * 1e6f + 0.5f);
        t_us_ += d; slot_us_ += d; age_ += dt;
        acc_[0] += fn * kG0 * dt; acc_[1] += fe * kG0 * dt; acc_[2] += fd * kG0 * dt;
        if (slot_us_ >= kStepUs) {                               // the slot is full: keep what was gathered in it
            ring_end_[head_] = t_us_; ring_len_[head_] = slot_us_; for (int k = 0; k < 3; k++) ring_d_[head_][k] = acc_[k];
            head_ = (head_ + 1) % kSlots; if (count_ < kSlots) count_++;
            slot_us_ = 0; acc_[0] = acc_[1] = acc_[2] = 0;
        }
        if (valid_ && age_ > cfg.timeout_s) valid_ = false;
    }

    // A velocity over the ground (m/s, north / east) that was true `age_s` ago. Returns true when a new error was worked out.
    WB_FAST bool on_velocity(float vn, float ve, float age_s) {
        const uint32_t tm = t_us_ - (age_s > 0 ? (uint32_t)(age_s * 1e6f) : 0u);
        if (blk_n_ > 0 && (int32_t)(tm - blk_t0_) >= (int32_t)(cfg.block_s * 1e6f)) {      // the block is full: close it, compare with the one before
            const float n = (float)blk_n_; const uint32_t bt = blk_t0_ + (uint32_t)(int32_t)(blk_dt_ / n); const float bvn = blk_vn_ / n, bve = blk_ve_ / n;
            bool done = false;
            if (have_prev_) done = compare(prev_t_, prev_vn_, prev_ve_, bt, bvn, bve);
            prev_t_ = bt; prev_vn_ = bvn; prev_ve_ = bve; have_prev_ = true; blk_n_ = 0;
            add(tm, vn, ve);
            return done;
        }
        add(tm, vn, ve);
        return false;
    }
    // The measurements stopped or jumped (a new GPS fix after an outage, flow reacquired): the next comparison starts afresh.
    void restart() { blk_n_ = 0; have_prev_ = false; }

    bool valid() const { return valid_; }
    Vec3 error_world() const { return err_; }                // rotation vector (rad; north, east, down), down part always 0
    float accel_n() const { return an_; } float accel_e() const { return ae_; }      // the measured acceleration of the last comparison (m/s^2)

private:
    // The integral of the specific force is kept as what was gathered in each 5 ms slot (1.28 s of history), and the stretch
    // between two velocity blocks is summed from those. Time is a 32-bit microsecond count and every sum is a small number:
    // single precision is enough however long the flight is, and the ESP32-S3 has no double-precision hardware (a running
    // sum over the whole flight would need it, at 500 soft-float steps a second inside the control cycle).
    static constexpr int kSlots = 256;
    static constexpr uint32_t kStepUs = 5000;
    WB_FAST void add(uint32_t tm, float vn, float ve) { if (blk_n_ == 0) { blk_t0_ = tm; blk_dt_ = 0; blk_vn_ = blk_ve_ = 0; } blk_dt_ += (float)(int32_t)(tm - blk_t0_); blk_vn_ += vn; blk_ve_ += ve; blk_n_++; }
    // What the specific force added up to (m/s) between the times t0 and t1. false: the history does not reach back to t0.
    WB_FAST bool integral(uint32_t t0, uint32_t t1, float out[3]) const {
        out[0] = out[1] = out[2] = 0.0f;
        // as ages (microseconds before now): the stretch asked for is [a1, a0]; a time after now counts as now
        const int32_t a0 = (int32_t)(t_us_ - t0); int32_t a1 = (int32_t)(t_us_ - t1); if (a1 < 0) a1 = 0;
        if (a0 <= a1) return false;
        int32_t young = 0;                                      // age of the younger end of the slot looked at
        for (int i = -1; i < count_; i++) {                     // -1: the slot that is still being filled
            const int idx = (head_ - 1 - i + 2 * kSlots) % kSlots;
            const int32_t len = (int32_t)(i < 0 ? slot_us_ : ring_len_[idx]); const float* d = i < 0 ? acc_ : ring_d_[idx];
            const int32_t old = young + len, lo = a1 > young ? a1 : young, hi = a0 < old ? a0 : old;
            if (hi > lo && len > 0) { const float f = (hi - lo) == len ? 1.0f : (float)(hi - lo) / (float)len; for (int k = 0; k < 3; k++) out[k] += d[k] * f; }
            if (old >= a0) return true;
            young = old;
        }
        return false;
    }
    WB_FAST bool compare(uint32_t t0, float vn0, float ve0, uint32_t t1, float vn1, float ve1) {
        const float w = (float)(int32_t)(t1 - t0) * 1e-6f;
        if (w < 0.5f * cfg.block_s || w > cfg.max_gap_s) return false;
        float c[3]; if (!integral(t0, t1, c)) return false;
        const float an = (vn1 - vn0) / w, ae = (ve1 - ve0) / w;
        if (an * an + ae * ae > cfg.max_accel * cfg.max_accel) return false;
        // measured: mean specific force over [t0, t1] (m/s^2); expected: (an, ae, -g)
        float m[3] = {c[0] / w, c[1] / w, c[2] / w}, e[3] = {an, ae, -kG0};
        const float mn = sqrtf(m[0] * m[0] + m[1] * m[1] + m[2] * m[2]), en = sqrtf(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]);
        if (mn < 0.3f * kG0 || mn > 3.0f * kG0) return false;             // free fall, an impact: no direction to compare
        for (int k = 0; k < 3; k++) { m[k] /= mn; e[k] /= en; }
        err_.x = m[1] * e[2] - m[2] * e[1]; err_.y = m[2] * e[0] - m[0] * e[2]; err_.z = 0.0f;       // m x e, tilt only
        an_ = an; ae_ = ae; valid_ = true; age_ = 0;
        return true;
    }
    uint32_t ring_end_[kSlots] = {}, ring_len_[kSlots] = {}; float ring_d_[kSlots][3] = {}, acc_[3] = {0, 0, 0};
    uint32_t t_us_ = 0, slot_us_ = 0; int head_ = 0, count_ = 0;
    uint32_t blk_t0_ = 0; float blk_dt_ = 0, blk_vn_ = 0, blk_ve_ = 0; int blk_n_ = 0;
    uint32_t prev_t_ = 0; float prev_vn_ = 0, prev_ve_ = 0; bool have_prev_ = false;
    bool valid_ = false; float age_ = 1e9f, an_ = 0, ae_ = 0; Vec3 err_;
};

} // namespace fc
