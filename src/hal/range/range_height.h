/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>
#include <math.h>
#include "range_driver.h"

// ============================================================================
// RangeHeight: raw beam distance -> height above ground, ready for VerticalKf::update_range().   Portable, unit-tested.
//   * tilt correction:  height = distance * cos(tilt)   (cos(tilt) = Attitude::up_cos()); steep tilt => sample rejected
//   * gating: min/max range of the chip, signal quality, impossible jump (needs 3 consistent samples to accept a new level)
//   * ground clearance: the sensor sits a few cm above the ground when landed. Learned automatically while DISARMED, level and
//     reading < 0.5 m (slow filter), frozen once armed - or set manually (>= 0 disables learning)
//   * sigma grows with height (ToF accuracy is ~ percent of distance) -> kf.cfg.r_range
//   * valid() times out 250 ms after the last accepted sample; the Kalman filter then simply stops using it (baro keeps going).
// ============================================================================
struct RangeHeightConfig {
    float max_tilt_deg_cos = 0.866f;      // cos(30 deg): beyond that the beam no longer looks straight down
    float min_quality_pct = 20.0f;
    float max_speed_mps = 8.0f;           // physically plausible vertical speed for jump rejection
    float jump_margin_m = 0.30f;
    uint8_t jump_accept_count = 3;        // consecutive consistent samples that overrule the jump check
    float stale_ms = 250.0f;
    float range_margin = 0.90f;           // use only up to 90 % of the chip's max range (edge readings are noisy)
    float ground_clearance_m = -1.0f;     // < 0: learn automatically; >= 0: fixed value
    float learn_max_height_m = 0.5f;
    float learn_min_cos = 0.95f;          // must be nearly level to learn the clearance
    float learn_tau_s = 2.0f;
    float sigma_base_m = 0.02f, sigma_per_m = 0.015f;
};

class RangeHeight {
public:
    RangeHeightConfig cfg;

    void begin(float min_range_m, float max_range_m) { min_m_ = min_range_m; max_m_ = max_range_m * cfg.range_margin; reset(); }
    void reset() { have_ = false; last_ms_ = 0; height_ = 0; last_h_ = 0; jump_n_ = 0; clearance_ = cfg.ground_clearance_m >= 0 ? cfg.ground_clearance_m : 0.0f; learned_ = cfg.ground_clearance_m >= 0; learn_t_ = 0; accepted_ = 0; rejected_ = 0; }

    // Returns true if this sample produced a new, usable height.
    bool update(const RangeReading& r, float up_cos, uint32_t now_ms, bool armed) {
        const float dt = have_ ? (now_ms - last_ms_) * 1e-3f : 0.0f;
        if (up_cos < cfg.max_tilt_deg_cos) return reject();
        if (!(r.distance_m >= min_m_ && r.distance_m <= max_m_)) return reject();
        if (r.quality_pct < cfg.min_quality_pct) return reject();
        const float corrected = r.distance_m * up_cos;
        float h = corrected - clearance_;

        // jump check against the last accepted height
        if (have_) {
            const float allowed = cfg.max_speed_mps * (dt > 0.5f ? 0.5f : dt) + cfg.jump_margin_m;
            if (fabsf(h - last_h_) > allowed) {
                if (fabsf(h - jump_ref_) < cfg.jump_margin_m && jump_n_ > 0) jump_n_++; else { jump_n_ = 1; }
                jump_ref_ = h;
                if (jump_n_ < cfg.jump_accept_count) return reject();
            }
        }
        jump_n_ = 0;

        // learn the ground clearance on the ground
        if (!armed && cfg.ground_clearance_m < 0 && up_cos > cfg.learn_min_cos && corrected < cfg.learn_max_height_m) {
            if (!learned_) { clearance_ = corrected; learned_ = true; }
            else if (dt > 0 && dt < 1.0f) clearance_ += (corrected - clearance_) * (dt / (cfg.learn_tau_s + dt));
            h = corrected - clearance_;
        }
        if (h < 0.0f) h = 0.0f;
        height_ = h; last_h_ = h; raw_ = r.distance_m; quality_ = r.quality_pct;
        last_ms_ = now_ms; have_ = true; accepted_++;
        return true;
    }
    bool valid(uint32_t now_ms) const { return have_ && (now_ms - last_ms_) <= (uint32_t)cfg.stale_ms; }
    float height_m() const { return height_; }
    float sigma_m() const { return cfg.sigma_base_m + cfg.sigma_per_m * height_; }
    float raw_m() const { return raw_; }
    float clearance_m() const { return clearance_; }
    bool clearance_known() const { return learned_; }
    uint32_t accepted() const { return accepted_; }
    uint32_t rejected() const { return rejected_; }

private:
    bool reject() { rejected_++; return false; }
    float min_m_ = 0.02f, max_m_ = 3.0f;
    bool have_ = false, learned_ = false;
    float height_ = 0, last_h_ = 0, jump_ref_ = 0, clearance_ = 0, raw_ = 0, learn_t_ = 0;
    uint8_t quality_ = 0, jump_n_ = 0;
    uint32_t last_ms_ = 0, accepted_ = 0, rejected_ = 0;
};
