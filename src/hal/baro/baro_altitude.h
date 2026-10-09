/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>
#include <math.h>
#include "baro_driver.h"

// ============================================================================
// Pressure -> altitude above the takeoff point, with the housekeeping Altitude Hold needs (portable, unit-tested).
//   * ground reference: averaged at start-up (while still), then followed slowly while DISARMED so weather drift before takeoff doesn't
//     become a fake altitude; FROZEN when armed.  (Betaflight calibrates the ground pressure at boot/arm the same way.)
//   * altitude = 44330 * (1 - (p / p_ground)^0.190295)   (Betaflight pressureToAltitude, relative form)
//   * spike rejection, plausibility and staleness -> valid()
//   * sigma: measured noise of the altitude estimate while still -> feed it to VerticalKf as the baro measurement sigma
// The baro is NOT trusted alone in flight: drift is the estimator's job (VerticalKf baro-offset state, range finder / GPS).
// ============================================================================
struct BaroAltConfig {
    uint16_t calib_samples = 40;       // samples averaged for the initial ground pressure
    float ground_track_tau_s = 20.0f;  // disarmed: ground pressure follows the air with this time constant
    float max_jump_m = 20.0f;          // a single sample further than this from the last good one is an outlier
    float stale_ms = 500.0f;           // no new sample for this long => invalid
    float still_sigma_floor_m = 0.05f; // never claim the baro is better than this
    float p_min_pa = 30000.0f, p_max_pa = 125000.0f;
};

class BaroAltitude {
public:
    BaroAltConfig cfg;
    void reset() { calib_n_ = 0; calib_sum_ = 0; calibrated_ = false; last_ms_ = 0; have_ = false; alt_ = 0; sigma_ = 0.3f; rejects_ = 0; var_ = 0.09f; }

    // Returns true if the sample was accepted (altitude updated).
    bool update(const BaroSample& s, uint32_t now_ms, bool armed) {
        if (!(s.pressure_pa > cfg.p_min_pa && s.pressure_pa < cfg.p_max_pa)) return false;
        last_ms_ = now_ms; have_ = true;
        if (!calibrated_) {
            calib_sum_ += s.pressure_pa; calib_n_++;
            if (calib_n_ >= cfg.calib_samples) { p0_ = (float)(calib_sum_ / calib_n_); calibrated_ = true; alt_ = 0; last_good_ = 0; prev_ms_ = now_ms; }
            return false;
        }
        const float dt = (now_ms - prev_ms_) * 1e-3f; prev_ms_ = now_ms;
        float a = altitude_of(s.pressure_pa, p0_);
        if (fabsf(a - last_good_) > cfg.max_jump_m) { if (++rejects_ < 5) return false; }   // 5 in a row = it really moved (e.g. a big throw)
        rejects_ = 0;
        if (!armed && dt > 0 && dt < 1.0f) {
            // follow the air slowly while on the ground, and measure the noise
            const float k = dt / (cfg.ground_track_tau_s + dt);
            var_ += 0.02f * (a * a - var_);                     // a = altitude about the tracked ground reference => scatter = noise
            p0_ += k * (s.pressure_pa - p0_);
            a = altitude_of(s.pressure_pa, p0_);
            const float sg = sqrtf(var_ > 0 ? var_ : 0);
            sigma_ = sg < cfg.still_sigma_floor_m ? cfg.still_sigma_floor_m : sg;
        }
        alt_ = a; last_good_ = a; temp_c_ = s.temp_c; pressure_pa_ = s.pressure_pa;
        return true;
    }
    bool calibrated() const { return calibrated_; }
    bool valid(uint32_t now_ms) const { return calibrated_ && have_ && (now_ms - last_ms_) <= (uint32_t)cfg.stale_ms; }
    float altitude_m() const { return alt_; }          // up positive, 0 = ground pressure
    float sigma_m() const { return sigma_; }
    float ground_pressure_pa() const { return p0_; }
    float pressure_pa() const { return pressure_pa_; }
    float temp_c() const { return temp_c_; }

    static float altitude_of(float p, float p_ref) { return 44330.0f * (1.0f - powf(p / p_ref, 0.190295f)); }

private:
    uint16_t calib_n_ = 0; double calib_sum_ = 0; bool calibrated_ = false, have_ = false;
    float p0_ = 101325.0f, alt_ = 0, last_good_ = 0, sigma_ = 0.3f, var_ = 0.09f, temp_c_ = 0, pressure_pa_ = 0;
    uint32_t last_ms_ = 0, prev_ms_ = 0; uint8_t rejects_ = 0;
};
