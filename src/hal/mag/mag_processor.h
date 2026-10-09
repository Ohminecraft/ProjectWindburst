/*
 * This file is part of Windburst Project
 */

#pragma once

#include <math.h>
#include "mag_calibration.h"
#include "../acc_gyro/acc_gyro_util.h"

// ============================================================================
// MagProcessor: raw chip sample -> calibrated vector in the BOARD frame (FRD), plus health.
//   1. calibration (hard + soft iron, in the chip frame)           2. mounting alignment (8 discrete orientations + roll/pitch/yaw trim,
//   the same maths as the IMU)       3. field-strength monitor: in flight the field must stay within +-30 % of the calibrated strength;
//   motor/ESC current and nearby wires bend it, and the attitude filter must not follow a distorted vector.
// valid(now): calibrated, recent, and the field magnitude is plausible.   ready_for_fusion = valid && not interfered.
// ============================================================================
struct MagProcessorConfig {
    float strength_tolerance = 0.30f;     // +-30 % of the calibrated field radius
    float stale_ms = 300.0f;
    float uncal_min = 0.15f, uncal_max = 0.90f;   // plausible earth-field range when no calibration is stored
};

class MagProcessor {
public:
    MagProcessorConfig cfg;
    void set_calibration(const MagCal& c) { cal_ = c; }
    const MagCal& calibration() const { return cal_; }
    void set_alignment(AccGyroAlign discrete, float roll_deg = 0, float pitch_deg = 0, float yaw_deg = 0) {
        align_ = discrete; fine_ = (roll_deg != 0 || pitch_deg != 0 || yaw_deg != 0);
        if (fine_) accgyro::build_rotation(rot_, roll_deg, pitch_deg, yaw_deg);
    }

    // feed one raw chip-frame sample (gauss)
    void update(const float raw[3], uint32_t now_ms) {
        float v[3];
        if (cal_.valid) cal_.apply(raw, v); else { v[0] = raw[0]; v[1] = raw[1]; v[2] = raw[2]; }
        accgyro::align_chip(v, align_);                 // chip axes -> board frame (x forward, y right, z down), orientation named as in Betaflight
        if (fine_) accgyro::apply_rotation(v, rot_);
        for (int k = 0; k < 3; k++) board_[k] = v[k];
        const float n = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
        strength_ = n; last_ms_ = now_ms; have_ = true; samples_++;
        if (cal_.valid) { const float dev = fabsf(n - cal_.radius) / cal_.radius; interference_ = dev; field_ok_ = dev <= cfg.strength_tolerance; }
        else { interference_ = 0; field_ok_ = n >= cfg.uncal_min && n <= cfg.uncal_max; }
    }
    bool valid(uint32_t now_ms) const { return have_ && (now_ms - last_ms_) <= (uint32_t)cfg.stale_ms; }
    bool calibrated() const { return cal_.valid; }
    // only a calibrated, fresh, undisturbed vector may steer the yaw estimate
    bool ready_for_fusion(uint32_t now_ms) const { return cal_.valid && valid(now_ms) && field_ok_; }
    const float* board() const { return board_; }               // gauss, FRD board frame
    float strength() const { return strength_; }
    float interference() const { return interference_; }        // |B| deviation from the calibrated radius, fraction
    uint32_t samples() const { return samples_; }

private:
    MagCal cal_;
    AccGyroAlign align_ = CW0_DEG; bool fine_ = false; float rot_[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    float board_[3] = {0, 0, 0}, strength_ = 0, interference_ = 0; bool have_ = false, field_ok_ = false; uint32_t last_ms_ = 0, samples_ = 0;
};
