#pragma once
#include "../config/fast_code.h"
#include "filters.h"
#include "dyn_notch.h"

// ============================================================================
// The gyro filter chain for the three axes, in Betaflight's order (sensors/gyro_filter_impl.c):
//
//   raw gyro -> low-pass 2 -> RPM notches -> static notch 1 -> static notch 2 -> low-pass 1 (static or dynamic) -> dynamic notches
//
//   low-pass 2      a fixed anti-alias / noise low-pass (default PT1 500 Hz)
//   RPM notches     one notch per motor per harmonic at the frequency the ESC reports (bidirectional DShot): removes motor noise
//                   exactly where it is, with almost no delay. Off without rpm telemetry.
//   static notches  for a resonance that does not move (frame, camera mount): centre + lower cutoff, 0 = off
//   low-pass 1      the main gyro low-pass. Dynamic = its cutoff rises with the throttle from dyn_min to dyn_max (more filtering
//                   at low throttle, where the noise is low in frequency; less delay at high throttle)
//   dynamic notches follow whatever peaks are left (frame resonances that move, noise the RPM filter cannot see)
//
// Settings use Betaflight's names, units and defaults; the app's filter slider scales the low-pass cutoffs (FilterSliders).
// ============================================================================
namespace fc {

struct RpmFilterConfig {
    uint8_t harmonics = 3;         // 0 = off, up to 3
    uint8_t weights[3] = {100, 100, 100};      // % effect of harmonic 1 / 2 / 3
    float min_hz = 100.0f;         // a notch never goes below this; it fades out over fade_range above it
    float fade_range_hz = 50.0f;
    float q = 5.0f;                // Betaflight rpm_filter_q / 100
    float lpf_hz = 150.0f;         // smoothing of the reported motor frequency
};

constexpr int kRpmHarmonicsMax = 3;

class RpmFilter {
public:
    void configure(const RpmFilterConfig& c, float loop_hz, int motors) {
        cfg_ = c; motors_ = motors > kMaxMotors ? kMaxMotors : (motors < 0 ? 0 : motors);
        harmonics_ = c.harmonics > kRpmHarmonicsMax ? kRpmHarmonicsMax : c.harmonics;
        dt_ = 1.0f / loop_hz; max_hz_ = 0.48f * loop_hz;                     // not quite up to Nyquist
        for (int h = 0; h < kRpmHarmonicsMax; h++) weight_[h] = clampf(c.weights[h] * 0.01f, 0.0f, 1.0f);
        for (int m = 0; m < kMaxMotors; m++) {
            freq_lpf_[m].set_cutoff(c.lpf_hz, dt_); freq_lpf_[m].reset(0.0f);
            for (int h = 0; h < kRpmHarmonicsMax; h++) { notch_[m][h].reset(); notch_[m][h].set(fmaxf(c.min_hz * (float)(h + 1), 1.0f), dt_, c.q, 0.0f); }
        }
        // all notches are refreshed within 1 ms, a few per cycle (Betaflight RPM_FILTER_DURATION_S)
        const float cycles_per_update = 0.001f * loop_hz;
        per_cycle_ = (int)ceilf((float)(motors_ * harmonics_) / fmaxf(cycles_per_update, 1.0f));
        if (per_cycle_ < 1) per_cycle_ = 1;
        motor_i_ = 0; harm_i_ = 0;
    }
    bool active() const { return harmonics_ > 0 && motors_ > 0; }
    float notch_hz(int motor, int harmonic) const { return hz_[motor][harmonic]; }
    float notch_weight(int motor, int harmonic) const { return w_[motor][harmonic]; }

    // rpm: mechanical rpm per motor (logical order), nullptr = no valid telemetry (the notches fade out).
    WB_FAST void run(float v[3], const float* rpm) {
        if (!active()) return;
        for (int m = 0; m < motors_; m++) mfreq_[m] = freq_lpf_[m].apply(rpm ? rpm[m] * (1.0f / 60.0f) : 0.0f);
        for (int i = 0; i < per_cycle_; i++) {
            if (weight_[harm_i_] > 0.0f) {
                const float f = clampf((float)(harm_i_ + 1) * mfreq_[motor_i_], cfg_.min_hz, max_hz_);
                const float margin = f - cfg_.min_hz;
                float w = 1.0f;
                if (margin < cfg_.fade_range_hz) w = margin / cfg_.fade_range_hz;     // fade out towards min_hz
                w *= weight_[harm_i_];
                notch_[motor_i_][harm_i_].set(f, dt_, cfg_.q, w);
                hz_[motor_i_][harm_i_] = f; w_[motor_i_][harm_i_] = w;
            }
            harm_i_ = (harm_i_ + 1) % harmonics_;
            if (harm_i_ == 0) motor_i_ = (motor_i_ + 1) % motors_;
        }
        for (int h = 0; h < harmonics_; h++) {
            if (weight_[h] <= 0.0f) continue;
            for (int m = 0; m < motors_; m++) notch_[m][h].apply(v);
        }
    }
private:
    RpmFilterConfig cfg_; int motors_ = 4, harmonics_ = 0, per_cycle_ = 1, motor_i_ = 0, harm_i_ = 0;
    float dt_ = 0.00025f, max_hz_ = 1920.0f, weight_[kRpmHarmonicsMax] = {1, 1, 1}, mfreq_[kMaxMotors] = {}, hz_[kMaxMotors][kRpmHarmonicsMax] = {}, w_[kMaxMotors][kRpmHarmonicsMax] = {};
    Pt1 freq_lpf_[kMaxMotors]; RpmNotch notch_[kMaxMotors][kRpmHarmonicsMax];
};

// Cutoff of a dynamic low-pass for a throttle 0..1 (Betaflight dynLpfCutoffFreq / dynThrottle).
WB_FAST inline float dyn_lpf_cutoff(float throttle, float min_hz, float max_hz, uint8_t expo) {
    if (expo > 0) { const float e = expo * 0.1f, curve = throttle * (1.0f - throttle) * e + throttle; return (max_hz - min_hz) * curve + min_hz; }
    return fmaxf(throttle * (1.0f - throttle * throttle / 3.0f) * 1.5f * max_hz, min_hz);
}

struct GyroFilterConfig {
    uint8_t lpf1_type = LPF_PT1; uint16_t lpf1_static_hz = 250, lpf1_dyn_min_hz = 250, lpf1_dyn_max_hz = 500; uint8_t lpf1_dyn_expo = 5;
    uint8_t lpf2_type = LPF_PT1; uint16_t lpf2_hz = 500;
    uint16_t notch1_hz = 0, notch1_cutoff = 0, notch2_hz = 0, notch2_cutoff = 0;
    DynNotchConfig dyn;
    RpmFilterConfig rpm;
};

class GyroFilter {
public:
    void configure(const GyroFilterConfig& c, float loop_hz, int motors) {
        cfg_ = c; const float dt = 1.0f / loop_hz;
        dynamic_ = c.lpf1_dyn_min_hz > 0;
        const float lpf1_hz = dynamic_ ? c.lpf1_dyn_min_hz : c.lpf1_static_hz;
        for (int a = 0; a < 3; a++) {
            lpf1_[a].configure(c.lpf1_type, lpf1_hz, dt); lpf2_[a].configure(c.lpf2_type, c.lpf2_hz, dt);
            n1_[a].configure(c.notch1_hz, c.notch1_cutoff, dt); n2_[a].configure(c.notch2_hz, c.notch2_cutoff, dt);
        }
        dyn_.configure(c.dyn, loop_hz); rpm_.configure(c.rpm, loop_hz, motors);
        lpf1_now_ = lpf1_hz;
    }
    // Dynamic low-pass 1: call when the (quantised) throttle changed. See ThrottleDynamics in pid.h for the pacing.
    void set_throttle(float throttle) {
        if (!dynamic_) return;
        lpf1_now_ = dyn_lpf_cutoff(throttle, cfg_.lpf1_dyn_min_hz, cfg_.lpf1_dyn_max_hz, cfg_.lpf1_dyn_expo);
        for (int a = 0; a < 3; a++) lpf1_[a].set_cutoff(lpf1_now_);
    }
    // v[3] in place. rpm = mechanical rpm per motor or nullptr.
    WB_FAST void apply(float v[3], const float* rpm) {
        for (int a = 0; a < 3; a++) v[a] = lpf2_[a].apply(v[a]);
        rpm_.run(v, rpm);
        for (int a = 0; a < 3; a++) {
            float x = n2_[a].apply(n1_[a].apply(v[a]));
            x = lpf1_[a].apply(x);
            if (dyn_.count()) { dyn_.push(a, x); x = dyn_.apply(a, x); }
            v[a] = x;
        }
        dyn_.update();
    }
    void reset() { for (int a = 0; a < 3; a++) { lpf1_[a].reset(); lpf2_[a].reset(); } }
    float lpf1_cutoff_hz() const { return lpf1_[0].active() ? lpf1_now_ : 0.0f; }
    const DynNotch& dyn_notch() const { return dyn_; }
    const RpmFilter& rpm_filter() const { return rpm_; }
private:
    GyroFilterConfig cfg_; bool dynamic_ = true; float lpf1_now_ = 0.0f;
    Lowpass lpf1_[3], lpf2_[3]; StaticNotch n1_[3], n2_[3]; DynNotch dyn_; RpmFilter rpm_;
};
} // namespace fc
