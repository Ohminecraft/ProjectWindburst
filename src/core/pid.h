#pragma once
#include "../config/fast_code.h"
#include "filters.h"
#include "gyro_filter.h"

// ============================================================================
// Rate (inner) loop: a port of Betaflight's PID controller (flight/pid.c, pid_init.c), with its units, scale factors, defaults
// and option names, so that numbers from a Betaflight tune - and the configurator's sliders - mean the same thing here.
//
//   error      = setpoint - gyro                                   (deg/s, user sign convention, see types.h)
//   P          = Kp * error                    * TPA (mode PD)      yaw: low-passed (yaw_lowpass_hz)
//   I         += (Ki + anti-gravity) * dT * relaxed error           limited to iterm_windup % of the PID sum limit; yaw Ki x 2.5
//   D          = -Kd * d(filtered gyro)/dt     * D-max boost * TPA   D acts on the MEASUREMENT: no kick from a stick step
//   F          = Kf * feedforward                                   the stick's own speed (see setpoint.h), not on self-levelled axes
//   sum        = P + I + D + F, limited to +-pidsum_limit, / 1000 = fraction of the motor range for the mixer
//
//   Kp = 0.032029 * p     Ki = 0.244381 * i     Kd = 0.000529 * d     Kf = 0.013754 * f / 100        (pid.h: PTERM_SCALE ...)
//
// D-term filtering: gyro -> D-term notch -> low-pass 1 (static or dynamic with the throttle) -> low-pass 2 -> differentiate.
// dT is the nominal loop period (not the measured one): a late cycle must not produce a D spike.
//
// Left out on purpose (not for this craft): crash recovery, launch control, acro trainer, wing code (S-term, SPA), I-term
// rotation, yaw spin recovery, thrust linearisation, the chirp generator.
// ============================================================================
namespace fc {

constexpr float kPtermScale = 0.032029f, kItermScale = 0.244381f, kDtermScale = 0.000529f, kFeedforwardScale = 0.013754f;
constexpr float kPidMixerScaling = 1000.0f;
constexpr float kInvPidMixerScaling = 1.0f / kPidMixerScaling;   // (a float division is a library call on the ESP32: the cycle multiplies)
constexpr float kItermRelaxSetpointThreshold = 40.0f;          // deg/s of high-passed setpoint at which the I-term is fully frozen
constexpr float kInvItermRelaxThreshold = 1.0f / kItermRelaxSetpointThreshold;
constexpr float kAntiGravityKi = 0.34f, kAntiGravityKp = 0.0034f;
constexpr float kDMaxRangeHz = 85.0f, kDMaxLowpassHz = 35.0f, kDMaxGyroGainFactor = 0.00008f, kDMaxSetpointGainFactor = 0.00008f;
constexpr int kDynLpfThrottleSteps = 100;
constexpr float kDynLpfUpdateDelayS = 0.005f;

enum ItermRelax : uint8_t { RELAX_OFF, RELAX_RP, RELAX_RPY, RELAX_RP_INC, RELAX_RPY_INC, RELAX_COUNT };
enum ItermRelaxType : uint8_t { RELAX_GYRO, RELAX_SETPOINT };
enum TpaMode : uint8_t { TPA_PD, TPA_D };

struct PidConfig {
    // ---- gains (Betaflight numbers: 0..250, feedforward 0..1000). Defaults = Betaflight's.
    uint8_t p[3] = {45, 47, 45}, i[3] = {80, 84, 80}, d[3] = {30, 34, 0}, d_max[3] = {40, 46, 0};
    uint16_t f[3] = {120, 125, 120};
    uint16_t pidsum_limit = 500, pidsum_limit_yaw = 400;
    uint8_t iterm_windup = 80;                    // I-term limit, % of the PID sum limit
    // ---- I-term relax
    uint8_t iterm_relax = RELAX_RP, iterm_relax_type = RELAX_SETPOINT, iterm_relax_cutoff = 15;
    // ---- anti-gravity
    bool anti_gravity = true; uint8_t anti_gravity_gain = 80, anti_gravity_cutoff_hz = 5, anti_gravity_p_gain = 100;
    // ---- D max ("dynamic damping")
    uint8_t d_max_gain = 0, d_max_advance = 35;
    // ---- TPA: less D (and P) at high throttle, where the motors respond harder
    uint8_t tpa_mode = TPA_D, tpa_rate = 65; float tpa_breakpoint = 0.35f;        // throttle 0..1 (Betaflight 1350)
    uint8_t tpa_low_rate = 20; float tpa_low_breakpoint = 0.05f; bool tpa_low_always = false;
    // ---- D-term filters, yaw low-pass
    uint8_t dterm_lpf1_type = LPF_PT1; uint16_t dterm_lpf1_static_hz = 75, dterm_lpf1_dyn_min_hz = 75, dterm_lpf1_dyn_max_hz = 150; uint8_t dterm_lpf1_dyn_expo = 5;
    uint8_t dterm_lpf2_type = LPF_PT1; uint16_t dterm_lpf2_hz = 150;
    uint16_t dterm_notch_hz = 0, dterm_notch_cutoff = 0;
    uint16_t yaw_lowpass_hz = 100;
};

struct PidInput {
    float setpoint[3] = {0, 0, 0};                // deg/s
    float gyro[3] = {0, 0, 0};                    // filtered gyro, deg/s
    float feedforward[3] = {0, 0, 0};             // speed of the stick setpoint, deg/s per second (0 for an axis that is self-levelled)
    bool angle_mode = false;                      // roll / pitch are self-levelled: I-term relax reacts five times earlier
    bool i_enabled = true;                        // false: on the ground / zero throttle without airmode -> the I-terms are held at zero
};

class PidController {
public:
    float p_term[3] = {}, i_term[3] = {}, d_term[3] = {}, f_term[3] = {};      // Betaflight "PID sum" units (the mixer gets sum / 1000)

    void configure(const PidConfig& c, float loop_hz) {
        cfg_ = c; dt_ = 1.0f / loop_hz; freq_ = loop_hz;
        for (int a = 0; a < 3; a++) {
            kp_[a] = kPtermScale * c.p[a]; ki_[a] = kItermScale * c.i[a]; kd_[a] = kDtermScale * c.d[a]; kf_[a] = kFeedforwardScale * (c.f[a] * 0.01f);
            d_max_percent_[a] = (c.d[a] > 0 && c.d_max[a] > c.d[a]) ? (float)c.d_max[a] / (float)c.d[a] : 1.0f;
        }
        ki_[YAW] *= 2.5f;
        iterm_limit_ = 0.01f * c.iterm_windup * c.pidsum_limit; iterm_limit_yaw_ = 0.01f * c.iterm_windup * c.pidsum_limit_yaw;
        ag_p_gain_ = c.anti_gravity_p_gain * 0.01f * kAntiGravityKp;
        d_max_gyro_gain_ = kDMaxGyroGainFactor * c.d_max_gain / kDMaxLowpassHz; d_max_setpoint_gain_ = kDMaxSetpointGainFactor * c.d_max_advance / kDMaxLowpassHz;
        tpa_break_ = clampf(c.tpa_breakpoint, 0.0f, 0.99f);
        tpa_mult_ = (c.tpa_rate * 0.01f) / (1.0f - tpa_break_);
        tpa_low_break_ = minf(clampf(c.tpa_low_breakpoint, 0.01f, 1.0f), tpa_break_);
        tpa_low_mult_ = tpa_low_break_ > 0.0f ? c.tpa_low_rate / (100.0f * tpa_low_break_) : 0.0f;

        dterm_dynamic_ = c.dterm_lpf1_dyn_min_hz > 0;
        const float lpf1 = dterm_dynamic_ ? c.dterm_lpf1_dyn_min_hz : c.dterm_lpf1_static_hz;
        for (int a = 0; a < 3; a++) {
            d_notch_[a].configure(c.dterm_notch_hz, c.dterm_notch_cutoff, dt_);
            d_lpf1_[a].configure(c.dterm_lpf1_type, lpf1, dt_); d_lpf2_[a].configure(c.dterm_lpf2_type, c.dterm_lpf2_hz, dt_);
            relax_lpf_[a].set_cutoff(c.iterm_relax_cutoff, dt_);
            d_max_range_[a].set_cutoff(kDMaxRangeHz, dt_); d_max_lpf_[a].set_cutoff(kDMaxLowpassHz, dt_);
        }
        yaw_lpf_.set_cutoff(c.yaw_lowpass_hz, dt_);
        ag_lpf_.set_cutoff(c.anti_gravity_cutoff_hz, dt_);
        reset();
    }

    // Disarmed / (re)starting: nothing carries over.
    WB_FAST void reset() {
        for (int a = 0; a < 3; a++) {
            p_term[a] = i_term[a] = d_term[a] = f_term[a] = 0.0f; have_prev_[a] = false; prev_d_gyro_[a] = 0.0f;
            d_notch_[a].configure(cfg_.dterm_notch_hz, cfg_.dterm_notch_cutoff, dt_); d_lpf1_[a].reset(); d_lpf2_[a].reset();
            relax_lpf_[a].reset(); d_max_range_[a].reset(0.0f); d_max_lpf_[a].reset(1.0f);
        }
        yaw_lpf_.reset(); ag_lpf_.reset(0.0f);
        prev_throttle_ = 0.0f; have_throttle_ = false; ag_throttle_d_ = 0.0f; tpa_factor_ = 1.0f; tpa_low_faded_ = false;
        dyn_quantized_ = -1; dyn_wait_ = 0.0f; dterm_lpf1_now_ = d_lpf1_[0].cutoff_hz();
    }

    // Once per control cycle BEFORE update(): everything that follows the throttle (0..1, as sent to the mixer before the boost).
    // gyro_filter: its dynamic low-pass is moved on the same schedule (may be nullptr).
    // from_pilot: the throttle is the stick's. false (Altitude Hold, failsafe descent): no anti-gravity - that reacts to a hand
    // moving the stick, not to a controller working against barometer noise.
    WB_FAST void update_throttle(float throttle, GyroFilter* gyro_filter, bool from_pilot = true) {
        throttle = clampf(throttle, 0.0f, 1.0f);
        // ---- anti-gravity: the low-passed speed of the throttle, weighted towards low throttle and towards throttle going down
        float thr_d = have_throttle_ && from_pilot ? fabsf(throttle - prev_throttle_) * freq_ : 0.0f;
        const float inv = 1.0f - throttle;
        thr_d *= inv * inv;
        if (throttle > prev_throttle_) thr_d *= inv * 0.5f;
        prev_throttle_ = throttle; have_throttle_ = true;
        ag_throttle_d_ = ag_lpf_.apply(thr_d);
        // ---- TPA
        tpa_factor_ = 1.0f - tpa_rate_at(throttle);
        // ---- dynamic low-passes (gyro low-pass 1, D-term low-pass 1): at most every 5 ms and only when the throttle moved by 1 %
        dyn_wait_ += dt_;
        if (dyn_wait_ >= kDynLpfUpdateDelayS) {
            const int q = (int)lrintf(throttle * (float)kDynLpfThrottleSteps);
            if (q != dyn_quantized_) {
                const float t = (float)q / (float)kDynLpfThrottleSteps;
                if (gyro_filter) gyro_filter->set_throttle(t);
                if (dterm_dynamic_) {
                    dterm_lpf1_now_ = dyn_lpf_cutoff(t, cfg_.dterm_lpf1_dyn_min_hz, cfg_.dterm_lpf1_dyn_max_hz, cfg_.dterm_lpf1_dyn_expo);
                    for (int a = 0; a < 3; a++) d_lpf1_[a].set_cutoff(dterm_lpf1_now_);
                }
                dyn_quantized_ = q; dyn_wait_ = 0.0f;
            }
        }
    }

    // u[] = PID sum / 1000, limited: the authority the mixer gets (fraction of the motor range).
    WB_FAST void update(const PidInput& in, float u[3]) {
        const float ag_d = anti_gravity(), iterm_accel = ag_d * kAntiGravityKi;

        for (int a = 0; a < 3; a++) {
            float sp = in.setpoint[a];
            const float gyro = in.gyro[a];
            float error = sp - gyro, iterm_error = error;

            // ---- I-term relax: while the stick is moving, the error is "the craft has not caught up yet", not something to integrate
            const float sp_lpf = relax_lpf_[a].apply(sp), sp_hpf = fabsf(sp - sp_lpf);
            if (cfg_.iterm_relax != RELAX_OFF && (a < YAW || cfg_.iterm_relax == RELAX_RPY || cfg_.iterm_relax == RELAX_RPY_INC)) {
                const float factor = maxf(0.0f, 1.0f - sp_hpf * (in.angle_mode ? 5.0f : 1.0f) * kInvItermRelaxThreshold);
                const bool decreasing = (i_term[a] > 0.0f && iterm_error < 0.0f) || (i_term[a] < 0.0f && iterm_error > 0.0f);
                if (cfg_.iterm_relax >= RELAX_RP_INC && decreasing) { /* the I-term may always shrink */ }
                else if (cfg_.iterm_relax_type == RELAX_SETPOINT) iterm_error *= factor;
                else {                                           // GYRO: the error against the low-passed setpoint, with a dead band as wide as the stick movement
                    const float e = sp_lpf - gyro;
                    iterm_error = fabsf(e) <= sp_hpf ? 0.0f : (e > 0.0f ? e - sp_hpf : e + sp_hpf);
                }
            }

            // ---- P
            float p = kp_[a] * error * (cfg_.tpa_mode == TPA_PD ? tpa_factor_ : 1.0f);
            if (a == YAW && cfg_.yaw_lowpass_hz > 0) p = yaw_lpf_.apply(p);

            // ---- I
            float ki = ki_[a], limit = iterm_limit_, accel = iterm_accel;
            if (a == YAW) { limit = iterm_limit_yaw_; accel = 0.0f; }          // no anti-gravity on yaw
            if (in.i_enabled) i_term[a] = clampf(i_term[a] + (ki + accel) * dt_ * iterm_error, -limit, limit);
            else i_term[a] = 0.0f;

            // ---- D on the filtered gyro
            float gd = d_notch_[a].apply(gyro);
            gd = d_lpf2_[a].apply(d_lpf1_[a].apply(gd));
            float d = 0.0f;
            if (kd_[a] > 0.0f) {
                const float delta = have_prev_[a] ? -(gd - prev_d_gyro_[a]) * freq_ : 0.0f;
                float mult = 1.0f;
                if (d_max_percent_[a] > 1.0f) {                  // D max: more D while the gyro or the stick is moving fast
                    const float gyro_factor = fabsf(d_max_range_[a].apply(delta)) * d_max_gyro_gain_;
                    const float sp_factor = fabsf(in.feedforward[a]) * d_max_setpoint_gain_;
                    mult += (d_max_percent_[a] - 1.0f) * maxf(gyro_factor, sp_factor);
                    mult = d_max_lpf_[a].apply(mult);
                    if (mult > d_max_percent_[a]) mult = d_max_percent_[a];
                }
                d = kd_[a] * delta * mult * tpa_factor_;
            }
            prev_d_gyro_[a] = gd; have_prev_[a] = true;

            // ---- F
            const float f = kf_[a] * in.feedforward[a];

            // ---- anti-gravity P boost on roll / pitch, less while a turn is commanded
            if (a != YAW) {
                const float atten = maxf(fabsf(sp) * 0.02f, 1.0f);
                p *= 1.0f + (ag_d / atten) * ag_p_gain_;
            }

            p_term[a] = p; d_term[a] = d; f_term[a] = f;
            const float lim = a == YAW ? (float)cfg_.pidsum_limit_yaw : (float)cfg_.pidsum_limit;
            u[a] = clampf(p + i_term[a] + d + f, -lim, lim) * kInvPidMixerScaling;
        }
    }

    float tpa_factor() const { return tpa_factor_; }
    float anti_gravity() const { return (ag_switch_ < 0 ? cfg_.anti_gravity : ag_switch_ > 0) ? ag_throttle_d_ * cfg_.anti_gravity_gain : 0.0f; }
    // The ANTI_GRAVITY switch (rc.h): -1 = no switch assigned (cfg.anti_gravity decides), 0 = off, 1 = on.
    void set_anti_gravity_switch(int8_t s) { ag_switch_ = s; }
    WB_FAST void reset_i(int axis) { i_term[axis] = 0.0f; }
    float dterm_lpf1_cutoff_hz() const { return d_lpf1_[0].active() ? dterm_lpf1_now_ : 0.0f; }
    const PidConfig& config() const { return cfg_; }

private:
    // how much of P / D is taken away at this throttle (Betaflight getTpaFactorClassic)
    float tpa_rate_at(float throttle) {
        const bool past_low = throttle >= tpa_low_break_ || tpa_low_break_ <= 0.01f;
        if (past_low || tpa_low_faded_) {
            if (!cfg_.tpa_low_always && !tpa_low_faded_) tpa_low_faded_ = true;      // the low-throttle part only acts until the throttle first rises past it
            return tpa_mult_ * maxf(throttle - tpa_break_, 0.0f);
        }
        return tpa_low_mult_ * (tpa_low_break_ - throttle);
    }

    PidConfig cfg_; float dt_ = 0.00025f, freq_ = 4000.0f;
    float kp_[3] = {}, ki_[3] = {}, kd_[3] = {}, kf_[3] = {}, d_max_percent_[3] = {1, 1, 1};
    float iterm_limit_ = 400.0f, iterm_limit_yaw_ = 320.0f, ag_p_gain_ = 0.0f, d_max_gyro_gain_ = 0.0f, d_max_setpoint_gain_ = 0.0f;
    float tpa_break_ = 0.35f, tpa_mult_ = 0.0f, tpa_low_break_ = 0.05f, tpa_low_mult_ = 0.0f, tpa_factor_ = 1.0f; bool tpa_low_faded_ = false;
    float prev_throttle_ = 0.0f, ag_throttle_d_ = 0.0f; bool have_throttle_ = false; int8_t ag_switch_ = -1;
    float prev_d_gyro_[3] = {}; bool have_prev_[3] = {};
    bool dterm_dynamic_ = true; float dterm_lpf1_now_ = 0.0f, dyn_wait_ = 0.0f; int dyn_quantized_ = -1;
    StaticNotch d_notch_[3]; Lowpass d_lpf1_[3], d_lpf2_[3]; Pt1 relax_lpf_[3], yaw_lpf_; Pt2 d_max_range_[3], d_max_lpf_[3], ag_lpf_;
};

// Throttle boost (Betaflight USE_THROTTLE_BOOST): adds the high-passed throttle, so a fast stick movement reaches the motors
// before the smoothing lets it. boost 0..100 -> x 0.1, cutoff in Hz.
class ThrottleBoost {
public:
    void configure(uint8_t boost, uint8_t cutoff_hz, float loop_hz) { gain_ = boost * 0.1f; lpf_.set_cutoff(cutoff_hz, 1.0f / loop_hz); lpf_.reset(); }
    void reset() { lpf_.reset(); }
    // from_pilot = false: the throttle passes unchanged (the low-pass keeps following it, so nothing kicks when the pilot takes over)
    float apply(float throttle, bool from_pilot = true) {
        if (gain_ <= 0.0f) return throttle;
        const float hpf = throttle - lpf_.apply(throttle);
        return from_pilot ? clampf(throttle + gain_ * hpf, 0.0f, 1.0f) : throttle;
    }
private:
    float gain_ = 0.5f; Pt1 lpf_;
};
} // namespace fc
