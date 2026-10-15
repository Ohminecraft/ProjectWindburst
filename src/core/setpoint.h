#pragma once
#include "../config/fast_code.h"
#include "filters.h"
#include "rc.h"

// ============================================================================
// From receiver frames to what the rate loop needs every cycle - a port of the stick path of Betaflight's fc/rc.c:
//
//   a NEW receiver frame (50 .. 1000 per second)                         every control cycle (e.g. 4000 per second)
//     sticks -> rates -> raw setpoint (deg/s)                              setpoint, feedforward, sticks, throttle
//     feedforward = how fast the setpoint is moving (deg/s per s),          -> PT3 low-passes ("RC smoothing")
//       with jitter reduction, smoothing, boost, limit, averaging
//
// RC smoothing turns the staircase of receiver frames into a smooth curve; its cutoff follows the measured frame rate
// (auto factor) so a slow link is smoothed more. Feedforward is what makes the craft start turning with the stick instead of
// after the error has built up; it is computed per frame because that is when new information arrives.
//
// The frame rate is measured from the control cycles between frames. Frames are polled at 1 kHz, so one interval is only known
// to a millisecond: the SMOOTHED rate is used for the speed calculation (a stick delta normally spans exactly one frame period).
// A frame that comes clearly late - one was lost on the way - carries the stick movement of the whole gap: its speed is taken
// over the time that really passed (Betaflight divides by the measured interval of every frame), and after a gap too long to
// be a frame interval at all there is no speed to speak of.
// ============================================================================
namespace fc {

struct FeedforwardConfig {
    uint8_t transition = 0;        // % of stick travel around centre over which feedforward fades in (0 = full everywhere)
    uint8_t averaging = 1;         // 0 off, 1 = 2 point, 2 = 3 point, 3 = 4 point moving average
    uint8_t smooth_factor = 65;    // 0..95: low-pass on the setpoint speed, relative to a 250 Hz link
    uint8_t jitter_factor = 7;     // stick steps (of 1000 over the full travel) below which feedforward is reduced: hides finger tremor
    uint8_t boost = 15;            // adds this much of the setpoint ACCELERATION: helps the motors over their lag at the start of a move
    uint8_t max_rate_limit = 90;   // cuts feedforward when the stick is about to reach its end, to avoid overshooting the top rate
    uint8_t yaw_hold_gain = 15, yaw_hold_time = 100;   // sustained yaw feedforward (high-passed setpoint), time constant in ms
    bool interpolate = false;      // repeat-frame handling for links that may send a frame twice (everything but CRSF, the default receiver)
};

struct RcSmoothingConfig {
    bool enabled = true;
    uint8_t auto_factor = 30;          // setpoint / feedforward: cutoff = frame rate * 1.5 / (1 + factor / 10), at least 15 Hz
    uint8_t auto_factor_throttle = 30;
};

constexpr float kRxIntervalMinS = 0.0008f, kRxIntervalMaxS = 0.0655f;       // 1000 Hz .. 15 Hz
constexpr float kRcSmoothingMinCutoffHz = 15.0f;

// Smoothed receiver frame rate with outlier rejection (Betaflight shouldUpdateSmoothing()).
class RxRate {
public:
    static constexpr float kLateFrame = 1.5f;
    void reset(float hz = 150.0f) { smoothed_hz_ = hz; current_hz_ = hz; interval_s_ = 1.0f / hz; valid_ = 0; outliers_ = 0; sign_ = 0; gap_ = false; }
    // interval since the previous frame; returns true when the filter cutoffs should be recomputed (3 good intervals in a row)
    WB_FAST bool on_frame(float interval_s) {
        const bool in_range = interval_s >= kRxIntervalMinS && interval_s <= kRxIntervalMaxS;
        interval_s_ = clampf(interval_s, kRxIntervalMinS, kRxIntervalMaxS); current_hz_ = 1.0f / interval_s_;
        gap_ = interval_s > kRxIntervalMaxS;
        if (!in_range) { valid_ = 0; outliers_ = 0; return false; }
        const float delta = current_hz_ - smoothed_hz_;
        if (fabsf(delta) > smoothed_hz_ * 0.2f) {
            const int8_t s = delta < 0.0f ? -1 : 1;
            if (outliers_ == 0 || s != sign_) { outliers_ = (outliers_ == 0) ? 1 : 0; sign_ = s; }      // a reversal restarts the count
            else outliers_++;
            valid_ = 0;
        } else { smoothed_hz_ += 0.1f * delta; valid_++; outliers_ = 0; }
        if (valid_ >= 3) { valid_ = 0; return true; }
        if (outliers_ >= 3) { smoothed_hz_ = current_hz_; outliers_ = 0; return true; }             // the link rate really changed
        return false;
    }
    float smoothed_hz() const { return smoothed_hz_; }
    // What a stick delta is divided by to get its speed: the smoothed rate; the real interval when that was at least one and a
    // half frame periods (a lost frame); 0 after a gap that was no frame interval any more.
    float speed_hz() const { return gap_ ? 0.0f : (interval_s_ * smoothed_hz_ >= kLateFrame ? 1.0f / interval_s_ : smoothed_hz_); }
    float current_hz() const { return current_hz_; }
    float interval_s() const { return interval_s_; }
private:
    float smoothed_hz_ = 150.0f, current_hz_ = 150.0f, interval_s_ = 1.0f / 150.0f; int valid_ = 0, outliers_ = 0; int8_t sign_ = 0; bool gap_ = false;
};

// Feedforward for one axis, one call per receiver frame.
class FeedforwardAxis {
public:
    void configure(const FeedforwardConfig& c, int axis) {
        cfg_ = c; axis_ = axis;
        transition_ = c.transition * 0.01f; jitter_inv_ = 1.0f / (1.0f + c.jitter_factor); boost_ = 0.001f * c.boost;
        const float s = 0.01f * (c.smooth_factor > 95 ? 95 : c.smooth_factor);
        smooth_tau_ = ((1.0f / 250.0f) * s) / (1.0f - s);                       // normalised to a 250 Hz link
        yaw_hold_time_ = 0.001f * c.yaw_hold_time; yaw_hold_gain_ = c.yaw_hold_gain;
        if (c.yaw_hold_time < 100) yaw_hold_gain_ *= 150.0f / (float)(c.yaw_hold_time + 50);
        reset();
    }
    void reset() {
        prev_cmd_ = prev_sp_ = prev_speed_ = prev_speed_delta_ = prev_cmd_delta_abs_ = 0.0f; prev_interval_ = 0.004f; prev_dup_ = false;
        speed_lpf_.reset(0.0f); delta_lpf_.reset(0.0f); yaw_hold_lpf_.reset(0.0f); avg_.init(cfg_.averaging ? (uint8_t)(cfg_.averaging + 1) : 1); out_ = 0.0f;
    }
    void set_rx_rate(float smoothed_hz) { const float k = pt1_gain_from_delay(smooth_tau_, 1.0f / smoothed_hz); speed_lpf_.set_gain(k); delta_lpf_.set_gain(k); }

    // rc_command: stick after the deadband in Betaflight's units (-500..500); setpoint: raw rate setpoint (deg/s);
    // max_rate: the top rate of this axis; deflection_abs: |stick| 0..1.
    WB_FAST float on_frame(float rc_command, float setpoint, float max_rate, float deflection_abs, const RxRate& rx) {
        const float interval = rx.interval_s();
        float cmd_delta_abs = fabsf(rc_command - prev_cmd_);
        const bool duplicate = cmd_delta_abs == 0.0f;
        const float sp_delta = setpoint - prev_sp_;
        prev_cmd_ = rc_command; prev_sp_ = setpoint;
        float rate = rx.speed_hz(), speed = 0.0f;
        if (cfg_.interpolate) {
            if (!duplicate) {
                if (prev_dup_ && rate > 0.0f) rate = 1.0f / (interval + prev_interval_);    // the move took two frame periods
                speed = sp_delta * rate;
            } else if (!prev_dup_) {
                // first duplicate after movement: assume the frame was a repeat and carry the speed on - but not right before the stick's end
                if (fabsf(setpoint) < 0.90f * max_rate) { speed = prev_speed_ + prev_speed_delta_; cmd_delta_abs = prev_cmd_delta_abs_; }
            } else { speed = 0.0f; prev_speed_ = 0.0f; }                     // the stick really stopped
            prev_interval_ = interval;
        } else speed = sp_delta * rate;
        prev_dup_ = duplicate;

        float jitter = ((cmd_delta_abs + prev_cmd_delta_abs_) * 0.5f + 1.0f) * jitter_inv_;
        if (jitter > 1.0f) jitter = 1.0f;
        prev_cmd_delta_abs_ = cmd_delta_abs;

        speed = speed_lpf_.apply(speed);
        float speed_delta = speed - prev_speed_;
        prev_speed_ = speed;
        speed_delta = delta_lpf_.apply(speed_delta);
        prev_speed_delta_ = speed_delta;

        float ff = speed;
        if (axis_ != YAW) {
            ff += speed_delta * rate * boost_;
            ff *= jitter;
            if (cfg_.max_rate_limit && ff * setpoint > 0.0f) {
                const float limit = (max_rate - fabsf(setpoint)) * cfg_.max_rate_limit;
                ff = limit > 0.0f ? clampf(ff, -limit, limit) : 0.0f;
            }
        } else {
            ff *= jitter;
            yaw_hold_lpf_.set_gain(pt1_gain_from_delay(yaw_hold_time_, interval));
            ff += yaw_hold_gain_ * (setpoint - yaw_hold_lpf_.apply(setpoint));
        }
        if (transition_ != 0.0f && deflection_abs < transition_) ff *= deflection_abs / transition_;
        if (cfg_.averaging) ff = avg_.apply(ff);
        out_ = ff;
        return ff;
    }
    float value() const { return out_; }
private:
    FeedforwardConfig cfg_; int axis_ = 0;
    float transition_ = 0.0f, jitter_inv_ = 0.125f, boost_ = 0.015f, smooth_tau_ = 0.0f, yaw_hold_time_ = 0.1f, yaw_hold_gain_ = 15.0f;
    float prev_cmd_ = 0, prev_sp_ = 0, prev_speed_ = 0, prev_speed_delta_ = 0, prev_cmd_delta_abs_ = 0, prev_interval_ = 0.004f, out_ = 0; bool prev_dup_ = false;
    Pt1 speed_lpf_, delta_lpf_, yaw_hold_lpf_; MovingAverage avg_;
};

struct StickSetpoint {
    float rate[3] = {0, 0, 0};        // deg/s, smoothed
    float feedforward[3] = {0, 0, 0}; // deg/s per s, smoothed
    float stick[3] = {0, 0, 0};       // -1..1, smoothed (roll, pitch, yaw)
    float throttle = 0.0f;            // 0..1 after the throttle curve, smoothed
};

class SetpointGenerator {
public:
    void configure(const FeedforwardConfig& ff, const RcSmoothingConfig& sm, float loop_hz) {
        sm_ = sm; dt_ = 1.0f / loop_hz;
        for (int a = 0; a < 3; a++) ff_[a].configure(ff, a);
        rx_.reset();
        reset();
        update_cutoffs();
    }
    void reset() {
        for (int a = 0; a < 3; a++) { ff_[a].reset(); raw_rate_[a] = raw_ff_[a] = raw_stick_[a] = 0.0f; f_rate_[a].reset(0.0f); f_ff_[a].reset(0.0f); f_stick_[a].reset(0.0f); }
        raw_thr_ = 0.0f; f_thr_.reset(0.0f); since_frame_ = 0.0f; have_frame_ = false;
    }
    // A new receiver frame: sticks -1..1 (after the deadband), throttle 0..1 after its curve.
    WB_FAST void on_frame(const float stick[3], float throttle, const RateProfile& rates) {
        if (have_frame_ && rx_.on_frame(since_frame_)) update_cutoffs();
        have_frame_ = true; since_frame_ = 0.0f;
        for (int a = 0; a < 3; a++) {
            raw_stick_[a] = stick[a];
            raw_rate_[a] = apply_actual_rate(rates, a, stick[a]);
            raw_ff_[a] = ff_[a].on_frame(stick[a] * 500.0f, raw_rate_[a], rates.max_rate_dps[a], fabsf(stick[a]), rx_);
        }
        raw_thr_ = throttle;
    }
    // The pilot's input is not to be used (failsafe): centred sticks, no feedforward. Throttle is left to the caller.
    void neutral() { for (int a = 0; a < 3; a++) { raw_stick_[a] = raw_rate_[a] = raw_ff_[a] = 0.0f; } }
    // Every control cycle.
    WB_FAST StickSetpoint update() {
        since_frame_ += dt_;
        if (since_frame_ > 1.0f) since_frame_ = 1.0f;
        StickSetpoint o;
        for (int a = 0; a < 3; a++) {
            if (sm_.enabled) { o.rate[a] = f_rate_[a].apply(raw_rate_[a]); o.feedforward[a] = f_ff_[a].apply(raw_ff_[a]); o.stick[a] = f_stick_[a].apply(raw_stick_[a]); }
            else { o.rate[a] = raw_rate_[a]; o.feedforward[a] = raw_ff_[a]; o.stick[a] = raw_stick_[a]; }
        }
        o.throttle = sm_.enabled ? f_thr_.apply(raw_thr_) : raw_thr_;
        return o;
    }
    const RxRate& rx_rate() const { return rx_; }
    float setpoint_cutoff_hz() const { return sp_cutoff_; }
    float throttle_cutoff_hz() const { return thr_cutoff_; }
private:
    WB_FAST void update_cutoffs() {
        const float hz = rx_.smoothed_hz();
        sp_cutoff_ = maxf(kRcSmoothingMinCutoffHz, hz * 1.5f / (1.0f + sm_.auto_factor / 10.0f));
        thr_cutoff_ = maxf(kRcSmoothingMinCutoffHz, hz * 1.5f / (1.0f + sm_.auto_factor_throttle / 10.0f));
        const float k = pt1_gain(sp_cutoff_ * kPt3Correction, dt_), kt = pt1_gain(thr_cutoff_ * kPt3Correction, dt_);
        for (int a = 0; a < 3; a++) { f_rate_[a].set_gain(k); f_ff_[a].set_gain(k); f_stick_[a].set_gain(k); ff_[a].set_rx_rate(hz); }
        f_thr_.set_gain(kt);
    }
    RcSmoothingConfig sm_; float dt_ = 0.00025f, since_frame_ = 0.0f, sp_cutoff_ = 15.0f, thr_cutoff_ = 15.0f; bool have_frame_ = false;
    RxRate rx_; FeedforwardAxis ff_[3];
    float raw_rate_[3] = {}, raw_ff_[3] = {}, raw_stick_[3] = {}, raw_thr_ = 0.0f;
    Pt3 f_rate_[3], f_ff_[3], f_stick_[3], f_thr_;
};
} // namespace fc
