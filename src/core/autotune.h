#pragma once
#include "types.h"
#include "filters.h"
#include "pid.h"

// ============================================================================
// In-flight autotune: ArduPilot's multicopter "twitch" autotune (AC_AutoTune + AC_AutoTune_Multi, libraries/AC_AutoTune),
// ported to this firmware's units (Betaflight-scale rate PID, ArduPilot-scale Stabilize P).
//
// Per axis (roll, pitch, yaw) the craft is levelled, then "twitched" - a step of the rate setpoint (or, for the angle steps, of
// the attitude target) - and the response measured: how far the rate climbs and how much it bounces back afterwards. The gains
// change after each twitch; a step is done after AUTOTUNE_SUCCESS_COUNT (4) measurements in a row agree:
//   RATE_D_UP    raise D until the rate bounces back by more than `aggr` of its peak   (updating_rate_d_up)
//   RATE_D_DOWN  lower D until it no longer does                                        (updating_rate_d_down)
//   RATE_P_UP    raise P until the rate overshoots the target by aggr / 2               (updating_rate_p_up_d_down), then back off
//   ANGLE_P_DOWN lower Stabilize P until the angle no longer overshoots                 (updating_angle_p_down)
//   ANGLE_P_UP   raise it until it does / bounces, then back off; the reached acceleration becomes the acceleration limit
// The yaw axis: P (RATE_P_UP) and Stabilize P only - ArduPilot tunes its yaw error filter there, which this firmware does not have.
// The angle steps run only when the craft flies Stabilize (cfg.angle_p): on Angle (Betaflight) Stabilize P is never used.
//
// Differences from ArduPilot, all for the units: the limits are this firmware's setting ranges (P 5..250, D 15..250, Stabilize P
// 1..12); I follows P with the pilot's own I / P ratio (ArduPilot's "I = P" is its scale: in Betaflight's it would be a tenth of
// the usual I); D max keeps its ratio to D. The level check takes absolute rates (ArduPilot compares the signed gyro).
// ============================================================================
namespace fc {

// The gains the autotune changes. Rate: Betaflight numbers (PidConfig); Stabilize: ArduPilot's (StabilizeConfig).
struct TuneGains {
    float p[3] = {45, 47, 45}, i[3] = {80, 84, 80}, d[3] = {30, 34, 0}, d_max[3] = {40, 46, 0}, f[3] = {120, 125, 120};
    float stab_p[3] = {4.5f, 4.5f, 4.5f};
    float accel_rp_dss = 1100.0f, accel_yaw_dss = 270.0f;
};

struct AutoTuneConfig {
    uint8_t axes = 7;              // bit 0 roll, 1 pitch, 2 yaw (AUTOTUNE_AXES)
    float aggr = 0.075f;           // AUTOTUNE_AGGR: the bounce that counts, a fraction of the peak (0.05 .. 0.2)
    float backoff = 0.25f;         // AUTOTUNE_GMBK: the final P / D (and Stabilize P) are this much lower than the limit found (0 .. 0.5)
    float angle_max_deg = 30.0f;   // the lean limit (ANGLE_MAX): the twitch sizes and the aborts are fractions of it
    bool angle_p = false;          // tune Stabilize P and the acceleration limits too (the craft flies Stabilize)
    float pid_hz = 4000.0f, dterm_lpf_hz = 75.0f;   // the rate loop's rate and its D low-pass: how hard D answers the first moment of a twitch
};

class AutoTune {
public:
    enum Mode : uint8_t { OFF, TUNING, FINISHED, FAILED };
    enum Step : uint8_t { WAITING_FOR_LEVEL, EXECUTING_TEST, UPDATE_GAINS, ABORT };
    enum Type : uint8_t { RATE_D_UP, RATE_D_DOWN, RATE_P_UP, ANGLE_P_DOWN, ANGLE_P_UP, COMPLETE };
    enum GainSet : uint8_t { ORIGINAL, INTRA_TEST, TEST };          // what to fly with (load_gains)
    // why it failed (the schema's autotune_fail): never level for 6 s / every twitch leaned too far even at a fifth of its size /
    // P, D or Stabilize P at their floor and still overshooting (the craft cannot be tuned this way: too much noise or flex)
    enum Fail : uint8_t { FAIL_NONE, FAIL_LEVEL, FAIL_LEAN, FAIL_P_MIN, FAIL_D_MIN, FAIL_ANGLE_P_MIN };

    // ArduPilot's numbers (AC_AutoTune.cpp / AC_AutoTune_Multi.cpp), in degrees
    static constexpr int kSuccessCount = 4;                         // AUTOTUNE_SUCCESS_COUNT
    static constexpr float kStepTimeoutMs = 2000.0f;                // AUTOTUNE_TESTING_STEP_TIMEOUT_MS
    static constexpr float kLevelAngle = 2.5f, kLevelRateRp = 5.0f, kLevelRateY = 7.5f;   // AUTOTUNE_LEVEL_ANGLE_CD / _RATE_RP_CD / _RATE_Y_CD
    static constexpr float kLevelTimeMs = 250.0f, kLevelTimeoutMs = 2000.0f;               // AUTOTUNE_REQUIRED_LEVEL_TIME_MS / _LEVEL_TIMEOUT_MS
    static constexpr float kOverrideMs = 500.0f;                    // AUTOTUNE_PILOT_OVERRIDE_TIMEOUT_MS
    static constexpr float kRateRp = 180.0f, kRateRpMin = 45.0f, kRateY = 90.0f, kRateYMin = 15.0f;   // AUTOTUNE_TARGET_(MIN_)RATE_*
    static constexpr float kStep = 0.05f;                           // AUTOTUNE_RD_STEP / RP_STEP / SP_STEP
    static constexpr float kDUpDownMargin = 0.2f;                   // AUTOTUNE_D_UP_DOWN_MARGIN
    static constexpr float kPiTest = 0.01f, kPiIntra = 0.1f;        // I during a twitch / between twitches, of its usual size
    static constexpr float kPMin = 5.0f, kPMax = 250.0f, kDMin = 15.0f, kDMax = 250.0f, kSpMin = 1.0f, kSpMax = 12.0f;   // this firmware's ranges
    static constexpr float kAccelRpMin = 100.0f, kAccelRpMax = 3000.0f, kAccelYMin = 20.0f, kAccelYMax = 1000.0f;       // (deg/s^2, the settings' ranges)
    static constexpr float kFiltRpHz = 40.0f, kFiltYHz = 10.0f;     // the measured rate's low-pass (2 x ArduPilot's default D filter; AUTOTUNE_Y_FILT_FREQ)

    AutoTuneConfig cfg;

    // Start tuning from these gains (the switch was turned on in flight).
    void start(const TuneGains& orig) {
        fail_ = FAIL_NONE; orig_ = tune_ = orig; accel_[0] = accel_[1] = orig.accel_rp_dss; accel_[2] = orig.accel_yaw_dss; mode_ = TUNING; done_ = 0; axis_ = -1; next_axis(); step_scaler_ = 1.0f; positive_ = true;
        aggr_ = clampf(cfg.aggr, 0.05f, 0.2f); backoff_ = clampf(cfg.backoff, 0.0f, 0.5f);
        override_ = true; override_ms_ = -1e9f;           // levels first, then starts (as after a pilot override)
        t_ms_ = 0.0f; begin_wait();
        if (axis_ < 0) mode_ = FINISHED;
    }
    void stop() { mode_ = OFF; }
    Fail fail_reason() const { return fail_; }
    Mode mode() const { return mode_; }
    Step step() const { return step_; }
    Type type() const { return type_; }
    int axis() const { return axis_; }                       // 0 roll, 1 pitch, 2 yaw; -1 none left
    uint8_t axes_done() const { return done_; }              // bit per axis whose tune is complete
    bool overridden() const { return override_; }
    const TuneGains& tuned() const { return tune_; }

    struct In {
        float gyro_dps[3] = {0, 0, 0};       // body rates, user convention (as FlightCore's g_f_)
        float att_deg[3] = {0, 0, 0};        // roll, pitch, yaw - user convention (as FlightCore's att_)
        float level_deg[2] = {0, 0};         // the tilt the usual controller steers to (a Loiter leaning into the wind): "level" is there (desired_roll / _pitch)
        float dt = 0.0f;
        bool pilot = false;                  // a stick out of its centre (roll, pitch, yaw, the throttle out of the hold band)
        float hover = 0.5f;                  // the throttle (0..1) the craft hovers at: how far the motors can go before they saturate
    };
    struct Out {
        bool control = false;                // true: the autotune gives the rate setpoints (feed-forward 0); false: the usual controller flies
        float sp_dps[3] = {0, 0, 0};
        GainSet gains = ORIGINAL;
    };

    // Once per control cycle while the switch is on and the craft flies (AC_AutoTune::run).
    Out update(const In& in) {
        Out o;
        if (!(in.dt > 0.0f)) return o;
        t_ms_ += in.dt * 1000.0f;
        if (mode_ != TUNING) return o;                                  // FINISHED / FAILED: the original gains, the usual controller
        if (in.pilot) { override_ = true; override_ms_ = t_ms_; }
        else if (override_ && t_ms_ - override_ms_ > kOverrideMs) {    // the sticks back in the middle for half a second: start over from level
            override_ = false; begin_wait(); hold_att_[2] = in.att_deg[2];
        }
        if (override_) return o;                                        // the pilot flies, with the original gains
        control_attitude(in, o);
        return o;
    }

    // The gains for a set (load_orig_gains / load_intra_test_gains / load_test_gains). The axis being twitched gets TEST, the others INTRA_TEST.
    TuneGains gains(GainSet set) const {
        TuneGains g = orig_;
        if (set == ORIGINAL) return g;
        for (int a = 0; a < 3; a++) { g.i[a] = orig_.i[a] * kPiIntra; g.d_max[a] = g.d[a]; }
        if (set == TEST && axis_ >= 0) {
            const int a = axis_;
            g.p[a] = tune_.p[a]; g.i[a] = i_for(a, tune_.p[a]) * kPiTest; g.f[a] = 0.0f;
            if (a != YAW) g.d[a] = tune_.d[a];
            g.d_max[a] = g.d[a]; g.stab_p[a] = tune_.stab_p[a];
        }
        return g;
    }
    // The result to save: the original gains with the completed axes replaced (save_tuning_gains).
    TuneGains result() const {
        TuneGains g = orig_;
        for (int a = 0; a < 3; a++) {
            if (!((done_ >> a) & 1u)) continue;
            g.p[a] = tune_.p[a]; g.i[a] = clampf(i_for(a, tune_.p[a]), 0.0f, 250.0f);
            if (a != YAW) { g.d_max[a] = orig_.d[a] > 0.0f ? clampf(orig_.d_max[a] * tune_.d[a] / orig_.d[a], tune_.d[a], 250.0f) : orig_.d_max[a]; g.d[a] = tune_.d[a]; }
            if (cfg.angle_p) g.stab_p[a] = tune_.stab_p[a];
        }
        if (cfg.angle_p) {                       // (roll and pitch share one limit here: the lower of the two measured)
            float rp = 1e9f; for (int a = ROLL; a <= PITCH; a++) if ((done_ >> a) & 1u) rp = minf(rp, accel_[a]);
            if (rp < 1e9f) g.accel_rp_dss = rp;
            if ((done_ >> YAW) & 1u) g.accel_yaw_dss = accel_[YAW];
        }
        return g;
    }

    // measurements of the last twitch (the log, the tests)
    float target() const { return is_angle() ? target_angle_ : target_rate_; }
    float meas_max() const { return is_angle() ? angle_max_ : rate_max_; }
    float meas_min() const { return is_angle() ? angle_min_ : rate_min_; }

private:
    bool is_angle() const { return type_ == ANGLE_P_DOWN || type_ == ANGLE_P_UP; }
    float i_for(int a, float p) const { return orig_.p[a] > 0.0f ? orig_.i[a] * p / orig_.p[a] : orig_.i[a]; }    // the pilot's I / P ratio

    void next_axis() {
        int a = axis_ + 1;
        while (a < 3 && !((cfg.axes >> a) & 1u)) a++;
        axis_ = a < 3 ? a : -1;
        if (axis_ >= 0) { seq_i_ = 0; type_ = seq(0); }
        success_ = 0; ignore_next_ = false; step_scaler_ = 1.0f;
    }
    // AC_AutoTune_Multi::set_tune_sequence, without the angle steps on Angle and without the D steps on yaw
    Type seq(int i) const {
        static const Type full[] = {RATE_D_UP, RATE_D_DOWN, RATE_P_UP, ANGLE_P_DOWN, ANGLE_P_UP, COMPLETE};
        int k = 0;
        for (Type t : full) {
            if (t != COMPLETE && axis_ == YAW && (t == RATE_D_UP || t == RATE_D_DOWN)) continue;
            if (t != COMPLETE && !cfg.angle_p && (t == ANGLE_P_DOWN || t == ANGLE_P_UP)) continue;
            if (k++ == i) return t;
        }
        return COMPLETE;
    }
    void begin_wait() { step_ = WAITING_FOR_LEVEL; step_start_ms_ = level_start_ms_ = t_ms_; }

    // AC_AutoTune::currently_level (absolute rates; the heading only when it is held - on Stabilize)
    bool level(const In& in) {
        if (t_ms_ - level_start_ms_ > 3.0f * kLevelTimeoutMs) { fail(FAIL_LEVEL); return false; }
        const float mul = clampf((t_ms_ - level_start_ms_) / kLevelTimeoutMs, 0.0f, 2.0f);
        if (fabsf(in.att_deg[ROLL] - in.level_deg[ROLL]) > mul * kLevelAngle || fabsf(in.att_deg[PITCH] - in.level_deg[PITCH]) > mul * kLevelAngle) return false;
        if (cfg.angle_p && fabsf(wrap180(in.att_deg[YAW] - hold_att_[2])) > mul * kLevelAngle) return false;
        if (fabsf(in.gyro_dps[ROLL]) > mul * kLevelRateRp || fabsf(in.gyro_dps[PITCH]) > mul * kLevelRateRp) return false;
        if (fabsf(in.gyro_dps[YAW]) > mul * kLevelRateY) return false;
        return true;
    }

    // AC_AutoTune::control_attitude
    void control_attitude(const In& in, Out& o) {
        const float dir = positive_ ? 1.0f : -1.0f;
        switch (step_) {
        case WAITING_FOR_LEVEL:
            o.gains = INTRA_TEST;                                       // the usual controller holds level (and the heading)
            if (!level(in)) step_start_ms_ = t_ms_;
            if (mode_ != TUNING) { o.gains = ORIGINAL; return; }
            if (t_ms_ - step_start_ms_ > kLevelTimeMs) {
                step_ = EXECUTING_TEST; step_start_ms_ = t_ms_; step_timeout_ms_ = kStepTimeoutMs;
                start_rate_ = in.gyro_dps[axis_]; start_angle_ = in.att_deg[axis_];
                for (int a = 0; a < 3; a++) hold_att_[a] = in.att_deg[a];
                hover_ = in.hover;
                test_init();                                            // (the twitch itself from the next cycle on, as ArduPilot: the timeout
            }                                                           //  is 3 x the time since the start - 0 in this very cycle)
            return;
        case EXECUTING_TEST:
            test_run(in, dir, o);
            if (lean_ <= -cfg.angle_max_deg * (1.0f / 5.0f) || fabsf(in.att_deg[ROLL]) > cfg.angle_max_deg * (2.5f / 3.0f) || fabsf(in.att_deg[PITCH]) > cfg.angle_max_deg * (2.5f / 3.0f))
                step_ = ABORT;                                          // AUTOTUNE_ANGLE_NEG_RP_SCALE / AUTOTUNE_ANGLE_ABORT_RP_SCALE
            if (step_ == UPDATE_GAINS) update_gains();
            if (step_ == ABORT) { begin_wait(); positive_ = !positive_; hold_att_[2] = in.att_deg[2]; }
            return;
        default:
            begin_wait(); positive_ = !positive_; o.gains = INTRA_TEST;
            return;
        }
    }

    // AC_AutoTune_Multi::test_init
    void test_init() {
        const float lean_max = cfg.angle_max_deg;
        // max_rate_step_bf_*: the rate step that takes the rate controller's output to 2 x the hover throttle (ArduPilot
        // AC_AttitudeControl_Multi::max_rate_step_bf_roll: 2 hover / (alpha_rem^3 alpha kD / dt + kP), alpha the D filter's). The D
        // part keeps a well-damped (high D) craft's twitches small: a step it could not follow before leaning too far is never asked.
        const float u_max = 2.0f * clampf(hover_, 0.1f, 0.5f) * 1000.0f;
        const float dt = 1.0f / maxf(cfg.pid_hz, 100.0f), alpha = cfg.dterm_lpf_hz > 0.0f ? dt / (dt + 1.0f / (2.0f * kPi * cfg.dterm_lpf_hz)) : 1.0f, rem = 1.0f - alpha;
        const float kd = axis_ == YAW ? kDtermScale * orig_.d[YAW] : kDtermScale * tune_.d[axis_];
        const float max_step = u_max / maxf(rem * rem * rem * alpha * kd / dt + kPtermScale * tune_.p[axis_], 1e-3f);
        if (axis_ != YAW) {
            angle_abort_ = lean_max * 0.5f;                             // AUTOTUNE_TARGET_ANGLE_MAX_RP_SCALE
            target_rate_ = clampf(max_step, kRateRpMin, maxf(kRateRpMin, step_scaler_ * kRateRp));
            target_angle_ = clampf(target_rate_ / maxf(tune_.stab_p[axis_], 0.1f), lean_max / 3.0f, lean_max * 0.5f);
            filt_.set_cutoff(kFiltRpHz, 0.001f);
        } else {
            angle_abort_ = lean_max;                                    // AUTOTUNE_TARGET_ANGLE_MAX_Y_SCALE
            target_rate_ = clampf(max_step * 0.75f, kRateYMin, maxf(kRateYMin, step_scaler_ * kRateY));
            target_angle_ = clampf(0.75f * target_rate_ / maxf(tune_.stab_p[YAW], 0.1f), lean_max / 6.0f, lean_max);
            filt_.set_cutoff(kFiltYHz, 0.001f);
        }
        filt_y_ = is_angle() ? start_rate_ : 0.0f;
        rate_max_ = rate_min_ = angle_max_ = angle_min_ = 0.0f; accel_rate_max_ = 0.0f; accel_max_ = 0.0f;
    }

    // AC_AutoTune_Multi::test_run: the twitch and its measurement
    void test_run(const In& in, float dir, Out& o) {
        o.control = true; o.gains = TEST;
        if (is_angle()) {                                               // the attitude target steps; a plain P controller holds it (sqrt controller off)
            const float tgt[3] = {hold_att_[0] + (axis_ == ROLL ? dir * target_angle_ : 0.0f), hold_att_[1] + (axis_ == PITCH ? dir * target_angle_ : 0.0f),
                                  hold_att_[2] + (axis_ == YAW ? dir * target_angle_ : 0.0f)};
            const TuneGains g = gains(TEST);
            for (int a = 0; a < 3; a++) o.sp_dps[a] = g.stab_p[a] * wrap180(tgt[a] - in.att_deg[a]);
        } else {                                                        // the rate setpoint steps on the axis, the others are asked for nothing
            o.sp_dps[axis_] = dir * target_rate_ + start_rate_;
        }
        lean_ = dir * (axis_ == YAW ? wrap180(in.att_deg[YAW] - start_angle_) : in.att_deg[axis_] - start_angle_);
        const float v = is_angle() ? dir * in.gyro_dps[axis_] : dir * (in.gyro_dps[axis_] - start_rate_);
        filt_.k = pt1_gain(axis_ == YAW ? kFiltYHz : kFiltRpHz, in.dt);
        filt_y_ += filt_.k * (v - filt_y_);
        const float rate = filt_y_;
        switch (type_) {
        case RATE_D_UP: case RATE_D_DOWN:
            test_rate(rate, target_rate_); measure_accel(rate); abort_rate(rate); break;
        case RATE_P_UP:
            test_rate(rate, target_rate_ * (1.0f + 0.5f * aggr_)); measure_accel(rate); abort_rate(rate); break;
        case ANGLE_P_DOWN: case ANGLE_P_UP:
            test_angle(rate, target_angle_ * (1.0f + 0.5f * aggr_)); measure_accel(rate - dir * start_rate_); break;
        default: break;
        }
    }
    // twitching_test_rate
    void test_rate(float rate, float rate_target_max) {
        if (rate > rate_max_) { rate_max_ = rate; rate_min_ = rate; angle_min_ = lean_; }
        if (rate < rate_min_ && rate_max_ > rate_target_max * 0.25f) { rate_min_ = rate; angle_min_ = lean_; }
        if (rate_max_ < rate_target_max * 0.6321f) step_timeout_ms_ = minf((t_ms_ - step_start_ms_) * 3.0f, kStepTimeoutMs);
        if (rate_max_ > rate_target_max) step_ = UPDATE_GAINS;
        if (rate_max_ - rate_min_ > rate_max_ * aggr_) step_ = UPDATE_GAINS;
        if (t_ms_ - step_start_ms_ >= step_timeout_ms_) step_ = UPDATE_GAINS;
    }
    // twitching_abort_rate
    void abort_rate(float rate) {
        if (lean_ >= angle_abort_) {
            if (rate == rate_min_ || angle_min_ > 0.95f * angle_abort_) {
                if (step_scaler_ > 0.2f) step_scaler_ *= 0.9f; else fail(FAIL_LEAN);
                step_ = ABORT;
            } else step_ = UPDATE_GAINS;
        }
    }
    // twitching_test_angle
    void test_angle(float rate, float angle_target_max) {
        if (lean_ > angle_max_) { angle_max_ = lean_; angle_min_ = lean_; }
        if (lean_ < angle_min_ && angle_max_ > angle_target_max * 0.25f) angle_min_ = lean_;
        if (rate > rate_max_) { rate_max_ = rate; rate_min_ = rate; }
        if (rate < rate_min_) rate_min_ = rate;
        if (angle_max_ < angle_target_max * 0.6321f) step_timeout_ms_ = minf((t_ms_ - step_start_ms_) * 3.0f, kStepTimeoutMs);
        if (angle_max_ > angle_target_max) step_ = UPDATE_GAINS;
        if (angle_max_ - angle_min_ > angle_max_ * aggr_) step_ = UPDATE_GAINS;
        if (t_ms_ - step_start_ms_ >= step_timeout_ms_) step_ = UPDATE_GAINS;
    }
    // twitching_measure_acceleration (deg/s^2)
    void measure_accel(float rate) {
        if (accel_rate_max_ < rate) { accel_rate_max_ = rate; const float t = (t_ms_ - step_start_ms_) * 0.001f; if (t > 0.0f) accel_max_ = rate / t; }
    }

    // AC_AutoTune::control_attitude, case UPDATE_GAINS
    void update_gains() {
        const int a = axis_;
        float dummy_d = kDMin * 2.0f;                                   // (yaw: no D to tune - ArduPilot's yaw error filter is not here)
        float& d = a == YAW ? dummy_d : tune_.d[a];
        switch (type_) {
        case RATE_D_UP:   rate_d_up(d, tune_.p[a]); break;
        case RATE_D_DOWN: rate_d_down(d, tune_.p[a]); break;
        case RATE_P_UP:   rate_p_up_d_down(d, tune_.p[a], a != YAW); break;
        case ANGLE_P_DOWN: angle_p_down(tune_.stab_p[a]); break;
        case ANGLE_P_UP:  angle_p_up(tune_.stab_p[a]); break;
        default: success_ = kSuccessCount; break;
        }
        if (mode_ != TUNING) return;
        if (success_ >= kSuccessCount) {
            success_ = 0; step_scaler_ = 1.0f;
            backoff(a);
            type_ = seq(++seq_i_);
            if (type_ == COMPLETE) {
                done_ |= (uint8_t)(1u << a);
                next_axis();
                if (axis_ < 0) mode_ = FINISHED;
            }
        }
    }
    // set_tuning_gains_with_backoff
    void backoff(int a) {
        if (type_ == RATE_P_UP) {
            if (a != YAW) tune_.d[a] = maxf(tune_.d[a] * (1.0f - backoff_), kDMin);
            tune_.p[a] = maxf(tune_.p[a] * (1.0f - backoff_), kPMin);
        } else if (type_ == ANGLE_P_UP) {
            tune_.stab_p[a] = clampf(tune_.stab_p[a] * (1.0f - backoff_) * (1.0f - aggr_), kSpMin, kSpMax);
            accel_[a] = a == YAW ? clampf(accel_max_, kAccelYMin, kAccelYMax) : clampf(accel_max_, kAccelRpMin, kAccelRpMax);   // (AUTOTUNE_ACCEL_*_BACKOFF 1.0)
        }
    }
    // updating_rate_d_up
    void rate_d_up(float& d, float& p) {
        if (rate_max_ > target_rate_) {
            p -= p * kStep;
            if (p < kPMin) { p = kPMin; d -= d * kStep; if (d <= kDMin) { d = kDMin; success_ = kSuccessCount; } }
        } else if (rate_max_ < target_rate_ * (1.0f - kDUpDownMargin) && p <= kPMax) {
            p += p * kStep; if (p >= kPMax) p = kPMax;
        } else if (rate_max_ - rate_min_ > rate_max_ * aggr_) {
            ignore_next_ = true; success_++;
        } else if (!ignore_next_) {
            if (success_ > 0) success_--;
            d += d * kStep * 2.0f;
            if (d >= kDMax) { d = kDMax; success_ = kSuccessCount; }
        } else ignore_next_ = false;
    }
    // updating_rate_d_down
    void rate_d_down(float& d, float& p) {
        if (rate_max_ > target_rate_) {
            p -= p * kStep;
            if (p < kPMin) { p = kPMin; d -= d * kStep; if (d <= kDMin) { d = kDMin; success_ = kSuccessCount; } }
        } else if (rate_max_ < target_rate_ * (1.0f - kDUpDownMargin) && p <= kPMax) {
            p += p * kStep; if (p >= kPMax) p = kPMax;
        } else if (rate_max_ - rate_min_ < rate_max_ * aggr_) {
            if (!ignore_next_) success_++; else ignore_next_ = false;
        } else {
            ignore_next_ = true; if (success_ > 0) success_--;
            d -= d * kStep;
            if (d <= kDMin) { d = kDMin; success_ = kSuccessCount; }
        }
    }
    // updating_rate_p_up_d_down
    void rate_p_up_d_down(float& d, float& p, bool fail_min_d) {
        if (rate_max_ > target_rate_ * (1.0f + 0.5f * aggr_)) {
            ignore_next_ = true; success_++;
        } else if (rate_max_ < target_rate_ && rate_max_ > target_rate_ * (1.0f - kDUpDownMargin) && rate_max_ - rate_min_ > rate_max_ * aggr_ && d > kDMin) {
            if (success_ > 0) success_--;
            d -= d * kStep;
            if (d <= kDMin) { d = kDMin; if (fail_min_d) fail(FAIL_D_MIN); }
            p -= p * kStep;
            if (p <= kPMin) { p = kPMin; fail(FAIL_P_MIN); }
        } else if (!ignore_next_) {
            if (success_ > 0) success_--;
            p += p * kStep;
            if (p >= kPMax) { p = kPMax; success_ = kSuccessCount; }
        } else ignore_next_ = false;
    }
    // updating_angle_p_down
    void angle_p_down(float& sp) {
        if (angle_max_ < target_angle_ * (1.0f + 0.5f * aggr_)) {
            if (!ignore_next_) success_++; else ignore_next_ = false;
        } else {
            ignore_next_ = true; if (success_ > 0) success_--;
            sp -= sp * kStep;
            if (sp <= kSpMin) { sp = kSpMin; fail(FAIL_ANGLE_P_MIN); }
        }
    }
    // updating_angle_p_up
    void angle_p_up(float& sp) {
        if (angle_max_ > target_angle_ * (1.0f + 0.5f * aggr_) || (angle_max_ > target_angle_ && rate_min_ < -rate_max_ * aggr_)) {
            ignore_next_ = true; success_++;
        } else if (!ignore_next_) {
            if (success_ > 0) success_--;
            sp += sp * kStep;
            if (sp >= kSpMax) { sp = kSpMax; success_ = kSuccessCount; }
        } else ignore_next_ = false;
    }

    void fail(Fail f) { mode_ = FAILED; fail_ = f; }
    TuneGains orig_, tune_; Fail fail_ = FAIL_NONE;
    Mode mode_ = OFF; Step step_ = WAITING_FOR_LEVEL; Type type_ = RATE_D_UP;
    int axis_ = -1, seq_i_ = 0, success_ = 0; uint8_t done_ = 0; bool ignore_next_ = false, positive_ = true, override_ = false;
    float aggr_ = 0.075f, backoff_ = 0.25f, step_scaler_ = 1.0f;
    float t_ms_ = 0, step_start_ms_ = 0, level_start_ms_ = 0, step_timeout_ms_ = kStepTimeoutMs, override_ms_ = 0;
    float start_rate_ = 0, start_angle_ = 0, hold_att_[3] = {0, 0, 0}, lean_ = 0, angle_abort_ = 15;
    float target_rate_ = 180, target_angle_ = 10, rate_max_ = 0, rate_min_ = 0, angle_max_ = 0, angle_min_ = 0, accel_rate_max_ = 0, accel_max_ = 0;
    Pt1 filt_; float filt_y_ = 0, accel_[3] = {1100, 1100, 270}, hover_ = 0.5f;
};

} // namespace fc
