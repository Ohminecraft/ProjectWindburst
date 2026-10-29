/*
 * This file is part of Windburst Project
 */

#pragma once
#include "../../config/fast_code.h"
#include "../types.h"
#include "../kinematics.h"
#include "alt_state.h"

// ============================================================================
// Altitude Hold, rewritten (nav rewrite P2 + P3, docs/NAV_ROADMAP.md), in four parts that each do one thing:
//
//   HoverEstimator   the motor output that carries the craft's weight, measured in flight (P2)
//   AltSetpoint      the pilot's throttle stick (or an autopilot) -> a wanted climb rate; the height ceiling (P3, P6)
//   AltController    wanted climb rate -> jerk-limited target (position, velocity, acceleration) -> position / velocity /
//                    measured-acceleration cascade -> thrust (P2: velocity loop, P3: position loop)
//   AltHoldMode      the state machine: standing, ready, take-off (a throttle ramp until the craft lifts), flying, landing (P2, P3)
//
// Frame NED (down +) like the estimator: a climb is a negative vel_d. What the outside sees (the stick, the climb rates in the
// settings, AhState) stays in the pilot's terms.
//
// Algorithms (rewritten, not copied):
//   ArduPilot ModeAltHold / AC_PosControl (D axis): the stick as a climb rate around a fixed middle, jerk-limited shaping of the
//     target (kinematics.h), position -> velocity -> acceleration P/PI with the measured acceleration closing the inner loop, the
//     integrators frozen in the direction the motors cannot give, over-speed handling, take-off to a set height, land detection,
//     the throttle boost for tilt with the "inverted factor", estimator reset handling (handle_ekf_z_reset: the target moves
//     with the estimate's step instead of the craft flying it).
//   PX4 (mc_pos_control): thrust normalised by the hover thrust; hover thrust estimated from the acceleration (ratio of
//     low-passed output and low-passed specific force, so motor lag and vibration cancel).
//   Betaflight / INAV: altitude-scaled failsafe descent; INAV's take-off detection by climb or height gained.
// ============================================================================
namespace fc {
namespace nav {

struct AltConfig {
    // the throttle stick (0..1 as the receiver gives it; the throttle curve is not used here)
    float mid_stick = 0.5f, deadband = 0.05f, max_stick = 1.0f, low_stick = 0.06f;
    float climb_max_mps = 2.0f, descend_max_mps = 1.2f;
    // the target's shaping
    float accel_max_mps2 = 2.5f;        // the wanted climb rate changes with at most this acceleration (PILOT_ACCEL_Z)
    float jerk_max_mps3 = 5.0f;         // ... and this jerk (PSC_JERK_D)
    // the cascade
    float pos_p = 1.2f;                 // 1/s: position error -> velocity
    float vel_p = 5.0f, vel_i = 0.0f;   // 1/s, 1/s^2: velocity error -> acceleration
    float vel_i_max_mps2 = 4.0f;
    float acc_p = 0.5f, acc_i = 1.0f;   // the measured-acceleration loop: share of the error added, its integral (1/s)
    float acc_i_max_mps2 = 6.0f;        // a hover estimate 40 % off
    float acc_filt_hz = 20.0f;
    float up_accel_max_mps2 = 6.0f, down_accel_max_mps2 = 7.8f;
    float pos_err_max_m = 2.0f;         // the target is never further from the craft than this
    // take-off and landing
    float takeoff_alt_m = 0.6f, takeoff_climb_mps = 0.8f;
    float takeoff_ramp_per_s = 0.40f;   // the throttle rises this fast until the craft leaves the ground
    float liftoff_vel_mps = 0.15f, liftoff_alt_m = 0.08f;
    float takeoff_abort_throttle = 0.90f;
    float land_vel_mps = 0.3f;          // |climb rate| below this, wanting down, output near idle, 1 s: landed
    // hover
    float hover_default = 0.40f, hover_min = 0.06f, hover_max = 0.85f;
    // the ceiling (P6 fence): 0 = none. The climb rate is taken back so the craft stops at it; above it, it descends.
    float ceiling_m = 0.0f;
};

// ---------------------------------------------------------------------------------------------------------------- hover
// Thrust ~ motor output, so  output x cos(tilt) / (1 - a_d / g)  is the output that would carry the weight, whatever the craft is
// doing. Output = MOTOR value (idle + throttle x range): at zero throttle the motors still idle.
class HoverEstimator {
public:
    float min = 0.06f, max = 0.85f;
    void reset(float start) { value_ = clampf(start, min, max); num_lp_ = den_lp_ = 0.0f; primed_ = false; }
    // the craft just left the ground at this output, accelerating with acc_d (down +): a first reading, taken at once
    void seed(float output, float up_cos, float acc_d) { value_ = clampf(output * maxf(up_cos, 0.5f) / maxf(1.0f - acc_d / kG0, 0.5f), min, max); primed_ = false; }
    WB_FAST void update(float output, float up_cos, float acc_d, float dt, bool slow) {
        if (up_cos < 0.7f || !(dt > 0.0f)) return;                              // leaning more than 45 deg: not a hover measurement
        const float num = output * up_cos, den = 1.0f - acc_d / kG0;
        if (!primed_) { num_lp_ = num; den_lp_ = den; primed_ = true; }
        const float k = dt / (kSignalTc + dt);
        num_lp_ += k * (num - num_lp_); den_lp_ += k * (den - den_lp_);
        if (num_lp_ < 0.04f || den_lp_ < 0.7f || den_lp_ > 1.4f) return;       // motors barely on, or accelerating hard
        const float tc = slow ? kTcSlow : kTcFast;
        value_ = clampf(value_ + dt / (tc + dt) * (num_lp_ / den_lp_ - value_), min, max);
    }
    float value() const { return value_; }
private:
    // fast while the pilot flies (the failsafe descent has nothing else), slow under Altitude Hold where it is the feed-forward
    // and a fast one would fight the integrators (ArduPilot learns MOT_THST_HOVER over 10 s)
    static constexpr float kSignalTc = 0.10f, kTcFast = 0.40f, kTcSlow = 3.0f;
    float value_ = 0.40f, num_lp_ = 0.0f, den_lp_ = 0.0f; bool primed_ = false;
};

// ---------------------------------------------------------------------------------------------------------------- setpoint
struct AltSetpoint {
    // The climb rate the stick asks for (m/s, UP +): 0 inside mid +- deadband, linear to the full rate at max_stick / low_stick.
    static WB_FAST float stick_climb(const AltConfig& c, float st) {
        const float up0 = c.mid_stick + c.deadband, dn0 = c.mid_stick - c.deadband;
        if (st > up0) return clampf((st - up0) / maxf(c.max_stick - up0, 0.02f), 0.0f, 1.0f) * c.climb_max_mps;
        if (st < dn0) return -clampf((dn0 - st) / maxf(dn0 - c.low_stick, 0.02f), 0.0f, 1.0f) * c.descend_max_mps;
        return 0.0f;
    }
    // failsafe descent: slow near the ground, brisker high up (Betaflight's altitude-scaled descent)
    static float failsafe_climb(float height_m) { return -clampf(0.3f + 0.15f * height_m, 0.3f, 1.5f); }
    // The ceiling: no faster towards it than the craft can stop from with `accel` (and a margin of half a metre); above it, down.
    static WB_FAST float ceiling_limit(const AltConfig& c, float climb_mps, float height_m, float dt) {
        if (!(c.ceiling_m > 0.0f)) return climb_mps;
        const float room = c.ceiling_m - 0.5f - height_m;
        const float max_up = room > 0.0f ? kin::sqrt_ctl(room, c.pos_p, c.accel_max_mps2, dt) : -minf(-room * c.pos_p, c.descend_max_mps);
        return minf(climb_mps, max_up);
    }
};

// ---------------------------------------------------------------------------------------------------------------- controller
class AltController {
public:
    // the motor range this cycle: throttle 0..1 maps to output idle..top (idle rises with dynamic idle)
    void set_limits(float idle, float top) { idle_ = idle; top_ = top > idle + 0.05f ? top : idle + 0.05f; }
    WB_FAST float to_output(float throttle) const { return idle_ + throttle * (top_ - idle_); }
    WB_FAST float to_throttle(float output) const { return clampf((output - idle_) / (top_ - idle_), 0.0f, 1.0f); }
    float idle() const { return idle_; }

    // the targets start at the craft's state (AC_PosControl::init_z_controller)
    void reset(const AltConfig& c, float pos_d, float vel_d = 0.0f, float acc_d = 0.0f) {
        pos_des_ = pos_d; vel_des_ = vel_d; acc_des_ = clampf(acc_d, -c.accel_max_mps2, c.accel_max_mps2);
        vel_int_ = acc_int_ = acc_err_ = vel_err_ = 0.0f; fresh_ = true; lim_ = 0;
    }
    // ... and the acceleration integrator carries what the present output gives, so the throttle does not jump
    void seed_output(const AltConfig& c, float output, float up_cos, float hover) {
        acc_int_ = clampf(-((output * maxf(up_cos, 0.5f)) / hover - 1.0f) * kG0 - acc_des_, -c.acc_i_max_mps2, c.acc_i_max_mps2);
    }
    // the estimator SET its height (a reset): the target moves by the same step - the craft does not fly it
    void on_estimator_reset(float step_d) { pos_des_ += step_d; }

    // One step. climb_mps: wanted climb rate (UP +). Returns the throttle 0..1.
    WB_FAST float update(const AltConfig& c, float climb_mps, float pos_d, float vel_d, float acc_d, float up_cos, float hover, float dt) {
        if (!(dt > 0.0f) || dt > 0.1f) return to_throttle(last_out_);
        const float cos_t = maxf(up_cos, 0.5f);
        // ---- the target moves on by itself, not further where the motors cannot follow (lim_), then is shaped towards the
        //      wanted climb rate
        kin::update_pos_vel_accel(pos_des_, vel_des_, acc_des_, dt, (float)lim_, pos_des_ - pos_d, vel_err_);
        const float up = maxf(c.climb_max_mps, climb_mps), dn = maxf(c.descend_max_mps, -climb_mps);      // (an autopilot may ask for more)
        float og = 1.0f;                                                            // over-speed: taken over faster than the limits
        if (-vel_des_ > up) og = kOverspeedGain * -vel_des_ / up; else if (vel_des_ > dn) og = kOverspeedGain * vel_des_ / dn;
        kin::shape_vel_accel(clampf(-climb_mps, -up, dn), 0.0f, vel_des_, acc_des_, -c.accel_max_mps2 * og, c.accel_max_mps2 * og, c.jerk_max_mps3 * og, dt, false);
        pos_des_ = clampf(pos_des_, pos_d - c.pos_err_max_m, pos_d + c.pos_err_max_m);
        // ---- position (P3) -> velocity (P2) -> acceleration
        const float vel_cmd = clampf(c.pos_p * (pos_des_ - pos_d) + vel_des_, -up * 1.5f, dn * 1.5f);
        vel_err_ = vel_cmd - vel_d;
        const float vi = c.vel_i * vel_err_ * dt;
        if (!(vi * (float)lim_ > 0.0f)) vel_int_ = clampf(vel_int_ + vi, -c.vel_i_max_mps2, c.vel_i_max_mps2);
        const float acc_cmd = clampf(c.vel_p * vel_err_ + vel_int_ + acc_des_, -c.up_accel_max_mps2, c.down_accel_max_mps2);
        // ---- the measured-acceleration loop: a gust or a sagging battery is answered by the thrust at once
        const float e = acc_cmd - acc_d;
        if (fresh_) { acc_err_ = e; fresh_ = false; } else acc_err_ += (e - acc_err_) * dt / (dt + 1.0f / (2.0f * kPi * c.acc_filt_hz));
        const float ai = c.acc_i * acc_err_ * dt;
        if (!(ai * (float)lim_ > 0.0f)) acc_int_ = clampf(acc_int_ + ai, -c.acc_i_max_mps2, c.acc_i_max_mps2);
        const float a = clampf(acc_cmd + c.acc_p * acc_err_ + acc_int_, -(c.up_accel_max_mps2 + c.acc_i_max_mps2), c.down_accel_max_mps2);
        // ---- thrust: a_d = g (1 - output cos / hover). Leaning: / cos (at most x2); past 84 deg taken back, on its back none
        const float inverted = clampf(10.0f * up_cos, 0.0f, 1.0f);
        const float out = hover * maxf(1.0f - a / kG0, 0.0f) / cos_t * inverted;
        lim_ = out >= top_ ? -1 : (out <= idle_ ? 1 : 0);                        // (down +: at the top it cannot go further UP)
        last_out_ = out;
        return to_throttle(out);
    }
    float pos_target_d() const { return pos_des_; }
    float vel_target_d() const { return vel_des_; }
    float acc_target_d() const { return acc_des_; }
    float acc_integrator() const { return acc_int_; }
    float vel_error() const { return vel_err_; }
private:
    static constexpr float kOverspeedGain = 2.0f;
    float idle_ = 0.055f, top_ = 1.0f, last_out_ = 0.0f;
    float pos_des_ = 0, vel_des_ = 0, acc_des_ = 0, vel_err_ = 0, acc_err_ = 0, acc_int_ = 0, vel_int_ = 0; bool fresh_ = true; int8_t lim_ = 0;
};

// ---------------------------------------------------------------------------------------------------------------- mode
struct AltInput {
    bool armed = false;
    float stick01 = 0.0f;               // throttle stick 0..1
    float pos_d = 0, vel_d = 0, acc_d = 0;   // the estimate (NED, down +); acc_d without gravity
    float up_cos = 1.0f;
    bool failsafe_descent = false;      // a controlled descent, overrides the stick
    bool ext_climb = false; float climb_mps = 0.0f;   // an autopilot gives the climb rate (UP +)
    bool pilot_absent = false;          // the stick is not the pilot's hand (link lost / failsafe switch)
    uint8_t reset_count = 0; float reset_step_d = 0.0f;   // the estimator's height resets (AltEstimator::reset_count / reset_step_d)
};

class AltHoldMode {
public:
    AltConfig cfg;

    void init() { hover_.min = cfg.hover_min; hover_.max = cfg.hover_max; hover_.reset(cfg.hover_default); state_ = AhState::STOPPED; ctl_.reset(cfg, 0.0f); have_reset_ = false; }
    void apply_config() { hover_.min = cfg.hover_min; hover_.max = cfg.hover_max; }
    void set_limits(float idle, float top) { ctl_.set_limits(idle, top); }
    float to_output(float throttle) const { return ctl_.to_output(throttle); }
    // the throttle that makes `scale` x the weight in thrust (1 = hover)
    float hover_throttle(float scale = 1.0f) const { return ctl_.to_throttle(hover_.value() * scale); }
    float hover() const { return hover_.value(); }
    void set_hover(float output) { hover_.reset(output); }        // a hover output known from before (a test, a saved value)

    // Every control cycle while armed, in every flight mode, with the throttle that went to the mixer: keeps the hover output
    // current (Altitude Hold needs it the moment it is switched on, the failsafe descent without a height needs it too).
    void learn(float throttle, float up_cos, float acc_d, float dt) {
        if (state_ == AhState::TAKEOFF && !lifted_) return;                      // the take-off ramp is not a hover measurement
        hover_.update(ctl_.to_output(throttle), up_cos, acc_d, dt, lifted());
    }

    // Taken over in flight: from the present throttle, the targets at the craft's state.
    void begin_flight(float pos_d, float vel_d, float acc_d, float throttle, float up_cos) {
        state_ = AhState::FLYING; land_t_ = 0;
        ctl_.reset(cfg, pos_d, vel_d, acc_d); ctl_.seed_output(cfg, ctl_.to_output(throttle), up_cos, hover_.value());
    }
    void on_disengage() { state_ = AhState::STOPPED; }

    // Returns the throttle 0..1.
    WB_FAST float update(const AltInput& in, float dt) {
        // an estimator reset since the last step: the targets move with it (P6 reset handling)
        if (have_reset_ && in.reset_count != reset_seen_) { ctl_.on_estimator_reset(in.reset_step_d); takeoff_pos_d_ += in.reset_step_d; }
        reset_seen_ = in.reset_count; have_reset_ = true;
        if (!in.armed) { state_ = AhState::STOPPED; ctl_.reset(cfg, in.pos_d); return 0.0f; }
        switch (state_) {
        case AhState::STOPPED:
            ctl_.reset(cfg, in.pos_d); state_ = AhState::LANDED_IDLE; need_low_ = in.stick01 > cfg.low_stick;
            return 0.0f;
        case AhState::LANDED_IDLE:
            ctl_.reset(cfg, in.pos_d);
            if (in.pilot_absent || in.failsafe_descent) return 0.0f;             // nobody at the stick: it stays where it stands
            if (in.stick01 <= cfg.low_stick) need_low_ = false;                  // the stick has been down since landing / arming
            if (!need_low_ && in.stick01 > cfg.low_stick + 0.04f) state_ = AhState::LANDED_PRE_TAKEOFF;
            return 0.0f;
        case AhState::LANDED_PRE_TAKEOFF:
            ctl_.reset(cfg, in.pos_d);
            if (in.pilot_absent || in.failsafe_descent) { state_ = AhState::LANDED_IDLE; need_low_ = true; return 0.0f; }
            if (in.stick01 <= cfg.low_stick) { state_ = AhState::LANDED_IDLE; return 0.0f; }
            if (in.stick01 > cfg.mid_stick + cfg.deadband) { takeoff_t_ = 0; takeoff_pos_d_ = in.pos_d; ramp_ = 0.0f; lifted_ = false; state_ = AhState::TAKEOFF; }
            return 0.0f;                                                         // motors at idle, no lift
        case AhState::TAKEOFF: return takeoff(in, dt);
        case AhState::FLYING: default: return flying(in, dt);
        }
    }

    AhState state() const { return state_; }
    bool lifted() const { return state_ == AhState::FLYING || (state_ == AhState::TAKEOFF && lifted_); }
    const AltController& controller() const { return ctl_; }
    float target_height() const { return -ctl_.pos_target_d(); }

private:
    WB_FAST float takeoff(const AltInput& in, float dt) {
        const float cos_t = maxf(in.up_cos, 0.5f);
        if (!lifted_) {
            // raise the throttle gently until the craft leaves the ground: the throttle then IS (a little above) the hover throttle
            if (in.failsafe_descent || in.pilot_absent) { state_ = AhState::LANDED_IDLE; need_low_ = true; return 0.0f; }
            if (in.stick01 <= cfg.low_stick) { state_ = AhState::LANDED_PRE_TAKEOFF; return 0.0f; }
            ramp_ += cfg.takeoff_ramp_per_s * dt;
            if (-in.vel_d > cfg.liftoff_vel_mps || takeoff_pos_d_ - in.pos_d > cfg.liftoff_alt_m) {
                lifted_ = true; hover_.seed(ctl_.to_output(ramp_), cos_t, in.acc_d);
                ctl_.reset(cfg, in.pos_d, in.vel_d); takeoff_t_ = 0;
            } else if (ramp_ >= cfg.takeoff_abort_throttle) { state_ = AhState::LANDED_IDLE; need_low_ = true; return 0.0f; }   // it does not fly
            else return ramp_;
        }
        takeoff_t_ += dt;
        // climb to the take-off height, shaped like any other climb; done when the TARGET gets there (counting from where it
        // would come to rest from its present climb rate)
        const float target_up = -ctl_.pos_target_d() - ctl_.vel_target_d() * cfg.accel_max_mps2 / cfg.jerk_max_mps3;
        const float left = -takeoff_pos_d_ + cfg.takeoff_alt_m - target_up;
        const float climb = left > 0.0f ? minf(cfg.takeoff_climb_mps, kin::sqrt_ctl(left, 0.0f, kTakeoffBrake, dt)) : 0.0f;
        const float thr = ctl_.update(cfg, climb, in.pos_d, in.vel_d, in.acc_d, in.up_cos, hover_.value(), dt);
        const bool reached = left <= 0.02f || -in.pos_d >= -takeoff_pos_d_ + cfg.takeoff_alt_m - 0.05f || takeoff_t_ > 4.0f;
        // off the ground and the stick comes down (or the failsafe wants down): that is "descend", flown by FLYING
        if (reached || (!in.pilot_absent && in.stick01 <= cfg.low_stick) || in.failsafe_descent) state_ = AhState::FLYING;
        return thr;
    }
    WB_FAST float flying(const AltInput& in, float dt) {
        const bool low = !in.pilot_absent && in.stick01 <= cfg.low_stick;
        float climb = 0.0f;
        if (in.ext_climb) climb = in.climb_mps;
        else if (in.failsafe_descent) climb = AltSetpoint::failsafe_climb(-in.pos_d);
        else if (low) climb = -cfg.descend_max_mps;
        else if (!in.pilot_absent) climb = AltSetpoint::stick_climb(cfg, in.stick01);   // (nobody at the stick, no descent ordered: hold)
        climb = AltSetpoint::ceiling_limit(cfg, climb, -in.pos_d, dt);
        const float thr = ctl_.update(cfg, climb, in.pos_d, in.vel_d, in.acc_d, in.up_cos, hover_.value(), dt);
        // landed: down is wanted, the craft does not move, and the output sits near idle (or the target is far below the craft:
        // it stands on something)
        const bool want_down = in.ext_climb ? in.climb_mps < 0.0f : (low || in.failsafe_descent);
        if (want_down && fabsf(in.vel_d) < cfg.land_vel_mps && (ctl_.to_output(thr) < hover_.value() * 0.85f || ctl_.pos_target_d() - in.pos_d > 0.8f)) land_t_ += dt; else land_t_ = 0;
        if (land_t_ > 1.0f) { state_ = AhState::LANDED_IDLE; need_low_ = true; land_t_ = 0; return 0.0f; }   // the stick down before the next take-off
        return thr;
    }

    static constexpr float kTakeoffBrake = 1.0f;     // m/s^2 the take-off climb is planned to stop with
    AltController ctl_; HoverEstimator hover_;
    AhState state_ = AhState::STOPPED;
    bool need_low_ = true, lifted_ = false, have_reset_ = false; uint8_t reset_seen_ = 0;
    float takeoff_t_ = 0, takeoff_pos_d_ = 0, land_t_ = 0, ramp_ = 0;
};

}  // namespace nav
}  // namespace fc
