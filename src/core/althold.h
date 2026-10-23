#pragma once
#include "../config/fast_code.h"
#include "types.h"
#include "kinematics.h"

// ============================================================================
// Altitude Hold - structure follows ArduCopter ModeAltHold::run(): a 5-state machine so the position target can never
// "chase a ghost" while on the ground.  Throttle stick -> CLIMB RATE around a FIXED mid (ArduPilot style, option (b) of the spec):
//   * stick inside the deadband around mid  -> hold altitude
//   * outside                               -> target climb rate proportional to deflection (harder push = faster)
// Fixed mid means the controller MUST seed its integrator from the throttle the craft is actually using when the mode
// engages (AC_PosControl::D_init_controller does the same) - otherwise throttle jumps.  Done in begin_flight().
//
// The HOVER THROTTLE - the one number that turns "accelerate up by a" into a throttle - is not a setting to get right: it differs
// by a factor of five between a heavy camera craft (50 %) and a racing quad (10 %). It is measured all the time, in every flight
// mode (HoverEstimator), and a take-off raises the throttle gently until the craft leaves the ground instead of jumping to a guess.
//
// The controller (0.28) is AC_PosControl's vertical cascade (D_update_controller), with PX4's hover normalisation:
//   stick -> wanted climb rate -> jerk-limited shaping (kinematics.h: accel <= accel_z, jerk <= jerk_z) -> desired position,
//   velocity and acceleration
//   position error -> P (kp_z)                              -> + desired velocity  = velocity target
//   velocity error -> P (kp_v) (+ I ki_v, 0 by default)     -> + desired accel     = acceleration target
//   acceleration error (target - MEASURED, low-passed acc_filt_hz) -> P (acc_p) + I (acc_i)  -> + target (feed-forward)
//   thrust = hover x (1 + that / g) / cos(tilt)
// The measured-acceleration loop is what ArduPilot has and the controller before 0.28 had not: a gust, prop wash or a sagging
// battery is answered by the thrust at once, not first through a velocity error; and its integrator - not the velocity's - takes
// up what the hover estimate gets wrong. While Altitude Hold flies, the hover estimate itself learns slowly (3 s): it is the
// feed-forward there, and a fast one fought the integrators (ArduPilot learns over 10 s, MOT_THST_HOVER). Flown by hand it keeps
// learning fast (0.4 s): the failsafe descent without a height estimate has nothing else.
// ============================================================================
namespace fc {

// Hover output from the craft's own motion: thrust ~ motor output, so   output x cos(tilt) / (1 + a_up / g)   is the output
// that would carry the weight exactly - whatever the craft is doing, and whatever the estimate was before.
// "Output" is the MOTOR value (idle + throttle x usable range), not the throttle: at zero throttle the motors still idle, and on
// a craft that hovers at 13 % that idle is 40 % of its weight.
//   standing on the ground with the motors turning (a_up = 0): the result is the output itself - never more than the true
//   value, so a wrong low reading is on the safe side and is corrected as soon as the craft flies.
// Only moderate accelerations are used (thrust is not really proportional to the output far from the hover point).
class HoverEstimator {
public:
    float min = 0.06f, max = 0.85f;
    float value = 0.40f;               // motor output 0..1 at which the craft hovers
    void reset(float start) { value = start < min ? min : (start > max ? max : start); thr_lp_ = den_lp_ = 0.0f; primed_ = false; }
    // The craft just left the ground at this throttle, accelerating up with a_up (m/s^2): a first reading, taken at once.
    void seed(float throttle, float up_cos, float a_up) { value = clampf(throttle * maxf(up_cos, 0.5f) / maxf(1.0f + a_up / kG0, 0.5f), min, max); primed_ = false; }
    // throttle: here the mean MOTOR output 0..1. a_up: world-frame upward acceleration without gravity (m/s^2).
    WB_FAST void update(float throttle, float up_cos, float a_up, float dt, bool slow = false) {
        if (up_cos < 0.7f || dt <= 0.0f) return;                                 // tilted more than 45 deg: not a hover measurement
        const float num = throttle * up_cos, den = 1.0f + a_up / kG0;
        if (!primed_) { thr_lp_ = num; den_lp_ = den; primed_ = true; }
        const float k = dt / (kSignalTc + dt);                                   // the same smoothing on both: motor lag and vibration cancel in the ratio
        thr_lp_ += k * (num - thr_lp_); den_lp_ += k * (den - den_lp_);
        if (thr_lp_ < 0.04f || den_lp_ < 0.7f || den_lp_ > 1.4f) return;         // motors barely on, or accelerating hard
        const float tc = slow ? kTcSlow : kTc;
        value = clampf(value + dt / (tc + dt) * (thr_lp_ / den_lp_ - value), min, max);
    }
private:
    static constexpr float kSignalTc = 0.10f, kTc = 0.40f, kTcSlow = 3.0f;
    float thr_lp_ = 0.0f, den_lp_ = 0.0f; bool primed_ = false;
};

enum class AhState : uint8_t { STOPPED, LANDED_IDLE, LANDED_PRE_TAKEOFF, TAKEOFF, FLYING };

struct AltHoldConfig {
    float max_climb_mps = 2.0f, max_descend_mps = 1.2f;
    // the throttle stick (0..1, as it comes from the receiver: the throttle curve is not used here) - the althold section
    float mid_stick = 0.5f;             // holds the height
    float deadband = 0.05f;             // +- this around mid_stick still holds (0.8.7 and before: 0.10 of the half stick, the same 5 %)
    float max_stick = 1.0f;             // full climb rate here; full descent rate at low_stick
    float kp_z = 1.2f;                  // 1/s   position error -> velocity
    float kp_v = 5.0f, ki_v = 0.0f;     // velocity error -> accel (m/s^2), integral of it (PSC_VELZ_P / _I: the I is the accel loop's)
    float accel_z_mps2 = 2.5f;          // the desired climb rate changes with at most this acceleration (PILOT_ACCEL_Z)
    float jerk_z_mps3 = 5.0f;           // ... and this jerk (PSC_JERK_D)
    float acc_p = 0.5f, acc_i = 1.0f;   // measured-acceleration loop: share of the error added (1/1), its integral (1/s)
    float acc_i_max = 6.0f;             // m/s^2 the integrator may hold (a hover estimate 40 % off)
    float acc_filt_hz = 20.0f;          // low-pass of the acceleration error (PSC_ACCZ_FLTE)
    float i_limit_mps2 = 4.0f;          // velocity integrator limit (ki_v)
    float takeoff_alt_m = 0.6f, takeoff_climb_mps = 0.8f;
    float low_stick = 0.06f;            // below this = "throttle low"
    float hover_default = 0.40f;        // motor output that hovers: only the guess before the first measurement
    float hover_min = 0.06f, hover_max = 0.85f;
    float land_speed_mps = 0.3f;        // |v| below this + stick low + 1 s  => landed
    float max_accel_mps2 = 6.0f;        // strongest acceleration asked for, upwards
    float max_down_accel_mps2 = 7.8f;   // ... and downwards (0.8 g: the throttle can go down to a fifth of the hover throttle)
    float takeoff_ramp_per_s = 0.40f;   // take-off: the throttle rises this fast until the craft leaves the ground
    float liftoff_v_mps = 0.15f, liftoff_z_m = 0.08f;       // "it left the ground": climbing this fast, or this far above where it stood
    float takeoff_abort_throttle = 0.90f;                   // not off the ground at this throttle (props missing, held down): stop trying
};

struct AhInput {
    bool armed = false;
    float stick01 = 0.0f;               // throttle stick 0..1 (mid = 0.5)
    float z = 0, v = 0;                 // estimated height above takeoff (m) and climb rate (m/s)
    float up_cos = 1.0f;                // cos(tilt)
    float a_up = 0.0f;                  // world-frame upward acceleration without gravity (m/s^2)
    bool failsafe_descent = false;      // controlled descent request, overrides the stick
    bool ext_climb = false; float climb_mps = 0.0f;       // an autopilot (GPS Rescue) gives the climb rate instead of the stick; it may be faster than the pilot's limits
    bool pilot_absent = false;          // the stick is not the pilot's hand (link lost or failsafe switch, from the first moment of the
                                        // guard time on): nothing is started with it, and a climb or descent it asks for is not flown
    bool alt_valid = true;              // estimator healthy
};

class AltHold {
public:
    AltHoldConfig cfg;
    HoverEstimator hover_est;
    float hover = 0.40f;                // MOTOR output at which the craft hovers (= hover_est.value, refreshed by learn() / at lift-off)

    void init() { hover_est.min = cfg.hover_min; hover_est.max = cfg.hover_max; hover_est.reset(cfg.hover_default); hover = hover_est.value; state = AhState::STOPPED; reset_ctl(0); }

    // The motor layer's range this cycle (idle rises with dynamic idle): throttle 0..1 maps to motor output idle..top.
    void set_limits(float idle, float top) { idle_ = idle; top_ = top > idle + 0.05f ? top : idle + 0.05f; }
    WB_FAST float to_motor(float throttle) const { return idle_ + throttle * (top_ - idle_); }
    WB_FAST float to_throttle(float motor) const { return clampf((motor - idle_) / (top_ - idle_), 0.0f, 1.0f); }
    // The throttle command that makes `scale` x the weight of thrust (1 = hover).
    WB_FAST float hover_throttle(float scale = 1.0f) const { return to_throttle(hover * scale); }

    // Every control cycle while ARMED, in every flight mode, with the throttle that went to the mixer: keeps the hover output
    // current (Altitude Hold needs it the moment it is switched on, the failsafe descent needs it without an altitude estimate).
    void learn(float throttle, float up_cos, float a_up, float dt) {
        if (state == AhState::TAKEOFF && !lifted_) return;           // the take-off ramp is not a hover measurement: the lift-off is (below)
        hover_est.update(to_motor(throttle), up_cos, a_up, dt, lifted()); hover = hover_est.value;
    }

    // Returns throttle 0..1 for the mixer.
    WB_FAST float update(const AhInput& in, float dt) {
        float cosT = maxf(in.up_cos, 0.5f);
        switch (state) {
        case AhState::STOPPED:
            reset_ctl(in.z);
            if (in.armed) { state = AhState::LANDED_IDLE; need_low = in.stick01 > cfg.low_stick; }
            return 0.0f;
        case AhState::LANDED_IDLE:
            reset_ctl(in.z);
            if (!in.armed) { state = AhState::STOPPED; return 0.0f; }
            if (in.pilot_absent || in.failsafe_descent) return 0.0f;                 // nobody at the stick: it stays where it stands
            if (in.stick01 <= cfg.low_stick) need_low = false;                       // throttle has been brought down since landing/arming
            if (!need_low && in.stick01 > cfg.low_stick + 0.04f) state = AhState::LANDED_PRE_TAKEOFF;
            return 0.0f;
        case AhState::LANDED_PRE_TAKEOFF:
            reset_ctl(in.z);
            if (!in.armed) { state = AhState::STOPPED; return 0.0f; }
            if (in.pilot_absent || in.failsafe_descent) { state = AhState::LANDED_IDLE; need_low = true; return 0.0f; }
            if (in.stick01 <= cfg.low_stick) { state = AhState::LANDED_IDLE; return 0.0f; }
            if (in.stick01 > cfg.mid_stick + cfg.deadband) { takeoff_t = 0; z0 = in.z; ramp_ = 0.0f; lifted_ = false; state = AhState::TAKEOFF; }
            return 0.0f;                                 // idle spin only; motors turn, no lift
        case AhState::TAKEOFF: {
            if (!in.armed) { state = AhState::STOPPED; return 0.0f; }
            if (!lifted_) {
                // Raise the throttle gently until the craft leaves the ground. The throttle at that moment IS the hover throttle
                // (a little above it: corrected by how hard the craft is already accelerating) - no guess, no jump.
                // the link went while it still stood (even for a moment): no take-off without a pilot, and the stick down first
                if (in.failsafe_descent || in.pilot_absent) { state = AhState::LANDED_IDLE; need_low = true; return 0.0f; }
                if (in.stick01 <= cfg.low_stick) { state = AhState::LANDED_PRE_TAKEOFF; return 0.0f; }
                ramp_ += cfg.takeoff_ramp_per_s * dt;
                if (in.v > cfg.liftoff_v_mps || in.z - z0 > cfg.liftoff_z_m) {
                    lifted_ = true; hover_est.seed(to_motor(ramp_), cosT, in.a_up); hover = hover_est.value;
                    reset_ctl(in.z, in.v); takeoff_t = 0;
                } else if (ramp_ >= cfg.takeoff_abort_throttle) { state = AhState::LANDED_IDLE; need_low = true; return 0.0f; }      // it does not fly: stop, and want the stick down first
                else return ramp_;
            }
            takeoff_t += dt;
            // the climb to the take-off height, shaped like any other (it starts gently); the take-off is complete when the TARGET
            // gets there (ArduCopter's takeoff), and the FLYING state stops it - or climbs on, if the stick says so
            // (the climb rate falls as sqrt(2 a d) towards the height: the target arrives there in finite time; `left` counts from
            // where the target would come to rest from its present climb - the shaping stops it in about accel_z / jerk_z seconds)
            const float left = z0 + cfg.takeoff_alt_m - (pos_des_ + vel_des_ * cfg.accel_z_mps2 / cfg.jerk_z_mps3);
            const float vt = left > 0.0f ? minf(cfg.takeoff_climb_mps, kin::sqrt_ctl(left, 0.0f, kTakeoffBrake, dt)) : 0.0f;
            float thr = control(in, dt, vt, cosT);
            bool reached = left <= 0.02f || in.z >= z0 + cfg.takeoff_alt_m - 0.05f || takeoff_t > 4.0f;
            // Off the ground and the pilot pulls the stick down (or the failsafe wants down): that is "descend", flown by the
            // FLYING state at its descent rate until the craft stands again - not a motor cut from wherever it is (ArduCopter
            // ends a take-off the same way when the climb request turns negative).
            if (reached || (!in.pilot_absent && in.stick01 <= cfg.low_stick) || in.failsafe_descent) state = AhState::FLYING;
            return thr;
        }
        case AhState::FLYING:
        default: {
            if (!in.armed) { state = AhState::STOPPED; return 0.0f; }
            float vt = 0.0f;
            bool low = !in.pilot_absent && in.stick01 <= cfg.low_stick;
            if (in.ext_climb) vt = in.climb_mps;
            else if (in.failsafe_descent) vt = -failsafe_rate(in.z);
            else if (low) vt = -cfg.max_descend_mps;
            else if (!in.pilot_absent) {                 // (nobody at the stick, no descent ordered yet: hold the height)
                vt = stick_rate(in.stick01);
            }
            float thr = control(in, dt, vt, cosT);
            // landing detection: pilot/failsafe wants down, we are not moving, and the controller is asking for (near) min throttle
            bool want_down = in.ext_climb ? in.climb_mps < 0.0f : (low || in.failsafe_descent);
            // (target far below reality while not moving = we are standing on something)
            if (want_down && fabsf(in.v) < cfg.land_speed_mps && (to_motor(thr) < hover * 0.85f || in.z - pos_des_ > 0.8f)) land_t += dt; else land_t = 0;
            if (land_t > 1.0f) { state = AhState::LANDED_IDLE; need_low = true; land_t = 0; return 0.0f; } // must bring throttle low before the next takeoff
            return thr;
        }
        }
    }

    // The climb rate the stick asks for (m/s, up +): 0 inside mid_stick +- deadband; from its edge up to max_stick linearly to the
    // full climb rate, down to low_stick to the full descent rate (an off-centre mid_stick gives each side its own travel).
    WB_FAST float stick_rate(float st) const {
        const float up0 = cfg.mid_stick + cfg.deadband, dn0 = cfg.mid_stick - cfg.deadband;
        if (st > up0) { const float span = maxf(cfg.max_stick - up0, 0.02f); return clampf((st - up0) / span, 0.0f, 1.0f) * cfg.max_climb_mps; }
        if (st < dn0) { const float span = maxf(dn0 - cfg.low_stick, 0.02f); return -clampf((dn0 - st) / span, 0.0f, 1.0f) * cfg.max_descend_mps; }
        return 0.0f;
    }
    // Call when Altitude Hold is (re)engaged in flight from another mode; seeds integrator so throttle does not jump.
    // (AC_PosControl::D_init_controller: the targets start at the craft's state, the integrator at what the present throttle gives)
    void begin_flight(float z, float current_throttle01, float up_cos, float v = 0.0f, float a_up = 0.0f) {
        state = AhState::FLYING;
        reset_ctl(z, v, a_up);
        acc_int_ = clampf(((to_motor(current_throttle01) * maxf(up_cos, 0.5f)) / hover - 1.0f) * kG0 - acc_des_, -cfg.acc_i_max, cfg.acc_i_max);
    }
    void on_disengage() { if (state != AhState::STOPPED) { state = AhState::STOPPED; } }
    AhState get_state() const { return state; }
    bool lifted() const { return state == AhState::FLYING || (state == AhState::TAKEOFF && lifted_); }      // off the ground under Altitude Hold
    float target_z() const { return pos_des_; }
    float target_v() const { return vel_des_; }          // the shaped climb rate
    float target_a() const { return acc_des_; }
    float integrator() const { return acc_int_; }        // the acceleration loop's integrator (m/s^2): the hover error it carries

private:
    WB_FAST float control(const AhInput& in, float dt, float vt, float cosT) {
        // ---- the target: moved on by its own velocity / acceleration, then shaped towards the wanted climb rate. Not further in
        //      a direction the motors cannot give any more (sat_: AC_PosControl's limit vector) - the target would run away.
        const float lim = (float)sat_;
        kin::update_pos_vel_accel(pos_des_, vel_des_, acc_des_, dt, lim, pos_des_ - in.z, vel_err_);
        // (a commanded rate beyond the pilot's own - GPS Rescue - gets the room it needs)
        const float up = maxf(cfg.max_climb_mps, vt), dn = maxf(cfg.max_descend_mps, -vt);
        // over-speed (AC_PosControl::calculate_overspeed_gain): taken over while climbing or sinking faster than the limits allow,
        // the target is brought back with acceleration and jerk raised in proportion - not a metre and more of coasting on
        float og = 1.0f;
        if (vel_des_ > up) og = kOverspeedGain * vel_des_ / up;
        else if (vel_des_ < -dn) og = kOverspeedGain * -vel_des_ / dn;
        kin::shape_vel_accel(clampf(vt, -dn, up), 0.0f, vel_des_, acc_des_, -cfg.accel_z_mps2 * og, cfg.accel_z_mps2 * og, cfg.jerk_z_mps3 * og, dt, false);
        // never further than 2 m from the craft (ArduPilot: the position error is limited by the stopping distances)
        pos_des_ = clampf(pos_des_, in.z - 2.0f, in.z + 2.0f);
        // ---- position -> velocity -> acceleration
        const float v_cmd = clampf(cfg.kp_z * (pos_des_ - in.z) + vel_des_, -dn * 1.5f, up * 1.5f);
        vel_err_ = v_cmd - in.v;
        // Anti-windup (AC_PosControl limits its integrators the same way): while the throttle stands at one of its ends, an
        // integrator may only move away from that end. Otherwise a craft held down, or one that cannot climb any faster, comes
        // back with seconds of stored-up error.
        const float vi_step = cfg.ki_v * vel_err_ * dt;
        if (!((sat_ > 0 && vi_step > 0.0f) || (sat_ < 0 && vi_step < 0.0f))) vel_int = clampf(vel_int + vi_step, -cfg.i_limit_mps2, cfg.i_limit_mps2);
        const float a_cmd = clampf(cfg.kp_v * vel_err_ + vel_int + acc_des_, -cfg.max_down_accel_mps2, cfg.max_accel_mps2);
        // ---- the measured-acceleration loop
        const float e = a_cmd - in.a_up;
        if (fresh_) { acc_err_ = e; fresh_ = false; }
        else acc_err_ += (e - acc_err_) * dt / (dt + 1.0f / (2.0f * kPi * cfg.acc_filt_hz));
        const float ai_step = cfg.acc_i * acc_err_ * dt;
        if (!((sat_ > 0 && ai_step > 0.0f) || (sat_ < 0 && ai_step < 0.0f))) acc_int_ = clampf(acc_int_ + ai_step, -cfg.acc_i_max, cfg.acc_i_max);
        const float a = clampf(a_cmd + cfg.acc_p * acc_err_ + acc_int_, -cfg.max_down_accel_mps2, cfg.max_accel_mps2 + cfg.acc_i_max);
        // thrust ~ motor output: a = g * (output * cos / hover - 1). Leaning, the output is raised by 1 / cos (at most x2, 60 deg).
        // Past 84 deg it is taken back and on its back the craft gets none: thrust would push it DOWN there (ArduPilot's
        // "inverted factor", AC_AttitudeControl_Multi::get_throttle_boosted).
        const float inverted = clampf(10.0f * in.up_cos, 0.0f, 1.0f);
        const float out = hover * maxf(1.0f + a / kG0, 0.0f) / cosT * inverted;
        sat_ = out >= top_ ? 1 : (out <= idle_ ? -1 : 0);
        return to_throttle(out);
    }
    static constexpr float kOverspeedGain = 2.0f;           // POSCONTROL_OVERSPEED_GAIN_U
    static constexpr float kTakeoffBrake = 1.0f;            // m/s^2 the take-off climb is planned to stop with
    static float failsafe_rate(float z) { return clampf(0.3f + 0.15f * z, 0.3f, 1.5f); } // slow near ground, brisker up high (BF alt-scaled descent)
    void reset_ctl(float z, float v = 0.0f, float a = 0.0f) {
        pos_des_ = z; vel_des_ = v; acc_des_ = clampf(a, -cfg.accel_z_mps2, cfg.accel_z_mps2);
        vel_int = 0; acc_int_ = 0; acc_err_ = 0; vel_err_ = 0; fresh_ = true; land_t = 0; steady_t = 0; sat_ = 0;
    }

    AhState state = AhState::STOPPED;
    bool need_low = true, lifted_ = false; int8_t sat_ = 0;      // sat_: the output of the last cycle stood at the top (+1) / the bottom (-1) of the motor range
    float idle_ = 0.055f, top_ = 1.0f;
    float pos_des_ = 0, vel_des_ = 0, acc_des_ = 0, vel_err_ = 0, acc_err_ = 0, acc_int_ = 0; bool fresh_ = true;     // shaped target, loop state
    float vel_int = 0, takeoff_t = 0, z0 = 0, land_t = 0, steady_t = 0, ramp_ = 0;
};
} // namespace fc
