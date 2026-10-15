#pragma once
#include "../config/fast_code.h"
#include "types.h"

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
    WB_FAST void update(float throttle, float up_cos, float a_up, float dt) {
        if (up_cos < 0.7f || dt <= 0.0f) return;                                 // tilted more than 45 deg: not a hover measurement
        const float num = throttle * up_cos, den = 1.0f + a_up / kG0;
        if (!primed_) { thr_lp_ = num; den_lp_ = den; primed_ = true; }
        const float k = dt / (kSignalTc + dt);                                   // the same smoothing on both: motor lag and vibration cancel in the ratio
        thr_lp_ += k * (num - thr_lp_); den_lp_ += k * (den - den_lp_);
        if (thr_lp_ < 0.04f || den_lp_ < 0.7f || den_lp_ > 1.4f) return;         // motors barely on, or accelerating hard
        value = clampf(value + dt / (kTc + dt) * (thr_lp_ / den_lp_ - value), min, max);
    }
private:
    static constexpr float kSignalTc = 0.10f, kTc = 0.40f;
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
    float kp_v = 5.0f, ki_v = 3.0f;     // velocity error -> accel (m/s^2), integral of it
    float i_limit_mps2 = 4.0f;
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
        hover_est.update(to_motor(throttle), up_cos, a_up, dt); hover = hover_est.value;
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
                    reset_ctl(in.z); takeoff_t = 0;
                } else if (ramp_ >= cfg.takeoff_abort_throttle) { state = AhState::LANDED_IDLE; need_low = true; return 0.0f; }      // it does not fly: stop, and want the stick down first
                else return ramp_;
            }
            takeoff_t += dt;
            float ramp = clampf(takeoff_t / 0.6f, 0.0f, 1.0f); // ease the climb target in, don't slam
            float vt = cfg.takeoff_climb_mps * ramp;
            z_target = minf(z_target + vt * dt, z0 + cfg.takeoff_alt_m);
            float thr = control(in, dt, vt, cosT);
            bool reached = in.z >= z0 + cfg.takeoff_alt_m - 0.05f || takeoff_t > 4.0f;
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
            if (vt == 0.0f) {                            // holding: target stays put (like D_set_pos_target_from_climb_rate with 0)
                // nothing to integrate
            } else z_target += vt * dt;
            // never let the target run away from reality (e.g. motor saturation): bounded lead, like ArduPilot's pos error limit
            z_target = clampf(z_target, in.z - 2.0f, in.z + 2.0f);
            float thr = control(in, dt, vt, cosT);
            // landing detection: pilot/failsafe wants down, we are not moving, and the controller is asking for (near) min throttle
            bool want_down = in.ext_climb ? in.climb_mps < 0.0f : (low || in.failsafe_descent);
            // (target far below reality while not moving = we are standing on something)
            if (want_down && fabsf(in.v) < cfg.land_speed_mps && (to_motor(thr) < hover * 0.85f || in.z - z_target > 0.8f)) land_t += dt; else land_t = 0;
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
    void begin_flight(float z, float current_throttle01, float up_cos) {
        state = AhState::FLYING;
        z_target = z; vel_int = ((to_motor(current_throttle01) * maxf(up_cos, 0.5f)) / hover - 1.0f) * kG0;
        vel_int = clampf(vel_int, -cfg.i_limit_mps2, cfg.i_limit_mps2);
        land_t = 0; steady_t = 0; sat_ = 0;
    }
    void on_disengage() { if (state != AhState::STOPPED) { state = AhState::STOPPED; } }
    AhState get_state() const { return state; }
    bool lifted() const { return state == AhState::FLYING || (state == AhState::TAKEOFF && lifted_); }      // off the ground under Altitude Hold
    float target_z() const { return z_target; }
    float integrator() const { return vel_int; }

private:
    WB_FAST float control(const AhInput& in, float dt, float vt_ff, float cosT) {
        // (the limits follow a commanded rate that is beyond the pilot's own)
        float v_des = clampf(cfg.kp_z * (z_target - in.z) + vt_ff, -maxf(cfg.max_descend_mps, -vt_ff) * 1.5f, maxf(cfg.max_climb_mps, vt_ff) * 1.5f);
        float verr = v_des - in.v;
        // Anti-windup (AC_PosControl limits its integrator the same way): while the throttle stands at one of its ends, the
        // integrator may only move away from that end. Otherwise a craft held down, or one that cannot climb any faster, comes
        // back with seconds of stored-up error.
        const float i_step = cfg.ki_v * verr * dt;
        if (!((sat_ > 0 && i_step > 0.0f) || (sat_ < 0 && i_step < 0.0f))) vel_int = clampf(vel_int + i_step, -cfg.i_limit_mps2, cfg.i_limit_mps2);
        float a = clampf(cfg.kp_v * verr + vel_int, -cfg.max_down_accel_mps2, cfg.max_accel_mps2);
        // thrust ~ motor output: a = g * (output * cos / hover - 1). Leaning, the output is raised by 1 / cos (at most x2, 60 deg).
        // Past 84 deg it is taken back and on its back the craft gets none: thrust would push it DOWN there (ArduPilot's
        // "inverted factor", AC_AttitudeControl_Multi::get_throttle_boosted).
        const float inverted = clampf(10.0f * in.up_cos, 0.0f, 1.0f);
        const float out = hover * (1.0f + a / kG0) / cosT * inverted;
        sat_ = out >= top_ ? 1 : (out <= idle_ ? -1 : 0);
        return to_throttle(out);
    }
    static float failsafe_rate(float z) { return clampf(0.3f + 0.15f * z, 0.3f, 1.5f); } // slow near ground, brisker up high (BF alt-scaled descent)
    void reset_ctl(float z) { z_target = z; vel_int = 0; land_t = 0; steady_t = 0; sat_ = 0; }

    AhState state = AhState::STOPPED;
    bool need_low = true, lifted_ = false; int8_t sat_ = 0;      // sat_: the output of the last cycle stood at the top (+1) / the bottom (-1) of the motor range
    float idle_ = 0.055f, top_ = 1.0f;
    float z_target = 0, vel_int = 0, takeoff_t = 0, z0 = 0, land_t = 0, steady_t = 0, ramp_ = 0;
};
} // namespace fc
