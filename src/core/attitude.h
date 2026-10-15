#pragma once
#include "../config/fast_code.h"
#include "filters.h"

// ============================================================================
// Self-levelling: two ways to turn sticks + attitude into rate setpoints for the PID loop.
//
//   AngleController       Betaflight's Angle mode (flight/pid.c pidLevel()): direct, a strength number, stick feedforward.
//   StabilizeController   ArduPilot's Stabilize (AC_AttitudeControl): a shaped attitude target (acceleration- and jerk-limited),
//                         a square-root controller per axis with its own P gain (Stabilize Roll / Pitch / Yaw), and yaw holds
//                         the heading when the stick is centred.
//
// Angles in degrees and rates in deg/s, user sign convention (types.h): +roll right wing down, +pitch nose down, +yaw nose right.
// ============================================================================
namespace fc {

// ---------------------------------------------------------------------------------------------------------------------
// ArduPilot's square-root controller and input shaping (AP_Math/control.cpp).
// A P controller whose output grows only with the square root of a large error, so that the response to it never asks for
// more than `second_ord_lim` of acceleration: fast for small errors, never overshooting for big ones.
// ---------------------------------------------------------------------------------------------------------------------
WB_FAST inline float sqrt_controller(float error, float p, float second_ord_lim, float dt) {
    float rate;
    if (second_ord_lim <= 0.0f) rate = error * p;
    else if (p == 0.0f) rate = error > 0.0f ? sqrtf(2.0f * second_ord_lim * error) : (error < 0.0f ? -sqrtf(2.0f * second_ord_lim * -error) : 0.0f);
    else {
        const float linear = second_ord_lim / (p * p);           // inside this error the plain P term is gentle enough
        if (error > linear) rate = sqrtf(2.0f * second_ord_lim * (error - linear * 0.5f));
        else if (error < -linear) rate = -sqrtf(2.0f * second_ord_lim * (-error - linear * 0.5f));
        else rate = error * p;
    }
    if (dt > 0.0f) { const float m = fabsf(error) / dt; rate = clampf(rate, -m, m); }      // never past the target within one step
    return rate;
}
// The rate at which the sqrt_controller's output is changing when the error closes at rate_state: a "derivative" of the
// controller along its own curve (ArduPilot sqrt_controller_accel). Zero while moving away from the target.
inline float sqrt_controller_accel(float error, float rate_cmd, float rate_state, float p, float second_ord_lim) {
    if (!(rate_cmd * rate_state > 0.0f)) return 0.0f;
    if (!(second_ord_lim > 0.0f)) return -p * rate_state;
    if (!(p > 0.0f)) return rate_cmd == 0.0f ? 0.0f : -(second_ord_lim / fabsf(rate_cmd)) * rate_state;
    const float linear = second_ord_lim / (p * p);
    if (fabsf(error) <= linear) return -p * rate_state;
    return rate_cmd == 0.0f ? 0.0f : -(second_ord_lim / fabsf(rate_cmd)) * rate_state;
}
// accel follows accel_desired, changing by at most jerk_max per second
inline void shape_accel(float accel_desired, float& accel, float jerk_max, float dt) {
    if (jerk_max <= 0.0f || dt <= 0.0f) return;
    accel += clampf(accel_desired - accel, -jerk_max * dt, jerk_max * dt);
}
// ArduPilot shape_pos_vel_accel (1D), written for an ERROR (desired - current position): the jerk-limited acceleration that
// closes pos_error and ends at vel_desired, never asking for more than accel_max. In its linear range it is critically damped
// with the time constant accel_max / jerk_max - the target glides in without overshoot. vel_max 0 = no limit on the velocity.
WB_FAST inline void shape_pos_vel_accel(float pos_error, float vel_desired, float accel_desired, float vel, float& accel, float vel_max, float accel_max, float jerk_max, float dt, bool limit_total) {
    if (accel_max <= 0.0f || jerk_max <= 0.0f) return;
    const float k_v = jerk_max / accel_max;                      // 1 / time constant
    const float vel_corr = vel - vel_desired;                    // how fast the error is closing
    float vel_corr_cmd = sqrt_controller(pos_error, k_v, accel_max, dt);
    vel_corr_cmd += sqrt_controller_accel(pos_error, vel_corr_cmd, vel_corr, k_v, accel_max) / k_v;      // lead: start braking in time
    if (vel_max > 0.0f) vel_corr_cmd = clampf(vel_corr_cmd, -vel_max, vel_max);
    float vel_target = vel_desired + vel_corr_cmd;
    if (limit_total && vel_max > 0.0f) vel_target = clampf(vel_target, -vel_max, vel_max);
    float accel_target = clampf((vel_target - vel) * k_v, -accel_max, accel_max) + accel_desired;
    if (limit_total) accel_target = clampf(accel_target, -accel_max, accel_max);
    shape_accel(accel_target, accel, jerk_max, dt);
}
// AC_AttitudeControl::attitude_command_model: one axis of the pilot's input -> the target's rate and acceleration.
//   angle input: error_angle = desired - target angle, desired_vel 0        rate input: error_angle 0, desired_vel = the rate
WB_FAST inline void attitude_command_model(float error_angle, float desired_vel, float& target_vel, float& target_accel, float vel_max, float accel_max, float input_tc, float dt) {
    if (dt <= 0.0f) return;
    if (accel_max <= 0.0f) accel_max = 1800.0f;                  // no limit set: ArduPilot's fallback (deg/s^2)
    if (input_tc <= 0.0f) input_tc = dt * 10.0f;                 // no time constant: full acceleration within 10 cycles
    shape_pos_vel_accel(error_angle, desired_vel, 0.0f, target_vel, target_accel, vel_max, accel_max, accel_max / input_tc, dt, true);
    target_vel += target_accel * dt;
}

// ---------------------------------------------------------------------------------------------------------------------
// Angle mode (Betaflight)
// ---------------------------------------------------------------------------------------------------------------------
struct AngleConfig {
    float limit_deg = 45.0f;       // tilt at full stick
    uint8_t strength = 50;         // Betaflight "Angle strength" (level P): rate = strength / 10 x angle error
    uint8_t feedforward = 50;      // % of the stick's own speed added (level F): the craft starts tilting with the stick
    uint8_t ff_smoothing_ms = 80;  // time constant of the low-pass on that feedforward
    uint8_t earth_ref = 100;       // % of yaw "co-ordination": while tilted, a yaw command turns the craft about the vertical, not about its own tilted axis
};

constexpr float kAttitudeCutoffHz = 50.0f;     // PT3 on the commanded rate: smooths the 500 Hz attitude steps

class AngleController {
public:
    void configure(const AngleConfig& c, float loop_hz) {
        cfg_ = c; const float dt = 1.0f / loop_hz;
        gain_ = c.strength * 0.1f; ff_gain_ = c.feedforward * 0.01f; earth_ = c.earth_ref * 0.01f;
        const float ff_hz = 1000.0f / (2.0f * kPi * (c.ff_smoothing_ms < 1 ? 1 : c.ff_smoothing_ms));
        for (int a = 0; a < 2; a++) { out_lpf_[a].set_cutoff(kAttitudeCutoffHz, dt); ff_lpf_[a].set_cutoff(ff_hz, dt); }
        reset();
    }
    WB_FAST void reset() { for (int a = 0; a < 2; a++) { out_lpf_[a].reset(); ff_lpf_[a].reset(0.0f); target_[a] = 0.0f; } }

    // sp[3]: in = the Acro rate setpoints of the sticks; out = roll / pitch replaced by the levelling rates, yaw attenuated.
    // ff[2]: stick feedforward of roll / pitch (deg/s per s). max_rate[2]: top rates of the rate profile.
    // ext_target: nullptr = the sticks give the angle; else {roll, pitch} targets in degrees from an autopilot (no stick feedforward).
    WB_FAST void update(float sp[3], const float ff[2], float roll_deg, float pitch_deg, const float max_rate[2], const float* ext_target = nullptr, float ext_limit_deg = 0.0f) {
        const float att[2] = {roll_deg, pitch_deg};
        float limit = cfg_.limit_deg, angle_ff[2] = {0.0f, 0.0f};
        for (int a = 0; a < 2; a++) {
            const float inv = max_rate[a] > 0.0f ? 1.0f / max_rate[a] : 0.0f;
            float t = limit * sp[a] * inv;                       // the rate curve (expo ...) shapes the angle command as well
            float f = ff_lpf_[a].apply(limit * ff[a] * ff_gain_ * inv);
            if (ext_target) { t = ext_target[a]; f = 0.0f; }
            angle_ff[a] = f;
            const float lim = ext_target ? ext_limit_deg : limit;
            target_[a] = clampf(t, -lim, lim);
        }
        const float yaw_sp = sp[YAW];
        // earth referencing: an earth-frame turn rate w needs body rates  roll = w sin(pitch),  pitch = -w sin(roll)
        const float coupling[2] = {sin_approx(target_[PITCH] * kDeg2Rad), -sin_approx(target_[ROLL] * kDeg2Rad)};
        for (int a = 0; a < 2; a++) {
            float rate = (target_[a] - att[a]) * gain_ + angle_ff[a];
            rate += yaw_sp * coupling[a] * earth_;
            sp[a] = out_lpf_[a].apply(rate);
        }
        if (earth_ > 0.0f) sp[YAW] = yaw_sp * cos_approx(earth_ * maxf(fabsf(target_[ROLL]), fabsf(target_[PITCH])) * kDeg2Rad);
    }
    float target_deg(int axis) const { return target_[axis]; }
    const AngleConfig& config() const { return cfg_; }
private:
    AngleConfig cfg_; float gain_ = 5.0f, ff_gain_ = 0.5f, earth_ = 1.0f, target_[2] = {0, 0};
    Pt3 out_lpf_[2], ff_lpf_[2];
};

// ---------------------------------------------------------------------------------------------------------------------
// Stabilize (ArduPilot)
// ---------------------------------------------------------------------------------------------------------------------
struct StabilizeConfig {
    float limit_deg = 45.0f;                       // tilt at full stick (ANGLE_MAX)
    float kp[3] = {4.5f, 4.5f, 4.5f};              // Stabilize Roll / Pitch / Yaw P (ATC_ANG_RLL_P, ATC_ANG_PIT_P, ATC_ANG_YAW_P): deg/s per degree
    float accel_rp_dss = 1100.0f;                  // ATC_ACCEL_R_MAX / ATC_ACCEL_P_MAX, deg/s^2
    float accel_yaw_dss = 270.0f;                  // ATC_ACCEL_Y_MAX
    float input_tc = 0.15f;                        // ATC_INPUT_TC: how soft the attitude target follows the stick (0.1 crisp .. 0.3 very soft)
    bool rate_ff = true;                           // ATC_RATE_FF_ENAB: shaped target + rate feedforward; off = the target jumps with the stick
};

class StabilizeController {
public:
    static constexpr float kYawErrorMaxDeg = 45.0f;        // AC_ATTITUDE_YAW_MAX_ERROR_ANGLE: the heading target never leads by more

    void configure(const StabilizeConfig& c) { cfg_ = c; }
    // Not flying (disarmed / on the ground): the target is where the craft is, nothing is moving.
    void reset(float roll_deg, float pitch_deg, float yaw_deg) {
        target_[ROLL] = roll_deg; target_[PITCH] = pitch_deg; target_[YAW] = yaw_deg;
        for (int a = 0; a < 3; a++) rate_[a] = accel_[a] = 0.0f;
    }
    // On the ground with the motors idling the heading target follows the craft (ArduPilot reset_yaw_target_and_rate()).
    void hold_yaw(float yaw_deg) { target_[YAW] = yaw_deg; rate_[YAW] = accel_[YAW] = 0.0f; }

    // stick[2]: roll / pitch -1..1; yaw_rate_cmd: deg/s from the yaw stick. att: roll, pitch, yaw in degrees. sp[3] out.
    WB_FAST void update(const float stick[2], float yaw_rate_cmd, const float att[3], float dt, float sp[3]) {
        if (dt <= 0.0f) return;
        const float accel[3] = {cfg_.accel_rp_dss, cfg_.accel_rp_dss, cfg_.accel_yaw_dss};
        // ---- the attitude target moves with the rates decided in the previous cycle
        for (int a = 0; a < 3; a++) target_[a] += rate_[a] * dt;
        target_[YAW] = wrap180(target_[YAW]);
        // ---- shape the pilot's input into target rates
        const float desired[2] = {clampf(stick[ROLL], -1.0f, 1.0f) * cfg_.limit_deg, clampf(stick[PITCH], -1.0f, 1.0f) * cfg_.limit_deg};
        if (cfg_.rate_ff) {
            for (int a = 0; a < 2; a++) attitude_command_model(wrap180(desired[a] - target_[a]), 0.0f, rate_[a], accel_[a], 0.0f, accel[a], cfg_.input_tc, dt);
            attitude_command_model(0.0f, yaw_rate_cmd, rate_[YAW], accel_[YAW], 0.0f, accel[YAW], 0.0f, dt);      // (PILOT_Y_RATE_TC 0)
        } else {
            target_[ROLL] = desired[ROLL]; target_[PITCH] = desired[PITCH]; target_[YAW] = wrap180(target_[YAW] + yaw_rate_cmd * dt);
            for (int a = 0; a < 3; a++) rate_[a] = accel_[a] = 0.0f;
        }
        // ---- a heading target far ahead of the craft (it cannot follow: saturated, held) is pulled back
        float yaw_err = wrap180(target_[YAW] - att[YAW]);
        if (fabsf(yaw_err) > kYawErrorMaxDeg) { yaw_err = clampf(yaw_err, -kYawErrorMaxDeg, kYawErrorMaxDeg); target_[YAW] = wrap180(att[YAW] + yaw_err); }
        // ---- attitude error -> rate: square-root controller with half the acceleration limit, plus the target's own rate
        const float err[3] = {wrap180(target_[ROLL] - att[ROLL]), target_[PITCH] - att[PITCH], yaw_err};
        float e[3];                                              // rates of the Euler angles (roll, pitch, heading)
        for (int a = 0; a < 3; a++) {
            const float lim = a == YAW ? clampf(accel[a] * 0.5f, 10.0f, 120.0f) : clampf(accel[a] * 0.5f, 40.0f, 720.0f);
            e[a] = sqrt_controller(err[a], cfg_.kp[a], accel[a] > 0.0f ? lim : 0.0f, dt) + rate_[a];
        }
        // ---- Euler rates -> body rates: a heading change of a tilted craft is partly roll and pitch for the airframe
        const float sr = sin_approx(att[ROLL] * kDeg2Rad), cr = cos_approx(att[ROLL] * kDeg2Rad), sp_ = sin_approx(att[PITCH] * kDeg2Rad), cp = cos_approx(att[PITCH] * kDeg2Rad);
        sp[ROLL] = e[ROLL] + e[YAW] * sp_;
        sp[PITCH] = e[PITCH] * cr - e[YAW] * cp * sr;
        sp[YAW] = e[PITCH] * sr + e[YAW] * cp * cr;
    }
    float target_deg(int axis) const { return target_[axis]; }
    float target_rate(int axis) const { return rate_[axis]; }
    const StabilizeConfig& config() const { return cfg_; }
    float target_accel(int axis) const { return accel_[axis]; }       // deg/s^2: how fast the shaped target's rate is changing (the rate loop's feedforward)
private:
    StabilizeConfig cfg_; float target_[3] = {0, 0, 0}, rate_[3] = {0, 0, 0}, accel_[3] = {0, 0, 0};
};
} // namespace fc
