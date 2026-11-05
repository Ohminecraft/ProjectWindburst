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
// Stabilize (ArduPilot) - since 0.29 the quaternion attitude controller of AC_AttitudeControl:
//   input_euler_angle_roll_pitch_euler_rate_yaw   the sticks give roll / pitch ANGLES and a yaw RATE; each is shaped
//                                                 (attitude_command_model: acceleration-limited, input time constant) into an
//                                                 Euler target with its Euler rate and acceleration
//   attitude_controller_run_quat                  target and craft as quaternions; the error between them is split the way
//                                                 thrust_vector_rotation_angles does it - FIRST the rotation that brings the
//                                                 thrust vector (the craft's "up") onto the target's, THEN the heading about it.
//                                                 A tilt error is never traded against a heading error; the heading error is
//                                                 limited (45 deg) and the target pulled along when the craft cannot follow.
//   update_ang_vel_target_from_att_error          per body axis a square-root controller (Stabilize P, half the acceleration
//                                                 limit) on that error vector
//   feed-forward                                  the target's own Euler rates turned into body rates (euler_rate_to_ang_vel,
//                                                 at the TARGET attitude) and rotated into the craft's frame; faded out between
//                                                 30 and 60 deg of thrust error, beyond that the yaw is left as it is (gyro)
// Before 0.29 the error was taken per Euler angle (target roll - roll ...), which is right only for small tilts: banked and
// pitched together, or close to vertical, the axes mix and a heading error could pull the craft off its tilt.
// Internally ArduPilot's frame (NED body: roll + right wing down, pitch + NOSE UP, yaw + nose right); the interface keeps the
// firmware's user convention (pitch + = nose down).
// ---------------------------------------------------------------------------------------------------------------------
struct StabilizeConfig {
    float limit_deg = 45.0f;                       // tilt at full stick (ANGLE_MAX)
    float kp[3] = {4.5f, 4.5f, 4.5f};              // Stabilize Roll / Pitch / Yaw P (ATC_ANG_RLL_P, ATC_ANG_PIT_P, ATC_ANG_YAW_P): deg/s per degree
    float accel_rp_dss = 1100.0f;                  // ATC_ACCEL_R_MAX / ATC_ACCEL_P_MAX, deg/s^2
    float accel_yaw_dss = 270.0f;                  // ATC_ACCEL_Y_MAX
    float input_tc = 0.15f;                        // ATC_INPUT_TC: how soft the attitude target follows the stick (0.1 crisp .. 0.3 very soft)
    bool rate_ff = true;                           // ATC_RATE_FF_ENAB: shaped target + rate feedforward; off = the target jumps with the stick
};

// A unit quaternion (w, x, y, z), the rotation from a body frame to the earth (NED) frame - ArduPilot's Quaternion, the part used here.
struct Quatf {
    float w = 1, x = 0, y = 0, z = 0;
    static WB_FAST Quatf from_euler(float r, float p, float yw) {      // radians, 3-2-1
        const float cr = cosf(r * 0.5f), sr = sinf(r * 0.5f), cp = cosf(p * 0.5f), sp = sinf(p * 0.5f), cy = cosf(yw * 0.5f), sy = sinf(yw * 0.5f);
        Quatf q; q.w = cr * cp * cy + sr * sp * sy; q.x = sr * cp * cy - cr * sp * sy; q.y = cr * sp * cy + sr * cp * sy; q.z = cr * cp * sy - sr * sp * cy; return q;
    }
    WB_FAST void to_euler(float& r, float& p, float& yw) const {
        r = atan2f(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y));
        p = asinf(clampf(2.0f * (w * y - z * x), -1.0f, 1.0f));
        yw = atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
    }
    static WB_FAST Quatf from_axis_angle(const float v[3], float angle) {    // v: a unit axis
        Quatf q; const float h = 0.5f * angle, s = sinf(h); q.w = cosf(h); q.x = v[0] * s; q.y = v[1] * s; q.z = v[2] * s; return q;
    }
    static WB_FAST Quatf from_rotation_vector(const float rv[3]) {          // axis x angle
        const float a = sqrtf(rv[0] * rv[0] + rv[1] * rv[1] + rv[2] * rv[2]);
        if (a < 1e-9f) return Quatf();
        const float u[3] = {rv[0] / a, rv[1] / a, rv[2] / a}; return from_axis_angle(u, a);
    }
    WB_FAST void to_axis_angle(float v[3]) const {                            // -> rotation vector (the shorter way round)
        const float l = sqrtf(x * x + y * y + z * z);
        v[0] = v[1] = v[2] = 0.0f;
        if (l < 1e-9f) return;
        float a = 2.0f * atan2f(l, w);
        if (a > kPi) a -= 2.0f * kPi; else if (a < -kPi) a += 2.0f * kPi;
        v[0] = x / l * a; v[1] = y / l * a; v[2] = z / l * a;
    }
    WB_FAST Quatf inverse() const { Quatf q; q.w = w; q.x = -x; q.y = -y; q.z = -z; return q; }
    WB_FAST Quatf operator*(const Quatf& b) const {
        Quatf q; q.w = w * b.w - x * b.x - y * b.y - z * b.z; q.x = w * b.x + x * b.w + y * b.z - z * b.y;
        q.y = w * b.y - x * b.z + y * b.w + z * b.x; q.z = w * b.z + x * b.y - y * b.x + z * b.w; return q;
    }
    WB_FAST void rotate(const float v[3], float out[3]) const {                // q v q*
        const float tx = 2.0f * (y * v[2] - z * v[1]), ty = 2.0f * (z * v[0] - x * v[2]), tz = 2.0f * (x * v[1] - y * v[0]);
        out[0] = v[0] + w * tx + (y * tz - z * ty); out[1] = v[1] + w * ty + (z * tx - x * tz); out[2] = v[2] + w * tz + (x * ty - y * tx);
    }
};

// AC_AttitudeControl::thrust_vector_rotation_angles: the error between the craft (body) and the target, as a rotation vector in
// the body frame whose x / y bring the thrust vector onto the target's and whose z is the heading left after that.
// thrust_correction: that first rotation; thrust_angle / thrust_error: the craft's tilt and the angle between the two thrust vectors.
inline WB_FAST void thrust_vector_rotation_angles(const Quatf& target, const Quatf& body, Quatf& thrust_correction, float err[3], float& thrust_angle, float& thrust_error) {
    const float up[3] = {0.0f, 0.0f, -1.0f};
    float tt[3], bt[3]; target.rotate(up, tt); body.rotate(up, bt);
    thrust_angle = acosf(clampf(-bt[2], -1.0f, 1.0f));
    float cr[3] = {bt[1] * tt[2] - bt[2] * tt[1], bt[2] * tt[0] - bt[0] * tt[2], bt[0] * tt[1] - bt[1] * tt[0]};
    thrust_error = acosf(clampf(bt[0] * tt[0] + bt[1] * tt[1] + bt[2] * tt[2], -1.0f, 1.0f));
    const float len = sqrtf(cr[0] * cr[0] + cr[1] * cr[1] + cr[2] * cr[2]);
    if (len < 1e-9f || thrust_error < 1e-9f) { cr[0] = up[0]; cr[1] = up[1]; cr[2] = up[2]; }
    else { cr[0] /= len; cr[1] /= len; cr[2] /= len; }
    float axis_b[3]; body.inverse().rotate(cr, axis_b);                           // the axis, in the body frame
    thrust_correction = Quatf::from_axis_angle(axis_b, thrust_error);
    float rv[3]; thrust_correction.to_axis_angle(rv);
    err[0] = rv[0]; err[1] = rv[1];
    const Quatf heading = thrust_correction.inverse() * body.inverse() * target;
    heading.to_axis_angle(rv);
    err[2] = rv[2];
}

class StabilizeController {
public:
    static constexpr float kYawErrorMaxDeg = 45.0f;        // AC_ATTITUDE_YAW_MAX_ERROR_ANGLE: the heading target never leads by more
    static constexpr float kThrustErrorDeg = 30.0f;        // AC_ATTITUDE_THRUST_ERROR_ANGLE: from here on the feed-forward fades out

    void configure(const StabilizeConfig& c) { cfg_ = c; }
    // Not flying (disarmed / on the ground): the target is where the craft is, nothing is moving. (degrees, user convention)
    void reset(float roll_deg, float pitch_deg, float yaw_deg) {
        tgt_[0] = roll_deg * kDeg2Rad; tgt_[1] = -pitch_deg * kDeg2Rad; tgt_[2] = yaw_deg * kDeg2Rad;
        for (int a = 0; a < 3; a++) erate_[a] = eaccel_[a] = body_accel_[a] = 0.0f;
    }
    // On the ground with the motors idling the heading target follows the craft (ArduPilot reset_yaw_target_and_rate()).
    void hold_yaw(float yaw_deg) { tgt_[2] = yaw_deg * kDeg2Rad; erate_[2] = eaccel_[2] = 0.0f; }

    // stick[2]: roll / pitch -1..1; yaw_rate_cmd: deg/s from the yaw stick. att: roll, pitch, yaw in degrees (user convention).
    // gyro_dps: the craft's body rates (user convention), for the yaw when the thrust error is large; nullptr: not used. sp[3] out.
    // ext_target: nullptr = the sticks give the tilt; else {roll, pitch} in degrees (user convention) from an autopilot (Position Hold,
    // Loiter, the rescue - 0.44, Flycam on Stabilize), clamped to ext_limit_deg. It is shaped like the stick (ArduCopter's Loiter and
    // Altitude Hold use the same attitude controller as Stabilize).
    WB_FAST void update(const float stick[2], float yaw_rate_cmd, const float att[3], float dt, float sp[3], const float* gyro_dps = nullptr,
                        const float* ext_target = nullptr, float ext_limit_deg = 0.0f) {
        if (dt <= 0.0f) return;
        const float acc_rp = cfg_.accel_rp_dss * kDeg2Rad, acc_y = cfg_.accel_yaw_dss * kDeg2Rad;
        // ---- the Euler target moves with its rate, then the pilot's input is shaped (input_euler_angle_roll_pitch_euler_rate_yaw)
        for (int a = 0; a < 3; a++) tgt_[a] += erate_[a] * dt;
        tgt_[2] = wrap_pi(tgt_[2]);
        const float lim = cfg_.limit_deg * kDeg2Rad;
        float want_r = clampf(stick[ROLL], -1.0f, 1.0f) * lim, want_p = -clampf(stick[PITCH], -1.0f, 1.0f) * lim;
        if (ext_target) { const float el = ext_limit_deg * kDeg2Rad; want_r = clampf(ext_target[ROLL] * kDeg2Rad, -el, el); want_p = -clampf(ext_target[PITCH] * kDeg2Rad, -el, el); }
        if (cfg_.rate_ff) {
            attitude_command_model(wrap_pi(want_r - tgt_[0]), 0.0f, erate_[0], eaccel_[0], 0.0f, acc_rp, cfg_.input_tc, dt);
            attitude_command_model(wrap_pi(want_p - tgt_[1]), 0.0f, erate_[1], eaccel_[1], 0.0f, acc_rp, cfg_.input_tc, dt);
            attitude_command_model(0.0f, yaw_rate_cmd * kDeg2Rad, erate_[2], eaccel_[2], 0.0f, acc_y, 0.0f, dt);       // (PILOT_Y_RATE_TC 0)
        } else {
            tgt_[0] = want_r; tgt_[1] = want_p; tgt_[2] = wrap_pi(tgt_[2] + yaw_rate_cmd * kDeg2Rad * dt);
            for (int a = 0; a < 3; a++) erate_[a] = eaccel_[a] = 0.0f;
        }
        Quatf target = Quatf::from_euler(tgt_[0], tgt_[1], tgt_[2]);
        const Quatf body = Quatf::from_euler(att[ROLL] * kDeg2Rad, -att[PITCH] * kDeg2Rad, att[YAW] * kDeg2Rad);
        // ---- the error: thrust vector first, then the heading; the heading error limited, the target pulled along
        Quatf tc; float err[3], tilt, terr;
        thrust_vector_rotation_angles(target, body, tc, err, tilt, terr);
        const float hmax = kYawErrorMaxDeg * kDeg2Rad;
        if (fabsf(err[2]) > hmax) {
            err[2] = clampf(err[2], -hmax, hmax);
            const float hz[3] = {0.0f, 0.0f, err[2]};
            target = body * tc * Quatf::from_rotation_vector(hz);
            target.to_euler(tgt_[0], tgt_[1], tgt_[2]);
        }
        thrust_error_ = terr;
        // ---- error -> body rates: a square-root controller per axis with half the acceleration limit (update_ang_vel_target_from_att_error)
        float w[3];
        for (int a = 0; a < 3; a++) {
            const float acc = a == 2 ? acc_y : acc_rp;
            const float l2 = a == 2 ? clampf(acc * 0.5f, 10.0f * kDeg2Rad, 120.0f * kDeg2Rad) : clampf(acc * 0.5f, 40.0f * kDeg2Rad, 720.0f * kDeg2Rad);
            w[a] = sqrt_controller(err[a], cfg_.kp[a], acc > 0.0f ? l2 : 0.0f, dt);
        }
        // ---- feed-forward: the target's Euler rates as body rates at the target, rotated into the craft's frame
        float ff_t[3], acc_t[3], ff[3], accb[3];
        euler_to_body(tgt_, erate_, ff_t); euler_to_body(tgt_, eaccel_, acc_t);
        const Quatf t2b = body.inverse() * target;
        t2b.rotate(ff_t, ff); t2b.rotate(acc_t, accb);
        const float te = kThrustErrorDeg * kDeg2Rad;
        if (terr > 2.0f * te) { if (gyro_dps) w[2] = gyro_dps[YAW] * kDeg2Rad; }
        else if (terr > te) {
            const float k = 1.0f - (terr - te) / te;
            w[0] += ff[0] * k; w[1] += ff[1] * k; w[2] += ff[2];
            if (gyro_dps) w[2] = gyro_dps[YAW] * kDeg2Rad * (1.0f - k) + w[2] * k;
        } else for (int a = 0; a < 3; a++) w[a] += ff[a];
        // ---- back to the firmware's convention (pitch + = nose down), deg/s
        sp[ROLL] = w[0] * kRad2Deg; sp[PITCH] = -w[1] * kRad2Deg; sp[YAW] = w[2] * kRad2Deg;
        body_accel_[ROLL] = accb[0] * kRad2Deg; body_accel_[PITCH] = -accb[1] * kRad2Deg; body_accel_[YAW] = accb[2] * kRad2Deg;
    }
    // the attitude target, degrees in the user convention
    float target_deg(int axis) const { return axis == PITCH ? -tgt_[1] * kRad2Deg : tgt_[axis] * kRad2Deg; }
    float target_rate(int axis) const { return axis == PITCH ? -erate_[1] * kRad2Deg : erate_[axis] * kRad2Deg; }     // Euler rate of the target, deg/s
    const StabilizeConfig& config() const { return cfg_; }
    float target_accel(int axis) const { return body_accel_[axis]; }  // deg/s^2: how fast the target's BODY rate is changing (the rate loop's feedforward)
    float thrust_error_deg() const { return thrust_error_ * kRad2Deg; }
private:
    static WB_FAST float wrap_pi(float a) { while (a > kPi) a -= 2.0f * kPi; while (a < -kPi) a += 2.0f * kPi; return a; }
    // AC_AttitudeControl::euler_rate_to_ang_vel (also used for the accelerations, as euler_derivative_to_body does)
    static WB_FAST void euler_to_body(const float e[3], const float d[3], float out[3]) {
        const float sr = sinf(e[0]), cr = cosf(e[0]), sp = sinf(e[1]), cp = cosf(e[1]);
        out[0] = d[0] - sp * d[2];
        out[1] = cr * d[1] + sr * cp * d[2];
        out[2] = -sr * d[1] + cr * cp * d[2];
    }
    StabilizeConfig cfg_;
    float tgt_[3] = {0, 0, 0}, erate_[3] = {0, 0, 0}, eaccel_[3] = {0, 0, 0}, body_accel_[3] = {0, 0, 0}, thrust_error_ = 0.0f;     // ArduPilot frame, radians
};
} // namespace fc
