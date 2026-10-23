#pragma once
#include "../config/fast_code.h"
#include "types.h"

// ============================================================================
// Jerk-limited target shaping - ArduPilot AP_Math/control.cpp (shape_accel, shape_vel_accel, update_pos_vel_accel and their 2-D
// forms), the "kinematic shaping" that AC_PosControl puts between a pilot's stick and the controllers.
//
// A stick that jumps from "hold" to "climb at 2 m/s" must not hand the controller a velocity step: the velocity error would jump
// by 2 m/s, the acceleration asked for by kp_v x 2 = 10 m/s^2, the motors would slam and the craft overshoot. Shaped, the
// DESIRED velocity reaches the stick's value with the acceleration limited to accel_max and its change (the jerk) to jerk_max,
// and the desired position follows by integration. The controllers then track a target that a real craft can follow, and the
// acceleration of the target is handed down as a feed-forward.
//
// Up is positive here (the firmware's AltHold frame); the 2-D forms are north / east.
// ============================================================================
namespace fc {
namespace kin {

// sqrt_controller with ArduPilot's last-step clamp: never asks for more than the whole error in one step dt.
inline WB_FAST float sqrt_ctl(float error, float p, float second_ord_lim, float dt) {
    float r;
    if (second_ord_lim <= 0.0f) r = error * p;
    else if (p <= 0.0f) r = error > 0.0f ? sqrtf(2.0f * second_ord_lim * error) : (error < 0.0f ? -sqrtf(-2.0f * second_ord_lim * error) : 0.0f);
    else {
        const float linear = second_ord_lim / (p * p);
        if (error > linear) r = sqrtf(2.0f * second_ord_lim * (error - 0.5f * linear));
        else if (error < -linear) r = -sqrtf(2.0f * second_ord_lim * (-error - 0.5f * linear));
        else r = error * p;
    }
    if (dt > 0.0f) { const float m = fabsf(error) / dt; r = clampf(r, -m, m); }
    return r;
}

// Jerk-limited step of an acceleration towards accel_desired.
inline WB_FAST void shape_accel(float accel_desired, float& accel, float jerk_max, float dt) {
    if (!(jerk_max > 0.0f) || !(dt > 0.0f)) return;
    accel += clampf(accel_desired - accel, -jerk_max * dt, jerk_max * dt);
}

// Acceleration that brings vel to vel_desired (+ the feed-forward accel_desired) within accel_min .. accel_max, jerk-limited.
// accel_min < 0 < accel_max.
inline WB_FAST void shape_vel_accel(float vel_desired, float accel_desired, float vel, float& accel, float accel_min, float accel_max, float jerk_max, float dt, bool limit_total) {
    if (!(accel_min < 0.0f) || !(accel_max > 0.0f) || !(jerk_max > 0.0f)) return;
    const float err = vel_desired - vel;
    const float kpa = err > 0.0f ? jerk_max / accel_max : jerk_max / -accel_min;
    float a = clampf(sqrt_ctl(err, kpa, jerk_max, dt), accel_min, accel_max) + accel_desired;
    if (limit_total) a = clampf(a, accel_min, accel_max);
    shape_accel(a, accel, jerk_max, dt);
}

// Moves the target velocity forward. limit (+1 / -1 / 0): the output is saturated in that direction - then the velocity does
// not grow further in it (unless it is coming back towards 0); vel_error: the sign of the controller's velocity error.
inline WB_FAST void update_vel_accel(float& vel, float accel, float dt, float limit, float vel_error) {
    float dv = accel * dt;
    if (dv * limit > 0.0f && vel_error * limit > 0.0f) {
        if (vel * limit < 0.0f) dv = clampf(dv, -fabsf(vel), fabsf(vel));
        else dv = 0.0f;
    }
    vel += dv;
}
inline WB_FAST void update_pos_vel_accel(float& pos, float& vel, float accel, float dt, float limit, float pos_error, float vel_error) {
    float dp = vel * dt + 0.5f * accel * dt * dt;
    if (dp * limit > 0.0f && pos_error * limit > 0.0f) dp = 0.0f;
    pos += dp;
    update_vel_accel(vel, accel, dt, limit, vel_error);
}

// ---- 2-D (north, east): the same along the direction of the error, limited by length
inline WB_FAST void limit_len(float v[2], float len) { const float l = sqrtf(v[0] * v[0] + v[1] * v[1]); if (l > len && l > 0.0f) { v[0] *= len / l; v[1] *= len / l; } }
inline WB_FAST void shape_accel_xy(const float accel_desired[2], float accel[2], float jerk_max, float dt) {
    if (!(jerk_max > 0.0f) || !(dt > 0.0f)) return;
    float d[2] = {accel_desired[0] - accel[0], accel_desired[1] - accel[1]};
    limit_len(d, jerk_max * dt);
    accel[0] += d[0]; accel[1] += d[1];
}
inline WB_FAST void shape_vel_accel_xy(const float vel_desired[2], const float accel_desired[2], const float vel[2], float accel[2], float accel_max, float jerk_max, float dt, bool limit_total) {
    if (!(accel_max > 0.0f) || !(jerk_max > 0.0f)) return;
    const float e[2] = {vel_desired[0] - vel[0], vel_desired[1] - vel[1]}, len = sqrtf(e[0] * e[0] + e[1] * e[1]);
    float a[2] = {0, 0};
    if (len > 0.0f) { const float m = sqrt_ctl(len, jerk_max / accel_max, jerk_max, dt); a[0] = e[0] / len * m; a[1] = e[1] / len * m; }
    limit_len(a, accel_max);
    a[0] += accel_desired[0]; a[1] += accel_desired[1];
    if (limit_total) limit_len(a, accel_max);
    shape_accel_xy(a, accel, jerk_max, dt);
}
inline WB_FAST void update_vel_accel_xy(float vel[2], const float accel[2], float dt) { vel[0] += accel[0] * dt; vel[1] += accel[1] * dt; }

} // namespace kin
} // namespace fc
