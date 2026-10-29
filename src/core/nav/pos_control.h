/*
 * This file is part of Windburst Project
 */

#pragma once
#include "../../config/fast_code.h"
#include "../types.h"
#include "../kinematics.h"
#include "nav_math.h"

// ============================================================================
// PosController - the horizontal (north / east) controller of the new stack (nav rewrite P5):
//
//   position target --sqrt controller--> velocity correction --(+ wanted velocity, jerk-limited)--> velocity error
//     --PID (error and D low-passed)--> acceleration --> lean angles for the Angle controller
//
// Used by Position Hold (a position target) and by an autopilot (GPS Rescue, later RTH / waypoints: a wanted velocity, the
// target follows the craft - P7 hooks). Units m, m/s, m/s^2; leans in degrees, pilot convention.
// The acceleration is limited to what the lean limit gives; while limited, the integrator may turn but not grow (AC_PID_2D's
// limit vector). A position reset of the estimator moves the target with it (on_estimator_reset, P6) - ArduPilot
// AC_PosControl::handle_ekf_xy_reset.
// Algorithms: ArduPilot AC_PosControl (NE part) and AP_Math control.cpp (shaping); written anew.
// ============================================================================
namespace fc {
namespace nav {

struct PosCtlConfig {
    float pos_p = 1.0f;                                // 1/s            PSC_NE_POS_P
    float vel_p = 2.0f, vel_i = 1.0f, vel_d = 0.25f;   // 1/s, 1/s^2, -  PSC_NE_VEL_P / I / D
    float filt_hz = 5.0f;                              // low-pass of the velocity error and its derivative
    float pos_err_max_m = 2.0f;                        // the target is never further than this from the craft
    float vel_corr_max_mps = 2.0f;                     // a position error is flown out at most this fast
    float accel_corr_mps2 = 2.5f;                      // the deceleration the square-root curve is laid out for
    float angle_max_deg = 30.0f;
    float i_max_mps2 = 5.0f;                           // what the integrator may hold (wind: 5 m/s^2 = 27 deg)
    float shape_accel_mps2 = 2.5f, shape_jerk_mps3 = 5.0f;     // a wanted velocity is reached with at most this (PSC_NE_ACC / PSC_JERK_NE)
};

class PosController {
public:
    PosCtlConfig cfg;

    // (Re)start without a jump: the target where the craft is, the integrator holding what the present lean already gives.
    WB_FAST void init(float pos_n, float pos_e, float vel_n, float vel_e, float roll_deg, float pitch_deg, float yaw_deg) {
        tgt_[0] = pos_n; tgt_[1] = pos_e;
        float af, ar; lean_to_accel(roll_deg, pitch_deg, af, ar);
        body_to_ne(af, ar, yaw_deg, i_[0], i_[1]); limit_len(i_, cfg.i_max_mps2);
        acc_[0] = i_[0]; acc_[1] = i_[1];
        const float v[2] = {vel_n, vel_e};
        for (int a = 0; a < 2; a++) { err_[a] = err_last_[a] = d_[a] = 0; vel_tgt_[a] = vel_des_[a] = v[a]; acc_des_[a] = 0; }
        limited_ = false; fresh_ = true;
    }
    void set_target(float pos_n, float pos_e) { tgt_[0] = pos_n; tgt_[1] = pos_e; }
    // let what it holds fade (landing, the sticks in charge): time constant tc seconds
    void relax(float dt, float tc = 0.16f) { const float k = dt / (dt + tc); for (int a = 0; a < 2; a++) { i_[a] -= i_[a] * k; acc_[a] -= acc_[a] * k; } }
    // the estimator SET its position (a reset): the target moves with it
    void on_estimator_reset(float dn, float de) { tgt_[0] += dn; tgt_[1] += de; }

    // hold_position: steer to the target (else only the wanted velocity counts and the target follows the craft).
    // vel_want_*: what an autopilot wants (m/s north / east), shaped before it is used.
    WB_FAST void update(float pos_n, float pos_e, float vel_n, float vel_e, float dt, bool hold_position, float vel_want_n = 0, float vel_want_e = 0) {
        if (!(dt > 0.0f) || dt > 0.1f) return;
        const float pos[2] = {pos_n, pos_e}, vel[2] = {vel_n, vel_e};
        { const float want[2] = {vel_want_n, vel_want_e}, ff[2] = {0, 0};
          kin::update_vel_accel_xy(vel_des_, acc_des_, dt);
          kin::shape_vel_accel_xy(want, ff, vel_des_, acc_des_, cfg.shape_accel_mps2, cfg.shape_jerk_mps3, dt, false); }
        float corr[2] = {0, 0};
        if (hold_position) {
            float e[2] = {tgt_[0] - pos[0], tgt_[1] - pos[1]};
            const float len = sqrtf(e[0] * e[0] + e[1] * e[1]);
            if (len > cfg.pos_err_max_m) { for (int a = 0; a < 2; a++) { e[a] *= cfg.pos_err_max_m / len; tgt_[a] = pos[a] + e[a]; } }   // the target is pulled along
            const float l2 = sqrtf(e[0] * e[0] + e[1] * e[1]);
            if (l2 > 1e-6f) { const float v = minf(sqrt_controller(l2, cfg.pos_p, cfg.accel_corr_mps2), cfg.vel_corr_max_mps); corr[0] = e[0] / l2 * v; corr[1] = e[1] / l2 * v; }
        } else { tgt_[0] = pos[0]; tgt_[1] = pos[1]; }
        pos_err_ = sqrtf((tgt_[0] - pos[0]) * (tgt_[0] - pos[0]) + (tgt_[1] - pos[1]) * (tgt_[1] - pos[1]));
        const float alpha = dt / (dt + 1.0f / (2.0f * kPi * cfg.filt_hz));
        for (int a = 0; a < 2; a++) {
            vel_tgt_[a] = corr[a] + vel_des_[a];
            const float e = vel_tgt_[a] - vel[a];
            if (fresh_) { err_[a] = err_last_[a] = e; d_[a] = 0; }
            else { err_last_[a] = err_[a]; err_[a] += (e - err_[a]) * alpha; d_[a] += ((err_[a] - err_last_[a]) / dt - d_[a]) * alpha; }
        }
        fresh_ = false;
        const float i_len = sqrtf(i_[0] * i_[0] + i_[1] * i_[1]);
        const float di[2] = {err_[0] * cfg.vel_i * dt, err_[1] * cfg.vel_i * dt};
        i_[0] += di[0]; i_[1] += di[1];
        if (limited_ && di[0] * lim_dir_[0] + di[1] * lim_dir_[1] > 0.0f) limit_len(i_, i_len);   // limited: it may turn, not grow
        limit_len(i_, cfg.i_max_mps2);
        float out[2]; for (int a = 0; a < 2; a++) out[a] = err_[a] * cfg.vel_p + i_[a] + d_[a] * cfg.vel_d + acc_des_[a];
        const float amax = kG0 * tanf(cfg.angle_max_deg * kDeg2Rad), olen = sqrtf(out[0] * out[0] + out[1] * out[1]);
        limited_ = olen > amax;
        if (limited_) { lim_dir_[0] = out[0] / olen; lim_dir_[1] = out[1] / olen; out[0] *= amax / olen; out[1] *= amax / olen; }
        acc_[0] = out[0]; acc_[1] = out[1];
    }
    // the lean that gives the acceleration target at this heading
    WB_FAST void lean(float yaw_deg, float& roll_deg, float& pitch_deg) const { float f, r; ne_to_body(acc_[0], acc_[1], yaw_deg, f, r); accel_to_lean(f, r, roll_deg, pitch_deg); }

    float accel_n() const { return acc_[0]; } float accel_e() const { return acc_[1]; }
    float target_n() const { return tgt_[0]; } float target_e() const { return tgt_[1]; }
    float vel_target_n() const { return vel_tgt_[0]; } float vel_target_e() const { return vel_tgt_[1]; }
    float vel_desired_n() const { return vel_des_[0]; } float vel_desired_e() const { return vel_des_[1]; }
    float i_n() const { return i_[0]; } float i_e() const { return i_[1]; }
    float pos_error() const { return pos_err_; }
    bool limited() const { return limited_; }
private:
    float vel_des_[2] = {0, 0}, acc_des_[2] = {0, 0};
    float tgt_[2] = {0, 0}, vel_tgt_[2] = {0, 0}, err_[2] = {0, 0}, err_last_[2] = {0, 0}, d_[2] = {0, 0}, i_[2] = {0, 0}, acc_[2] = {0, 0}, lim_dir_[2] = {0, 0}, pos_err_ = 0;
    bool limited_ = false, fresh_ = true;
};

}  // namespace nav
}  // namespace fc
