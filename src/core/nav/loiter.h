/*
 * This file is part of Windburst Project
 */

#pragma once
#include "../../config/fast_code.h"
#include "../types.h"
#include "pos_control.h"

// ============================================================================
// Loiter (0.41) - ArduCopter's Loiter mode (mode_loiter.cpp, AC_WPNav/AC_Loiter.cpp), read from its sources and written anew.
// Next to Position Hold (pos_hold.h: ArduCopter's PosHold, the stick gives the LEAN) this is the other way to hold a spot: the
// craft is always under the position controller, the stick asks for an ACCELERATION.
//
//   stick  -> the lean it would be in Angle, at most angle_max (LOIT_ANG_MAX; 0 = 2/3 of the controller's lean limit)
//          -> the acceleration that lean gives (north / east at this heading): the wanted acceleration, fed forward
//   the lean the craft will really have a moment later (AC_Loiter predicts it with the attitude controller's command model;
//          here a first order lag of input_tc_s, ATC_INPUT_TC) -> its acceleration, integrated into the wanted velocity
//   drag   the wanted velocity loses (accel at angle_max) x speed / speed_max: full stick ends at speed_max (LOIT_SPEED_MS),
//          half stick at half of it
//   brake  stick let go for brake_delay_s (LOIT_BRK_DELAY): a deceleration sqrt_controller(speed, vel_p / 2, brake_jerk) up to
//          brake_accel (LOIT_BRK_ACC_M), reached and dropped with at most brake_jerk (LOIT_BRK_JRK_M); taken off the fed-forward
//          acceleration too
//   the wanted velocity, limited to speed_max, is integrated into the target position; position, velocity and acceleration go
//          to the PosController as they are (set_pva); it holds the target, its integrator learns the wind
// Not in AC_Loiter: the fence of Position Hold (a radius around home, fence_radius_m), the way AC_Avoid limits a velocity: outwards
// the wanted velocity is at most what still stops 1 m inside it (sqrt_controller of the distance left, pos_p, brake_accel); the
// target is never let past that line. On the ground (not airborne) the sticks give the lean and nothing is held, like
// Position Hold. Units m, m/s, m/s^2; leans in degrees, pilot convention (pitch positive = forwards).
// ============================================================================
namespace fc {
namespace nav {

struct LoiterConfig {
    float speed_max_mps = 12.5f;           // LOIT_SPEED_MS (nav.loiter_speed)
    float brake_accel_mps2 = 2.5f;         // LOIT_BRK_ACC_M
    float brake_jerk_mps3 = 5.0f;          // LOIT_BRK_JRK_M
    float brake_delay_s = 1.0f;            // LOIT_BRK_DELAY
    float angle_max_deg = 0.0f;            // LOIT_ANG_MAX (0: 2/3 of the controller's lean limit)
    float input_tc_s = 0.15f;              // ATC_INPUT_TC: how quickly the craft takes a lean it is given
    float fence_radius_m = 0.0f;           // 0 = no fence (Position Hold's)
};

struct LoiterInput {
    float stick_roll = 0, stick_pitch = 0;                 // -1 .. 1 (the pilot's, after the rate curve's shape)
    float yaw_deg = 0;
    float pos_n = 0, pos_e = 0, vel_n = 0, vel_e = 0;      // from home
    float roll_deg = 0, pitch_deg = 0;                     // the lean the craft is given now (a smooth start)
    bool airborne = false;
    uint8_t reset_count = 0; float reset_step_n = 0, reset_step_e = 0;   // the estimator's position resets (PosEstimator)
};

class LoiterMode {
public:
    LoiterConfig cfg;
    PosController ctl;

    float angle_max() const {
        const float lim = ctl.cfg.angle_max_deg;
        return cfg.angle_max_deg > 0.0f ? minf(cfg.angle_max_deg, lim) : lim * (2.0f / 3.0f);
    }

    // start where the craft is, with the velocity it has (AC_Loiter::init_target: the velocity controller relaxed, the prediction
    // from the present lean); the brake may start at once
    WB_FAST void init(const LoiterInput& in) {
        ctl.init(in.pos_n, in.pos_e, in.vel_n, in.vel_e, in.roll_deg, in.pitch_deg, in.yaw_deg);
        pred_[0] = in.roll_deg; pred_[1] = in.pitch_deg; brake_ = 0; since_stick_ = cfg.brake_delay_s; reset_seen_ = in.reset_count; fence_ = false;
        out_[0] = in.roll_deg; out_[1] = in.pitch_deg;
    }

    WB_FAST void update(const LoiterInput& in, float dt, float& roll_deg, float& pitch_deg) {
        if (!(dt > 0.0f) || dt > 0.1f) { roll_deg = out_[0]; pitch_deg = out_[1]; return; }
        if (in.reset_count != reset_seen_) { ctl.on_estimator_reset(in.reset_step_n, in.reset_step_e); reset_seen_ = in.reset_count; }
        const float amax = angle_max();
        float want[2] = {clampf(in.stick_roll, -1.0f, 1.0f) * amax, clampf(in.stick_pitch, -1.0f, 1.0f) * amax};
        { const float l = sqrtf(want[0] * want[0] + want[1] * want[1]); if (l > amax && l > 0.0f) { want[0] *= amax / l; want[1] *= amax / l; } }   // (a circle, not a square)
        if (!in.airborne) {                                    // on the ground: the sticks alone, the target follows the craft
            ctl.init(in.pos_n, in.pos_e, in.vel_n, in.vel_e, 0, 0, in.yaw_deg);
            pred_[0] = want[0]; pred_[1] = want[1]; brake_ = 0; fence_ = false;
            out_[0] = roll_deg = want[0]; out_[1] = pitch_deg = want[1]; return;
        }
        // the wanted acceleration, and the one the craft will have (its lean a moment later)
        float acc[2], pacc[2];
        to_ne(want[0], want[1], in.yaw_deg, acc);
        since_stick_ = (want[0] != 0.0f || want[1] != 0.0f) ? 0.0f : since_stick_ + dt;
        const float k = dt / (cfg.input_tc_s + dt);
        pred_[0] += (want[0] - pred_[0]) * k; pred_[1] += (want[1] - pred_[1]) * k;
        to_ne(pred_[0], pred_[1], in.yaw_deg, pacc);
        // the wanted velocity: integrated, drag, brake (AC_Loiter::calc_desired_velocity)
        const float vmax = maxf(cfg.speed_max_mps, 0.2f), pilot_acc_max = kG0 * tanf(amax * kDeg2Rad);
        float vel[2] = {ctl.vel_desired_n() + pacc[0] * dt, ctl.vel_desired_e() + pacc[1] * dt};
        float speed = sqrtf(vel[0] * vel[0] + vel[1] * vel[1]);
        if (speed > 0.0f) {
            const float dir[2] = {vel[0] / speed, vel[1] / speed};
            const float drag = pilot_acc_max * speed / vmax;
            float brake_want = 0.0f;
            if (since_stick_ > maxf(cfg.brake_delay_s, dt))
                brake_want = clampf(minf(sqrt_controller(speed, ctl.cfg.vel_p * 0.5f, cfg.brake_jerk_mps3), speed / dt), 0.0f, cfg.brake_accel_mps2);
            brake_ += clampf(brake_want - brake_, -cfg.brake_jerk_mps3 * dt, cfg.brake_jerk_mps3 * dt);
            acc[0] -= dir[0] * brake_; acc[1] -= dir[1] * brake_;
            speed = maxf(speed - (drag + brake_) * dt, 0.0f);
            vel[0] = dir[0] * speed; vel[1] = dir[1] * speed;
        }
        if (speed > vmax) { vel[0] *= vmax / speed; vel[1] *= vmax / speed; }
        float tgt[2] = {ctl.target_n(), ctl.target_e()};
        fence(tgt, vel, acc);
        tgt[0] += vel[0] * dt; tgt[1] += vel[1] * dt;
        ctl.set_pva(tgt[0], tgt[1], vel[0], vel[1], acc[0], acc[1]);
        ctl.update(in.pos_n, in.pos_e, in.vel_n, in.vel_e, dt, true);
        ctl.lean(in.yaw_deg, out_[0], out_[1]);
        roll_deg = out_[0]; pitch_deg = out_[1];
    }

    bool fence_breached() const { return fence_; }
    float brake_mps2() const { return brake_; }

private:
    static WB_FAST void to_ne(float roll_deg, float pitch_deg, float yaw_deg, float out[2]) { float f, r; lean_to_accel(roll_deg, pitch_deg, f, r); body_to_ne(f, r, yaw_deg, out[0], out[1]); }
    // the fence (AC_Avoid::limit_velocity_circle): outwards no faster than what still stops 1 m inside; the target never past that line
    WB_FAST void fence(float tgt[2], float vel[2], float acc[2]) {
        fence_ = false;
        if (!(cfg.fence_radius_m > 1.0f)) return;
        const float r = cfg.fence_radius_m - 1.0f, d = sqrtf(tgt[0] * tgt[0] + tgt[1] * tgt[1]);
        if (d < 1e-3f) return;
        const float u[2] = {tgt[0] / d, tgt[1] / d};
        if (d > r) { tgt[0] = u[0] * r; tgt[1] = u[1] * r; fence_ = true; }
        const float vmax_out = sqrt_controller(maxf(r - d, 0.0f), ctl.cfg.pos_p, cfg.brake_accel_mps2);
        const float vo = vel[0] * u[0] + vel[1] * u[1];
        if (vo > vmax_out) {
            vel[0] -= (vo - vmax_out) * u[0]; vel[1] -= (vo - vmax_out) * u[1]; fence_ = true;
            const float ao = acc[0] * u[0] + acc[1] * u[1]; if (ao > 0.0f) { acc[0] -= ao * u[0]; acc[1] -= ao * u[1]; }
        }
    }
    float pred_[2] = {0, 0}, out_[2] = {0, 0}, brake_ = 0, since_stick_ = 0;
    uint8_t reset_seen_ = 0; bool fence_ = false;
};

}  // namespace nav
}  // namespace fc
