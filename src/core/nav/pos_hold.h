/*
 * This file is part of Windburst Project
 */

#pragma once
#include "../../config/fast_code.h"
#include "../types.h"
#include "pos_control.h"
#include "pos_state.h"

// ============================================================================
// Position Hold, rewritten (nav rewrite P5; the fence P6). The pilot's feel is ArduCopter's PosHold (mode_poshold.cpp), per axis:
//
//   PILOT        the stick gives the lean (as in Angle) plus the lean learned against the wind; let go, the lean is taken back
//                quickly but not dropped
//   BRAKE        stick let go: a lean against the motion, larger the faster the craft moves, built up and taken back at
//                brake_rate - an open-loop brake that feels the same every time
//   BRAKE_READY  this axis has stopped; it waits for the other
//   TO_LOITER    both stopped: the brake hands over to the position controller (PosController) in to_loiter_s
//   LOITER       the controller holds the spot; standing still, the acceleration it needs is the wind (learned for PILOT)
//   TO_PILOT     a stick moved: from the controller's lean to the pilot's in to_pilot_s
//
// Roll and pitch are separate until both have stopped. Only roll / pitch leans come out; height and yaw are not touched.
// New against the PosHold before 0.35:
//   - the estimator's position resets move the hold target with them (the craft does not fly the step: P6 reset handling);
//   - a fence: a radius around home (fence_radius_m, 0 = none). Outside it the sticks are not followed; both axes go to the
//     controller with the target on the fence (1 m inside) until the craft is back inside - ArduPilot AC_Fence's circle with
//     a "brake / hold at the edge" action instead of RTL (a RTL hook is there for P7).
// Time constants instead of per-loop factors (the feel does not change with the loop rate). Angles in degrees, pilot
// convention; velocities m/s north / east.
// ============================================================================
namespace fc {
namespace nav {

struct PosHoldConfig {
    float brake_rate_dps = 8.0f;           // PHLD_BRK_RATE
    float brake_angle_max_deg = 30.0f;     // PHLD_BRK_ANGLE
    float angle_max_deg = 30.0f;           // lean limit in this mode (also of the controller)
    float stopped_mps = 0.10f;             // slower than this counts as stopped
    float to_loiter_s = 1.5f, to_pilot_s = 0.5f, wind_start_s = 1.5f, wind_tc_s = 1.0f, brake_max_s = 6.0f;
    float wind_lean_share = 0.6666f;       // the learned wind lean never takes more than this share of the lean limit
    float release_snap_deg = 18.0f;        // while the stick asks for more than this, its lean is followed at once
    float smooth_rate_hz = 5.0f;           // how fast the pilot's lean is taken back after the stick is let go (at least brake_rate)
    float fence_radius_m = 0.0f;           // P6: 0 = no fence
};

struct PosHoldInput {
    float pilot_roll_deg = 0, pilot_pitch_deg = 0;
    bool roll_centred = true, pitch_centred = true;
    float yaw_deg = 0;
    float pos_n = 0, pos_e = 0, vel_n = 0, vel_e = 0;     // from home
    bool airborne = false;                                 // false: on the ground / taking off - sticks only
    uint8_t reset_count = 0; float reset_step_n = 0, reset_step_e = 0;   // the estimator's position resets (PosEstimator)
};

class PosHoldMode {
public:
    PosHoldConfig cfg;
    PosController ctl;

    WB_FAST void init(const PosHoldInput& in) {
        for (int a = 0; a < 2; a++) { pilot_[a] = brake_[a] = wind_[a] = out_[a] = 0; st_[a] = in.airborne ? PhAxis::PILOT : PhAxis::LOITER; }
        ctl.cfg.angle_max_deg = cfg.angle_max_deg;
        ctl.init(in.pos_n, in.pos_e, in.vel_n, in.vel_e, 0, 0, in.yaw_deg);
        t_mix_ = t_wind_ = 0; reset_seen_ = in.reset_count; fence_ = false;
    }

    WB_FAST void update(const PosHoldInput& in, float dt, float& roll_deg, float& pitch_deg) {
        if (!(dt > 0.0f) || dt > 0.1f) { roll_deg = out_[0]; pitch_deg = out_[1]; return; }
        ctl.cfg.angle_max_deg = cfg.angle_max_deg;
        if (in.reset_count != reset_seen_) { ctl.on_estimator_reset(in.reset_step_n, in.reset_step_e); reset_seen_ = in.reset_count; }
        const float want[2] = {in.pilot_roll_deg, in.pilot_pitch_deg};
        bool centred[2] = {in.roll_centred, in.pitch_centred};
        float v_fwd, v_right; ne_to_body(in.vel_n, in.vel_e, in.yaw_deg, v_fwd, v_right);
        const float v_axis[2] = {v_right, v_fwd};                       // a roll right speeds up to the right, nose down forwards
        if (!in.airborne) {                                             // on the ground: the sticks alone, the target follows the craft
            for (int a = 0; a < 2; a++) { st_[a] = PhAxis::PILOT; pilot_[a] = want[a]; brake_[a] = 0; out_[a] = want[a]; }
            wind_[0] = wind_[1] = 0; ctl.init(in.pos_n, in.pos_e, in.vel_n, in.vel_e, 0, 0, in.yaw_deg);
            roll_deg = out_[0]; pitch_deg = out_[1]; return;
        }
        if (fence_check(in)) { centred[0] = centred[1] = true; }       // outside the fence: the sticks are not followed
        float wind_lean[2]; { float wf, wr; ne_to_body(wind_[0], wind_[1], in.yaw_deg, wf, wr); accel_to_lean(wf, wr, wind_lean[0], wind_lean[1]); }
        const bool both_ctl = ctl_axis(0) && ctl_axis(1);

        for (int a = 0; a < 2; a++) {
            switch (st_[a]) {
            case PhAxis::PILOT:
                smooth_pilot(pilot_[a], want[a], dt);
                out_[a] = pilot_[a] + wind_lean[a];
                if (centred[a] && fabsf(pilot_[a]) < 0.02f * cfg.brake_rate_dps) { st_[a] = PhAxis::BRAKE; brake_[a] = brake_peak_[a] = 0; brake_timing_[a] = false; brake_left_[a] = cfg.brake_max_s; }
                break;
            case PhAxis::BRAKE: case PhAxis::BRAKE_READY:
                if (!centred[a]) { to_pilot(a); break; }
                update_brake(a, v_axis[a], dt);
                out_[a] = brake_[a] + wind_lean[a];
                break;
            case PhAxis::TO_PILOT:
                smooth_pilot(pilot_[a], want[a], dt);
                t_axis_[a] += dt;
                { const float m = clampf(t_axis_[a] / cfg.to_pilot_s, 0.0f, 1.0f); out_[a] = (1.0f - m) * final_[a] + m * (pilot_[a] + wind_lean[a]); }
                if (t_axis_[a] >= cfg.to_pilot_s) st_[a] = PhAxis::PILOT;
                break;
            default: break;                                             // TO_LOITER / LOITER: both axes together, below
            }
        }
        // both stopped (or the fence): the controller takes over
        if ((st_[0] == PhAxis::BRAKE_READY && st_[1] == PhAxis::BRAKE_READY) || (fence_ && !both_ctl)) {
            st_[0] = st_[1] = PhAxis::TO_LOITER; t_mix_ = t_wind_ = 0;
            ctl.init(in.pos_n, in.pos_e, in.vel_n, in.vel_e, out_[0], out_[1], in.yaw_deg);
        }
        if (both_ctl && ctl_axis(0)) {
            const bool loiter = st_[0] == PhAxis::LOITER;
            for (int a = 0; a < 2; a++) if (!centred[a] && ctl_axis(a)) {      // a stick takes its axis back; the other one brakes and waits
                const int o = 1 - a;
                to_pilot(a);
                if (centred[o]) { st_[o] = PhAxis::BRAKE_READY; if (loiter) brake_[o] = 0; out_[o] = brake_[o] + wind_lean[o]; }
            }
            if (ctl_axis(0) && ctl_axis(1)) {
                if (fence_) ctl.set_target(fence_tgt_[0], fence_tgt_[1]);
                ctl.update(in.pos_n, in.pos_e, in.vel_n, in.vel_e, dt, true);
                float lr, lp; ctl.lean(in.yaw_deg, lr, lp); const float hold[2] = {lr, lp};
                if (!loiter) {
                    t_mix_ += dt;
                    const float m = clampf(t_mix_ / cfg.to_loiter_s, 0.0f, 1.0f);
                    for (int a = 0; a < 2; a++) { update_brake(a, v_axis[a], dt); out_[a] = (1.0f - m) * (brake_[a] + wind_lean[a]) + m * hold[a]; }
                    if (t_mix_ >= cfg.to_loiter_s) st_[0] = st_[1] = PhAxis::LOITER;
                } else {
                    out_[0] = hold[0]; out_[1] = hold[1];
                    t_wind_ += dt;                                      // standing still, what the controller asks for is the wind
                    if (t_wind_ >= cfg.wind_start_s && in.vel_n * in.vel_n + in.vel_e * in.vel_e <= cfg.stopped_mps * cfg.stopped_mps) {
                        const float acc[2] = {ctl.accel_n(), ctl.accel_e()}, k = dt / (dt + cfg.wind_tc_s);
                        for (int a = 0; a < 2; a++) wind_[a] = wind_[a] == 0.0f ? acc[a] : wind_[a] + (acc[a] - wind_[a]) * k;
                        limit_len(wind_, kG0 * tanf(cfg.wind_lean_share * cfg.angle_max_deg * kDeg2Rad));
                    }
                }
            }
        }
        for (int a = 0; a < 2; a++) out_[a] = clampf(out_[a], -cfg.angle_max_deg, cfg.angle_max_deg);
        roll_deg = out_[0]; pitch_deg = out_[1];
    }

    PhAxis state(int axis) const { return st_[axis]; }
    bool holding() const { return st_[0] == PhAxis::LOITER && st_[1] == PhAxis::LOITER; }
    bool fence_breached() const { return fence_; }
    float wind_n() const { return wind_[0]; } float wind_e() const { return wind_[1]; }       // learned acceleration against the wind
    float brake_deg(int axis) const { return brake_[axis]; }
    float out_deg(int axis) const { return out_[axis]; }

private:
    bool ctl_axis(int a) const { return st_[a] == PhAxis::TO_LOITER || st_[a] == PhAxis::LOITER; }
    // The fence acts where the craft could still stop before it: distance + (outward speed)^2 / (2 x half the deceleration the
    // lean limit gives) beyond the radius. Then: hold on the circle (1 m inside). A stick that points back home gives the craft
    // back to the pilot at once.
    WB_FAST bool fence_check(const PosHoldInput& in) {
        if (!(cfg.fence_radius_m > 0.0f)) { fence_ = false; return false; }
        const float d = sqrtf(in.pos_n * in.pos_n + in.pos_e * in.pos_e);
        if (d < 1e-3f) { fence_ = false; return false; }
        if (fence_ && !(in.roll_centred && in.pitch_centred)) {
            float f, r, an, ae; lean_to_accel(in.pilot_roll_deg, in.pilot_pitch_deg, f, r); body_to_ne(f, r, in.yaw_deg, an, ae);
            if (an * in.pos_n + ae * in.pos_e < 0.0f) { fence_ = false; return false; }
        }
        const float v_out = (in.vel_n * in.pos_n + in.vel_e * in.pos_e) / d;
        const float decel = 0.5f * kG0 * tanf(cfg.angle_max_deg * kDeg2Rad);
        const float stop = v_out > 0.0f ? v_out * v_out / (2.0f * maxf(decel, 0.5f)) : 0.0f;
        if (d + stop > cfg.fence_radius_m) {
            if (!fence_) { const float k = maxf(cfg.fence_radius_m - 1.0f, 0.0f) / d; fence_tgt_[0] = in.pos_n * k; fence_tgt_[1] = in.pos_e * k; }
            fence_ = true;
        }
        return fence_;
    }
    // the pilot's lean after the stick is let go is taken back at brake speed (unless it was large, or the stick reverses)
    WB_FAST void smooth_pilot(float& held, float raw, float dt) const {
        if ((held > 0.0f && raw < 0.0f) || (held < 0.0f && raw > 0.0f) || fabsf(raw) > cfg.release_snap_deg) { held = raw; return; }
        const float step = cfg.brake_rate_dps * dt;
        if (held > 0.0f) { held -= maxf(held * cfg.smooth_rate_hz * dt, step); held = maxf(held, raw); }
        else { held += maxf(-held * cfg.smooth_rate_hz * dt, step); held = minf(held, raw); }
    }
    // ArduCopter update_brake_angle_from_velocity, and the timing of the brake phase
    WB_FAST void update_brake(int a, float v, float dt) {
        const float gain = (15.0f * cfg.brake_rate_dps + 95.0f) * 0.01f;         // deg per m/s
        const float lean = -gain * v * (1.0f + 5.0f / (fabsf(v) + 0.60f)), step = cfg.brake_rate_dps * dt;
        brake_[a] = clampf(clampf(lean, brake_[a] - step, brake_[a] + step), -cfg.brake_angle_max_deg, cfg.brake_angle_max_deg);
        if (st_[a] != PhAxis::BRAKE) return;
        if (!brake_timing_[a]) {                                         // the lean still builds: follow its peak; when it first comes back,
            if (fabsf(brake_[a]) >= brake_peak_[a]) brake_peak_[a] = fabsf(brake_[a]);          // the rest takes ~1.5 x the time to take it back
            else { brake_left_[a] = minf(cfg.brake_max_s, 1.5f * fabsf(brake_[a]) / cfg.brake_rate_dps); brake_timing_[a] = true; }
        }
        if (fabsf(v) <= cfg.stopped_mps && brake_left_[a] > 0.5f) brake_left_[a] = 0.5f;
        brake_left_[a] -= dt;
        if (brake_left_[a] <= 0.0f) st_[a] = PhAxis::BRAKE_READY;
    }
    void to_pilot(int a) { st_[a] = PhAxis::TO_PILOT; pilot_[a] = 0; final_[a] = out_[a]; t_axis_[a] = 0; }

    PhAxis st_[2] = {PhAxis::PILOT, PhAxis::PILOT};
    float pilot_[2] = {0, 0}, brake_[2] = {0, 0}, brake_peak_[2] = {0, 0}, brake_left_[2] = {0, 0}, t_axis_[2] = {0, 0}, final_[2] = {0, 0}, out_[2] = {0, 0};
    bool brake_timing_[2] = {false, false};
    float wind_[2] = {0, 0}, t_mix_ = 0, t_wind_ = 0;
    uint8_t reset_seen_ = 0; bool fence_ = false; float fence_tgt_[2] = {0, 0};
};

}  // namespace nav
}  // namespace fc
