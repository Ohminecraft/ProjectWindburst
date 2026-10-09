#pragma once
#include "poscontrol.h"

// ============================================================================
// Position Hold - ArduCopter's PosHold (mode_poshold.cpp), per axis:
//
//   PILOT        the stick gives the lean (as in Angle), plus the lean that was learned to stand against the wind. Let go, the
//                lean is taken back quickly but not dropped (5 /s, at least brake_rate)
//   BRAKE        stick let go: a lean against the motion, the larger the faster the craft moves, built up and taken back at
//                brake_rate degrees per second - an open-loop brake that feels the same every time
//   BRAKE_READY  this axis has stopped; it waits for the other one
//   TO_LOITER    both stopped: 1.5 s in which the brake hands over to the position controller (poscontrol.h)
//   LOITER       the position controller holds the spot; the lean it needs when the craft stands still is the wind
//   TO_PILOT     the stick moved: 0.5 s from the controller's lean to the pilot's
//
// Roll and pitch are separate until both have stopped: the pilot can fly sideways while the craft already brakes forwards.
// Position Hold only gives roll / pitch leans; height (stick or Altitude Hold) and yaw are untouched (yaw stays the pilot's).
//
// Angles are degrees in the pilot's convention (roll + = right side down, pitch + = nose down); velocities m/s.
// Differences to ArduCopter 4.7-dev, on purpose: time constants instead of per-loop factors (the feel does not change with
// the loop rate); the brake-to-loiter blend really goes from the brake to the controller; the "nearly stopped" shortcut leaves
// half a second and cannot restart itself for ever.
// ============================================================================
namespace fc {

struct PosHoldConfig {
    float brake_rate_dps = 8.0f;           // PHLD_BRK_RATE: how fast the brake lean builds up and is taken back
    float brake_angle_max_deg = 30.0f;     // PHLD_BRK_ANGLE
    float angle_max_deg = 30.0f;           // overall lean limit in this mode
    float stopped_ms = 0.10f;              // m/s: slower than this counts as stopped (wind learning, brake shortcut)
    float to_loiter_s = 1.5f, to_pilot_s = 0.5f, wind_start_s = 1.5f, wind_tc_s = 1.0f, brake_max_s = 6.0f;
    float wind_lean_share = 0.6666f;       // the learned wind lean never takes more than this share of the lean limit: the pilot can always override
    float release_snap_deg = 18.0f;        // while the stick still asks for more than this, its lean is followed at once (no smoothing; ArduCopter POSHOLD_STICK_RELEASE_SMOOTH_ANGLE)
    float smooth_rate = 5.0f;              // 1/s: how fast the pilot's lean is taken back after the stick is let go (at least brake_rate)
};

enum class PhAxis : uint8_t { PILOT, BRAKE, BRAKE_READY, TO_LOITER, LOITER, TO_PILOT };

struct PosHoldInput {
    float pilot_roll_deg = 0, pilot_pitch_deg = 0;      // the lean the sticks ask for
    bool roll_centred = true, pitch_centred = true;      // stick inside its deadband
    float yaw_deg = 0;                                   // heading (0 = north, clockwise)
    float pos_n = 0, pos_e = 0, vel_n = 0, vel_e = 0;
    bool airborne = false;                               // false: on the ground / taking off - sticks only, nothing is learned or held
};

class PosHold {
public:
    PosHoldConfig cfg;
    PosControlNE ctl;

    // Entering the mode. airborne: the pilot keeps the sticks first ("to avoid a hard twitch"); on the ground it holds at once.
    WB_FAST void init(const PosHoldInput& in) {
        pilot_[0] = pilot_[1] = 0; brake_[0] = brake_[1] = 0; wind_[0] = wind_[1] = 0; out_[0] = out_[1] = 0;
        st_[0] = st_[1] = in.airborne ? PhAxis::PILOT : PhAxis::LOITER;
        ctl.cfg.angle_max_deg = cfg.angle_max_deg;
        ctl.init(in.pos_n, in.pos_e, in.vel_n, in.vel_e, 0, 0, in.yaw_deg);
        t_mix_ = 0; t_wind_ = 0;
    }

    WB_FAST void update(const PosHoldInput& in, float dt, float& roll_deg, float& pitch_deg) {
        ctl.cfg.angle_max_deg = cfg.angle_max_deg;
        const float want[2] = {in.pilot_roll_deg, in.pilot_pitch_deg}; const bool centred[2] = {in.roll_centred, in.pitch_centred};
        float v_fwd, v_right; ne_to_body(in.vel_n, in.vel_e, in.yaw_deg, v_fwd, v_right);
        const float v_axis[2] = {v_right, v_fwd};                         // a roll to the right speeds up to the right, nose down speeds up forwards
        if (!in.airborne) {                                               // on the ground, taking off: the sticks alone; the target follows the craft
            for (int a = 0; a < 2; a++) { st_[a] = PhAxis::PILOT; pilot_[a] = want[a]; brake_[a] = 0; out_[a] = want[a]; }
            wind_[0] = wind_[1] = 0; ctl.init(in.pos_n, in.pos_e, in.vel_n, in.vel_e, 0, 0, in.yaw_deg);
            roll_deg = out_[0]; pitch_deg = out_[1]; return;
        }
        // the lean against the wind, in the craft's axes (as long as not both axes are under the controller, which holds it itself)
        float wind_lean[2]; { float wf, wr; ne_to_body(wind_[0], wind_[1], in.yaw_deg, wf, wr); accel_to_lean(wf, wr, wind_lean[0], wind_lean[1]); }
        const bool both_ctl = (st_[0] == PhAxis::TO_LOITER || st_[0] == PhAxis::LOITER) && (st_[1] == PhAxis::TO_LOITER || st_[1] == PhAxis::LOITER);

        // ---- each axis by itself
        for (int a = 0; a < 2; a++) {
            switch (st_[a]) {
            case PhAxis::PILOT:
                smooth_pilot(pilot_[a], want[a], dt);
                out_[a] = pilot_[a] + wind_lean[a];
                if (centred[a] && fabsf(pilot_[a]) < 0.02f * cfg.brake_rate_dps) /* 0.16 deg: the pilot's lean has been taken back */ { st_[a] = PhAxis::BRAKE; brake_[a] = 0; brake_peak_[a] = 0; brake_timing_[a] = false; brake_left_[a] = cfg.brake_max_s; }
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
            default: break;                                               // TO_LOITER / LOITER: below, both axes together
            }
        }
        // ---- both axes have stopped: hand over to the position controller
        if (st_[0] == PhAxis::BRAKE_READY && st_[1] == PhAxis::BRAKE_READY) {
            st_[0] = st_[1] = PhAxis::TO_LOITER; t_mix_ = 0; t_wind_ = 0;
            ctl.init(in.pos_n, in.pos_e, in.vel_n, in.vel_e, out_[0], out_[1], in.yaw_deg);
        }
        if (both_ctl && (st_[0] == PhAxis::TO_LOITER || st_[0] == PhAxis::LOITER)) {
            const bool loiter = st_[0] == PhAxis::LOITER;
            // a stick takes its axis back; the other one keeps braking (from a standstill: nothing to brake) and waits
            for (int a = 0; a < 2; a++) if (!centred[a] && (st_[a] == PhAxis::TO_LOITER || st_[a] == PhAxis::LOITER)) {
                const int o = 1 - a;
                to_pilot(a);
                if (centred[o]) { st_[o] = PhAxis::BRAKE_READY; if (loiter) brake_[o] = 0; out_[o] = brake_[o] + wind_lean[o]; }
            }
            if ((st_[0] == PhAxis::TO_LOITER || st_[0] == PhAxis::LOITER) && (st_[1] == PhAxis::TO_LOITER || st_[1] == PhAxis::LOITER)) {
                ctl.update(in.pos_n, in.pos_e, in.vel_n, in.vel_e, dt, true);
                float lr, lp; ctl.lean(in.yaw_deg, lr, lp); const float hold[2] = {lr, lp};
                if (!loiter) {
                    t_mix_ += dt;
                    const float m = clampf(t_mix_ / cfg.to_loiter_s, 0.0f, 1.0f);
                    for (int a = 0; a < 2; a++) { update_brake(a, v_axis[a], dt); out_[a] = (1.0f - m) * (brake_[a] + wind_lean[a]) + m * hold[a]; }
                    if (t_mix_ >= cfg.to_loiter_s) st_[0] = st_[1] = PhAxis::LOITER;
                } else {
                    out_[0] = hold[0]; out_[1] = hold[1];
                    // standing still, the acceleration the controller asks for is what the wind takes: remember it for when the pilot flies
                    t_wind_ += dt;
                    if (t_wind_ >= cfg.wind_start_s && in.vel_n * in.vel_n + in.vel_e * in.vel_e <= cfg.stopped_ms * cfg.stopped_ms) {
                        const float acc[2] = {ctl.accel_n(), ctl.accel_e()}, k = dt / (dt + cfg.wind_tc_s);
                        for (int a = 0; a < 2; a++) wind_[a] = wind_[a] == 0.0f ? acc[a] : wind_[a] + (acc[a] - wind_[a]) * k;
                        const float lim = kG0 * tanf(cfg.wind_lean_share * cfg.angle_max_deg * kDeg2Rad), len = sqrtf(wind_[0] * wind_[0] + wind_[1] * wind_[1]);
                        if (len > lim) { wind_[0] *= lim / len; wind_[1] *= lim / len; }
                    }
                }
            }
        }
        for (int a = 0; a < 2; a++) out_[a] = clampf(out_[a], -cfg.angle_max_deg, cfg.angle_max_deg);
        roll_deg = out_[0]; pitch_deg = out_[1];
    }

    PhAxis state(int axis) const { return st_[axis]; }
    bool holding() const { return st_[0] == PhAxis::LOITER && st_[1] == PhAxis::LOITER; }
    float brake_deg(int axis) const { return brake_[axis]; }
    float wind_n() const { return wind_[0]; } float wind_e() const { return wind_[1]; }       // learned acceleration against the wind, m/s^2
    float out_deg(int axis) const { return out_[axis]; }

private:
    // The pilot's lean after the stick is let go is not dropped at once (unless it was large): it is taken back at brake speed.
    WB_FAST void smooth_pilot(float& held, float raw, float dt) const {
        if ((held > 0.0f && raw < 0.0f) || (held < 0.0f && raw > 0.0f) || fabsf(raw) > cfg.release_snap_deg) { held = raw; return; }
        const float step = cfg.brake_rate_dps * dt;
        if (held > 0.0f) { held -= fmaxf(held * cfg.smooth_rate * dt, step); held = fmaxf(held, raw); }
        else { held += fmaxf(-held * cfg.smooth_rate * dt, step); held = fminf(held, raw); }
    }
    // ArduCopter update_brake_angle_from_velocity + the timing of the brake phase
    WB_FAST void update_brake(int a, float v, float dt) {
        const float gain = (15.0f * cfg.brake_rate_dps + 95.0f) * 0.01f;                 // deg per m/s
        const float lean = -gain * v * (1.0f + 5.0f / (fabsf(v) + 0.60f)), step = cfg.brake_rate_dps * dt;
        brake_[a] = clampf(clampf(lean, brake_[a] - step, brake_[a] + step), -cfg.brake_angle_max_deg, cfg.brake_angle_max_deg);
        if (st_[a] != PhAxis::BRAKE) return;
        if (!brake_timing_[a]) {                                         // the lean still builds up: follow its peak. When it first comes back, the
            if (fabsf(brake_[a]) >= brake_peak_[a]) brake_peak_[a] = fabsf(brake_[a]);          // rest of the braking takes about 1.5 x the time to take the lean back
            else { brake_left_[a] = fminf(cfg.brake_max_s, 1.5f * fabsf(brake_[a]) / cfg.brake_rate_dps); brake_timing_[a] = true; }
        }
        if (fabsf(v) <= cfg.stopped_ms && brake_left_[a] > 0.5f) brake_left_[a] = 0.5f;      // nearly stopped: half a second more at most
        brake_left_[a] -= dt;
        if (brake_left_[a] <= 0.0f) st_[a] = PhAxis::BRAKE_READY;
    }
    void to_pilot(int a) { st_[a] = PhAxis::TO_PILOT; pilot_[a] = 0; final_[a] = out_[a]; t_axis_[a] = 0; }

    PhAxis st_[2] = {PhAxis::PILOT, PhAxis::PILOT};
    float pilot_[2] = {0, 0}, brake_[2] = {0, 0}, brake_peak_[2] = {0, 0}, brake_left_[2] = {0, 0}, t_axis_[2] = {0, 0}, final_[2] = {0, 0}, out_[2] = {0, 0};
    bool brake_timing_[2] = {false, false};
    float wind_[2] = {0, 0}, t_mix_ = 0, t_wind_ = 0;
};

} // namespace fc
