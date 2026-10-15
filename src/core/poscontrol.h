#pragma once
#include "types.h"

// ============================================================================
// Horizontal position / velocity controller (north, east) - the cascade of ArduPilot's AC_PosControl (NE part):
//
//   position error --P with a square-root limit--> velocity correction --(+ wanted velocity)--> velocity error
//       --PID (error and D low-passed at 5 Hz)--> acceleration --> lean angles of the craft
//
// Units: metres, m/s, m/s^2; angles in degrees in the PILOT's convention (roll + = right side down, pitch + = nose down), which
// is what the Angle controller takes. Used by Position Hold (position target fixed) and by GPS Rescue (velocity wanted, no
// position target). The acceleration is limited to what the lean limit can give; the integrator may then only turn, not grow
// (AC_PID_2D's limit vector).
// ============================================================================
namespace fc {

// ArduPilot AP_Math/control.cpp sqrt_controller without the dt clamp: linear (error x p) near the target, a square-root curve
// (constant deceleration `accel`) further out, so that a far target is approached at a speed from which the craft can still stop.
inline WB_FAST float sqrt_controller(float error, float p, float accel) {
    if (accel <= 0.0f || p <= 0.0f) return error * p;
    const float linear = accel / (p * p), a = fabsf(error);
    if (a <= linear) return error * p;
    return (error > 0.0f ? 1.0f : -1.0f) * sqrtf(2.0f * accel * (a - 0.5f * linear));
}
// lean angles (deg, pilot convention) <-> acceleration over the ground (m/s^2), for a craft that holds its height
inline WB_FAST void accel_to_lean(float a_fwd, float a_right, float& roll_deg, float& pitch_deg) {
    const float pitch = atanf(a_fwd / kG0);
    pitch_deg = pitch * kRad2Deg; roll_deg = atanf(a_right * cosf(pitch) / kG0) * kRad2Deg;
}
inline WB_FAST void lean_to_accel(float roll_deg, float pitch_deg, float& a_fwd, float& a_right) {
    const float r = roll_deg * kDeg2Rad, p = pitch_deg * kDeg2Rad;
    a_fwd = kG0 * tanf(p); a_right = kG0 * tanf(r) / maxf(cosf(p), 0.3f);
}
WB_FAST inline void ne_to_body(float n, float e, float yaw_deg, float& fwd, float& right) { const float y = yaw_deg * kDeg2Rad, c = cosf(y), s = sinf(y); fwd = n * c + e * s; right = -n * s + e * c; }
WB_FAST inline void body_to_ne(float fwd, float right, float yaw_deg, float& n, float& e) { const float y = yaw_deg * kDeg2Rad, c = cosf(y), s = sinf(y); n = fwd * c - right * s; e = fwd * s + right * c; }

struct PosControlConfig {
    float pos_p = 1.0f;                                // 1/s            PSC_NE_POS_P
    float vel_p = 2.0f, vel_i = 1.0f, vel_d = 0.25f;   //                PSC_NE_VEL_P / I / D
    float filt_hz = 5.0f;                              // low-pass of the velocity error and of its derivative (PSC_NE_VEL_FLTE / FLTD)
    float pos_err_max = 2.0f;                          // m: the target is never further from the craft than this
    float vel_corr_max = 2.0f;                         // m/s: how fast a position error is flown out at most
    float accel_corr = 2.5f;                           // m/s^2: the deceleration the square-root curve is laid out for
    float angle_max_deg = 30.0f;                       // lean limit of the controller
    float i_max = 5.0f;                                // m/s^2: what the integrator may hold (wind: 5 m/s^2 = 27 deg of lean)
};

class PosControlNE {
public:
    PosControlConfig cfg;

    // Start (or restart) without a jump: the target is where the craft is, and the integrator carries the acceleration that the
    // present lean (roll / pitch target in degrees, heading) already gives.
    WB_FAST void init(float pos_n, float pos_e, float vel_n, float vel_e, float roll_deg, float pitch_deg, float yaw_deg) {
        tgt_[0] = pos_n; tgt_[1] = pos_e;
        float af, ar; lean_to_accel(roll_deg, pitch_deg, af, ar);
        body_to_ne(af, ar, yaw_deg, i_[0], i_[1]); limit2(i_, cfg.i_max);
        acc_[0] = i_[0]; acc_[1] = i_[1];
        for (int a = 0; a < 2; a++) { err_[a] = 0; err_last_[a] = 0; d_[a] = 0; vel_tgt_[a] = a == 0 ? vel_n : vel_e; }
        limited_ = false; fresh_ = true;
    }
    void set_target(float pos_n, float pos_e) { tgt_[0] = pos_n; tgt_[1] = pos_e; }
    // Let the integrator fade (landing, sticks in charge): tc seconds
    void relax(float dt, float tc = 0.16f) { const float k = dt / (dt + tc); for (int a = 0; a < 2; a++) { i_[a] -= i_[a] * k; acc_[a] -= acc_[a] * k; } }

    // hold_position: steer to the target position (else only the wanted velocity counts and the target follows the craft).
    // vel_want: feed-forward (m/s; north, east).
    WB_FAST void update(float pos_n, float pos_e, float vel_n, float vel_e, float dt, bool hold_position, float vel_want_n = 0, float vel_want_e = 0) {
        const float pos[2] = {pos_n, pos_e}, vel[2] = {vel_n, vel_e}, vw[2] = {vel_want_n, vel_want_e};
        float corr[2] = {0, 0};
        if (hold_position) {
            float e[2] = {tgt_[0] - pos[0], tgt_[1] - pos[1]};
            const float len = sqrtf(e[0] * e[0] + e[1] * e[1]);
            if (len > cfg.pos_err_max) { for (int a = 0; a < 2; a++) { e[a] *= cfg.pos_err_max / len; tgt_[a] = pos[a] + e[a]; } }    // the target is pulled along
            const float l2 = sqrtf(e[0] * e[0] + e[1] * e[1]);
            if (l2 > 1e-6f) { const float v = minf(sqrt_controller(l2, cfg.pos_p, cfg.accel_corr), cfg.vel_corr_max); corr[0] = e[0] / l2 * v; corr[1] = e[1] / l2 * v; }
        } else { tgt_[0] = pos[0]; tgt_[1] = pos[1]; }
        pos_err_ = sqrtf((tgt_[0] - pos[0]) * (tgt_[0] - pos[0]) + (tgt_[1] - pos[1]) * (tgt_[1] - pos[1]));
        const float alpha = dt / (dt + 1.0f / (2.0f * kPi * cfg.filt_hz));
        float out[2];
        for (int a = 0; a < 2; a++) {
            vel_tgt_[a] = corr[a] + vw[a];
            const float e = vel_tgt_[a] - vel[a];
            if (fresh_) { err_[a] = e; err_last_[a] = e; d_[a] = 0; }
            else { err_last_[a] = err_[a]; err_[a] += (e - err_[a]) * alpha; d_[a] += ((err_[a] - err_last_[a]) / dt - d_[a]) * alpha; }
        }
        fresh_ = false;
        // integrator: while the output is limited it may turn but not grow in the limited direction
        const float i_len = sqrtf(i_[0] * i_[0] + i_[1] * i_[1]);
        float delta[2] = {err_[0] * cfg.vel_i * dt, err_[1] * cfg.vel_i * dt};
        i_[0] += delta[0]; i_[1] += delta[1];
        if (limited_ && delta[0] * lim_dir_[0] + delta[1] * lim_dir_[1] > 0.0f) limit2(i_, i_len);
        limit2(i_, cfg.i_max);
        for (int a = 0; a < 2; a++) out[a] = err_[a] * cfg.vel_p + i_[a] + d_[a] * cfg.vel_d;
        const float amax = kG0 * tanf(cfg.angle_max_deg * kDeg2Rad), olen = sqrtf(out[0] * out[0] + out[1] * out[1]);
        limited_ = olen > amax;
        if (limited_) { lim_dir_[0] = out[0] / olen; lim_dir_[1] = out[1] / olen; out[0] *= amax / olen; out[1] *= amax / olen; }
        acc_[0] = out[0]; acc_[1] = out[1];
    }
    // the lean that gives the acceleration target at this heading (deg, pilot convention)
    WB_FAST void lean(float yaw_deg, float& roll_deg, float& pitch_deg) const { float f, r; ne_to_body(acc_[0], acc_[1], yaw_deg, f, r); accel_to_lean(f, r, roll_deg, pitch_deg); }

    float accel_n() const { return acc_[0]; } float accel_e() const { return acc_[1]; }
    float target_n() const { return tgt_[0]; } float target_e() const { return tgt_[1]; }
    float vel_target_n() const { return vel_tgt_[0]; } float vel_target_e() const { return vel_tgt_[1]; }
    float i_n() const { return i_[0]; } float i_e() const { return i_[1]; }
    float pos_error() const { return pos_err_; }
    bool limited() const { return limited_; }
private:
    WB_FAST static void limit2(float v[2], float len) { const float l = sqrtf(v[0] * v[0] + v[1] * v[1]); if (l > len && l > 0.0f) { v[0] *= len / l; v[1] *= len / l; } }
    float tgt_[2] = {0, 0}, vel_tgt_[2] = {0, 0}, err_[2] = {0, 0}, err_last_[2] = {0, 0}, d_[2] = {0, 0}, i_[2] = {0, 0}, acc_[2] = {0, 0}, lim_dir_[2] = {0, 0}, pos_err_ = 0;
    bool limited_ = false, fresh_ = true;
};

} // namespace fc
