/*
 * This file is part of Windburst Project
 */

#pragma once
#include "../../config/fast_code.h"
#include "../types.h"
#include "state_history.h"

// ============================================================================
// PosEstimator - where the craft is and how fast it moves over the ground, north and east from home (nav rewrite P4).
//
// One Kalman filter per axis, x = [position (m), velocity (m/s), accelerometer bias (m/s^2)], predicted with the NED
// acceleration (gravity removed) and corrected with
//   GPS        position and velocity (the module's hAcc / sAcc as sigmas, floored), compared with the state when the solution
//              was TAKEN (`gps_delay_s` back, StateHistory) - not brought forward with the acceleration as before 0.35;
//   velocity   a velocity over the ground from optical flow (flow.h: flow x height), with its own age.
// Position is relative to home; with flow alone it has no anchor (it only keeps the velocity from running away).
//
// Glitches and resets (ArduPilot AP_NavEKF3 "GPS glitch" handling, PX4 EKF2 position reset):
//   - a GPS solution outside the innovation gate is not used (a multipath jump, a satellite lost);
//   - rejected for `glitch_reset_s` while solutions keep coming: the GPS is right and the estimate is not (it ran off during a
//     dropout, or the module really moved its fix). Position and velocity are SET to the GPS, and the reset is counted with its
//     step: a controller holding a position target moves the target by the same step (P6), the craft does not fly to it;
//   - the variance grows while nothing corrects it, so the first measurement after a gap counts in full (Betaflight saw
//     "several m/s of phantom velocity" after a flow dropout when the old gain was kept).
// Algorithms from ArduPilot (EKF3 glitch / reset), PX4 EKF2 (delayed fusion), INAV (navigation position estimator: GPS + flow);
// written anew. Every entry point copes with a bad dt and NaN readings.
// ============================================================================
namespace fc {
namespace nav {

struct PosEstConfig {
    float accel_noise_mps2 = 0.8f;       // attitude errors leak gravity into the horizontal axes
    float bias_noise_mps3 = 0.01f;
    float bias_max_mps2 = 1.0f;
    float gps_delay_s = 0.15f;           // typical u-blox latency at 10 Hz
    float gps_pos_sigma_min_m = 0.5f, gps_vel_sigma_min_mps = 0.1f;
    float gate_sigma = 6.0f;
    float glitch_reset_s = 2.0f;         // GPS rejected this long (solutions still coming): the estimate is set to it
};

struct PosInnov { float pos[2] = {0, 0}, vel[2] = {0, 0}, ratio = 0; bool used = false; };   // the last GPS fusion (ratio: largest of the four)

class PosEstimator {
public:
    PosEstConfig cfg;

    void reset(float pn = 0, float pe = 0, float vn = 0, float ve = 0) {
        const float p[2] = {pn, pe}, v[2] = {vn, ve};
        for (int a = 0; a < 2; a++) {
            ax_[a] = Axis(); ax_[a].x[0] = p[a]; ax_[a].x[1] = v[a];
        }
        hist_.reset(); have_ = false; rejected_s_ = 0.0f; reset_count_ = 0; reset_step_[0] = reset_step_[1] = 0; glitches_ = 0; in_ = PosInnov();
    }

    WB_FAST void predict(float acc_n, float acc_e, float dt) {
        if (!(dt > 0.0f) || dt > 0.1f || acc_n != acc_n || acc_e != acc_e) return;
        ax_[0].predict(acc_n, dt, cfg); ax_[1].predict(acc_e, dt, cfg);
        if (have_ && rejected_s_ > 0.0f) rejected_s_ += dt;
        const float h[4] = {ax_[0].x[0], ax_[0].x[1], ax_[1].x[0], ax_[1].x[1]}; hist_.push(h, dt);
    }

    // A GPS solution: metres / m/s from home, as the module measured it gps_delay_s ago. Returns true when it was used.
    WB_FAST bool fuse_gps(float pn, float pe, float vn, float ve, float sigma_pos, float sigma_vel) {
        if (pn != pn || pe != pe || vn != vn || ve != ve) return false;
        const float sp = maxf(sigma_pos, cfg.gps_pos_sigma_min_m), sv = maxf(sigma_vel, cfg.gps_vel_sigma_min_mps);
        const float pm[2] = {pn, pe}, vm[2] = {vn, ve};
        if (!have_) { for (int a = 0; a < 2; a++) { ax_[a].x[0] = pm[a]; ax_[a].x[1] = vm[a]; } have_ = true; sync_history(); return true; }
        float past[4]; if (!hist_.at(cfg.gps_delay_s, past)) { past[0] = ax_[0].x[0]; past[1] = ax_[0].x[1]; past[2] = ax_[1].x[0]; past[3] = ax_[1].x[1]; }
        // the four innovations against the delayed state; the solution is used as a whole or not at all (a glitch moves both)
        float ratio = 0;
        for (int a = 0; a < 2; a++) {
            in_.pos[a] = pm[a] - past[2 * a]; in_.vel[a] = vm[a] - past[2 * a + 1];
            ratio = maxf(ratio, in_.pos[a] * in_.pos[a] / (cfg.gate_sigma * cfg.gate_sigma * (ax_[a].P[0][0] + sp * sp)));
            ratio = maxf(ratio, in_.vel[a] * in_.vel[a] / (cfg.gate_sigma * cfg.gate_sigma * (ax_[a].P[1][1] + sv * sv)));
        }
        in_.ratio = ratio; in_.used = ratio <= 1.0f;
        if (!in_.used) {
            if (rejected_s_ <= 0.0f) rejected_s_ = 1e-6f;                          // the clock of the glitch starts
            if (rejected_s_ >= cfg.glitch_reset_s) { set_to(pm, vm, sp, sv); glitches_++; rejected_s_ = 0.0f; return true; }
            return false;
        }
        rejected_s_ = 0.0f;
        float dx[4] = {0, 0, 0, 0};
        for (int a = 0; a < 2; a++) {
            float d[2] = {0, 0}, e[2] = {0, 0};
            ax_[a].update(0, in_.pos[a], sp, cfg, d);
            ax_[a].update(1, vm[a] - (past[2 * a + 1] + d[1]), sv, cfg, e);          // (the position update moved the velocity too)
            dx[2 * a] = d[0] + e[0]; dx[2 * a + 1] = d[1] + e[1];
        }
        hist_.shift(dx);
        return true;
    }
    // A velocity over the ground (m/s) that was true delay_s ago (optical flow); sigma never below 0.1 m/s.
    WB_FAST bool fuse_velocity(float vn, float ve, float sigma_vel, float delay_s) {
        if (vn != vn || ve != ve) return false;
        const float vm[2] = {vn, ve}, sv = maxf(sigma_vel, 0.1f);
        if (!have_) { ax_[0].x[1] = vn; ax_[1].x[1] = ve; have_ = true; sync_history(); return true; }
        float past[4]; if (!hist_.at(delay_s, past)) { past[1] = ax_[0].x[1]; past[3] = ax_[1].x[1]; }
        float dx[4] = {0, 0, 0, 0}; bool ok = true;
        for (int a = 0; a < 2; a++) {
            const float y = vm[a] - past[2 * a + 1];
            if (y * y > cfg.gate_sigma * cfg.gate_sigma * (ax_[a].P[1][1] + sv * sv)) { ok = false; continue; }
            float d[2] = {0, 0}; ax_[a].update(1, y, sv, cfg, d); dx[2 * a] = d[0]; dx[2 * a + 1] = d[1];
        }
        hist_.shift(dx);
        return ok;
    }

    float pos_n() const { return ax_[0].x[0]; } float pos_e() const { return ax_[1].x[0]; }
    float vel_n() const { return ax_[0].x[1]; } float vel_e() const { return ax_[1].x[1]; }
    float bias_n() const { return ax_[0].x[2]; } float bias_e() const { return ax_[1].x[2]; }
    float speed() const { return sqrtf(vel_n() * vel_n() + vel_e() * vel_e()); }
    float pos_sigma() const { return sqrtf(maxf(0.5f * (ax_[0].P[0][0] + ax_[1].P[0][0]), 0.0f)); }
    float vel_sigma() const { return sqrtf(maxf(0.5f * (ax_[0].P[1][1] + ax_[1].P[1][1]), 0.0f)); }
    bool initialised() const { return have_; }
    // position resets (set to the GPS after a glitch): count and the last step (m, north / east)
    uint8_t reset_count() const { return reset_count_; }
    float reset_step_n() const { return reset_step_[0]; } float reset_step_e() const { return reset_step_[1]; }
    uint32_t glitch_resets() const { return glitches_; }
    bool gps_rejecting() const { return rejected_s_ > 0.0f; }
    const PosInnov& gps_innov() const { return in_; }

private:
    struct Axis {
        float x[3] = {0, 0, 0}, P[3][3] = {{25, 0, 0}, {0, 4, 0}, {0, 0, 0.1f}};
        WB_FAST void predict(float a, float dt, const PosEstConfig& c) {
            x[0] += x[1] * dt + 0.5f * (a - x[2]) * dt * dt; x[1] += (a - x[2]) * dt;
            const float F[3][3] = {{1, dt, -0.5f * dt * dt}, {0, 1, -dt}, {0, 0, 1}};
            float FP[3][3];
            for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) { float s = 0; for (int k = 0; k < 3; k++) s += F[i][k] * P[k][j]; FP[i][j] = s; }
            for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) { float s = 0; for (int k = 0; k < 3; k++) s += FP[i][k] * F[j][k]; P[i][j] = s; }
            const float qa = c.accel_noise_mps2 * c.accel_noise_mps2;
            P[0][0] += qa * dt * dt * dt / 3.0f; P[0][1] += qa * dt * dt * 0.5f; P[1][0] += qa * dt * dt * 0.5f; P[1][1] += qa * dt; P[2][2] += c.bias_noise_mps3 * c.bias_noise_mps3 * dt;
            x[2] = clampf(x[2], -c.bias_max_mps2, c.bias_max_mps2);
        }
        // innovation y of state idx (already against the delayed state); d_out: the changes of position and velocity
        WB_FAST void update(int idx, float y, float sigma, const PosEstConfig& c, float (&d_out)[2]) {
            const float S = P[idx][idx] + sigma * sigma;
            if (!(S > 1e-9f)) return;
            float K[3]; for (int i = 0; i < 3; i++) K[i] = P[i][idx] / S;
            const float d0 = K[0] * y, d1 = K[1] * y;
            for (int i = 0; i < 3; i++) x[i] += K[i] * y;
            float HP[3]; for (int j = 0; j < 3; j++) HP[j] = P[idx][j];
            for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) P[i][j] -= K[i] * HP[j];
            for (int i = 0; i < 3; i++) for (int j = i + 1; j < 3; j++) { const float m = 0.5f * (P[i][j] + P[j][i]); P[i][j] = P[j][i] = m; }
            for (int i = 0; i < 3; i++) if (P[i][i] < 1e-9f) P[i][i] = 1e-9f;
            x[2] = clampf(x[2], -c.bias_max_mps2, c.bias_max_mps2);
            d_out[0] = d0; d_out[1] = d1;
        }
    } ax_[2];
    // set outright to a measurement: a counted reset
    void set_to(const float (&p)[2], const float (&v)[2], float sp, float sv) {
        for (int a = 0; a < 2; a++) {
            reset_step_[a] = p[a] - ax_[a].x[0];
            ax_[a].x[0] = p[a]; ax_[a].x[1] = v[a];
            for (int j = 0; j < 3; j++) { ax_[a].P[0][j] = ax_[a].P[j][0] = 0.0f; ax_[a].P[1][j] = ax_[a].P[j][1] = 0.0f; }
            ax_[a].P[0][0] = sp * sp; ax_[a].P[1][1] = sv * sv;
        }
        reset_count_++; sync_history();
    }
    void sync_history() { hist_.reset(); const float h[4] = {ax_[0].x[0], ax_[0].x[1], ax_[1].x[0], ax_[1].x[1]}; hist_.push(h, 0.0f); }

    StateHistory<4> hist_;
    bool have_ = false; float rejected_s_ = 0.0f;
    uint8_t reset_count_ = 0; float reset_step_[2] = {0, 0}; uint32_t glitches_ = 0;
    PosInnov in_;
};

}  // namespace nav
}  // namespace fc
