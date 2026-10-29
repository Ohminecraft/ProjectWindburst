/*
 * This file is part of Windburst Project
 */

#pragma once
#include "../../config/fast_code.h"
#include "../types.h"
#include "state_history.h"

// ============================================================================
// AltEstimator - the height and climb rate the navigation stack flies on (nav rewrite P1, docs/NAV_ROADMAP.md).
//
// Frame NED with the origin at home: pos_d is DOWN (a craft 2 m above the take-off point has pos_d = -2), vel_d down +.
// State x = [pos_d (m), vel_d (m/s), bias_d (accelerometer bias along down, m/s^2), baro_off (m)]:
//   predict   pos_d += vel_d dt + (a_d - bias_d) dt^2 / 2,  vel_d += (a_d - bias_d) dt     a_d: NED down acceleration, gravity removed
//   baro      height_baro = -pos_d + baro_off                 (baro_off takes the weather / warmth drift, slowly)
//   range     height_range = -pos_d - ground                  ("ground" = level of what is under the craft, see below)
//   GPS       height_gps = -pos_d (+ a reference, see below);  GPS vertical velocity = vel_d
//   still     vel_d = 0 while disarmed (zero-velocity update)
//
// What is new against the estimator before 0.35 (vertical_est.h, removed):
//   - NED, the frame every other part of the new stack uses;
//   - late measurements are compared with the state when they were TAKEN (StateHistory: GPS height and vertical velocity
//     `gps_delay_s`, barometer `baro_delay_s`), not brought forward with the acceleration;
//   - every measurement leaves its innovation and test ratio (innovation^2 / (gate^2 x its variance): > 1 = rejected) for the log;
//   - a reset of the height (the barometer rejected for too long, the GPS taken as the height) is COUNTED with its step:
//     controllers that hold a height target move the target by the same step instead of flying the step (ArduPilot's
//     "reset handling": AP_NavEKF3 getPosDownReset / AC_PosControl::handle_ekf_z_reset).
//
// Design taken from (algorithms only, written anew here):
//   ArduPilot AP_NavEKF3 - height innovation gate, "height reset" after a timeout of rejected baro readings, ground effect
//     (baro variance raised near the ground while the motors run), zero-velocity fusion on the ground.
//   PX4 EKF2 (ECL) - fusing late measurements against the delayed state.
//   Betaflight / INAV - the range finder re-anchored when the ground under it changes (a table), the GPS height averaged against
//     the barometer instead of trusted (INAV: GPS / baro blend; Betaflight: 200 s time constant).
//
// Range finder: with a barometer it is fused RELATIVE to an anchor (the ground level under it). A reading that fits corrects the
// height (and the baro drift through the correlation); `reanchor_after` misfits in a row, or the first reading after a pause, mean
// the ground changed - the anchor moves, the height does not. Without a barometer it is the height itself.
// GPS height: as THE height when nothing else exists (update_gps_abs: `gps_reref_after` misfits in a row -> the height is set to
// it, a counted reset); next to a barometer only its CHANGE counts (update_gps_rel: the mean difference over the first
// `gps_ref_samples` readings is the reference; fused it corrects the baro offset slowly, not fused the reference follows so the GPS
// can take over without a step when the barometer goes silent).
// Every update returns whether the reading was used. Every entry point copes with a bad dt (<= 0, NaN, > 0.1 s: ignored).
// ============================================================================
namespace fc {
namespace nav {

struct AltEstConfig {
    float accel_noise_mps2 = 0.1f;       // process noise of the acceleration (m/s^2 per sqrt(s))
    float bias_noise_mps3 = 0.002f;      // random walk of the accelerometer bias
    float baro_drift_mps = 0.02f;        // random walk of the baro offset (m per sqrt(s))
    float baro_sigma_m = 0.4f;           // baro noise when the sensor layer gives none
    float baro_sigma_min_m = 0.3f, baro_sigma_max_m = 1.0f;     // the sensor layer's own figure is believed inside this range only
    float baro_delay_s = 0.0f;           // how old a baro reading is when it arrives
    float ground_effect_scale = 2.0f;    // baro sigma x this near the ground with the motors running
    float range_sigma_m = 0.03f;
    float gate_sigma = 5.0f;             // innovation gate, in sigmas
    uint8_t reanchor_after = 10;         // range readings in a row outside the gate: the ground changed
    float range_gap_s = 0.5f;            // no accepted range reading this long: the next starts from a new anchor
    float bias_max_mps2 = 1.0f;
    float gps_delay_s = 0.15f;           // how old a GPS solution is when it arrives
    float gps_sigma_min_m = 0.5f;        // GPS height sigma floor when it is the height
    float gps_blend_sigma_min_m = 3.0f;  // ... when it only takes out the baro drift (averaged over about a minute)
    float gps_vel_sigma_min_mps = 0.15f;
    uint8_t gps_ref_samples = 20;
    uint8_t gps_reref_after = 10;
    float gps_jump_m = 4.0f;             // next to a barometer: a GPS height this far off is a jump, not used
    float gps_gap_s = 3.0f;              // no GPS height this long: a new reference
    float gps_ref_tau_s = 10.0f;         // not fused: the reference follows GPS - height with this time constant
    float used_window_s = 2.0f;          // "GPS used" = fused within this long
    float baro_reset_s = 1.5f;           // no baro reading accepted this long while they keep coming: the height is reset to the baro
    float still_sigma_mps = 0.3f;
};

// one innovation, for the log
struct Innov { float value = 0, ratio = 0; bool used = false; };

class AltEstimator {
public:
    AltEstConfig cfg;

    void reset(float height_m = 0.0f) {
        for (int i = 0; i < 4; i++) { x_[i] = 0; for (int j = 0; j < 4; j++) P_[i][j] = 0; }
        x_[0] = -height_m; P_[0][0] = 1.0f; P_[1][1] = 1.0f; P_[2][2] = 0.05f; P_[3][3] = 0.25f;
        hist_.reset();
        ground_ = 0; anchored_ = false; range_rejects_ = 0; reanchors_ = 0;
        range_age_ = gps_age_ = gps_vel_age_ = gps_seen_age_ = baro_age_ = kNever;
        restart_gps_ref(); gps_resets_ = 0; baro_resets_ = 0; reset_count_ = 0; reset_step_ = 0;
        baro_in_ = range_in_ = gps_in_ = gps_vel_in_ = Innov();
    }

    WB_FAST void predict(float acc_down_mps2, float dt) {
        if (!(dt > 0.0f) || dt > 0.1f || acc_down_mps2 != acc_down_mps2) return;
        age(range_age_, dt); age(gps_age_, dt); age(gps_vel_age_, dt); age(gps_seen_age_, dt); age(baro_age_, dt);
        const float a = acc_down_mps2 - x_[2];
        x_[0] += x_[1] * dt + 0.5f * a * dt * dt;
        x_[1] += a * dt;
        // P = F P F' + Q,  F = [[1, dt, -dt^2/2, 0], [0, 1, -dt, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
        const float F[4][4] = {{1, dt, -0.5f * dt * dt, 0}, {0, 1, -dt, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
        float FP[4][4];
        for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++) { float s = 0; for (int k = 0; k < 4; k++) s += F[i][k] * P_[k][j]; FP[i][j] = s; }
        for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++) { float s = 0; for (int k = 0; k < 4; k++) s += FP[i][k] * F[j][k]; P_[i][j] = s; }
        const float qa = cfg.accel_noise_mps2 * cfg.accel_noise_mps2;          // white acceleration noise, continuous
        P_[0][0] += qa * dt * dt * dt / 3.0f; P_[0][1] += qa * dt * dt * 0.5f; P_[1][0] += qa * dt * dt * 0.5f; P_[1][1] += qa * dt;
        P_[2][2] += cfg.bias_noise_mps3 * cfg.bias_noise_mps3 * dt;
        P_[3][3] += cfg.baro_drift_mps * cfg.baro_drift_mps * dt;
        x_[2] = clampf(x_[2], -cfg.bias_max_mps2, cfg.bias_max_mps2);
        const float h[2] = {x_[0], x_[1]}; hist_.push(h, dt);
    }

    // Barometer height above the take-off point (m, up +). sigma: the sensor layer's noise (<= 0: the configured one).
    // ground_effect: on or just off the ground with the motors running.
    bool update_baro(float height_m, float sigma_m = 0.0f, bool ground_effect = false) {
        if (height_m != height_m) return false;
        float s = sigma_m > 0.0f ? clampf(sigma_m, cfg.baro_sigma_min_m, cfg.baro_sigma_max_m) : cfg.baro_sigma_m;
        if (ground_effect) s *= cfg.ground_effect_scale;
        const float H[4] = {-1, 0, 0, 1};
        if (fuse(height_m, H, s, cfg.baro_delay_s, true, baro_in_)) { baro_age_ = 0.0f; return true; }
        if (baro_age_ > cfg.baro_reset_s) {                    // (also the very first reading: "never" is older than anything)
            set_height(height_m - x_[3], s);
            x_[1] = 0.0f; for (int j = 0; j < 4; j++) P_[1][j] = P_[j][1] = 0.0f; P_[1][1] = 1.0f;
            baro_age_ = 0.0f; baro_resets_++; return true;
        }
        return false;
    }
    // disarmed: the craft does not climb (not gated: an accelerometer spike at touch-down that left a large vel_d is exactly
    // the case; far off, the variance is widened to the error first so the update takes most of it out at once)
    bool update_still() {
        if (fabsf(x_[1]) > 3.0f * cfg.still_sigma_mps) { const float w = x_[1] * x_[1]; if (P_[1][1] < w) P_[1][1] = w; }
        const float H[4] = {0, 1, 0, 0}; Innov dummy;
        return fuse(0.0f, H, cfg.still_sigma_mps, 0.0f, false, dummy);
    }
    // Range finder height (tilt-corrected, m). relative: a barometer holds the height (see the top); false: it is the height.
    bool update_range(float height_m, float sigma_m = 0.0f, bool relative = false) {
        if (height_m != height_m) return false;
        const float s = sigma_m > 0.0f ? sigma_m : cfg.range_sigma_m;
        if (relative && anchored_ && range_age_ > cfg.range_gap_s) { reanchor(height_m); return false; }
        const float H[4] = {-1, 0, 0, 0};
        if (fuse(height_m + ground_, H, s, 0.0f, true, range_in_)) { range_rejects_ = 0; range_age_ = 0.0f; anchored_ = true; return true; }
        if (relative && ++range_rejects_ >= cfg.reanchor_after) reanchor(height_m);
        return false;
    }
    // GPS height above home as THE height (nothing else there)
    bool update_gps_abs(float height_m, float sigma_m) {
        if (height_m != height_m) return false;
        gps_seen_age_ = 0.0f;
        const float s = maxf(sigma_m, cfg.gps_sigma_min_m), H[4] = {-1, 0, 0, 0};
        if (fuse(height_m, H, s, cfg.gps_delay_s, true, gps_in_)) { gps_rejects_ = 0; gps_age_ = 0.0f; return true; }
        if (++gps_rejects_ >= cfg.gps_reref_after) { set_height(height_m, s); gps_rejects_ = 0; gps_age_ = 0.0f; gps_resets_++; return true; }
        return false;
    }
    // GPS height next to a barometer: only its change since the reference counts. fuse = false: the reference only follows.
    bool update_gps_rel(float height_m, float sigma_m, bool fuse_it) {
        if (height_m != height_m) return false;
        const float since = gps_seen_age_; gps_seen_age_ = 0.0f;
        if (since > cfg.gps_gap_s) restart_gps_ref();
        float xd[2]; const float h_then = hist_.at(cfg.gps_delay_s, xd) ? -xd[0] : height();
        const float d = height_m - h_then;
        if (!ref_ok_) {
            ref_sum_ += d;
            if (++ref_n_ >= cfg.gps_ref_samples) { gps_ref_ = ref_sum_ / (float)ref_n_; ref_ok_ = true; gps_rejects_ = 0; }
            return false;
        }
        if (!fuse_it) { const float k = since < kNever ? since / (cfg.gps_ref_tau_s + since) : 1.0f; gps_ref_ += (d - gps_ref_) * k; return false; }
        // (a jump is judged in metres: the sigma of a blending GPS is made large on purpose, to slow it down)
        const float H[4] = {-1, 0, 0, 0};
        if (fabsf(d - gps_ref_) <= cfg.gps_jump_m && fuse(height_m - gps_ref_, H, maxf(sigma_m, cfg.gps_sigma_min_m), cfg.gps_delay_s, true, gps_in_)) { gps_rejects_ = 0; gps_age_ = 0.0f; return true; }
        if (++gps_rejects_ >= cfg.gps_reref_after) { restart_gps_ref(); gps_resets_++; }
        return false;
    }
    // GPS vertical velocity (m/s, DOWN +), as the module measured it gps_delay_s ago
    bool update_gps_vel(float vel_d_mps, float sigma_mps) {
        if (vel_d_mps != vel_d_mps) return false;
        const float H[4] = {0, 1, 0, 0};
        const bool ok = fuse(vel_d_mps, H, maxf(sigma_mps, cfg.gps_vel_sigma_min_mps), cfg.gps_delay_s, true, gps_vel_in_);
        if (ok) gps_vel_age_ = 0.0f;
        return ok;
    }
    void restart_gps_ref() { ref_ok_ = false; ref_n_ = 0; ref_sum_ = 0.0f; gps_ref_ = 0.0f; gps_rejects_ = 0; }

    // ---- the estimate
    float pos_d() const { return x_[0]; }
    float vel_d() const { return x_[1]; }
    float height() const { return -x_[0]; }                  // m above the take-off point
    float climb_rate() const { return -x_[1]; }              // m/s up
    float accel_bias() const { return x_[2]; }               // m/s^2 along down
    float baro_offset() const { return x_[3]; }
    float height_sigma() const { return sqrtf(maxf(P_[0][0], 0.0f)); }
    bool range_used() const { return range_age_ <= cfg.range_gap_s; }
    bool gps_used() const { return gps_age_ <= cfg.used_window_s; }
    bool gps_vel_used() const { return gps_vel_age_ <= cfg.used_window_s; }
    bool gps_ref_ok() const { return ref_ok_; }
    float gps_ref() const { return gps_ref_; }
    float range_ground() const { return ground_; }            // level of the ground under the craft (m, relative to where height 0 reads 0)
    uint32_t range_reanchors() const { return reanchors_; }
    uint32_t baro_resets() const { return baro_resets_; }
    uint32_t gps_resets() const { return gps_resets_; }
    // every step the height was SET (not filtered towards a reading): count and the last step of pos_d (m, down +)
    uint8_t reset_count() const { return reset_count_; }
    float reset_step_d() const { return reset_step_; }
    const Innov& baro_innov() const { return baro_in_; }
    const Innov& range_innov() const { return range_in_; }
    const Innov& gps_innov() const { return gps_in_; }
    const Innov& gps_vel_innov() const { return gps_vel_in_; }

private:
    static constexpr float kNever = 1e9f;
    static void age(float& a, float dt) { if (a < kNever) a += dt; }

    // One scalar measurement z = H x (H over the 4 states), compared with the state `delay_s` ago, gated (gate = false: not).
    WB_FAST bool fuse(float z, const float (&H)[4], float sigma, float delay_s, bool gate, Innov& rec) {
        float past[2] = {x_[0], x_[1]};
        if (delay_s > 0.0f) hist_.at(delay_s, past);
        const float xp[4] = {past[0], past[1], x_[2], x_[3]};
        float PH[4]; for (int i = 0; i < 4; i++) { PH[i] = 0; for (int k = 0; k < 4; k++) PH[i] += P_[i][k] * H[k]; }
        float S = sigma * sigma; for (int k = 0; k < 4; k++) S += H[k] * PH[k];
        float hx = 0; for (int k = 0; k < 4; k++) hx += H[k] * xp[k];
        const float y = z - hx;
        rec.value = y; rec.ratio = S > 1e-9f ? y * y / (cfg.gate_sigma * cfg.gate_sigma * S) : 0.0f; rec.used = false;
        if (!(S > 1e-9f) || (gate && rec.ratio > 1.0f)) return false;
        float K[4], dx[4]; for (int i = 0; i < 4; i++) { K[i] = PH[i] / S; dx[i] = K[i] * y; x_[i] += dx[i]; }
        float HP[4]; for (int j = 0; j < 4; j++) { HP[j] = 0; for (int k = 0; k < 4; k++) HP[j] += H[k] * P_[k][j]; }
        for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++) P_[i][j] -= K[i] * HP[j];
        for (int i = 0; i < 4; i++) for (int j = i + 1; j < 4; j++) { const float m = 0.5f * (P_[i][j] + P_[j][i]); P_[i][j] = P_[j][i] = m; }
        for (int i = 0; i < 4; i++) if (P_[i][i] < 1e-9f) P_[i][i] = 1e-9f;
        x_[2] = clampf(x_[2], -cfg.bias_max_mps2, cfg.bias_max_mps2);
        const float sh[2] = {dx[0], dx[1]}; hist_.shift(sh);
        rec.used = true;
        return true;
    }
    // the height is taken from a measurement outright: a counted reset (controllers move their targets by the step)
    void set_height(float height_m, float sigma) {
        const float step = -height_m - x_[0];
        x_[0] = -height_m; for (int j = 0; j < 4; j++) P_[0][j] = P_[j][0] = 0.0f; P_[0][0] = sigma * sigma;
        const float sh[2] = {step, 0.0f}; hist_.shift(sh);
        reset_count_++; reset_step_ = step;
    }
    void reanchor(float height_m) { ground_ = height() - height_m; anchored_ = true; range_rejects_ = 0; range_age_ = 0.0f; reanchors_++; }

    float x_[4] = {}, P_[4][4] = {};
    StateHistory<2> hist_;
    float ground_ = 0.0f; bool anchored_ = false; uint8_t range_rejects_ = 0; uint32_t reanchors_ = 0;
    float range_age_ = kNever, gps_age_ = kNever, gps_vel_age_ = kNever, gps_seen_age_ = kNever, baro_age_ = kNever;
    float gps_ref_ = 0.0f, ref_sum_ = 0.0f; bool ref_ok_ = false; uint8_t ref_n_ = 0, gps_rejects_ = 0; uint32_t gps_resets_ = 0, baro_resets_ = 0;
    uint8_t reset_count_ = 0; float reset_step_ = 0.0f;
    Innov baro_in_, range_in_, gps_in_, gps_vel_in_;
};

}  // namespace nav
}  // namespace fc
