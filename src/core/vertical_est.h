#pragma once
#include "../config/fast_code.h"
#include "types.h"

// ============================================================================
// Vertical Kalman filter for Altitude Hold (up positive).
// State x = [ z (m), v (m/s), b (accel bias, m/s^2), o (baro offset/drift, m) ]
//   predict : z += v dt ; v += (a_meas - b) dt ; b, o = random walk
//   baro    : measures z + o      (o absorbs weather/thermal drift  -> observable only when range/GPS are present)
//   range   : measures z - ground (drift-free, only near ground, tilt-corrected by caller)
//   gps     : measures z          (drift-free but noisy, variance from GPS vAcc) - only for a craft without barometer
// Why this instead of the old fixed-gain observer: gains come from the covariance, so a source that is missing
// or untrusted simply stops contributing, and a position-only source cannot silently drag velocity
// (the correlation is handled by P, not by hand-tuned L2). Baro drift is modelled the way ArduPilot
// (baroHgtOffset + baroDriftRate) and Betaflight (slow cross-calibration) do: slowly, never by yanking z.
//
// z is the height above the take-off point, and the BAROMETER is what ties it there. The range finder measures something
// else - the distance to whatever is under the craft - and is far more exact (3 cm against 30), so fused as "z" it would win
// every argument: flying over a table, z would step by the height of the table and Altitude Hold would climb by as much.
// So with a barometer the range finder is fused RELATIVE to an anchor (the level of the ground under it):
//   - a reading that fits (inside the gate) corrects z as before - over even ground the height is exact, and the baro drift
//     is learned from it;
//   - `reanchor_after` readings in a row that do not fit, or the first reading after a pause: the ground has changed, not
//     the height. The anchor moves, z does not (Betaflight re-anchors after 10 rejected samples, ArduCopter keeps the
//     surface as a separate offset). The very first reading is measured from the ground the craft was switched on at.
// Without a barometer the range finder is all there is: no re-anchoring, it is the height.
// ============================================================================
namespace fc {
struct KfConfig {
    float q_accel = 0.1f;        // accel noise density (m/s^2 per sqrt(s)); ~ sigma_sample*sqrt(dt) + model error
    float q_bias = 0.002f;       // accel bias walk
    float q_offset = 0.02f;      // baro drift rate (m per sqrt(s)); ArduPilot uses ~0.05 m/s worst case
    float r_baro = 0.4f;         // baro sigma (m) when the sensor layer gives none
    float r_baro_min = 0.3f, r_baro_max = 1.0f;      // what the sensor layer measured on the ground is believed inside this range only:
                                 // it sees the craft being carried about (too large) or a quiet minute indoors (too small for the air in flight)
    float ground_effect_scale = 2.0f;                // baro sigma x this near the ground with the motors running (ArduPilot: variance x 4)
    float r_range = 0.03f;       // rangefinder sigma (m)
    float gate_sigma = 5.0f;     // innovation gate (in sigmas) - rejects glass tables, hands, GPS jumps
    uint8_t reanchor_after = 10; // range readings in a row outside the gate after which the ground is taken to have changed
    float range_gap_s = 0.5f;    // no accepted range reading for this long: the next one starts from a new anchor
    float bias_limit = 1.0f;     // |b| clamp, m/s^2
};

struct VerticalState { float z = 0, v = 0; bool range_used = false, gps_used = false; };      // *_used: fused within the last range_gap_s

class VerticalEstimator {
public:
    KfConfig cfg;
    void reset(float z0 = 0) {
        for (int i = 0; i < 4; i++) { x[i] = 0; for (int j = 0; j < 4; j++) P[i][j] = 0; }
        x[0] = z0; P[0][0] = 1.0f; P[1][1] = 1.0f; P[2][2] = 0.05f; P[3][3] = 0.25f;
        anchor_ = 0.0f; anchored_ = false; range_rejects_ = 0; range_age_ = gps_age_ = 1e9f; reanchors_ = 0;
    }
    WB_FAST void predict(float a_up_mps2, float dt) {
        if (!(dt > 0.0f) || dt > 0.1f) return;                 // (written so that a NaN does not pass)
        if (range_age_ < 1e8f) range_age_ += dt;
        if (gps_age_ < 1e8f) gps_age_ += dt;
        x[0] += x[1] * dt + 0.5f * (a_up_mps2 - x[2]) * dt * dt;
        x[1] += (a_up_mps2 - x[2]) * dt;
        // P = F P F^T + Q with F = [[1,dt,-dt^2/2,0],[0,1,-dt,0],[0,0,1,0],[0,0,0,1]]
        float F[4][4] = {{1, dt, -0.5f * dt * dt, 0}, {0, 1, -dt, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
        float FP[4][4], N[4][4];
        for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++) { float s = 0; for (int k = 0; k < 4; k++) s += F[i][k] * P[k][j]; FP[i][j] = s; }
        for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++) { float s = 0; for (int k = 0; k < 4; k++) s += FP[i][k] * F[j][k]; N[i][j] = s; }
        float qa = cfg.q_accel * cfg.q_accel; // continuous white-noise accel: Q = qa*[[dt^3/3, dt^2/2],[dt^2/2, dt]]
        N[0][0] += qa * dt * dt * dt / 3.0f;
        N[0][1] += qa * dt * dt * 0.5f; N[1][0] += qa * dt * dt * 0.5f;
        N[1][1] += qa * dt;
        N[2][2] += cfg.q_bias * cfg.q_bias * dt;
        N[3][3] += cfg.q_offset * cfg.q_offset * dt;
        for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++) P[i][j] = N[i][j];
        x[2] = clampf(x[2], -cfg.bias_limit, cfg.bias_limit);
    }
    // Returns false if the sample was gated out.
    // sigma = the sensor layer's own noise estimate (BaroAltitude::sigma_m(), RangeHeight::sigma_m(), GPS vAcc); <= 0 => configured default.
    // ground_effect: the craft is on / just off the ground with the motors running - the air under it is pushed about.
    bool update_baro(float baro_m, float sigma = 0.0f, bool ground_effect = false) {
        float s = sigma > 0 ? clampf(sigma, cfg.r_baro_min, cfg.r_baro_max) : cfg.r_baro;
        if (ground_effect) s *= cfg.ground_effect_scale;
        return scalar(baro_m, 1, 1, s);
    }
    // relative: a barometer holds z (see the top of this file); false: the range finder is the only height there is.
    bool update_range(float height_m /*tilt-corrected*/, float sigma = 0.0f, bool relative = false) {
        const float sg = sigma > 0 ? sigma : cfg.r_range;
        if (relative && anchored_ && range_age_ > cfg.range_gap_s) { reanchor(height_m); return false; }      // back after a pause: a new stretch of ground, its level unknown
        if (scalar(height_m - anchor_, 1, 0, sg)) { range_rejects_ = 0; range_age_ = 0.0f; anchored_ = true; return true; }
        if (relative && ++range_rejects_ >= cfg.reanchor_after) reanchor(height_m);
        return false;
    }
    bool update_gps(float alt_m, float sigma = 1.0f) { const bool ok = scalar(alt_m, 1, 0, fmaxf(sigma, 0.5f)); if (ok) gps_age_ = 0.0f; return ok; }
    VerticalState state() const { VerticalState s; s.z = x[0]; s.v = x[1]; s.range_used = range_age_ <= cfg.range_gap_s; s.gps_used = gps_age_ <= cfg.range_gap_s * 4.0f; return s; }
    float baro_offset() const { return x[3]; }
    float accel_bias() const { return x[2]; }
    float z_sigma() const { return sqrtf(fmaxf(P[0][0], 0)); }
    float range_anchor() const { return anchor_; }           // level of the ground under the craft, relative to where z = 0 would read 0
    uint32_t range_reanchors() const { return reanchors_; }
private:
    // H = [hz, 0, 0, ho]
    WB_FAST bool scalar(float meas, float hz, float ho, float sigma) {
        float Hx = hz * x[0] + ho * x[3], y = meas - Hx;
        float PH[4]; for (int i = 0; i < 4; i++) PH[i] = P[i][0] * hz + P[i][3] * ho;
        float S = hz * PH[0] + ho * PH[3] + sigma * sigma;
        if (S <= 1e-9f) return false;
        if (y * y > cfg.gate_sigma * cfg.gate_sigma * S) return false;
        float K[4]; for (int i = 0; i < 4; i++) K[i] = PH[i] / S;
        for (int i = 0; i < 4; i++) x[i] += K[i] * y;
        // Joseph-free but symmetric update: P = P - K (H P)
        float HP[4]; for (int j = 0; j < 4; j++) HP[j] = hz * P[0][j] + ho * P[3][j];
        for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++) P[i][j] -= K[i] * HP[j];
        for (int i = 0; i < 4; i++) for (int j = i + 1; j < 4; j++) { float m = 0.5f * (P[i][j] + P[j][i]); P[i][j] = P[j][i] = m; }
        for (int i = 0; i < 4; i++) if (P[i][i] < 1e-9f) P[i][i] = 1e-9f;
        x[2] = clampf(x[2], -cfg.bias_limit, cfg.bias_limit);
        return true;
    }
    // the ground under the craft is somewhere else now: from here on the readings are measured against this one
    void reanchor(float height_m) { anchor_ = height_m - x[0]; anchored_ = true; range_rejects_ = 0; range_age_ = 0.0f; reanchors_++; }
    float x[4] = {}, P[4][4] = {};
    float anchor_ = 0.0f, range_age_ = 1e9f, gps_age_ = 1e9f; bool anchored_ = false; uint8_t range_rejects_ = 0; uint32_t reanchors_ = 0;
};
} // namespace fc
