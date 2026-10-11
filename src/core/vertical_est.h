#pragma once
#include "../config/fast_code.h"
#include "types.h"

// ============================================================================
// Vertical Kalman filter for Altitude Hold (up positive).
// State x = [ z (m), v (m/s), b (accel bias, m/s^2), o (baro offset/drift, m) ]
//   predict : z += v dt ; v += (a_meas - b) dt ; b, o = random walk
//   baro    : measures z + o      (o absorbs weather/thermal drift  -> observable only when range/GPS are present)
//   range   : measures z - ground (drift-free, only near ground, tilt-corrected by caller)
//   gps     : measures z          (drift-free but noisy, variance from GPS vAcc) - see "GPS" below
//   gps vel : measures v          (u-blox velD; the module's speed accuracy) - drift-free, ~0.1-0.3 m/s, needs no baro
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
//
// GPS. Its height wanders by one to a few metres over a minute (the satellites move, the air above changes); a barometer is
// steadier over seconds but drifts with the weather and the craft's own warmth. So the GPS height is used in two ways:
//   absolute (update_gps): it IS the height - a craft with neither barometer nor range finder. Measured as z (above home);
//     `gps_reref_after` readings in a row outside the gate: z is set to it (nothing better exists; ArduPilot resets its
//     height to the GPS the same way when the innovations keep failing).
//   relative (update_gps_relative): next to a barometer. The difference GPS - z is averaged over the first `gps_ref_samples`
//     readings (the reference); after that only the CHANGE of the GPS height counts. Fused, it corrects z through the baro
//     offset o, slowly (sigma >= gps_blend_sigma_min: about a minute): the baro's weather drift is taken out over a long
//     flight, a metre of GPS wander is not followed. Not fused, the reference only follows GPS - z, so that the GPS can
//     take over without a step when the barometer goes silent in flight.
//     The reference starts again on arming (restart_gps_ref), after `gps_gap_s` without GPS, and after `gps_reref_after`
//     rejected readings in a row (the GPS jumped: a new reference, z is not dragged).
// The GPS vertical velocity (update_gps_vel) is a different thing: it does not wander, and it is what a barometer is worst
// at (prop wash, fast climbs). The caller brings it forward by the GPS latency with the measured acceleration.
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
    // GPS (see the top of this file)
    float gps_sigma_min = 0.5f;          // GPS height sigma floor (m) when it is the height
    float gps_blend_sigma_min = 3.0f;    // ... when it only corrects the baro drift: its error is correlated over minutes, so it is averaged long
    float gps_vel_sigma_min = 0.15f;     // GPS vertical velocity sigma floor (m/s)
    uint8_t gps_ref_samples = 20;        // GPS heights averaged for the reference (2 s at 10 Hz)
    uint8_t gps_reref_after = 10;        // GPS heights in a row outside the gate: new reference (relative) / z set to it (absolute)
    float gps_jump_m = 4.0f;             // relative: a GPS height this far from the estimate is not used (a jump: satellites lost / gained)
    float gps_gap_s = 3.0f;              // no GPS height for this long: the next one starts a new reference
    float gps_ref_track_tau_s = 10.0f;   // not fused: the reference follows GPS - z with this time constant
    float gps_used_s = 2.0f;             // "GPS used" = fused within this long
};

struct VerticalState { float z = 0, v = 0; bool range_used = false, gps_used = false, gps_vel_used = false; };      // *_used: fused within the last range_gap_s / gps_used_s

class VerticalEstimator {
public:
    KfConfig cfg;
    void reset(float z0 = 0) {
        for (int i = 0; i < 4; i++) { x[i] = 0; for (int j = 0; j < 4; j++) P[i][j] = 0; }
        x[0] = z0; P[0][0] = 1.0f; P[1][1] = 1.0f; P[2][2] = 0.05f; P[3][3] = 0.25f;
        anchor_ = 0.0f; anchored_ = false; range_rejects_ = 0; range_age_ = gps_age_ = gps_vel_age_ = gps_seen_age_ = 1e9f; reanchors_ = 0;
        restart_gps_ref(); gps_resets_ = 0;
    }
    WB_FAST void predict(float a_up_mps2, float dt) {
        if (!(dt > 0.0f) || dt > 0.1f) return;                 // (written so that a NaN does not pass)
        if (range_age_ < 1e8f) range_age_ += dt;
        if (gps_age_ < 1e8f) gps_age_ += dt;
        if (gps_vel_age_ < 1e8f) gps_vel_age_ += dt;
        if (gps_seen_age_ < 1e8f) gps_seen_age_ += dt;
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
    // GPS height (m above home) as THE height: a craft with nothing else
    bool update_gps(float alt_m, float sigma = 1.0f) {
        gps_seen_age_ = 0.0f;
        const float s = fmaxf(sigma, cfg.gps_sigma_min);
        if (scalar(alt_m, 1, 0, s)) { gps_rejects_ = 0; gps_age_ = 0.0f; return true; }
        if (++gps_rejects_ >= cfg.gps_reref_after) { set_z(alt_m, s); gps_rejects_ = 0; gps_age_ = 0.0f; gps_resets_++; return true; }
        return false;
    }
    // GPS height next to a barometer: only its change since the reference counts. fuse = false: the reference only follows.
    // Returns true when the reading corrected the estimate.
    bool update_gps_relative(float alt_m, float sigma, bool fuse) {
        const float since = gps_seen_age_; gps_seen_age_ = 0.0f;
        if (since > cfg.gps_gap_s) restart_gps_ref();
        const float d = alt_m - x[0];
        if (!ref_ok_) {
            ref_sum_ += d;
            if (++ref_n_ >= cfg.gps_ref_samples) { gps_ref_ = ref_sum_ / (float)ref_n_; ref_ok_ = true; gps_rejects_ = 0; }
            return false;
        }
        if (!fuse) { const float k = since < 1e8f ? since / (cfg.gps_ref_track_tau_s + since) : 1.0f; gps_ref_ += (d - gps_ref_) * k; return false; }
        // (a jump is judged in metres, not in sigmas: the sigma of a blending GPS is made large on purpose, to slow it down)
        const float y = alt_m - gps_ref_ - x[0];
        if (fabsf(y) <= cfg.gps_jump_m && scalar(alt_m - gps_ref_, 1, 0, fmaxf(sigma, cfg.gps_sigma_min))) { gps_rejects_ = 0; gps_age_ = 0.0f; return true; }
        if (++gps_rejects_ >= cfg.gps_reref_after) { restart_gps_ref(); gps_resets_++; }
        return false;
    }
    // GPS vertical velocity (m/s, up positive), already brought forward by the GPS latency
    bool update_gps_vel(float vz_up, float sigma) {
        const bool ok = scalar(vz_up, 0, 0, fmaxf(sigma, cfg.gps_vel_sigma_min), 1);
        if (ok) gps_vel_age_ = 0.0f;
        return ok;
    }
    void restart_gps_ref() { ref_ok_ = false; ref_n_ = 0; ref_sum_ = 0.0f; gps_ref_ = 0.0f; gps_rejects_ = 0; }
    bool gps_ref_ok() const { return ref_ok_; }
    float gps_ref() const { return gps_ref_; }               // GPS height - z, as latched (relative use)
    uint32_t gps_resets() const { return gps_resets_; }      // new references / z set to the GPS after rejected readings
    VerticalState state() const {
        VerticalState s; s.z = x[0]; s.v = x[1]; s.range_used = range_age_ <= cfg.range_gap_s;
        s.gps_used = gps_age_ <= cfg.gps_used_s; s.gps_vel_used = gps_vel_age_ <= cfg.gps_used_s; return s;
    }
    float baro_offset() const { return x[3]; }
    float accel_bias() const { return x[2]; }
    float z_sigma() const { return sqrtf(fmaxf(P[0][0], 0)); }
    float range_anchor() const { return anchor_; }           // level of the ground under the craft, relative to where z = 0 would read 0
    uint32_t range_reanchors() const { return reanchors_; }
private:
    // H = [hz, hv, 0, ho]
    WB_FAST bool scalar(float meas, float hz, float ho, float sigma, float hv = 0.0f) {
        float Hx = hz * x[0] + hv * x[1] + ho * x[3], y = meas - Hx;
        float PH[4]; for (int i = 0; i < 4; i++) PH[i] = P[i][0] * hz + P[i][1] * hv + P[i][3] * ho;
        float S = hz * PH[0] + hv * PH[1] + ho * PH[3] + sigma * sigma;
        if (S <= 1e-9f) return false;
        if (y * y > cfg.gate_sigma * cfg.gate_sigma * S) return false;
        float K[4]; for (int i = 0; i < 4; i++) K[i] = PH[i] / S;
        for (int i = 0; i < 4; i++) x[i] += K[i] * y;
        // Joseph-free but symmetric update: P = P - K (H P)
        float HP[4]; for (int j = 0; j < 4; j++) HP[j] = hz * P[0][j] + hv * P[1][j] + ho * P[3][j];
        for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++) P[i][j] -= K[i] * HP[j];
        for (int i = 0; i < 4; i++) for (int j = i + 1; j < 4; j++) { float m = 0.5f * (P[i][j] + P[j][i]); P[i][j] = P[j][i] = m; }
        for (int i = 0; i < 4; i++) if (P[i][i] < 1e-9f) P[i][i] = 1e-9f;
        x[2] = clampf(x[2], -cfg.bias_limit, cfg.bias_limit);
        return true;
    }
    // the ground under the craft is somewhere else now: from here on the readings are measured against this one
    // the height is taken from a measurement outright: z jumps to it, its uncertainty is the measurement's, nothing correlates with it
    void set_z(float z, float sigma) {
        x[0] = z; for (int j = 0; j < 4; j++) { P[0][j] = P[j][0] = 0.0f; } P[0][0] = sigma * sigma;
    }
    void reanchor(float height_m) { anchor_ = height_m - x[0]; anchored_ = true; range_rejects_ = 0; range_age_ = 0.0f; reanchors_++; }
    float x[4] = {}, P[4][4] = {};
    float anchor_ = 0.0f, range_age_ = 1e9f, gps_age_ = 1e9f; bool anchored_ = false; uint8_t range_rejects_ = 0; uint32_t reanchors_ = 0;
    float gps_vel_age_ = 1e9f, gps_seen_age_ = 1e9f, gps_ref_ = 0.0f, ref_sum_ = 0.0f; bool ref_ok_ = false; uint8_t ref_n_ = 0, gps_rejects_ = 0; uint32_t gps_resets_ = 0;
};
} // namespace fc
