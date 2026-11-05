/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>
#include <math.h>
#include <string.h>
#include "mag_calibration.h"

// ============================================================================
// Automatic compass calibration, the way newer Betaflight does it (4.5: started by a stick command or the app, then the pilot
// just picks the craft up and turns it around; no "finish" button). The gyro says when the craft moves; while it moves, every
// compass sample goes into a least-squares sphere fit of the hard-iron offset:
//
//     |u|^2 = 2 b.u + c          u = M * raw (the soft-iron matrix of the calibration in use, identity without one)
//                                b = the offset in u space, c = R^2 - |b|^2, R = the field strength
//
// accumulated as 4x4 normal equations (= recursive least squares without forgetting). The offset is taken back to raw space
// (offset = M^-1 b), the matrix is kept: a full ellipsoid calibration (MagCalibrator) stays valid, only its hard iron is renewed
// - what changes when a battery strap, a GPS mast or a screw is moved.
// It ends by itself when the samples cover enough of the sphere (18 of the 32 bins of MagCalibrator), the estimate has stopped
// moving for 1.5 s, and the craft was turned for at least 4 s; or after 30 s of the window (then with what it has, if that is
// enough). Waiting more than 15 s for the first movement ends it as NO_MOTION. Checks at the end: the field strength 0.15 ..
// 0.9 gauss, the residual under 8 % of it.
// Portable (test_mag); the flight app feeds it from the sensor thread and saves the result itself (Telemetry::fw_saved_*).
// ============================================================================
class MagAutoCal {
public:
    struct Config {
        float motion_dps = 30.0f;                  // the gyro rate that counts as "being turned"
        uint32_t wait_ms = 15000, window_ms = 30000;
        float min_spacing = 0.02f;                 // gauss between kept samples
        uint32_t min_interval_ms = 40;             // and at most 25 samples a second (300 samples = 12 s of turning, noise or not)
        int min_samples = 60, min_bins = 18;
        float settle_g = 0.004f; uint32_t settle_ms = 1500, min_motion_ms = 4000;
        float max_rms_pct = 8.0f, min_radius = 0.15f, max_radius = 0.90f;
        float max_offset = 1.8f;                   // gauss, any axis (ArduPilot COMPASS_OFFS_MAX)
    } cfg;
    enum State : uint8_t { IDLE, WAITING, COLLECTING, DONE, FAILED };
    static constexpr int kMaxSamples = 300, kBins = 32;

    // current: the calibration in use (its matrix is kept; identity when it is not valid)
    void start(uint32_t now_ms, const MagCal& current) {
        st_ = WAITING; t0_ = now_ms; motion_t0_ = 0; motion_ms_ = 0; last_ms_ = now_ms; n_ = 0; result_ = MagCalResult::OK; finished_ = false;
        memset(N_, 0, sizeof(N_)); memset(r_, 0, sizeof(r_)); have_est_ = false; settle_t0_ = now_ms;
        for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) M_[i][j] = current.valid ? current.m[i][j] : (i == j ? 1.0f : 0.0f);
        if (!inv3(M_, Mi_)) for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) { M_[i][j] = Mi_[i][j] = i == j ? 1.0f : 0.0f; }
    }
    void cancel() { st_ = IDLE; finished_ = false; }
    bool active() const { return st_ == WAITING || st_ == COLLECTING; }
    State state() const { return st_; }
    MagCalResult result() const { return result_; }
    const MagCal& calibration() const { return out_; }
    int samples() const { return n_; }
    // DONE / FAILED once, the call after it happened (the app applies / beeps); IDLE otherwise
    State take_finished() { if (!finished_) return IDLE; finished_ = false; return st_; }
    uint8_t progress() const {
        if (st_ == DONE) return 100;
        if (st_ != COLLECTING) return 0;
        const float p = fminf((float)bins() / (float)cfg.min_bins, fminf((float)n_ / (float)cfg.min_samples, (float)motion_ms_ / (float)cfg.min_motion_ms));
        return (uint8_t)(fminf(p, 0.99f) * 100.0f);
    }

    // one compass sample (raw, the chip frame, gauss) and how fast the craft turns (|gyro|, deg/s)
    void feed(const float raw[3], float gyro_dps, uint32_t now_ms) {
        if (!active()) return;
        uint32_t dt = now_ms - last_ms_; last_ms_ = now_ms; if (dt > 100u) dt = 100u;
        const bool moving = gyro_dps >= cfg.motion_dps;
        if (moving && st_ == WAITING) { st_ = COLLECTING; motion_t0_ = now_ms; settle_t0_ = now_ms; }
        if (st_ == COLLECTING && moving) {
            motion_ms_ += dt;
            float u[3]; for (int i = 0; i < 3; i++) u[i] = M_[i][0] * raw[0] + M_[i][1] * raw[1] + M_[i][2] * raw[2];
            if (keep(u)) {
                const double phi[4] = {2.0 * u[0], 2.0 * u[1], 2.0 * u[2], 1.0}, y = (double)u[0] * u[0] + (double)u[1] * u[1] + (double)u[2] * u[2];
                for (int a = 0; a < 4; a++) { for (int b = 0; b < 4; b++) N_[a][b] += phi[a] * phi[b]; r_[a] += phi[a] * y; }
                if (n_ >= 12) {
                    double th[4];
                    if (solve4(th)) {
                        const float b[3] = {(float)th[0], (float)th[1], (float)th[2]};
                        if (!have_est_ || dist(b, est_) > cfg.settle_g) settle_t0_ = now_ms;
                        memcpy(est_, b, sizeof(est_)); c_ = (float)th[3]; have_est_ = true;
                    }
                }
            }
        }
        tick(now_ms);
    }
    // the timeouts (also without samples: a compass that went quiet must not keep the session open)
    void tick(uint32_t now_ms) {
        if (st_ == WAITING && (uint32_t)(now_ms - t0_) >= cfg.wait_ms) { fail(MagCalResult::NO_MOTION); return; }
        if (st_ != COLLECTING) return;
        const bool enough = n_ >= cfg.min_samples && have_est_ && bins() >= cfg.min_bins && motion_ms_ >= cfg.min_motion_ms;
        if (enough && (uint32_t)(now_ms - settle_t0_) >= cfg.settle_ms) { finish(); return; }
        if ((uint32_t)(now_ms - motion_t0_) >= cfg.window_ms) {
            if (n_ < cfg.min_samples || !have_est_) fail(MagCalResult::TOO_FEW_SAMPLES);
            else if (bins() < cfg.min_bins) fail(MagCalResult::POOR_COVERAGE);
            else finish();
        }
    }
    // how many of the 32 bins (8 azimuth sectors x 4 equal-area elevation bands around the estimate) hold a sample
    int bins() const {
        if (!have_est_) return 0;
        uint32_t seen = 0;
        for (int i = 0; i < n_; i++) {
            const float x = s_[i][0] - est_[0], y = s_[i][1] - est_[1], z = s_[i][2] - est_[2], r = sqrtf(x * x + y * y + z * z);
            if (r < 1e-6f) continue;
            float az = atan2f(y, x); if (az < 0) az += 6.28318530718f;
            const float sz = z / r; const int band = sz < -0.5f ? 0 : (sz < 0.0f ? 1 : (sz < 0.5f ? 2 : 3));
            int sec = (int)(az * (8.0f / 6.28318530718f)); if (sec > 7) sec = 7;
            seen |= 1u << (band * 8 + sec);
        }
        int n = 0; for (int b = 0; b < kBins; b++) n += (seen >> b) & 1u;
        return n;
    }

private:
    bool keep(const float u[3]) {
        if (n_ >= kMaxSamples) return false;
        if (u[0] == 0.0f && u[1] == 0.0f && u[2] == 0.0f) return false;
        if (n_ > 0 && (uint32_t)(last_ms_ - kept_ms_) < cfg.min_interval_ms) return false;
        float sp = cfg.min_spacing;                      // (5 % of the field strength: the buffer lasts as long whatever the field is)
        if (have_est_) { const float R2 = c_ + est_[0] * est_[0] + est_[1] * est_[1] + est_[2] * est_[2]; if (R2 > 0.0f && 0.05f * sqrtf(R2) > sp) sp = 0.05f * sqrtf(R2); }
        for (int i = n_ - 1; i >= 0 && i >= n_ - 8; i--) if (dist(u, s_[i]) < sp) return false;     // (not the same spot again: a craft held still on one side)
        memcpy(s_[n_++], u, sizeof(s_[0])); kept_ms_ = last_ms_;
        return true;
    }
    void finish() {
        const float R2 = c_ + est_[0] * est_[0] + est_[1] * est_[1] + est_[2] * est_[2];
        if (!(R2 > 0.0f)) { fail(MagCalResult::NOT_ELLIPSOID); return; }
        const float R = sqrtf(R2);
        if (R < cfg.min_radius || R > cfg.max_radius) { fail(MagCalResult::BAD_FIELD_STRENGTH); return; }
        double ss = 0; for (int i = 0; i < n_; i++) { const double e = dist(s_[i], est_) - R; ss += e * e; }
        const float rms = (float)(100.0 * sqrt(ss / n_) / R);
        if (rms > cfg.max_rms_pct) { fail(MagCalResult::BAD_FIT); return; }
        float off[3]; for (int i = 0; i < 3; i++) off[i] = Mi_[i][0] * est_[0] + Mi_[i][1] * est_[1] + Mi_[i][2] * est_[2];
        if (fabsf(off[0]) > cfg.max_offset || fabsf(off[1]) > cfg.max_offset || fabsf(off[2]) > cfg.max_offset) { fail(MagCalResult::OFFSET_TOO_LARGE); return; }
        out_ = MagCal(); out_.valid = true; out_.radius = R; out_.fit_rms_pct = rms;
        for (int i = 0; i < 3; i++) { out_.offset[i] = off[i]; for (int j = 0; j < 3; j++) out_.m[i][j] = M_[i][j]; }
        st_ = DONE; result_ = MagCalResult::OK; finished_ = true;
    }
    void fail(MagCalResult r) { st_ = FAILED; result_ = r; finished_ = true; }
    static float dist(const float a[3], const float b[3]) { const float x = a[0] - b[0], y = a[1] - b[1], z = a[2] - b[2]; return sqrtf(x * x + y * y + z * z); }
    bool solve4(double th[4]) const {                     // Gauss with partial pivoting on a copy of the normal equations
        double A[4][5]; for (int i = 0; i < 4; i++) { for (int j = 0; j < 4; j++) A[i][j] = N_[i][j]; A[i][4] = r_[i]; }
        for (int c = 0; c < 4; c++) {
            int p = c; for (int i = c + 1; i < 4; i++) if (fabs(A[i][c]) > fabs(A[p][c])) p = i;
            if (fabs(A[p][c]) < 1e-12) return false;
            if (p != c) for (int j = 0; j < 5; j++) { const double t = A[c][j]; A[c][j] = A[p][j]; A[p][j] = t; }
            for (int i = 0; i < 4; i++) if (i != c) { const double f = A[i][c] / A[c][c]; for (int j = c; j < 5; j++) A[i][j] -= f * A[c][j]; }
        }
        for (int i = 0; i < 4; i++) th[i] = A[i][4] / A[i][i];
        return true;
    }
    static bool inv3(const float a[3][3], float o[3][3]) {
        const float d = a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
        if (fabsf(d) < 1e-9f) return false;
        o[0][0] = (a[1][1] * a[2][2] - a[1][2] * a[2][1]) / d; o[0][1] = (a[0][2] * a[2][1] - a[0][1] * a[2][2]) / d; o[0][2] = (a[0][1] * a[1][2] - a[0][2] * a[1][1]) / d;
        o[1][0] = (a[1][2] * a[2][0] - a[1][0] * a[2][2]) / d; o[1][1] = (a[0][0] * a[2][2] - a[0][2] * a[2][0]) / d; o[1][2] = (a[0][2] * a[1][0] - a[0][0] * a[1][2]) / d;
        o[2][0] = (a[1][0] * a[2][1] - a[1][1] * a[2][0]) / d; o[2][1] = (a[0][1] * a[2][0] - a[0][0] * a[2][1]) / d; o[2][2] = (a[0][0] * a[1][1] - a[0][1] * a[1][0]) / d;
        return true;
    }

    State st_ = IDLE; MagCalResult result_ = MagCalResult::OK; bool finished_ = false;
    uint32_t t0_ = 0, motion_t0_ = 0, motion_ms_ = 0, last_ms_ = 0, settle_t0_ = 0, kept_ms_ = 0;
    float M_[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}, Mi_[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    double N_[4][4] = {}, r_[4] = {};
    float s_[kMaxSamples][3]; int n_ = 0;
    float est_[3] = {0, 0, 0}, c_ = 0; bool have_est_ = false;
    MagCal out_;
};
