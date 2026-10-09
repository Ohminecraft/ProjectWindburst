/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>
#include <math.h>
#include "acc_gyro_util.h"

// ============================================================================
// Accelerometer calibration (portable, no Arduino).
//
//  - GyroCalibration : non-blocking gyro bias (the loop keeps running while it averages).
//  - AccCalibrator   : 6-face calibration. The craft is laid still on each of its six faces in ANY order; every face is one
//                      averaged reading. The face is recognised from the reading itself (dominant axis + sign), so the user
//                      cannot capture "the wrong one". fit() then solves offset[3] + scale[3] so that all six readings have a
//                      length of exactly 1 g. Because it fits the LENGTH, the faces do not have to be perfectly square to
//                      gravity (a 10 degree tilted table gives the same answer) - unlike the usual "(max+min)/2" method.
//  - level_trim()    : roll/pitch board-alignment trim that makes the current resting attitude read as level.
//
// The calibration lives in the CHIP frame (before the board alignment), like the compass calibration: changing the board
// alignment later does not invalidate it.            calibrated = (raw - offset) * scale
// ============================================================================

struct AccCal {
    bool valid = false;
    float offset[3] = {0, 0, 0};       // g
    float scale[3] = {1, 1, 1};
};

inline void acc_apply_cal(float v[3], const AccCal& c) {
    if (!c.valid) return;
    for (int k = 0; k < 3; k++) v[k] = (v[k] - c.offset[k]) * c.scale[k];
}

enum class CalState : uint8_t { IDLE, RUNNING, DONE, FAILED };      // == schema enum cal_state

// Averages N samples of a 3-vector and tells whether the source was still while doing it.
class StillAverager {
public:
    void begin(uint16_t samples) { want_ = samples ? samples : 1; n_ = 0; for (int k = 0; k < 3; k++) { s_[k] = 0; s2_[k] = 0; } state_ = CalState::RUNNING; }
    void cancel() { state_ = CalState::IDLE; }
    // returns true on the call that completes the average
    bool feed(const float v[3]) {
        if (state_ != CalState::RUNNING) return false;
        for (int k = 0; k < 3; k++) { s_[k] += v[k]; s2_[k] += (double)v[k] * v[k]; }
        if (++n_ < want_) return false;
        for (int k = 0; k < 3; k++) { mean_[k] = (float)(s_[k] / n_); const double var = s2_[k] / n_ - (double)mean_[k] * mean_[k]; std_[k] = var > 0 ? (float)sqrt(var) : 0.0f; }
        state_ = CalState::DONE;
        return true;
    }
    CalState state() const { return state_; }
    bool running() const { return state_ == CalState::RUNNING; }
    uint8_t progress() const { return state_ == CalState::RUNNING ? (uint8_t)((uint32_t)n_ * 100u / want_) : (state_ == CalState::IDLE ? 0 : 100); }
    const float* mean() const { return mean_; }
    float max_std() const { float m = std_[0]; if (std_[1] > m) m = std_[1]; if (std_[2] > m) m = std_[2]; return m; }
    void fail() { state_ = CalState::FAILED; }
private:
    uint16_t want_ = 1, n_ = 0; double s_[3] = {}, s2_[3] = {}; float mean_[3] = {}, std_[3] = {}; CalState state_ = CalState::IDLE;
};

// Gyro bias: same acceptance rule as IMUAccGyro::calibrateGyro() (mean < 30 deg/s, variance < 4 (deg/s)^2), but fed from the running loop.
class GyroCalibration {
public:
    void begin(uint16_t samples) { avg_.begin(samples); }
    // raw = gyro in deg/s WITHOUT bias removal. true when it just finished successfully: bias[] is the new bias.
    bool feed(const float raw[3], float bias[3]) {
        if (!avg_.feed(raw)) return false;
        const float* m = avg_.mean();
        if (fabsf(m[0]) > 30.0f || fabsf(m[1]) > 30.0f || fabsf(m[2]) > 30.0f || avg_.max_std() > 2.0f) { avg_.fail(); return false; }
        for (int k = 0; k < 3; k++) bias[k] = m[k];
        return true;
    }
    CalState state() const { return avg_.state(); }
    bool running() const { return avg_.running(); }
    uint8_t progress() const { return avg_.progress(); }
private:
    StillAverager avg_;
};

class AccCalibrator {
public:
    float max_std_g = 0.02f;           // a hand-held craft is ~0.05 g, one resting on a table ~0.003 g
    float min_dominance = 0.80f;       // the face axis must carry >= 80 % of the reading (<= 37 degrees off)

    void reset() { mask_ = 0; avg_.cancel(); last_face_ = -1; }
    // Start averaging one face (~1 s of samples). Does not clear the faces already captured.
    void begin_point(uint16_t samples) { avg_.begin(samples); last_face_ = -1; }
    void cancel_point() { avg_.cancel(); }
    // raw accelerometer in g, chip frame, UNcalibrated. true when a face was just captured.
    bool feed(const float raw[3]) {
        if (!avg_.feed(raw)) return false;
        const float* m = avg_.mean();
        const float n = sqrtf(m[0] * m[0] + m[1] * m[1] + m[2] * m[2]);
        int ax = 0; if (fabsf(m[1]) > fabsf(m[ax])) ax = 1; if (fabsf(m[2]) > fabsf(m[ax])) ax = 2;
        if (avg_.max_std() > max_std_g || n < 0.7f || n > 1.3f || fabsf(m[ax]) < min_dominance * n) { avg_.fail(); return false; }
        const int face = ax * 2 + (m[ax] < 0 ? 1 : 0);
        for (int k = 0; k < 3; k++) pt_[face][k] = m[k];
        mask_ |= (uint8_t)(1u << face); last_face_ = face;
        return true;
    }
    CalState state() const { return avg_.state(); }
    bool running() const { return avg_.running(); }
    uint8_t progress() const { return avg_.progress(); }
    uint8_t mask() const { return mask_; }                 // bit: +X -X +Y -Y +Z -Z (sign of the raw reading on that chip axis)
    bool complete() const { return mask_ == 0x3F; }
    int last_face() const { return last_face_; }
    const float* current() const { return avg_.mean(); }

    // Solve offset + scale from the six faces. false: not all faces yet, or the result is implausible (out stays untouched).
    bool fit(AccCal& out) const {
        if (!complete()) return false;
        double p[6];                                         // o0 o1 o2 s0 s1 s2, initial guess from the opposite faces
        for (int k = 0; k < 3; k++) {
            const double hi = pt_[k * 2][k], lo = pt_[k * 2 + 1][k];
            if (hi - lo < 1.0) return false;
            p[k] = 0.5 * (hi + lo); p[3 + k] = 2.0 / (hi - lo);
        }
        for (int it = 0; it < 12; it++) {                     // Newton on r_i = |S (m_i - o)|^2 - 1 = 0, six equations, six unknowns
            double J[6][7];
            for (int i = 0; i < 6; i++) {
                double r = -1.0;
                for (int k = 0; k < 3; k++) {
                    const double d = pt_[i][k] - p[k], s = p[3 + k];
                    r += s * s * d * d;
                    J[i][k] = -2.0 * s * s * d;
                    J[i][3 + k] = 2.0 * s * d * d;
                }
                J[i][6] = -r;
            }
            double dx[6];
            if (!solve6(J, dx)) return false;
            double step = 0;
            for (int k = 0; k < 6; k++) { p[k] += dx[k]; step += fabs(dx[k]); }
            if (step < 1e-9) break;
        }
        double worst = 0;
        for (int i = 0; i < 6; i++) { double r = -1.0; for (int k = 0; k < 3; k++) { const double d = (pt_[i][k] - p[k]) * p[3 + k]; r += d * d; } if (fabs(r) > worst) worst = fabs(r); }
        if (!(worst < 1e-4)) return false;
        for (int k = 0; k < 3; k++) if (!(fabs(p[k]) < 0.5) || !(p[3 + k] > 0.8 && p[3 + k] < 1.25)) return false;     // an offset of 0.5 g / a 25 % scale error is a broken chip or a wrong range
        out.valid = true;
        for (int k = 0; k < 3; k++) { out.offset[k] = (float)p[k]; out.scale[k] = (float)p[3 + k]; }
        return true;
    }

private:
    static bool solve6(double a[6][7], double x[6]) {        // Gaussian elimination with partial pivoting
        for (int c = 0; c < 6; c++) {
            int piv = c; for (int r = c + 1; r < 6; r++) if (fabs(a[r][c]) > fabs(a[piv][c])) piv = r;
            if (fabs(a[piv][c]) < 1e-12) return false;
            if (piv != c) for (int k = 0; k < 7; k++) { const double t = a[c][k]; a[c][k] = a[piv][k]; a[piv][k] = t; }
            for (int r = c + 1; r < 6; r++) { const double f = a[r][c] / a[c][c]; for (int k = c; k < 7; k++) a[r][k] -= f * a[c][k]; }
        }
        for (int r = 5; r >= 0; r--) { double s = a[r][6]; for (int k = r + 1; k < 6; k++) s -= a[r][k] * x[k]; x[r] = s / a[r][r]; }
        return true;
    }
    StillAverager avg_; float pt_[6][3] = {}; uint8_t mask_ = 0; int last_face_ = -1;
};

// Level trim. acc_board = the accelerometer in the board frame as IMUAccGyro::read() delivers it right now (g, specific force, any
// length), cur_* = the fine alignment angles that are currently applied. Returns the TOTAL roll/pitch fine alignment (degrees) for
// IMUAccGyro::setAlignment() that makes this resting attitude read exactly level (0, 0, -1 g). The yaw trim is not changed.
// false if the vector is not a usable gravity reading or the craft is more than max_deg from level (wrong board alignment, not a trim).
inline bool level_trim(const float acc_board[3], float cur_roll, float cur_pitch, float cur_yaw, float& roll, float& pitch, float max_deg = 20.0f) {
    const float n = sqrtf(acc_board[0] * acc_board[0] + acc_board[1] * acc_board[1] + acc_board[2] * acc_board[2]);
    if (!(n > 0.7f && n < 1.3f)) return false;
    float m[3][3]; accgyro::build_rotation(m, cur_roll, cur_pitch, cur_yaw);
    // apply_rotation() computes board = M^T * sensor, so sensor = M * board
    float g[3];
    for (int i = 0; i < 3; i++) g[i] = (m[i][0] * acc_board[0] + m[i][1] * acc_board[1] + m[i][2] * acc_board[2]) / n;
    // we need M(r,p)^T * g = (0,0,-1)  =>  g = -(third column of M) = (-sin p, sin r cos p, -cos r cos p)
    const float sp = g[0] < -1 ? 1.0f : (g[0] > 1 ? -1.0f : -g[0]);
    const float p = asinf(sp), r = atan2f(g[1], -g[2]);
    const float rad2deg = 57.29577951308232f;
    if (fabsf(p * rad2deg) > max_deg || fabsf(r * rad2deg) > max_deg) return false;
    roll = r * rad2deg; pitch = p * rad2deg;
    return true;
}
