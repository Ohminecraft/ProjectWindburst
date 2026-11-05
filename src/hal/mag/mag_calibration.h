/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>
#include <math.h>
#include <string.h>

// ============================================================================
// Magnetometer calibration: hard-iron offset + full 3x3 soft-iron matrix from an ellipsoid fit, like ArduPilot's CompassCalibrator
// (offsets, diagonal and off-diagonal terms) - Betaflight only estimates an offset. Portable, unit-tested on synthetic data with known distortion.
//
//   calibrated = M * (raw - offset)          (gauss; in the CHIP frame, before mounting alignment, so changing alignment never invalidates it)
//
// Procedure the app drives: begin() -> feed every raw sample with add() while the user rotates the craft through all orientations ->
// progress() (0..1, sphere coverage) -> solve() -> store MagCal in settings. The 32 coverage bins are 8 azimuth sectors x 4 equal-area
// elevation bands around the provisional center; ~26 of 32 must be visited, as a pure rotation about one axis is not observable.
// ============================================================================
struct MagCal {
    bool valid = false;
    float offset[3] = {0, 0, 0};
    float m[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    float radius = 0.0f;               // gauss: the field strength after calibration (should be 0.25..0.65 near the surface)
    float fit_rms_pct = 0.0f;          // residual of |calibrated| around the radius, percent
    void apply(const float raw[3], float out[3]) const {
        const float x = raw[0] - offset[0], y = raw[1] - offset[1], z = raw[2] - offset[2];
        out[0] = m[0][0] * x + m[0][1] * y + m[0][2] * z;
        out[1] = m[1][0] * x + m[1][1] * y + m[1][2] * z;
        out[2] = m[2][0] * x + m[2][1] * y + m[2][2] * z;
    }
};

enum class MagCalResult : uint8_t { OK, TOO_FEW_SAMPLES, POOR_COVERAGE, SINGULAR, NOT_ELLIPSOID, TOO_DISTORTED, BAD_FIELD_STRENGTH, BAD_FIT, NO_MOTION, OFFSET_TOO_LARGE };   // == schema enum mag_cal_result (NO_MOTION: 0.18, the automatic one)

class MagCalibrator {
public:
    static constexpr int kMaxSamples = 300, kBins = 32;
    float min_spacing = 0.03f;         // gauss between kept samples
    int min_samples = 80, min_bins = 26;
    float max_axis_ratio = 2.0f, max_rms_pct = 6.0f, min_radius = 0.15f, max_radius = 0.90f;
    float max_offset = 1.8f;           // gauss, any axis (ArduPilot COMPASS_OFFS_MAX 1800 mG): more is a magnet or steel next to the chip

    void begin() { n_ = 0; rejected_ = 0; }
    int count() const { return n_; }
    // returns true if the sample was kept
    bool add(const float raw[3]) {
        if (n_ >= kMaxSamples) return false;
        if (n_ > 0) {
            const float dx = raw[0] - s_[n_ - 1][0], dy = raw[1] - s_[n_ - 1][1], dz = raw[2] - s_[n_ - 1][2];
            if (dx * dx + dy * dy + dz * dz < min_spacing * min_spacing) return false;
        }
        if (raw[0] == 0.0f && raw[1] == 0.0f && raw[2] == 0.0f) return false;
        memcpy(s_[n_++], raw, 3 * sizeof(float));
        return true;
    }
    // 0..1: how much of the sphere has been visited (and enough samples)
    float progress() const {
        if (n_ < 12) return 0.0f;
        const float p = (float)covered_bins() / (float)min_bins, q = (float)n_ / (float)min_samples;
        const float r = p < q ? p : q;
        return r > 1.0f ? 1.0f : r;
    }
    int covered_bins() const {
        if (n_ < 12) return 0;
        float c[3];
        if (!sphere_center(c)) return 0;                              // provisional centre from a linear sphere fit (the sample mean would be biased by one-sided data)
        float sd[3] = {0, 0, 0};                                    // per-axis spread: a strongly distorted sensor must not look like 'poor coverage'
        for (int i = 0; i < n_; i++) for (int k = 0; k < 3; k++) { const float d = s_[i][k] - c[k]; sd[k] += d * d; }
        for (int k = 0; k < 3; k++) { sd[k] = sqrtf(sd[k] / (float)n_); if (sd[k] < 1e-6f) sd[k] = 1e-6f; }
        int cnt[kBins]; memset(cnt, 0, sizeof(cnt));
        for (int i = 0; i < n_; i++) {
            const float x = (s_[i][0] - c[0]) / sd[0], y = (s_[i][1] - c[1]) / sd[1], z = (s_[i][2] - c[2]) / sd[2], r = sqrtf(x * x + y * y + z * z);
            if (r < 1e-6f) continue;
            float az = atan2f(y, x); if (az < 0) az += 6.28318530718f;
            const float sz = z / r;                                  // equal-area bands in sin(elevation)
            int band = sz < -0.5f ? 0 : (sz < 0.0f ? 1 : (sz < 0.5f ? 2 : 3));
            int sec = (int)(az * (8.0f / 6.28318530718f)); if (sec > 7) sec = 7;
            cnt[band * 8 + sec]++;
        }
        int n = 0; for (int b = 0; b < kBins; b++) if (cnt[b] >= 2) n++;
        return n;
    }

    MagCalResult solve(MagCal& out) const {
        if (n_ < min_samples) return MagCalResult::TOO_FEW_SAMPLES;
        if (covered_bins() < min_bins) return MagCalResult::POOR_COVERAGE;
        // normalise: u = (x - mu) / sc
        double mu[3] = {0, 0, 0};
        for (int i = 0; i < n_; i++) for (int k = 0; k < 3; k++) mu[k] += s_[i][k];
        for (int k = 0; k < 3; k++) mu[k] /= n_;
        double sc = 0; for (int i = 0; i < n_; i++) for (int k = 0; k < 3; k++) { const double d = s_[i][k] - mu[k]; sc += d * d; }
        sc = sqrt(sc / n_); if (sc < 1e-6) return MagCalResult::SINGULAR;
        // normal equations for  [x2 y2 z2 2xy 2xz 2yz 2x 2y 2z] . p = 1
        double N[9][10]; memset(N, 0, sizeof(N));
        for (int i = 0; i < n_; i++) {
            const double x = (s_[i][0] - mu[0]) / sc, y = (s_[i][1] - mu[1]) / sc, z = (s_[i][2] - mu[2]) / sc;
            const double r[9] = {x * x, y * y, z * z, 2 * x * y, 2 * x * z, 2 * y * z, 2 * x, 2 * y, 2 * z};
            for (int a = 0; a < 9; a++) { for (int b = 0; b < 9; b++) N[a][b] += r[a] * r[b]; N[a][9] += r[a]; }
        }
        double p[9];
        if (!gauss9(N, p)) return MagCalResult::SINGULAR;
        double A[3][3] = {{p[0], p[3], p[4]}, {p[3], p[1], p[5]}, {p[4], p[5], p[2]}}, bvec[3] = {p[6], p[7], p[8]};
        double Ai[3][3];
        if (!inv3(A, Ai)) return MagCalResult::SINGULAR;
        double cu[3]; for (int i = 0; i < 3; i++) cu[i] = -(Ai[i][0] * bvec[0] + Ai[i][1] * bvec[1] + Ai[i][2] * bvec[2]);
        double kk = 1.0 + bvec[0] * (Ai[0][0] * bvec[0] + Ai[0][1] * bvec[1] + Ai[0][2] * bvec[2]) + bvec[1] * (Ai[1][0] * bvec[0] + Ai[1][1] * bvec[1] + Ai[1][2] * bvec[2]) + bvec[2] * (Ai[2][0] * bvec[0] + Ai[2][1] * bvec[1] + Ai[2][2] * bvec[2]);
        if (!(kk > 1e-9)) return MagCalResult::NOT_ELLIPSOID;
        double Q[3][3]; for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) Q[i][j] = A[i][j] / kk / (sc * sc);     // (x-c)' Q (x-c) = 1 in original units
        double c[3]; for (int k = 0; k < 3; k++) c[k] = mu[k] + sc * cu[k];
        double lam[3], V[3][3];
        jacobi3(Q, lam, V);
        if (!(lam[0] > 0 && lam[1] > 0 && lam[2] > 0)) return MagCalResult::NOT_ELLIPSOID;
        const double lmin = fmin(lam[0], fmin(lam[1], lam[2])), lmax = fmax(lam[0], fmax(lam[1], lam[2]));
        if (sqrt(lmax / lmin) > max_axis_ratio) return MagCalResult::TOO_DISTORTED;      // longest/shortest semi-axis
        const double R = pow(lam[0] * lam[1] * lam[2], -1.0 / 6.0);                       // geometric-mean semi-axis = field strength
        if (R < min_radius || R > max_radius) return MagCalResult::BAD_FIELD_STRENGTH;
        // S = R * Q^(1/2)
        double S[3][3];
        for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) { double v = 0; for (int k = 0; k < 3; k++) v += V[i][k] * sqrt(lam[k]) * V[j][k]; S[i][j] = R * v; }
        // residual
        double ss = 0;
        for (int i = 0; i < n_; i++) {
            const double x = s_[i][0] - c[0], y = s_[i][1] - c[1], z = s_[i][2] - c[2];
            double v[3]; for (int a = 0; a < 3; a++) v[a] = S[a][0] * x + S[a][1] * y + S[a][2] * z;
            const double e = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) - R; ss += e * e;
        }
        const double rms = 100.0 * sqrt(ss / n_) / R;
        if (rms > max_rms_pct) return MagCalResult::BAD_FIT;
        if (fabs(c[0]) > max_offset || fabs(c[1]) > max_offset || fabs(c[2]) > max_offset) return MagCalResult::OFFSET_TOO_LARGE;
        out.valid = true; out.radius = (float)R; out.fit_rms_pct = (float)rms;
        for (int k = 0; k < 3; k++) out.offset[k] = (float)c[k];
        for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) out.m[i][j] = (float)S[i][j];
        return MagCalResult::OK;
    }
    static const char* result_name(MagCalResult r) {
        static const char* n[] = {"ok", "too few samples", "turn the craft through more orientations", "singular data", "data is not an ellipsoid", "sensor too distorted (metal / magnets nearby?)", "field strength implausible", "poor fit", "the craft was not turned", "offset too large (a magnet or steel next to the compass?)"};
        return n[(int)r];
    }

private:
    // linear least-squares sphere fit: |x|^2 = 2 c.x + d  ->  centre c. false if singular.
    bool sphere_center(float c[3]) const {
        double N[4][5]; memset(N, 0, sizeof(N));
        for (int i = 0; i < n_; i++) {
            const double x = s_[i][0], y = s_[i][1], z = s_[i][2], r[4] = {2 * x, 2 * y, 2 * z, 1.0}, t = x * x + y * y + z * z;
            for (int a = 0; a < 4; a++) { for (int b = 0; b < 4; b++) N[a][b] += r[a] * r[b]; N[a][4] += r[a] * t; }
        }
        for (int col = 0; col < 4; col++) {
            int piv = col; for (int r = col + 1; r < 4; r++) if (fabs(N[r][col]) > fabs(N[piv][col])) piv = r;
            if (fabs(N[piv][col]) < 1e-12) return false;
            if (piv != col) for (int k = 0; k < 5; k++) { const double t = N[col][k]; N[col][k] = N[piv][k]; N[piv][k] = t; }
            for (int r = 0; r < 4; r++) if (r != col) { const double f = N[r][col] / N[col][col]; for (int k = col; k < 5; k++) N[r][k] -= f * N[col][k]; }
        }
        for (int k = 0; k < 3; k++) c[k] = (float)(N[k][4] / N[k][k]);
        return true;
    }
    static bool gauss9(double N[9][10], double p[9]) {
        for (int c = 0; c < 9; c++) {
            int piv = c; for (int r = c + 1; r < 9; r++) if (fabs(N[r][c]) > fabs(N[piv][c])) piv = r;
            if (fabs(N[piv][c]) < 1e-12) return false;
            if (piv != c) for (int k = 0; k < 10; k++) { const double t = N[c][k]; N[c][k] = N[piv][k]; N[piv][k] = t; }
            for (int r = 0; r < 9; r++) if (r != c) { const double f = N[r][c] / N[c][c]; for (int k = c; k < 10; k++) N[r][k] -= f * N[c][k]; }
        }
        for (int i = 0; i < 9; i++) p[i] = N[i][9] / N[i][i];
        return true;
    }
    static bool inv3(const double a[3][3], double o[3][3]) {
        const double det = a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
        if (fabs(det) < 1e-14) return false;
        o[0][0] = (a[1][1] * a[2][2] - a[1][2] * a[2][1]) / det; o[0][1] = (a[0][2] * a[2][1] - a[0][1] * a[2][2]) / det; o[0][2] = (a[0][1] * a[1][2] - a[0][2] * a[1][1]) / det;
        o[1][0] = (a[1][2] * a[2][0] - a[1][0] * a[2][2]) / det; o[1][1] = (a[0][0] * a[2][2] - a[0][2] * a[2][0]) / det; o[1][2] = (a[0][2] * a[1][0] - a[0][0] * a[1][2]) / det;
        o[2][0] = (a[1][0] * a[2][1] - a[1][1] * a[2][0]) / det; o[2][1] = (a[0][1] * a[2][0] - a[0][0] * a[2][1]) / det; o[2][2] = (a[0][0] * a[1][1] - a[0][1] * a[1][0]) / det;
        return true;
    }
    // Jacobi eigen-decomposition of a symmetric 3x3: a = V diag(w) V'
    static void jacobi3(const double a_in[3][3], double w[3], double V[3][3]) {
        double a[3][3]; memcpy(a, a_in, sizeof(a));
        for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) V[i][j] = (i == j) ? 1.0 : 0.0;
        for (int sweep = 0; sweep < 50; sweep++) {
            double off = fabs(a[0][1]) + fabs(a[0][2]) + fabs(a[1][2]);
            if (off < 1e-15) break;
            for (int p = 0; p < 2; p++) for (int q = p + 1; q < 3; q++) {
                if (fabs(a[p][q]) < 1e-18) continue;
                const double th = (a[q][q] - a[p][p]) / (2 * a[p][q]);
                const double t = (th >= 0 ? 1.0 : -1.0) / (fabs(th) + sqrt(th * th + 1));
                const double c = 1 / sqrt(t * t + 1), s = t * c;
                for (int k = 0; k < 3; k++) { const double akp = a[k][p], akq = a[k][q]; a[k][p] = c * akp - s * akq; a[k][q] = s * akp + c * akq; }
                for (int k = 0; k < 3; k++) { const double apk = a[p][k], aqk = a[q][k]; a[p][k] = c * apk - s * aqk; a[q][k] = s * apk + c * aqk; }
                for (int k = 0; k < 3; k++) { const double vkp = V[k][p], vkq = V[k][q]; V[k][p] = c * vkp - s * vkq; V[k][q] = s * vkp + c * vkq; }
            }
        }
        for (int i = 0; i < 3; i++) w[i] = a[i][i];
    }
    float s_[kMaxSamples][3];
    int n_ = 0; int rejected_ = 0;
};
