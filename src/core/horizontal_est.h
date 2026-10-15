#pragma once
#include "../config/fast_code.h"
#include "types.h"

// Horizontal (north/east) position + velocity estimator: one 3-state Kalman filter per axis, x = [position, velocity, accel bias].
//   predict : earth-frame horizontal acceleration from Ahrs::world_accel()
//   GPS     : position and velocity measurements, with the receiver's latency compensated (measurement is `delay` seconds old:
//             it is moved forward with the current velocity / acceleration estimate before the update)
//   flow    : a velocity over the ground (optical flow x height, flow.h), fused as a velocity measurement - which also keeps the
//             position from running away, but only relative to where the craft was: without GPS the position has no anchor
//   position is relative to the home point (metres, north/east).
// A filter that got no measurement for a while must not be believed afterwards (Betaflight met "several m/s of phantom velocity"
// after a flow dropout): the variance grows while only the prediction runs, so the first measurement after the gap counts in
// full (gain close to 1) and passes the gate however far the prediction has run off.
namespace fc {
struct HorizontalConfig {
    float q_accel = 0.8f;       // accel noise (m/s^2 per sqrt(s)) - attitude error leaks gravity into the horizontal axes
    float q_bias = 0.01f;
    float gps_delay_s = 0.15f;  // typical u-blox latency at 10 Hz
    float bias_limit = 1.0f;
    float gate_sigma = 6.0f;
};

class HorizontalEstimator {
public:
    HorizontalConfig cfg;
    void reset(float pn = 0, float pe = 0, float vn = 0, float ve = 0) {
        for (int a = 0; a < 2; a++) { ax_[a] = Axis(); ax_[a].x[0] = a == 0 ? pn : pe; ax_[a].x[1] = a == 0 ? vn : ve; }
        have_ = false; last_an_ = last_ae_ = 0;
    }
    WB_FAST void predict(float a_north, float a_east, float dt) {
        if (dt <= 0 || dt > 0.1f) return;
        ax_[0].predict(a_north, dt, cfg); ax_[1].predict(a_east, dt, cfg);
        last_an_ = a_north; last_ae_ = a_east;
    }
    // GPS solution in metres relative to home. sigma_pos/sigma_vel from the receiver (hAcc / sAcc), floored to sane values.
    WB_FAST void fuse_gps(float pn, float pe, float vn, float ve, float sigma_pos, float sigma_vel) {
        const float tau = cfg.gps_delay_s;
        const float sp = maxf(sigma_pos, 0.5f), sv = maxf(sigma_vel, 0.1f);
        const float pm[2] = {pn, pe}, vm[2] = {vn, ve}, acc[2] = {last_an_, last_ae_};
        if (!have_) { for (int a = 0; a < 2; a++) { ax_[a].x[0] = pm[a]; ax_[a].x[1] = vm[a]; } have_ = true; }
        for (int a = 0; a < 2; a++) {
            const float v_now = vm[a] + (acc[a] - ax_[a].x[2]) * tau;                 // the velocity measured tau ago, moved to now
            const float p_now = pm[a] + vm[a] * tau + 0.5f * acc[a] * tau * tau;
            ax_[a].update(0, p_now, sp, cfg);
            ax_[a].update(1, v_now, sv, cfg);
        }
    }
    // A velocity over the ground (m/s) that was true `delay_s` ago; sigma as flow.h works it out (never below 0.1 m/s).
    WB_FAST void fuse_velocity(float vn, float ve, float sigma_vel, float delay_s) {
        const float sv = sigma_vel, vm[2] = {vn, ve}, acc[2] = {last_an_, last_ae_};
        if (!have_) { for (int a = 0; a < 2; a++) ax_[a].x[1] = vm[a]; have_ = true; }
        for (int a = 0; a < 2; a++) ax_[a].update(1, vm[a] + (acc[a] - ax_[a].x[2]) * delay_s, sv, cfg);
    }
    float pos_n() const { return ax_[0].x[0]; } float pos_e() const { return ax_[1].x[0]; }
    float vel_n() const { return ax_[0].x[1]; } float vel_e() const { return ax_[1].x[1]; }
    float speed() const { return sqrtf(vel_n() * vel_n() + vel_e() * vel_e()); }
    float pos_sigma() const { return sqrtf(maxf(0.5f * (ax_[0].P[0][0] + ax_[1].P[0][0]), 0.0f)); }
    float vel_sigma() const { return sqrtf(maxf(0.5f * (ax_[0].P[1][1] + ax_[1].P[1][1]), 0.0f)); }
    float bias_n() const { return ax_[0].x[2]; } float bias_e() const { return ax_[1].x[2]; }
    bool initialised() const { return have_; }
private:
    struct Axis {
        float x[3] = {0, 0, 0}, P[3][3] = {{25, 0, 0}, {0, 4, 0}, {0, 0, 0.1f}};
        WB_FAST void predict(float a, float dt, const HorizontalConfig& c) {
            x[0] += x[1] * dt + 0.5f * (a - x[2]) * dt * dt; x[1] += (a - x[2]) * dt;
            const float F[3][3] = {{1, dt, -0.5f * dt * dt}, {0, 1, -dt}, {0, 0, 1}};
            float FP[3][3], N[3][3];
            for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) { float s = 0; for (int k = 0; k < 3; k++) s += F[i][k] * P[k][j]; FP[i][j] = s; }
            for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) { float s = 0; for (int k = 0; k < 3; k++) s += FP[i][k] * F[j][k]; N[i][j] = s; }
            const float qa = c.q_accel * c.q_accel;
            N[0][0] += qa * dt * dt * dt / 3.0f; N[0][1] += qa * dt * dt * 0.5f; N[1][0] += qa * dt * dt * 0.5f; N[1][1] += qa * dt; N[2][2] += c.q_bias * c.q_bias * dt;
            for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) P[i][j] = N[i][j];
            x[2] = clampf(x[2], -c.bias_limit, c.bias_limit);
        }
        WB_FAST bool update(int idx, float meas, float sigma, const HorizontalConfig& c) {      // H = e_idx
            const float y = meas - x[idx], S = P[idx][idx] + sigma * sigma;
            if (S <= 1e-9f || y * y > c.gate_sigma * c.gate_sigma * S) return false;
            float K[3]; for (int i = 0; i < 3; i++) K[i] = P[i][idx] / S;
            for (int i = 0; i < 3; i++) x[i] += K[i] * y;
            float HP[3]; for (int j = 0; j < 3; j++) HP[j] = P[idx][j];
            for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) P[i][j] -= K[i] * HP[j];
            for (int i = 0; i < 3; i++) for (int j = i + 1; j < 3; j++) { const float m = 0.5f * (P[i][j] + P[j][i]); P[i][j] = P[j][i] = m; }
            for (int i = 0; i < 3; i++) if (P[i][i] < 1e-9f) P[i][i] = 1e-9f;
            x[2] = clampf(x[2], -c.bias_limit, c.bias_limit);
            return true;
        }
    } ax_[2];
    bool have_ = false; float last_an_ = 0, last_ae_ = 0;
};
} // namespace fc
