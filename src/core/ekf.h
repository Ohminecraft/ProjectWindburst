#pragma once
#include <math.h>
#include <stdint.h>
#include <string.h>
#include "types.h"

// ============================================================================
// Error-state extended Kalman filter: attitude, velocity, position and the IMU's biases in ONE filter (0.9.0).
//
// Why another estimator. The rest of the firmware estimates in separate pieces: the attitude (ahrs.h, a Mahony filter), then the
// position and velocity with that attitude taken as true (nav/pos_estimator.h / nav/alt_estimator.h, linear Kalman filters). A heading
// that is wrong - a compass disturbed by the motor current, as in the log of 2026-10-11 - is invisible to the position filter: it
// rotates the measured acceleration the wrong way and blames the GPS. Here the attitude is a state of the same filter as the
// velocity, and the correlation between them is kept: when the GPS velocity disagrees with what the accelerometer integrated
// through the estimated attitude, the attitude (the heading too) is corrected, not only the velocity. That is what ArduPilot's
// EKF3 and PX4's EKF2 do; this one is much smaller (15 states, no wind, no magnetic field states).
//
// "Extended": the rotation of the accelerometer into the earth frame is not linear in the attitude, so the model is linearised
// around the current estimate at every step. "Error state": the filter estimates the small ERROR of a nominal state that is
// propagated with the full non-linear equations (the quaternion stays a quaternion); after every correction the error is folded
// into the nominal state and set back to zero (Sola, "Quaternion kinematics for the error-state Kalman filter", 2017, global
// angular error). This is the form PX4 and most visual-inertial systems use.
//
//   nominal  q (body -> earth, NED), v (m/s NED), p (m NED from home / the start), bg (gyro bias rad/s), ba (accel bias m/s^2, body)
//   error    dx = [ dth(3) earth-frame rotation, dv(3), dp(3), dbg(3), dba(3) ]
//   predict  from the IMU's delta angle and delta velocity (body FRD) over dt:
//              q <- q * exp(dang - bg dt),  v <- v + R (dvel - ba dt) + g dt,  p <- p + v dt
//              dth' = -R dbg,  dv' = -[R f]x dth - R dba,  dp' = dv
//   measure  GPS position N/E and velocity N/E (/D), each compared with the state of gps_delay_s ago (a history of the nominal
//            state is kept); barometer height; compass HEADING only (a disturbed field can never tilt the craft - as ahrs.h does);
//            with nothing else, a position that stays put (sigma 10 m, ArduPilot's EK3_NOAID_M_NSE) so the tilt cannot run away;
//            standing (disarmed, still): zero velocity.
// Every scalar measurement is gated (innovation larger than gate x its sigma: not used). A source rejected for a long time resets
// the state to it (position / height), as ArduPilot does.
// Portable, float, ~30 us per step on an ESP32-S3 (the 15 x 15 covariance); meant for the sensor thread at 200 - 250 Hz.
// ============================================================================
namespace fc {

struct EkfConfig {
    // process noise (densities: per sqrt(s))
    float gyro_noise = 0.01f;          // rad/s/sqrt(Hz) incl. vibration
    float accel_noise = 0.35f;         // m/s^2/sqrt(Hz) incl. vibration and the drag the thrust model leaves out (ArduPilot EK3_ACC_P_NSE)
    float gyro_bias_walk = 2e-4f;      // rad/s/sqrt(s)
    float accel_bias_walk = 2e-3f;     // m/s^2/sqrt(s), the z axis
    float accel_bias_walk_xy = 1e-4f;  // ... x / y: on a multirotor these are hardly observable (the thrust is along z), so they are kept
                                       // on a short leash (as PX4 EKF2 does). The PC tests show no difference either way (test_ekf passes
                                       // with them free): a precaution for the board, not something a test proves
    float bias_limit_gyro = 0.1f;      // rad/s (6 deg/s)
    float bias_limit_accel = 1.5f;     // m/s^2
    // measurements
    float gps_pos_sigma_min = 0.5f, gps_vel_sigma_min = 0.25f;   // floors on what the receiver says (m, m/s)
    float gps_delay_s = 0.15f;         // the GPS solution is this old when it arrives
    float baro_sigma = 1.0f;           // m
    float mag_sigma = 0.5f;            // rad (29 deg): the compass's error is not white noise - near motors it is a slowly changing offset -
    float mag_interval_s = 0.1f;       // ... so it is fused at 10 Hz at most and believed loosely (ArduPilot EK3_YAW_M_NSE is 0.5 rad).
                                       // A heading the GPS velocity shows (every acceleration) then outweighs it; in a steady hover the
                                       // compass is all there is
    float noaid_sigma = 10.0f;         // m: "stays where it is" without GPS / flow
    float still_sigma = 0.1f;          // m/s: standing
    float gate_gps = 5.0f, gate_baro = 5.0f, gate_mag = 3.0f;    // sigmas
    float reset_after_s = 5.0f;        // GPS position / baro rejected this long: the state is set to it
};

struct EkfStatus {
    bool aligned = false;              // initialised (tilt from the accelerometer, heading from the compass or set)
    bool gps_aiding = false;           // GPS fused within the last second
    bool heading_ok = false;           // the heading uncertainty is below 10 deg
    uint32_t gps_rejects = 0, mag_rejects = 0, baro_rejects = 0, resets = 0;
    float test_vel = 0, test_pos = 0, test_mag = 0, test_baro = 0;   // last innovation / sigma (for the log)
};

class Ekf {
    struct Hist { float t = 0, v[3] = {}, p[3] = {}; };
public:
    static constexpr int N = 15;
    EkfConfig cfg;

    // Initial alignment: tilt from a resting accelerometer (specific force, g, FRD), heading in rad (from the compass, or 0 with a
    // large uncertainty when there is none: yaw_known = false).
    void init(Vec3 acc_g, float yaw_rad, bool yaw_known) {
        const float roll = atan2f(-acc_g.y, -acc_g.z), pitch = atan2f(acc_g.x, sqrtf(acc_g.y * acc_g.y + acc_g.z * acc_g.z));
        from_euler(roll, pitch, yaw_rad);
        for (int i = 0; i < 3; i++) { v_[i] = p_[i] = bg_[i] = ba_[i] = 0.0f; }
        memset(P_, 0, sizeof(P_));
        const float att = 0.05f * 0.05f, yaw = yaw_known ? 0.2f * 0.2f : 3.0f * 3.0f;
        P_[0][0] = att; P_[1][1] = att; P_[2][2] = yaw;
        for (int i = 3; i < 6; i++) P_[i][i] = 0.5f * 0.5f;
        for (int i = 6; i < 9; i++) P_[i][i] = 2.0f * 2.0f;
        for (int i = 9; i < 12; i++) P_[i][i] = 0.01f * 0.01f;
        P_[12][12] = P_[13][13] = 0.05f * 0.05f; P_[14][14] = 0.3f * 0.3f;
        st_ = EkfStatus(); st_.aligned = true; hist_n_ = 0; hist_i_ = 0; t_ = 0.0f; last_mag_t_ = -1e9f;
        last_gps_t_ = last_aid_t_ = -1e9f; last_gps_pos_ok_t_ = last_baro_ok_t_ = 0.0f; noaid_n_ = p_[0]; noaid_e_ = p_[1];
    }
    bool aligned() const { return st_.aligned; }

    // One IMU interval: dang = integral of the gyro (rad, body), dvel = integral of the specific force (m/s, body), over dt.
    void predict(Vec3 dang, Vec3 dvel, float dt) {
        if (!st_.aligned || !(dt > 0.0f) || dt > 0.1f) return;
        t_ += dt;
        // ---- nominal state
        const float ax = dang.x - bg_[0] * dt, ay = dang.y - bg_[1] * dt, az = dang.z - bg_[2] * dt;
        const float fx = dvel.x - ba_[0] * dt, fy = dvel.y - ba_[1] * dt, fz = dvel.z - ba_[2] * dt;   // (m/s over dt)
        float R[3][3]; rot(R);
        const float dvn = R[0][0] * fx + R[0][1] * fy + R[0][2] * fz, dve = R[1][0] * fx + R[1][1] * fy + R[1][2] * fz, dvd = R[2][0] * fx + R[2][1] * fy + R[2][2] * fz;
        const float vn0 = v_[0], ve0 = v_[1], vd0 = v_[2];
        v_[0] += dvn; v_[1] += dve; v_[2] += dvd + kG0 * dt;
        p_[0] += 0.5f * (vn0 + v_[0]) * dt; p_[1] += 0.5f * (ve0 + v_[1]) * dt; p_[2] += 0.5f * (vd0 + v_[2]) * dt;
        rotate_body(ax, ay, az);
        // ---- covariance: P = F P F' + Q,  F = I + A dt
        //   A[th][bg] = -R,  A[v][th] = -[a_e]x (a_e = R f, the earth-frame specific force),  A[v][ba] = -R,  A[p][v] = I
        const float an = dvn / dt, ae = dve / dt, ad = dvd / dt;
        float F[N][N]; memset(F, 0, sizeof(F));
        for (int i = 0; i < N; i++) F[i][i] = 1.0f;
        for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) { F[i][9 + j] = -R[i][j] * dt; F[3 + i][12 + j] = -R[i][j] * dt; }
        // -[a]x = [[0, ad, -ae], [-ad, 0, an], [ae, -an, 0]]
        // (rows: velocity 3..5, columns: the attitude error 0..2)
        F[3][1] = ad * dt; F[3][2] = -ae * dt; F[4][0] = -ad * dt; F[4][2] = an * dt; F[5][0] = ae * dt; F[5][1] = -an * dt;
        for (int i = 0; i < 3; i++) F[6 + i][3 + i] = dt;
        // F has ~40 non-zeros of 225: each row's are listed once, both products run over those only (~1000 multiply-adds, not 6750)
        uint8_t nzc[N][8]; float nzv[N][8]; uint8_t nzn[N];
        for (int i = 0; i < N; i++) { nzn[i] = 0; for (int k = 0; k < N; k++) if (F[i][k] != 0.0f && nzn[i] < 8) { nzc[i][nzn[i]] = (uint8_t)k; nzv[i][nzn[i]] = F[i][k]; nzn[i]++; } }
        float FP[N][N];
        for (int i = 0; i < N; i++) for (int j = 0; j < N; j++) { float s = 0; for (int q = 0; q < nzn[i]; q++) s += nzv[i][q] * P_[nzc[i][q]][j]; FP[i][j] = s; }
        for (int i = 0; i < N; i++) for (int j = i; j < N; j++) { float s = 0; for (int q = 0; q < nzn[j]; q++) s += FP[i][nzc[j][q]] * nzv[j][q]; P_[i][j] = P_[j][i] = s; }
        const float qg = cfg.gyro_noise * cfg.gyro_noise * dt, qa = cfg.accel_noise * cfg.accel_noise * dt;
        const float qbg = cfg.gyro_bias_walk * cfg.gyro_bias_walk * dt, qba = cfg.accel_bias_walk * cfg.accel_bias_walk * dt, qbaxy = cfg.accel_bias_walk_xy * cfg.accel_bias_walk_xy * dt;
        for (int i = 0; i < 3; i++) { P_[i][i] += qg; P_[3 + i][3 + i] += qa; P_[9 + i][9 + i] += qbg; P_[12 + i][12 + i] += i < 2 ? qbaxy : qba; }
        condition();
        // ---- history for the delayed GPS
        Hist& h = hist_[hist_i_]; h.t = t_; for (int i = 0; i < 3; i++) { h.v[i] = v_[i]; h.p[i] = p_[i]; }
        hist_i_ = (hist_i_ + 1) % kHist; if (hist_n_ < kHist) hist_n_++;
        // ---- nothing to tell where the craft is: it stays where it was (weakly) - the tilt is kept from wandering off
        if (t_ - last_aid_t_ > 1.0f && t_ - noaid_t_ >= 0.2f) {
            noaid_t_ = t_;
            fuse(6, noaid_n_, cfg.noaid_sigma, 0.0f, nullptr);
            fuse(7, noaid_e_, cfg.noaid_sigma, 0.0f, nullptr);
        }
    }

    // GPS (m from home NED, m/s NED; sigmas from the receiver, <= 0: the floors). vd_valid: the receiver measures a vertical
    // velocity (u-blox; NMEA does not).
    void fuse_gps(float pn, float pe, float vn, float ve, float vd, bool vd_valid, float sigma_pos, float sigma_vel) {
        if (!st_.aligned) return;
        Hist d; delayed(cfg.gps_delay_s, d);
        const float sp = fmaxf(sigma_pos, cfg.gps_pos_sigma_min), sv = fmaxf(sigma_vel, cfg.gps_vel_sigma_min);
        if (!(last_gps_t_ > -1e8f)) { set_position(pn, pe); d.p[0] = pn; d.p[1] = pe; }   // the first fix: the position starts there
        const bool vel_ok = fuse_gps_vel_(d, vn, ve, vd, vd_valid, sv);
        float tp = 0;
        // (the innovation is taken against the delayed state; the correction is applied to the present one)
        bool pos_ok = fuse(6, pn - d.p[0] + p_[0], sp, cfg.gate_gps, &tp);
        pos_ok &= fuse(7, pe - d.p[1] + p_[1], sp, cfg.gate_gps, &tp);
        st_.test_pos = tp;
        if (pos_ok) last_gps_pos_ok_t_ = t_;
        else if (t_ - last_gps_pos_ok_t_ > cfg.reset_after_s) {          // rejected for long: the GPS is believed, the state jumps to it
            p_[0] = pn; p_[1] = pe; v_[0] = vn; v_[1] = ve;
            for (int i = 3; i < 9; i++) { for (int j = 0; j < N; j++) { P_[i][j] = P_[j][i] = 0.0f; } }
            P_[3][3] = P_[4][4] = sv * sv; P_[5][5] = 1.0f; P_[6][6] = P_[7][7] = sp * sp; P_[8][8] = 4.0f;
            last_gps_pos_ok_t_ = t_; st_.resets++;
        }
        if (!vel_ok || !pos_ok) st_.gps_rejects++;
        noaid_n_ = p_[0]; noaid_e_ = p_[1];
    }
    // The GPS velocity alone (a position that cannot be used: a log whose coordinates were rounded, a receiver without one).
    void fuse_gps_vel(float vn, float ve, float vd, bool vd_valid, float sigma_vel) {
        if (!st_.aligned) return;
        Hist d; delayed(cfg.gps_delay_s, d);
        if (!fuse_gps_vel_(d, vn, ve, vd, vd_valid, fmaxf(sigma_vel, cfg.gps_vel_sigma_min))) st_.gps_rejects++;
        noaid_n_ = p_[0]; noaid_e_ = p_[1];
    }
    // Barometric height above the start (m, up positive).
    void fuse_baro(float h_up) {
        if (!st_.aligned) return;
        float t = 0;
        if (fuse(8, -h_up, cfg.baro_sigma, cfg.gate_baro, &t)) last_baro_ok_t_ = t_;
        else {
            st_.baro_rejects++;
            if (t_ - last_baro_ok_t_ > cfg.reset_after_s) { p_[2] = -h_up; v_[2] = 0.0f; for (int j = 0; j < N; j++) { P_[8][j] = P_[j][8] = P_[5][j] = P_[j][5] = 0.0f; } P_[8][8] = cfg.baro_sigma * cfg.baro_sigma; P_[5][5] = 1.0f; last_baro_ok_t_ = t_; st_.resets++; }
        }
        st_.test_baro = t;
    }
    // Compass: a calibrated body-frame field (any unit). Only the HEADING it gives is used (declination east positive, deg).
    bool fuse_mag(Vec3 m, float declination_deg) {
        if (!st_.aligned || t_ - last_mag_t_ < cfg.mag_interval_s) return false;
        last_mag_t_ = t_;
        float R[3][3]; rot(R);
        const float mn = R[0][0] * m.x + R[0][1] * m.y + R[0][2] * m.z, me = R[1][0] * m.x + R[1][1] * m.y + R[1][2] * m.z;
        const float nrm = sqrtf(m.x * m.x + m.y * m.y + m.z * m.z), hh = sqrtf(mn * mn + me * me);
        if (nrm < 1e-6f || hh < 0.2f * nrm) return false;
        // The field, rotated into the earth frame with the estimated attitude, must point to magnetic north: its angle phi from true
        // north is the declination. phi depends on the whole attitude error (R' = (I + [dth]x) R  ->  m' = m + dth x m), not only on
        // the yaw: d(phi)/d(dth) = [-mn md, -me md, mn^2 + me^2] / (mn^2 + me^2). (The Jacobian of the Euler yaw would be the one of
        // the body's x axis - the same only when level; with it a lean of 25 deg made the compass tilt the estimate.)
        const float md = R[2][0] * m.x + R[2][1] * m.y + R[2][2] * m.z, h2 = mn * mn + me * me;
        float H[N] = {}; H[0] = -mn * md / h2; H[1] = -me * md / h2; H[2] = 1.0f;
        float y = declination_deg * kDeg2Rad - atan2f(me, mn); while (y > kPi) y -= 2 * kPi; while (y < -kPi) y += 2 * kPi;
        float t = 0; const bool ok = fuse_h(H, y, cfg.mag_sigma, cfg.gate_mag, &t);
        st_.test_mag = t; if (!ok) st_.mag_rejects++;
        return ok;
    }
    // A heading from anywhere else (dual GPS, a yaw set on the ground), rad.
    bool fuse_yaw(float yaw_meas, float sigma, float gate) {
        float R[3][3]; rot(R);
        // d(yaw) / d(dth): yaw = atan2(R10, R00); with R' = (I + [dth]x) R
        const float h2 = R[0][0] * R[0][0] + R[1][0] * R[1][0];
        if (h2 < 1e-6f) return false;
        float H[N] = {}; H[0] = -R[0][0] * R[2][0] / h2; H[1] = -R[1][0] * R[2][0] / h2; H[2] = 1.0f;
        float y = yaw_meas - yaw(); while (y > kPi) y -= 2 * kPi; while (y < -kPi) y += 2 * kPi;
        return fuse_h(H, y, sigma, gate, nullptr);
    }
    // Standing still (disarmed, not moved): the velocity is 0.
    void fuse_still() { if (!st_.aligned) return; for (int i = 0; i < 3; i++) fuse(3 + i, 0.0f, cfg.still_sigma, 0.0f, nullptr); }

    // ---- outputs
    void euler_rad(float& roll, float& pitch, float& yaw_out) const {
        roll = atan2f(2 * (q_[0] * q_[1] + q_[2] * q_[3]), 1 - 2 * (q_[1] * q_[1] + q_[2] * q_[2]));
        pitch = asinf(clampf(2 * (q_[0] * q_[2] - q_[3] * q_[1]), -1.0f, 1.0f));
        yaw_out = yaw();
    }
    float yaw() const { return atan2f(2 * (q_[0] * q_[3] + q_[1] * q_[2]), 1 - 2 * (q_[2] * q_[2] + q_[3] * q_[3])); }
    const float* q() const { return q_; }
    const float* vel() const { return v_; }
    const float* pos() const { return p_; }
    const float* gyro_bias() const { return bg_; }
    const float* accel_bias() const { return ba_; }
    float sigma(int i) const { return sqrtf(fmaxf(P_[i][i], 0.0f)); }
    float yaw_sigma() const { return sigma(2); }
    const EkfStatus& status() {
        st_.gps_aiding = t_ - last_gps_t_ < 1.0f; st_.heading_ok = st_.aligned && yaw_sigma() < 10.0f * kDeg2Rad;
        return st_;
    }
    float time() const { return t_; }
    // shift the position reference (a home point set later: the GPS position jumps by this much)
    void set_position(float pn, float pe) { p_[0] = pn; p_[1] = pe; noaid_n_ = pn; noaid_e_ = pe; for (int i = 0; i < kHist; i++) { hist_[i].p[0] = pn; hist_[i].p[1] = pe; } }

private:
    bool fuse_gps_vel_(const Hist& d, float vn, float ve, float vd, bool vd_valid, float sv) {
        last_gps_t_ = t_; last_aid_t_ = t_;
        bool ok = true; float tv = 0;
        ok &= fuse(3, vn - d.v[0] + v_[0], sv, cfg.gate_gps, &tv);
        ok &= fuse(4, ve - d.v[1] + v_[1], sv, cfg.gate_gps, &tv);
        if (vd_valid) ok &= fuse(5, vd - d.v[2] + v_[2], sv * 1.5f, cfg.gate_gps, nullptr);
        st_.test_vel = tv;
        return ok;
    }
    float q_[4] = {1, 0, 0, 0}, v_[3] = {}, p_[3] = {}, bg_[3] = {}, ba_[3] = {};
    float P_[N][N] = {};
    EkfStatus st_; float t_ = 0.0f, last_gps_t_ = -1e9f, last_aid_t_ = -1e9f, last_gps_pos_ok_t_ = 0.0f, last_baro_ok_t_ = 0.0f, noaid_t_ = 0.0f, noaid_n_ = 0.0f, noaid_e_ = 0.0f, last_mag_t_ = -1e9f;
    static constexpr int kHist = 128;                  // 0.5 s at 250 Hz
    Hist hist_[kHist]; int hist_n_ = 0, hist_i_ = 0;

    void delayed(float age, Hist& out) const {
        out.t = t_; for (int i = 0; i < 3; i++) { out.v[i] = v_[i]; out.p[i] = p_[i]; }
        if (age <= 0.0f || hist_n_ == 0) return;
        const float want = t_ - age;
        for (int k = 1; k <= hist_n_; k++) {
            const Hist& h = hist_[(hist_i_ - k + kHist) % kHist];
            if (h.t <= want) { out = h; return; }
        }
        out = hist_[(hist_i_ - hist_n_ + kHist) % kHist];  // (older than the history: the oldest there is)
    }
    void rot(float R[3][3]) const {
        const float w = q_[0], x = q_[1], y = q_[2], z = q_[3];
        R[0][0] = 1 - 2 * (y * y + z * z); R[0][1] = 2 * (x * y - w * z); R[0][2] = 2 * (x * z + w * y);
        R[1][0] = 2 * (x * y + w * z); R[1][1] = 1 - 2 * (x * x + z * z); R[1][2] = 2 * (y * z - w * x);
        R[2][0] = 2 * (x * z - w * y); R[2][1] = 2 * (y * z + w * x); R[2][2] = 1 - 2 * (x * x + y * y);
    }
    void from_euler(float r, float p, float y) {
        const float cr = cosf(r / 2), sr = sinf(r / 2), cp = cosf(p / 2), sp = sinf(p / 2), cy = cosf(y / 2), sy = sinf(y / 2);
        q_[0] = cr * cp * cy + sr * sp * sy; q_[1] = sr * cp * cy - cr * sp * sy; q_[2] = cr * sp * cy + sr * cp * sy; q_[3] = cr * cp * sy - sr * sp * cy;
    }
    static void qmul(const float a[4], const float b[4], float o[4]) {
        o[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
        o[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
        o[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
        o[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
    }
    static void qexp(float x, float y, float z, float o[4]) {
        const float a = sqrtf(x * x + y * y + z * z);
        if (a < 1e-8f) { o[0] = 1; o[1] = 0.5f * x; o[2] = 0.5f * y; o[3] = 0.5f * z; return; }
        const float s = sinf(0.5f * a) / a; o[0] = cosf(0.5f * a); o[1] = x * s; o[2] = y * s; o[3] = z * s;
    }
    void normalise() { const float n = 1.0f / sqrtf(q_[0] * q_[0] + q_[1] * q_[1] + q_[2] * q_[2] + q_[3] * q_[3]); for (float& c : q_) c *= n; }
    void rotate_body(float x, float y, float z) { float d[4], o[4]; qexp(x, y, z, d); qmul(q_, d, o); memcpy(q_, o, sizeof(o)); normalise(); }
    void rotate_earth(float x, float y, float z) { float d[4], o[4]; qexp(x, y, z, d); qmul(d, q_, o); memcpy(q_, o, sizeof(o)); normalise(); }
    void condition() {
        for (int i = 0; i < N; i++) {
            if (!(P_[i][i] > 1e-12f)) P_[i][i] = 1e-12f;           // (also catches a NaN)
            if (P_[i][i] > 1e6f) P_[i][i] = 1e6f;
        }
        if (P_[2][2] > 4.0f * kPi * kPi) P_[2][2] = 4.0f * kPi * kPi;
    }
    // one state measured directly: y = z - x[i] (x[i] here is the nominal value it is compared with)
    bool fuse(int i, float z, float sigma, float gate, float* test) {
        float H[N] = {}; H[i] = 1.0f;
        const float x = i < 3 ? 0.0f : i < 6 ? v_[i - 3] : i < 9 ? p_[i - 6] : 0.0f;
        return fuse_h(H, z - x, sigma, gate, test);
    }
    bool fuse_h(const float H[N], float y, float sigma, float gate, float* test) {
        float PH[N]; float S = sigma * sigma;
        for (int i = 0; i < N; i++) { float s = 0; for (int k = 0; k < N; k++) if (H[k] != 0.0f) s += P_[i][k] * H[k]; PH[i] = s; }
        for (int k = 0; k < N; k++) if (H[k] != 0.0f) S += H[k] * PH[k];
        if (!(S > 1e-12f)) return false;
        const float t = fabsf(y) / sqrtf(S);
        if (test) *test = fmaxf(*test, t);
        if (gate > 0.0f && t > gate) return false;
        float K[N]; for (int i = 0; i < N; i++) K[i] = PH[i] / S;
        // P = P - K H P  (= P - K PH'), kept symmetric
        for (int i = 0; i < N; i++) for (int j = i; j < N; j++) { const float v = P_[i][j] - K[i] * PH[j]; P_[i][j] = P_[j][i] = v; }
        condition();
        // fold the error into the nominal state
        float dx[N]; for (int i = 0; i < N; i++) dx[i] = K[i] * y;
        rotate_earth(dx[0], dx[1], dx[2]);
        for (int i = 0; i < 3; i++) { v_[i] += dx[3 + i]; p_[i] += dx[6 + i];
            bg_[i] = clampf(bg_[i] + dx[9 + i], -cfg.bias_limit_gyro, cfg.bias_limit_gyro); ba_[i] = clampf(ba_[i] + dx[12 + i], -cfg.bias_limit_accel, cfg.bias_limit_accel); }
        return true;
    }
};

} // namespace fc
