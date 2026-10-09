#pragma once
#include "../config/fast_code.h"
#include "types.h"

// Attitude estimator: Mahony quaternion filter + optional magnetometer yaw correction (Betaflight imu.c family).
//
// The accelerometer of a multirotor in flight does NOT show which way is down: it feels the thrust (always along the craft's own
// vertical axis) and the air drag, never gravity. Leaning over and speeding up, it keeps reading "level"; only once the drag has
// grown to balance the thrust (seconds later) does it show the tilt. So in flight the gyro carries the attitude and the
// accelerometer may only pull it slowly:
//   * kp_acc 0.25 /s (Betaflight's dcm_kp; ArduPilot DCM and PX4 use 0.2): a time constant of 4 s. The caller raises it tenfold while
//     disarmed (kp_scale), when the craft stands still and the reading really is gravity
//   * the reading is low-passed first (vibration is not a tilt), and it counts the less the further its SIZE is from what the
//     sensor reads at rest - not at all from 0.1 g on: a lean that holds its height (thrust = weight / cos tilt: 1.1 g at 25 deg), a
//     climb, a drop are left to the gyro. (Betaflight switches at 0.1 g; fading towards it halves the error of a long lean.) "At
//     rest" is learned while disarmed and still, so an accelerometer that was never calibrated (1.08 g lying flat) counts as well
//   * the integral learns the gyro's bias, but no faster than a real gyro drifts (0.5 deg/s per minute, ArduPilot's limit): a few
//     seconds of acceleration cannot teach it a bias that is not there
//   * with a measured velocity (GPS, optical flow) the real acceleration is known and "down" can be told from the accelerometer even
//     while the craft speeds up: tilt_aid.h works out the error, set_aid() puts it in the accelerometer's place
//   * the compass corrects YAW ONLY (a rotation about earth-Down), so a distorted field can never tilt the craft. Its gain (0.5 /s)
//     is raised with the accelerometer's while disarmed (Betaflight: one gain for all error sources): put down, the heading is right in a second
namespace fc {
class Ahrs {
public:
    struct Config {
        float kp_acc = 0.25f, ki_acc = 0.01f, kp_mag = 0.5f;     // (ki: the bias follows with a time constant of 20 s; ArduPilot DCM 0.0087)
        float accel_gate_g = 0.1f;            // weight = 1 - | |a| / |a at rest| - 1 | / this
        float rest_tc_s = 0.5f;               // how fast the size "at rest" is learned
        float accel_lpf_hz = 10.0f;           // second-order low-pass of the accelerometer before its direction and size are used (0 = none)
        float bias_slew_dps_min = 0.5f;       // the learned gyro bias may change this fast at most (0 = no limit)
        float bias_limit_dps = 3.0f;          // and never grows beyond this (0 = no limit)
        float kp_aid = 0.5f;                  // gain of the velocity-aided tilt error (set_aid): it is right in every manoeuvre, so it may pull harder
    } cfg;

    // gyro rad/s (FRD), acc g (FRD specific force), mag = calibrated body-frame field (any unit) or nullptr, declination east-positive.
    // at_rest: disarmed and not being moved - the size of the reading is taken as "1 g" for this sensor.
    WB_FAST void update(Vec3 g, Vec3 a, float dt, const Vec3* mag = nullptr, float declination_deg = 0.0f, float kp_scale = 1.0f, bool at_rest = false) {
        if (mag && cfg.kp_mag > 0.0f) {
            bool ok = false; const float e = mag_error(*mag, declination_deg, ok);
            if (ok) { const float k = -cfg.kp_mag * kp_scale * e; g.x += k * 2 * (q1 * q3 - q0 * q2); g.y += k * 2 * (q2 * q3 + q0 * q1); g.z += k * (1 - 2 * (q1 * q1 + q2 * q2)); }
        }
        if (!af_ok_ || cfg.accel_lpf_hz <= 0.0f) { af1_ = af_ = a; af_ok_ = true; }
        else {                                                 // two first-order stages: a motor's vibration (100 Hz and up) is cut a hundredfold and more
            const float k = dt / (dt + 1.0f / (2.0f * kPi * cfg.accel_lpf_hz));
            af1_.x += (a.x - af1_.x) * k; af1_.y += (a.y - af1_.y) * k; af1_.z += (a.z - af1_.z) * k;
            af_.x += (af1_.x - af_.x) * k; af_.y += (af1_.y - af_.y) * k; af_.z += (af1_.z - af_.z) * k;
        }
        a = af_;
        const float an = sqrtf(a.x * a.x + a.y * a.y + a.z * a.z);
        acc_weight_ = 0.0f;
        if (at_rest && an > kRestMin && an < kRestMax) rest_g_ += (an - rest_g_) * fminf(dt / cfg.rest_tc_s, 1.0f);
        float ex = 0, ey = 0, ez = 0, w = 0, kp = 0;              // the tilt error (body axes), how much it counts, its gain
        if (an > 1e-3f) {
            w = weight_of(an);
            acc_weight_ = w;
            const float mx = -a.x / an, my = -a.y / an, mz = -a.z / an;                         // measured "down" in body
            const float vx = 2 * (q1 * q3 - q0 * q2), vy = 2 * (q2 * q3 + q0 * q1), vz = 1 - 2 * (q1 * q1 + q2 * q2);   // estimated down = 3rd row of R
            ex = my * vz - mz * vy; ey = mz * vx - mx * vz; ez = mx * vy - my * vx;
            kp = cfg.kp_acc * kp_scale * w;
        }
        if (aid_on_) {                                             // the aided error, world -> body (R^T), takes the accelerometer's place
            ex = (1 - 2 * (q2 * q2 + q3 * q3)) * aid_.x + 2 * (q1 * q2 + q0 * q3) * aid_.y + 2 * (q1 * q3 - q0 * q2) * aid_.z;
            ey = 2 * (q1 * q2 - q0 * q3) * aid_.x + (1 - 2 * (q1 * q1 + q3 * q3)) * aid_.y + 2 * (q2 * q3 + q0 * q1) * aid_.z;
            ez = 2 * (q1 * q3 + q0 * q2) * aid_.x + 2 * (q2 * q3 - q0 * q1) * aid_.y + (1 - 2 * (q1 * q1 + q2 * q2)) * aid_.z;
            w = 1.0f; kp = cfg.kp_aid;
        }
        if (w > 0.0f) {
            const float slew = cfg.bias_slew_dps_min > 0.0f ? cfg.bias_slew_dps_min * kDeg2Rad / 60.0f * dt : 1e9f;
            const float lim = cfg.bias_limit_dps > 0.0f ? cfg.bias_limit_dps * kDeg2Rad : 1e9f;
            ix = clampf(ix + clampf(cfg.ki_acc * w * ex * dt, -slew, slew), -lim, lim);
            iy = clampf(iy + clampf(cfg.ki_acc * w * ey * dt, -slew, slew), -lim, lim);
            iz = clampf(iz + clampf(cfg.ki_acc * w * ez * dt, -slew, slew), -lim, lim);
        }
        g.x += kp * ex + ix; g.y += kp * ey + iy; g.z += kp * ez + iz;
        const float dq0 = 0.5f * (-q1 * g.x - q2 * g.y - q3 * g.z), dq1 = 0.5f * (q0 * g.x + q2 * g.z - q3 * g.y);
        const float dq2 = 0.5f * (q0 * g.y - q1 * g.z + q3 * g.x), dq3 = 0.5f * (q0 * g.z + q1 * g.y - q2 * g.x);
        q0 += dq0 * dt; q1 += dq1 * dt; q2 += dq2 * dt; q3 += dq3 * dt;
        const float n = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= n; q1 *= n; q2 *= n; q3 *= n;
    }
    // The tilt error from a measured velocity (TiltAid::error_world), or nullptr when there is none: while it is set, the
    // accelerometer's own direction is not used (its size still is, for acc_weight()).
    void set_aid(const Vec3* error_world) { aid_on_ = error_world != nullptr; if (aid_on_) aid_ = *error_world; }
    bool aided() const { return aid_on_; }
    // Snap roll/pitch to the accelerometer (yaw 0). Call while disarmed and still so arming starts from a correct attitude.
    void reset_to_accel(Vec3 a) {
        const float roll = atan2f(-a.y, -a.z), pitch = atan2f(a.x, sqrtf(a.y * a.y + a.z * a.z));
        const float cr = cosf(roll / 2), sr = sinf(roll / 2), cp = cosf(pitch / 2), sp = sinf(pitch / 2);
        q0 = cr * cp; q1 = sr * cp; q2 = cr * sp; q3 = -sr * sp; ix = iy = iz = 0; af1_ = af_ = a; af_ok_ = true;
        const float an = sqrtf(a.x * a.x + a.y * a.y + a.z * a.z);
        if (an > kRestMin && an < kRestMax) rest_g_ = an;
        acc_weight_ = weight_of(an);
    }
    // Heading error of a body-frame magnetic vector vs magnetic north (which lies `declination_deg` east of true north):
    // sin(error), + = the estimate points too far clockwise. ok=false for a zero / nearly vertical field.
    float mag_error(Vec3 m, float declination_deg, bool& ok) const {
        ok = false;
        const float n = sqrtf(m.x * m.x + m.y * m.y + m.z * m.z);
        if (n < 1e-4f) return 0.0f;
        float mn, me; earth_horizontal(m, mn, me);
        const float h = sqrtf(mn * mn + me * me);
        if (h < 0.2f * n) return 0.0f;
        ok = true;
        const float d = declination_deg * kDeg2Rad, rn = cosf(d), re = sinf(d), hn = mn / h, he = me / h;
        const float dot = hn * rn + he * re, cross = rn * he - re * hn;
        return dot > 0.0f ? cross : (cross < 0.0f ? -1.0f : 1.0f);          // > 90 deg off: saturate instead of folding back
    }
    // Rotate about earth-Down so the heading matches the compass exactly (initial alignment on the ground).
    bool seed_yaw_from_mag(Vec3 m, float declination_deg) {
        const float n = sqrtf(m.x * m.x + m.y * m.y + m.z * m.z);
        if (n < 1e-4f) return false;
        float mn, me; earth_horizontal(m, mn, me);
        if (sqrtf(mn * mn + me * me) < 0.2f * n) return false;
        const float e = atan2f(me, mn) - declination_deg * kDeg2Rad;
        const float cz = cosf(-e * 0.5f), sz = sinf(-e * 0.5f);
        const float a0 = cz * q0 - sz * q3, a1 = cz * q1 - sz * q2, a2 = cz * q2 + sz * q1, a3 = cz * q3 + sz * q0;
        q0 = a0; q1 = a1; q2 = a2; q3 = a3;
        return true;
    }
    WB_FAST EulerDeg euler_aero() const {
        EulerDeg e;
        e.roll = atan2f(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2)) * kRad2Deg;
        e.pitch = asinf(clampf(2 * (q0 * q2 - q3 * q1), -1.0f, 1.0f)) * kRad2Deg;
        e.yaw = atan2f(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3)) * kRad2Deg;
        return e;
    }
    EulerDeg euler_user() const { EulerDeg e = euler_aero(); e.pitch = -e.pitch; return e; }     // THE place the pilot convention is applied
    WB_FAST float up_cos() const { return 1 - 2 * (q1 * q1 + q2 * q2); }                                 // cos(tilt from vertical) = R[2][2]
    WB_FAST float tilt_deg() const { return acosf(clampf(up_cos(), -1.0f, 1.0f)) * kRad2Deg; }
    // World-frame acceleration without gravity, m/s^2. up positive; north/east from the body specific force (g) and the attitude.
    WB_FAST void world_accel(Vec3 a, float& north, float& east, float& up) const {
        const float fx = a.x * kG0, fy = a.y * kG0, fz = a.z * kG0;
        north = (1 - 2 * (q2 * q2 + q3 * q3)) * fx + 2 * (q1 * q2 - q0 * q3) * fy + 2 * (q1 * q3 + q0 * q2) * fz;
        east = 2 * (q1 * q2 + q0 * q3) * fx + (1 - 2 * (q1 * q1 + q3 * q3)) * fy + 2 * (q2 * q3 - q0 * q1) * fz;
        const float down = 2 * (q1 * q3 - q0 * q2) * fx + 2 * (q2 * q3 + q0 * q1) * fy + (1 - 2 * (q1 * q1 + q2 * q2)) * fz + kG0;
        up = -down;
    }
    // Where the (low-passed) accelerometer alone says "down" is, as roll / pitch in the aero convention. Standing still that is the
    // attitude; in a hover it is where the THRUST points relative to the board - the level error the in-flight trim measures.
    EulerDeg accel_euler_aero() const {
        EulerDeg e; e.roll = atan2f(-af_.y, -af_.z) * kRad2Deg; e.pitch = atan2f(af_.x, sqrtf(af_.y * af_.y + af_.z * af_.z)) * kRad2Deg; e.yaw = 0;
        return e;
    }
    float rest_g() const { return rest_g_; }                                                     // what this sensor reads lying still (1.0 for a calibrated one)
    float acc_weight() const { return acc_weight_; }                                              // 0..1: how much the accelerometer counted in the last update
    Vec3 gyro_bias_dps() const { return {-ix * kRad2Deg, -iy * kRad2Deg, -iz * kRad2Deg}; }        // what the integral has learned (the gyro reads this much too high)
    float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
private:
    WB_FAST void earth_horizontal(const Vec3& m, float& mn, float& me) const {
        mn = (1 - 2 * (q2 * q2 + q3 * q3)) * m.x + 2 * (q1 * q2 - q0 * q3) * m.y + 2 * (q1 * q3 + q0 * q2) * m.z;
        me = 2 * (q1 * q2 + q0 * q3) * m.x + (1 - 2 * (q1 * q1 + q3 * q3)) * m.y + 2 * (q2 * q3 - q0 * q1) * m.z;
    }
    static constexpr float kRestMin = 0.8f, kRestMax = 1.2f;                                      // a reading outside this is never "at rest" (a broken or badly scaled sensor)
    WB_FAST float weight_of(float size_g) const { return clampf(1.0f - fabsf(size_g / rest_g_ - 1.0f) / cfg.accel_gate_g, 0.0f, 1.0f); }
    float ix = 0, iy = 0, iz = 0;
    Vec3 aid_{0, 0, 0}; bool aid_on_ = false;
    Vec3 af1_{0, 0, 0}, af_{0, 0, 0}; bool af_ok_ = false; float acc_weight_ = 0, rest_g_ = 1.0f;
};

// How much more the accelerometer counts while DISARMED (Ahrs::update's kp_scale) - Betaflight's imuCalcKpGain:
//   armed                                   x 1     (the reading is thrust, not gravity)
//   disarmed, being moved (gyro > 15 deg/s, or the reading is not 1 g)      x 1: carried about, righted after a crash - no better than flying
//   lying still for 0.25 s                  x 100 for half a second: the horizon is set afresh (a crash, a hard landing or a flip may have left it anywhere)
//   after that, while it stays still        x 10
// After power-up it starts with the half second of x 100: nothing is known yet.
class AccelGainSchedule {
public:
    struct Config { float disarmed = 10.0f, reset = 100.0f, quiet_s = 0.25f, reset_s = 0.5f, gyro_limit_dps = 15.0f; } cfg;
    // gyro_dps: the largest of the three rates (absolute). accel_ok: the reading is near 1 g (Ahrs::acc_weight() > 0).
    WB_FAST float update(bool armed, float gyro_dps, bool accel_ok, float dt) {
        if (armed) { state_ = ARMED; return 1.0f; }
        if (state_ == ARMED || gyro_dps > cfg.gyro_limit_dps || !accel_ok) { state_ = QUIET; t_ = 0.0f; }
        switch (state_) {
        case QUIET: t_ += dt; if (t_ >= cfg.quiet_s) { state_ = RESET; t_ = 0.0f; } return 1.0f;
        case RESET: t_ += dt; if (t_ >= cfg.reset_s) state_ = SETTLED; return cfg.reset;
        default: return cfg.disarmed;
        }
    }
    bool settled() const { return state_ == SETTLED; }
private:
    enum State : uint8_t { ARMED, QUIET, RESET, SETTLED } state_ = RESET;
    float t_ = 0.0f;
};
} // namespace fc
