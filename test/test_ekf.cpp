// The error-state EKF (src/core/ekf.h) against a simulated multirotor whose true state is known exactly.
//
// The truth is a smooth trajectory (position as functions of time); the attitude is what a multirotor must have to fly it - its
// thrust axis along the needed specific force, its nose where the yaw profile says - so the accelerometer reads what a real one
// reads in flight: thrust only, (0, 0, -|f|) in the body, never "down". The gyro is derived from the attitude's rate of change.
// IMU at 250 Hz with noise and biases, GPS at 10 Hz or 1 Hz with its delay, baro, compass with an optional disturbance that grows
// with the thrust (the 2026-10-11 log: |B| 0.40 .. 0.58 with throttle).
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <random>
#include <vector>
#include "../src/core/ekf.h"

static int g_pass = 0, g_fail = 0;
static bool g_verbose = getenv("WB_VERBOSE") != nullptr;      // WB_VERBOSE=1: print the numbers of the checks that pass too
#define CHECK(cond, ...) do { if (cond) { g_pass++; if (g_verbose) { printf("  ok: "); printf(__VA_ARGS__); printf("\n"); } } else { g_fail++; printf("  FAIL line %d: %s   ", __LINE__, #cond); printf(__VA_ARGS__); printf("\n"); } } while (0)

using namespace fc;
static std::mt19937 rng(77); static float gauss(float s) { std::normal_distribution<float> n(0, 1); return n(rng) * s; }
static float wrap(float a) { while (a > kPi) a -= 2 * kPi; while (a < -kPi) a += 2 * kPi; return a; }

struct Truth { double p[3], v[3], a[3]; double yaw; };
struct Traj {
    double A = 0, B = 0, w = 0.25, ramp = 8, yaw0 = 0.3, yaw_rate = 0, climb = 0;
    Truth at(double t) const {
        // amplitude ramp 0 -> 1 over `ramp` s, smooth up to the 2nd derivative (quintic smoothstep): a consistent truth has no
        // velocity or acceleration steps the IMU would not see
        Truth s{}; const double u = t < ramp ? t / ramp : 1.0;
        const double r = t < ramp ? u * u * u * (10 - 15 * u + 6 * u * u) : 1.0;
        const double rd = t < ramp ? (30 * u * u - 60 * u * u * u + 30 * u * u * u * u) / ramp : 0.0;
        const double rdd = t < ramp ? (60 * u - 180 * u * u + 120 * u * u * u) / (ramp * ramp) : 0.0;
        auto f = [&](double amp, double ww, double& p, double& v, double& a) {
            const double sn = sin(ww * t), cs = cos(ww * t);
            p = r * amp * sn; v = rd * amp * sn + r * amp * ww * cs; a = rdd * amp * sn + 2 * rd * amp * ww * cs - r * amp * ww * ww * sn; };
        f(A, w, s.p[0], s.v[0], s.a[0]); f(B, 2 * w, s.p[1], s.v[1], s.a[1]);
        s.p[2] = -climb * r * 2; s.v[2] = -climb * rd * 2; s.a[2] = -climb * rdd * 2;
        s.yaw = yaw0 + yaw_rate * t; return s; }
};
// body -> earth rotation of a multirotor flying `s`: thrust axis (body z, down) opposite the specific force
static void attitude(const Truth& s, double R[3][3]) {
    double f[3] = {s.a[0], s.a[1], s.a[2] - kG0}; const double n = sqrt(f[0] * f[0] + f[1] * f[1] + f[2] * f[2]);
    double z[3] = {-f[0] / n, -f[1] / n, -f[2] / n}, xc[3] = {cos(s.yaw), sin(s.yaw), 0};
    double y[3] = {z[1] * xc[2] - z[2] * xc[1], z[2] * xc[0] - z[0] * xc[2], z[0] * xc[1] - z[1] * xc[0]}; const double yn = sqrt(y[0] * y[0] + y[1] * y[1] + y[2] * y[2]);
    for (double& c : y) c /= yn;
    double x[3] = {y[1] * z[2] - y[2] * z[1], y[2] * z[0] - y[0] * z[2], y[0] * z[1] - y[1] * z[0]};
    for (int i = 0; i < 3; i++) { R[i][0] = x[i]; R[i][1] = y[i]; R[i][2] = z[i]; }
}
static void euler_of(const double R[3][3], double& r, double& p, double& y) { r = atan2(R[2][1], R[2][2]); p = asin(-R[2][0]); y = atan2(R[1][0], R[0][0]); }

struct Sim {
    Traj tr; Ekf ekf; double t = 0; const double dt = 0.004;
    double gps_hz = 10, gps_delay = 0.15, gps_sp = 0.3, gps_sv = 0.1; bool gps_on = true; double gps_jump_n = 0;
    double gbias[3] = {0, 0, 0}, abias[3] = {0, 0, 0};
    double mag_e[3] = {0.20, 0.0, 0.40}; double mag_dist = 0;          // disturbance: body-frame vector of this size x (thrust / weight - 0.5)
    bool use_mag = true, use_baro = true, disarmed_still = false;
    double next_gps = 0, next_mag = 0, next_baro = 0;
    std::vector<Truth> past;
    double worst_tilt = 0, worst_yaw = 0, worst_pos = 0, sum_yaw2 = 0, mag_only_worst = 0, mag_only2 = 0; int n_err = 0, n_mag = 0; double err_from = 0;
    void start(float yaw_init, bool known) {
        Truth s = tr.at(0); double R[3][3]; attitude(s, R);
        const double f[3] = {0, 0, -kG0}; Vec3 a; a.x = (float)((R[0][0] * f[0] + R[1][0] * f[1] + R[2][0] * f[2]) / kG0); a.y = (float)((R[0][1] * f[0] + R[1][1] * f[1] + R[2][1] * f[2]) / kG0); a.z = (float)((R[0][2] * f[0] + R[1][2] * f[1] + R[2][2] * f[2]) / kG0);
        ekf.init(a, yaw_init, known);
    }
    void run(double seconds) {
        const int n = (int)(seconds / dt + 0.5);
        for (int k = 0; k < n; k++) {
            Truth s0 = tr.at(t), s1 = tr.at(t + dt); double R0[3][3], R1[3][3]; attitude(s0, R0); attitude(s1, R1);
            // body rate: R1 = R0 exp([w dt]x)  ->  [w dt]x ~ R0' R1 - I (antisymmetric part)
            double M[3][3]; for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) { M[i][j] = 0; for (int q = 0; q < 3; q++) M[i][j] += R0[q][i] * R1[q][j]; }
            const double wx = 0.5 * (M[2][1] - M[1][2]), wy = 0.5 * (M[0][2] - M[2][0]), wz = 0.5 * (M[1][0] - M[0][1]);
            // specific force at the middle of the step, body frame
            Truth sm = tr.at(t + 0.5 * dt); double Rm[3][3]; attitude(sm, Rm); const double fe[3] = {sm.a[0], sm.a[1], sm.a[2] - kG0};
            double fb[3]; for (int i = 0; i < 3; i++) fb[i] = Rm[0][i] * fe[0] + Rm[1][i] * fe[1] + Rm[2][i] * fe[2];
            Vec3 dang{(float)(wx + gbias[0] * dt + gauss(0.004f) * dt), (float)(wy + gbias[1] * dt + gauss(0.004f) * dt), (float)(wz + gbias[2] * dt + gauss(0.004f) * dt)};
            Vec3 dvel{(float)((fb[0] + abias[0]) * dt + gauss(0.3f) * dt), (float)((fb[1] + abias[1]) * dt + gauss(0.3f) * dt), (float)((fb[2] + abias[2]) * dt + gauss(0.3f) * dt)};
            ekf.predict(dang, dvel, (float)dt);
            t += dt; past.push_back(s1);
            if (gps_on && t >= next_gps && t > gps_delay) {
                next_gps += 1.0 / gps_hz;
                const Truth g = tr.at(t - gps_delay);
                ekf.fuse_gps((float)(g.p[0] + gps_jump_n + gauss((float)gps_sp)), (float)(g.p[1] + gauss((float)gps_sp)), (float)(g.v[0] + gauss((float)gps_sv)), (float)(g.v[1] + gauss((float)gps_sv)),
                             (float)(g.v[2] + gauss((float)gps_sv)), true, (float)gps_sp * 2, (float)gps_sv * 2);
            }
            double Rt[3][3]; attitude(s1, Rt);
            const double fmag = sqrt(fe[0] * fe[0] + fe[1] * fe[1] + fe[2] * fe[2]) / kG0;
            if (use_mag && t >= next_mag) {
                next_mag += 0.02;
                double mb[3]; for (int i = 0; i < 3; i++) mb[i] = Rt[0][i] * mag_e[0] + Rt[1][i] * mag_e[1] + Rt[2][i] * mag_e[2];
                const double d = mag_dist * (fmag - 0.5); mb[0] += d * 0.8; mb[1] += d * 0.6;
                ekf.fuse_mag({(float)mb[0], (float)mb[1], (float)mb[2]}, 0.0f);
                // what the compass alone (perfect tilt) says: its heading error
                double r, p, y; euler_of(Rt, r, p, y);
                const double mn = Rt[0][0] * mb[0] + Rt[0][1] * mb[1] + Rt[0][2] * mb[2], me = Rt[1][0] * mb[0] + Rt[1][1] * mb[1] + Rt[1][2] * mb[2];
                if (t > err_from) { const double e = fabs(atan2(me, mn)); mag_only_worst = fmax(mag_only_worst, e); mag_only2 += e * e; n_mag++; }
            }
            if (use_baro && t >= next_baro) { next_baro += 0.02; ekf.fuse_baro((float)(-s1.p[2] + gauss(0.3f))); }
            if (disarmed_still && fmod(t, 0.02) < dt) ekf.fuse_still();
            if (t > err_from) {
                double r, p, y; euler_of(Rt, r, p, y); float er, ep, ey; ekf.euler_rad(er, ep, ey);
                worst_tilt = fmax(worst_tilt, fmax(fabs(er - r), fabs(ep - p))); const double ye = fabs(wrap((float)(ey - y)));
                worst_yaw = fmax(worst_yaw, ye); sum_yaw2 += ye * ye; n_err++;
                worst_pos = fmax(worst_pos, hypot(ekf.pos()[0] - s1.p[0], ekf.pos()[1] - s1.p[1]));
            }
        }
    }
    void reset_errors() { worst_tilt = worst_yaw = worst_pos = sum_yaw2 = mag_only_worst = mag_only2 = 0; n_err = n_mag = 0; err_from = t; }
    double yaw_rms() const { return n_err ? sqrt(sum_yaw2 / n_err) : 0; }
    double mag_rms() const { return n_mag ? sqrt(mag_only2 / n_mag) : 0; }
    double deg(double r) const { return r * 57.29578; }
    bool finite() const { for (int i = 0; i < 3; i++) if (!std::isfinite(ekf.pos()[i]) || !std::isfinite(ekf.vel()[i])) return false; return std::isfinite(ekf.q()[0]); }
};

static void t_basic() {
    printf("[ekf] hover and a figure eight, GPS 10 Hz, compass, baro\n");
    { Sim s; s.start(0.3f, true); s.run(20); s.reset_errors(); s.run(60);
      CHECK(s.finite() && s.deg(s.worst_tilt) < 0.5 && s.deg(s.worst_yaw) < 2.0 && s.worst_pos < 1.0, "hover 60 s: tilt within %.2f deg, heading %.2f deg, position %.2f m", s.deg(s.worst_tilt), s.deg(s.worst_yaw), s.worst_pos); }
    { Sim s; s.tr.A = 30; s.tr.B = 15; s.start(0.3f, true); s.run(20); s.reset_errors(); s.run(90);
      CHECK(s.finite() && s.deg(s.worst_tilt) < 1.5 && s.deg(s.worst_yaw) < 3.0 && s.worst_pos < 1.5, "figure eight 30 x 15 m (leaning up to ~25 deg): tilt within %.2f deg, heading %.2f deg, position %.2f m",
            s.deg(s.worst_tilt), s.deg(s.worst_yaw), s.worst_pos);
      const EkfStatus& st = s.ekf.status(); CHECK(st.gps_aiding && st.heading_ok && st.resets == 0, "aiding, heading known, no resets (gps rejects %u, mag rejects %u)", (unsigned)st.gps_rejects, (unsigned)st.mag_rejects); }
}

static void t_heading() {
    printf("[ekf] heading: from the GPS velocity when the compass is wrong or missing\n");
    { Sim s; s.tr.A = 30; s.tr.B = 15; s.use_mag = false; s.start(0.3f + 0.5f, true); s.run(40); s.reset_errors(); s.run(40);
      CHECK(s.deg(s.worst_yaw) < 4.0, "no compass, started 29 deg off: flying the eight, the GPS velocity brings the heading in (%.2f deg over the last 40 s)", s.deg(s.worst_yaw)); }
    { Sim s; s.tr.A = 30; s.tr.B = 15; s.mag_dist = 0.25; s.start(0.3f, true); s.run(20); s.reset_errors(); s.run(90);
      CHECK(s.deg(s.mag_rms()) > 10.0 && s.yaw_rms() < 0.5 * s.mag_rms(),
            "a compass disturbed by the motor current: alone it is off by %.1f deg RMS (worst %.0f); the EKF by %.1f deg RMS (worst %.1f) - the GPS velocity outweighs it whenever the craft accelerates",
            s.deg(s.mag_rms()), s.deg(s.mag_only_worst), s.deg(s.yaw_rms()), s.deg(s.worst_yaw)); }
    { Sim s; s.tr.yaw_rate = 0.3; s.start(0.3f, true); s.gbias[2] = 1.0 * kDeg2Rad; s.run(30); s.reset_errors(); s.run(90);
      CHECK(fabsf(s.ekf.gyro_bias()[2] - (float)s.gbias[2]) < 0.2f * kDeg2Rad && s.deg(s.worst_yaw) < 3.0, "a yaw gyro reading 1 deg/s too high, turning slowly in a hover: learned %.2f deg/s, heading within %.2f deg once learned (from 30 s on)", s.ekf.gyro_bias()[2] * kRad2Deg, s.deg(s.worst_yaw)); }
}

static void t_gps_trouble() {
    printf("[ekf] GPS: 1 Hz, a jump, gone\n");
    double w10 = 0, w1 = 0;
    for (int hz : {10, 1}) { Sim s; s.tr.A = 20; s.tr.B = 10; s.gps_hz = hz; s.start(0.3f, true); s.run(20); s.reset_errors(); s.run(60); (hz == 10 ? w10 : w1) = s.worst_pos;
      CHECK(s.finite() && s.deg(s.worst_tilt) < 2.0, "GPS %d Hz: tilt within %.2f deg", hz, s.deg(s.worst_tilt)); }
    CHECK(w1 < 3.0 && w1 > w10, "1 Hz GPS: position within %.2f m (10 Hz: %.2f) - the accelerometer fills the second between two fixes", w1, w10);
    { Sim s; s.tr.A = 10; s.start(0.3f, true); s.run(30); s.reset_errors(); s.gps_jump_n = 25; s.run(3); s.gps_jump_n = 0; s.run(10);
      CHECK(s.worst_pos < 1.5 && s.ekf.status().gps_rejects > 10, "the GPS jumps 25 m for 3 s: refused (%u), the estimate stays within %.2f m", (unsigned)s.ekf.status().gps_rejects, s.worst_pos); }
    { Sim s; s.tr.A = 10; s.start(0.3f, true); s.run(30); s.gps_jump_n = 25; s.run(8);
      CHECK(s.ekf.status().resets >= 1 && fabs(s.ekf.pos()[0] - (s.tr.at(s.t).p[0] + 25)) < 3.0, "... and stays there: after 5 s the state is set to it (resets %u)", (unsigned)s.ekf.status().resets); }
    { Sim s; s.start(0.3f, true); s.run(20); s.gps_on = false; s.reset_errors(); s.run(60);
      CHECK(s.finite() && s.deg(s.worst_tilt) < 0.6 && !s.ekf.status().gps_aiding, "hovering without any GPS for 60 s: the tilt stays within %.2f deg (the 'stays where it is' constraint; without it 0.9)", s.deg(s.worst_tilt)); }
}

static void t_ground() {
    printf("[ekf] on the ground\n");
    Sim s; s.disarmed_still = true; s.abias[2] = 0.3; s.start(0.3f, true); s.run(60);
    CHECK(fabsf(s.ekf.vel()[0]) < 0.05f && fabsf(s.ekf.vel()[2]) < 0.05f && fabsf(s.ekf.accel_bias()[2] - 0.3f) < 0.1f, "standing: velocity %.3f / %.3f m/s, the accelerometer's 0.3 m/s^2 error on z learned (%.2f)", s.ekf.vel()[0], s.ekf.vel()[2], s.ekf.accel_bias()[2]);
}

int main() {
    t_basic(); t_heading(); t_gps_trouble(); t_ground();
    printf("%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
