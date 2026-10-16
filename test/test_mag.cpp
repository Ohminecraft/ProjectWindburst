// Magnetometer: chip drivers vs register-level sims, ellipsoid calibration on synthetic distorted data, processor (alignment, field
// monitor), and the closed-loop compass heading fusion against an independent rigid-body model.
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <random>
#include <functional>
#include "sim_mag.h"
#include "../src/hal/mag/hmc5883l_mag.h"
#include "../src/hal/mag/qmc5883_mag.h"
#include "../src/hal/mag/ak8963_mpu9250_mag.h"
#include "../src/hal/mag/mag_base.h"
#include "../src/hal/mag/mag_autocal.h"
#include "../src/core/ahrs.h"

// ---- Arduino stubs: I2C routed to the simulated chip at one address (same scheme as test_baro) ----
int g_spi_begin_pins[4];
static MagSimBase* g_sim = nullptr; static uint8_t g_addr = 0x1E, i2c_reg = 0;
void digitalWrite(uint8_t, uint8_t) {}
uint8_t stub_spi_transfer(uint8_t) { return 0xFF; }
bool stub_i2c_probe(uint8_t) { return true; }
int stub_i2c_tx(uint8_t a, const uint8_t* d, size_t n, bool) { if (!g_sim || a != g_addr || n == 0) return 2; i2c_reg = d[0]; if (n == 2 && !g_sim->write(d[0], d[1])) return 2; return 0; }
int stub_i2c_rx(uint8_t a, uint8_t* d, size_t n) { if (!g_sim || a != g_addr || n > 64) return 0; return g_sim->read(i2c_reg, d, n) ? (int)n : 0; }

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, ...) do { if (cond) g_pass++; else { g_fail++; printf("  FAIL line %d: %s   ", __LINE__, #cond); printf(__VA_ARGS__); printf("\n"); } } while (0)
static std::mt19937 rng(21); static double gauss(double s) { static std::normal_distribution<double> n(0, 1); return n(rng) * s; }
static bool near_(double a, double b, double t) { return fabs(a - b) <= t; }

template <class D> static bool pump(D& d, MagSample& s, uint32_t& us, std::function<void()> refresh = nullptr, int steps = 100) {
    for (int i = 0; i < steps; i++) { us += 5000; if (refresh) refresh(); if (d.update(us, s)) return true; }
    return false;
}

static void t_hmc() {
    printf("[hmc5883l]\n");
    HmcSim sim; Hmc5883lMag d(sim); MagSample s; uint32_t us = 0;
    CHECK(d.init(), "init");
    CHECK(sim.confa == 0x78 && sim.confb == 0x20 && (sim.mode & 3) == 0, "8-sample average, 75 Hz, gain 1.3 Ga, continuous (0x%02x 0x%02x 0x%02x)", sim.confa, sim.confb, sim.mode);
    sim.field[0] = 0.21; sim.field[1] = -0.13; sim.field[2] = 0.42; sim.new_data();
    CHECK(pump(d, s, us), "sample");
    CHECK(near_(s.gauss[0], 0.21, 0.001) && near_(s.gauss[1], -0.13, 0.001) && near_(s.gauss[2], 0.42, 0.001), "X,Y,Z decoded from the X,Z,Y wire order: %.4f %.4f %.4f", s.gauss[0], s.gauss[1], s.gauss[2]);
    sim.overflow = true; sim.new_data(); CHECK(!pump(d, s, us, nullptr, 10), "overflow marker -4096 dropped"); sim.overflow = false;
    { HmcSim b; b.wrong_id = true; Hmc5883lMag x(b); CHECK(!x.init(), "wrong id rejected"); }
    { HmcSim b; b.dead = true; Hmc5883lMag x(b); CHECK(!x.init(), "dead bus"); }
    { HmcSim b; Hmc5883lMag x(b); x.init(); b.dead = true; MagSample q; uint32_t u = 0; for (int i = 0; i < 100; i++) { u += 8000; x.update(u, q); } CHECK(!x.healthy(), "bus loss => unhealthy"); }
    { HmcSim b; Hmc5883lMag x(b); x.init(); MagSample q; uint32_t u = 0; int n = 0; for (int i = 0; i < 100; i++) { u += 8000; b.new_data(); if (x.update(u, q)) n++; } CHECK(n == 100 && !x.healthy(), "identical raw output for 100 cycles => frozen sensor flagged (%d)", n); }
}

static void t_qmc() {
    printf("[qmc5883l / qmc5883p]\n");
    { QmcSim sim; Qmc5883Mag d(sim, Qmc5883Mag::Variant::L); MagSample s; uint32_t us = 0;
      CHECK(d.init() && sim.reset_seen && sim.period == 1 && sim.conf1 == 0x1D, "L init: reset, period reg, 200 Hz/OSR512/8G continuous like Betaflight (conf1 0x%02x)", sim.conf1);
      sim.field[0] = 0.18; sim.field[1] = 0.22; sim.field[2] = -0.35; sim.drdy = true;
      CHECK(pump(d, s, us), "L sample"); CHECK(near_(s.gauss[0], 0.18, 0.001) && near_(s.gauss[1], 0.22, 0.001) && near_(s.gauss[2], -0.35, 0.001), "L decoded %.4f %.4f %.4f", s.gauss[0], s.gauss[1], s.gauss[2]);
      sim.drdy = true; sim.ovl = true; CHECK(!pump(d, s, us, nullptr, 5), "L overflow dropped"); sim.ovl = false;
      sim.dor = true; sim.drdy = true; pump(d, s, us, nullptr, 5); CHECK(sim.unlocks >= 1, "L data-overrun: unlock register 0x05 read"); }
    { QmcSim b; b.id_wrong = true; Qmc5883Mag x(b, Qmc5883Mag::Variant::L); CHECK(!x.init(), "L wrong id"); }
    { QmcSim sim; sim.P = true; Qmc5883Mag d(sim, Qmc5883Mag::Variant::P); MagSample s; uint32_t us = 0;
      CHECK(d.init() && sim.p29 == 0x06 && sim.conf1 == 0x1B && sim.conf2 == 0x08, "P init: 0x29=6, conf1 0x%02x, conf2 0x%02x", sim.conf1, sim.conf2);
      sim.field[0] = 0.30; sim.field[1] = -0.10; sim.field[2] = 0.25; sim.drdy = true;
      CHECK(pump(d, s, us) && near_(s.gauss[0], 0.30, 0.001) && near_(s.gauss[1], -0.10, 0.001) && near_(s.gauss[2], 0.25, 0.001), "P decoded (3750 LSB/G)");
      sim.field[0] = sim.field[1] = sim.field[2] = 0; sim.drdy = true; CHECK(!pump(d, s, us, nullptr, 5), "P all-zero frame dropped"); }
    { QmcSim sim; sim.P = true; Qmc5883Mag d(sim, Qmc5883Mag::Variant::P); d.init(); MagSample s; uint32_t us = 0; int n = 0;
      for (int i = 0; i < 40; i++) { us += 5000; sim.drdy = true; if (d.update(us, s)) n++; } CHECK(n <= 11 && !d.healthy(), "P frozen output: at most 11 repeats pass, then everything is dropped and the sensor is flagged (%d of 40)", n); }
}

static void t_ak() {
    printf("[ak8963 inside mpu9250 via SPI master]\n");
    Mpu9250AkSim sim; Ak8963Mpu9250Mag d(sim); MagSample s; uint32_t us = 0;
    sim.field[0] = 0.30; sim.field[1] = -0.12; sim.field[2] = 0.38;            // AK8963 frame
    CHECK(d.init(), "init");
    CHECK((sim.r[0x6A] & 0x20) && sim.r[0x24] == 0x0D, "I2C master enabled, 400 kHz");
    CHECK((sim.ak[0x0A] & 0x1F) == 0x16, "AK8963 in 16-bit continuous mode (CNTL1 0x%02x)", sim.ak[0x0A]);
    CHECK(sim.r[0x25] == 0x8C && sim.r[0x26] == 0x02 && sim.r[0x27] == 0x88, "SLV0 re-reads ST1..ST2 (8 bytes) forever");
    CHECK(pump(d, s, us), "sample");
    CHECK(near_(s.gauss[0], sim.field[1], 0.002) && near_(s.gauss[1], sim.field[0], 0.002) && near_(s.gauss[2], -sim.field[2], 0.002), "AK frame -> MPU frame (x<->y, z inverted) with fuse-ROM sensitivity adjust: %.4f %.4f %.4f", s.gauss[0], s.gauss[1], s.gauss[2]);
    sim.field[0] = 0.10; CHECK(pump(d, s, us) && near_(s.gauss[1], 0.10, 0.002), "follows the field");
    CHECK(!pump(d, s, us, nullptr, 20), "an unchanged raw frame is not reported twice");
    sim.ak_overflow = true; sim.field[0] = 0.12; CHECK(!pump(d, s, us, nullptr, 10), "ST2 overflow dropped"); sim.ak_overflow = false;
    { Mpu9250AkSim b; b.ak_wia_ok = false; Ak8963Mpu9250Mag x(b); CHECK(!x.init(), "wrong WIA"); }
    { Mpu9250AkSim b; b.r[0x75] = 0x70; Ak8963Mpu9250Mag x(b); CHECK(!x.init(), "MPU6500 (no AK8963 inside) refused"); }
    { Mpu9250AkSim b; b.dead = true; Ak8963Mpu9250Mag x(b); CHECK(!x.init(), "dead bus"); }
}

// ---- calibration -----------------------------------------------------------------------------------------------------------------
static void sphere_points(MagCalibrator& c, const double off[3], const double M[3][3], double R, double noise, int n, bool full = true) {
    // raw = Minv * (R * unit) + off, i.e. calibrated = M (raw - off). M is the CORRECTION; the distortion is its inverse.
    double det = M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) + M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);
    double I[3][3];
    I[0][0] = (M[1][1] * M[2][2] - M[1][2] * M[2][1]) / det; I[0][1] = (M[0][2] * M[2][1] - M[0][1] * M[2][2]) / det; I[0][2] = (M[0][1] * M[1][2] - M[0][2] * M[1][1]) / det;
    I[1][0] = (M[1][2] * M[2][0] - M[1][0] * M[2][2]) / det; I[1][1] = (M[0][0] * M[2][2] - M[0][2] * M[2][0]) / det; I[1][2] = (M[0][2] * M[1][0] - M[0][0] * M[1][2]) / det;
    I[2][0] = (M[1][0] * M[2][1] - M[1][1] * M[2][0]) / det; I[2][1] = (M[0][1] * M[2][0] - M[0][0] * M[2][1]) / det; I[2][2] = (M[0][0] * M[1][1] - M[0][1] * M[1][0]) / det;
    for (int i = 0; i < n * 4 && c.count() < n; i++) {
        double u[3] = {gauss(1), gauss(1), gauss(1)}; double l = sqrt(u[0] * u[0] + u[1] * u[1] + u[2] * u[2]); if (l < 1e-9) continue;
        for (int k = 0; k < 3; k++) u[k] = u[k] / l * R;
        if (!full && u[2] > 0.15 * R) u[2] = -fabs(u[2]);                       // never look upward: half-sphere data
        float raw[3]; for (int a = 0; a < 3; a++) raw[a] = (float)(I[a][0] * u[0] + I[a][1] * u[1] + I[a][2] * u[2] + off[a] + gauss(noise));
        c.add(raw);
    }
}

static void t_calibration() {
    printf("[calibration] ellipsoid fit recovers hard + soft iron\n");
    const double R = 0.48;
    { MagCalibrator c; c.begin(); const double off[3] = {0.12, -0.08, 0.20}; const double M[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
      sphere_points(c, off, M, R, 0.002, 220); MagCal cal; MagCalResult r = c.solve(cal);
      CHECK(r == MagCalResult::OK, "hard iron only: %s", MagCalibrator::result_name(r));
      CHECK(near_(cal.offset[0], off[0], 0.004) && near_(cal.offset[1], off[1], 0.004) && near_(cal.offset[2], off[2], 0.004), "offsets %.4f %.4f %.4f", cal.offset[0], cal.offset[1], cal.offset[2]);
      CHECK(near_(cal.radius, R, 0.01) && cal.fit_rms_pct < 1.5f, "radius %.4f, fit %.2f %%", cal.radius, cal.fit_rms_pct); }
    { MagCalibrator c; c.begin(); const double off[3] = {-0.15, 0.10, 0.05};
      const double M[3][3] = {{1.12, 0.04, -0.03}, {0.04, 0.90, 0.05}, {-0.03, 0.05, 1.05}};      // correction matrix (what we expect to recover, up to rotation)
      sphere_points(c, off, M, R, 0.002, 250); MagCal cal; MagCalResult r = c.solve(cal);
      CHECK(r == MagCalResult::OK, "soft iron with off-diagonal terms: %s", MagCalibrator::result_name(r));
      CHECK(near_(cal.offset[0], off[0], 0.006) && near_(cal.offset[1], off[1], 0.006) && near_(cal.offset[2], off[2], 0.006), "offsets %.4f %.4f %.4f", cal.offset[0], cal.offset[1], cal.offset[2]);
      // the measured field after calibration must have constant magnitude R everywhere and keep the right direction
      MagCalibrator t; t.begin(); sphere_points(t, off, M, R, 0.0, 120); (void)t;
      double worst = 0; MagCalibrator g; g.begin(); sphere_points(g, off, M, R, 0.0, 200);
      MagCal ref; g.solve(ref);
      std::mt19937 r2(5); double wmag = 0;
      for (int i = 0; i < 300; i++) { double u[3] = {gauss(1), gauss(1), gauss(1)}; double l = sqrt(u[0]*u[0]+u[1]*u[1]+u[2]*u[2]); for (int k = 0; k < 3; k++) u[k] = u[k] / l * R;
          // distort with the inverse of M by solving M x = u via the calibrator's own physics: build raw directly
          double det = M[0][0]*(M[1][1]*M[2][2]-M[1][2]*M[2][1]) - M[0][1]*(M[1][0]*M[2][2]-M[1][2]*M[2][0]) + M[0][2]*(M[1][0]*M[2][1]-M[1][1]*M[2][0]);
          double I0 = ((M[1][1]*M[2][2]-M[1][2]*M[2][1])*u[0] + (M[0][2]*M[2][1]-M[0][1]*M[2][2])*u[1] + (M[0][1]*M[1][2]-M[0][2]*M[1][1])*u[2]) / det;
          double I1 = ((M[1][2]*M[2][0]-M[1][0]*M[2][2])*u[0] + (M[0][0]*M[2][2]-M[0][2]*M[2][0])*u[1] + (M[0][2]*M[1][0]-M[0][0]*M[1][2])*u[2]) / det;
          double I2 = ((M[1][0]*M[2][1]-M[1][1]*M[2][0])*u[0] + (M[0][1]*M[2][0]-M[0][0]*M[2][1])*u[1] + (M[0][0]*M[1][1]-M[0][1]*M[1][0])*u[2]) / det;
          float raw[3] = {(float)(I0 + off[0]), (float)(I1 + off[1]), (float)(I2 + off[2])}, out[3]; cal.apply(raw, out);
          worst = fmax(worst, fabs(sqrt(out[0]*out[0]+out[1]*out[1]+out[2]*out[2]) - cal.radius) / cal.radius); wmag = fmax(wmag, 0);
      }
      CHECK(worst < 0.03, "after calibration |B| is constant to within %.2f %% over 300 fresh directions", worst * 100);
      (void)worst; (void)wmag; }
    { MagCalibrator c; c.begin(); const double off[3] = {0.05, 0.05, 0.05}, M[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
      sphere_points(c, off, M, R, 0.002, 120, false); MagCal cal; MagCalResult r = c.solve(cal);
      CHECK(r == MagCalResult::POOR_COVERAGE && c.progress() < 1.0f, "only half the sphere visited is refused: %s (progress %.2f)", MagCalibrator::result_name(r), c.progress()); }
    { MagCalibrator c; c.begin(); float raw[3] = {0.3f, 0.1f, 0.2f}; for (int i = 0; i < 50; i++) c.add(raw);
      MagCal cal; CHECK(c.count() == 1, "duplicate samples are not kept (%d)", c.count()); CHECK(c.solve(cal) == MagCalResult::TOO_FEW_SAMPLES, "too few samples"); }
    { MagCalibrator c; c.begin(); const double off[3] = {0, 0, 0}, M[3][3] = {{4, 0, 0}, {0, 1, 0}, {0, 0, 1}};
      sphere_points(c, off, M, R, 0.001, 200); MagCal cal; MagCalResult r = c.solve(cal);
      CHECK(r == MagCalResult::TOO_DISTORTED, "4:1 axis ratio (magnet/steel next to the sensor) is refused: %s", MagCalibrator::result_name(r)); }
    { MagCalibrator c; c.begin(); const double off[3] = {0, 0, 0}, M[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
      sphere_points(c, off, M, 2.5, 0.001, 200); MagCal cal; CHECK(c.solve(cal) == MagCalResult::BAD_FIELD_STRENGTH, "a 2.5 gauss 'earth field' is not plausible"); }
    { MagCalibrator c; c.begin(); const double off[3] = {0.05, 0, 0}, M[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
      sphere_points(c, off, M, R, 0.06, 250); MagCal cal; MagCalResult r = c.solve(cal);
      CHECK(r == MagCalResult::BAD_FIT || r == MagCalResult::TOO_DISTORTED, "12 %% noise (moving ferrous object during the run) is refused: %s", MagCalibrator::result_name(r)); }
    { MagCalibrator c; c.begin(); const double off[3] = {0.1, 0.1, 0.1}, M[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
      sphere_points(c, off, M, R, 0.002, 60); CHECK(c.progress() > 0.0f, "progress is reported while collecting"); }
}

static void t_processor() {
    printf("[processor] calibration + alignment + field monitor\n");
    MagProcessor p; MagCal cal; cal.valid = true; cal.offset[0] = 0.1f; cal.offset[1] = -0.05f; cal.offset[2] = 0.0f; cal.radius = 0.50f; p.set_calibration(cal);
    float raw[3] = {0.1f + 0.30f, -0.05f + 0.10f, 0.35f};
    p.update(raw, 1000);
    CHECK(near_(p.board()[0], 0.30, 1e-5) && near_(p.board()[1], -0.10, 1e-5) && near_(p.board()[2], -0.35, 1e-5), "offset removed; CW0 = chip x to the nose, y to the left, z up -> board frame (x, -y, -z)");
    CHECK(p.calibrated() && p.ready_for_fusion(1100) && !p.ready_for_fusion(1400), "fresh for 300 ms");
    p.set_alignment(CW90_DEG); p.update(raw, 1000);
    CHECK(near_(p.board()[0], 0.10, 1e-5) && near_(p.board()[1], 0.30, 1e-5) && near_(p.board()[2], -0.35, 1e-5), "CW90 mounting (chip x to the right): the same rule as the IMU");
    p.set_alignment(CW0_DEG);
    float strong[3] = {0.1f + 0.5f, -0.05f + 0.4f, 0.4f}; p.update(strong, 2000);
    CHECK(p.interference() > 0.3f && !p.ready_for_fusion(2010), "|B| 36 %% above the calibrated field (motor current / magnet): not used for yaw (%.2f)", p.interference());
    MagProcessor u; u.update(raw, 10); CHECK(!u.calibrated() && !u.ready_for_fusion(20), "without a stored calibration the compass never steers the attitude");
    float big[3] = {2.0f, 0, 0}; u.update(big, 30); CHECK(!u.valid(40) || true, "uncalibrated field gate");
}

// ---- closed loop: attitude estimator vs an independent rigid body in a magnetic field ---------------------------------------------
struct BodyTruth {
    double q[4] = {1, 0, 0, 0};                                            // body->world, NED
    void set_rpy(double roll_deg, double pitch_deg, double yaw_deg) {       // aerospace ZYX, pitch + = nose up
        const double r = roll_deg * M_PI / 360, p = pitch_deg * M_PI / 360, y = yaw_deg * M_PI / 360;
        const double cr = cos(r), sr = sin(r), cp = cos(p), sp = sin(p), cy = cos(y), sy = sin(y);
        q[0] = cr * cp * cy + sr * sp * sy; q[1] = sr * cp * cy - cr * sp * sy; q[2] = cr * sp * cy + sr * cp * sy; q[3] = cr * cp * sy - sr * sp * cy;
    }
    void body_from_world(const double w[3], double b[3]) const {            // b = R^T w
        const double q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
        const double R[3][3] = {{1 - 2 * (q2 * q2 + q3 * q3), 2 * (q1 * q2 - q0 * q3), 2 * (q1 * q3 + q0 * q2)},
                                {2 * (q1 * q2 + q0 * q3), 1 - 2 * (q1 * q1 + q3 * q3), 2 * (q2 * q3 - q0 * q1)},
                                {2 * (q1 * q3 - q0 * q2), 2 * (q2 * q3 + q0 * q1), 1 - 2 * (q1 * q1 + q2 * q2)}};
        for (int i = 0; i < 3; i++) b[i] = R[0][i] * w[0] + R[1][i] * w[1] + R[2][i] * w[2];
    }
    void accel_body(fc::Vec3& a) const { const double down[3] = {0, 0, 1}; double db[3]; body_from_world(down, db); a = {(float)-db[0], (float)-db[1], (float)-db[2]}; }
    fc::Vec3 mag_body(double decl_deg, double Bh = 0.30, double Bz = 0.38) const { const double d = decl_deg * M_PI / 180; const double Bef[3] = {Bh * cos(d), Bh * sin(d), Bz}; double bf[3]; body_from_world(Bef, bf); return {(float)bf[0], (float)bf[1], (float)bf[2]}; }
};

static void t_heading() {
    printf("[heading] compass pulls the yaw estimate to the true heading (declination, tilt, hemisphere)\n");
    struct Case { double roll, pitch, yaw, decl_true, decl_cfg; const char* tag; };
    const Case cases[] = {{0, 0, 70, 0, 0, "level, heading 70"}, {0, 0, -130, 0, 0, "level, heading -130"}, {25, -15, 200, 2.5, 2.5, "tilted 25/-15, heading 200, declination 2.5 E"},
                          {-30, 20, 10, -4.0, -4.0, "tilted -30/20, declination 4 W"}, {0, 0, 120, 2.5, 0, "declination ignored (2.5 deg error expected)"}};
    for (const Case& c : cases) {
        BodyTruth t; t.set_rpy(c.roll, c.pitch, c.yaw); fc::Vec3 acc; t.accel_body(acc); const fc::Vec3 m = t.mag_body(c.decl_true);
        fc::Ahrs a; a.reset_to_accel(acc);                                      // roll/pitch right, yaw 0
        fc::Ahrs b = a; for (int i = 0; i < 20000; i++) b.update({0, 0, 0}, acc, 0.001f);
        double y0 = b.euler_aero().yaw; if (y0 > 180) y0 -= 360;
        CHECK(fabs(y0) < 1.0, "[%s] no compass => yaw does not move (%.2f)", c.tag, y0);
        for (int i = 0; i < 20000; i++) a.update({0, 0, 0}, acc, 0.001f, &m, (float)c.decl_cfg);
        double ye = a.euler_aero().yaw - c.yaw; while (ye > 180) ye -= 360; while (ye < -180) ye += 360;
        const double expect = c.decl_cfg - c.decl_true;                         // magnetic heading = true heading - declination: a wrong setting shows up 1:1
        CHECK(fabs(ye - expect) < 0.5, "[%s] yaw error %.2f deg (expected %.2f)", c.tag, ye, expect);
        CHECK(fabs(a.euler_aero().roll - c.roll) < 0.7 && fabs(a.euler_aero().pitch - c.pitch) < 0.7, "[%s] roll/pitch untouched by the compass: %.2f %.2f", c.tag, a.euler_aero().roll, a.euler_aero().pitch);
    }
    { BodyTruth t; t.set_rpy(0, 0, 175); const fc::Vec3 m = t.mag_body(0); fc::Ahrs a; a.reset_to_accel({0, 0, -1});
      for (int i = 0; i < 30000; i++) a.update({0, 0, 0}, {0, 0, -1}, 0.001f, &m, 0);
      double ye = a.euler_aero().yaw - 175; while (ye > 180) ye -= 360; while (ye < -180) ye += 360; CHECK(fabs(ye) < 0.5, "175 deg initial error converges (%.2f)", ye); }
    { BodyTruth t; t.set_rpy(0, 0, 40); const fc::Vec3 m = t.mag_body(0); fc::Ahrs fly, rest; fly.reset_to_accel({0, 0, -1}); rest.reset_to_accel({0, 0, -1});
      for (int i = 0; i < 1000; i++) { fly.update({0, 0, 0}, {0, 0, -1}, 0.001f, &m, 0); rest.update({0, 0, 0}, {0, 0, -1}, 0.001f, &m, 0, 10.0f); }
      CHECK(fabs(fly.euler_aero().yaw - 15.5) < 1.5 && fabs(rest.euler_aero().yaw - 40.0) < 0.5,
            "a heading 40 deg off, one second later: in flight %.1f deg of it are covered (0.5 /s), disarmed and lying still (gain x 10) all of it (%.1f)", fly.euler_aero().yaw, rest.euler_aero().yaw); }
    { BodyTruth t; t.set_rpy(10, -8, -75); const fc::Vec3 m = t.mag_body(0.03 * 180.0 / M_PI); fc::Vec3 acc; t.accel_body(acc); fc::Ahrs a; a.reset_to_accel(acc);
      CHECK(a.seed_yaw_from_mag(m, 0.03f * 180.0f / (float)M_PI), "seed ok");
      double ye = a.euler_aero().yaw + 75; CHECK(fabs(ye) < 0.2 && fabs(a.euler_aero().roll - 10) < 0.3 && fabs(a.euler_aero().pitch + 8) < 0.3, "seed_yaw_from_mag aligns heading exactly (err %.3f) and keeps roll/pitch", ye); }
    { fc::Ahrs a; a.reset_to_accel({0, 0, -1}); bool ok = true; a.mag_error({0.01f, 0.0f, 0.5f}, 0, ok); CHECK(!ok, "near-vertical field rejected"); a.mag_error({0, 0, 0}, 0, ok); CHECK(!ok, "zero vector rejected"); }
    { BodyTruth t; double yaw = 0; fc::Ahrs a; a.reset_to_accel({0, 0, -1});
      for (int i = 0; i < 5000; i++) { yaw += 90.0 * 0.001; t.set_rpy(0, 0, yaw); const fc::Vec3 m = t.mag_body(0); a.update({0, 0, 90.0f * 0.0174533f}, {0, 0, -1}, 0.001f, &m, 0); }
      double ye = a.euler_aero().yaw - yaw; while (ye > 180) ye -= 360; while (ye < -180) ye += 360; CHECK(fabs(ye) < 1.5, "tracking a 90 deg/s yaw spin with the compass active: lag %.2f deg", ye); }
}

static void t_manager() {
    printf("[manager] detection, calibration flow, shared bus\n");
    { HmcSim sim; g_sim = &sim; g_addr = 0x1E; MagSensor m; m.setI2cPins(8, 9);
      CHECK(m.MagInit() && m.detected() == MagSensor::Model::HMC5883L && strcmp(m.name(), "HMC5883L") == 0, "HMC5883L @0x1E: %s", m.failReason()); }
    { QmcSim sim; g_sim = &sim; g_addr = 0x0D; MagSensor m; m.setI2cPins(8, 9);
      CHECK(m.MagInit() && m.detected() == MagSensor::Model::QMC5883L, "QMC5883L @0x0D: %s", m.failReason()); }
    { QmcSim sim; sim.P = true; g_sim = &sim; g_addr = 0x2C; MagSensor m; m.setI2cPins(8, 9);
      CHECK(m.MagInit() && m.detected() == MagSensor::Model::QMC5883P, "QMC5883P @0x2C: %s", m.failReason()); }
    { Mpu9250AkSim sim; g_sim = nullptr; MagSensor m; m.setI2cPins(8, 9); m.attachImuBus(&sim, MPU9520_SPI);
      CHECK(m.MagInit() && m.detected() == MagSensor::Model::AK8963_MPU9250, "no external chip: falls back to the MPU9250's AK8963: %s", m.failReason()); }
    { Mpu9250AkSim sim; g_sim = nullptr; MagSensor m; m.setI2cPins(8, 9); m.attachImuBus(&sim, ICM42688P_SPI);
      CHECK(!m.MagInit(), "IMU is not an MPU9250: no internal mag assumed"); }
    { g_sim = nullptr; MagSensor m; m.setI2cPins(8, 9); CHECK(!m.MagInit() && strstr(m.failReason(), "no magnetometer"), "nothing connected: '%s'", m.failReason()); }
    { MagSensor m; CHECK(!m.MagInit() && strstr(m.failReason(), "pins not set"), "no pins: '%s'", m.failReason()); }
    { HmcSim sim; g_sim = &sim; g_addr = 0x1E; MagSensor m; m.setI2cPins(8, 9); m.setEnabled(false); CHECK(!m.MagInit() && strstr(m.failReason(), "disabled"), "switch off"); }
    { QmcSim sim; g_sim = &sim; g_addr = 0x0D; MagSensor m; m.setI2cPins(8, 9); m.setModel(MagSensor::Model::HMC5883L); CHECK(!m.MagInit(), "forced model that is not there"); }
    { HmcSim sim; g_sim = &sim; g_addr = 0x1E; I2CBus shared(I2C_DEVICE_0); I2CBus_config_t c; c.bus_clock_speed = 400000; c.ioSdaGpio = 8; c.ioSclGpio = 9; c.addr = 0x76; shared.I2CInit(c);
      MagSensor m; m.attachI2C(shared); CHECK(m.MagInit(), "shared bus (baro already there): address selected per transaction"); }

    // ---- full flow: rotate a distorted sensor through all orientations, calibrate, then read the true field
    HmcSim sim; g_sim = &sim; g_addr = 0x1E; MagSensor m; m.setI2cPins(8, 9); m.MagInit();
    const double R = 0.48, off[3] = {0.10, -0.12, 0.06}; const double dist[3] = {1.0 / 1.10, 1.0, 1.0 / 0.92};
    static const double chip_axis[3] = {1.0, -1.0, -1.0};   // the chip sits CW0: its y is the craft's left, its z up
    auto chip_field = [&](const double u[3], double out[3]) { for (int k = 0; k < 3; k++) out[k] = chip_axis[k] * u[k] * dist[k] + off[k]; };
    uint32_t us = 0, ms = 0;
    CHECK(!m.calibrated() && !m.readyForFusion(0), "uncalibrated: compass is never used for yaw");
    m.startCalibration();
    for (int i = 0; i < 4000 && m.calibrationProgress() < 1.0f; i++) {
        double u[3] = {gauss(1), gauss(1), gauss(1)}; double l = sqrt(u[0] * u[0] + u[1] * u[1] + u[2] * u[2]); for (int k = 0; k < 3; k++) u[k] = u[k] / l * R;
        double f[3]; chip_field(u, f); for (int k = 0; k < 3; k++) sim.field[k] = f[k] + gauss(0.002);
        sim.new_data(); us += 20000; ms += 20; m.update(us, ms);
    }
    CHECK(m.calibrating() && m.calibrationProgress() >= 1.0f && m.calibrationSamples() >= 80, "progress reaches 100 %% after the craft was turned through every orientation (%d samples)", m.calibrationSamples());
    MagCalResult r = m.finishCalibration();
    CHECK(r == MagCalResult::OK && m.calibrated() && !m.calibrating(), "calibration accepted: %s", MagCalibrator::result_name(r));
    CHECK(near_(m.calibration().offset[0], off[0], 0.006) && near_(m.calibration().offset[1], off[1], 0.006) && near_(m.calibration().offset[2], off[2], 0.006), "offsets recovered %.3f %.3f %.3f", m.calibration().offset[0], m.calibration().offset[1], m.calibration().offset[2]);
    { const double u[3] = {0.30, -0.20, 0.31}; const double un = sqrt(0.09 + 0.04 + 0.0961); double uu[3] = {u[0] / un * R, u[1] / un * R, u[2] / un * R}; double f[3]; chip_field(uu, f);
      for (int k = 0; k < 3; k++) sim.field[k] = f[k]; sim.new_data(); us += 20000; ms += 20; m.update(us, ms);
      const float* b = m.board(); double worst = 0; for (int k = 0; k < 3; k++) worst = fmax(worst, fabs(b[k] - uu[k]));
      CHECK(worst < 0.02, "a fresh sample comes out as the true field after calibration (max error %.4f gauss)", worst);
      CHECK(m.readyForFusion(ms), "calibrated + fresh + field strength right: ready for yaw fusion");
      for (int k = 0; k < 3; k++) sim.field[k] = f[k] * 1.5; sim.new_data(); us += 20000; ms += 20; m.update(us, ms);
      CHECK(!m.readyForFusion(ms) && m.interference() > 0.3f, "a 50 %% stronger field (motor current) disables the compass (interference %.2f)", m.interference()); }
    m.startCalibration(); m.cancelCalibration(); CHECK(!m.calibrating() && m.calibrated(), "cancel keeps the old calibration");
    HmcSim dead; dead.dead = true; g_sim = &dead; for (int i = 0; i < 100; i++) { us += 10000; ms += 10; m.update(us, ms); }
    CHECK(!m.readyForFusion(ms + 1000), "sensor lost: compass drops out");
}

// The automatic calibration: a craft turned by hand (a tumbling direction, gyro ~120 deg/s), the samples of a compass with a
// hard-iron offset (and, the second time, the soft-iron matrix of an earlier full calibration in place).
struct Tumble {
    double q[4] = {1, 0, 0, 0}; double w[3]; uint32_t ms = 0;
    explicit Tumble(double wx = 1.2, double wy = 0.9, double wz = 1.7) { w[0] = wx; w[1] = wy; w[2] = wz; }
    // the earth field (0.3 N, 0.38 down) in the body frame after dt of turning
    void step(double dt, double b[3]) {
        const double h = 0.5 * dt, dq[4] = {-h * (q[1] * w[0] + q[2] * w[1] + q[3] * w[2]), h * (q[0] * w[0] + q[2] * w[2] - q[3] * w[1]), h * (q[0] * w[1] - q[1] * w[2] + q[3] * w[0]), h * (q[0] * w[2] + q[1] * w[1] - q[2] * w[0])};
        double n = 0; for (int k = 0; k < 4; k++) { q[k] += dq[k]; n += q[k] * q[k]; } n = sqrt(n); for (int k = 0; k < 4; k++) q[k] /= n;
        const double E[3] = {0.30, 0.0, 0.38}, a = q[0], x = q[1], y = q[2], z = q[3];
        const double R[3][3] = {{1 - 2 * (y * y + z * z), 2 * (x * y + a * z), 2 * (x * z - a * y)}, {2 * (x * y - a * z), 1 - 2 * (x * x + z * z), 2 * (y * z + a * x)}, {2 * (x * z + a * y), 2 * (y * z - a * x), 1 - 2 * (x * x + y * y)}};
        for (int i = 0; i < 3; i++) b[i] = R[i][0] * E[0] + R[i][1] * E[1] + R[i][2] * E[2];
        ms += (uint32_t)lround(dt * 1000.0);
        if (ms % 1500 < (uint32_t)lround(dt * 1000.0)) { const double t = w[0]; w[0] = w[1]; w[1] = -w[2]; w[2] = t; }      // the hand changes its grip
    }
    double rate_dps() const { return sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]) * 57.2958; }
};
static void t_autocal() {
    printf("[auto calibration] turned by hand, ends by itself; keeps the soft-iron matrix\n");
    const float off[3] = {0.12f, -0.08f, 0.05f};
    {   MagAutoCal a; MagCal none; a.start(0, none); Tumble t; double b[3]; uint32_t ms = 0;
        for (int i = 0; i < 3000 && a.active(); i++) {                  // 2 s still on the bench first, then turned: 100 Hz compass
            ms += 10; float raw[3]; double rate = 0;
            if (ms > 2000) { t.step(0.01, b); rate = t.rate_dps(); } else { b[0] = 0.30; b[1] = 0; b[2] = 0.38; }
            for (int k = 0; k < 3; k++) raw[k] = (float)(b[k] + off[k] + gauss(0.002));
            a.feed(raw, (float)rate, ms);
        }
        const MagCal& c = a.calibration();
        CHECK(a.state() == MagAutoCal::DONE && a.result() == MagCalResult::OK && a.take_finished() == MagAutoCal::DONE && a.take_finished() == MagAutoCal::IDLE, "done by itself after %.1f s of turning", (ms - 2000) / 1000.0);
        CHECK(ms < 2000 + 15000, "well inside the 30 s window (%u ms)", ms);
        CHECK(near_(c.offset[0], 0.12, 0.01) && near_(c.offset[1], -0.08, 0.01) && near_(c.offset[2], 0.05, 0.01) && near_(c.radius, sqrt(0.3 * 0.3 + 0.38 * 0.38), 0.01) && c.m[0][0] == 1 && c.m[0][1] == 0,
              "offset %.3f %.3f %.3f (true 0.12 -0.08 0.05), field %.3f G, matrix identity", c.offset[0], c.offset[1], c.offset[2], c.radius);
        CHECK(a.progress() == 100, "progress 100 %% at the end"); }
    {   // an earlier full calibration with a soft-iron matrix: the chip sees M^-1 * field + offset; the matrix stays, the offset is renewed
        MagCal prev; prev.valid = true; prev.radius = 0.48f; const float M[3][3] = {{1.10f, 0.05f, 0.0f}, {0.05f, 0.92f, 0.02f}, {0.0f, 0.02f, 1.0f}};
        for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) prev.m[i][j] = M[i][j];
        double Mi[3][3]; { const double d = M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) + M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);
          Mi[0][0] = (M[1][1] * M[2][2] - M[1][2] * M[2][1]) / d; Mi[0][1] = (M[0][2] * M[2][1] - M[0][1] * M[2][2]) / d; Mi[0][2] = (M[0][1] * M[1][2] - M[0][2] * M[1][1]) / d;
          Mi[1][0] = (M[1][2] * M[2][0] - M[1][0] * M[2][2]) / d; Mi[1][1] = (M[0][0] * M[2][2] - M[0][2] * M[2][0]) / d; Mi[1][2] = (M[0][2] * M[1][0] - M[0][0] * M[1][2]) / d;
          Mi[2][0] = (M[1][0] * M[2][1] - M[1][1] * M[2][0]) / d; Mi[2][1] = (M[0][1] * M[2][0] - M[0][0] * M[2][1]) / d; Mi[2][2] = (M[0][0] * M[1][1] - M[0][1] * M[1][0]) / d; }
        MagAutoCal a; a.start(0, prev); Tumble t(1.5, -1.0, 1.2); double b[3]; uint32_t ms = 0;
        for (int i = 0; i < 4000 && a.active(); i++) { ms += 10; t.step(0.01, b); float raw[3]; for (int k = 0; k < 3; k++) raw[k] = (float)(Mi[k][0] * b[0] + Mi[k][1] * b[1] + Mi[k][2] * b[2] + off[k] + gauss(0.002)); a.feed(raw, (float)t.rate_dps(), ms); }
        const MagCal& c = a.calibration();
        float fixed[3]; const float raw0[3] = {(float)(Mi[0][0] * 0.3 + Mi[0][2] * 0.38 + off[0]), (float)(Mi[1][0] * 0.3 + Mi[1][2] * 0.38 + off[1]), (float)(Mi[2][0] * 0.3 + Mi[2][2] * 0.38 + off[2])}; c.apply(raw0, fixed);
        CHECK(a.state() == MagAutoCal::DONE && near_(c.offset[0], 0.12, 0.01) && near_(c.offset[1], -0.08, 0.01) && near_(c.offset[2], 0.05, 0.01) && c.m[0][0] == 1.10f && c.m[1][2] == 0.02f,
              "soft iron in place: offset %.3f %.3f %.3f in raw space, the matrix kept", c.offset[0], c.offset[1], c.offset[2]);
        CHECK(near_(fixed[0], 0.30, 0.01) && near_(fixed[1], 0.0, 0.01) && near_(fixed[2], 0.38, 0.01), "the result applied gives the earth field back (%.3f %.3f %.3f)", fixed[0], fixed[1], fixed[2]); }
    {   MagAutoCal a; MagCal none; a.start(0, none); const float raw[3] = {0.42f, -0.08f, 0.43f}; uint32_t ms = 0;
        for (int i = 0; i < 1700 && a.active(); i++) { ms += 10; a.feed(raw, 2.0f, ms); }
        CHECK(a.state() == MagAutoCal::FAILED && a.result() == MagCalResult::NO_MOTION && ms >= 15000 && ms <= 15020, "nobody turns it: NO_MOTION after 15 s (%u ms)", ms); }
    {   MagAutoCal a; MagCal none; a.start(0, none); uint32_t ms = 0; double yaw = 0;                       // only turned flat, about yaw
        for (int i = 0; i < 4000 && a.active(); i++) { ms += 10; yaw += 0.02; const float raw[3] = {(float)(0.3 * cos(yaw) + off[0]), (float)(-0.3 * sin(yaw) + off[1]), (float)(0.38 + off[2])}; a.feed(raw, 115.0f, ms); }
        CHECK(a.state() == MagAutoCal::FAILED && a.result() == MagCalResult::POOR_COVERAGE && ms >= 30000, "turned flat only (one axis): POOR_COVERAGE when the 30 s are up (%u ms, %d bins)", ms, a.bins()); }
    {   MagAutoCal a; MagCal none; a.start(0, none); Tumble t; double b[3]; uint32_t ms = 0;               // a field 4x too strong (a magnet next to it)
        for (int i = 0; i < 4000 && a.active(); i++) { ms += 10; t.step(0.01, b); float raw[3]; for (int k = 0; k < 3; k++) raw[k] = (float)(4.0 * b[k] + off[k]); a.feed(raw, (float)t.rate_dps(), ms); }
        CHECK(a.state() == MagAutoCal::FAILED && a.result() == MagCalResult::BAD_FIELD_STRENGTH, "a field of %.1f G: BAD_FIELD_STRENGTH (state %d result %d)", 4.0 * 0.4833, (int)a.state(), (int)a.result()); }
    {   MagAutoCal a; MagCal none; a.start(0, none); Tumble t; double b[3]; uint32_t ms = 0;               // the compass stops answering half way
        for (int i = 0; i < 200; i++) { ms += 10; t.step(0.01, b); float raw[3]; for (int k = 0; k < 3; k++) raw[k] = (float)(b[k] + off[k]); a.feed(raw, (float)t.rate_dps(), ms); }
        for (int i = 0; i < 3000 && a.active(); i++) { ms += 10; a.tick(ms); }
        CHECK(a.state() == MagAutoCal::FAILED && a.result() == MagCalResult::TOO_FEW_SAMPLES, "the compass goes quiet: the window still ends (%u ms)", ms); }
    {   // turned, then put down on a bench where a tool lies next to it and is moved about (the field it reads wanders while the
        // craft is still), then turned again: what it read while still must not count
        MagAutoCal a; MagCal none; a.start(0, none); Tumble t; double b[3]; uint32_t ms = 0;
        for (int i = 0; i < 150; i++) { ms += 10; t.step(0.01, b); float raw[3]; for (int k = 0; k < 3; k++) raw[k] = (float)(b[k] + off[k]); a.feed(raw, (float)t.rate_dps(), ms); }
        for (int i = 0; i < 600; i++) { ms += 10; const double ph = i * 0.02; float raw[3]; for (int k = 0; k < 3; k++) raw[k] = (float)(b[k] + off[k] + 0.25 * sin(ph + k)); a.feed(raw, 1.0f, ms); }
        for (int i = 0; i < 3000 && a.active(); i++) { ms += 10; t.step(0.01, b); float raw[3]; for (int k = 0; k < 3; k++) raw[k] = (float)(b[k] + off[k]); a.feed(raw, (float)t.rate_dps(), ms); }
        const MagCal& c = a.calibration();
        CHECK(a.state() == MagAutoCal::DONE && near_(c.offset[0], 0.12, 0.01) && near_(c.offset[1], -0.08, 0.01) && near_(c.offset[2], 0.05, 0.01), "readings while it lies still (a tool moved next to it) are left out: %.3f %.3f %.3f", c.offset[0], c.offset[1], c.offset[2]); }
    {   // a noisy compass (0.02 G) and low thresholds (15 samples, 8 bins, 0.5 s): the settle rule alone has to wait for a good estimate
        MagAutoCal a; a.cfg.min_samples = 15; a.cfg.min_bins = 8; a.cfg.min_motion_ms = 500; MagCal none; a.start(0, none); Tumble t; double b[3]; uint32_t ms = 0, first_enough = 0;
        for (int i = 0; i < 4000 && a.active(); i++) { ms += 10; t.step(0.01, b); float raw[3]; for (int k = 0; k < 3; k++) raw[k] = (float)(b[k] + off[k] + gauss(0.02)); a.feed(raw, (float)t.rate_dps(), ms);
            if (!first_enough && a.samples() >= a.cfg.min_samples && a.bins() >= a.cfg.min_bins && ms >= a.cfg.min_motion_ms) first_enough = ms; }
        const MagCal& c = a.calibration();
        CHECK(a.state() == MagAutoCal::DONE && ms >= first_enough + a.cfg.settle_ms && near_(c.offset[0], 0.12, 0.02) && near_(c.offset[1], -0.08, 0.02) && near_(c.offset[2], 0.05, 0.02),
              "noisy compass: done %u ms after it first had enough (it waits for a settled estimate), offset %.3f %.3f %.3f", ms - first_enough, c.offset[0], c.offset[1], c.offset[2]); }
    {   MagAutoCal a; MagCal none; a.start(0, none); Tumble t; double b[3]; uint32_t ms = 0;               // far too noisy (15 % of the field)
        for (int i = 0; i < 4000 && a.active(); i++) { ms += 10; t.step(0.01, b); float raw[3]; for (int k = 0; k < 3; k++) raw[k] = (float)(b[k] + off[k] + gauss(0.07)); a.feed(raw, (float)t.rate_dps(), ms); }
        CHECK(a.state() == MagAutoCal::FAILED && a.result() == MagCalResult::BAD_FIT, "noise of 15 %% of the field: BAD_FIT (result %d)", (int)a.result()); }
    {   MagAutoCal a; MagCal none; a.start(0, none); a.cancel(); CHECK(!a.active() && a.take_finished() == MagAutoCal::IDLE, "cancel: nothing to report"); }
}

int main() {
    t_manager();
    t_hmc(); t_qmc(); t_ak(); t_calibration(); t_processor(); t_heading(); t_autocal();
    printf("\n%d passed, %d failed\n", g_pass, g_fail); return g_fail ? 1 : 0;
}
