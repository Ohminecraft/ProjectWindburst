// Unit tests of the new flight core: filters, AHRS, vertical / horizontal estimators, RC, mixer (vs physics), arming, failsafe.
// Closed-loop tests with the full FlightCore live in test_flight.cpp.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <random>
#include "sim_plant.h"
#include "../src/core/filters.h"
#include "../src/core/ahrs.h"
#include "../src/core/vertical_est.h"
#include "../src/core/horizontal_est.h"
#include "../src/core/rc.h"
#include "../src/core/mixer.h"
#include "../src/core/safety.h"
#include "../src/core/althold.h"
using namespace fc;

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, ...) do { const bool ok_ = (cond); if (getenv("V")) { printf("    "); printf(__VA_ARGS__); printf("\n"); } if (ok_) g_pass++; else { g_fail++; printf("  FAIL line %d: %s   ", __LINE__, #cond); printf(__VA_ARGS__); printf("\n"); } } while (0)
static std::mt19937 rng(1234); static float gauss(float s) { static std::normal_distribution<float> n(0, 1); return n(rng) * s; }

static void t_filters() {
    printf("[helpers]\n");
    CHECK(fabsf(wrap180(190) + 170) < 1e-4f && fabsf(wrap180(-190) - 170) < 1e-4f && fabsf(wrap180(720 + 30) - 30) < 1e-3f, "wrap180");
    // (the filters, the gyro chain and the PID have their own file: test_control.cpp)
}

static void t_fast_math() {
    printf("[fast trigonometry] the approximations the control cycle uses instead of the C library\n");
    double es = 0, ec = 0, ea = 0, eac = 0, eas = 0;
    for (float x = -7.0f; x < 7.0f; x += 1e-4f) { es = fmax(es, fabs(sin_approx(x) - sin((double)x))); ec = fmax(ec, fabs(cos_approx(x) - cos((double)x))); }
    for (float a = -3.14159f; a < 3.14159f; a += 1e-4f) for (float r : {0.01f, 1.0f, 300.0f}) { const float y = sinf(a) * r, x = cosf(a) * r; ea = fmax(ea, fabs(atan2_approx(y, x) - atan2((double)y, (double)x))); }
    for (float x = -1.0f; x <= 1.0f; x += 1e-5f) { eac = fmax(eac, fabs(acos_approx(x) - acos((double)x))); eas = fmax(eas, fabs(asin_approx(x) - asin((double)x))); }
    CHECK(es < 2e-6 && ec < 2e-6, "sin / cos over +-7 rad: largest error %.1e / %.1e", es, ec);
    CHECK(ea < 1e-6 && eas < 1e-6, "atan2 all around (three radii), asin over -1 .. 1: %.1e / %.1e rad", ea, eas);
    CHECK(eac < 7e-5, "acos: %.1e rad (%.4f deg)", eac, eac * 57.29578);
    CHECK(fabsf(atan2_approx(0.0f, 0.0f)) < 1e-6f && atan2_approx(0.0f, -1.0f) > 3.14f && fabsf(atan2_approx(1.0f, 0.0f) - 1.5707963f) < 1e-6f && acos_approx(1.5f) == 0.0f && asin_approx(-2.0f) < -1.5707f,
          "the corners: atan2(0, 0) = 0, atan2(0, -1) = pi, atan2(1, 0) = pi/2, out-of-range acos / asin clamped");
    CHECK(sin_approx(1e9f) == 0.0f && sin_approx(NAN) == 0.0f, "nonsense in: 0 out, no endless loop");
    // tan for the filter coefficients: the largest argument a filter gets is pi * 0.48 (0.48 of the sampling rate)
    double et = 0, et_low = 0;
    for (float x = 0.001f; x < kPi * 0.48f; x += 1e-4f) { const double e = fabs(tan_approx(x) / tan((double)x) - 1.0); et = fmax(et, e); if (x < kPi * 0.25f) et_low = fmax(et_low, e); }
    CHECK(et < 4e-5 && et_low < 5e-6, "tan_approx up to 0.48 of the sampling rate: relative error %.1e (below 0.25: %.1e)", et, et_low);
}

static void t_minmax() {
    printf("[maxf / minf] the same answers as fmaxf / fminf, NaN included\n");
    const float v[] = {-3.5f, -0.0f, 0.0f, 1.0f, 2.5f, 1e30f, -1e30f, INFINITY, -INFINITY, NAN};
    bool same = true;
    for (float a : v) for (float b : v) {
        const float m = maxf(a, b), mr = fmaxf(a, b), n = minf(a, b), nr = fminf(a, b);
        same &= (m == mr || (m != m && mr != mr)) && (n == nr || (n != n && nr != nr));
    }
    CHECK(same, "100 combinations of ordinary numbers, zeros, infinities and NaN agree with the C library");
}

static void t_stall_guard() {
    printf("[stall guard] the rules\n");
    StallGuard g; g.cfg.enabled = true; const float dt = 0.001f;
    float cmd[4] = {0.5f, 0.5f, 0.5f, 0.5f}, rpm[4] = {12000, 12000, 12000, 12000};
    StallGuard::Input in; in.armed = true; in.motors = 4; in.command = cmd; in.rpm = rpm; in.valid_mask = 0x0F;
    auto run = [&](float s) { int m = 0; for (int i = 0; i < (int)(s / dt + 0.5f) && !m; i++) m = g.update(in, dt); return m; };
    CHECK(run(1.0f) == 0, "four motors turning: nothing");
    rpm[1] = 0; CHECK(run(0.05f) == 0, "50 ms without speed: still nothing (a glitch)");
    rpm[1] = 12000; run(0.2f); rpm[1] = 0; const int m = run(0.2f); CHECK(m == 2 && g.last() == 2, "motor 2 at 0 rpm for 100 ms: tripped (motor %d)", m);
    rpm[1] = 12000; run(0.5f);
    rpm[2] = 3500; CHECK(run(0.3f) == 3, "motor 3 drops from 12000 to 3500 rpm at the same command (above min_rpm): tripped");
    rpm[2] = 12000; run(0.5f); cmd[0] = 0.3f; rpm[0] = 3500; CHECK(run(0.5f) == 0, "a motor asked to slow down to 60 %%: its new speed is no drop");
    cmd[0] = 0.5f; rpm[0] = 12000; run(0.2f);
    cmd[3] = 0.1f; rpm[3] = 0; CHECK(run(0.5f) == 0, "a motor at idle (below min_command) is not judged");
    cmd[3] = 0.5f; rpm[3] = 0; in.valid_mask = 0x07; CHECK(run(0.5f) == 0, "a motor without speed telemetry is not judged");
    in.valid_mask = 0x0F; in.armed = false; CHECK(run(0.5f) == 0, "disarmed: nothing");
    g.cfg.enabled = false; in.armed = true; CHECK(run(0.5f) == 0, "switched off: nothing");
}

static void t_ahrs() {
    printf("[ahrs] sign convention + estimation\n");
    const float s20 = sinf(20 * kDeg2Rad), c20 = cosf(20 * kDeg2Rad);
    { Ahrs a; a.reset_to_accel({s20, 0, -c20}); EulerDeg u = a.euler_user(); CHECK(fabsf(u.pitch + 20) < 0.5f && fabsf(u.roll) < 0.5f, "nose up 20: pitch %.2f (want -20)", u.pitch); }
    { Ahrs a; a.reset_to_accel({0, s20, -c20}); EulerDeg u = a.euler_user(); CHECK(fabsf(u.roll + 20) < 0.5f, "roll left 20: %.2f (want -20)", u.roll); }
    { Ahrs a; a.reset_to_accel({-s20, 0, -c20}); CHECK(fabsf(a.euler_user().pitch - 20) < 0.5f, "nose down: pitch + "); }
    { Ahrs a; for (int i = 0; i < 3000; i++) a.update({0, 0, 0}, {s20, 0, -c20}, 0.001f, nullptr, 0, 10.0f); CHECK(fabsf(a.euler_user().pitch + 20) < 1.0f && fabsf(a.up_cos() - c20) < 0.01f, "standing still (gain x 10, as while disarmed): converges from level in 3 s"); }
    { Ahrs a; for (int i = 0; i < 1000; i++) a.update({0, 30 * kDeg2Rad, 0}, {0, 0, 0}, 0.001f); CHECK(fabsf(a.euler_user().pitch + 30) < 0.5f, "+30 dps about y = nose up = pitch -30 (%.2f)", a.euler_user().pitch); }
    { Ahrs a; for (int i = 0; i < 1000; i++) a.update({-40 * kDeg2Rad, 0, 0}, {0, 0, 0}, 0.001f); CHECK(fabsf(a.euler_user().roll + 40) < 0.5f, "roll left rate"); }
    { Ahrs a; const Vec3 lvl{0, 0, -1}; a.reset_to_accel(lvl); float n, e, up; a.world_accel(lvl, n, e, up);
      CHECK(fabsf(n) < 0.01f && fabsf(e) < 0.01f && fabsf(up) < 0.01f, "hover => zero world acceleration");
      a.world_accel({0, 0, -1.2f}, n, e, up); CHECK(fabsf(up - 0.2f * kG0) < 0.01f, "climbing 0.2 g up"); }
    { Ahrs a; a.reset_to_accel({0, 0, -1}); float n, e, up; a.world_accel({0.2f, 0, -1}, n, e, up);
      CHECK(n > 1.9f && n < 2.0f && fabsf(e) < 0.01f, "accelerating forward (level, nose pointing north) = +north (%.2f)", n); }
    { Ahrs a; a.reset_to_accel({0, 0, -1}); for (int i = 0; i < 4000; i++) a.update({0, 0, 0}, {0, 0, -1}, 0.001f); a.reset_to_accel({0, 0.5f, -0.86f});
      EulerDeg u = a.euler_user(); CHECK(u.roll < -25 && u.roll > -35, "reset_to_accel: left wing down is negative roll (%.1f)", u.roll); }
    { Ahrs a; a.reset_to_accel({0, 0, -1}); for (int i = 0; i < 2000; i++) a.update({0, 0, 0}, {0, 2.0f, 0}, 0.001f); CHECK(fabsf(a.euler_user().roll) < 5.0f, "a 2 g sideways reading (impact, vibration) is ignored by the accel gate (roll %.1f)", a.euler_user().roll); }
}

// How much the accelerometer may say about the attitude. In flight it shows the thrust, not gravity (core/ahrs.h): the estimate
// follows it slowly, only while its size says "not accelerating", and never through vibration.
static void t_ahrs_trust() {
    printf("[ahrs] how far the accelerometer is believed\n");
    const float s20 = sinf(20 * kDeg2Rad), c20 = cosf(20 * kDeg2Rad);
    // the pull: after t seconds of a steady 20 degree reading the estimate has covered this share of the way
    auto share = [&](float seconds, float size_g, float kp_scale, float lpf = 10.0f) { Ahrs a; a.cfg.accel_lpf_hz = lpf; a.cfg.ki_acc = 0;
        for (int i = 0; i < (int)(seconds * 1000 + 0.5f); i++) a.update({0, 0, 0}, {s20 * size_g, 0, -c20 * size_g}, 0.001f, nullptr, 0, kp_scale);
        return -a.euler_user().pitch / 20.0f; };
    { const float in4 = share(4.0f, 1.0f, 1.0f), in1 = share(1.0f, 1.0f, 1.0f);
      CHECK(fabsf(in4 - 0.63f) < 0.05f && fabsf(in1 - 0.22f) < 0.04f, "armed: a time constant of 4 s (Betaflight's 0.25 /s) - %.0f %% of the way after 1 s, %.0f %% after 4 s (63 %% = one time constant)", in1 * 100, in4 * 100);
      const float g04 = share(0.4f, 1.0f, 10.0f);
      CHECK(fabsf(g04 - 0.63f) < 0.06f, "ten times that while disarmed: %.0f %% after 0.4 s", g04 * 100); }
    { const float full = share(4.0f, 1.0f, 1.0f);
      const float half_hi = share(4.0f, 1.05f, 1.0f), half_lo = share(4.0f, 0.95f, 1.0f);
      CHECK(fabsf(half_hi - 0.39f) < 0.04f && fabsf(half_lo - 0.39f) < 0.04f && full > half_hi + 0.2f, "a reading of 1.05 g or 0.95 g counts half (%.0f %% / %.0f %% of the way in 4 s; 39 %% = half the gain)", half_hi * 100, half_lo * 100);
      CHECK(fabsf(share(4.0f, 1.11f, 1.0f)) < 0.005f && fabsf(share(4.0f, 0.89f, 1.0f)) < 0.005f && fabsf(share(4.0f, 3.0f, 1.0f)) < 0.005f,
            "from 0.1 g on (a lean that holds its height at 25 deg, a climb, a drop, an impact) not at all: %.1f %% / %.1f %%", share(4.0f, 1.11f, 1.0f) * 100, share(4.0f, 0.89f, 1.0f) * 100);
      Ahrs a; a.update({0, 0, 0}, {0, 0, -1.05f}, 0.001f); const float w_half = a.acc_weight(); a.reset_to_accel({0, 0, -1}); a.update({0, 0, 0}, {0, 0, -1}, 0.001f); const float w_full = a.acc_weight();
      a.update({0, 0, 0}, {0, 0, -1.3f}, 1.0f);
      CHECK(fabsf(w_half - 0.5f) < 0.01f && w_full == 1.0f && a.acc_weight() == 0.0f, "acc_weight() says so: %.2f at 1.05 g, %.0f at 1 g, %.0f at 1.3 g", w_half, w_full, a.acc_weight());
      Ahrs d; d.cfg.accel_lpf_hz = 0; d.reset_to_accel({0, 0, -1}); d.update({0, 0, 0}, {0, 0, -1}, 0.001f); const float before = d.acc_weight(); d.update({0, 0, 0}, {0, 0, 0}, 0.001f);
      CHECK(before == 1.0f && d.acc_weight() == 0.0f && fabsf(d.euler_user().roll) < 1e-3f, "a reading of nothing at all (sensor dead, free fall): weight 0, the horizon stays"); }
    // a sensor that was never calibrated reads 1.08 g lying flat: "1 g" is what it reads at rest
    { Ahrs a; a.update({0, 0, 0}, {0, 0, -1.08f}, 0.001f); const float w0 = a.acc_weight();
      for (int i = 0; i < 1500; i++) a.update({0, 0, 0}, {0, 0, -1.08f}, 0.001f, nullptr, 0, 10.0f, true);
      const float w_rest = a.acc_weight(), learned = a.rest_g();
      for (int i = 0; i < 1000; i++) { a.update({0, 0, 0}, {0, 0, -1.08f * 1.05f}, 0.001f); }
      const float w_lean = a.acc_weight();
      for (int i = 0; i < 1000; i++) { a.update({0, 0, 0}, {0, 0, -1.0f}, 0.001f); }
      const float w_low = a.acc_weight();
      CHECK(fabsf(w0 - 0.2f) < 0.01f && fabsf(learned - 1.08f) < 0.01f && w_rest > 0.95f && fabsf(w_lean - 0.46f) < 0.03f && fabsf(w_low - 0.29f) < 0.03f && fabsf(a.rest_g() - learned) < 1e-6f,
            "it would count %.0f %%; after 1.5 s at rest the 1.08 g are its 1 g (%.3f): %.0f %% lying still, %.0f %% at 5 %% more, and in flight nothing is learned any more", w0 * 100, learned, w_rest * 100, w_lean * 100);
      Ahrs h; h.update({0, 0, 0}, {0, 0, -1.08f}, 0.001f, nullptr, 0, 10.0f, true); for (int i = 0; i < 249; i++) h.update({0, 0, 0}, {0, 0, -1.08f}, 0.001f, nullptr, 0, 10.0f, true);
      CHECK(fabsf(h.rest_g() - (1.0f + 0.08f * (1.0f - expf(-0.5f)))) < 0.004f, "learned with a time constant of half a second (%.3f after 0.25 s)", h.rest_g());
      Ahrs b; for (int i = 0; i < 3000; i++) b.update({0, 0, 0}, {0, 0, -1.25f}, 0.001f, nullptr, 0, 10.0f, true); Ahrs c; for (int i = 0; i < 3000; i++) c.update({0, 0, 0}, {0, 0, -0.75f}, 0.001f, nullptr, 0, 10.0f, true);
      Ahrs d; d.reset_to_accel({0, 0, -1.07f}); Ahrs e; e.reset_to_accel({0, 0, -1.3f});
      CHECK(b.rest_g() == 1.0f && c.rest_g() == 1.0f && fabsf(d.rest_g() - 1.07f) < 1e-5f && d.acc_weight() == 1.0f && e.rest_g() == 1.0f && e.acc_weight() == 0.0f,
            "a reading more than 0.2 g off is never taken for rest (a broken sensor); reset_to_accel() takes the size over as well (%.2f)", d.rest_g()); }
    // vibration: +-1 g at 200 Hz along the thrust axis. Sample by sample the size is anywhere from 0 to 2 g; low-passed it is 1 g
    { auto shaken = [&](float lpf) { Ahrs a; a.cfg.accel_lpf_hz = lpf; a.cfg.ki_acc = 0; float w_sum = 0; const int n = 16000;
          for (int i = 0; i < n; i++) { const float v = 1.0f + 1.0f * sinf(2 * kPi * 200.0f * i * 0.00025f); a.update({0, 0, 0}, {s20 * v, 0, -c20 * v}, 0.00025f); w_sum += a.acc_weight(); }
          return std::pair<float, float>(-a.euler_user().pitch / 20.0f, w_sum / n); };
      const auto with = shaken(10.0f), without = shaken(0.0f);
      CHECK(with.first > 0.58f && with.second > 0.95f && without.second < 0.15f && without.first < 0.2f * with.first,
            "a frame shaking +-1 g at 200 Hz: low-passed the reading still counts %.0f %% (%.0f %% of the way in 4 s); sample by sample it would count %.0f %% (%.0f %%)", with.second * 100, with.first * 100, without.second * 100, without.first * 100); }
    { Ahrs a; a.reset_to_accel({0, 0, -1}); float worst = 0;
      for (int i = 0; i < 16000; i++) { a.update({0, 0, 0}, {0, 0.6f * sinf(2 * kPi * 150.0f * i * 0.00025f), -1}, 0.00025f); worst = fmaxf(worst, fabsf(a.euler_user().roll)); }
      CHECK(worst < 0.1f, "shaking +-0.6 g sideways at 150 Hz does not rock the horizon (%.3f deg at most)", worst);
      // the low-pass itself: second order at 10 Hz - half the amplitude at 10 Hz, a hundredth at 100 Hz
      auto through = [](float hz) { Ahrs b; b.reset_to_accel({0, 0, -1}); float peak = 0; for (int i = 0; i < 8000; i++) { b.update({0, 0, 0}, {0, 0.1f * sinf(2 * kPi * hz * i * 0.00025f), -1}, 0.00025f); if (i > 4000) peak = fmaxf(peak, fabsf(b.accel_euler_aero().roll)); }
          return tanf(peak * kDeg2Rad) / 0.1f; };
      CHECK(fabsf(through(10.0f) - 0.5f) < 0.02f && fabsf(through(100.0f) - 0.0099f) < 0.001f && fabsf(through(1.0f) - 0.99f) < 0.01f,
            "the low-pass: second order at 10 Hz - a sway at 1 Hz passes (%.2f), at 10 Hz half of it (%.2f), of a 100 Hz vibration a hundredth (%.4f)", through(1.0f), through(10.0f), through(100.0f)); }
    // the gyro's bias is learned - slowly: a real gyro drifts by fractions of a degree per second over minutes
    { Ahrs a; a.reset_to_accel({0, 0, -1}); float err40 = 0;
      for (int i = 0; i < 180000; i++) { a.update({0.3f * kDeg2Rad, 0, 0}, {0, 0, -1}, 0.001f); if (i == 39999) err40 = a.euler_user().roll; }
      Ahrs n; n.cfg.ki_acc = 0; n.reset_to_accel({0, 0, -1}); for (int i = 0; i < 180000; i++) n.update({0.3f * kDeg2Rad, 0, 0}, {0, 0, -1}, 0.001f);
      CHECK(fabsf(n.euler_user().roll - 1.2f) < 0.1f && fabsf(a.euler_user().roll) < 0.15f && fabsf(a.gyro_bias_dps().x - 0.3f) < 0.04f && fabsf(a.gyro_bias_dps().y) < 0.01f,
            "a gyro reading 0.3 deg/s too high would tilt the horizon %.1f deg for good; the bias is learned (%.2f deg/s after 3 minutes) and the horizon is right again (%.2f deg; %.2f after 40 s)", n.euler_user().roll, a.gyro_bias_dps().x, a.euler_user().roll, err40); }
    { Ahrs a; a.reset_to_accel({0, 0, -1}); for (int i = 0; i < 4000; i++) a.update({0, 0, 0}, {0, -s20, -c20}, 0.001f);       // 4 s of a reading 20 deg off (a manoeuvre)
      Ahrs f; f.cfg.bias_slew_dps_min = 0; f.reset_to_accel({0, 0, -1}); for (int i = 0; i < 4000; i++) f.update({0, 0, 0}, {0, -s20, -c20}, 0.001f);
      CHECK(fabsf(a.gyro_bias_dps().x) > 0.025f && fabsf(a.gyro_bias_dps().x) < 0.036f && fabsf(f.gyro_bias_dps().x) > 0.2f,
            "4 s of a reading 20 deg off teach it %.3f deg/s of bias (0.5 deg/s per minute at most = 0.033); unlimited it would be %.2f deg/s, a horizon %.1f deg off afterwards", fabsf(a.gyro_bias_dps().x), fabsf(f.gyro_bias_dps().x), fabsf(f.gyro_bias_dps().x) / 0.25f);
      Ahrs z; z.reset_to_accel({0, 0, -1}); for (int i = 0; i < 4000; i++) z.update({0, 0, 0}, {1.3f * s20, -1.3f * s20, -1.3f * c20}, 0.001f);      // the same, but the reading is 1.3 g: it does not count
      CHECK(fabsf(z.gyro_bias_dps().x) < 0.003f && fabsf(z.gyro_bias_dps().y) < 0.003f && z.acc_weight() == 0.0f, "a reading that does not count teaches nothing (%.4f / %.4f deg/s from 4 s of 1.3 g, 28 deg off)", z.gyro_bias_dps().x, z.gyro_bias_dps().y);
      Ahrs b; b.reset_to_accel({0, 0, -1}); float at3 = 0; for (int i = 0; i < 120000; i++) { b.update({6.0f * kDeg2Rad, 0, 0}, {0, 0, -1}, 0.01f); if (i == 17999) at3 = b.gyro_bias_dps().x; }   // a gyro 6 deg/s off, 20 minutes
      CHECK(fabsf(at3 - 1.5f) < 0.02f && fabsf(b.gyro_bias_dps().x - 3.0f) < 0.01f, "and never more than 3 deg/s, whatever happens (a gyro 6 deg/s off: %.2f after 3 minutes, %.2f after 20)", at3, b.gyro_bias_dps().x); }
    // disarmed the reading is gravity - when the craft lies still (Betaflight's schedule)
    { AccelGainSchedule g; struct Seen { float first = -1, last = -1, fast_s = 0; };
      auto run = [&](float dur, bool armed, float gyro, bool ok) { Seen o; for (int i = 0; i < (int)(dur * 1000 + 0.5f); i++) { o.last = g.update(armed, gyro, ok, 0.001f); if (o.first < 0) o.first = o.last; if (o.last == 100.0f) o.fast_s += 0.001f; } return o; };
      const Seen boot = run(2.0f, false, 0.5f, true);
      CHECK(boot.first == 100.0f && fabsf(boot.fast_s - 0.5f) < 0.002f && boot.last == 10.0f && g.settled(), "after power-up: x 100 for half a second (nothing is known yet; %.3f s), then x 10", boot.fast_s);
      const Seen armed = run(2.0f, true, 300.0f, true);
      CHECK(armed.first == 1.0f && armed.last == 1.0f && armed.fast_s == 0 && !g.settled(), "armed: x 1, whatever the gyro says");
      const Seen quiet = run(0.24f, false, 3.0f, true), edge = run(0.02f, false, 3.0f, true), fast = run(0.47f, false, 3.0f, true), after = run(1.0f, false, 3.0f, true);
      CHECK(quiet.first == 1.0f && quiet.last == 1.0f && quiet.fast_s == 0 && edge.last == 100.0f && fast.first == 100.0f && fast.last == 100.0f && fabsf(edge.fast_s + fast.fast_s + after.fast_s - 0.5f) < 0.002f && after.last == 10.0f,
            "disarmed: x 1 for the first quarter second, then x 100 for half a second (the horizon is set afresh), then x 10");
      const Seen moved = run(0.3f, false, 16.0f, true), slow = run(0.24f, false, 14.0f, true), again = run(0.3f, false, 2.0f, true);
      CHECK(moved.first == 1.0f && moved.last == 1.0f && moved.fast_s == 0 && slow.last == 1.0f && slow.fast_s == 0 && again.last == 100.0f && fabsf(again.fast_s - 0.29f) < 0.002f,
            "picked up (turning faster than 15 deg/s): x 1 as long as it moves and a quarter second longer, then set afresh again");
      const Seen bump = run(0.05f, false, 2.0f, false), wait = run(0.24f, false, 2.0f, true), redo = run(2.0f, false, 2.0f, true);
      CHECK(bump.first == 1.0f && bump.last == 1.0f && wait.last == 1.0f && wait.fast_s == 0 && fabsf(redo.fast_s - 0.5f) < 0.002f && redo.last == 10.0f,
            "a reading that is not 1 g (shaken, dropped) in the middle of that: the wait and the half second start over"); }
    { Ahrs a; AccelGainSchedule g; a.reset_to_accel({0, 0, -1}); for (int i = 0; i < 1000; i++) a.update({0, 0, 0}, {0, 0, -1}, 0.001f, nullptr, 0, g.update(true, 0, true, 0.001f));
      for (int i = 0; i < 500; i++) a.update({120 * kDeg2Rad, 0, 0}, {0, 0, -3.0f}, 0.001f, nullptr, 0, g.update(true, 120, a.acc_weight() > 0, 0.001f));      // a crash: the estimate ends up 60 deg from the truth
      const float wrong = a.euler_user().roll; float at_02 = 0;
      for (int i = 0; i < 800; i++) { a.update({0, 0, 0}, {0, 0, -1}, 0.001f, nullptr, 0, g.update(false, 0.3f, a.acc_weight() > 0, 0.001f)); if (i == 199) at_02 = a.euler_user().roll; }
      CHECK(fabsf(wrong) > 50 && fabsf(at_02) > 45 && fabsf(a.euler_user().roll) < 0.5f, "a crash leaves the horizon %.0f deg wrong; disarmed and lying still it is right again after 0.8 s (%.2f deg; still %.0f after 0.2 s)", wrong, a.euler_user().roll, at_02); }
    // what the accelerometer alone says (the in-flight level trim measures this)
    { Ahrs a; a.cfg.ki_acc = 0; a.reset_to_accel({0, 0, -1}); for (int i = 0; i < 2000; i++) a.update({0, 0, 0}, {-sinf(3 * kDeg2Rad), sinf(5 * kDeg2Rad), -1}, 0.001f, nullptr, 0, 0.0f);
      const EulerDeg e = a.accel_euler_aero();
      CHECK(fabsf(a.euler_aero().roll) < 0.01f && fabsf(e.roll + 5.0f) < 0.05f && fabsf(e.pitch + 3.0f) < 0.05f, "accel_euler_aero(): the low-passed accelerometer by itself (roll %.2f, pitch %.2f for a reading 5 deg left wing down, 3 deg nose down) while the estimate says %.2f", e.roll, e.pitch, a.euler_aero().roll);
      a.reset_to_accel({0, sinf(10 * kDeg2Rad), -cosf(10 * kDeg2Rad)});
      CHECK(fabsf(a.accel_euler_aero().roll + 10.0f) < 0.01f && fabsf(a.euler_aero().roll + 10.0f) < 0.01f, "reset_to_accel() takes the reading over at once, in the low-pass too (%.2f)", a.accel_euler_aero().roll); }
}

static void t_vertical() {
    printf("[vertical estimator]\n");
    const float dt = 0.005f, bias = 0.3f;
    { VerticalEstimator kf; kf.reset(2.0f); float drift = 0, maxerr = 0, maxv = 0;
      for (int i = 0; i < 12000; i++) { const float t = i * dt; drift = 0.05f * t; kf.predict(bias + gauss(0.3f), dt);
          if (i % 20 == 0) kf.update_baro(2.0f + drift + gauss(0.3f), 0.3f);
          if (i % 40 == 0) kf.update_range(2.0f + gauss(0.03f), 0.03f);
          if (t > 15) { maxerr = fmaxf(maxerr, fabsf(kf.state().z - 2.0f)); maxv = fmaxf(maxv, fabsf(kf.state().v)); } }
      CHECK(maxerr < 0.15f && maxv < 0.2f, "baro drifting 5 cm/s + lidar: height error %.3f, |v| %.3f (the old firmware ramped to -95 cm/s)", maxerr, maxv);
      CHECK(fabsf(kf.baro_offset() - 3.0f) < 0.4f && fabsf(kf.accel_bias() - bias) < 0.15f, "learned baro offset %.2f, accel bias %.2f", kf.baro_offset(), kf.accel_bias()); }
    { VerticalEstimator kf; kf.reset(0.0f); float maxv = 0;
      for (int i = 0; i < 6000; i++) { kf.predict(gauss(0.3f), dt); if (i % 20 == 0) kf.update_baro(3.0f + gauss(0.3f), 0.3f); if (i % 40 == 0) kf.update_range(gauss(0.03f), 0.03f); if (i > 2000) maxv = fmaxf(maxv, fabsf(kf.state().v)); }
      CHECK(maxv < 0.15f && fabsf(kf.state().z) < 0.15f, "baro/lidar disagree by 3 m: follows the lidar, no velocity ramp"); }
    { VerticalEstimator kf; kf.reset(1.0f); for (int i = 0; i < 2000; i++) { kf.predict(0, dt); if (i % 20 == 0) kf.update_range(1.0f + gauss(0.03f), 0.03f); }
      const bool acc = kf.update_range(6.0f, 0.03f); CHECK(!acc && fabsf(kf.state().z - 1.0f) < 0.1f, "lidar outlier gated"); }
    { VerticalEstimator kf; kf.reset(0); float z = 0, v = 0; for (int i = 0; i < 1600; i++) { const float t = i * dt; const float a = (t < 1) ? 1.0f : 0.0f; if (t < 1) v += a * dt; z += v * dt;
          kf.predict(a + gauss(0.2f), dt); if (i % 20 == 0) kf.update_baro(z + gauss(0.3f), 0.3f); }
      CHECK(fabsf(kf.state().z - z) < 0.5f, "tracks a climb: est %.2f true %.2f", kf.state().z, z); }
    { VerticalEstimator kf; kf.reset(0); for (int i = 0; i < 4000; i++) { kf.predict(0, dt); if (i % 20 == 0) kf.update_baro(gauss(0.05f), 0.05f); }
      VerticalEstimator kg; kg.reset(0); for (int i = 0; i < 4000; i++) { kg.predict(0, dt); if (i % 20 == 0) kg.update_baro(gauss(0.05f), 5.0f); }
      CHECK(kf.z_sigma() < kg.z_sigma(), "a smaller sigma from the sensor layer gives a tighter estimate (%.3f vs %.3f)", kf.z_sigma(), kg.z_sigma()); }

    // ---- the estimate runs away (an accelerometer that clipped on a hard landing), the barometer stays at 0
    // (the generator is put back afterwards: the tests below were tuned on its sequence)
    const std::mt19937 rng_saved = rng;
    { VerticalEstimator kf; kf.reset(0); const float dt2 = 0.0005f; int n_ok = 0; float worst_after = 0;
      for (int i = 0; i < 4000; i++) { kf.predict(gauss(0.1f), dt2); if (i % 40 == 0) kf.update_baro(gauss(0.2f), 0.3f); }
      for (int i = 0; i < 200; i++) kf.predict(-40.0f, dt2);                          // 0.1 s of -4 g: v -4 m/s
      for (int i = 0; i < 6000; i++) kf.predict(gauss(0.1f), dt2);                   // ... and 3 s without a baro reading: z -12 m
      const float v_kick = kf.state().v; const uint32_t r0 = kf.baro_resets();
      for (int i = 0; i < 8000; i++) { kf.predict(gauss(0.1f), dt2); if (i % 40 == 0 && kf.update_baro(gauss(0.2f), 0.3f)) n_ok++;
          if (i * dt2 > 2.0f) worst_after = fmaxf(worst_after, fabsf(kf.state().z)); }
      CHECK(v_kick < -3.0f && kf.baro_resets() >= r0 + 1 && kf.baro_resets() <= r0 + 2 && worst_after < 0.6f && fabsf(kf.state().v) < 0.3f,
            "kicked to %.1f m/s: after %.1f s of rejected baro readings z is reset to the baro (resets %u, |z| after 2 s <= %.2f m, v %.2f) - the old filter ran to -290 m",
            v_kick, kf.cfg.baro_reset_s, (unsigned)(kf.baro_resets() - r0), worst_after, kf.state().v); }
    { VerticalEstimator kf; kf.reset(0); const float dt2 = 0.0005f; float maxv = 0;
      for (int i = 0; i < 40000; i++) { kf.predict(0.3f, dt2); if (i % 40 == 0) { kf.update_baro(gauss(0.5f), 1.0f); kf.update_still(); } if (i > 20000) maxv = fmaxf(maxv, fabsf(kf.state().v)); }
      CHECK(maxv < 0.15f && kf.baro_resets() <= 1, "disarmed with a 0.3 m/s^2 accel error and a noisy baro: the zero-velocity update keeps |v| <= %.2f", maxv); }
    for (int still = 0; still < 2; still++) {                // the touch-down spike, disarmed: the zero-velocity update takes it out at once
      VerticalEstimator kf; kf.reset(0); const float dt2 = 0.0005f;
      for (int i = 0; i < 4000; i++) { kf.predict(0, dt2); if (i % 40 == 0) { kf.update_baro(gauss(0.1f), 0.3f); if (still) kf.update_still(); } }
      for (int i = 0; i < 200; i++) kf.predict(-40.0f, dt2);
      for (int i = 0; i < 400; i++) { kf.predict(0, dt2); if (i % 40 == 0) { kf.update_baro(gauss(0.1f), 0.3f); if (still) kf.update_still(); } }
      if (still) CHECK(fabsf(kf.state().v) < 0.5f && fabsf(kf.state().z) < 0.5f, "a -4 m/s spike at disarm: 0.2 s later v %.2f m/s, z %.2f m (the zero-velocity update is not gated)", kf.state().v, kf.state().z);
      else CHECK(kf.state().v < -2.0f, "(without it the same spike is still at %.1f m/s 0.2 s later)", kf.state().v); }
    rng = rng_saved;
    // ---- with a barometer, the range finder measures from the ground under the craft - and that ground changes
    // the craft moves from z0 to z1 (above the take-off point) in `seconds`, over ground lying `ground` above the take-off point
    auto fly = [&](VerticalEstimator& kf, float z0, float z1, float ground, float seconds, bool baro = true, bool range = true, bool relative = true) {
        const int n = (int)(seconds / dt + 0.5f);
        for (int i = 0; i < n; i++) { const float z = z0 + (z1 - z0) * (float)(i + 1) / (float)n; kf.predict(gauss(0.1f), dt);
            if (baro && i % 4 == 0) kf.update_baro(z + gauss(0.2f), 0.3f);
            if (range && i % 8 == 0) kf.update_range(z - ground + gauss(0.02f), 0.03f, relative); } };
    { VerticalEstimator kf; kf.reset(0); fly(kf, 1.5f, 1.5f, 0.0f, 10.0f);
      CHECK(fabsf(kf.state().z - 1.5f) < 0.05f && kf.range_reanchors() == 0 && kf.range_anchor() == 0.0f && kf.state().range_used, "over the ground it took off from, the range finder gives the height itself (%.3f of 1.5 m, anchor %.2f)", kf.state().z, kf.range_anchor());
      // a table 0.5 m high comes under it: the readings drop to 1.0 m
      const float z0 = kf.state().z; bool any = false; for (int i = 0; i < 9; i++) { kf.predict(0, dt * 8); any |= kf.update_range(1.0f, 0.03f, true); }
      CHECK(!any && kf.range_reanchors() == 0 && kf.range_anchor() == 0.0f && fabsf(kf.state().z - z0) < 0.02f, "a table 0.5 m high comes under it: nine readings in a row do not fit - not believed, not used (z %.3f)", kf.state().z);
      kf.predict(0, dt * 8); const bool tenth = kf.update_range(1.0f, 0.03f, true);
      CHECK(!tenth && kf.range_reanchors() == 1 && fabsf(kf.range_anchor() - (1.0f - kf.state().z)) < 1e-5f && fabsf(kf.state().z - z0) < 0.02f,
            "the tenth: the GROUND is taken to have changed (anchor %.2f m), the height has not (%.3f m) - fused as a height it would have dropped half a metre, and Altitude Hold would have climbed by as much", kf.range_anchor(), kf.state().z);
      kf.predict(0, dt * 8); kf.update_range(0.4f, 0.03f, true);
      CHECK(kf.range_reanchors() == 1, "one odd reading right after that (a hand): it is the first of a new count, the anchor stays");
      kf.predict(0, dt * 8); CHECK(kf.update_range(1.0f, 0.03f, true), "the next reading fits and is used");
      fly(kf, 1.5f, 1.5f, 0.5f, 4.0f); CHECK(fabsf(kf.state().z - 1.5f) < 0.07f && kf.range_reanchors() == 1, "hovering over the table: z %.3f", kf.state().z);
      fly(kf, 1.5f, 2.1f, 0.5f, 3.0f); CHECK(fabsf(kf.state().z - 2.1f) < 0.10f && kf.range_reanchors() == 1, "a climb of 0.2 m/s over the table is a climb, not a change of the ground: z %.3f of 2.1, still one anchor move", kf.state().z);
      fly(kf, 2.1f, 2.1f, 0.0f, 2.0f); CHECK(fabsf(kf.state().z - 2.1f) < 0.10f && kf.range_reanchors() == 2 && fabsf(kf.range_anchor()) < 0.12f, "off the table again: the anchor is back at the floor (%.2f m), z %.3f", kf.range_anchor(), kf.state().z); }
    { VerticalEstimator kf; kf.reset(0); fly(kf, 1.5f, 1.5f, 0.0f, 10.0f);          // a hand under the sensor now and then
      for (int k = 0; k < 2; k++) { for (int i = 0; i < 9; i++) { kf.predict(0, dt * 8); kf.update_range(0.4f, 0.03f, true); } kf.predict(0, dt * 8); kf.update_range(kf.state().z, 0.03f, true); }
      CHECK(kf.range_reanchors() == 0, "nine odd readings, a good one, nine odd ones: the count starts again with every reading that fits"); }
    { VerticalEstimator kf; kf.reset(0); fly(kf, 1.5f, 1.5f, 0.0f, 10.0f); fly(kf, 1.5f, 1.5f, 0.0f, 0.4f, true, false);
      const bool used = kf.state().range_used; kf.update_range(1.0f, 0.03f, true);
      CHECK(used && kf.range_reanchors() == 0, "no reading for 0.4 s (a steep bank), then one that does not fit: still the same stretch of ground - it only counts as an odd one");
      VerticalEstimator kg; kg.reset(0); fly(kg, 1.5f, 1.5f, 0.0f, 10.0f); fly(kg, 1.5f, 1.5f, 0.0f, 0.6f, true, false);
      const bool stale = !kg.state().range_used; const float z0 = kg.state().z; const bool first = kg.update_range(1.0f, 0.03f, true);
      CHECK(stale && !first && kg.range_reanchors() == 1 && kg.state().z == z0 && fabsf(kg.range_anchor() - (1.0f - z0)) < 1e-5f, "no reading for 0.6 s: the next one starts from a new anchor at once, whatever it says - z does not move (%.3f)", kg.state().z);
      kg.predict(0, dt * 8); CHECK(kg.update_range(1.0f, 0.03f, true) && kg.state().range_used, "... and the readings after it are used"); }
    { VerticalEstimator kf; kf.reset(0); fly(kf, 1.5f, 1.5f, 0.0f, 6.0f, false, true, false);          // a craft without barometer
      fly(kf, 1.5f, 1.5f, 0.5f, 6.0f, false, true, false);
      CHECK(kf.range_reanchors() == 0 && kf.range_anchor() == 0.0f && fabsf(kf.state().z - 1.0f) < 0.1f, "without a barometer the range finder is the only height there is: no anchor, z follows it (%.3f, readings 1.0)", kf.state().z);
      VerticalEstimator kg; kg.reset(0); fly(kg, 1.5f, 1.5f, 0.0f, 6.0f, false, true, false); fly(kg, 1.5f, 1.5f, 0.0f, 2.0f, false, false); kg.update_range(1.0f, 0.03f, false);
      CHECK(kg.range_reanchors() == 0, "... nor after a pause"); }
    // ---- what the barometer's own noise figure may say
    { auto run = [&](float sigma, bool ground_effect) { VerticalEstimator k; k.reset(0);
          for (int i = 0; i < 2000; i++) { k.predict(0.05f * sinf(i * 0.01f), dt); if (i % 4 == 0) k.update_baro(0.3f * sinf(i * 0.003f) + 0.1f * sinf(i * 0.7f), sigma, ground_effect); } return k.state().z; };
      CHECK(run(0.05f, false) == run(0.3f, false) && run(0.29f, false) == run(0.3f, false) && run(0.31f, false) != run(0.3f, false), "a barometer that reports 5 cm of noise (a quiet minute indoors) is taken at 30 cm: the air in flight is not that quiet");
      CHECK(run(3.0f, false) == run(1.0f, false) && run(0.99f, false) != run(1.0f, false), "one that reports 3 m (the craft was carried about) at 1 m");
      CHECK(run(0.0f, false) == run(0.4f, false), "none reported: the configured 0.4 m");
      CHECK(run(0.3f, true) == run(0.6f, false) && run(0.3f, true) != run(0.3f, false), "near the ground with the motors running it counts half (sigma x 2)"); }
    // ---- odd calls
    { VerticalEstimator kf; kf.reset(1.0f); kf.predict(2.0f, NAN); kf.predict(2.0f, 0.2f); kf.predict(2.0f, -0.01f); kf.predict(2.0f, 0.0f);
      CHECK(kf.state().z == 1.0f && kf.state().v == 0.0f && !kf.state().range_used && !kf.state().gps_used, "a time step that is not a number, too long, negative or zero changes nothing");
      kf.update_gps(1.0f, 2.0f); const bool now = kf.state().gps_used; for (int i = 0; i < 500; i++) kf.predict(0, dt);
      CHECK(now && !kf.state().gps_used, "'GPS used' means within the last two seconds, not ever"); }
}

// GPS in the height estimate (vertical_est.h "GPS"): vertical velocity, the height as the only source, blended with a drifting barometer
static void t_vertical_gps() {
    printf("[vertical estimator: GPS]\n");
    const float dt = 0.005f;
    // GPS height error: wanders (first-order, ~1 m, 30 s) plus 0.5 m of noise on every reading - what a u-blox does outdoors
    struct Wander { float e = 0; float next(float step_s) { e += -e * step_s / 30.0f + gauss(1.0f * sqrtf(2.0f * step_s / 30.0f)); return e + gauss(0.5f); } };
    // climbs and descents of +-3 m/s, a 0.3 m/s^2 accelerometer bias, a noisy barometer (prop wash): RMS error of v
    auto climbs = [&](bool baro, bool gps_vel, float& z_rms) {
        VerticalEstimator kf; kf.reset(0); float z = 0, v = 0, e2 = 0, ez2 = 0; int n = 0;
        for (int i = 0; i < 12000; i++) { const float t = i * dt, a = 3.0f * 0.6f * cosf(0.6f * t); v += a * dt; z += v * dt;
            kf.predict(a + 0.3f + gauss(0.3f), dt);
            if (baro && i % 8 == 0) kf.update_baro(z + gauss(0.6f), 0.6f);
            if (gps_vel && i % 20 == 0) kf.update_gps_vel(v + gauss(0.1f), 0.15f);
            if (t > 10) { e2 += (kf.state().v - v) * (kf.state().v - v); ez2 += (kf.state().z - z) * (kf.state().z - z); n++; } }
        z_rms = sqrtf(ez2 / n); return sqrtf(e2 / n); };
    { float zb, zg, zv; const float only_baro = climbs(true, false, zb), with_gps = climbs(true, true, zg), gps_alone = climbs(false, true, zv);
      CHECK(with_gps < 0.6f * only_baro && with_gps < 0.12f, "climbing and descending at 3 m/s: climb-rate error %.3f m/s with the GPS vertical velocity, %.3f with the barometer alone", with_gps, only_baro);
      CHECK(zg <= zb * 1.05f, "... and the height is no worse for it (%.2f vs %.2f m RMS)", zg, zb);
      CHECK(gps_alone < 0.15f, "the GPS vertical velocity alone (no height at all) keeps the climb rate: %.3f m/s RMS", gps_alone); }
    { VerticalEstimator kf; kf.reset(0); for (int i = 0; i < 2000; i++) { kf.predict(0, dt); if (i % 8 == 0) kf.update_baro(gauss(0.3f), 0.3f); if (i % 20 == 0) kf.update_gps_vel(gauss(0.1f), 0.15f); }
      const bool took = kf.update_gps_vel(8.0f, 0.15f);
      CHECK(!took && fabsf(kf.state().v) < 0.1f && kf.state().gps_vel_used, "a GPS vertical velocity of 8 m/s while hovering is gated out"); }

    // hovering at 10 m for 5 minutes, the barometer drifting 1 m a minute (weather, the board warming)
    auto hover = [&](int mode /*0 baro alone, 1 blend*/, float gps_offset, float jump_at_s, float jump_m, float& worst) {
        VerticalEstimator kf; kf.reset(10.0f); Wander w; worst = 0;
        for (int i = 0; i < 60000; i++) { const float t = i * dt, drift = t / 60.0f;
            kf.predict(gauss(0.3f), dt);
            if (i % 8 == 0) kf.update_baro(10.0f + drift + gauss(0.3f), 0.3f);
            if (mode && i % 20 == 0) { const float g = 10.0f + gps_offset + w.next(0.1f) + (jump_at_s > 0 && t >= jump_at_s ? jump_m : 0.0f);
                kf.update_gps_relative(g, fmaxf(1.5f, kf.cfg.gps_blend_sigma_min), true); }
            if (t > 30) worst = fmaxf(worst, fabsf(kf.state().z - 10.0f)); }
        return fabsf(kf.state().z - 10.0f); };
    { float wb, wg; const float baro = hover(0, 0, 0, 0, wb), blend = hover(1, 0, 0, 0, wg);
      CHECK(baro > 4.0f && blend < 1.5f && wg < 2.0f, "baro drifting 1 m/min for 5 min: off by %.1f m alone, by %.2f m with the GPS blended in (worst %.2f after the first 30 s)", baro, blend, wg); }
    { float w; hover(1, 25.0f, 0, 0, w); float w0; hover(1, 0, 0, 0, w0);
      CHECK(w < w0 + 0.3f, "a GPS height 25 m off from the start (home taken long ago, MSL vs. take-off): the reference takes it out (worst %.2f vs %.2f m)", w, w0); }
    { auto jump = [&](float gate_m, uint32_t& resets) { VerticalEstimator kf; kf.cfg.gps_jump_m = gate_m; kf.reset(10.0f); Wander w; float worst = 0;
          for (int i = 0; i < 24000; i++) { const float t = i * dt; kf.predict(gauss(0.3f), dt); if (i % 8 == 0) kf.update_baro(10.0f + gauss(0.3f), 0.3f);
              if (i % 20 == 0) kf.update_gps_relative(10.0f + w.next(0.1f) + (t >= 60.0f ? 8.0f : 0.0f), 3.0f, true);
              if (t >= 60) worst = fmaxf(worst, fabsf(kf.state().z - 10.0f)); }
          resets = kf.gps_resets(); return worst; };
      uint32_t r1 = 0, r2 = 0; const float kept = jump(4.0f, r1), dragged = jump(1e6f, r2);
      CHECK(kept < 1.0f && r1 >= 1 && dragged > 2.5f, "the GPS height jumps by 8 m and stays there: a new reference, the estimate stays within %.2f m (resets %u); taken as a correction it would have moved %.1f m", kept, (unsigned)r1, dragged); }
    { VerticalEstimator kf; kf.reset(0); int fused = 0;
      for (int i = 0; i < 4000; i++) { kf.predict(0, dt); if (i % 8 == 0) kf.update_baro(gauss(0.3f), 0.3f); if (i % 20 == 0) fused += kf.update_gps_relative(gauss(0.5f), 1.0f, true); }
      const int before = fused; for (int i = 0; i < 2000; i++) { kf.predict(0, dt); if (i % 8 == 0) kf.update_baro(gauss(0.3f), 0.3f); }      // 10 s without GPS
      kf.update_gps_relative(5.0f, 1.0f, true);
      CHECK(before == 200 - kf.cfg.gps_ref_samples && !kf.gps_ref_ok() && fabsf(kf.state().z) < 0.3f, "the first %d readings only set the reference (%d of 200 fused); after 10 s without GPS it starts again", kf.cfg.gps_ref_samples, before); }

    // the barometer goes silent in flight: the GPS takes over from the reference it kept (FALLBACK), no step
    { VerticalEstimator kf; kf.reset(0); Wander w; float z = 0, at = 0, worst = 0, end = 0;
      for (int i = 0; i < 36000; i++) { const float t = i * dt; z = t < 20 ? t * 0.5f : 10.0f; const bool baro = t < 60;
          kf.predict((t < 20 ? 0.0f : 0.0f) + gauss(0.3f), dt);
          if (baro && i % 8 == 0) kf.update_baro(z + t / 60.0f + gauss(0.3f), 0.3f);
          if (i % 20 == 0) { const float g = z + w.next(0.1f); if (baro) kf.update_gps_relative(g, 1.5f, false); else kf.update_gps_relative(g, 1.5f, true); }
          if (i % 20 == 0) kf.update_gps_vel((t < 20 ? 0.5f : 0.0f) + gauss(0.1f), 0.15f);
          if (fabsf(t - 59.99f) < dt) at = kf.state().z;
          if (t > 60 && t < 62) worst = fmaxf(worst, fabsf(kf.state().z - at));
          end = kf.state().z - z; }
      CHECK(worst < 0.5f && kf.state().gps_used && fabsf(end) < 3.0f, "baro silent after 60 s: the estimate moves %.2f m in the next 2 s (the reference followed GPS - z), %.1f m off truth after 2 more minutes on GPS alone", worst, end); }

    // a craft with GPS and nothing else: the GPS height IS the height
    { VerticalEstimator kf; kf.reset(0); Wander w; float worst = 0;
      for (int i = 0; i < 12000; i++) { const float t = i * dt; kf.predict(gauss(0.3f), dt);
          if (i % 20 == 0) { kf.update_gps(5.0f + w.next(0.1f), 1.5f); kf.update_gps_vel(gauss(0.1f), 0.15f); }
          if (t > 10) worst = fmaxf(worst, fabsf(kf.state().z - 5.0f)); }
      CHECK(worst < 3.5f && kf.z_sigma() < 1.5f, "GPS alone, hovering at 5 m for a minute: within %.2f m (sigma %.2f)", worst, kf.z_sigma());
      for (int i = 0; i < 400; i++) { kf.predict(gauss(0.3f), dt); if (i % 20 == 0) kf.update_gps(35.0f, 1.5f); }
      CHECK(kf.gps_resets() == 1 && fabsf(kf.state().z - 35.0f) < 0.5f, "it jumps by 30 m and stays: after %d readings outside the gate the height is set to it (nothing better exists)", kf.cfg.gps_reref_after); }
}

static void t_horizontal() {
    printf("[horizontal estimator]\n");
    auto run = [&](float delay_cfg, bool maneuver, float outage_from, float outage_to, float& pos_err, float& vel_err, float& max_pos_err) {
        HorizontalEstimator h; h.cfg.gps_delay_s = delay_cfg; const float dt = 0.002f, gps_dt = 0.1f, true_delay = 0.15f;
        auto truth = [&](double t, double& p, double& v, double& a) { if (maneuver) { v = 5 + 3 * sin(0.5 * t); a = 1.5 * cos(0.5 * t); p = 5 * t - 6 * cos(0.5 * t) + 6; } else { v = 5; a = 0; p = 5 * t; } };
        h.reset(0, 0, 5, 0); double next_gps = 0; max_pos_err = 0; pos_err = vel_err = 0;
        for (int i = 0; i < 15000; i++) {
            const double t = i * dt; double p, v, a; truth(t, p, v, a);
            h.predict((float)(a + 0.3 + gauss(0.2f)), 0.0f + gauss(0.2f), dt);                    // north accel bias 0.3 m/s^2
            if (t >= next_gps) { next_gps += gps_dt; double pp, vv, aa; truth(t - true_delay, pp, vv, aa);
                if (!(t > outage_from && t < outage_to)) h.fuse_gps((float)(pp + gauss(0.8f)), gauss(0.8f), (float)(vv + gauss(0.1f)), gauss(0.1f), 0.8f, 0.1f); }
            if (t > 10) { pos_err = fabsf(h.pos_n() - (float)p); vel_err = fabsf(h.vel_n() - (float)v); max_pos_err = fmaxf(max_pos_err, pos_err); }
        }
    };
    float pe, ve, mpe;
    run(0.15f, false, 1e9, 1e9, pe, ve, mpe); CHECK(mpe < 1.5f && ve < 0.3f, "cruise at 5 m/s with noisy delayed GPS and an accel bias: max pos error %.2f m, vel error %.2f m/s", mpe, ve);
    float pe0, ve0, mpe0; run(0.0f, true, 1e9, 1e9, pe0, ve0, mpe0); float pe1, ve1, mpe1; run(0.15f, true, 1e9, 1e9, pe1, ve1, mpe1);
    CHECK(mpe1 < mpe0 * 0.8f, "latency compensation helps while accelerating: %.2f m vs %.2f m without", mpe1, mpe0);
    run(0.15f, false, 6.0f, 9.0f, pe, ve, mpe); CHECK(mpe < 4.0f, "3 s GPS outage bridged by prediction: max error %.2f m", mpe);
    { HorizontalEstimator h; h.reset(); for (int i = 0; i < 500; i++) { h.predict(0, 0, 0.002f); if (i % 50 == 0) h.fuse_gps(0, 0, 0, 0, 0.8f, 0.1f); }
      h.fuse_gps(60.0f, 0, 0, 0, 0.8f, 0.1f); CHECK(fabsf(h.pos_n()) < 1.0f, "a 60 m GPS glitch is rejected (%.2f)", h.pos_n()); }
    { HorizontalEstimator h; h.reset(); h.fuse_gps(10, -4, 1, 2, 1.0f, 0.2f); CHECK(h.initialised() && fabsf(h.pos_n() - 10) < 0.5f && fabsf(h.vel_e() - 2) < 0.2f, "first fix initialises the state"); }
}

static void t_rc() {
    printf("[rc]\n");
    RateProfile p; p.center_sens_dps[0] = 70; p.max_rate_dps[0] = 670; p.expo[0] = 0;
    CHECK(fabsf(apply_actual_rate(p, ROLL, 1.0f) - 670) < 0.01f && fabsf(apply_actual_rate(p, ROLL, -1.0f) + 670) < 0.01f && fabsf(apply_actual_rate(p, ROLL, 0.0f)) < 1e-6f, "Actual rates: full/centre/symmetry");
    CHECK(fabsf(apply_actual_rate(p, ROLL, 0.1f) - 13.0f) < 0.01f, "0.1 stick = 13 dps (70*0.1 + 600*0.01)");
    p.expo[0] = 1.0f; CHECK(fabsf(apply_actual_rate(p, ROLL, 0.5f) - 44.375f) < 0.01f, "expo 1");
    RcInterpreter rc; rc.modes.add(M_ARM, 0, 1700, 2100); rc.modes.add(M_ALTHOLD, 1, 1400, 1700);
    uint16_t ch[16]; for (auto& c : ch) c = 1500; ch[2] = 1000; ch[4] = 2000; ch[5] = 1500;
    RcCommand c = rc.process(ch, true);
    CHECK(c.roll == 0 && c.pitch == 0 && c.yaw == 0 && c.throttle_raw == 0 && c.link_ok, "centred sticks");
    CHECK((c.modes & (1u << M_ARM)) && (c.modes & (1u << M_ALTHOLD)) && (c.modes & (1u << M_ANGLE)), "arm + AltHold (which forces Angle) from aux1/aux2");
    ch[0] = 2000; ch[1] = 1000; ch[3] = 1750; ch[2] = 1500; c = rc.process(ch, true);
    CHECK(fabsf(c.roll - 1.0f) < 1e-5f && fabsf(c.pitch + 1.0f) < 1e-5f && c.yaw > 0.4f && c.yaw < 0.55f && fabsf(c.throttle_raw - 0.5f) < 1e-5f, "AETR map: roll right +, pitch back -, yaw right +, throttle");
    ch[0] = 1510; c = rc.process(ch, true); CHECK(c.roll == 0, "2 %% deadband around centre");
    // the throttle the stick asks for: Betaflight's mapping, zero below min_check (1050 us)
    { RcInterpreter t; uint16_t k[16]; for (auto& x : k) x = 1500;
      auto at = [&](uint16_t us) { k[2] = us; return t.process(k, true); };
      CHECK(at(1000).throttle == 0 && at(1049).throttle == 0 && at(1049).throttle_low && at(1000).throttle_low, "stick below 1050 us: 'low', throttle exactly 0 (a transmitter whose stick ends at 1030 still gives zero)");
      CHECK(!at(1050).throttle_low && at(1050).throttle == 0 && fabsf(at(1525).throttle - 0.5f) < 1e-5f && at(2000).throttle == 1.0f && fabsf(at(1145).throttle_cmd - 0.1f) < 1e-5f,
            "1050 us is not low any more; 1050 .. 2000 us is 0 .. 1 (1525 -> %.3f, 1145 -> %.3f)", at(1525).throttle, at(1145).throttle_cmd);
      CHECK(fabsf(at(1500).throttle_raw - 0.5f) < 1e-6f && fabsf(at(1050).throttle_raw - 0.05f) < 1e-6f, "the stick's POSITION (Altitude Hold: the middle holds) stays 1000 .. 2000 us"); }
    rc.cfg.throttle_mid = 0.3f; ch[2] = 1525; c = rc.process(ch, true); CHECK(fabsf(c.throttle - 0.3f) < 1e-4f && fabsf(c.throttle_raw - 0.525f) < 1e-5f, "throttle mid pins half throttle to 0.3 but AltHold still sees the raw stick");
    rc.cfg.throttle_mid = 0.5f; rc.cfg.throttle_limit = 0.8f; ch[2] = 2000; c = rc.process(ch, true); CHECK(fabsf(c.throttle - 0.8f) < 1e-4f, "throttle limit");
    rc.cfg.ch_roll = 3; rc.cfg.ch_yaw = 0; ch[3] = 1500; ch[0] = 2000; c = rc.process(ch, true); CHECK(c.yaw > 0.99f && c.roll == 0, "remappable channels (TAER style)");
}

// physics check of the mixer: apply a unit command with an exact-thrust plant and look at the resulting angular acceleration
static void mixer_physics(Layout l, int plant_layout, const char* name, const Plant* geo = nullptr) {
    Mixer m; m.cfg.layout = l; m.configure(); MotorLimits lim; lim.idle = 0.0f; lim.limit = 1.0f;
    for (int axis = 0; axis < 3; axis++) {
        Plant pl = geo ? *geo : Plant(plant_layout); pl.lag = 0.0005f; float out[kMaxMotors];
        float cmd[3] = {0, 0, 0}; cmd[axis] = 0.1f;
        m.mix(lim, 0.5f, cmd[0], cmd[1], cmd[2], out);
        const float base = 0.5f * pl.n * pl.Tmax; pl.step(out, 0.0005f);
        const float w[3] = {pl.w[0], pl.w[1], pl.w[2]};
        const float want = axis == 0 ? w[0] : (axis == 1 ? -w[1] : w[2]);        // pitch: +cmd = nose DOWN = negative rate about FRD y
        float others = 0; for (int k = 0; k < 3; k++) if (k != axis) others = fmaxf(others, fabsf(w[k]));
        CHECK(want > 0, "[%s] +%s command turns the right way (w=%.4f)", name, axis == 0 ? "roll" : axis == 1 ? "pitch" : "yaw", want);
        CHECK(others < 0.03f * fabsf(want), "[%s] %s is decoupled from the other axes (%.5f vs %.5f)", name, axis == 0 ? "roll" : axis == 1 ? "pitch" : "yaw", others, want);
        CHECK(fabsf(pl.total_thrust - base) < 0.02f * base, "[%s] axis command does not change total thrust (%.3f vs %.3f)", name, pl.total_thrust, base);
    }
}

static void t_mixer() {
    printf("[mixer] rows derived from geometry, checked against the physical plant\n");
    mixer_physics(QUAD_X, 0, "quad X"); mixer_physics(QUAD_PLUS, 1, "quad +"); mixer_physics(HEX_X, 2, "hex X");
    { MixRow r0 = mixer_row(QUAD_X, 0), r1 = mixer_row(QUAD_X, 1), r2 = mixer_row(QUAD_X, 2), r3 = mixer_row(QUAD_X, 3);
      auto eq = [](float a, float b) { return fabsf(a - b) < 1e-4f; };
      CHECK(eq(r0.roll, -1) && eq(r0.pitch, 1) && eq(r0.yaw, -1) && eq(r1.roll, -1) && eq(r1.pitch, -1) && eq(r1.yaw, 1) && eq(r2.roll, 1) && eq(r2.pitch, 1) && eq(r2.yaw, 1) && eq(r3.roll, 1) && eq(r3.pitch, -1) && eq(r3.yaw, -1), "quad X table is Betaflight's mixerQuadX literally: {roll, pitch, yaw} = REAR_R {-1,1,-1} FRONT_R {-1,-1,1} REAR_L {1,1,1} FRONT_L {1,-1,-1} (props in: 1 CW, 2 CCW, 3 CCW, 4 CW)"); }
    { MixRow p0 = mixer_row(QUAD_PLUS, 0), p1 = mixer_row(QUAD_PLUS, 1), p2 = mixer_row(QUAD_PLUS, 2), p3 = mixer_row(QUAD_PLUS, 3);
      CHECK(p0.yaw == -1 && p1.yaw == 1 && p2.yaw == 1 && p3.yaw == -1 && p0.pitch == 1 && p3.pitch == -1 && p1.roll == -1 && p2.roll == 1, "quad + table is Betaflight's mixerQuadP: REAR {0,1,-1} RIGHT {-1,0,1} LEFT {1,0,1} FRONT {0,-1,-1}"); }
    { Mixer m; m.configure(); MotorLimits lim{0.055f, 1.0f}; float o[kMaxMotors];
      m.mix(lim, 0.5f, 0, 0, 0, o); CHECK(fabsf(o[0] - (0.055f + 0.5f * 0.945f)) < 1e-5f && o[4] == 0, "throttle maps into [idle, limit] (absolute)");
      m.mix(lim, 0.95f, 0.6f, 0.6f, 0.6f, o); bool ok = true; for (int i = 0; i < 4; i++) if (o[i] < 0.055f - 1e-5f || o[i] > 1.0f + 1e-5f) ok = false; CHECK(ok, "saturated command stays inside [idle, limit]");
      m.mix(lim, 0.0f, 0.3f, 0, 0, o); CHECK(fmaxf(fmaxf(o[0], o[1]), fmaxf(o[2], o[3])) > 0.2f && fminf(fminf(o[0], o[1]), fminf(o[2], o[3])) >= 0.055f - 1e-5f, "airmode keeps authority at zero throttle without dropping under idle");
      MotorLimits l2{0.10f, 0.80f}; m.mix(l2, 1.0f, 0.5f, 0.0f, 0.0f, o); CHECK(fmaxf(fmaxf(o[0], o[1]), fmaxf(o[2], o[3])) <= 0.8f + 1e-5f, "output limit respected");
      m.mix(lim, 0.0f, 0.5f, 0.5f, 0.5f, o, true); CHECK(o[0] == 0.055f && o[1] == 0.055f && o[2] == 0.055f && o[3] == 0.055f && o[4] == 0, "ground idle: plain idle on every motor, whatever the rate loop asks for");
      m.mix(lim, 0.7f, 0.5f, 0.5f, 0.5f, o, true); CHECK(o[0] == 0.055f && o[3] == 0.055f, "... and whatever the throttle says");
      m.mix(lim, 0.02f, 0.3f, 0, 0, o); CHECK(fmaxf(fmaxf(o[0], o[1]), fmaxf(o[2], o[3])) > 0.2f, "a throttle of 2 %% is not ground idle: the pattern keeps its authority (it used to vanish below 5 %%)"); }
    { Mixer m; m.configure(); MotorLimits lim{0.055f, 1.0f}; float o[kMaxMotors]; m.mix(lim, 0.5f, 1.0f, 0.2f, 0.0f, o);      // far more roll than the headroom allows
      double pr = 0, pp = 0; for (int i = 0; i < 4; i++) { const MixRow row = mixer_row(QUAD_X, i); pr += row.roll * o[i]; pp += row.pitch * o[i]; }
      CHECK(pr > 0 && fabs(pp / pr - 0.2) < 0.03, "headroom exceeded: ALL axes are scaled together, the roll:pitch ratio survives (%.3f, want 0.2)", pp / pr); }
    { Mixer m; m.cfg.props_out = true; m.configure(); Plant pl(0); for (auto& s : pl.spin) s = -s; pl.lag = 0.0005f; float o[kMaxMotors]; m.mix({0, 1}, 0.5f, 0, 0, 0.1f, o); pl.step(o, 0.0005f);
      CHECK(pl.w[2] > 0.0005f, "props-out frame (all spins reversed) + props_out flag: yaw right still turns right"); }
}

// The frames added in 0.8.2 (Betaflight's tables): each checked against a plant built from where the motors really sit and which
// way their props turn - roll, pitch and yaw commands turn the right way, do not leak into the other axes, do not change the
// total thrust. (V-tail / A-tail have tilted motors, which the plant cannot build: their tables are checked literally.)
static void t_layouts() {
    printf("[mixer] the Betaflight frames: hex +, hex X (Betaflight order), H6, Y6, Y4, octo X8, octo flat + / X, quad X 1234\n");
    const float d = 0.7071068f;
    struct G { Layout l; const char* name; Plant p; } g[] = {
        {HEX_PLUS, "hex +", Plant({120, 60, 240, 300, 0, 180}, {1, -1, 1, -1, 1, -1})},
        {HEX_X_BF, "hex X (Betaflight order)", Plant({150, 30, 210, 330, 90, 270}, {1, 1, -1, -1, -1, 1})},
        {HEX_H, "H6", Plant({135, 45, 225, 315, 90, 270}, {-1, 1, 1, -1, 1, -1}, {1, 1, 1, 1, d, d})},
        {Y6, "Y6", Plant({180, 60, 300, 180, 60, 300}, {1, -1, -1, -1, 1, 1})},
        {OCTO_X8, "octo X8", Plant({135, 45, 225, 315, 135, 45, 225, 315}, {-1, 1, 1, -1, 1, -1, -1, 1})},
        {OCTO_FLAT_PLUS, "octo flat +", Plant({315, 45, 135, 225, 0, 90, 180, 270}, {1, 1, 1, 1, -1, -1, -1, -1})},
        {OCTO_FLAT_X, "octo flat X", Plant({292.5f, 22.5f, 112.5f, 202.5f, 337.5f, 67.5f, 157.5f, 247.5f}, {1, 1, 1, 1, -1, -1, -1, -1})},
        {QUAD_X_1234, "quad X 1234", Plant({315, 45, 135, 225}, {-1, 1, -1, 1})},
    };
    for (auto& x : g) {
        CHECK(mixer_motor_count(x.l) == x.p.n, "[%s] %d motors", x.name, mixer_motor_count(x.l));
        mixer_physics(x.l, 0, x.name, &x.p);
    }
    auto eq = [](float a, float b) { return fabsf(a - b) < 1e-4f; };
    { MixRow a = mixer_row(HEX_PLUS, 0), f = mixer_row(HEX_PLUS, 4);
      CHECK(eq(a.roll, -1) && eq(a.pitch, 0.5f) && a.yaw == 1 && eq(f.roll, 0) && eq(f.pitch, -1) && f.yaw == 1, "hex +: Betaflight HEX6 normalised (rear-right roll -0.866 -> -1, front pitch -1)"); }
    { MixRow r5 = mixer_row(HEX_H, 4), r0 = mixer_row(HEX_H, 0); CHECK(r5.roll == 0 && r5.pitch == 0 && r5.yaw == 0 && eq(r0.roll, -1) && eq(r0.pitch, 1) && r0.yaw == -1, "H6: the two middle motors only lift (Betaflight HEX6H)"); }
    { MixRow v1 = mixer_row(VTAIL4, 1), v0 = mixer_row(VTAIL4, 0), a1 = mixer_row(ATAIL4, 1), a0 = mixer_row(ATAIL4, 0);
      CHECK(eq(v0.roll, -1) && eq(v0.pitch, 1) && v0.yaw == 1 && eq(v1.roll, -0.46f / 0.58f) && eq(v1.pitch, -0.39f / 0.58f) && eq(v1.yaw, -0.5f), "V-tail: Betaflight VTAIL4, each axis scaled by its largest value");
      CHECK(a0.roll == 0 && a0.pitch == 1 && a0.yaw == 1 && a1.roll == -1 && a1.yaw == 0, "A-tail: Betaflight ATAIL4"); }
    { MixRow y0 = mixer_row(Y4, 0), y1 = mixer_row(Y4, 1); CHECK(y0.yaw == -1 && y1.yaw == 0 && y1.roll == -1, "Y4: yaw only from the rear pair");
      // Betaflight's Y4 is not decoupled: rolling speeds one front prop up and the other (turning the other way) down, which also
      // yaws a little - the yaw loop takes it out. Here only: every command turns the right way, the total thrust stays.
      Mixer m; m.cfg.layout = Y4; m.configure();
      for (int axis = 0; axis < 3; axis++) {
          Plant pl({180, 45, 180, 315}, {-1, 1, 1, -1}, {d, 1, d, 1}); pl.lag = 0.0005f; float out[kMaxMotors]; float cmd[3] = {0, 0, 0}; cmd[axis] = 0.1f;
          m.mix({0, 1}, 0.5f, cmd[0], cmd[1], cmd[2], out); const float base = 0.5f * pl.n * pl.Tmax; pl.step(out, 0.0005f);
          const float want = axis == 0 ? pl.w[0] : axis == 1 ? -pl.w[1] : pl.w[2];
          CHECK(want > 0 && fabsf(pl.total_thrust - base) < 0.02f * base, "[Y4] +%s turns the right way, total thrust kept", axis == 0 ? "roll" : axis == 1 ? "pitch" : "yaw");
      } }
    { float sum[3] = {0, 0, 0};
      for (int l = HEX_PLUS; l < LAYOUT_COUNT; l++) for (int k = 0; k < 3; k++) { if (l == VTAIL4 && k == 1) continue;         /* (V-tail: tilted motors, Betaflight's pitch column does not sum to 0) */ float z = 0; for (int i = 0; i < mixer_motor_count((Layout)l); i++) { const MixRow r = mixer_row((Layout)l, i); z += k == 0 ? r.roll : k == 1 ? r.pitch : r.yaw; } sum[k] += fabsf(z); }
      CHECK(sum[0] < 1e-3f && sum[1] < 0.02f && sum[2] < 1e-3f, "every frame: each axis sums to zero over its motors (a command never changes the total thrust; V-tail pitch excepted) (%g %g %g)", sum[0], sum[1], sum[2]); }
    { Mixer m; m.cfg.layout = OCTO_X8; m.configure(); float o[kMaxMotors]; m.mix({0.05f, 1.0f}, 0.5f, 0.1f, 0, 0, o);
      CHECK(m.motor_count() == 8 && o[0] < o[2] && o[4] < o[6] && o[0] == o[4], "octo X8: 8 outputs; roll right lowers both right-hand motors of each pair"); }
    CHECK(mixer_motor_count((Layout)200) == 4, "an unknown layout number: treated as a quad (never past the tables)");
}

static void t_safety() {
    printf("[safety] failsafe, arming, modes\n");
    { // ---- link lost: guard, descent, and the link coming back
      Failsafe fs; FsInput in; in.armed = true; in.alt_valid = true; in.vz = -1.0f; const float dt = 0.01f; FsOutput o;
      for (int i = 0; i < 100; i++) { o = fs.update(in, dt); } CHECK(fs.get_stage() == FsStage::NONE && o.cause == 0 && !o.neutral_sticks, "quiet");
      in.rx_ok = false; for (int i = 0; i < 30; i++) { o = fs.update(in, dt); }
      CHECK(o.stage == FsStage::GUARD && o.cause == FS_LINK && o.neutral_sticks && !o.alt_descent && !o.disarm, "link gone 0.3 s: guard - sticks neutral, throttle left alone");
      in.rx_ok = true; o = fs.update(in, dt); CHECK(o.stage == FsStage::NONE && !o.neutral_sticks && o.cause == 0, "the link is back inside the guard time: control is back in the SAME cycle (not a second later)");
      in.rx_ok = false; for (int i = 0; i < 80; i++) { o = fs.update(in, dt); }
      CHECK(o.stage == FsStage::DESCENT && o.alt_descent && o.neutral_sticks && !o.throttle_fallback && o.cause == FS_LINK, "gone for longer than the guard time: controlled descent with an altitude estimate");
      in.rx_ok = true; for (int i = 0; i < 90; i++) { o = fs.update(in, dt); }
      CHECK(o.stage == FsStage::DESCENT && o.neutral_sticks && o.alt_descent, "the link returns: still descending for the recovery time (0.9 s of 1 s)");
      in.rx_ok = false; o = fs.update(in, dt); in.rx_ok = true; for (int i = 0; i < 90; i++) { o = fs.update(in, dt); }
      CHECK(o.stage == FsStage::DESCENT, "a link that drops again starts the recovery time over");
      for (int i = 0; i < 15; i++) { o = fs.update(in, dt); }
      CHECK(o.stage == FsStage::NONE && !o.neutral_sticks && !o.alt_descent && !o.disarm && o.cause == 0, "1 s of unbroken link: the pilot has the craft back (Betaflight leaves its landing the same way)");
      in.rx_ok = false; for (int i = 0; i < 80; i++) { o = fs.update(in, dt); } in.landed = true; o = fs.update(in, dt);
      CHECK(o.disarm && o.stage == FsStage::LANDED, "no link all the way down: disarm once landed");
      in.armed = false; in.rx_ok = false; o = fs.update(in, dt); CHECK(o.stage == FsStage::LANDED, "stays LANDED while the link is still gone");
      in.rx_ok = true; in.landed = false; o = fs.update(in, dt); CHECK(o.stage == FsStage::NONE, "link back, disarmed: ready again"); }
    { // short dropouts add up
      Failsafe fs; FsInput in; in.armed = true; in.alt_valid = true; const float dt = 0.01f; FsOutput o; int cycles = 0;
      for (; cycles < 40 && fs.get_stage() != FsStage::DESCENT; cycles++) { in.rx_ok = false; for (int i = 0; i < 30; i++) { o = fs.update(in, dt); } in.rx_ok = true; for (int i = 0; i < 20; i++) { o = fs.update(in, dt); } }
      in.rx_ok = false; o = fs.update(in, dt);
      CHECK(o.stage == FsStage::DESCENT && cycles >= 3 && cycles <= 6, "a link that is gone 0.3 s of every 0.5 s: each loss alone is inside the guard time, together they are a failsafe (after %d losses)", cycles);
      Failsafe f2; in = FsInput(); in.armed = true; bool ever = false;
      for (int k = 0; k < 40; k++) { in.rx_ok = false; for (int i = 0; i < 10; i++) f2.update(in, dt); in.rx_ok = true; for (int i = 0; i < 40; i++) f2.update(in, dt); ever |= f2.get_stage() == FsStage::DESCENT; }
      CHECK(!ever, "0.1 s lost in every 0.5 s (a marginal but usable link): never a descent"); }
    { Failsafe f2; FsInput i2; i2.armed = true; i2.rx_ok = false; i2.alt_valid = false; FsOutput o; for (int i = 0; i < 80; i++) { o = f2.update(i2, 0.01f); } CHECK(o.throttle_fallback && !o.alt_descent, "no altitude estimate => throttle fallback, never a motor cut"); }
    { // the time limit is a net for a craft that does not come down - it never cuts a descent that makes progress
      Failsafe fs; fs.cfg.max_descent_s = 60.0f; FsInput in; in.armed = true; in.rx_ok = false; in.alt_valid = true; in.vz = -1.5f; FsOutput o; const float dt = 0.02f;
      for (int i = 0; i < (int)(200.0f / dt); i++) { o = fs.update(in, dt); }
      CHECK(o.stage == FsStage::DESCENT && !o.disarm, "sinking 1.5 m/s for 200 s (from 300 m): still descending, the motors are NOT cut in the air after 60 s");
      in.vz = 0.0f; for (int i = 0; i < (int)(59.0f / dt); i++) { o = fs.update(in, dt); } CHECK(o.stage == FsStage::DESCENT, "not sinking any more (it sits on something, undetected): 59 s ...");
      for (int i = 0; i < (int)(2.0f / dt); i++) { o = fs.update(in, dt); } CHECK(o.disarm && o.stage == FsStage::LANDED, "... 61 s without progress: motors off");
      Failsafe fn; fn.cfg.max_descent_s = 20.0f; in = FsInput(); in.armed = true; in.rx_ok = false; in.alt_valid = false; in.vz = -5.0f;
      for (int i = 0; i < (int)(20.4f / dt); i++) { o = fn.update(in, dt); } CHECK(o.stage == FsStage::DESCENT && o.throttle_fallback, "without a height estimate the limit is plain time: 20 s set, still going at 19.9 s of descent");
      for (int i = 0; i < (int)(0.4f / dt); i++) { o = fn.update(in, dt); } CHECK(o.disarm, "... and off after it (vz is not believed without a height)");
      Failsafe fl; in = FsInput(); in.armed = true; in.rx_ok = false; in.alt_valid = true; in.vz = -0.3f; fl.cfg.max_descent_s = 5.0f;
      for (int i = 0; i < (int)(30.0f / dt); i++) { o = fl.update(in, dt); } CHECK(o.stage == FsStage::DESCENT, "the slow last metres (0.3 m/s) count as progress"); }
    { // the failsafe switch: a test of a lost link, ended by the pilot's hand
      Failsafe fs; FsInput in; in.armed = true; in.alt_valid = true; in.switch_on = true; FsOutput o; const float dt = 0.01f;
      for (int i = 0; i < 80; i++) { o = fs.update(in, dt); }
      CHECK(o.stage == FsStage::DESCENT && o.cause == FS_SWITCH && o.neutral_sticks && o.alt_descent, "switch on for longer than the guard time: the same descent, cause SWITCH");
      in.switch_on = false; o = fs.update(in, dt); CHECK(o.stage == FsStage::NONE && !o.neutral_sticks, "switch off: control back at once (the link never was in doubt)");
      in.switch_on = true; for (int i = 0; i < 80; i++) { o = fs.update(in, dt); } in.rx_ok = false; in.switch_on = false; for (int i = 0; i < 50; i++) { o = fs.update(in, dt); }
      CHECK(o.stage == FsStage::DESCENT && (o.cause & FS_LINK) && (o.cause & FS_SWITCH), "the link goes during a switch failsafe (the channel now reads 'off'): that is not the pilot's hand - the descent goes on");
      in.rx_ok = true; for (int i = 0; i < 50; i++) { o = fs.update(in, dt); } CHECK(o.stage == FsStage::DESCENT && o.cause == FS_LINK, "link back with the switch off: the switch cause ends, the link has to prove itself");
      for (int i = 0; i < 60; i++) { o = fs.update(in, dt); } CHECK(o.stage == FsStage::NONE, "... 1 s later: normal"); }
    { Failsafe fs; FsInput in; in.armed = true; in.alt_valid = true; in.vz = -1.0f; in.rx_ok = false; in.switch_on = true; FsOutput o; for (int i = 0; i < 20; i++) { o = fs.update(in, 0.01f); }
      const uint8_t guard = o.cause; for (int i = 0; i < 100; i++) { o = fs.update(in, 0.01f); }
      CHECK(guard == FS_LINK && o.stage == FsStage::DESCENT && o.cause == FS_LINK, "the link is lost and the switch channel happens to read 'on': the cause is the link, not the switch (0x%x, 0x%x)", guard, o.cause); }
    ArmInput a; a.imu_ok = a.gyro_calibrated = a.rx_ok = true; a.baro_ok = true; a.throttle_low = true;
    CHECK(arm_blockers(a) == 0, "clean");
    a.althold_switch = true; a.baro_ok = false; CHECK(arm_blockers(a) & AB_ALTHOLD_NO_BARO, "AltHold without baro"); a.baro_ok = true;
    a.tilt_deg = 40; CHECK(arm_blockers(a) & AB_ANGLE_TOO_HIGH, "tilt"); a.tilt_deg = 0; a.throttle_low = false; CHECK(arm_blockers(a) & AB_THROTTLE_HIGH, "throttle"); a.throttle_low = true;
    a.use_mag = true; a.mag_calibrated = false; CHECK(arm_blockers(a) & AB_MAG_NOT_CALIBRATED, "compass enabled but not calibrated"); a.mag_calibrated = true; CHECK(arm_blockers(a) == 0, "...and fine once calibrated");
    a.loop_slow = true; CHECK(arm_blockers(a) == AB_LOOP_RATE, "loop below its configured rate"); a.loop_slow = false;
    a.batt_low = true; CHECK(arm_blockers(a) == AB_BATT_LOW, "battery at the warning level: no take-off"); a.batt_low = false; CHECK(arm_blockers(a) == 0, "battery fine");
    { // battery at its minimum: a forced landing that the pilot still steers
      Failsafe fb; FsInput ib; ib.armed = true; ib.alt_valid = true; ib.vz = -1.0f; ib.batt_critical = true; FsOutput o; for (int i = 0; i < 40; i++) { o = fb.update(ib, 0.01f); }
      CHECK(o.stage == FsStage::GUARD && !o.disarm && !o.neutral_sticks && o.cause == FS_BATTERY, "critical battery in flight: guard time first (0.5 s), no reaction to a blip - and the sticks stay the pilot's");
      for (int i = 0; i < 20; i++) { o = fb.update(ib, 0.01f); }
      CHECK(o.stage == FsStage::DESCENT && o.alt_descent && !o.disarm, "then a controlled descent, never a motor cut");
      CHECK(!o.neutral_sticks && o.cause == FS_BATTERY, "the link is fine: roll / pitch / yaw stay with the pilot, who chooses where it comes down (ArduCopter's LAND)");
      ib.batt_critical = false; for (int i = 0; i < 300; i++) { o = fb.update(ib, 0.01f); }
      CHECK(o.stage == FsStage::DESCENT && o.cause == FS_BATTERY, "a pack that recovers once the load is gone does not cancel the landing");
      ib.rx_ok = false; for (int i = 0; i < 5; i++) { o = fb.update(ib, 0.01f); }
      CHECK(o.neutral_sticks && o.cause == (FS_BATTERY | FS_LINK), "the link goes as well: now the craft is levelled");
      ib.rx_ok = true; for (int i = 0; i < 110; i++) { o = fb.update(ib, 0.01f); }
      CHECK(o.stage == FsStage::DESCENT && !o.neutral_sticks && o.cause == FS_BATTERY, "link back for 1 s: the pilot steers again, the landing goes on");
      Failsafe fd; FsInput id; id.armed = false; id.batt_critical = true; for (int i = 0; i < 200; i++) { o = fd.update(id, 0.01f); }
      CHECK(o.stage == FsStage::NONE, "disarmed on the ground: a critical battery starts nothing"); }
    { // runaway take-off prevention (Betaflight)
      RunawayTakeoff rt; RunawayTakeoff::Input in; in.armed = true; in.motors_mixed = true; in.throttle = 0.15f; const float dt = 0.001f; rt.on_arm();
      bool fired = false; in.pid_sum[0] = 650; in.gyro_dps[0] = 0;
      for (int i = 0; i < 2000; i++) { fired |= rt.update(in, dt); }
      CHECK(!fired, "bench without props: the PID sum is at its limit but nothing turns - no disarm");
      in.gyro_dps[0] = 40; int ms = 0; for (; ms < 500 && !rt.update(in, dt); ms++) {}
      CHECK(ms >= 74 && ms <= 77, "PID sum 650 while the craft rolls at 40 deg/s: disarm after %d ms (75)", ms);
      rt.on_arm(); in.gyro_dps[0] = 40; for (int i = 0; i < 60; i++) { rt.update(in, dt); } in.pid_sum[0] = 300; rt.update(in, dt); in.pid_sum[0] = 650; fired = false; for (int i = 0; i < 60; i++) { fired |= rt.update(in, dt); }
      CHECK(!fired, "the 75 ms must be unbroken");
      rt.on_arm(); in.pid_sum[0] = 0; in.pid_sum[2] = -700; in.gyro_dps[0] = 0; in.gyro_dps[2] = 45; fired = false; for (int i = 0; i < 300; i++) { fired |= rt.update(in, dt); }
      CHECK(!fired, "yaw needs 50 deg/s (45 is not enough)"); in.gyro_dps[2] = -55; for (int i = 0; i < 100; i++) { fired |= rt.update(in, dt); } CHECK(fired, "yaw sum -700 at -55 deg/s: disarm");
      rt.on_arm(); in = RunawayTakeoff::Input(); in.armed = true; in.motors_mixed = false; in.pid_sum[1] = 800; in.gyro_dps[1] = 100; fired = false; for (int i = 0; i < 500; i++) { fired |= rt.update(in, dt); }
      CHECK(!fired, "standing at ground idle (the PID output does not reach the motors): carried around by hand is not a runaway");
      // a normal flight switches it off for the rest of this power-up
      rt.reset(); rt.on_arm(); in = RunawayTakeoff::Input(); in.armed = true; in.motors_mixed = true; in.throttle = 0.25f; in.stick = 0.05f; in.pid_sum[0] = 50;
      for (int i = 0; i < 2000; i++) { rt.update(in, dt); } CHECK(!rt.flown(), "25 %% throttle with the sticks centred: not yet counted as a flight");
      in.stick = 0.2f; for (int i = 0; i < 490; i++) { rt.update(in, dt); } CHECK(!rt.flown(), "25 %% throttle and a stick moved, quiet PID sums: 0.49 s ...");
      for (int i = 0; i < 20; i++) { rt.update(in, dt); } CHECK(rt.flown(), "... 0.5 s in total: it flies, the check is off");
      in.pid_sum[0] = 900; in.gyro_dps[0] = 500; fired = false; for (int i = 0; i < 500; i++) { fired |= rt.update(in, dt); } CHECK(!fired, "a flip later in the flight is not a runaway");
      rt.on_arm(); for (int i = 0; i < 500; i++) { fired |= rt.update(in, dt); } CHECK(!fired && rt.flown(), "it stays off for the following arms (until the next power-up)");
      rt.reset(); rt.on_arm(); in.pid_sum[0] = 50; in.gyro_dps[0] = 0; in.stick = 0; in.throttle = 0.45f; for (int i = 0; i < 510; i++) { rt.update(in, dt); } CHECK(rt.flown(), "40 %% throttle counts without stick movement");
      rt.reset(); rt.on_arm(); in.throttle = 0.80f; for (int i = 0; i < 260; i++) { rt.update(in, dt); } CHECK(rt.flown(), "above 75 %% throttle half the time is enough");
      rt.reset(); rt.on_arm(); in.throttle = 0.45f; in.pid_sum[1] = 150; for (int i = 0; i < 3000; i++) { rt.update(in, dt); } CHECK(!rt.flown(), "PID sums above 100 are not a quiet flight");
      rt.reset(); rt.cfg.enabled = false; rt.on_arm(); in.pid_sum[0] = 900; in.gyro_dps[0] = 300; fired = false; for (int i = 0; i < 500; i++) { fired |= rt.update(in, dt); } CHECK(!fired, "switched off in the settings: never"); }
    { // why the craft was disarmed stays on show until the switch is off
      Arming ar; ArmInput ok; ok.imu_ok = ok.gyro_calibrated = ok.rx_ok = true; ok.throttle_low = true;
      ar.update(false, true, ok); CHECK(ar.update(true, true, ok), "armed");
      ar.force_disarm(AB_RUNAWAY_TAKEOFF);
      CHECK(!ar.update(true, true, ok) && (ar.blockers() & AB_RUNAWAY_TAKEOFF) && (ar.blockers() & AB_ARM_SWITCH_ON), "runaway disarm: the reason is a blocker while the switch is still on (0x%x)", ar.blockers());
      for (int i = 0; i < 20; i++) { ar.update(true, false, ok); } CHECK(ar.blockers() & AB_RUNAWAY_TAKEOFF, "... also across a link loss");
      ar.update(false, true, ok); CHECK(ar.blockers() == 0, "switch off: cleared");
      CHECK(ar.update(true, true, ok) && ar.blockers() == 0, "and it arms again"); }
    { Arming ar; ArmInput ok; ok.imu_ok = ok.gyro_calibrated = ok.rx_ok = true; ok.throttle_low = true;
      CHECK(!ar.update(true, true, ok) && (ar.blockers() & AB_ARM_SWITCH_ON), "switch on at power-up: refused");
      ar.update(false, true, ok); CHECK(!ar.armed(), "off"); CHECK(ar.update(true, true, ok), "turn off then on: armed");
      CHECK(!ar.update(false, true, ok), "switch off disarms at once");
      ArmInput bad = ok; bad.throttle_low = false; const uint32_t r0 = ar.refused(); CHECK(!ar.update(true, true, bad) && ar.refused() == r0 + 1 && (ar.blockers() & AB_THROTTLE_HIGH), "high throttle: arming refused and counted");
      CHECK(!ar.update(true, true, ok), "fixing the blocker with the switch still on does NOT arm (needs a fresh edge)");
      CHECK(ar.blockers() == AB_ARM_SWITCH_ON, "...and the pilot is told why: the switch has to be cycled (0x%x)", ar.blockers());
      ar.update(false, true, ok); CHECK(ar.blockers() == 0, "switch off: nothing blocks"); CHECK(ar.update(true, true, ok) && ar.blockers() == 0, "fresh edge arms, no blocker reported while armed");
      ar.force_disarm(); CHECK(!ar.armed() && !ar.update(true, true, ok) && ar.blockers() == AB_ARM_SWITCH_ON, "after a failsafe disarm the switch must be cycled, and that is reported"); ar.update(false, true, ok); CHECK(ar.update(true, true, ok), "cycled: arms again"); }
    // the switch is only believed on a live link
    { ArmInput ok; ok.imu_ok = ok.gyro_calibrated = true; ok.throttle_low = true; ArmInput nolink = ok; ok.rx_ok = true;
      { Arming ar;                                                    // powered up before the transmitter: the channel reads its default (off), then the first frame says ON
        for (int i = 0; i < 100; i++) { ar.update(false, false, nolink); }
        CHECK(!ar.update(true, true, ok) && !ar.armed() && (ar.blockers() & AB_ARM_SWITCH_ON), "transmitter switched on with the arm switch already on: the first frame does NOT arm (the 'off' before it was a default, not a frame)");
        for (int i = 0; i < 100; i++) { ar.update(true, true, ok); }
        CHECK(!ar.armed(), "and it stays disarmed while the switch stays on");
        ar.update(false, true, ok); CHECK(ar.blockers() == 0 && ar.update(true, true, ok), "switch off once, on again: armed"); }
      { Arming ar; ar.update(false, true, ok); ar.update(true, true, ok); ar.force_disarm(AB_RUNAWAY_TAKEOFF); ar.update(true, true, ok);
        ar.update(false, true, ok, true); const bool kept = (ar.blockers() & AB_RUNAWAY_TAKEOFF) != 0; ar.update(false, true, ok);
        CHECK(kept && !(ar.blockers() & AB_RUNAWAY_TAKEOFF), "keep_reason (stick arming, sticks at rest): the reason of a forced disarm stays; it goes with the next switch-off without it"); }
      { Arming ar; ar.update(false, true, ok);                        // disarmed, link good, switch off; then out of range and back with the switch on
        for (int i = 0; i < 50; i++) { ar.update(false, false, nolink); }
        CHECK(!ar.update(true, true, ok) && (ar.blockers() & AB_ARM_SWITCH_ON), "the link comes back with the arm switch on: no arming (it has not been seen off since)");
        for (int i = 0; i < 50; i++) { ar.update(true, false, nolink); }  // lost again with the receiver holding 'on'
        ar.update(false, true, ok); CHECK(ar.update(true, true, ok), "seen off on the live link, then on: armed"); }
      { Arming ar; ar.update(false, true, ok); ar.update(true, true, ok);     // armed; the link drops and the receiver's failsafe value for the channel is OFF
        bool stayed = true; for (int i = 0; i < 300; i++) stayed &= ar.update(false, false, nolink);
        CHECK(stayed && ar.armed(), "armed, link lost, the channel falls to 'off': NOT a disarm (the failsafe decides what happens to a craft in the air)");
        CHECK(ar.update(true, true, ok) && ar.blockers() == 0, "link back, switch still on: still armed, and no blocker is shown for it (0x%x)", ar.blockers());
        CHECK(!ar.update(false, true, ok), "the pilot switches off on the live link: disarmed");
        Arming a2; a2.update(false, true, ok); a2.update(true, true, ok); a2.force_disarm(); for (int i = 0; i < 10; i++) a2.update(false, false, nolink);
        CHECK(!a2.update(true, true, ok) && !a2.update(false, false, nolink) && !a2.update(true, true, ok), "after a forced disarm an 'off' that arrives without a link does not count as the cycle");
        a2.update(false, true, ok); CHECK(a2.update(true, true, ok), "a real cycle does"); } }
    // standing on the ground?
    { LandDetector ld; LandDetector::Input in; in.throttle_down = true; in.accel_1g = true; in.gyro_dps = 3.0f;
      bool early = false; for (int i = 0; i < 990; i++) early |= ld.update(in, 0.001f); bool at_1s = false; for (int i = 0; i < 20; i++) at_1s |= ld.update(in, 0.001f);
      CHECK(!early && at_1s, "throttle down, not turning, 1 g: standing on the ground - after a full second");
      auto never = [](LandDetector::Input i, const char* what) { LandDetector d; bool any = false; for (int k = 0; k < 5000; k++) any |= d.update(i, 0.001f); CHECK(!any, "%s", what); };
      LandDetector::Input x = in; x.throttle_down = false; never(x, "not with the throttle up");
      x = in; x.gyro_dps = 16.0f; never(x, "not while it turns faster than 15 deg/s");
      x = in; x.accel_1g = false; never(x, "not while the accelerometer does not read 1 g (falling, braking, pushed)");
      x = in; x.alt_valid = true; x.vz = -0.6f; never(x, "not while it sinks at 0.6 m/s by the altimeter (a level fall reads 1 g, too)");
      x = in; x.alt_valid = false; x.vz = -9.0f; { LandDetector d; bool any = false; for (int k = 0; k < 1100; k++) any |= d.update(x, 0.001f); CHECK(any, "without an altimeter the sink rate is not asked"); }
      LandDetector d; for (int k = 0; k < 900; k++) d.update(in, 0.001f); x = in; x.gyro_dps = 40; d.update(x, 0.001f); bool any = false; for (int k = 0; k < 900; k++) any |= d.update(in, 0.001f);
      CHECK(!any && d.still_for() > 0.89f, "one moment of movement starts the second over");
      d.reset(); CHECK(d.still_for() == 0.0f, "reset()"); }
    ModeMap mm; mm.add(M_ARM, 4, 1700, 2100); mm.add(M_ALTHOLD, 5, 1400, 1700); uint16_t aux[kAuxCount] = {1000, 1000, 1000, 1000, 2000, 1500, 1000, 1000}; uint32_t m = mm.resolve(aux);
    CHECK((m & (1u << M_ARM)) && (m & (1u << M_ALTHOLD)) && (m & (1u << M_ANGLE)), "AltHold forces Angle"); aux[5] = 1800; m = mm.resolve(aux); CHECK(!(m & (1u << M_ALTHOLD)) && !(m & (1u << M_ANGLE)), "ranges");
}

// A craft that goes straight up and down: motor output o (0..1) gives thrust o / hover_out x weight, through a motor lag.
// The throttle command is mapped to the output the way the mixer does it: idle + throttle x (1 - idle).
struct Lift {
    float hover_out, idle = 0.055f, lag = 0.03f; float z = 0, v = 0, out = 0, a = 0; bool held = false;
    explicit Lift(float h) : hover_out(h) {}
    void step(float throttle, float dt) {
        const float o = throttle > 0 ? idle + throttle * (1 - idle) : 0.0f;       // (0 from the controller = motors at idle / stopped: no lift either way)
        out += (o - out) * (dt / lag);
        a = held ? 0.0f : kG0 * (out / hover_out - 1.0f);
        if (z <= 0 && a < 0) { a = 0; v = 0; z = 0; } else { v += a * dt; z += v * dt; if (z < 0) { z = 0; v = 0; a = 0; } }
    }
};

static void t_althold_unit() {
    printf("[althold] state machine\n");
    AltHold ah; ah.init(); AhInput in; const float dt = 0.004f; float thr = 0;
    thr = ah.update(in, dt); CHECK(ah.get_state() == AhState::STOPPED && thr == 0, "disarmed");
    in.stick01 = 0.3f; in.armed = true; ah.update(in, dt); CHECK(ah.get_state() == AhState::LANDED_IDLE, "armed on the ground with the throttle stick already up"); for (int i = 0; i < 300; i++) thr = ah.update(in, dt);
    CHECK(ah.get_state() == AhState::LANDED_IDLE && thr == 0, "throttle was already up when arming: waits for the stick to come down first (no ghost takeoff)");
    in.stick01 = 0.0f; for (int i = 0; i < 100; i++) ah.update(in, dt); in.stick01 = 0.3f; for (int i = 0; i < 100; i++) ah.update(in, dt); CHECK(ah.get_state() == AhState::LANDED_PRE_TAKEOFF, "pre-takeoff");
    in.stick01 = 0.8f; for (int i = 0; i < 100; i++) ah.update(in, dt); CHECK(ah.get_state() == AhState::TAKEOFF, "takeoff");
    AltHold a2; a2.init(); a2.begin_flight(5.0f, 0.55f, 1.0f); AhInput i2; i2.armed = true; i2.stick01 = 0.5f; i2.z = 5.0f; const float first = a2.update(i2, dt); CHECK(fabsf(first - 0.55f) < 0.02f, "engaging in flight keeps the throttle (%.3f)", first);

    // ---- the stick (althold section, 0.8.8): hold point, deadband, the ends of the travel
    { AltHold m; m.init();
      CHECK(m.stick_rate(0.5f) == 0 && m.stick_rate(0.549f) == 0 && m.stick_rate(0.451f) == 0 && fabsf(m.stick_rate(1.0f) - 2.0f) < 1e-5f && fabsf(m.stick_rate(0.06f) + 1.2f) < 1e-5f && fabsf(m.stick_rate(0.0f) + 1.2f) < 1e-5f,
            "defaults: centre +-5 %% holds, full stick 2 m/s up, the bottom 1.2 m/s down - the same as before 0.8.8");
      CHECK(fabsf(m.stick_rate(0.775f) - 1.0f) < 1e-4f, "half way between the deadband and the top: half the climb rate (%.3f)", m.stick_rate(0.775f));
      m.cfg.mid_stick = 0.40f; m.cfg.deadband = 0.03f; m.cfg.max_stick = 0.90f; m.cfg.max_climb_mps = 3.0f; m.cfg.max_descend_mps = 1.0f;
      CHECK(m.stick_rate(0.40f) == 0 && m.stick_rate(0.42f) == 0 && fabsf(m.stick_rate(0.90f) - 3.0f) < 1e-5f && fabsf(m.stick_rate(0.95f) - 3.0f) < 1e-5f && fabsf(m.stick_rate(0.06f) + 1.0f) < 1e-5f && m.stick_rate(0.30f) < -0.2f,
            "hover at 40 %% of the stick, +-3 %%, full climb at 90 %%: 3 m/s from 90 %% up, 1 m/s down at the bottom");
      AhInput t; t.armed = true; AltHold k; k.init(); k.cfg = m.cfg; t.stick01 = 0.0f; k.update(t, dt); for (int i = 0; i < 50; i++) k.update(t, dt);
      t.stick01 = 0.2f; for (int i = 0; i < 50; i++) k.update(t, dt); t.stick01 = 0.45f; for (int i = 0; i < 20; i++) k.update(t, dt);
      CHECK(k.get_state() == AhState::TAKEOFF, "take-off starts above the hold point + deadband (45 %% with the hold at 40 %%)"); }

    // ---- throttle <-> motor output (the mixer's own mapping: idle at zero throttle)
    { AltHold m; m.init(); m.set_limits(0.055f, 1.0f); CHECK(fabsf(m.to_motor(0) - 0.055f) < 1e-6f && fabsf(m.to_motor(1) - 1.0f) < 1e-6f && fabsf(m.to_motor(0.5f) - 0.5275f) < 1e-6f && fabsf(m.to_throttle(0.5275f) - 0.5f) < 1e-6f && m.to_throttle(0.01f) == 0 && m.to_throttle(2) == 1,
          "throttle 0 .. 1 is motor output idle .. limit");
      m.hover = 0.30f; CHECK(fabsf(m.to_motor(m.hover_throttle()) - 0.30f) < 1e-6f && fabsf(m.to_motor(m.hover_throttle(0.85f)) - 0.255f) < 1e-6f, "hover_throttle(k) = the throttle for k x the weight of thrust");
      m.set_limits(0.12f, 0.8f); CHECK(fabsf(m.to_motor(0.5f) - 0.46f) < 1e-6f, "dynamic idle and the output limit move the mapping");
      m.set_limits(0.5f, 0.2f); CHECK(m.to_motor(1.0f) > 0.5f, "a nonsense range is not divided by"); }
}

// The hover output is MEASURED: from the craft's own motion, in any flight mode, from any starting guess.
static void t_hover_estimator() {
    printf("[hover throttle] measured from the craft's motion - a camera craft hovers at 50 %%, a racing quad at 10 %%\n");
    const float dt = 0.002f;
    for (float H : {0.12f, 0.30f, 0.55f, 0.75f}) for (float guess : {0.40f, 0.06f, 0.85f}) {
        HoverEstimator e; e.reset(guess); Lift c(H); c.z = 5; c.out = H; float t_ok = -1;
        for (int i = 0; i < 4000; i++) { const float t = i * dt, out_cmd = H * (1.0f + 0.15f * sinf(2 * kPi * 0.7f * t) + 0.05f * sinf(2 * kPi * 3.1f * t));      // a pilot working the throttle around the hover point
            c.out += (out_cmd - c.out) * (dt / c.lag); c.a = kG0 * (c.out / H - 1); e.update(out_cmd, 1.0f, c.a + gauss(0.8f), dt); if (t_ok < 0 && fabsf(e.value - H) < 0.05f * H) t_ok = t; }
        CHECK(fabsf(e.value - H) < 0.04f * H && t_ok >= 0 && t_ok < 2.5f, "true hover output %.2f, first guess %.2f: found within 5 %% after %.1f s, %.3f in the end", H, guess, t_ok, e.value);
    }
    { HoverEstimator e; e.reset(0.40f); for (int i = 0; i < 3000; i++) { e.update(0.22f, 1.0f, 0.0f, dt); }
      CHECK(fabsf(e.value - 0.22f) < 0.005f, "standing on the ground with the motors at 22 %%: the reading is 22 %% - the craft hovers at that or more, never less (%.3f)", e.value);
      auto fed = [&](float out, float cosv, float a) { HoverEstimator x; x.reset(0.22f); for (int i = 0; i < 3000; i++) x.update(out, cosv, a, dt); return x.value; };
      CHECK(fed(0.02f, 1.0f, 0.0f) == 0.22f, "motors barely turning (2 %%): not a measurement");
      CHECK(fed(0.60f, 1.0f, 9.0f) == 0.22f && fed(0.05f, 1.0f, -6.0f) == 0.22f, "a punch (0.9 g up) and a drop (0.6 g down) are not used: thrust is not proportional to the output that far from hover");
      CHECK(fed(0.50f, 0.6f, 0.0f) == 0.22f, "tilted 53 degrees: not a hover");
      CHECK(fabsf(fed(0.30f / 0.9f, 0.9f, 0.0f) - 0.30f) < 0.003f && fabsf(fed(0.30f * 1.2f, 1.0f, 0.2f * kG0) - 0.30f) < 0.003f, "tilted 26 degrees and holding height with 33 %%, or level and climbing with 0.2 g at 36 %%: the hover is 30 %% both times");
      e.seed(0.50f, 1.0f, 0.25f * kG0); CHECK(fabsf(e.value - 0.40f) < 1e-4f, "seed at lift-off: 50 %% while already accelerating up with 0.25 g means a hover at 40 %%");
      e.seed(0.02f, 1.0f, 0.0f); CHECK(e.value == e.min, "never below the minimum"); e.reset(5.0f); CHECK(e.value == e.max, "nor above the maximum");
      const float before = e.value; e.update(0.3f, 1.0f, 0.0f, 0.0f); CHECK(e.value == before, "dt 0: nothing"); }
}

// Take-off in Altitude Hold: the throttle is raised until the craft leaves the ground - whatever it weighs.
static void t_althold_takeoff() {
    printf("[althold] take-off finds the hover throttle by itself\n");
    const float dt = 0.002f;
    for (float H : {0.10f, 0.133f, 0.25f, 0.40f, 0.60f, 0.78f}) {
        AltHold ah; ah.init(); ah.set_limits(0.055f, 1.0f); Lift c(H); AhInput in; in.armed = true; float thr = 0, peak = 0, vmax = 0, t_lift = -1, hover_at_lift = 0;
        for (int i = 0; i < 100; i++) thr = ah.update(in, dt);                               // armed, stick down
        in.stick01 = 0.3f; for (int i = 0; i < 100; i++) thr = ah.update(in, dt);
        in.stick01 = 0.8f;
        for (int i = 0; i < 6000; i++) { const float t = i * dt; if (t > 0.5f) in.stick01 = 0.5f;
            in.z = c.z; in.v = c.v; in.a_up = c.a; thr = ah.update(in, dt); ah.learn(thr, 1.0f, c.a, dt); c.step(thr, dt);
            peak = fmaxf(peak, c.z); vmax = fmaxf(vmax, c.v); if (t_lift < 0 && c.z > 0.01f) t_lift = t; if (hover_at_lift == 0 && c.z > 0.12f) hover_at_lift = ah.hover; }
        CHECK(ah.get_state() == AhState::FLYING && fabsf(c.z - 0.6f) < 0.08f && peak < 0.85f && vmax < 1.6f, "hover output %.3f: off the ground after %.2f s, up to %.2f m at most (target 0.6), fastest %.2f m/s, then %.2f m", H, t_lift, peak, vmax, c.z);
        CHECK(fabsf(hover_at_lift - H) < 0.25f * H + 0.02f && fabsf(ah.hover - H) < 0.05f * H + 0.005f, "... the hover output read at lift-off is %.3f, %.3f after 12 s (true %.3f)", hover_at_lift, ah.hover, H);
    }
    // it never leaves the ground (held down, props missing): the throttle does not keep rising
    { AltHold ah; ah.init(); ah.set_limits(0.055f, 1.0f); Lift c(0.3f); c.held = true; AhInput in; in.armed = true; float thr = 0, top = 0;
      for (int i = 0; i < 100; i++) { ah.update(in, dt); }
      in.stick01 = 0.3f; for (int i = 0; i < 100; i++) { ah.update(in, dt); }
      in.stick01 = 0.8f;
      for (int i = 0; i < 3000; i++) { in.z = c.z; in.v = c.v; thr = ah.update(in, dt); c.step(thr, dt); top = fmaxf(top, thr); }
      CHECK(ah.get_state() == AhState::LANDED_IDLE && thr == 0 && top <= 0.9f + 1e-4f && top > 0.85f, "held on the ground: the ramp stops at 90 %% and the motors go back to idle (reached %.2f)", top);
      for (int i = 0; i < 500; i++) { thr = ah.update(in, dt); }
      CHECK(ah.get_state() == AhState::LANDED_IDLE && thr == 0, "and it does not try again until the stick has been down");
      in.stick01 = 0; for (int i = 0; i < 50; i++) ah.update(in, dt); in.stick01 = 0.3f; for (int i = 0; i < 50; i++) ah.update(in, dt); CHECK(ah.get_state() == AhState::LANDED_PRE_TAKEOFF, "stick down and up again: ready for the next try"); }
    // the pilot changes his mind during the ramp
    { AltHold ah; ah.init(); AhInput in; in.armed = true; float thr = 0; for (int i = 0; i < 100; i++) ah.update(in, dt); in.stick01 = 0.3f; for (int i = 0; i < 100; i++) ah.update(in, dt); in.stick01 = 0.8f;
      for (int i = 0; i < 100; i++) { thr = ah.update(in, dt); }
      CHECK(ah.get_state() == AhState::TAKEOFF && fabsf(thr - 0.4f * 0.2f) < 0.01f, "the ramp: 40 %% of throttle per second (%.3f after 0.2 s)", thr);
      const float h0 = ah.hover; ah.learn(thr, 1.0f, 0.0f, dt); CHECK(ah.hover == h0, "the ramp itself is not taken for a hover measurement");
      in.stick01 = 0; thr = ah.update(in, dt); CHECK(ah.get_state() == AhState::LANDED_PRE_TAKEOFF && thr == 0, "stick down during the ramp: back to idle at once"); }
    // the throttle can go far enough down to bring a craft with a lot of thrust back
    { AltHold ah; ah.init(); ah.set_limits(0.055f, 1.0f); ah.hover = 0.133f; ah.begin_flight(10.0f, ah.hover_throttle(), 1.0f); AhInput in; in.armed = true; in.stick01 = 0.5f; in.z = 10; in.v = 3.0f; const float thr = ah.update(in, dt);
      CHECK(ah.to_motor(thr) < 0.5f * 0.133f + 0.03f, "hovering at 13 %% and climbing at 3 m/s unasked: the output goes down to %.3f (a fifth of the hover output, or idle)", ah.to_motor(thr)); }
    // ... but not all the way: 0.8 g of deceleration, so the attitude loop keeps motors to work with
    { AltHold ah; ah.init(); ah.set_limits(0.055f, 1.0f); ah.hover = 0.5f; ah.begin_flight(10.0f, ah.hover_throttle(), 1.0f); AhInput in; in.armed = true; in.stick01 = 0.5f; in.z = 10; in.v = 5.0f; const float thr = ah.update(in, dt);
      CHECK(fabsf(ah.to_motor(thr) - 0.5f * (1.0f - 7.8f / kG0)) < 0.004f, "hovering at 50 %% and shooting up at 5 m/s: output %.3f = hover x (1 - 0.8) - the strongest braking it asks for", ah.to_motor(thr));
      AltHold b; b.init(); b.set_limits(0.055f, 1.0f); b.hover = 0.5f; b.begin_flight(10.0f, b.hover_throttle(), 1.0f); AhInput i2; i2.armed = true; i2.stick01 = 0.5f; i2.z = 10; i2.up_cos = 0.866f; const float tilted = b.update(i2, dt);
      CHECK(fabsf(b.to_motor(tilted) - 0.5f / 0.866f) < 0.003f, "holding height while tilted 30 degrees needs 1 / cos more thrust: output %.3f", b.to_motor(tilted)); }
    // the hover output stays inside the limits of the settings, however it is measured
    { HoverEstimator e; e.reset(0.40f); for (int i = 0; i < 4000; i++) e.update(0.95f, 1.0f, 0.0f, dt); const float hi = e.value; for (int i = 0; i < 4000; i++) e.update(0.045f, 1.0f, 0.0f, dt);
      CHECK(hi == e.max && e.value == e.min, "the estimate never leaves min .. max (%.2f, %.2f)", hi, e.value);
      AltHold ah; ah.cfg.hover_min = 0.20f; ah.cfg.hover_max = 0.50f; ah.init(); ah.set_limits(0.055f, 1.0f); ah.begin_flight(5.0f, 0.5f, 1.0f);
      for (int i = 0; i < 4000; i++) { ah.learn(ah.to_throttle(0.70f), 1.0f, 0.0f, dt); }
      const float top = ah.hover; for (int i = 0; i < 4000; i++) { ah.learn(ah.to_throttle(0.10f), 1.0f, 0.0f, dt); }
      CHECK(top == 0.50f && ah.hover == 0.20f, "hover_min / hover_max of the settings are the estimator's limits (%.2f, %.2f)", top, ah.hover); }
    // lift-off is also seen by the height alone (a slow, cushioned start: the climb rate never reaches 0.15 m/s)
    { AltHold ah; ah.init(); ah.set_limits(0.055f, 1.0f); AhInput in; in.armed = true; for (int i = 0; i < 100; i++) ah.update(in, dt); in.stick01 = 0.3f; for (int i = 0; i < 100; i++) ah.update(in, dt); in.stick01 = 0.8f; for (int i = 0; i < 100; i++) ah.update(in, dt);
      const bool before = ah.lifted(); in.v = 0.10f; in.z = 0.10f; ah.update(in, dt);
      CHECK(ah.get_state() == AhState::TAKEOFF && !before && ah.lifted(), "10 cm above where it stood, climbing at only 0.10 m/s: off the ground"); }
    // a second take-off after a landing starts its ramp from zero again
    { AltHold ah; ah.init(); ah.set_limits(0.055f, 1.0f); Lift c(0.30f); AhInput in; in.armed = true; float thr = 0;
      auto fly = [&](float stick, float seconds) { in.stick01 = stick; for (int i = 0; i < (int)(seconds / dt); i++) { in.z = c.z; in.v = c.v; in.a_up = c.a; thr = ah.update(in, dt); ah.learn(thr, 1.0f, c.a, dt); c.step(thr, dt); } };
      fly(0.0f, 0.2f); fly(0.3f, 0.2f); fly(0.8f, 0.5f); fly(0.5f, 6.0f); const float z_up = c.z; fly(0.0f, 8.0f);
      CHECK(z_up > 0.4f && ah.get_state() == AhState::LANDED_IDLE && c.z == 0, "first flight: up (%.2f m), down, landed", z_up);
      fly(0.3f, 0.2f); in.stick01 = 0.8f; in.z = c.z; in.v = c.v; ah.update(in, dt); const float first = ah.update(in, dt); const bool in_ramp = ah.get_state() == AhState::TAKEOFF && !ah.lifted(); fly(0.8f, 0.1f);
      CHECK(in_ramp && first < 0.005f && fabsf(thr - 0.4f * 0.1f) < 0.01f && c.z == 0, "second take-off: the ramp starts at zero again (%.4f, %.3f after 0.1 s) - it does not jump to where the first one ended", first, thr); }
    // a slow descent is not a landing: the motors must really be unloaded (matters for a craft that hovers near idle)
    { AltHold ah; ah.cfg.land_speed_mps = 0.5f; ah.init(); ah.set_limits(0.055f, 1.0f); Lift c(0.133f); c.z = 2.0f; c.out = 0.133f; ah.hover = 0.133f; ah.hover_est.reset(0.133f); ah.begin_flight(2.0f, ah.hover_throttle(), 1.0f);
      AhInput in; in.armed = true; in.stick01 = 0.5f; in.failsafe_descent = true; float z_end = -1, thr = 0;
      for (int i = 0; i < 10000 && z_end < 0; i++) { in.z = c.z; in.v = c.v; in.a_up = c.a; thr = ah.update(in, dt); c.step(thr, dt); if (ah.get_state() != AhState::FLYING) z_end = c.z; }
      CHECK(z_end >= 0 && z_end < 0.02f, "failsafe descent of a craft hovering at 13 %% output, sinking slower than the landing speed threshold for seconds: the motors stay on until it stands on the ground (given up at %.2f m)", z_end); }
}

// Altitude Hold where it is not simply flying: a take-off that is called off, nobody at the stick, the throttle at one of its
// ends, the craft on its back.
static void t_althold_edges() {
    printf("[althold] a take-off called off, nobody at the stick, throttle at its ends, upside down\n");
    const float dt = 0.002f;
    auto ready = [&](AltHold& ah, AhInput& in) { ah.init(); ah.set_limits(0.055f, 1.0f); in = AhInput(); in.armed = true; for (int i = 0; i < 100; i++) { ah.update(in, dt); } in.stick01 = 0.3f; for (int i = 0; i < 100; i++) { ah.update(in, dt); } };
    // raise the stick and fly until the craft is `height` above the ground
    auto lift_off = [&](AltHold& ah, Lift& c, AhInput& in, float height) { ready(ah, in); in.stick01 = 0.8f;
        for (int i = 0; i < 6000 && c.z < height; i++) { in.z = c.z; in.v = c.v; in.a_up = c.a; const float thr = ah.update(in, dt); ah.learn(thr, 1.0f, c.a, dt); c.step(thr, dt); } };
    // fly on until it stands; returns whether the motors were ever stopped in the air, and how fast it came down at most
    auto land = [&](AltHold& ah, Lift& c, AhInput& in, bool& cut, float& fastest) { cut = false; fastest = 0;
        for (int i = 0; i < 10000 && (ah.get_state() == AhState::FLYING || ah.get_state() == AhState::TAKEOFF); i++) { in.z = c.z; in.v = c.v; in.a_up = c.a; const float thr = ah.update(in, dt);
            if (thr == 0.0f && c.z > 0.03f) cut = true;
            c.step(thr, dt); fastest = fminf(fastest, c.v); } };
    { AltHold ah; AhInput in; Lift c(0.30f); lift_off(ah, c, in, 0.25f);
      CHECK(ah.get_state() == AhState::TAKEOFF && ah.lifted(), "25 cm up, still in the take-off");
      in.stick01 = 0.0f; bool cut; float fastest; land(ah, c, in, cut, fastest);
      CHECK(!cut && ah.get_state() == AhState::LANDED_IDLE && c.z == 0 && fastest > -1.6f && fastest < -0.2f, "the pilot pulls the stick down during the take-off: that is 'descend' - it comes down under power (%.2f m/s at most) and stands; the motors are not stopped 25 cm up", fastest); }
    { AltHold ah; AhInput in; Lift c(0.30f); lift_off(ah, c, in, 0.25f);
      in.failsafe_descent = true; bool cut; float fastest; land(ah, c, in, cut, fastest);
      CHECK(!cut && ah.get_state() == AhState::LANDED_IDLE && c.z == 0 && fastest > -0.6f, "the failsafe asks for the descent during the take-off: the same, at the failsafe's rate (%.2f m/s)", fastest); }
    { AltHold ah; AhInput in; Lift c(0.30f); lift_off(ah, c, in, 0.25f);                                  // the link drops for a moment while it climbs out
      in.pilot_absent = true; in.stick01 = 0.0f; float peak = 0; for (int i = 0; i < 1500; i++) { in.z = c.z; in.v = c.v; in.a_up = c.a; c.step(ah.update(in, dt), dt); peak = fmaxf(peak, c.z); }
      CHECK(ah.get_state() == AhState::FLYING && fabsf(c.z - 0.6f) < 0.1f && fabsf(c.v) < 0.1f, "nobody at the stick while it climbs out (whatever the channel reads): it finishes the take-off and holds %.2f m", c.z);
      in.stick01 = 1.0f; for (int i = 0; i < 1000; i++) { in.z = c.z; in.v = c.v; in.a_up = c.a; c.step(ah.update(in, dt), dt); }
      CHECK(fabsf(c.z - 0.6f) < 0.1f, "... and a stick that reads 'full up' does not make it climb (%.2f m)", c.z);
      in.pilot_absent = false; for (int i = 0; i < 500; i++) { in.z = c.z; in.v = c.v; in.a_up = c.a; c.step(ah.update(in, dt), dt); }
      CHECK(c.z > 1.0f, "the pilot is back: the stick counts again (%.2f m)", c.z); }
    for (int why = 0; why < 2; why++) { AltHold ah; AhInput in; ready(ah, in); float thr = 0; in.stick01 = 0.8f; for (int i = 0; i < 100; i++) { thr = ah.update(in, dt); }
      const bool ramping = ah.get_state() == AhState::TAKEOFF && !ah.lifted() && thr > 0.05f;
      (why ? in.failsafe_descent : in.pilot_absent) = true; thr = ah.update(in, dt);
      CHECK(ramping && ah.get_state() == AhState::LANDED_IDLE && thr == 0.0f, "%s while the throttle is still being raised: it has not left the ground, and does not - no take-off without a pilot", why ? "the failsafe descent begins" : "the link drops");
      for (int i = 0; i < 500; i++) { thr = ah.update(in, dt); }
      const bool stays = ah.get_state() == AhState::LANDED_IDLE && thr == 0.0f; in.failsafe_descent = in.pilot_absent = false; for (int i = 0; i < 500; i++) { thr = ah.update(in, dt); }
      CHECK(stays && ah.get_state() == AhState::LANDED_IDLE && thr == 0.0f, "... not while it lasts, and not when the pilot is back with the stick still up");
      in.stick01 = 0.0f; for (int i = 0; i < 50; i++) { ah.update(in, dt); } in.stick01 = 0.3f; for (int i = 0; i < 50; i++) { ah.update(in, dt); }
      CHECK(ah.get_state() == AhState::LANDED_PRE_TAKEOFF, "stick down and up again: ready"); }
    for (int why = 0; why < 2; why++) { AltHold ah; AhInput in; ready(ah, in);
      const bool pre = ah.get_state() == AhState::LANDED_PRE_TAKEOFF; (why ? in.failsafe_descent : in.pilot_absent) = true; ah.update(in, dt);
      const bool back = ah.get_state() == AhState::LANDED_IDLE; in.stick01 = 0.8f; for (int i = 0; i < 300; i++) { ah.update(in, dt); }
      CHECK(pre && back && ah.get_state() == AhState::LANDED_IDLE, "standing with the stick a little up when %s: back to idle, and a stick that reads 80 %% starts nothing", why ? "the failsafe acts" : "the link drops");
      int flips = 0; in.stick01 = 0.0f; for (int i = 0; i < 50; i++) { ah.update(in, dt); if (ah.get_state() != AhState::LANDED_IDLE) flips++; }
      in.stick01 = 0.3f; for (int i = 0; i < 50; i++) { ah.update(in, dt); if (ah.get_state() != AhState::LANDED_IDLE) flips++; }
      CHECK(ah.get_state() == AhState::LANDED_IDLE && flips == 0, "a stick going down and up while nobody is there is not the pilot's either: the state never leaves 'idle' (left it %d times - every change is an entry in the flight log)", flips);
      in.failsafe_descent = in.pilot_absent = false; for (int i = 0; i < 50; i++) { ah.update(in, dt); }
      CHECK(ah.get_state() == AhState::LANDED_IDLE, "back, stick at 30 %%: it wants the stick down first");
      in.stick01 = 0.0f; ah.update(in, dt); in.stick01 = 0.3f; ah.update(in, dt); CHECK(ah.get_state() == AhState::LANDED_PRE_TAKEOFF, "down, up: ready"); }

    // ---- the integrator while the throttle stands at one of its ends
    { AltHold ah; ah.init(); ah.set_limits(0.055f, 1.0f); ah.hover = 0.70f; ah.begin_flight(5.0f, ah.hover_throttle(), 1.0f); AhInput in; in.armed = true; in.stick01 = 1.0f; in.z = 5.0f; float thr = 0;
      for (int i = 0; i < 1500; i++) { thr = ah.update(in, dt); }
      CHECK(thr == 1.0f && ah.integrator() >= 0.0f && ah.integrator() < 0.5f, "a heavy craft held down for 3 s with the stick up: full throttle, and the integrator has not stored the 3 s (%.3f m/s^2; unlimited it would stand at its 6.0 limit)", ah.integrator()); }
    { AltHold ah; ah.init(); ah.set_limits(0.055f, 0.90f); ah.hover = 0.75f; ah.begin_flight(5.0f, 1.0f, 1.0f); AhInput in; in.armed = true; in.stick01 = 0.5f; in.z = 5.0f; in.v = 0.1f; in.up_cos = 0.9f;      // asks for 96 % of output: above the 90 % limit, below 100 %
      const float i0 = ah.integrator(); float thr = 0; for (int i = 0; i < 50; i++) { thr = ah.update(in, dt); }
      CHECK(i0 > 1.0f && thr == 1.0f && ah.integrator() < i0 - 0.02f, "at the top of the range (leaning in a turn, output limit 90 %%) and climbing a little: the integrator may come DOWN (%.3f -> %.3f)", i0, ah.integrator());
      in.v = -1.0f; ah.update(in, dt); const float i1 = ah.integrator(); for (int i = 0; i < 200; i++) { thr = ah.update(in, dt); }
      CHECK(thr == 1.0f && ah.integrator() <= i1, "... but not up (falling, at the top: %.3f -> %.3f)", i1, ah.integrator()); }
    { AltHold ah; ah.init(); ah.set_limits(0.055f, 0.90f); ah.hover = 0.75f; ah.begin_flight(5.0f, 1.0f, 0.9f); AhInput in; in.armed = true; in.stick01 = 0.5f; in.z = 5.0f; in.v = -0.05f; in.up_cos = 0.9f;
      float thr = ah.update(in, dt); const float i1 = ah.integrator(); for (int i = 0; i < 200; i++) { thr = ah.update(in, dt); }
      CHECK(thr == 1.0f && ah.integrator() <= i1 + 1e-6f, "sinking a little and asking for about 93 %% of the output - above the 90 %% limit, below 100 %%: the integrator does not grow (%.3f -> %.3f)", i1, ah.integrator()); }
    { AltHold ah; ah.init(); ah.set_limits(0.055f, 1.0f); ah.hover = 0.10f; ah.begin_flight(5.0f, ah.hover_throttle(), 1.0f); AhInput in; in.armed = true; in.stick01 = 0.0f; in.z = 5.0f; in.v = -0.4f; float thr = 1;
      for (int i = 0; i < 1500; i++) { thr = ah.update(in, dt); }
      CHECK(ah.get_state() == AhState::FLYING && thr == 0.0f && ah.integrator() <= 0.0f && ah.integrator() > -0.3f, "a light craft in an updraught, sinking slower than the stick at the bottom asks for 3 s: motors at idle, the integrator not wound down (%.3f; unlimited it would stand at -4.0)", ah.integrator());
      in.stick01 = 0.5f; in.v = -2.5f; for (int i = 0; i < 50; i++) { ah.update(in, dt); }
      CHECK(ah.integrator() > 0.0f, "and free to come up again when it falls faster than asked (%.3f)", ah.integrator()); }
    { AltHold ah; ah.init(); ah.set_limits(0.055f, 1.0f); ah.hover = 0.10f; ah.begin_flight(5.0f, ah.hover_throttle(), 1.0f); AhInput in; in.armed = true; in.stick01 = 0.5f; in.z = 5.0f; in.v = -1.0f; in.up_cos = 0.01f; float thr = 1;
      for (int i = 0; i < 100; i++) { thr = ah.update(in, dt); }
      CHECK(thr == 0.0f && ah.integrator() > 0.3f, "nearly on its side and falling (the output is held at idle there): the integrator may come UP from the bottom all the same (%.2f)", ah.integrator()); }
    // ---- thrust only helps while the craft is the right way up
    { auto out_at = [&](float up_cos) { AltHold ah; ah.init(); ah.set_limits(0.055f, 1.0f); ah.hover = 0.30f; ah.begin_flight(5.0f, ah.hover_throttle(), 1.0f); AhInput in; in.armed = true; in.stick01 = 0.5f; in.z = 5.0f; in.up_cos = up_cos; return ah.to_motor(ah.update(in, dt)); };
      CHECK(fabsf(out_at(1.0f) - 0.30f) < 1e-4f && fabsf(out_at(0.5f) - 0.60f) < 1e-4f && fabsf(out_at(0.2f) - 0.60f) < 1e-4f, "level: the hover output; leaning 60 degrees: twice that, and never more (78 degrees: %.2f)", out_at(0.2f));
      CHECK(fabsf(out_at(0.1f) - 0.60f) < 1e-4f && fabsf(out_at(0.05f) - 0.30f) < 1e-4f && fabsf(out_at(0.0f) - 0.055f) < 1e-4f && fabsf(out_at(-0.5f) - 0.055f) < 1e-4f && fabsf(out_at(-1.0f) - 0.055f) < 1e-4f,
            "from 84 degrees on it is taken back (87 degrees: %.2f), and on its side or on its back the craft gets idle: thrust would push it down there", out_at(0.05f)); }
}

// ---- 0.28: jerk-limited shaping (kinematics.h, ArduPilot AP_Math/control.cpp) and the AC_PosControl vertical cascade
static void t_kinematics() {
    printf("[kinematics] jerk-limited shaping as ArduPilot's control.cpp\n");
    using namespace kin;
    CHECK(sqrt_ctl(0.1f, 2.0f, 5.0f, 0.0f) == 0.2f && fabsf(sqrt_ctl(10.0f, 2.0f, 5.0f, 0.0f) - sqrtf(2.0f * 5.0f * (10.0f - 0.5f * 5.0f / 4.0f))) < 1e-4f && sqrt_ctl(-10.0f, 2.0f, 5.0f, 0.0f) == -sqrt_ctl(10.0f, 2.0f, 5.0f, 0.0f),
          "sqrt controller: linear near, square root far, odd");
    CHECK(fabsf(sqrt_ctl(0.01f, 2.0f, 5.0f, 0.1f) - 0.02f) < 1e-6f && fabsf(sqrt_ctl(0.01f, 50.0f, 0.0f, 0.1f) - 0.1f) < 1e-6f, "the last step never asks for more than the whole error in dt (0.01 / 0.1 = 0.1)");
    CHECK(fabsf(sqrt_ctl(8.0f, 0.0f, 2.0f, 0.0f) - sqrtf(32.0f)) < 1e-5f && fabsf(sqrt_ctl(-8.0f, 0.0f, 2.0f, 0.0f) + sqrtf(32.0f)) < 1e-5f, "no P: pure sqrt(2 a e)");
    {   // a step of the wanted velocity 0 -> 2 m/s, accel 2.5, jerk 5
        float v = 0, a = 0, amax = 0, jmax = 0, vmax = 0; const float dt = 0.0025f; int n_reach = -1;
        for (int i = 0; i < 2000; i++) { const float a0 = a; update_vel_accel(v, a, dt, 0, 0); shape_vel_accel(2.0f, 0.0f, v, a, -2.5f, 2.5f, 5.0f, dt, false);
            amax = fmaxf(amax, fabsf(a)); jmax = fmaxf(jmax, fabsf(a - a0) / dt); vmax = fmaxf(vmax, v); if (n_reach < 0 && v > 1.96f) n_reach = i; }
        CHECK(amax <= 2.5f + 1e-4f && jmax <= 5.0f + 1e-3f, "acceleration never above 2.5 m/s^2 (%.3f), jerk never above 5 m/s^3 (%.3f)", amax, jmax);
        CHECK(vmax < 2.02f && fabsf(v - 2.0f) < 0.01f && n_reach > 0 && n_reach * dt < 2.5f, "2 m/s reached in %.2f s, overshoot %.3f m/s", n_reach * dt, vmax - 2.0f);
    }
    {   float v = 1.0f, p = 0; update_pos_vel_accel(p, v, 2.0f, 0.1f, 1.0f, 1.0f, 1.0f);
        CHECK(p == 0.0f && v == 1.0f, "limited upwards with the errors upwards: neither position nor velocity grows");
        v = -1.0f; update_vel_accel(v, 2.0f, 0.1f, 1.0f, 1.0f);
        CHECK(fabsf(v + 0.8f) < 1e-5f, "... but a velocity on the other side may come back towards 0 (%.2f)", v);
        v = 1.0f; p = 0; update_pos_vel_accel(p, v, 2.0f, 0.1f, 0.0f, 1.0f, 1.0f);
        CHECK(fabsf(p - 0.11f) < 1e-5f && fabsf(v - 1.2f) < 1e-5f, "not limited: p += v dt + a dt^2 / 2, v += a dt");
    }
    {   float v[2] = {0, 0}, a[2] = {0, 0}; const float want[2] = {3.0f, 4.0f}, ff[2] = {0, 0}; float amax = 0;
        for (int i = 0; i < 2000; i++) { update_vel_accel_xy(v, a, 0.0025f); shape_vel_accel_xy(want, ff, v, a, 2.5f, 5.0f, 0.0025f, false); amax = fmaxf(amax, hypotf(a[0], a[1])); }
        CHECK(amax <= 2.5f + 1e-3f && fabsf(v[0] - 3.0f) < 0.02f && fabsf(v[1] - 4.0f) < 0.02f, "2-D: the length of the acceleration is limited (%.3f), the velocity reaches (3, 4)", amax);
    }
}

// A craft on a vertical axis: thrust ~ motor output, the true hover output may differ from the controller's estimate; a force
// from outside (gust, prop wash) adds to it. The motors follow with a 30 ms lag.
struct VPlant { float z = 5, v = 0, a = 0, hover_true = 0.4f, out = 0.4f, push = 0; };
static void vstep(VPlant& p, AltHold& ah, AhInput& in, float dt) {
    in.z = p.z; in.v = p.v; in.a_up = p.a;
    const float thr = ah.update(in, dt), want = ah.to_motor(thr);
    p.out += (want - p.out) * dt / (0.03f + dt);
    p.a = kG0 * (p.out / p.hover_true - 1.0f) + p.push;
    p.v += p.a * dt; p.z += p.v * dt;
}
static void t_althold_cascade() {
    printf("[althold 0.28] AC_PosControl's vertical cascade: shaped target, measured-acceleration loop\n");
    const float dt = 0.0025f;
    auto craft = [&](float hover_est, float hover_true) { AltHold ah; ah.init(); ah.set_limits(0.0f, 1.0f); ah.hover = hover_est; ah.begin_flight(5.0f, ah.hover_throttle(), 1.0f); return ah; };
    {   // the hover estimate 20 % low: the acceleration loop's integrator takes it up, the height is held
        AltHold ah = craft(0.40f, 0.50f); VPlant p; p.hover_true = 0.5f; p.out = 0.4f; AhInput in; in.armed = true; in.stick01 = 0.5f;
        float lo = 5, hi = 5; for (int i = 0; i < 4000; i++) { vstep(p, ah, in, dt); lo = fminf(lo, p.z); hi = fmaxf(hi, p.z); }
        const float want_i = kG0 * (0.5f / 0.4f - 1.0f);
        CHECK(fabsf(p.z - 5.0f) < 0.03f && fabsf(p.v) < 0.02f && fabsf(ah.integrator() - want_i) < 0.15f, "hover estimate 0.40, true 0.50: holds 5 m (%.3f), the integrator carries %.2f m/s^2 (%.2f is the error)", p.z, ah.integrator(), want_i);
        CHECK(5.0f - lo < 0.5f, "and the sag before it is taken up stays small (%.2f m)", 5.0f - lo); }
    {   // a downdraught of 3 m/s^2 for one second: with the measured-acceleration loop the craft loses less height
        auto loss = [&](float acc_p, float acc_i) { AltHold ah = craft(0.40f, 0.40f); ah.cfg.acc_p = acc_p; ah.cfg.acc_i = acc_i; ah.cfg.ki_v = acc_i > 0 ? 0.0f : 3.0f;   // (without it: the old velocity integrator)
            VPlant p; AhInput in; in.armed = true; in.stick01 = 0.5f; for (int i = 0; i < 800; i++) vstep(p, ah, in, dt);
            float lo = p.z; p.push = -3.0f; for (int i = 0; i < 400; i++) { vstep(p, ah, in, dt); lo = fminf(lo, p.z); } p.push = 0;
            for (int i = 0; i < 2000; i++) { vstep(p, ah, in, dt); lo = fminf(lo, p.z); }
            return 5.0f - lo; };
        const float with = loss(0.5f, 1.0f), without = loss(0.0f, 0.0f);
        CHECK(with < 0.75f * without, "a 1 s downdraught of 3 m/s^2: %.3f m lost with the acceleration loop, %.3f m without it (velocity PI only, as before 0.28)", with, without); }
    {   // the stick from hold to full climb: the target accelerates with at most accel_z, the throttle does not step
        AltHold ah = craft(0.40f, 0.40f); VPlant p; AhInput in; in.armed = true; in.stick01 = 0.5f; for (int i = 0; i < 400; i++) vstep(p, ah, in, dt);
        in.stick01 = 1.0f; float amax = 0, step = 0, prev = ah.to_motor(ah.hover_throttle()); float vmax = 0;
        for (int i = 0; i < 2000; i++) { vstep(p, ah, in, dt); amax = fmaxf(amax, fabsf(ah.target_a())); step = fmaxf(step, fabsf(p.out - prev)); prev = p.out; vmax = fmaxf(vmax, p.v); }
        CHECK(amax <= ah.cfg.accel_z_mps2 + 1e-3f && fabsf(ah.target_v() - ah.cfg.max_climb_mps) < 0.01f, "full climb: the target's acceleration stays within %.1f m/s^2 (%.3f), its climb rate gets to %.1f m/s", ah.cfg.accel_z_mps2, amax, ah.target_v());
        CHECK(step < 0.004f && vmax < ah.cfg.max_climb_mps * 1.1f, "the motor output moves at most %.4f per 2.5 ms, the craft climbs at most %.2f m/s", step, vmax);
        in.stick01 = 0.5f; const float z_rel = p.z; float hi = p.z; for (int i = 0; i < 2000; i++) { vstep(p, ah, in, dt); hi = fmaxf(hi, p.z); }
        CHECK(hi - z_rel < 1.7f && fabsf(p.v) < 0.05f && fabsf(p.z - ah.target_z()) < 0.05f, "stick back to the middle at 2 m/s: stops %.2f m higher (about v^2 / 2a + the jerk ramp) and holds", hi - z_rel); }
    {   // taken over while climbing at 5 m/s (more than the 2 m/s allowed): the over-speed gain stops it briskly
        AltHold ah; ah.init(); ah.set_limits(0.0f, 1.0f); ah.hover = 0.4f; ah.begin_flight(5.0f, ah.hover_throttle(), 1.0f, 5.0f, 0.0f);
        VPlant p; p.v = 5.0f; AhInput in; in.armed = true; in.stick01 = 0.5f; float amax = 0, hi = p.z;
        for (int i = 0; i < 2000; i++) { vstep(p, ah, in, dt); amax = fmaxf(amax, fabsf(ah.target_a())); hi = fmaxf(hi, p.z); }
        CHECK(amax > ah.cfg.accel_z_mps2 * 1.5f && hi - 5.0f < 4.5f && fabsf(p.v) < 0.05f, "over-speed: the target decelerates at up to %.1f m/s^2 (limit %.1f in normal flight), the craft stops %.2f m higher (5^2 / (2 x 2.5) = 5 m without it)", amax, ah.cfg.accel_z_mps2, hi - 5.0f); }
    {   // the measured acceleration: falling at 2 m/s^2 while target, height and climb rate all agree -> more thrust at once
        AltHold ah; ah.init(); ah.set_limits(0.0f, 1.0f); ah.hover = 0.4f; ah.begin_flight(5.0f, ah.hover_throttle(), 1.0f);
        AhInput in; in.armed = true; in.stick01 = 0.5f; in.z = 5.0f; in.v = 0.0f; in.a_up = -2.0f; float thr = 0;
        for (int i = 0; i < 20; i++) thr = ah.update(in, dt);
        const float out = ah.to_motor(thr), want = 0.4f * (1.0f + 0.5f * 2.0f / kG0);
        CHECK(out > 0.4f * (1.0f + 0.4f * 2.0f / kG0) && out < want * 1.02f, "the craft starts to sink at 2 m/s^2 (a gust) before height or climb rate show it: the thrust rises at once (%.4f, hover 0.4, P 0.5 x 2 m/s^2 -> %.4f)", out, want); }
    {   // one vibration spike in the accelerometer: the 20 Hz filter takes most of it out
        AltHold ah; ah.init(); ah.set_limits(0.0f, 1.0f); ah.hover = 0.4f; ah.begin_flight(5.0f, ah.hover_throttle(), 1.0f);
        AhInput in; in.armed = true; in.stick01 = 0.5f; in.z = 5.0f; for (int i = 0; i < 20; i++) ah.update(in, dt);
        const float before = ah.to_motor(ah.update(in, dt)); in.a_up = 20.0f; const float spike = ah.to_motor(ah.update(in, dt));
        const float raw = 0.4f * 0.5f * 20.0f / kG0;
        CHECK(fabsf(spike - before) < 0.4f * raw, "a single 20 m/s^2 spike moves the thrust by %.4f (unfiltered it would be %.4f)", fabsf(spike - before), raw); }
    {   // the target is not let run away: the craft is held (it does not move), the stick asks for a climb
        AltHold ah; ah.init(); ah.set_limits(0.0f, 1.0f); ah.hover = 0.10f; ah.begin_flight(5.0f, ah.hover_throttle(), 1.0f);
        AhInput in; in.armed = true; in.stick01 = 1.0f; in.z = 5.0f; for (int i = 0; i < 1000; i++) ah.update(in, dt);
        CHECK(ah.target_z() - in.z <= 2.0f + 1e-4f && ah.target_z() - in.z > 1.9f, "held where it is, not saturated (a light craft): the target goes no further than 2 m ahead (%.2f m)", ah.target_z() - in.z);
        AltHold hv; hv.init(); hv.set_limits(0.0f, 1.0f); hv.hover = 0.70f; hv.begin_flight(5.0f, hv.hover_throttle(), 1.0f);
        for (int i = 0; i < 1000; i++) hv.update(in, dt);
        CHECK(hv.target_z() - in.z < 0.5f, "a heavy craft at full throttle: the target stops where the motors cannot follow (%.2f m ahead)", hv.target_z() - in.z); }
    {   // the hand-over keeps the throttle
        AltHold ah; ah.init(); ah.set_limits(0.055f, 1.0f); ah.hover = 0.4f; const float thr0 = 0.42f; ah.begin_flight(5.0f, thr0, 1.0f, 0.0f, 0.0f);
        AhInput in; in.armed = true; in.stick01 = 0.5f; in.z = 5.0f; const float thr1 = ah.update(in, 0.0025f);
        CHECK(fabsf(thr1 - thr0) < 0.005f, "Altitude Hold switched on in flight: the throttle continues (%.3f -> %.3f)", thr0, thr1); }
}

// ---- 0.32: the failsafe as PX4's framework: conditions -> actions, the most severe wins, fallbacks, user takeover
static void t_failsafe_actions() {
    printf("[failsafe 0.32] actions by condition, severity, HOLD, DROP, battery low, takeover\n");
    const float dt = 0.01f;
    auto run = [&](Failsafe& f, FsInput& in, float s) { FsOutput o; for (int i = 0; i < (int)(s / dt + 0.5f); i++) o = f.update(in, dt); return o; };
    {   Failsafe f; CHECK(f.action_of(0) == FA_NONE && f.action_of(FS_LINK) == FA_LAND && f.action_of(FS_BATTERY_LOW) == FA_WARN && f.action_of(FS_BATTERY) == FA_LAND, "defaults: link -> LAND, battery low -> WARN, minimum -> LAND");
        f.cfg.procedure = FSP_RESCUE; CHECK(f.action_of(FS_LINK) == FA_RTL && f.action_of(FS_LINK | FS_BATTERY) == FA_LAND, "RESCUE: RTL - but with the pack at its minimum LAND wins (the more severe)");
        f.cfg.procedure = FSP_HOLD; f.cfg.batt_low = FBL_RTL; CHECK(f.action_of(FS_SWITCH) == FA_HOLD && f.action_of(FS_SWITCH | FS_BATTERY_LOW) == FA_RTL, "HOLD < RTL");
        f.cfg.procedure = FSP_DROP; CHECK(f.action_of(FS_LINK | FS_BATTERY) == FA_DROP, "DROP is the most severe"); }
    {   // battery low, WARN (the default): nothing is flown, it is only said
        Failsafe f; FsInput in; in.armed = true; in.alt_valid = true; in.batt_low = true; const FsOutput o = run(f, in, 2.0f);
        CHECK(o.stage == FsStage::NONE && o.warn && o.action == FA_WARN && o.cause == FS_BATTERY_LOW && !o.alt_descent && !o.neutral_sticks, "battery low, WARN: stage NONE, a warning with its cause"); }
    {   // battery low, RTL: at once (no guard - the battery monitor has its delays), the pilot can take it back
        Failsafe f; f.cfg.batt_low = FBL_RTL; FsInput in; in.armed = true; in.alt_valid = true; in.vz = -0.5f; in.batt_low = true;
        FsOutput o = f.update(in, dt); o = f.update(in, dt);
        CHECK(o.stage == FsStage::DESCENT && o.rescue && o.action == FA_RTL && !o.neutral_sticks, "battery low, RTL: the rescue starts at once; the link is fine, the sticks stay the pilot's");
        in.rescue_active = true; o = run(f, in, 5.0f); CHECK(o.stage == FsStage::DESCENT && o.rescue, "flying home");
        in.stick_move = 0.6f; o = f.update(in, dt); in.stick_move = 0; in.rescue_active = false;
        CHECK(o.stage == FsStage::NONE && o.warn && !o.rescue, "the pilot moves a stick: he has the craft back, it stays a warning");
        o = run(f, in, 3.0f); CHECK(o.stage == FsStage::NONE && o.warn, "and the RTL is not asked for again while the pack is only low (PX4: user takeover)");
        in.batt_critical = true; o = run(f, in, 0.7f);
        CHECK(o.stage == FsStage::DESCENT && o.action == FA_LAND && o.alt_descent && !o.rescue, "the pack reaches its minimum: LAND (critical action), taking it back does not hold any more"); }
    {   // battery low, LAND; then the link goes: still LAND (not softened to the procedure's RTL)
        Failsafe f; f.cfg.batt_low = FBL_LAND; f.cfg.procedure = FSP_RESCUE; FsInput in; in.armed = true; in.alt_valid = true; in.vz = -0.5f; in.batt_low = true;
        FsOutput o = run(f, in, 0.1f); CHECK(o.action == FA_LAND && o.alt_descent, "battery low, LAND");
        in.rx_ok = false; o = run(f, in, 1.0f); CHECK(o.action == FA_LAND && !o.rescue && o.neutral_sticks && (o.cause & FS_LINK), "the link goes as well: still LAND - an action is never softened while its cause lasts"); }
    {   // RTL that cannot fly falls back to a landing (the caller does not start the rescue: rescue_active false)
        Failsafe f; f.cfg.procedure = FSP_RESCUE; f.cfg.max_descent_s = 5; FsInput in; in.armed = true; in.alt_valid = true; in.rx_ok = false; in.vz = 0;
        FsOutput o = run(f, in, 1.0f); CHECK(o.rescue && o.alt_descent, "RTL asked for; the descent is there for when the rescue cannot fly");
        o = run(f, in, 5.0f); CHECK(o.disarm, "and the time net counts while it lands on the spot instead (no rescue flying)"); }
    {   // HOLD: holds place and height for hold_s, then lands; without a height it lands at once
        Failsafe f; f.cfg.procedure = FSP_HOLD; f.cfg.hold_s = 10; f.cfg.max_descent_s = 5; FsInput in; in.armed = true; in.alt_valid = true; in.rx_ok = false; in.vz = 0;
        FsOutput o = run(f, in, 1.0f);
        CHECK(o.stage == FsStage::DESCENT && o.hold && o.action == FA_HOLD && !o.alt_descent && o.neutral_sticks, "link lost, HOLD: Altitude / Position Hold with neutral sticks, no descent");
        o = run(f, in, 8.0f); CHECK(o.hold && !o.disarm, "9 s of holding (the time net does not count: holding is not a stuck landing)");
        o = run(f, in, 2.0f); CHECK(!o.hold && o.alt_descent && o.action == FA_LAND, "10 s of holding: it lands");
        in.rx_ok = true; o = run(f, in, 1.2f); CHECK(o.stage == FsStage::NONE, "the link back for 1 s: the pilot has it");
        Failsafe g; g.cfg.procedure = FSP_HOLD; FsInput j; j.armed = true; j.alt_valid = false; j.rx_ok = false;
        o = run(g, j, 1.0f); CHECK(!o.hold && o.throttle_fallback && o.action == FA_LAND, "HOLD without a height: lands at once (the open-loop descent)"); }
    {   // a battery-low landing outlives a lost link that comes back: taking it back during the link's recovery does not count
        Failsafe f; f.cfg.batt_low = FBL_LAND; f.cfg.procedure = FSP_HOLD; FsInput in; in.armed = true; in.alt_valid = true; in.vz = -0.5f; in.batt_low = true;
        FsOutput o = run(f, in, 0.2f); in.rx_ok = false; o = run(f, in, 1.0f); CHECK(o.action == FA_LAND, "battery low LAND, then the link goes (HOLD would be milder): LAND");
        in.rx_ok = true; in.stick_move = 0.6f; o = run(f, in, 0.2f); in.stick_move = 0; o = run(f, in, 1.5f);
        CHECK(o.stage == FsStage::DESCENT && o.action == FA_LAND && (o.cause & FS_BATTERY_LOW), "a stick moved while the link still had to prove itself: the battery landing goes on");
        in.stick_move = 0.6f; o = f.update(in, dt); CHECK(o.stage == FsStage::NONE && o.warn, "moved again on a link that is back: the pilot has it"); in.stick_move = 0;
        in.batt_low = false; o = run(f, in, 0.1f); in.batt_low = true; o = run(f, in, 0.1f);
        CHECK(o.stage == FsStage::DESCENT && o.action == FA_LAND, "the pack recovers, then sags again: the landing is asked for again"); }
    {   // DROP is the most severe: a landing already going on becomes a motor stop when its cause comes
        Failsafe f; f.cfg.procedure = FSP_DROP; FsInput in; in.armed = true; in.alt_valid = true; in.vz = -0.5f; in.batt_critical = true;
        FsOutput o = run(f, in, 1.0f); CHECK(o.stage == FsStage::DESCENT && o.action == FA_LAND, "battery landing");
        in.rx_ok = false; o = run(f, in, 0.1f); CHECK(o.disarm && o.stage == FsStage::LANDED, "the link goes, procedure DROP: motors off"); }
    {   // DROP: motors off after the guard time
        Failsafe f; f.cfg.procedure = FSP_DROP; FsInput in; in.armed = true; in.alt_valid = true; in.rx_ok = false;
        FsOutput o = run(f, in, 0.3f); CHECK(o.stage == FsStage::GUARD && !o.disarm, "DROP: the guard time first (a blip of the link does not stop the motors)");
        o = run(f, in, 0.3f); CHECK(o.stage == FsStage::LANDED && o.disarm, "then the motors stop"); }
}

int main() {
    t_stall_guard();
    t_fast_math();
    t_minmax();
    t_filters(); t_ahrs(); t_ahrs_trust(); t_vertical(); t_vertical_gps(); t_horizontal(); t_rc(); t_mixer(); t_layouts(); t_safety(); t_althold_unit(); t_hover_estimator(); t_althold_takeoff(); t_althold_edges(); t_kinematics(); t_althold_cascade(); t_failsafe_actions();
    printf("\n%d passed, %d failed\n", g_pass, g_fail); return g_fail ? 1 : 0;
}
