// Unit tests of the rewritten navigation estimators (src/core/nav/): StateHistory, AltEstimator, PosEstimator.
// Closed-loop tests with the full FlightCore live in test_flight.cpp and test_fw.cpp.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <random>
#include "../src/core/nav/state_history.h"
#include "../src/core/nav/alt_estimator.h"
#include "../src/core/nav/pos_estimator.h"
using namespace fc;
using namespace fc::nav;

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, ...) do { const bool ok_ = (cond); if (getenv("V")) { printf("    "); printf(__VA_ARGS__); printf("\n"); } if (ok_) g_pass++; else { g_fail++; printf("  FAIL line %d: %s   ", __LINE__, #cond); printf(__VA_ARGS__); printf("\n"); } } while (0)
static std::mt19937 rng(4321); static float gauss(float s) { static std::normal_distribution<float> n(0, 1); return n(rng) * s; }

static void t_history() {
    printf("[state history] the past states a late measurement is compared with\n");
    StateHistory<2, 8> h; float x[2] = {0, 0}, o[2] = {9, 9};
    CHECK(!h.at(0.1f, o) && o[0] == 9, "empty: nothing to give");
    for (int i = 1; i <= 5; i++) { const float s[2] = {(float)i, 10.0f * i}; h.push(s, 0.01f); }
    h.at(0.0f, o); CHECK(o[0] == 5 && o[1] == 50, "delay 0: the newest state");
    h.at(0.02f, o); CHECK(o[0] == 3, "20 ms ago at 100 Hz: two steps back (%g)", o[0]);
    h.at(0.024f, o); CHECK(o[0] == 3, "24 ms: the nearest step at or before (%g)", o[0]);
    h.at(1.0f, o); CHECK(o[0] == 1, "longer than stored: the oldest (%g)", o[0]);
    for (int i = 6; i <= 20; i++) { const float s[2] = {(float)i, 0}; h.push(s, 0.01f); }
    h.at(1.0f, o); CHECK(h.size() == 8 && o[0] == 13, "full ring of 8: the oldest is 13 (%g)", o[0]);
    const float d[2] = {0.5f, -1.0f}; h.shift(d); h.at(0.0f, o); float old[2] = {0, 0}; h.at(1.0f, old);
    CHECK(o[0] == 20.5f && o[1] == -1.0f && old[0] == 13.5f, "a correction moves the whole stored past");
    StateHistory<1, 4> z; const float s1[1] = {1}; z.push(s1, -1.0f); z.push(s1, NAN); float q[1]; CHECK(z.at(0.5f, q) && q[0] == 1, "a bad dt is stored as 0 (no NaN walks the search)");
    (void)x;
}

// the craft's true height over time; the estimator gets the down acceleration (+ an error) and the sensors
struct Truth { float h = 0, v = 0; };

static void t_alt_basic() {
    printf("[alt estimator] NED, baro + lidar, outliers, sigmas\n");
    const float dt = 0.005f, bias = 0.3f;
    { AltEstimator e; e.reset(2.0f); float maxerr = 0, maxv = 0;
      for (int i = 0; i < 12000; i++) { const float t = i * dt; e.predict(bias + gauss(0.3f), dt);
          if (i % 20 == 0) e.update_baro(2.0f + 0.05f * t + gauss(0.3f), 0.3f);
          if (i % 40 == 0) e.update_range(2.0f + gauss(0.03f), 0.03f);
          if (t > 15) { maxerr = fmaxf(maxerr, fabsf(e.height() - 2.0f)); maxv = fmaxf(maxv, fabsf(e.vel_d())); } }
      CHECK(fabsf(e.pos_d() + e.height()) < 1e-6f && maxerr < 0.15f && maxv < 0.2f, "NED: pos_d = -height; baro drifting 5 cm/s + lidar: error %.3f m, |vel_d| %.3f", maxerr, maxv);
      CHECK(fabsf(e.baro_offset() - 3.0f) < 0.4f && fabsf(e.accel_bias() - bias) < 0.15f, "learned baro offset %.2f m, accelerometer bias %.2f (down)", e.baro_offset(), e.accel_bias()); }
    { AltEstimator e; e.reset(1.0f); for (int i = 0; i < 2000; i++) { e.predict(0, dt); if (i % 20 == 0) e.update_range(1.0f + gauss(0.03f), 0.03f); }
      const bool used = e.update_range(6.0f, 0.03f);
      CHECK(!used && fabsf(e.height() - 1.0f) < 0.1f && !e.range_innov().used && e.range_innov().ratio > 1.0f && fabsf(e.range_innov().value - 5.0f) < 0.1f,
            "lidar outlier gated: innovation %.2f m logged, test ratio %.0f (> 1 = rejected)", e.range_innov().value, e.range_innov().ratio); }
    { Truth tr; AltEstimator e; e.reset(0);
      for (int i = 0; i < 1600; i++) { const float a_up = i * dt < 1 ? 1.0f : 0.0f; tr.v += a_up * dt; tr.h += tr.v * dt; e.predict(-a_up + gauss(0.2f), dt); if (i % 20 == 0) e.update_baro(tr.h + gauss(0.3f), 0.3f); }
      CHECK(fabsf(e.height() - tr.h) < 0.5f && e.climb_rate() > 0.5f, "a climb (up = negative down acceleration): height %.2f, true %.2f, climbing %.2f m/s", e.height(), tr.h, e.climb_rate()); }
    { AltEstimator a, b; a.reset(0); b.reset(0);
      for (int i = 0; i < 4000; i++) { a.predict(0, dt); b.predict(0, dt); if (i % 20 == 0) { const float n = gauss(0.05f); a.update_baro(n, 0.05f); b.update_baro(n, 5.0f); } }
      CHECK(a.height_sigma() < b.height_sigma(), "the sensor layer's sigma counts: %.3f vs %.3f m (clamped to 0.3 .. 1.0)", a.height_sigma(), b.height_sigma()); }
    { AltEstimator e; e.reset(0); const float h0 = e.height(), v0 = e.vel_d();
      e.predict(1.0f, -0.01f); e.predict(1.0f, NAN); e.predict(1.0f, 0.5f); e.predict(NAN, 0.005f);
      CHECK(e.height() == h0 && e.vel_d() == v0, "bad dt (negative, NaN, 0.5 s) or a NaN acceleration: nothing happens");
      CHECK(!e.update_baro(NAN) && !e.update_range(NAN) && !e.update_gps_vel(NAN, 0.1f) && e.height() == h0, "NaN readings are not fused"); }
}

static void t_alt_resets() {
    printf("[alt estimator] the height taken from a reading outright: counted, with its step\n");
    const float dt = 0.0005f;
    { AltEstimator e; e.reset(0); float worst = 0;
      for (int i = 0; i < 4000; i++) { e.predict(gauss(0.1f), dt); if (i % 40 == 0) e.update_baro(gauss(0.2f), 0.3f); }
      const uint8_t c0 = e.reset_count();
      for (int i = 0; i < 200; i++) e.predict(40.0f, dt);                          // 0.1 s of +4 g DOWN (a clipped accelerometer on a hard landing)
      for (int i = 0; i < 6000; i++) e.predict(gauss(0.1f), dt);                    // 3 s without a baro reading: 12 m "below"
      const float before = e.height();
      for (int i = 0; i < 8000; i++) { e.predict(gauss(0.1f), dt); if (i % 40 == 0) e.update_baro(gauss(0.2f), 0.3f); if (i * dt > 2.0f) worst = fmaxf(worst, fabsf(e.height())); }
      CHECK(before < -8.0f && e.baro_resets() >= 1 && e.baro_resets() <= 2 && worst < 0.6f && fabsf(e.vel_d()) < 0.3f, "run away to %.1f m: reset to the barometer after %.1f s of rejected readings (%u), then within %.2f m",
            before, e.cfg.baro_reset_s, (unsigned)e.baro_resets(), worst);
      CHECK((uint8_t)(e.reset_count() - c0) == e.baro_resets() && e.reset_step_d() < -8.0f, "the reset is counted (%u) and its step kept: pos_d moved by %.1f m (up)", (unsigned)(e.reset_count() - c0), e.reset_step_d()); }
    { AltEstimator e; e.reset(0); float maxv = 0;
      for (int i = 0; i < 40000; i++) { e.predict(-0.3f, dt); if (i % 40 == 0) { e.update_baro(gauss(0.5f), 1.0f); e.update_still(); } if (i > 20000) maxv = fmaxf(maxv, fabsf(e.vel_d())); }
      CHECK(maxv < 0.15f && e.baro_resets() <= 1, "disarmed, a 0.3 m/s^2 accelerometer error, a noisy baro: the zero-velocity update keeps |vel_d| <= %.2f", maxv); }
    { AltEstimator e; e.reset(0); e.cfg.gps_reref_after = 5;
      for (int i = 0; i < 4000; i++) { e.predict(0, 0.005f); if (i % 20 == 0) e.update_gps_abs(10.0f + gauss(0.3f), 0.5f); }
      const uint8_t c0 = e.reset_count(); const uint32_t g0 = e.gps_resets(); bool all_rej = true;     // (starting at 0 m, the first readings were already taken that way)
      for (int k = 0; k < 4; k++) {
          for (int i = 0; i < 20; i++) e.predict(0, 0.005f);
          all_rej &= !e.update_gps_abs(40.0f, 0.5f); }
      for (int i = 0; i < 20; i++) e.predict(0, 0.005f);
      const bool taken = e.update_gps_abs(40.0f, 0.5f);
      CHECK(all_rej && taken && fabsf(e.height() - 40.0f) < 0.1f && e.gps_resets() == g0 + 1 && (uint8_t)(e.reset_count() - c0) == 1 && fabsf(e.reset_step_d() + 30.0f) < 0.5f,
            "GPS as the height jumps by 30 m: 4 readings rejected, the 5th taken outright - one counted reset, step %.1f m", e.reset_step_d()); }
}

static void t_alt_delay() {
    printf("[alt estimator] late measurements compared with the state when they were taken\n");
    // a craft climbing at 2 m/s then stopping; GPS vertical velocity (the only aiding) arrives 150 ms late. The accelerometer
    // has a bias of 0.4 m/s^2 that only the GPS velocity can show.
    auto run = [](float delay_cfg, float& worst) {
        AltEstimator e; e.cfg.gps_delay_s = delay_cfg; e.reset(0); Truth tr; const float dt = 0.002f; float vq[200] = {}; int qn = 0; worst = 0;
        for (int i = 0; i < 10000; i++) {
            const float t = i * dt; const float a_up = (t > 8 && t < 9) ? 2.0f : (t > 12 && t < 13) ? -2.0f : 0.0f;
            tr.v += a_up * dt; tr.h += tr.v * dt;
            vq[qn % 200] = tr.v; qn++;
            e.predict(-a_up + 0.4f, dt);
            if (i % 50 == 0 && qn > 75) e.update_gps_vel(-vq[(qn - 1 - 75) % 200], 0.1f);     // 75 steps = 150 ms old
            if (t > 5) worst = fmaxf(worst, fabsf(e.climb_rate() - tr.v));
        }
        return e.accel_bias(); };
    float w0, w1; run(0.0f, w0); const float b = run(0.15f, w1);
    CHECK(w1 < 0.6f * w0 && w1 < 0.25f, "the climb rate through a start and a stop: worst error %.2f m/s compared with the state 150 ms back, %.2f m/s compared with now", w1, w0);
    CHECK(fabsf(b - 0.4f) < 0.1f, "the accelerometer bias is still found (%.2f m/s^2 of 0.4)", b);
}

static void t_alt_range() {
    printf("[alt estimator] the range finder over changing ground\n");
    const float dt = 0.005f;
    AltEstimator e; e.reset(0); float worst = 0;
    for (int i = 0; i < 2000; i++) { e.predict(0, dt); if (i % 20 == 0) e.update_baro(1.0f + gauss(0.2f), 0.3f); if (i % 4 == 0) e.update_range(1.0f + gauss(0.02f), 0.02f, true); }
    for (int i = 0; i < 2000; i++) { e.predict(0, dt); if (i % 20 == 0) e.update_baro(1.0f + gauss(0.2f), 0.3f); if (i % 4 == 0) e.update_range(0.65f + gauss(0.02f), 0.02f, true); worst = fmaxf(worst, fabsf(e.height() - 1.0f)); }
    CHECK(e.range_reanchors() == 1 && fabsf(e.range_ground() - 0.35f) < 0.08f && worst < 0.12f, "a table 35 cm high: the ground moves (%.2f m, %u re-anchor), the height does not (off by %.3f m at most)", e.range_ground(), (unsigned)e.range_reanchors(), worst);
    for (int i = 0; i < 200; i++) e.predict(0, dt);
    const bool first = e.update_range(3.0f, 0.02f, true);
    CHECK(!first && e.range_reanchors() == 2 && fabsf(e.range_ground() + 2.0f) < 0.15f, "back after a 1 s pause: a new stretch of ground, the first reading anchors (%.2f m)", e.range_ground());
    int used = 0; for (int i = 0; i < 40; i++) { e.predict(0, dt); if (i % 4 == 0 && e.update_range(3.0f + gauss(0.02f), 0.02f, true)) used++; }
    CHECK(used == 10 && e.range_reanchors() == 2 && e.range_used(), "the readings after the new anchor are used (%d of 10), no further anchors", used);
    // one odd reading (a hand under it), then odd and good ones taking turns: never a new anchor
    e.update_range(1.0f, 0.02f, true); for (int i = 0; i < 4; i++) e.predict(0, dt);
    for (int k = 0; k < 30; k++) { for (int i = 0; i < 4; i++) e.predict(0, dt); e.update_range(k % 2 ? 1.0f : 3.0f + gauss(0.02f), 0.02f, true); }
    CHECK(e.range_reanchors() == 2, "a single odd reading, then odd and good ones taking turns for 0.6 s: no new anchor (%u)", (unsigned)e.range_reanchors());
    // after an anchor moved by odd readings, the count starts again: one more odd reading is not a new ground
    for (int k = 0; k < 12 && e.range_reanchors() == 2; k++) { for (int i = 0; i < 4; i++) e.predict(0, dt); e.update_range(2.0f + gauss(0.02f), 0.02f, true); }      // (up to the new anchor, not one reading more)
    const uint32_t r1 = e.range_reanchors();
    for (int i = 0; i < 4; i++) e.predict(0, dt);
    e.update_range(0.5f, 0.02f, true);
    CHECK(r1 == 3 && e.range_reanchors() == 3, "a 1 m step for good: one new anchor (%u); one odd reading right after it: none more (%u)", (unsigned)r1, (unsigned)e.range_reanchors());
    // without a barometer the range finder IS the height: no anchors, not even after a pause
    AltEstimator a; a.reset(0);
    for (int i = 0; i < 400; i++) { a.predict(0, dt); if (i % 4 == 0) a.update_range(1.0f + gauss(0.02f), 0.02f, false); }
    for (int i = 0; i < 200; i++) a.predict(0, dt);
    const bool after_pause = a.update_range(1.0f, 0.02f, false);
    for (int k = 0; k < 20; k++) { for (int i = 0; i < 4; i++) a.predict(0, dt); a.update_range(5.0f, 0.02f, false); }
    CHECK(after_pause && a.range_reanchors() == 0 && fabsf(a.range_ground()) < 1e-6f, "no barometer: after a pause the reading is used as it is, 20 odd readings move no anchor");
}

static void t_alt_shift() {
    printf("[alt estimator] a late correction is remembered: no double counting\n");
    // the height jumps to 2 m (the estimate starts at 0); GPS height every 100 ms, 150 ms late. Each solution is compared with
    // the state 150 ms back - which must already carry the corrections made since, or the same error is corrected twice.
    AltEstimator e; e.cfg.gps_delay_s = 0.15f; e.cfg.gps_reref_after = 200; e.reset(0); const float dt = 0.005f; float top = 0;
    for (int i = 0; i < 1000; i++) { e.predict(0, dt); if (i % 20 == 0) e.update_gps_abs(2.0f, 0.3f); if (i > 30) top = fmaxf(top, e.height()); }
    CHECK(top < 2.3f && fabsf(e.height() - 2.0f) < 0.05f, "settles at %.3f m, never above %.3f m (without the corrected past: 2.76)", e.height(), top);
}

static void t_alt_gps() {
    printf("[alt estimator] GPS height: the reference next to a barometer\n");
    const float dt = 0.005f;
    AltEstimator e; e.reset(0);
    for (int i = 0; i < 4000; i++) { e.predict(0, dt); if (i % 20 == 0) e.update_baro(gauss(0.2f), 0.3f); if (i % 20 == 0) e.update_gps_rel(55.0f + gauss(0.5f), 1.0f, false); }
    CHECK(e.gps_ref_ok() && fabsf(e.gps_ref() - 55.0f) < 0.6f && !e.gps_used(), "not fused: the reference is GPS - height (%.2f, true 55)", e.gps_ref());
    int used = 0; for (int i = 0; i < 2000; i++) { e.predict(0, dt); if (i % 20 == 0) { e.update_baro(gauss(0.2f), 0.3f); if (e.update_gps_rel(55.0f + gauss(0.5f), 3.0f, true)) used++; } }
    CHECK(used > 80 && e.gps_used() && fabsf(e.height()) < 0.3f, "fused: only its change counts - %d readings used, height %.2f", used, e.height());
    const bool jump = e.update_gps_rel(62.0f, 3.0f, true);
    CHECK(!jump, "a 7 m jump (more than gps_jump_m) is not used");
}

// a craft flying north: 0 -> 5 m/s and back to 0, GPS at 10 Hz, 150 ms late, a little noise; acc_err = accelerometer error
struct Fly2D {
    float pn = 0, vn = 0, t = 0; float qp[100] = {}, qv[100] = {}; int qn = 0;
    float accel(float tt) const { return (tt > 3 && tt < 5) ? 2.5f : (tt > 9 && tt < 11) ? -2.5f : 0.0f; }
    void step(float dt) { const float a = accel(t); vn += a * dt; pn += vn * dt; t += dt; qp[qn % 100] = pn; qv[qn % 100] = vn; qn++; }
    float late_p(int n) const { return qp[(qn - 1 - n) % 100]; } float late_v(int n) const { return qv[(qn - 1 - n) % 100]; }
};

static void t_pos() {
    printf("[pos estimator] GPS late, glitches, resets, flow\n");
    auto run = [](float delay_cfg, float& worst_v) {
        PosEstimator e; e.cfg.gps_delay_s = delay_cfg; e.reset(); Fly2D f; const float dt = 0.005f; worst_v = 0;
        for (int i = 0; i < 3000; i++) { e.predict(f.accel(f.t) + gauss(0.2f), gauss(0.2f), dt); f.step(dt);
            if (i % 20 == 0 && f.qn > 30) e.fuse_gps(f.late_p(30) + gauss(0.3f), gauss(0.3f), f.late_v(30) + gauss(0.05f), gauss(0.05f), 0.8f, 0.15f);
            if (f.t > 2) worst_v = fmaxf(worst_v, fabsf(e.vel_n() - f.vn)); }
        return e; };
    float w0, w1; run(0.0f, w0); const PosEstimator e = run(0.15f, w1);
    CHECK(w1 < 0.7f * w0, "speeding up to 5 m/s and stopping, GPS 150 ms late: worst velocity error %.2f m/s against the delayed state, %.2f m/s against the present", w1, w0);
    CHECK(e.initialised() && e.reset_count() == 0 && e.gps_innov().used, "no glitch, no reset; the last solution was used (ratio %.2f)", e.gps_innov().ratio);
    { PosEstimator p; p.reset(); const float dt = 0.005f; int rejected = 0;
      for (int i = 0; i < 2000; i++) { p.predict(0, 0, dt); if (i % 20 == 0) p.fuse_gps(10.0f + gauss(0.2f), -5.0f + gauss(0.2f), 0, 0, 0.8f, 0.15f); }
      if (!p.fuse_gps(40.0f, -5.0f, 0, 0, 0.8f, 0.15f)) rejected++;
      for (int i = 0; i < 100; i++) { p.predict(0, 0, dt); if (i % 20 == 0) p.fuse_gps(10.0f + gauss(0.2f), -5.0f + gauss(0.2f), 0, 0, 0.8f, 0.15f); }
      CHECK(rejected == 1 && fabsf(p.pos_n() - 10.0f) < 0.3f && p.reset_count() == 0 && !p.gps_rejecting(), "a single 30 m jump: rejected (ratio > 1), the position stays at %.2f, no reset", p.pos_n());
      // the fix moves for good (a new constellation): rejected for glitch_reset_s, then taken outright - a counted reset
      int n_rej = 0; float t_reset = -1;
      for (int i = 0; i < 1000; i++) { p.predict(0, 0, dt); if (i % 20 == 0) { const bool ok = p.fuse_gps(25.0f + gauss(0.2f), 3.0f + gauss(0.2f), 0, 0, 0.8f, 0.15f); if (!ok) n_rej++; if (p.reset_count() == 1 && t_reset < 0) t_reset = i * dt; } }
      CHECK(p.reset_count() == 1 && p.glitch_resets() == 1 && t_reset > 1.8f && t_reset < 2.3f && fabsf(p.reset_step_n() - 15.0f) < 0.6f && fabsf(p.reset_step_e() - 8.0f) < 0.6f && fabsf(p.pos_n() - 25.0f) < 0.4f,
            "the fix moves 15 m north, 8 m east for good: %d rejected, reset after %.2f s with the step (%.1f, %.1f) - the position is the GPS's again", n_rej, t_reset, p.reset_step_n(), p.reset_step_e()); }
    { PosEstimator p; p.reset(); const float dt = 0.005f; float worst = 0;
      for (int i = 0; i < 2000; i++) { const float vt = i * dt < 5 ? 1.0f : -0.5f; p.predict(gauss(0.3f), gauss(0.3f), dt); if (i % 10 == 0) p.fuse_velocity(vt + gauss(0.1f), 0.2f + gauss(0.1f), 0.15f, 0.05f);
          if (i * dt > 1 && fabsf(i * dt - 5) > 1) worst = fmaxf(worst, fabsf(p.vel_n() - vt)); }
      CHECK(worst < 0.25f && fabsf(p.vel_e() - 0.2f) < 0.15f, "optical flow alone: the velocity follows (worst %.2f m/s off)", worst); }
    { PosEstimator p; p.reset(); p.fuse_gps(1, 2, 0, 0, 1, 0.2f); const float n0 = p.pos_n();
      p.predict(1.0f, 1.0f, -0.01f); p.predict(1.0f, 1.0f, NAN); p.predict(NAN, 0, 0.005f); p.predict(1.0f, 1.0f, 0.5f);
      CHECK(p.pos_n() == n0 && p.vel_n() == 0.0f && !p.fuse_gps(NAN, 0, 0, 0, 1, 1) && !p.fuse_velocity(0, NAN, 0.1f, 0), "bad dt or NaN: nothing changes"); }
}

int main() {
    t_history();
    t_alt_basic(); t_alt_resets(); t_alt_delay(); t_alt_range(); t_alt_shift(); t_alt_gps();
    t_pos();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
