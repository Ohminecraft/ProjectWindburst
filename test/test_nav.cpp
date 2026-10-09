// Unit tests of the navigation blocks: optical flow -> velocity (flow.h), the tilt from a measured acceleration (tilt_aid.h), the
// horizontal position controller (poscontrol.h) and Position Hold (poshold.h). Each block alone, against a model that is written
// here from the physics (a point mass that leans, with drag and wind), not from the code under test.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <random>
#include <vector>
#include <string>
#include "../src/core/flow.h"
#include "../src/core/ahrs.h"
#include "../src/core/tilt_aid.h"
#include "../src/core/poscontrol.h"
#include "../src/core/poshold.h"
#include "../src/core/rescue.h"
#include "../src/core/horizontal_est.h"
using namespace fc;

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, ...) do { const bool ok_ = (cond); if (getenv("V")) { printf("    "); printf(__VA_ARGS__); printf("\n"); } if (ok_) g_pass++; else { g_fail++; printf("  FAIL line %d: %s   ", __LINE__, #cond); printf(__VA_ARGS__); printf("\n"); } } while (0)
static std::mt19937 rng(777); static float gauss(float s) { static std::normal_distribution<float> n(0, 1); return n(rng) * s; }

// ---------------------------------------------------------------------------------------------------------------------
// optical flow
// ---------------------------------------------------------------------------------------------------------------------
static void t_flow() {
    printf("[flow] the picture moves because the craft turns and because it travels\n");
    FlowConfig c;
    // The sensor looks down. Travelling FORWARD, the ground runs backwards through the picture: the same as the sensor turning
    // nose-up... no: nose-DOWN is a negative rotation about Y (FRD: +Y rotation lifts the nose). Travelling forward at v over a
    // ground h below shifts the picture like a rotation of +v / h about Y (ArduPilot: flow_y = +v_forward / h).
    { FlowSample s; s.quality = 90; s.y = 2.0f / 1.5f; const FlowVelocity v = flow_velocity(c, s, 0, 0, 1.5f, 1.0f);
      CHECK(v.valid && fabsf(v.forward - 2.0f) < 1e-4f && fabsf(v.right) < 1e-5f, "2 m/s forward, 1.5 m up: flow about Y = +1.33 rad/s -> forward %.3f, right %.3f", v.forward, v.right); }
    { FlowSample s; s.quality = 90; s.x = -1.0f / 2.0f; const FlowVelocity v = flow_velocity(c, s, 0, 0, 2.0f, 1.0f);
      CHECK(v.valid && fabsf(v.right - 1.0f) < 1e-4f && fabsf(v.forward) < 1e-5f, "1 m/s to the right, 2 m up: flow about X = -0.5 rad/s -> right %.3f", v.right); }
    { FlowSample s; s.quality = 90; s.x = 0.8f; s.y = -0.5f; const FlowVelocity v = flow_velocity(c, s, 0.8f, -0.5f, 2.0f, 1.0f);
      CHECK(v.valid && fabsf(v.forward) < 1e-5f && fabsf(v.right) < 1e-5f, "rolling and pitching on the spot: the flow is the gyro's, the velocity is zero (%.4f, %.4f)", v.forward, v.right); }
    { FlowSample s; s.quality = 90; s.x = 0.3f - 1.0f / 2.0f; s.y = -0.2f + 3.0f / 2.0f; const FlowVelocity v = flow_velocity(c, s, 0.3f, -0.2f, 2.0f, 1.0f);
      CHECK(fabsf(v.forward - 3.0f) < 1e-4f && fabsf(v.right - 1.0f) < 1e-4f, "both at once: the gyro's part is taken away (%.3f forward, %.3f right)", v.forward, v.right); }
    { FlowSample s; s.quality = 90; s.y = 1.0f; const FlowVelocity level = flow_velocity(c, s, 0, 0, 2.0f, 1.0f), tilted = flow_velocity(c, s, 0, 0, 2.0f, cosf(30 * kDeg2Rad));
      CHECK(fabsf(level.forward - 2.0f) < 1e-4f && fabsf(tilted.forward - 2.0f / cosf(30 * kDeg2Rad)) < 1e-4f, "tilted 30 deg the ground is further away along the lens: height / cos(tilt) (%.3f instead of %.3f m/s)", tilted.forward, level.forward); }
    { FlowSample s; s.quality = 90; s.y = 1.0f; FlowConfig m = c; m.yaw_deg = 90; m.scale = 1.1f; const FlowVelocity v = flow_velocity(m, s, 0, 0, 1.0f, 1.0f);
      // sensor turned 90 deg clockwise: its Y axis points backwards; a rotation about the sensor's Y is one about the craft's -X
      CHECK(fabsf(v.right - 1.1f) < 1e-4f && fabsf(v.forward) < 1e-4f, "sensor mounted 90 deg round, lens scale 1.1: its Y flow is the craft's -X flow -> %.2f m/s to the right", v.right); }
    // not used when it cannot be trusted
    { FlowSample s; s.y = 1.0f; s.quality = 29; const FlowVelocity q = flow_velocity(c, s, 0, 0, 1.0f, 1.0f); s.quality = 30; const FlowVelocity q2 = flow_velocity(c, s, 0, 0, 1.0f, 1.0f);
      CHECK(!q.valid && q.reject == FLOW_QUALITY && q.forward == 0 && q2.valid && q2.reject == FLOW_OK, "surface quality 29 of 100: not used (30: used)");
      s.quality = 90;
      const FlowVelocity lo = flow_velocity(c, s, 0, 0, 0.09f, 1.0f), lo2 = flow_velocity(c, s, 0, 0, 0.10f, 1.0f), hi = flow_velocity(c, s, 0, 0, 4.01f, 1.0f), hi2 = flow_velocity(c, s, 0, 0, 4.0f, 1.0f), nan = flow_velocity(c, s, 0, 0, NAN, 1.0f);
      CHECK(!lo.valid && lo.reject == FLOW_HEIGHT && lo2.valid && !hi.valid && hi.reject == FLOW_HEIGHT && hi2.valid && !nan.valid, "closer than 0.1 m or further than 4 m (or no height at all): not used");
      const FlowVelocity t = flow_velocity(c, s, 0, 0, 1.0f, 0.70f), t2 = flow_velocity(c, s, 0, 0, 1.0f, 0.72f);
      CHECK(!t.valid && t.reject == FLOW_TILT && t2.valid, "tilted more than 45 deg: not used");
      const float lim = 200 * kDeg2Rad; const FlowVelocity g1 = flow_velocity(c, s, lim * 1.01f, 0, 1.0f, 1.0f), g2 = flow_velocity(c, s, 0, -lim * 1.01f, 1.0f, 1.0f), g3 = flow_velocity(c, s, lim * 0.99f, lim * 0.99f, 1.0f, 1.0f);
      CHECK(!g1.valid && g1.reject == FLOW_TURNING && !g2.valid && g2.reject == FLOW_TURNING && g3.valid, "turning faster than 200 deg/s about roll or pitch: not used"); }
    { FlowSample s; s.y = 1.0f; s.quality = 100; const float a = flow_velocity(c, s, 0, 0, 0.5f, 1.0f).sigma, b = flow_velocity(c, s, 0, 0, 3.0f, 1.0f).sigma; s.quality = 47; const float d = flow_velocity(c, s, 0, 0, 3.0f, 1.0f).sigma;
      CHECK(fabsf(a - 0.10f) < 1e-4f && fabsf(b - 0.30f) < 1e-4f && fabsf(d - 0.30f / sqrtf(17.0f / 70.0f)) < 1e-3f, "how sure: 0.1 rad/s of noise x the distance, at least 0.10 m/s (%.2f at 0.5 m, %.2f at 3 m), less sure on a poor surface (%.2f at quality 47)", a, b, d); }
    // the gyro of the same stretch of time
    { GyroHistory h; h.configure(0.002f); float gx = 9, gy = 9;
      CHECK(!h.average(0, 0.02f, gx, gy) && h.span_s() == 0, "no history yet: no answer");
      for (int i = 0; i < 100; i++) h.push(0.01f * i, -0.02f * i);            // sample i is (100 - 1 - i) steps old
      CHECK(h.average(0.0f, 0.020f, gx, gy) && fabsf(gx - 0.945f) < 1e-4f && fabsf(gy + 1.89f) < 1e-4f, "the last 20 ms: mean of the last 10 samples (%.3f)", gx);
      CHECK(h.average(0.050f, 0.020f, gx, gy) && fabsf(gx - 0.695f) < 1e-4f, "20 ms that ended 50 ms ago: samples 65 .. 74 (%.3f)", gx);
      CHECK(h.average(0.050f, 0.0005f, gx, gy) && fabsf(gx - 0.74f) < 1e-4f, "a window shorter than a step is one sample (%.2f)", gx);
      CHECK(!h.average(0.150f, 0.060f, gx, gy) && h.average(0.150f, 0.050f, gx, gy), "a window that reaches back further than the history: no answer");
      for (int i = 0; i < 300; i++) h.push(1.0f, 2.0f);
      CHECK(fabsf(h.span_s() - 0.256f) < 1e-5f && h.average(0.2f, 0.056f, gx, gy) && gx == 1.0f && gy == 2.0f && !h.average(0.2f, 0.058f, gx, gy), "it keeps the last 128 steps (256 ms at 500 Hz)");
      h.reset(); CHECK(!h.average(0, 0.002f, gx, gy), "reset() empties it"); }
}

// ---------------------------------------------------------------------------------------------------------------------
// tilt from a measured acceleration
// ---------------------------------------------------------------------------------------------------------------------
// A craft whose true attitude is known flies a manoeuvre; its accelerometer reads what physics says (thrust along the craft's
// axis + drag, no gravity); a GPS reports the velocity late. The estimator starts wrong or is misled by the acceleration.
struct AidRig {
    Ahrs truth, est; TiltAid aid; float vn = 0, ve = 0, drag = 0.35f, delay = 0.15f, vel_noise = 0, given_delay = -1; bool use_aid = true; float rate_hz = 10;
    std::vector<float> hist_n, hist_e; double t = 0, next_fix = 0.3; float max_err = 0; int fixes = 0, used = 0;
    // roll (rad) of the truth is driven to `roll_deg` at 60 deg/s; thrust holds the height
    void run(float seconds, float roll_deg, const float dt = 0.002f) {
        for (int i = 0; i < (int)(seconds / dt + 0.5f); i++) {
            const float cur = truth.euler_aero().roll, want = roll_deg, rate = clampf((want - cur) * 10.0f, -60.0f, 60.0f) * kDeg2Rad;
            const Vec3 gyro{rate, 0, 0};
            // physics: thrust T along body -z such that the vertical part carries the weight; drag against the velocity
            const float T = 1.0f / fmaxf(truth.up_cos(), 0.3f);                    // in g
            float an, ae, up; truth.world_accel({0, 0, -T}, an, ae, up);           // kinematic acceleration from the thrust alone
            const float dn = -drag * vn, de = -drag * ve; an += dn; ae += de;
            // the accelerometer: thrust + drag in body axes (drag turned into the body frame through the truth)
            Vec3 acc{0, 0, -T}; { const float r = truth.euler_aero().roll * kDeg2Rad; acc.y += de * cosf(r) / kG0; acc.z += -de * sinf(r) / kG0; acc.x += dn / kG0; }
            vn += an * dt; ve += ae * dt; t += dt;
            truth.update(gyro, {0, 0, -1}, dt, nullptr, 0, 0.0f);                  // the truth is pure integration
            hist_n.push_back(vn); hist_e.push_back(ve);
            // estimator side
            float fn, fe, fu; est.world_accel(acc, fn, fe, fu);
            aid.push_force(fn / kG0, fe / kG0, -fu / kG0 - 1.0f, dt);
            if (t >= next_fix) { next_fix += 1.0 / rate_hz; const size_t back = (size_t)(delay / dt + 0.5f); fixes++;
                if (hist_n.size() > back) { const size_t k = hist_n.size() - 1 - back; if (aid.on_velocity(hist_n[k] + gauss(vel_noise), hist_e[k] + gauss(vel_noise), given_delay >= 0 ? given_delay : delay)) used++; } }
            const Vec3 e = aid.error_world(); est.set_aid(use_aid && aid.valid() ? &e : nullptr);
            est.update(gyro, acc, dt);
            max_err = fmaxf(max_err, fabsf(est.euler_aero().roll - truth.euler_aero().roll));
        }
    }
    float err() const { return est.euler_aero().roll - truth.euler_aero().roll; }
};

// the horizontal estimate with a velocity that arrives late (optical flow): it is moved to "now" with the acceleration
static void t_flow_fusion() {
    printf("[horizontal estimate] a velocity measured 0.1 s ago, while the craft accelerates\n");
    auto run = [](float delay, float told) { HorizontalEstimator h; h.reset(); const float dt = 0.002f, a = 2.0f; float v = 0, worst = 0; std::vector<float> past;
        for (int i = 0; i < 1500; i++) { const float t = i * dt; const float acc = t < 2.0f ? a : 0.0f; v += acc * dt; past.push_back(v); h.predict(acc, 0, dt);
            if (i % 10 == 9) { const size_t back = (size_t)(delay / dt + 0.5f); const float vm = past.size() > back ? past[past.size() - 1 - back] : 0.0f; h.fuse_velocity(vm, 0, 0.1f, told); }
            if (t > 0.5f && t < 2.0f) worst = fmaxf(worst, fabsf(h.vel_n() - v)); }
        return worst; };
    const float right = run(0.1f, 0.1f), ignored = run(0.1f, 0.0f);
    CHECK(right < 0.05f && ignored > 2.0f * right, "accelerating at 2 m/s^2 with a velocity that is 0.1 s old: %.3f m/s off at most when its age is known, %.3f when it is taken for fresh", right, ignored);
}

static void t_tilt_aid() {
    printf("[tilt aid] which way is down while the craft speeds up - from the measured change of velocity\n");
    // a 25 deg lean held for 4 s (the craft gathers 13 m/s), then level again
    { AidRig plain; plain.use_aid = false; plain.run(1.0f, 0); plain.run(4.0f, 25); const float p_end = plain.err(); plain.run(3.0f, 0);
      AidRig aided; aided.run(1.0f, 0); aided.run(4.0f, 25); const float a_end = aided.err(); aided.run(3.0f, 0);
      CHECK(fabsf(p_end) > 4.0f && plain.max_err > 4.0f, "accelerometer alone: the estimate gives way %.1f deg in a 25 deg lean of 4 s (the reading says \"level\")", fabsf(p_end));
      CHECK(aided.max_err < 0.5f && fabsf(a_end) < 0.3f && aided.used > 20, "with the GPS velocity (10 Hz, 150 ms late): %.2f deg at most, %.2f at the end of the lean (%d comparisons)", aided.max_err, fabsf(a_end), aided.used); }
    // an estimate that is wrong to begin with is pulled right - while accelerating, where the accelerometer alone would pull it wrong
    { AidRig r; r.est.reset_to_accel({0, sinf(8 * kDeg2Rad), -cosf(8 * kDeg2Rad)}); const float e0 = r.err(); r.run(1.0f, 20); const float e1 = r.err(); r.run(8.0f, 20);
      CHECK(fabsf(e0 + 8.0f) < 0.1f && fabsf(e1) < fabsf(e0) && fabsf(r.err()) < 0.5f, "an estimate 8 deg off: %.1f deg after 1 s, %.2f after 9 s of flying a 20 deg lean", e1, r.err());
      AidRig o; o.est.reset_to_accel({0, -sinf(8 * kDeg2Rad), -cosf(8 * kDeg2Rad)}); o.run(9.0f, -20);
      CHECK(fabsf(o.err()) < 0.5f, "and from the other side, leaning the other way: %.2f", o.err()); }
    // the lateness of the measurement has to be known
    { AidRig right; right.run(1.0f, 0); for (int k = 0; k < 6; k++) { right.run(1.0f, 25); right.run(1.0f, -25); }
      AidRig wrong; wrong.given_delay = 0.0f; wrong.delay = 0.4f; wrong.run(1.0f, 0); for (int k = 0; k < 6; k++) { wrong.run(1.0f, 25); wrong.run(1.0f, -25); }
      CHECK(right.max_err < 1.0f && wrong.max_err > 2.0f * right.max_err, "leaning to and fro every second: %.2f deg at most; with measurements 0.4 s late that are taken for fresh, %.1f", right.max_err, wrong.max_err); }
    // noise: 0.1 m/s per fix
    { AidRig r; r.vel_noise = 0.1f; r.run(1.0f, 0); r.run(4.0f, 25); r.run(15.0f, 0);
      CHECK(r.max_err < 2.0f, "a GPS velocity with 0.1 m/s of noise: the estimate stays within %.2f deg", r.max_err); }
    // flow-like: 50 measurements a second, 20 ms late
    { AidRig r; r.rate_hz = 50; r.delay = 0.02f; r.vel_noise = 0.05f; r.run(1.0f, 0); r.run(4.0f, 25); r.run(3.0f, 0);
      CHECK(r.max_err < 1.0f && r.used > 30 && r.used < 45, "50 measurements a second are gathered in blocks of 0.2 s (%d comparisons in 8 s): %.2f deg at most", r.used, r.max_err); }

    // the pieces
    { TiltAid a; a.reset(); CHECK(!a.valid() && a.error_world().x == 0, "nothing measured yet: not valid");
      // hovering, the estimate tilted so that it sees 0.1 g of the thrust pointing north: no velocity change -> it is the estimate that is wrong
      for (int i = 0; i < 400; i++) { a.push_force(0.1f, 0, -0.995f, 0.002f); if (i % 50 == 49) a.on_velocity(1.0f, -2.0f, 0.0f); }
      const Vec3 e = a.error_world();
      // m = (0.1, 0, -0.995), e = (0, 0, -1): m x e = (0, 0.1, 0) -> a rotation about +east (y) of asin(0.1)
      CHECK(a.valid() && fabsf(e.x) < 1e-4f && fabsf(e.y - 0.1f) < 2e-3f && e.z == 0.0f, "a constant velocity and a force that leans north: the error is a rotation about east (%.4f, %.4f, %.1f)", e.x, e.y, e.z);
      CHECK(fabsf(a.accel_n()) < 1e-4f && fabsf(a.accel_e()) < 1e-4f, "measured acceleration 0");
      // (the last comparison was made with the fix at 0.7 s; it is 0.8 s now)
      for (int i = 0; i < 240; i++) a.push_force(0.1f, 0, -0.995f, 0.002f);
      const bool still = a.valid(); for (int i = 0; i < 20; i++) a.push_force(0.1f, 0, -0.995f, 0.002f);
      CHECK(still && !a.valid(), "no new comparison for 0.6 s: not valid any more"); }
    { TiltAid a; a.reset();                                                     // the same force, but the velocity really changes by 0.981 m/s every second northwards: nothing is wrong
      float v = 0; bool any = false; for (int i = 0; i < 500; i++) { a.push_force(0.1f, 0, -1.0f, 0.002f); v += 0.1f * kG0 * 0.002f; if (i % 50 == 49) any |= a.on_velocity(v, 0, 0.0f); }
      CHECK(any && a.valid() && fabsf(a.error_world().y) < 2e-3f && fabsf(a.accel_n() - 0.981f) < 0.01f, "the craft really accelerates 0.98 m/s^2 north: no error (%.4f), measured %.3f m/s^2", a.error_world().y, a.accel_n()); }
    { TiltAid a; a.reset(); float v = 0; int done = 0;                         // force east, velocity north: a heading error - not this block's business
      for (int i = 0; i < 500; i++) { a.push_force(0, 0.2f, -1.0f, 0.002f); v += 0.2f * kG0 * 0.002f; if (i % 50 == 49 && a.on_velocity(v, 0, 0.0f)) done++; }
      const Vec3 e = a.error_world();
      CHECK(done >= 3 && e.z == 0.0f && fabsf(e.x) > 0.1f && fabsf(e.y) > 0.1f, "a force that points east while the velocity grows northwards: the down part of the error stays 0 (%.3f, %.3f, %.1f)", e.x, e.y, e.z); }
    { auto count = [](int miss_from, int miss_to) { TiltAid a; a.reset(); int done = 0;
          for (int i = 0; i < 1000; i++) { a.push_force(0, 0, -1.0f, 0.002f); const int fix = (i + 1) / 50; if (i % 50 == 49 && !(fix >= miss_from && fix <= miss_to) && a.on_velocity(0, 0, 0)) done++; }
          return done; };
      // fixes every 0.1 s for 2 s, two to a block: 9 blocks are closed, 8 comparisons. Fixes 10 .. 15 missing: the blocks on the two
      // sides of the gap (means at 0.9 s and 1.65 s) are 0.75 s apart - not compared; three more comparisons never happen for lack of fixes
      CHECK(count(99, 99) == 8 && count(10, 15) == 5, "fixes missing for 0.7 s: the blocks around the gap are further apart than 0.6 s and are not compared (%d comparisons instead of %d)", count(10, 15), count(99, 99));
      CHECK(count(10, 13) == 7, "(a gap of 0.5 s is bridged: the force of the whole stretch is on record)"); }
    { auto count = [](float jump, float& acc) { TiltAid a; a.reset(); int done = 0; float v = 0;
          for (int i = 0; i < 600; i++) { a.push_force(0, 0, -1.0f, 0.002f); if (i == 300) v = jump; if (i % 50 == 49 && a.on_velocity(v, 0, 0)) done++; }
          acc = a.accel_n(); return done; };
      float acc_ok = 0, acc_jump = 0; const int ok = count(2.0f, acc_ok), jumped = count(4.0f, acc_jump);
      CHECK(ok == 4 && jumped == 3 && fabsf(acc_jump) < 1e-3f, "a velocity that jumps 4 m/s between two blocks (20 m/s^2: a glitch, not a manoeuvre) is not believed (%d comparisons instead of %d; 2 m/s = 10 m/s^2 is)", jumped, ok); }
    { TiltAid a; a.reset(); int done = 0;
      for (int i = 0; i < 600; i++) { a.push_force(0, 0, -0.1f, 0.002f); if (i % 50 == 49 && a.on_velocity(0, 0, 0)) done++; }
      CHECK(done == 0 && !a.valid(), "free fall (0.1 g of force): no direction to compare");
      auto count = [](bool restart) { TiltAid b; b.reset(); int n = 0; for (int i = 0; i < 500; i++) { b.push_force(0, 0, -1.0f, 0.002f); if (i % 50 == 49) { if (restart && i == 199) b.restart(); if (b.on_velocity(0, 0, 0)) n++; } } return n; };
      CHECK(count(false) == 3 && count(true) == 2, "restart() (the measurements had stopped, the estimate was reset): the next block has nothing to be compared with (%d comparisons instead of %d in 1 s)", count(true), count(false)); }
    // the stretch between two blocks is cut out of the force record to the millisecond, also inside the 5 ms pieces it is kept in
    { TiltAid a; a.reset(); int done = 0;
      // fixes 3 ms past every 100 ms from 103 on: blocks {103, 203} (mean 153 ms) and {303, 403} (353 ms), compared at 503 ms.
      // A push of 1 g north from 345 to 360 ms, the velocity says nothing changed: 8 ms of the push lie in [153, 353] - all of the
      // piece 345 .. 350, three fifths of 350 .. 355 - and the piece still being filled at 503 ms (500 .. 503) shifts all others
      for (int ms = 1; ms <= 503; ms++) { a.push_force(ms > 345 && ms <= 360 ? 1.0f : 0.0f, 0, -1.0f, 0.001f); if (ms % 100 == 3 && ms > 100) done += a.on_velocity(0, 0, 0); }
      const float fn = 0.008f * kG0 / 0.2f, want = fn / sqrtf(fn * fn + kG0 * kG0);     // m x e with m = (fn, 0, -g) / |m|, e = (0, 0, -1)
      CHECK(done == 1 && fabsf(a.error_world().y - want) < 1e-3f, "a push of 15 ms of which 8 fall between the two blocks: the error is that of 8 ms (%.4f, want %.4f; 5 ms would be %.4f, 10 ms %.4f)",
            a.error_world().y, want, 0.005f * kG0 / 0.2f / kG0, 0.010f * kG0 / 0.2f / kG0); }
    { auto compared = [](const std::vector<int>& fixes, int until) { TiltAid a; a.reset(); int done = 0; size_t k = 0;
          for (int ms = 1; ms <= until; ms++) { a.push_force(0, 0, -1.0f, 0.001f); if (k < fixes.size() && fixes[k] == ms) { done += a.on_velocity(k + 2 >= fixes.size() ? 0.01f : 0.0f, 0, 0); k++; } }
          return done; };
      // fixes bunched at the end of a block: its mean (264 ms) lies only 36 ms before the next block's (300 ms) - a velocity
      // change of 0.01 m/s over 36 ms is noise, not an acceleration of 0.28 m/s^2
      const int normal = compared({100, 200, 300, 400, 500}, 500), bunched = compared({100, 295, 296, 297, 298, 299, 300, 500}, 500);
      CHECK(normal == 1 && bunched == 0, "two blocks whose means lie less than half a block apart are not compared (%d; evenly spaced fixes: %d)", bunched, normal); }
    { TiltAid a; a.reset(); bool at500 = true, at700 = false;
      // GPS fixes that are 180 ms old, from 100 ms after the start: the first block's mean (30 ms before the start) lies before
      // the force record begins - that comparison cannot be made; the next one can
      for (int ms = 1; ms <= 700; ms++) { a.push_force(0, 0, -1.0f, 0.001f); if (ms % 100 == 0) { const bool d = a.on_velocity(0, 0, 0.18f); if (ms == 500) at500 = d; if (ms == 700) at700 = d; } }
      CHECK(!at500 && at700, "a stretch that begins before the force record does is not compared (the force of its missing part is unknown)"); }
}

// ---------------------------------------------------------------------------------------------------------------------
// a point mass that leans
// ---------------------------------------------------------------------------------------------------------------------
struct Mass {
    float pn = 0, pe = 0, vn = 0, ve = 0, yaw = 0, roll = 0, pitch = 0, drag = 0.35f, wind_n = 0, wind_e = 0, lag = 0.12f;       // wind: m/s of air over the ground
    void step(float roll_cmd, float pitch_cmd, float dt) {
        roll += (roll_cmd - roll) * dt / lag; pitch += (pitch_cmd - pitch) * dt / lag;
        float af, ar; lean_to_accel(roll, pitch, af, ar); float an, ae; body_to_ne(af, ar, yaw, an, ae);
        an += -drag * (vn - wind_n); ae += -drag * (ve - wind_e);
        vn += an * dt; ve += ae * dt; pn += vn * dt; pe += ve * dt;
    }
    float speed() const { return hypotf(vn, ve); }
};

static void t_poscontrol() {
    printf("[position controller] position -> velocity -> acceleration -> lean (ArduPilot's cascade)\n");
    { CHECK(fabsf(sqrt_controller(0.5f, 1.0f, 2.5f) - 0.5f) < 1e-6f && fabsf(sqrt_controller(-2.0f, 1.0f, 2.5f) + 2.0f) < 1e-6f, "near the target: speed = error x P (0.5 m -> 0.5 m/s, -2 m -> -2 m/s)");
      // linear up to accel / p^2 = 2.5 m; beyond: sqrt(2 a (e - 1.25))
      CHECK(fabsf(sqrt_controller(2.5f, 1.0f, 2.5f) - 2.5f) < 1e-5f && fabsf(sqrt_controller(6.25f, 1.0f, 2.5f) - 5.0f) < 1e-5f && fabsf(sqrt_controller(-6.25f, 1.0f, 2.5f) + 5.0f) < 1e-5f,
            "further out a square-root curve: 6.25 m -> %.2f m/s (a speed from which 2.5 m/s^2 still stops in time), same at the joint (2.5 m -> %.2f)", sqrt_controller(6.25f, 1.0f, 2.5f), sqrt_controller(2.5f, 1.0f, 2.5f));
      CHECK(sqrt_controller(3.0f, 2.0f, 0.0f) == 6.0f, "no acceleration given: plain P"); }
    { float r, p; accel_to_lean(kG0, 0, r, p); const float p45 = p; accel_to_lean(0, kG0 * tanf(20 * kDeg2Rad), r, p);
      CHECK(fabsf(p45 - 45.0f) < 1e-3f && fabsf(r - 20.0f) < 1e-3f && fabsf(p) < 1e-5f, "1 g forwards = 45 deg nose down (pitch +); g tan 20 to the right = 20 deg right side down (roll +)");
      float af, ar; lean_to_accel(-12.0f, 25.0f, af, ar); accel_to_lean(af, ar, r, p);
      CHECK(fabsf(r + 12.0f) < 1e-3f && fabsf(p - 25.0f) < 1e-3f && af > 0 && ar < 0, "and back: a lean of -12 / 25 deg is %.2f m/s^2 forwards, %.2f to the right", af, ar);
      float f, rt; ne_to_body(1.0f, 0.0f, 90.0f, f, rt); float n, e; body_to_ne(f, rt, 90.0f, n, e);
      CHECK(fabsf(f) < 1e-6f && fabsf(rt + 1.0f) < 1e-6f && fabsf(n - 1.0f) < 1e-6f && fabsf(e) < 1e-6f, "nose pointing east: north is to the LEFT (forward %.1f, right %.1f)", f, rt); }
    const float dt = 0.002f;
    // 1.8 m to go
    { Mass m; PosControlNE c; c.init(0, 0, 0, 0, 0, 0, 0); c.set_target(1.8f, 0); float peak_v = 0, over = 0, t_arrive = -1, worst_east = 0;
      for (int i = 0; i < 6000; i++) { c.update(m.pn, m.pe, m.vn, m.ve, dt, true); float r, p; c.lean(m.yaw, r, p); m.step(r, p, dt);
          peak_v = fmaxf(peak_v, m.speed()); over = fmaxf(over, m.pn - 1.8f); worst_east = fmaxf(worst_east, fabsf(m.pe)); if (t_arrive < 0 && fabsf(m.pn - 1.8f) < 0.1f) t_arrive = i * dt; }
      CHECK(t_arrive > 1.0f && t_arrive < 5.0f && over < 0.2f && fabsf(m.pn - 1.8f) < 0.03f && m.speed() < 0.02f && worst_east < 0.01f, "a target 1.8 m north: there after %.1f s, %.2f m past it at most, then still (%.3f m off)", t_arrive, over, m.pn - 1.8f);
      CHECK(peak_v > 0.6f && peak_v < 1.9f, "on the way never faster than the position error allows (1.8 m x P 1.0; %.2f m/s)", peak_v); }
    // the same with the nose pointing south-west: the leans change, the path does not
    { Mass m; m.yaw = 225; PosControlNE c; c.init(0, 0, 0, 0, 0, 0, m.yaw); c.set_target(1.8f, 0); float first_r = 0, first_p = 0;
      for (int i = 0; i < 6000; i++) { c.update(m.pn, m.pe, m.vn, m.ve, dt, true); float r, p; c.lean(m.yaw, r, p); if (i == 100) { first_r = r; first_p = p; } m.step(r, p, dt); }
      CHECK(first_p < -1.0f && first_r > 1.0f && fabsf(m.pn - 1.8f) < 0.03f && fabsf(m.pe) < 0.03f, "nose south-west, target to the north: it leans back and to the right (pitch %.1f, roll %.1f) and arrives all the same", first_p, first_r); }
    // far away: the target is pulled along, 2 m ahead of the craft
    { Mass m; PosControlNE c; c.init(0, 0, 0, 0, 0, 0, 0); c.set_target(0, 50.0f); c.update(m.pn, m.pe, m.vn, m.ve, dt, true);
      CHECK(fabsf(c.target_e() - 2.0f) < 1e-4f && fabsf(c.pos_error() - 2.0f) < 1e-4f && fabsf(c.vel_target_e() - sqrt_controller(2.0f, 1.0f, 2.5f)) < 1e-4f, "a target 50 m away is pulled back to 2 m from the craft (%.2f): no race towards it", c.target_e());
      PosControlNE s; s.cfg.pos_p = 3.0f; s.init(0, 0, 0, 0, 0, 0, 0); s.set_target(0, 50.0f); s.update(0, 0, 0, 0, dt, true);
      CHECK(sqrt_controller(2.0f, 3.0f, 2.5f) > 2.5f && fabsf(s.vel_target_e() - 2.0f) < 1e-4f, "with a stiffer P (3.0) the speed it flies a position error out with is still at most 2 m/s (%.2f; the curve alone would give %.2f)", s.vel_target_e(), sqrt_controller(2.0f, 3.0f, 2.5f)); }
    // wind: the integrator learns the lean that stands against it
    { Mass m; m.wind_n = -6.0f; m.vn = 0; PosControlNE c; c.init(0, 0, 0, 0, 0, 0, 0); float worst = 0;
      for (int i = 0; i < 15000; i++) { c.update(m.pn, m.pe, m.vn, m.ve, dt, true); float r, p; c.lean(m.yaw, r, p); m.step(r, p, dt); worst = fmaxf(worst, fabsf(m.pn)); }
      // the air moves 6 m/s southwards: drag pushes with 0.35 x 6 = 2.1 m/s^2; the craft must accelerate 2.1 m/s^2 north to stand still
      CHECK(fabsf(m.pn) < 0.05f && fabsf(c.i_n() - 2.1f) < 0.1f && worst < 1.6f && fabsf(m.pitch - atanf(2.1f / kG0) * kRad2Deg) < 0.5f, "a head wind of 6 m/s from the north: blown back %.2f m at most, then back on the spot (%.3f m), leaning %.1f deg into it (integrator %.2f m/s^2)", worst, m.pn, m.pitch, c.i_n()); }
    // the limit: more than the lean limit gives is not asked for, and the integrator does not wind up meanwhile
    { PosControlNE c; c.cfg.angle_max_deg = 20; c.init(0, 0, 0, 0, 0, 0, 0); const float amax = kG0 * tanf(20 * kDeg2Rad);
      for (int i = 0; i < 3000; i++) c.update(0, 0, -8.0f, 0, dt, false, 4.0f, 0);           // wants 4 m/s north, is blown south at 8 m/s and stays so: error 12 m/s for 6 s
      const float len = hypotf(c.accel_n(), c.accel_e()); float r, p; c.lean(0, r, p);
      CHECK(c.limited() && fabsf(len - amax) < 1e-3f && fabsf(p - 20.0f) < 0.05f, "a demand beyond the lean limit: %.2f m/s^2 = 20 deg (limit %.2f)", len, amax);
      CHECK(fabsf(c.i_n()) < 0.05f, "and the integrator has not grown while the output was at the limit (%.3f m/s^2 after 6 s of 12 m/s error)", c.i_n());
      PosControlNE d; d.cfg.angle_max_deg = 20; d.init(0, 0, 0, 0, 0, 0, 0);
      for (int i = 0; i < 3000; i++) d.update(0, 0, 3.9f, 0, dt, false, 4.0f, 0);            // a small error, not limited: it does integrate
      CHECK(!d.limited() && fabsf(d.i_n() - 0.1f * 1.0f * 6.0f) < 0.02f, "(unlimited it integrates: 0.1 m/s of error x I 1.0 x 6 s = %.2f m/s^2)", d.i_n());
      PosControlNE w; w.cfg.angle_max_deg = 60; w.init(0, 0, 0, 0, 0, 0, 0); for (int i = 0; i < 20000; i++) w.update(0, 0, 0.4f, 0.3f, dt, false);
      CHECK(fabsf(hypotf(w.i_n(), w.i_e()) - 5.0f) < 1e-3f, "the integrator itself stops at 5 m/s^2 (%.2f)", hypotf(w.i_n(), w.i_e())); }
    // no jump when it takes over from a craft that already leans
    { PosControlNE c; c.init(5, -3, 0, 0, 10.0f, -6.0f, 30.0f); c.update(5, -3, 0, 0, dt, true); float r, p; c.lean(30.0f, r, p);
      CHECK(fabsf(r - 10.0f) < 0.3f && fabsf(p + 6.0f) < 0.3f && c.target_n() == 5 && c.target_e() == -3, "taking over a craft that leans 10 / -6 deg: the first output is that lean (%.2f / %.2f), the target is where it is", r, p);
      PosControlNE big; big.init(0, 0, 0, 0, 0, 60.0f, 0); CHECK(fabsf(hypotf(big.i_n(), big.i_e()) - 5.0f) < 1e-3f, "(a lean beyond what the integrator may hold is cut to its limit)"); }
    // velocity only (GPS Rescue): the target follows the craft
    { Mass m; PosControlNE c; c.init(0, 0, 0, 0, 0, 0, 0);
      for (int i = 0; i < 5000; i++) { c.update(m.pn, m.pe, m.vn, m.ve, dt, false, 3.0f, -4.0f); float r, p; c.lean(m.yaw, r, p); m.step(r, p, dt); }
      CHECK(fabsf(m.vn - 3.0f) < 0.05f && fabsf(m.ve + 4.0f) < 0.05f && c.pos_error() == 0.0f && c.target_n() == m.pn - m.vn * dt, "velocity wanted 3 / -4 m/s, no position target: it flies %.2f / %.2f m/s and the target stays with the craft", m.vn, m.ve); }
    // relax
    { PosControlNE c; c.init(0, 0, 0, 0, 20.0f, 0, 0); const float a0 = c.accel_e(); for (int i = 0; i < 80; i++) c.relax(dt);
      CHECK(a0 > 3.0f && fabsf(c.accel_e() / a0 - expf(-1.0f)) < 0.02f && fabsf(c.i_e() / a0 - expf(-1.0f)) < 0.02f, "relax(): what it holds fades with a time constant of 0.16 s (%.0f %% after 0.16 s)", c.accel_e() / a0 * 100); }
    // taking over with a velocity error already there (a gust, the rescue's first command): the filters start from it, no kick
    { PosControlNE c; c.cfg.vel_i = 0; c.init(0, 0, 0, 0, 0, 0, 0); c.update(0, 0, -1.0f, 0, dt, false); const float a1 = c.accel_n(); c.update(0, 0, -1.0f, 0, dt, false); const float a2 = c.accel_n();
      CHECK(fabsf(a1 - 2.0f) < 1e-3f && fabsf(a2 - 2.0f) < 1e-3f, "1 m/s of velocity error from the first step on: P x error at once and nothing from the D part (%.3f, %.3f m/s^2)", a1, a2); }
    // the filters: a step in the velocity error reaches the output through a 5 Hz low-pass; the D part is the low-passed slope
    { PosControlNE c; c.cfg.vel_i = 0; c.cfg.vel_d = 0; c.init(0, 0, 0, 0, 0, 0, 0); c.update(0, 0, 0, 0, dt, false);
      int n63 = -1; for (int i = 0; i < 200; i++) { c.update(0, 0, -1.0f, 0, dt, false); if (n63 < 0 && c.accel_n() >= 2.0f * (1.0f - expf(-1.0f))) n63 = i + 1; }
      CHECK(abs(n63 - 16) <= 1 && fabsf(c.accel_n() - 2.0f) < 0.01f, "velocity error 1 m/s: P 2.0 gives 2 m/s^2, through a 5 Hz low-pass (63 %% after %d ms; 1 / (2 pi 5) = 32 ms)", n63 * 2);
      PosControlNE d; d.cfg.vel_i = 0; d.cfg.vel_p = 0; d.init(0, 0, 0, 0, 0, 0, 0); float v = 0; d.update(0, 0, 0, 0, dt, false);
      for (int i = 0; i < 1000; i++) { v -= 2.0f * dt; d.update(0, 0, v, 0, dt, false); }
      CHECK(fabsf(d.accel_n() - 0.25f * 2.0f) < 0.01f, "a velocity error that grows 2 m/s per second: D 0.25 gives %.2f m/s^2", d.accel_n()); }
}

// ---------------------------------------------------------------------------------------------------------------------
// Position Hold
// ---------------------------------------------------------------------------------------------------------------------
struct PhRig {
    Mass m; PosHold ph; PosHoldInput in; float roll = 0, pitch = 0, t = 0, max_step = 0; const float dt = 0.0025f;
    PhRig() { in.airborne = true; ph.init(in); }
    // sticks: lean asked for (deg); 0 = centred
    void run(float seconds, float stick_roll, float stick_pitch) {
        for (int i = 0; i < (int)(seconds / dt + 0.5f); i++) {
            in.pilot_roll_deg = stick_roll; in.pilot_pitch_deg = stick_pitch; in.roll_centred = stick_roll == 0; in.pitch_centred = stick_pitch == 0;
            in.yaw_deg = m.yaw; in.pos_n = m.pn; in.pos_e = m.pe; in.vel_n = m.vn; in.vel_e = m.ve;
            float r, p; ph.update(in, dt, r, p);
            max_step = fmaxf(max_step, fmaxf(fabsf(r - roll), fabsf(p - pitch)));
            roll = r; pitch = p; m.step(r, p, dt); t += dt;
        }
    }
    bool run_until(float timeout, PhAxis s0, PhAxis s1) { for (float e = 0; e < timeout; e += 0.02f) { if (ph.state(0) == s0 && ph.state(1) == s1) return true; run(0.02f, 0, 0); } return ph.state(0) == s0 && ph.state(1) == s1; }
};

static void t_poshold() {
    printf("[position hold] the pilot flies, lets go, it brakes and stays (ArduCopter PosHold)\n");
    // flying: the stick is the lean
    { PhRig r; r.run(0.5f, 0, 12.0f);
      CHECK(r.ph.state(1) == PhAxis::PILOT && fabsf(r.pitch - 12.0f) < 1e-4f && fabsf(r.roll) < 0.5f && !r.ph.holding(), "stick forward: 12 deg nose down, as in Angle (pitch axis PILOT)");
      r.run(3.0f, 0, 12.0f); const float v0 = r.m.speed(); r.max_step = 0;
      // let go at speed
      r.run(0.02f, 0, 0); const float just = r.pitch; r.run(0.2f, 0, 0); const PhAxis s_02 = r.ph.state(1); float most = 0, t_most = 0; const float t0 = r.t, p0 = r.m.pn;
      while (r.t - t0 < 12.0f && !r.ph.holding()) { r.run(0.02f, 0, 0); if (-r.pitch > most) { most = -r.pitch; t_most = r.t - t0; } }
      const float t_hold = r.t - t0, dist = r.m.pn - p0;
      CHECK(v0 > 3.5f && just > 9.0f && just < 11.5f && s_02 == PhAxis::PILOT, "let go at %.1f m/s: the lean is not dropped at once (%.1f deg after 20 ms), it is taken back", v0, just);
      CHECK(most > 8.0f && most <= 30.0f && t_most > 1.0f, "then it leans back against the motion: up to %.1f deg nose up, built up at 8 deg/s (the peak %.1f s after the stick was let go)", most, t_most);
      CHECK(r.ph.holding() && t_hold < 11.0f && dist > 3.0f && dist < 25.0f, "holding %.1f s after the stick was let go, %.1f m further on", t_hold, dist);
      const float pn = r.m.pn; r.run(5.0f, 0, 0);
      CHECK(r.m.speed() < 0.03f && fabsf(r.m.pn - pn) < 0.6f && fabsf(r.m.pe) < 0.1f && fabsf(r.pitch) < 0.3f, "and it stays: %.2f m from where the hold began, %.3f m/s", r.m.pn - pn, r.m.speed());
      CHECK(r.max_step < 0.3f, "no jump of the lean anywhere on the way, apart from the stick's own (largest step %.2f deg in 2.5 ms)", r.max_step); }
    // the brake is the same shape every time: lean = -gain x v x (1 + 5 / (|v| + 0.6)), gain = (15 x 8 + 95) / 100 = 2.15 deg per m/s
    { PhRig r; r.m.vn = 1.0f; r.run(0.02f, 0, 0); r.m.vn = 1.0f; r.m.drag = 0; r.m.lag = 1e9f;                    // a craft that keeps its 1 m/s whatever the lean
      const float b0 = r.ph.brake_deg(1);
      for (int i = 0; i < 400; i++) { r.m.vn = 1.0f; r.run(0.0025f, 0, 0); if (i == 199) { CHECK(fabsf(r.ph.brake_deg(1) - b0 + 8.0f * 0.5f) < 0.02f, "braking 1 m/s: the lean grows 8 deg/s (%.2f deg in 0.5 s)", b0 - r.ph.brake_deg(1)); } }
      const float want = 2.15f * 1.0f * (1.0f + 5.0f / 1.6f);
      r.run(1.0f, 0, 0);
      CHECK(fabsf(-r.ph.brake_deg(1) - want) < 0.05f, "and ends at %.2f deg nose up for 1 m/s (2.15 x 1 x (1 + 5 / 1.6) = %.2f)", -r.ph.brake_deg(1), want);
      PhRig f; f.m.drag = 0; f.m.lag = 1e9f; for (int i = 0; i < 4000; i++) { f.m.ve = 20.0f; f.run(0.0025f, 0, 0); }
      CHECK(fabsf(f.ph.brake_deg(0) + 30.0f) < 1e-3f, "never more than the brake limit of 30 deg (20 m/s to the right: %.1f deg, left side down)", f.ph.brake_deg(0)); }
    // let go while standing still: half a second of brake phase, 1.5 s of hand-over
    { PhRig r; r.run(0.1f, 0, 0); const PhAxis s = r.ph.state(0); const bool ready = r.run_until(1.0f, PhAxis::TO_LOITER, PhAxis::TO_LOITER); const float t_ready = r.t; const bool hold = r.run_until(3.0f, PhAxis::LOITER, PhAxis::LOITER);
      CHECK(s == PhAxis::BRAKE && ready && fabsf(t_ready - 0.5f) < 0.06f && hold && fabsf(r.t - t_ready - 1.5f) < 0.05f, "mode switched on in a hover: the brake phase (nothing to brake) ends after %.2f s, hand-over %.2f s, holding", t_ready, r.t - t_ready); }
    // wind: held against it, and the lean it takes is remembered for the pilot
    { PhRig r; r.m.wind_e = 5.0f; r.run_until(8.0f, PhAxis::LOITER, PhAxis::LOITER); r.run(12.0f, 0, 0);
      const float need = 0.35f * 5.0f;                                          // the wind pushes east with 1.75 m/s^2: the craft must accelerate 1.75 m/s^2 west
      CHECK(r.ph.holding() && r.m.speed() < 0.05f && fabsf(r.ph.wind_e() + need) < 0.15f && fabsf(r.ph.wind_n()) < 0.1f, "a wind of 5 m/s from the west: it stands (%.3f m/s) and has learned %.2f m/s^2 against it (%.2f needed)", r.m.speed(), -r.ph.wind_e(), need);
      { PhRig g = r; g.in.airborne = false; g.run(0.05f, 0, 0);
        CHECK(g.ph.wind_n() == 0 && g.ph.wind_e() == 0, "landed (or taking off again): what was learned about the wind is forgotten"); }
      const float lean_w = atanf(need / kG0) * kRad2Deg;
      r.run(1.0f, 0, 8.0f);                                                     // the pilot flies forwards: the roll axis keeps holding against the wind by itself
      CHECK(r.ph.state(1) == PhAxis::PILOT && fabsf(r.pitch - 8.0f) < 0.3f && fabsf(r.roll + lean_w) < 2.5f, "the pilot pushes forward: pitch is his (%.1f deg), roll still leans %.1f deg into the wind (%.1f needed)", r.pitch, -r.roll, lean_w);
      r.run(0.6f, 6.0f, 8.0f);
      CHECK(r.ph.state(0) == PhAxis::PILOT && fabsf(r.roll - (6.0f - lean_w)) < 0.6f, "both sticks: the roll is the stick's 6 deg plus the lean against the wind (%.1f = 6 - %.1f)", r.roll, lean_w);
      { PhRig h = r; for (int i = 0; i < 6000 && h.ph.state(0) != PhAxis::TO_LOITER; i++) h.run(0.0025f, 0, 0);
        float lr = 0, lp = 0; h.ph.ctl.lean(h.m.yaw, lr, lp);
        CHECK(h.ph.state(0) == PhAxis::TO_LOITER && fabsf(lr - h.roll) < 0.05f && fabsf(lp - h.pitch) < 0.05f && h.roll < -2.0f, "sticks let go, braked: the controller takes over from the lean the craft has (%.1f / %.1f deg; it gives %.1f / %.1f) - the wind's share included", h.roll, h.pitch, lr, lp);
        h.run_until(3.0f, PhAxis::LOITER, PhAxis::LOITER); const float wn = h.ph.wind_n(), we = h.ph.wind_e(); float slowest = 9; for (int i = 0; i < 560; i++) { h.run(0.0025f, 0, 0); slowest = fminf(slowest, h.m.speed()); }
        CHECK(h.ph.holding() && h.ph.wind_n() == wn && h.ph.wind_e() == we && slowest < 0.1f, "holding again: for the first 1.4 s what was learned is left as it is (the controller is still settling), although the craft already stands (%.3f m/s)", slowest); }
      PhRig big; big.m.wind_n = -12.0f; big.run_until(10.0f, PhAxis::LOITER, PhAxis::LOITER); big.run(20.0f, 0, 0);      // 12 m/s: 4.2 m/s^2 = 23 deg of lean
      const float cap = kG0 * tanf(0.6666f * 30.0f * kDeg2Rad);
      CHECK(big.ph.holding() && big.m.speed() < 0.05f && fabsf(hypotf(big.ph.wind_n(), big.ph.wind_e()) - cap) < 1e-3f, "what is remembered never takes more than two thirds of the lean limit (%.2f of %.2f m/s^2): the pilot can always fly against it", hypotf(big.ph.wind_n(), big.ph.wind_e()), cap); }
    // a craft the wind keeps moving while it brakes: the brake ends by its own timing (1.5 x the time to take the lean back), not after the 6 s limit
    { PhRig r; r.m.wind_e = 5.0f; r.m.ve = 4.0f; float t_ready = -1, v_ready = 0; for (int i = 0; i < 4000 && t_ready < 0; i++) { r.run(0.0025f, 0, 0); if (r.ph.state(0) != PhAxis::BRAKE && r.ph.state(0) != PhAxis::PILOT) { t_ready = r.t; v_ready = r.m.ve; } }
      CHECK(t_ready > 3.0f && t_ready < 5.2f && v_ready > 0.5f && r.run_until(8.0f, PhAxis::LOITER, PhAxis::LOITER), "moving downwind at 4 m/s in a 5 m/s wind: the roll brake ends after %.1f s although the wind still carries the craft (%.2f m/s); the controller takes it from there", t_ready, v_ready); }
    // the wind is learned only while the craft stands still, and not in the first 1.5 s of holding
    { PhRig w; w.m.wind_e = 3.0f; w.run_until(8.0f, PhAxis::LOITER, PhAxis::LOITER); w.run(1.4f, 0, 0);
      CHECK(w.ph.holding() && w.ph.wind_n() == 0 && w.ph.wind_e() == 0 && fabsf(w.ph.ctl.accel_e()) > 0.5f, "the first 1.4 s of holding in a wind: nothing learned yet (the controller is still settling; it asks for %.2f m/s^2)", w.ph.ctl.accel_e());
      PhRig r; r.run_until(8.0f, PhAxis::LOITER, PhAxis::LOITER); r.run(1.4f, 0, 0);
      r.run(3.0f, 0, 0); const float w0 = hypotf(r.ph.wind_n(), r.ph.wind_e()); r.m.vn = 2.5f; r.run(0.6f, 0, 0); const float w1 = hypotf(r.ph.wind_n(), r.ph.wind_e());
      CHECK(w0 < 0.02f && w1 < 0.02f && r.ph.holding(), "a push of 2.5 m/s while holding: the controller brakes hard, but that is not wind (learned %.3f m/s^2, before %.3f)", w1, w0); }
    // a stick takes ITS axis back, half a second from the controller's lean to the pilot's; the other axis waits
    { PhRig r; r.m.wind_n = 4.0f; r.run_until(8.0f, PhAxis::LOITER, PhAxis::LOITER); r.run(6.0f, 0, 0); const float held_pitch = r.pitch, held_roll = r.roll;
      r.run(0.0025f, 10.0f, 0); const PhAxis a0 = r.ph.state(0), a1 = r.ph.state(1); const float first = r.roll;
      r.run(0.25f, 10.0f, 0); const float mid = r.roll; r.run(0.3f, 10.0f, 0);
      CHECK(a0 == PhAxis::TO_PILOT && a1 == PhAxis::BRAKE_READY && fabsf(first - held_roll) < 0.3f && mid > 2.0f && mid < 8.0f && r.ph.state(0) == PhAxis::PILOT && fabsf(r.roll - 10.0f) < 0.6f,
            "roll stick moved while holding: roll goes to the pilot over 0.5 s (%.1f -> %.1f -> %.1f deg), pitch waits ready", first, mid, r.roll);
      CHECK(fabsf(r.pitch - held_pitch) < 1.5f, "the pitch axis keeps its lean against the wind meanwhile (%.1f, was %.1f)", r.pitch, held_pitch);
      r.run(1.0f, 10.0f, 0); r.run(0.5f, 0, 0); const bool again = r.run_until(12.0f, PhAxis::LOITER, PhAxis::LOITER);
      CHECK(again && r.m.speed() < 0.3f, "stick let go again: it brakes the roll axis and holds at the new place"); }
    // the pilot's lean is taken back at brake speed unless it was large
    { PhRig r; r.run(1.0f, 10.0f, 0); r.run(0.1f, 0, 0); const float after = r.roll; r.run(0.4f, 0, 0); const bool still_his = r.ph.state(0) == PhAxis::PILOT; r.run(0.1f, 0, 0);
      // 10 deg: taken back by 5 /s of what is left (10 x e^-0.5 = 6.1 after 0.1 s) down to 1.6 deg (0.37 s), from there at 8 deg/s: gone after 0.57 s
      CHECK(fabsf(after - 10.0f * expf(-0.5f)) < 0.3f && still_his && r.ph.state(0) == PhAxis::BRAKE, "stick let go from 10 deg: %.1f deg are still held 0.1 s later; after 0.57 s the lean is gone and the brake has the axis", after);
      PhRig lin; lin.ph.cfg.smooth_rate = 0; lin.run(1.0f, 10.0f, 0); lin.run(0.5f, 0, 0);
      CHECK(fabsf(lin.roll - 6.0f) < 0.1f && lin.ph.state(0) == PhAxis::PILOT, "(at least at brake speed: with the 5 /s off, 8 deg/s x 0.5 s leave %.1f of 10 deg)", lin.roll);
      PhRig rev; rev.run(1.0f, 10.0f, 0); rev.run(0.0025f, -3.0f, 0);
      CHECK(fabsf(rev.roll + 3.0f) < 1e-3f, "stick to the other side: followed at once (%.1f)", rev.roll);
      PhRig up; up.run(1.0f, 4.0f, 0); up.run(0.0025f, 12.0f, 0); const float more = up.roll; up.run(0.0025f, 25.0f, 0);
      CHECK(more == 12.0f && up.roll == 25.0f, "more stick: followed at once as well (%.0f, %.0f)", more, up.roll);
      PhRig big; big.run(1.0f, 28.0f, 0); big.run(0.0025f, 20.0f, 0);
      CHECK(big.roll == 20.0f, "less stick, but still more than 18 deg: followed at once (%.1f; only a stick near the middle is smoothed)", big.roll); }
    // on the ground: the sticks alone, nothing held
    { PhRig r; r.in.airborne = false; r.ph.init(r.in); r.m.vn = 0; r.run(1.0f, 0, 0); const bool none = r.ph.state(0) == PhAxis::PILOT && r.ph.state(1) == PhAxis::PILOT && r.roll == 0 && r.pitch == 0;
      r.run(0.5f, 5.0f, -4.0f);
      CHECK(none && r.roll == 5.0f && r.pitch == -4.0f && r.ph.wind_n() == 0, "on the ground / taking off: the sticks pass through, no brake, no hold");
      r.in.airborne = true; r.run(0.1f, 0, 0); CHECK(r.ph.state(0) == PhAxis::PILOT || r.ph.state(0) == PhAxis::BRAKE, "airborne: it starts from the pilot's side"); }
    // the limit of the mode
    { PhRig r; r.ph.cfg.angle_max_deg = 20; r.run(0.5f, 35.0f, -35.0f); CHECK(fabsf(r.roll - 20.0f) < 1e-4f && fabsf(r.pitch + 20.0f) < 1e-4f, "the lean limit of the mode holds for the stick too (35 asked, %.0f given)", r.roll); }
}

// ---------------------------------------------------------------------------------------------------------------------
// GPS Rescue
// ---------------------------------------------------------------------------------------------------------------------
// A craft that does what it is told, with a little lag: the velocity over the ground, the climb rate, the yaw rate.
struct RescSim {
    Rescue r; RescueInput in; RescueOutput out; float t = 0, wind_n = 0, wind_e = 0, climb_factor = 1.0f, vz = 0; const float dt = 0.01f;
    std::vector<RescuePhase> seen; float max_speed = 0, max_yaw_rate = 0, max_z = 0, turned = 0;
    RescSim(float pn, float pe, float z, float yaw) { in.armed = in.airborne = true; in.pos_valid = in.alt_valid = in.heading_valid = true; in.sats = 12; in.pos_n = pn; in.pos_e = pe; in.z = z; in.yaw_deg = yaw; }
    void step() {
        out = r.update(in, dt); t += dt;
        if (seen.empty() || seen.back() != r.phase()) seen.push_back(r.phase());
        if (!out.active) return;
        in.vel_n += (out.vel_n + wind_n - in.vel_n) * dt / 0.8f; in.vel_e += (out.vel_e + wind_e - in.vel_e) * dt / 0.8f;
        in.pos_n += in.vel_n * dt; in.pos_e += in.vel_e * dt;
        vz += (out.climb_mps * climb_factor - vz) * dt / 0.4f; in.z = fmaxf(in.z + vz * dt, 0.0f);
        if (out.yaw_control) { in.yaw_deg = Rescue::wrap180(in.yaw_deg + out.yaw_rate_dps * dt); turned += out.yaw_rate_dps * dt; }
        max_speed = fmaxf(max_speed, hypotf(in.vel_n, in.vel_e)); max_yaw_rate = fmaxf(max_yaw_rate, fabsf(out.yaw_rate_dps)); max_z = fmaxf(max_z, in.z);
    }
    void run(float seconds) { for (int i = 0; i < (int)(seconds / dt + 0.5f); i++) step(); }
    bool until(RescuePhase p, float timeout) { for (float e = 0; e < timeout && r.phase() != p; e += dt) step(); return r.phase() == p; }
    float dist() const { return hypotf(in.pos_n, in.pos_e); }
};
static const char* phase_name(RescuePhase p) { static const char* n[] = {"IDLE", "CLIMB", "ROTATE", "FLY_HOME", "DESCENT", "LANDING", "FAILED"}; return n[(int)p]; }

static void t_rescue() {
    printf("[gps rescue] climb, turn towards home, fly there, come down (Betaflight's sequence)\n");
    using P = RescuePhase;
    // 150 m out to the north-east, 20 m up, nose pointing away; the flight had been up to 35 m
    { RescSim s(106, 106, 35, 45); s.run(1.0f); s.in.z = 20; s.run(1.0f);
      CHECK(s.r.phase() == P::IDLE && !s.out.active && fabsf(s.r.max_height() - 35.0f) < 1e-3f, "not asked for: idle, it only remembers how high the flight went (%.0f m)", s.r.max_height());
      s.in.request = true; s.step();
      CHECK(s.r.phase() == P::CLIMB && s.out.active && fabsf(s.r.return_height() - 45.0f) < 1e-3f && s.out.climb_mps == 5.0f && s.out.vel_n == 0 && !s.out.yaw_control, "asked for: climb at 5 m/s to the highest point of the flight + 10 m (%.0f m), standing still", s.r.return_height());
      const bool rot = s.until(P::ROTATE, 20); const float t_climb = s.t - 2.0f, z_rot = s.in.z, d_rot = s.dist();
      // 20 m at 5 m/s take 4 s; the last 5 m are flown at "1 m/s per metre left" (5 .. 0.5 m/s): about 2.3 s more, and it is
      // within half a metre of the return height when it gets there - not past it
      CHECK(rot && t_climb > 5.2f && t_climb < 7.5f && z_rot >= 44.4f && z_rot <= 45.2f && fabsf(d_rot - 150.0f) < 0.5f, "after %.1f s at %.1f m: turning", t_climb, z_rot);
      const bool fly = s.until(P::FLY_HOME, 20); const float yaw_fly = s.in.yaw_deg, t_rot = s.t;
      // home lies to the south-west: heading -135. From 45 that is half a turn; the nose must be within 30 deg before it flies
      CHECK(fly && fabsf(Rescue::wrap180(yaw_fly + 135.0f)) < 30.0f && s.max_yaw_rate <= 90.0f + 1e-3f && s.max_yaw_rate > 80.0f && s.dist() > 149.0f, "turned to within 30 deg of the way home (heading %.0f, home at -135) at 90 deg/s at most, without leaving the spot", yaw_fly);
      s.run(0.25f); const float v_q = hypotf(s.out.vel_n, s.out.vel_e); s.run(0.75f); const float v_1s = hypotf(s.out.vel_n, s.out.vel_e);
      CHECK(fabsf(v_q - 7.5f * 0.25f) < 0.15f, "the speed is eased in over the first second of flying home (%.2f m/s after 0.25 s)", v_q);
      s.run(5.0f); const float v_cmd = hypotf(s.out.vel_n, s.out.vel_e), dir = atan2f(s.out.vel_e, s.out.vel_n) * kRad2Deg;
      CHECK(fabsf(v_1s - 7.5f) < 0.05f && fabsf(v_cmd - 7.5f) < 1e-3f && fabsf(Rescue::wrap180(dir + 135.0f)) < 0.5f && fabsf(Rescue::wrap180(s.in.yaw_deg + 135.0f)) < 2.0f && s.out.climb_mps == 0,
            "flying home: 7.5 m/s towards -135 deg (reached within a second: %.1f), the nose on the way (%.0f), the height held", v_1s, s.in.yaw_deg);
      (void)t_rot;
      // slowing down over the last 40 m
      while (s.dist() > 30.0f && s.r.phase() == P::FLY_HOME) s.step();
      CHECK(fabsf(hypotf(s.out.vel_n, s.out.vel_e) - 7.5f * 30.0f / 40.0f) < 0.1f, "30 m from home: %.2f m/s (7.5 x 30 / 40)", hypotf(s.out.vel_n, s.out.vel_e));
      const bool desc = s.until(P::DESCENT, 20); const float d_desc = s.dist();
      CHECK(desc && fabsf(d_desc - 20.0f) < 0.3f && s.max_z < 45.5f, "20 m from home the descent begins (%.1f m out, %.1f m up; never higher than %.1f m: the climb did not shoot past the return height)", d_desc, s.in.z, s.max_z);
      s.step(); const float sink_45 = -s.out.climb_mps;
      CHECK(fabsf(sink_45 - 1.5f * (0.6f + 2.4f * (s.in.z - 10.0f) / 40.0f)) < 0.01f && sink_45 > 3.5f, "sinking %.2f m/s at %.0f m: the higher the faster (1.5 x 0.6 below 10 m ... x 3 from 50 m on)", sink_45, s.in.z);
      const bool land = s.until(P::LANDING, 60); const float z_land = s.in.z, d_land = s.dist(); s.step();
      CHECK(land && fabsf(z_land - 4.0f) < 0.1f && fabsf(-s.out.climb_mps - 0.9f) < 1e-3f && !s.out.yaw_control, "below 4 m: landing at 0.9 m/s, %.1f m from home, no more turning", d_land);
      s.run(8.0f);
      CHECK(s.in.z == 0.0f && s.dist() < 2.5f && s.r.phase() == P::LANDING && s.out.active, "down %.1f m from home after %.0f s (it keeps asking for the descent: the landing detection of Altitude Hold ends the flight)", s.dist(), s.t);
      std::string order; for (P p : s.seen) { order += phase_name(p); order += ' '; }
      CHECK(order == "IDLE CLIMB ROTATE FLY_HOME DESCENT LANDING ", "the phases in order: %s", order.c_str()); }
    // the other ways to choose the height; coming from above
    { RescSim s(0, -200, 50, 90); s.r.cfg.alt_mode = RESCUE_ALT_FIXED; s.in.request = true; s.step();
      CHECK(s.r.return_height() == 30.0f && fabsf(s.out.climb_mps + 1.5f) < 1e-6f, "fixed return height 30 m, the craft is at 50: it comes DOWN to it at 1.5 m/s first");
      const bool rot = s.until(P::ROTATE, 30); CHECK(rot && s.in.z <= 30.6f && s.in.z > 29.4f, "and turns when it is there (%.1f m; slower over the last metre)", s.in.z);
      const bool fly = s.until(P::FLY_HOME, 5); CHECK(fly && s.t < 16.5f, "the nose already points home (east): no time lost turning");
      RescSim c(300, 0, 12, 0); c.run(0.5f); c.in.z = 25; c.run(0.5f); c.in.z = 12; c.r.cfg.alt_mode = RESCUE_ALT_CURRENT; c.in.request = true; c.step();
      CHECK(c.r.return_height() == 22.0f, "\"present height + margin\": 12 + 10 = %.0f m (it had been at 25)", c.r.return_height()); }
    // started close to home
    { RescSim s(6, 5, 2, 10); s.in.request = true; s.step(); const float rz = s.r.return_height(); s.until(P::ROTATE, 10); s.step();
      CHECK(rz == 12.0f && s.r.phase() == P::DESCENT, "8 m from home, 2 m up: up by the margin (to %.0f m), then straight into the descent - no turning, no flying", rz);
      RescSim low(3, 0, -4.5f, 0); low.in.z = -4.5f; low.in.request = true; low.step(); CHECK(low.r.return_height() == 7.5f, "(never lower than 7.5 m)");
      s.until(P::LANDING, 30); s.run(10.0f); CHECK(s.in.z == 0.0f && s.dist() < 2.5f, "and down, %.1f m from home", s.dist()); }
    // the turn: the short way round, eased in over a second
    { RescSim s(-100, 0, 40, -100); s.r.cfg.alt_mode = RESCUE_ALT_FIXED; s.r.cfg.return_alt_m = 40; s.in.z = 40.5f; s.in.request = true; s.until(P::ROTATE, 5);
      s.run(0.25f); const float early = s.out.yaw_rate_dps, off = -s.in.yaw_deg; s.until(P::FLY_HOME, 10);
      // home is to the north (heading 0); the nose points to -100: turn right (+). Rate = 1.8 deg/s per degree, eased in over the first second
      CHECK(fabsf(early - off * 1.8f * 0.25f) < 2.0f && early < 50.0f && s.turned > 60.0f && s.turned < 100.0f, "nose at -100, home to the north: it turns right, the short way (%.0f deg turned); a quarter second in, a quarter of the rate (%.0f deg/s for %.0f deg off)", s.turned, early, off);
      RescSim o(-100, 0, 40.5f, 100); o.r.cfg.alt_mode = RESCUE_ALT_FIXED; o.r.cfg.return_alt_m = 40; o.in.request = true; o.until(P::FLY_HOME, 15);
      CHECK(o.turned < -60.0f, "nose at +100: left (%.0f deg)", o.turned);
      RescSim stuck(-100, 0, 40.5f, 170); stuck.r.cfg.alt_mode = RESCUE_ALT_FIXED; stuck.r.cfg.return_alt_m = 40; stuck.r.cfg.max_yaw_rate_dps = 0.001f; stuck.in.request = true; stuck.until(P::ROTATE, 5); const float t0 = stuck.t; const bool fly = stuck.until(P::FLY_HOME, 15);
      CHECK(fly && fabsf(stuck.t - t0 - 10.0f) < 0.1f, "a craft that does not turn: after 10 s it flies home anyway (the velocity is steered over the ground, the nose is not needed for it)"); }
    // the nose and home on the two sides of south: the error is taken the short way across +-180
    { RescSim s(100, -1, 40.5f, -170); s.r.cfg.alt_mode = RESCUE_ALT_FIXED; s.r.cfg.return_alt_m = 40; s.in.request = true; s.until(P::FLY_HOME, 10); s.run(3.0f);
      // home lies at a heading of 179.4 deg (south, a little east); the nose points to -170 (190): 10.6 deg to the left
      CHECK(s.turned < -5.0f && s.turned > -15.0f, "nose at -170, home at 179: it turns %.1f deg to the left, not 349 to the right", s.turned); }
    // it does not start without what it needs
    { auto start = [](int what) { RescSim s(100, 0, 20, 0); if (what == 0) s.in.pos_valid = false; if (what == 1) s.in.heading_valid = false; if (what == 2) s.in.alt_valid = false; s.in.request = true; s.step(); return s; };
      RescSim a = start(0), b = start(1), c = start(2);
      CHECK(a.r.phase() == P::FAILED && a.r.failure() == RescueFail::NO_POSITION && !a.out.active && b.r.failure() == RescueFail::NO_HEADING && c.r.failure() == RescueFail::NO_ALTITUDE,
            "no position / no compass heading / no height: it fails at once and says which (the caller lands the plain way)");
      CHECK(!Rescue::available(a.in) && !Rescue::available(b.in) && !Rescue::available(c.in) && Rescue::available(RescSim(1, 1, 1, 0).in), "available() tells beforehand");
      a.in.pos_valid = true; a.run(1.0f); const bool still = a.r.phase() == P::FAILED, still_active = a.out.active; a.in.request = false; a.step(); const bool idle = a.r.phase() == P::IDLE && a.r.failure() == RescueFail::NONE; a.in.request = true; a.step();
      CHECK(still && !still_active && idle && a.r.phase() == P::CLIMB, "a failure stays until the request is taken back (and nothing is flown meanwhile); asked again with a position, it starts"); }
    // lost on the way
    { RescSim s(200, 0, 30, 180); s.r.cfg.alt_mode = RESCUE_ALT_FIXED; s.in.request = true; s.until(P::FLY_HOME, 10); s.run(3.0f); s.in.pos_valid = false; s.step();
      CHECK(s.r.phase() == P::FAILED && s.r.failure() == RescueFail::GPS_LOST && !s.out.active, "the position is lost while flying home: failed (GPS_LOST)");
      RescSim h(200, 0, 30, 180); h.r.cfg.alt_mode = RESCUE_ALT_FIXED; h.in.request = true; h.until(P::FLY_HOME, 10); h.in.heading_valid = false; h.step(); CHECK(h.r.failure() == RescueFail::NO_HEADING, "the compass is lost: failed");
      RescSim z(200, 0, 30, 180); z.r.cfg.alt_mode = RESCUE_ALT_FIXED; z.in.request = true; z.until(P::FLY_HOME, 10); z.in.alt_valid = false; z.step(); CHECK(z.r.failure() == RescueFail::NO_ALTITUDE, "the height is lost: failed"); }
    { RescSim s(2000, 0, 30, 180); s.r.cfg.alt_mode = RESCUE_ALT_FIXED; s.in.request = true; s.until(P::FLY_HOME, 10); s.in.sats = 4; const float t0 = s.t; s.run(14.0f); const bool yet = s.r.phase() == P::FLY_HOME; s.until(P::FAILED, 3);
      CHECK(yet && s.r.failure() == RescueFail::LOW_SATS && fabsf(s.t - t0 - 15.0f) < 1.1f, "fewer than 5 satellites for 15 s: failed (after %.0f s)", s.t - t0);
      RescSim b(2000, 0, 30, 180); b.r.cfg.alt_mode = RESCUE_ALT_FIXED; b.in.request = true; b.until(P::FLY_HOME, 10);
      for (int k = 0; k < 10; k++) { b.in.sats = 4; b.run(10.0f); b.in.sats = 9; b.run(10.0f); }
      CHECK(b.r.phase() == P::FLY_HOME || b.r.phase() == P::DESCENT, "10 s of few satellites, 10 s of enough, again and again: the count goes down again, it carries on"); }
    // not getting anywhere
    { RescSim s(300, 0, 5, 0); s.r.cfg.alt_mode = RESCUE_ALT_FIXED; s.r.cfg.return_alt_m = 100; s.climb_factor = 0.3f; s.in.request = true; s.step(); const float t0 = s.t; s.until(P::FAILED, 30);
      CHECK(s.r.failure() == RescueFail::STALLED && fabsf(s.t - t0 - 10.0f) < 1.1f, "climbing at less than half the wanted rate (1.5 of 5 m/s) for 10 s: failed (STALLED, after %.0f s)", s.t - t0);
      RescSim ok(300, 0, 5, 0); ok.climb_factor = 0.6f; ok.in.request = true; const bool rot = ok.until(P::ROTATE, 30); CHECK(rot, "at 60 %% of the rate it is slow but gets there"); }
    { RescSim s(500, 0, 30, 180); s.r.cfg.alt_mode = RESCUE_ALT_FIXED; s.wind_n = 7.0f; s.in.request = true; s.until(P::FLY_HOME, 10); const float t0 = s.t; s.until(P::FAILED, 40);
      CHECK(s.r.failure() == RescueFail::FLYAWAY && fabsf(s.t - t0 - 20.0f) < 2.5f && s.dist() > 480.0f, "a head wind that leaves 0.5 of the 7.5 m/s: after 20 s of closing in at less than a fifth of the wanted speed, failed (FLYAWAY, after %.0f s)", s.t - t0);
      RescSim ok(500, 0, 30, 180); ok.r.cfg.alt_mode = RESCUE_ALT_FIXED; ok.wind_n = 5.0f; ok.in.request = true; ok.until(P::FLY_HOME, 10); ok.run(60.0f);
      CHECK(ok.r.phase() == P::FLY_HOME && ok.dist() < 380.0f, "one that leaves 2.5 m/s (a third): slow, but it carries on (%.0f m to go after a minute)", ok.dist()); }
    // disarmed: everything forgotten
    { RescSim s(100, 0, 40, 0); s.run(0.5f); s.in.request = true; s.run(2.0f); s.in.armed = false; s.step();
      CHECK(s.r.phase() == P::IDLE && s.r.max_height() == 0 && !s.out.active, "disarmed: idle, the highest point of the flight forgotten"); }
}

int main() {
    t_flow(); t_flow_fusion(); t_tilt_aid(); t_poscontrol(); t_poshold(); t_rescue();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
