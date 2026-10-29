// Unit tests of the rewritten navigation controllers (src/core/nav/): Altitude Hold (alt_control.h), Position Hold (pos_control.h).
// Each against a small plant of its own; the full craft is in test_flight.cpp / test_fw.cpp.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../src/core/nav/alt_control.h"
#include "../src/core/nav/pos_hold.h"
using namespace fc;
using namespace fc::nav;

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, ...) do { const bool ok_ = (cond); if (getenv("V")) { printf("    "); printf(__VA_ARGS__); printf("\n"); } if (ok_) g_pass++; else { g_fail++; printf("  FAIL line %d: %s   ", __LINE__, #cond); printf(__VA_ARGS__); printf("\n"); } } while (0)

// a craft on a vertical line: thrust ~ motor output (hover at `hover`), a ground at height 0, a lag of the motors
struct Plant1D {
    float hover = 0.45f, idle = 0.055f, h = 0, v = 0, a = 0, out = 0, lag_s = 0.03f;
    void step(float throttle, float dt) {
        const float cmd = idle + clampf(throttle, 0.0f, 1.0f) * (1.0f - idle);
        out += (cmd - out) * dt / (lag_s + dt);
        a = kG0 * (out / hover - 1.0f);
        v += a * dt; h += v * dt;
        if (h <= 0.0f) { h = 0.0f; if (v < 0.0f) v = 0.0f; a = v > 0.0f ? a : 0.0f; }
    }
};

struct AltRig {
    AltHoldMode m; Plant1D p; float t = 0, offset_d = 0; uint8_t resets = 0; float step_d = 0;
    AltRig() { m.init(); m.set_limits(p.idle, 1.0f); }
    float run(float seconds, float stick, bool armed = true, float dt = 0.002f) {
        float thr = 0;
        for (int i = 0; i < (int)(seconds / dt + 0.5f); i++) {
            AltInput in; in.armed = armed; in.stick01 = stick; in.pos_d = -p.h + offset_d; in.vel_d = -p.v; in.acc_d = -p.a; in.reset_count = resets; in.reset_step_d = step_d;
            thr = m.update(in, dt); m.learn(thr, 1.0f, -p.a, dt); p.step(armed ? thr : 0.0f, dt); t += dt;
        }
        return thr;
    }
};

static void t_alt_setpoint() {
    printf("[alt setpoint] the stick as a climb rate, the ceiling\n");
    AltConfig c;
    CHECK(AltSetpoint::stick_climb(c, 0.5f) == 0.0f && AltSetpoint::stick_climb(c, 0.54f) == 0.0f && AltSetpoint::stick_climb(c, 0.46f) == 0.0f, "inside mid +- deadband: hold");
    CHECK(fabsf(AltSetpoint::stick_climb(c, 1.0f) - c.climb_max_mps) < 1e-6f && fabsf(AltSetpoint::stick_climb(c, c.low_stick) + c.descend_max_mps) < 1e-6f, "the ends: full climb / descent rate");
    CHECK(fabsf(AltSetpoint::stick_climb(c, 0.775f) - 0.5f * c.climb_max_mps) < 1e-3f, "linear in between (half way up = half the rate)");
    CHECK(AltSetpoint::failsafe_climb(0.0f) == -0.3f && AltSetpoint::failsafe_climb(20.0f) == -1.5f, "failsafe descent: 0.3 m/s at the ground, 1.5 m/s high up");
    CHECK(AltSetpoint::ceiling_limit(c, 2.0f, 50.0f, 0.01f) == 2.0f, "no ceiling set: nothing limited");
    c.ceiling_m = 30.0f;
    CHECK(AltSetpoint::ceiling_limit(c, 2.0f, 10.0f, 0.01f) == 2.0f && AltSetpoint::ceiling_limit(c, 2.0f, 29.3f, 0.01f) < 0.3f && AltSetpoint::ceiling_limit(c, 0.0f, 31.0f, 0.01f) < -1.0f,
          "ceiling 30 m: free far below, slowed just under it, sent down above it");
}

static void t_alt_takeoff_hold_land() {
    printf("[alt hold] take-off, hold, climb, land on a plant that hovers at 45 %%\n");
    AltRig r; r.run(0.5f, 0.0f);
    CHECK(r.m.state() == AhState::LANDED_IDLE && r.p.h == 0.0f, "armed, stick down: idle on the ground");
    r.run(0.2f, 0.8f); CHECK(r.m.state() == AhState::TAKEOFF && !r.m.lifted(), "stick up: the throttle ramps (not lifted yet)");
    float peak = 0; for (int i = 0; i < 400; i++) { r.run(0.01f, i < 50 ? 0.8f : 0.5f); peak = fmaxf(peak, r.p.h); }
    CHECK(r.m.state() == AhState::FLYING && fabsf(r.p.h - 0.6f) < 0.15f && peak < 0.9f, "take-off to %.2f m (0.6 asked, %.2f at most)", r.p.h, peak);
    CHECK(fabsf(r.m.hover() - r.p.hover) < 0.03f, "hover output found at lift-off and in the hover: %.3f (true %.3f)", r.m.hover(), r.p.hover);
    const float h0 = r.p.h; float dev = 0; for (int i = 0; i < 300; i++) { r.run(0.01f, 0.5f); dev = fmaxf(dev, fabsf(r.p.h - h0)); }
    CHECK(dev < 0.05f, "stick in the middle: holds within %.3f m", dev);
    float vmax = 0, amax = 0; for (int i = 0; i < 300; i++) { r.run(0.01f, 1.0f); vmax = fmaxf(vmax, r.p.v); amax = fmaxf(amax, fabsf(r.m.controller().acc_target_d())); }
    CHECK(fabsf(vmax - 2.0f) < 0.25f && amax <= 2.5f + 1e-3f, "stick up: climbs at %.2f m/s (2 asked), the target never accelerates faster than 2.5 m/s^2 (%.2f)", vmax, amax);
    const float h1 = r.p.h; r.run(3.0f, 0.5f); const float h2 = r.p.h; r.run(2.0f, 0.5f);
    CHECK(h2 - h1 < 2.0f && fabsf(r.p.h - h2) < 0.05f && fabsf(r.p.v) < 0.05f, "back to the middle: stops jerk-limited (%.2f m of coasting from 2 m/s) and holds", h2 - h1);
    r.run(12.0f, 0.0f);
    CHECK(r.m.state() == AhState::LANDED_IDLE && r.p.h < 0.01f, "stick down: descends, lands, idles (h %.3f)", r.p.h);
    r.run(0.1f, 0.8f); CHECK(r.m.state() == AhState::TAKEOFF, "the stick was down on the ground: up again takes off again");
    r.run(0.1f, 0.8f, false); CHECK(r.m.state() == AhState::STOPPED && r.run(0.01f, 0.8f, false) == 0.0f, "disarmed: stopped, no throttle");
}

static void t_alt_edges() {
    printf("[alt hold] what the pilot is not in charge of\n");
    { AltRig r; r.run(0.5f, 0.0f); AltInput in; in.armed = true; in.stick01 = 0.9f; in.pilot_absent = true;
      for (int i = 0; i < 500; i++) r.m.update(in, 0.002f);
      CHECK(r.m.state() == AhState::LANDED_IDLE, "no pilot at the stick (link lost): no take-off whatever the stick shows"); }
    { AltRig r; r.p.hover = 0.6f; r.run(0.3f, 0.0f); int n = 0; while (r.m.state() != AhState::LANDED_IDLE || n == 0) { r.run(0.01f, 0.8f); r.p.h = 0; r.p.v = 0; if (++n > 400) break; }
      CHECK(n < 400 && !r.m.lifted(), "held down (it never leaves the ground): the ramp gives up at 90 %% and wants the stick down (%.1f s)", n * 0.01f); }
    { AltRig r; r.run(0.3f, 0.0f); r.run(0.6f, 0.8f); r.run(6.0f, 0.5f); const float h0 = r.p.h;
      // the estimator sets its height 3 m lower (a reset): the target moves with it - the craft does not climb 3 m
      r.offset_d = 3.0f; r.step_d = 3.0f; r.resets++; float lo = h0, hi = h0;
      for (int i = 0; i < 300; i++) { r.run(0.01f, 0.5f); lo = fminf(lo, r.p.h); hi = fmaxf(hi, r.p.h); }
      CHECK(hi - h0 < 0.05f && h0 - lo < 0.05f, "an estimator reset of 3 m in the hover: the craft stays (%.2f .. %.2f of %.2f)", lo, hi, h0);
      AltRig q; q.run(0.3f, 0.0f); q.run(0.6f, 0.8f); q.run(6.0f, 0.5f); const float g0 = q.p.h; q.offset_d = 3.0f; q.run(3.0f, 0.5f);
      CHECK(q.p.h - g0 > 1.5f, "(the same step without the reset reported: the craft flies it, %.1f m up)", q.p.h - g0); }
    { AltRig r; r.m.cfg.ceiling_m = 5.0f; r.run(0.3f, 0.0f); r.run(0.6f, 0.8f); float top = 0; for (int i = 0; i < 1000; i++) { r.run(0.01f, 1.0f); top = fmaxf(top, r.p.h); }
      CHECK(top < 5.0f && top > 4.0f, "ceiling 5 m, stick held full up for 10 s: stops at %.2f m", top); }
    { AltRig r; r.run(0.3f, 0.0f); r.run(0.6f, 0.8f); r.run(5.0f, 0.5f); const float th = r.run(0.002f, 0.5f);
      AltInput in; in.armed = true; in.stick01 = 0.5f; in.pos_d = -r.p.h; const float t1 = r.m.update(in, -1.0f), t2 = r.m.update(in, NAN), t3 = r.m.update(in, 5.0f);
      CHECK(fabsf(t1 - th) < 0.05f && t1 == t2 && t2 == t3, "a bad dt (negative, NaN, 5 s): the last throttle again (%.3f), nothing integrated", t1); }
    { AltRig r; r.run(0.3f, 0.0f); r.run(0.6f, 0.8f); r.run(5.0f, 0.5f);
      AltInput in; in.armed = true; in.stick01 = 0.5f; in.failsafe_descent = true; float t = 0;
      for (int i = 0; i < 6000 && r.m.state() != AhState::LANDED_IDLE; i++) { in.pos_d = -r.p.h; in.vel_d = -r.p.v; in.acc_d = -r.p.a; const float thr = r.m.update(in, 0.002f); r.p.step(thr, 0.002f); t += 0.002f; }
      CHECK(r.m.state() == AhState::LANDED_IDLE && r.p.h < 0.01f && t > 1.5f, "failsafe descent from %.1f m: down slowly and landed after %.1f s", 0.6f, t); }
    { AltRig r; r.p.hover = 0.2f; r.run(0.3f, 0.0f); r.run(0.6f, 0.8f); r.run(8.0f, 0.5f);
      CHECK(r.m.state() == AhState::FLYING && fabsf(r.p.h - 0.6f) < 0.15f && fabsf(r.m.hover() - 0.2f) < 0.02f, "a strong craft (hovers at 20 %%): the same take-off, hover %.3f found", r.m.hover()); }
}

static void t_nav_math() {
    printf("[nav math] square-root controller, lean <-> acceleration, frames\n");
    CHECK(sqrt_controller(0.5f, 1.0f, 2.5f) == 0.5f && fabsf(sqrt_controller(10.0f, 1.0f, 2.5f) - sqrtf(2 * 2.5f * (10 - 1.25f))) < 1e-4f && sqrt_controller(-10.0f, 1.0f, 2.5f) < -6.0f, "linear near, square root far, odd");
    float r, p, f, rt; accel_to_lean(kG0 * tanf(10 * kDeg2Rad), 0, r, p); CHECK(fabsf(p - 10) < 1e-3f && fabsf(r) < 1e-4f, "a forward acceleration of g tan 10 deg = 10 deg nose down");
    lean_to_accel(12.0f, -7.0f, f, rt); accel_to_lean(f, rt, r, p); CHECK(fabsf(r - 12.0f) < 0.05f && fabsf(p + 7.0f) < 0.01f, "lean -> acceleration -> lean: the same lean (%.2f, %.2f)", r, p);
    float n, e; body_to_ne(1.0f, 0.0f, 90.0f, n, e); CHECK(fabsf(n) < 1e-5f && fabsf(e - 1.0f) < 1e-5f, "nose east: forwards = east");
    ne_to_body(3.0f, 4.0f, 30.0f, f, rt); body_to_ne(f, rt, 30.0f, n, e); CHECK(fabsf(n - 3.0f) < 1e-4f && fabsf(e - 4.0f) < 1e-4f, "and back");
    float v[2] = {3, 4}; limit_len(v, 1.0f); CHECK(fabsf(v[0] - 0.6f) < 1e-5f && fabsf(v[1] - 0.8f) < 1e-5f, "limit_len keeps the direction");
}

// a craft on the ground plane: the lean asked for is reached with a 0.1 s lag, thrust tilts it, air drag, wind
struct Plant2D {
    float pn = 0, pe = 0, vn = 0, ve = 0, roll = 0, pitch = 0, yaw = 0, drag = 0.35f, wind_n = 0, wind_e = 0;
    void step(float roll_cmd, float pitch_cmd, float dt) {
        roll += (roll_cmd - roll) * dt / (0.1f + dt); pitch += (pitch_cmd - pitch) * dt / (0.1f + dt);
        float f, r; lean_to_accel(roll, pitch, f, r); float an, ae; body_to_ne(f, r, yaw, an, ae);
        an += drag * (wind_n - vn); ae += drag * (wind_e - ve);
        vn += an * dt; ve += ae * dt; pn += vn * dt; pe += ve * dt;
    }
};
struct PhRig {
    PosHoldMode m; Plant2D p; float off_n = 0, off_e = 0; uint8_t resets = 0; float sn = 0, se = 0;
    PhRig() { PosHoldInput in; in.airborne = true; m.init(in); }
    void run(float seconds, float stick_roll_deg = 0, float stick_pitch_deg = 0, float dt = 0.005f) {
        for (int i = 0; i < (int)(seconds / dt + 0.5f); i++) {
            PosHoldInput in; in.airborne = true; in.yaw_deg = p.yaw; in.pos_n = p.pn + off_n; in.pos_e = p.pe + off_e; in.vel_n = p.vn; in.vel_e = p.ve;
            in.pilot_roll_deg = stick_roll_deg; in.pilot_pitch_deg = stick_pitch_deg; in.roll_centred = stick_roll_deg == 0; in.pitch_centred = stick_pitch_deg == 0;
            in.reset_count = resets; in.reset_step_n = sn; in.reset_step_e = se;
            float r, q; m.update(in, dt, r, q); p.step(r, q, dt);
        }
    }
    float dist(float n, float e) const { return hypotf(p.pn - n, p.pe - e); }
};

static void t_pos_hold() {
    printf("[pos hold] brake, hold, wind, resets, the fence\n");
    { PhRig r; r.run(4.0f); CHECK(r.m.holding() && r.dist(0, 0) < 0.1f, "let alone: holds the spot (%.3f m)", r.dist(0, 0));
      r.run(3.0f, 0, 15.0f); const float v = r.p.vn;
      CHECK(r.m.state(1) == PhAxis::PILOT && fabsf(r.p.pitch - 15.0f) < 0.5f && v > 3.0f, "pitch stick: the pilot's lean (%.1f deg), %.1f m/s north", r.p.pitch, v);
      const float n0 = r.p.pn; float t_hold = -1, back = 0;
      float jump = 0, prev = r.m.out_deg(1);
      for (int i = 0; i < 3000; i++) { r.run(0.005f); back = fmaxf(back, -r.p.pitch); if (r.m.state(1) == PhAxis::TO_LOITER || r.m.state(1) == PhAxis::LOITER) jump = fmaxf(jump, fabsf(r.m.out_deg(1) - prev)); prev = r.m.out_deg(1); if (t_hold < 0 && r.m.holding()) t_hold = 0.005f * i; }
      CHECK(jump < 0.1f, "the brake hands over to the position controller smoothly: %.3f deg at most per 5 ms (0.19 without the blend)", jump);
      CHECK(t_hold > 1.0f && t_hold < 10.0f && back > 4.0f && hypotf(r.p.vn, r.p.ve) < 0.1f, "let go at %.1f m/s: brakes (%.0f deg back), stops %.1f m on, holds after %.1f s", v, back, r.p.pn - n0, t_hold); }
    { PhRig r; r.run(4.0f); r.p.wind_e = 5.0f; float far = 0; for (int i = 0; i < 2500; i++) { r.run(0.01f); far = fmaxf(far, r.dist(0, 0)); }
      CHECK(far < 3.0f && r.dist(0, 0) < 0.3f && r.m.wind_e() < -1.2f, "5 m/s of wind from the west: blown %.2f m at most, back within %.2f m, wind learned %.2f m/s^2", far, r.dist(0, 0), r.m.wind_e()); }
    { PhRig r; r.run(4.0f); r.off_n = 12.0f; r.off_e = -4.0f; r.sn = 12.0f; r.se = -4.0f; r.resets++; float far = 0;
      for (int i = 0; i < 300; i++) { r.run(0.01f); far = fmaxf(far, r.dist(0, 0)); }
      CHECK(far < 0.1f, "the estimator resets its position by 12 m north, 4 m west: the target moves with it, the craft stays (%.3f m)", far);
      PhRig q; q.run(4.0f); q.off_n = 12.0f; q.off_e = -4.0f; q.run(8.0f);
      CHECK(q.dist(0, 0) > 1.5f, "(the same step without the reset reported: the craft flies off towards the old target, %.1f m)", q.dist(0, 0)); }
    { PhRig r; r.m.cfg.fence_radius_m = 20.0f; r.run(2.0f); float far = 0; bool breached = false;
      for (int i = 0; i < 1500; i++) { r.run(0.01f, 0, 20.0f); far = fmaxf(far, r.dist(0, 0)); breached |= r.m.fence_breached(); }
      CHECK(breached && far < 27.0f && r.dist(0, 0) < 21.0f && r.m.holding(), "fence 20 m, the stick held forwards for 15 s: stopped at %.1f m (%.1f at most), holding at the edge", r.dist(0, 0), far);
      for (int i = 0; i < 300; i++) r.run(0.01f, 0, -15.0f);
      CHECK(!r.m.fence_breached() && r.p.pn < 15.0f, "stick back towards home: the pilot flies again (%.1f m)", r.p.pn); }
    { PhRig r; r.run(4.0f, 0, 0); float a, b; PosHoldInput in; in.airborne = true; r.m.update(in, -1.0f, a, b); float c, d; r.m.update(in, NAN, c, d);
      CHECK(a == r.m.out_deg(0) && c == a && d == b, "a bad dt: the last lean again"); }
    { PosHoldMode m; PosHoldInput in; in.airborne = false; in.pilot_roll_deg = 7; in.roll_centred = false; m.init(in); float r, p; m.update(in, 0.01f, r, p);
      CHECK(r == 7.0f && p == 0.0f && m.state(0) == PhAxis::PILOT, "on the ground: the sticks pass through, nothing is held"); }
}

int main() {
    t_alt_setpoint(); t_alt_takeoff_hold_land(); t_alt_edges();
    t_nav_math(); t_pos_hold();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
