// In-flight autotune (src/core/autotune.h, ArduPilot's AC_AutoTune_Multi) against a simulated craft: three independent axes, each
// a motor lag, an inertia and some drag, flown by a Betaflight-scale PID (the firmware's own gain scales). The plant's numbers are a
// 5-inch quad's (about 50000 deg/s^2 of roll authority at half the motor range, a yaw ten times weaker, 4 ms from the gyro to the motors).
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <random>
#include "../src/core/autotune.h"
#include "../src/core/pid.h"

using namespace fc;
static int g_pass = 0, g_fail = 0;
#define CHECK(cond, ...) do { if (cond) g_pass++; else { g_fail++; printf("  FAIL line %d: %s   ", __LINE__, #cond); printf(__VA_ARGS__); printf("\n"); } } while (0)

static std::mt19937 rng(7);
static float gauss(float s) { std::normal_distribution<float> d(0.0f, s); return d(rng); }

struct Plant {
    float authority[3] = {100000.0f, 100000.0f, 10000.0f}; // deg/s^2 per unit of motor command (u = PID sum / 1000): Betaflight's P 45 is a 25 Hz rate loop
    float drag = 2.0f, tau = 0.02f, noise = 0.5f; int delay = 4;      // (delay: samples of the gyro -> ESC -> motor pipeline, 1 ms each)
};

struct Craft {
    Plant pl; float rate[3] = {}, att[3] = {}, ulag[3] = {}, iterm[3] = {}, dlpf[3] = {}, prev[3] = {}, pipe[3][16] = {}; int head = 0;
    float gyro(int a) { return rate[a] + gauss(pl.noise); }
    // one 1 kHz step of the rate loop on all three axes with these gains
    void step(const float sp[3], const TuneGains& g, float dt) {
        for (int a = 0; a < 3; a++) {
            const float gy = gyro(a), e = sp[a] - gy;
            const float kp = kPtermScale * g.p[a], ki = kItermScale * g.i[a] * (a == YAW ? 2.5f : 1.0f), kd = kDtermScale * g.d[a];
            iterm[a] = clampf(iterm[a] + ki * e * dt, -400.0f, 400.0f);
            const float dg = (gy - prev[a]) / dt; prev[a] = gy; dlpf[a] += (dg - dlpf[a]) * (dt / (dt + 1.0f / (2.0f * kPi * 100.0f)));
            const float u = clampf((kp * e + iterm[a] - kd * dlpf[a]) / 1000.0f, -0.5f, 0.5f);
            pipe[a][head] = u; const float ud = pipe[a][(head - pl.delay + 16) % 16];
            ulag[a] += (ud - ulag[a]) * dt / pl.tau;
            rate[a] += (pl.authority[a] * ulag[a] - pl.drag * rate[a]) * dt;
            att[a] += rate[a] * dt;
        }
        head = (head + 1) % 16;
    }
};

// Fly the autotune until it ends (or `limit_s`); the usual controller between the twitches: Stabilize-like level and heading hold.
struct Run { AutoTune::Mode mode; float seconds; int twitches; float max_tilt; };
static Run fly(AutoTune& at, Craft& c, const TuneGains& g0, float limit_s, bool heading_hold = true) {
    const float dt = 0.001f; at.start(g0); int twitches = 0; bool was = false; float tilt = 0;
    int i = 0;
    for (; i < (int)(limit_s / dt) && at.mode() == AutoTune::TUNING; i++) {
        AutoTune::In in; for (int a = 0; a < 3; a++) { in.gyro_dps[a] = c.rate[a]; in.att_deg[a] = c.att[a]; } in.dt = dt;
        const AutoTune::Out o = at.update(in);
        const TuneGains g = at.gains(o.gains);
        float sp[3];
        if (o.control) for (int a = 0; a < 3; a++) sp[a] = o.sp_dps[a];
        else { sp[0] = g.stab_p[0] * -c.att[0]; sp[1] = g.stab_p[1] * -c.att[1]; sp[2] = heading_hold ? g.stab_p[2] * -c.att[2] : 0.0f; }
        if (o.control && !was) twitches++;
        was = o.control; tilt = fmaxf(tilt, fmaxf(fabsf(c.att[0]), fabsf(c.att[1])));
        c.step(sp, g, dt);
    }
    return Run{at.mode(), i * dt, twitches, tilt};
}

// The step response of one axis with fixed gains: overshoot (fraction) and 90 % rise time (s)
static void response(const TuneGains& g, int axis, float step_dps, float& overshoot, float& rise_s) {
    Craft c; c.pl.noise = 0.0f; const float dt = 0.001f; float peak = 0; rise_s = -1;
    for (int i = 0; i < 600; i++) { float sp[3] = {0, 0, 0}; sp[axis] = step_dps; c.step(sp, g, dt); peak = fmaxf(peak, c.rate[axis]); if (rise_s < 0 && c.rate[axis] > 0.9f * step_dps) rise_s = i * dt; }
    overshoot = peak / step_dps - 1.0f;
}

static void t_tunes_rate() {
    printf("[autotune] rate D, then P, on roll and pitch; yaw P - ends, and the result flies well\n");
    TuneGains g0; AutoTune at; at.cfg.axes = 7; Craft c;
    const Run r = fly(at, c, g0, 1200.0f);
    const TuneGains t = at.result();
    CHECK(r.mode == AutoTune::FINISHED && at.axes_done() == 7, "the three axes done after %.0f s of flight, %d twitches (mode %d)", r.seconds, r.twitches, (int)r.mode);
    CHECK(r.max_tilt < 0.9f * at.cfg.angle_max_deg, "never tilted beyond the limit (%.1f deg)", r.max_tilt);
    CHECK(t.p[0] != g0.p[0] && t.d[0] != g0.d[0] && t.p[0] >= AutoTune::kPMin && t.p[0] <= AutoTune::kPMax && t.d[0] >= AutoTune::kDMin && t.d[0] <= AutoTune::kDMax,
          "roll: P %.0f -> %.0f, D %.0f -> %.0f (within the ranges)", g0.p[0], t.p[0], g0.d[0], t.d[0]);
    CHECK(t.d[YAW] == g0.d[YAW] && t.p[YAW] != g0.p[YAW], "yaw: P %.0f -> %.0f, its D left alone", g0.p[YAW], t.p[YAW]);
    // (the step with the I-term off: in the firmware I-term relax holds it during a stick step; this toy PID has no relax)
    TuneGains tn = t; tn.i[0] = 0; float os, rise; response(tn, ROLL, 200.0f, os, rise);
    CHECK(os < 0.12f && rise > 0.0f && rise < 0.04f, "with the result (P %.0f, D %.0f): a 200 deg/s roll step overshoots %.0f %% (the aggressiveness: 7.5 %%) and gets there in %.0f ms", t.p[0], t.d[0], os * 100, rise * 1000);
    CHECK(fabsf(t.i[0] / t.p[0] - g0.i[0] / g0.p[0]) < 1e-3f && fabsf(t.d_max[0] / t.d[0] - g0.d_max[0] / g0.d[0]) < 1e-3f,
          "I and D max keep the pilot's ratios to P and D (I/P %.2f, Dmax/D %.2f)", t.i[0] / t.p[0], t.d_max[0] / t.d[0]);
    CHECK(t.stab_p[0] == g0.stab_p[0] && t.accel_rp_dss == g0.accel_rp_dss, "on Angle (angle_p off): Stabilize P and the acceleration limit untouched");
    // a craft with twice the authority (lighter, stronger motors) ends with lower gains
    TuneGains g1; AutoTune a2; a2.cfg.axes = 1; Craft c2; c2.pl.authority[0] = 200000.0f;
    fly(a2, c2, g1, 1200.0f);
    CHECK(a2.mode() == AutoTune::FINISHED && a2.result().p[0] < t.p[0], "twice the authority: a lower P (%.0f against %.0f)", a2.result().p[0], t.p[0]);
}

static void t_tunes_angle() {
    printf("[autotune] on Stabilize: the angle steps too - Stabilize P and the acceleration limit\n");
    TuneGains g0; AutoTune at; at.cfg.axes = 3; at.cfg.angle_p = true; Craft c;
    const Run r = fly(at, c, g0, 1500.0f);
    const TuneGains t = at.result();
    CHECK(r.mode == AutoTune::FINISHED && at.axes_done() == 3, "roll and pitch done (%.0f s)", r.seconds);
    CHECK(t.stab_p[0] != g0.stab_p[0] && t.stab_p[0] >= AutoTune::kSpMin && t.stab_p[0] <= AutoTune::kSpMax, "Stabilize P roll %.2f -> %.2f", g0.stab_p[0], t.stab_p[0]);
    CHECK(t.accel_rp_dss != g0.accel_rp_dss && t.accel_rp_dss >= AutoTune::kAccelRpMin && t.accel_rp_dss <= AutoTune::kAccelRpMax && t.stab_p[YAW] == g0.stab_p[YAW],
          "the roll / pitch acceleration limit measured (%.0f deg/s^2); yaw not tuned: its P untouched", t.accel_rp_dss);
}

static void t_sequence_and_gains() {
    printf("[autotune] the steps per axis, the gains flown in each phase\n");
    TuneGains g0; AutoTune at; at.cfg.axes = 4; at.start(g0);
    CHECK(at.axis() == YAW && at.type() == AutoTune::RATE_P_UP, "yaw alone: starts with RATE_P_UP (no D steps on yaw)");
    AutoTune ab; ab.cfg.axes = 1; ab.start(g0);
    CHECK(ab.axis() == ROLL && ab.type() == AutoTune::RATE_D_UP, "roll: starts with RATE_D_UP");
    const TuneGains te = ab.gains(AutoTune::TEST), in = ab.gains(AutoTune::INTRA_TEST), og = ab.gains(AutoTune::ORIGINAL);
    CHECK(te.f[0] == 0.0f && fabsf(te.i[0] - g0.i[0] * 0.01f) < 1e-4f && te.d_max[0] == te.d[0] && te.f[1] == g0.f[1] && fabsf(te.i[1] - g0.i[1] * 0.1f) < 1e-4f,
          "twitching roll: its feed-forward 0, its I 1 %%, no D max boost; pitch between twitches: I 10 %%, its feed-forward kept");
    CHECK(fabsf(in.i[0] - g0.i[0] * 0.1f) < 1e-4f && in.f[0] == g0.f[0] && og.i[0] == g0.i[0] && og.d_max[0] == g0.d_max[0], "between twitches I 10 %%; ORIGINAL is untouched");
    AutoTune none; none.cfg.axes = 0; none.start(g0);
    CHECK(none.mode() == AutoTune::FINISHED && none.axes_done() == 0, "no axis chosen: nothing to do");
}

static void t_pilot_and_level() {
    printf("[autotune] the pilot takes over; a twitch only from level\n");
    TuneGains g0; AutoTune at; at.cfg.axes = 1; at.start(g0); const float dt = 0.001f;
    AutoTune::In in; in.dt = dt; int ctl = 0;
    for (int i = 0; i < 2000; i++) { in.pilot = i < 1000; const AutoTune::Out o = at.update(in); if (o.control) ctl++; if (i == 999) CHECK(at.overridden() && ctl == 0, "a stick out of the centre: nothing twitches, the pilot flies"); }
    CHECK(ctl > 0, "sticks back for 0.5 s, level for 0.25 s: the twitches start");
    AutoTune lv; lv.cfg.axes = 1; lv.start(g0); AutoTune::In t; t.dt = dt; t.att_deg[ROLL] = 10.0f; int c2 = 0;
    for (int i = 0; i < 3000; i++) { if (lv.update(t).control) c2++; }
    CHECK(c2 == 0 && lv.mode() == AutoTune::TUNING, "tilted 10 deg: no twitch for 3 s");
    for (int i = 0; i < 4000; i++) lv.update(t);
    CHECK(lv.mode() == AutoTune::FAILED, "never level for 6 s: failed (ArduPilot: 3 x the level timeout), the original gains from here on");
    const AutoTune::Out f = lv.update(t);
    CHECK(!f.control && f.gains == AutoTune::ORIGINAL, "failed: the usual controller with the original gains");
}

static void t_aborts() {
    printf("[autotune] a twitch that leans too far is cut short\n");
    TuneGains g0; AutoTune at; at.cfg.axes = 1; at.cfg.angle_max_deg = 30; Craft c; c.pl.authority[0] = 4000.0f;   // a weak craft: 180 deg/s takes long, it leans far
    fly(at, c, g0, 60.0f);
    float worst = 0; Craft c2; c2.pl.authority[0] = 4000.0f; AutoTune a2; a2.cfg.axes = 1; a2.start(g0);
    for (int i = 0; i < 60000 && a2.mode() == AutoTune::TUNING; i++) {
        AutoTune::In in; for (int a = 0; a < 3; a++) { in.gyro_dps[a] = c2.rate[a]; in.att_deg[a] = c2.att[a]; } in.dt = 0.001f;
        const AutoTune::Out o = a2.update(in); const TuneGains g = a2.gains(o.gains); float sp[3];
        if (o.control) for (int a = 0; a < 3; a++) sp[a] = o.sp_dps[a]; else for (int a = 0; a < 3; a++) sp[a] = g.stab_p[a] * -c2.att[a];
        c2.step(sp, g, 0.001f); worst = fmaxf(worst, fabsf(c2.att[0]));
    }
    CHECK(worst < 30.0f * 0.9f, "a weak craft: the twitches stop before 5/6 of the lean limit (%.1f deg at most)", worst);
}

int main() {
    t_sequence_and_gains(); t_pilot_and_level(); t_aborts(); t_tunes_rate(); t_tunes_angle();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
