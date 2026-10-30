// Unit tests of the mission runner (nav/mission.h): ArduCopter AUTO semantics (mode_auto.cpp, AP_Mission.cpp) against a point
// mass that follows the target point with its velocity feed-forward and climbs at the rate asked for. The model is written here
// from what a craft does, not from the code under test.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <vector>
#include <string>
#include "../src/core/nav/mission.h"
using namespace fc;

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, ...) do { const bool ok_ = (cond); if (getenv("V")) { printf("    "); printf(__VA_ARGS__); printf("\n"); } if (ok_) g_pass++; else { g_fail++; printf("  FAIL line %d: %s   ", __LINE__, #cond); printf(__VA_ARGS__); printf("\n"); } } while (0)

static MissionItem item(MissionCmd c, float n = 0, float e = 0, float alt = 0, uint16_t p = 0, uint8_t arg = 0, bool pos = true, bool has_alt = true) {
    MissionItem i; i.cmd = c; i.n = n; i.e = e; i.has_pos = pos; i.alt_m = alt; i.has_alt = has_alt; i.p = p; i.arg = arg; return i;
}
static MissionItem takeoff(float alt) { return item(MissionCmd::TAKEOFF, 0, 0, alt, 0, 0, false); }
static MissionItem wp(float n, float e, float alt, uint16_t hold = 0) { return item(MissionCmd::WAYPOINT, n, e, alt, hold); }
static MissionItem loiter(uint16_t s) { return item(MissionCmd::LOITER_TIME, 0, 0, 0, s, 0, false, false); }
static MissionItem servo(uint8_t n, uint16_t us) { return item(MissionCmd::SERVO, 0, 0, 0, us, n, false, false); }
static MissionItem land(float n, float e) { return item(MissionCmd::LAND, n, e, 0, 0, 0, true, false); }
static MissionItem delay(uint16_t s) { return item(MissionCmd::DELAY, 0, 0, 0, s, 0, false, false); }
static MissionItem rtl() { return item(MissionCmd::RTL, 0, 0, 0, 0, 0, false, false); }
static MissionItem speed(float mps) { return item(MissionCmd::SPEED, 0, 0, 0, (uint16_t)lroundf(mps * 10.0f), 0, false, false); }

static const char* state_name(MissionState s) {
    static const char* n[] = {"IDLE", "TAKEOFF", "WAYPOINT", "LOITER", "LANDING", "GROUND_WAIT", "ARMING", "RTL", "DONE", "REFUSED", "FAILED"};
    return (int)s < 11 ? n[(int)s] : "?";
}

struct ServoEvent { float t; int servo; uint16_t us; };
struct MSim {
    MissionRunner m; MissionPlan plan; MissionInput in; MissionOutput out;
    float t = 0, vz = 0, wind_n = 0, wind_e = 0, low_t = 0; const float dt = 0.01f;
    std::string order; int last_state = -1, last_index = -1;
    std::vector<ServoEvent> servos; int disarms = 0, arms = 0; float warn_first = -1, warn_last = -1, max_speed = 0, max_z = 0, max_ff = 0;
    float vcap = 12.0f, max_lead = 0; int want_arm_steps = 0; bool arm_warned = true;      // vcap: the most the craft flies through the air
    std::vector<float> arm_t, disarm_t;
    MSim() { m.cfg.ground_speed_mps = 5.0f; in.pos_valid = in.alt_valid = true; in.armed = true; in.arm_switch_on = true; }
    void add(const MissionItem& i) { plan.item[plan.count++] = i; }
    bool start() { return m.start(plan, in); }
    void step() {
        out = m.update(in, dt); t += dt;
        if ((int)m.state() != last_state || (int)m.index() != last_index) {
            if ((int)m.state() != last_state) { order += state_name(m.state()); order += ' '; }
            last_state = (int)m.state(); last_index = m.index();
        }
        if (out.servo >= 0) servos.push_back({t, out.servo, out.servo_us});
        if (out.horizontal) max_ff = fmaxf(max_ff, hypotf(out.vel_n, out.vel_e));
        if (out.horizontal && m.state() == MissionState::WAYPOINT) max_lead = fmaxf(max_lead, hypotf(out.tgt_n - in.pos_n, out.tgt_e - in.pos_e));
        if (out.want_arm) { want_arm_steps++; if (!out.warn) arm_warned = false; }
        if (out.warn) { if (warn_first < 0) warn_first = t; warn_last = t; }
        if (out.want_disarm && in.armed) { in.armed = false; disarms++; disarm_t.push_back(t); }
        else if (out.want_arm && !in.armed && in.arm_switch_on) { in.armed = true; arms++; arm_t.push_back(t); }
        if (out.want_rtl) {                                   // the rescue flies home (5 m/s) and lands, then reports it
            const float d = hypotf(in.pos_n, in.pos_e);
            if (d > 0.5f) { in.vel_n = -in.pos_n / d * fminf(5.0f, d); in.vel_e = -in.pos_e / d * fminf(5.0f, d); } else in.vel_n = in.vel_e = 0;
            in.pos_n += in.vel_n * dt; in.pos_e += in.vel_e * dt;
            if (d < 0.5f) in.z = fmaxf(in.z - 1.0f * dt, 0.0f);
            if (d < 0.5f && in.z <= 0.0f) { in.armed = false; in.rtl_done = true; }
            in.airborne = in.z > 0.2f; return;
        }
        if (in.armed && out.active) {
            if (out.horizontal) {
                float an = out.vel_n + (out.tgt_n - in.pos_n) - wind_n, ae = out.vel_e + (out.tgt_e - in.pos_e) - wind_e;
                const float c = hypotf(an, ae); if (c > vcap) { an *= vcap / c; ae *= vcap / c; }
                in.vel_n += (an + wind_n - in.vel_n) * dt / 0.8f; in.vel_e += (ae + wind_e - in.vel_e) * dt / 0.8f;
            } else { in.vel_n += -in.vel_n * dt / 0.8f; in.vel_e += -in.vel_e * dt / 0.8f; }
            if (in.airborne || out.takeoff || out.climb_mps < 0.0f) vz += (out.climb_mps - vz) * dt / 0.4f; else vz = 0;
        } else { vz = 0; if (!in.airborne) in.vel_n = in.vel_e = 0; }
        if (in.airborne) { in.pos_n += in.vel_n * dt; in.pos_e += in.vel_e * dt; }
        in.z = fmaxf(in.z + vz * dt, 0.0f); in.vz = vz; if (in.z <= 0.0f && vz < 0) vz = 0;
        in.airborne = in.z > 0.2f;
        // Altitude Hold: on the ground, down wanted, not moving for a second = landed
        low_t = (in.armed && in.z <= 0.05f && out.climb_mps < 0.0f) ? low_t + dt : 0.0f; in.landed = low_t > 1.0f;
        if (in.yaw_deg != in.yaw_deg) in.yaw_deg = 0;
        if (out.yaw_control) in.yaw_deg += out.yaw_rate_dps * dt;
        max_speed = fmaxf(max_speed, hypotf(in.vel_n, in.vel_e)); max_z = fmaxf(max_z, in.z);
    }
    void run(float s) { for (int i = 0; i < (int)(s / dt + 0.5f); i++) step(); }
    bool until(MissionState st, float timeout) { for (float e = 0; e < timeout && m.state() != st; e += dt) step(); return m.state() == st; }
};

static void t_mav_map() {
    printf("[mission] MAV_CMD numbers\n");
    const MissionCmd c[] = {MissionCmd::TAKEOFF, MissionCmd::WAYPOINT, MissionCmd::LOITER_TIME, MissionCmd::LAND, MissionCmd::DELAY, MissionCmd::RTL, MissionCmd::SERVO, MissionCmd::SPEED};
    const uint16_t mav[] = {22, 16, 19, 21, 93, 20, 183, 178};
    bool ok = true; for (int i = 0; i < 8; i++) ok = ok && mav_cmd_of(c[i]) == mav[i] && mission_cmd_of_mav(mav[i]) == c[i];
    CHECK(ok && mission_cmd_of_mav(201) == MissionCmd::NONE && mav_cmd_of(MissionCmd::NONE) == 0, "each command has its MAV_CMD number both ways; one not handled is NONE");
    CHECK(mission_is_nav(MissionCmd::TAKEOFF) && mission_is_nav(MissionCmd::DELAY) && mission_is_nav(MissionCmd::RTL) && !mission_is_nav(MissionCmd::SERVO) && !mission_is_nav(MissionCmd::SPEED), "NAV and DO commands as AP_Mission::is_nav_cmd");
}

static void t_refuse() {
    printf("[mission] what does not start\n");
    { MSim s; s.in.z = 0; s.in.airborne = false; CHECK(!s.start() && s.m.state() == MissionState::REFUSED, "an empty plan"); s.step(); CHECK(!s.out.active, "nothing flies"); }
    { MSim s; s.in.airborne = false; s.add(wp(10, 0, 10)); CHECK(!s.start() && s.m.state() == MissionState::REFUSED, "on the ground, the first NAV command is not TAKEOFF (ArduCopter 'Missing Takeoff Cmd')"); s.run(1.0f); CHECK(!s.out.active && s.in.z == 0.0f, "the craft stays on the ground"); }
    { MSim s; s.in.airborne = false; s.add(servo(1, 1800)); s.add(wp(10, 0, 10)); CHECK(!s.start(), "a DO command before it does not count as a take-off"); }
    { MSim s; s.in.airborne = false; s.in.pos_valid = false; s.add(takeoff(5)); s.add(wp(10, 0, 10)); CHECK(!s.start() && s.m.state() == MissionState::REFUSED, "no position"); }
    { MSim s; s.in.z = 10; s.in.airborne = true; s.add(wp(10, 0, 10)); CHECK(s.start() && s.m.state() != MissionState::REFUSED, "in the air a mission may begin with a waypoint"); }
}

static void t_takeoff_wp() {
    printf("[mission] take-off and two waypoints\n");
    MSim s; s.in.airborne = false; s.add(takeoff(10)); s.add(wp(40, 0, 10)); s.add(wp(40, 30, 15));
    CHECK(s.start(), "starts on the ground with TAKEOFF first");
    s.step(); CHECK(s.out.active && s.out.takeoff && s.out.climb_mps > 0.4f && s.m.state() == MissionState::TAKEOFF, "TAKEOFF: Altitude Hold is asked to take off, a climb is asked for (%.1f m/s)", s.out.climb_mps);
    float left_prev = 1e9f; bool left_down = true; int legs_seen = 0, idx = -1; float max_up = 0;
    for (float e = 0; e < 80 && s.m.state() != MissionState::DONE; e += s.dt) {
        s.step(); max_up = fmaxf(max_up, s.in.z);
        if (s.m.index() != idx) { idx = s.m.index(); legs_seen++; left_prev = 1e9f; }
        if (s.m.state() == MissionState::WAYPOINT) { if (s.m.left() > left_prev + 0.05f) left_down = false; left_prev = s.m.left(); }
    }
    CHECK(s.order == "TAKEOFF WAYPOINT DONE ", "the states: %s", s.order.c_str());
    CHECK(hypotf(s.in.pos_n - 40, s.in.pos_e - 30) < 2.0f && fabsf(s.in.z - 15) < 1.0f, "ended at the last point (%.1f m off, %.1f m up)", hypotf(s.in.pos_n - 40, s.in.pos_e - 30), s.in.z);
    CHECK(s.max_z < 15.5f && s.max_ff <= 5.01f && s.max_speed < 6.5f, "never above the highest point (%.1f m); the target never runs faster than the speed (%.2f m/s; the craft, lagging behind it, %.1f)", s.max_z, s.max_ff, s.max_speed);
    CHECK(left_down && legs_seen == 3, "the way left only shrinks along each leg (%d commands)", legs_seen);
    s.run(5.0f); CHECK(s.out.active && s.out.horizontal && hypotf(s.in.pos_n - 40, s.in.pos_e - 30) < 1.0f && s.m.state() == MissionState::DONE, "the end of the plan in the air: it holds that point (ArduPilot exit_mission -> loiter)");
}

static void t_do_order() {
    printf("[mission] DO commands run when the NAV command before them starts (AP_Mission::update)\n");
    MSim s; s.in.z = 10; s.in.airborne = true;
    s.add(wp(20, 0, 10)); s.add(loiter(3)); s.add(servo(2, 1800)); s.add(loiter(2)); s.add(servo(2, 1100)); s.add(wp(20, 20, 10));
    CHECK(s.start(), "start in the air");
    float arrived = -1; for (float e = 0; e < 40 && s.servos.size() < 2; e += s.dt) { s.step(); if (arrived < 0 && hypotf(s.in.pos_n - 20, s.in.pos_e) < 2.0f) arrived = s.t; }
    CHECK(s.servos.size() == 2 && s.servos[0].servo == 2 && s.servos[0].us == 1800 && s.servos[1].us == 1100, "servo 2 to 1800 us, then to 1100 us, each once (%zu commands)", s.servos.size());
    CHECK(s.servos.size() == 2 && s.servos[0].t >= arrived - 0.05f && s.servos[0].t < arrived + 2.5f && fabsf(s.servos[1].t - s.servos[0].t - 3.0f) < 0.15f, "the first on arrival (%.2f s after it was within 2 m), the second after the 3 s wait (%.2f s)", s.servos.empty() ? 0 : s.servos[0].t - arrived, s.servos.size() < 2 ? 0 : s.servos[1].t - s.servos[0].t);
    const float n0 = s.in.pos_n, e0 = s.in.pos_e; s.run(1.0f);
    CHECK(hypotf(s.in.pos_n - n0, s.in.pos_e - e0) < 0.5f && s.m.state() == MissionState::LOITER, "it does not leave before the last wait is over");
    s.run(1.5f); CHECK(s.m.state() == MissionState::WAYPOINT && s.m.index() == 5, "then the next waypoint (%s, command %d)", state_name(s.m.state()), s.m.index());
}

static void t_land_wait_takeoff() {
    printf("[mission] land, wait with the motors off, take off again\n");
    MSim s; s.in.z = 10; s.in.airborne = true;
    s.add(land(30, 0)); s.add(delay(20)); s.add(takeoff(12)); s.add(wp(30, 30, 12));
    CHECK(s.start(), "start");
    bool held_at_disarm = false; for (float e = 0; e < 90.0f && !s.out.want_disarm; e += s.dt) s.step(); held_at_disarm = s.out.want_disarm && s.m.holds_home();
    CHECK(held_at_disarm, "home is held from the moment it asks for the motors to stop (no cycle where it is neither armed nor held)");
    CHECK(s.until(MissionState::GROUND_WAIT, 90.0f), "lands at the point and waits (%s)", s.order.c_str());
    const float t_land = s.t;
    CHECK(s.disarms == 1 && !s.in.armed && hypotf(s.in.pos_n - 30, s.in.pos_e) < 2.0f && s.m.holds_home(), "touched down %.1f m from the point; the motors were turned off once; home is kept", hypotf(s.in.pos_n - 30, s.in.pos_e));
    s.run(5.0f); CHECK(s.m.wait_s() >= 14 && s.m.wait_s() <= 16 && s.warn_first < 0, "5 s into the wait: %u s left, no warning yet", (unsigned)s.m.wait_s());
    CHECK(s.until(MissionState::TAKEOFF, 30.0f) && s.arms == 1, "then armed again by itself (%s)", s.order.c_str());
    CHECK(s.warn_first > 0 && !s.arm_t.empty() && fabsf(s.arm_t[0] - s.warn_first - 10.0f) < 0.2f && s.warn_last >= s.arm_t[0] - 0.02f, "the warning sounds for the last 10 s before the motors start (%.1f s)", s.arm_t.empty() ? 0 : s.arm_t[0] - s.warn_first);
    CHECK(s.want_arm_steps > 0 && s.arm_warned, "the warning sounds while it asks to arm, too");
    CHECK(!s.arm_t.empty() && fabsf(s.arm_t[0] - t_land - 20.0f) < 2.5f, "after the 20 s of the wait (%.1f s after landing)", s.arm_t.empty() ? 0 : s.arm_t[0] - t_land);
    CHECK(s.until(MissionState::DONE, 60.0f) && hypotf(s.in.pos_n - 30, s.in.pos_e - 30) < 2.0f && fabsf(s.in.z - 12) < 1.0f, "flew on to the next point (%s)", s.order.c_str());
    CHECK(!s.m.holds_home(), "home is no longer held once the mission is over");
    // the pilot turns the ARM switch off during the wait: it never takes off by itself
    MSim q; q.in.z = 10; q.in.airborne = true; q.add(land(10, 0)); q.add(delay(10)); q.add(takeoff(10)); q.add(wp(20, 0, 10)); q.start();
    q.until(MissionState::GROUND_WAIT, 60.0f); q.in.arm_switch_on = false; q.run(30.0f);
    CHECK(!q.in.armed && q.arms == 0 && q.in.z == 0.0f && !q.out.want_arm && (q.m.state() == MissionState::DONE || q.m.state() == MissionState::IDLE), "ARM switch off while it waits: no take-off (%s)", q.order.c_str());
    // LAND not followed by TAKEOFF: the mission ends on the ground (ArduCopter verify_land: disarm and stop)
    MSim r; r.in.z = 8; r.in.airborne = true; r.add(land(5, 0)); r.add(wp(30, 0, 10)); r.start(); r.run(60.0f);
    CHECK(!r.in.armed && r.m.state() == MissionState::DONE && r.arms == 0 && r.order == "LANDING DONE ", "a LAND without a TAKEOFF after it: down, motors off, the mission ends (%s)", r.order.c_str());
    r.in.armed = true; r.run(2.0f); CHECK(r.in.armed && !r.out.want_disarm, "the pilot arms again afterwards: the finished mission does not turn the motors off again");
    r.m.stop(); r.step(); CHECK(r.m.state() == MissionState::IDLE && !r.out.active, "stop() after the end: IDLE");
}

static void t_rtl_end() {
    printf("[mission] RTL\n");
    MSim s; s.in.z = 10; s.in.airborne = true; s.in.pos_n = 50; s.add(wp(50, 20, 10)); s.add(rtl()); s.start();
    CHECK(s.until(MissionState::RTL, 30.0f), "after the waypoint: RTL (%s)", s.order.c_str());
    s.step(); CHECK(s.out.want_rtl, "it asks for the return home");
    CHECK(s.until(MissionState::DONE, 120.0f) && s.in.rtl_done, "done once the rescue has landed");
}

static void t_speed_cmd() {
    printf("[mission] DO_CHANGE_SPEED\n");
    MSim s; s.in.z = 10; s.in.airborne = true; s.add(speed(3.0f)); s.add(wp(60, 0, 10)); s.start(); s.until(MissionState::DONE, 60.0f);
    CHECK(fabsf(s.max_ff - 3.0f) < 0.01f && s.max_speed < 4.0f && hypotf(s.in.pos_n - 60, s.in.pos_e) < 2.0f, "the leg flown at 3 m/s (the target %.2f, the craft %.2f at most)", s.max_ff, s.max_speed);
}

static void t_lost_position() {
    printf("[mission] the position lost on a leg\n");
    MSim s; s.in.z = 10; s.in.airborne = true; s.add(wp(80, 0, 10)); s.start(); s.run(4.0f); s.in.pos_valid = false; s.run(0.5f);
    CHECK(s.m.state() == MissionState::WAYPOINT, "half a second without a position: it carries on (a GPS glitch)");
    s.run(1.0f); s.step();
    CHECK(s.m.state() == MissionState::FAILED && s.out.want_land && !s.out.horizontal, "over a second: FAILED, it asks to land where it is");
}

static void t_details() {
    printf("[mission] details: arming, arrival, landing speed, the way left, the stopping point, the leash, the nose\n");
    { MSim s; s.in.armed = false; s.in.z = 10; s.in.airborne = true; s.add(wp(10, 0, 10)); CHECK(!s.start() && s.m.state() == MissionState::REFUSED, "not armed: refused"); }
    { MSim s; s.in.z = 10; s.in.airborne = true; s.add(wp(60, 0, 10)); s.start(); s.run(3.0f); s.in.armed = false; s.step();
      CHECK(s.m.state() == MissionState::DONE && !s.out.active, "disarmed by the pilot on a leg: over"); }
    { MSim s; s.in.z = 10; s.in.airborne = true; s.add(wp(0.5f, 0, 30)); s.add(wp(20, 0, 30)); s.start();
      for (float e = 0; e < 40 && s.m.index() == 0; e += s.dt) s.step();
      CHECK(s.m.index() == 1 && s.in.z > 29.0f, "a point straight above: reached only at its height (%.1f m when it went on)", s.in.z); }
    { MSim s; s.in.z = 10; s.in.airborne = true; s.in.vel_n = 10.0f; s.add(wp(0, 0, 10)); s.add(loiter(1)); s.start(); s.run(0.5f);
      CHECK(s.m.state() == MissionState::WAYPOINT && s.m.index() == 0, "over the point but flying at 10 m/s: not reached yet - the target first runs to the stopping point and back (ArduPilot: the leg must be finished)"); }
    { MSim s; s.in.z = 20; s.in.airborne = true; s.add(item(MissionCmd::LAND, 0, 0, 0, 0, 0, false, false)); s.start();
      for (float e = 0; e < 60 && s.disarms == 0; e += s.dt) s.step();
      CHECK(s.disarms == 1 && s.t < 32.0f, "down from 20 m: fast high up, the land speed only below landing_alt (%.0f s)", s.t); }
    { MSim s; s.in.z = 10; s.in.airborne = true; s.add(wp(40, 0, 10)); s.add(wp(40, 30, 10)); s.start(); s.step();
      CHECK(fabsf(s.m.left() - 70.0f) < 2.0f, "the way left at the start: both legs (%.1f m of 70)", s.m.left()); }
    { MSim s; s.in.z = 10; s.in.airborne = true; s.in.vel_n = 8.0f; s.add(wp(0, 30, 10)); s.start(); s.step();
      CHECK(s.out.tgt_n > 12.0f, "flying north at 8 m/s, a point to the east: the leg starts where the craft can stop (%.1f m north), it is not yanked back", s.out.tgt_n); }
    { MSim s; s.vcap = 2.0f; s.in.z = 10; s.in.airborne = true; s.add(wp(60, 0, 10)); s.start(); s.run(20.0f);
      CHECK(s.max_lead < 6.0f, "a craft that cannot keep up (2 m/s): the target waits within the leash (%.1f m ahead at most)", s.max_lead); }
    { MSim s; s.in.z = 10; s.in.airborne = true; s.add(wp(0, 80, 10)); s.start(); s.run(10.0f);
      CHECK(fabsf(s.in.yaw_deg - 90.0f) < 15.0f, "on a leg to the east the nose turns east (%.0f deg)", s.in.yaw_deg); }
}

int main() {
    setvbuf(stdout, NULL, _IONBF, 0);
    t_mav_map(); t_refuse(); t_takeoff_wp(); t_do_order(); t_land_wait_takeoff(); t_rtl_end(); t_speed_cmd(); t_lost_position(); t_details();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
