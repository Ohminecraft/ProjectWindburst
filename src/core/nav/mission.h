/*
 * This file is part of Windburst Project
 */

#pragma once
#include "../rescue.h"
#include "nav_math.h"

// ============================================================================
// The waypoint mission (0.37) - ArduCopter's AUTO mode, read from its sources (ArduCopter/mode_auto.cpp, AP_Mission.cpp) and
// written anew, smaller. A plan is a list of commands with the MAV_CMD meanings, so a ground station's mission maps onto it:
//
//   TAKEOFF      (22)  straight up where the craft stands to `alt` (Altitude Hold takes off by itself: `takeoff`)
//   WAYPOINT     (16)  to the point at `alt`; there, hold p seconds (MAV_CMD_NAV_WAYPOINT param1)
//   LOITER_TIME  (19)  hold a point (lat = lon = 0: the one it is at) p seconds, counted from when it is there
//   LAND         (21)  (to the point at the present height, then) down; on the ground the motors are turned OFF (want_disarm)
//   DELAY        (93)  p seconds; on the ground with the motors off it is the wait at a delivery point
//   RTL          (20)  the GPS rescue flies home and lands (want_rtl); the mission is done when it has
//   SERVO       (183)  DO_SET_SERVO: servo `arg` (1..4) to p us
//   SPEED       (178)  DO_CHANGE_SPEED: p dm/s for the legs after it
//
// NAV commands (the first six) run one after the other; the DO commands that follow a NAV command run when that NAV command
// STARTS, one per step (AP_Mission::update / advance_current_do_cmd), so "WAYPOINT P, LOITER_TIME 3 s, SERVO" moves the servo
// when the craft is at P and then waits 3 s. DO commands before the first NAV command run at the start.
//
// Where it differs from ArduCopter, on purpose:
//   * LAND followed (after DELAY / DO commands) by TAKEOFF: ArduCopter (MIS_OPTIONS CONTINUE_AFTER_LAND) keeps the motors at
//     ground idle; here they are turned off - people walk up to the craft - and the TAKEOFF arms them again (want_arm), after a
//     warning (`warn`) during the last kWarnS of the DELAY. The pilot's ARM switch turned off while it waits ends the mission.
//     From that landing on the home point and the height origin are kept (holds_home): the second arming must not move them.
//   * every waypoint is flown to a stop (no corner blending): the craft is exactly over each point.
//   * a lost position for more than kLostS in the air: FAILED, `want_land` (the caller lands where it is).
// The legs are flown like the rescue's path (rescue.h): a target point runs along the leg from the craft's stopping point,
// speeding up and braking with accel_mps2, kept within a leash of the craft; the speeds and limits are the rescue's (RescueConfig).
// This block says what is wanted; FlightCore flies it with the position controller and Altitude Hold.
// ============================================================================
namespace fc {

constexpr int kMissionMax = 32;
// the order of the schema's enums mission_cmd / mission_state
enum class MissionCmd : uint8_t { NONE, TAKEOFF, WAYPOINT, LOITER_TIME, LAND, DELAY, RTL, SERVO, SPEED };
enum class MissionState : uint8_t { IDLE, TAKEOFF, WAYPOINT, LOITER, LANDING, GROUND_WAIT, ARMING, RTL, DONE, REFUSED, FAILED };

inline uint16_t mav_cmd_of(MissionCmd c) {
    switch (c) {
    case MissionCmd::TAKEOFF: return 22; case MissionCmd::WAYPOINT: return 16; case MissionCmd::LOITER_TIME: return 19; case MissionCmd::LAND: return 21;
    case MissionCmd::DELAY: return 93; case MissionCmd::RTL: return 20; case MissionCmd::SERVO: return 183; case MissionCmd::SPEED: return 178;
    default: return 0;
    }
}
inline MissionCmd mission_cmd_of_mav(uint16_t m) {
    switch (m) {
    case 22: return MissionCmd::TAKEOFF; case 16: return MissionCmd::WAYPOINT; case 19: return MissionCmd::LOITER_TIME; case 21: return MissionCmd::LAND;
    case 93: return MissionCmd::DELAY; case 20: return MissionCmd::RTL; case 183: return MissionCmd::SERVO; case 178: return MissionCmd::SPEED;
    default: return MissionCmd::NONE;
    }
}
inline bool mission_is_nav(MissionCmd c) { return c >= MissionCmd::TAKEOFF && c <= MissionCmd::RTL; }

// one command, positions already in metres from home (north, east) and heights above home
struct MissionItem { MissionCmd cmd = MissionCmd::NONE; uint8_t arg = 0; float n = 0, e = 0; bool has_pos = false; float alt_m = 0; bool has_alt = false; uint16_t p = 0; };
struct MissionPlan { MissionItem item[kMissionMax]; uint8_t count = 0; };

struct MissionInput {
    bool armed = false, airborne = false, pos_valid = false, alt_valid = false;
    bool landed = false;           // Altitude Hold has put the craft down (LANDED_IDLE)
    bool arm_switch_on = true;     // the pilot's ARM switch (true while the link is lost: the mission carries on)
    bool rtl_done = false;         // the rescue asked for by want_rtl has landed
    float pos_n = 0, pos_e = 0, vel_n = 0, vel_e = 0, z = 0, vz = 0, yaw_deg = 0;
};
struct MissionOutput {
    bool active = false, horizontal = false;
    float tgt_n = 0, tgt_e = 0, vel_n = 0, vel_e = 0, climb_mps = 0;
    bool takeoff = false;          // Altitude Hold: take off by itself (the craft stands on the ground)
    bool want_arm = false, want_disarm = false, warn = false, want_rtl = false, want_land = false;
    bool yaw_control = false; float yaw_rate_dps = 0;
    int8_t servo = -1; uint16_t servo_us = 0;      // DO_SET_SERVO this step: servo 1..4 (-1: none)
};

class MissionRunner {
public:
    RescueConfig cfg;              // speeds, acceleration, leash, landing: the rescue's
    static constexpr float kArriveZ = 1.0f;        // m: a waypoint's height counts as reached this close
    static constexpr float kTakeoffZ = 0.5f;       // m: the take-off height counts as reached this close
    static constexpr float kWarnS = 10.0f;         // s: the warning before the motors are armed again
    static constexpr float kLostS = 1.0f;          // s: without a position this long in the air = FAILED
    static constexpr float kZp = 1.0f;             // 1/s: a height off the leg's is flown out at 1 m/s per metre

    // false (state REFUSED): an empty plan, not armed, no position / height, or on the ground with a first NAV command that is
    // not TAKEOFF (ArduCopter ModeAuto::init "Missing Takeoff Cmd")
    bool start(const MissionPlan& plan, const MissionInput& in) {
        plan_ = plan; if (plan_.count > kMissionMax) plan_.count = kMissionMax;
        landed_mid_ = false; lost_t_ = 0; speed_ = cfg.ground_speed_mps;
        const int first = next_nav(0);
        if (first < 0 || !in.armed || !in.pos_valid || !in.alt_valid || (!in.airborne && plan_.item[first].cmd != MissionCmd::TAKEOFF)) { state_ = MissionState::REFUSED; return false; }
        last_n_ = in.pos_n; last_e_ = in.pos_e; last_z_ = in.z;
        do_next_ = 0;                                   // (the DO commands before the first NAV command run first)
        begin(first, in);
        return true;
    }
    void stop() { state_ = MissionState::IDLE; landed_mid_ = false; }

    MissionOutput update(const MissionInput& in, float dt) {
        MissionOutput o;
        if (state_ == MissionState::IDLE || state_ == MissionState::REFUSED) return o;
        if (state_ == MissionState::FAILED) { o.active = true; o.want_land = true; return o; }
        if (state_ == MissionState::DONE) { if (!in.armed) done_off_ = true; done_out(in, o); return o; }
        // the pilot's ARM switch off while the mission keeps the motors off: no take-off by itself, the mission ends
        if (!in.armed && !in.arm_switch_on && landed_mid_) { finish(); return o; }
        // disarmed by anyone but the mission while it flies: over
        if (!in.armed && (state_ == MissionState::WAYPOINT || state_ == MissionState::LOITER || (state_ == MissionState::TAKEOFF && in.airborne))) { finish(); return o; }
        // the position (and the height) in the air
        const bool needs = in.armed && in.airborne && state_ != MissionState::RTL;
        if (needs && !(in.pos_valid && in.alt_valid)) {
            lost_t_ += dt;
            if (lost_t_ > kLostS) { state_ = MissionState::FAILED; landed_mid_ = false; o.active = true; o.want_land = true; return o; }
            o.active = true; return o;                  // a glitch: levelled, height held, for a moment
        }
        lost_t_ = 0;
        run_do(o);
        o.active = true;
        switch (state_) {
        case MissionState::TAKEOFF:
            if (!in.armed) { if (landed_mid_) state_ = MissionState::ARMING; else { finish(); return MissionOutput(); } break; }
            o.takeoff = !in.airborne; o.horizontal = in.airborne;
            hold(o, last_n_, last_e_);
            { const float left = tz_ - in.z;
              if (left > kTakeoffZ) o.climb_mps = clampf(left, 0.5f, cfg.ascend_mps); else { last_z_ = tz_; advance(in); } }
            break;
        case MissionState::ARMING:
            o.warn = true; o.want_arm = true;                 // (the ARM switch turned off has ended the mission above)
            if (in.armed) state_ = MissionState::TAKEOFF;
            break;
        case MissionState::WAYPOINT: case MissionState::LOITER:
            fly(in, dt, o);
            if (!reached_) reached_ = done_ && dist_xy(in, tn_, te_) < cfg.accept_m && fabsf(tz_ - in.z) < kArriveZ;
            if (reached_) { wait_ -= dt; if (wait_ <= 0.0f) { last_n_ = tn_; last_e_ = te_; last_z_ = tz_; advance(in); } }
            break;
        case MissionState::LANDING:
            if (in.armed) {
                if (!done_) fly(in, dt, o);
                else {
                    o.horizontal = true; hold(o, tn_, te_);
                    o.climb_mps = in.z > cfg.landing_alt_m ? -sink_rate(in.z, cfg.landing_alt_m) : -cfg.land_speed_mps;
                    o.want_disarm = in.landed;
                    if (in.landed && takeoff_ahead(idx_ + 1)) landed_mid_ = true;      // (home held before the motors stop: never a cycle disarmed and not held)
                }
            } else {                                    // down, motors off: on only when a TAKEOFF comes (ArduCopter verify_land)
                if (takeoff_ahead(idx_ + 1)) { landed_mid_ = true; last_n_ = tn_; last_e_ = te_; advance(in); }
                else { finish(); return MissionOutput(); }
            }
            break;
        case MissionState::GROUND_WAIT:
            wait_ -= dt;
            o.warn = takeoff_ahead(idx_ + 1) && wait_ <= kWarnS;
            if (wait_ <= 0.0f) advance(in);
            break;
        case MissionState::RTL:
            o.want_rtl = true;
            if (in.rtl_done) { finish(); return MissionOutput(); }
            break;
        default: break;
        }
        left_ = (has_target() ? dist_xy(in, tn_, te_) : 0.0f) + rest_;
        dist_ = has_target() ? dist_xy(in, tn_, te_) : 0.0f;
        return o;
    }

    MissionState state() const { return state_; }
    uint8_t index() const { return (uint8_t)idx_; }
    uint8_t count() const { return plan_.count; }
    float dist() const { return dist_; }            // m from the craft to the point of the present command (0: none)
    float left() const { return left_; }            // m the mission still flies (to its last point; RTL: to home)
    uint8_t wait_s() const { return (state_ == MissionState::GROUND_WAIT || ((state_ == MissionState::WAYPOINT || state_ == MissionState::LOITER) && reached_)) && wait_ > 0.0f ? (uint8_t)minf(ceilf(wait_), 255.0f) : 0; }
    bool holds_home() const { return landed_mid_; }       // home and the height origin must not move (a mid-mission landing; every end clears it)
    bool running() const { return state_ != MissionState::IDLE && state_ != MissionState::DONE && state_ != MissionState::REFUSED && state_ != MissionState::FAILED; }
    const MissionPlan& plan() const { return plan_; }

private:
    int next_nav(int from) const { for (int i = from; i < plan_.count; i++) if (mission_is_nav(plan_.item[i].cmd)) return i; return -1; }
    // a TAKEOFF comes next - after waits and DO commands (LAND, DELAY, TAKEOFF: the delivery stop)
    bool takeoff_ahead(int from) const {
        for (int i = from; i < plan_.count; i++) { const MissionCmd c = plan_.item[i].cmd; if (c == MissionCmd::TAKEOFF) return true; if (c != MissionCmd::DELAY && mission_is_nav(c)) return false; }
        return false;
    }
    bool has_target() const { return state_ == MissionState::WAYPOINT || state_ == MissionState::LOITER || state_ == MissionState::LANDING; }
    void finish() { state_ = MissionState::DONE; landed_mid_ = false; done_off_ = false; dist_ = left_ = 0; }
    void advance(const MissionInput& in) { const int k = next_nav(idx_ + 1); if (k < 0) { finish(); return; } begin(k, in); }

    // NAV command k starts; the DO commands after it are queued (they run one per step)
    void begin(int k, const MissionInput& in) {
        idx_ = k; do_end_ = next_nav(k + 1); if (do_end_ < 0) do_end_ = plan_.count;
        if (do_next_ > k || do_next_ < 0) do_next_ = k + 1;
        const MissionItem& it = plan_.item[k];
        reached_ = false; wait_ = (float)it.p; done_ = true; v_ = 0; ramp_yaw_ = 0;
        tn_ = last_n_; te_ = last_e_; tz_ = last_z_;
        switch (it.cmd) {
        case MissionCmd::TAKEOFF:
            tz_ = it.alt_m; last_n_ = in.pos_n; last_e_ = in.pos_e;
            state_ = in.armed ? MissionState::TAKEOFF : MissionState::ARMING;
            break;
        case MissionCmd::WAYPOINT:
            leg(in, it.n, it.e, it.has_alt ? it.alt_m : in.z); state_ = MissionState::WAYPOINT;
            break;
        case MissionCmd::LOITER_TIME:
            // no point given: the one it was sent to last, already reached (ArduCopter do_loiter_unlimited keeps wp_nav's destination)
            if (it.has_pos || it.has_alt) leg(in, it.has_pos ? it.n : last_n_, it.has_pos ? it.e : last_e_, it.has_alt ? it.alt_m : last_z_);
            else reached_ = true;
            state_ = MissionState::LOITER;
            break;
        case MissionCmd::LAND:
            if (it.has_pos) leg(in, it.n, it.e, in.z); else { tn_ = in.pos_n; te_ = in.pos_e; tz_ = in.z; done_ = true; }
            state_ = MissionState::LANDING;
            break;
        case MissionCmd::DELAY:
            if (!in.armed && !in.airborne) state_ = MissionState::GROUND_WAIT;
            else { leg(in, last_n_, last_e_, last_z_); state_ = MissionState::LOITER; }
            break;
        case MissionCmd::RTL: state_ = MissionState::RTL; break;
        default: break;
        }
        // the way the rest of the plan flies (a point to the next, RTL: to home)
        rest_ = 0; float pn = has_target() ? tn_ : in.pos_n, pe = has_target() ? te_ : in.pos_e;
        for (int j = k + 1; j < plan_.count; j++) {
            const MissionItem& q = plan_.item[j];
            if (q.cmd == MissionCmd::RTL) { rest_ += sqrtf(pn * pn + pe * pe); break; }
            if ((q.cmd == MissionCmd::WAYPOINT || q.cmd == MissionCmd::LAND || q.cmd == MissionCmd::LOITER_TIME) && q.has_pos) { rest_ += sqrtf((q.n - pn) * (q.n - pn) + (q.e - pe) * (q.e - pe)); pn = q.n; pe = q.e; }
        }
    }
    // one DO command per step (AP_Mission: one active do command)
    void run_do(MissionOutput& o) {
        while (do_next_ < do_end_ && do_next_ < plan_.count && !is_do(plan_.item[do_next_].cmd)) do_next_++;
        if (do_next_ >= do_end_ || do_next_ >= plan_.count) return;
        const MissionItem& it = plan_.item[do_next_++];
        if (it.cmd == MissionCmd::SERVO && it.arg >= 1 && it.arg <= 4) { o.servo = (int8_t)it.arg; o.servo_us = it.p; }
        else if (it.cmd == MissionCmd::SPEED && it.p > 0) speed_ = (float)it.p * 0.1f;
    }
    static bool is_do(MissionCmd c) { return c == MissionCmd::SERVO || c == MissionCmd::SPEED; }

    // a leg from where the craft can stop (ArduPilot: the origin is the stopping point) to (n, e) at height z
    void leg(const MissionInput& in, float n, float e, float z) {
        float sn = in.pos_n, se = in.pos_e; const float v = sqrtf(in.vel_n * in.vel_n + in.vel_e * in.vel_e);
        if (v > 0.1f && cfg.accel_mps2 > 0.0f) { const float d = v / (2.0f * cfg.accel_mps2); sn += in.vel_n * d; se += in.vel_e * d; }
        on_ = sn; oe_ = se; oz_ = in.z; tn_ = n; te_ = e; tz_ = z;
        const float dn = n - sn, de = e - se; len_ = sqrtf(dn * dn + de * de);
        if (len_ > 0.05f) { dir_n_ = dn / len_; dir_e_ = de / len_; done_ = false; } else { len_ = 0; done_ = true; }
        s_ = 0; v_ = 0;
    }
    // the target point along the leg (rescue.h follow, one leg that ends in a stop)
    WB_FAST void fly(const MissionInput& in, float dt, MissionOutput& o) {
        const float a = maxf(cfg.accel_mps2, 0.1f);
        float want_z = tz_, ff_z = 0.0f;
        if (!done_) {
            float vmax = minf(speed_, sqrtf(2.0f * a * maxf(len_ - s_, 0.0f)));
            const float tn = on_ + dir_n_ * s_, te = oe_ + dir_e_ * s_, lead = sqrtf((tn - in.pos_n) * (tn - in.pos_n) + (te - in.pos_e) * (te - in.pos_e));
            const float along = maxf(in.vel_n * dir_n_ + in.vel_e * dir_e_, 0.0f);
            vmax = minf(vmax, along + sqrtf(2.0f * a * maxf(cfg.leash_m - lead, 0.0f)));
            v_ = v_ < vmax ? minf(v_ + a * dt, vmax) : maxf(v_ - 2.0f * a * dt, vmax);
            s_ += v_ * dt;
            if (s_ >= len_ || (v_ <= 0.0f && len_ - s_ < 0.05f)) { s_ = len_; done_ = true; v_ = 0.0f; }
            const float f = len_ > 0.0f ? s_ / len_ : 1.0f;
            want_z = oz_ + (tz_ - oz_) * f; ff_z = len_ > 0.0f ? (tz_ - oz_) / len_ * v_ : 0.0f;
        }
        o.horizontal = true;
        o.tgt_n = done_ ? tn_ : on_ + dir_n_ * s_; o.tgt_e = done_ ? te_ : oe_ + dir_e_ * s_;
        o.vel_n = done_ ? 0.0f : dir_n_ * v_; o.vel_e = done_ ? 0.0f : dir_e_ * v_;
        o.climb_mps = clampf(ff_z + kZp * (want_z - in.z), -sink_rate(in.z, 0.0f), cfg.ascend_mps);
        if (!done_ && v_ > 1.0f) {                     // the nose along the leg (PX4 MPC_YAW_MODE 0, as the rescue)
            o.yaw_control = true; ramp_yaw_ = minf(ramp_yaw_ + dt, 1.0f);
            const float err = Rescue::wrap180(in.yaw_deg - atan2f(dir_e_, dir_n_) * kRad2Deg);
            o.yaw_rate_dps = clampf(-err * cfg.yaw_p * ramp_yaw_, -cfg.max_yaw_rate_dps, cfg.max_yaw_rate_dps);
        }
    }
    // the plan is over: in the air it holds the last point (ArduCopter exit_mission -> loiter), on the ground the motors stop
    void done_out(const MissionInput& in, MissionOutput& o) const {
        if (!in.armed) return;
        o.active = true;
        if (!in.airborne) { o.want_disarm = !done_off_; o.active = !done_off_; return; }      // (once: an arming after that is the pilot's)
        o.horizontal = in.pos_valid; hold(o, last_n_, last_e_);
        o.climb_mps = clampf(kZp * (last_z_ - in.z), -cfg.descend_mps, cfg.ascend_mps);
    }
    float sink_rate(float z, float to_z) const {      // the rescue's: faster high up, never faster than what is left
        const float r = cfg.descend_mps * (0.6f + 2.4f * clampf((z - 10.0f) / 40.0f, 0.0f, 1.0f));
        return clampf(z - to_z, 0.5f, maxf(r, 0.5f));
    }
    static void hold(MissionOutput& o, float n, float e) { o.tgt_n = n; o.tgt_e = e; o.vel_n = o.vel_e = 0.0f; }
    static float dist_xy(const MissionInput& in, float n, float e) { return sqrtf((n - in.pos_n) * (n - in.pos_n) + (e - in.pos_e) * (e - in.pos_e)); }

    MissionPlan plan_; MissionState state_ = MissionState::IDLE;
    int idx_ = 0, do_next_ = 0, do_end_ = 0;
    bool landed_mid_ = false, reached_ = false, done_ = true, done_off_ = false;
    float speed_ = 7.5f, wait_ = 0, lost_t_ = 0, dist_ = 0, left_ = 0, rest_ = 0, ramp_yaw_ = 0;
    float last_n_ = 0, last_e_ = 0, last_z_ = 0;                 // the point the craft was last sent to (holds)
    float on_ = 0, oe_ = 0, oz_ = 0, tn_ = 0, te_ = 0, tz_ = 0, len_ = 0, s_ = 0, v_ = 0, dir_n_ = 1, dir_e_ = 0;
};

}  // namespace fc
