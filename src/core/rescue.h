#pragma once
#include "types.h"
#include "nav/breadcrumbs.h"

// ============================================================================
// GPS Rescue - since 0.36 the way ArduPilot and PX4 do it (read from their sources, written anew). Three kinds (nav.rescue_mode):
//
// RTL (PX4 navigator RTL / rtl_direct, ArduCopter mode_rtl):
//   CLIMB      straight up to the return height, holding the point where the craft can stop (ArduPilot: the origin of the
//              path is the stopping point, not where the craft happens to be, so a craft flying fast is not yanked back)
//                CONE (default; PX4 RTL::computeReturnAltitude): the return height, rescue_return_alt - except within
//                     rescue_min_start_dist (RTL_MIN_DIST) of home, where it is only what the cone of half-angle rescue_cone
//                     (RTL_CONE_ANG) over home needs: distance / tan(angle), but at least twice the acceptance radius
//                FIXED rescue_return_alt; MAX the highest point of this flight + margin; CURRENT the present height + margin
//              A craft that is higher already keeps its height (PX4: constrain(.., current, max); ArduPilot "ensure we do
//              not descend") - except FIXED, which is fixed.
//   FLY_HOME   a target point moves along the path at ground_speed, speeding up and slowing down with accel_mps2 so that it
//              stops at home (corners: slowly, looking ahead); the position controller follows it with its velocity as feed-forward
//              (AC_WPNav / PX4 position smoothing, simpler). The target waits while the craft is more than a leash behind it.
//              The nose turns along the way (PX4 MPC_YAW_MODE 0).
//   DESCENT    over home, down to descend_alt if higher (PX4 LOITER_DOWN, RTL_DESCEND_ALT)
//   LOITER     holds over home for loiter_s (PX4 LOITER_HOLD, RTL_LAND_DELAY; -1 = for good, never lands by itself)
//   LANDING    straight down over home, holding the position (PX4 LAND): descend rate above landing_alt, land_speed below
//              (ArduPilot LAND_ALT_LOW / LAND_SPD); the landing detection of Altitude Hold ends the flight (the caller disarms)
//
// SMART_RTL (ArduCopter mode_smart_rtl + AP_SmartRTL): the way this flight came, backwards (nav/breadcrumbs.h) - around the
//   trees and buildings it flew around, at the heights it flew there, no climb first. The last point is home 2 m up
//   (path_follow_run: "add 2m to the target alt"), then DESCENT / LOITER / LANDING as RTL. A path that could not be kept
//   (the store full for 15 s, or no position for 15 s) - or a start closer than rescue_min_start_dist - flies as RTL.
//
// LAND (ArduCopter mode_land): down where the craft is, holding that point (gps_run) - or, without a position or a heading,
//   only the descent (nogps_run: the craft levelled, the sticks of a pilot who is there move it). Needs only a height.
//
// This block only says WHAT is wanted - a climb rate, a target point with its velocity, a yaw rate; Altitude Hold
// (nav/alt_control.h), the position controller (nav/pos_control.h) and the rate loop fly it. Home is the origin of the horizontal
// estimate (set when the craft is armed with a fix).
//
// RTL / SMART_RTL need a position (GPS), a height, and a heading that does not come from the gyro alone (compass). Without one of
// them they do not start; when one is lost on the way, or the craft does not get higher / nearer although it should
// (Betaflight's sanity checks, kept), it FAILS - and the caller falls back to the plain failsafe landing (or hands the craft
// back to the pilot when the rescue came from a switch).
// ============================================================================
namespace fc {

enum RescueAltMode : uint8_t { RESCUE_ALT_MAX = 0, RESCUE_ALT_FIXED = 1, RESCUE_ALT_CURRENT = 2, RESCUE_ALT_CONE = 3 };
enum RescueKind : uint8_t { RESCUE_MODE_RTL = 0, RESCUE_MODE_SMART_RTL = 1, RESCUE_MODE_LAND = 2 };

struct RescueConfig {
    uint8_t mode = RESCUE_MODE_RTL;
    uint8_t alt_mode = RESCUE_ALT_CONE;
    float initial_climb_m = 10.0f;        // margin above the highest point of the flight (MAX) or the present height (CURRENT)
    float return_alt_m = 30.0f;           // FIXED; CONE: the return height (RTL_RETURN_ALT)
    float cone_deg = 45.0f;               // CONE: half-angle of the cone over home (RTL_CONE_ANG); 0 = always return_alt_m
    float min_start_dist_m = 15.0f;       // CONE: within this of home the cone sets the height (RTL_MIN_DIST); SMART: closer = RTL
    float accept_m = 2.0f;                // acceptance radius: this close to home counts as there (NAV_ACC_RAD)
    float ascend_mps = 5.0f, descend_mps = 1.5f;
    float ground_speed_mps = 7.5f;
    float accel_mps2 = 2.0f;              // along the path, and to work out the stopping point (WPNAV_ACCEL)
    float leash_m = 5.0f;                 // the target waits while the craft is further than this behind it
    float descend_alt_m = 10.0f;          // over home: down to this before the wait (RTL_DESCEND_ALT)
    float loiter_s = 5.0f;                // over home before landing (RTL_LAND_DELAY); < 0: never lands by itself
    float landing_alt_m = 4.0f;           // below this: land_speed (LAND_ALT_LOW)
    float land_speed_mps = 0.5f;          // LAND_SPD
    float yaw_p = 1.8f, max_yaw_rate_dps = 90.0f;
    uint8_t min_sats = 5;                 // fewer than this for lowsats_s: failed
    float stalled_s = 10.0f, flyaway_s = 20.0f, lowsats_s = 15.0f;
    float smart_timeout_s = 15.0f;        // SMARTRTL_TIMEOUT: the path is given up after this long without a position / room
};

// ROTATE is not used any more (the nose turns while flying); kept so that the numbers of the others stay what they were
enum class RescuePhase : uint8_t { IDLE, CLIMB, ROTATE, FLY_HOME, DESCENT, LANDING, FAILED, LOITER };
enum class RescueFail : uint8_t { NONE, NO_POSITION, NO_HEADING, NO_ALTITUDE, GPS_LOST, LOW_SATS, STALLED, FLYAWAY };

struct RescueInput {
    bool armed = false, airborne = false, request = false;
    float pos_n = 0, pos_e = 0, vel_n = 0, vel_e = 0;      // relative to home (m, m/s)
    float z = 0;                                           // height above the take-off point (m)
    float yaw_deg = 0;                                     // heading, 0 = north, clockwise
    bool pos_valid = false, alt_valid = false, heading_valid = false;
    uint8_t sats = 0;
};
struct RescueOutput {
    bool active = false;                                   // false: nothing below counts (idle or failed)
    float climb_mps = 0;                                   // for Altitude Hold
    bool horizontal = false;                               // fly to the target (LAND without a position: only the descent)
    float tgt_n = 0, tgt_e = 0;                            // where the craft should be now (m from home)
    float vel_n = 0, vel_e = 0;                            // the target's own velocity: the feed-forward (m/s)
    bool yaw_control = false; float yaw_rate_dps = 0;      // + = nose to the right
};

class Rescue {
public:
    static constexpr int kCrumbs = 128;
    RescueConfig cfg;

    bool available(const RescueInput& in) const { return cfg.mode == RESCUE_MODE_LAND ? in.alt_valid : in.pos_valid && in.alt_valid && in.heading_valid; }

    WB_FAST RescueOutput update(const RescueInput& in, float dt) {
        RescueOutput o;
        if (!in.armed) { phase_ = RescuePhase::IDLE; fail_ = RescueFail::NONE; max_z_ = 0; crumbs_.reset(); bad_s_ = 0; full_s_ = 0; left_ = 0; remaining_ = 0; return o; }
        if (!in.request) { idle(in, dt); return o; }
        if (phase_ == RescuePhase::FAILED) return o;
        const bool land = cfg.mode == RESCUE_MODE_LAND;
        if (phase_ == RescuePhase::IDLE) {                                   // start
            if (!land && !in.pos_valid) return failed(RescueFail::NO_POSITION);
            if (!land && !in.heading_valid) return failed(RescueFail::NO_HEADING);
            if (!in.alt_valid) return failed(RescueFail::NO_ALTITUDE);
            start(in);
        }
        // ---- what must not be lost on the way (LAND carries on without a position or a heading: only the descent)
        if (!in.alt_valid) return failed(RescueFail::NO_ALTITUDE);
        if (!land && !in.pos_valid) return failed(RescueFail::GPS_LOST);
        if (!land && !in.heading_valid) return failed(RescueFail::NO_HEADING);

        // ---- once a second: is it getting anywhere? (Betaflight's sanity checks)
        sec_ += dt; t_phase_ += dt;
        if (sec_ >= 1.0f) {
            sec_ -= 1.0f;
            if (!land) {
                lowsats_ = in.sats < cfg.min_sats ? lowsats_ + 1 : (lowsats_ > 0 ? lowsats_ - 1 : 0);
                if ((float)lowsats_ >= cfg.lowsats_s) return failed(RescueFail::LOW_SATS);
            }
            if (phase_ == RescuePhase::CLIMB) {                   // (against what was asked for in that second: the last metres are flown slowly)
                const float want = climb_1s_, got = in.z - z_1s_;
                stalled_ = (fabsf(want) > 0.3f && got / want < 0.5f) ? stalled_ + 1 : (stalled_ > 0 ? stalled_ - 1 : 0);
                if ((float)stalled_ >= cfg.stalled_s) return failed(RescueFail::STALLED);
            }
            const float left = left_from(in);
            if (phase_ == RescuePhase::FLY_HOME) {               // the way still to go must shrink by a fifth of the speed planned
                flyaway_ = (left_1s_ - left < 0.2f * speed_1s_) ? flyaway_ + 1 : (flyaway_ > 0 ? flyaway_ - 1 : 0);
                if ((float)flyaway_ >= cfg.flyaway_s) return failed(RescueFail::FLYAWAY);
            }
            z_1s_ = in.z; left_1s_ = left; speed_1s_ = phase_ == RescuePhase::FLY_HOME ? vplan_ : 0.0f;
        }

        o.active = true; o.horizontal = true;
        float want_z = in.z, ff_z = 0.0f; bool fly_z = false;
        switch (phase_) {
        case RescuePhase::CLIMB:
            hold(o, route_[0].n, route_[0].e);
            if (fabsf(return_z_ - in.z) > kArrive) {
                // at the configured rate, the last metres more slowly (1 m/s per metre left): it arrives instead of shooting past
                const float left = fabsf(return_z_ - in.z);
                o.climb_mps = return_z_ > in.z ? clampf(left, 0.5f, cfg.ascend_mps) : -clampf(left, 0.5f, cfg.descend_mps);
            } else next(RescuePhase::FLY_HOME);
            break;
        case RescuePhase::FLY_HOME:
            follow(in, dt, o, want_z, ff_z); fly_z = true;
            if (done_ && hypot2(in.pos_n, in.pos_e) < cfg.accept_m && fabsf(want_z - in.z) < kArrive) next(in.z > cfg.descend_alt_m + kArrive ? RescuePhase::DESCENT : RescuePhase::LOITER);
            break;
        case RescuePhase::DESCENT:
            hold(o, 0.0f, 0.0f);
            if (in.z > cfg.descend_alt_m + kArrive) o.climb_mps = -sink_rate(in.z, cfg.descend_alt_m);
            else next(RescuePhase::LOITER);
            break;
        case RescuePhase::LOITER:
            hold(o, 0.0f, 0.0f);
            if (cfg.loiter_s >= 0.0f && t_phase_ >= cfg.loiter_s) next(RescuePhase::LANDING);
            break;
        case RescuePhase::LANDING:
            hold(o, land ? route_[0].n : 0.0f, land ? route_[0].e : 0.0f);
            if (land && !(in.pos_valid && in.heading_valid)) o.horizontal = false;      // (nogps_run)
            o.climb_mps = in.z > cfg.landing_alt_m ? -sink_rate(in.z, cfg.landing_alt_m) : -cfg.land_speed_mps;
            break;
        default: break;
        }
        if (fly_z) o.climb_mps = clampf(ff_z + kZp * (want_z - in.z), -sink_rate(in.z, 0.0f), cfg.ascend_mps);
        climb_1s_ = o.climb_mps;
        remaining_ = phase_ == RescuePhase::CLIMB || phase_ == RescuePhase::FLY_HOME ? left_from(in) : 0.0f;
        // the nose along the way while it flies (PX4 MPC_YAW_MODE 0: towards the waypoint)
        if (phase_ == RescuePhase::FLY_HOME && v_ > 1.0f) {
            o.yaw_control = true;
            ramp_yaw_ = minf(ramp_yaw_ + dt, 1.0f);
            const float err = wrap180(in.yaw_deg - atan2f(dir_e_, dir_n_) * kRad2Deg);
            o.yaw_rate_dps = clampf(-err * cfg.yaw_p * ramp_yaw_, -cfg.max_yaw_rate_dps, cfg.max_yaw_rate_dps);
        }
        return o;
    }

    RescuePhase phase() const { return phase_; }
    RescueFail failure() const { return fail_; }
    float return_height() const { return return_z_; }
    float max_height() const { return max_z_; }
    bool smart() const { return smart_; }                 // this return flies the way the flight came
    int route_points() const { return route_n_; }
    const nav::Breadcrumbs<kCrumbs>& crumbs() const { return crumbs_; }
    float path_left() const { return left_; }             // along the path, from the target to home (m)
    float remaining() const { return remaining_; }        // the way the craft still has to go home (m; 0 once there, and in LAND)
    static constexpr float kArrive = 0.5f;                 // m: this close to a height counts as there
    static constexpr float kZp = 1.0f;                     // 1/s: a height off the path's is flown out at 1 m/s per metre
    static constexpr float kSmartLastUp = 2.0f;            // m: SmartRTL's last point is home this much up
    static float wrap180(float d) { while (d > 180.0f) d -= 360.0f; while (d < -180.0f) d += 360.0f; return d; }

    // CONE: PX4 RTL::computeReturnAltitude, home as the destination (height 0) - before the "never lower than now"
    float cone_height(float dist) const {
        if (!(cfg.cone_deg > 0.0f)) return cfg.return_alt_m;
        if (dist <= cfg.accept_m) return 2.0f * dist;
        float h = cfg.return_alt_m;
        if (dist <= cfg.min_start_dist_m) h = minf(dist / tanf(clampf(cfg.cone_deg, 1.0f, 89.0f) * kDeg2Rad), h);
        return maxf(h, 2.0f * cfg.accept_m);
    }

private:
    // not asked for: remember how high this flight went, and the way it came (from the arming on: the first point is home)
    void idle(const RescueInput& in, float dt) {
        phase_ = RescuePhase::IDLE; fail_ = RescueFail::NONE; remaining_ = 0;
        if (in.alt_valid && in.z > max_z_) max_z_ = in.z;
        if (in.pos_valid && in.alt_valid) {
            bad_s_ = 0;
            full_s_ = crumbs_.record(in.pos_n, in.pos_e, in.z) ? 0.0f : full_s_ + dt;
        } else bad_s_ += dt;
        if (full_s_ > cfg.smart_timeout_s || (in.airborne && bad_s_ > cfg.smart_timeout_s)) crumbs_.give_up();
    }
    void start(const RescueInput& in) {
        // ArduPilot build_path: the path starts where the craft can stop (its velocity, braked at accel_mps2)
        float sn = in.pos_n, se = in.pos_e;
        const float v = hypot2(in.vel_n, in.vel_e);
        if (in.pos_valid && v > 0.1f && cfg.accel_mps2 > 0.0f) { const float d = v / (2.0f * cfg.accel_mps2); sn += in.vel_n * d; se += in.vel_e * d; }
        if (!in.pos_valid) { sn = se = 0; }
        const float dist = hypot2(sn, se);
        route_[0].n = sn; route_[0].e = se; route_[0].z = in.z;
        route_n_ = 1; leg_ = 0; s_ = 0; v_ = vplan_ = 0; done_ = false; smart_ = false;
        ramp_yaw_ = 0; sec_ = 0; stalled_ = flyaway_ = lowsats_ = 0; z_1s_ = in.z; speed_1s_ = 0; climb_1s_ = 0; t_phase_ = 0;
        if (cfg.mode == RESCUE_MODE_LAND) { return_z_ = in.z; left_ = 0; left_1s_ = 0; phase_ = RescuePhase::LANDING; return; }

        if (cfg.alt_mode == RESCUE_ALT_FIXED) return_z_ = cfg.return_alt_m;
        else if (cfg.alt_mode == RESCUE_ALT_CURRENT) return_z_ = in.z + cfg.initial_climb_m;
        else if (cfg.alt_mode == RESCUE_ALT_MAX) return_z_ = maxf(max_z_, in.z) + cfg.initial_climb_m;
        else return_z_ = cone_height(dist);
        if (cfg.alt_mode != RESCUE_ALT_FIXED) return_z_ = maxf(return_z_, in.z);       // (never down first - but FIXED is fixed)
        route_[0].z = return_z_;
        smart_ = cfg.mode == RESCUE_MODE_SMART_RTL && crumbs_.ok() && crumbs_.size() >= 2 && dist >= cfg.min_start_dist_m;
        if (smart_) {
            // the way it came, newest point first, at the heights flown there; no climb; home 2 m up at the end
            route_[0].z = return_z_ = in.z;
            for (int i = crumbs_.size() - 1; i >= 1 && route_n_ < kCrumbs + 1; i--) route_[route_n_++] = crumbs_.at(i);
            route_[route_n_].n = 0; route_[route_n_].e = 0; route_[route_n_].z = maxf(crumbs_.at(0).z, 0.0f) + kSmartLastUp; route_n_++;
        } else {
            route_[1].n = 0; route_[1].e = 0; route_[1].z = return_z_; route_n_ = 2;
        }
        // the speed the target may have at each point of the route: a corner slowly, home 0. The change of velocity there
        // (2 v sin(turn / 2)) no more than one second of the path's acceleration
        for (int k = 0; k < route_n_; k++) {
            corner_v_[k] = k == 0 ? cfg.ground_speed_mps : 0.0f;
            if (k == 0 || k + 1 >= route_n_) continue;
            const float pn = route_[k].n - route_[k - 1].n, pe = route_[k].e - route_[k - 1].e, qn = route_[k + 1].n - route_[k].n, qe = route_[k + 1].e - route_[k].e;
            const float pl = hypot2(pn, pe), ql = hypot2(qn, qe);
            if (pl < 1e-3f || ql < 1e-3f) { corner_v_[k] = cfg.ground_speed_mps; continue; }
            const float c = clampf((pn * qn + pe * qe) / (pl * ql), -1.0f, 1.0f), half = sqrtf(0.5f * (1.0f - c));
            corner_v_[k] = half > 1e-3f ? minf(cfg.ground_speed_mps, cfg.accel_mps2 / (2.0f * half)) : cfg.ground_speed_mps;
        }
        set_leg();
        phase_ = smart_ ? RescuePhase::FLY_HOME : RescuePhase::CLIMB;
        left_1s_ = left_from(in);
    }
    // the leg leg_ .. leg_+1: its direction and length
    void set_leg() {
        const nav::Crumb& a = route_[leg_]; const nav::Crumb& b = route_[leg_ + 1];
        const float dn = b.n - a.n, de = b.e - a.e;
        len_ = sqrtf(dn * dn + de * de);
        if (len_ > 1e-3f) { dir_n_ = dn / len_; dir_e_ = de / len_; }
        rest_ = 0; for (int k = leg_ + 1; k + 1 < route_n_; k++) rest_ += hypot2(route_[k + 1].n - route_[k].n, route_[k + 1].e - route_[k].e);
        left_ = len_ - s_ + rest_;
    }
    // the target moves along the route
    WB_FAST void follow(const RescueInput& in, float dt, RescueOutput& o, float& want_z, float& ff_z) {
        if (!done_) {
            // as fast as the corners ahead and the stop at home still allow - looking as far ahead as it takes to brake from
            // ground_speed (the points of a SmartRTL path are only metres apart) - and never further than a leash ahead
            float vmax = cfg.ground_speed_mps, d = maxf(len_ - s_, 0.0f);
            const float horizon = cfg.ground_speed_mps * cfg.ground_speed_mps / (2.0f * maxf(cfg.accel_mps2, 0.1f));
            for (int k = leg_ + 1; k < route_n_; k++) {
                vmax = minf(vmax, sqrtf(corner_v_[k] * corner_v_[k] + 2.0f * cfg.accel_mps2 * d));
                if (k + 1 >= route_n_ || d > horizon) break;
                d += hypot2(route_[k + 1].n - route_[k].n, route_[k + 1].e - route_[k].e);
            }
            vplan_ = vmax;                                     // (what the sanity check holds the craft to: not the leash's stop)
            // the leash: the target runs ahead of the craft only by what it could still brake within leash_m (AC_WPNav's leash)
            const float tn = route_[leg_].n + dir_n_ * s_, te = route_[leg_].e + dir_e_ * s_, lead = hypot2(tn - in.pos_n, te - in.pos_e);
            const float along = maxf(in.vel_n * dir_n_ + in.vel_e * dir_e_, 0.0f);
            vmax = minf(vmax, along + sqrtf(2.0f * cfg.accel_mps2 * maxf(cfg.leash_m - lead, 0.0f)));
            v_ = v_ < vmax ? minf(v_ + cfg.accel_mps2 * dt, vmax) : maxf(v_ - 2.0f * cfg.accel_mps2 * dt, vmax);
            s_ += v_ * dt;
            while (s_ >= len_ && !done_) {
                if (leg_ + 2 < route_n_) { s_ -= len_; leg_++; set_leg(); }
                else { s_ = len_; done_ = true; v_ = vplan_ = 0.0f; }
            }
        }
        const float f = len_ > 1e-3f ? s_ / len_ : 1.0f;
        const nav::Crumb& a = route_[leg_]; const nav::Crumb& b = route_[leg_ + 1];
        o.tgt_n = done_ ? b.n : a.n + dir_n_ * s_; o.tgt_e = done_ ? b.e : a.e + dir_e_ * s_;
        o.vel_n = dir_n_ * v_; o.vel_e = dir_e_ * v_;
        want_z = a.z + (b.z - a.z) * f;
        ff_z = len_ > 1e-3f ? (b.z - a.z) / len_ * v_ : 0.0f;
        left_ = len_ - s_ + rest_;
    }
    // the way still to go from the craft: to the target, then along the route
    float left_from(const RescueInput& in) const {
        if (phase_ == RescuePhase::CLIMB) return hypot2(route_[0].n - in.pos_n, route_[0].e - in.pos_e) + left_;
        if (phase_ != RescuePhase::FLY_HOME) return hypot2(in.pos_n, in.pos_e);
        const float tn = done_ ? route_[leg_ + 1].n : route_[leg_].n + dir_n_ * s_, te = done_ ? route_[leg_ + 1].e : route_[leg_].e + dir_e_ * s_;
        return hypot2(tn - in.pos_n, te - in.pos_e) + left_;
    }
    // fast high up, slowly near the ground: x 0.6 below 10 m, x 3 from 50 m on; never faster than what is left down to `to_z`
    float sink_rate(float z, float to_z) const {
        const float r = cfg.descend_mps * (0.6f + 2.4f * clampf((z - 10.0f) / 40.0f, 0.0f, 1.0f));
        return clampf(z - to_z, 0.5f, maxf(r, 0.5f));
    }
    static void hold(RescueOutput& o, float n, float e) { o.tgt_n = n; o.tgt_e = e; o.vel_n = o.vel_e = 0.0f; }
    static float hypot2(float a, float b) { return sqrtf(a * a + b * b); }
    RescueOutput failed(RescueFail why) { phase_ = RescuePhase::FAILED; fail_ = why; remaining_ = 0; return RescueOutput(); }
    void next(RescuePhase p) { phase_ = p; t_phase_ = 0; }

    RescuePhase phase_ = RescuePhase::IDLE; RescueFail fail_ = RescueFail::NONE;
    float max_z_ = 0, return_z_ = 0, t_phase_ = 0, ramp_yaw_ = 0, sec_ = 0, z_1s_ = 0, left_1s_ = 0, speed_1s_ = 0, climb_1s_ = 0;
    int stalled_ = 0, flyaway_ = 0, lowsats_ = 0;
    nav::Breadcrumbs<kCrumbs> crumbs_; float bad_s_ = 0, full_s_ = 0;
    nav::Crumb route_[kCrumbs + 2]; float corner_v_[kCrumbs + 2]; int route_n_ = 0, leg_ = 0; bool smart_ = false, done_ = false;
    float s_ = 0, v_ = 0, vplan_ = 0, len_ = 0, dir_n_ = 1, dir_e_ = 0, rest_ = 0, left_ = 0, remaining_ = 0;
};

} // namespace fc
