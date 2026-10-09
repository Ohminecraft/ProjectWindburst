#pragma once
#include "types.h"

// ============================================================================
// GPS Rescue - bring the craft back to where it was armed and land it. The sequence is Betaflight's (flight/gps_rescue):
//
//   CLIMB      to the return height (the highest point of this flight + a margin, a fixed height, or the present height + a margin)
//   ROTATE     turn the nose towards home (the craft holds its place meanwhile); closer than descent_dist: straight to DESCENT
//   FLY_HOME   towards home, at ground_speed, slowing down over the last 2 x descent_dist
//   DESCENT    closer than descent_dist: still homing, and sinking - fast high up, slowly near the ground
//   LANDING    below landing_alt: slow descent until the craft stands (the caller's landing detection disarms)
//
// This block only says WHAT is wanted - a climb rate, a velocity over the ground, a yaw rate; Altitude Hold (althold.h), the
// position controller in velocity mode (poscontrol.h) and the rate loop fly it. Home is the origin of the horizontal estimate
// (set when the craft is armed with a fix).
//
// It needs: a position (GPS), a height, and a heading that does not come from the gyro alone (compass). Without one of them it
// does not start; when one is lost on the way, or the craft does not get nearer / higher although it should, it FAILS - and the
// caller falls back to the plain failsafe landing (or hands the craft back to the pilot when the rescue came from a switch).
// ============================================================================
namespace fc {

enum RescueAltMode : uint8_t { RESCUE_ALT_MAX = 0, RESCUE_ALT_FIXED = 1, RESCUE_ALT_CURRENT = 2 };

struct RescueConfig {
    uint8_t alt_mode = RESCUE_ALT_MAX;
    float initial_climb_m = 10.0f;        // margin above the highest point of the flight (MAX) or the present height (CURRENT)
    float return_alt_m = 30.0f;           // FIXED
    float ascend_mps = 5.0f, descend_mps = 1.5f;
    float ground_speed_mps = 7.5f;
    float descent_dist_m = 20.0f;         // closer than this the descent begins
    float min_start_dist_m = 15.0f;       // started closer than this: climb the margin (at least to 7.5 m), then come down
    float landing_alt_m = 4.0f;
    float yaw_p = 1.8f, max_yaw_rate_dps = 90.0f, rotate_ok_deg = 30.0f;
    uint8_t min_sats = 5;                 // fewer than this for lowsats_s: failed
    float stalled_s = 10.0f, flyaway_s = 20.0f, lowsats_s = 15.0f, rotate_timeout_s = 10.0f;
};

enum class RescuePhase : uint8_t { IDLE, CLIMB, ROTATE, FLY_HOME, DESCENT, LANDING, FAILED };
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
    float vel_n = 0, vel_e = 0;                            // velocity over the ground wanted (m/s)
    bool yaw_control = false; float yaw_rate_dps = 0;      // + = nose to the right
};

class Rescue {
public:
    RescueConfig cfg;

    static bool available(const RescueInput& in) { return in.pos_valid && in.alt_valid && in.heading_valid; }

    WB_FAST RescueOutput update(const RescueInput& in, float dt) {
        RescueOutput o;
        if (!in.armed) { phase_ = RescuePhase::IDLE; fail_ = RescueFail::NONE; max_z_ = 0; return o; }
        if (!in.request) {                                                   // not asked for: remember how high this flight went
            phase_ = RescuePhase::IDLE; fail_ = RescueFail::NONE;
            if (in.alt_valid && in.z > max_z_) max_z_ = in.z;
            return o;
        }
        if (phase_ == RescuePhase::FAILED) return o;
        const float dist = sqrtf(in.pos_n * in.pos_n + in.pos_e * in.pos_e);
        if (phase_ == RescuePhase::IDLE) {                                   // start
            if (!in.pos_valid) return failed(RescueFail::NO_POSITION);
            if (!in.heading_valid) return failed(RescueFail::NO_HEADING);
            if (!in.alt_valid) return failed(RescueFail::NO_ALTITUDE);
            if (dist < cfg.min_start_dist_m) return_z_ = fmaxf(7.5f, in.z + cfg.initial_climb_m);
            else if (cfg.alt_mode == RESCUE_ALT_FIXED) return_z_ = cfg.return_alt_m;
            else if (cfg.alt_mode == RESCUE_ALT_CURRENT) return_z_ = in.z + cfg.initial_climb_m;
            else return_z_ = fmaxf(max_z_, in.z) + cfg.initial_climb_m;
            from_below_ = in.z < return_z_;
            phase_ = RescuePhase::CLIMB; t_phase_ = 0; ramp_yaw_ = ramp_xy_ = 0; sec_ = 0; stalled_ = flyaway_ = lowsats_ = 0; z_1s_ = in.z; dist_1s_ = dist; speed_1s_ = 0; climb_1s_ = 0;
        }
        // ---- what must not be lost on the way
        if (!in.pos_valid) return failed(RescueFail::GPS_LOST);
        if (!in.alt_valid) return failed(RescueFail::NO_ALTITUDE);
        if (!in.heading_valid) return failed(RescueFail::NO_HEADING);

        // ---- once a second: is it getting anywhere? (Betaflight's sanity checks)
        sec_ += dt; t_phase_ += dt;
        if (sec_ >= 1.0f) {
            sec_ -= 1.0f;
            lowsats_ = in.sats < cfg.min_sats ? lowsats_ + 1 : (lowsats_ > 0 ? lowsats_ - 1 : 0);
            if ((float)lowsats_ >= cfg.lowsats_s) return failed(RescueFail::LOW_SATS);
            if (phase_ == RescuePhase::CLIMB) {                   // (against what was asked for in that second: the last metres are flown slowly)
                const float want = climb_1s_, got = in.z - z_1s_;
                stalled_ = (fabsf(want) > 0.3f && got / want < 0.5f) ? stalled_ + 1 : (stalled_ > 0 ? stalled_ - 1 : 0);
                if ((float)stalled_ >= cfg.stalled_s) return failed(RescueFail::STALLED);
            }
            if (phase_ == RescuePhase::FLY_HOME) {
                flyaway_ = (dist_1s_ - dist < 0.2f * speed_1s_) ? flyaway_ + 1 : (flyaway_ > 0 ? flyaway_ - 1 : 0);
                if ((float)flyaway_ >= cfg.flyaway_s) return failed(RescueFail::FLYAWAY);
            }
            z_1s_ = in.z; dist_1s_ = dist;
        }

        // ---- towards home: the heading that points there, and how far the nose is off it
        float err = 0.0f;
        if (dist > 0.2f) err = wrap180(in.yaw_deg - atan2f(-in.pos_e, -in.pos_n) * kRad2Deg);
        const float slowdown = fmaxf(2.0f * cfg.descent_dist_m, 5.0f);
        float speed = 0.0f;

        o.active = true;
        switch (phase_) {
        case RescuePhase::CLIMB:
            // towards the return height at the configured rate, the last metres more slowly (1 m/s per metre left): it arrives
            // there instead of shooting past
            if (from_below_ ? in.z < return_z_ - kArrive : in.z > return_z_ + kArrive) {
                const float left = fabsf(return_z_ - in.z);
                o.climb_mps = from_below_ ? clampf(left, 0.5f, cfg.ascend_mps) : -clampf(left, 0.5f, cfg.descend_mps);
            } else next(RescuePhase::ROTATE);
            break;
        case RescuePhase::ROTATE:
            if (dist < cfg.descent_dist_m) { next(RescuePhase::DESCENT); break; }
            ramp_yaw_ = fminf(ramp_yaw_ + dt, 1.0f);
            o.yaw_control = true;
            if (fabsf(err) < cfg.rotate_ok_deg || t_phase_ >= cfg.rotate_timeout_s) next(RescuePhase::FLY_HOME);
            break;
        case RescuePhase::FLY_HOME:
            ramp_yaw_ = fminf(ramp_yaw_ + dt, 1.0f); ramp_xy_ = fminf(ramp_xy_ + dt, 1.0f);
            o.yaw_control = true;
            if (dist < cfg.descent_dist_m) { next(RescuePhase::DESCENT); break; }
            speed = cfg.ground_speed_mps * clampf(dist / slowdown, 0.0f, 1.0f) * ramp_xy_;
            break;
        case RescuePhase::DESCENT: case RescuePhase::LANDING:
            ramp_xy_ = fminf(ramp_xy_ + dt, 1.0f);
            speed = cfg.ground_speed_mps * clampf(dist / slowdown, 0.0f, 1.0f) * ramp_xy_;
            // fast high up, slowly near the ground: x 0.6 below 10 m, x 3 from 50 m on
            o.climb_mps = -cfg.descend_mps * (0.6f + 2.4f * clampf((in.z - 10.0f) / 40.0f, 0.0f, 1.0f));
            if (phase_ == RescuePhase::DESCENT && in.z < cfg.landing_alt_m) next(RescuePhase::LANDING);
            break;
        default: break;
        }
        if (speed > 0.0f && dist > 0.2f) { o.vel_n = -in.pos_n / dist * speed; o.vel_e = -in.pos_e / dist * speed; }
        speed_1s_ = speed; climb_1s_ = o.climb_mps;
        if (o.yaw_control) o.yaw_rate_dps = clampf(-err * cfg.yaw_p * ramp_yaw_, -cfg.max_yaw_rate_dps, cfg.max_yaw_rate_dps);
        return o;
    }

    RescuePhase phase() const { return phase_; }
    RescueFail failure() const { return fail_; }
    float return_height() const { return return_z_; }
    float max_height() const { return max_z_; }
    static constexpr float kArrive = 0.5f;                 // m: this close to the return height counts as there
    static float wrap180(float d) { while (d > 180.0f) d -= 360.0f; while (d < -180.0f) d += 360.0f; return d; }

private:
    RescueOutput failed(RescueFail why) { phase_ = RescuePhase::FAILED; fail_ = why; return RescueOutput(); }
    void next(RescuePhase p) { phase_ = p; t_phase_ = 0; }
    RescuePhase phase_ = RescuePhase::IDLE; RescueFail fail_ = RescueFail::NONE;
    float max_z_ = 0, return_z_ = 0, t_phase_ = 0, ramp_yaw_ = 0, ramp_xy_ = 0, sec_ = 0, z_1s_ = 0, dist_1s_ = 0, speed_1s_ = 0, climb_1s_ = 0;
    bool from_below_ = true; int stalled_ = 0, flyaway_ = 0, lowsats_ = 0;
};

} // namespace fc
