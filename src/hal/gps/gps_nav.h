/*
 * This file is part of Windburst Project
 */

#pragma once

#include <math.h>
#include "gps_types.h"

// ============================================================================
// GPS geometry + home point, portable and unit-tested. Mirrors Betaflight io/gps.c:
//   * home is (re)set when ARMING, requires a valid 3D fix and >= min_sats (BF: tryArm -> GPS_reset_home_position)
//   * gps_set_home_point_once: keep the first home for the whole power cycle; otherwise forget home on every disarm
//   * distance / direction to home (m, deci-degrees), distance flown while armed (counts only above 15 cm/s)
//   * "ready" = disarmed and a home fix would succeed right now (BF: READY beep)
//   * ENU offsets in cm (handles the +-180 degree meridian) for the position estimator / Position Hold / Return-To-Home that come next
// ============================================================================
namespace gps {

constexpr float kEarthAngleToCm = 111.3195f * 1000.0f * 100.0f / 10000000.0f;   // 1e-7 degree of latitude in cm (BF EARTH_ANGLE_TO_CM)
constexpr uint16_t kFlownMinSpeedCms = 15;

struct Location { int32_t lat = 0, lon = 0, alt_cm = 0; };

inline float cos_lat(int32_t lat_e7) { return cosf((float)lat_e7 * 1e-7f * 0.017453292519943295f); }

// East/north offset in cm of `to` relative to `from` (positive east / north).
inline void offset_cm(const Location& from, const Location& to, float& east, float& north) {
    int64_t dlon = (int64_t)to.lon - (int64_t)from.lon;            // 64-bit: two longitudes near +-180 differ by up to 3.6e9 (overflows int32)
    const int64_t deg180 = 1800000000ll;
    if (dlon > deg180) dlon -= 2 * deg180; else if (dlon <= -deg180) dlon += 2 * deg180;
    east = (float)dlon * cos_lat(from.lat) * kEarthAngleToCm;
    north = (float)(to.lat - from.lat) * kEarthAngleToCm;
}
// BF GPS_distance_cm_bearing: distance in cm, bearing in deci-degrees (0 = north, clockwise).
inline void distance_bearing(const Location& from, const Location& to, bool dist3d, uint32_t* dist_cm, int32_t* bearing_ddeg) {
    float e, n; offset_cm(from, to, e, n);
    const float dalt = dist3d ? (float)(to.alt_cm - from.alt_cm) : 0.0f;
    if (dist_cm) *dist_cm = (uint32_t)sqrtf(e * e + n * n + dalt * dalt);
    if (bearing_ddeg) {
        int32_t b = (int32_t)(9000.0f - atan2f(n, e) * 57.29577951308232f * 100.0f);   // clockwise from north, 0.01 deg
        if (b < 0) b += 36000;
        *bearing_ddeg = b / 10;
    }
}

class Home {
public:
    void configure(const Config& c) { cfg_ = c; }

    // Call every cycle with the latest solution and arming state.
    void update(const Solution& s, bool armed) {
        sol_loc_.lat = s.lat; sol_loc_.lon = s.lon; sol_loc_.alt_cm = s.alt_cm;
        if (armed && !was_armed_) reset(s);                         // arming edge
        if (!armed && !cfg_.set_home_point_once) has_home_ = false;  // BF: forget between arms
        if (!armed) {
            ready_ = s.fix && s.num_sat >= cfg_.min_sats;
            if (!ready_beeped_ && ready_) { ready_beeped_ = true; ready_edge_ = true; }
        }
        was_armed_ = armed;
        if (has_home_) {
            uint32_t d; int32_t b;
            distance_bearing(sol_loc_, home_, false, &d, &b);
            dist_cm_ = d; dir_ddeg_ = (uint16_t)b;
            if (armed) accumulate(s);
        } else { dist_cm_ = 0; dir_ddeg_ = 0; }
        last_ = sol_loc_;
    }
    // Explicit (re)set - BF also does this from the gyro-calibration stick gesture.
    void reset(const Solution& s) {
        if (!has_home_ || !cfg_.set_home_point_once) {
            if (s.fix && s.num_sat >= cfg_.min_sats) { home_ = {s.lat, s.lon, s.alt_cm}; has_home_ = true; }
        }
        flown_cm_ = 0; last_ = {s.lat, s.lon, s.alt_cm};
    }
    bool has_home() const { return has_home_; }
    const Location& home() const { return home_; }
    uint32_t distance_to_home_cm() const { return dist_cm_; }
    uint16_t distance_to_home_m() const { return (uint16_t)(dist_cm_ / 100); }
    uint16_t direction_to_home_ddeg() const { return dir_ddeg_; }
    uint32_t distance_flown_cm() const { return flown_cm_; }
    bool ready() const { return ready_; }
    bool take_ready_edge() { const bool e = ready_edge_; ready_edge_ = false; return e; }   // one-shot: beep "GPS ready"
    // East/north (cm) of the current position relative to home - what a position controller needs.
    bool offset_from_home_cm(float& east, float& north) const { if (!has_home_) return false; offset_cm(home_, sol_loc_, east, north); return true; }

private:
    void accumulate(const Solution& s) {
        const uint16_t speed = cfg_.use_3d_speed ? s.speed3d_cms : s.ground_speed_cms;
        if (speed > kFlownMinSpeedCms) { uint32_t d; distance_bearing(last_, sol_loc_, cfg_.use_3d_speed, &d, nullptr); flown_cm_ += d; }
    }
    Config cfg_;
    Location home_, sol_loc_, last_;
    bool has_home_ = false, was_armed_ = false, ready_ = false, ready_beeped_ = false, ready_edge_ = false;
    uint32_t dist_cm_ = 0, flown_cm_ = 0; uint16_t dir_ddeg_ = 0;
};

} // namespace gps
