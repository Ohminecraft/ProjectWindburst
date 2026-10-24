/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>

// ============================================================================
// What the navigation stack (src/core/nav/) shows of itself, for the blackbox (group NAV_DEBUG, every fast record - at least
// 100 Hz at the default log rate) and nothing else. The flight core fills it at each estimator step; the recorder reads it.
// Units SI, frame NED with the origin at home (north, east, down): an acceleration "down" is positive towards the ground.
//
// P0: the raw inputs - accelerometer turned into NED with gravity taken away, its vibration and clipping, the barometer, the GPS
// as it arrives. Later phases add the estimate, innovations / test ratios, reject / reset flags, setpoints and controller terms
// (docs/NAV_ROADMAP.md).
// ============================================================================
namespace fc {
namespace nav {

struct Log {
    // ---- inputs
    float acc_ned[3] = {0, 0, 0};       // m/s^2, gravity removed (at rest: 0, 0, 0)
    float vib = 0;                      // m/s^2: how much the acceleration shakes about its own mean (low-passed |a - mean(a)|)
    uint8_t clip = 0;                   // accelerometer samples at the end of its range since the previous step (saturates)
    float baro_alt = 0;                 // m above the take-off point (up +), the newest reading
    float gps_pos[3] = {0, 0, 0};       // m from home: north, east, DOWN (the newest fix)
    float gps_vel[3] = {0, 0, 0};       // m/s NED (down 0 when the module does not measure it)
    float gps_hacc = 0, gps_sacc = 0;   // m, m/s: the module's own accuracy estimates
    uint8_t gps_sats = 0;
    uint8_t gps_flags = 0;              // bit 0 a fix has come, bit 1 vertical velocity measured, bit 2 used for XY
    uint8_t gps_seq = 0;                // counts the fixes (wraps): where it steps, a new fix came in - the log shows the GPS's real rate
};

}  // namespace nav
}  // namespace fc
