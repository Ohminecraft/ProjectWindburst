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
// The raw inputs (P0): accelerometer in NED with gravity taken away, its vibration and clipping, the barometer, the GPS as it
// arrives. The estimates (P1, P4) with their test ratios (innovation^2 / (gate^2 x variance): above 1 = rejected) and reset
// counters; the targets of the controllers (P2, P3, P5) and the fence (P6). docs/NAV_ROADMAP.md.
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
    // ---- estimates
    float est_pos_d = 0, est_vel_d = 0;  // AltEstimator: m, m/s (down +)
    float est_bias_d = 0, est_baro_off = 0;      // its accelerometer bias (m/s^2) and baro offset (m)
    float baro_ratio = 0, gps_ratio = 0;         // test ratios of the last baro reading / GPS solution (PosEstimator)
    uint8_t alt_resets = 0, pos_resets = 0;      // reset counters of the two estimators (wrap)
    float est_pos[2] = {0, 0}, est_vel[2] = {0, 0};   // PosEstimator: m, m/s north / east
    // ---- controllers
    float alt_tgt_pos_d = 0, alt_tgt_vel_d = 0;  // Altitude Hold's shaped target (m, m/s down +)
    float hover = 0;                             // the hover output it uses (0..1)
    float pos_tgt[2] = {0, 0};                   // Position Hold's target (m north / east)
    uint8_t ctl_flags = 0;                       // bit 0 Altitude Hold flies, 1 Position Hold flies, 2 holding (LOITER), 3 fence acts
};

}  // namespace nav
}  // namespace fc
