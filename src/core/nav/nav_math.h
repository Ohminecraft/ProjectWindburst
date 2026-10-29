/*
 * This file is part of Windburst Project
 */

#pragma once
#include "../../config/fast_code.h"
#include "../types.h"

// ============================================================================
// Small pure helpers of the navigation stack (unit-tested in test_nav_ctl):
//   sqrt_controller   ArduPilot AP_Math sqrt_controller: linear (error x p) near the target, a square-root curve (constant
//                     deceleration `accel`) further out, so a far target is approached at a speed the craft can still stop from
//   accel <-> lean    the lean (deg, pilot convention: roll + = right side down, pitch + = nose down) that gives a horizontal
//                     acceleration at constant height, and back
//   ne <-> body       north / east turned into forwards / right by the heading (0 = north, clockwise), and back
//   limit_len         a 2-D vector shortened to a length
// ============================================================================
namespace fc {
namespace nav {

inline WB_FAST float sqrt_controller(float error, float p, float accel) {
    if (!(accel > 0.0f) || !(p > 0.0f)) return error * p;
    const float linear = accel / (p * p), a = fabsf(error);
    if (a <= linear) return error * p;
    return (error > 0.0f ? 1.0f : -1.0f) * sqrtf(2.0f * accel * (a - 0.5f * linear));
}
inline WB_FAST void accel_to_lean(float a_fwd, float a_right, float& roll_deg, float& pitch_deg) {
    const float pitch = atanf(a_fwd / kG0);
    pitch_deg = pitch * kRad2Deg; roll_deg = atanf(a_right * cosf(pitch) / kG0) * kRad2Deg;
}
inline WB_FAST void lean_to_accel(float roll_deg, float pitch_deg, float& a_fwd, float& a_right) {
    const float r = roll_deg * kDeg2Rad, p = pitch_deg * kDeg2Rad;
    a_fwd = kG0 * tanf(p); a_right = kG0 * tanf(r) / maxf(cosf(p), 0.3f);
}
inline WB_FAST void ne_to_body(float n, float e, float yaw_deg, float& fwd, float& right) { const float y = yaw_deg * kDeg2Rad, c = cosf(y), s = sinf(y); fwd = n * c + e * s; right = -n * s + e * c; }
inline WB_FAST void body_to_ne(float fwd, float right, float yaw_deg, float& n, float& e) { const float y = yaw_deg * kDeg2Rad, c = cosf(y), s = sinf(y); n = fwd * c - right * s; e = fwd * s + right * c; }
// What an outside autopilot (a mission, a RTH of its own, a companion computer - nav rewrite P7) asks the hold modes to fly.
// FlightCore::set_nav_request; flown only while Position Hold is on, its position is valid and the sticks are centred, and only
// while it is refreshed (nav_request_timeout_ms): the pilot's switch and sticks always come first.
struct NavRequest {
    bool position = true;            // true: fly to (north, east) and hold there; false: fly the velocity (vel_n, vel_e)
    float north_m = 0, east_m = 0;   // from home
    float vel_n_mps = 0, vel_e_mps = 0;
    float speed_mps = 5.0f;          // the most it flies towards a position
    bool height = false; float height_m = 0;      // also a height above the take-off point (Altitude Hold flies it)
};
inline WB_FAST void limit_len(float v[2], float len) { const float l = sqrtf(v[0] * v[0] + v[1] * v[1]); if (l > len && l > 0.0f) { v[0] *= len / l; v[1] *= len / l; } }

}  // namespace nav
}  // namespace fc
