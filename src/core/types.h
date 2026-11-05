#pragma once
#include "../config/fast_code.h"
// Windburst flight core - basic types. Hardware independent, header-only, namespace fc.
//
// FRAME / SIGN CONVENTION (the single source of truth for the whole core)
//   Board / body frame: FRD (x forward, y right, z down). Accelerometer = specific force in g: still & level => (0, 0, -1).
//   Gyro in deg/s (board frame). World frame: NED. Attitude quaternion = body -> world.
//   USER convention (what the pilot, the mixer, the PID and the app use): +roll = right wing down, +pitch = NOSE DOWN,
//   +yaw = nose right (clockwise from above). Stick forward / right / yaw-right = positive. The conversion happens in exactly one
//   place: Ahrs::euler_user() and the gyro conversion in FlightCore::update().
#include <math.h>
#include <stdint.h>

namespace fc {
constexpr float kPi = 3.14159265358979f;
constexpr float kG0 = 9.80665f;
constexpr float kDeg2Rad = kPi / 180.0f;
constexpr float kRad2Deg = 180.0f / kPi;
constexpr int kMaxMotors = 8;

enum Axis { ROLL = 0, PITCH = 1, YAW = 2 };

struct Vec3 { float x = 0, y = 0, z = 0; };
struct EulerDeg { float roll = 0, pitch = 0, yaw = 0; };

inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
// fmaxf / fminf with the same answer for NaN (the other argument), but inline: the C library's versions live in flash, and the core
// calls them dozens of times in a control cycle (tools/iram_report.py lists them).
WB_FAST inline float maxf(float a, float b) { return a != a ? b : (a < b ? b : a); }
WB_FAST inline float minf(float a, float b) { return a != a ? b : (b < a ? b : a); }
inline float signf(float v) { return v < 0.0f ? -1.0f : 1.0f; }
// Inf / NaN give 0 (no endless loop: Inf - 360 == Inf, and so is 1e30 - 360); a far-off angle is brought near with fmodf first.
WB_FAST inline float wrap180(float deg) {
    if (!(deg > -1e6f && deg < 1e6f)) { if (!isfinite(deg)) return 0.0f; deg = fmodf(deg, 360.0f); }
    while (deg > 180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}
inline uint32_t age_ms(uint32_t now_ms, uint32_t then_ms) { return now_ms - then_ms; }   // wraps correctly

// ---- fast trigonometry for the control cycle (Betaflight's maths.c approximations). The C library's sinf / atan2f / acosf are
// accurate to the last bit, but on the ESP32 they live in flash (WB_FAST cannot move library code) and take microseconds each:
// the attitude every cycle and the Angle / Stabilize controllers armed call several of them. These are plain polynomials, in RAM
// with the caller. Errors (test_core): sin / cos < 2e-6, atan2 / asin < 1e-6 rad, acos < 7e-5 rad (0.004 deg).
WB_FAST inline float sin_approx(float x) {
    if (!(x > -32.0f && x < 32.0f)) return 0.0f;            // (nonsense in, no endless loop out; NaN included)
    while (x > kPi) x -= 2.0f * kPi;
    while (x < -kPi) x += 2.0f * kPi;
    if (x > 0.5f * kPi) x = kPi - x; else if (x < -0.5f * kPi) x = -kPi - x;      // fold into [-pi/2, pi/2]
    const float x2 = x * x;
    return x + x * x2 * (-1.666568107e-1f + x2 * (8.312366210e-3f + x2 * -1.849218155e-4f));
}
WB_FAST inline float cos_approx(float x) { return sin_approx(x + 0.5f * kPi); }
WB_FAST inline float atan2_approx(float y, float x) {
    const float ax = fabsf(x), ay = fabsf(y), hi = ax > ay ? ax : ay, lo = ax > ay ? ay : ax;
    float r = hi > 0.0f ? lo / hi : 0.0f;
    r = -((((0.05030176425872175f * r - 0.3099814292351353f) * r - 0.14744007058297684f) * r - 0.99997356613987f) * r - 3.14551665884836e-07f)
        / ((0.6444640676891548f * r + 0.1471039133652469f) * r + 1.0f);
    if (ay > ax) r = 0.5f * kPi - r;
    if (x < 0.0f) r = kPi - r;
    if (y < 0.0f) r = -r;
    return r;
}
WB_FAST inline float acos_approx(float x) {
    const float xa = fabsf(x) > 1.0f ? 1.0f : fabsf(x);
    const float r = sqrtf(1.0f - xa) * (1.5707288f + xa * (-0.2121144f + xa * (0.0742610f + (-0.0187293f * xa))));
    return x < 0.0f ? kPi - r : r;
}
// tan for the filter coefficients (a few per gyro sample: the RPM notches are moved a little every cycle). Relative error < 4e-5 up
// to 0.48 of the sampling rate (the largest centre frequency a filter gets), < 5e-6 below 0.25. Only for 0 < x < pi / 2.
WB_FAST inline float tan_approx(float x) { return sin_approx(x) / cos_approx(x); }
WB_FAST inline float asin_approx(float x) { const float c = x > 1.0f ? 1.0f : (x < -1.0f ? -1.0f : x); return atan2_approx(c, sqrtf(1.0f - c * c)); }
} // namespace fc
