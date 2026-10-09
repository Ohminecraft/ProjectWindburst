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
inline float signf(float v) { return v < 0.0f ? -1.0f : 1.0f; }
WB_FAST inline float wrap180(float deg) { while (deg > 180.0f) deg -= 360.0f; while (deg < -180.0f) deg += 360.0f; return deg; }
inline uint32_t age_ms(uint32_t now_ms, uint32_t then_ms) { return now_ms - then_ms; }   // wraps correctly
} // namespace fc
