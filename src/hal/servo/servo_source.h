/*
 * This file is part of Windburst Project
 */

#pragma once
#include <stdint.h>

namespace servo {
// servo.source: 0 off, 1 .. 16 the RC channel, kSourceMission: only the mission moves it (DO_SET_SERVO), servo.init_us until then
constexpr uint8_t kSourceMission = 17;
}  // namespace servo
