/*
 * This file is part of Windburst Project
 */

#pragma once
#include <stdint.h>

namespace fc {
// What Position Hold does on one axis (schema enum ph_axis: sent to the app and logged - append only)
enum class PhAxis : uint8_t { PILOT, BRAKE, BRAKE_READY, TO_LOITER, LOITER, TO_PILOT };
}  // namespace fc
