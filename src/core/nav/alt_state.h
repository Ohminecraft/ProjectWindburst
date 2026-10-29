/*
 * This file is part of Windburst Project
 */

#pragma once
#include <stdint.h>

namespace fc {
// What Altitude Hold is doing (schema enum ah_state: the values are sent to the app and logged - append only)
enum class AhState : uint8_t { STOPPED, LANDED_IDLE, LANDED_PRE_TAKEOFF, TAKEOFF, FLYING };
}  // namespace fc
