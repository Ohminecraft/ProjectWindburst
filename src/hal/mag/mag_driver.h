/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>
#include "../bus/reg_bus.h"

// ============================================================================
// Magnetometer driver interface (portable). Output unit: GAUSS (earth's field is ~0.25 .. 0.65 G), axes = the CHIP's own frame
// (mounting is applied later, once, in MagProcessor). update() never blocks: it returns true only for a NEW sample, polling the chip
// no faster than half its output period so a fast caller does not flood the bus.
// ============================================================================
struct MagSample {
    float gauss[3] = {0, 0, 0};
};

class MagDriver {
public:
    virtual ~MagDriver() = default;
    virtual const char* name() const = 0;
    virtual bool init() = 0;                                   // false = chip missing / wrong id / does not respond
    virtual bool update(uint32_t now_us, MagSample& out) = 0;
    virtual float rate_hz() const = 0;
    virtual bool healthy() const = 0;                          // false after repeated bus errors / a frozen output
};
