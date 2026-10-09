/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>

// ============================================================================
// Downward range sensor (lidar / sonar) interface - portable, no Arduino.
// Drivers are NON-BLOCKING: update() returns true only when a genuinely NEW measurement is ready.
// distance_m is the raw distance ALONG THE BEAM; tilt correction is done once, in RangeHeight, because only it knows the attitude.
// ============================================================================
struct RangeReading {
    float distance_m = 0;
    uint8_t quality_pct = 0;     // 0..100, 0 = unknown/garbage; drivers derive it from the chip's status / signal rate
};

class RangeDriver {
public:
    virtual ~RangeDriver() = default;
    virtual const char* name() const = 0;
    virtual bool init() = 0;                                   // false = chip missing / firmware boot failed
    virtual bool update(uint32_t now_us, RangeReading& out) = 0;
    virtual float max_range_m() const = 0;                     // readings beyond this are not trusted
    virtual float min_range_m() const = 0;
    virtual float nominal_rate_hz() const = 0;
    virtual bool healthy() const = 0;
};
