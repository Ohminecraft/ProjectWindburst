/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>
#include "../bus/reg_bus.h"

// ============================================================================
// Barometer driver interface (portable: no Arduino). Drivers are NON-BLOCKING and self-paced: call update(now_us) as often as you like
// (a 1 kHz task is fine). It triggers a forced-mode conversion, waits for the datasheet conversion time, checks the chip's status bits,
// reads the raw frame and compensates it. Returns true exactly when a NEW sample is available.
// ============================================================================
struct BaroSample {
    float pressure_pa = 0;
    float temp_c = 0;
};

struct BaroInitConfig {
    uint8_t osr_p = 3;      // pressure oversampling as log2: 0 = x1, 1 = x2, 2 = x4, 3 = x8 (default, same as Betaflight), 4 = x16 (BMP388 also 5 = x32)
    uint8_t osr_t = 0;      // temperature oversampling as log2: 0 = x1 (default)
};

class BaroDriver {
public:
    virtual ~BaroDriver() = default;
    virtual const char* name() const = 0;
    // reset + calibration read + first conversion self-check. false = chip missing/wrong, calibration unusable, or result implausible.
    virtual bool init(const BaroInitConfig& cfg) = 0;
    virtual bool update(uint32_t now_us, BaroSample& out) = 0;
    virtual float nominal_rate_hz() const = 0;      // 1 / (conversion time + scheduling gap)
    virtual bool healthy() const = 0;                // false after repeated bus errors / conversion timeouts
};
