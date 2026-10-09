/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>
#include <stddef.h>

// ============================================================================
// Portable layer shared by every acc/gyro chip driver (the register-bus interface itself lives in hal/bus/reg_bus.h). Nothing here includes Arduino/SPI/Wire, so the chip drivers
// compile and are unit-tested on a PC against a simulated chip (see test/test_accgyro.cpp).
// The Arduino side (acc_gyro_base.h) wraps I2CBus / SPIBus in AgRegBus adapters.
// ============================================================================

#include "../bus/reg_bus.h"

// What every chip driver hands upward: physical units, SENSOR axes (board alignment is applied later, once, in IMUAccGyro).
struct AgSample {
    float acc_g[3];
    float gyro_dps[3];
};

struct AgInitConfig {
    uint16_t gyro_odr_hz = 4000;   // requested gyro output rate; driver picks the nearest rate the chip supports
    uint8_t hw_lpf = 0;            // 0 = normal, 1 = wider, 2 = widest chip-internal gyro filter (Betaflight "gyro_hardware_lpf")
};

class AccGyroDriver {
public:
    virtual ~AccGyroDriver() = default;
    virtual const char* name() const = 0;
    // Full reset + configure. Must verify WHO_AM_I again (detection is only a hint). false = chip missing/wrong/not responding.
    virtual bool init(const AgInitConfig& cfg) = 0;
    // One burst read of accel+gyro. false on bus error (out untouched).
    virtual bool read(AgSample& out) = 0;
    // Cheap polling helper (status register). INT-pin driven sampling is preferred in flight; this is for bring-up/testing.
    virtual bool data_ready() { return true; }
    virtual float gyro_rate_hz() const = 0;   // actual configured gyro ODR
    virtual float acc_rate_hz() const = 0;

    static int16_t be16(const uint8_t* p) { return (int16_t)((p[0] << 8) | p[1]); }
    static int16_t le16(const uint8_t* p) { return (int16_t)((p[1] << 8) | p[0]); }
};
