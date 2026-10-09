/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>
#include <stddef.h>

// ============================================================================
// Portable register-bus interface shared by every I2C/SPI sensor driver (IMU, barometer, magnetometer, ...).
// No Arduino/SPI/Wire in here: drivers compile and are unit-tested on a PC against simulated chips.
// The Arduino side (reg_bus_arduino.h) wraps your I2CBus / SPIBus classes in adapters.
// ============================================================================

#if defined(ARDUINO)
  #include <Arduino.h>
  inline void ag_delay_ms(uint32_t ms) { delay(ms); }
  inline void ag_delay_us(uint32_t us) { delayMicroseconds(us); }
#else
  inline void ag_delay_ms(uint32_t) {}
  inline void ag_delay_us(uint32_t) {}
#endif

// Register-mapped bus. `reg` is always the plain register address: the SPI adapter adds the read flag (0x80),
// drivers never do. Implementations return false on a bus error.
class AgRegBus {
public:
    virtual ~AgRegBus() = default;
    virtual bool read(uint8_t reg, uint8_t* buf, size_t len) = 0;
    virtual bool write(uint8_t reg, uint8_t value) = 0;
    virtual bool write_buf(uint8_t reg, const uint8_t* data, size_t len) = 0;  // one burst, one CS assertion
    virtual bool is_spi() const = 0;

    bool read_u8(uint8_t reg, uint8_t& out) { return read(reg, &out, 1); }
    // read-modify-write; returns false on bus error
    bool update(uint8_t reg, uint8_t mask, uint8_t value) {
        uint8_t v;
        if (!read_u8(reg, v)) return false;
        v = (uint8_t)((v & ~mask) | (value & mask));
        return write(reg, v);
    }
    // write then read back and compare (used for the few registers where a silent miss would be dangerous)
    bool write_verify(uint8_t reg, uint8_t value, uint8_t verify_mask = 0xFF) {
        if (!write(reg, value)) return false;
        uint8_t v;
        if (!read_u8(reg, v)) return false;
        return (v & verify_mask) == (value & verify_mask);
    }
};
