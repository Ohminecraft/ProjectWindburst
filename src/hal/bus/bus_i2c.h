/*
 * This file is part of Windburst Project
 */

#pragma once

#include <Arduino.h>
#include <Wire.h>

struct I2CBus_config_t {
    uint8_t ioSdaGpio;
    uint8_t ioSclGpio;
    uint8_t addr;
    uint32_t bus_clock_speed;
};

enum I2CBus_device_e {
    I2C_DEVICE_0,
    I2C_DEVICE_1,
    I2C_DEVICE_2,
    I2C_DEVICE_3,
    I2C_DEVICE_4,
    I2C_DEVICE_5,
    I2C_DEVICE_COUNT_MAX
};

class I2CBus {
public:
    I2CBus(I2CBus_device_e port)
        : _wire((int)port) {}

    bool I2CInit(const I2CBus_config_t& config) {
        _config = config;
        _addr = config.addr;

        return _wire.begin(
            config.ioSdaGpio,
            config.ioSclGpio,
            config.bus_clock_speed
        );
    }

    bool I2CReadRegister(
        uint8_t reg,
        uint8_t* buf,
        uint8_t length
    ) {
        if (buf == nullptr || length == 0) {
            return false;
        }

        _wire.beginTransmission(_addr);
        _wire.write(reg);

        if (_wire.endTransmission(false) != 0) {
            return false;
        }

        if (_wire.requestFrom(
                static_cast<int>(_addr),
                static_cast<int>(length)
            ) != length) {
            return false;
        }

        for (uint8_t i = 0; i < length; i++) {
            buf[i] = _wire.read();
        }

        return true;
    }

    bool I2CWriteRegister(uint8_t reg, uint8_t value) {
        _wire.beginTransmission(_addr);
        _wire.write(reg);
        _wire.write(value);

        return _wire.endTransmission() == 0;
    }

    // Select the device address for the next transactions (several sensors on one bus share this object).
    void I2CSetAddress(uint8_t addr) { _addr = addr; }

    // The underlying TwoWire, for libraries that drive the bus themselves (Pololu VL53 drivers).
    TwoWire* I2CGetWire() { return &_wire; }

    I2CBus_device_e I2CGetPort() {
        return (I2CBus_device_e)_wire.getBusNum();
    }

private:
    TwoWire _wire;
    I2CBus_config_t _config{};
    uint8_t _addr = 0;
};