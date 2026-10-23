/*
 * This file is part of Windburst Project
 */

#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "../../port/platform.h"
#if WB_PLATFORM_STM32
#include "pinmap.h"
#endif

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
#if WB_PLATFORM_STM32
    // STM32: the I2C instance (I2C1, I2C2 ...) is the one the two pins belong to; the device number does not matter.
    I2CBus(I2CBus_device_e port)
        : _wire(), _port(port) {}
#else
    I2CBus(I2CBus_device_e port)
        : _wire((int)port), _port(port) {}
#endif

    bool I2CInit(const I2CBus_config_t& config) {
        _config = config;
        _addr = config.addr;

#if WB_PLATFORM_STM32
        if (config.ioSdaGpio == 255 || config.ioSclGpio == 255) return false;
        void* sda = pinmap_peripheral((PinName)config.ioSdaGpio, PinMap_I2C_SDA);
        void* scl = pinmap_peripheral((PinName)config.ioSclGpio, PinMap_I2C_SCL);
        if (!sda || sda != scl) return false;                   // the two pins must be SDA and SCL of the same I2C
        _wire.setSDA((PinName)config.ioSdaGpio);
        _wire.setSCL((PinName)config.ioSclGpio);
        _wire.begin();
        _wire.setClock(config.bus_clock_speed);
        return true;
#else
        return _wire.begin(
            config.ioSdaGpio,
            config.ioSclGpio,
            config.bus_clock_speed
        );
#endif
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

    // One byte, nothing after it: a command for chips without registers (MS5611).
    bool I2CWriteByte(uint8_t value) {
        _wire.beginTransmission(_addr);
        _wire.write(value);

        return _wire.endTransmission() == 0;
    }

    // Select the device address for the next transactions (several sensors on one bus share this object).
    void I2CSetAddress(uint8_t addr) { _addr = addr; }

    // The underlying TwoWire, for libraries that drive the bus themselves (Pololu VL53 drivers).
    TwoWire* I2CGetWire() { return &_wire; }

    I2CBus_device_e I2CGetPort() {
        return _port;
    }

private:
    TwoWire _wire;
    I2CBus_config_t _config{};
    uint8_t _addr = 0;
    I2CBus_device_e _port;
};