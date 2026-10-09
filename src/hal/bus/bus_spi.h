/*
 * This file is part of Windburst Project
 */

#pragma once
#include "../../config/fast_code.h"

#include <Arduino.h>
#include <SPI.h>

struct SPIBus_config_t
{
    uint8_t ioSckGpio;
    uint8_t ioSdiGpio;   // MOSI: MCU -> device
    uint8_t ioSdoGpio;   // MISO: device -> MCU
    uint8_t ioCsGpio;

    uint32_t bus_clock_slow_hz = 1000000;
    uint32_t bus_clock_fast_hz = 8000000;
};

// The numbers are the Arduino core's SPI bus numbers, and they differ per chip:
//   classic ESP32            : HSPI = 2, VSPI = 3          (FSPI = 1 belongs to the flash)
//   ESP32-S2 / S3 / C3 / C5..: FSPI = 0 (SPI2), HSPI = 1 (SPI3 - S2 / S3 only); any other number makes SPIClass::begin() fail.
// (The first version of this file used 2 and 3 for every chip: on an ESP32-S3 the IMU's SPI bus then never started.)
enum SPIBus_device_e
{
#if defined(CONFIG_IDF_TARGET_ESP32)
    SPI_DEVICE_0 = HSPI,
    SPI_DEVICE_1 = VSPI,
#elif defined(HSPI)
    SPI_DEVICE_0 = FSPI,
    SPI_DEVICE_1 = HSPI,
#else
    SPI_DEVICE_0 = FSPI,
    SPI_DEVICE_1 = FSPI,     // this chip has one general-purpose SPI bus
#endif
    SPI_DEVICE_COUNT_MAX
};

class SPIBus {
public:
    SPIBus(SPIBus_device_e port = SPI_DEVICE_0)
        : _spi((int)port)
    {
    }

    bool SPIInit(const SPIBus_config_t& config)
    {
        _config = config;

        if (!_spi.begin(
            _config.ioSckGpio,
            _config.ioSdoGpio,  // MISO
            _config.ioSdiGpio,  // MOSI
            _config.ioCsGpio    // CS
        )) return false;

        pinMode(_config.ioCsGpio, OUTPUT);
        digitalWrite(_config.ioCsGpio, HIGH);

        _slowSettings = SPISettings(_config.bus_clock_slow_hz, MSBFIRST, SPI_MODE3);
        _fastSettings = SPISettings(_config.bus_clock_fast_hz, MSBFIRST, SPI_MODE3);

        _initialized = true;

        return true;
    }


    /*
     * Write one byte.
     */
    bool SPIWrite(uint8_t value)
    {
        if (!_initialized) {
            return false;
        }

        _spi.beginTransaction(_slowSettings);

        digitalWrite(_config.ioCsGpio, LOW);

        _spi.transfer(value);

        digitalWrite(_config.ioCsGpio, HIGH);

        _spi.endTransaction();

        return true;
    }


    /*
     * Read one byte by transmitting dummy data.
     */
    bool SPIRead(uint8_t& value)
    {
        if (!_initialized) {
            return false;
        }

        _spi.beginTransaction(_fastSettings);

        digitalWrite(_config.ioCsGpio, LOW);

        value = _spi.transfer(0x00);

        digitalWrite(_config.ioCsGpio, HIGH);

        _spi.endTransaction();

        return true;
    }


    /*
     * Write multiple bytes.
     */
    bool SPIWrite(
        const uint8_t* data,
        size_t length
    )
    {
        if (!_initialized || data == nullptr || length == 0) {
            return false;
        }

        _spi.beginTransaction(_slowSettings);

        digitalWrite(_config.ioCsGpio, LOW);

        for (size_t i = 0; i < length; i++) {
            _spi.transfer(data[i]);
        }

        digitalWrite(_config.ioCsGpio, HIGH);

        _spi.endTransaction();

        return true;
    }


    /*
     * Read multiple bytes.
     */
    bool SPIRead(
        uint8_t* data,
        size_t length
    )
    {
        if (!_initialized || data == nullptr || length == 0) {
            return false;
        }

        _spi.beginTransaction(_fastSettings);

        digitalWrite(_config.ioCsGpio, LOW);

        for (size_t i = 0; i < length; i++) {
            data[i] = _spi.transfer(0x00);
        }

        digitalWrite(_config.ioCsGpio, HIGH);

        _spi.endTransaction();

        return true;
    }

    /*
     * Write a register.
     */
    bool SPIWriteRegister(
        uint8_t reg,
        uint8_t value
    )
    {
        if (!_initialized) {
            return false;
        }

        _spi.beginTransaction(_slowSettings);

        digitalWrite(_config.ioCsGpio, LOW);

        _spi.transfer(reg);
        _spi.transfer(value);

        digitalWrite(_config.ioCsGpio, HIGH);

        _spi.endTransaction();

        return true;
    }


    /*
     * Read a register.
     */
    bool SPIReadRegister(
        uint8_t reg,
        uint8_t& value
    )
    {
        if (!_initialized) {
            return false;
        }

        _spi.beginTransaction(_fastSettings);

        digitalWrite(_config.ioCsGpio, LOW);

        _spi.transfer(reg);
        value = _spi.transfer(0x00);

        digitalWrite(_config.ioCsGpio, HIGH);

        _spi.endTransaction();

        return true;
    }


    /*
     * Read multiple bytes from a register.
     */
    bool SPIReadRegister(
        uint8_t reg,
        uint8_t* buffer,
        size_t length
    )
    {
        if (!_initialized || buffer == nullptr || length == 0) {
            return false;
        }

        _spi.beginTransaction(_fastSettings);

        digitalWrite(_config.ioCsGpio, LOW);

        _spi.transfer(reg);

        for (size_t i = 0; i < length; i++) {
            buffer[i] = _spi.transfer(0x00);
        }

        digitalWrite(_config.ioCsGpio, HIGH);

        _spi.endTransaction();

        return true;
    }

    /*
     * Write a register.
     */
    bool SPIWriteRegisterMask(
        uint8_t reg,
        uint8_t value
    )
    {
        if (!_initialized) {
            return false;
        }

        _spi.beginTransaction(_slowSettings);

        digitalWrite(_config.ioCsGpio, LOW);

        _spi.transfer(reg & 0x7F);
        _spi.transfer(value);

        digitalWrite(_config.ioCsGpio, HIGH);

        _spi.endTransaction();

        return true;
    }


    /*
     * Read a register with Mask.
     */
    bool SPIReadRegisterMask(
        uint8_t reg,
        uint8_t& value
    )
    {
        if (!_initialized) {
            return false;
        }

        _spi.beginTransaction(_fastSettings);

        digitalWrite(_config.ioCsGpio, LOW);

        _spi.transfer(reg | 0x80);
        value = _spi.transfer(0x00);

        digitalWrite(_config.ioCsGpio, HIGH);

        _spi.endTransaction();

        return true;
    }


    /*
     * Read multiple bytes from a register with Mask.
     */
    WB_FAST bool SPIReadRegisterMask(
        uint8_t reg,
        uint8_t* buffer,
        size_t length
    )
    {
        if (!_initialized || buffer == nullptr || length == 0) {
            return false;
        }

        _spi.beginTransaction(_fastSettings);

        digitalWrite(_config.ioCsGpio, LOW);

        if (length <= kBurstMax) {
            // One burst: the address and every data byte go through the SPI hardware in a single transfer (about 1 us per
            // byte at 8 MHz). Byte-by-byte calls cost several microseconds each, which was most of an IMU read at 4 kHz.
            uint8_t tx[kBurstMax + 1] = {};
            uint8_t rx[kBurstMax + 1];
            tx[0] = reg | 0x80;
            _spi.transferBytes(tx, rx, (uint32_t)(length + 1));
            for (size_t i = 0; i < length; i++) {
                buffer[i] = rx[i + 1];
            }
        } else {
            _spi.transfer(reg | 0x80);

            for (size_t i = 0; i < length; i++) {
                buffer[i] = _spi.transfer(0x00);
            }
        }

        digitalWrite(_config.ioCsGpio, HIGH);

        _spi.endTransaction();

        return true;
    }


private:
    static constexpr size_t kBurstMax = 31;   // address + data fit the SPI hardware buffer (64 bytes) with room to spare

    SPIClass _spi;

    SPIBus_config_t _config{};

    SPISettings _slowSettings;
    SPISettings _fastSettings;

    bool _initialized = false;
};