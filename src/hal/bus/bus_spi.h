/*
 * This file is part of Windburst Project
 */

#pragma once
#include "../../config/fast_code.h"

#include <Arduino.h>
#include <SPI.h>
#include "../../port/gpio.h"

// WB_SPI_DIRECT: the IMU's burst read goes straight to the SPI controller's registers instead of through the Arduino SPI library
// (ESP32-S3 only). The library takes two mutexes and sets the clock up again for every transfer, and sets the chip-select pin
// with digitalWrite - all of it code in flash: about two thirds of the 37 us an IMU read took at 6 kHz on the first board that
// measured it (0.17). Only the IMU uses that SPI host (SPI_DEVICE_0: the barometer, the flow sensor, the LED strip and the SD
// card are on SPI_DEVICE_1), and only the flight task reads it, so no lock is missed. IMUAccGyro switches it on after the chip
// is set up and checks the readings with it; when they are not plausible it goes back to the library.
//   Off for a comparison:   build_flags = ... -DWB_SPI_DIRECT=0
#if !defined(WB_SPI_DIRECT)
#if defined(CONFIG_IDF_TARGET_ESP32S3) && !WB_PLATFORM_STM32
#define WB_SPI_DIRECT 1
#else
#define WB_SPI_DIRECT 0
#endif
#endif
#if WB_SPI_DIRECT
#include "soc/spi_struct.h"
#include "soc/gpio_struct.h"
#include "hal/gpio_ll.h"
#endif

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
#if WB_PLATFORM_STM32
    // STM32: the SPI instance (SPI1, SPI2 ...) is the one the pins belong to; the number only tells two buses apart.
    SPI_DEVICE_0 = 0,
    SPI_DEVICE_1 = 1,
#elif defined(CONFIG_IDF_TARGET_ESP32)
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
#if WB_PLATFORM_STM32
    SPIBus(SPIBus_device_e = SPI_DEVICE_0)
        : _spi(PNUM_NOT_DEFINED, PNUM_NOT_DEFINED, PNUM_NOT_DEFINED)
    {
    }
#else
    SPIBus(SPIBus_device_e port = SPI_DEVICE_0)
        : _host(port), _spi((int)port)
    {
    }
#endif

    // The clock of the burst reads (SPIReadRegisterMask with a buffer): the chip's limit, set once the chip is known. Writes and
    // single-register reads keep the slow clock.
    void setFastClock(uint32_t hz) { _config.bus_clock_fast_hz = hz; _fastSettings = SPISettings(hz, MSBFIRST, SPI_MODE3); }
    uint32_t fastClock() const { return _config.bus_clock_fast_hz; }
    // The burst reads past the Arduino library (WB_SPI_DIRECT). false: not on this chip / this bus, it stays off.
    bool setDirect(bool on)
    {
#if WB_SPI_DIRECT
        _direct = on && _initialized && _host == SPI_DEVICE_0;
        _programmed = false;
        return _direct;
#else
        (void)on; return false;
#endif
    }
    bool direct() const { return _direct; }

    bool SPIInit(const SPIBus_config_t& config)
    {
        _config = config;

#if WB_PLATFORM_STM32
        // The three pins must be SCK / MISO / MOSI of the same SPI instance; CS is a plain GPIO driven here.
        if (!port::pin_exists(_config.ioCsGpio)) return false;
        void* sck = pinmap_peripheral((PinName)_config.ioSckGpio, PinMap_SPI_SCLK);
        if (!sck || sck != pinmap_peripheral((PinName)_config.ioSdoGpio, PinMap_SPI_MISO) || sck != pinmap_peripheral((PinName)_config.ioSdiGpio, PinMap_SPI_MOSI)) return false;
        _spi.setSCLK((PinName)_config.ioSckGpio);
        _spi.setMISO((PinName)_config.ioSdoGpio);
        _spi.setMOSI((PinName)_config.ioSdiGpio);
        _spi.begin();
#else
        if (!_spi.begin(
            _config.ioSckGpio,
            _config.ioSdoGpio,  // MISO
            _config.ioSdiGpio,  // MOSI
            _config.ioCsGpio    // CS
        )) return false;
#endif
        _cs = port::apin(_config.ioCsGpio);

        pinMode(_cs, OUTPUT);
        digitalWrite(_cs, HIGH);

        _slowSettings = SPISettings(_config.bus_clock_slow_hz, MSBFIRST, SPI_MODE3);
        _fastSettings = SPISettings(_config.bus_clock_fast_hz, MSBFIRST, SPI_MODE3);

        _initialized = true;

        return true;
    }


    // A second device on a bus another SPIBus has started (SPIInit): its own CS pin, the host's SPIClass. The bus pins are not
    // attached again - on the ESP32 a second SPIClass on the same host would take the pins from the first one.
    // The host must outlive this object; both must be used from the same thread (or under one lock).
    bool SPIInitShared(SPIBus& host, uint8_t csGpio)
    {
        if (!host._initialized || !port::pin_exists(csGpio) || csGpio == host._config.ioCsGpio) return false;
        _config = host._config;
        _config.ioCsGpio = csGpio;
        _bus = host._bus;
        _host = host._host;
        _cs = port::apin(csGpio);
        pinMode(_cs, OUTPUT);
        digitalWrite(_cs, HIGH);
        _slowSettings = host._slowSettings;
        _fastSettings = host._fastSettings;
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

        _bus->beginTransaction(_slowSettings);

        digitalWrite(_cs, LOW);

        _bus->transfer(value);

        digitalWrite(_cs, HIGH);

        _bus->endTransaction();

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

        _bus->beginTransaction(_fastSettings);

        digitalWrite(_cs, LOW);

        value = _bus->transfer(0x00);

        digitalWrite(_cs, HIGH);

        _bus->endTransaction();

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

        _bus->beginTransaction(_slowSettings);

        digitalWrite(_cs, LOW);

        for (size_t i = 0; i < length; i++) {
            _bus->transfer(data[i]);
        }

        digitalWrite(_cs, HIGH);

        _bus->endTransaction();

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

        _bus->beginTransaction(_fastSettings);

        digitalWrite(_cs, LOW);

        for (size_t i = 0; i < length; i++) {
            data[i] = _bus->transfer(0x00);
        }

        digitalWrite(_cs, HIGH);

        _bus->endTransaction();

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

        _bus->beginTransaction(_slowSettings);

        digitalWrite(_cs, LOW);

        _bus->transfer(reg);
        _bus->transfer(value);

        digitalWrite(_cs, HIGH);

        _bus->endTransaction();

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

        _bus->beginTransaction(_fastSettings);

        digitalWrite(_cs, LOW);

        _bus->transfer(reg);
        value = _bus->transfer(0x00);

        digitalWrite(_cs, HIGH);

        _bus->endTransaction();

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

        _bus->beginTransaction(_fastSettings);

        digitalWrite(_cs, LOW);

        _bus->transfer(reg);

        for (size_t i = 0; i < length; i++) {
            buffer[i] = _bus->transfer(0x00);
        }

        digitalWrite(_cs, HIGH);

        _bus->endTransaction();

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

        _bus->beginTransaction(_slowSettings);

        digitalWrite(_cs, LOW);

        _bus->transfer(reg & 0x7F);
        _bus->transfer(value);

        digitalWrite(_cs, HIGH);

        _bus->endTransaction();

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

        _bus->beginTransaction(_fastSettings);

        digitalWrite(_cs, LOW);

        _bus->transfer(reg | 0x80);
        value = _bus->transfer(0x00);

        digitalWrite(_cs, HIGH);

        _bus->endTransaction();

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
#if WB_SPI_DIRECT
        if (_direct && length <= kBurstMax) return direct_read(reg, buffer, length);
#endif

        _bus->beginTransaction(_fastSettings);

        digitalWrite(_cs, LOW);

        if (length <= kBurstMax) {
            // One burst: the address and every data byte go through the SPI hardware in a single transfer (about 1 us per
            // byte at 8 MHz). Byte-by-byte calls cost several microseconds each, which was most of an IMU read at 4 kHz.
            uint8_t tx[kBurstMax + 1] = {};
            uint8_t rx[kBurstMax + 1];
            tx[0] = reg | 0x80;
#if WB_PLATFORM_STM32
            _bus->transfer(tx, rx, length + 1);
#else
            _bus->transferBytes(tx, rx, (uint32_t)(length + 1));
#endif
            for (size_t i = 0; i < length; i++) {
                buffer[i] = rx[i + 1];
            }
        } else {
            _bus->transfer(reg | 0x80);

            for (size_t i = 0; i < length; i++) {
                buffer[i] = _bus->transfer(0x00);
            }
        }

        digitalWrite(_cs, HIGH);

        _bus->endTransaction();

        return true;
    }


private:
    static constexpr size_t kBurstMax = 31;   // address + data fit the SPI hardware buffer (64 bytes) with room to spare

#if WB_SPI_DIRECT
    // One full-duplex transfer of address + data, as the library's spiTransferBytesNL does it on the S3 (the controller was left
    // full-duplex, MSB first, by SPIClass::begin), with the chip select set straight in the GPIO registers.
    WB_FAST bool direct_read(uint8_t reg, uint8_t* buffer, size_t length)
    {
        spi_dev_t* d = &GPSPI2;                                  // SPI_DEVICE_0 = FSPI = SPI2 on the S3
        // clock and mode: set through the library once, and again whenever someone else's transfer changed them (the slow
        // writes, the second IMU with another clock)
        if (!_programmed || d->clock.val != _clock_reg) {
            _bus->beginTransaction(_fastSettings); _bus->endTransaction();
            _clock_reg = d->clock.val; _programmed = true;
        }
        const uint32_t n = (uint32_t)length + 1, words = (n + 3) >> 2;
        uint32_t w[(kBurstMax + 1 + 3) / 4] = {};
        w[0] = (uint32_t)(reg | 0x80);                           // the first byte out is the low byte of the first word
        gpio_ll_set_level(&GPIO, _cs, 0);
        d->ms_dlen.ms_data_bitlen = n * 8 - 1;
        for (uint32_t i = 0; i < words; i++) d->data_buf[i] = w[i];
        bool ok = true;
        d->cmd.update = 1;
        for (uint32_t k = 0; d->cmd.update; k++) if (k > kSpinMax) { ok = false; break; }
        if (ok) {
            d->cmd.usr = 1;
            for (uint32_t k = 0; d->cmd.usr; k++) if (k > kSpinMax) { ok = false; break; }   // (32 bytes at 8 MHz: 32 us)
        }
        if (ok) for (uint32_t i = 0; i < words; i++) w[i] = d->data_buf[i];
        gpio_ll_set_level(&GPIO, _cs, 1);
        if (!ok) { _programmed = false; return false; }          // a controller that never finishes: one failed read (the loop skips the sample)
        const uint8_t* rx = (const uint8_t*)w;
        for (size_t i = 0; i < length; i++) buffer[i] = rx[i + 1];
        return true;
    }
    static constexpr uint32_t kSpinMax = 20000;                  // register polls, ~1 ms: far beyond any transfer
    uint32_t _clock_reg = 0;
    bool _programmed = false;
#endif
    bool _direct = false;
    SPIBus_device_e _host = SPI_DEVICE_0;

    SPIClass _spi;
    SPIClass* _bus = &_spi;    // the bus the transfers go through: our own, or the host's (SPIInitShared)

    SPIBus_config_t _config{};

    SPISettings _slowSettings;
    SPISettings _fastSettings;

    bool _initialized = false;
    uint32_t _cs = 0;          // the CS pin as the core numbers it (port/gpio.h)
};