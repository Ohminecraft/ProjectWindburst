/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include "../../app/led_logic.h"

// ============================================================================
// WS2812 / SK6812 LEDs on one data pin (pins.led), sent through the MOSI line of an SPI bus at 3.2 MHz: every data bit is four
// SPI bits - 1000 for a 0 (0.31 us high), 1110 for a 1 (0.94 us high) - so the timing comes from the SPI clock, not from the CPU.
// GRB order, then 40 zero bytes (100 us low) that latch the strip. The ESP32-S3's RMT transmitters all go to DShot with four
// motors; the second SPI bus (HSPI) is usually free (not when the SD card is on SPI or a PMW3901 is fitted).
// encode() is portable (test_app); SpiStrip is the ESP32 end.
// ============================================================================
namespace led {

constexpr uint32_t kSpiHz = 3200000;
constexpr size_t kLatchBytes = 40;
inline size_t bytes_for(int n) { return (size_t)n * 12 + kLatchBytes; }

inline size_t encode(const app::Rgb* c, int n, uint8_t* out, size_t cap) {
    if (bytes_for(n) > cap) return 0;
    size_t k = 0;
    for (int i = 0; i < n; i++) {
        const uint8_t v[3] = {c[i].g, c[i].r, c[i].b};
        for (int j = 0; j < 3; j++)
            for (int b = 7; b >= 0; b -= 2) {                    // two data bits per SPI byte
                const uint8_t hi = (v[j] >> b) & 1u ? 0xE0 : 0x80, lo = (v[j] >> (b - 1)) & 1u ? 0x0E : 0x08;
                out[k++] = (uint8_t)(hi | lo);
            }
    }
    for (size_t i = 0; i < kLatchBytes; i++) out[k++] = 0;
    return k;
}
// back from SPI bytes to colors (the tests check the encoding with it)
inline int decode(const uint8_t* in, size_t n, app::Rgb* c, int max) {
    int leds = 0;
    for (size_t k = 0; k + 12 <= n && leds < max; k += 12) {
        uint8_t v[3] = {};
        bool any = false;
        for (int j = 0; j < 3; j++) for (int q = 0; q < 4; q++) {
            const uint8_t x = in[k + j * 4 + q]; if (x) any = true;
            const uint8_t hi = (x >> 4) == 0xE ? 1 : 0, lo = (x & 0x0F) == 0xE ? 1 : 0;
            v[j] = (uint8_t)((v[j] << 2) | (hi << 1) | lo);
        }
        if (!any) break;
        c[leds].g = v[0]; c[leds].r = v[1]; c[leds].b = v[2]; leds++;
    }
    return leds;
}

// ---- the STM32 way (port/stm32/ws2812_stm32.h, Betaflight's ws2811 driver): the pin's timer channel at 800 kHz, a DMA writes the
// compare value of every bit (GRB, MSB first): a 1 is high for 2/3 of the 1.25 us bit (0.83 us), a 0 for 1/3 (0.42 us); after the
// last LED two rows of 0 keep the line low (the latch is the pause until the next picture, 20 ms at 50 pictures a second).
constexpr uint32_t kTimerHz = 800000;
constexpr int kRowTail = 2;
inline int rows_for(int n) { return n * 24 + kRowTail; }
inline uint32_t row_one(uint32_t period) { return (period * 2u + 1u) / 3u; }
inline uint32_t row_zero(uint32_t period) { return (period + 1u) / 3u; }
inline int timer_rows(const app::Rgb* c, int n, uint32_t period, uint32_t* out, int cap) {
    if (n < 0 || rows_for(n) > cap) return 0;
    int k = 0;
    const uint32_t one = row_one(period), zero = row_zero(period);
    for (int i = 0; i < n; i++) {
        const uint8_t v[3] = {c[i].g, c[i].r, c[i].b};
        for (int j = 0; j < 3; j++) for (int b = 7; b >= 0; b--) out[k++] = (v[j] >> b) & 1u ? one : zero;
    }
    for (int t = 0; t < kRowTail; t++) out[k++] = 0;
    return k;
}
// back from the rows to colors (the tests): -1 when a row is neither a clean 1 nor a clean 0
inline int timer_rows_read(const uint32_t* rows, int k, uint32_t period, app::Rgb* c, int max) {
    int leds = 0;
    for (int i = 0; i + 24 <= k && leds < max; i += 24) {
        if (rows[i] == 0) break;
        uint8_t v[3] = {};
        for (int j = 0; j < 3; j++) for (int b = 0; b < 8; b++) {
            const uint32_t x = rows[i + j * 8 + b];
            if (x != row_one(period) && x != row_zero(period)) return -1;
            v[j] = (uint8_t)((v[j] << 1) | (x == row_one(period) ? 1 : 0));
        }
        c[leds].g = v[0]; c[leds].r = v[1]; c[leds].b = v[2]; leds++;
    }
    return leds;
}

} // namespace led

#if defined(ARDUINO_ARCH_ESP32)
#include <SPI.h>
#include "../bus/bus_spi.h"
namespace led {
// The core's SPI calls (esp32-hal-spi.h), not SPIClass: SPIClass::begin() attaches a clock pin first and fails without one
// (spiAttachSCK(-1) returns false, Arduino-ESP32 3.x), so with begin(-1, -1, pin, -1) the strip never started. Only MOSI is routed.
class SpiStrip {
public:
    bool begin(uint8_t pin) {
        if (bus_) return true;
        div_ = spiFrequencyToClockDiv(nullptr, kSpiHz);
        spi_t* b = spiStartBus((uint8_t)SPI_DEVICE_1, div_, SPI_MODE0, SPI_MSBFIRST);
        if (!b) { why_ = "the second SPI bus did not start"; return false; }
        if (!spiAttachMOSI(b, (int8_t)pin)) { spiStopBus(b); why_ = "the LED pin could not be routed to the SPI bus"; return false; }
        bus_ = b; return true;
    }
    void show(const app::Rgb* c, int n) {
        if (!bus_ || n <= 0) return;
        const size_t k = encode(c, n, buf_, sizeof(buf_)); if (!k) return;
        spiTransaction(bus_, div_, SPI_MODE0, SPI_MSBFIRST); spiWriteNL(bus_, buf_, (uint32_t)k); spiEndTransaction(bus_);
    }
    const char* why() const { return why_; }
private:
    spi_t* bus_ = nullptr; uint32_t div_ = 0; const char* why_ = nullptr;
    alignas(4) uint8_t buf_[32 * 12 + kLatchBytes];      // (spiWriteNL reads it as 32-bit words)
};
using Strip = SpiStrip;
}
#define WB_HAS_LED_STRIP 1
#elif defined(ARDUINO_ARCH_STM32)
#include "../../port/stm32/ws2812_stm32.h"
#define WB_HAS_LED_STRIP 1
#else
#define WB_HAS_LED_STRIP 0
#endif
