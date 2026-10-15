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

} // namespace led

#if defined(ARDUINO_ARCH_ESP32)
#include <SPI.h>
#include "../bus/bus_spi.h"
namespace led {
class SpiStrip {
public:
    bool begin(uint8_t pin) {
        if (!spi_.begin(-1, -1, (int8_t)pin, -1)) return false;       // MOSI only: the clock and MISO pins stay unrouted
        ok_ = true; return true;
    }
    void show(const app::Rgb* c, int n) {
        if (!ok_ || n <= 0) return;
        const size_t k = encode(c, n, buf_, sizeof(buf_)); if (!k) return;
        spi_.beginTransaction(SPISettings(kSpiHz, MSBFIRST, SPI_MODE0)); spi_.transferBytes(buf_, nullptr, (uint32_t)k); spi_.endTransaction();
    }
private:
    SPIClass spi_{(int)SPI_DEVICE_1}; bool ok_ = false; uint8_t buf_[32 * 12 + kLatchBytes];
};
}
#define WB_HAS_LED_STRIP 1
#else
#define WB_HAS_LED_STRIP 0
#endif
