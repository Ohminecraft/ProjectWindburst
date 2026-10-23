#pragma once
#include "Arduino.h"
struct SPISettings { uint32_t hz = 0; SPISettings() {} SPISettings(uint32_t h, uint8_t, uint8_t) : hz(h) {} };
inline uint32_t g_stub_spi_max_hz = 0;      // the fastest clock a transaction was started with (reset it before a check)
// transfer() is routed to the simulated chip by test_manager.cpp
uint8_t stub_spi_transfer(uint8_t v);
extern int g_spi_begin_pins[4];   // sck, miso, mosi, ss as passed to begin()
// SPI bus numbers as the Arduino core of the ESP32-S3 defines them: two general-purpose buses. begin() on any other number
// fails there (spiStartBus: spi_num >= SPI_COUNT), and so it does here.
#define FSPI 0
#define HSPI 1
inline int g_stub_spi_bus_started = -1;     // bus number of the last successful begin()
inline int g_stub_spi_bus_refused = 0;
inline uint32_t g_stub_spi_bursts = 0;      // transferBytes() calls (a multi-byte register read must be one burst)      // begin() calls with a bus number this chip does not have
inline int g_stub_spi2_pins[4] = {-1, -1, -1, -1}; inline int g_stub_spi2_begins = 0;      // the second bus (HSPI): pins of its last begin(), how often it was started
inline uint8_t (*g_stub_spi2)(uint8_t) = nullptr;   // what a chip on the second SPI bus (HSPI) answers; nullptr: that bus goes to stub_spi_transfer as well
class SPIClass {
public:
    explicit SPIClass(int bus) : bus_(bus) {}
    bool begin(int8_t sck, int8_t miso, int8_t mosi, int8_t ss) {
        if (bus_ < 0 || bus_ > HSPI) { g_stub_spi_bus_refused++; return false; }
        if (bus_ == HSPI && g_stub_spi2) { g_stub_spi2_pins[0] = sck; g_stub_spi2_pins[1] = miso; g_stub_spi2_pins[2] = mosi; g_stub_spi2_pins[3] = ss; g_stub_spi2_begins++; return true; }
        g_stub_spi_bus_started = bus_;
        g_spi_begin_pins[0] = sck; g_spi_begin_pins[1] = miso; g_spi_begin_pins[2] = mosi; g_spi_begin_pins[3] = ss; return true;
    }
    void beginTransaction(SPISettings s) { if (s.hz > g_stub_spi_max_hz) g_stub_spi_max_hz = s.hz; }
    void endTransaction() {}
    // (the second bus has its own chip: a test program that simulates one sets g_stub_spi2)
    uint8_t transfer(uint8_t v) { return bus_ == HSPI && g_stub_spi2 ? g_stub_spi2(v) : stub_spi_transfer(v); }
    void transferBytes(const uint8_t* data, uint8_t* out, uint32_t size) { g_stub_spi_bursts++; for (uint32_t i = 0; i < size; i++) { const uint8_t r = transfer(data ? data[i] : 0xFF); if (out) out[i] = r; } }
private:
    int bus_;
};
