#pragma once
#include "Arduino.h"
// Routed to the simulated chip by test_manager.cpp
bool stub_i2c_probe(uint8_t addr);
int stub_i2c_tx(uint8_t addr, const uint8_t* d, size_t n, bool stop);
int stub_i2c_rx(uint8_t addr, uint8_t* d, size_t n);
inline int g_i2c_begin_pins[2] = {-1, -1};   // sda, scl as passed to begin()
class TwoWire {
public:
    TwoWire(int) {}
    bool begin(int sda, int scl, uint32_t) { g_i2c_begin_pins[0] = sda; g_i2c_begin_pins[1] = scl; return true; }
    void beginTransmission(int a) { addr_ = (uint8_t)a; n_ = 0; }
    size_t write(uint8_t v) { if (n_ < sizeof(buf_)) buf_[n_++] = v; return 1; }
    uint8_t endTransmission(bool stop = true) { return (uint8_t)stub_i2c_tx(addr_, buf_, n_, stop); }
    size_t requestFrom(int a, int n) { rn_ = 0; rl_ = 0; int got = stub_i2c_rx((uint8_t)a, rx_, (size_t)n); rl_ = got; return (size_t)got; }
    int read() { return rn_ < rl_ ? rx_[rn_++] : -1; }
    int getBusNum() { return 0; }
private:
    uint8_t addr_ = 0, buf_[300]; size_t n_ = 0; uint8_t rx_[64]; int rn_ = 0, rl_ = 0;
};
