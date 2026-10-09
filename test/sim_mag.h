// Register-level simulated magnetometers. Raw counts are produced from a physical field vector (gauss) with each chip's own
// range / endianness / axis order / status behaviour, independent of the drivers.
#pragma once
#pragma GCC diagnostic ignored "-Wmisleading-indentation"
#include <string.h>
#include <math.h>
#include <stdint.h>
#include "../src/hal/bus/reg_bus.h"

struct MagSimBase : AgRegBus {
    bool spi = false, dead = false;
    double field[3] = {0.20, -0.05, 0.40};          // gauss, in the CHIP's own frame
    bool is_spi() const override { return spi; }
    static int16_t q(double v, double lsb_per_g) { double x = v * lsb_per_g; if (x > 32767) x = 32767; if (x < -32768) x = -32768; return (int16_t)llround(x); }
};

// ---------------- HMC5883L: X,Z,Y big-endian, 1090 LSB/G at gain 1.3 Ga, id 'H43', RDY bit, -4096 = overflow
struct HmcSim : MagSimBase {
    uint8_t confa = 0, confb = 0, mode = 0x03; bool rdy = true; bool overflow = false; bool wrong_id = false; int reads_to_rdy = 0;
    uint8_t reg(uint8_t r) const {
        if (r == 0x0A) return wrong_id ? 'X' : 'H'; if (r == 0x0B) return '4'; if (r == 0x0C) return '3';
        if (r == 0x00) return confa; if (r == 0x01) return confb; if (r == 0x02) return mode;
        if (r == 0x09) return rdy ? 0x01 : 0x00;
        if (r >= 0x03 && r <= 0x08) {
            const bool active = (mode & 3) == 0;
            const double lsb = (confb >> 5) == 1 ? 1090.0 : (confb >> 5) == 3 ? 660.0 : 1090.0;
            int16_t x = active ? q(field[0], lsb) : 0, y = active ? q(field[1], lsb) : 0, z = active ? q(field[2], lsb) : 0;
            if (overflow) y = -4096;
            const uint8_t d[6] = {(uint8_t)(x >> 8), (uint8_t)x, (uint8_t)(z >> 8), (uint8_t)z, (uint8_t)(y >> 8), (uint8_t)y};
            return d[r - 0x03];
        }
        return 0;
    }
    bool read(uint8_t r, uint8_t* b, size_t n) override { if (dead) return false; for (size_t i = 0; i < n; i++) b[i] = reg((uint8_t)(r + i)); if (r == 0x03) rdy = false; return true; }
    bool write(uint8_t r, uint8_t v) override { if (dead) return false; if (r == 0x00) confa = v; else if (r == 0x01) confb = v; else if (r == 0x02) mode = v; return true; }
    bool write_buf(uint8_t, const uint8_t*, size_t) override { return false; }
    void new_data() { rdy = true; }
};

// ---------------- QMC5883L / QMC5883P
struct QmcSim : MagSimBase {
    bool P = false; uint8_t conf1 = 0, conf2 = 0, period = 0, unlock = 0; bool drdy = true, dor = false, ovl = false, id_wrong = false; int unlocks = 0;
    bool reset_seen = false; uint8_t p29 = 0;
    double lsb() const { const bool g8 = (conf1 >> 4) & 1; return P ? (g8 ? 3750.0 : 15000.0) : (g8 ? 3000.0 : 12000.0); }
    bool running() const { return P ? (conf1 & 3) == 3 : (conf1 & 3) == 1; }
    uint8_t reg(uint8_t r) const {
        if (!P) {
            if (r == 0x0D) return id_wrong ? 0x00 : 0xFF; if (r == 0x09) return conf1; if (r == 0x0A) return conf2;
            if (r == 0x06) return (uint8_t)((drdy ? 1 : 0) | (ovl ? 2 : 0) | (dor ? 4 : 0));
            if (r <= 0x05) { if (!running()) return 0; const int16_t v[3] = {q(field[0], lsb()), q(field[1], lsb()), q(field[2], lsb())}; return (r & 1) ? (uint8_t)(v[r / 2] >> 8) : (uint8_t)v[r / 2]; }
        } else {
            if (r == 0x00) return id_wrong ? 0x00 : 0x80; if (r == 0x0A) return conf1; if (r == 0x0B) return conf2; if (r == 0x09) return (uint8_t)((drdy ? 1 : 0) | (ovl ? 2 : 0));
            if (r >= 0x01 && r <= 0x06) { if (!running()) return 0; const int16_t v[3] = {q(field[0], lsb()), q(field[1], lsb()), q(field[2], lsb())}; const int i = r - 1; return (i & 1) ? (uint8_t)(v[i / 2] >> 8) : (uint8_t)v[i / 2]; }
        }
        return 0;
    }
    bool read(uint8_t r, uint8_t* b, size_t n) override { if (dead) return false; for (size_t i = 0; i < n; i++) b[i] = reg((uint8_t)(r + i)); if (!P && r == 0x05) { dor = false; unlocks++; } if ((!P && r == 0x00) || (P && r == 0x01)) drdy = false; return true; }
    bool write(uint8_t r, uint8_t v) override {
        if (dead) return false;
        if (!P) { if (r == 0x0A) { if (v & 0x80) { conf1 = 0; conf2 = 0; reset_seen = true; } else conf2 = v; } else if (r == 0x09) conf1 = v; else if (r == 0x0B) period = v; }
        else { if (r == 0x29) p29 = v; else if (r == 0x0A) conf1 = v; else if (r == 0x0B) conf2 = v; }
        return true;
    }
    bool write_buf(uint8_t, const uint8_t*, size_t) override { return false; }
};

// ---------------- MPU9250 + AK8963 behind the MPU's I2C master (SPI)
struct Mpu9250AkSim : MagSimBase {
    uint8_t r[256]; uint8_t ak[32]; uint8_t asa[3] = {138, 120, 128}; bool ak_wia_ok = true; bool ak_overflow = false;
    int master_ops = 0;
    // field in the AK8963 frame (what the chip measures)
    Mpu9250AkSim() { memset(r, 0, sizeof(r)); memset(ak, 0, sizeof(ak)); spi = true; r[0x75] = 0x71; }
    bool master_on() const { return (r[0x6A] & 0x20) && (r[0x24] & 0x0F) != 0; }
    int16_t ak_axis(int i) const {                                   // adjusted-sensitivity inverse: raw = H / (adj * 0.0015)
        const double adj = ((double)asa[i] - 128.0) / 256.0 + 1.0;
        return q(field[i] / (adj * 0.0015), 1.0);
    }
    void ak_fill() {
        ak[0x00] = ak_wia_ok ? 0x48 : 0x00; ak[0x10] = asa[0]; ak[0x11] = asa[1]; ak[0x12] = asa[2];
        const bool cont = (ak[0x0A] & 0x0F) == 0x06; ak[0x02] = cont ? 1 : 0; ak[0x09] = ak_overflow ? 0x08 : 0x00;
        for (int i = 0; i < 3; i++) { const int16_t v = cont ? ak_axis(i) : 0; ak[0x03 + 2 * i] = (uint8_t)v; ak[0x04 + 2 * i] = (uint8_t)(v >> 8); }
    }
    void run_slv0() {                                                // one I2C-master transaction for SLV0
        if (!master_on() || !(r[0x27] & 0x80)) return;
        master_ops++;
        const uint8_t addr = r[0x25], reg = r[0x26], n = r[0x27] & 0x0F;
        if ((addr & 0x7F) != 0x0C) return;
        ak_fill();
        if (addr & 0x80) { for (int i = 0; i < n && i < 24; i++) r[0x49 + i] = ak[(reg + i) & 31]; }
        else { ak[reg & 31] = r[0x63]; if (reg == 0x0B && (r[0x63] & 1)) { memset(ak + 2, 0, 10); } }
    }
    bool read(uint8_t reg, uint8_t* b, size_t n) override {
        if (dead) return false;
        if (reg >= 0x49 && reg < 0x49 + 24 && (r[0x27] & 0x80) && (r[0x25] & 0x80)) run_slv0();      // continuous mode: data refreshed every sample
        for (size_t i = 0; i < n; i++) b[i] = r[(uint8_t)(reg + i)];
        return true;
    }
    bool write(uint8_t reg, uint8_t v) override {
        if (dead) return false; r[reg] = v;
        if (reg == 0x27) run_slv0();
        return true;
    }
    bool write_buf(uint8_t, const uint8_t*, size_t) override { return false; }
};
