// Register-level simulated IMUs shared by test_accgyro.cpp and test_manager.cpp (see test_accgyro.cpp header comment).
#pragma once
#include <string.h>
#include <math.h>
#include <vector>
#include "../src/hal/acc_gyro/acc_gyro_regbus.h"
#include "../src/hal/acc_gyro/bmi270_config.h"

enum Chip { C_MPU6000, C_MPU6050, C_MPU6500, C_MPU9250, C_ICM20601, C_ICM20602, C_ICM42622P, C_ICM42686P, C_ICM42688P,
            C_ICM45605, C_ICM45686, C_BMI160, C_BMI270, C_ICM20948, C_ICM20649, C_COUNT };

struct Sim : AgRegBus {
    Chip c; bool spi;
    uint8_t r[8][256]; int bank = 0;
    bool spi_mode = true, fail = false, init_ok = false, bad_upload = false;
    uint8_t pmu_gyr = 0, pmu_acc = 0;
    std::vector<uint8_t> ucode = std::vector<uint8_t>(8192, 0); size_t ucode_hits = 0;
    uint16_t ireg_addr = 0; uint8_t ireg[65536];
    float tg[3] = {10.0f, -20.0f, 30.0f}, ta[3] = {0.10f, -0.20f, 0.97f};

    Sim(Chip chip, bool is_spi) : c(chip), spi(is_spi) { memset(ireg, 0, sizeof(ireg)); hard_reset(); }
    bool is_spi() const override { return spi; }

    bool bmi() const { return c == C_BMI160 || c == C_BMI270; }
    bool icm209() const { return c == C_ICM20948 || c == C_ICM20649; }
    void hard_reset() {
        memset(r, 0, sizeof(r)); bank = 0; pmu_gyr = pmu_acc = 0; init_ok = false;
        if (c <= C_ICM20602) r[0][0x6B] = 0x40;                      // MPU: sleeping after reset
        if (bmi() && spi) spi_mode = false;                          // BMI boots in I2C mode until a CS edge
        if (icm209()) r[0][0x06] = 0x41;                             // ICM-209xx: PWR_MGMT_1 after reset = sleep
    }
    // ---- scales implied by the range registers ----
    float gyro_lsb() const {
        switch (c) {
        case C_ICM20601: { static const float t[4] = {65.5f, 32.8f, 16.4f, 8.2f}; return t[(r[0][0x1B] >> 3) & 3]; }
        case C_MPU6000: case C_MPU6050: case C_MPU6500: case C_MPU9250: case C_ICM20602: { static const float t[4] = {131.f, 65.5f, 32.8f, 16.4f}; return t[(r[0][0x1B] >> 3) & 3]; }
        case C_ICM42686P: return 8.2f * (1 << ((r[0][0x4F] >> 5) & 7));            // FS_SEL 0 = 4000 dps
        case C_ICM42622P: case C_ICM42688P: return 16.4f * (1 << ((r[0][0x4F] >> 5) & 7));   // FS_SEL 0 = 2000 dps
        case C_ICM45605: case C_ICM45686: return 8.2f * (1 << ((r[0][0x1C] >> 4) & 15));    // FS_SEL 0 = 4000 dps, 1 = 2000
        case C_BMI160: return 16.384f * (1 << (r[0][0x43] & 7));                 // 0 = 2000 dps
        case C_ICM20948: { static const float t[4] = {131.f, 65.5f, 32.8f, 16.4f}; return t[(r[2][0x01] >> 1) & 3]; }   // bank 2 GYRO_CONFIG_1
        case C_ICM20649: { static const float t[4] = {65.5f, 32.8f, 16.4f, 8.2f}; return t[(r[2][0x01] >> 1) & 3]; }
        default: return 16.384f * (1 << (r[0][0x43] & 7));                       // BMI270
        }
    }
    float acc_lsb() const {
        switch (c) {
        case C_ICM20601: { static const float t[4] = {8192.f, 4096.f, 2048.f, 1024.f}; return t[(r[0][0x1C] >> 3) & 3]; }
        case C_MPU6000: case C_MPU6050: case C_MPU6500: case C_MPU9250: case C_ICM20602: { static const float t[4] = {16384.f, 8192.f, 4096.f, 2048.f}; return t[(r[0][0x1C] >> 3) & 3]; }
        case C_ICM42686P: return 1024.f * (1 << ((r[0][0x50] >> 5) & 7));         // FS_SEL 0 = 32 g
        case C_ICM42622P: case C_ICM42688P: return 2048.f * (1 << ((r[0][0x50] >> 5) & 7));   // FS_SEL 0 = 16 g, 1 = 8 g ...
        case C_ICM45605: case C_ICM45686: return 1024.f * (1 << ((r[0][0x1B] >> 4) & 7));    // 0 = 32 g, 1 = 16 g
        case C_ICM20948: { static const float t[4] = {16384.f, 8192.f, 4096.f, 2048.f}; return t[(r[2][0x14] >> 1) & 3]; }   // bank 2 ACCEL_CONFIG
        case C_ICM20649: { static const float t[4] = {8192.f, 4096.f, 2048.f, 1024.f}; return t[(r[2][0x14] >> 1) & 3]; }
        case C_BMI160: { switch (r[0][0x41]) { case 0x03: return 16384.f; case 0x05: return 8192.f; case 0x08: return 4096.f; case 0x0C: return 2048.f; default: return 16384.f; } }
        default: { static const float t[4] = {16384.f, 8192.f, 4096.f, 2048.f}; return t[r[0][0x41] & 3]; }
        }
    }
    static void put(uint8_t* d, float v, float lsb, bool big) {
        float x = v * lsb; if (x > 32767.f) x = 32767.f; if (x < -32768.f) x = -32768.f;
        int16_t i = (int16_t)lrintf(x);
        if (big) { d[0] = (uint8_t)(i >> 8); d[1] = (uint8_t)i; } else { d[0] = (uint8_t)i; d[1] = (uint8_t)(i >> 8); }
    }
    // Data window: returns true and fills `win` (starting at base) when the chip would present live data right now.
    bool window(uint8_t& base, uint8_t win[14], size_t& n) const {
        memset(win, 0, 14);
        const float gl = gyro_lsb(), al = acc_lsb();
        if (c <= C_ICM20602) {                                          // MPU family: ax ay az temp gx gy gz, big-endian @0x3B
            base = 0x3B; n = 14;
            if (r[0][0x6B] & 0x40) return true;                          // sleeping: zeros
            for (int i = 0; i < 3; i++) { put(win + 2 * i, ta[i], al, true); put(win + 8 + 2 * i, tg[i], gl, true); }
            return true;
        }
        if (c == C_ICM42622P || c == C_ICM42686P || c == C_ICM42688P) { // ax ay az gx gy gz big-endian @0x1F (bank 0 only)
            base = 0x1F; n = 12;
            if (bank != 0 || (r[0][0x4E] & 0x0F) != 0x0F) return true;
            for (int i = 0; i < 3; i++) { put(win + 2 * i, ta[i], al, true); put(win + 6 + 2 * i, tg[i], gl, true); }
            return true;
        }
        if (c == C_ICM45605 || c == C_ICM45686) {                       // ax ay az gx gy gz little-endian @0x00
            base = 0x00; n = 12;
            if (r[0][0x10] != 0x0F) return true;
            for (int i = 0; i < 3; i++) { put(win + 2 * i, ta[i], al, false); put(win + 6 + 2 * i, tg[i], gl, false); }
            return true;
        }
        if (c == C_BMI160) {                                            // gx gy gz ax ay az little-endian @0x0C
            base = 0x0C; n = 12;
            if (!(pmu_gyr == 1 && pmu_acc == 1)) return true;
            for (int i = 0; i < 3; i++) { put(win + 2 * i, tg[i], gl, false); put(win + 6 + 2 * i, ta[i], al, false); }
            return true;
        }
        if (icm209()) {                                                 // ax ay az gx gy gz big-endian @0x2D, bank 0 only
            base = 0x2D; n = bank == 0 ? 12 : 0;
            if (r[0][0x06] & 0x40 || r[0][0x07] != 0) return true;      // asleep, or a sensor off: zeros
            for (int i = 0; i < 3; i++) { put(win + 2 * i, ta[i], al, true); put(win + 6 + 2 * i, tg[i], gl, true); }
            return true;
        }
        base = 0x0C; n = 12;                                            // BMI270: ax ay az gx gy gz little-endian @0x0C
        if (!init_ok || (r[0][0x7D] & 0x0E) != 0x0E) return true;
        for (int i = 0; i < 3; i++) { put(win + 2 * i, ta[i], al, false); put(win + 6 + 2 * i, tg[i], gl, false); }
        return true;
    }
    uint8_t getreg(uint16_t a) const {
        uint8_t base, win[14]; size_t n;
        window(base, win, n);
        if (a >= base && a < base + n) return win[a - base];
        switch (c) {
        case C_MPU6000: case C_MPU6050: if (a == 0x75) return 0x68; break;
        case C_MPU6500: if (a == 0x75) return 0x70; break;
        case C_MPU9250: if (a == 0x75) return 0x71; break;
        case C_ICM20601: if (a == 0x75) return 0xAC; break;
        case C_ICM20602: if (a == 0x75) return 0x12; break;
        case C_ICM42622P: if (a == 0x75 && bank == 0) return 0x46; break;
        case C_ICM42686P: if (a == 0x75 && bank == 0) return 0x44; break;
        case C_ICM42688P: if (a == 0x75 && bank == 0) return 0x47; break;
        case C_ICM45605: if (a == 0x72) return 0xE5; break;
        case C_ICM45686: if (a == 0x72) return 0xE9; break;
        case C_BMI160: if (a == 0x00) return 0xD1; if (a == 0x03) return (uint8_t)((pmu_gyr << 2) | (pmu_acc << 4)); if (a == 0x1B) return 0x40; break;
        case C_BMI270: if (a == 0x00) return 0x24; if (a == 0x03) return 0x40; if (a == 0x21) return init_ok ? 0x01 : 0x00; break;
        case C_ICM20948: if (a == 0x00 && bank == 0) return 0xEA; break;
        case C_ICM20649: if (a == 0x00 && bank == 0) return 0xE1; break;
        default: break;
        }
        if (c <= C_ICM20602 && a == 0x3A) return 0x01;
        if ((c == C_ICM45605 || c == C_ICM45686)) { if (a == 0x7F) return (uint8_t)(r[0][0x7F] | 0x01); if (a == 0x7E) return ireg[ireg_addr]; if (a == 0x19) return 0x04; }
        if (c == C_ICM42622P || c == C_ICM42686P || c == C_ICM42688P) { if (a == 0x2D) return 0x08; return r[bank][a]; }
        if (icm209()) { if (a == 0x1A && bank == 0) return 0x01; return r[bank][a]; }
        return r[0][a];
    }

    bool read(uint8_t reg, uint8_t* buf, size_t len) override {
        if (fail) return false;
        if (bmi() && spi && !spi_mode) { spi_mode = true; for (size_t i = 0; i < len; i++) buf[i] = 0xFF; return true; }  // first access = wake, data is junk
        size_t off = 0;
        if (c == C_BMI270 && spi) { buf[0] = 0x00; off = 1; }            // BMI270 SPI reads: dummy byte first
        for (size_t i = off; i < len; i++) buf[i] = getreg((uint16_t)(reg + (i - off)));
        return true;
    }
    bool write(uint8_t reg, uint8_t v) override {
        if (fail) return false;
        if (bmi() && spi && !spi_mode) return true;                      // ignored while still in I2C mode
        switch (c) {
        case C_MPU6000: case C_MPU6050: case C_MPU6500: case C_MPU9250: case C_ICM20601: case C_ICM20602:
            if (reg == 0x6B && (v & 0x80)) { hard_reset(); return true; } break;
        case C_ICM42622P: case C_ICM42686P: case C_ICM42688P:
            if (reg == 0x76) { bank = v & 7; return true; }
            if (reg == 0x11 && (v & 1) && bank == 0) { hard_reset(); return true; }
            r[bank][reg] = v; return true;
        case C_ICM45605: case C_ICM45686:
            if (reg == 0x7F && (v & 0x02)) { hard_reset(); r[0][0x7F] = 0; return true; } break;
        case C_ICM20948: case C_ICM20649:
            if (reg == 0x7F) { bank = (v >> 4) & 3; return true; }
            if (reg == 0x06 && bank == 0 && (v & 0x80)) { hard_reset(); return true; }   // (the bank stays 0 after a reset)
            break;
        case C_BMI160:
            if (reg == 0x7E) { if (v == 0xB6) { hard_reset(); } else if (v == 0x15) pmu_gyr = 1; else if (v == 0x11) pmu_acc = 1; return true; } break;
        case C_BMI270:
            if (reg == 0x7E && v == 0xB6) { hard_reset(); return true; }
            if (reg == 0x59 && v == 1) { init_ok = (memcmp(ucode.data(), bmi270_config_file, 8192) == 0) && !bad_upload; }
            break;
        default: break;
        }
        r[bank][reg] = v; return true;
    }
    bool write_buf(uint8_t reg, const uint8_t* d, size_t len) override {
        if (fail) return false;
        if (bmi() && spi && !spi_mode) return true;
        if ((c == C_ICM45605 || c == C_ICM45686) && reg == 0x7C) {       // IREG: addr(15:8), addr(7:0), [data]
            if (len < 2) return false;
            ireg_addr = (uint16_t)((d[0] << 8) | d[1]);
            if (len >= 3) ireg[ireg_addr] = d[2];
            return true;
        }
        if (c == C_BMI270 && reg == 0x5E) {
            if (r[0][0x7C] != 0x00) bad_upload = true;                   // advanced power save must be off during upload
            size_t at = (size_t)(((r[0][0x5C] << 4) | (r[0][0x5B] & 0x0F)) * 2);
            if (at + len > 8192) { bad_upload = true; return true; }
            memcpy(ucode.data() + at, d, len); ucode_hits += len; return true;
        }
        for (size_t i = 0; i < len; i++) r[bank][reg + i] = d[i];
        return true;
    }
};

