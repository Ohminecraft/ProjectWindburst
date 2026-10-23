/*
 * This file is part of Windburst Project
 */

#pragma once

#include "mag_driver.h"

// ============================================================================
// PNI RM3100, I2C 0x20 .. 0x23 (SA0 / SA1) or SPI (mode 0/3, read flag 0x80). Values from ArduPilot's AP_Compass_RM3100:
//   detection  the cycle-count registers CCX / CCY / CCZ (0x04 .. 0x09, 16 bits big-endian) hold their default 200 (0x00C8) -
//              ArduPilot's "whoami"; REVID (0x36) is not checked (clones report other values). Init writes 200 again, so a warm
//              restart is found the same way.
//   setup      TMRC (0x0B) = 0x94: 150 Hz; CMM (0x01) = 0x71: continuous, X Y Z, data ready when all three are done
//   data       STATUS (0x34) bit 7 = data ready; MX MY MZ at 0x24, 24-bit two's complement big-endian (9 bytes)
//   scale      cycle count 200: 75 LSB/uT = 7500 LSB/gauss (range +-800 uT)
// A sample of 0, 0, 0 and a frozen output are dropped, as for the QMC5883.
// ============================================================================
class Rm3100Mag : public MagDriver {
public:
    explicit Rm3100Mag(AgRegBus& bus) : bus_(bus) {}
    const char* name() const override { return "RM3100"; }

    bool init() override {
        ok_ = false; errors_ = 0; repeat_ = 0; have_last_ = false;
        uint8_t cc[6];
        if (!bus_.read(REG_CCX1, cc, 6)) return false;
        for (int i = 0; i < 6; i += 2) if (cc[i] != 0x00 || cc[i + 1] != 0xC8) return false;
        if (!bus_.write(REG_TMRC, 0x94) || !bus_.write(REG_CMM, 0x71)) return false;
        for (int i = 0; i < 6; i += 2) { bus_.write((uint8_t)(REG_CCX1 + i), 0x00); bus_.write((uint8_t)(REG_CCX1 + i + 1), 0xC8); }
        uint8_t tmrc = 0, cmm = 0;
        if (!bus_.read_u8(REG_TMRC, tmrc) || !bus_.read_u8(REG_CMM, cmm) || tmrc != 0x94 || (cmm & 0x71) != 0x71) return false;
        ag_delay_ms(10);
        ok_ = true;
        return true;
    }
    bool update(uint32_t now_us, MagSample& out) override {
        if (!ok_) return false;
        if (now_us - t_poll_ < 3300) return false;                      // 150 Hz => 6.7 ms period, polled at half of it
        t_poll_ = now_us;
        uint8_t st;
        if (!bus_.read_u8(REG_STATUS, st)) { fail(); return false; }
        if (!(st & 0x80)) return false;
        uint8_t b[9];
        if (!bus_.read(REG_MX2, b, 9)) { fail(); return false; }        // (reading the data clears data ready)
        int32_t v[3];
        for (int k = 0; k < 3; k++) v[k] = (int32_t)(((uint32_t)b[3 * k] << 24) | ((uint32_t)b[3 * k + 1] << 16) | ((uint32_t)b[3 * k + 2] << 8)) >> 8;
        if (v[0] == 0 && v[1] == 0 && v[2] == 0) return false;
        errors_ = 0;
        if (have_last_ && v[0] == last_[0] && v[1] == last_[1] && v[2] == last_[2]) { if (repeat_ < 255) repeat_++; } else repeat_ = 0;
        for (int k = 0; k < 3; k++) last_[k] = v[k];
        have_last_ = true;
        if (repeat_ > 10) return false;
        for (int k = 0; k < 3; k++) out.gauss[k] = (float)v[k] * (1.0f / 7500.0f);
        return true;
    }
    float rate_hz() const override { return 150.0f; }
    bool healthy() const override { return ok_ && errors_ < 10 && repeat_ < 30; }

private:
    void fail() { if (errors_ < 255) errors_++; }
    static constexpr uint8_t REG_CMM = 0x01, REG_CCX1 = 0x04, REG_TMRC = 0x0B, REG_MX2 = 0x24, REG_STATUS = 0x34;
    AgRegBus& bus_;
    int32_t last_[3] = {0, 0, 0};
    uint32_t t_poll_ = 0;
    uint8_t errors_ = 0, repeat_ = 0;
    bool ok_ = false, have_last_ = false;
};
