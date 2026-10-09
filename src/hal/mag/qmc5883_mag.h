/*
 * This file is part of Windburst Project
 */

#pragma once

#include "mag_driver.h"

// ============================================================================
// QMC5883L (I2C 0x0D) and QMC5883P (I2C 0x2C). Values from Betaflight compass_qmc5883.c.
//   L: write CONF2(0x0A)=0x80 reset, wait 20 ms, id @0x0D == 0xFF and CONF1 (0x09) mode == standby | period reg 0x0B = 1 |
//      CONF1 = continuous | ODR 200 Hz | OSR 512 | +-8 G (0x1D, Betaflight QMC5883L_ODR_200HZ).  status @0x06 (DRDY 1, OVL 2, DOR 4; DOR cleared by reading 0x05).
//      data @0x00, LITTLE-endian X,Y,Z, +-8 G => 3000 LSB/gauss.
//   P: id @0x00 == 0x80 | 0x29 = 0x06 | CONF1 (0x0A) = continuous | 100 Hz | +-8 G | OSR1 8 (0x1B) | CONF2 (0x0B) = OSR2 8 (0x08).
//      status @0x09 (DRDY 1, OVL 2), data @0x01 LE X,Y,Z, +-8 G => 3750 LSB/gauss. Samples of 0,0,0 / -1,-1,-1 and a frozen output are dropped.
// ============================================================================
class Qmc5883Mag : public MagDriver {
public:
    enum class Variant : uint8_t { L, P };
    Qmc5883Mag(AgRegBus& bus, Variant v) : bus_(bus), v_(v) {}
    const char* name() const override { return v_ == Variant::L ? "QMC5883L" : "QMC5883P"; }

    bool init() override {
        ok_ = false; errors_ = 0; repeat_ = 0; have_last_ = false;
        if (v_ == Variant::L) {
            bus_.write(0x0A, 0x80);                               // soft reset (result ignored: BF does the same)
            ag_delay_ms(20);
            uint8_t id = 0, c1 = 0xFF;
            if (!bus_.read_u8(0x0D, id) || id != 0xFF) return false;
            if (!bus_.read_u8(0x09, c1) || (c1 & 0x03) != 0x00) return false;       // must be in standby after the reset, else it is not a QMC
            if (!bus_.write(0x0B, 0x01)) return false;
            if (!bus_.write(0x09, 0x01 | (3 << 2) | (0 << 6) | (1 << 4))) return false;   // continuous | 200 Hz | OSR 512 | 8 G
            scale_ = 1.0f / 3000.0f;
        } else {
            uint8_t id = 0;
            if (!bus_.read_u8(0x00, id) || id != 0x80) return false;
            if (!bus_.write(0x29, 0x06)) return false;
            if (!bus_.write(0x0A, 0x03 | (2 << 2) | (1 << 4) | (0 << 6))) return false;
            if (!bus_.write(0x0B, 0x08)) return false;
            scale_ = 1.0f / 3750.0f;
        }
        ag_delay_ms(20);
        ok_ = true;
        return true;
    }
    bool update(uint32_t now_us, MagSample& out) override {
        if (!ok_) return false;
        if (now_us - t_poll_ < 2500) return false;                 // 200 Hz => 5 ms period (P: 100 Hz) - poll at half the period
        t_poll_ = now_us;
        const bool L = v_ == Variant::L;
        uint8_t st;
        if (!bus_.read_u8(L ? 0x06 : 0x09, st)) { fail(); return false; }
        if (L && (st & 0x04)) { uint8_t dummy; bus_.read_u8(0x05, dummy); }          // data-overrun: reading 0x05 unlocks the registers
        if (!(st & 0x01)) return false;
        uint8_t b[6];
        if (!bus_.read(L ? 0x00 : 0x01, b, 6)) { fail(); return false; }
        if (st & 0x02) return false;                                // magnetic overflow: out of range
        const int16_t x = (int16_t)((b[1] << 8) | b[0]), y = (int16_t)((b[3] << 8) | b[2]), z = (int16_t)((b[5] << 8) | b[4]);
        if (!L && ((x == 0 && y == 0 && z == 0) || (x == -1 && y == -1 && z == -1))) return false;
        errors_ = 0;
        // A live sensor never repeats the exact same 3-axis count many times in a row (noise is several LSB): count consecutive repeats,
        // drop the repeats once it looks stuck, and flag the sensor unhealthy when it stays that way.
        if (have_last_ && x == lx_ && y == ly_ && z == lz_) { if (repeat_ < 255) repeat_++; } else repeat_ = 0;
        lx_ = x; ly_ = y; lz_ = z; have_last_ = true;
        if (repeat_ > 10) return false;
        out.gauss[0] = x * scale_; out.gauss[1] = y * scale_; out.gauss[2] = z * scale_;
        return true;
    }
    float rate_hz() const override { return v_ == Variant::L ? 200.0f : 100.0f; }
    bool healthy() const override { return ok_ && errors_ < 10 && repeat_ < 30; }
private:
    void fail() { if (errors_ < 255) errors_++; }
    AgRegBus& bus_; Variant v_;
    float scale_ = 1.0f / 3000.0f; bool ok_ = false, have_last_ = false; uint8_t errors_ = 0, repeat_ = 0; int16_t lx_ = 0, ly_ = 0, lz_ = 0; uint32_t t_poll_ = 0;
};
