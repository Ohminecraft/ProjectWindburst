/*
 * This file is part of Windburst Project
 */

#pragma once

#include "mag_driver.h"

// ============================================================================
// Honeywell HMC5883L, I2C 0x1E. Values from Betaflight compass_hmc5883l.c:
//   id regs 0x0A..0x0C = 'H','4','3' | CONFA = 8-sample average, 75 Hz, normal (0x78) | CONFB = +-1.3 Ga (0x20) | MODE = continuous
//   data @0x03, 6 bytes BIG-endian in the order X, Z, Y.  1.3 Ga => 1090 LSB/gauss.  Overflow reads as -4096.
// ============================================================================
class Hmc5883lMag : public MagDriver {
public:
    explicit Hmc5883lMag(AgRegBus& bus) : bus_(bus) {}
    const char* name() const override { return "HMC5883L"; }

    bool init() override {
        ok_ = false; errors_ = 0; frozen_ = 0; have_last_ = false;
        uint8_t id[3] = {0, 0, 0};
        if (!bus_.read(0x0A, id, 3) || id[0] != 'H' || id[1] != '4' || id[2] != '3') return false;
        if (!bus_.write(0x00, (3 << 5) | (6 << 2) | 0x00)) return false;
        bus_.write(0x01, 0x20);
        bus_.write(0x02, 0x00);
        ag_delay_ms(100);
        ok_ = true;
        return true;
    }
    bool update(uint32_t now_us, MagSample& out) override {
        if (!ok_) return false;
        if (now_us - t_poll_ < 6000) return false;                 // 75 Hz => 13.3 ms period; poll at >= 6.7 ms spacing
        t_poll_ = now_us;
        uint8_t st;
        if (!bus_.read_u8(0x09, st)) { fail(); return false; }
        if (!(st & 0x01)) return false;                            // RDY
        uint8_t b[6];
        if (!bus_.read(0x03, b, 6)) { fail(); return false; }
        const int16_t x = (int16_t)((b[0] << 8) | b[1]), z = (int16_t)((b[2] << 8) | b[3]), y = (int16_t)((b[4] << 8) | b[5]);
        if (x == -4096 || y == -4096 || z == -4096) return false;   // overflow marker
        errors_ = 0;
        if (have_last_ && x == lx_ && y == ly_ && z == lz_) { if (frozen_ < 255) frozen_++; } else frozen_ = 0;
        lx_ = x; ly_ = y; lz_ = z; have_last_ = true;
        out.gauss[0] = x * (1.0f / 1090.0f); out.gauss[1] = y * (1.0f / 1090.0f); out.gauss[2] = z * (1.0f / 1090.0f);
        return true;
    }
    float rate_hz() const override { return 75.0f; }
    bool healthy() const override { return ok_ && errors_ < 10 && frozen_ < 20; }
private:
    void fail() { if (errors_ < 255) errors_++; }
    AgRegBus& bus_;
    bool ok_ = false, have_last_ = false; uint8_t errors_ = 0, frozen_ = 0; int16_t lx_ = 0, ly_ = 0, lz_ = 0; uint32_t t_poll_ = 0;
};
