/*
 * This file is part of Windburst Project
 */

#pragma once
#include "../../config/fast_code.h"

#include "acc_gyro_regbus.h"
#include "acc_gyro_types.h"

// ============================================================================
// TDK ICM-45686 / ICM-45605 (SPI, UI interface, low-noise mode). Values from Betaflight accgyro_spi_icm456xx.c (which follows
// the TDK reference driver / DS-000577).  NOTE: there is NO bank-select register (0x75 is reserved) - do not use ICM426xx code.
//   WHO_AM_I @0x72 (0xE9 / 0xE5), soft reset via REG_MISC2 (0x7F) bit1, PWR_MGMT0 (0x10): gyro LN (3<<2) | accel LN (3).
//   Internal filter registers live behind the indirect-register interface (IREG): 0x7C/0x7D = address, 0x7E = data.
//   Data little-endian, accel @0x00, gyro @0x06 (contiguous 12-byte burst). 16.384 LSB/dps (+-2000), 2048 LSB/g (+-16 g).
// ============================================================================
class Icm456xxAccGyro : public AccGyroDriver {
public:
    enum class Variant : uint8_t { ICM45605, ICM45686 };
    Icm456xxAccGyro(AgRegBus& bus, Variant v) : bus_(bus), v_(v) {}

    const char* name() const override { return v_ == Variant::ICM45686 ? "ICM45686" : "ICM45605"; }

    bool init(const AgInitConfig& cfg) override {
        bus_.write(REG_MISC2, SOFT_RESET);                       // soft reset
        uint32_t waited = 0; uint8_t m = SOFT_RESET;
        while (waited < 20000) {                                 // bit clears when the reset is done (20 ms max)
            if (bus_.read_u8(REG_MISC2, m) && !(m & SOFT_RESET)) break;
            ag_delay_us(10); waited += 10;
        }
        if (m & SOFT_RESET) return false;
        bus_.write(PWR_MGMT0, 0x00);

        uint8_t who = 0; bool ok = false;
        for (int i = 0; i < 20 && !ok; i++) {
            ag_delay_ms(1);
            ok = bus_.read_u8(WHO_AM_I, who) && who == (v_ == Variant::ICM45686 ? ICM45686_WHO_AM_I : ICM45605_WHO_AM_I);
        }
        if (!ok) return false;

        bus_.write(PWR_MGMT0, (uint8_t)((3 << 2) | 3));          // gyro + accel low-noise
        ag_delay_ms(1);
        bus_.write(ACCEL_CONFIG0, (uint8_t)((0x01 << 4) | ACCEL_ODR_1K6));   // +-16 g, 1.6 kHz
        ag_delay_ms(10);

        // Filters are nice-to-have: on failure leave the chip default and carry on (BF does the same, "don't halt boot over a filter")
        if (!ireg_update(ACCEL_SRC_CTRL, 0x03, 0x02)) ireg_update(ACCEL_SRC_CTRL, 0x03, 0x00);       // interpolator + FIR AAF
        if (!ireg_update(ACCEL_UI_LPF_CFG, 0x07, 0x02)) ireg_update(ACCEL_UI_LPF_CFG, 0x07, 0x00);   // ODR/8
        if (!ireg_update(GYRO_SRC_CTRL, 0x03 << 5, 0x02 << 5)) ireg_update(GYRO_SRC_CTRL, 0x03 << 5, 0x00);
        static const uint8_t lpf_div[3] = {0x04 /*ODR/32*/, 0x03 /*ODR/16*/, 0x02 /*ODR/8*/};
        const uint8_t sel = lpf_div[cfg.hw_lpf > 2 ? 0 : cfg.hw_lpf];
        if (!ireg_update(GYRO_UI_LPF_CFG, 0x07, sel)) ireg_update(GYRO_UI_LPF_CFG, 0x07, 0x00);

        uint8_t odr = ODR_3K2; gyro_hz_ = 3200.0f;
        if (cfg.gyro_odr_hz >= 4800)      { odr = ODR_6K4; gyro_hz_ = 6400.0f; }
        else if (cfg.gyro_odr_hz >= 2400) { odr = ODR_3K2; gyro_hz_ = 3200.0f; }
        else if (cfg.gyro_odr_hz >= 1200) { odr = ODR_1K6; gyro_hz_ = 1600.0f; }
        else                              { odr = ODR_800; gyro_hz_ = 800.0f; }
        bus_.write(GYRO_CONFIG0, (uint8_t)((0x01 << 4) | odr));  // +-2000 dps
        ag_delay_ms(35);                                         // gyro start-up (DS Table 9-6)

        bus_.write(INT1_CONFIG2, 0x01);                          // pulsed, push-pull, active high
        bus_.write(INT1_CONFIG0, 0x04);                          // DRDY

        AgSample s;
        return read(s);
    }

    WB_FAST bool read(AgSample& out) override {
        uint8_t b[12];
        if (!bus_.read(ACCEL_DATA_X1, b, 12)) return false;      // little-endian
        for (int i = 0; i < 3; i++) {
            out.acc_g[i] = le16(b + 2 * i) * (1.0f / 2048.0f);
            out.gyro_dps[i] = le16(b + 6 + 2 * i) * (1.0f / 16.384f);
        }
        return true;
    }

    bool data_ready() override { uint8_t v; return bus_.read_u8(INT1_STATUS0, v) && (v & 0x04); }
    float gyro_rate_hz() const override { return gyro_hz_; }
    uint32_t max_spi_read_hz() const override { return 20000000; }         // datasheet: 24 MHz
    float acc_rate_hz() const override { return 1600.0f; }

private:
    // --- indirect register access (DS-000577 "Host Indirect Access Register interface") ---
    bool ireg_ready() {
        for (uint32_t w = 0; w < 5000; w += 4) {
            uint8_t v;
            if (bus_.read_u8(REG_MISC2, v) && (v & IREG_DONE)) return true;
            ag_delay_us(4);
        }
        return false;
    }
    bool ireg_read(uint16_t reg, uint8_t& val) {
        if (!ireg_ready()) return false;
        const uint8_t addr[2] = {(uint8_t)(reg >> 8), (uint8_t)reg};
        if (!bus_.write_buf(IREG_ADDR_15_8, addr, 2)) return false;
        ag_delay_us(4);
        bool ok = bus_.read_u8(IREG_DATA, val);
        ag_delay_us(4);
        return ok;
    }
    bool ireg_write(uint16_t reg, uint8_t val) {
        if (!ireg_ready()) return false;
        const uint8_t d[3] = {(uint8_t)(reg >> 8), (uint8_t)reg, val};
        bool ok = bus_.write_buf(IREG_ADDR_15_8, d, 3);
        ag_delay_us(4);
        return ok;
    }
    bool ireg_update(uint16_t reg, uint8_t mask, uint8_t value) {
        for (int attempt = 0; attempt < 3; attempt++) {
            uint8_t cur;
            if (!ireg_read(reg, cur)) return false;
            const uint8_t want = (uint8_t)((cur & ~mask) | (value & mask));
            if (want != cur && !ireg_write(reg, want)) return false;
            uint8_t back;
            if (ireg_read(reg, back) && (back & mask) == (value & mask)) return true;
        }
        return false;
    }

    static constexpr uint8_t PWR_MGMT0 = 0x10, INT1_CONFIG0 = 0x16, INT1_CONFIG2 = 0x18, INT1_STATUS0 = 0x19;
    static constexpr uint8_t ACCEL_CONFIG0 = 0x1B, GYRO_CONFIG0 = 0x1C, WHO_AM_I = 0x72, REG_MISC2 = 0x7F;
    static constexpr uint8_t ACCEL_DATA_X1 = 0x00;
    static constexpr uint8_t SOFT_RESET = 1 << 1, IREG_DONE = 1 << 0;
    static constexpr uint8_t IREG_ADDR_15_8 = 0x7C, IREG_DATA = 0x7E;          // 0x7D (addr 7:0) follows in the same burst
    static constexpr uint8_t ODR_6K4 = 0x03, ODR_3K2 = 0x04, ODR_1K6 = 0x05, ODR_800 = 0x06, ACCEL_ODR_1K6 = 0x05;
    static constexpr uint16_t ACCEL_SRC_CTRL = 0xA57B, ACCEL_UI_LPF_CFG = 0xA583, GYRO_SRC_CTRL = 0xA4A6, GYRO_UI_LPF_CFG = 0xA4AC;
    AgRegBus& bus_;
    Variant v_;
    float gyro_hz_ = 3200.0f;
};
