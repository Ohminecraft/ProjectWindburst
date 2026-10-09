/*
 * This file is part of Windburst Project
 */

#pragma once
#include "../../config/fast_code.h"

#include "acc_gyro_regbus.h"
#include "acc_gyro_types.h"

// ============================================================================
// Bosch BMI160 (SPI). Values from Betaflight accgyro_spi_bmi160.c.
//   CHIPID 0x00 -> 0xD1. The chip boots in I2C mode: a CS pulse (dummy write/read) switches it to SPI. A soft reset drops it back.
//   CMD 0x7E: 0xB6 reset, 0x15 gyro normal (<=80 ms), 0x11 accel normal (<=3.8 ms). PMU_STATUS 0x03 & 0x3C == 0x14 when both are normal.
//   ACC_CONF 0x40 = OSR2 | ODR800; GYR_CONF 0x42 = BWP | ODR; ACC_RANGE 0x41 = 0x08 (+-8 g); GYR_RANGE 0x43 = 0x00 (+-2000 dps).
//   Data little-endian, gyro @0x0C then accel @0x12 (contiguous 12 bytes). SPI reads have NO dummy byte (BF reads 1+6).
//   Scale: 16.384 LSB/dps, 4096 LSB/g.
// ============================================================================
class Bmi160AccGyro : public AccGyroDriver {
public:
    Bmi160AccGyro(AgRegBus& bus) : bus_(bus) {}
    const char* name() const override { return "BMI160"; }

    bool init(const AgInitConfig& cfg) override {
        wake_spi();
        ag_delay_ms(100);
        bus_.write(CMD, 0xB6);                                   // soft reset
        ag_delay_ms(100);
        wake_spi();                                              // reset returns the interface to I2C mode
        ag_delay_ms(10);

        uint8_t id = 0;
        if (!bus_.read_u8(CHIPID, id) || id != BMI160_WHO_AM_I) return false;

        if (!bus_.write(CMD, 0x15)) return false;               // gyro normal
        ag_delay_ms(100);
        if (!bus_.write(CMD, 0x11)) return false;               // accel normal
        ag_delay_ms(5);
        uint8_t pmu = 0;
        if (!bus_.read_u8(PMU_STATUS, pmu) || (pmu & 0x3C) != 0x14) return false;

        static const uint8_t bwp[3] = {0x00 /*OSR4*/, 0x10 /*OSR2*/, 0x20 /*normal*/};
        uint8_t odr = 0x0D; gyro_hz_ = 3200.0f;                  // 0x0D=3200 0x0C=1600 0x0B=800 (BMI160 tops out at 3200 Hz)
        if (cfg.gyro_odr_hz < 2400) { odr = 0x0C; gyro_hz_ = 1600.0f; }
        if (cfg.gyro_odr_hz < 1200) { odr = 0x0B; gyro_hz_ = 800.0f; }
        bus_.write(ACC_CONF, (uint8_t)(0x00 | 0x10 | 0x0B)); ag_delay_ms(1);   // ACC_US=0 (high perf) | OSR2 | 800 Hz
        bus_.write(GYR_CONF, (uint8_t)(bwp[cfg.hw_lpf > 2 ? 0 : cfg.hw_lpf] | odr)); ag_delay_ms(1);
        bus_.write(ACC_RANGE, 0x08); ag_delay_ms(1);
        bus_.write(GYR_RANGE, 0x00); ag_delay_ms(1);
        uint8_t off;
        if (bus_.read_u8(OFFSET_0, off)) bus_.write(OFFSET_0, (uint8_t)(off | 0xC0));   // enable gyro/acc offset compensation (BF)
        bus_.write(INT_EN1, 0x10); ag_delay_ms(1);               // DRDY interrupt
        bus_.write(INT_OUT_CTRL, 0x0A); ag_delay_ms(1);          // INT1 output, push-pull, active high
        bus_.write(INT_MAP1, 0x80); ag_delay_ms(1);              // DRDY -> INT1

        AgSample s;
        return read(s);
    }

    WB_FAST bool read(AgSample& out) override {
        uint8_t b[12];
        if (!bus_.read(GYR_DATA_X_LSB, b, 12)) return false;    // gyro(6) then accel(6), little-endian
        for (int i = 0; i < 3; i++) {
            out.gyro_dps[i] = le16(b + 2 * i) * (1.0f / 16.384f);
            out.acc_g[i] = le16(b + 6 + 2 * i) * (1.0f / 4096.0f);
        }
        return true;
    }

    bool data_ready() override { uint8_t v; return bus_.read_u8(STATUS, v) && (v & 0x40); }   // drdy_gyr
    float gyro_rate_hz() const override { return gyro_hz_; }
    float acc_rate_hz() const override { return 800.0f; }

private:
    void wake_spi() { if (bus_.is_spi()) { uint8_t d; bus_.read_u8(0x7F, d); } }   // dummy read = CS falling+rising edge

    static constexpr uint8_t CHIPID = 0x00, PMU_STATUS = 0x03, GYR_DATA_X_LSB = 0x0C, STATUS = 0x1B;
    static constexpr uint8_t ACC_CONF = 0x40, ACC_RANGE = 0x41, GYR_CONF = 0x42, GYR_RANGE = 0x43;
    static constexpr uint8_t INT_EN1 = 0x51, INT_OUT_CTRL = 0x53, INT_MAP1 = 0x56, OFFSET_0 = 0x77, CMD = 0x7E;
    AgRegBus& bus_;
    float gyro_hz_ = 3200.0f;
};
