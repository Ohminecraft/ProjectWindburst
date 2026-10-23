/*
 * This file is part of Windburst Project
 */

#pragma once
#include "../../config/fast_code.h"

#include "acc_gyro_regbus.h"
#include "acc_gyro_types.h"

// ============================================================================
// TDK InvenSense ICM-20948 and ICM-20649 (the "v2" register map with four banks, REG_BANK_SEL 0x7F): ArduPilot's
// AP_InertialSensor_Invensensev2 and Betaflight's accgyro_spi_icm20649.c. Only the accel / gyro part (the ICM-20948's AK09916
// compass sits behind its internal I2C master: not used).
//   WHO_AM_I    bank 0 reg 0x00: 0xEA ICM-20948, 0xE1 ICM-20649
//   reset       PWR_MGMT_1 (bank 0, 0x06) = 0x80, 100 ms; then 0x01 (best clock, awake). SPI: USER_CTRL (0x03) I2C_IF_DIS.
//   ranges      bank 2: GYRO_CONFIG_1 (0x01) FS bits 2:1, ACCEL_CONFIG (0x14) FS bits 2:1
//               ICM-20948: gyro 2000 dps (FS 3, 16.4 LSB/dps), accel 16 g (FS 3, 2048 LSB/g)
//               ICM-20649 (high range): gyro 2000 dps (FS 2, 16.4 LSB/dps), accel 16 g (FS 2, 2048 LSB/g)
//   rate        SPI and more than 1.1 kHz asked: the digital low-pass bypassed (FCHOICE 0): gyro 9 kHz, accel 4.5 kHz -
//               Betaflight does the same above 1.1 kHz. Otherwise (I2C): DLPF on (config 0, 197 Hz), 1125 Hz.
//   data        bank 0 0x2D: accel x y z, gyro x y z, big-endian (12 bytes); INT_STATUS_1 (0x1A) bit 0 = raw data ready
// The driver leaves bank 0 selected: every read is a single burst.
// ============================================================================
class Icm20948AccGyro : public AccGyroDriver {
public:
    enum class Variant : uint8_t { ICM20948, ICM20649 };
    Icm20948AccGyro(AgRegBus& bus, Variant v) : bus_(bus), v_(v) {}
    const char* name() const override { return v_ == Variant::ICM20948 ? "ICM20948" : "ICM20649"; }

    bool init(const AgInitConfig& cfg) override {
        const bool spi = bus_.is_spi();
        bank(0);
        uint8_t who = 0;
        if (!bus_.read_u8(REG_WHO_AM_I, who) || who != want_who()) return false;
        bus_.write(REG_PWR_MGMT_1, 0x80);                    // device reset
        ag_delay_ms(100);
        bank(0);
        bus_.write(REG_PWR_MGMT_1, 0x01);                    // awake, best clock (PLL when ready)
        ag_delay_ms(15);
        if (spi) { bus_.write(REG_USER_CTRL, 0x10); ag_delay_ms(1); }      // I2C_IF_DIS: SPI only from here
        bus_.write(REG_PWR_MGMT_2, 0x00);                    // accel and gyro on
        bus_.write(REG_LP_CONFIG, 0x00);                     // no duty cycling
        const bool fast = spi && cfg.gyro_odr_hz > 1125;
        const uint8_t fs = v_ == Variant::ICM20948 ? 3 : 2;  // 2000 dps / 16 g on both (the 20649 counts from 500 dps / 4 g)
        bank(2);
        bus_.write(REG_GYRO_SMPLRT_DIV, 0x00);
        bus_.write(REG_GYRO_CONFIG_1, (uint8_t)((fs << 1) | (fast ? 0 : 1)));      // DLPFCFG 0, FCHOICE (1 = DLPF on)
        bus_.write(REG_ACCEL_SMPLRT_DIV_1, 0x00); bus_.write(REG_ACCEL_SMPLRT_DIV_2, 0x00);
        bus_.write(REG_ACCEL_CONFIG, (uint8_t)((fs << 1) | (fast ? 0 : 1)));
        bank(0);
        bus_.write(REG_INT_ENABLE_1, 0x01);                  // RAW_DATA_0_RDY_EN
        gyro_hz_ = fast ? 9000.0f : 1125.0f; acc_hz_ = fast ? 4500.0f : 1125.0f;
        ag_delay_ms(20);
        uint8_t who2 = 0;
        if (!bus_.read_u8(REG_WHO_AM_I, who2) || who2 != who) return false;
        AgSample s;
        return read(s);
    }
    WB_FAST bool read(AgSample& out) override {
        uint8_t b[12];
        if (!bus_.read(REG_ACCEL_XOUT_H, b, 12)) return false;
        bool all_ff = true; for (int i = 0; i < 12 && all_ff; i++) all_ff = b[i] == 0xFF;
        if (all_ff) return false;                            // (a missing chip on SPI reads as all ones)
        for (int i = 0; i < 3; i++) {
            out.acc_g[i] = be16(b + 2 * i) * (1.0f / 2048.0f);
            out.gyro_dps[i] = be16(b + 6 + 2 * i) * (1.0f / 16.4f);
        }
        return true;
    }
    bool data_ready() override { uint8_t v; return bus_.read_u8(REG_INT_STATUS_1, v) && (v & 0x01); }
    float gyro_rate_hz() const override { return gyro_hz_; }
    float acc_rate_hz() const override { return acc_hz_; }
    uint32_t max_spi_read_hz() const override { return 7000000u; }      // 7 MHz (datasheet)

private:
    uint8_t want_who() const { return v_ == Variant::ICM20948 ? ICM20948_WHO_AM_I : ICM20649_WHO_AM_I; }
    void bank(uint8_t b) { bus_.write(REG_BANK_SEL, (uint8_t)(b << 4)); }
    static constexpr uint8_t REG_WHO_AM_I = 0x00, REG_USER_CTRL = 0x03, REG_LP_CONFIG = 0x05, REG_PWR_MGMT_1 = 0x06, REG_PWR_MGMT_2 = 0x07;
    static constexpr uint8_t REG_INT_ENABLE_1 = 0x11, REG_INT_STATUS_1 = 0x1A, REG_ACCEL_XOUT_H = 0x2D, REG_BANK_SEL = 0x7F;
    static constexpr uint8_t REG_GYRO_SMPLRT_DIV = 0x00, REG_GYRO_CONFIG_1 = 0x01, REG_ACCEL_SMPLRT_DIV_1 = 0x10, REG_ACCEL_SMPLRT_DIV_2 = 0x11, REG_ACCEL_CONFIG = 0x14;   // bank 2
    AgRegBus& bus_; Variant v_;
    float gyro_hz_ = 1125.0f, acc_hz_ = 1125.0f;
};
