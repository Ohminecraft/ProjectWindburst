/*
 * This file is part of Windburst Project
 */

#pragma once
#include "../../config/fast_code.h"

#include "acc_gyro_regbus.h"
#include "acc_gyro_types.h"

// ============================================================================
// MPU6500 (I2C/SPI), MPU9250/9255 (SPI, accel+gyro only), ICM20601, ICM20602: same register map as the MPU6000 family with the
// MPU6500 reset sequence (Betaflight accgyro_mpu6500.c + accgyro_spi_mpu6500.c).
// Gyro +-2000 dps (16.384 LSB/dps), accel +-16 g (2048 LSB/g). ICM20601 has a different FS_SEL numbering (0=500dps..3=4000dps,
// accel 0=4g..3=32g) so its 2000 dps / 16 g codes are ICM_HIGH_RANGE_* == 2 (same scale factors afterwards).
// Rates: as Mpu6050AccGyro (SPI 8 kHz/(div+1) with DLPF off, I2C capped at 1 kHz with 184 Hz DLPF).
// ============================================================================
class Mpu6500Icm2060xAccGyro : public AccGyroDriver {
public:
    enum class Variant : uint8_t { MPU6500, MPU9250, ICM20601, ICM20602, ICM20602_FAKE };
    Mpu6500Icm2060xAccGyro(AgRegBus& bus, Variant v) : bus_(bus), v_(v) {}

    const char* name() const override {
        switch (v_) { case Variant::MPU6500: return "MPU6500"; case Variant::MPU9250: return "MPU9250";
                      case Variant::ICM20601: return "ICM20601"; case Variant::ICM20602_FAKE: return "ICM20602_FAKE"; default: return "ICM20602"; }
    }

    bool init(const AgInitConfig& cfg) override {
        const bool spi = bus_.is_spi();
        // WHO_AM_I first: on SPI the chip must answer before we start poking it
        uint8_t who = 0;
        if (!bus_.read_u8(MPU_WHO_AM_I_CMD, who) || !who_ok(who)) return false;

        uint8_t gyro_fs = INV_FSR_2000DPS, acc_fs = INV_FSR_16G;
        if (v_ == Variant::ICM20601) { gyro_fs = ICM_HIGH_RANGE_FSR_2000DPS; acc_fs = ICM_HIGH_RANGE_FSR_16G; }

        bus_.write(MPU_PWR_MGMT_1, 0x80);                    // reset
        ag_delay_ms(100);
        bus_.write(MPU_SIGNAL_PATH_RESET, 0x07);
        ag_delay_ms(100);
        bus_.write(MPU_PWR_MGMT_1, 0x00);
        ag_delay_ms(100);
        bus_.write(MPU_PWR_MGMT_1, INV_CLK_PLL);
        ag_delay_ms(15);
        if (spi) { bus_.write(MPU_USER_CTRL, 0x10); ag_delay_ms(15); }  // I2C_IF_DIS (BF does it after config; before is safe and stops I2C glitches)
        bus_.write(MPU_GYRO_CONFIG, (uint8_t)(gyro_fs << 3));   // FCHOICE_B = 0 => DLPF path enabled
        ag_delay_ms(15);
        bus_.write(MPU_ACCEL_CONFIG, (uint8_t)(acc_fs << 3));
        ag_delay_ms(15);

        uint8_t dlpf, div;
        if (spi && cfg.gyro_odr_hz > 1000) {
            dlpf = 0;
            int d = (int)(8000.0f / (float)cfg.gyro_odr_hz + 0.5f) - 1;
            div = (uint8_t)(d < 0 ? 0 : (d > 7 ? 7 : d));
            gyro_hz_ = 8000.0f / (float)(div + 1);
        } else { dlpf = 1; div = 0; gyro_hz_ = 1000.0f; }
        bus_.write(MPU_CONFIG, dlpf);
        ag_delay_ms(15);
        bus_.write(MPU_SMPLRT_DIV, div);
        ag_delay_ms(15);
        bus_.write(MPU_INT_PIN_CFG, 0x10);                   // INT_ANYRD_2CLEAR
        ag_delay_ms(15);
        bus_.write(MPU_INT_ENABLE, 0x01);                    // RAW_RDY_EN
        ag_delay_ms(15);
        if (spi) ag_delay_ms(100);

        uint8_t who2 = 0;                                    // still alive after the reset dance?
        if (!bus_.read_u8(MPU_WHO_AM_I_CMD, who2) || who2 != who) return false;
        AgSample s;
        return read(s);
    }

    WB_FAST bool read(AgSample& out) override {
        uint8_t b[14];
        if (!bus_.read(MPU_ACCEL_XOUT_H, b, 14)) return false;
        if (b[0] == 0xFF && b[1] == 0xFF && b[8] == 0xFF && b[9] == 0xFF && b[10] == 0xFF && b[11] == 0xFF) return false;
        for (int i = 0; i < 3; i++) {
            out.acc_g[i] = be16(b + 2 * i) * (1.0f / 2048.0f);
            out.gyro_dps[i] = be16(b + 8 + 2 * i) * (1.0f / 16.384f);
        }
        return true;
    }

    bool data_ready() override { uint8_t v; return bus_.read_u8(MPU_INT_STATUS, v) && (v & 0x01); }
    float gyro_rate_hz() const override { return gyro_hz_; }
    float acc_rate_hz() const override { return 1000.0f; }

private:
    bool who_ok(uint8_t w) const {
        switch (v_) {
        case Variant::MPU6500: return w == MPU6500_WHO_I_AM;
        case Variant::MPU9250: return w == MPU9250_WHO_I_AM || w == MPU9255_WHO_I_AM;
        case Variant::ICM20601: return w == ICM20601_WHO_AM_I;
        case Variant::ICM20602_FAKE: return w == ICM20602_FAKE_WHO_AM_I;
        default: return w == ICM20602_WHO_AM_I;
        }
    }
    static constexpr uint8_t MPU_SMPLRT_DIV = 0x19, MPU_CONFIG = 0x1A, MPU_GYRO_CONFIG = 0x1B, MPU_ACCEL_CONFIG = 0x1C;
    static constexpr uint8_t MPU_INT_PIN_CFG = 0x37, MPU_INT_ENABLE = 0x38, MPU_INT_STATUS = 0x3A, MPU_ACCEL_XOUT_H = 0x3B;
    static constexpr uint8_t MPU_SIGNAL_PATH_RESET = 0x68, MPU_USER_CTRL = 0x6A, MPU_PWR_MGMT_1 = 0x6B;
    AgRegBus& bus_;
    Variant v_;
    float gyro_hz_ = 1000.0f;
};
