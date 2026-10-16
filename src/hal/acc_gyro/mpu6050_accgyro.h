/*
 * This file is part of Windburst Project
 */

#pragma once
#include "../../config/fast_code.h"

#include "acc_gyro_regbus.h"
#include "acc_gyro_types.h"

// ============================================================================
// MPU6050 (I2C) and MPU6000 (SPI): identical register map, different bus (WHO_AM_I is 0x68 for both, the bus tells them apart).
// Sequence follows Betaflight accgyro_spi_mpu6000.c. Gyro +-2000 dps (16.384 LSB/dps), accel +-16 g (2048 LSB/g).
// Rates: SPI  -> gyro 8 kHz / (SMPLRT_DIV+1) with the chip DLPF off (DLPF_CFG=0), accel updates at 1 kHz.
//        I2C  -> a 14-byte read takes ~0.4 ms at 400 kHz, so the rate is capped at 1 kHz with the 188 Hz DLPF (DLPF_CFG=1).
// ============================================================================
class Mpu6050AccGyro : public AccGyroDriver {
public:
    Mpu6050AccGyro(AgRegBus& bus) : bus_(bus) {}

    const char* name() const override { return bus_.is_spi() ? "MPU6000" : "MPU6050"; }

    bool init(const AgInitConfig& cfg) override {
        const bool spi = bus_.is_spi();
        bus_.write(MPU_PWR_MGMT_1, 0x80);                    // device reset
        ag_delay_ms(100);
        bus_.write(MPU_SIGNAL_PATH_RESET, 0x07);             // gyro | accel | temp signal paths
        ag_delay_ms(100);

        uint8_t who = 0;
        if (!bus_.read_u8(MPU_WHO_AM_I_CMD, who) || who != MPU6050_WHO_I_AM) return false;

        bus_.write(MPU_PWR_MGMT_1, 0x03);                    // clock: PLL with Z gyro reference (BF), also wakes the chip
        ag_delay_us(15);
        if (spi) { bus_.write(MPU_USER_CTRL, 0x10); ag_delay_us(15); }   // disable the I2C interface
        bus_.write(MPU_PWR_MGMT_2, 0x00);
        ag_delay_us(15);

        uint8_t dlpf, div;
        if (spi && cfg.gyro_odr_hz > 1000) {                 // 8 kHz base, chip DLPF off
            dlpf = 0;
            int d = (int)(8000.0f / (float)cfg.gyro_odr_hz + 0.5f) - 1;
            div = (uint8_t)(d < 0 ? 0 : (d > 7 ? 7 : d));
            gyro_hz_ = 8000.0f / (float)(div + 1);
        } else {                                             // 1 kHz base with 188 Hz DLPF
            dlpf = 1; div = 0; gyro_hz_ = 1000.0f;
        }
        bus_.write(MPU_SMPLRT_DIV, div);
        ag_delay_us(15);
        bus_.write(MPU_GYRO_CONFIG, INV_FSR_2000DPS << 3);
        ag_delay_us(15);
        bus_.write(MPU_ACCEL_CONFIG, INV_FSR_16G << 3);
        ag_delay_us(15);
        bus_.write(MPU_CONFIG, dlpf);
        ag_delay_us(15);
        bus_.write(MPU_INT_PIN_CFG, 0x10);                   // INT_ANYRD_2CLEAR: INT clears on any read
        ag_delay_us(15);
        bus_.write(MPU_INT_ENABLE, 0x01);                    // DATA_RDY
        ag_delay_us(15);

        // a sleeping / dead chip reads all 0xFF (BF's "-1,-1" check)
        AgSample s;
        if (!read(s)) return false;
        return true;
    }

    WB_FAST bool read(AgSample& out) override {
        uint8_t b[14];
        if (!bus_.read(MPU_ACCEL_XOUT_H, b, 14)) return false;      // accel(6) temp(2) gyro(6), big-endian
        if (b[0] == 0xFF && b[1] == 0xFF && b[8] == 0xFF && b[9] == 0xFF && b[10] == 0xFF && b[11] == 0xFF) return false; // floating MISO
        for (int i = 0; i < 3; i++) {
            out.acc_g[i] = be16(b + 2 * i) * (1.0f / 2048.0f);
            out.gyro_dps[i] = be16(b + 8 + 2 * i) * (1.0f / 16.384f);
        }
        return true;
    }

    bool data_ready() override { uint8_t v; return bus_.read_u8(MPU_INT_STATUS, v) && (v & 0x01); }
    float gyro_rate_hz() const override { return gyro_hz_; }
    uint32_t max_spi_read_hz() const override { return 20000000; }         // MPU6000: 20 MHz for the sensor registers (1 MHz for the others: writes stay slow)
    float acc_rate_hz() const override { return 1000.0f; }

private:
    static constexpr uint8_t MPU_SMPLRT_DIV = 0x19, MPU_CONFIG = 0x1A, MPU_GYRO_CONFIG = 0x1B, MPU_ACCEL_CONFIG = 0x1C;
    static constexpr uint8_t MPU_INT_PIN_CFG = 0x37, MPU_INT_ENABLE = 0x38, MPU_INT_STATUS = 0x3A, MPU_ACCEL_XOUT_H = 0x3B;
    static constexpr uint8_t MPU_SIGNAL_PATH_RESET = 0x68, MPU_USER_CTRL = 0x6A, MPU_PWR_MGMT_1 = 0x6B, MPU_PWR_MGMT_2 = 0x6C;
    AgRegBus& bus_;
    float gyro_hz_ = 1000.0f;
};
