/*
 * This file is part of Windburst Project
 */

#pragma once

#include "mag_driver.h"

// ============================================================================
// AK8963 magnetometer inside the MPU9250/MPU9255, reached through the MPU's I2C master (so it works with the MPU on SPI).
// `bus` is the MPU's own register bus (same SPI device as the IMU driver). Call AFTER the IMU driver's init(): its reset would
// otherwise wipe this setup.
//   MPU: USER_CTRL |= I2C_MST_EN(0x20) | I2C_MST_CTRL=0x0D (400 kHz) | SLV0_ADDR/REG/CTRL/DO | EXT_SENS_DATA_00 @0x49
//   AK : WIA @0x00 == 0x48 | CNTL2 reset | fuse ROM (CNTL1=0x0F) to read ASAX/Y/Z @0x10..0x12 | CNTL1 = 0x16 (16-bit, continuous 100 Hz)
//   steady state: SLV0 re-reads ST1,HXL..HZH,ST2 (8 bytes) from the AK every MPU sample into EXT_SENS_DATA; a read is then a plain
//   register read. Data are accepted when they differ from the previous raw (ST1.DRDY would be caught only 1 time in 10 at a 100 Hz poll).
// Sensitivity: H = raw * ((ASA - 128) / 256 + 1) * 0.15 uT/LSB (0.0015 G).
// AXES: the AK8963 axes differ from the MPU9250 gyro/accel axes (x<->y swapped, z inverted); output is converted into the MPU frame,
// so the mag shares the IMU's mounting alignment.
// ============================================================================
class Ak8963Mpu9250Mag : public MagDriver {
public:
    explicit Ak8963Mpu9250Mag(AgRegBus& mpu_bus) : bus_(mpu_bus) {}
    const char* name() const override { return "AK8963(MPU9250)"; }

    bool init() override {
        ok_ = false; errors_ = 0; frozen_ = 0; have_last_ = false;
        uint8_t who = 0;
        if (!bus_.read_u8(0x75, who) || (who != 0x71 && who != 0x73)) return false;
        if (!bus_.update(0x6A, 0x20, 0x20)) return false;                          // I2C master enable
        bus_.write(0x24, 0x0D);                                                    // 400 kHz
        ag_delay_ms(2);
        uint8_t wia = 0;
        if (!slv_read(0x00, &wia, 1) || wia != 0x48) return false;
        slv_write(0x0B, 0x01); ag_delay_ms(10);                                    // AK soft reset
        slv_write(0x0A, 0x00); ag_delay_ms(10);                                    // power down
        slv_write(0x0A, 0x0F); ag_delay_ms(10);                                    // fuse ROM access
        uint8_t asa[3] = {128, 128, 128};
        if (!slv_read(0x10, asa, 3)) return false;
        slv_write(0x0A, 0x00); ag_delay_ms(10);
        for (int i = 0; i < 3; i++) adj_[i] = ((float)asa[i] - 128.0f) / 256.0f + 1.0f;
        slv_write(0x0A, 0x16); ag_delay_ms(10);                                    // 16-bit, continuous measurement mode 2 (100 Hz)
        bus_.write(0x25, 0x8C); bus_.write(0x26, 0x02); bus_.write(0x27, 0x88);    // SLV0: continuous 8-byte read from ST1
        ag_delay_ms(20);
        ok_ = true;
        return true;
    }
    bool update(uint32_t now_us, MagSample& out) override {
        if (!ok_) return false;
        if (now_us - t_poll_ < 5000) return false;
        t_poll_ = now_us;
        uint8_t b[8];
        if (!bus_.read(0x49, b, 8)) { if (errors_ < 255) errors_++; return false; }
        errors_ = 0;
        if (b[7] & 0x08) return false;                                             // ST2.HOFL: sensor overflow
        const int16_t rx = (int16_t)((b[2] << 8) | b[1]), ry = (int16_t)((b[4] << 8) | b[3]), rz = (int16_t)((b[6] << 8) | b[5]);
        if (have_last_ && rx == lx_ && ry == ly_ && rz == lz_) { if (frozen_ < 255) frozen_++; return false; }
        frozen_ = 0; lx_ = rx; ly_ = ry; lz_ = rz; have_last_ = true;
        const float k = 0.0015f;                                                   // 0.15 uT/LSB
        const float ax = rx * adj_[0] * k, ay = ry * adj_[1] * k, az = rz * adj_[2] * k;
        out.gauss[0] = ay; out.gauss[1] = ax; out.gauss[2] = -az;                  // AK8963 frame -> MPU9250 gyro/accel frame
        return true;
    }
    float rate_hz() const override { return 100.0f; }
    bool healthy() const override { return ok_ && errors_ < 10 && frozen_ < 100; }

private:
    bool slv_write(uint8_t reg, uint8_t val) {
        bus_.write(0x25, 0x0C); bus_.write(0x26, reg); bus_.write(0x63, val); bus_.write(0x27, 0x81);
        ag_delay_ms(5); bus_.write(0x27, 0x00); return true;
    }
    bool slv_read(uint8_t reg, uint8_t* buf, uint8_t n) {
        bus_.write(0x25, 0x8C); bus_.write(0x26, reg); bus_.write(0x27, (uint8_t)(0x80 | (n & 0x0F)));
        ag_delay_ms(8);
        const bool ok = bus_.read(0x49, buf, n);
        bus_.write(0x27, 0x00);
        return ok;
    }
    AgRegBus& bus_;
    float adj_[3] = {1, 1, 1}; bool ok_ = false, have_last_ = false; uint8_t errors_ = 0, frozen_ = 0; int16_t lx_ = 0, ly_ = 0, lz_ = 0; uint32_t t_poll_ = 0;
};
