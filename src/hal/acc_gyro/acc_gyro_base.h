/*
 * This file is part of Windburst Project
 */

#pragma once
#include "../../config/fast_code.h"

#include <stdint.h>
#include <math.h>
#include "../bus/bus_i2c.h"
#include "../bus/bus_spi.h"

#include "../bus/reg_bus_arduino.h"
#include "acc_gyro_regbus.h"
#include "acc_gyro_types.h"
#include "mpu6050_accgyro.h"
#include "mpu6500_icm2060x_accgyro.h"
#include "icm426xx_accgyro.h"
#include "icm456xx_accgyro.h"
#include "bmi160_accgyro.h"
#include "bmi270_accgyro.h"
#include "acc_gyro_util.h"
#include "acc_calibration.h"

#include "../../config/hardware_config.h"

// ---------------------------------------------------------------------------------------------------------------------
// IMUAccGyro: detect -> init -> read. Output is in the BOARD frame (FRD), physical units (g, deg/s), gyro bias removed.
// ---------------------------------------------------------------------------------------------------------------------
class IMUAccGyro {
public:
    // Same entry point as before. On failure failReason() says why.
    bool IMUInit(bool use_spi_imu) { return IMUInit(use_spi_imu, AgInitConfig()); }

    bool IMUInit(bool use_spi_imu, const AgInitConfig& cfg) {
        destroy();
        imu_detected = NONE; fail_ = "";
        if (!use_spi_imu) {
            uint8_t pins[2];
            if (pins_i2c_set_) { pins[0] = pins_i2c_[0]; pins[1] = pins_i2c_[1]; } else hw_.getI2CAccGyroSensorPins(pins);
            if (pins[0] == GPIO_UNUSED || pins[1] == GPIO_UNUSED) { fail_ = "I2C pins not set (app: Pins tab, or hardware_config.h)"; return false; }
            I2CBus_config_t config;
            config.bus_clock_speed = 400000;
            config.ioSdaGpio = pins[0];
            config.ioSclGpio = pins[1];
            const uint8_t addrs[2] = {0x68, 0x69};              // AD0 low / high
            for (uint8_t a : addrs) {
                config.addr = a;
                i2c_.I2CInit(config);
                uint8_t who = 0;
                if (i2c_.I2CReadRegister(MPU_WHO_AM_I_CMD, &who, 1)) { imu_detected = accgyro::classify_i2c(who); if (imu_detected != NONE) break; }
            }
            if (imu_detected == NONE) { fail_ = "no MPU6050/MPU6500 answered on I2C 0x68/0x69"; return false; }
            proxy_.set(&i2c_bus_);
        } else {
            uint8_t pins[4];
            if (pins_spi_set_) { for (int i = 0; i < 4; i++) pins[i] = pins_spi_[i]; } else hw_.getSPIAccGyroSensorPins(pins);
            if (pins[0] == GPIO_UNUSED || pins[1] == GPIO_UNUSED || pins[2] == GPIO_UNUSED || pins[3] == GPIO_UNUSED) { fail_ = "SPI pins not set (app: Pins tab, or hardware_config.h)"; return false; }
            SPIBus_config_t config;
            config.ioCsGpio = pins[0];
            config.ioSdoGpio = pins[1];
            config.ioSdiGpio = pins[2];
            config.ioSckGpio = pins[3];
            if (!spi_.SPIInit(config)) { fail_ = "SPI bus init failed"; return false; }
            proxy_.set(&spi_bus_);
            for (int attempt = 0; attempt < 3 && imu_detected == NONE; attempt++) {
                delay(5);
                uint8_t d2[2] = {0, 0}, r00 = 0, r72 = 0, r75 = 0;
                spi_.SPIReadRegisterMask(0x00, r00);             // first access = CS edge that switches BMI16x/27x into SPI mode
                delay(2);
                spi_.SPIReadRegisterMask(0x00, d2, 2);
                spi_.SPIReadRegisterMask(0x00, r00);
                spi_.SPIReadRegisterMask(0x72, r72);
                spi_.SPIReadRegisterMask(0x75, r75);
                imu_detected = accgyro::classify_spi(d2[1], r00, r72, r75);
            }
            if (imu_detected == NONE) { fail_ = "no known IMU answered on SPI (check CS/MISO/MOSI/SCK pins and power)"; return false; }
        }

        if (!create_driver()) { fail_ = "unsupported device"; return false; }
        if (!drv_->init(cfg)) { fail_ = "WHO_AM_I / init failed after detection"; destroy(); return false; }
        return sanity_check();
    }

    // Optional runtime pin override (instead of hardware_config.h), e.g. pins loaded from settings. Call before IMUInit().
    void setI2cPins(uint8_t sda, uint8_t scl) { pins_i2c_[0] = sda; pins_i2c_[1] = scl; pins_i2c_set_ = true; }
    void setSpiPins(uint8_t cs, uint8_t miso, uint8_t mosi, uint8_t sck) { pins_spi_[0] = cs; pins_spi_[1] = miso; pins_spi_[2] = mosi; pins_spi_[3] = sck; pins_spi_set_ = true; }

    // Board mounting. discrete = how the chip sits on the PCB, fine = extra roll/pitch/yaw trim of the whole board (BF board_alignment).
    void setAlignment(AccGyroAlign discrete, float roll_deg = 0, float pitch_deg = 0, float yaw_deg = 0) {
        align_ = discrete;
        fine_ = (roll_deg != 0 || pitch_deg != 0 || yaw_deg != 0);
        if (fine_) accgyro::build_rotation(rot_, roll_deg, pitch_deg, yaw_deg);
    }

    // Average N still samples into the gyro bias. false = drone was moving (or bus errors); bias left unchanged.
    bool calibrateGyro(uint16_t samples = 500) {
        if (!drv_) return false;
        double s[3] = {0, 0, 0}, s2[3] = {0, 0, 0}; uint16_t ok = 0;
        const uint32_t wait_us = (uint32_t)(1e6f / drv_->gyro_rate_hz());
        for (uint16_t i = 0; i < samples; i++) {
            AgSample r;
            if (drv_->read(r)) { for (int k = 0; k < 3; k++) { s[k] += r.gyro_dps[k]; s2[k] += (double)r.gyro_dps[k] * r.gyro_dps[k]; } ok++; }
            ag_delay_us(wait_us < 2000 ? 2000 : wait_us);
        }
        float bias[3];
        if (!accgyro::bias_from_sums(s, s2, ok, samples, bias)) return false;
        for (int k = 0; k < 3; k++) bias_[k] = bias[k];
        calibrated_ = true;
        return true;
    }

    // Non-blocking gyro calibration for a running loop: the next `samples` read() calls are averaged (use ~1 s worth).
    // gyroCalState(): RUNNING -> DONE (new bias in use) or FAILED (the board moved; the old bias is kept).
    void startGyroCalibration(uint16_t samples) { gcal_.begin(samples); }
    CalState gyroCalState() const { return gcal_.state(); }
    uint8_t gyroCalProgress() const { return gcal_.progress(); }

    // Accelerometer offset/scale from the 6-face calibration (AccCalibrator), applied in the chip frame.
    void setAccCalibration(const AccCal& c) { acc_cal_ = c; }
    const AccCal& accCalibration() const { return acc_cal_; }
    // The last accelerometer sample exactly as the chip delivered it (g, chip frame, no calibration, no alignment): input of AccCalibrator.
    const float* rawAcc() const { return raw_acc_; }
    // The alignment currently in use (level trim needs it).
    AccGyroAlign alignment() const { return align_; }

    // read() fails once this many consecutive samples were bit-identical (default 32 = 8 ms at 4 kHz). 0 disables the check.
    void setFrozenLimit(uint16_t samples) { frozen_limit_ = samples; }
    bool frozen() const { return frozen_limit_ && repeats_ >= frozen_limit_; }

    // One sample in the board frame. false on bus error or a frozen chip (out untouched).
    WB_FAST bool read(AgSample& out) {
        if (!drv_) return false;
        AgSample r;
        if (!drv_->read(r)) return false;
        // A live MEMS chip has noise in its last bits: six axes that repeat bit for bit, sample after sample, mean the chip or
        // the bus is dead (a broken MISO wire reads as constant 0xFF / 0x00 - which decodes to a perfectly plausible "not moving").
        bool same = true;
        for (int k = 0; k < 3; k++) { if (r.acc_g[k] != prev_.acc_g[k] || r.gyro_dps[k] != prev_.gyro_dps[k]) same = false; }
        prev_ = r;
        if (!same) repeats_ = 0; else if (repeats_ < 0xFFFF) repeats_++;
        if (frozen_limit_ && repeats_ >= frozen_limit_) return false;
        for (int k = 0; k < 3; k++) raw_acc_[k] = r.acc_g[k];
        if (gcal_.running() && gcal_.feed(r.gyro_dps, bias_)) calibrated_ = true;      // non-blocking calibration: the bias is replaced only on success
        for (int k = 0; k < 3; k++) r.gyro_dps[k] -= bias_[k];
        acc_apply_cal(r.acc_g, acc_cal_);                                              // chip frame, before the board alignment
        accgyro::align_chip(r.acc_g, align_);
        accgyro::align_chip(r.gyro_dps, align_);
        if (fine_) { accgyro::apply_rotation(r.acc_g, rot_); accgyro::apply_rotation(r.gyro_dps, rot_); }
        out = r;
        return true;
    }

    // The IMU's register bus (same device the driver uses): lets a sensor living behind the IMU (MPU9250's AK8963) share it.
    AgRegBus* regBus() { return drv_ ? &proxy_ : nullptr; }
    bool dataReady() { return drv_ && drv_->data_ready(); }
    bool available() const { return drv_ != nullptr; }
    bool gyroCalibrated() const { return calibrated_; }
    const char* name() const { return drv_ ? drv_->name() : "none"; }
    IMUAccGyroDevice detected() const { return imu_detected; }
    float gyroRateHz() const { return drv_ ? drv_->gyro_rate_hz() : 0.0f; }
    const char* failReason() const { return fail_; }

private:
    // A live chip on a still board reads |acc| ~ 1 g. Catches wrong scale, wrong endianness, dead/floating bus.
    bool sanity_check() {
        float nmin = 1e9f, nmax = 0; int good = 0;
        for (int i = 0; i < 20; i++) {
            AgSample s;
            if (drv_->read(s)) {
                const float n = sqrtf(s.acc_g[0] * s.acc_g[0] + s.acc_g[1] * s.acc_g[1] + s.acc_g[2] * s.acc_g[2]);
                if (n < nmin) nmin = n;
                if (n > nmax) nmax = n;
                good++;
            }
            ag_delay_us(2000);
        }
        if (good < 10) { fail_ = "IMU detected but reads keep failing"; destroy(); return false; }
        if (nmin < 0.6f || nmax > 1.6f) { fail_ = "IMU accel magnitude is not ~1 g (board moving, or wrong scale)"; destroy(); return false; }
        return true;
    }

    bool create_driver() {
        switch (imu_detected) {
        case MPU6050_I2C: case MPU6000_SPI: drv_ = &d_mpu6050_; break;
        case MPU6500_I2C: case MPU6500_SPI: drv_ = &d_mpu6500_; break;
        case MPU9520_SPI:                   drv_ = &d_mpu9250_; break;
        case ICM20601_SPI:                  drv_ = &d_icm20601_; break;
        case ICM20602_SPI:                  drv_ = &d_icm20602_; break;
        case ICM20602_FAKE_SPI:             drv_ = &d_icm20602_fake_; break;
        case ICM42622P_SPI:                 drv_ = &d_icm42622_; break;
        case ICM42686P_SPI:                 drv_ = &d_icm42686_; break;
        case ICM42688P_SPI:                 drv_ = &d_icm42688_; break;
        case ICM45605_SPI:                  drv_ = &d_icm45605_; break;
        case ICM45686_SPI:                  drv_ = &d_icm45686_; break;
        case BMI160_SPI:                    drv_ = &d_bmi160_; break;
        case BMI270_SPI:                    drv_ = &d_bmi270_; break;
        default: return false;
        }
        return true;
    }
    void destroy() { drv_ = nullptr; }

    // One instance of every driver (a few dozen bytes each, no heap, no placement new); drv_ points at the detected one.
    AgBusProxy proxy_;
    Mpu6050AccGyro d_mpu6050_{proxy_};
    Mpu6500Icm2060xAccGyro d_mpu6500_{proxy_, Mpu6500Icm2060xAccGyro::Variant::MPU6500};
    Mpu6500Icm2060xAccGyro d_mpu9250_{proxy_, Mpu6500Icm2060xAccGyro::Variant::MPU9250};
    Mpu6500Icm2060xAccGyro d_icm20601_{proxy_, Mpu6500Icm2060xAccGyro::Variant::ICM20601};
    Mpu6500Icm2060xAccGyro d_icm20602_{proxy_, Mpu6500Icm2060xAccGyro::Variant::ICM20602};
    Mpu6500Icm2060xAccGyro d_icm20602_fake_{proxy_, Mpu6500Icm2060xAccGyro::Variant::ICM20602_FAKE};
    Icm426xxAccGyro d_icm42622_{proxy_, Icm426xxAccGyro::Variant::ICM42622P};
    Icm426xxAccGyro d_icm42686_{proxy_, Icm426xxAccGyro::Variant::ICM42686P};
    Icm426xxAccGyro d_icm42688_{proxy_, Icm426xxAccGyro::Variant::ICM42688P};
    Icm456xxAccGyro d_icm45605_{proxy_, Icm456xxAccGyro::Variant::ICM45605};
    Icm456xxAccGyro d_icm45686_{proxy_, Icm456xxAccGyro::Variant::ICM45686};
    Bmi160AccGyro d_bmi160_{proxy_};
    Bmi270AccGyro d_bmi270_{proxy_};
    AccGyroDriver* drv_ = nullptr;

    HardwareConfig hw_;
    uint8_t pins_i2c_[2] = {GPIO_UNUSED, GPIO_UNUSED}, pins_spi_[4] = {GPIO_UNUSED, GPIO_UNUSED, GPIO_UNUSED, GPIO_UNUSED};
    bool pins_i2c_set_ = false, pins_spi_set_ = false;
    I2CBus i2c_{I2C_DEVICE_0};
    SPIBus spi_{SPI_DEVICE_0};
    AgI2CRegBus i2c_bus_{i2c_};
    AgSpiRegBus spi_bus_{spi_};

    IMUAccGyroDevice imu_detected = NONE;
    AccGyroAlign align_ = CW0_DEG;
    bool fine_ = false, calibrated_ = false;
    float rot_[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    float bias_[3] = {0, 0, 0};
    float raw_acc_[3] = {0, 0, 0};
    AgSample prev_ = {{0, 0, 0}, {0, 0, 0}}; uint16_t repeats_ = 0, frozen_limit_ = 32;
    AccCal acc_cal_;
    GyroCalibration gcal_;
    const char* fail_ = "";
};
