/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>
#include "../bus/bus_i2c.h"
#include "../bus/reg_bus_arduino.h"
#include "../acc_gyro/acc_gyro_util.h"
#include "mag_driver.h"
#include "hmc5883l_mag.h"
#include "qmc5883_mag.h"
#include "ak8963_mpu9250_mag.h"
#include "mag_processor.h"
#include "../../config/hardware_config.h"

// ============================================================================
// MagSensor (the "Magnetometer" tab): HMC5883L (0x1E), QMC5883L (0x0D), QMC5883P (0x2C) on I2C, or the AK8963 inside an MPU9250 that
// the IMU driver already found on SPI.
//
//   MagSensor mag;
//   mag.attachI2C(sensorBus);                         // share the sensor I2C bus (baro, lidar, compass) - or setI2cPins()
//   mag.attachImuBus(imu.regBus(), imu.detected());   // optional: lets AUTO fall back to the MPU9250's internal AK8963
//   if (!mag.MagInit()) Serial.println(mag.failReason());
//   mag.setAlignment(CW90_DEG);  mag.setCalibration(saved);          // from settings
//   loop:  if (mag.update(micros(), millis())) {...}
//          in.mag_body = mag.board(); in.mag_valid = mag.readyForFusion(millis());      // -> FlightCore
//   calibrate (app button):  mag.startCalibration(); ... mag.calibrationProgress() ... mag.finishCalibration() -> store mag.calibration()
// ============================================================================
class MagSensor {
public:
    enum class Model : uint8_t { AUTO, HMC5883L, QMC5883L, QMC5883P, AK8963_MPU9250 };

    void attachI2C(I2CBus& shared) { shared_ = &shared; }
    void attachImuBus(AgRegBus* imu_bus, IMUAccGyroDevice imu_device) { imu_bus_ = imu_bus; imu_dev_ = imu_device; }
    void setEnabled(bool on) { enabled_ = on; }                        // app switch "magnetometer on/off"
    void setI2cPins(uint8_t sda, uint8_t scl) { pins_[0] = sda; pins_[1] = scl; pins_set_ = true; }
    void setModel(Model m) { model_ = m; }
    void setAlignment(AccGyroAlign d, float roll = 0, float pitch = 0, float yaw = 0) { proc_.set_alignment(d, roll, pitch, yaw); }
    void setCalibration(const MagCal& c) { proc_.set_calibration(c); }

    bool MagInit() {
        drv_ = nullptr; fail_ = ""; detected_ = Model::AUTO;
        if (!enabled_) { fail_ = "magnetometer disabled"; return false; }
        const bool want_i2c = model_ == Model::AUTO || (model_ != Model::AK8963_MPU9250);
        if (want_i2c && (shared_ || pins_ok())) {
            I2CBus* bus = shared_;
            if (!bus) {
                I2CBus_config_t c; c.bus_clock_speed = 400000; c.ioSdaGpio = pins_[0]; c.ioSclGpio = pins_[1]; c.addr = 0x1E;
                own_.I2CInit(c); bus = &own_;
            }
            if (try_i2c(*bus, 0x1E, Model::HMC5883L, hmc_)) return true;
            if (try_i2c(*bus, 0x0D, Model::QMC5883L, qmcl_)) return true;
            if (try_i2c(*bus, 0x2C, Model::QMC5883P, qmcp_)) return true;
        }
        if ((model_ == Model::AUTO || model_ == Model::AK8963_MPU9250) && imu_bus_ && imu_dev_ == MPU9520_SPI) {
            proxy_.set(imu_bus_);
            if (ak_.init()) { drv_ = &ak_; detected_ = Model::AK8963_MPU9250; proc_.cfg = MagProcessorConfig(); return true; }
        }
        if (want_i2c && !shared_ && !pins_ok() && !(imu_bus_ && imu_dev_ == MPU9520_SPI)) fail_ = "I2C pins not set (app: Pins tab, or hardware_config.h)";
        else fail_ = "no magnetometer answered (HMC5883L 0x1E, QMC5883L 0x0D, QMC5883P 0x2C, MPU9250 AK8963)";
        return false;
    }

    // Call often. Returns true when a NEW sample was taken.
    bool update(uint32_t now_us, uint32_t now_ms) {
        if (!drv_) return false;
        MagSample s;
        if (!drv_->update(now_us, s)) return false;
        for (int k = 0; k < 3; k++) raw_[k] = s.gauss[k];
        if (cal_active_) cal_.add(raw_);
        proc_.update(raw_, now_ms);
        return true;
    }

    // ---- calibration (driven by the app: "calibrate" button, rotate the craft through every orientation) ----
    void startCalibration() { cal_.begin(); cal_active_ = true; }
    void cancelCalibration() { cal_active_ = false; }
    bool calibrating() const { return cal_active_; }
    float calibrationProgress() const { return cal_.progress(); }
    int calibrationSamples() const { return cal_.count(); }
    const float* rawGauss() const { return raw_; }                      // for the app's live 3D point cloud
    // Fit now. On success the new calibration is applied immediately and returned via calibration().
    MagCalResult finishCalibration() {
        MagCal c; const MagCalResult r = cal_.solve(c);
        if (r == MagCalResult::OK) { proc_.set_calibration(c); cal_active_ = false; }
        return r;
    }

    bool available() const { return drv_ != nullptr; }
    bool healthy() const { return drv_ && drv_->healthy(); }
    bool calibrated() const { return proc_.calibrated(); }
    bool valid(uint32_t now_ms) const { return drv_ && drv_->healthy() && proc_.valid(now_ms); }
    bool readyForFusion(uint32_t now_ms) const { return drv_ && drv_->healthy() && proc_.ready_for_fusion(now_ms); }
    const float* board() const { return proc_.board(); }                // calibrated, aligned, FRD (gauss)
    float strength() const { return proc_.strength(); }
    float interference() const { return proc_.interference(); }
    const MagCal& calibration() const { return proc_.calibration(); }
    float rateHz() const { return drv_ ? drv_->rate_hz() : 0.0f; }
    const char* name() const { return drv_ ? drv_->name() : "none"; }
    Model detected() const { return detected_; }
    const char* failReason() const { return fail_; }
    MagProcessor& processor() { return proc_; }

private:
    bool pins_ok() { if (!pins_set_) { hw_.getI2CMagSensorPins(pins_); pins_set_ = true; } return pins_[0] != GPIO_UNUSED && pins_[1] != GPIO_UNUSED; }
    template <class D> bool try_i2c(I2CBus& bus, uint8_t addr, Model m, D& d) {
        if (model_ != Model::AUTO && model_ != m) return false;
        i2c_bus_.bind(bus, addr);
        proxy_.set(&i2c_bus_);
        if (!d.init()) return false;
        drv_ = &d; detected_ = m; return true;
    }
    I2CBus* shared_ = nullptr; I2CBus own_{I2C_DEVICE_1};
    AgRegBus* imu_bus_ = nullptr; IMUAccGyroDevice imu_dev_ = NONE;
    AgI2CRegBus i2c_bus_{own_};
    AgBusProxy proxy_;
    Hmc5883lMag hmc_{proxy_};
    Qmc5883Mag qmcl_{proxy_, Qmc5883Mag::Variant::L};
    Qmc5883Mag qmcp_{proxy_, Qmc5883Mag::Variant::P};
    Ak8963Mpu9250Mag ak_{proxy_};
    MagDriver* drv_ = nullptr;
    MagProcessor proc_;
    MagCalibrator cal_;
    float raw_[3] = {0, 0, 0};
    HardwareConfig hw_;
    uint8_t pins_[2] = {GPIO_UNUSED, GPIO_UNUSED};
    bool pins_set_ = false, enabled_ = true, cal_active_ = false;
    Model model_ = Model::AUTO, detected_ = Model::AUTO;
    const char* fail_ = "";
};
