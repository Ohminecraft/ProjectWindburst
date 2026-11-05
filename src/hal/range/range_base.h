/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>
#include "../bus/bus_i2c.h"
#include "range_driver.h"
#include "range_height.h"
#include "range_pololu.h"
#include "../../config/hardware_config.h"

// ============================================================================
// RangeFinder (the "Sonar" tab): VL53L1X / VL53L0X time-of-flight lidar on I2C 0x29.
//
//   RangeFinder sonar;
//   sonar.attachI2C(bus);                 // share the sensor I2C bus with the barometer / compass - or setI2cPins() for its own bus
//   if (!sonar.RangeInit()) Serial.println(sonar.failReason());
//   loop:   if (sonar.update(micros(), millis(), att.up_cos(), armed)) { kf.cfg.r_range = sonar.sigmaM(); kf.update_range(sonar.heightM()); }
//           arm.range_ok = sonar.valid(millis());
//
// Detection tries the VL53L1X first (model id 0xEACC), then the VL53L0X (0xEE); each library checks its own id and refuses the other chip.
// Altitude Hold does NOT depend on this sensor: it only improves accuracy near the ground (the Kalman filter ignores it outside the
// sensor's range / when tilted / when invalid) - the baro keeps working alone.
// ============================================================================
class RangeFinder {
public:
    enum class Model : uint8_t { AUTO, VL53L1X, VL53L0X, MSP };       // MSP: the lidar inside an MSP optical-flow sensor (flight_app hands its driver over with useDriver())

    void attachI2C(I2CBus& shared) { shared_ = &shared; }
    void setEnabled(bool on) { enabled_ = on; }                            // app switch "sonar on/off"
    void setI2cPins(uint8_t sda, uint8_t scl) { pins_[0] = sda; pins_[1] = scl; pins_set_ = true; }
    void setModel(Model m) { model_ = m; }
    void setL1xMode(uint8_t mode_0short_1medium_2long) { l1x_mode_ = mode_0short_1medium_2long > 2 ? 2 : mode_0short_1medium_2long; }

    bool RangeInit() {
        drv_ = nullptr; fail_ = "";
        if (!enabled_) { fail_ = "sonar disabled"; return false; }
        if (model_ == Model::MSP) { fail_ = "the lidar of an MSP flow sensor: no such flow sensor is set up (flow.type)"; return false; }
        I2CBus* bus = shared_;
        if (!bus) {
            uint8_t pins[2];
            if (pins_set_) { pins[0] = pins_[0]; pins[1] = pins_[1]; } else hw_.getI2CRangeSensorPins(pins);
            if (pins[0] == GPIO_UNUSED || pins[1] == GPIO_UNUSED) { fail_ = "I2C pins not set (app: Pins tab, or hardware_config.h)"; return false; }
            I2CBus_config_t c; c.bus_clock_speed = 400000; c.ioSdaGpio = pins[0]; c.ioSclGpio = pins[1]; c.addr = 0x29;
            own_.I2CInit(c);
            bus = &own_;
        }
        TwoWire& w = *bus->I2CGetWire();
        (void)w;
#ifdef WB_HAS_VL53L1X
        if (model_ != Model::VL53L0X) { l1x_.bind(w, (Vl53l1xRange::Mode)l1x_mode_); if (l1x_.init()) drv_ = &l1x_; }
#endif
#ifdef WB_HAS_VL53L0X
        if (!drv_ && model_ != Model::VL53L1X) { l0x_.bind(w); if (l0x_.init()) drv_ = &l0x_; }
#endif
        if (!drv_) {
#if !defined(WB_HAS_VL53L1X) && !defined(WB_HAS_VL53L0X)
            fail_ = "VL53 libraries missing: add pololu/VL53L0X and pololu/VL53L1X to lib_deps";
#else
            fail_ = "no VL53L1X / VL53L0X answered on I2C 0x29 (check wiring, 2.8 V supply, XSHUT)";
#endif
            return false;
        }
        height_.begin(drv_->min_range_m(), drv_->max_range_m());
        return true;
    }
    // Test hook / other sensor types: use any RangeDriver (must already be init()-ed).
    bool useDriver(RangeDriver* d) { if (!d) return false; drv_ = d; height_.begin(d->min_range_m(), d->max_range_m()); fail_ = ""; seen_ = false; return true; }

    // Returns true when a NEW tilt-corrected height was produced.
    bool update(uint32_t now_us, uint32_t now_ms, float up_cos, bool armed) {
        if (!drv_) return false;
        if (!seen_) { seen_ = true; last_new_ms_ = last_try_ms_ = now_ms; }
        RangeReading r;
        if (!drv_->update(now_us, r)) { watchdog(now_ms, armed); return false; }
        last_new_ms_ = now_ms;
        raw_ = r;
        return height_.update(r, up_cos, now_ms, armed);
    }
    bool available() const { return drv_ != nullptr; }
    bool healthy() const { return drv_ && drv_->healthy(); }
    bool valid(uint32_t now_ms) const { return drv_ && drv_->healthy() && height_.valid(now_ms); }
    float heightM() const { return height_.height_m(); }
    float sigmaM() const { return height_.sigma_m(); }
    float rawM() const { return raw_.distance_m; }
    uint8_t quality() const { return raw_.quality_pct; }
    float maxRangeM() const { return drv_ ? drv_->max_range_m() : 0.0f; }
    float rateHz() const { return drv_ ? drv_->nominal_rate_hz() : 0.0f; }
    const char* name() const { return drv_ ? drv_->name() : "none"; }
    const char* failReason() const { return fail_; }
    RangeHeight& height() { return height_; }
    uint32_t restarts() const { return restarts_; }                     // how often the watchdog started the sensor again

private:
#ifdef WB_HAS_VL53L1X
    Vl53l1xRange l1x_;
#endif
#ifdef WB_HAS_VL53L0X
    Vl53l0xRange l0x_;
#endif
    I2CBus* shared_ = nullptr;
    I2CBus own_{I2C_DEVICE_1};
    HardwareConfig hw_;
    RangeDriver* drv_ = nullptr;
    RangeHeight height_;
    RangeReading raw_;
    uint8_t pins_[2] = {GPIO_UNUSED, GPIO_UNUSED}, l1x_mode_ = 2;
    bool pins_set_ = false, enabled_ = true;
    Model model_ = Model::AUTO;
    const char* fail_ = "";
    // A chip that stopped answering (its own reset after a supply dip, continuous mode lost) gets init() again - only while
    // disarmed: a restart blocks the I2C bus for tens of ms. In flight it is only stale (valid() false), as ArduPilot marks it NoData.
    void watchdog(uint32_t now_ms, bool armed) {
        const float period_ms = drv_->nominal_rate_hz() > 0.0f ? 1000.0f / drv_->nominal_rate_hz() : 100.0f;
        const uint32_t silent_ms = (uint32_t)(10.0f * period_ms) > 500u ? (uint32_t)(10.0f * period_ms) : 500u;
        if (armed || now_ms - last_new_ms_ < silent_ms || now_ms - last_try_ms_ < (restarts_ ? 1000u : silent_ms)) return;
        last_try_ms_ = now_ms; restarts_++;
        drv_->init();
    }
    uint32_t last_new_ms_ = 0, last_try_ms_ = 0, restarts_ = 0; bool seen_ = false;
};
