/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>
#include "../bus/bus_i2c.h"
#include "../bus/bus_spi.h"
#include "../bus/reg_bus_arduino.h"
#include "baro_driver.h"
#include "bmp280_baro.h"
#include "bmp388_baro.h"
#include "baro_altitude.h"
#include "external_baro.h"
#include "../../config/hardware_config.h"

// ============================================================================
// BaroSensor: detect -> init -> update.  Supports BMP280 / BME280 and BMP388 / BMP390 on I2C (0x76, 0x77) or SPI,
// or a DroneCAN module (setModel(DRONECAN): the samples come through pushExternal(), from the CAN node).
//
//   BaroSensor baro;
//   baro.IMUInit-style:  if (!baro.BaroInit(false)) Serial.println(baro.failReason());
//   loop (any rate, 500 Hz..1 kHz is fine):  if (baro.update(micros(), millis(), armed)) kf.update_baro(baro.altitudeM());
//
// I2C: pass an existing I2CBus with attachI2C() when other sensors (magnetometer, lidar) share the bus - two TwoWire objects on one
// hardware port cannot coexist. Without attachI2C() the sensor creates its own bus on I2C_DEVICE_1.
// SPI: needs its own SPI device (SPI_DEVICE_1); sharing the IMU's SPI host with a second SPIBus object is not supported by SPIBus.
// ============================================================================
class BaroSensor {
public:
    void attachI2C(I2CBus& shared) { shared_i2c_ = &shared; }
    void setEnabled(bool on) { enabled_ = on; }                      // app switch "barometer on/off"
    enum class Model : uint8_t { AUTO = 0, DRONECAN = 1 };           // == the schema's enum baro_model
    void setModel(Model m) { model_ = m; }
    Model model() const { return model_; }
    void pushExternal(float pa, float temp_c, uint32_t now_us) { if (drv_ == &can_) can_.push(pa, temp_c, now_us); }
    void setI2cPins(uint8_t sda, uint8_t scl) { pins_i2c_[0] = sda; pins_i2c_[1] = scl; pins_i2c_set_ = true; }
    void setSpiPins(uint8_t cs, uint8_t miso, uint8_t mosi, uint8_t sck) { pins_spi_[0] = cs; pins_spi_[1] = miso; pins_spi_[2] = mosi; pins_spi_[3] = sck; pins_spi_set_ = true; }

    bool BaroInit(bool use_spi) { return BaroInit(use_spi, BaroInitConfig()); }
    bool BaroInit(bool use_spi, const BaroInitConfig& cfg) {
        drv_ = nullptr; detected_ = NONE_; fail_ = ""; alt_.reset();
        if (!enabled_) { fail_ = "barometer disabled"; return false; }
        if (model_ == Model::DRONECAN) { can_.init(cfg); drv_ = &can_; return true; }
        if (!use_spi) {
            I2CBus* bus = shared_i2c_;
            if (!bus) {
                uint8_t pins[2];
                if (pins_i2c_set_) { pins[0] = pins_i2c_[0]; pins[1] = pins_i2c_[1]; } else hw_.getI2CBaroSensorPins(pins);
                if (pins[0] == GPIO_UNUSED || pins[1] == GPIO_UNUSED) { fail_ = "I2C pins not set (app: Pins tab, or hardware_config.h)"; return false; }
                I2CBus_config_t c; c.bus_clock_speed = 400000; c.ioSdaGpio = pins[0]; c.ioSclGpio = pins[1]; c.addr = 0x76;
                own_i2c_.I2CInit(c);
                bus = &own_i2c_;
            }
            const uint8_t addrs[2] = {0x76, 0x77};                   // SDO low / high
            for (uint8_t a : addrs) {
                i2c_bus_.bind(*bus, a);
                uint8_t id280 = 0, id388 = 0;
                i2c_bus_.read_u8(0xD0, id280);
                if (id280 == 0x58 || id280 == 0x60 || id280 == 0x56 || id280 == 0x57) { detected_ = BMP280_; }
                else if (i2c_bus_.read_u8(0x00, id388) && (id388 == 0x50 || id388 == 0x60)) { detected_ = BMP388_; }
                if (detected_ != NONE_) { proxy_.set(&i2c_bus_); break; }
            }
            if (detected_ == NONE_) { fail_ = "no BMP280/BMP388 answered on I2C 0x76/0x77"; return false; }
        } else {
            uint8_t pins[4];
            if (pins_spi_set_) { for (int i = 0; i < 4; i++) pins[i] = pins_spi_[i]; } else hw_.getSPIBaroSensorPins(pins);
            for (int i = 0; i < 4; i++) if (pins[i] == GPIO_UNUSED) { fail_ = "SPI pins not set (hardware_config.h)"; return false; }
            SPIBus_config_t c; c.ioCsGpio = pins[0]; c.ioSdoGpio = pins[1]; c.ioSdiGpio = pins[2]; c.ioSckGpio = pins[3];
            if (!spi_.SPIInit(c)) { fail_ = "SPI bus init failed"; return false; }
            uint8_t dummy[2]; spi_bus_.read(0x00, dummy, 2);          // first CS edge (BMP3xx boots in I2C mode)
            delay(2);
            uint8_t id280 = 0, two[2] = {0, 0};
            spi_bus_.read(0xD0, &id280, 1);
            spi_bus_.read(0x00, two, 2);                              // BMP3xx: dummy byte, then chip id
            if (id280 == 0x58 || id280 == 0x60 || id280 == 0x56 || id280 == 0x57) detected_ = BMP280_;
            else if (two[1] == 0x50 || two[1] == 0x60) detected_ = BMP388_;
            if (detected_ == NONE_) { fail_ = "no BMP280/BMP388 answered on SPI (check CS/MISO/MOSI/SCK)"; return false; }
            proxy_.set(&spi_bus_);
        }
        drv_ = (detected_ == BMP280_) ? static_cast<BaroDriver*>(&d280_) : static_cast<BaroDriver*>(&d388_);
        if (!drv_->init(cfg)) { fail_ = "chip found but init failed (bad calibration, implausible reading or bus error)"; drv_ = nullptr; return false; }
        return true;
    }

    // Call often. Returns true when a NEW altitude sample was accepted.
    bool update(uint32_t now_us, uint32_t now_ms, bool armed) {
        if (!drv_) return false;
        BaroSample s;
        if (!drv_->update(now_us, s)) return false;
        raw_ = s; rate_n_++;
        return alt_.update(s, now_ms, armed);
    }
    bool available() const { return drv_ != nullptr; }
    bool healthy() const { return drv_ && drv_->healthy(); }
    bool valid(uint32_t now_ms) const { return drv_ && drv_->healthy() && alt_.valid(now_ms); }     // use for the AB_ALTHOLD_NO_BARO arm check
    float altitudeM() const { return alt_.altitude_m(); }
    float sigmaM() const { return alt_.sigma_m(); }
    float pressurePa() const { return raw_.pressure_pa; }
    float tempC() const { return raw_.temp_c; }
    float rateHz() const { return drv_ ? drv_->nominal_rate_hz() : 0.0f; }
    bool groundCalibrated() const { return alt_.calibrated(); }
    const char* name() const { return drv_ ? drv_->name() : "none"; }
    const char* failReason() const { return fail_; }
    BaroAltitude& altitude() { return alt_; }

private:
    enum Kind : uint8_t { NONE_, BMP280_, BMP388_ };
    I2CBus* shared_i2c_ = nullptr;
    I2CBus own_i2c_{I2C_DEVICE_1};
    SPIBus spi_{SPI_DEVICE_1};
    AgI2CRegBus i2c_bus_{own_i2c_};
    AgSpiRegBus spi_bus_{spi_};
    AgBusProxy proxy_;
    Bmp280Baro d280_{proxy_};
    Bmp388Baro d388_{proxy_};
    ExternalBaro can_{"DroneCAN"};
    Model model_ = Model::AUTO;
    BaroDriver* drv_ = nullptr;
    BaroAltitude alt_;
    BaroSample raw_;
    HardwareConfig hw_;
    uint8_t pins_i2c_[2] = {GPIO_UNUSED, GPIO_UNUSED}, pins_spi_[4] = {GPIO_UNUSED, GPIO_UNUSED, GPIO_UNUSED, GPIO_UNUSED};
    bool pins_i2c_set_ = false, pins_spi_set_ = false, enabled_ = true;
    Kind detected_ = NONE_;
    uint32_t rate_n_ = 0;
    const char* fail_ = "";
};
