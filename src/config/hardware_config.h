/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>

#define GPIO_UNUSED 255

// ============================================================================
// Which GPIO does each module use?
//
// Two sources, merged here so that every driver asks ONE place:
//   1. Board defaults: the constants typed in the getters below (compile time). GPIO_UNUSED = this board has no fixed pin for it.
//   2. Pins sent by the app (tab "Pins / Ports", settings section "pins"): loaded into this class at boot with
//      HardwareConfig::setAppPins(). A pin the app left empty (GPIO_UNUSED) keeps its board default.
//
// Where the app's pins are KEPT: not in this file. The app sends the "pins" section over the WBP link, the firmware keeps it
// in RAM (wbp::Settings::pins), writes it to NVS flash on SAVE (wbp::SettingsStore, key "wbs11") and reads it back at every boot;
// FlightApp::begin() then hands it to setAppPins() before any driver starts. Pin changes therefore need SAVE + REBOOT.
// ============================================================================
struct HardwarePins {
    uint8_t imu_cs = GPIO_UNUSED, imu_miso = GPIO_UNUSED, imu_mosi = GPIO_UNUSED, imu_sck = GPIO_UNUSED;   // IMU on SPI (a CS pin selects SPI)
    uint8_t i2c_sda = GPIO_UNUSED, i2c_scl = GPIO_UNUSED;   // shared sensor I2C bus: barometer, lidar, compass (and the IMU when it has no CS pin)
    uint8_t gps_rx = GPIO_UNUSED, gps_tx = GPIO_UNUSED;     // GPS UART
    uint8_t rx_rx = GPIO_UNUSED, rx_tx = GPIO_UNUSED;       // serial receiver UART (CRSF / SBUS / IBUS)
    uint8_t motor[8] = {GPIO_UNUSED, GPIO_UNUSED, GPIO_UNUSED, GPIO_UNUSED, GPIO_UNUSED, GPIO_UNUSED, GPIO_UNUSED, GPIO_UNUSED};   // hardware slot order
    uint8_t batt_adc = GPIO_UNUSED;                         // battery voltage divider -> ADC pin
    uint8_t buzzer = GPIO_UNUSED;                           // buzzer (active: on/off, passive: PWM tone)
    uint8_t sd_cs = GPIO_UNUSED, sd_clk = GPIO_UNUSED, sd_cmd = GPIO_UNUSED, sd_d0 = GPIO_UNUSED;   // SD card: SPI CS / SCK / MOSI / MISO, or SDMMC - / CLK / CMD / D0
};

class HardwareConfig {
public:
    // ---- pins from the app (one table shared by every HardwareConfig object) -------------------------------
    static void setAppPins(const HardwarePins& p) { app() = p; }
    static void clearAppPins() { app() = HardwarePins(); }
    static const HardwarePins& appPins() { return app(); }
    // The merge rule: the app's pin when it set one, otherwise the board default.
    static uint8_t appOr(uint8_t board_default, uint8_t app_pin) { return app_pin != GPIO_UNUSED ? app_pin : board_default; }

    // ---- getters used by the drivers: board default first, then the app's pin if it set one -----------------
    void getI2CAccGyroSensorPins(uint8_t* pin) {
        pin[0] = GPIO_UNUSED; // SDA
        pin[1] = GPIO_UNUSED; // SCL
        fromApp(pin[0], app().i2c_sda); fromApp(pin[1], app().i2c_scl);
    }

    void getSPIAccGyroSensorPins(uint8_t* pin) {
        pin[0] = GPIO_UNUSED; // CS
        pin[1] = GPIO_UNUSED; // MISO
        pin[2] = GPIO_UNUSED; // MOSI
        pin[3] = GPIO_UNUSED; // SCK
        fromApp(pin[0], app().imu_cs); fromApp(pin[1], app().imu_miso); fromApp(pin[2], app().imu_mosi); fromApp(pin[3], app().imu_sck);
    }

    // Motor/ESC signal pins in HARDWARE SLOT order (slot k = pin k). Logical motor -> slot mapping is the motor-reorder setting.
    void getMotorPins(uint8_t* pin) {
        for (int i = 0; i < 8; i++) pin[i] = GPIO_UNUSED;
        for (int i = 0; i < 8; i++) fromApp(pin[i], app().motor[i]);
    }

    // Barometer (BMP280 / BMP388): normally on the shared sensor I2C bus, optional SPI on its own SPI device.
    void getI2CBaroSensorPins(uint8_t* pin) {
        pin[0] = GPIO_UNUSED; // SDA
        pin[1] = GPIO_UNUSED; // SCL
        fromApp(pin[0], app().i2c_sda); fromApp(pin[1], app().i2c_scl);
    }

    void getSPIBaroSensorPins(uint8_t* pin) {
        pin[0] = GPIO_UNUSED; // CS
        pin[1] = GPIO_UNUSED; // MISO
        pin[2] = GPIO_UNUSED; // MOSI
        pin[3] = GPIO_UNUSED; // SCK
    }

    // Sonar / lidar (VL53L1X, VL53L0X): normally on the shared sensor I2C bus.
    void getI2CRangeSensorPins(uint8_t* pin) {
        pin[0] = GPIO_UNUSED; // SDA
        pin[1] = GPIO_UNUSED; // SCL
        fromApp(pin[0], app().i2c_sda); fromApp(pin[1], app().i2c_scl);
    }

    // GPS UART: pin[0] = RX (ESP32 receives from the module's TX), pin[1] = TX, pin[2] = UART number (informational).
    void getGpsPins(uint8_t* pin) {
        pin[0] = GPIO_UNUSED; // RX
        pin[1] = GPIO_UNUSED; // TX
        pin[2] = 1;           // UART1
        fromApp(pin[0], app().gps_rx); fromApp(pin[1], app().gps_tx);
    }

    // Magnetometer (HMC5883L / QMC5883L / QMC5883P): normally on the shared sensor I2C bus.
    void getI2CMagSensorPins(uint8_t* pin) {
        pin[0] = GPIO_UNUSED; // SDA
        pin[1] = GPIO_UNUSED; // SCL
        fromApp(pin[0], app().i2c_sda); fromApp(pin[1], app().i2c_scl);
    }

    // Serial receiver UART (CRSF / SBUS / IBUS): pin[0] = RX (from the receiver's TX), pin[1] = TX (telemetry, may stay unused), pin[2] = UART number.
    void getSerialRxPins(uint8_t* pin) {
        pin[0] = GPIO_UNUSED; // RX
        pin[1] = GPIO_UNUSED; // TX
        pin[2] = 2;           // UART2
        fromApp(pin[0], app().rx_rx); fromApp(pin[1], app().rx_tx);
    }

    // Battery voltage: pin[0] = ADC pin behind the resistor divider (ESP32-S3: GPIO1..GPIO20).
    void getBatteryAdcPin(uint8_t* pin) {
        pin[0] = GPIO_UNUSED;
        fromApp(pin[0], app().batt_adc);
    }

    // Buzzer: pin[0] = output that switches the buzzer (active) or carries the tone (passive).
    void getBuzzerPin(uint8_t* pin) {
        pin[0] = GPIO_UNUSED;
        fromApp(pin[0], app().buzzer);
    }

    // SD card for the flight log. SPI mode: pin[0] = CS, pin[1] = SCK, pin[2] = MOSI (card DI), pin[3] = MISO (card DO).
    // SDMMC 1-bit mode: pin[0] unused, pin[1] = CLK, pin[2] = CMD, pin[3] = D0.
    void getSdCardPins(uint8_t* pin) {
        pin[0] = GPIO_UNUSED; // CS
        pin[1] = GPIO_UNUSED; // SCK / CLK
        pin[2] = GPIO_UNUSED; // MOSI / CMD
        pin[3] = GPIO_UNUSED; // MISO / D0
        fromApp(pin[0], app().sd_cs); fromApp(pin[1], app().sd_clk); fromApp(pin[2], app().sd_cmd); fromApp(pin[3], app().sd_d0);
    }

private:
    static HardwarePins& app() { static HardwarePins pins; return pins; }
    static void fromApp(uint8_t& pin, uint8_t app_pin) { pin = appOr(pin, app_pin); }
};
