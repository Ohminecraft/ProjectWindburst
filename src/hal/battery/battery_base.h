/*
 * This file is part of Windburst Project
 */

#pragma once

#include <Arduino.h>
#include "../../port/gpio.h"
#include "battery_monitor.h"
#include "../../config/hardware_config.h"

// ============================================================================
// BatterySensor: where the battery voltage and the current come from + batt::Monitor + batt::CurrentMeter.
//
//   BatterySensor battery;
//   battery.setSource(batt::SRC_AUTO); battery.setDivider(11.0f); battery.monitor.cfg = ...;   // from the settings (section "battery")
//   battery.setCurrent(batt::CUR_AUTO, pin, 400, 0);   // current sensor: source, ADC pin (app: Pins tab, "curr_adc"), scale, offset
//   battery.begin();                                   // voltage ADC pin from HardwareConfig (app: Pins tab, "batt_adc")
//   loop (~1 kHz): battery.update(millis(), escVolts, escAmps, escAmpsValid);    // samples at 50 Hz
//                  (escVolts = MotorOutputManager::esc_voltage(), 0 = unknown; escAmps = MotorOutputManager::esc_current())
//   battery.monitor.state() / voltage() / cells();  battery.current.amps() / mah_drawn()
//
// ADC: a resistor divider from the battery to an ADC pin. analogReadMilliVolts() already applies the chip's factory calibration,
// so the only number to set is the divider ratio (battery voltage / pin voltage; 11.0 for 10k : 1k). The pin must stay below
// ~3.1 V at the highest battery voltage (default attenuation 11 dB). ESP32-S3: GPIO1..GPIO10 are ADC1, GPIO11..GPIO20 are ADC2.
// ESC: the pack voltage the ESCs report over bidirectional DShot (extended telemetry); no wiring, but only while the ESCs talk.
// One ADC conversion takes a few tens of microseconds and runs in the flight loop's slow part, 50 times a second.
// ============================================================================
class BatterySensor {
public:
    batt::Monitor monitor;
    batt::CurrentMeter current;

    void setSource(batt::Source s) { wanted_ = s; }
    // the current sensor (live: takes effect at the next sample; a new pin only with begin())
    void setCurrent(batt::CurrentSource src, uint8_t pin, int16_t scale, int16_t offset_ma) { cur_wanted_ = src; cur_pin_wanted_ = pin; cur_scale_ = scale; cur_offset_ = offset_ma; }
    void setDivider(float ratio) { divider_ = ratio > 0.0f ? ratio : 1.0f; }

    // true when the configured source can deliver a voltage (AUTO without an ADC pin relies on the ESCs: still true)
    bool begin() {
        uint8_t pin[1]; hw_.getBatteryAdcPin(pin);
        pin_ = pin[0]; adc_ok_ = false; fail_ = ""; pin_mv_ = 0; last_ms_ = 0; started_ = false;
        monitor.reset();
        if (pin_ != GPIO_UNUSED) {
            if (!port::adc_pin_ok(pin_)) fail_ = WB_PLATFORM_STM32 ? "battery pin is not an ADC pin" : "battery pin is not an ADC pin (ESP32-S3: GPIO1..GPIO20)";
            else { pinMode(port::apin(pin_), INPUT); adc_ok_ = true; }
        } else if (wanted_ == batt::SRC_ADC) fail_ = "battery ADC pin not set (app: Pins tab, or hardware_config.h)";
        cur_pin_ = GPIO_UNUSED; cur_adc_ok_ = false; cur_fail_ = ""; cur_mv_ = 0; current.reset();
        if (cur_pin_wanted_ != GPIO_UNUSED) {
            if (!port::adc_pin_ok(cur_pin_wanted_)) cur_fail_ = WB_PLATFORM_STM32 ? "current sensor pin is not an ADC pin" : "current sensor pin is not an ADC pin (ESP32-S3: GPIO1..GPIO20)";
            else { cur_pin_ = cur_pin_wanted_; pinMode(port::apin(cur_pin_), INPUT); cur_adc_ok_ = true; }
        } else if (cur_wanted_ == batt::CUR_ADC) cur_fail_ = "current sensor ADC pin not set (app: Pins tab, curr_adc)";
        if (wanted_ == batt::SRC_NONE) { fail_ = "battery monitoring switched off"; return false; }
        if (wanted_ == batt::SRC_ADC) return adc_ok_;
        return true;
    }

    void update(uint32_t now_ms, float esc_volts, float esc_amps = 0.0f, bool esc_amps_valid = false) {
        if (started_ && (uint32_t)(now_ms - last_ms_) < kPeriodMs) return;
        const uint32_t prev = last_ms_; const bool first = !started_;
        last_ms_ = started_ ? last_ms_ + kPeriodMs : now_ms; started_ = true;
        if ((uint32_t)(now_ms - last_ms_) > 5 * kPeriodMs) last_ms_ = now_ms;      // fell behind: do not burst
        // ---- current: an ADC sensor, the ESCs, or nothing
        if (cur_adc_ok_) cur_mv_ = (uint16_t)port::adc_read_mv(cur_pin_);       // (read even when not the source: the app shows it for calibrating)
        const float dt = first ? 0.0f : (float)(uint32_t)(last_ms_ - prev) * 0.001f;
        switch (currentSource()) {
        case batt::CUR_ADC: current.update(batt::adc_to_amps(cur_mv_, cur_scale_, cur_offset_), cur_adc_ok_, dt); break;
        case batt::CUR_ESC: current.update(esc_amps, esc_amps_valid, dt); break;
        default: current.update(0.0f, false, dt); break;
        }
        const batt::Source src = source();
        if (adc_ok_) pin_mv_ = (uint16_t)port::adc_read_mv(pin_);               // read even when not the source: the app shows it for the divider calibration
        switch (src) {
        case batt::SRC_ADC: monitor.update(now_ms, batt::adc_to_volts(pin_mv_, divider_), adc_ok_); break;
        case batt::SRC_ESC: monitor.update(now_ms, esc_volts, esc_volts > 0.5f); break;
        default: monitor.update(now_ms, 0.0f, false); break;
        }
    }

    batt::Source source() const { return batt::pick_source(wanted_, adc_ok_); }      // what is really used
    batt::CurrentSource currentSource() const { return batt::pick_current_source(cur_wanted_, cur_adc_ok_); }
    uint16_t currentPinMv() const { return cur_mv_; }
    const char* currentFailReason() const { return cur_fail_; }
    const char* currentName() const {
        switch (currentSource()) { case batt::CUR_ADC: return "current sensor (ADC)"; case batt::CUR_ESC: return "ESC telemetry"; default: return "off"; }
    }
    bool adcOk() const { return adc_ok_; }
    uint16_t pinMv() const { return pin_mv_; }
    float divider() const { return divider_; }
    const char* failReason() const { return fail_; }
    const char* name() const {
        switch (source()) { case batt::SRC_ADC: return "ADC"; case batt::SRC_ESC: return "ESC telemetry"; default: return "off"; }
    }

private:
    static constexpr uint32_t kPeriodMs = 20;            // batt::kRateHz
    HardwareConfig hw_;
    batt::Source wanted_ = batt::SRC_AUTO; float divider_ = 11.0f;
    uint8_t pin_ = GPIO_UNUSED; bool adc_ok_ = false, started_ = false; uint16_t pin_mv_ = 0; uint32_t last_ms_ = 0; const char* fail_ = "";
    batt::CurrentSource cur_wanted_ = batt::CUR_AUTO; uint8_t cur_pin_wanted_ = GPIO_UNUSED, cur_pin_ = GPIO_UNUSED; bool cur_adc_ok_ = false;
    int16_t cur_scale_ = 400, cur_offset_ = 0; uint16_t cur_mv_ = 0; const char* cur_fail_ = "";
};
