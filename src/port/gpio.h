/*
 * This file is part of Windburst Project
 */

#pragma once
#include "platform.h"
#include <Arduino.h>
#if WB_PLATFORM_STM32
#include "pinmap.h"
#endif

// ============================================================================
// Pins and the ADC, given the one-byte pin number of the settings (port/pin_names.h says what it means per chip).
//   apin(p)          the Arduino pin number for pinMode / digitalWrite / attachInterrupt (the same number on an ESP32)
//   adc_pin_ok(p)    the pin can measure a voltage
//   adc_read_mv(p)   millivolts at the pin
// ESP32: analogReadMilliVolts() applies the chip's factory calibration.
// STM32: 12-bit reading scaled by the supply voltage, which is measured against the internal reference (VREFINT, calibrated
//        in the factory) every 50th reading - a 3.3 V regulator that sits at 3.25 V is then no 1.5 % error on the battery.
// ============================================================================
namespace port {

#if WB_PLATFORM_STM32
inline uint32_t apin(uint8_t p) { return pinNametoDigitalPin((PinName)p); }
inline bool pin_exists(uint8_t p) { return p != 255 && apin(p) < NUM_DIGITAL_PINS; }
inline bool adc_pin_ok(uint8_t p) { return pin_exists(p) && pin_in_pinmap((PinName)p, PinMap_ADC); }
inline uint32_t adc_vdda_mv() {
    static uint32_t vdda = 3300; static uint8_t n = 0;
#if defined(AVREF) && defined(__LL_ADC_CALC_VREFANALOG_VOLTAGE)
    if (n++ % 50 == 0) {
        analogReadResolution(12);
        const uint32_t raw = (uint32_t)analogRead(AVREF);
        if (raw > 0) { const uint32_t v = __LL_ADC_CALC_VREFANALOG_VOLTAGE(raw, LL_ADC_RESOLUTION_12B); if (v > 2500 && v < 3700) vdda = v; }
    }
#else
    (void)n;
#endif
    return vdda;
}
inline uint32_t adc_read_mv(uint8_t p) {
    const uint32_t vdda = adc_vdda_mv();
    analogReadResolution(12);
    return (uint32_t)analogRead(apin(p)) * vdda / 4095u;
}
#else
inline uint8_t apin(uint8_t p) { return p; }
inline bool pin_exists(uint8_t p) { return p != 255; }
inline bool adc_pin_ok(uint8_t p) { return digitalPinToAnalogChannel(p) >= 0; }
inline uint32_t adc_read_mv(uint8_t p) { return (uint32_t)analogReadMilliVolts(p); }
#endif

} // namespace port
