/*
 * This file is part of Windburst Project
 */

#pragma once

#include <Arduino.h>
#include "../../port/pwm.h"
#include "../../port/gpio.h"
#include "beeper_core.h"
#include "../../config/hardware_config.h"

// ============================================================================
// BuzzerOutput: the pin that makes the noise.
//
//   ACTIVE   a buzzer with its own oscillator: the pin (through a transistor) switches it on and off.
//   PASSIVE  a bare piezo / speaker: the pin outputs a tone (LEDC PWM) at `frequency`; `volume` sets the duty cycle
//            (100 % = a 50 % square wave, the loudest a piezo gets).
//   inverted: the buzzer sounds when the pin is LOW (P-channel / PNP driver, or a buzzer wired to +V).
//
//   BuzzerOutput buzzer;
//   buzzer.configure(type, inverted, frequency, volume);      // settings section "beeper"
//   buzzer.begin();                                            // pin from HardwareConfig (app: Pins tab, "buzzer")
//   loop:  buzzer.set(beeper.update(millis()));
//
// The pin is driven to its silent level in begin() and whenever nothing plays. set() only touches the pin when the state changes.
// ============================================================================
class BuzzerOutput {
public:
    enum Type : uint8_t { ACTIVE, PASSIVE };

    void configure(Type type, bool inverted, uint16_t frequency_hz, uint8_t volume_pct) {
        const bool rewire = began_ && (type != type_ || inverted != inverted_ || frequency_hz != freq_);
        type_ = type; inverted_ = inverted; freq_ = frequency_hz; volume_ = volume_pct > 100 ? 100 : volume_pct;
        if (rewire) begin();                                   // a live change from the app
        else if (ok_ && on_) drive(true);                      // only the volume changed while sounding
    }

    bool begin() {
        if (began_ && ok_ && attached_pwm_) { port::pwm_detach(pin_); attached_pwm_ = false; }
        uint8_t pin[1]; hw_.getBuzzerPin(pin);
        pin_ = pin[0]; ok_ = false; fail_ = ""; began_ = true; on_ = false;
        if (pin_ == GPIO_UNUSED) { fail_ = "buzzer pin not set (app: Pins tab, or hardware_config.h)"; return false; }
        if (type_ == PASSIVE) {
            if (!port::pwm_attach(pin_, freq_, kBits)) { fail_ = WB_PLATFORM_STM32 ? "buzzer tone: the pin has no timer channel, or its timer runs at another rate (motors / servos)" : "no free PWM channel for the buzzer tone (the motor outputs use them)"; return false; }
            attached_pwm_ = true;
        } else pinMode(port::apin(pin_), OUTPUT);
        ok_ = true;
        drive(false);
        return true;
    }

    void set(bool on) { if (!ok_ || on == on_) return; on_ = on; drive(on); }
    bool on() const { return on_; }
    bool ok() const { return ok_; }
    const char* failReason() const { return fail_; }
    const char* name() const { return type_ == PASSIVE ? "passive buzzer (PWM tone)" : "active buzzer"; }

private:
    static constexpr uint8_t kBits = 10;
    void drive(bool on) {
        if (type_ == PASSIVE) {
            const uint32_t full = (1u << kBits) - 1u;
            uint32_t duty = on ? (full * volume_) / 200u : 0u;                 // 100 % volume = 50 % duty
            if (inverted_) duty = full - duty;
            port::pwm_write(pin_, duty);
        } else digitalWrite(port::apin(pin_), (on != inverted_) ? HIGH : LOW);
    }
    HardwareConfig hw_;
    Type type_ = ACTIVE; bool inverted_ = false; uint16_t freq_ = 2700; uint8_t volume_ = 100;
    uint8_t pin_ = GPIO_UNUSED; bool ok_ = false, began_ = false, on_ = false, attached_pwm_ = false; const char* fail_ = "";
};
