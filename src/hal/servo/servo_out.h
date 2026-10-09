/*
 * This file is part of Windburst Project
 */

#pragma once

#include <Arduino.h>

// ============================================================================
// Servo outputs: up to four servos, each follows one RC channel (a gimbal tilt on a slider, a gripper on a switch...).
//
//   RC channel 1000 .. 2000 us  ->  pulse min_us .. max_us (reverse: max_us .. min_us), clamped to the channel's ends
//   no RC channel yet (power-up, no receiver): the middle of min_us .. max_us; link lost: the last position is held
//
// Hardware: the LEDC PWM peripheral (one of its 8 channels per servo; motors in PWM mode and a passive buzzer use the same
// channels), 14-bit duty at rate_hz (50 Hz analog servos, up to 330 Hz digital ones). Once a pulse is set the hardware repeats
// it: update() only writes when a pulse changes, from the slow loop. Power servos from their own 5 / 6 V BEC, grounds joined.
//
//   ServoOutputs servos;
//   servos.begin(pins, rate_hz);                  // false + failReason(): a pin could not get a PWM channel
//   servos.configure(src, min_us, max_us, reverse) // live: from the settings ("servo" section)
//   slow loop: servos.update(rc_channels, have_rc);
// ============================================================================
namespace servo {

constexpr int kServos = 4;

// The pulse (us) for an RC channel value (us): 1000 -> lo, 2000 -> hi, linear, clamped.
inline uint16_t map_us(uint16_t rc_us, uint16_t min_us, uint16_t max_us, bool reverse) {
    const float lo = reverse ? max_us : min_us, hi = reverse ? min_us : max_us;
    float x = ((float)rc_us - 1000.0f) / 1000.0f;
    if (x < 0.0f) x = 0.0f;
    if (x > 1.0f) x = 1.0f;
    return (uint16_t)(lo + (hi - lo) * x + 0.5f);
}
// 14-bit LEDC duty for a pulse of `us` at `hz` pulses per second
inline uint32_t duty_for(uint16_t us, uint16_t hz) { return (uint32_t)(((uint64_t)us * hz * 16384u + 500000u) / 1000000u); }

} // namespace servo

class ServoOutputs {
public:
    static constexpr uint8_t kBits = 14;

    bool begin(const uint8_t pins[servo::kServos], uint16_t rate_hz) {
        stop();
        rate_ = rate_hz < 50 ? 50 : rate_hz > 330 ? 330 : rate_hz; fail_ = ""; count_ = 0;
        for (int i = 0; i < servo::kServos; i++) {
            pin_[i] = 255; out_[i] = 0;
            if (pins[i] == 255) continue;                               // no pin: nothing (a servo switched OFF keeps its pin, without pulses)
            if (!ledcAttach(pins[i], rate_, kBits)) { fail_ = "no free PWM (LEDC) channel for a servo (motors in PWM mode and a passive buzzer use them too)"; continue; }
            pin_[i] = pins[i]; count_++;
        }
        have_rc_ = false;
        return *fail_ == 0;
    }
    void stop() { for (int i = 0; i < servo::kServos; i++) if (pin_[i] != 255) { ledcDetach(pin_[i]); pin_[i] = 255; out_[i] = 0; } count_ = 0; }

    // src: 0 = off, n = RC channel n (1-based); live
    void configure(const uint8_t src[servo::kServos], const uint16_t min_us[servo::kServos], const uint16_t max_us[servo::kServos], const uint8_t reverse[servo::kServos]) {
        for (int i = 0; i < servo::kServos; i++) { src_[i] = src[i]; min_[i] = min_us[i]; max_[i] = max_us[i]; rev_[i] = reverse[i] != 0; }
        dirty_ = true;
    }

    // rc: 16 channels (us); have_rc: they are real (a frame arrived since power-up). Without them the servos sit in the middle.
    void update(const uint16_t* rc, bool have_rc) {
        if (have_rc) have_rc_ = true;
        for (int i = 0; i < servo::kServos; i++) {
            if (pin_[i] == 255) continue;
            uint16_t us = 0;
            if (src_[i] >= 1 && src_[i] <= 16) us = have_rc_ ? servo::map_us(rc[src_[i] - 1], min_[i], max_[i], rev_[i]) : (uint16_t)((min_[i] + max_[i]) / 2);
            if (us != out_[i] || dirty_) { out_[i] = us; ledcWrite(pin_[i], us ? servo::duty_for(us, rate_) : 0); }
        }
        dirty_ = false;
    }

    uint16_t pulse(int i) const { return i >= 0 && i < servo::kServos ? out_[i] : 0; }
    int count() const { return count_; }
    uint16_t rateHz() const { return rate_; }
    const char* failReason() const { return fail_; }

private:
    uint8_t pin_[servo::kServos] = {255, 255, 255, 255}, src_[servo::kServos] = {};
    uint16_t min_[servo::kServos] = {1000, 1000, 1000, 1000}, max_[servo::kServos] = {2000, 2000, 2000, 2000}, out_[servo::kServos] = {};
    bool rev_[servo::kServos] = {}, have_rc_ = false, dirty_ = true; uint16_t rate_ = 50; int count_ = 0; const char* fail_ = "";
};
