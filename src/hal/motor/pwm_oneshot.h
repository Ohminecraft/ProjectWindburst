/*
 * This file is part of Windburst Project
 */

#pragma once

#include <Arduino.h>
#include "motor_hw.h"

// ============================================================================
// PWM / Oneshot125 / Oneshot42 / Multishot / Brushed (duty cycle, 16 kHz) on the LEDC peripheral (hardware PWM, zero CPU cost, up to 8 motors on ESP32-S3).
// The ESC sees a pulse train at pulse_hz whose width is set by write(). Pulse widths: motor::pulse_us().
// Values come in as the app's 1000..2000 scale (0 = no pulse, used for "not armed"), so arm/disarm/max endpoints are all the manager's job.
// ============================================================================
class PulseHw : public MotorHw {
public:
    const char* name() const override { return "Pulse/LEDC"; }

    bool begin(const uint8_t* pins, int count, motor::Protocol proto, uint16_t pulse_hz, bool) override {
        if (motor::is_dshot(proto) || count < 1 || count > 8) return false;
        ledcSetClockSource(LEDC_USE_APB_CLK);
        proto_ = proto; count_ = count; hz_ = pulse_hz ? pulse_hz : motor::default_pulse_hz(proto);
        if (1e6f / hz_ < motor::pulse_period_min_us(proto)) return false;      // period shorter than the longest pulse
#ifdef CONFIG_IDF_TARGET_ESP32S3                                      // the IDF driver cannot do more than 4 kHz on the S3 (the RMT path is one-way DShot only)
        bits_ = 14;
#else
        bits_ = 16;
#endif
        while (bits_ > 8 && (uint64_t)hz_ * (1u << bits_) > 80000000ull) bits_--;
        for (int i = 0; i < count; i++) {
            pins_[i] = pins[i];
            if (pins[i] == 255 || !ledcAttach(pins[i], hz_, bits_)) return false;
            ledcWrite(pins[i], 0);
        }
        return true;
    }
    void write(const motor::Frame& f) override {
        if (!f.send) return;
        for (int i = 0; i < count_; i++) {
            const float frac = proto_ == motor::PROTO_BRUSHED ? motor::brushed_duty((float)f.value[i]) : motor::pulse_us(proto_, (float)f.value[i]) * 1e-6f * (float)hz_;
            uint32_t duty = (uint32_t)lroundf(frac * (float)(1u << bits_));
            const uint32_t maxd = (1u << bits_) - 1; if (duty > maxd) duty = maxd;
            ledcWrite(pins_[i], duty);
        }
    }
private:
    motor::Protocol proto_ = motor::PROTO_ONESHOT125;
    int count_ = 0; uint16_t hz_ = 2000; uint8_t bits_ = 14; uint8_t pins_[8] = {};
};
