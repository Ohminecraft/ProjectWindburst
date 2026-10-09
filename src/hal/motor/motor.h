/*
 * This file is part of Windburst Project
 */

#pragma once
#include "../../config/fast_code.h"

#include <Arduino.h>
#include "motor_output.h"
#if defined(ESP32)
#include "dshot.h"
#include "pwm_oneshot.h"
#else                                   // PC build (tests / simulator): frames are captured instead of driving pins
#include "motor_hw_host.h"
typedef HostMotorHw DshotHw;
typedef HostMotorHw PulseHw;
#endif
#include "../../config/hardware_config.h"

// ============================================================================
// MotorSystem = MotorOutputManager (logic, unit-tested) + the right hardware backend for the chosen protocol.
//
//   MotorSystem motors;
//   motors.cfg().protocol = motor::PROTO_DSHOT600;      // fill the config from the app/settings
//   motors.begin(1.0f / 4000);
//   loop @ flight rate:   mixer.idle = motors.idleFraction(millis(), throttleRaised);
//                         motors.update(micros(), fc_out.motor, armed);
//   slower task (~1 kHz): motors.pollTelemetry(millis());
// ============================================================================
class MotorSystem {
public:
    static constexpr int kMaxDigitalMotors = 4;          // DShot / ProShot: one RMT transmitter per motor, the ESP32-S3 has 4
    motor::MotorConfig& cfg() { return mgr_.cfg; }
    motor::MotorOutputManager& manager() { return mgr_; }

    // Optional runtime pin override (pins loaded from the settings). A slot left at GPIO_UNUSED keeps its hardware_config.h pin. Call before begin().
    void setPins(const uint8_t pins[8]) { for (int i = 0; i < 8; i++) pins_[i] = pins[i]; }

    bool begin(float dt_s) {
        uint8_t pins[8];
        hw_cfg_.getMotorPins(pins);
        for (int i = 0; i < 8; i++) if (pins_[i] != GPIO_UNUSED) pins[i] = pins_[i];
        const motor::MotorConfig& c = mgr_.cfg;
        mgr_.begin(dt_s);
        hw_ = motor::is_dshot(c.protocol) ? static_cast<MotorHw*>(&dshot_) : static_cast<MotorHw*>(&pulse_);
        // the hardware slot order is the pin order: slot k -> pins[k]
        hw_->set_driver(c.dshot_driver);
        if (motor::is_dshot(c.protocol) && c.count > kMaxDigitalMotors) { ok_ = false; return false; }      // (the same rule on a PC as on the chip)
        ok_ = hw_->begin(pins, c.count, c.protocol, mgr_.pulse_hz(), c.bidirectional && motor::is_dshot_wire(c.protocol));
        if (ok_) { motor::Frame f; float z[8] = {}; mgr_.update(0, z, false, f); hw_->write(f); }   // drive the disarmed level immediately
        return ok_;
    }
    bool ok() const { return ok_; }
    MotorHw* hw() { return hw_; }
    const char* backend() const { return hw_ ? hw_->name() : "none"; }

    // now_us from micros(). cmd[] = mixer output for logical motors, absolute 0..1.
    WB_FAST void update(uint32_t now_us, const float cmd[], bool armed) {
        if (!ok_) return;
        motor::Frame f;
        mgr_.update(now_us, cmd, armed, f);
        hw_->write(f);
    }
    WB_FAST void pollTelemetry(uint32_t now_ms) {
        if (!ok_ || !mgr_.cfg.bidirectional || !hw_->bidirectional()) return;
        for (int s = 0; s < mgr_.cfg.count; s++) { uint32_t g; if (hw_->poll_reply(s, g)) mgr_.feed_reply(s, g, now_ms); }
    }
    float idleFraction(uint32_t now_ms, bool throttle_raised) { return mgr_.idle_fraction(now_ms, throttle_raised); }
    motor::HwStats hwStats() const { return hw_ ? hw_->stats() : motor::HwStats(); }
    bool bidirectionalActive() const { return ok_ && hw_ && hw_->bidirectional(); }   // false with the IDF DShot driver even when the setting is on
    bool escAnswers(int hardware_slot) const { return hw_ && hw_->rx_responsive(hardware_slot); }   // false: ESC does not send bidirectional replies

private:
    motor::MotorOutputManager mgr_;
    HardwareConfig hw_cfg_;
    DshotHw dshot_;
    PulseHw pulse_;
    MotorHw* hw_ = nullptr;
    uint8_t pins_[8] = {GPIO_UNUSED, GPIO_UNUSED, GPIO_UNUSED, GPIO_UNUSED, GPIO_UNUSED, GPIO_UNUSED, GPIO_UNUSED, GPIO_UNUSED};
    bool ok_ = false;
};
