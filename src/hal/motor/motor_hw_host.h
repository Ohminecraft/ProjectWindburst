/*
 * This file is part of Windburst Project
 */

#pragma once

#include "motor_hw.h"

// Motor "hardware" for PC builds (tests, simulator): remembers the last frame instead of driving pins, and lets the simulator
// inject ESC telemetry replies. On the ESP32 the real backends (dshot.h / pwm_oneshot.h) are used instead.
class HostMotorHw : public MotorHw {
public:
    motor::Frame last; uint32_t frames = 0; int count = 0; motor::Protocol proto = motor::PROTO_DSHOT300; bool bidir = false; uint8_t pins[8] = {};
    bool has_reply[8] = {}; uint32_t reply[8] = {};

    bool begin(const uint8_t* p, int n, motor::Protocol pr, uint16_t, bool bidirectional) override {
        for (int i = 0; i < n; i++) { if (p[i] == 255) return false; pins[i] = p[i]; }      // same rule as the real backends: every used slot needs a pin
        count = n; proto = pr; bidir = bidirectional; return n > 0;
    }
    void write(const motor::Frame& f) override {
        last = f; frames++;
        for (int i = 0; i < count && i < 8; i++) {                // (the simulator injects a reply before each frame)
            if (has_reply[i] && reply[i] != motor::kGcrInvalid) { silent[i] = 0; continue; }
            if (silent[i] < 0xFFFF) silent[i]++;
            if (bidir && frames > 1) { reply[i] = motor::kGcrInvalid; has_reply[i] = true; }      // no answer to the last frame: a lost reply, as DshotDirect reports it
        }
        if (on_write) on_write();
    }
    void (*on_write)() = nullptr;                                  // tests: e.g. let the output take time
    uint16_t silent[8] = {};                                       // frames in a row without a reply
    void set_pullup(bool internal) override { internal_pullup = internal; }
    bool internal_pullup = true;                                  // (tests: what the firmware asked for)
    bool bidirectional() const override { return bidir; }
    motor::HwStats stats() const override { motor::HwStats s; s.frames = frames; return s; }
    bool poll_reply(int slot, uint32_t& gcr20) override { if (slot < 0 || slot >= 8 || !has_reply[slot]) return false; has_reply[slot] = false; gcr20 = reply[slot]; return true; }
    bool rx_responsive(int slot) const override { return bidirectional() && slot >= 0 && slot < count && silent[slot] < 20; }
    const char* name() const override { return "host"; }
};
