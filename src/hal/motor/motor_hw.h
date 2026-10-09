/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>
#include "motor_protocol.h"
#include "motor_output.h"

namespace motor {
// Counters of the output driver since start (shown in the Motor tab; they tell a wiring problem from a timing problem).
struct HwStats {
    uint32_t frames = 0;          // frames put on the wire
    uint32_t skipped = 0;         // updates that came before the previous frame was over (loop faster than the protocol allows)
    uint32_t late_switch = 0;     // bidirectional: the pin was not handed to the ESC in time, that reply was lost
    uint32_t replies = 0;         // bidirectional: replies captured
    uint32_t garbled = 0;         // bidirectional: captured, but not a clean 21-bit reply
};
}

// Hardware side of the motor output: takes the raw per-slot values MotorOutputManager produced and puts them on the pins.
class MotorHw {
public:
    virtual ~MotorHw() = default;
    virtual bool begin(const uint8_t* pins, int count, motor::Protocol proto, uint16_t pulse_hz, bool bidirectional) = 0;
    virtual void write(const motor::Frame& f) = 0;
    // Bidirectional DShot: true when a new ESC reply was captured for this hardware slot (20 GCR bits, or kGcrInvalid for a garbled one).
    virtual bool poll_reply(int slot, uint32_t& gcr20) { return false; }
    virtual void set_driver(uint8_t) {}                             // DShot only: 0 = direct RMT, 1 = ESP-IDF driver (call before begin)
    virtual bool bidirectional() const { return false; }            // what the backend really does (the IDF DShot path is one-way)
    virtual motor::HwStats stats() const { return motor::HwStats(); }
    virtual bool rx_responsive(int slot) const { return false; }   // false = the ESC on this slot is not answering (no bidirectional firmware / wiring)
    virtual const char* name() const = 0;
};
