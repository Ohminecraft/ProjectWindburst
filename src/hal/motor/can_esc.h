/*
 * This file is part of Windburst Project
 */

#pragma once

#include "motor_hw.h"
#include "../can/dronecan_node.h"

// ============================================================================
// ESCs on the CAN bus (motors.protocol DRONECAN): the per-slot values of MotorOutputManager (0 = stop, 1..8191 armed) go out as
// one uavcan.equipment.esc.RawCommand, value k = ESC index k = hardware slot k (the motor order tab still applies). No pins.
// Rate: one transfer every `divider` control cycles, so that the bus sees about pulse_hz of them (400 Hz by default: 4 ESCs
// are one 7-byte frame, ~130 us of a 1 Mbit/s bus; 8 ESCs three frames, with the transfer CRC). The ESCs' own esc.Status (rpm, voltage, current,
// temperature) comes back through the CAN node and FlightApp::slow_update(), not through poll_reply().
// Portable (the node does the bus): the same code on the chip and in the tests.
// ============================================================================
class CanEscHw : public MotorHw {
public:
    void set_node(dcan::Node* n) { node_ = n; }
    // the control rate the writes come at (call before begin())
    void set_loop_hz(float hz) { loop_hz_ = hz > 0 ? hz : 1000.0f; }
    bool begin(const uint8_t*, int count, motor::Protocol, uint16_t pulse_hz, bool) override {
        n_ = count < 0 ? 0 : count > motor::kMaxMotors ? motor::kMaxMotors : count; k_ = 0; st_ = motor::HwStats();
        const float hz = pulse_hz ? (float)pulse_hz : 400.0f;
        div_ = (uint16_t)(loop_hz_ / hz + 0.5f); if (div_ < 1) div_ = 1;
        return node_ != nullptr;
    }
    void write(const motor::Frame& f) override {
        if (!node_) return;
        if (++k_ < div_) return;
        k_ = 0;
        int16_t cmd[motor::kMaxMotors];
        for (int i = 0; i < n_; i++) cmd[i] = (int16_t)(f.value[i] > motor::kCanMax ? motor::kCanMax : f.value[i]);
        if (node_->send_esc(cmd, n_)) st_.frames++; else st_.skipped++;
    }
    motor::HwStats stats() const override { return st_; }
    uint16_t divider() const { return div_; }
    const char* name() const override { return "DroneCAN"; }
private:
    dcan::Node* node_ = nullptr; float loop_hz_ = 1000.0f; int n_ = 0; uint16_t div_ = 1, k_ = 0; motor::HwStats st_;
};
