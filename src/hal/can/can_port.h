/*
 * This file is part of Windburst Project
 */

#pragma once

#include "dronecan_node.h"

// The CAN controller as the flight controller sees it: a dcan::Bus that can be started on two pins and reports its health.
// twai_bus.h on the ESP32 (TWAI + an external transceiver); the tests have a virtual one.
struct CanPort : dcan::Bus {
    enum State : uint8_t { OFF = 0, RUNNING = 1, BUS_OFF = 2, FAILED = 3 };       // == the schema's enum can_state
    virtual bool start(uint8_t tx_pin, uint8_t rx_pin, uint32_t bitrate) = 0;
    virtual const char* why() const = 0;                     // why start() failed
    virtual void service(uint32_t now_ms) {}                 // often: bus-off recovery and the like
    virtual uint8_t state() const = 0;
    virtual uint8_t tec() const { return 0; }                // transmit / receive error counters
    virtual uint8_t rec() const { return 0; }
    virtual uint32_t bus_offs() const { return 0; }
};
