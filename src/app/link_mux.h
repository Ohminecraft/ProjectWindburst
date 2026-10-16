/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include "tx_queue.h"

namespace app {

// ============================================================================
// One app link, two wires: the serial port (USB / UART0) and WiFi (TCP). There is one wbp::Server; what it sends goes to the
// wire the app last spoke on. The link task calls heard(i) whenever bytes come in on wire i: when that is the other wire, the
// replies and streams follow it from then on (the app connecting there starts with HELLO anyway; the streams it asked for on
// the other wire are its own business). A wire that is not up (no TCP client) takes nothing: frames queued for it are dropped
// by the transport's queue like those of a closed serial monitor.
// ============================================================================
// A network wire (WiFi TCP, app/wifi_link.h): the link task polls it and reads it like the serial port.
struct NetWire : RawPort {
    virtual void poll(uint32_t now_ms) = 0;                  // every pass of the link task: accept a client, follow the radio
    virtual int read(uint8_t* buf, int cap) = 0;             // bytes from the client; 0 = none (or no client)
    virtual bool take_dropped() = 0;                         // the client went away since the last call
    virtual void datagram(const uint8_t*, size_t) {}          // MAVLink telemetry: one UDP broadcast to the ground stations (port 14550)
};

class LinkMux : public RawPort {
public:
    enum Wire : uint8_t { SERIAL_WIRE = 0, WIFI_WIRE = 1 };      // == the schema's enum link_via
    LinkMux(RawPort& serial, RawPort* wifi) : port_{&serial, wifi} {}

    // bytes came in on wire w; true = the link changed wires (the caller drops a half-received frame of the other one)
    bool heard(Wire w) {
        if (w == active_ || !port_[w]) return false;
        active_ = w; switches_++;
        return true;
    }
    // the wire went down (TCP client gone): back to the serial port
    void lost(Wire w) { if (w == active_ && w != SERIAL_WIRE) { active_ = SERIAL_WIRE; switches_++; } }

    size_t room() override { return port_[active_]->room(); }
    size_t put(const uint8_t* d, size_t n) override { return port_[active_]->put(d, n); }

    Wire active() const { return active_; }
    uint32_t switches() const { return switches_; }

private:
    RawPort* port_[2];
    Wire active_ = SERIAL_WIRE;
    uint32_t switches_ = 0;
};

} // namespace app
