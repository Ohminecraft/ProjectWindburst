/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "../proto/wbp_server.h"

// ============================================================================
// The transmit side of the app link, the same for every kind of serial port.
//
// The three ports an ESP32-S3 offers behave very differently when asked "how much can I write now?":
//   UART0 behind a USB bridge     the driver's buffer (what setTxBufferSize() gave it)
//   USB Serial/JTAG  (HWCDC)      the same, a ring buffer of the core
//   native USB CDC   (TinyUSB)    64 bytes at most - one USB packet - whatever is asked
// The protocol decides per frame "fits -> send, does not fit -> drop" (telemetry, live log), so on the third kind every frame
// longer than 64 bytes would be dropped for ever. Hence one queue of our own in front of the port:
//   write()     a frame goes into the queue whole or not at all
//   writable()  free bytes of the queue - what the protocol's drop rule looks at
//   pump()      once per millisecond: hands the port as many bytes as it takes now, never more (so the port never blocks)
// Used by one task only (the link task). Portable; test_app runs it against ports of all three kinds.
// ============================================================================
namespace app {

// What the queue needs from a port.
struct RawPort {
    virtual ~RawPort() = default;
    virtual size_t room() = 0;                               // bytes the port takes right now without waiting
    virtual size_t put(const uint8_t* d, size_t n) = 0;      // n <= room(); returns what was taken
};

// A port that has reported no room for `after_ms` is offered one byte anyway (USB Serial/JTAG, link_arduino.h: only a write
// re-arms the interrupt that empties its ring). The byte is the next one of the queue: taken or not, nothing is lost or doubled.
struct StallKick {
    uint32_t after_ms = 0, since_ms = 0, kicks = 0;        // after_ms 0 = never
    size_t room(int free_bytes, uint32_t now_ms) {
        if (free_bytes > 0) { since_ms = now_ms; return (size_t)free_bytes; }
        if (after_ms && (uint32_t)(now_ms - since_ms) >= after_ms) { since_ms = now_ms; kicks++; return 1; }
        return 0;
    }
};

class QueuedTransport : public wbp::Transport {
public:
    static constexpr size_t kSize = 8192;                    // > 8 download frames of 492 bytes, or 0.35 s of a 230400 baud link
    static constexpr int kWaitMs = 200;                      // how long a frame that must not be lost (a reply) waits for room
    using WaitFn = void (*)(void*);                          // sleep about 1 ms

    // bytes_per_s: what the link carries (UART: baud / 10); 0 = fast, no need to budget (USB)
    QueuedTransport(RawPort& port, uint32_t bytes_per_s, WaitFn wait = nullptr, void* ctx = nullptr) : port_(port), bps_(bytes_per_s), wait_(wait), ctx_(ctx) {}

    size_t writable() override { return kSize - used_; }
    uint32_t bytes_per_second() override { return bps_; }

    // The callers ask writable() first for everything that may be dropped. A frame that arrives here without room is one that
    // must not be lost: it waits (the port is pumped meanwhile) and is only given up when the port takes nothing for kWaitMs.
    size_t write(const uint8_t* d, size_t n) override {
        if (n == 0 || n > kSize) return 0;
        for (int waited = 0; kSize - used_ < n; waited++) {
            pump();
            if (kSize - used_ >= n) break;
            if (!wait_ || waited >= kWaitMs) { lost_++; return 0; }
            wait_(ctx_);
        }
        const size_t head = (tail_ + used_) % kSize, first = n < kSize - head ? n : kSize - head;
        memcpy(buf_ + head, d, first);
        if (first < n) memcpy(buf_, d + first, n - first);
        used_ += n;
        pump();
        return n;
    }

    void pump() {
        while (used_) {
            const size_t room = port_.room();
            if (!room) return;
            size_t n = used_ < kSize - tail_ ? used_ : kSize - tail_;          // up to the end of the array
            if (n > room) n = room;
            const size_t k = port_.put(buf_ + tail_, n);
            if (!k) return;
            tail_ = (tail_ + k) % kSize; used_ -= k; sent_ += k;
        }
    }
    size_t queued() const { return used_; }
    uint32_t frames_lost() const { return lost_; }           // frames that had to be given up (a port that takes nothing)
    uint32_t bytes_sent() const { return sent_; }

private:
    RawPort& port_; uint32_t bps_; WaitFn wait_; void* ctx_;
    size_t tail_ = 0, used_ = 0; uint32_t lost_ = 0, sent_ = 0;
    uint8_t buf_[kSize];
};

} // namespace app
