/*
 * This file is part of Windburst Project
 */

#pragma once
#include "../config/fast_code.h"

#include <stdint.h>
#include <string.h>
#include <atomic>
#include "telemetry.h"

// ============================================================================
// Exchange: the ONLY thing shared between the flight loop and the app link task.
//
//   flight loop (core 1, never blocks)          link task (core 0, may block on USB)
//   ---------------------------------           ------------------------------------
//   publish(telemetry)      ----- seqlock ----> snapshot(telemetry)
//   service(exec)           <---- mailbox ----- call(request)      (waits for the flight loop to run it)
//
// - publish() is wait-free: the flight loop never waits for the link task, whatever the USB host is doing.
// - Every command and every "apply these settings" runs INSIDE the flight loop (service()), between two control cycles, so the
//   drivers and the core are only ever touched by one thread. While call() waits, the link task does nothing else, so the
//   flight loop may read the Settings struct the link task owns.
// - One request at a time; a second one gets ERR_BUSY. A request the flight loop did not pick up in time is withdrawn.
// Portable (std::atomic only). Unit-tested with real threads on a PC.
// ============================================================================
namespace app {

enum : uint8_t { REQ_APPLY_SECTION = 0xF0, REQ_REBOOT = 0xF1, REQ_ESC_PASSTHROUGH = 0xF2 };   // internal requests; everything below 0x80 is a wbp::Cmd

struct Request { uint8_t cmd = 0; uint8_t n = 0; uint8_t args[8] = {}; };
struct Reply { wbp::Err err = wbp::ERR_OK; uint8_t n = 0; uint8_t data[8] = {}; Reply() {} Reply(wbp::Err e) : err(e) {} Reply(wbp::Err e, uint8_t b0) : err(e), n(1) { data[0] = b0; } };

class Exchange {
public:
    // ---------------- flight loop side
    WB_FAST void publish(const Telemetry& t) {
        seq_.fetch_add(1, std::memory_order_acq_rel);          // odd: write in progress
        memcpy(&tel_, &t, sizeof(Telemetry));
        seq_.fetch_add(1, std::memory_order_release);          // even: stable
    }
    // exec(const Request&) -> Reply (a plain wbp::Err converts). Runs at most one pending request. Returns true if one was executed.
    template <typename F> bool service(F&& exec) {
        uint8_t expect = PENDING;
        if (!state_.compare_exchange_strong(expect, RUNNING)) return false;
        reply_ = exec(req_);
        state_.store(DONE, std::memory_order_release);
        return true;
    }

    // ---------------- link task side
    void snapshot(Telemetry& out) const {
        for (;;) {
            const uint32_t a = seq_.load(std::memory_order_acquire);
            if (a & 1u) continue;
            memcpy(&out, &tel_, sizeof(Telemetry));
            if (seq_.load(std::memory_order_acquire) == a) return;
        }
    }
    // wait(ctx): sleep ~1 ms (vTaskDelay(1) on the ESP32). max_waits = timeout in calls of wait().
    using WaitFn = void (*)(void*);
    Reply call(const Request& r, WaitFn wait, void* ctx, int max_waits = 200) {
        uint8_t expect = IDLE;
        if (!state_.compare_exchange_strong(expect, FILLING)) return wbp::ERR_BUSY;
        req_ = r;
        state_.store(PENDING, std::memory_order_release);
        for (int i = 0; i < max_waits && state_.load(std::memory_order_acquire) != DONE; i++) wait(ctx);
        if (state_.load(std::memory_order_acquire) != DONE) {
            expect = PENDING;
            if (state_.compare_exchange_strong(expect, IDLE)) { timeouts_++; return wbp::ERR_BUSY; }   // never picked up: withdrawn, will not run later
            while (state_.load(std::memory_order_acquire) != DONE) wait(ctx);                           // it is running right now: its result is real
        }
        const Reply e = reply_;
        state_.store(IDLE, std::memory_order_release);
        return e;
    }
    uint32_t timeouts() const { return timeouts_; }

    // ---------------- "is the app there?" (link task writes, flight loop reads)
    // The link task calls link_touch() for every valid frame it receives. The flight loop asks link_active(): the beeper keeps
    // quiet on the bench, the lost-model beacon does not scream on the desk.
    void link_touch(uint32_t now_ms) { link_ms_.store(now_ms, std::memory_order_relaxed); link_seen_.store(true, std::memory_order_release); }
    bool link_active(uint32_t now_ms, uint32_t timeout_ms = 3000) const {
        return link_seen_.load(std::memory_order_acquire) && (uint32_t)(now_ms - link_ms_.load(std::memory_order_relaxed)) < timeout_ms;
    }

private:
    enum : uint8_t { IDLE, FILLING, PENDING, RUNNING, DONE };
    Telemetry tel_;
    std::atomic<uint32_t> seq_{0};
    std::atomic<uint8_t> state_{IDLE};
    Request req_; Reply reply_; uint32_t timeouts_ = 0;
    std::atomic<uint32_t> link_ms_{0}; std::atomic<bool> link_seen_{false};
};

} // namespace app
