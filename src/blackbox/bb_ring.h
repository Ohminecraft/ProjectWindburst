/*
 * This file is part of Windburst Project
 */

#pragma once
#include "../config/fast_code.h"

#include <stdint.h>
#include <string.h>
#include <atomic>

// ============================================================================
// Record queue between the flight loop (the only writer) and the link task (the only reader). Wait-free on both sides:
// the flight loop never waits for flash or for the USB host. A record that does not fit is DROPPED and counted - losing a
// log line is always better than delaying a control cycle.
// Each record is stored as [length u8][bytes]. Capacity must be a power of two.
// ============================================================================
namespace bb {

template <uint32_t N>
class Ring {
    static_assert((N & (N - 1)) == 0 && N >= 256, "capacity must be a power of two");
public:
    // ---- writer (flight loop)
    WB_FAST bool push(const uint8_t* rec, uint8_t n) {
        const uint32_t h = head_.load(std::memory_order_relaxed), t = tail_.load(std::memory_order_acquire);
        if (N - (h - t) < (uint32_t)n + 1u) { dropped_.fetch_add(1, std::memory_order_relaxed); return false; }
        put(h, &n, 1); put(h + 1, rec, n);
        head_.store(h + n + 1u, std::memory_order_release);
        return true;
    }
    // ---- reader (link task)
    // Length of the next record, 0 if the queue is empty.
    uint8_t peek() const {
        const uint32_t t = tail_.load(std::memory_order_relaxed);
        if (head_.load(std::memory_order_acquire) == t) return 0;
        return buf_[t & (N - 1)];
    }
    // Copies the next record to `out` (which must hold peek() bytes) and removes it. Returns its length.
    uint8_t pop(uint8_t* out) {
        const uint32_t t = tail_.load(std::memory_order_relaxed);
        if (head_.load(std::memory_order_acquire) == t) return 0;
        const uint8_t n = buf_[t & (N - 1)];
        get(t + 1, out, n);
        tail_.store(t + n + 1u, std::memory_order_release);
        return n;
    }
    void clear() { tail_.store(head_.load(std::memory_order_acquire), std::memory_order_release); }     // reader side only
    uint32_t used() const { return head_.load(std::memory_order_acquire) - tail_.load(std::memory_order_acquire); }
    uint32_t dropped() const { return dropped_.load(std::memory_order_relaxed); }
    static constexpr uint32_t capacity() { return N; }

private:
    void put(uint32_t pos, const uint8_t* d, uint32_t n) {
        const uint32_t i = pos & (N - 1), first = n < N - i ? n : N - i;
        memcpy(buf_ + i, d, first); if (n > first) memcpy(buf_, d + first, n - first);
    }
    void get(uint32_t pos, uint8_t* d, uint32_t n) const {
        const uint32_t i = pos & (N - 1), first = n < N - i ? n : N - i;
        memcpy(d, buf_ + i, first); if (n > first) memcpy(d + first, buf_, n - first);
    }
    uint8_t buf_[N];
    std::atomic<uint32_t> head_{0}, tail_{0}, dropped_{0};
};

} // namespace bb
