/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>
#include "rx_proto.h"

// ============================================================================
// Pulse receivers (portable, no Arduino): PPM (all channels on one wire) and PWM (one wire per channel).
// Both work on edge timestamps (microseconds) that a GPIO interrupt records (rx::EdgeRing); the decoding runs in the sensor
// thread, never in the interrupt. Limits and the PPM frame logic are Betaflight's (drivers/rx/rx_pwm.c):
//
//   PPM: the time between two edges of the same kind is one channel. A gap longer than 2.7 ms is the sync; a frame is accepted
//        when it has 4..12 channels of 0.75..2.25 ms each and the same count as the frames before it (the count is learnt
//        from 25 frames in a row, so a single damaged frame neither changes it nor gets through). Positive or negative PPM:
//        the interrupt only looks at one kind of edge, so both work.
//   PWM: rising edge -> falling edge on each input = that channel (0.75..2.25 ms). A frame is reported when input 0 finishes
//        a pulse and every wired input had a good pulse in the last 100 ms (a receiver that stops its pulses on signal loss
//        then counts as lost after the receiver timeout).
// Channels a receiver does not have read 1500 us (as IBUS channels 15, 16).
// ============================================================================
namespace rx {

struct Edge { uint32_t us; uint8_t input; uint8_t level; };

// Single producer (the GPIO interrupt) / single consumer (the sensor thread). N must be a power of two.
template <int N> class EdgeRing {
    static_assert((N & (N - 1)) == 0, "N must be a power of two");
public:
    // interrupt side
    inline void push(uint32_t us, uint8_t input, uint8_t level) {
        const uint32_t h = head_, t = tail_;
        if ((uint32_t)(h - t) >= (uint32_t)N) { lost_ = lost_ + 1; return; }      // full: the newest edge is dropped (the decoders resync)
        buf_[h & (N - 1)] = Edge{us, input, level};
        __atomic_store_n(&head_, h + 1, __ATOMIC_RELEASE);
    }
    // thread side
    bool pop(Edge& e) {
        const uint32_t t = tail_, h = __atomic_load_n(&head_, __ATOMIC_ACQUIRE);
        if (t == h) return false;
        e = buf_[t & (N - 1)];
        __atomic_store_n(&tail_, t + 1, __ATOMIC_RELEASE);
        return true;
    }
    uint32_t lost() const { return lost_; }
    void clear() { tail_ = head_; }
private:
    Edge buf_[N] = {}; volatile uint32_t head_ = 0, tail_ = 0, lost_ = 0;
};

constexpr uint32_t kPulseMinUs = 750, kPulseMaxUs = 2250;          // Betaflight PPM_IN_MIN / MAX_CHANNEL_PULSE_US
inline bool good_pulse(uint32_t us) { return us > kPulseMinUs && us < kPulseMaxUs; }

// ---------------------------------------------------------------- PPM
class PpmDecoder {
public:
    static constexpr uint32_t kSyncUs = 2700;                      // PPM_IN_MIN_SYNC_PULSE_US
    static constexpr int kMinCh = 4, kMaxCh = 12, kStableFrames = 25;
    const char* name() const { return "PPM"; }
    void reset() { *this = PpmDecoder(); }

    // one edge (always the same kind). true: a frame ended and it was well formed (written to out)
    bool edge(uint32_t us, Frame& out) {
        if (!have_last_) { have_last_ = true; last_us_ = us; return false; }
        const uint32_t d = us - last_us_; last_us_ = us;
        bool got = false;
        if (d > kSyncUs) {
            if (index_ == prev_count_ && index_ >= kMinCh && index_ <= kMaxCh) {
                if (stable_ < kStableFrames) stable_++; else count_ = index_;
            } else stable_ = 0;
            if (tracking_ && count_ > 0 && index_ == count_) {
                for (int i = 0; i < kChannels; i++) out.ch[i] = i < count_ ? clamp_us((float)cap_[i]) : 1500;
                out.failsafe = false; out.frame_lost = false; frames_++; got = true;
            } else if (tracking_ && count_ > 0) errors_++;                  // a frame with a channel too many / too few
            tracking_ = true; prev_count_ = index_; index_ = 0;
        } else if (tracking_) {
            if (good_pulse(d) && index_ < kMaxCh) cap_[index_++] = (uint16_t)d;
            else { tracking_ = false; errors_++; }                         // not a channel: wait for the next sync
        }
        return got;
    }
    int channels() const { return count_; }                                // learnt channel count, 0 = not yet
    uint32_t frames() const { return frames_; }
    uint32_t errors() const { return errors_; }
private:
    uint16_t cap_[kMaxCh] = {}; uint32_t last_us_ = 0, frames_ = 0, errors_ = 0;
    int index_ = 0, prev_count_ = 0, count_ = 0, stable_ = 0; bool tracking_ = false, have_last_ = false;
};

// ---------------------------------------------------------------- PWM
class PwmDecoder {
public:
    static constexpr int kMaxInputs = 8;
    static constexpr uint32_t kStaleUs = 100000;
    const char* name() const { return "PWM"; }
    void reset(uint8_t wired_mask) { const uint8_t m = wired_mask; *this = PwmDecoder(); wired_ = m; }

    // one edge on one input (level after the edge). true: a frame (written to out)
    bool edge(uint8_t input, uint8_t level, uint32_t us, Frame& out) {
        if (input >= kMaxInputs || !(wired_ & (1u << input))) return false;
        In& c = in_[input];
        if (level) { c.rise_us = us; c.high = true; return false; }
        if (!c.high) return false;                                         // a falling edge without its rising one (start-up)
        c.high = false;
        const uint32_t w = us - c.rise_us;
        if (!good_pulse(w)) { errors_++; return false; }
        c.us = (uint16_t)w; c.seen_us = us; c.seen = true;
        if (input != first_input()) return false;
        for (int i = 0; i < kMaxInputs; i++) if ((wired_ & (1u << i)) && (!in_[i].seen || (uint32_t)(us - in_[i].seen_us) > kStaleUs)) return false;
        for (int i = 0; i < kChannels; i++) out.ch[i] = (i < kMaxInputs && (wired_ & (1u << i))) ? clamp_us((float)in_[i].us) : 1500;
        out.failsafe = false; out.frame_lost = false; frames_++;
        return true;
    }
    uint32_t frames() const { return frames_; }
    uint32_t errors() const { return errors_; }
private:
    int first_input() const { for (int i = 0; i < kMaxInputs; i++) if (wired_ & (1u << i)) return i; return 0; }
    struct In { uint32_t rise_us = 0, seen_us = 0; uint16_t us = 1500; bool high = false, seen = false; };
    In in_[kMaxInputs]; uint8_t wired_ = 0; uint32_t frames_ = 0, errors_ = 0;
};

} // namespace rx
