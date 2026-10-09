/*
 * This file is part of Windburst Project
 */

#pragma once
#include "../../config/fast_code.h"

#include <stdint.h>
#include "motor_protocol.h"
#include "motor_output.h"

// ============================================================================
// DShot frame sequencing on a pulse-train peripheral (ESP32 RMT), written against a small "port" so the whole sequence runs on
// a PC against a model of the wire (test/test_dshot_direct.cpp). The ESP32 port lives in dshot.h.
//
// One cycle, bidirectional DShot (line idles HIGH, waveform inverted):
//
//   send()      collect the previous reply, drive the pins, load 16 symbols + end mark per motor, start all transmitters,
//               ask for a timer interrupt at the end of the frame
//   on_alarm()  (timer interrupt, a few microseconds after the last bit) pins become inputs with pull-up, receivers start
//   ESC         30 us after the frame: 21 bits at 5/4 of the DShot bit rate, driven by the ESC
//   send()      next cycle: stop the receivers, decode what they captured
//
// The pin is PUSH-PULL while we talk and an INPUT while the ESC talks - the same as Betaflight and esp-fc. An open-drain pin
// with only the internal pull-up cannot produce DShot300/600 edges (rise time of several bit times).
//
// Port (all calls are short and never block):
//   uint32_t micros();                         free-running microseconds
//   uint32_t tick_hz();                        resolution of the symbol durations
//   void alarm_in(uint32_t us);                call on_alarm() from the timer interrupt after this time (one shot)
//   void drive(int ch);  void release(int ch); pin = output following the transmitter / pin = input with pull-up
//   void tx_load(int ch, const uint32_t* words, int n);   void tx_start(int ch);
//   void rx_start(int ch);                     capture from the next edge on
//   int  rx_stop(int ch, uint32_t* words, int max);       stop capturing, copy the captured words, return how many
//   void delay_us(uint32_t us);                busy wait (only used by loopback(), at start-up)
// A word is one RMT symbol: duration0 (15 bits) | level0 << 15 | duration1 << 16 | level1 << 31; duration 0 = end.
// ============================================================================
namespace motor {

constexpr int kDshotTxWords = 17;                  // 16 bits + end mark
constexpr int kDshotRxWords = 16;                  // 21 reply bits = at most 21 runs = 11 words, plus the end mark
constexpr uint32_t kDshotReplyDelayUs = 30;        // the ESC answers this long after the frame
constexpr uint32_t kDshotReplySlackUs = 20;        // a slow ESC may start a little later; the reply must be over before the next frame
constexpr uint32_t kDshotSwitchMarginUs = 3;       // pins are released this long after the last bit (the ESC waits 30 us)
constexpr uint16_t kDshotSilentFrames = 20;        // this many frames in a row without a reply = "ESC does not answer"

inline uint32_t rmt_word(uint16_t d0, uint8_t l0, uint16_t d1, uint8_t l1) {
    return (uint32_t)(d0 & 0x7FFF) | ((uint32_t)(l0 & 1) << 15) | ((uint32_t)(d1 & 0x7FFF) << 16) | ((uint32_t)(l1 & 1) << 31);
}

// Length of one frame on the wire and the shortest allowed distance between two frames.
inline uint32_t dshot_frame_us(Protocol p) { return (dshot_bit_ns(p) * 16 + 999) / 1000; }
inline uint32_t dshot_reply_us(Protocol p) { return (reply_bit_ns(p) * 21 + 999) / 1000; }
inline uint32_t dshot_idle_gap_us(Protocol p) { return (reply_bit_ns(p) * 8 + 999) / 1000; }        // no edge for this long = reply over
inline uint32_t dshot_cycle_us(Protocol p, bool bidirectional) {
    const uint32_t f = dshot_frame_us(p);
    if (!bidirectional) { const uint32_t gap = (dshot_bit_ns(p) * 2 + 999) / 1000; return f + (gap > 30 ? gap : 30); }
    return f + kDshotReplyDelayUs + dshot_reply_us(p) + dshot_idle_gap_us(p) + kDshotReplySlackUs;
}

template <class Port>
class DshotDirect {
public:
    struct Stats {
        uint32_t frames = 0;          // frames put on the wire
        uint32_t skipped = 0;         // update came before the previous frame (and its reply) was over
        uint32_t late_switch = 0;     // the timer interrupt had not run when the next frame was due: that reply is lost
        uint32_t replies = 0;         // replies captured (valid or garbled)
        uint32_t garbled = 0;         // captured, but not 21 clean bits
    };

    bool begin(Port& port, int count, Protocol proto, bool bidirectional) {
        port_ = &port; count_ = 0;
        if (!is_dshot(proto) || count < 1 || count > kMaxCh) return false;
        proto_ = proto; bidir_ = bidirectional; count_ = count;
        frame_us_ = dshot_frame_us(proto); cycle_us_ = dshot_cycle_us(proto, bidirectional);
        reply_bit_q4_ = (uint32_t)(((uint64_t)reply_bit_ns(proto) * port.tick_hz() * 16u + 500000000ull) / 1000000000ull);      // one reply bit in 1/16 timer tick
        started_ = false; pending_ = false; stats_ = Stats();
        for (int c = 0; c < kMaxCh; c++) { rx_on_[c] = false; has_reply_[c] = false; silent_[c] = 0; }
        for (int c = 0; c < count_; c++) port_->drive(c);
        return true;
    }
    int count() const { return count_; }
    uint32_t cycle_us() const { return cycle_us_; }
    uint32_t frame_us() const { return frame_us_; }
    const Stats& stats() const { return stats_; }

    // Put one frame per motor on the wire. Returns false when it is too early (the previous frame is still on the wire).
    WB_FAST bool send(const uint16_t* value, const bool* telem_request) {
        if (count_ < 1) return false;
        const uint32_t now = port_->micros();
        if (started_ && (uint32_t)(now - t_tx_) < cycle_us_) { stats_.skipped++; return false; }
        if (bidir_) {
            if (pending_) { pending_ = false; stats_.late_switch++; }          // the pins are still ours; nothing was captured
            for (int c = 0; c < count_; c++) { harvest(c); port_->drive(c); }
        }
        const uint32_t tick = port_->tick_hz();
        const uint8_t idle = bidir_ ? 1 : 0;
        for (int c = 0; c < count_; c++) {
            Sym sym[16];
            dshot_symbols(dshot_packet(value[c], telem_request && telem_request[c], bidir_), proto_, tick, bidir_, sym);
            uint32_t w[kDshotTxWords];
            for (int b = 0; b < 16; b++) w[b] = rmt_word(sym[b].dur0, sym[b].lvl0, sym[b].dur1, sym[b].lvl1);
            w[16] = rmt_word(0, idle, 0, idle);
            port_->tx_load(c, w, kDshotTxWords);
        }
        for (int c = 0; c < count_; c++) port_->tx_start(c);                    // back to back: the motors get their frames within a microsecond or two
        t_tx_ = port_->micros();
        started_ = true; stats_.frames++;
        if (bidir_) { pending_ = true; port_->alarm_in(frame_us_ + kDshotSwitchMarginUs); }
        return true;
    }

    // Timer interrupt: the frame is out, hand the line to the ESC.
    WB_FAST void on_alarm() {
        if (!pending_) return;
        if ((uint32_t)(port_->micros() - t_tx_) < frame_us_) return;            // an old interrupt that arrives inside a new frame must not cut it
        pending_ = false;
        for (int c = 0; c < count_; c++) port_->release(c);
        for (int c = 0; c < count_; c++) { port_->rx_start(c); rx_on_[c] = true; }
    }

    // Newest reply of this channel since the last call: 20 GCR bits, or kGcrInvalid for a garbled one.
    WB_FAST bool take_reply(int ch, uint32_t& gcr20) {
        if (ch < 0 || ch >= count_ || !has_reply_[ch]) return false;
        has_reply_[ch] = false; gcr20 = reply_[ch];
        return true;
    }
    // Start-up check of the board itself, bidirectional only: the receiver listens to the pin while one frame of `value` goes out
    // (an ESP32 pad reads back what it drives). Proves that the transmitter reaches the pin and that the receiver hears it - the
    // two things that depend on the chip's pin routing and cannot be checked on a PC. The pin is released right after the frame,
    // as in flight (no fight with the ESC's answer), and driven again once that is over. Returns the 16-bit packet the receiver saw, or
    // kLoopNothing (no edge at all) / kLoopGarbled (edges, but not 16 DShot bits); `words` = what the receiver recorded.
    static constexpr uint32_t kLoopNothing = 0xFFFFFFFFu, kLoopGarbled = 0xFFFFFFFEu;
    uint32_t loopback(int c, uint16_t value, int& words) {
        words = 0;
        if (!bidir_ || c < 0 || c >= count_) return kLoopNothing;
        const uint16_t pkt = dshot_packet(value, false, true);
        Sym sym[16]; dshot_symbols(pkt, proto_, port_->tick_hz(), true, sym);
        uint32_t w[kDshotTxWords];
        for (int b = 0; b < 16; b++) w[b] = rmt_word(sym[b].dur0, sym[b].lvl0, sym[b].dur1, sym[b].lvl1);
        w[16] = rmt_word(0, 1, 0, 1);
        port_->drive(c); port_->rx_start(c);
        port_->tx_load(c, w, kDshotTxWords); port_->tx_start(c);
        port_->delay_us(frame_us_ + kDshotSwitchMarginUs); port_->release(c);           // as in flight: the line is the ESC's 30 us after the frame
        port_->delay_us(kDshotReplyDelayUs + dshot_reply_us(proto_) + dshot_idle_gap_us(proto_));
        uint32_t rw[kLoopWords]; const int n = port_->rx_stop(c, rw, kLoopWords); words = n;      // (the ESC's answer may follow our frame in it: only the first 16 bits count)
        port_->drive(c);
        started_ = false;                                                      // (the next send() does not wait for this frame)
        if (n <= 0) return kLoopNothing;
        // runs: idle (high) before the first edge may be reported; then 16 x (active low, idle high), the last idle = end
        Run runs[2 * kLoopWords]; int nr = 0;
        for (int i = 0; i < n; i++) {
            const uint32_t d0 = rw[i] & 0x7FFF, d1 = (rw[i] >> 16) & 0x7FFF;
            if (d0 == 0) break;
            runs[nr++] = {(uint8_t)((rw[i] >> 15) & 1), d0};
            if (d1 == 0) break;
            runs[nr++] = {(uint8_t)(rw[i] >> 31), d1};
        }
        int k = (nr > 0 && runs[0].level == 1) ? 1 : 0;
        const uint32_t bit = (uint32_t)(((uint64_t)dshot_bit_ns(proto_) * port_->tick_hz() + 500000000ull) / 1000000000ull);
        uint32_t seen = 0; int bits = 0;
        for (; k < nr && bits < 16; k += 2) {
            if (runs[k].level != 0) return kLoopGarbled;
            const uint32_t lo = runs[k].dur;
            if (lo < bit / 5 || lo > bit) return kLoopGarbled;
            seen = (seen << 1) | (lo * 100 > bit * 56 ? 1u : 0u);              // between the short (37.5 %) and the long (75 %) pulse
            bits++;
        }
        return bits == 16 ? seen : kLoopGarbled;
    }

    // false = this ESC has not answered for a while (no bidirectional firmware, wiring, or the line is too slow)
    bool responsive(int ch) const { return bidir_ && ch >= 0 && ch < count_ && ch < kMaxCh && silent_[ch] < kDshotSilentFrames; }

    static constexpr int kMaxCh = 4;
    static constexpr int kLoopWords = 48;          // the whole receive memory: our frame (16 words) and maybe the ESC's answer after it

private:
    WB_FAST void harvest(int c) {
        if (!rx_on_[c]) { if (silent_[c] < 0xFFFF) silent_[c]++; return; }
        rx_on_[c] = false;
        uint32_t w[kDshotRxWords];
        const int n = port_->rx_stop(c, w, kDshotRxWords);
        // durations in 1/16 timer tick, like reply_bit_q4_: 15 bits of ticks stay inside 32 bits, and a reply bit that is not a
        // whole number of ticks (106.7 at DShot600 / 80 MHz) is still divided by exactly
        Run runs[2 * kDshotRxWords]; int nr = 0;
        for (int i = 0; i < n; i++) {
            const uint32_t d0 = w[i] & 0x7FFF, d1 = (w[i] >> 16) & 0x7FFF;
            if (d0 == 0) break;
            runs[nr++] = {(uint8_t)((w[i] >> 15) & 1), d0 << 4};
            if (d1 == 0) break;
            runs[nr++] = {(uint8_t)(w[i] >> 31), d1 << 4};
        }
        if (nr == 0) { if (silent_[c] < 0xFFFF) silent_[c]++; return; }        // the line never moved: no reply
        const int first = runs[0].level == 0 ? 0 : 1;                           // a receiver may also report the idle-high time before the start bit
        const uint32_t g = first < nr ? runs_to_gcr(runs + first, nr - first, reply_bit_q4_) : kGcrInvalid;
        reply_[c] = g; has_reply_[c] = true; silent_[c] = 0;
        stats_.replies++; if (g == kGcrInvalid) stats_.garbled++;
    }

    Port* port_ = nullptr;
    Protocol proto_ = PROTO_DSHOT300;
    int count_ = 0; bool bidir_ = false, started_ = false;
    uint32_t frame_us_ = 54, cycle_us_ = 200, reply_bit_q4_ = 3413, t_tx_ = 0;
    volatile bool pending_ = false;
    volatile bool rx_on_[kMaxCh] = {};
    bool has_reply_[kMaxCh] = {}; uint32_t reply_[kMaxCh] = {}; uint16_t silent_[kMaxCh] = {};
    Stats stats_;
};

} // namespace motor
