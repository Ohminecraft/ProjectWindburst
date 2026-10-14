/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

// ============================================================================
// What an ESC tool did on the app's port, for the next time the app connects (0.9.2+, docs/ESC_PASSTHROUGH.md "Chẩn đoán").
// BLHeliSuite32 only says "Connection to Flightcontroller failed"; this shows what it sent and what the firmware made of it:
//   * the first bytes of every burst that does not start like a WBP frame (a burst: bytes after 300 ms of silence)
//   * the MSP requests the firmware saw, answered or refused (msp::Bridge keeps their numbers)
//   * on the native USB build: the DTR / RTS line states and the baud rates the PC set (TinyUSB tells the firmware)
// The link task feeds it and writes it into the app's log as a few lines once the app talks again (Server::log), then
// starts afresh. Line events come from the USB driver's task: plain counters, a lost or doubled entry does not matter here.
// Portable: test_esc4way checks the lines.
// ============================================================================
namespace app {

class PortDiag {
public:
    static constexpr int kBursts = 4, kBurstBytes = 20, kLines = 12;
    static constexpr uint32_t kGapMs = 300;

    // Every byte from the PC, in order.
    void on_bytes(const uint8_t* d, size_t n, uint32_t now_ms) {
        for (size_t i = 0; i < n; i++) {
            if (!any_ || (uint32_t)(now_ms - last_ms_) >= kGapMs) {          // a new burst
                cur_ = -1;
                if (d[i] != 0xA5) {                                           // (WBP frames start with A5 5A: the app itself)
                    cur_ = n_bursts_ < kBursts ? n_bursts_++ : kBursts - 1;
                    Burst& b = bursts_[cur_]; b.n = 0; b.total = 0; b.at_ms = now_ms;
                }
                any_ = true;
            }
            if (cur_ >= 0) { Burst& b = bursts_[cur_]; if (b.n < kBurstBytes) b.bytes[b.n++] = d[i]; b.total++; }
            last_ms_ = now_ms;
        }
    }
    // The PC set the control lines / the line coding (native USB only).
    void on_lines(bool dtr, bool rts, uint32_t now_ms) { push('L', (dtr ? 1u : 0u) | (rts ? 2u : 0u), now_ms); }
    void on_baud(uint32_t baud, uint32_t now_ms) { push('B', baud, now_ms); }

    bool worth_reporting(uint32_t msp_seen) const { return n_bursts_ > 0 || n_lines_ > 0 || msp_seen > 0; }

    // Up to 4 lines into out[i] (each `cap` bytes); returns how many. msp_cmds: the last MSP request numbers, oldest first.
    int report(char (*out)[160], int max_lines, uint32_t msp_frames, uint32_t msp_bad, uint32_t msp_refused,
               const uint16_t* msp_cmds, int n_cmds, uint32_t passthroughs) {
        int k = 0;
        if (k < max_lines) {
            int p = snprintf(out[k], 160, "esc tool: MSP %lu requests (%lu bad checksum, %lu unknown / refused), passthrough %lu; last:",
                             (unsigned long)msp_frames, (unsigned long)msp_bad, (unsigned long)msp_refused, (unsigned long)passthroughs);
            for (int i = 0; i < n_cmds && p < 150; i++) p += snprintf(out[k] + p, 160 - p, " %u", (unsigned)msp_cmds[i]);
            if (!n_cmds && p < 150) snprintf(out[k] + p, 160 - p, " -");
            k++;
        }
        for (int b = 0; b < n_bursts_ && k < max_lines; b++) {
            const Burst& u = bursts_[b];
            int p = snprintf(out[k], 160, "esc tool: burst %d, %lu bytes:", b + 1, (unsigned long)u.total);
            for (int i = 0; i < u.n && p < 154; i++) p += snprintf(out[k] + p, 160 - p, " %02X", u.bytes[i]);
            k++;
        }
        if (n_lines_ && k < max_lines) {
            int p = snprintf(out[k], 160, "esc tool: port lines:");
            const uint32_t t0 = lines_[0].at_ms;
            for (int i = 0; i < n_lines_ && p < 140; i++) {
                const Line& l = lines_[i];
                if (l.kind == 'L') p += snprintf(out[k] + p, 160 - p, " +%lu DTR%u RTS%u", (unsigned long)(l.at_ms - t0), (unsigned)(l.v & 1u), (unsigned)((l.v >> 1) & 1u));
                else p += snprintf(out[k] + p, 160 - p, " +%lu %lu baud", (unsigned long)(l.at_ms - t0), (unsigned long)l.v);
            }
            k++;
        }
        return k;
    }
    void clear() { n_bursts_ = 0; n_lines_ = 0; cur_ = -1; }

private:
    struct Burst { uint8_t bytes[kBurstBytes]; uint8_t n = 0; uint32_t total = 0, at_ms = 0; };
    struct Line { char kind; uint32_t v, at_ms; };
    void push(char kind, uint32_t v, uint32_t now_ms) {
        const int i = n_lines_ < kLines ? n_lines_++ : kLines - 1;
        lines_[i].kind = kind; lines_[i].v = v; lines_[i].at_ms = now_ms;
    }
    Burst bursts_[kBursts]; int n_bursts_ = 0, cur_ = -1;
    Line lines_[kLines]; volatile int n_lines_ = 0;
    uint32_t last_ms_ = 0; bool any_ = false;
};

} // namespace app
