/*
 * This file is part of Windburst Project
 */

#pragma once
#include "../../config/fast_code.h"

#include <stdint.h>
#include "motor_protocol.h"
#include "dshot_direct.h"

// ============================================================================
// DShot on an STM32 timer, "DMA burst" (Betaflight's DSHOT_DMAR, pwm_output_dshot.c): the motors on one timer share one DMA
// transfer. The timer runs at the DShot bit rate (one period = one bit); at every update event the DMA writes the next row of
// compare values into TIMx->DMAR, which the timer spreads over CCRa .. CCRb (DCR: base = the first channel, length = the
// number of channels). With the compare registers preloaded, a row written during bit k shapes bit k + 1.
//
//   row 0 .. 15   the 16 bits of each motor's packet, MSB first: a 1 is high for 14 / 20 of the bit, a 0 for 7 / 20
//                 (Betaflight MOTOR_BIT_1 / MOTOR_BIT_0)
//   row 16, 17    0: the line stays low after the frame (the last row stays in the registers until the next frame)
//
// A channel between the first and the last that is not a motor gets 0 in every row (its pin is not a timer output).
// This file only fills and reads the rows: portable, tested on the PC (test_dshot). The chip side is port/stm32/dshot_stm32.h.
// One-way DShot only (no inverted line, no reply capture yet). ProShot is not done this way.
// ============================================================================
namespace motor {

constexpr int kBurstBits = 16, kBurstTail = 2, kBurstRows = kBurstBits + kBurstTail, kBurstMaxCh = 4;

// compare value of a 1 / a 0 for a timer period of `period` ticks (ARR + 1)
inline uint32_t burst_one(uint32_t period) { return (period * 14u + 10u) / 20u; }
inline uint32_t burst_zero(uint32_t period) { return (period * 7u + 10u) / 20u; }

// rows for channels first .. first + n - 1 (1-based timer channels); packet[c - 1] for each channel c with used[c - 1].
// out: kBurstRows * n words. Returns the number of words.
WB_FAST inline int burst_fill(uint32_t* out, int first, int n, const uint16_t packet[kBurstMaxCh], const bool used[kBurstMaxCh], uint32_t period) {
    if (first < 1 || n < 1 || first + n - 1 > kBurstMaxCh) return 0;
    const uint32_t one = burst_one(period), zero = burst_zero(period);
    for (int b = 0; b < kBurstRows; b++)
        for (int k = 0; k < n; k++) {
            const int c = first - 1 + k;
            uint32_t v = 0;
            if (b < kBurstBits && used[c]) v = ((packet[c] >> (15 - b)) & 1u) ? one : zero;
            out[b * n + k] = v;
        }
    return kBurstRows * n;
}

// The packet a channel's rows carry (the tests read the rows back): -1 when a row is neither a clean 1 nor a clean 0, or the
// tail is not low.
inline int burst_read(const uint32_t* rows, int first, int n, int channel, uint32_t period) {
    const int k = channel - first;
    if (k < 0 || k >= n) return -1;
    uint32_t p = 0;
    for (int b = 0; b < kBurstBits; b++) {
        const uint32_t v = rows[b * n + k];
        if (v == burst_one(period)) p = (p << 1) | 1u; else if (v == burst_zero(period)) p <<= 1; else return -1;
    }
    for (int b = kBurstBits; b < kBurstRows; b++) if (rows[b * n + k] != 0) return -1;
    return (int)p;
}

// The timer period (ticks) for a DShot rate from the timer's clock: 0 if the clock cannot make it with enough resolution
// (fewer than 20 ticks a bit: the 7 / 14 shares would not be distinct enough) or does not fit a 16-bit counter.
inline uint32_t burst_period(uint32_t timer_hz, Protocol p) {
    if (!is_dshot_wire(p)) return 0;
    const uint32_t rate = dshot_kbps(p) * 1000u;
    const uint32_t period = (timer_hz + rate / 2u) / rate;
    return period >= 20u && period <= 65536u ? period : 0;
}

// Bidirectional DShot on an STM32: the ESC's reply as input-capture timestamps (one counter value per edge, both edges, a 16-bit
// free-running counter) -> the RMT-style words DshotDirect decodes (dshot_direct.h). The first edge is the reply's start bit
// (falling: the line idles high), then the levels alternate; after the last edge the line rests high (a long run, like the
// ESP32 receiver's idle threshold). Returns the number of words including the end mark (0 when there was no edge).
WB_FAST inline int capture_to_words(const uint32_t* cap, int n, uint32_t* words, int max) {
    if (n <= 0 || max < 2) return 0;
    int w = 0, i = 0;
    auto run = [&](int k) -> uint32_t { if (k + 1 >= n) return 0x7FFF; const uint32_t d = (uint16_t)(cap[k + 1] - cap[k]); return d > 0x7FFF ? 0x7FFF : d; };
    for (; i < n && w < max - 1; i += 2) {
        const uint32_t d0 = run(i), d1 = i + 1 < n ? run(i + 1) : 0;
        words[w++] = rmt_word((uint16_t)d0, (uint8_t)(i & 1), (uint16_t)d1, (uint8_t)((i + 1) & 1));
        if (i + 1 >= n || d1 == 0x7FFF) break;
    }
    if (w < max) words[w++] = 0;
    return w;
}

} // namespace motor
