/*
 * This file is part of Windburst Project
 */

#pragma once
#include "../../config/fast_code.h"

#include <stdint.h>
#include <math.h>

// ============================================================================
// Motor protocol maths - portable (no Arduino / IDF), unit-tested on a PC (test/test_motor.cpp).
// Formulas follow Betaflight: drivers/dshot.c (prepareDshotPacket, telemetry decode, dshotInitEndpoints),
// drivers/dshot_bitbang_decode.c (GCR table + checksum), drivers/dshot_command.c (command repeats / delays),
// drivers/pwm_output.c (analogInitEndpoints).
// ============================================================================
namespace motor {

// Round to the nearest integer, halves up, for values 0 .. 65535 (timer ticks, DShot values). The C library's rounding
// function lives in flash, and the frame of every motor used to call it three times each cycle.
WB_FAST inline float round_pos(float v) { return v > 0.0f ? (float)(uint32_t)(v + 0.5f) : 0.0f; }     // (negative and NaN: 0, as the library function gave after the cast to 16 bits - never a huge number on the way to an ESC)

enum Protocol : uint8_t {
    PROTO_PWM, PROTO_ONESHOT125, PROTO_ONESHOT42, PROTO_MULTISHOT,
    PROTO_DSHOT150, PROTO_DSHOT300, PROTO_DSHOT600, PROTO_DSHOT1200,
    PROTO_PROSHOT1000, PROTO_BRUSHED,                                  // (appended in 0.8.2: the stored numbers of the others stay)
    PROTO_DRONECAN,                                                    // (0.17) ESCs on the CAN bus: uavcan.equipment.esc.RawCommand
};
// is_dshot(): a digital protocol that carries DShot packets - values 48..2047, the commands (beacon, direction...), checksum.
// DShot itself and ProShot1000 (Betaflight counts ProShot as a DShot protocol: the same 16-bit packet, sent as 4 nibble pulses).
inline bool is_dshot(Protocol p) { return (p >= PROTO_DSHOT150 && p <= PROTO_DSHOT1200) || p == PROTO_PROSHOT1000; }
// is_dshot_wire(): the DShot bit encoding itself - the only one with a bidirectional (inverted, ESC answers) variant
inline bool is_dshot_wire(Protocol p) { return p >= PROTO_DSHOT150 && p <= PROTO_DSHOT1200; }
// is_can(): no pins at all - the commands go out as DroneCAN RawCommand, 0 (stop) .. 8191 (full) per ESC index (hal/motor/can_esc.h)
inline bool is_can(Protocol p) { return p == PROTO_DRONECAN; }
constexpr uint16_t kCanMax = 8191;

constexpr int kMaxMotors = 8;
constexpr uint16_t kDshotMinThrottle = 48;      // 1..47 are commands
constexpr uint16_t kDshotMaxThrottle = 2047;
constexpr uint16_t kDshotRange = kDshotMaxThrottle - kDshotMinThrottle;   // 1999
constexpr uint16_t kDshotMaxCommand = 47;

// DShot commands (drivers/dshot_command.h)
enum DshotCmd : uint8_t {
    CMD_MOTOR_STOP = 0, CMD_BEACON1 = 1, CMD_BEACON2, CMD_BEACON3, CMD_BEACON4, CMD_BEACON5, CMD_ESC_INFO,
    CMD_SPIN_DIRECTION_1 = 7, CMD_SPIN_DIRECTION_2 = 8, CMD_3D_MODE_OFF = 9, CMD_3D_MODE_ON = 10,
    CMD_SETTINGS_REQUEST = 11, CMD_SAVE_SETTINGS = 12, CMD_EXTENDED_TELEMETRY_ENABLE = 13, CMD_EXTENDED_TELEMETRY_DISABLE = 14,
    CMD_SPIN_DIRECTION_NORMAL = 20, CMD_SPIN_DIRECTION_REVERSED = 21,
};
// Timing from dshot_command.c
constexpr uint32_t kCmdInitialDelayUs = 10000, kCmdRepeatDelayUs = 1000, kCmdBeepDelayUs = 100000;
inline uint8_t cmd_repeats(uint8_t cmd) {
    switch (cmd) {
    case CMD_SPIN_DIRECTION_1: case CMD_SPIN_DIRECTION_2: case CMD_3D_MODE_OFF: case CMD_3D_MODE_ON: case CMD_SAVE_SETTINGS:
    case CMD_SPIN_DIRECTION_NORMAL: case CMD_SPIN_DIRECTION_REVERSED: case CMD_EXTENDED_TELEMETRY_ENABLE: case CMD_EXTENDED_TELEMETRY_DISABLE:
        return 10;
    default: return 1;
    }
}

// ---------------------------------------------------------------------------------------------------------------------
// DShot frame
// ---------------------------------------------------------------------------------------------------------------------
// 16 bits: value(11) | telemetry-request(1) | checksum(4).  Checksum = XOR of the three nibbles of (value<<1|telem); inverted when bidirectional.
WB_FAST inline uint16_t dshot_packet(uint16_t value, bool telem_request, bool bidirectional) {
    if (value > kDshotMaxThrottle) value = kDshotMaxThrottle;
    unsigned packet = (unsigned)(value << 1) | (telem_request ? 1u : 0u);
    unsigned csum = 0, d = packet;
    for (int i = 0; i < 3; i++) { csum ^= d; d >>= 4; }
    if (bidirectional) csum = ~csum;
    return (uint16_t)((packet << 4) | (csum & 0xF));
}
inline bool dshot_packet_valid(uint16_t pkt, bool bidirectional) {
    unsigned p = pkt >> 4, csum = 0, d = p;
    for (int i = 0; i < 3; i++) { csum ^= d; d >>= 4; }
    if (bidirectional) csum = ~csum;
    return (csum & 0xF) == (pkt & 0xF);
}

WB_FAST inline uint32_t dshot_bit_ns(Protocol p) {
    switch (p) { case PROTO_DSHOT150: return 6667; case PROTO_DSHOT300: return 3333; case PROTO_DSHOT600: return 1667; case PROTO_PROSHOT1000: return 1000; default: return 833; }
}   // (ProShot: 4 nibbles of 4 us = 16 us a frame, the same as 16 "bits" of 1 us)

// One RMT-style symbol: level0 for dur0 ticks, then level1 for dur1 ticks.
struct Sym { uint16_t dur0; uint8_t lvl0; uint16_t dur1; uint8_t lvl1; };

// Encode a 16-bit packet as 16 symbols. DShot: bit period T, "0" = high 37.5 % of T, "1" = high 75 % of T (then low for the rest).
// inverted (bidirectional DShot): the whole waveform is inverted, line idles high.
WB_FAST inline void dshot_symbols(uint16_t packet, Protocol p, uint32_t tick_hz, bool inverted, Sym out[16]) {
    const float bit_ticks = (float)dshot_bit_ns(p) * 1e-9f * (float)tick_hz;
    const uint16_t t = (uint16_t)round_pos(bit_ticks);
    const uint16_t h0 = (uint16_t)round_pos(bit_ticks * 0.375f), h1 = (uint16_t)round_pos(bit_ticks * 0.75f);
    for (int b = 0; b < 16; b++) {
        const bool one = (packet >> (15 - b)) & 1;
        const uint16_t hi = one ? h1 : h0;
        out[b].dur0 = hi; out[b].lvl0 = inverted ? 0 : 1;
        out[b].dur1 = (uint16_t)(t - hi); out[b].lvl1 = inverted ? 1 : 0;
    }
}

// ProShot1000 (Betaflight pwm_output_dshot_shared.c, loadDmaBufferProshot): the same 16-bit packet as 4 nibbles, most significant
// first, one pulse each in a 4 us slot: high for 1 us + nibble x 125 ns, then low. 16 us a frame. One-way only.
constexpr uint32_t kProshotSlotNs = 4000, kProshotBaseNs = 1000, kProshotStepNs = 125;
WB_FAST inline void proshot_symbols(uint16_t packet, uint32_t tick_hz, Sym out[4]) {
    const float k = 1e-9f * (float)tick_hz;
    const uint16_t slot = (uint16_t)round_pos(kProshotSlotNs * k);
    for (int i = 0; i < 4; i++) {
        const uint32_t nib = (packet >> (12 - 4 * i)) & 0xF;
        const uint16_t hi = (uint16_t)round_pos((float)(kProshotBaseNs + nib * kProshotStepNs) * k);
        out[i].dur0 = hi; out[i].lvl0 = 1; out[i].dur1 = (uint16_t)(slot - hi); out[i].lvl1 = 0;
    }
}
// Both digital encodings: 16 symbols (DShot) or 4 (ProShot). Returns the number written.
WB_FAST inline int digital_symbols(uint16_t packet, Protocol p, uint32_t tick_hz, bool inverted, Sym out[16]) {
    if (p == PROTO_PROSHOT1000) { proshot_symbols(packet, tick_hz, out); return 4; }
    dshot_symbols(packet, p, tick_hz, inverted, out); return 16;
}

// ---------------------------------------------------------------------------------------------------------------------
// Bidirectional DShot reply: 21 bits at 5/4 of the DShot bit rate. Line level changes = 1, no change = 0 (see dshot_bitbang_decode.c)
// After the first (start) bit, 20 bits = 4 GCR groups of 5 bits = 16 data bits (12-bit value + 4-bit checksum).
// ---------------------------------------------------------------------------------------------------------------------
constexpr uint32_t kGcrInvalid = 0xFFFFFFFFu;
inline uint32_t reply_bit_ns(Protocol p) { return (uint32_t)(dshot_bit_ns(p) * 4 / 5); }

// 20 GCR bits (first transmitted bit = MSB) -> 12-bit value. kGcrInvalid on any bad group / checksum.
WB_FAST inline uint32_t gcr_decode(uint32_t v20) {
    #define iv 0xFFFFFFFF
    static const uint32_t dec[32] = {iv, iv, iv, iv, iv, iv, iv, iv, iv, 9, 10, 11, iv, 13, 14, 15,
                                    iv, iv, 2, 3, iv, 5, 6, 7, iv, 0, 8, 1, iv, 4, 12, iv};
    v20 &= 0xFFFFF;
    uint32_t out = dec[v20 & 0x1F];
    out |= dec[(v20 >> 5) & 0x1F] << 4;
    out |= dec[(v20 >> 10) & 0x1F] << 8;
    out |= dec[(v20 >> 15) & 0x1F] << 12;

    uint32_t c = out; c ^= c >> 8; c ^= c >> 4;
    if ((c & 0xF) != 0xF || out > 0xFFFF) return kGcrInvalid;            // checksum: XOR of nibbles must be 0xF (inverted CRC)
    return out >> 4;
}
// inverse, used by tests and by the ESC simulator
inline uint32_t gcr_encode(uint16_t value12) {
    static const uint8_t enc[16] = {0x19,0x1B,0x12,0x13,0x1D,0x15,0x16,0x17,0x1A,0x09,0x0A,0x0B,0x1E,0x0D,0x0E,0x0F};
    uint32_t d = (uint32_t)(value12 & 0xFFF) << 4;
    uint32_t c = d; c ^= c >> 8; c ^= c >> 4;
    d |= (~c) & 0xF;
    uint32_t out = 0;
    for (int g = 0; g < 4; g++) out |= (uint32_t)enc[(d >> (4 * g)) & 0xF] << (5 * g);
    return out;
}
// 20 GCR bits -> 21 line levels (bit 0 = start, first level after the idle-high line goes low). Returns number of levels (21).
inline int gcr_to_levels(uint32_t v20, uint8_t levels[21]) {
    uint8_t lvl = 1;                                       // line idles high
    levels[0] = lvl = 0;                                   // start bit: falling edge
    for (int i = 0; i < 20; i++) {
        const bool bit = (v20 >> (19 - i)) & 1;
        if (bit) lvl ^= 1;
        levels[1 + i] = lvl;
    }
    return 21;
}
// 21 sampled levels -> 20 GCR bits (transition = 1).
inline uint32_t levels_to_gcr(const uint8_t levels[21]) {
    uint32_t v = 0;
    for (int i = 0; i < 20; i++) v = (v << 1) | ((levels[1 + i] != levels[i]) ? 1u : 0u);
    return v;
}
// Edge-time capture (what an RMT receiver gives): consecutive (level, duration) runs starting with the start bit's low level.
// Duration is rounded to a whole number of reply bits. Returns 20 GCR bits or kGcrInvalid if the frame does not fit 21 bits.
// `dur` and `bit` are in the same unit, whichever the caller has: nanoseconds, or timer ticks (the flight loop: no division
// by a clock rate for every edge of every reply).
struct Run { uint8_t level; uint32_t dur; };
WB_FAST inline uint32_t runs_to_gcr(const Run* runs, int n, uint32_t bit) {
    uint8_t lv[21]; int pos = 0;
    if (bit == 0) return kGcrInvalid;
    for (int i = 0; i < n && pos < 21; i++) {
        int bits = (int)((runs[i].dur + bit / 2) / bit);
        if (bits < 1) return kGcrInvalid;
        for (int k = 0; k < bits && pos < 21; k++) lv[pos++] = runs[i].level;
    }
    // When the reply ends on a high level, that last run cannot be told from the idle line: the receiver reports it as "end",
    // without a length. A GCR group ends in at most one 0, so it is 1 or 2 bits long (Betaflight accepts 18 bits and up).
    if (pos < 18) return kGcrInvalid;
    while (pos < 21) lv[pos++] = 1;                        // the rest is the idle-high line
    return levels_to_gcr(lv);
}

// 12-bit reply value -> telemetry (drivers/dshot.c). eRPM frame: eeemmmmmmmmm = period in us.
enum TelemType : uint8_t { TT_NONE, TT_ERPM, TT_TEMP, TT_VOLT, TT_CURRENT, TT_DEBUG1, TT_DEBUG2, TT_DEBUG3, TT_STATE };
struct Telem { TelemType type = TT_NONE; uint32_t value = 0; };    // value: ERPM => eRPM/100 ; TEMP => degC ; VOLT => 0.25 V ; CURRENT => A
constexpr uint32_t kTelemInvalid = 0xFFFFFFFFu;

inline uint32_t erpm_from_reply(uint16_t v) {
    if (v == 0x0FFF) return 0;                              // stopped
    const uint32_t period = (uint32_t)(v & 0x01FF) << ((v & 0xFE00) >> 9);
    if (!period) return kTelemInvalid;
    return (1000000u * 60u / 100u + period / 2) / period;   // erpm / 100
}
WB_FAST inline Telem telem_decode(uint16_t v12, bool edt) {
    Telem t;
    const unsigned type = (v12 & 0x0F00) >> 8;
    const bool is_erpm = !edt || (type & 1) || type == 0;
    if (is_erpm) { t.type = TT_ERPM; t.value = erpm_from_reply(v12); return t; }
    static const TelemType lut[8] = {TT_ERPM, TT_TEMP, TT_VOLT, TT_CURRENT, TT_DEBUG1, TT_DEBUG2, TT_DEBUG3, TT_STATE};
    t.type = lut[type >> 1]; t.value = v12 & 0x00FF;
    return t;
}
inline float erpm100_to_rpm(uint32_t erpm100, uint8_t pole_count) {
    if (pole_count < 2) return 0.0f;
    return (float)erpm100 * 100.0f / (float)(pole_count / 2);
}

// ---------------------------------------------------------------------------------------------------------------------
// Pulse protocols (PWM / Oneshot125 / Oneshot42 / Multishot). Everything is expressed on the classic 1000..2000 scale
// ("disarmed / armed / max" in the app) and converted to the real pulse width per protocol.
// ---------------------------------------------------------------------------------------------------------------------
struct PulseEndpoints { uint16_t disarmed = 0, armed = 1000, max = 2000; };   // app defaults: not armed 0, armed 1000, max throttle 2000

inline float pulse_us(Protocol p, float v) {
    if (v <= 0.0f) return 0.0f;                             // 0 = no pulse at all
    switch (p) {
    case PROTO_PWM:        return v;                        // 1000..2000 us
    case PROTO_ONESHOT125: return v * 0.125f;               // 125..250 us
    case PROTO_ONESHOT42:  return v * (1.0f / 24.0f);       // 41.7..83.3 us
    case PROTO_MULTISHOT:  return 5.0f + (v - 1000.0f) * 0.02f;   // 5..25 us
    default:               return 0.0f;                     // (BRUSHED is a duty cycle, not a pulse width: brushed_duty())
    }
}
// Brushed motors (a MOSFET per motor, Betaflight BRUSHED): 1000..2000 on the app's scale = 0..100 % duty at pulse_hz (16 kHz)
inline float brushed_duty(float v) { return v <= 1000.0f ? 0.0f : v >= 2000.0f ? 1.0f : (v - 1000.0f) * 0.001f; }
inline uint16_t default_pulse_hz(Protocol p) {
    switch (p) { case PROTO_PWM: return 400; case PROTO_ONESHOT125: return 2000; case PROTO_ONESHOT42: return 4000; case PROTO_BRUSHED: return 16000; case PROTO_DRONECAN: return 400; default: return 4000; }
}
inline float pulse_period_min_us(Protocol p) {              // frame period must exceed the longest pulse
    switch (p) { case PROTO_PWM: return 2100; case PROTO_ONESHOT125: return 260; case PROTO_ONESHOT42: return 90; case PROTO_BRUSHED: return 0; default: return 30; }
}

// ---------------------------------------------------------------------------------------------------------------------
// Endpoints in output units (DShot value / pulse scale value), Betaflight dshotInitEndpoints / analogInitEndpoints.
// idle_frac = motor idle (0..1 of range), limit = motor output limit (0..1).
// ---------------------------------------------------------------------------------------------------------------------
struct Endpoints { float disarm, low, high; };
inline Endpoints endpoints(Protocol p, const PulseEndpoints& pe, float idle_frac, float limit) {
    Endpoints e;
    if (is_can(p)) {                                        // 0 = stop; armed: never below 1 (a 0 in flight would stop the motor)
        e.disarm = 0.0f;
        e.low = 1.0f + idle_frac * (kCanMax - 1);
        e.high = kCanMax - (kCanMax - e.low) * (1.0f - limit);
    } else if (is_dshot(p)) {
        e.disarm = 0.0f;
        e.low = kDshotMinThrottle + idle_frac * kDshotRange;
        e.high = kDshotMaxThrottle - kDshotRange * (1.0f - limit);
    } else {
        e.disarm = pe.disarmed;
        e.low = pe.armed + idle_frac * (pe.max - pe.armed);
        e.high = pe.max - (pe.max - e.low) * (1.0f - limit);
    }
    return e;
}

} // namespace motor
