/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>
#include <string.h>
#include "../hal/rx/rx_proto.h"
#include "../hal/radio/nrf24.h"

// ============================================================================
// The Windburst nRF24 link: what the DIY transmitter (src/tx/) and the flight controller's nRF24 receiver (hal/rx/rx_nrf24.h)
// say to each other. Every packet starts with its type and ends with a CRC8 (DVB-S2, as CRSF) over all bytes before it - on top
// of the radio's own 16-bit CRC, so a packet the hardware let through damaged is still thrown away whole.
//   transmitter -> receiver                                      receiver -> transmitter (the ACK payload of the next packet)
//   0x01 control  [type][seq][8 x 10 bit: us - 1000][crc]   13   0x81 telemetry [type][flags][mode][batt cV u16][lq][crc]   7
//   0x02 bind     [type][addr 5][rf ch][rate][pps][crc]     10   0x82 bind ACK  [type][addr 5][crc]                          7
//   0x03 config   [type][seq][rf ch][rate][pps][crc]         6
// Bind: on kBindAddr / kBindChannel, the transmitter offers a new address + RF channel; the receiver echoes the address in its
// ACK, then both move there. Config: sent on the old settings, both move after it was ACKed.
// ============================================================================
namespace wbr {

enum Type : uint8_t { T_CONTROL = 0x01, T_BIND = 0x02, T_CONFIG = 0x03, T_TELEMETRY = 0x81, T_BIND_ACK = 0x82 };
constexpr uint8_t kChannels = 8, kControlBytes = 13, kTelemetryBytes = 7;
static_assert(kControlBytes == nrf::Radio::kCtlPayload && kTelemetryBytes == nrf::Radio::kAckPayload, "the radio's retries are timed for these sizes");
constexpr uint8_t kBindAddr[5] = {0x57, 0x42, 0x4E, 0x44, 0x21};      // "WBND!"
constexpr uint8_t kBindChannel = 83;                 // 2483 MHz: the top of the 2.4 GHz band, away from where pairs fly (74 .. 82)
constexpr uint8_t kPairChannelLo = 74, kPairChannels = 9;   // a bind picks 74 .. 82: between WiFi channel 11 (up to 2473 MHz) and the band edge

inline uint8_t crc(const uint8_t* p, uint8_t n) { uint8_t c = 0; for (uint8_t i = 0; i < n; i++) c = rx::crc8_dvb_s2(c, p[i]); return c; }
inline bool sealed(const uint8_t* p, uint8_t n, uint8_t want_n, uint8_t type) { return n == want_n && p[0] == type && crc(p, (uint8_t)(n - 1)) == p[n - 1]; }

// ---- control: 8 channels of 1000..2000 us, 10 bits each, least significant bit first
inline uint8_t encode_control(uint8_t seq, const uint16_t ch_us[kChannels], uint8_t out[32]) {
    out[0] = T_CONTROL; out[1] = seq;
    uint32_t acc = 0; int bits = 0, k = 2;
    for (int i = 0; i < kChannels; i++) {
        const uint16_t us = ch_us[i] < 1000 ? 1000 : ch_us[i] > 2000 ? 2000 : ch_us[i];
        acc |= (uint32_t)(us - 1000) << bits; bits += 10;
        while (bits >= 8) { out[k++] = (uint8_t)acc; acc >>= 8; bits -= 8; }
    }
    out[k] = crc(out, (uint8_t)k); return (uint8_t)(k + 1);
}
inline bool decode_control(const uint8_t* p, uint8_t n, uint8_t& seq, uint16_t ch_us[kChannels]) {
    if (!sealed(p, n, kControlBytes, T_CONTROL)) return false;
    uint16_t v[kChannels]; uint32_t acc = 0; int bits = 0, k = 2;
    for (int i = 0; i < kChannels; i++) {
        while (bits < 10) { acc |= (uint32_t)p[k++] << bits; bits += 8; }
        v[i] = (uint16_t)(acc & 0x3FF); acc >>= 10; bits -= 10;
        if (v[i] > 1000) return false;
    }
    seq = p[1]; for (int i = 0; i < kChannels; i++) ch_us[i] = (uint16_t)(1000 + v[i]);
    return true;
}

// ---- telemetry (the receiver's ACK payload)
struct Telemetry { bool armed = false, failsafe = false; uint8_t mode = 0; uint16_t batt_cv = 0; uint8_t lq = 0; };
inline uint8_t encode_telemetry(const Telemetry& t, uint8_t out[32]) {
    out[0] = T_TELEMETRY; out[1] = (uint8_t)((t.armed ? 1 : 0) | (t.failsafe ? 2 : 0)); out[2] = t.mode; out[3] = (uint8_t)t.batt_cv; out[4] = (uint8_t)(t.batt_cv >> 8);
    out[5] = t.lq > 100 ? 100 : t.lq; out[6] = crc(out, 6); return kTelemetryBytes;
}
inline bool decode_telemetry(const uint8_t* p, uint8_t n, Telemetry& t) {
    if (!sealed(p, n, kTelemetryBytes, T_TELEMETRY) || p[2] > wbp::FLY_MODE_AUTOTUNE || p[5] > 100) return false;
    t.armed = p[1] & 1; t.failsafe = (p[1] >> 1) & 1; t.mode = p[2]; t.batt_cv = (uint16_t)(p[3] | (p[4] << 8)); t.lq = p[5]; return true;
}

// ---- bind
struct BindOffer { uint8_t addr[5] = {}; uint8_t rf_channel = 100, rate = wbp::NRF_RATE_R250K, pps = wbp::NRF_PPS_HZ100; };
inline bool radio_values_ok(uint8_t ch, uint8_t rate, uint8_t pps) { return ch <= 125 && rate <= wbp::NRF_RATE_R2M && pps <= wbp::NRF_PPS_HZ150; }
inline uint8_t encode_bind(const BindOffer& o, uint8_t out[32]) {
    out[0] = T_BIND; memcpy(out + 1, o.addr, 5); out[6] = o.rf_channel; out[7] = o.rate; out[8] = o.pps; out[9] = crc(out, 9); return 10;
}
inline bool decode_bind(const uint8_t* p, uint8_t n, BindOffer& o) {
    if (!sealed(p, n, 10, T_BIND) || !radio_values_ok(p[6], p[7], p[8])) return false;
    memcpy(o.addr, p + 1, 5); o.rf_channel = p[6]; o.rate = p[7]; o.pps = p[8]; return true;
}
inline uint8_t encode_bind_ack(const uint8_t addr[5], uint8_t out[32]) { out[0] = T_BIND_ACK; memcpy(out + 1, addr, 5); out[6] = crc(out, 6); return 7; }
inline bool decode_bind_ack(const uint8_t* p, uint8_t n, uint8_t addr[5]) { if (!sealed(p, n, 7, T_BIND_ACK)) return false; memcpy(addr, p + 1, 5); return true; }

// ---- radio settings changed in the transmitter's app tab
struct RadioCfg { uint8_t seq = 0, rf_channel = 100, rate = wbp::NRF_RATE_R250K, pps = wbp::NRF_PPS_HZ100; };
inline uint8_t encode_config(const RadioCfg& r, uint8_t out[32]) { out[0] = T_CONFIG; out[1] = r.seq; out[2] = r.rf_channel; out[3] = r.rate; out[4] = r.pps; out[5] = crc(out, 5); return 6; }
inline bool decode_config(const uint8_t* p, uint8_t n, RadioCfg& r) {
    if (!sealed(p, n, 6, T_CONFIG) || !radio_values_ok(p[2], p[3], p[4])) return false;
    r.seq = p[1]; r.rf_channel = p[2]; r.rate = p[3]; r.pps = p[4]; return true;
}

// ---- the packet sequence at the receiver: newer ones are taken, a copy (its ACK was lost, the transmitter sent it again) or an
// older one is dropped, a gap is counted as lost; after timeout_ms of silence anything starts it again (the transmitter restarted)
class SeqFilter {
public:
    bool accept(uint8_t seq, uint32_t now_ms, uint16_t timeout_ms) {
        if (!started_ || (uint32_t)(now_ms - last_ms_) > timeout_ms) { started_ = true; last_ = seq; last_ms_ = now_ms; return true; }
        const int8_t d = (int8_t)(uint8_t)(seq - last_);
        if (d <= 0) return false;
        lost_ += (uint32_t)(d - 1); last_ = seq; last_ms_ = now_ms; return true;
    }
    uint32_t lost() const { return lost_; }
    void reset() { started_ = false; }
private:
    bool started_ = false; uint8_t last_ = 0; uint32_t last_ms_ = 0, lost_ = 0;
};

// ---- a new pair address from two random words: not preamble-like at the start (0x00 / 0x55 / 0xAA / 0xFF make the radio's
// address detection unreliable), never the bind address
inline void make_address(uint32_t r0, uint32_t r1, uint8_t a[5]) {
    a[0] = (uint8_t)r0; a[1] = (uint8_t)(r0 >> 8); a[2] = (uint8_t)(r0 >> 16); a[3] = (uint8_t)(r0 >> 24); a[4] = (uint8_t)r1;
    if (a[0] == 0x00 || a[0] == 0x55 || a[0] == 0xAA || a[0] == 0xFF) a[0] ^= 0x3C;
    if (!memcmp(a, kBindAddr, 5)) a[4] ^= 0x01;
}

} // namespace wbr
