/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>
#include <string.h>

// ============================================================================
// Serial receiver protocols (portable, no Arduino): CRSF (TBS Crossfire / ExpressLRS), SBUS (FrSky / Futaba), IBUS (FlySky),
// Spektrum DSM2 / DSMX satellites (below).
// Every decoder turns a byte stream into rx::Frame = 16 channels in microseconds (1000..2000 nominal), the same scaling
// Betaflight uses, so stick endpoints and switch ranges match what a pilot knows from Betaflight.
//
//   CRSF  420000 8N1            [0xC8][len][type][payload...][crc8 DVB-S2 over type+payload]      us = 0.62477 * raw + 881
//   SBUS  100000 8E2 INVERTED   [0x0F][22 bytes = 16 x 11 bit][flags][0x00]  (no checksum!)       us = 5/8 * raw + 880
//   IBUS  115200 8N1            [0x20][0x40][14 x u16 LE][checksum = 0xFFFF - sum]                us = raw
//
// The bytes are drained from the UART in blocks (~1 kHz), so byte timing is only coarse. CRSF and IBUS therefore re-synchronise
// from their checksum (a failed frame is re-scanned from its second byte: a false start byte never hides a real frame).
// SBUS has no checksum; it uses the >= 4 ms silence between frames plus the start and end byte.
// rx::Link turns frames into "link ok / lost" (timeout + the protocol's own failsafe flag) and holds the last good channels.
// ============================================================================
namespace rx {

enum Protocol : uint8_t { PROTO_CRSF, PROTO_SBUS, PROTO_IBUS, PROTO_SPEKTRUM, PROTO_NRF24, PROTO_PPM, PROTO_PWM, PROTO_SPEKTRUM1024 };   // == schema enum rx_protocol
// (PROTO_SPEKTRUM = 2048 mode: DSMX, DSM2 11 ms)

constexpr int kChannels = 16;
constexpr uint16_t kUsMin = 885, kUsMax = 2115;           // Betaflight's PWM_PULSE_MIN / MAX: anything outside is not a stick position

struct Frame {
    uint16_t ch[kChannels] = {};
    bool failsafe = false;        // the receiver itself says it lost the transmitter (SBUS flag): the channels are its failsafe values
    bool frame_lost = false;      // SBUS: this frame repeats old data
};

inline uint16_t clamp_us(float us) { return us < kUsMin ? kUsMin : (us > kUsMax ? kUsMax : (uint16_t)(us + 0.5f)); }

// 16 x 11-bit little-endian bit-packed channels (CRSF and SBUS use the same packing)
inline void unpack_11bit(const uint8_t* p, uint16_t raw[kChannels]) {
    uint32_t acc = 0; int bits = 0, n = 0;
    for (int i = 0; i < 22 && n < kChannels; i++) {
        acc |= (uint32_t)p[i] << bits; bits += 8;
        if (bits >= 11) { raw[n++] = (uint16_t)(acc & 0x7FF); acc >>= 11; bits -= 11; }
    }
}

class Decoder {
public:
    virtual ~Decoder() = default;
    virtual const char* name() const = 0;
    virtual uint32_t baud() const = 0;
    virtual bool inverted() const { return false; }
    virtual bool even_parity_2stop() const { return false; }
    // true when this byte completed a valid RC frame (written to out)
    virtual bool feed(uint8_t c, uint32_t now_us, Frame& out) = 0;
    virtual void reset() = 0;
    virtual uint8_t link_quality() const { return 0; }        // 0..100 %, 0 = the protocol does not report it
    uint32_t frames() const { return frames_; }
    uint32_t errors() const { return errors_; }
protected:
    uint32_t frames_ = 0, errors_ = 0;
};

inline uint8_t crc8_dvb_s2(uint8_t crc, uint8_t a) {
    crc ^= a;
    for (int i = 0; i < 8; i++) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
    return crc;
}

// ---------------------------------------------------------------- CRSF
class CrsfDecoder : public Decoder {
public:
    static constexpr uint8_t kSync = 0xC8, kTypeLinkStats = 0x14, kTypeChannels = 0x16;
    const char* name() const override { return "CRSF"; }
    uint32_t baud() const override { return 420000; }
    void reset() override { n_ = 0; }
    uint8_t link_quality() const override { return lq_; }
    int rssi_dbm() const { return rssi_dbm_; }

    bool feed(uint8_t c, uint32_t, Frame& out) override {
        if (n_ < sizeof(buf_)) buf_[n_++] = c;
        bool got = false;
        for (;;) {
            size_t skip = 0; while (skip < n_ && buf_[skip] != kSync) skip++;
            if (skip) drop(skip);
            if (n_ < 2) break;
            const uint8_t len = buf_[1];                                  // type + payload + crc
            if (len < 2 || len > 62) { errors_++; drop(1); continue; }
            if (n_ < (size_t)len + 2) break;
            uint8_t crc = 0; for (int i = 2; i < len + 1; i++) crc = crc8_dvb_s2(crc, buf_[i]);
            if (crc != buf_[len + 1]) { errors_++; drop(1); continue; }   // not a frame: re-scan from the next byte
            const uint8_t type = buf_[2];
            if (type == kTypeChannels && len == 24) {
                uint16_t raw[kChannels]; unpack_11bit(buf_ + 3, raw);
                for (int i = 0; i < kChannels; i++) out.ch[i] = clamp_us(0.62477120195241f * (float)raw[i] + 881.0f);
                out.failsafe = false; out.frame_lost = false; frames_++; got = true;
            } else if (type == kTypeLinkStats && len == 12) {             // uplink RSSI ant1, ant2, LQ, SNR, active antenna, ...
                rssi_dbm_ = -(int)(buf_[7] ? buf_[4] : buf_[3]); lq_ = buf_[5] > 100 ? 100 : buf_[5];
            }
            drop((size_t)len + 2);
        }
        return got;
    }
private:
    void drop(size_t k) { n_ -= k; memmove(buf_, buf_ + k, n_); }
    uint8_t buf_[64]; size_t n_ = 0; uint8_t lq_ = 0; int rssi_dbm_ = 0;
};

// ---------------------------------------------------------------- SBUS
class SbusDecoder : public Decoder {
public:
    static constexpr uint32_t kGapUs = 2500;                  // frames are >= 4 ms apart, bytes inside a frame arrive together
    const char* name() const override { return "SBUS"; }
    uint32_t baud() const override { return 100000; }
    bool inverted() const override { return true; }
    bool even_parity_2stop() const override { return true; }
    void reset() override { n_ = 0; }

    bool feed(uint8_t c, uint32_t now_us, Frame& out) override {
        if (n_ > 0 && (uint32_t)(now_us - last_us_) > kGapUs) { errors_++; n_ = 0; }      // a frame that stopped half way
        last_us_ = now_us;
        if (n_ == 0 && c != 0x0F) return false;
        buf_[n_++] = c;
        if (n_ < 25) return false;
        n_ = 0;
        const uint8_t end = buf_[24];
        if (!(end == 0x00 || (end & 0x0F) == 0x04)) { errors_++; return false; }           // 0x00 = SBUS, 0x04/0x14/0x24/0x34 = SBUS2 telemetry slots
        uint16_t raw[kChannels]; unpack_11bit(buf_ + 1, raw);
        for (int i = 0; i < kChannels; i++) out.ch[i] = clamp_us(0.625f * (float)raw[i] + 880.0f);
        out.frame_lost = (buf_[23] & 0x04) != 0; out.failsafe = (buf_[23] & 0x08) != 0;
        frames_++;
        return true;
    }
private:
    uint8_t buf_[25]; size_t n_ = 0; uint32_t last_us_ = 0;
};

// ---------------------------------------------------------------- IBUS
class IbusDecoder : public Decoder {
public:
    const char* name() const override { return "IBUS"; }
    uint32_t baud() const override { return 115200; }
    void reset() override { n_ = 0; }

    bool feed(uint8_t c, uint32_t, Frame& out) override {
        if (n_ < sizeof(buf_)) buf_[n_++] = c;
        bool got = false;
        for (;;) {
            size_t skip = 0; while (skip < n_ && buf_[skip] != 0x20) skip++;
            if (skip) drop(skip);
            if (n_ < 2) break;
            if (buf_[1] != 0x40) { drop(1); continue; }
            if (n_ < 32) break;
            uint16_t sum = 0xFFFF; for (int i = 0; i < 30; i++) sum = (uint16_t)(sum - buf_[i]);
            if (sum != (uint16_t)(buf_[30] | (buf_[31] << 8))) { errors_++; drop(1); continue; }
            for (int i = 0; i < 14; i++) out.ch[i] = clamp_us((float)((buf_[2 + 2 * i] | (buf_[3 + 2 * i] << 8)) & 0x0FFF));
            out.ch[14] = out.ch[15] = 1500;                   // IBUS carries 14 channels
            out.failsafe = false; out.frame_lost = false; frames_++; got = true;
            drop(32);
        }
        return got;
    }
private:
    void drop(size_t k) { n_ -= k; memmove(buf_, buf_ + k, n_); }
    uint8_t buf_[32]; size_t n_ = 0;
};

// ---------------------------------------------------------------- Spektrum (DSM2 / DSMX satellite, serial)
// 115200 8N1, a 16-byte frame every 11 or 22 ms: [fades][system][7 x u16 big-endian]. Each word carries a channel number and
// its value: 2048 mode (DSMX, DSM2 11 ms) bits 14..11 = channel, 10..0 = value; 1024 mode (DSM2 22 ms) bits 13..10 / 9..0.
// 0xFFFF = empty slot. With more than 7 channels the frames take turns carrying them, so the decoder keeps every channel's last
// value. No checksum: frames are found from the silence between them (> 4 ms; a frame takes 1.4 ms), then counted in 16s.
// Scaling and channel limit as Betaflight (rx/spektrum.c): us = 988 + value / 2 (2048) or 988 + value (1024); channels 0..11.
// The channel order is Spektrum's (throttle, aileron, elevator, rudder, gear, aux1...): set the channel map to TAER in the
// receiver settings.
class SpektrumDecoder : public Decoder {
public:
    static constexpr uint32_t kGapUs = 4000; static constexpr int kMaxCh = 12;
    explicit SpektrumDecoder(bool hires = true) : hires_(hires) {}
    void setResolution(bool hires) { hires_ = hires; }
    const char* name() const override { return hires_ ? "SPEKTRUM2048" : "SPEKTRUM1024"; }
    uint32_t baud() const override { return 115200; }
    void reset() override { n_ = 0; synced_ = false; for (int i = 0; i < kChannels; i++) ch_[i] = 1500; }

    bool feed(uint8_t c, uint32_t now_us, Frame& out) override {
        if (have_us_ && (uint32_t)(now_us - last_us_) > kGapUs) { if (n_ != 0) errors_++; n_ = 0; synced_ = true; }    // a gap: a frame starts here
        last_us_ = now_us; have_us_ = true;
        if (!synced_) return false;
        buf_[n_++] = c;
        if (n_ < 16) return false;
        n_ = 0;
        const int shift = hires_ ? 11 : 10; const uint16_t vmask = hires_ ? 0x07FF : 0x03FF;
        for (int w = 0; w < 7; w++) {
            const uint16_t word = (uint16_t)((buf_[2 + 2 * w] << 8) | buf_[3 + 2 * w]);
            const int chn = (word >> shift) & 0x0F;
            if (chn >= kMaxCh) continue;                      // (an empty slot, 0xFFFF, reads as channel 15)
            const uint16_t v = word & vmask;
            ch_[chn] = clamp_us(988.0f + (hires_ ? (float)v * 0.5f : (float)v));
        }
        for (int i = 0; i < kChannels; i++) out.ch[i] = ch_[i];
        out.failsafe = false; out.frame_lost = false; frames_++; fades_ = buf_[0];
        return true;
    }
    uint8_t fades() const { return fades_; }                  // the receiver's count of lost frames (byte 0)
private:
    bool hires_; uint8_t buf_[16] = {}; size_t n_ = 0; uint32_t last_us_ = 0; bool have_us_ = false, synced_ = false; uint8_t fades_ = 0;
    uint16_t ch_[kChannels] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
};

// ---------------------------------------------------------------- link state
// ok() = a frame without the receiver's failsafe flag arrived within timeout_ms. The channels keep their last good value when
// the link drops (the flight core's failsafe decides what to do; it never sees stale sticks as "ok").
class Link {
public:
    uint16_t timeout_ms = 300;
    void on_frame(const Frame& f, uint32_t now_ms) {
        count_++;
        if (f.failsafe) { failsafe_frames_++; return; }
        for (int i = 0; i < kChannels; i++) ch_[i] = f.ch[i];
        last_ms_ = now_ms; have_ = true;
    }
    void tick(uint32_t now_ms) {
        if ((uint32_t)(now_ms - window_ms_) >= 1000) { rate_ = (uint16_t)count_; count_ = 0; window_ms_ = now_ms; }
    }
    bool ok(uint32_t now_ms) const { return have_ && (uint32_t)(now_ms - last_ms_) <= timeout_ms; }
    const uint16_t* channels() const { return ch_; }
    uint16_t rate_hz() const { return rate_; }
    uint32_t failsafe_frames() const { return failsafe_frames_; }
private:
    uint16_t ch_[kChannels] = {1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};
    uint32_t last_ms_ = 0, window_ms_ = 0, count_ = 0, failsafe_frames_ = 0; uint16_t rate_ = 0; bool have_ = false;
};

} // namespace rx
