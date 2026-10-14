/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>
#include <stddef.h>

// ============================================================================
// ESC telemetry on its own wire (Betaflight: "ESC sensor", sensors/esc_sensor.c), KISS protocol: BLHeli_32, AM32, KISS ESCs.
// The telemetry pads of all ESCs are joined to ONE UART RX at 115200 baud. The firmware asks one ESC at a time: the DShot frame
// to that motor carries the telemetry-request bit, and the ESC answers with 10 bytes (~0.9 ms on the wire):
//     temperature (degC), voltage (0.01 V, 2 bytes), current (0.01 A, 2), consumption (mAh, 2), eRPM / 100 (2), CRC8 (poly 0x07)
// all big-endian. Then the next motor. An ESC that does not answer within 100 ms is skipped (counted as a timeout), a frame with
// a bad CRC is dropped (counted). As in Betaflight the first request waits 5 s after start: the ESCs are still booting.
// The whole round with 4 ESCs takes ~5 ms (one answer per millisecond tick of the sensor thread): good for the battery, the
// temperatures and the app; far too slow for the RPM filter (that needs bidirectional DShot).
// Portable: test_motor runs it.
// ============================================================================
namespace escs {

constexpr uint32_t kBaud = 115200, kBootMs = 5000, kTimeoutMs = 100;
constexpr int kFrame = 10;

struct Reading { uint8_t temp_c = 0; float volt = 0, amps = 0; uint16_t mah = 0; uint32_t erpm = 0; };

inline uint8_t crc8(const uint8_t* b, int n) {
    uint8_t c = 0;
    for (int i = 0; i < n; i++) { c ^= b[i]; for (int k = 0; k < 8; k++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1); }
    return c;
}
inline bool decode(const uint8_t f[kFrame], Reading& r) {
    if (crc8(f, kFrame - 1) != f[kFrame - 1]) return false;
    r.temp_c = f[0];
    r.volt = (float)((f[1] << 8) | f[2]) * 0.01f;
    r.amps = (float)((f[3] << 8) | f[4]) * 0.01f;
    r.mah = (uint16_t)((f[5] << 8) | f[6]);
    r.erpm = (uint32_t)((f[7] << 8) | f[8]) * 100u;
    return true;
}

// The request / answer round. Each millisecond tick: feed the bytes that came in, then ask_now() says which logical motor gets the
// telemetry bit in the next DShot frame (-1: nobody now). A finished answer is taken with take().
class Poller {
public:
    void begin(int motors, uint32_t now_ms) { n_ = motors; m_ = 0; st_ = BOOT; t0_ = now_ms; got_ = 0; have_ = false; }
    void feed(const uint8_t* d, size_t n, uint32_t now_ms) {
        for (size_t i = 0; i < n; i++) {
            if (st_ != WAIT) { stray_++; continue; }                   // (a KISS ESC sends a line of text at start-up)
            buf_[got_++] = d[i];
            if (got_ == kFrame) {
                Reading r;
                if (decode(buf_, r)) { out_ = r; out_m_ = m_; have_ = true; frames_++; last_ok_ms_ = now_ms; }
                else crc_errors_++;
                next(now_ms);
            }
        }
    }
    int ask_now(uint32_t now_ms) {
        if (n_ <= 0) return -1;
        if (st_ == BOOT) { if ((uint32_t)(now_ms - t0_) < kBootMs) return -1; st_ = READY; }
        if (st_ == WAIT) { if ((uint32_t)(now_ms - t0_) < kTimeoutMs) return -1; timeouts_++; next(now_ms); }
        st_ = WAIT; t0_ = now_ms; got_ = 0;
        return m_;
    }
    bool take(int& motor, Reading& r) { if (!have_) return false; have_ = false; motor = out_m_; r = out_; return true; }
    uint32_t frames() const { return frames_; }
    uint32_t crc_errors() const { return crc_errors_; }
    uint32_t timeouts() const { return timeouts_; }
    uint32_t stray() const { return stray_; }
    bool alive(uint32_t now_ms) const { return frames_ && (uint32_t)(now_ms - last_ok_ms_) < 1000u; }
private:
    enum St : uint8_t { BOOT, READY, WAIT };
    void next(uint32_t) { st_ = READY; got_ = 0; m_ = (m_ + 1) % (n_ > 0 ? n_ : 1); }
    int n_ = 0, m_ = 0, out_m_ = 0; St st_ = BOOT; uint32_t t0_ = 0; uint8_t buf_[kFrame] = {}; int got_ = 0;
    Reading out_; bool have_ = false;
    uint32_t frames_ = 0, crc_errors_ = 0, timeouts_ = 0, stray_ = 0, last_ok_ms_ = 0;
};

} // namespace escs
