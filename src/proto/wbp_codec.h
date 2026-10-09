#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>

// ============================================================================
// WBP framing (Windburst Protocol), portable.
//   | 0xA5 | 0x5A | len_lo | len_hi | type | seq | payload (len bytes) | crc_lo | crc_hi |
//   len = payload length (<= kMaxPayload). CRC16-CCITT (poly 0x1021, init 0xFFFF) over len_lo, len_hi, type, seq, payload.
//   Everything multi-byte is little-endian. The parser resynchronises on the next 0xA5 0x5A after any error, so a half frame / noise /
//   boot-log garbage on the USB serial costs at most one frame.
// ============================================================================
namespace wbp {

constexpr uint16_t kMaxPayload = 512;
constexpr uint8_t kSync1 = 0xA5, kSync2 = 0x5A;
constexpr size_t kOverhead = 8;                      // sync(2) + len(2) + type + seq + crc(2)

enum MsgType : uint8_t {
    MSG_HELLO = 0x01, MSG_DEVICE_INFO = 0x02,
    MSG_SECTION_GET = 0x10, MSG_SECTION = 0x11, MSG_SECTION_SET = 0x12, MSG_SETTINGS_SAVE = 0x13, MSG_SETTINGS_DEFAULTS = 0x14, MSG_REBOOT = 0x15,
    MSG_STREAM_CFG = 0x20, MSG_STREAM = 0x21,
    MSG_COMMAND = 0x30, MSG_CMD_RESULT = 0x31,
    MSG_LOG = 0x40,
    MSG_ACK = 0x7F,
};

inline uint16_t crc16(const uint8_t* d, size_t n, uint16_t crc = 0xFFFF) {
    for (size_t i = 0; i < n; i++) { crc ^= (uint16_t)d[i] << 8; for (int b = 0; b < 8; b++) crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1); }
    return crc;
}

// Encode one frame. Returns total length, 0 if it does not fit.
inline size_t encode(uint8_t* out, size_t cap, uint8_t type, uint8_t seq, const uint8_t* payload, uint16_t len) {
    if (len > kMaxPayload || cap < (size_t)len + kOverhead) return 0;
    out[0] = kSync1; out[1] = kSync2; out[2] = (uint8_t)len; out[3] = (uint8_t)(len >> 8); out[4] = type; out[5] = seq;
    if (len) memcpy(out + 6, payload, len);
    const uint16_t c = crc16(out + 2, (size_t)len + 4);
    out[6 + len] = (uint8_t)c; out[7 + len] = (uint8_t)(c >> 8);
    return (size_t)len + kOverhead;
}

struct Frame { uint8_t type = 0, seq = 0; uint16_t len = 0; const uint8_t* payload = nullptr; };

class Parser {
public:
    // true when a complete frame with a valid CRC has just been received; `f.payload` points into the parser (valid until the next call).
    // After a true result call pump() until it returns false: a failed frame is re-scanned byte by byte (its tail may contain the start of
    // the next real frame, e.g. "A5 A5 5A" noise followed by a good frame), so one bad frame never hides the good one behind it.
    bool feed(uint8_t c, Frame& f) { push(c); return pump(f); }
    bool pump(Frame& f) {
        while (head_ != tail_) { const uint8_t c = fifo_[head_]; head_ = (head_ + 1) % kFifo; if (step(c, f)) return true; }
        return false;
    }
    // The line went quiet in the middle of a frame (call after ~20 ms without bytes): a false header with a long length would otherwise
    // wait forever for bytes that never come. Abandons it and re-scans what it swallowed; returns true if that uncovered a frame.
    bool idle(Frame& f) { if (st_ == S_SYNC1) return false; rescan(); return pump(f); }
    bool mid_frame() const { return st_ != S_SYNC1; }
    uint32_t frames() const { return frames_; }
    uint32_t errors() const { return errors_; }
    void reset() { st_ = S_SYNC1; head_ = tail_ = 0; raw_n_ = 0; }
private:
    static constexpr size_t kFifo = 2 * (kMaxPayload + kOverhead) + 8;
    void push(uint8_t c) { const size_t n = (tail_ + 1) % kFifo; if (n == head_) { errors_++; return; } fifo_[tail_] = c; tail_ = n; }
    // a frame failed: drop its first byte (the false sync) and queue the rest (placed IN FRONT of anything not processed yet) for re-scanning
    void rescan() {
        errors_++; st_ = S_SYNC1;
        const size_t n = raw_n_; raw_n_ = 0;
        for (size_t i = n; i > 1; i--) { head_ = (head_ + kFifo - 1) % kFifo; fifo_[head_] = raw_[i - 1]; }
    }
    bool step(uint8_t c, Frame& f) {
        if (st_ != S_SYNC1 && raw_n_ < sizeof(raw_)) raw_[raw_n_++] = c;
        switch (st_) {
        case S_SYNC1: if (c == kSync1) { st_ = S_SYNC2; raw_[0] = c; raw_n_ = 1; } break;
        case S_SYNC2: if (c == kSync2) st_ = S_LEN1; else if (c == kSync1) { raw_n_ = 1; raw_[0] = c; } else { st_ = S_SYNC1; raw_n_ = 0; } break;
        case S_LEN1: len_ = c; st_ = S_LEN2; break;
        case S_LEN2: len_ |= (uint16_t)(c << 8); if (len_ > kMaxPayload) rescan(); else st_ = S_TYPE; break;
        case S_TYPE: type_ = c; st_ = S_SEQ; break;
        case S_SEQ: seq_ = c; n_ = 0; st_ = len_ ? S_PAY : S_CRC1; break;
        case S_PAY: buf_[n_++] = c; if (n_ >= len_) st_ = S_CRC1; break;
        case S_CRC1: crc_ = c; st_ = S_CRC2; break;
        case S_CRC2: {
            const uint8_t hdr[4] = {(uint8_t)len_, (uint8_t)(len_ >> 8), type_, seq_};
            uint16_t want = crc16(hdr, 4); want = crc16(buf_, len_, want);
            if (want == (uint16_t)(crc_ | (c << 8))) { st_ = S_SYNC1; raw_n_ = 0; f.type = type_; f.seq = seq_; f.len = len_; f.payload = buf_; frames_++; return true; }
            rescan(); break; }
        }
        return false;
    }
    enum S : uint8_t { S_SYNC1, S_SYNC2, S_LEN1, S_LEN2, S_TYPE, S_SEQ, S_PAY, S_CRC1, S_CRC2 } st_ = S_SYNC1;
    uint8_t type_ = 0, seq_ = 0; uint16_t len_ = 0, n_ = 0, crc_ = 0; uint32_t frames_ = 0, errors_ = 0;
    uint8_t buf_[kMaxPayload], raw_[kMaxPayload + kOverhead], fifo_[kFifo]; size_t raw_n_ = 0, head_ = 0, tail_ = 0;
};

} // namespace wbp
