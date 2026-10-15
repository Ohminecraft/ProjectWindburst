/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <math.h>

// ============================================================================
// DroneCAN (UAVCAN v0) - the transport layer and the bit-level serialisation, as libcanard does them. Hardware independent:
// CAN frames in and out (29-bit identifiers, classic CAN, 8 bytes). Checked against pydronecan (test/test_can.cpp, vectors made
// with tools/dronecan_vectors.py).
//
//   CAN id, message           : priority[28:24] | data type id[23:8]  | 0 (service)[7] | source node[6:0]
//   CAN id, anonymous message : priority[28:24] | discriminator[23:10] | type id & 3[9:8] | 0[7] | 0 (source)[6:0]
//   CAN id, service           : priority[28:24] | service type id[23:16] | request[15] | destination[14:8] | 1[7] | source[6:0]
//   last byte of every frame  : start of transfer[7] | end of transfer[6] | toggle[5] | transfer id[4:0]
//   a transfer of more than 7 bytes: split into frames of 7, the first two bytes of the first frame are the transfer CRC
//   (CRC-16-CCITT-FALSE over the data type signature, 8 bytes little-endian, then the payload); the toggle bit starts at 0.
//
// Serialisation: every field is packed bit after bit with no padding; a field's value is laid out little-endian byte by byte,
// and its bits enter the stream most significant first (a field that does not end on a byte boundary keeps its high bits in
// the last byte). Dynamic arrays carry a length prefix, except the last field of a transfer when its elements are 8 bits or
// longer ("tail array optimisation").
// ============================================================================
namespace dcan {

struct Frame { uint32_t id = 0; uint8_t len = 0; uint8_t data[8] = {}; };

enum Kind : uint8_t { MESSAGE = 0, REQUEST = 1, RESPONSE = 2 };
constexpr uint8_t kPrioHigh = 8, kPrioMedium = 16, kPrioLow = 24, kPrioLowest = 31;
constexpr uint8_t kMaxNodeId = 127;

// ---------------------------------------------------------------- CRC
inline uint16_t crc_add(uint16_t crc, uint8_t b) {
    crc ^= (uint16_t)((uint16_t)b << 8);
    for (int i = 0; i < 8; i++) crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
    return crc;
}
inline uint16_t crc_add(uint16_t crc, const uint8_t* p, size_t n) { for (size_t i = 0; i < n; i++) crc = crc_add(crc, p[i]); return crc; }
inline uint16_t transfer_crc(uint64_t signature, const uint8_t* p, size_t n) {
    uint16_t c = 0xFFFFu;
    for (int i = 0; i < 8; i++) c = crc_add(c, (uint8_t)(signature >> (8 * i)));
    return crc_add(c, p, n);
}

// ---------------------------------------------------------------- float16 (IEEE 754 half)
inline uint16_t f16_from(float v) {
    uint32_t f; memcpy(&f, &v, 4);
    const uint32_t sign = (f >> 16) & 0x8000u; int32_t e = (int32_t)((f >> 23) & 0xFF) - 127 + 15; uint32_t m = f & 0x7FFFFFu;
    if (((f >> 23) & 0xFF) == 0xFF) return (uint16_t)(sign | 0x7C00u | (m ? 0x200u : 0));          // inf / NaN
    if (e >= 31) return (uint16_t)(sign | 0x7C00u);                                                // too large: infinity
    if (e <= 0) {                                                                                  // subnormal or zero
        if (e < -10) return (uint16_t)sign;
        m |= 0x800000u; const int shift = 14 - e; uint32_t h = m >> shift;
        if ((m >> (shift - 1)) & 1u) h++;                                                          // round half up
        return (uint16_t)(sign | h);
    }
    uint32_t h = sign | ((uint32_t)e << 10) | (m >> 13);
    if (m & 0x1000u) h++;                                                                          // round (may carry into the exponent: correct)
    return (uint16_t)h;
}
inline float f16_to(uint16_t h) {
    const uint32_t sign = (uint32_t)(h & 0x8000u) << 16; const int e = (h >> 10) & 0x1F; uint32_t m = h & 0x3FFu; uint32_t f;
    if (e == 0) {
        if (!m) f = sign;
        else { int ee = -1; do { ee++; m <<= 1; } while (!(m & 0x400u)); f = sign | ((uint32_t)(127 - 15 - ee) << 23) | ((m & 0x3FFu) << 13); }
    } else if (e == 31) f = sign | 0x7F800000u | (m << 13);
    else f = sign | ((uint32_t)(e - 15 + 127) << 23) | (m << 13);
    float v; memcpy(&v, &f, 4); return v;
}

// ---------------------------------------------------------------- bit-level writer / reader
class Writer {
public:
    Writer(uint8_t* buf, size_t cap) : b_(buf), cap_bits_(cap * 8) { memset(buf, 0, cap); }
    void u(uint64_t v, int bits) {
        uint8_t t[8]; for (int i = 0; i < 8; i++) t[i] = (uint8_t)(v >> (8 * i));
        if (bits % 8) t[bits / 8] = (uint8_t)(t[bits / 8] << (8 - bits % 8));
        for (int i = 0; i < bits; i++) {
            if (bit_ >= cap_bits_) { over_ = true; return; }
            if ((t[i / 8] >> (7 - i % 8)) & 1u) b_[bit_ / 8] |= (uint8_t)(0x80u >> (bit_ % 8));
            bit_++;
        }
    }
    void i(int64_t v, int bits) { u((uint64_t)v & (bits >= 64 ? ~0ull : ((1ull << bits) - 1)), bits); }
    void f32(float v) { uint32_t x; memcpy(&x, &v, 4); u(x, 32); }
    void f16(float v) { u(f16_from(v), 16); }
    void boolean(bool v) { u(v ? 1 : 0, 1); }
    void skip(int bits) { bit_ += (size_t)bits; if (bit_ > cap_bits_) { bit_ = cap_bits_; over_ = true; } }
    size_t bytes() const { return (bit_ + 7) / 8; }
    size_t bits() const { return bit_; }
    bool overflow() const { return over_; }
private:
    uint8_t* b_; size_t cap_bits_, bit_ = 0; bool over_ = false;
};

class Reader {
public:
    Reader(const uint8_t* buf, size_t n) : b_(buf), n_bits_(n * 8) {}
    uint64_t u(int bits) {
        uint8_t t[8] = {};
        for (int i = 0; i < bits; i++) {
            if (bit_ >= n_bits_) { short_ = true; return 0; }
            if ((b_[bit_ / 8] >> (7 - bit_ % 8)) & 1u) t[i / 8] |= (uint8_t)(0x80u >> (i % 8));
            bit_++;
        }
        if (bits % 8) t[bits / 8] = (uint8_t)(t[bits / 8] >> (8 - bits % 8));
        uint64_t v = 0; for (int i = 0; i < 8; i++) v |= (uint64_t)t[i] << (8 * i);
        return v;
    }
    int64_t i(int bits) { const uint64_t v = u(bits); if (bits < 64 && (v >> (bits - 1)) & 1u) return (int64_t)(v | ~((1ull << bits) - 1)); return (int64_t)v; }
    float f32() { const uint32_t x = (uint32_t)u(32); float v; memcpy(&v, &x, 4); return v; }
    float f16() { return f16_to((uint16_t)u(16)); }
    bool boolean() { return u(1) != 0; }
    void skip(int bits) { bit_ += (size_t)bits; if (bit_ > n_bits_) { bit_ = n_bits_; short_ = true; } }
    size_t bits_left() const { return n_bits_ - bit_; }
    bool too_short() const { return short_; }
private:
    const uint8_t* b_; size_t n_bits_, bit_ = 0; bool short_ = false;
};

// ---------------------------------------------------------------- CAN identifiers
inline uint32_t message_id(uint8_t prio, uint16_t type_id, uint8_t src) { return ((uint32_t)(prio & 31) << 24) | ((uint32_t)type_id << 8) | (src & 127u); }
inline uint32_t anonymous_id(uint8_t prio, uint16_t discriminator, uint16_t type_id) { return ((uint32_t)(prio & 31) << 24) | ((uint32_t)(discriminator & 0x3FFFu) << 10) | ((uint32_t)(type_id & 3u) << 8); }
inline uint32_t service_id(uint8_t prio, uint8_t type_id, bool request, uint8_t dst, uint8_t src) {
    return ((uint32_t)(prio & 31) << 24) | ((uint32_t)type_id << 16) | ((request ? 1u : 0u) << 15) | ((uint32_t)(dst & 127u) << 8) | 0x80u | (src & 127u);
}
inline uint8_t tail(bool sot, bool eot, bool toggle, uint8_t tid) { return (uint8_t)((sot ? 0x80 : 0) | (eot ? 0x40 : 0) | (toggle ? 0x20 : 0) | (tid & 31)); }

// One transfer -> frames. Returns the number of frames (0: too long for `max_frames`). A single-frame transfer has no CRC.
inline int encode_transfer(uint32_t can_id, uint64_t signature, uint8_t tid, const uint8_t* payload, size_t n, Frame* out, int max_frames) {
    if (n <= 7) {
        if (max_frames < 1) return 0;
        out[0].id = can_id; memcpy(out[0].data, payload, n); out[0].data[n] = tail(true, true, false, tid); out[0].len = (uint8_t)(n + 1);
        return 1;
    }
    const uint16_t crc = transfer_crc(signature, payload, n);
    const size_t total = n + 2; const int frames = (int)((total + 6) / 7);
    if (frames > max_frames) return 0;
    size_t k = 0; bool toggle = false;
    for (int f = 0; f < frames; f++) {
        Frame& fr = out[f]; fr.id = can_id; uint8_t m = 0;
        while (m < 7 && k < total) { fr.data[m++] = k == 0 ? (uint8_t)crc : k == 1 ? (uint8_t)(crc >> 8) : payload[k - 2]; k++; }
        fr.data[m] = tail(f == 0, f == frames - 1, toggle, tid); fr.len = (uint8_t)(m + 1); toggle = !toggle;
    }
    return frames;
}

// ---------------------------------------------------------------- what a frame says
struct Header {
    Kind kind = MESSAGE; uint16_t type_id = 0; uint8_t prio = 0, src = 0, dst = 0; bool anonymous = false;
    uint8_t sot = 0, eot = 0, toggle = 0, tid = 0;
};
inline bool parse(const Frame& f, Header& h) {
    if (f.len < 1 || f.len > 8) return false;
    const uint32_t id = f.id & 0x1FFFFFFFu; const uint8_t t = f.data[f.len - 1];
    h.prio = (uint8_t)(id >> 24); h.src = (uint8_t)(id & 127u);
    h.sot = (t >> 7) & 1; h.eot = (t >> 6) & 1; h.toggle = (t >> 5) & 1; h.tid = t & 31;
    if (id & 0x80u) { h.kind = (id >> 15) & 1u ? REQUEST : RESPONSE; h.type_id = (uint8_t)(id >> 16); h.dst = (uint8_t)((id >> 8) & 127u); h.anonymous = false; }
    else {
        h.kind = MESSAGE; h.dst = 0; h.anonymous = h.src == 0;
        h.type_id = h.anonymous ? (uint16_t)((id >> 8) & 3u) : (uint16_t)((id >> 8) & 0xFFFFu);
        if (h.anonymous && !(h.sot && h.eot)) return false;              // anonymous transfers are single-frame only
    }
    return true;
}

// ---------------------------------------------------------------- receiving: frames -> transfers
struct Transfer { Kind kind = MESSAGE; uint16_t type_id = 0; uint8_t src = 0, dst = 0, tid = 0, prio = 0; bool anonymous = false; size_t len = 0; const uint8_t* data = nullptr; };

// Reassembles multi-frame transfers. signature(kind, type_id) says which types are wanted (0 = not interested: the frames are
// dropped without using a slot) and gives the data type signature for the CRC check.
template <size_t kSlots = 8, size_t kMaxPayload = 256>
class Receiver {
public:
    using SignatureFn = uint64_t (*)(Kind, uint16_t, void*);
    Receiver(SignatureFn sig, void* ctx) : sig_(sig), ctx_(ctx) {}
    void set_node_id(uint8_t id) { me_ = id; }

    // One frame. true = a transfer is complete (out.data valid until the next call).
    bool feed(const Frame& f, uint32_t now_ms, Transfer& out) {
        Header h; if (!parse(f, h)) { bad_++; return false; }
        if (h.kind != MESSAGE && h.dst != me_) return false;                                    // a service call between two other nodes
        const uint64_t signature = sig_(h.kind, h.type_id, ctx_); if (!signature) return false;
        const uint8_t n = (uint8_t)(f.len - 1);
        if (h.sot && h.eot) {                                                                   // single frame
            memcpy(single_, f.data, n);
            out.kind = h.kind; out.type_id = h.type_id; out.src = h.src; out.dst = h.dst; out.tid = h.tid; out.prio = h.prio; out.anonymous = h.anonymous;
            out.data = single_; out.len = n; done_++;
            return true;
        }
        Slot* s = find(h, now_ms);
        if (h.sot) {
            if (!s) s = take(h, now_ms);
            if (!s) { dropped_++; return false; }
            s->tid = h.tid; s->toggle = 0; s->len = 0; s->active = true;
            if (n < 2) { s->active = false; bad_++; return false; }
            s->crc = (uint16_t)(f.data[0] | (f.data[1] << 8));
            append(*s, f.data + 2, (size_t)(n - 2));
            s->toggle = 1; s->ms = now_ms;
            return false;
        }
        if (!s || !s->active || h.tid != s->tid || h.toggle != s->toggle) { if (s && s->active) { s->active = false; } bad_++; return false; }
        append(*s, f.data, n); s->toggle ^= 1; s->ms = now_ms;
        if (!h.eot) return false;
        s->active = false;
        if (s->over || transfer_crc(signature, s->buf, s->len) != s->crc) { crc_errors_++; return false; }
        out.kind = h.kind; out.type_id = h.type_id; out.src = h.src; out.dst = h.dst; out.tid = h.tid; out.prio = h.prio; out.anonymous = false;
        out.data = s->buf; out.len = s->len; done_++;
        return true;
    }
    uint32_t transfers() const { return done_; }
    uint32_t crc_errors() const { return crc_errors_; }
    uint32_t bad_frames() const { return bad_; }
    uint32_t no_slot() const { return dropped_; }

private:
    struct Slot { bool active = false, over = false; Kind kind = MESSAGE; uint16_t type_id = 0; uint8_t src = 0, tid = 0, toggle = 0; uint16_t crc = 0; size_t len = 0; uint32_t ms = 0; uint8_t buf[kMaxPayload]; };
    static constexpr uint32_t kTimeoutMs = 2000;
    Slot* find(const Header& h, uint32_t now_ms) {
        for (Slot& s : slots_) if (s.active && s.kind == h.kind && s.type_id == h.type_id && s.src == h.src) { if ((uint32_t)(now_ms - s.ms) > kTimeoutMs) { s.active = false; return &s; } return &s; }
        return nullptr;
    }
    Slot* take(const Header& h, uint32_t now_ms) {
        for (Slot& s : slots_) if (!s.active || (uint32_t)(now_ms - s.ms) > kTimeoutMs) { s.kind = h.kind; s.type_id = h.type_id; s.src = h.src; s.over = false; return &s; }
        return nullptr;
    }
    static void append(Slot& s, const uint8_t* d, size_t n) {
        if (s.len + n > kMaxPayload) { s.over = true; n = kMaxPayload - s.len; }
        memcpy(s.buf + s.len, d, n); s.len += n;
    }
    SignatureFn sig_; void* ctx_; uint8_t me_ = 0;
    Slot slots_[kSlots];
    uint8_t single_[8] = {};
    uint32_t done_ = 0, crc_errors_ = 0, bad_ = 0, dropped_ = 0;
};

} // namespace dcan
