/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "settings_store.h"
#include "../port/qw_flash.h"

// ============================================================================
// The settings records (wbp::Storage: key -> bytes) in internal flash with quad-word ECC (port/qw_flash.h), for a chip
// without NVS: the STM32H5. The ESP32 keeps them in NVS (app/link_arduino.h: NvsStorage).
//
// The region is split into two halves. One is active: a 16-byte head ("WBKV", generation) and then records, one after the
// other, each starting on a quad-word:
//      head (16)  : magic "WR", key length, data length, crc of these (16 bits), crc32 of key + data
//      body       : key, then data, padded to the next 16 bytes (the padding is never programmed separately)
// Writing a key APPENDS a record (head first, then the body); the newest record of a key that checks out is its value.
// When the active half is full, the other one is erased, the newest record of every key is copied into it (with the new value
// of the key being written), and only then is its head programmed - with the next generation. That head is the commit:
//   * power lost while a record is written: its head is there and its crc does not match - skipped, the older value stays;
//   * power lost during a compaction: the other half has no head, the old one is still the active one, nothing is lost;
//   * a record whose head itself is not readable stops the scan: the rest of that half counts as full, and the next write
//     compacts into the other half.
// A write of the value a key already has programs nothing (a SAVE in the app writes every section).
// One thread uses it (the link task saves only while disarmed). Portable: test_proto runs it against test/sim_qwflash.h.
// ============================================================================
namespace wbp {

class KvFlashStorage : public Storage {
public:
    static constexpr int kMaxKeys = 40;
    static constexpr uint32_t kMaxKey = 15, kMaxData = 2048;
    explicit KvFlashStorage(port::QwFlash& f) : f_(f) {}

    // Finds the active half (formats a blank or unreadable region). false: the region is too small / the flash refuses.
    bool begin() {
        ok_ = false; n_keys_ = 0; end_ = 0; full_ = false;
        const uint32_t sec = f_.sector();
        if (!sec || f_.size() < 2 * sec || f_.size() % (2 * sec)) return false;
        half_ = f_.size() / 2;
        uint32_t gen[2] = {0, 0}; bool valid[2] = {false, false};
        for (int h = 0; h < 2; h++) valid[h] = read_group(h, gen[h]);
        if (!valid[0] && !valid[1]) {                      // blank or foreign: start afresh in half 0
            if (!erase_half(0) || !write_group(0, 1)) return false;
            active_ = 0; gen_ = 1;
        } else {
            active_ = (valid[0] && (!valid[1] || (int32_t)(gen[0] - gen[1]) > 0)) ? 0 : 1;
            gen_ = gen[active_];
        }
        scan();
        ok_ = true;
        return true;
    }
    bool ok() const { return ok_; }

    int read(const char* key, uint8_t* buf, size_t cap) override {
        if (!ok_) return -1;
        const int i = find(key); if (i < 0) return -1;
        RecHead r; if (!read_head(keys_[i].at, r)) return -1;
        if (r.dlen > cap) return -1;
        if (!f_.read(base(active_) + keys_[i].at + 16 + r.klen, buf, r.dlen)) return -1;
        return (int)r.dlen;
    }

    bool write(const char* key, const uint8_t* data, size_t n) override {
        if (!ok_) return false;
        const size_t klen = strlen(key);
        if (klen == 0 || klen > kMaxKey || n > kMaxData) return false;
        const int i = find(key);
        if (i >= 0 && same(keys_[i].at, data, n)) return true;            // unchanged: nothing to program
        if (i < 0 && n_keys_ >= kMaxKeys) return false;
        const uint32_t need = rec_size((uint32_t)klen, (uint32_t)n);
        if (16 + need > half_) return false;
        if (full_ || end_ + need > half_) return compact(key, data, (uint32_t)n);
        uint32_t at = 0;
        if (!append(active_, end_, key, data, (uint32_t)n, at)) { full_ = true; return compact(key, data, (uint32_t)n); }
        end_ = at + need;
        remember(key, at);
        return true;
    }

    // ---- for the boot report and the tests
    uint32_t generation() const { return gen_; }
    uint32_t used() const { return end_; }
    uint32_t half_size() const { return half_; }
    int keys() const { return n_keys_; }
    uint32_t compactions() const { return compactions_; }

private:
    static constexpr uint32_t kGroupMagic = 0x564B4257u;   // "WBKV"
    static constexpr uint16_t kRecMagic = 0x5257;          // "WR"
    struct RecHead { uint16_t magic; uint8_t klen, rsv; uint16_t dlen, hcrc; uint32_t crc, rsv2; };
    static_assert(sizeof(RecHead) == 16, "one quad-word");
    struct Key { char name[kMaxKey + 1]; uint32_t at; };

    static uint32_t crc32(const uint8_t* d, size_t n, uint32_t c = 0xFFFFFFFFu) {
        for (size_t i = 0; i < n; i++) { c ^= d[i]; for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u))); }
        return c;
    }
    static uint16_t head_crc(const RecHead& r) { const uint8_t b[6] = {(uint8_t)r.magic, (uint8_t)(r.magic >> 8), r.klen, r.rsv, (uint8_t)r.dlen, (uint8_t)(r.dlen >> 8)};
                                                 return (uint16_t)(crc32(b, 6) ^ 0xFFFFu); }
    static uint32_t rec_size(uint32_t klen, uint32_t n) { return 16 + (klen + n + 15) / 16 * 16; }
    uint32_t base(int h) const { return (uint32_t)h * half_; }

    bool read_group(int h, uint32_t& gen) {
        uint32_t w[4]; if (!f_.read(base(h), (uint8_t*)w, 16)) return false;
        if (w[0] != kGroupMagic || w[2] != (crc32((const uint8_t*)&w[1], 4) ^ 0xFFFFFFFFu)) return false;
        gen = w[1]; return true;
    }
    bool write_group(int h, uint32_t gen) {
        uint32_t w[4] = {kGroupMagic, gen, 0, 0xFFFFFFFFu};
        w[2] = crc32((const uint8_t*)&w[1], 4) ^ 0xFFFFFFFFu;
        return f_.program(base(h), (const uint8_t*)w);
    }
    bool erase_half(int h) {
        for (uint32_t a = 0; a < half_; a += f_.sector()) if (!f_.erase_sector(base(h) + a)) return false;
        return true;
    }
    bool read_head(uint32_t at, RecHead& r) { return f_.read(base(active_) + at, (uint8_t*)&r, 16); }
    static bool blank(const uint8_t* p, size_t n) { for (size_t i = 0; i < n; i++) if (p[i] != 0xFF) return false; return true; }
    bool head_ok(const RecHead& r) const { return r.magic == kRecMagic && r.hcrc == head_crc(r) && r.klen >= 1 && r.klen <= kMaxKey && r.dlen <= kMaxData; }

    // The index of the active half, and where the next record goes.
    void scan() {
        n_keys_ = 0; full_ = false;
        uint32_t at = 16;
        while (at + 16 <= half_) {
            RecHead r; if (!read_head(at, r)) { full_ = true; break; }
            if (blank((const uint8_t*)&r, 16)) break;                         // erased: the end
            if (!head_ok(r)) { full_ = true; break; }                         // unreadable: no way to know its length
            const uint32_t sz = rec_size(r.klen, r.dlen);
            if (at + sz > half_) { full_ = true; break; }
            char key[kMaxKey + 1]; uint8_t tmp[64];
            bool good = f_.read(base(active_) + at + 16, (uint8_t*)key, r.klen);
            key[good ? r.klen : 0] = 0;
            uint32_t c = crc32((const uint8_t*)key, r.klen);
            for (uint32_t done = 0; good && done < r.dlen; ) {
                const uint32_t k = r.dlen - done > sizeof(tmp) ? (uint32_t)sizeof(tmp) : r.dlen - done;
                good = f_.read(base(active_) + at + 16 + r.klen + done, tmp, k); c = crc32(tmp, k, c); done += k;
            }
            if (good && (c ^ 0xFFFFFFFFu) == r.crc) remember(key, at);
            at += sz;
        }
        end_ = at > half_ ? half_ : at;
    }
    int find(const char* key) const { for (int i = 0; i < n_keys_; i++) if (!strcmp(keys_[i].name, key)) return i; return -1; }
    void remember(const char* key, uint32_t at) {
        int i = find(key);
        if (i < 0) { if (n_keys_ >= kMaxKeys) return; i = n_keys_++; size_t l = strlen(key); if (l > kMaxKey) l = kMaxKey; memcpy(keys_[i].name, key, l); keys_[i].name[l] = 0; }
        keys_[i].at = at;
    }
    bool same(uint32_t at, const uint8_t* data, size_t n) {
        RecHead r; if (!read_head(at, r) || r.dlen != n) return false;
        uint8_t tmp[64];
        for (size_t done = 0; done < n; ) {
            const size_t k = n - done > sizeof(tmp) ? sizeof(tmp) : n - done;
            if (!f_.read(base(active_) + at + 16 + r.klen + (uint32_t)done, tmp, (uint32_t)k) || memcmp(tmp, data + done, k)) return false;
            done += k;
        }
        return true;
    }
    // Programs one record in half h at offset at (head first). The body is assembled quad-word by quad-word.
    bool append(int h, uint32_t at, const char* key, const uint8_t* data, uint32_t n, uint32_t& where) {
        RecHead r{}; r.magic = kRecMagic; r.klen = (uint8_t)strlen(key); r.rsv = 0xFF; r.dlen = (uint16_t)n; r.hcrc = head_crc(r);
        r.crc = crc32((const uint8_t*)key, r.klen); r.crc = crc32(data, n, r.crc) ^ 0xFFFFFFFFu; r.rsv2 = 0xFFFFFFFFu;
        if (!f_.program(base(h) + at, (const uint8_t*)&r)) return false;
        const uint32_t body = r.klen + n;
        for (uint32_t q = 0; q < body; q += 16) {
            uint8_t quad[16]; memset(quad, 0xFF, 16);
            for (uint32_t k = 0; k < 16 && q + k < body; k++) { const uint32_t i = q + k; quad[k] = i < r.klen ? (uint8_t)key[i] : data[i - r.klen]; }
            if (!f_.program(base(h) + at + 16 + q, quad)) return false;
        }
        where = at;
        return true;
    }
    // Copies the newest record of every key into the other half (the new value for `key`), then commits it with its head.
    bool compact(const char* key, const uint8_t* data, uint32_t n) {
        const int other = 1 - active_;
        if (!erase_half(other)) return false;
        uint32_t at = 16;
        Key moved[kMaxKeys]; int moved_n = 0;
        for (int i = 0; i < n_keys_; i++) {
            if (!strcmp(keys_[i].name, key)) continue;
            RecHead r; if (!read_head(keys_[i].at, r)) return false;
            const uint32_t sz = rec_size(r.klen, r.dlen);
            if (at + sz > half_) return false;
            for (uint32_t q = 0; q < sz; q += 16) {
                uint8_t quad[16];
                if (!f_.read(base(active_) + keys_[i].at + q, quad, 16) || !f_.program(base(other) + at + q, quad)) return false;
            }
            moved[moved_n] = keys_[i]; moved[moved_n].at = at; moved_n++;
            at += sz;
        }
        const uint32_t need = rec_size((uint32_t)strlen(key), n);
        if (at + need > half_) return false;
        uint32_t where = 0;
        if (!append(other, at, key, data, n, where)) return false;
        if (!write_group(other, gen_ + 1)) return false;                      // the commit
        active_ = other; gen_++; compactions_++;
        n_keys_ = 0;
        for (int i = 0; i < moved_n; i++) remember(moved[i].name, moved[i].at);
        remember(key, where);
        end_ = at + need; full_ = false;
        return true;
    }

    port::QwFlash& f_;
    bool ok_ = false, full_ = false; int active_ = 0; uint32_t gen_ = 0, half_ = 0, end_ = 0, compactions_ = 0;
    Key keys_[kMaxKeys]; int n_keys_ = 0;
};

} // namespace wbp
