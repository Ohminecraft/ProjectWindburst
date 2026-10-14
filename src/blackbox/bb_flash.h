/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>
#include <string.h>
#include "bb_format.h"
#include "bb_storage.h"

// ============================================================================
// FlashLog: blackbox sessions in a raw NOR-flash region, one after the other (the way Betaflight's flashfs does it).
//
// Why not a file system: on the ESP32 every flash operation stops BOTH cores for its duration (the cache is switched off).
// A sector erase takes ~45 ms - a file system erases whenever it needs a block, i.e. in the middle of a flight. Here the region is
// erased only on request, on the ground; in flight there are only page programs (256 bytes, well under a millisecond each).
//
// Layout: a session starts on a 4 kB sector boundary with a BB_HEADER frame whose text begins with "WBLOG"; everything after it
// up to the next session start (or the first erased page) belongs to it. Nothing else is stored: no directory that a power cut
// could leave half-written. After a reset the sessions are found again by looking at the first bytes of every sector.
//
// NOR flash rules this code relies on and checks:
//   * programming can only clear bits, so data is only ever written to pages that are still erased (0xFF);
//     the free space is VERIFIED to be erased before it is used (a region that held another file system, or an interrupted
//     erase, is reported as "needs erase" instead of producing a corrupt log);
//   * erase runs from the END of the region towards the start, so an interrupted erase leaves "sessions, then free space".
// A log frame is at most 248 bytes, so every 256-byte page inside a session contains at least one frame header: a page that
// is entirely 0xFF is free space, which is how the end of the data is found.
// Portable. All calls are made by ONE task (the link task).
// ============================================================================
namespace bb {

constexpr uint32_t kSector = 4096, kPage = 256, kEraseBlock = 65536;

class BlockStore {                               // the flash region: an esp_partition on the ESP32, RAM in the tests
public:
    virtual ~BlockStore() = default;
    virtual uint32_t size() const = 0;                                       // bytes, a multiple of kSector
    virtual bool read(uint32_t addr, uint8_t* buf, uint32_t n) = 0;
    virtual bool write(uint32_t addr, const uint8_t* data, uint32_t n) = 0;  // NOR program
    virtual bool erase(uint32_t addr, uint32_t n) = 0;                       // addr and n multiples of kSector
    virtual const char* name() const { return "flash"; }
    virtual void sync() {}                                                   // end of a session: nothing may wait in RAM (bb_qword.h)
};

class FlashLog : public Storage {
public:
    static constexpr int kMaxSessions = 128;

    // Finds the sessions and the end of the data. Takes ~size / 4096 short reads.
    bool begin(BlockStore* store) {
        st_ = store; n_sessions_ = 0; total_sessions_ = 0; head_ = 0; verified_ = 0; needs_erase_ = false; writing_ = false; erasing_ = false; stage_n_ = 0; io_error_ = false;
        if (!st_ || st_->size() < 4 * kSector || st_->size() % kSector) { st_ = nullptr; return false; }
        const uint32_t size = st_->size();
        uint32_t last_start = 0; bool any = false, first_is_ours = false;
        for (uint32_t a = 0; a < size; a += kSector) {
            uint8_t b[12];
            if (!st_->read(a, b, sizeof(b))) { io_error_ = true; return false; }
            if (is_session_start(b)) { add_session(a); last_start = a; any = true; if (a == 0) first_is_ours = true; }
        }
        // the region must begin with a session of ours or be blank; anything else (an old file system, another firmware's data) is not touched
        if (!first_is_ours && !page_erased(0)) { needs_erase_ = true; head_ = size; verified_ = size; n_sessions_ = 0; total_sessions_ = 0; return !io_error_; }
        // end of data: the first all-erased page at or after the last session start (binary search over "data, then free space")
        uint32_t lo = (any ? last_start : 0) / kPage, hi = size / kPage;
        while (lo < hi) { const uint32_t mid = lo + (hi - lo) / 2; if (page_erased(mid)) hi = mid; else lo = mid + 1; }
        head_ = lo * kPage; verified_ = next_start();
        return !io_error_;
    }
    bool ok() const override { return st_ != nullptr; }
    uint32_t capacity() const override { return st_ ? st_->size() : 0; }
    uint32_t capacity_kb() const override { return capacity() / 1024u; }
    uint32_t used_kb() const override { return (head_ + 1023u) / 1024u; }
    uint32_t used() const override { return head_; }                                   // bytes from the start of the region that hold data (whole pages once a session is closed)
    bool needs_erase() const override { return needs_erase_; }
    bool io_error() const override { return io_error_; }
    bool full() const override { return st_ && !needs_erase_ && next_start() + kSector > capacity(); }
    bool writing() const override { return writing_; }

    // ---- sessions
    int sessions() const override { return total_sessions_; }
    // index 0 = oldest listed. When more than kMaxSessions exist, the oldest ones are no longer listed (they are still in the region).
    int listed() const override { return n_sessions_; }
    uint32_t session_start(int i) const override { return i >= 0 && i < n_sessions_ ? start_[i] : 0; }
    uint32_t session_size(int i) const override { if (i < 0 || i >= n_sessions_) return 0; return (i + 1 < n_sessions_ ? start_[i + 1] : head_) - start_[i]; }

    // ---- free-space check. Everything from the next session start up to verified_end() has been READ and found erased; only that
    // part is ever written. Call repeatedly on the ground: one sector per call. True while there is more to check.
    bool verify_step() override {
        if (!st_ || needs_erase_ || erasing_ || verified_ >= capacity()) return false;
        uint8_t b[kPage];
        for (uint32_t a = verified_; a < verified_ + kSector; a += kPage) {
            if (!st_->read(a, b, kPage)) { io_error_ = true; return false; }
            for (uint32_t i = 0; i < kPage; i++) if (b[i] != 0xFF) { needs_erase_ = true; return false; }
        }
        verified_ += kSector;
        return verified_ < capacity();
    }
    uint32_t verified_end() const { return verified_; }
    // bytes a session can still take right now (the running one, or a new one)
    uint32_t writable() const override {
        if (!st_ || needs_erase_ || erasing_) return 0;
        const uint32_t s = writing_ ? pos_ + stage_n_ : next_start();
        return verified_ > s ? verified_ - s : 0;
    }

    // ---- writing a session
    bool start_session() override {
        if (!st_ || writing_ || erasing_ || needs_erase_) return false;
        const uint32_t s = next_start();
        if (s + kSector > verified_) return false;                           // no (verified) room for even a header
        pos_ = s; stage_n_ = 0; writing_ = true; session_at_ = s; session_listed_ = false;
        return true;
    }
    // Appends bytes to the session. They wait in a small buffer; pump() programs them page by page. Returns false when they do
    // not fit: the buffer is full for the moment (stage_room()), or the region is (writable()). Nothing of the call is taken then.
    bool append(const uint8_t* d, uint32_t n) override {
        if (!writing_) return false;
        if (pos_ + stage_n_ + n > verified_ || n > sizeof(stage_) - stage_n_) return false;
        memcpy(stage_ + stage_n_, d, n); stage_n_ += n;
        return true;
    }
    uint32_t stage_room() const { return (uint32_t)sizeof(stage_) - stage_n_; }
    uint32_t staged() const override { return stage_n_; }
    // Programs at most ONE page (so one short stall of the flight loop per call). all = also a partly filled page (end of session,
    // periodic flush). Returns the number of bytes programmed.
    uint32_t pump(bool all) override {
        if (!writing_ || stage_n_ == 0) return 0;
        const uint32_t room = kPage - (pos_ % kPage);                         // up to the end of the current page
        uint32_t n = stage_n_ < room ? stage_n_ : room;
        if (n < room && !all) return 0;                                       // wait for a full page
        if (!st_->write(pos_, stage_, n)) { io_error_ = true; writing_ = false; return 0; }
        pos_ += n; head_ = pos_;
        memmove(stage_, stage_ + n, stage_n_ - n); stage_n_ -= n;
        if (!session_listed_) { add_session(session_at_); session_listed_ = true; }
        return n;
    }
    void end_session() override {
        while (writing_ && stage_n_) if (!pump(true)) break;
        if (writing_ && st_) st_->sync();
        writing_ = false; stage_n_ = 0;
        head_ = (head_ + kPage - 1) / kPage * kPage;                       // "used" counts whole pages: the same number a scan after a reset finds
        if (verified_ < next_start()) verified_ = next_start();
    }

    // ---- erase everything (on the ground). erase_step() erases one 64 kB block per call, from the end towards the start.
    bool erase_start() override {
        if (!st_ || writing_) return false;
        erasing_ = true; erase_at_ = capacity(); stage_n_ = 0;
        return true;
    }
    bool erasing() const { return erasing_; }
    // True while more remains.
    bool erase_step() override {
        if (!erasing_) return false;
        const uint32_t n = erase_at_ % kEraseBlock ? erase_at_ % kEraseBlock : (erase_at_ < kEraseBlock ? erase_at_ : kEraseBlock);
        erase_at_ -= n;
        if (!st_->erase(erase_at_, n)) { io_error_ = true; erasing_ = false; needs_erase_ = true; return false; }
        if (erase_at_ == 0) { erasing_ = false; needs_erase_ = false; io_error_ = false; head_ = 0; verified_ = capacity(); n_sessions_ = 0; total_sessions_ = 0; }
        return erasing_;
    }
    uint8_t erase_percent() const override { return erasing_ && capacity() ? (uint8_t)(100u - (uint64_t)erase_at_ * 100u / capacity()) : (uint8_t)100; }

    // ---- reading back (download). Only data below used() exists.
    bool read(uint32_t addr, uint8_t* buf, uint32_t n) override {
        if (!st_ || addr >= head_ || n > head_ - addr) return false;
        return st_->read(addr, buf, n);
    }
    const char* store_name() const override { return st_ ? st_->name() : "none"; }

    // A sector that begins with a BB_HEADER frame, sequence 0, whose payload starts with "WBLOG".
    static bool is_session_start(const uint8_t* b) {
        return b[0] == 0xA5 && b[1] == 0x5A && b[4] == kMsgHeader && b[5] == 0 && b[6] == 'W' && b[7] == 'B' && b[8] == 'L' && b[9] == 'O' && b[10] == 'G';
    }

private:
    uint32_t next_start() const { return (head_ + kSector - 1) / kSector * kSector; }
    bool page_erased(uint32_t page) {
        uint8_t b[kPage];
        if (!st_->read(page * kPage, b, kPage)) { io_error_ = true; return false; }
        for (uint32_t i = 0; i < kPage; i++) if (b[i] != 0xFF) return false;
        return true;
    }
    void add_session(uint32_t at) {
        total_sessions_++;
        if (n_sessions_ == kMaxSessions) { memmove(start_, start_ + 1, sizeof(uint32_t) * (kMaxSessions - 1)); n_sessions_--; }
        start_[n_sessions_++] = at;
    }

    BlockStore* st_ = nullptr;
    uint32_t start_[kMaxSessions] = {}; int n_sessions_ = 0, total_sessions_ = 0;
    uint32_t head_ = 0, verified_ = 0, pos_ = 0, erase_at_ = 0, session_at_ = 0;
    bool needs_erase_ = false, writing_ = false, erasing_ = false, io_error_ = false, session_listed_ = false;
    uint8_t stage_[2048]; uint32_t stage_n_ = 0;
};

} // namespace bb
