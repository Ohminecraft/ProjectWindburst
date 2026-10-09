/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <atomic>
#include "bb_storage.h"

// ============================================================================
// CardLog: flight logs as FILES on an SD card - /wblog/wb_00001.wblog, wb_00002.wblog ... one per session, the same bytes the
// flash log stores and the app records from the live stream. The card can be taken out and the files opened on a PC.
//
// Why it needs its own task. A card answers a write in 1 .. 5 ms most of the time and in 50 .. 300 ms now and then (it moves
// data around internally). Nothing that has a deadline may wait for that:
//
//   flight loop (core 1) --records--> Recorder ring (16 kB) --every ms--> link task: frames --> ByteFifo (big, in PSRAM)
//                                                                                                   |
//   writer task (core 0, low priority):  work()  <--------------- 4 kB chunks, sync every 250 ms ---+
//
// The link task never touches the card while a session runs: append() only copies into the FIFO. The writer task is the only one
// that calls write() / sync(). Everything that happens on the ground (open, close, list, read back, delete) takes the lock, so
// the two tasks never use the file system at the same time.
// Unlike the internal flash, a card write does not stop the CPU cores: logging to a card costs the flight loop nothing.
//
// What a power cut can lose: what was written after the last sync (at most 250 ms) plus what was still in the FIFO.
// Portable: the card itself is a FileStore (bb_card_esp.h on the ESP32, RAM in the tests).
// ============================================================================
namespace bb {

constexpr char kCardDir[] = "/wblog";                   // the log directory on the card

// ---- file names: wb_<number, 5 digits>.wblog
inline void card_file_name(uint32_t number, char* out, size_t cap) { snprintf(out, cap, "wb_%05lu.wblog", (unsigned long)number); }
// The number of a log file name (a leading path is ignored, upper / lower case both accepted). 0 = not one of ours.
inline uint32_t card_file_number(const char* name) {
    if (!name) return 0;
    const char* b = name; for (const char* p = name; *p; p++) if (*p == '/') b = p + 1;
    auto low = [](char c) { return (c >= 'A' && c <= 'Z') ? (char)(c + 32) : c; };
    if (low(b[0]) != 'w' || low(b[1]) != 'b' || b[2] != '_') return 0;
    uint32_t n = 0; int digits = 0; const char* p = b + 3;
    for (; *p >= '0' && *p <= '9'; p++) { if (++digits > 9) return 0; n = n * 10u + (uint32_t)(*p - '0'); }
    if (digits == 0) return 0;
    const char* ext = ".wblog";
    for (int i = 0; i < 6; i++) if (low(p[i]) != ext[i]) return 0;
    return p[6] == 0 ? n : 0;
}

// The card and its file system, reduced to what the log needs. Log files are addressed by their number.
class FileStore {
public:
    virtual ~FileStore() = default;
    virtual void configure(uint8_t /*mode*/, uint8_t /*mhz*/) {}   // wiring (SPI / SDMMC) and clock; takes effect at the next mount()
    virtual bool mount() = 0;                                // (re)initialise the card and make sure the log directory exists; may take a second
    virtual void unmount() = 0;
    virtual bool alive() = 0;                                // cheap check that the card still answers
    virtual uint64_t total_bytes() = 0;
    virtual uint64_t free_bytes() = 0;
    using ListFn = void (*)(void* ctx, uint32_t number, uint32_t size);
    virtual bool list(ListFn fn, void* ctx) = 0;             // every log file in the log directory, in any order
    virtual bool open_write(uint32_t number) = 0;            // create (or truncate) a log file and keep it open for writing
    virtual bool write(const uint8_t* d, uint32_t n) = 0;    // append to the open file; blocks as long as the card takes
    virtual bool sync() = 0;                                 // data and file size safely on the card
    virtual void close_write() = 0;
    virtual bool read(uint32_t number, uint32_t offset, uint8_t* buf, uint32_t n) = 0;
    virtual bool remove(uint32_t number) = 0;
    virtual const char* name() const { return "sd"; }
    virtual const char* why() const { return ""; }           // why the last mount() failed
};

// The two tasks never use the FileStore at the same time: a mutex on the ESP32, nothing in single-threaded tests.
class Lock {
public:
    virtual ~Lock() = default;
    virtual void lock() {}
    virtual void unlock() {}
};

// Byte queue between the link task (the only writer) and the writer task (the only reader). Wait-free on both sides.
class ByteFifo {
public:
    void init(uint8_t* mem, uint32_t size) {                  // size is rounded down to a power of two
        uint32_t n = 1; while (n * 2u <= size && n < 0x40000000u) n *= 2u;
        mem_ = size ? mem : nullptr; mask_ = mem_ ? n - 1u : 0; head_.store(0); tail_.store(0);
    }
    uint32_t capacity() const { return mem_ ? mask_ + 1u : 0; }
    uint32_t used() const { return head_.load(std::memory_order_acquire) - tail_.load(std::memory_order_acquire); }
    uint32_t room() const { return capacity() - used(); }
    bool push(const uint8_t* d, uint32_t n) {                 // writer
        if (!mem_ || n > room()) return false;
        const uint32_t h = head_.load(std::memory_order_relaxed), i = h & mask_, first = n < capacity() - i ? n : capacity() - i;
        memcpy(mem_ + i, d, first); if (n > first) memcpy(mem_, d + first, n - first);
        head_.store(h + n, std::memory_order_release);
        return true;
    }
    uint32_t peek(uint8_t* out, uint32_t max) const {         // reader: copy without removing
        if (!mem_) return 0;
        const uint32_t t = tail_.load(std::memory_order_relaxed); uint32_t n = head_.load(std::memory_order_acquire) - t;
        if (n > max) n = max;
        const uint32_t i = t & mask_, first = n < capacity() - i ? n : capacity() - i;
        memcpy(out, mem_ + i, first); if (n > first) memcpy(out + first, mem_, n - first);
        return n;
    }
    void consume(uint32_t n) { tail_.store(tail_.load(std::memory_order_relaxed) + n, std::memory_order_release); }   // reader
    void clear() { tail_.store(head_.load(std::memory_order_acquire), std::memory_order_release); }                 // while nobody writes
private:
    uint8_t* mem_ = nullptr; uint32_t mask_ = 0;
    std::atomic<uint32_t> head_{0}, tail_{0};
};

class CardLog : public Storage {
public:
    static constexpr int kMaxSessions = 128;
    static constexpr uint32_t kChunk = 4096;                 // bytes per card write
    static constexpr uint32_t kSyncMs = 250;                 // file size / allocation written to the card this often while logging
    static constexpr uint32_t kRetryMs = 3000;               // no card: look again this often
    static constexpr uint32_t kProbeMs = 3000;               // idle: check that the card is still there
    static constexpr uint32_t kReserveKb = 1024;             // the card is never filled to the last block
    static constexpr uint32_t kMaxSpan = 0xFFF00000u;        // size of the download address range (and the largest file)

    CardLog(FileStore& fs, Lock& lock) : fs_(fs), lock_(lock) {}
    // The write buffer (PSRAM when there is some). Before begin().
    void set_buffer(uint8_t* mem, uint32_t size) { fifo_.init(mem, size); }
    uint32_t buffer_size() const { return fifo_.capacity(); }

    // Mount the card and find the log files on it. Called at boot when the settings select the card; later the writer task
    // mounts it by itself (card pushed in after power-on, card selected in the app without a reboot).
    bool begin() {
        lock_.lock();
        mode_ = want_mode_.load(); mhz_ = want_mhz_.load(); was_enabled_ = enabled();
        const bool ok = scan();
        lock_.unlock();
        return ok;
    }
    // Storage: called by the link task whenever the blackbox settings change. Picked up by the writer task between sessions.
    void configure_device(bool selected, uint8_t mode, uint8_t mhz) override {
        want_mode_.store(mode, std::memory_order_release); want_mhz_.store(mhz, std::memory_order_release); enabled_.store(selected, std::memory_order_release);
    }
    bool enabled() const { return enabled_.load(std::memory_order_acquire); }
    const char* why() const { return fs_.why(); }

    // ------------------------------------------------------------------------------------------------ writer task
    // Call every few milliseconds (it blocks while the card is busy). Returns true when it wrote something.
    bool work(uint32_t now_ms) {
        if (!writing_.load(std::memory_order_acquire)) {
            if (!enabled()) { was_enabled_ = false; return false; }   // another device is selected: the card and its pins are left alone
            if (!was_enabled_) { was_enabled_ = true; retry_ms_ = now_ms - kRetryMs; }   // just selected: look for the card now
            if (want_mode_.load(std::memory_order_acquire) != mode_ || want_mhz_.load(std::memory_order_acquire) != mhz_) {     // rewired in the settings: mount again
                lock_.lock();
                mode_ = want_mode_.load(); mhz_ = want_mhz_.load();
                if (mounted_.load(std::memory_order_acquire)) { fs_.unmount(); mounted_.store(false, std::memory_order_release); }
                retry_ms_ = now_ms - kRetryMs;
                lock_.unlock();
            }
        }
        if (!mounted_.load(std::memory_order_acquire)) {
            if ((uint32_t)(now_ms - retry_ms_) < kRetryMs) return false;
            retry_ms_ = now_ms;
            lock_.lock(); scan(); lock_.unlock();
            return false;
        }
        if (!writing_.load(std::memory_order_acquire)) {
            if (erasing_ || (uint32_t)(now_ms - probe_ms_) < kProbeMs) return false;
            probe_ms_ = now_ms;
            lock_.lock();
            if (!writing_.load(std::memory_order_acquire) && !fs_.alive()) { fs_.unmount(); mounted_.store(false, std::memory_order_release); retry_ms_ = now_ms; }
            lock_.unlock();
            return false;
        }
        bool wrote = false;
        lock_.lock();
        if (writing_.load(std::memory_order_acquire)) {
            const bool sync_due = (uint32_t)(now_ms - sync_ms_) >= kSyncMs;
            for (int k = 0; k < 8; k++) {                         // at most 32 kB per call, so the lock is released regularly
                const uint32_t have = fifo_.used();
                if (have == 0 || (have < kChunk && !sync_due)) break;
                const uint32_t n = fifo_.peek(chunk_, kChunk);
                if (!fs_.write(chunk_, n)) { fail_locked(now_ms); break; }
                fifo_.consume(n); written_.fetch_add(n, std::memory_order_release); dirty_ = true; wrote = true;
            }
            if (writing_.load(std::memory_order_acquire) && dirty_ && sync_due) {
                if (!fs_.sync()) fail_locked(now_ms); else { dirty_ = false; sync_ms_ = now_ms; }
            }
        }
        lock_.unlock();
        return wrote;
    }
    bool mounted() const { return mounted_.load(std::memory_order_acquire); }

    // ------------------------------------------------------------------------------------------------ Storage (link task)
    bool ok() const override { return mounted_.load(std::memory_order_acquire); }
    uint32_t capacity() const override { const uint64_t b = (uint64_t)total_kb_.load() * 1024u; return b > 0xFFFFFFFFu ? 0xFFFFFFFFu : (uint32_t)b; }
    uint32_t capacity_kb() const override { return total_kb_.load(std::memory_order_acquire); }
    uint32_t used_kb() const override {
        const uint32_t t = total_kb_.load(std::memory_order_acquire), f = free_kb_.load(std::memory_order_acquire);
        const uint32_t base = t > f ? t - f : 0;
        return base + (writing_.load(std::memory_order_acquire) ? written_.load(std::memory_order_acquire) / 1024u : 0u);
    }
    uint32_t used() const override { return span_.load(std::memory_order_acquire); }
    bool needs_erase() const override { return false; }
    bool io_error() const override { return io_error_.load(std::memory_order_acquire); }
    bool full() const override { return ok() && !writing() && writable() < 64u * 1024u; }
    bool writing() const override { return writing_.load(std::memory_order_acquire); }

    int sessions() const override { return (int)total_files_.load(std::memory_order_acquire); }
    int listed() const override { lock_.lock(); const int n = n_; lock_.unlock(); return n; }
    uint32_t session_start(int i) const override { lock_.lock(); const uint32_t v = (i >= 0 && i < n_) ? start_[i] : 0; lock_.unlock(); return v; }
    uint32_t session_size(int i) const override { lock_.lock(); const uint32_t v = (i >= 0 && i < n_) ? size_[i] : 0; lock_.unlock(); return v; }
    uint32_t session_number(int i) const { lock_.lock(); const uint32_t v = (i >= 0 && i < n_) ? num_[i] : 0; lock_.unlock(); return v; }   // the file's number

    bool verify_step() override { return false; }
    uint32_t writable() const override {
        if (!ok() || erasing_) return 0;
        uint64_t b = (uint64_t)free_kb_.load(std::memory_order_acquire) * 1024u;
        const uint64_t reserve = (uint64_t)kReserveKb * 1024u;
        if (b <= reserve) return 0;
        b -= reserve;
        if (b > kMaxSpan) b = kMaxSpan;                           // one file never grows past the address range (FAT32 ends at 4 GB anyway)
        if (writing()) { const uint64_t taken = (uint64_t)written_.load(std::memory_order_acquire) + fifo_.used(); if (taken >= b) return 0; b -= taken; }
        return (uint32_t)b;
    }
    bool start_session() override {
        if (!ok() || writing() || erasing_ || fifo_.capacity() < 2u * kChunk || writable() < 64u * 1024u) return false;
        lock_.lock();
        const bool opened = fs_.open_write(next_number_);
        if (opened) { fifo_.clear(); written_.store(0, std::memory_order_release); cur_number_ = next_number_; dirty_ = false; peak_ = 0; writing_.store(true, std::memory_order_release); }
        lock_.unlock();
        if (!opened) { lock_.lock(); fail_locked(retry_ms_); lock_.unlock(); }
        return opened;
    }
    bool append(const uint8_t* d, uint32_t n) override {
        if (!writing() || n > writable()) return false;
        if (!fifo_.push(d, n)) { peak_ = 100; return false; }     // the card is behind: the caller keeps the frame, records pile up (and are counted when they are lost)
        const uint32_t pct = (uint32_t)((uint64_t)fifo_.used() * 100u / fifo_.capacity());
        if (pct > peak_) peak_ = (uint8_t)pct;
        return true;
    }
    uint32_t staged() const override { return writing() ? fifo_.used() : 0; }
    uint32_t pump(bool) override { return 0; }                    // the writer task moves the bytes
    // Writes what is still queued, then closes the file. On the ground (the link task waits for the card here).
    void end_session() override {
        lock_.lock();
        if (writing_.load(std::memory_order_acquire)) {
            bool good = true;
            while (good && fifo_.used()) { const uint32_t n = fifo_.peek(chunk_, kChunk); good = fs_.write(chunk_, n); if (good) { fifo_.consume(n); written_.fetch_add(n, std::memory_order_release); } }
            if (good) good = fs_.sync();
            if (!good) fail_locked(retry_ms_);
            else {
                fs_.close_write(); writing_.store(false, std::memory_order_release);
                const uint32_t sz = written_.load(std::memory_order_acquire);
                if (sz == 0) fs_.remove(cur_number_);              // nothing was logged: no empty file
                else { add(cur_number_, sz); total_files_.fetch_add(1, std::memory_order_release); layout(); }
                next_number_ = cur_number_ + 1u;
                refresh_space();
            }
        }
        fifo_.clear();
        lock_.unlock();
    }

    bool erase_start() override {
        if (!ok() || writing()) return false;
        erasing_ = true; erase_total_ = total_files_.load(std::memory_order_acquire); erase_done_ = 0;
        return true;
    }
    // Deletes a few log files per call (only wb_*.wblog in the log directory - nothing else on the card is touched).
    bool erase_step() override {
        if (!erasing_) return false;
        lock_.lock();
        Pick pick; bool good = fs_.list(&CardLog::pick_cb, &pick);
        for (int i = 0; good && i < pick.n; i++) { good = fs_.remove(pick.num[i]); if (good) erase_done_++; }
        if (!good) { erasing_ = false; fail_locked(retry_ms_); }
        else if (pick.n == 0) { erasing_ = false; n_ = 0; total_files_.store(0, std::memory_order_release); next_number_ = 1; layout(); refresh_space(); }
        lock_.unlock();
        return erasing_;
    }
    uint8_t erase_percent() const override {
        if (!erasing_) return 100;
        return erase_total_ ? (uint8_t)(erase_done_ >= erase_total_ ? 99u : erase_done_ * 100u / erase_total_) : (uint8_t)0;
    }

    bool read(uint32_t addr, uint8_t* buf, uint32_t n) override {
        if (!ok() || writing()) return false;
        lock_.lock();
        bool good = true;
        while (good && n) {                                    // an address outside every file ends the read: nothing past the end of the data
            int i = 0; while (i < n_ && !(addr >= start_[i] && addr - start_[i] < size_[i])) i++;
            if (i == n_) { good = false; break; }
            const uint32_t off = addr - start_[i], part = n < size_[i] - off ? n : size_[i] - off;
            good = fs_.read(num_[i], off, buf, part);
            addr += part; buf += part; n -= part;
        }
        lock_.unlock();
        return good;
    }
    const char* store_name() const override { return fs_.name(); }
    uint8_t take_buffer_peak() override { const uint8_t p = peak_; peak_ = 0; return p; }
    uint32_t next_number() const { return next_number_; }

private:
    // ---- everything below runs with the lock held
    void fail_locked(uint32_t now_ms) {                           // the card stopped answering: give up the session, look for a card again later
        io_error_.store(true, std::memory_order_release);
        if (writing_.load(std::memory_order_acquire)) { fs_.close_write(); writing_.store(false, std::memory_order_release); }
        fs_.unmount(); mounted_.store(false, std::memory_order_release); retry_ms_ = now_ms;
    }
    bool scan() {
        n_ = 0; total_files_.store(0, std::memory_order_release); max_number_ = 0; span_.store(0, std::memory_order_release);
        fs_.configure(mode_, mhz_);
        if (!fs_.mount() || !fs_.list(&CardLog::scan_cb, this)) { fs_.unmount(); mounted_.store(false, std::memory_order_release); return false; }
        next_number_ = max_number_ + 1u;
        layout(); refresh_space();
        io_error_.store(false, std::memory_order_release); mounted_.store(true, std::memory_order_release);
        return true;
    }
    static void scan_cb(void* ctx, uint32_t number, uint32_t size) {
        CardLog& c = *static_cast<CardLog*>(ctx);
        if (number > c.max_number_) c.max_number_ = number;       // an empty file (power cut right after it was created) still uses up its number
        if (size == 0) return;
        c.total_files_.fetch_add(1, std::memory_order_release);
        c.add(number, size);
    }
    // keeps the newest kMaxSessions, sorted by number
    void add(uint32_t number, uint32_t size) {
        if (n_ == kMaxSessions) {
            if (number < num_[0]) return;
            memmove(num_, num_ + 1, sizeof(uint32_t) * (kMaxSessions - 1)); memmove(size_, size_ + 1, sizeof(uint32_t) * (kMaxSessions - 1)); n_--;
        }
        int i = n_; while (i > 0 && num_[i - 1] > number) { num_[i] = num_[i - 1]; size_[i] = size_[i - 1]; i--; }
        num_[i] = number; size_[i] = size; n_++;
    }
    // download addresses: the listed files one after the other; the oldest ones leave the list when the range would not hold them
    void layout() {
        uint64_t total = 0; for (int i = 0; i < n_; i++) total += size_[i];
        while (n_ > 0 && total > kMaxSpan) { total -= size_[0]; memmove(num_, num_ + 1, sizeof(uint32_t) * (size_t)(n_ - 1)); memmove(size_, size_ + 1, sizeof(uint32_t) * (size_t)(n_ - 1)); n_--; }
        uint32_t at = 0; for (int i = 0; i < n_; i++) { start_[i] = at; at += size_[i]; }
        span_.store(at, std::memory_order_release);
    }
    void refresh_space() {
        const uint64_t t = fs_.total_bytes() / 1024u, f = fs_.free_bytes() / 1024u;
        total_kb_.store(t > 0xFFFFFFFFu ? 0xFFFFFFFFu : (uint32_t)t, std::memory_order_release);
        free_kb_.store(f > 0xFFFFFFFFu ? 0xFFFFFFFFu : (uint32_t)f, std::memory_order_release);
    }
    struct Pick { uint32_t num[4]; int n = 0; };
    static void pick_cb(void* ctx, uint32_t number, uint32_t) { Pick& p = *static_cast<Pick*>(ctx); if (p.n < 4) p.num[p.n++] = number; }

    FileStore& fs_; Lock& lock_; ByteFifo fifo_;
    std::atomic<bool> mounted_{false}, io_error_{false}, writing_{false}, enabled_{false};
    std::atomic<uint8_t> want_mode_{0}, want_mhz_{20}; uint8_t mode_ = 0, mhz_ = 20;
    std::atomic<uint32_t> written_{0}, total_kb_{0}, free_kb_{0}, span_{0}, total_files_{0};
    uint32_t num_[kMaxSessions] = {}, size_[kMaxSessions] = {}, start_[kMaxSessions] = {}; int n_ = 0;
    uint32_t next_number_ = 1, cur_number_ = 0, max_number_ = 0;
    uint32_t retry_ms_ = 0, probe_ms_ = 0, sync_ms_ = 0; bool dirty_ = false, was_enabled_ = false;
    std::atomic<bool> erasing_{false}; uint32_t erase_total_ = 0, erase_done_ = 0;
    uint8_t peak_ = 0;
    uint8_t chunk_[kChunk];
};

} // namespace bb
