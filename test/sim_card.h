// An SD card in RAM for the blackbox tests and the simulator. It behaves like a card, not like a vector:
//   * a write BLOCKS: a few milliseconds normally, a long pause now and then (the card's own housekeeping). The time is handed to
//     the `busy` hook; whoever plays the writer task does not run again until that time has passed - everything else goes on;
//   * what was written after the last sync() is LOST by a power cut (the file size is only updated by a sync);
//   * the card can be pulled out and pushed back in, be write protected, be full, or arrive unformatted.
#pragma once
#include <functional>
#include <map>
#include <string>
#include <vector>
#include <stdint.h>
#include <string.h>
#include "../src/blackbox/bb_card.h"

struct MemCard : bb::FileStore {
    struct F { std::vector<uint8_t> data; size_t synced = 0; };
    std::map<uint32_t, F> files;                // log files by number
    std::map<std::string, size_t> other;        // anything else in the log directory: must never be touched
    uint64_t size_bytes = 64ull * 1024 * 1024;
    bool inserted = true, formatted = true, read_only = false, mounted = false, fail_writes = false, fail_sync = false;
    int mounts = 0, writes = 0, syncs = 0, reads = 0, removes = 0, opens = 0, lists = 0; uint32_t largest_write = 0; uint64_t bytes_written = 0;
    long open_number = -1;
    // timing: every write takes write_us; every stall_every-th write takes stall_us instead (0 = never)
    uint32_t write_us = 3000, sync_us = 6000, stall_us = 180000; int stall_every = 0;
    std::function<void(uint32_t)> busy;

    bool mount() override { mounts++; mounted = inserted && formatted; if (!mounted) reason = inserted ? "card is not FAT formatted" : "no SD card"; return mounted; }
    void unmount() override { mounted = false; open_number = -1; }
    bool alive() override { return mounted && inserted; }
    uint64_t total_bytes() override { return live() ? size_bytes : 0; }
    uint64_t free_bytes() override {
        if (!live()) return 0;
        uint64_t used = 0; for (auto& kv : files) used += kv.second.data.size(); for (auto& kv : other) used += kv.second;
        return used >= size_bytes ? 0 : size_bytes - used;
    }
    bool list(ListFn fn, void* ctx) override {
        lists++;
        if (!live()) return false;
        // in an order that is neither creation nor number order, like a real directory after deletions
        std::vector<uint32_t> nums; for (auto& kv : files) nums.push_back(kv.first);
        for (size_t i = 0; i < nums.size(); i += 2) fn(ctx, nums[i], (uint32_t)files[nums[i]].synced);
        for (size_t i = 1; i < nums.size(); i += 2) fn(ctx, nums[i], (uint32_t)files[nums[i]].synced);
        return true;
    }
    bool open_write(uint32_t number) override {
        opens++;
        if (!live() || read_only) return false;
        files[number] = F(); open_number = (long)number;
        return true;
    }
    bool write(const uint8_t* d, uint32_t n) override {
        if (!live() || open_number < 0 || fail_writes) return false;
        writes++; if (n > largest_write) largest_write = n;
        if (busy) busy((stall_every > 0 && writes % stall_every == 0) ? stall_us : write_us);
        F& f = files[(uint32_t)open_number];
        if (free_bytes() < n) return false;
        f.data.insert(f.data.end(), d, d + n); bytes_written += n;
        return true;
    }
    bool sync() override {
        if (!live() || open_number < 0 || fail_sync) return false;
        syncs++; if (busy) busy(sync_us);
        F& f = files[(uint32_t)open_number]; f.synced = f.data.size();
        return true;
    }
    void close_write() override { open_number = -1; }
    bool read(uint32_t number, uint32_t offset, uint8_t* buf, uint32_t n) override {
        reads++;
        auto it = files.find(number);
        if (!live() || it == files.end() || offset > it->second.synced || n > it->second.synced - offset) return false;
        memcpy(buf, it->second.data.data() + offset, n);
        return true;
    }
    bool remove(uint32_t number) override { removes++; if (!live() || read_only) return false; if ((long)number == open_number) open_number = -1; return files.erase(number) == 1; }
    const char* name() const override { return "sim-sd"; }
    const char* why() const override { return reason; }

    // ---- what happens to a card
    void pull() { inserted = false; }                         // taken out while powered: every later call fails
    void push() { inserted = true; }
    void power_cut() { for (auto& kv : files) kv.second.data.resize(kv.second.synced); mounted = false; open_number = -1; }   // unsynced bytes never reached the file
    const std::vector<uint8_t>& file(uint32_t number) { return files[number].data; }
    void put(uint32_t number, size_t bytes, uint8_t fill = 0x5A) { F f; f.data.assign(bytes, fill); f.synced = bytes; files[number] = f; }

private:
    bool live() const { return mounted && inserted; }
    const char* reason = "";
};
