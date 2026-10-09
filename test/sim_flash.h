// A NOR flash region in RAM for the blackbox tests and the simulator. It behaves like the chip, not like an array:
//   * programming can only CLEAR bits (new = old AND data). Writing to a place that is not erased leaves garbage - and is counted
//     in bad_writes, so a test can prove that the log never does it;
//   * erase works on whole 4 kB sectors and sets them to 0xFF;
//   * every operation takes time during which the flight loop does not run (on the ESP32 both cores stop while the flash is busy):
//     the `stall` hook lets the virtual board advance its clock and keep the motors where they were.
#pragma once
#include <functional>
#include <vector>
#include <stdint.h>
#include <string.h>
#include "../src/blackbox/bb_flash.h"

struct MemFlash : bb::BlockStore {
    std::vector<uint8_t> mem;
    uint32_t writes = 0, erases = 0, reads = 0, bad_writes = 0, bytes_written = 0, longest_write = 0;
    bool fail_writes = false, fail_reads = false, fail_erase = false;
    long cut_after_writes = -1;                 // power cut: write number N (0 = the next one) programs only its first half, later ones nothing
    // datasheet-typical times (W25Q / GD25Q class): 256-byte program 0.4 .. 0.8 ms, 4 kB sector erase 45 ms, 64 kB block erase 150 ms
    uint32_t write_us = 700, erase_us_per_sector = 45000, erase_us_per_block = 150000, read_us = 0;
    std::function<void(uint32_t)> stall;

    explicit MemFlash(uint32_t bytes = 1024 * 1024) : mem(bytes, 0xFF) {}
    uint32_t size() const override { return (uint32_t)mem.size(); }
    bool read(uint32_t addr, uint8_t* buf, uint32_t n) override {
        reads++;
        if (fail_reads || addr > mem.size() || n > mem.size() - addr) return false;
        memcpy(buf, mem.data() + addr, n);
        if (stall && read_us) stall(read_us);
        return true;
    }
    bool write(uint32_t addr, const uint8_t* d, uint32_t n) override {
        if (fail_writes || addr > mem.size() || n > mem.size() - addr) return false;
        uint32_t todo = n;
        if (cut_after_writes == 0) { todo = n / 2; cut_after_writes = -2; }
        else if (cut_after_writes == -2) todo = 0;
        else if (cut_after_writes > 0) cut_after_writes--;
        for (uint32_t i = 0; i < todo; i++) { if ((mem[addr + i] & d[i]) != d[i]) bad_writes++; mem[addr + i] &= d[i]; }
        writes++; bytes_written += todo; if (n > longest_write) longest_write = n;
        if ((addr % bb::kPage) + n > bb::kPage) bad_writes++;                // a program operation must not cross a page boundary
        if (stall && write_us) stall(write_us);
        return true;
    }
    bool erase(uint32_t addr, uint32_t n) override {
        if (fail_erase || addr % bb::kSector || n % bb::kSector || addr > mem.size() || n > mem.size() - addr) return false;
        memset(mem.data() + addr, 0xFF, n); erases++;
        if (stall) stall((addr % bb::kEraseBlock == 0 && n == bb::kEraseBlock) ? erase_us_per_block : erase_us_per_sector * (n / bb::kSector));
        return true;
    }
    const char* name() const override { return "sim"; }
    // what a brand-new board may hold in that partition: an old file system, random bytes
    void fill_garbage(uint32_t from, uint32_t n, uint8_t seed = 7) { uint32_t x = seed; for (uint32_t i = 0; i < n && from + i < mem.size(); i++) { x = x * 1103515245u + 12345u; mem[from + i] = (uint8_t)(x >> 16); } }
};
