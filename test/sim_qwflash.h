/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>
#include <string.h>
#include <vector>
#include "../src/port/qw_flash.h"

// ============================================================================
// A model of flash with quad-word ECC (the STM32H5's) for the tests: RAM plus a "programmed" mark per quad-word.
// It enforces what the chip enforces: program only aligned, erased quad-words (a second program of one is refused and
// counted in `violations`), erase whole sectors. Power cuts: `cut_after` = programs (or erases) left before the power goes;
// the cut one is torn (some of its bytes changed, the rest not - unreadable as a record head), everything after fails until
// power_on(). The contents survive, as on the chip.
// ============================================================================
class SimQwFlash : public port::QwFlash {
public:
    SimQwFlash(uint32_t size, uint32_t sector = 8192) : mem_(size, 0xFF), prog_(size / 16, 0), sector_(sector) {}
    uint32_t size() const override { return (uint32_t)mem_.size(); }
    uint32_t sector() const override { return sector_; }
    bool read(uint32_t a, uint8_t* buf, uint32_t n) override {
        if (a > size() || n > size() - a) return false;
        reads++; memcpy(buf, &mem_[a], n); return true;
    }
    bool program(uint32_t a, const uint8_t* q) override {
        if (dead_ || a % 16 || a + 16 > size()) return false;
        if (prog_[a / 16]) { violations++; return false; }
        if (cut_after >= 0 && cut_after-- == 0) { for (int i = 0; i < 16; i += 3) mem_[a + i] = q[i] ^ 0x5A; prog_[a / 16] = 1; dead_ = true; return false; }
        memcpy(&mem_[a], q, 16); prog_[a / 16] = 1; programs++; return true;
    }
    bool erase_sector(uint32_t a) override {
        if (dead_ || a % sector_ || a >= size()) return false;
        if (cut_after >= 0 && cut_after-- == 0) { memset(&mem_[a], 0x00, sector_ / 2); dead_ = true; return false; }   // half erased
        memset(&mem_[a], 0xFF, sector_); for (uint32_t i = 0; i < sector_ / 16; i++) prog_[a / 16 + i] = 0; erases++; return true;
    }
    void power_on() { dead_ = false; cut_after = -1; }
    bool dead() const { return dead_; }
    int cut_after = -1;
    uint32_t programs = 0, erases = 0, reads = 0, violations = 0;
    uint8_t* raw() { return mem_.data(); }
private:
    std::vector<uint8_t> mem_; std::vector<uint8_t> prog_; uint32_t sector_; bool dead_ = false;
};
