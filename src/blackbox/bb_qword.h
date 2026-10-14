/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>
#include <string.h>
#include "bb_flash.h"
#include "../port/qw_flash.h"

// ============================================================================
// bb::BlockStore over internal flash with quad-word ECC (port/qw_flash.h: the STM32H5's bank 2), so that FlashLog (bb_flash.h)
// keeps the flight log there as it does in the ESP32's partition.
//
// FlashLog writes a session front to back, and between two pages it may program a partly filled one (a periodic flush) and
// later continue in the SAME page - fine on NOR flash, refused here once a quad-word has been programmed. So the last, partly
// filled quad-word stays in RAM (at most 15 bytes) until the next bytes complete it; read() sees those bytes as well. sync()
// (FlashLog::end_session) programs it padded with 0xFF: the next session starts on a new 4 kB sector anyway, and a page that
// ends in 0xFF is what FlashLog expects after the last frame. A power cut in flight loses at most those 15 bytes.
// Erasing: FlashLog erases 64 kB blocks; size() is cut to a multiple of 64 kB so that every erase is whole H5 sectors (8 kB).
// Portable (test_blackbox runs it against test/sim_qwflash.h).
// ============================================================================
namespace bb {

class QwBlockStore : public BlockStore {
public:
    explicit QwBlockStore(port::QwFlash& f, const char* name = "internal flash") : f_(f), name_(name) {}
    uint32_t size() const override { return f_.size() / kEraseBlock * kEraseBlock; }
    const char* name() const override { return name_; }

    bool read(uint32_t addr, uint8_t* buf, uint32_t n) override {
        if (addr > size() || n > size() - addr || !f_.read(addr, buf, n)) return false;
        if (tail_n_) {                                       // bytes still waiting in RAM override what the flash has (0xFF there)
            for (uint32_t i = 0; i < tail_n_; i++) {
                const uint32_t a = tail_at_ + i;
                if (a >= addr && a < addr + n) buf[a - addr] = tail_[i];
            }
        }
        return true;
    }

    bool write(uint32_t addr, const uint8_t* data, uint32_t n) override {
        if (addr > size() || n > size() - addr) return false;
        if (tail_n_ && addr != tail_at_ + tail_n_) { if (!sync_tail()) return false; }
        if (!tail_n_ && addr % port::QwFlash::kQuad) return false;  // only a continuation may start inside a quad-word
        if (!tail_n_) tail_at_ = addr;
        while (n) {
            const uint32_t take = port::QwFlash::kQuad - tail_n_ < n ? port::QwFlash::kQuad - tail_n_ : n;
            memcpy(tail_ + tail_n_, data, take); tail_n_ += take; data += take; n -= take;
            if (tail_n_ == port::QwFlash::kQuad) {
                if (!f_.program(tail_at_, tail_)) { tail_n_ = 0; return false; }
                tail_at_ += port::QwFlash::kQuad; tail_n_ = 0;
            }
        }
        return true;
    }

    bool erase(uint32_t addr, uint32_t n) override {
        const uint32_t sec = f_.sector();
        if (!sec || addr % sec || n % sec || addr > size() || n > size() - addr) return false;
        if (tail_n_ && tail_at_ >= addr && tail_at_ < addr + n) tail_n_ = 0;   // (what waited there is erased with it)
        for (uint32_t a = addr; a < addr + n; a += sec) if (!f_.erase_sector(a)) return false;
        return true;
    }

    void sync() override { sync_tail(); }
    uint32_t pending() const { return tail_n_; }                // (tests)

private:
    bool sync_tail() {
        if (!tail_n_) return true;
        memset(tail_ + tail_n_, 0xFF, port::QwFlash::kQuad - tail_n_);
        const bool ok = f_.program(tail_at_, tail_);
        tail_n_ = 0;
        return ok;
    }
    port::QwFlash& f_; const char* name_;
    uint8_t tail_[port::QwFlash::kQuad]; uint32_t tail_at_ = 0, tail_n_ = 0;
};

} // namespace bb
