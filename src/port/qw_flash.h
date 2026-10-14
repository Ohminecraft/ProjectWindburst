/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>

// ============================================================================
// Internal flash of the kind the STM32H5 has: every 16-byte quad-word carries its own ECC, so after an erase each quad-word
// can be programmed ONCE (programming it again, even with the same bits or with 0xFF, is refused by the chip), and erasing
// works on whole sectors (8 kB on the H5). NOR flash elsewhere (the ESP32's) allows clearing bits at any time; the code above
// this interface must not rely on that.
//   proto/kv_flash.h        the settings, a small key -> record log in two sector groups
//   blackbox/bb_qword.h     the flight log (bb::BlockStore over this)
//   port/stm32/flash_h5.h   the chip; test/sim_qwflash.h the model the tests run against (it enforces the rules)
// Addresses are offsets inside the region. Portable.
// ============================================================================
namespace port {

class QwFlash {
public:
    static constexpr uint32_t kQuad = 16;
    virtual ~QwFlash() = default;
    virtual uint32_t size() const = 0;                                    // bytes, a multiple of sector()
    virtual uint32_t sector() const = 0;                                  // erase unit, bytes
    virtual bool read(uint32_t addr, uint8_t* buf, uint32_t n) = 0;
    virtual bool program(uint32_t addr, const uint8_t* quad) = 0;         // 16 bytes at a multiple of 16, erased since
    virtual bool erase_sector(uint32_t addr) = 0;                         // addr: a multiple of sector()
};

// A part of another region (the settings and the log share bank 2 of the H5).
class QwFlashSlice : public QwFlash {
public:
    QwFlashSlice(QwFlash& base, uint32_t offset, uint32_t size) : b_(base), off_(offset), size_(size) {}
    uint32_t size() const override { return size_; }
    uint32_t sector() const override { return b_.sector(); }
    bool read(uint32_t a, uint8_t* buf, uint32_t n) override { return a <= size_ && n <= size_ - a && b_.read(off_ + a, buf, n); }
    bool program(uint32_t a, const uint8_t* q) override { return a + kQuad <= size_ && b_.program(off_ + a, q); }
    bool erase_sector(uint32_t a) override { return a < size_ && b_.erase_sector(off_ + a); }
private:
    QwFlash& b_; uint32_t off_, size_;
};

} // namespace port
