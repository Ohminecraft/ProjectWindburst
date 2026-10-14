/*
 * This file is part of Windburst Project
 */

#pragma once
#include <Arduino.h>
#include <string.h>
#include "../qw_flash.h"

// ============================================================================
// The STM32H5's internal flash, bank 2, as port::QwFlash (qw_flash.h says what the rules are). The firmware runs from bank 1;
// writing bank 2 does not stop it (the two banks read and write independently).
//   H563 / H562 (2 MB): bank 2 = 1 MB, 128 sectors of 8 kB.  H523 / H533 (512 kB): 256 kB, 32 sectors.
// main_stm32.cpp: the first 64 kB for the settings (proto/kv_flash.h), the rest for the flight log (blackbox/bb_qword.h).
//
// ECC: every quad-word carries 9 ECC bits. A quad-word whose programming was cut by a power loss can read back with a double
// error; the chip then raises an NMI. The NMI handler below clears the flag and notes it, read() reports the failure, and the
// settings / log code treats it as an unreadable record - instead of the firmware crashing at every start.
// The instruction cache (ICACHE) also caches flash reads: it is invalidated after every erase and program.
//
// NOT confirmed on hardware.
// ============================================================================
namespace port {

inline volatile uint32_t g_flash_ecc_errors = 0;

class FlashH5 : public QwFlash {
public:
    FlashH5() : base_(FLASH_BASE + FLASH_BANK_SIZE), size_(FLASH_BANK_SIZE) {}
    uint32_t size() const override { return size_; }
    uint32_t sector() const override { return FLASH_SECTOR_SIZE; }

    bool read(uint32_t addr, uint8_t* buf, uint32_t n) override {
        if (addr > size_ || n > size_ - addr) return false;
        const uint32_t before = g_flash_ecc_errors;
        memcpy(buf, (const void*)(uintptr_t)(base_ + addr), n);
        __DSB();
        return g_flash_ecc_errors == before;
    }
    bool program(uint32_t addr, const uint8_t* quad) override {
        if (addr % kQuad || addr + kQuad > size_) return false;
        uint32_t q[4]; memcpy(q, quad, kQuad);                // (the HAL reads it as 32-bit words)
        HAL_FLASH_Unlock();
        const bool ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, base_ + addr, (uint32_t)(uintptr_t)q) == HAL_OK;
        HAL_FLASH_Lock();
        invalidate();
        return ok;
    }
    bool erase_sector(uint32_t addr) override {
        if (addr % FLASH_SECTOR_SIZE || addr >= size_) return false;
        FLASH_EraseInitTypeDef e = {};
        e.TypeErase = FLASH_TYPEERASE_SECTORS; e.Banks = FLASH_BANK_2; e.Sector = addr / FLASH_SECTOR_SIZE; e.NbSectors = 1;
        uint32_t bad = 0;
        HAL_FLASH_Unlock();
        const bool ok = HAL_FLASHEx_Erase(&e, &bad) == HAL_OK;
        HAL_FLASH_Lock();
        invalidate();
        return ok;
    }
private:
    static void invalidate() {
#if defined(ICACHE) && defined(ICACHE_CR_EN)
        if (READ_BIT(ICACHE->CR, ICACHE_CR_EN)) { SET_BIT(ICACHE->CR, ICACHE_CR_CACHEINV); while (READ_BIT(ICACHE->SR, ICACHE_SR_BUSYF)) {} }
#endif
    }
    uint32_t base_, size_;
};

} // namespace port

// A double ECC error in a flash read (see above). Anything else that raises an NMI stops here as before (the default handler loops).
extern "C" void NMI_Handler(void) {
    if (READ_BIT(FLASH->ECCDETR, FLASH_ECCR_ECCD)) { SET_BIT(FLASH->ECCDETR, FLASH_ECCR_ECCD); port::g_flash_ecc_errors++; return; }
    for (;;) {}
}
