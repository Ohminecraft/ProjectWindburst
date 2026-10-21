/*
 * This file is part of Windburst Project
 */

#pragma once
#include <Arduino.h>
#include <string.h>
#include "../qw_flash.h"

// ============================================================================
// The STM32's own flash as port::QwFlash (qw_flash.h: each 16 bytes are programmed once after an erase, erasing works on whole
// sectors / pages). Which part of the flash, per family:
//
//   family   banks                    region                                   flight log there   program unit   erase unit
//   H5       2 (H563 / H562 2 x 1 MB,  bank 2                                   yes                quad-word      8 kB sector
//            H523 2 x 256 kB)
//   C5       2 (C552 2 x 256 kB)       bank 2                                   yes                quad-word      8 kB page
//   G4       G473: 2 (DBANK set, the   G473: bank 2 / G491: the last 16 kB     G473 only          2 double-words 2 kB page
//            default), G491: 1
//   F4       1 (F405 / F412: 16, 64,   the last two 128 kB sectors             no                 4 words        128 kB sector
//            128 kB sectors)
//
// Two banks: the firmware runs from bank 1 while bank 2 is erased / programmed (read-while-write) - the flight loop never
// waits, so the flight log may live there. One bank: every program / erase stalls the CPU (an F4 sector erase: 1 - 2 s), so
// that region only holds the settings, which are written on the ground; the flight log needs an SD card / SPI flash there.
// usable(): the region lies after the firmware image (the image grows into it otherwise: refused, said in the boot report).
//
// ECC (H5, C5, G4): a program cut by a power loss can read back with a double error, which raises an NMI. The handler at the end
// clears it and counts it; read() then fails, and the settings / log code treats that as an unreadable record - instead of the
// firmware crashing at every start. The F4's flash has no ECC.
//
// NOT confirmed on hardware.
// ============================================================================
namespace port {

inline volatile uint32_t g_flash_ecc_errors = 0;
extern "C" { extern char _sidata[], _sdata[], _edata[]; }     // the linker script's: initialised data, stored after the code
// Where the firmware image ends in flash (code, constants, the initial values of the variables).
inline uint32_t image_end() { return (uint32_t)(uintptr_t)_sidata + (uint32_t)(_edata - _sdata); }

class FlashStm32 : public QwFlash {
public:
    FlashStm32() {
#if defined(STM32H5xx)
        base_ = FLASH_BASE + FLASH_BANK_SIZE; size_ = FLASH_BANK_SIZE; sector_ = FLASH_SECTOR_SIZE; second_bank_ = true;
#elif defined(STM32C5xx)
        base_ = FLASH_BASE + FLASH_BANK_SIZE; size_ = FLASH_BANK_SIZE; sector_ = FLASH_PAGE_SIZE; second_bank_ = true;
#elif defined(STM32G4xx)
        sector_ = FLASH_PAGE_SIZE;
#if defined(FLASH_OPTR_DBANK)
        if (READ_BIT(FLASH->OPTR, FLASH_OPTR_DBANK)) { base_ = FLASH_BASE + FLASH_BANK_SIZE; size_ = FLASH_BANK_SIZE; second_bank_ = dual_ = true; }
        else
#endif
        { size_ = 16u * 1024u; base_ = FLASH_BASE + FLASH_SIZE - size_; second_bank_ = false; sector_ = 0x800u; }
#elif defined(STM32F4xx)
        sector_ = 128u * 1024u; size_ = 2u * sector_; base_ = FLASH_BASE + flash_size_f4() - size_; second_bank_ = false;
#else
#error "flash_stm32.h: this STM32 family is not supported yet"
#endif
        // A firmware bigger than bank 1 (H523, G473, C552: 2 x 256 kB, the firmware ~340 kB) runs partly from bank 2 as well:
        // writing there stalls the CPU like on a one-bank chip. Then only the last 32 kB (G4: 16 kB) are taken, for the settings.
        if (second_bank_ && image_end() > base_) {
            const uint32_t end = base_ + size_, want = sector_ <= 0x800u ? 16u * 1024u : 32u * 1024u;
            size_ = want; base_ = end - want; second_bank_ = false; shared_ = true;
        }
    }
    uint32_t size() const override { return size_; }
    uint32_t sector() const override { return sector_; }
    uint32_t base() const { return base_; }
    bool second_bank() const { return second_bank_; }                 // writes do not stall the CPU: the flight log may live here
    bool usable() const { return size_ && image_end() <= base_; }
    bool shares_bank_with_code() const { return shared_; }          // bank 2, but the firmware reaches into it
    const char* name() const { return second_bank_ ? "flash bank 2" : "the end of the flash"; }
    // why there is no flight log in the chip's flash (nullptr: there is room for one)
    const char* no_log_reason() const {
        if (!usable()) return "the firmware reaches into the settings region";
        if (shared_) return "the firmware is bigger than flash bank 1 and runs from bank 2 too: writing it would stall the flight loop";
        if (!second_bank_) return "one flash bank: writing it would stall the flight loop";
        return nullptr;
    }

    bool read(uint32_t addr, uint8_t* buf, uint32_t n) override {
        if (addr > size_ || n > size_ - addr) return false;
        const uint32_t before = g_flash_ecc_errors;
        memcpy(buf, (const void*)(uintptr_t)(base_ + addr), n);
        __DSB();
        return g_flash_ecc_errors == before;
    }
    bool program(uint32_t addr, const uint8_t* quad) override {
        if (!usable() || addr % kQuad || addr + kQuad > size_) return false;
        uint32_t q[4]; memcpy(q, quad, kQuad);                // (the HAL reads it as 32-bit words)
        const uint32_t a = base_ + addr;
        bool ok = true;
#if defined(STM32H5xx)
        HAL_FLASH_Unlock();
        ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, a, (uint32_t)(uintptr_t)q) == HAL_OK;
        HAL_FLASH_Lock();
#elif defined(STM32C5xx)
        c5_unlock();
        LL_FLASH_EnableProgramming(FLASH);
        for (int i = 0; i < 4; i++) ((volatile uint32_t*)(uintptr_t)a)[i] = q[i];    // one quad-word: four consecutive words
        ok = c5_wait();
        LL_FLASH_DisableProgramming(FLASH);
        LL_FLASH_Lock(FLASH);
#elif defined(STM32G4xx)
        HAL_FLASH_Unlock();
        for (int i = 0; i < 2 && ok; i++) { uint64_t d; memcpy(&d, q + 2 * i, 8); ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, a + 8u * i, d) == HAL_OK; }
        HAL_FLASH_Lock();
#elif defined(STM32F4xx)
        HAL_FLASH_Unlock();
        for (int i = 0; i < 4 && ok; i++) ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, a + 4u * i, q[i]) == HAL_OK;
        HAL_FLASH_Lock();
#endif
        invalidate();
        return ok;
    }
    bool erase_sector(uint32_t addr) override {
        if (!usable() || addr % sector_ || addr >= size_) return false;
        bool ok = true;
#if defined(STM32C5xx)
        c5_unlock();
        LL_FLASH_StartErasePage(FLASH, LL_FLASH_ERASE_BANK_2, LL_FLASH_ERASE_USER_AREA, (base_ + addr - FLASH_BASE - FLASH_BANK_SIZE) / sector_);
        ok = c5_wait();
        LL_FLASH_DisablePageErase(FLASH);
        LL_FLASH_Lock(FLASH);
#else
        FLASH_EraseInitTypeDef e = {};
        uint32_t bad = 0;
#if defined(STM32H5xx)
        e.TypeErase = FLASH_TYPEERASE_SECTORS; e.Banks = FLASH_BANK_2; e.Sector = (base_ + addr - FLASH_BASE - FLASH_BANK_SIZE) / sector_; e.NbSectors = 1;
#elif defined(STM32G4xx)
        e.TypeErase = FLASH_TYPEERASE_PAGES; e.NbPages = 1;
        const uint32_t off = base_ + addr - FLASH_BASE;              // from the start of the flash
#if defined(FLASH_BANK_2)
        if (dual_ && off >= FLASH_BANK_SIZE) { e.Banks = FLASH_BANK_2; e.Page = (off - FLASH_BANK_SIZE) / sector_; }
        else
#endif
        { e.Banks = FLASH_BANK_1; e.Page = off / sector_; }
#elif defined(STM32F4xx)
        e.TypeErase = FLASH_TYPEERASE_SECTORS; e.VoltageRange = FLASH_VOLTAGE_RANGE_3; e.NbSectors = 1;
        e.Sector = 5u + (base_ + addr - FLASH_BASE - 128u * 1024u) / sector_;      // sectors 5 .. : 128 kB each from 128 kB on
#endif
        HAL_FLASH_Unlock();
        ok = HAL_FLASHEx_Erase(&e, &bad) == HAL_OK;
        HAL_FLASH_Lock();
#endif
        invalidate();
        return ok;
    }

private:
    static void invalidate() {
#if defined(ICACHE) && defined(ICACHE_CR_EN)
        if (READ_BIT(ICACHE->CR, ICACHE_CR_EN)) { SET_BIT(ICACHE->CR, ICACHE_CR_CACHEINV); while (READ_BIT(ICACHE->SR, ICACHE_SR_BUSYF)) {} }
#endif
#if defined(STM32F4xx) || defined(STM32G4xx)
        // the data cache of the flash interface may still hold what was there before
        __HAL_FLASH_DATA_CACHE_DISABLE(); __HAL_FLASH_DATA_CACHE_RESET(); __HAL_FLASH_DATA_CACHE_ENABLE();
#endif
    }
#if defined(STM32F4xx)
    static uint32_t flash_size_f4() { return (uint32_t)(*(const uint16_t*)FLASHSIZE_BASE) * 1024u; }
#endif
#if defined(STM32C5xx)
    static void c5_unlock() { if (LL_FLASH_IsLocked(FLASH)) { LL_FLASH_SetUnlockKey(FLASH, LL_FLASH_KEY1); LL_FLASH_SetUnlockKey(FLASH, LL_FLASH_KEY2); } }
    static bool c5_wait() {
        const uint32_t t0 = millis();
        while (LL_FLASH_IsActiveFlag_BSY(FLASH) || LL_FLASH_IsActiveFlag_WBNE(FLASH)) if (millis() - t0 > 100u) return false;
        const bool bad = LL_FLASH_IsActiveFlag_WRPERR(FLASH) || LL_FLASH_IsActiveFlag_PGSERR(FLASH) || LL_FLASH_IsActiveFlag_STRBERR(FLASH) || LL_FLASH_IsActiveFlag_INCERR(FLASH);
        LL_FLASH_ClearFlag_EOP(FLASH); LL_FLASH_ClearFlag_WRPERR(FLASH); LL_FLASH_ClearFlag_PGSERR(FLASH); LL_FLASH_ClearFlag_STRBERR(FLASH); LL_FLASH_ClearFlag_INCERR(FLASH);
        return !bad;
    }
#endif
    uint32_t base_ = 0, size_ = 0, sector_ = 0; bool second_bank_ = false, shared_ = false, dual_ = false;      // dual_: G4 with DBANK set
};

// The chip's 96-bit unique id, word 0 .. 2.
inline uint32_t uid_word(int i) { return ((const uint32_t*)(uintptr_t)UID_BASE)[i]; }

// Why the chip started (the RCC reset flags), then the flags are cleared for the next start.
inline const char* reset_reason() {
#if defined(RCC_RSR_BORRSTF)
    const uint32_t f = RCC->RSR;
    const char* why = (f & RCC_RSR_BORRSTF) ? "BROWN-OUT or power on" : (f & RCC_RSR_IWDGRSTF) ? "CRASH (independent watchdog)" : (f & RCC_RSR_WWDGRSTF) ? "CRASH (window watchdog)"
                    : (f & RCC_RSR_SFTRSTF) ? "software restart" : (f & RCC_RSR_LPWRRSTF) ? "low-power reset" : (f & RCC_RSR_PINRSTF) ? "reset pin" : "unknown";
    SET_BIT(RCC->RSR, RCC_RSR_RMVF);
#else
    const uint32_t f = RCC->CSR;
#if defined(RCC_CSR_PORRSTF)
    const bool por = (f & (RCC_CSR_PORRSTF | RCC_CSR_BORRSTF)) != 0;
#else
    const bool por = (f & RCC_CSR_BORRSTF) != 0;
#endif
    const char* why = por ? "BROWN-OUT or power on" : (f & RCC_CSR_IWDGRSTF) ? "CRASH (independent watchdog)" : (f & RCC_CSR_WWDGRSTF) ? "CRASH (window watchdog)"
                    : (f & RCC_CSR_SFTRSTF) ? "software restart" : (f & RCC_CSR_LPWRRSTF) ? "low-power reset" : (f & RCC_CSR_PINRSTF) ? "reset pin" : "unknown";
    SET_BIT(RCC->CSR, RCC_CSR_RMVF);
#endif
    return why;
}

} // namespace port

// A double ECC error in a flash read (see above). Anything else that raises an NMI stops here as before (the default handler loops).
extern "C" void NMI_Handler(void) {
#if defined(STM32H5xx)
    if (READ_BIT(FLASH->ECCDETR, FLASH_ECCR_ECCD)) { SET_BIT(FLASH->ECCDETR, FLASH_ECCR_ECCD); port::g_flash_ecc_errors++; return; }
#elif defined(STM32C5xx)
    if (READ_BIT(FLASH->ECCDETR, FLASH_ECCDETR_ECCD)) { SET_BIT(FLASH->ECCDETR, FLASH_ECCDETR_ECCD); port::g_flash_ecc_errors++; return; }
#elif defined(STM32G4xx)
    if (READ_BIT(FLASH->ECCR, FLASH_ECCR_ECCD)) { SET_BIT(FLASH->ECCR, FLASH_ECCR_ECCD); port::g_flash_ecc_errors++; return; }
#endif
    for (;;) {}
}
