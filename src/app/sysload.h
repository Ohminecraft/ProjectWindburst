/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>
#if defined(ARDUINO)
#include <Arduino.h>
#endif

// ============================================================================
// What the app shows as "CPU" and "RAM".
//   CPU: the share of time the flight loop's core spends working (control cycle + sensors / receiver / app requests) instead of
//        waiting for the next cycle. 100 % = the loop cannot keep its rate. The other core (app link, SD card) is not in this number.
//   RAM: internal RAM in use (heap). PSRAM is reported separately; the flight loop never uses it.
// ============================================================================
namespace sys {

// Busy time accumulated between two reads.
class Load {
public:
    void add(uint32_t busy_us) { busy_ += busy_us; }
    // Percent of the wall time since the previous call (0..100). now_us = micros(); wraps correctly.
    uint8_t take_percent(uint32_t now_us) {
        const uint32_t span = now_us - mark_; mark_ = now_us;
        const uint64_t b = busy_; busy_ = 0;
        if (!started_) { started_ = true; return 0; }
        if (span == 0) return 0;
        const uint64_t pct = (b * 100u + span / 2u) / span;
        return pct > 100u ? (uint8_t)100 : (uint8_t)pct;
    }
private:
    uint64_t busy_ = 0; uint32_t mark_ = 0; bool started_ = false;
};

struct Memory {
    uint32_t heap_total = 0, heap_free = 0, psram_total = 0, psram_free = 0;
    bool psram_octal = false;                    // an "R8"-style module: GPIO35..37 belong to the PSRAM
    uint8_t used_percent() const {
        if (!heap_total || heap_free >= heap_total) return 0;
        return (uint8_t)(((uint64_t)(heap_total - heap_free) * 100u + heap_total / 2u) / heap_total);
    }
};
inline uint16_t kb16(uint32_t bytes) { const uint32_t k = bytes / 1024u; return k > 65535u ? (uint16_t)65535u : (uint16_t)k; }

#if defined(ARDUINO) && defined(ARDUINO_ARCH_STM32)
}  // namespace sys
#include <malloc.h>
extern "C" char _end, _estack;                            // (the linker script: end of the static data, top of the RAM)
namespace sys {
// STM32: no PSRAM; "heap" = the RAM between the static data and the top of the main stack, used = what malloc holds (FreeRTOS
// takes its task stacks from malloc in STM32duino's configuration).
inline Memory memory() {
    Memory m;
    m.heap_total = (uint32_t)(&_estack - &_end);
    const uint32_t used = (uint32_t)mallinfo().uordblks;
    m.heap_free = used < m.heap_total ? m.heap_total - used : 0;
    return m;
}
#elif defined(ARDUINO)
inline Memory memory() {
    Memory m;
    m.heap_total = ESP.getHeapSize(); m.heap_free = ESP.getFreeHeap(); m.psram_total = ESP.getPsramSize(); m.psram_free = ESP.getFreePsram();
#if defined(CONFIG_SPIRAM_MODE_OCT)
    m.psram_octal = m.psram_total != 0;
#endif
    return m;
}
#endif

} // namespace sys
