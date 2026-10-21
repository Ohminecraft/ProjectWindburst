/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>
#include <string.h>
#include "platform.h"
#if WB_PLATFORM_STM32
#include <Arduino.h>
#endif

// ============================================================================
// Which chip this firmware is built for, by name (WB_MCU_NAME: the app shows it, the boot report and the default USB name use
// it), and which pin numbers exist on it (chip_pins(): the app offers only those - on an STM32 the pins of the chip's package).
// The pin numbers are the settings' own (port/pin_names.h): the GPIO number on an ESP32, port * 16 + pin on an STM32.
// ============================================================================
#if defined(CONFIG_IDF_TARGET_ESP32S3)
#define WB_MCU_NAME "ESP32-S3"
#elif defined(CONFIG_IDF_TARGET_ESP32S2)
#define WB_MCU_NAME "ESP32-S2"
#elif defined(CONFIG_IDF_TARGET_ESP32C5)
#define WB_MCU_NAME "ESP32-C5"
#elif defined(CONFIG_IDF_TARGET_ESP32)
#define WB_MCU_NAME "ESP32"
#elif defined(STM32H563xx)
#define WB_MCU_NAME "STM32H563"
#elif defined(STM32H562xx)
#define WB_MCU_NAME "STM32H562"
#elif defined(STM32H523xx)
#define WB_MCU_NAME "STM32H523"
#elif defined(STM32H533xx)
#define WB_MCU_NAME "STM32H533"
#elif defined(STM32G473xx)
#define WB_MCU_NAME "STM32G473"
#elif defined(STM32G474xx)
#define WB_MCU_NAME "STM32G474"
#elif defined(STM32G491xx)
#define WB_MCU_NAME "STM32G491"
#elif defined(STM32G431xx)
#define WB_MCU_NAME "STM32G431"
#elif defined(STM32F405xx)
#define WB_MCU_NAME "STM32F405"
#elif defined(STM32F412Cx) || defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx)
#define WB_MCU_NAME "STM32F412"
#elif defined(STM32C552xx)
#define WB_MCU_NAME "STM32C552"
#elif defined(STM32H5xx)
#define WB_MCU_NAME "STM32H5"
#elif defined(STM32G4xx)
#define WB_MCU_NAME "STM32G4"
#elif defined(STM32F4xx)
#define WB_MCU_NAME "STM32F4"
#elif defined(STM32C5xx)
#define WB_MCU_NAME "STM32C5"
#elif WB_PLATFORM_STM32
#define WB_MCU_NAME "STM32"
#else
#define WB_MCU_NAME "PC"
#endif

namespace port {

constexpr int kPinMaskBytes = 32;                  // one bit per pin number 0 .. 255

inline void pin_mask_set(uint8_t* m, unsigned p) { if (p < 256u) m[p >> 3] |= (uint8_t)(1u << (p & 7u)); }
inline bool pin_mask_has(const uint8_t* m, unsigned p) { return p < 256u && ((m[p >> 3] >> (p & 7u)) & 1u); }

// ESP32-S3 module: GPIO 0 .. 21 and 26 .. 48, without 26 .. 32 (the module's flash) and, with octal PSRAM (N16R8), 33 .. 37.
inline void esp32s3_pins(uint8_t* m, bool octal_psram) {
    for (unsigned p = 0; p <= 48; p++) {
        if (p >= 22 && p <= 32) continue;
        if (octal_psram && p >= 33 && p <= 37) continue;
        pin_mask_set(m, p);
    }
}
// The plain ESP32 (WROOM-32): GPIO 0 .. 39 without 6 .. 11 (the module's flash) and the numbers the chip does not have
// (20, 24, 28 .. 31). 34 .. 39 are inputs only (the app says so).
inline void esp32_pins(uint8_t* m) {
    for (unsigned p = 0; p <= 39; p++) {
        if ((p >= 6 && p <= 11) || p == 20 || p == 24 || (p >= 28 && p <= 31)) continue;
        pin_mask_set(m, p);
    }
}
// Every pin of the first `ports` ports (PA .. ): what the simulator offers when it plays an STM32 board.
inline void stm32_port_pins(uint8_t* m, unsigned ports) { for (unsigned p = 0; p < ports * 16u && p < 256u; p++) pin_mask_set(m, p); }

// The pins of the chip this firmware runs on.
inline void chip_pins(uint8_t* m) {
    memset(m, 0, kPinMaskBytes);
#if WB_PLATFORM_STM32
    // STM32duino's list of the package's pins (the variant): PinName = port * 16 + pin, plus ALT bits above 0xFF
    for (uint32_t i = 0; i < NUM_DIGITAL_PINS; i++) { const PinName pn = digitalPin[i]; if (pn != NC) pin_mask_set(m, (unsigned)pn & 0xFFu); }
#elif defined(CONFIG_IDF_TARGET_ESP32)
    esp32_pins(m);
#elif defined(CONFIG_SPIRAM_MODE_OCT)
    esp32s3_pins(m, true);
#else
    esp32s3_pins(m, false);                        // (an S3 without octal PSRAM, and the PC: the simulator plays one)
#endif
}

} // namespace port
