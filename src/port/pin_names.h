/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <ctype.h>

// ============================================================================
// What the one-byte pin numbers in the settings (section "pins", hardware_config.h) mean on each chip:
//   ESP32: the GPIO number (0..48).
//   STM32: port * 16 + pin, the value of STM32duino's PinName: PA0 = 0, PA15 = 15, PB0 = 16, ... PK15 = 175.
//          (So a setting means the same pin whatever board variant the firmware was built with.)
//   255 (GPIO_UNUSED): no pin.
// Portable: the tests check the names both ways.
// ============================================================================
namespace port {

constexpr uint8_t kNoPin = 255;
constexpr uint8_t kStm32Ports = 11;                    // A .. K

// "GPIO12" / "PB3" / "-" into out (at least 8 bytes).
inline void pin_name(uint8_t code, bool stm32, char* out, size_t n) {
    if (code == kNoPin) { snprintf(out, n, "-"); return; }
    if (!stm32) { snprintf(out, n, "GPIO%u", (unsigned)code); return; }
    if (code >= kStm32Ports * 16) { snprintf(out, n, "?%u", (unsigned)code); return; }
    snprintf(out, n, "P%c%u", (char)('A' + code / 16), (unsigned)(code % 16));
}

// "PB3", "pb03", "B3" -> 19; "GPIO12" / "12" -> 12 on an ESP32. -1 = not a pin name.
inline int pin_parse(const char* s, bool stm32) {
    if (!s) return -1;
    while (*s == ' ') s++;
    if (stm32) {
        if ((s[0] == 'P' || s[0] == 'p') && isalpha((unsigned char)s[1])) s++;
        const char c = (char)toupper((unsigned char)s[0]);
        if (c < 'A' || c >= 'A' + kStm32Ports || !isdigit((unsigned char)s[1])) return -1;
        int n = 0; const char* d = s + 1;
        while (isdigit((unsigned char)*d)) { n = n * 10 + (*d - '0'); d++; if (n > 15) return -1; }
        while (*d == ' ') d++;
        return *d ? -1 : (c - 'A') * 16 + n;
    }
    if ((s[0] == 'G' || s[0] == 'g') && (s[1] == 'P' || s[1] == 'p') && (s[2] == 'I' || s[2] == 'i') && (s[3] == 'O' || s[3] == 'o')) s += 4;
    if (!isdigit((unsigned char)*s)) return -1;
    int n = 0;
    while (isdigit((unsigned char)*s)) { n = n * 10 + (*s - '0'); s++; if (n > 254) return -1; }
    while (*s == ' ') s++;
    return *s ? -1 : n;
}

} // namespace port
