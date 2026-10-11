/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>

// ============================================================================
// Pin plausibility check, run once at boot on the pins that are really in use.
// Two functions on one GPIO is never right, and some combinations are dangerous: an ADC or a buzzer configured on a motor's pin
// silently takes the pin away from the ESC signal - that motor then stays off when the craft arms. The firmware refuses to arm
// with such a pin map (arming blocker PIN_CONFLICT) and names the two functions in the boot report.
// Also: pins the ESP32-S3 module needs for itself (flash 26..32, octal PSRAM 35..37 on "R8" modules, and 19 / 20 when the app 
// link is the chip's USB).
// Portable.
// ============================================================================
namespace pins {

constexpr uint8_t kUnused = 255;
struct Use { uint8_t pin; const char* what; };

struct Conflict { bool found = false; uint8_t pin = kUnused; const char* a = ""; const char* b = ""; };

// First GPIO that appears twice in `use` (entries with pin == kUnused are skipped).
inline Conflict find_conflict(const Use* use, int n) {
    Conflict c;
    for (int i = 0; i < n && !c.found; i++) {
        if (use[i].pin == kUnused) continue;
        for (int j = i + 1; j < n; j++) if (use[j].pin == use[i].pin) { c.found = true; c.pin = use[i].pin; c.a = use[i].what; c.b = use[j].what; break; }
    }
    return c;
}

// ESP32-S3 module pins that are not free: the SPI flash (26..32) and, on modules with octal PSRAM ("R8", e.g. N16R8), 35..37.
inline bool s3_reserved(uint8_t pin, bool octal_psram) { return (pin >= 26 && pin <= 32) || (octal_psram && pin >= 35 && pin <= 37); }
// The chip's own USB socket is GPIO19 (D-) and GPIO20 (D+). When the app link runs over it, a function on one of them takes the
// link away the moment it starts.
inline bool s3_usb_pin(uint8_t pin) { return pin == 19 || pin == 20; }
inline const Use* find_on_usb(const Use* use, int n) {
    for (int i = 0; i < n; i++) if (use[i].pin != kUnused && s3_usb_pin(use[i].pin)) return &use[i];
    return nullptr;
}
// First entry of `use` that sits on such a pin, or nullptr.
inline const Use* find_reserved(const Use* use, int n, bool octal_psram) {
    for (int i = 0; i < n; i++) if (use[i].pin != kUnused && s3_reserved(use[i].pin, octal_psram)) return &use[i];
    return nullptr;
}

} // namespace pins
