/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>
#include <stddef.h>

// ============================================================================
// The board's name on USB (the product string the computer shows): general.usb_name, or "Windburst <MCU>" when that is empty -
// the name a freshly flashed board has. Only where the firmware runs the USB stack itself (ESP32-S3 rx_s3_otg: TinyUSB); the
// ESP32-S3's built-in USB Serial/JTAG answers with a name fixed in the chip.
// Portable (test_app).
// ============================================================================
#include "../port/chip.h"          // WB_MCU_NAME

namespace app {

constexpr size_t kUsbNameLen = 32;

// out: kUsbNameLen + 1 bytes. A USB string descriptor of this stack is 8-bit: printable ASCII only, anything else becomes '_'
// (a UTF-8 letter would show as two garbage characters). Spaces at both ends are dropped; nothing left = the default name.
inline const char* usb_product_name(const uint8_t* setting, const char* mcu, char* out) {
    size_t a = 0, b = 0;
    while (b < kUsbNameLen && setting[b]) b++;
    while (a < b && setting[a] == ' ') a++;
    while (b > a && setting[b - 1] == ' ') b--;
    size_t n = 0;
    for (size_t i = a; i < b; i++) out[n++] = (setting[i] >= 0x20 && setting[i] < 0x7F) ? (char)setting[i] : '_';
    if (n == 0) {
        const char* p = "Windburst ";
        while (*p && n < kUsbNameLen) out[n++] = *p++;
        while (*mcu && n < kUsbNameLen) out[n++] = *mcu++;
    }
    out[n] = 0;
    return out;
}

}  // namespace app
