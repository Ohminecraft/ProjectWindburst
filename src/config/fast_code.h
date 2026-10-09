/*
 * This file is part of Windburst Project
 */

#pragma once

// ============================================================================
// WB_FAST - "this function runs in every control cycle": on the ESP32 it is placed in internal RAM (IRAM) instead of flash.
//
// Why: code in flash is executed through a cache. The Arduino core for the ESP32-S3 is built with a 16 kB instruction cache,
// shared by both CPU cores, while one control cycle (IMU read, filters, attitude, PID, mixer, DShot) walks through more code
// than that - and the app link, the sensors and the blackbox run their own code in between. Every 32 bytes that are not in the
// cache are fetched from the flash chip first (a few microseconds each). Code in RAM is never fetched: the cycle costs the same
// every time.
//
// What it costs: RAM - the marked functions together are listed by tools/iram_report.py (they come off the heap).
// What it does not do: make one instruction faster, or keep the cycle running while the flash is being written (a flash write
// stops both cores whatever the code's address; only interrupt handlers registered for that run on).
//
// Switch it off for a comparison on the board:   build_flags = ... -DWB_FAST_CODE=0     (status tab: time per stage)
// On a PC (tests, simulator) the macro is empty.
//
// A marked function only helps if what it calls is in RAM as well: small helpers are inlined by the compiler, the larger ones
// carry the mark themselves. Library code (libm, the SPI driver of the Arduino core) stays in flash.
// ============================================================================
#if (defined(ARDUINO_ARCH_ESP32) || defined(ESP_PLATFORM)) && !defined(WB_FAST_CODE)
#define WB_FAST_CODE 1
#endif

#if (defined(ARDUINO_ARCH_ESP32) || defined(ESP_PLATFORM)) && WB_FAST_CODE
#include <esp_attr.h>
#define WB_FAST IRAM_ATTR
#else
#define WB_FAST
#endif
