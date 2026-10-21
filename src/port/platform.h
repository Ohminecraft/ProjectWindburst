/*
 * This file is part of Windburst Project
 */

#pragma once

// ============================================================================
// Which chip the firmware is built for. Exactly one of these is 1:
//   WB_PLATFORM_ESP32  - Arduino-ESP32 (ESP32-S3 flight controller: the boards this project started on)
//   WB_PLATFORM_STM32  - STM32duino (STM32H5 so far: docs/STM32.md)
//   WB_PLATFORM_HOST   - a PC: the tests and the simulator
// Code that differs per chip asks these, not the compilers' own macros, so that a new chip is one more line here.
// ============================================================================
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP_PLATFORM)
#define WB_PLATFORM_ESP32 1
#define WB_PLATFORM_STM32 0
#define WB_PLATFORM_HOST 0
#elif defined(ARDUINO_ARCH_STM32)
#define WB_PLATFORM_ESP32 0
#define WB_PLATFORM_STM32 1
#define WB_PLATFORM_HOST 0
#else
#define WB_PLATFORM_ESP32 0
#define WB_PLATFORM_STM32 0
#define WB_PLATFORM_HOST 1
#endif
// The chip by name, and the pins it has: port/chip.h
