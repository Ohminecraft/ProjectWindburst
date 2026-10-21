/*
 * This file is part of Windburst Project
 */

#pragma once

// ============================================================================
// WB_LOW_RAM: a chip with about 128 kB of RAM for everything (STM32F405: 128 kB + 64 kB CCM that DMA cannot reach, STM32G473:
// 128 kB, STM32G491: 112 kB). The other chips have 256 kB and more (ESP32-S3, STM32F412, H5). On a low-RAM chip the firmware
// keeps smaller buffers, so that its three tasks' stacks still fit after the static data:
//   the flight log's queue (bb_recorder.h)          16 kB -> 4 kB   (the log rate the queue rides out drops with it)
//   the app link's transmit queue (tx_queue.h)       8 kB -> 2 kB   (a download frame is 492 bytes)
//   the stacks of wb_flight / wb_sensor / wb_link   24 / 16 / 14 kB -> 8 / 8 / 6 kB (a Cortex-M needs far less stack than the
//                                                    ESP32's Xtensa with its register windows; NOT measured on a board yet)
// ============================================================================
#if !defined(WB_LOW_RAM)
#if defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32G4xx)
#define WB_LOW_RAM 1
#else
#define WB_LOW_RAM 0
#endif
#endif
