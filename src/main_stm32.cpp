/*
 * This file is part of Windburst Project
 *
 * Firmware entry point of the STM32 builds (H5, G4, F4, C5: docs/STM32.md). The ESP32 one is main.cpp; both run the same
 * FlightApp, settings, app link and blackbox - only the chip's ends differ:
 *   "wb_flight" : the control cycle, woken by TIM7                             [port/stm32/flight_task_stm32.h]
 *   "wb_sensor" : sensors, receiver, app requests, once per millisecond tick   [FlightApp::slow_update]
 *   "wb_link"   : app link over USB CDC (or a UART) + blackbox                 [LinkTask, wbp::Server, bb::Service]
 * One core: FreeRTOS priorities in that order (port/rtos.h). Settings (and on a chip with two flash banks the flight log) live
 * in the chip's own flash (port/stm32/flash_stm32.h).
 * Not in this build yet: DShot (motors: PWM / Oneshot / Multishot / brushed), the SD card, bidirectional ESC telemetry.
 * NOT confirmed on hardware.
 */
#if defined(ARDUINO_ARCH_STM32)
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#ifndef WB_LINK_BAUD
#define WB_LINK_BAUD 230400          // a UART link only; USB CDC ignores it
#endif
#include "config/version.h"

#include "port/platform.h"
#include "port/rtos.h"
#include "port/uart.h"
#include "port/stm32/flash_stm32.h"
#include "port/chip.h"
#include "app/flight_app.h"
#include "port/stm32/flight_task_stm32.h"
#include "app/link_arduino.h"
#include "proto/kv_flash.h"
#include "blackbox/bb_qword.h"
#include "blackbox/bb_header.h"
#include "blackbox/bb_card_stm32.h"
#include "hal/can/stm32_can.h"

#if defined(STM32F4xx)
// The clock of an F4 (instead of the generic variant's internal 16 MHz oscillator): a flight controller has a crystal - 8 MHz on
// most F4 boards, another one with the build flag WB_HSE_MHZ - and the USB needs its 48 MHz exact, which the internal oscillator
// (+-1 %) does not promise. F405: 168 MHz; F412: 96 MHz (its 100 MHz top gives no exact 48 MHz). No crystal answering: the
// internal oscillator, as before (the boot report says which).
#ifndef WB_HSE_MHZ
#define WB_HSE_MHZ 8
#endif
static bool g_clock_hse = false;
extern "C" void SystemClock_Config(void) {
    RCC_OscInitTypeDef o = {}; RCC_ClkInitTypeDef c = {};
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
#if defined(STM32F405xx) || defined(STM32F407xx)
    const uint32_t n = 336, q = 7, lat = FLASH_LATENCY_5, apb1 = RCC_HCLK_DIV4, apb2 = RCC_HCLK_DIV2;     // 336 / 2 = 168, 336 / 7 = 48
#else
    const uint32_t n = 192, q = 4, lat = FLASH_LATENCY_3, apb1 = RCC_HCLK_DIV2, apb2 = RCC_HCLK_DIV1;     // 192 / 2 = 96, 192 / 4 = 48
#endif
    o.PLL.PLLState = RCC_PLL_ON; o.PLL.PLLN = n; o.PLL.PLLP = RCC_PLLP_DIV2; o.PLL.PLLQ = q;
#if defined(RCC_PLLR_SUPPORT) || defined(RCC_PLLCFGR_PLLR)
    o.PLL.PLLR = 2;
#endif
    o.OscillatorType = RCC_OSCILLATORTYPE_HSE; o.HSEState = RCC_HSE_ON; o.PLL.PLLSource = RCC_PLLSOURCE_HSE; o.PLL.PLLM = WB_HSE_MHZ;   // 1 MHz into the PLL
    g_clock_hse = HAL_RCC_OscConfig(&o) == HAL_OK;
    if (!g_clock_hse) {
        o.OscillatorType = RCC_OSCILLATORTYPE_HSI; o.HSEState = RCC_HSE_OFF; o.HSIState = RCC_HSI_ON; o.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
        o.PLL.PLLSource = RCC_PLLSOURCE_HSI; o.PLL.PLLM = 16;
        if (HAL_RCC_OscConfig(&o) != HAL_OK) Error_Handler();
    }
    c.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    c.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; c.AHBCLKDivider = RCC_SYSCLK_DIV1; c.APB1CLKDivider = apb1; c.APB2CLKDivider = apb2;
    if (HAL_RCC_ClockConfig(&c, lat) != HAL_OK) Error_Handler();
#if defined(RCC_PERIPHCLK_CLK48)
    RCC_PeriphCLKInitTypeDef p = {}; p.PeriphClockSelection = RCC_PERIPHCLK_CLK48; p.Clk48ClockSelection = RCC_CLK48CLKSOURCE_PLLQ;
    HAL_RCCEx_PeriphCLKConfig(&p);
#endif
}
#define WB_CLOCK_NOTE (g_clock_hse ? ", crystal " : ", NO CRYSTAL (internal oscillator: USB may be unreliable)")
#else
#define WB_CLOCK_NOTE ""
#endif

static wbp::Settings settings;
static FlightApp flight;
static FlightTask flight_task;
static port::FlashStm32 flash;
// a second bank: the settings first (64 kB on an H5's 1 MB bank, 32 kB on the others), the flight log after them;
// one bank: the whole region is the settings' (there is no flight log in the chip's flash)
static uint32_t settings_bytes() { return !flash.second_bank() ? flash.size() : flash.size() >= 1024u * 1024u ? 64u * 1024u : 32u * 1024u; }
static port::QwFlashSlice settings_region(flash, 0, settings_bytes());
static port::QwFlashSlice log_region(flash, settings_bytes(), flash.size() - settings_bytes());
static wbp::KvFlashStorage kv(settings_region);
static wbp::SettingsStore store(kv);
static bb::QwBlockStore log_store(log_region, "flash bank 2");
static port::SerialPort uart_a, uart_b, uart_c;            // the instance (USART1, UART4 ...) follows the pins in the settings
static SerialTransport transport(Serial, WB_LINK_BYTES_PER_S);
static app::LinkHost host(flight.link, &LinkTask::wait_1ms, nullptr, &LinkTask::now_ms);
static wbp::Server server(settings, &store, host, transport, wbp::ROLE_RX, WB_FW_MAJOR, WB_FW_MINOR, WB_FW_PATCH, "Windburst RX",
                          wbp::FEAT_STREAMS | wbp::FEAT_CALIBRATION | wbp::FEAT_MOTOR_TEST | wbp::FEAT_GPS | wbp::FEAT_BLACKBOX | wbp::FEAT_BATTERY | wbp::FEAT_BEEPER | wbp::FEAT_STM32);
static LinkTask link_task(server, Serial, transport);
static app::MspLink msp_link(host, transport, &settings);
static app::Cli term_cli(settings, server, [](const char* t, void*) { transport.write((const uint8_t*)t, strlen(t)); }, nullptr);     // '#' on the serial port: the command line (app/cli.h)
static bb::Service blackbox(flight.blackbox, transport);
static bb::FirmwareHeader log_header{&settings, "Windburst RX", WB_FW_MAJOR, WB_FW_MINOR, WB_FW_PATCH};
static SdFatFileStore sd_store;                          // SD card on SPI (SdFat), only touched when blackbox.device = SDCARD
static RtosLockStm32 sd_lock;
static bb::CardLog card_log(sd_store, sd_lock);
static SdCardTaskStm32 sd_task(card_log);
#if WB_HAS_STM32_CAN
static Stm32CanBus can_bus;                              // FDCAN (G4 / H5) or bxCAN (F4): DroneCAN, section can
#endif
static TaskHandle_t sensor_handle = nullptr;

static void sensor_entry(void*) {
    for (;;) {
        if (flight_task.running()) { flight.slow_update(); vTaskDelay(1); }
        else flight.run();
    }
}

void setup() {
    // Everything that waits (delay(), sensor start-up) comes before the first FreeRTOS call: from that call until the scheduler
    // runs, SysTick - and so millis() / delay() - is masked (port/stm32/flight_task_stm32.h).
    Serial.begin(WB_LINK_BAUD);

    // 1. settings
    const bool kv_ok = kv.begin();
    const wbp::SettingsStore::LoadReport rep = store.load(settings);

    // 2. hardware
#if defined(USBCON) && defined(USBD_USE_CDC)
    flight.usb_link = true;
#endif
    flight.uart0 = &uart_c; flight.uart1 = &uart_a; flight.uart2 = &uart_b;
#if WB_HAS_STM32_CAN
    flight.can_port = &can_bus;
#endif
    {   // the chip's unique id (DroneCAN's hardware unique id)
        const uint32_t u[3] = {port::uid_word(0), port::uid_word(1), port::uid_word(2)}; memcpy(flight.chip_uid, u, sizeof(u));
    }
    flight.configure(settings);
    flight.begin();
    server.settings_loaded();                                // what flash holds, what the firmware runs with: "unsaved" / "reboot required" from here

    // 3. blackbox in bank 2 (a chip with one bank: none in the chip's flash)
    char line[176];                                          // (= FlightApp::kReportLen)
    const bool log_here = !flash.no_log_reason() && log_region.size() >= 64u * 1024u;
    blackbox.attach_store(log_here ? &log_store : nullptr);
    if (!log_here)
        snprintf(line, sizeof(line), "blackbox: FAILED - no room for a log in the flash of the %s (%s)", WB_MCU_NAME, flash.no_log_reason() ? flash.no_log_reason() : "too small");
    else if (blackbox.log().ok())
        snprintf(line, sizeof(line), "blackbox: %s %lu kB, %d sessions, %lu kB used%s", log_store.name(), (unsigned long)(log_store.size() / 1024u), blackbox.log().sessions(),
                 (unsigned long)(blackbox.log().used() / 1024u), blackbox.log().needs_erase() ? " - holds other data: erase it in the app" : "");
    else snprintf(line, sizeof(line), "blackbox: FAILED - the log region of flash bank 2 cannot be read");
    flight.add_report(line);
    //    SD card: mounted now when the settings select it; a card that is missing (or selected later in the app) is looked for
    //    every 3 s by the writer task
    if (settings.blackbox.device == bb::DEV_SDCARD) {
        card_log.configure_device(true, settings.blackbox.sd_mode, settings.blackbox.sd_mhz);
        SdCardTaskStm32::give_buffer(card_log);
        if (card_log.begin()) snprintf(line, sizeof(line), "blackbox: SD card '%s' %lu MB, %lu MB free, %d log files, buffer %lu kB", sd_store.name(), (unsigned long)(card_log.capacity_kb() / 1024u),
                                       (unsigned long)((card_log.capacity_kb() - card_log.used_kb()) / 1024u), card_log.sessions(), (unsigned long)(card_log.buffer_size() / 1024u));
        else snprintf(line, sizeof(line), "blackbox: SD card FAILED - %s (it is looked for again every 3 s)", card_log.why());
        flight.add_report(line);
    }
    blackbox.attach_card(&card_log);
    blackbox.set_header_source(&bb::FirmwareHeader::write, &log_header);
    blackbox.set_loop(flight.core.cfg.pid_hz(), flight.motors.cfg().count, flight.core.cfg.loop_hz);   // a record per control cycle at most
    host.set_blackbox(&blackbox, &settings);
    host.set_port(&transport, &g_link_kicks);
    {   // the chip's 96-bit unique id: the MSP side's UID, and (with the clock) a boot id that differs every start
        const uint32_t u0 = port::uid_word(0), u1 = port::uid_word(1), u2 = port::uid_word(2);
        uint8_t uid[12]; memcpy(uid, &u0, 4); memcpy(uid + 4, &u1, 4); memcpy(uid + 8, &u2, 4); host.set_uid(uid);
        host.set_boot_id(u0 ^ (u1 * 2654435761u) ^ u2 ^ (uint32_t)micros() ^ (port::g_flash_ecc_errors << 24));
        msp_link.set_firmware(WB_FW_MAJOR, WB_FW_MINOR, WB_FW_PATCH, __DATE__, __TIME__);
        link_task.set_msp(&msp_link, [](EscPins& p) { p.n = 0; for (int m = 0; m < 8 && flight.escPin(m) != GPIO_UNUSED; m++) p.pin[p.n++] = flight.escPin(m); p.pullup = flight.escPullup(); });
    }
    snprintf(line, sizeof(line), "reset: %s", port::reset_reason()); flight.add_report(line);      // why the chip started (RCC reset flags)
    snprintf(line, sizeof(line), "mcu: %s, %lu MHz%s, flash %lu kB (firmware %lu kB), settings in %s at 0x%08lx", WB_MCU_NAME, (unsigned long)(SystemCoreClock / 1000000u), WB_CLOCK_NOTE,
             (unsigned long)(*(const uint16_t*)FLASHSIZE_BASE), (unsigned long)((port::image_end() - FLASH_BASE) / 1024u), flash.name(), (unsigned long)(flash.base()));
    flight.add_report(line);
    if (WB_LINK_BYTES_PER_S) snprintf(line, sizeof(line), "link: %s, %lu bytes/s (the live log is slowed down to fit)", WB_LINK_KIND, (unsigned long)WB_LINK_BYTES_PER_S);
    else snprintf(line, sizeof(line), "link: %s", WB_LINK_KIND);
    flight.add_report(line);
    snprintf(line, sizeof(line), "settings: %s (gen %lu, %lu/%lu B), loaded 0x%06lx (0x%06lx older), defaulted 0x%06lx, clamped %u",
             kv_ok ? "flash ok" : !flash.usable() ? "flash FAILED - the firmware is too big: it reaches into the settings region" : "flash FAILED", (unsigned long)kv.generation(), (unsigned long)kv.used(), (unsigned long)kv.half_size(),
             (unsigned long)rep.loaded_mask, (unsigned long)rep.migrated_mask, (unsigned long)rep.defaulted_mask, (unsigned)(uint16_t)rep.clamped_fields);
    flight.add_report(line);
    if (port::g_flash_ecc_errors) { snprintf(line, sizeof(line), "flash: %lu unreadable quad-words (ECC) skipped - a write cut by a power loss", (unsigned long)port::g_flash_ecc_errors); flight.add_report(line); }

    // 4. the tasks (FreeRTOS from here on: no more delay() until the scheduler runs)
    const bool flight_ok = flight_task.start(flight);
    const bool sensor_ok = port::task_create(&sensor_entry, "wb_sensor", WB_LOW_RAM ? 8192 : 16384, nullptr, port::kPrioSensor, &sensor_handle, 0);
    snprintf(line, sizeof(line), "loop: %s, TIM7 every %lu us%s", flight_ok ? "flight task" : "FAILED to start the flight task", (unsigned long)flight.period_us(),
             sensor_ok ? "" : " - FAILED to start the sensor task");
    flight.add_report(line);
    // the SD card's writer task (a low-RAM chip starts it only when the card is selected at boot: choosing the card later in the
    // app then needs a restart there)
    if ((settings.blackbox.device == bb::DEV_SDCARD || !WB_LOW_RAM) && !sd_task.start()) flight.add_report("blackbox: FAILED to start the SD card task");
    { static uint8_t pins[port::kPinMaskBytes]; port::chip_pins(pins); server.set_chip(WB_MCU_NAME, pins); }      // the app: which MCU, which pins exist
    host.attach(&server);
    host.set_report(flight.report(), flight.report_lines());
    host.set_ports(&flight.port_use());
    for (int i = 0; i < flight.report_lines(); i++) server.log(1, flight.report()[i]);
    // the Betaflight App's CLI tab talks to this terminal: its commands (peripherals, gps_uart, ..., save = save + restart) first
    term_cli.set_ext([](const char* l, void*) { app::BfCli b(settings, server, [](const char* t, void*) { transport.write((const uint8_t*)t, strlen(t)); }, nullptr, host.ports()); return b.handle(l); }, nullptr);
    link_task.set_cli(&term_cli);
    link_task.start();
    vTaskStartScheduler();                                   // does not return
}

void loop() {}                                               // (never runs: the scheduler took over in setup())

#endif  // ARDUINO_ARCH_STM32
