/*
 * This file is part of Windburst Project
 *
 * Firmware entry point of the STM32 build (STM32H5 so far: docs/STM32.md). The ESP32 one is main.cpp; both run the same
 * FlightApp, settings, app link and blackbox - only the chip's ends differ:
 *   "wb_flight" : the control cycle, woken by TIM7                             [port/stm32/flight_task_stm32.h]
 *   "wb_sensor" : sensors, receiver, app requests, once per millisecond tick   [FlightApp::slow_update]
 *   "wb_link"   : app link over USB CDC (or a UART) + blackbox                 [LinkTask, wbp::Server, bb::Service]
 * One core: FreeRTOS priorities in that order (port/rtos.h). Settings and the flight log share flash bank 2 (port/stm32/flash_h5.h).
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
#include "port/stm32/flash_h5.h"
#include "app/flight_app.h"
#include "port/stm32/flight_task_stm32.h"
#include "app/link_arduino.h"
#include "proto/kv_flash.h"
#include "blackbox/bb_qword.h"
#include "blackbox/bb_header.h"

static constexpr uint32_t kSettingsBytes = 64u * 1024u;    // bank 2: settings first, the flight log after them

static wbp::Settings settings;
static FlightApp flight;
static FlightTask flight_task;
static port::FlashH5 flash;
static port::QwFlashSlice settings_region(flash, 0, kSettingsBytes);
static port::QwFlashSlice log_region(flash, kSettingsBytes, FLASH_BANK_SIZE - kSettingsBytes);
static wbp::KvFlashStorage kv(settings_region);
static wbp::SettingsStore store(kv);
static bb::QwBlockStore log_store(log_region, "flash bank 2");
static port::SerialPort uart_a, uart_b, uart_c;            // the instance (USART1, UART4 ...) follows the pins in the settings
static SerialTransport transport(Serial, WB_LINK_BYTES_PER_S);
static app::LinkHost host(flight.link, &LinkTask::wait_1ms, nullptr, &LinkTask::now_ms);
static wbp::Server server(settings, &store, host, transport, wbp::ROLE_RX, WB_FW_MAJOR, WB_FW_MINOR, WB_FW_PATCH, "Windburst RX",
                          wbp::FEAT_STREAMS | wbp::FEAT_CALIBRATION | wbp::FEAT_MOTOR_TEST | wbp::FEAT_GPS | wbp::FEAT_BLACKBOX | wbp::FEAT_BATTERY | wbp::FEAT_BEEPER | wbp::FEAT_STM32);
static LinkTask link_task(server, Serial, transport);
static app::MspLink msp_link(host, transport);
static app::Cli term_cli(settings, server, [](const char* t, void*) { transport.write((const uint8_t*)t, strlen(t)); }, nullptr);     // '#' on the serial port: the command line (app/cli.h)
static bb::Service blackbox(flight.blackbox, transport);
static bb::FirmwareHeader log_header{&settings, "Windburst RX", WB_FW_MAJOR, WB_FW_MINOR, WB_FW_PATCH};
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
    flight.configure(settings);
    flight.begin();

    // 3. blackbox in bank 2
    char line[176];                                          // (= FlightApp::kReportLen)
    blackbox.attach_store(&log_store);
    if (blackbox.log().ok())
        snprintf(line, sizeof(line), "blackbox: %s %lu kB, %d sessions, %lu kB used%s", log_store.name(), (unsigned long)(log_store.size() / 1024u), blackbox.log().sessions(),
                 (unsigned long)(blackbox.log().used() / 1024u), blackbox.log().needs_erase() ? " - holds other data: erase it in the app" : "");
    else snprintf(line, sizeof(line), "blackbox: FAILED - the log region of flash bank 2 cannot be read");
    flight.add_report(line);
    blackbox.set_header_source(&bb::FirmwareHeader::write, &log_header);
    blackbox.set_loop(flight.core.cfg.pid_hz(), flight.motors.cfg().count, flight.core.cfg.loop_hz);   // a record per control cycle at most
    host.set_blackbox(&blackbox, &settings);
    host.set_port(&transport, &g_link_kicks);
    {   // the chip's 96-bit unique id: the MSP side's UID, and (with the clock) a boot id that differs every start
        const uint32_t u0 = HAL_GetUIDw0(), u1 = HAL_GetUIDw1(), u2 = HAL_GetUIDw2();
        uint8_t uid[12]; memcpy(uid, &u0, 4); memcpy(uid + 4, &u1, 4); memcpy(uid + 8, &u2, 4); host.set_uid(uid);
        host.set_boot_id(u0 ^ (u1 * 2654435761u) ^ u2 ^ (uint32_t)micros() ^ (port::g_flash_ecc_errors << 24));
        msp_link.set_firmware(WB_FW_MAJOR, WB_FW_MINOR, WB_FW_PATCH, __DATE__, __TIME__);
        link_task.set_msp(&msp_link, [](EscPins& p) { p.n = 0; for (int m = 0; m < 8 && flight.escPin(m) != GPIO_UNUSED; m++) p.pin[p.n++] = flight.escPin(m); p.pullup = flight.escPullup(); });
    }
    {   // why the chip started (RCC reset flags), then clear them for the next time
        const uint32_t f = RCC->RSR;
        const char* why = (f & RCC_RSR_BORRSTF) ? "BROWN-OUT or power on" : (f & RCC_RSR_IWDGRSTF) ? "CRASH (independent watchdog)" : (f & RCC_RSR_WWDGRSTF) ? "CRASH (window watchdog)"
                        : (f & RCC_RSR_SFTRSTF) ? "software restart" : (f & RCC_RSR_LPWRRSTF) ? "low-power reset" : (f & RCC_RSR_PINRSTF) ? "reset pin" : "unknown";
        snprintf(line, sizeof(line), "reset: %s", why); flight.add_report(line);
        SET_BIT(RCC->RSR, RCC_RSR_RMVF);
    }
    if (WB_LINK_BYTES_PER_S) snprintf(line, sizeof(line), "link: %s, %lu bytes/s (the live log is slowed down to fit)", WB_LINK_KIND, (unsigned long)WB_LINK_BYTES_PER_S);
    else snprintf(line, sizeof(line), "link: %s", WB_LINK_KIND);
    flight.add_report(line);
    snprintf(line, sizeof(line), "settings: %s (gen %lu, %lu/%lu B), loaded 0x%06lx (0x%06lx older), defaulted 0x%06lx, clamped %u",
             kv_ok ? "flash ok" : "flash FAILED", (unsigned long)kv.generation(), (unsigned long)kv.used(), (unsigned long)kv.half_size(),
             (unsigned long)(rep.loaded_mask & 0xFFFFFFu), (unsigned long)(rep.migrated_mask & 0xFFFFFFu), (unsigned long)(rep.defaulted_mask & 0xFFFFFFu), (unsigned)(uint16_t)rep.clamped_fields);
    flight.add_report(line);
    if (port::g_flash_ecc_errors) { snprintf(line, sizeof(line), "flash: %lu unreadable quad-words (ECC) skipped - a write cut by a power loss", (unsigned long)port::g_flash_ecc_errors); flight.add_report(line); }

    // 4. the tasks (FreeRTOS from here on: no more delay() until the scheduler runs)
    const bool flight_ok = flight_task.start(flight);
    const bool sensor_ok = port::task_create(&sensor_entry, "wb_sensor", 16384, nullptr, port::kPrioSensor, &sensor_handle, 0);
    snprintf(line, sizeof(line), "loop: %s, TIM7 every %lu us%s", flight_ok ? "flight task" : "FAILED to start the flight task", (unsigned long)flight.period_us(),
             sensor_ok ? "" : " - FAILED to start the sensor task");
    flight.add_report(line);
    host.attach(&server);
    host.set_report(flight.report(), flight.report_lines());
    for (int i = 0; i < flight.report_lines(); i++) server.log(1, flight.report()[i]);
    link_task.set_cli(&term_cli);
    link_task.start();
    vTaskStartScheduler();                                   // does not return
}

void loop() {}                                               // (never runs: the scheduler took over in setup())

#endif  // ARDUINO_ARCH_STM32
