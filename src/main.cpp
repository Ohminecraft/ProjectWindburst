/*
 * This file is part of Windburst Project
 *
 * Firmware entry point.
 *   RX build (-D_WINDBURST_RX, ESP32-S3): the flight controller.
 *       core 1  "wb_flight" : the control cycle (IMU -> FlightCore -> motors), woken by a hardware timer    [FlightTask + FlightApp::flight_cycle]
 *       core 1  "wb_sensor" : sensors, receiver, app requests, about once per millisecond                   [FlightApp::slow_update]
 *                             (its own task since 0.8.8: Arduino's loop task is deleted at the end of setup(), loop() never runs)
 *       core 0  "wb_link"   : app link (WBP protocol over USB / UART0) + blackbox (log memory, stream)     [LinkTask + wbp::Server + bb::Service]
 *       core 0  "wb_sd"     : writes the flight log to the SD card, when the settings select the card      [SdCardTask + bb::CardLog]
 *   TX build (-D_WINDBURST_TX, ESP32-C5): the DIY transmitter - src/main_tx.cpp, not this file.
 */
#if defined(ARDUINO_ARCH_ESP32) && !defined(_WINDBURST_TX)      // (the STM32 build: main_stm32.cpp; the transmitter: main_tx.cpp)
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#ifndef WB_LINK_BAUD
#define WB_LINK_BAUD 115200          // UART0 / USB-serial bridge. Ignored on USB (see WB_LINK_KIND in app/link_arduino.h).
#endif
#include "config/version.h"

#include "app/flight_app.h"
#include "app/flight_task.h"
#include "app/link_arduino.h"
#include "hal/can/twai_bus.h"
#include "blackbox/bb_store_esp.h"
#include "blackbox/bb_card_esp.h"
#include "blackbox/bb_header.h"
#include "app/usb_name.h"
#include <esp_mac.h>
#if WB_USB_OTG
// rx_s3_otg: the firmware runs the USB stack (TinyUSB) and starts it in setup(), after the settings are read, so that the board
// shows up under its own name (general.usb_name). The app link is usb_cdc; `Serial` is UART0 there.
#include "USB.h"
#include "esp32-hal-tinyusb.h"
static USBCDC usb_cdc;                               // (the core defines USBSerial only when USB starts at boot)
#define WB_LINK usb_cdc
#define WB_FEAT_USB wbp::FEAT_USB_NAME
#else
#define WB_LINK Serial
#define WB_FEAT_USB 0u
#endif

static wbp::Settings settings;
#if defined(CONFIG_IDF_TARGET_ESP32)
// The plain ESP32: the Arduino core reserves the start of its data RAM for the Bluetooth controller, and the static data no longer
// fits. The flight app (~75 kB) lives in the internal heap instead (asked for explicitly: never PSRAM), made before
// everything below that uses it.
#include <new>
#include "esp_heap_caps.h"
static FlightApp& flight = *new (heap_caps_malloc(sizeof(FlightApp), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)) FlightApp;
#else
static FlightApp flight;
#endif
static FlightTask flight_task;
static NvsStorage nvs;
static wbp::SettingsStore store(nvs);
// the app link: the serial port (USB / UART0)
#if WB_HAS_TWAI
static TwaiBus can_bus;                                  // the CAN controller (section can: DroneCAN)
#endif
#if WB_USB_OTG
static TinyUsbPort usb_port;
#else
static StreamPort usb_port(Serial);
#endif
static app::QueuedTransport transport(usb_port, WB_LINK_BYTES_PER_S, &LinkTask::wait_1ms, nullptr);
static app::LinkHost host(flight.link, &LinkTask::wait_1ms, nullptr, &LinkTask::now_ms);
static wbp::Server server(settings, &store, host, transport, wbp::ROLE_RX, WB_FW_MAJOR, WB_FW_MINOR, WB_FW_PATCH, "Windburst RX",
                          wbp::FEAT_STREAMS | wbp::FEAT_CALIBRATION | wbp::FEAT_MOTOR_TEST | wbp::FEAT_GPS | wbp::FEAT_BLACKBOX | wbp::FEAT_BATTERY | wbp::FEAT_BEEPER | WB_FEAT_USB);
static LinkTask link_task(server, WB_LINK, transport);
static app::MspLink msp_link(host, transport, &settings);           // ESC tools on the same port (ESC passthrough)
static app::Cli term_cli(settings, server, [](const char* t, void*) { transport.write((const uint8_t*)t, strlen(t)); }, nullptr);     // '#' on the serial port: the command line (app/cli.h)
static EspPartitionStore log_store;                      // flash region of the flight log (partition "wblog", or the stock "spiffs" one)
static bb::Service blackbox(flight.blackbox, transport);
static bb::FirmwareHeader log_header{&settings, "Windburst RX", WB_FW_MAJOR, WB_FW_MINOR, WB_FW_PATCH};
static SdFileStore sd_store;                             // SD card (SPI or SDMMC), only touched when blackbox.device = SDCARD
static RtosLock sd_lock;
static bb::CardLog card_log(sd_store, sd_lock);
static SdCardTask sd_task(card_log);

// The sensor thread. Before 0.8.8 it was Arduino's loop(); now a task of its own, so that its stack and priority are ours
// (the loop task's 8 kB were set by the core, and the core may add work of its own to that task). Priority 1, as the loop task
// had: below the flight task on the same core, which preempts it for every control cycle.
static constexpr uint32_t kSensorStack = 16384;
RTC_NOINIT_ATTR static uint32_t g_boot_count;               // (link stream boot_id)
RTC_NOINIT_ATTR static esc4way::Trace g_4way_trace;         // the last ESC passthrough session: survives the restart that ends it

static void sensor_entry(void*) {
    for (;;) {
        if (flight_task.running()) { flight.slow_update(); vTaskDelay(1); }  // one pass per millisecond tick
        else flight.run();                                                    // no flight task: one thread does both, as on a PC
    }
}

#if WB_USB_OTG
// "1200 baud touch" (PlatformIO / Arduino IDE before an upload): restart into the ROM bootloader. DTR / RTS never restart the chip
// (USBCDC::enableReboot(false)): ESC tools and terminals toggle them (0.9.2).
static void on_usb_cdc(void*, esp_event_base_t, int32_t id, void* data) {
    if (id == ARDUINO_USB_CDC_LINE_CODING_EVENT && ((arduino_usb_cdc_event_data_t*)data)->line_coding.bit_rate == 1200) usb_persist_restart(RESTART_BOOTLOADER);
}
static char g_usb_name[app::kUsbNameLen + 1];
#endif

void setup() {
#if !WB_USB_OTG
    Serial.setRxBufferSize(2048);
#if !defined(WB_LINK_NO_TX_BUFFER)
    Serial.setTxBufferSize(4096);                        // (the queue in front of it is SerialTransport's own, 8 kB)
#endif
    Serial.begin(WB_LINK_BAUD);
#if WB_LINK_KICK
    Serial.setTxTimeoutMs(0);                            // a write to the USB port never waits (link_arduino.h: StreamPort)
#endif
#endif

    // 1. settings: a missing / resized / corrupted section falls back to its defaults, the others are kept
    const bool nvs_ok = nvs.begin();
    const wbp::SettingsStore::LoadReport rep = store.load(settings);
#if WB_USB_OTG
    //    ... then USB, under the name from the settings
    app::usb_product_name(settings.general.usb_name, WB_MCU_NAME, g_usb_name);
    USB.manufacturerName("Windburst"); USB.productName(g_usb_name);
    USB.VID(0x0483); USB.PID(0x5740); USB.firmwareVersion((WB_FW_MAJOR << 8) | WB_FW_MINOR); // (0x5740 = STMicroelectronics Virtual COM Port) for betaflight app compatibility
    usb_cdc.setRxBufferSize(2048);
    usb_cdc.enableReboot(false); usb_cdc.onEvent(on_usb_cdc);
    usb_cdc.begin();
    USB.begin();
#endif

    // 2. hardware
#if (defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT) || WB_USB_OTG
    flight.usb_link = true;
    flight.uart0 = &Serial0;                             // the app talks over USB: UART0 is free (an MSP optical flow sensor can have it)
#endif
    esp_efuse_mac_get_default(flight.chip_uid);              // (the MAC address: the chip's unique id, for DroneCAN too)
#if WB_HAS_TWAI
    flight.can_port = &can_bus;
#endif
    flight.configure(settings);
    flight.begin();
    server.settings_loaded();                                // what flash holds, what the firmware runs with: "unsaved" / "reboot required" from here

    // 3. blackbox: find the sessions already in the log memory (a few hundred short flash reads), hand the service to the link side
    char line[176];                                          // (= FlightApp::kReportLen)
    if (log_store.begin()) {
        blackbox.attach_store(&log_store);
        snprintf(line, sizeof(line), "blackbox: partition '%.16s' %lu kB, %d sessions, %lu kB used%s", log_store.name(), (unsigned long)(log_store.size() / 1024u), blackbox.log().sessions(),
                 (unsigned long)(blackbox.log().used() / 1024u), blackbox.log().needs_erase() ? " - holds other data: erase it in the app" : "");
    } else snprintf(line, sizeof(line), "blackbox: FAILED - no log partition (needs a data partition named wblog or spiffs)");
    flight.add_report(line);
    //    SD card: mounted now when the settings select it. A card that is missing (or selected later in the app) is looked for
    //    every 3 s by the writer task. With another device selected the card and its pins are never touched.
    if (settings.blackbox.device == bb::DEV_SDCARD) {
        card_log.configure_device(true, settings.blackbox.sd_mode, settings.blackbox.sd_mhz);
        SdCardTask::give_buffer(card_log);
        if (card_log.begin()) snprintf(line, sizeof(line), "blackbox: SD card '%s' %lu MB, %lu MB free, %d log files, buffer %lu kB", sd_store.name(), (unsigned long)(card_log.capacity_kb() / 1024u),
                                       (unsigned long)((card_log.capacity_kb() - card_log.used_kb()) / 1024u), card_log.sessions(), (unsigned long)(card_log.buffer_size() / 1024u));
        else snprintf(line, sizeof(line), "blackbox: SD card FAILED - %s (it is looked for again every 3 s)", card_log.why());
        flight.add_report(line);
    }
    blackbox.attach_card(&card_log);
    blackbox.set_header_source(&bb::FirmwareHeader::write, &log_header);
    blackbox.set_loop(flight.core.cfg.pid_hz(), flight.motors.cfg().count, flight.core.cfg.loop_hz);   // a record per control cycle at most
    host.set_blackbox(&blackbox, &settings);
    host.set_port(&transport, &g_link_kicks);                // (the link stream: transmit queue, USB kicks)
    // A number that differs every start: esp_random() alone is only pseudo-random with the radio off, so a counter in RTC memory
    // that survives a restart (garbage after power-on - just as good) and the microsecond clock are mixed in.
    g_boot_count++; host.set_boot_id(esp_random() ^ (g_boot_count * 2654435761u) ^ (uint32_t)micros());
    {   // ESC passthrough: the MSP side gets the chip's id and the build; the link task gets the motor pins when a tool asks
        uint8_t uid[12] = {}; esp_efuse_mac_get_default(uid); host.set_uid(uid);
        msp_link.set_firmware(WB_FW_MAJOR, WB_FW_MINOR, WB_FW_PATCH, __DATE__, __TIME__);
        link_task.set_msp(&msp_link, [](EscPins& p) { p.n = 0; for (int m = 0; m < 8 && flight.escPin(m) != GPIO_UNUSED; m++) p.pin[p.n++] = flight.escPin(m); p.pullup = flight.escPullup(); });
    }

    // 4. the control cycle gets its own task and timer. From here on FlightApp is used by two threads (app/sync.h).
    if (flight_task.start(flight)) snprintf(line, sizeof(line), "loop: flight task on core %d, timer every %lu us", (int)xPortGetCoreID(), (unsigned long)flight.period_us());
    else snprintf(line, sizeof(line), "loop: FAILED to start the flight task - the control cycle shares its thread with the sensors (slower)");
    flight.add_report(line);
    //    the sensor thread (starts now: from here the flight task and it share FlightApp; the link side does not exist yet)
    const bool sensor_task = xTaskCreatePinnedToCore(&sensor_entry, "wb_sensor", kSensorStack, nullptr, 1, nullptr, xPortGetCoreID()) == pdPASS;
    if (!sensor_task) flight.add_report("loop: FAILED to start the sensor task - it runs in Arduino's loop task instead");

    {   // why the chip started: a reset in flight (watchdog, crash, brown-out) shows here the next time the app connects
        const esp_reset_reason_t r = esp_reset_reason();
        const char* why = r == ESP_RST_POWERON ? "power on" : r == ESP_RST_SW ? "software restart" : r == ESP_RST_PANIC ? "CRASH (panic)" : r == ESP_RST_INT_WDT ? "CRASH (interrupt watchdog)"
                        : r == ESP_RST_TASK_WDT ? "CRASH (task watchdog)" : r == ESP_RST_WDT ? "CRASH (watchdog)" : r == ESP_RST_BROWNOUT ? "BROWN-OUT (supply voltage dropped)"
                        : r == ESP_RST_EXT ? "reset pin" : r == ESP_RST_DEEPSLEEP ? "deep sleep" : r == ESP_RST_USB ? "USB (the port's reset line)" : r == ESP_RST_JTAG ? "JTAG" : "unknown";
        snprintf(line, sizeof(line), "reset: %s", why); flight.add_report(line);
        snprintf(line, sizeof(line), "mcu: %s rev %u, %lu MHz, flash %lu MB, PSRAM %lu kB", WB_MCU_NAME, (unsigned)ESP.getChipRevision(), (unsigned long)getCpuFrequencyMhz(),
                 (unsigned long)(ESP.getFlashChipSize() >> 20), (unsigned long)(ESP.getPsramSize() >> 10));
        flight.add_report(line);
        // after an ESC passthrough session (it ends with a software restart): what the PC tool asked and what the ESC answered
        if (r == ESP_RST_SW && g_4way_trace.valid()) {
            static char tl[4][176]; const int n = g_4way_trace.lines(tl, 4);
            for (int i = 0; i < n; i++) flight.add_report(tl[i]);
        }
        g_4way_trace.clear();
    }
    if (WB_LINK_BYTES_PER_S) snprintf(line, sizeof(line), "link: %s, %lu bytes/s (the live log is slowed down to fit)", WB_LINK_KIND, (unsigned long)WB_LINK_BYTES_PER_S);
    else snprintf(line, sizeof(line), "link: %s", WB_LINK_KIND);
    flight.add_report(line);
#if WB_USB_OTG
    snprintf(line, sizeof(line), "usb: '%s' (general.usb_name; flashing: 1200 baud touch, DTR / RTS never restart the chip)", g_usb_name); flight.add_report(line);
#endif

    // 5. boot report (kept for the app's REPORT command) - sent once now, BEFORE the link task exists: from here on only that task uses the server
    // (bit = section id; 32-bit masks: the line provably fits its buffer)
    static_assert(wbp::kSectionCount < 32, "a section id of 32 or more needs wider masks in the boot report");
    snprintf(line, sizeof(line), "settings: %s, loaded 0x%08lx (0x%08lx from an older firmware), defaulted 0x%08lx, clamped %u",
             nvs_ok ? "NVS ok" : "NVS FAILED (nothing can be saved)", (unsigned long)rep.loaded_mask, (unsigned long)rep.migrated_mask, (unsigned long)rep.defaulted_mask, (unsigned)(uint16_t)rep.clamped_fields);
    flight.add_report(line);
    flight.add_report("============= End Initialization =============");
    { static uint8_t pins[port::kPinMaskBytes]; port::chip_pins(pins); server.set_chip(WB_MCU_NAME, pins); }      // the app: which MCU, which pins exist
    host.attach(&server);
    host.set_report(flight.report(), flight.report_lines());
    for (int i = 0; i < flight.report_lines(); i++) server.log(1, flight.report()[i]);

    // 6. app link, SD writer - then Arduino's loop task (this one) deletes itself
    link_task.set_cli(&term_cli);                            // (not "cli": that is the ESP32 core's interrupts-off macro)
    link_task.set_trace(&g_4way_trace);
    link_task.start();
    sd_task.start();
    if (sensor_task) vTaskDelete(nullptr);                   // (otherwise loop() below takes over)
}

void loop() { sensor_entry(nullptr); }                       // only if the sensor task could not be created

#endif  // ARDUINO_ARCH_ESP32, not the transmitter
