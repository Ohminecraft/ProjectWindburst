/*
 * This file is part of Windburst Project
 *
 * Firmware entry point of the DIY transmitter (-D_WINDBURST_TX, env tx_c5, ESP32-C5; docs/TX.md). The flight controller is
 * main.cpp / main_stm32.cpp.
 *   loop(): one packet period after the other: read the four joystick ADCs, TxApp::cycle() (sticks -> channels -> the nRF24 link),
 *           and in between the app link. One thread: nothing to lock.
 * The app link listens on BOTH sockets of the DevKit at once: UART0 (the "UART" socket, a CH343 / CP2102 bridge, 230400 baud -
 * the app's default) and the chip's own USB Serial/JTAG (the "USB" socket, any baud); the replies go out on the one the app
 * last spoke on. Which one `Serial` is (ARDUINO_USB_CDC_ON_BOOT) does not matter: Serial0 and the HWCDC are named here.
 * The packet period comes from the settings (tx_radio.pps); the loop looks at the clock every millisecond, so a packet leaves
 * at most ~1 ms late - the flight controller only counts packets, it does not need them on the microsecond.
 * NOT confirmed on hardware.
 */
#if defined(ARDUINO_ARCH_ESP32) && defined(_WINDBURST_TX)
#include <Arduino.h>
#include <SPI.h>
#include <esp_random.h>
#include "config/version.h"
#include "port/chip.h"
#include "proto/wbp_server.h"
#include "proto/settings_store.h"
#include "app/nvs_storage.h"
#include "hal/rx/rx_nrf24.h"
#include "tx/tx_app.h"

#include "HWCDC.h"
#if ARDUINO_USB_MODE && ARDUINO_USB_CDC_ON_BOOT
#define WB_USB_JTAG HWCDCSerial                     // (the core made it: it is `Serial` too)
#else
static HWCDC usb_jtag;                              // the core makes it only with ARDUINO_USB_CDC_ON_BOOT=1: made here
#define WB_USB_JTAG usb_jtag
#endif
#ifndef WB_LINK_BAUD
#define WB_LINK_BAUD 230400          // the UART socket; the app opens a port at 230400 unless told otherwise
#endif

// the app link: the wire the app last spoke on; frames that do not fit into its transmit buffer are skipped (the server never waits)
struct DualLink : wbp::Transport {
    bool usb = false;                                // false: UART0, true: USB Serial/JTAG
    size_t write(const uint8_t* d, size_t n) override { return usb ? WB_USB_JTAG.write(d, n) : Serial0.write(d, n); }
    size_t writable() override { const int a = usb ? WB_USB_JTAG.availableForWrite() : Serial0.availableForWrite(); return a > 0 ? (size_t)a : 0; }
    uint32_t bytes_per_second() override { return usb ? 0 : WB_LINK_BAUD / 10; }
};

static wbp::Settings settings;
static NvsStorage nvs;
static wbp::SettingsStore store(nvs);
static DualLink app_link;
static tx::TxApp txapp;
static wbp::Server server(settings, &store, txapp, app_link, wbp::ROLE_TX, WB_FW_MAJOR, WB_FW_MINOR, WB_FW_PATCH, "Windburst TX", wbp::FEAT_STREAMS);
static nrf::ArduinoBus radio_bus(FSPI);             // the C5 has one general-purpose SPI bus
static uint32_t next_us = 0, last_ms = 0;

static uint32_t hw_random() { return esp_random(); }
static tx::StickInput read_sticks() {
    tx::StickInput in;
    for (int i = 0; i < 4; i++) {                    // four readings each, averaged (the ADC's noise is a few counts)
        const uint8_t pin = settings.tx_sticks.adc_pin[i]; uint32_t sum = 0;
        for (int k = 0; k < 4; k++) sum += (uint32_t)analogRead(pin);
        in.raw[i] = (uint16_t)(sum / 4);
    }
    return in;
}

// what came in on one wire; the replies follow the wire the app speaks on (a half frame from the other one is given up)
static void pump(Stream& port, bool usb) {
    while (port.available() > 0) {
        if (app_link.usb != usb) { app_link.usb = usb; server.on_idle(); }
        server.on_byte((uint8_t)port.read());
    }
}

void setup() {
    Serial0.setTxBufferSize(2048); Serial0.setRxBufferSize(1024);
    Serial0.begin(WB_LINK_BAUD);
    WB_USB_JTAG.setTxBufferSize(2048); WB_USB_JTAG.setRxBufferSize(1024);
    WB_USB_JTAG.setTxTimeoutMs(0);                   // (no app on the USB socket: a write must not wait)
    WB_USB_JTAG.begin();
    analogReadResolution(12);
    const bool nvs_ok = nvs.begin("windburst_tx");
    store.load(settings);
    txapp.configure(settings); txapp.set_random(&hw_random); txapp.set_server(&server);
    const wbp::S_tx_radio& r = settings.tx_radio; char line[120];
    const bool bus = radio_bus.begin(r.pin_sck, r.pin_miso, r.pin_mosi, r.pin_cs, r.pin_ce);
    const bool radio = bus && txapp.begin(radio_bus);
    snprintf(line, sizeof(line), "radio: nRF24 %s (SCK %u, MISO %u, MOSI %u, CSN %u, CE %u), RF channel %u, %s", radio ? "found" : bus ? "NOT FOUND - wiring, 3.3 V, a 10 uF capacitor at its power pins" : "FAILED - a pin is not set",
             r.pin_sck, r.pin_miso, r.pin_mosi, r.pin_cs, r.pin_ce, (unsigned)r.rf_channel, r.bound ? "bound" : "not bound: BIND in the Radio tab");
    txapp.add_report(line);
    const wbp::S_tx_sticks& k = settings.tx_sticks;
    snprintf(line, sizeof(line), "sticks: ADC pins %u %u %u %u, throttle %s", k.adc_pin[0], k.adc_pin[1], k.adc_pin[2], k.adc_pin[3], k.throttle_mode == wbp::TX_THROTTLE_ALTPOS ? "ALTPOS (centre = hold the height)" : "MANUAL (moves up / down, stays when let go)");
    txapp.add_report(line);
    snprintf(line, sizeof(line), "settings: %s; link: UART0 %u baud + USB Serial/JTAG; mcu: %s", nvs_ok ? "NVS" : "NVS FAILED (defaults, nothing is kept)", (unsigned)WB_LINK_BAUD, WB_MCU_NAME);
    txapp.add_report(line);
    server.settings_loaded();
    next_us = micros(); last_ms = millis();
}

void loop() {
    const uint32_t now = micros();
    if ((int32_t)(now - next_us) >= 0) {
        txapp.cycle(now, read_sticks());
        next_us += txapp.period_us(); if ((int32_t)(now - next_us) > 0) next_us = now + txapp.period_us();     // (late: not made up in a burst)
    }
    pump(Serial0, false); pump(WB_USB_JTAG, true);
    const uint32_t ms = millis();
    if (ms != last_ms) { last_ms = ms; server.tick(ms); }
    if (txapp.reboot_requested()) { delay(50); ESP.restart(); }
    vTaskDelay(1);                                   // (the idle task needs the core now and then: the C5 has one)
}

#endif  // the transmitter
