/*
 * This file is part of Windburst Project
 */

#pragma once

#include <Arduino.h>
#include "../port/platform.h"
#include "../port/rtos.h"
#include "../port/gpio.h"
#include "../proto/wbp_server.h"
#include "link_host.h"
#include "tx_queue.h"
#include "link_mux.h"
#include "cli.h"
#include "../hal/motor/esc4way.h"
#if WB_PLATFORM_ESP32
#include <Preferences.h>
#include <driver/gpio.h>
#include <esp_rom_gpio.h>
#include <soc/gpio_sig_map.h>
#include <esp_timer.h>
#endif

// ============================================================================
// Arduino / ESP32 ends of the app link:
//   SerialTransport  - WBP frames over `Serial`, whichever it is: UART0, USB Serial/JTAG (HWCDC) or native USB CDC (TinyUSB).
//                      A queue of its own in front of the port (tx_queue.h): telemetry frames that do not fit are dropped, so a
//                      closed / slow serial monitor can never stall anything, and the port is never asked to wait.
//   NvsStorage       - settings records in NVS flash (Preferences), one key per settings section.
//   LinkTask         - a FreeRTOS task (core 0, low priority) that owns the wbp::Server: reads bytes, answers requests, emits streams.
//                      The flight loop (core 1) is never touched from here; requests cross through app::Exchange.
//                      The same bytes go to app::MspLink (ESC tools: BLHeliSuite32, ESC Configurator); when one of them asks for
//                      ESC passthrough, this task runs the 4-way interface (EspEscPort below) and restarts the firmware after it.
// NOTE: an NVS write stalls flash access on both cores for a few ms. The server only saves while disarmed.
// ============================================================================
// Which port `Serial` is in this build (the board's USB settings decide), and what it carries.
#if WB_PLATFORM_STM32
#if defined(USBCON) && defined(USBD_USE_CDC)
#define WB_LINK_KIND "USB CDC"                   // the chip's own USB (STM32duino: Serial is SerialUSB)
#define WB_LINK_BYTES_PER_S 0u
#else
#define WB_LINK_KIND "UART"
#define WB_LINK_BYTES_PER_S (WB_LINK_BAUD / 10u)
#endif
#elif defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
#if defined(ARDUINO_USB_MODE) && ARDUINO_USB_MODE
#define WB_LINK_KIND "USB Serial/JTAG"
#define WB_LINK_BYTES_PER_S 0u
#else
#define WB_LINK_KIND "native USB CDC"
#define WB_LINK_BYTES_PER_S 60000u       // one 64-byte packet per pass of the link task
#define WB_LINK_NO_TX_BUFFER 1           // (this port class has no setTxBufferSize)
#endif
#else
#define WB_LINK_KIND "UART0"
#define WB_LINK_BYTES_PER_S (WB_LINK_BAUD / 10u)
#endif

// USB Serial/JTAG (HWCDC): when its transmit ring is full and the interrupt that drains it is not armed (the USB link hiccuped:
// a bus reset, a moment the host did not poll, the core's start-of-frame watchdog flapping), only a write() arms it again. A
// sender that only writes when there is room then waits for ever: the app sees a firmware that never answers again. So after
// kLinkKickMs without room one byte is offered anyway - it is a byte of the next frame, taken or not, nothing is lost or
// doubled - and the core re-arms its interrupt. (The core's write timeout is 0 in this build: that write never waits.)
#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT && defined(ARDUINO_USB_MODE) && ARDUINO_USB_MODE
#define WB_LINK_KICK 1
#else
#define WB_LINK_KICK 0
#endif
static constexpr uint32_t kLinkKickMs = 50;
static inline volatile uint32_t g_link_kicks = 0;               // how often the port had to be kicked (reported in the app's log)

class StreamPort : public app::RawPort {
public:
    explicit StreamPort(Stream& s) : s_(s) { kick_.after_ms = WB_LINK_KICK ? kLinkKickMs : 0; }
    size_t room() override {
        const uint32_t k = kick_.kicks; const size_t r = kick_.room(s_.availableForWrite(), millis());
        if (kick_.kicks != k) g_link_kicks = kick_.kicks;
        return r;
    }
    size_t put(const uint8_t* d, size_t n) override { return s_.write(d, n); }
private:
    Stream& s_; app::StallKick kick_;
};

class SerialTransport : public app::QueuedTransport {
public:
    SerialTransport(Stream& s, uint32_t bytes_per_s) : app::QueuedTransport(port_, bytes_per_s, &SerialTransport::wait_1ms, nullptr), port_(s) {}
private:
    static void wait_1ms(void*) { vTaskDelay(1); }
    StreamPort port_;
};

#if WB_PLATFORM_ESP32
class NvsStorage : public wbp::Storage {
public:
    bool begin(const char* ns = "windburst") { ok_ = prefs_.begin(ns, false); return ok_; }
    int read(const char* key, uint8_t* buf, size_t cap) override {
        if (!ok_ || !prefs_.isKey(key)) return -1;
        const size_t len = prefs_.getBytesLength(key);
        if (len == 0 || len > cap) return -1;
        return (int)prefs_.getBytes(key, buf, len);
    }
    bool write(const char* key, const uint8_t* data, size_t n) override { return ok_ && prefs_.putBytes(key, data, n) == n; }
private:
    Preferences prefs_; bool ok_ = false;
};

#endif  // WB_PLATFORM_ESP32 (NvsStorage)

// The 4-way interface on the board: the PC on `Serial` (through the link's transmit queue), the ESC wires bit-banged with the GPIO
// driver. Between their bytes the wires are inputs, pulled up (inside, or by the 2k resistors of a board with motors.ext_pullup).
// lock(): a critical section - one byte on the wire takes 572 us and its bits must come 52 us apart (esc4way.h never holds it longer).
struct EscPins { uint8_t pin[8] = {}; int n = 0; bool pullup = true; };
#if WB_PLATFORM_ESP32
class EspEscPort {
public:
    static constexpr uint32_t kIdleMs = 300000;             // no byte from the PC for 5 minutes: the tool is gone, the firmware restarts
    EspEscPort(Stream& s, app::QueuedTransport& tx, const EscPins& p) : s_(s), tx_(tx), p_(p) { last_ms_ = millis(); }
    void begin() {
        for (int i = 0; i < p_.n; i++) {
            const gpio_num_t g = (gpio_num_t)p_.pin[i];
            gpio_reset_pin(g);                                // (takes the pin from the RMT / LEDC output: plain GPIO from here on)
            esp_rom_gpio_connect_out_signal(g, SIG_GPIO_OUT_IDX, false, false);
            gpio_set_level(g, 1);
            gpio_set_direction(g, GPIO_MODE_INPUT);
            if (p_.pullup) gpio_pullup_en(g); else gpio_pullup_dis(g);
            gpio_pulldown_dis(g);
        }
    }
    int host_read() { if (s_.available() > 0) { last_ms_ = millis(); return s_.read(); } return -1; }
    void host_write(const uint8_t* d, size_t n) { tx_.write(d, n); }
    void host_flush() { for (int i = 0; i < 500 && tx_.queued(); i++) { tx_.pump(); if (tx_.queued()) vTaskDelay(1); } }
    void host_wait() { tx_.pump(); vTaskDelay(1); }
    bool host_alive() { return (uint32_t)(millis() - last_ms_) < kIdleMs; }
    void esc_input(int c) { gpio_set_direction((gpio_num_t)p_.pin[c], GPIO_MODE_INPUT); }
    void esc_output(int c) { gpio_set_direction((gpio_num_t)p_.pin[c], GPIO_MODE_INPUT_OUTPUT); }
    void esc_hi(int c) { gpio_set_level((gpio_num_t)p_.pin[c], 1); }
    void esc_lo(int c) { gpio_set_level((gpio_num_t)p_.pin[c], 0); }
    bool esc_is_hi(int c) { return gpio_get_level((gpio_num_t)p_.pin[c]) != 0; }
    uint32_t micros() { return (uint32_t)esp_timer_get_time(); }
    void lock() { portENTER_CRITICAL(&mux_); }
    void unlock() { portEXIT_CRITICAL(&mux_); }
private:
    Stream& s_; app::QueuedTransport& tx_; EscPins p_; uint32_t last_ms_ = 0; portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
};
#else
class EspEscPort {                                          // (the name stays: one class per chip, picked at compile time)
public:
    static constexpr uint32_t kIdleMs = 300000;
    EspEscPort(Stream& s, app::QueuedTransport& tx, const EscPins& p) : s_(s), tx_(tx), p_(p) { last_ms_ = millis(); }
    void begin() {                                            // STM32: the motor timers let go of the pins, plain GPIO from here on
        for (int i = 0; i < p_.n; i++) { pin_[i] = (PinName)p_.pin[i]; pinMode(port::apin(p_.pin[i]), p_.pullup ? INPUT_PULLUP : INPUT); }
    }
    int host_read() { if (s_.available() > 0) { last_ms_ = millis(); return s_.read(); } return -1; }
    void host_write(const uint8_t* d, size_t n) { tx_.write(d, n); }
    void host_flush() { for (int i = 0; i < 500 && tx_.queued(); i++) { tx_.pump(); if (tx_.queued()) vTaskDelay(1); } }
    void host_wait() { tx_.pump(); vTaskDelay(1); }
    bool host_alive() { return (uint32_t)(millis() - last_ms_) < kIdleMs; }
    void esc_input(int c) { pinMode(port::apin(p_.pin[c]), p_.pullup ? INPUT_PULLUP : INPUT); }
    void esc_output(int c) { pinMode(port::apin(p_.pin[c]), OUTPUT); }
    void esc_hi(int c) { digitalWriteFast(pin_[c], HIGH); }
    void esc_lo(int c) { digitalWriteFast(pin_[c], LOW); }
    bool esc_is_hi(int c) { return digitalReadFast(pin_[c]) != 0; }
    uint32_t micros() { return ::micros(); }
    void lock() { crit_.hard_lock(); }                       // (bit timing: not even the USB interrupt in between)
    void unlock() { crit_.hard_unlock(); }
private:
    Stream& s_; app::QueuedTransport& tx_; EscPins p_; uint32_t last_ms_ = 0; PinName pin_[8] = {}; port::Critical crit_;
};
#endif

class LinkTask {
public:
    LinkTask(wbp::Server& server, Stream& port, app::QueuedTransport& tx) : server_(server), port_(port), tx_(tx) {}
    // ESC passthrough: the MSP side, and who knows the motor pins (called once the flight side has let go of them)
    using EscPinsFn = void (*)(EscPins&);
    void set_msp(app::MspLink* m, EscPinsFn pins) { msp_ = m; pins_ = pins; }
    // A second wire (WiFi): read like the serial port; mux decides where the replies go (link_mux.h). ESC tools stay on the serial port.
    void set_net(app::NetWire* net, app::LinkMux* mux) { net_ = net; mux_ = mux; }
    // The command line (cli.h): a terminal on the serial port sends '#' between two frames.
    void set_cli(app::Cli* cli) { cli_ = cli; }

    // stack: the server keeps two ~520 byte frame buffers on the stack while answering, a log download another ~1 kB, and the
    // SD card's file system (FatFs with long file names on the stack) about 2 kB below that when a file is opened / listed / read
    bool start(int core = 0, int priority = (int)port::kPrioLink) { return port::task_create(&LinkTask::entry, "wb_link", 14000, this, (UBaseType_t)priority, &handle_, core); }

    static void wait_1ms(void*) { vTaskDelay(1); }           // app::Exchange::WaitFn
    static uint32_t now_ms() { return millis(); }

private:
    static void entry(void* self) { static_cast<LinkTask*>(self)->run(); }
    void run() {
        uint32_t last_rx = millis(), kicks = g_link_kicks;
        for (;;) {
            if (g_link_kicks != kicks) {                          // the port was stuck and had to be kicked: say so (it shows up in the app's log once it flows)
                char line[96]; snprintf(line, sizeof(line), "link: the USB port took nothing for %lu ms and was restarted (%lu times since boot)", (unsigned long)kLinkKickMs, (unsigned long)g_link_kicks);
                kicks = g_link_kicks; server_.log(2, line);
            }
            uint8_t buf[128]; int n = 0;
            while (n < (int)sizeof(buf) && port_.available() > 0) buf[n++] = (uint8_t)port_.read();
            const uint32_t now = millis();
            if (n > 0) {
                if (mux_ && mux_->heard(app::LinkMux::SERIAL_WIRE)) server_.on_idle();              // (the other wire's half frame is given up)
                last_rx = now;
                for (int i = 0; i < n; i++) {                       // byte by byte: a '#' between two frames opens the command line
                    if (cli_ && cli_->active()) { cli_->feed(buf[i]); continue; }
                    if (cli_ && app::Cli::opens(buf[i], server_.rx_mid_frame() || (msp_ && msp_->mid_frame()))) { cli_->enter(); continue; }
                    server_.on_byte(buf[i]);
                    if (msp_) if (const uint8_t escs = msp_->on_byte(buf[i])) passthrough(escs);     // (does not return)
                }
            }
            int m = 0;
            if (net_) {
                net_->poll(now);
                if (net_->take_dropped() && mux_) mux_->lost(app::LinkMux::WIFI_WIRE);
                m = net_->read(buf, (int)sizeof(buf));
                if (m > 0) { if (mux_ && mux_->heard(app::LinkMux::WIFI_WIRE)) server_.on_idle(); server_.on_bytes(buf, (size_t)m); last_rx = now; }
            }
            if (n > 0 || m > 0) {}
            else if ((uint32_t)(now - last_rx) > 20u) { server_.on_idle(); last_rx = now; }   // a half-received frame is given up after 20 ms of silence
            if (!cli_ || !cli_->active()) server_.tick(now);     // (no stream frames into a terminal)
            tx_.pump();
            vTaskDelay(1);
        }
    }
    // The 4-way interface until the PC tool says goodbye (or is silent for 5 minutes), then a restart: the motor output, its RMT
    // channels and timers start afresh - simpler and safer than handing the pins back to a running driver.
    void passthrough(uint8_t escs) {
        tx_.pump(); for (int i = 0; i < 200 && tx_.queued(); i++) { vTaskDelay(1); tx_.pump(); }     // the MSP answer goes out first
        EscPins p; if (pins_) pins_(p);
        if (p.n > (int)escs) p.n = escs;
        EspEscPort port(port_, tx_, p); port.begin();
        esc4way::Interface<EspEscPort> itf(port, p.n); itf.begin();
        itf.run();
        port.host_flush(); vTaskDelay(50);
        port::restart();
    }
    wbp::Server& server_; Stream& port_; app::QueuedTransport& tx_; TaskHandle_t handle_ = nullptr;
    app::MspLink* msp_ = nullptr; EscPinsFn pins_ = nullptr;
    app::NetWire* net_ = nullptr; app::LinkMux* mux_ = nullptr; app::Cli* cli_ = nullptr;
};
