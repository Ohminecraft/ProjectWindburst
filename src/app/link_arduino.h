/*
 * This file is part of Windburst Project
 */

#pragma once

#include <Arduino.h>
#include <Preferences.h>
#include "../proto/wbp_server.h"
#include "link_host.h"
#include "tx_queue.h"

// ============================================================================
// Arduino / ESP32 ends of the app link:
//   SerialTransport  - WBP frames over `Serial`, whichever it is: UART0, USB Serial/JTAG (HWCDC) or native USB CDC (TinyUSB).
//                      A queue of its own in front of the port (tx_queue.h): telemetry frames that do not fit are dropped, so a
//                      closed / slow serial monitor can never stall anything, and the port is never asked to wait.
//   NvsStorage       - settings records in NVS flash (Preferences), one key per settings section.
//   LinkTask         - a FreeRTOS task (core 0, low priority) that owns the wbp::Server: reads bytes, answers requests, emits streams.
//                      The flight loop (core 1) is never touched from here; requests cross through app::Exchange.
// NOTE: an NVS write stalls flash access on both cores for a few ms. The server only saves while disarmed.
// ============================================================================
// Which port `Serial` is in this build (the board's USB settings decide), and what it carries.
#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
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

class StreamPort : public app::RawPort {
public:
    explicit StreamPort(Stream& s) : s_(s) {}
    size_t room() override { const int w = s_.availableForWrite(); return w > 0 ? (size_t)w : 0; }
    size_t put(const uint8_t* d, size_t n) override { return s_.write(d, n); }
private:
    Stream& s_;
};

class SerialTransport : public app::QueuedTransport {
public:
    SerialTransport(Stream& s, uint32_t bytes_per_s) : app::QueuedTransport(port_, bytes_per_s, &SerialTransport::wait_1ms, nullptr), port_(s) {}
private:
    static void wait_1ms(void*) { vTaskDelay(1); }
    StreamPort port_;
};

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

class LinkTask {
public:
    LinkTask(wbp::Server& server, Stream& port, app::QueuedTransport& tx) : server_(server), port_(port), tx_(tx) {}

    // stack: the server keeps two ~520 byte frame buffers on the stack while answering, a log download another ~1 kB, and the
    // SD card's file system (FatFs with long file names on the stack) about 2 kB below that when a file is opened / listed / read
    bool start(int core = 0, int priority = 1) { return xTaskCreatePinnedToCore(&LinkTask::entry, "wb_link", 12288, this, priority, &handle_, core) == pdPASS; }

    static void wait_1ms(void*) { vTaskDelay(1); }           // app::Exchange::WaitFn
    static uint32_t now_ms() { return millis(); }

private:
    static void entry(void* self) { static_cast<LinkTask*>(self)->run(); }
    void run() {
        uint32_t last_rx = millis();
        for (;;) {
            uint8_t buf[128]; int n = 0;
            while (n < (int)sizeof(buf) && port_.available() > 0) buf[n++] = (uint8_t)port_.read();
            const uint32_t now = millis();
            if (n > 0) { server_.on_bytes(buf, (size_t)n); last_rx = now; }
            else if ((uint32_t)(now - last_rx) > 20u) { server_.on_idle(); last_rx = now; }   // a half-received frame is given up after 20 ms of silence
            server_.tick(now);
            tx_.pump();
            vTaskDelay(1);
        }
    }
    wbp::Server& server_; Stream& port_; app::QueuedTransport& tx_; TaskHandle_t handle_ = nullptr;
};
