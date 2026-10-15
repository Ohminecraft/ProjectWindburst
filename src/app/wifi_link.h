/*
 * This file is part of Windburst Project
 */

#pragma once

#include "../port/platform.h"
#if WB_PLATFORM_ESP32
#include <sdkconfig.h>
#include <soc/soc_caps.h>
#endif

// ============================================================================
// The app link over WiFi (ESP32 with a radio; not on the STM32): section `wifi`.
//   AP  : a network of its own - "Windburst-XXXX" unless named, password "windburst" unless set - the board is 192.168.4.1
//   STA : joins a network (ssid / pass); its address is shown in the app (link stream: wifi_ip) and in the boot report once known
// WBP over TCP on `port` (5760: the simulator's port, so the app's socket://<address>:5760 is the same thing). One client at a
// time: a new connection replaces the old one (a client that vanished without closing its socket never blocks the link).
// Sockets are lwIP's own, non-blocking: nothing the network does can make the link task wait (send() takes what fits).
// Runs in the link task (core 0); the WiFi driver's own tasks run on core 0 as well, the flight loop is on core 1.
// NOT yet confirmed on hardware.
// ============================================================================
#if WB_PLATFORM_ESP32 && defined(SOC_WIFI_SUPPORTED) && SOC_WIFI_SUPPORTED
#define WB_HAS_WIFI 1
#include <Arduino.h>
#include <WiFi.h>
#include <esp_mac.h>
#include <lwip/sockets.h>
#include <lwip/netdb.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include "../proto/wbp_schema.h"
#include "link_mux.h"

class WifiLink : public app::NetWire {
public:
    // Starts the radio as the settings say. false: off, or it could not start (the line says why). line: the boot report.
    bool begin(const wbp::S_wifi& c, char* line, size_t cap) {
        mode_ = c.mode; port_ = c.port ? c.port : 5760;
        if (mode_ == wbp::WIFI_MODE_OFF) { state_ = wbp::WIFI_STATE_OFF; snprintf(line, cap, "wifi: off"); return false; }
        char ssid[33], pass[65]; text(c.ssid, sizeof(c.ssid), ssid); text(c.pass, sizeof(c.pass), pass);
        WiFi.persistent(false);                                   // (the settings are ours: nothing goes into the driver's own NVS)
        if (mode_ == wbp::WIFI_MODE_AP) {
            if (!ssid[0]) { uint8_t mac[6] = {}; esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP); snprintf(ssid, sizeof(ssid), "Windburst-%02X%02X", mac[4], mac[5]); }
            if (!pass[0]) strcpy(pass, "windburst");
            if (strlen(pass) < 8) { state_ = wbp::WIFI_STATE_FAILED; snprintf(line, cap, "wifi: FAILED - the AP password needs 8 characters or more"); return false; }
            if (!WiFi.mode(WIFI_AP) || !WiFi.softAP(ssid, pass, c.channel, 0, 2)) { state_ = wbp::WIFI_STATE_FAILED; snprintf(line, cap, "wifi: FAILED - the access point did not start"); return false; }
            WiFi.setSleep(false);
            state_ = wbp::WIFI_STATE_AP; ip_ = (uint32_t)WiFi.softAPIP();
            snprintf(line, cap, "wifi: AP '%s' channel %u, app at socket://%s:%u", ssid, (unsigned)c.channel, WiFi.softAPIP().toString().c_str(), (unsigned)port_);
        } else {
            if (!ssid[0]) { state_ = wbp::WIFI_STATE_FAILED; snprintf(line, cap, "wifi: FAILED - no network name (wifi.ssid) to join"); return false; }
            if (!WiFi.mode(WIFI_STA)) { state_ = wbp::WIFI_STATE_FAILED; snprintf(line, cap, "wifi: FAILED - the radio did not start"); return false; }
            WiFi.setSleep(false); WiFi.setAutoReconnect(true);
            WiFi.begin(ssid, pass[0] ? pass : nullptr);
            state_ = wbp::WIFI_STATE_CONNECTING;
            snprintf(line, cap, "wifi: joining '%s', app on TCP port %u (the address: System tab, once connected)", ssid, (unsigned)port_);
        }
        return true;
    }

    void poll(uint32_t now_ms) override {
        if (state_ == wbp::WIFI_STATE_OFF || state_ == wbp::WIFI_STATE_FAILED) return;
        if (mode_ == wbp::WIFI_MODE_STA && (uint32_t)(now_ms - sta_ms_) >= 250u) {          // (WiFi.status() asks the driver: four times a second is plenty)
            sta_ms_ = now_ms;
            const bool up = WiFi.status() == WL_CONNECTED;
            state_ = up ? wbp::WIFI_STATE_CONNECTED : wbp::WIFI_STATE_CONNECTING;
            ip_ = up ? (uint32_t)WiFi.localIP() : 0; rssi_ = up ? (int16_t)WiFi.RSSI() : 0;
            if (!up && client_ >= 0) drop();
        }
        if (mode_ == wbp::WIFI_MODE_AP && (uint32_t)(now_ms - sta_ms_) >= 500u) { sta_ms_ = now_ms; clients_ = (uint8_t)WiFi.softAPgetStationNum(); }
        if (listen_ < 0 && (uint32_t)(now_ms - listen_ms_) >= 1000u) { listen_ms_ = now_ms; open_listener(); }
        if (listen_ < 0) return;
        sockaddr_in from; socklen_t len = sizeof(from);
        const int fd = lwip_accept(listen_, (sockaddr*)&from, &len);
        if (fd >= 0) {
            if (client_ >= 0) drop();                                                     // the newest connection wins
            client_ = fd; nonblocking(fd);
            int one = 1; lwip_setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
            accepted_++;
        }
    }
    int read(uint8_t* buf, int cap) override {
        if (client_ < 0) return 0;
        const int n = lwip_recv(client_, buf, (size_t)cap, MSG_DONTWAIT);
        if (n > 0) return n;
        if (n == 0 || (errno != EAGAIN && errno != EWOULDBLOCK)) drop();                    // closed by the app, or broken
        return 0;
    }
    bool take_dropped() override { const bool d = dropped_; dropped_ = false; return d; }
    size_t room() override { return client_ < 0 ? 0 : 1460; }                              // (send() below takes what fits)
    size_t put(const uint8_t* d, size_t n) override {
        if (client_ < 0) return 0;
        const int k = lwip_send(client_, d, n, MSG_DONTWAIT);
        if (k > 0) return (size_t)k;
        if (k < 0 && errno != EAGAIN && errno != EWOULDBLOCK) drop();
        return 0;
    }

    uint8_t state() const { return state_; }
    uint32_t ip() const { return ip_; }                    // lowest byte first, as lwIP keeps it (192.168.4.1 = 0x0104A8C0)
    int16_t rssi() const { return rssi_; }
    uint8_t clients() const { return clients_; }
    bool connected() const { return client_ >= 0; }

private:
    static void text(const uint8_t* src, size_t n, char* out) { size_t i = 0; for (; i < n && src[i]; i++) out[i] = (char)src[i]; out[i] = 0; }
    static void nonblocking(int fd) { const int f = lwip_fcntl(fd, F_GETFL, 0); lwip_fcntl(fd, F_SETFL, f | O_NONBLOCK); }
    void open_listener() {
        const int fd = lwip_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (fd < 0) return;
        int one = 1; lwip_setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
        sockaddr_in a = {}; a.sin_family = AF_INET; a.sin_port = htons(port_); a.sin_addr.s_addr = htonl(INADDR_ANY);
        if (lwip_bind(fd, (sockaddr*)&a, sizeof(a)) != 0 || lwip_listen(fd, 1) != 0) { lwip_close(fd); return; }
        nonblocking(fd); listen_ = fd;
    }
    void drop() { if (client_ >= 0) { lwip_close(client_); client_ = -1; dropped_ = true; } }

    uint8_t mode_ = 0, state_ = 0, clients_ = 0; uint16_t port_ = 5760;
    int listen_ = -1, client_ = -1; bool dropped_ = false;
    uint32_t ip_ = 0, sta_ms_ = 0, listen_ms_ = 0, accepted_ = 0; int16_t rssi_ = 0;
};
#else
#define WB_HAS_WIFI 0
#endif
