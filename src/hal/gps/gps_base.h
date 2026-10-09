/*
 * This file is part of Windburst Project
 */

#pragma once

#include <Arduino.h>
#include <HardwareSerial.h>
#include "gps_engine.h"
#include "gps_nav.h"
#include "../../config/hardware_config.h"

// ============================================================================
// GpsSensor: UART GPS (u-blox UBX / NMEA) or injected data (MSP, virtual), with Betaflight-style auto baud, auto config and home point.
//
//   GpsSensor gps;
//   gps.config().baud = 57600;                 // set provider / baud / rate / sbas / models ... (gps::Config, see gps_types.h)
//   gps.begin(Serial1);                        // pins from hardware_config.h getGpsPins(), or gps.setPins(rx, tx) before begin()
//   loop (>= 100 Hz is plenty, it only drains the UART):
//        gps.update(millis(), armed);
//        if (gps.newSolution()) { ... gps.solution() ... }
//   arm checks:  arm.gps_fix = gps.usable();   // healthy link, 3D fix, enough satellites, sane accuracy
//
// The UART runs with a 1 KB RX buffer so a burst of NAV-PVT + NAV-SAT at 115200 baud is never lost while the flight loop is busy.
// ============================================================================
class ArduinoGpsPort : public gps::Port {
public:
    void attach(HardwareSerial& s, int rx, int tx) { s_ = &s; rx_ = rx; tx_ = tx; started_ = false; }
    int read() override { return s_ && s_->available() ? s_->read() : -1; }
    void write(const uint8_t* d, size_t n) override { if (s_) s_->write(d, n); }
    void set_baud(uint32_t baud) override {
        if (!s_) return;
        if (!started_) { s_->setRxBufferSize(1024); s_->begin(baud, SERIAL_8N1, rx_, tx_); started_ = true; }
        else { s_->flush(); s_->updateBaudRate(baud); }
        while (s_->available()) s_->read();                      // bytes received at the old rate are garbage now
    }
    bool tx_empty() override { return true; }                    // our frames are < 100 bytes and the TX ring is 256: write() never blocks
private:
    HardwareSerial* s_ = nullptr; int rx_ = -1, tx_ = -1; bool started_ = false;
};

class GpsSensor {
public:
    gps::Config& config() { return eng_.cfg; }
    void setPins(uint8_t rx, uint8_t tx) { rx_ = rx; tx_ = tx; pins_set_ = true; }
    void setEnabled(bool on) { enabled_ = on; }                  // app switch "GPS on/off"
    // limits for usable(): BF requires fix + min sats; accuracy limits protect Position Hold / Return-To-Home
    void setLimits(uint8_t min_sats, uint16_t max_hdop_x100, uint32_t max_hacc_mm) { min_sats_ = min_sats; max_hdop_ = max_hdop_x100; max_hacc_ = max_hacc_mm; }

    bool begin(HardwareSerial& uart) {
        fail_ = ""; started_ = false;
        if (!enabled_) { fail_ = "GPS disabled"; return false; }
        const gps::Provider p = eng_.cfg.provider;
        if (p == gps::PROV_NMEA || p == gps::PROV_UBLOX) {
            uint8_t pins[3];
            if (pins_set_) { pins[0] = rx_; pins[1] = tx_; pins[2] = 1; } else hw_.getGpsPins(pins);
            if (pins[0] == GPIO_UNUSED || pins[1] == GPIO_UNUSED) { fail_ = "GPS UART pins not set (app: Pins tab, or hardware_config.h)"; return false; }
            port_.attach(uart, pins[0], pins[1]);
        }
        home_.configure(eng_.cfg);
        eng_.begin(port_, millis());
        started_ = true;
        return true;
    }
    // Returns true when a NEW complete solution was received during this call.
    bool update(uint32_t now_ms, bool armed) {
        if (!started_) return false;
        eng_.set_armed(armed);
        fresh_ = eng_.update(now_ms);
        home_.update(eng_.solution(), armed);
        return fresh_;
    }
    bool newSolution() const { return fresh_; }
    // MSP / virtual provider: feed solutions from elsewhere.
    void inject(const gps::Solution& s, uint32_t now_ms) { eng_.inject(s, now_ms); }

    bool healthy() const { return started_ && eng_.healthy(); }
    // everything a navigation mode needs to trust the GPS
    bool usable() const {
        const gps::Solution& s = eng_.solution();
        return healthy() && s.fix && s.num_sat >= min_sats_ && s.hdop <= max_hdop_ && (s.h_acc_mm == 0 || s.h_acc_mm <= max_hacc_);
    }
    const gps::Solution& solution() const { return eng_.solution(); }
    gps::State state() const { return eng_.state(); }
    const char* stateName() const {
        static const char* n[] = {"unknown", "initialized", "detect baud", "change baud", "configure", "receiving", "lost"}; return n[(int)eng_.state()];
    }
    const char* module() const { return gps::ubx::version_name(eng_.version()); }
    uint32_t baud() const { return eng_.baud(); }
    float rateHz() const { return eng_.rate_hz(); }
    const gps::SatInfo* sats() const { return eng_.sats(); }
    int satCount() const { return eng_.sat_count(); }
    uint32_t timeouts() const { return eng_.timeouts(); }
    uint32_t frameErrors() const { return eng_.errors(); }
    uint32_t configNacks() const { return eng_.config_nacks(); }
    gps::Home& home() { return home_; }
    const char* failReason() const { return fail_; }
    gps::Engine& engine() { return eng_; }

private:
    gps::Engine eng_;
    gps::Home home_;
    ArduinoGpsPort port_;
    HardwareConfig hw_;
    uint8_t rx_ = GPIO_UNUSED, tx_ = GPIO_UNUSED, min_sats_ = 8; uint16_t max_hdop_ = 300; uint32_t max_hacc_ = 5000;
    bool pins_set_ = false, enabled_ = true, started_ = false, fresh_ = false;
    const char* fail_ = "";
};
