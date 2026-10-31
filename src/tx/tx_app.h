/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>
#include <string.h>
#include "../proto/wbp_server.h"
#include "sticks.h"
#include "tx_link.h"

// ============================================================================
// The DIY transmitter (docs/TX.md): sticks -> channels -> the nRF24 link, and the WBP host the app talks to (role TX: sections
// tx_radio / tx_sticks, stream tx, commands BIND / STICK_CAL). Portable: the ESP32-C5 end (src/main_tx.cpp) reads the ADC, gives
// the radio its bus and calls cycle() once per packet period; the PC tests and wb_sim do the same with simulated parts.
// ============================================================================
namespace tx {

class TxApp : public wbp::Host {
public:
    void configure(wbp::Settings& s) { s_ = &s; sticks_.configure(s.tx_sticks); memcpy(boot_pins_, &s.tx_radio.pin_sck, 5); }
    bool begin(nrf::Bus& bus) {
        nrf::Config c; memcpy(c.addr, s_->tx_radio.addr, 5); c.rf_channel = s_->tx_radio.rf_channel; c.rate = s_->tx_radio.rate; c.power = s_->tx_radio.power; c.pps = s_->tx_radio.pps;
        const bool ok = radio_.begin(bus, true, c); link_.begin(radio_, s_->tx_radio); return ok;
    }
    void set_random(uint32_t (*fn)()) { rand_ = fn; }
    // the boot report: lines written at start-up (radio found, pins), sent to the app as log frames on REPORT
    void set_server(wbp::Server* s) { srv_ = s; }
    void add_report(const char* line) { if (n_report_ < kReportLines) { strncpy(report_[n_report_], line, kReportLen - 1); report_[n_report_][kReportLen - 1] = 0; n_report_++; } }
    int report_lines() const { return n_report_; }
    // the packet period of the settings the link runs with now
    uint32_t period_us() const { return 1000000u / nrf::pps_hz(link_.applied().pps); }

    // once per packet period
    void cycle(uint32_t now_us, const StickInput& in) {
        last_us_ = now_us; now_ms_ = link_.clock(now_us); raw_ = in;
        sticks_.update(in, drone(), now_ms_, ch_);
        link_.tick(now_us, ch_);
        uint8_t a[5];
        if (link_.take_bound(a)) { memcpy(s_->tx_radio.addr, a, 5); s_->tx_radio.rf_channel = link_.applied().rf_channel; s_->tx_radio.bound = 1; autosave_ = wbp::SEC_TX_RADIO; }
        // radio settings: what the pair runs is what the flash keeps - a change the drone followed, or the old ones it went back to
        if (link_.take_config_ok()) autosave_ = wbp::SEC_TX_RADIO;
        if (link_.take_config_failed()) { show_applied(); changed_ = true; autosave_ = wbp::SEC_TX_RADIO; }
    }
    // what the transmitter knows about its drone: armed / disarmed from fresh telemetry; heard armed and not since = UNKNOWN
    // (it may be flying: the throttle is held, settings and restarts wait); never heard armed = DISARMED
    uint8_t drone() {
        wbr::Telemetry t;
        if (link_.telemetry(t, now_ms_)) { heard_armed_ = t.armed; return t.armed ? ARMED : DISARMED; }
        return heard_armed_ ? UNKNOWN : DISARMED;
    }

    // ---- wbp::Host
    bool armed() override { return drone() != DISARMED; }   // its drone flies (or may): the server refuses settings, defaults and restarts
    uint32_t now_ms() override { return now_ms_; }
    uint16_t fill_stream(uint8_t id, uint8_t* out, uint16_t cap) override {
        if (id != wbp::STR_TX || cap < sizeof(wbp::T_tx)) return 0;
        wbp::T_tx t; memset(&t, 0, sizeof(t));
        for (int i = 0; i < 8; i++) t.ch[i] = ch_[i];
        for (int i = 0; i < 4; i++) t.raw[i] = raw_.raw[i];
        const TxLink::Stats st = link_.stats(); t.link = st.link; t.ack_pct = st.ack_pct; t.pps = st.pps; t.bind_state = link_.bind_state(); t.calibrating = sticks_.calibrating();
        t.settings_dirty = srv_ ? srv_->flags() : 0;            // (the flight controller sends it in its status stream; this one has none)
        wbr::Telemetry d; if (link_.telemetry(d, now_ms_)) { t.d_seen = 1; t.d_armed = d.armed; t.d_failsafe = d.failsafe; t.d_mode = d.mode; t.d_batt_cv = d.batt_cv; t.d_lq = d.lq; }
        memcpy(out, &t, sizeof(t)); return sizeof(t);
    }
    wbp::Err command(uint8_t cmd, const uint8_t* a, uint16_t n, uint8_t* res, uint16_t& rlen) override {
        rlen = 0;
        switch (cmd) {
        case wbp::CMD_REPORT: if (srv_) for (int i = 0; i < n_report_; i++) srv_->log(1, report_[i]); return wbp::ERR_OK;
        case wbp::CMD_BIND: {
            if (n < 1) return wbp::ERR_BAD_ARG;
            if (a[0] == 0) { link_.stop_bind(); return wbp::ERR_OK; }
            if (armed()) return wbp::ERR_ARMED;
            const uint32_t r0 = random(), r1 = random(); link_.start_bind(r0, r1, last_us_); return wbp::ERR_OK; }
        case wbp::CMD_STICK_CAL: {
            if (n < 1) return wbp::ERR_BAD_ARG;
            if (a[0] == 0) { sticks_.cal_cancel(); return wbp::ERR_OK; }
            if (armed()) return wbp::ERR_ARMED;                  // (calibrating moves every stick to its ends)
            if (a[0] == 1) { sticks_.cal_start(); return wbp::ERR_OK; }
            const int axes = sticks_.cal_finish(s_->tx_sticks);
            res[0] = (uint8_t)axes; rlen = 1;
            if (!axes) return wbp::ERR_FAILED;
            autosave_ = wbp::SEC_TX_STICKS; return wbp::ERR_OK; }
        default: return wbp::ERR_NOT_SUPPORTED;
        }
    }
    bool apply_live(uint8_t sec) override {
        if (sec == wbp::SEC_TX_STICKS) { sticks_.configure(s_->tx_sticks); return true; }
        if (sec == wbp::SEC_TX_RADIO) {
            if (memcmp(boot_pins_, &s_->tx_radio.pin_sck, 5)) return false;        // other pins: after a reboot
            const wbp::S_tx_radio& r = s_->tx_radio;
            if (memcmp(r.addr, link_.applied().addr, 5) || (r.bound != 0) != link_.bound()) { link_.reset(r); return true; }   // another pair / defaults
            if (!link_.request_config(r, now_ms_)) show_applied();  // the drone not there, or armed: the values go back at once
            return true;
        }
        return false;
    }
    bool take_settings_changed() override { const bool c = changed_; changed_ = false; return c; }
    uint8_t take_autosave() override { const uint8_t a = autosave_; autosave_ = 0; return a; }
    void reboot() override { reboot_ = true; }
    bool reboot_requested() const { return reboot_; }

    const TxLink& link() const { return link_; }
    const Sticks& sticks() const { return sticks_; }
    const uint16_t* channels() const { return ch_; }

private:
    void show_applied() { const nrf::Config& c = link_.applied(); s_->tx_radio.rf_channel = c.rf_channel; s_->tx_radio.rate = c.rate; s_->tx_radio.pps = c.pps; s_->tx_radio.power = c.power; }
    uint32_t random() { if (rand_) return rand_(); seed_ ^= seed_ << 13; seed_ ^= seed_ >> 17; seed_ ^= seed_ << 5; return seed_ ^ now_ms_; }   // (the board gives the hardware RNG)
    static constexpr int kReportLines = 8, kReportLen = 120;
    wbp::Server* srv_ = nullptr; char report_[kReportLines][kReportLen] = {}; int n_report_ = 0;
    wbp::Settings* s_ = nullptr; nrf::Radio radio_; TxLink link_; Sticks sticks_; StickInput raw_;
    uint16_t ch_[8] = {1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000};
    uint32_t now_ms_ = 0, last_us_ = 0, seed_ = 2463534242u; bool heard_armed_ = false; uint32_t (*rand_)() = nullptr;
    uint8_t autosave_ = 0, boot_pins_[5] = {}; bool changed_ = false, reboot_ = false;
};

} // namespace tx
