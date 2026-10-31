/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>
#include <string.h>
#include "../radio/nrf24.h"
#include "../../radio/wb_radio.h"

// ============================================================================
// The flight controller's nRF24 receiver: the receiving side of the Windburst link (src/radio/wb_radio.h, docs/TX.md).
// update() is called from the sensor thread (~1 kHz): it reads every packet waiting in the radio (one STATUS read when there is
// none), never waits. Control packets give 8 channels (9..16 = 1000); each one is answered, on the ACK of the next packet, by the
// telemetry set_telemetry() gave (armed, failsafe, flight mode, battery) and the link quality measured here.
//   bind      LISTEN on the bind address / channel (the first 60 s after power-up while never bound, or BIND from the app). The
//             first offer is answered by echoing its address in the ACK payload; the second offer carries that echo back to the
//             transmitter, so both move to the offered address / channel then (BINDING). The first control packet there = BOUND
//             (take_bound: the settings to store); none within 2 s (the transmitter missed the echo) = back to LISTEN.
//             An offer received weaker than -64 dBm (RPD 0) is ignored: binding is done with the transmitter next to the drone.
//   settings  a radio-settings packet (RF channel, rate, packet rate): its ACK went out when it arrived, so it moves at once; no
//             control packet there within 1 s = back to the old settings (the transmitter does the same). Never while armed.
//   control   taken only when bound (or confirming a bind): never bound = deaf to control packets, so two unbound sets can not
//             fly each other's drone on the default address.
//   health    every 100 ms the chip is asked whether it still holds its settings (a brown-out resets it to powered down);
//             if not, it is set up again (reinits()).
// ============================================================================
namespace rx {

class Nrf24Rx {
public:
    bool begin(nrf::Bus& bus, const nrf::Config& c, bool bound, uint32_t now_ms) {
        cfg_ = c; state_ = bound ? wbp::BIND_STATE_BOUND : wbp::BIND_STATE_IDLE; prev_ = state_;
        present_ = radio_.begin(bus, false, c);
        if (!present_) return false;
        if (!bound) start_bind(now_ms, kBootWindowMs, false);
        else ack_tel();
        return true;
    }
    bool present() const { return present_; }

    // every waiting packet; true when a new control packet was taken
    bool update(uint32_t /*now_us*/, uint32_t now_ms) {
        if (!present_) return false;
        timers(now_ms);
        if ((uint32_t)(now_ms - check_ms_) >= 100u) {
            check_ms_ = now_ms;
            if (!radio_.healthy() && radio_.reinit()) { if (state_ == wbp::BIND_STATE_LISTEN) listen_again(); else ack_tel(); }
        }
        bool got = false; uint8_t p[32], n;
        for (int k = 0; k < 8 && radio_.receive(p, n); k++) {
            if (state_ == wbp::BIND_STATE_LISTEN) { on_offer(p, n, now_ms); continue; }
            uint8_t seq; uint16_t ch[wbr::kChannels]; wbr::RadioCfg rc;
            if (wbr::decode_control(p, n, seq, ch)) {
                if (state_ != wbp::BIND_STATE_BOUND && state_ != wbp::BIND_STATE_BINDING) continue;     // never bound: deaf
                if (!seq_.accept(seq, now_ms, 250)) continue;
                for (int i = 0; i < 16; i++) ch_[i] = i < wbr::kChannels ? ch[i] : 1000;
                packets_++; win_count_++; got = true;
                if (trial_) { trial_ = false; changed_ = true; }               // new radio settings confirmed: to be kept
                if (state_ == wbp::BIND_STATE_BINDING) { state_ = prev_ = wbp::BIND_STATE_BOUND; bound_new_ = true; }
                ack_tel();
            } else if (state_ == wbp::BIND_STATE_BOUND && !tel_.armed && wbr::decode_config(p, n, rc)) {
                if (!trial_) old_ = cfg_;
                cfg_.rf_channel = rc.rf_channel; cfg_.rate = rc.rate; cfg_.pps = rc.pps; move(cfg_);
                trial_ = true; trial_until_ms_ = now_ms + 1000u;
            } else if (p[0] != wbr::T_BIND && p[0] != wbr::T_CONFIG) bad_++;       // (a stray offer / settings packet in the wrong state: not damage)
        }
        if ((uint32_t)(now_ms - win_ms_) >= 1000u) {
            const uint32_t want = nrf::pps_hz(cfg_.pps), q = (win_count_ * 100u + want / 2) / want;
            lq_ = (uint8_t)(q > 100 ? 100 : q); win_count_ = 0; win_ms_ = now_ms;
        }
        return got;
    }
    const uint16_t* channels() const { return ch_; }
    void set_telemetry(const wbr::Telemetry& t) { tel_ = t; }

    // BIND from the app (never while armed): listen for an offer for window_ms
    bool start_bind(uint32_t now_ms, uint32_t window_ms, bool armed) {
        if (armed || !present_) return false;
        if (state_ == wbp::BIND_STATE_BINDING) cfg_ = before_bind_;      // (an offer being confirmed is forgotten)
        if (state_ != wbp::BIND_STATE_LISTEN && state_ != wbp::BIND_STATE_BINDING) prev_ = state_;
        nrf::Config b = cfg_; memcpy(b.addr, wbr::kBindAddr, 5); b.rf_channel = wbr::kBindChannel; b.rate = wbp::NRF_RATE_R250K;
        radio_.configure(b); state_ = wbp::BIND_STATE_LISTEN; window_until_ms_ = now_ms + window_ms; have_offer_ = false;
        return true;
    }
    void stop_bind() {
        if (state_ != wbp::BIND_STATE_LISTEN && state_ != wbp::BIND_STATE_BINDING) return;
        if (state_ == wbp::BIND_STATE_BINDING) cfg_ = before_bind_;      // (the offered address was never confirmed)
        state_ = prev_; move(cfg_);
    }
    // a bind finished: the settings to store (true once)
    bool take_bound(nrf::Config& now) { if (!bound_new_) return false; bound_new_ = false; now = cfg_; return true; }
    // radio settings from the transmitter, confirmed by its packets on them: the settings to store (true once)
    bool take_changed(nrf::Config& now) { if (!changed_) return false; changed_ = false; now = cfg_; return true; }
    uint8_t bind_state() const { return state_; }
    uint8_t lq() const { return lq_; }
    uint32_t bad() const { return bad_; }
    uint32_t packets() const { return packets_; }
    uint32_t reinits() const { return radio_.reinits(); }
    const nrf::Config& config() const { return cfg_; }

    static constexpr uint32_t kBootWindowMs = 60000;

private:
    void timers(uint32_t now_ms) {
        if (state_ == wbp::BIND_STATE_LISTEN && (int32_t)(now_ms - window_until_ms_) >= 0) { state_ = prev_; move(cfg_); }
        if (state_ == wbp::BIND_STATE_BINDING && (int32_t)(now_ms - confirm_until_ms_) >= 0) {
            if ((int32_t)(now_ms - window_until_ms_) < 0) { state_ = wbp::BIND_STATE_LISTEN; cfg_ = before_bind_; listen_again(); }
            else { cfg_ = before_bind_; state_ = prev_; move(cfg_); }
        }
        if (trial_ && (int32_t)(now_ms - trial_until_ms_) >= 0) { trial_ = false; cfg_ = old_; move(cfg_); }
    }
    void on_offer(const uint8_t* p, uint8_t n, uint32_t now_ms) {
        wbr::BindOffer o; if (!wbr::decode_bind(p, n, o)) return;
        if (!radio_.rpd()) { far_offers_++; return; }                   // weaker than -64 dBm: not the transmitter next to it (someone else's bind)
        if (have_offer_ && !memcmp(o.addr, offer_, 5)) {            // the echo went out with the ACK of this packet: move
            before_bind_ = cfg_; memcpy(cfg_.addr, o.addr, 5); cfg_.rf_channel = o.rf_channel; cfg_.rate = o.rate; cfg_.pps = o.pps;
            move(cfg_); state_ = wbp::BIND_STATE_BINDING; confirm_until_ms_ = now_ms + 2000u; have_offer_ = false; return;
        }
        memcpy(offer_, o.addr, 5); have_offer_ = true;
        uint8_t b[32]; radio_.set_ack(b, wbr::encode_bind_ack(o.addr, b));
    }
    void listen_again() { nrf::Config b = cfg_; memcpy(b.addr, wbr::kBindAddr, 5); b.rf_channel = wbr::kBindChannel; b.rate = wbp::NRF_RATE_R250K; radio_.configure(b); have_offer_ = false; }
    void move(const nrf::Config& c) { radio_.configure(c); seq_.reset(); ack_tel(); }
    void ack_tel() { wbr::Telemetry t = tel_; t.lq = lq_; uint8_t b[32]; radio_.set_ack(b, wbr::encode_telemetry(t, b)); }

    nrf::Radio radio_; nrf::Config cfg_, old_, before_bind_; wbr::SeqFilter seq_; wbr::Telemetry tel_;
    uint16_t ch_[16] = {1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};
    uint8_t state_ = wbp::BIND_STATE_IDLE, prev_ = wbp::BIND_STATE_IDLE, offer_[5] = {}, lq_ = 0;
    bool present_ = false, have_offer_ = false, bound_new_ = false, trial_ = false, changed_ = false;
    uint32_t far_offers_ = 0, check_ms_ = 0, window_until_ms_ = 0, confirm_until_ms_ = 0, trial_until_ms_ = 0, win_ms_ = 0, win_count_ = 0, packets_ = 0, bad_ = 0;
};

} // namespace rx

#if defined(ARDUINO) && !defined(ARDUINO_ARCH_STM32)          // (STM32: not yet - its SPIClass takes pins, not a bus number)
#define WB_HAS_NRF24 1
#include <Arduino.h>
#include <SPI.h>
#include "../bus/bus_spi.h"
namespace nrf {
// The radio on a board: SPI (8 MHz, mode 0) on its own pins + CSN / CE as plain GPIOs. The flight controller puts it on the
// second SPI bus (SPI_DEVICE_1, the IMU has the first), the ESP32-C5 transmitter on its only one.
class ArduinoBus : public Bus {
public:
    explicit ArduinoBus(int bus) : spi_(bus) {}
    bool begin(uint8_t sck, uint8_t miso, uint8_t mosi, uint8_t cs, uint8_t ce) {
        if (sck == 255 || miso == 255 || mosi == 255 || cs == 255 || ce == 255) return false;
        cs_ = cs; ce_ = ce;
        pinMode(cs, OUTPUT); digitalWrite(cs, HIGH); pinMode(ce, OUTPUT); digitalWrite(ce, LOW);
        return spi_.begin((int8_t)sck, (int8_t)miso, (int8_t)mosi, -1);
    }
    void select(bool on) override {
        if (on) { spi_.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0)); digitalWrite(cs_, LOW); }
        else { digitalWrite(cs_, HIGH); spi_.endTransaction(); }
    }
    uint8_t xfer(uint8_t v) override { return spi_.transfer(v); }
    void ce(bool on) override { digitalWrite(ce_, on ? HIGH : LOW); }
    uint32_t micros() override { return ::micros(); }
    void wait_us(uint32_t us) override { delayMicroseconds(us); }
private:
    SPIClass spi_; uint8_t cs_ = 255, ce_ = 255;
};
} // namespace nrf
#else
#define WB_HAS_NRF24 0
#endif
