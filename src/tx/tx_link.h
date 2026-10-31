/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>
#include <string.h>
#include "../hal/radio/nrf24.h"
#include "../radio/wb_radio.h"

// ============================================================================
// The transmitter's side of the link (src/radio/wb_radio.h), one tick per packet period:
//   1. what became of the last packet (one STATUS read): ACKed or not, and what the ACK carried (telemetry, a bind ACK)
//   2. the next packet: a bind offer while binding, a radio-settings packet while one waits, else the channels
// Link = an ACK within the last 500 ms; ack_pct / pps over the last full second; the drone's telemetry is kept for 1 s.
// Never bound: nothing is sent (an address every unbound set shares would let them fly each other's drone).
// Time: milliseconds counted from the microsecond clock's steps, so the 71.6-minute wrap of micros() changes nothing.
// Every 100 ms the radio is asked whether it still holds its settings (a brown-out of the module); if not, it is set up again.
// Bind: start_bind() makes a new address and an RF channel (74 .. 82) from two random words and offers them (with the rate /
// packet rate of the settings) on the bind address and channel at the lowest power. A bind ACK echoing the address: move there
// and send control packets; the first one ACKed there = BOUND (an echo alone may have ridden on a copy the drone never acted
// on); none within 2 s = back to offering. Nobody answers for 30 s: FAILED, back to the old settings.
// No register is written while a packet is on its way: a change asked for then waits for the next tick (apply()).
// Radio settings (RF channel, rate, packet rate): sent on the old ones, both move after the ACK; while the drone says it is armed
// nothing is changed. A drone that does not answer on the new settings within 1 s: both go back (the receiver does the same).
// A change of the power alone is the transmitter's own business: applied at once.
// ============================================================================
namespace tx {

class TxLink {
public:
    struct Stats { bool link = false; uint8_t ack_pct = 0, pps = 0; };

    void begin(nrf::Radio& r, const wbp::S_tx_radio& s) {
        radio_ = &r; reset(s);
    }
    // the settings changed under the link (defaults, another pair): run with them now, bound or not as they say
    void reset(const wbp::S_tx_radio& s) {
        cfg_ = to_cfg(s); bound_ = s.bound != 0; bind_state_ = bound_ ? wbp::BIND_STATE_BOUND : wbp::BIND_STATE_IDLE;
        cfg_pending_ = false; trial_ = false; confirming_ = false; if (radio_) apply(cfg_);
    }
    // the milliseconds of this link (monotonic across the wrap of micros()); tick() advances it too
    uint32_t clock(uint32_t now_us) {
        if (!clk_started_) { clk_started_ = true; last_us_ = now_us; clk_ms_ = now_us / 1000u; us_rest_ = now_us % 1000u; return clk_ms_; }
        us_rest_ += now_us - last_us_; last_us_ = now_us; clk_ms_ += us_rest_ / 1000u; us_rest_ %= 1000u; return clk_ms_;
    }

    void tick(uint32_t now_us, const uint16_t ch_us[8]) {
        if (!radio_) return;
        const uint32_t now_ms = clock(now_us);
        if ((uint32_t)(now_ms - check_ms_) >= 100u) { check_ms_ = now_ms; if (!radio_->healthy()) { radio_->reinit(); in_flight_ = false; } }
        // 1. the last packet
        if (in_flight_) {
            uint8_t ack[32], an = 0;
            const nrf::Radio::TxResult r = radio_->poll_tx(ack, an);
            if (r == nrf::Radio::PENDING) { roll(now_ms); return; }      // (not resolved within a period: skip one)
            in_flight_ = false;
            if (r == nrf::Radio::ACKED) {
                acked_++; last_ack_ms_ = now_ms; any_ack_ = true;
                if (trial_ && sent_type_ == wbr::T_CONTROL) { trial_ = false; config_ok_ = true; }      // the drone answers on the new settings: done
                if (confirming_ && sent_type_ == wbr::T_CONTROL) { confirming_ = false; bind_state_ = wbp::BIND_STATE_BOUND; bound_ = bound_new_ = true; }   // bound: the drone answers there
                on_ack(ack, an, now_ms);
            }
        }
        roll(now_ms);
        if (deferred_) { deferred_ = false; radio_->configure(defer_); }    // (asked for while a packet was on its way)
        // a binding that nobody answers / a settings change the drone did not follow
        if (confirming_ && (int32_t)(now_ms - confirm_until_ms_) >= 0) { confirming_ = false; cfg_ = pre_bind_; radio_->configure(bind_cfg()); }   // the echo was not acted on: offer again
        if (bind_state_ == wbp::BIND_STATE_BINDING && (int32_t)(now_ms - bind_until_ms_) >= 0) { if (confirming_) { confirming_ = false; cfg_ = pre_bind_; } bind_state_ = wbp::BIND_STATE_FAILED; radio_->configure(cfg_); }
        if (trial_ && (int32_t)(now_ms - trial_until_ms_) >= 0) { cfg_ = old_cfg_; radio_->configure(cfg_); trial_ = false; config_failed_ = true; in_flight_ = false; }
        // 2. the next packet
        uint8_t p[32], n;
        if (bind_state_ == wbp::BIND_STATE_BINDING && !confirming_) n = wbr::encode_bind(offer_, p);
        else if (confirming_) n = wbr::encode_control(seq_++, ch_us, p);
        else if (cfg_pending_) { wbr::RadioCfg rc; rc.seq = ++cfg_seq_; rc.rf_channel = want_.rf_channel; rc.rate = want_.rate; rc.pps = want_.pps; n = wbr::encode_config(rc, p); }
        else if (bound_) n = wbr::encode_control(seq_++, ch_us, p);
        else return;                                                       // never bound: nothing to say to anyone
        if (radio_->send(p, n)) { in_flight_ = true; sent_++; sent_type_ = p[0]; }
    }

    void start_bind(uint32_t r0, uint32_t r1, uint32_t now_us) {
        if (!radio_) return;
        const uint32_t now_ms = clock(now_us);
        if (confirming_) { confirming_ = false; cfg_ = pre_bind_; }
        wbr::make_address(r0, r1, offer_.addr); offer_.rf_channel = (uint8_t)(wbr::kPairChannelLo + r1 % wbr::kPairChannels); offer_.rate = cfg_.rate; offer_.pps = cfg_.pps;
        apply(bind_cfg()); cfg_pending_ = false;
        prev_state_ = bind_state_ == wbp::BIND_STATE_BINDING ? prev_state_ : bind_state_;
        bind_state_ = wbp::BIND_STATE_BINDING; bind_until_ms_ = now_ms + 30000u;
    }
    void stop_bind() {
        if (bind_state_ != wbp::BIND_STATE_BINDING) return;
        if (confirming_) { confirming_ = false; cfg_ = pre_bind_; }
        bind_state_ = prev_state_; apply(cfg_);
    }
    // a bind finished since the last call: the new address (the settings take it)
    bool take_bound(uint8_t addr[5]) { if (!bound_new_) return false; bound_new_ = false; memcpy(addr, cfg_.addr, 5); return true; }

    // new radio settings from the app. The power is the transmitter's own: at once. RF channel / rate / packet rate need the
    // drone heard just now and disarmed (it has to follow); false = nothing changes
    bool request_config(const wbp::S_tx_radio& s, uint32_t now_ms) {
        const nrf::Config w = to_cfg(s);
        if (w.rf_channel == cfg_.rf_channel && w.rate == cfg_.rate && w.pps == cfg_.pps && !memcmp(w.addr, cfg_.addr, 5)) {   // the power (or nothing)
            cfg_.power = w.power; if (bind_state_ != wbp::BIND_STATE_BINDING) apply(cfg_); return true;
        }
        wbr::Telemetry t;
        if (!bound_ || !telemetry(t, now_ms) || t.armed) return false;
        want_ = w; cfg_pending_ = true; return true;
    }
    // the drone did not follow a settings change: both went back (true once)
    bool take_config_failed() { const bool f = config_failed_; config_failed_ = false; return f; }
    // the drone answered on the new settings: they are the pair's now (true once)
    bool take_config_ok() { const bool f = config_ok_; config_ok_ = false; return f; }
    bool bound() const { return bound_; }
    const nrf::Config& applied() const { return cfg_; }

    Stats stats() const { Stats s; s.link = any_ack_ && (uint32_t)(now_ms_ - last_ack_ms_) <= 500u; s.ack_pct = ack_pct_; s.pps = pps_; return s; }
    bool telemetry(wbr::Telemetry& t, uint32_t now_ms) const { if (!tel_ok_ || (uint32_t)(now_ms - tel_ms_) > 1000u) return false; t = tel_; return true; }
    uint8_t bind_state() const { return bind_state_; }
    bool radio_ok() const { return radio_ && radio_->present(); }

private:
    nrf::Config bind_cfg() const { nrf::Config b = cfg_; memcpy(b.addr, wbr::kBindAddr, 5); b.rf_channel = wbr::kBindChannel; b.rate = wbp::NRF_RATE_R250K; b.power = wbp::NRF_POWER_MIN; return b; }
    void apply(const nrf::Config& c) { if (in_flight_) { defer_ = c; deferred_ = true; } else radio_->configure(c); }
    static nrf::Config to_cfg(const wbp::S_tx_radio& s) { nrf::Config c; memcpy(c.addr, s.addr, 5); c.rf_channel = s.rf_channel; c.rate = s.rate; c.power = s.power; c.pps = s.pps; return c; }
    void on_ack(const uint8_t* a, uint8_t n, uint32_t now_ms) {
        uint8_t addr[5]; wbr::Telemetry t;
        if (bind_state_ == wbp::BIND_STATE_BINDING && !confirming_) {
            if (wbr::decode_bind_ack(a, n, addr) && !memcmp(addr, offer_.addr, 5)) {
                if (!confirming_) { pre_bind_ = cfg_; memcpy(cfg_.addr, offer_.addr, 5); cfg_.rf_channel = offer_.rf_channel; radio_->configure(cfg_); confirming_ = true; confirm_until_ms_ = now_ms + 2000u; }
            }
            return;
        }
        if (cfg_pending_ && sent_type_ == wbr::T_CONFIG) {                     // the drone has the new settings: move
            old_cfg_ = cfg_; cfg_.rf_channel = want_.rf_channel; cfg_.rate = want_.rate; cfg_.pps = want_.pps; cfg_.power = want_.power;
            radio_->configure(cfg_); cfg_pending_ = false; trial_ = true; trial_until_ms_ = now_ms + 1000u;
        }
        if (wbr::decode_telemetry(a, n, t)) { tel_ = t; tel_ms_ = now_ms; tel_ok_ = true; }
    }
    // the per-second counts
    void roll(uint32_t now_ms) {
        now_ms_ = now_ms;
        if (!win_started_) { win_started_ = true; win_ms_ = now_ms; return; }
        if ((uint32_t)(now_ms - win_ms_) < 1000u) return;
        pps_ = (uint8_t)(sent_ > 255 ? 255 : sent_); ack_pct_ = (uint8_t)(sent_ ? (acked_ * 100u + sent_ / 2) / sent_ : 0);
        if (ack_pct_ > 100) ack_pct_ = 100;
        sent_ = acked_ = 0; win_ms_ = now_ms;
    }

    nrf::Radio* radio_ = nullptr; nrf::Config cfg_, want_, old_cfg_; wbr::BindOffer offer_;
    uint8_t bind_state_ = wbp::BIND_STATE_IDLE, prev_state_ = wbp::BIND_STATE_IDLE, seq_ = 0, cfg_seq_ = 0, sent_type_ = 0;
    bool in_flight_ = false, any_ack_ = false, bound_new_ = false, cfg_pending_ = false, trial_ = false, config_failed_ = false, config_ok_ = false, tel_ok_ = false, win_started_ = false;
    bool bound_ = false, clk_started_ = false, confirming_ = false, deferred_ = false; nrf::Config pre_bind_, defer_; uint32_t confirm_until_ms_ = 0; uint32_t last_us_ = 0, us_rest_ = 0, clk_ms_ = 0, check_ms_ = 0;
    uint32_t bind_until_ms_ = 0, last_ack_ms_ = 0, tel_ms_ = 0, trial_until_ms_ = 0, now_ms_ = 0, win_ms_ = 0, sent_ = 0, acked_ = 0;
    uint8_t pps_ = 0, ack_pct_ = 0; wbr::Telemetry tel_;
};

} // namespace tx
