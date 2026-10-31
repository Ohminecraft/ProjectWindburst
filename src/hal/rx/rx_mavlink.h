/*
 * This file is part of Windburst Project
 */

#pragma once

#include "rx_proto.h"
#include "../../telemetry/mavlink.h"

// ============================================================================
// MAVLink receiver (portable): an ExpressLRS receiver in MAVLink mode (ELRS >= 3.5, docs/ELRS_MAVLINK.md). One UART carries the sticks
// and the telemetry both ways, so no second radio is needed. What the receiver sends (its SerialMavlink.cpp):
//   RC_CHANNELS_OVERRIDE  every 10 ms while the link is up: 16 channels in microseconds (system 255, component TELEMETRY_RADIO)
//   RADIO_STATUS          every 10 ms: rssi = link quality x 2.55, remrssi = uplink RSSI (dBm x -1), txbuf = % of its 1 kB buffer free
//   anything the ground station sends (forwarded): kept for mav::Gcs (telemetry/mavlink_gcs.h) in a small queue, take_gcs()
// As Betaflight's rx/mavlink.c: 460800 8N1; a channel value of 0 or 65535 leaves that channel as it is (MAVLink: "ignore this field");
// no check of target_system (the receiver is the only sender on this wire). Out-of-range values are clamped to 885 .. 2115 like every
// other protocol here. Link loss: the receiver stops sending channels, rx::Link times out (receiver.rx_timeout_ms).
// RADIO_STATUS is the flow control of the telemetry going the other way: FlightApp hands each txbuf to mav::Scheduler::radio_status().
// ============================================================================
namespace rx {

class MavlinkDecoder : public Decoder {
public:
    const char* name() const override { return "MAVLINK"; }
    uint32_t baud() const override { return 460800; }
    static constexpr int kGcsQueue = 4;
    void reset() override { parser_ = mav::Parser(); for (uint16_t& c : ch_) c = 1500; lq_ = 0; rssi_dbm_ = 0; txbuf_ = 100; statuses_ = 0; q_head_ = q_n_ = 0; }
    uint8_t link_quality() const override { return lq_; }

    bool feed(uint8_t c, uint32_t, Frame& out) override {
        const uint32_t bad = parser_.bad();
        const bool whole = parser_.feed(c);
        if (parser_.bad() != bad) errors_++;                          // (a damaged frame: bad CRC)
        if (!whole) return false;
        const mav::Parser::Frame& f = parser_.frame();
        mav::RcChannelsOverride o; mav::RadioStatus s;
        if (mav::unpack(f, o)) {
            for (int i = 0; i < kChannels; i++) if (o.chan[i] != 0 && o.chan[i] != 0xFFFF) ch_[i] = clamp_us((float)o.chan[i]);
            for (int i = 0; i < kChannels; i++) out.ch[i] = ch_[i];
            out.failsafe = false; out.frame_lost = false; frames_++;
            return true;
        }
        if (mav::unpack(f, s)) {
            lq_ = (uint8_t)((s.rssi * 100u + 127u) / 255u); rssi_dbm_ = -(int)s.remrssi; txbuf_ = s.txbuf; statuses_++;     // (rounded: LQ 99 -> 252 -> 99)
            return false;
        }
        if (q_n_ == kGcsQueue) { gcs_dropped_++; return false; }          // (the ground station asks again)
        queue_[(q_head_ + q_n_) % kGcsQueue] = f; q_n_++;
        return false;
    }
    bool radio_status_seen() const { return statuses_ != 0; }
    uint32_t radio_status_count() const { return statuses_; }        // RADIO_STATUS messages so far (the caller takes each new one)
    uint8_t txbuf() const { return txbuf_; }                          // the last one's: % of the receiver's buffer free
    int rssi_dbm() const { return rssi_dbm_; }
    // the next frame from the ground station (false: none waiting)
    bool take_gcs(mav::Parser::Frame& f) { if (!q_n_) return false; f = queue_[q_head_]; q_head_ = (q_head_ + 1) % kGcsQueue; q_n_--; return true; }
    uint32_t gcs_dropped() const { return gcs_dropped_; }
private:
    mav::Parser parser_;
    uint16_t ch_[kChannels] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
    uint8_t lq_ = 0, txbuf_ = 100; int rssi_dbm_ = 0; uint32_t statuses_ = 0;
    mav::Parser::Frame queue_[kGcsQueue]; int q_head_ = 0, q_n_ = 0; uint32_t gcs_dropped_ = 0;
};

} // namespace rx
