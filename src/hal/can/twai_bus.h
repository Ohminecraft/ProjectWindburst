/*
 * This file is part of Windburst Project
 */

#pragma once

#include "can_port.h"
#include "../../port/platform.h"
#if WB_PLATFORM_ESP32
#include <soc/soc_caps.h>
#endif

// ============================================================================
// The ESP32's own CAN controller (TWAI, ESP-IDF's driver) with an external 3.3 V transceiver (SN65HVD230, TJA1051T/3 ...):
// two pins, TX to the transceiver's D input, RX from its R output. Extended (29-bit) frames only - DroneCAN uses nothing else.
// Never blocks: send() puts the frame in the driver's queue or reports it full, receive() takes what is waiting.
// A bus-off (a shorted bus, a missing terminator, the wrong speed) is recovered by itself; the count is reported.
// The driver's interrupt runs on the core that called start() (the setup's, core 1 - beside the flight task: a few us per frame).
// NOT yet confirmed on hardware.
// ============================================================================
#if WB_PLATFORM_ESP32 && defined(SOC_TWAI_SUPPORTED) && SOC_TWAI_SUPPORTED
#define WB_HAS_TWAI 1
#include <driver/twai.h>
#include <string.h>

class TwaiBus : public CanPort {
public:
    bool start(uint8_t tx_pin, uint8_t rx_pin, uint32_t bitrate) override {
        twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)tx_pin, (gpio_num_t)rx_pin, TWAI_MODE_NORMAL);
        g.tx_queue_len = 32; g.rx_queue_len = 64;
        twai_timing_config_t t;
        switch (bitrate) {
        case 125000: t = TWAI_TIMING_CONFIG_125KBITS(); break;
        case 250000: t = TWAI_TIMING_CONFIG_250KBITS(); break;
        case 500000: t = TWAI_TIMING_CONFIG_500KBITS(); break;
        default:     t = TWAI_TIMING_CONFIG_1MBITS(); break;
        }
        const twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
        if (twai_driver_install(&g, &t, &f) != ESP_OK) { why_ = "the TWAI driver did not install (pins?)"; state_ = FAILED; return false; }
        if (twai_start() != ESP_OK) { why_ = "the TWAI controller did not start"; state_ = FAILED; return false; }
        state_ = RUNNING; return true;
    }
    const char* why() const override { return why_; }
    bool send(const dcan::Frame& fr) override {
        if (state_ != RUNNING) return false;
        twai_message_t m; memset(&m, 0, sizeof(m)); m.extd = 1; m.identifier = fr.id & 0x1FFFFFFFu; m.data_length_code = fr.len; memcpy(m.data, fr.data, fr.len);
        return twai_transmit(&m, 0) == ESP_OK;
    }
    bool receive(dcan::Frame& fr) override {
        if (state_ == FAILED || state_ == OFF) return false;
        twai_message_t m;
        while (twai_receive(&m, 0) == ESP_OK) {
            if (!m.extd || m.rtr || m.data_length_code > 8) continue;                    // (not DroneCAN)
            fr.id = m.identifier; fr.len = m.data_length_code; memcpy(fr.data, m.data, fr.len);
            return true;
        }
        return false;
    }
    void service(uint32_t now_ms) override {
        if (state_ == FAILED || state_ == OFF || (uint32_t)(now_ms - check_ms_) < 20u) return;
        check_ms_ = now_ms;
        twai_status_info_t st; if (twai_get_status_info(&st) != ESP_OK) return;
        tec_ = st.tx_error_counter > 255 ? 255 : (uint8_t)st.tx_error_counter; rec_ = st.rx_error_counter > 255 ? 255 : (uint8_t)st.rx_error_counter;
        if (st.state == TWAI_STATE_BUS_OFF) { if (state_ != BUS_OFF) { bus_offs_++; state_ = BUS_OFF; twai_initiate_recovery(); } }
        else if (st.state == TWAI_STATE_STOPPED && state_ == BUS_OFF) { if (twai_start() == ESP_OK) state_ = RUNNING; }      // recovered: start again
        else if (st.state == TWAI_STATE_RUNNING) state_ = RUNNING;
    }
    uint8_t state() const override { return state_; }
    uint8_t tec() const override { return tec_; }
    uint8_t rec() const override { return rec_; }
    uint32_t bus_offs() const override { return bus_offs_; }
private:
    const char* why_ = ""; uint8_t state_ = OFF, tec_ = 0, rec_ = 0; uint32_t bus_offs_ = 0, check_ms_ = 0;
};
#else
#define WB_HAS_TWAI 0
#endif
