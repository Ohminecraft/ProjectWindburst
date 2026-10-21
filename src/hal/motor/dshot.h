/*
 * This file is part of Windburst Project
 */

#pragma once
#include "../../config/fast_code.h"

#include <Arduino.h>
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"
#include "motor_hw.h"
#include "dshot_direct.h"

#include "driver/gptimer.h"
#include "driver/gpio.h"
#include "esp_private/rmt.h"            // rmt_get_channel_id()
#include "esp_timer.h"
#include "hal/rmt_ll.h"
#include "hal/gpio_ll.h"
#include "soc/rmt_struct.h"
#include "soc/gpio_struct.h"
#include "soc/rmt_periph.h"             // rmt_periph_signals: the GPIO-matrix signal of every RMT channel
#include "soc/gpio_periph.h"            // GPIO_PIN_MUX_REG
#include "soc/io_mux_reg.h"             // PIN_INPUT_ENABLE / DISABLE
#include "esp_rom_gpio.h"
#include "rom/ets_sys.h"
// The RMT symbol memory (8 channels x 48 words on the ESP32-S3: transmitters 0..3, receivers 4..7). The address comes from the
// SDK's linker script; the SDK only declares it in a private header, so it is declared here under our own name.
extern "C" { extern volatile uint32_t wb_rmt_mem[] __asm__("RMTMEM"); }

// ============================================================================
// DShot150/300/600/1200 and ProShot1000 on the ESP32-S3 RMT peripheral. The channels are created with the ESP-IDF driver (clock,
// channel numbers, pin routing), then the driver is left out of the loop: each cycle the symbols are written straight into the RMT
// memory and all transmitters are started back to back. No queue, no encoder, no interrupt per frame: about 10 us per cycle for
// 4 motors (the ESP-IDF rmt_transmit() path took ~130 us and could not listen; it was removed in 0.12). Bidirectional DShot: the
// pin is push-pull while the frame goes out, a timer interrupt right after the last bit turns it into an input and starts the
// receiver (see dshot_direct.h).
// One TX channel per motor (ESP32-S3: 4 TX + 4 RX => max 4 DShot motors; hexa/octo need a parallel-output backend).
// The legacy driver/rmt.h must NOT be mixed in (boot abort) - Arduino-ESP32 3.x itself uses the new driver.
//
// NOT yet confirmed on hardware: the DIRECT path, and with it bidirectional DShot.
// ============================================================================

// The hardware side of motor::DshotDirect on the ESP32-S3.
class RmtDirectPort {
public:
    static constexpr int kMaxCh = 4;
    // where a receive channel's memory block and signal sit in the group: the S3 has 4 transmit-only channels and then 4
    // receive-only ones (receive channel r = block r + 4); the plain ESP32's 8 channels each do both (r = block r). (ESP-IDF
    // rmt_private.h, RMT_RX_CHANNEL_OFFSET_IN_GROUP)
    static constexpr int kRxOffset = SOC_RMT_CHANNELS_PER_GROUP - SOC_RMT_RX_CANDIDATES_PER_GROUP;
    typedef void (*AlarmFn)(void*);

    bool begin(const uint8_t* pins, int count, bool bidir, motor::Protocol proto, AlarmFn fn, void* arg, bool pullup = true) {
        end();
        count_ = count; bidir_ = bidir; fn_ = fn; arg_ = arg; pullup_ = pullup;
        for (uint32_t hz : {80000000u, 40000000u}) {                     // finer ticks first (DShot1200 needs ~12 ns)
            if (create(pins, hz, proto)) return true;
            end();
        }
        return false;
    }
    void end() {
        if (timer_) { gptimer_stop(timer_); gptimer_disable(timer_); gptimer_del_timer(timer_); timer_ = nullptr; }
        for (int c = 0; c < kMaxCh; c++) {
            if (rx_[c]) {
                rmt_ll_rx_enable(&RMT, rx_id_[c], false); rmt_del_channel(rx_[c]); rx_[c] = nullptr;
                PIN_INPUT_DISABLE(iomux_[c]);
                gpio_ll_set_output_enable_ctrl(&GPIO, pin_[c], true, false);   // output enable back to the peripheral, as a one-way channel has it
            }
            if (tx_[c]) { rmt_del_channel(tx_[c]); tx_[c] = nullptr; }
        }
        count_ = 0;
    }

    uint32_t micros() const { return (uint32_t)esp_timer_get_time(); }
    void delay_us(uint32_t us) const { esp_rom_delay_us(us); }
    const char* failStep() const { return fail_step_; }
    esp_err_t failErr() const { return fail_err_; }
    uint32_t tick_hz() const { return tick_hz_; }
    WB_FAST void alarm_in(uint32_t us) {
        gptimer_set_raw_count(timer_, 0);
        gptimer_alarm_config_t a = {};
        a.alarm_count = us; a.flags.auto_reload_on_alarm = false;        // one shot: the hardware disarms it when it fires
        gptimer_set_alarm_action(timer_, &a);
    }
    // Talking: the transmitter's signal routed to the pad again (as esp-fc EscDriverEsp32::enableTx does), output on. Listening:
    // output off. The output enable is the GPIO_ENABLE bit (OEN_SEL = 1), so gpio_ll_output_disable() really lets go of the line.
    // The input path is NOT switched (0.18.2): listen() turns it on once and it stays on, so the receiver never sees the 0 -> 1
    // step of an input being switched on (an ESP32 pad reads back the level it drives; the receiver is stopped while we talk).
    // (Before 0.8.2 the transmitter's signal was routed once and only the output enable was switched: on the board the ESC then
    // took no frame at all - the routing per switch stays.)
    WB_FAST void drive(int c) {
        if (!bidir_) { gpio_ll_output_enable(&GPIO, pin_[c]); return; }          // one-way: as in 0.6 - 0.8.1, the path known to spin motors
        esp_rom_gpio_connect_out_signal(pin_[c], tx_sig_[c], false, false);
        gpio_ll_set_output_enable_ctrl(&GPIO, pin_[c], false, false);
        gpio_ll_output_enable(&GPIO, pin_[c]);
    }
    WB_FAST void release(int c) { gpio_ll_output_disable(&GPIO, pin_[c]); }
    // the receiver hears the pin from now on, also while we drive it
    void listen(int c) { PIN_INPUT_ENABLE(iomux_[c]); esp_rom_gpio_connect_in_signal(pin_[c], rx_sig_[c], false); }
    WB_FAST void tx_load(int c, const uint32_t* w, int n) {
        volatile uint32_t* m = wb_rmt_mem + tx_id_[c] * SOC_RMT_MEM_WORDS_PER_CHANNEL;
        for (int i = 0; i < n; i++) m[i] = w[i];
    }
    WB_FAST void tx_start(int c) { rmt_ll_tx_reset_pointer(&RMT, tx_id_[c]); rmt_ll_tx_start(&RMT, tx_id_[c]); }
    WB_FAST void rx_start(int c) {
        const int id = rx_id_[c];
        rmt_ll_rx_set_mem_owner(&RMT, id, RMT_LL_MEM_OWNER_HW);
        rmt_ll_rx_reset_pointer(&RMT, id);
        rmt_ll_rx_enable(&RMT, id, true);
    }
    WB_FAST int rx_stop(int c, uint32_t* w, int max) {
        const int id = rx_id_[c];
        rmt_ll_rx_enable(&RMT, id, false);
        int n = (int)rmt_ll_rx_get_memory_writer_offset(&RMT, id);
        if (n < 0) n = 0;
        if (n > max) n = max;
        rmt_ll_rx_set_mem_owner(&RMT, id, RMT_LL_MEM_OWNER_SW);
        const volatile uint32_t* m = wb_rmt_mem + (id + kRxOffset) * SOC_RMT_MEM_WORDS_PER_CHANNEL;
        for (int i = 0; i < n; i++) w[i] = m[i];
        rmt_ll_rx_set_mem_owner(&RMT, id, RMT_LL_MEM_OWNER_HW);
        return n;
    }

private:
    static bool IRAM_ATTR on_timer(gptimer_handle_t, const gptimer_alarm_event_data_t*, void* user) {
        RmtDirectPort* p = (RmtDirectPort*)user;
        p->fn_(p->arg_);
        return false;
    }

    bool create(const uint8_t* pins, uint32_t hz, motor::Protocol proto) {
        for (int c = 0; c < count_; c++) {
            if (pins[c] == 255) { fail_step_ = "motor pin not set"; fail_err_ = ESP_ERR_INVALID_ARG; return false; }
            pin_[c] = pins[c];
            if (bidir_) {                                                  // receiver first: it sets the pin up as an input with pull-up
                rmt_rx_channel_config_t rc = {};
                rc.gpio_num = (gpio_num_t)pins[c];
                rc.clk_src = RMT_CLK_SRC_DEFAULT;
                rc.resolution_hz = hz;
                rc.mem_block_symbols = SOC_RMT_MEM_WORDS_PER_CHANNEL;
                if ((fail_err_ = rmt_new_rx_channel(&rc, &rx_[c])) != ESP_OK) { fail_step_ = "rmt_new_rx_channel"; return false; }
                if ((fail_err_ = rmt_get_channel_id(rx_[c], &rx_id_[c])) != ESP_OK) { fail_step_ = "rmt_get_channel_id (rx)"; return false; }
            }
            rmt_tx_channel_config_t tc = {};
            tc.gpio_num = (gpio_num_t)pins[c];
            tc.clk_src = RMT_CLK_SRC_DEFAULT;
            tc.resolution_hz = hz;
            tc.mem_block_symbols = SOC_RMT_MEM_WORDS_PER_CHANNEL;
            tc.trans_queue_depth = 1;
            tc.flags.init_level = bidir_ ? 1 : 0;
            if ((fail_err_ = rmt_new_tx_channel(&tc, &tx_[c])) != ESP_OK) { fail_step_ = "rmt_new_tx_channel"; return false; }
            if ((fail_err_ = rmt_get_channel_id(tx_[c], &tx_id_[c])) != ESP_OK) { fail_step_ = "rmt_get_channel_id (tx)"; return false; }
        }
        uint32_t real = hz;
        rmt_get_channel_resolution(tx_[0], &real);
        tick_hz_ = real;
        // The channels are never enabled in the driver, so the driver never transmits, receives or takes an interrupt for them.
        // From here on the registers are ours.
        const uint32_t idle_ticks = (uint32_t)(((uint64_t)tick_hz_ * motor::reply_bit_ns(proto) * 8) / 1000000000ull);
        uint32_t filter_ticks = (uint32_t)((80000000ull * (motor::reply_bit_ns(proto) / 4)) / 1000000000ull);     // glitches shorter than a quarter bit; counted in 80 MHz source clocks
        if (filter_ticks > 255) filter_ticks = 255;
        for (int c = 0; c < count_; c++) {
            const int t = tx_id_[c];
            // cached in RAM: the flash tables cannot be read from the timer interrupt while the flash is busy
            iomux_[c] = GPIO_PIN_MUX_REG[pin_[c]];
            tx_sig_[c] = (uint16_t)rmt_periph_signals.groups[0].channels[t].tx_sig;
            rx_sig_[c] = bidir_ ? (uint16_t)rmt_periph_signals.groups[0].channels[rx_id_[c] + kRxOffset].rx_sig : 0;
            rmt_ll_enable_interrupt(&RMT, RMT_LL_EVENT_TX_MASK(t) | RMT_LL_EVENT_TX_ERROR(t), false);
            rmt_ll_tx_enable_loop(&RMT, t, false);
            rmt_ll_tx_fix_idle_level(&RMT, t, bidir_ ? 1 : 0, true);
#if !defined(CONFIG_IDF_TARGET_ESP32)
            RMT.chnconf0[t].conf_update_chn = 1;                           // idle level valid before the first frame (the plain ESP32 takes it at once)
#endif
            if (bidir_) {
                const int r = rx_id_[c];
                rmt_ll_enable_interrupt(&RMT, RMT_LL_EVENT_RX_MASK(r) | RMT_LL_EVENT_RX_ERROR(r), false);
                rmt_ll_rx_set_filter_thres(&RMT, r, filter_ticks);
                rmt_ll_rx_enable_filter(&RMT, r, true);
                rmt_ll_rx_set_idle_thres(&RMT, r, idle_ticks);
                rmt_ll_rx_set_mem_owner(&RMT, r, RMT_LL_MEM_OWNER_HW);
                rmt_ll_rx_enable(&RMT, r, false);
                if (pullup_) gpio_pullup_en((gpio_num_t)pin_[c]);           // the line idles high while the ESC has it
                else gpio_pullup_dis((gpio_num_t)pin_[c]);                  // (resistors on the board do that: motors.ext_pullup)
                drive(c);
            }
        }
        if (bidir_) {
            gptimer_config_t gc = {};
            gc.clk_src = GPTIMER_CLK_SRC_DEFAULT;
            gc.direction = GPTIMER_COUNT_UP;
            gc.resolution_hz = 1000000;
            gc.intr_priority = 3;                                          // ahead of UART / SPI / USB interrupts: the ESC answers 30 us after the frame
            if ((fail_err_ = gptimer_new_timer(&gc, &timer_)) != ESP_OK) { fail_step_ = "gptimer_new_timer"; return false; }
            gptimer_event_callbacks_t cb = {};
            cb.on_alarm = on_timer;
            if ((fail_err_ = gptimer_register_event_callbacks(timer_, &cb, this)) != ESP_OK) { fail_step_ = "gptimer_register_event_callbacks"; return false; }
            if ((fail_err_ = gptimer_enable(timer_)) != ESP_OK || (fail_err_ = gptimer_start(timer_)) != ESP_OK) { fail_step_ = "gptimer_enable / start"; return false; }
        }
        return true;
    }

    rmt_channel_handle_t tx_[kMaxCh] = {}, rx_[kMaxCh] = {};
    int tx_id_[kMaxCh] = {}, rx_id_[kMaxCh] = {};
    uint8_t pin_[kMaxCh] = {};
    uint32_t iomux_[kMaxCh] = {}; uint16_t tx_sig_[kMaxCh] = {}, rx_sig_[kMaxCh] = {};
    gptimer_handle_t timer_ = nullptr;
    AlarmFn fn_ = nullptr; void* arg_ = nullptr;
    int count_ = 0; bool bidir_ = false, pullup_ = true; uint32_t tick_hz_ = 80000000;
    const char* fail_step_ = ""; esp_err_t fail_err_ = ESP_OK;
};

class DshotHw : public MotorHw {
public:
    ~DshotHw() override { end(); }
    void set_pullup(bool internal) override { pullup_ = internal; }
    const char* name() const override {
        const char* base = proto_ == motor::PROTO_PROSHOT1000 ? "ProShot1000/RMT" : bidir_ ? "DShot/RMT, bidirectional" : "DShot/RMT";
        const char* note = note_[0] ? note_ : (proto_ == motor::PROTO_PROSHOT1000 && wanted_bidir_) ? "bidirectional needs DShot" : "";
        if (!*note) return base;
        snprintf(name_, sizeof(name_), "%s (%s)", base, note); return name_;
    }
    const char* note() const { return note_; }      // what the start-up check found / why bidirectional DShot is off
    WB_FAST bool bidirectional() const override { return bidir_; }
    motor::HwStats stats() const override {
        motor::HwStats s;
        const auto& d = direct_.stats();
        s.frames = d.frames; s.skipped = d.skipped; s.late_switch = d.late_switch; s.replies = d.replies; s.garbled = d.garbled;
        s.no_start = d.no_start; s.bad_bits = d.bad_bits;
        return s;
    }
    bool rx_responsive(int slot) const override {
        return direct_.responsive(slot);
    }

    bool begin(const uint8_t* pins, int count, motor::Protocol proto, uint16_t, bool bidirectional) override {
        end();
        if (!motor::is_dshot(proto) || count < 1 || count > kMaxCh) return false;
        proto_ = proto; count_ = count; wanted_bidir_ = bidirectional;
        note_[0] = 0;
        bidir_ = bidirectional && motor::is_dshot_wire(proto);
        if (bidir_) {
            // bidirectional: set up, then prove on the board that every pin carries the frames and is heard by its receiver.
            // Anything wrong: one-way DShot instead (the motors must still spin and beep), and the boot report says why.
            if (port_.begin(pins, count, true, proto, on_alarm, this, pullup_) && direct_.begin(port_, count, proto, true)) {
                for (int c = 0; c < count; c++) {
                    int words = 0; const uint16_t want = motor::dshot_packet(0, false, true);
                    const uint32_t seen = direct_.loopback(c, 0, words);
                    if (seen == want) continue;
                    if (seen == decltype(direct_)::kLoopNothing) snprintf(note_, sizeof(note_), "bidirectional OFF: start-up check, the receiver of motor %d heard nothing (%d words)", c + 1, words);
                    else if (seen == decltype(direct_)::kLoopGarbled) snprintf(note_, sizeof(note_), "bidirectional OFF: start-up check, motor %d: %d words, not a DShot frame", c + 1, words);
                    else snprintf(note_, sizeof(note_), "bidirectional OFF: start-up check, motor %d sent 0x%04x, heard 0x%04x", c + 1, (unsigned)want, (unsigned)seen);
                    break;
                }
                if (!note_[0]) { snprintf(note_, sizeof(note_), "start-up check OK on %d pin%s, %s pull-up", count, count == 1 ? "" : "s", pullup_ ? "internal" : "external"); return true; }
            } else snprintf(note_, sizeof(note_), "bidirectional OFF: %s failed (%s)", port_.failStep(), esp_err_to_name(port_.failErr()));
            port_.end(); bidir_ = false;
        }
        if (port_.begin(pins, count, false, proto, on_alarm, this) && direct_.begin(port_, count, proto, false)) return true;
        snprintf(note_, sizeof(note_), "%s failed (%s)", port_.failStep(), esp_err_to_name(port_.failErr()));
        port_.end();
        return false;
    }

    WB_FAST void write(const motor::Frame& f) override {
        if (!f.send) return;
        direct_.send(f.value, f.telem_request);
    }

    WB_FAST bool poll_reply(int slot, uint32_t& gcr20) override {
        if (bidir_) return direct_.take_reply(slot, gcr20);
        (void)slot; (void)gcr20; return false;
    }

private:
    static constexpr int kMaxCh = 4;                                     // ESP32-S3: 4 RMT transmitters

    static void IRAM_ATTR on_alarm(void* self) { ((DshotHw*)self)->direct_.on_alarm(); }
    void end() {
        port_.end();
    }

    RmtDirectPort port_;
    motor::DshotDirect<RmtDirectPort> direct_;
    motor::Protocol proto_ = motor::PROTO_DSHOT300;
    char note_[100] = {}; mutable char name_[140] = {};
    int count_ = 0; bool bidir_ = false, wanted_bidir_ = false, pullup_ = true;
};
