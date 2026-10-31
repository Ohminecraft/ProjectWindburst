/*
 * This file is part of Windburst Project
 */

#pragma once

#include <Arduino.h>
#include "../../port/uart.h"
#include "../../port/gpio.h"
#include "rx_proto.h"
#include "rx_pulse.h"
#include "../../telemetry/crsf_telemetry.h"
#include "rx_nrf24.h"
#include "rx_mavlink.h"
#include "../../config/fast_code.h"

// ============================================================================
// RcReceiver: the RC receiver.
//   serial (CRSF / SBUS / IBUS / Spektrum / MAVLink) on a hardware UART, or
//   pulses on GPIO pins: PPM (one pin) or PWM (one pin per channel, up to 8), timed by a GPIO interrupt.
//
//   RcReceiver rc;
//   rc.begin(rx::PROTO_CRSF, Serial2, rxPin, txPin);        // false + failReason() if the protocol is not available / no pin
//   rc.beginPulse(rx::PROTO_PPM, &ppmPin, 1);              // or PWM: rc.beginPulse(rx::PROTO_PWM, pwmPins, 8)
//   loop (~1 kHz):  rc.update(micros(), millis());
//                   app.set_rc(rc.channels(), rc.linkOk(millis()));
//
// SBUS is an inverted signal: the ESP32 UART inverts it in hardware (no external inverter needed).
// The pulse interrupt only stores the time of the edge (rx::EdgeRing); update() decodes. An interrupt that comes late (another
// interrupt, a flash write with the cache off) only shifts a pulse by microseconds, or loses a frame that is then skipped.
//   or the nRF24 radio of the Windburst DIY transmitter on the second SPI bus (beginNrf24, rx_nrf24.h, docs/TX.md).
//
// Serial: a 2 kB receive buffer (48 ms of CRSF at full speed, so a sensor thread that is held up does not lose bytes), the
// hardware FIFO handed to it every 64 bytes (the driver's default at these rates is 120 of 128: 190 us of slack for its
// interrupt), and read in blocks of 128 bytes, up to 2 kB per update() (0.8.7: one read() per byte was what fell behind). What still goes wrong is counted: uartOverflows() (bytes the driver had no room
// for - the firmware did not read in time) and uartLineErrors() (framing / parity errors - noise on the wire, wrong baud),
// next to frameErrors() (frames the decoder threw away). The Receiver tab shows all three per second.
// ============================================================================
class RcReceiver {
public:
    static constexpr int kMaxPulsePins = rx::PwmDecoder::kMaxInputs;
    void setTimeoutMs(uint16_t ms) { link_.timeout_ms = ms; }

    bool begin(rx::Protocol proto, port::SerialPort& uart, uint8_t rx_pin, uint8_t tx_pin) {
        stop_pulses(); dec_ = nullptr; kind_ = K_NONE; fail_ = ""; tx_ok_ = false;
        switch (proto) {
        case rx::PROTO_CRSF: dec_ = &crsf_; break;
        case rx::PROTO_SBUS: dec_ = &sbus_; break;
        case rx::PROTO_IBUS: dec_ = &ibus_; break;
        case rx::PROTO_SPEKTRUM: case rx::PROTO_SPEKTRUM1024: spek_.setResolution(proto == rx::PROTO_SPEKTRUM); dec_ = &spek_; break;
        case rx::PROTO_MAVLINK: dec_ = &mav_; break;
        case rx::PROTO_PPM: case rx::PROTO_PWM: fail_ = "PPM / PWM receivers are started with beginPulse()"; return false;
        default: fail_ = "receiver protocol not implemented yet (available: CRSF, SBUS, IBUS, SPEKTRUM, MAVLINK, PPM, PWM)"; return false;
        }
        if (rx_pin == 255) { dec_ = nullptr; fail_ = "receiver RX pin not set (app: Pins tab, or hardware_config.h)"; return false; }
        uart_ = &uart;
#if !WB_PLATFORM_STM32
        if (proto == rx::PROTO_MAVLINK) uart.setTxBufferSize(1024);  // (the MAVLink telemetry goes out here: frames wait there, the sensor thread never does)
#endif
        // (1 ms of CRSF at 420 kbaud is 42 bytes; the buffer leaves room for a stalled loop)
        if (!port::uart_begin(uart, dec_->baud(), dec_->even_parity_2stop() ? SERIAL_8E2 : SERIAL_8N1, rx_pin, tx_pin, dec_->inverted(), kRxBuffer)) {
            dec_ = nullptr; fail_ = "receiver: the UART did not start on these pins (STM32: RX and TX must belong to the same UART)"; return false;
        }
        g_uart_ovf = 0; g_uart_line = 0;
#if !WB_PLATFORM_STM32
        uart.setRxFIFOFull(64);
        uart.onReceiveError([](hardwareSerial_error_t e) {        // (Arduino's serial event task: core 0, low priority - platformio.ini)
            if (e == UART_BUFFER_FULL_ERROR || e == UART_FIFO_OVF_ERROR) g_uart_ovf = g_uart_ovf + 1;
            else if (e == UART_FRAME_ERROR || e == UART_PARITY_ERROR) g_uart_line = g_uart_line + 1;
        });
#endif
        dec_->reset();
        kind_ = K_SERIAL; tx_ok_ = tx_pin != 255; proto_ = proto;
        return true;
    }

    // PPM: pins[0]. PWM: pins[0..n-1], one per channel (255 = that channel is not wired; at least the first one must be).
    bool beginPulse(rx::Protocol proto, const uint8_t* pins, int n) {
        stop_pulses(); dec_ = nullptr; kind_ = K_NONE; fail_ = "";
        if (proto != rx::PROTO_PPM && proto != rx::PROTO_PWM) { fail_ = "not a pulse receiver protocol"; return false; }
        if (n > kMaxPulsePins) n = kMaxPulsePins;
        if (proto == rx::PROTO_PPM) n = 1;
        uint8_t wired = 0;
        for (int i = 0; i < n; i++) if (pins[i] != 255) wired |= (uint8_t)(1u << i);
        if (!wired || pins[0] == 255) {
            fail_ = proto == rx::PROTO_PPM ? "PPM pin not set (app: Pins tab, ppm)" : "PWM receiver: the pin of channel 1 is not set (app: Pins tab, receiver PWM)";
            return false;
        }
        ring_.clear();
        for (int i = 0; i < n; i++) {
            pins_[i] = pins[i];
            if (pins[i] == 255) continue;
            ctx_[i] = Ctx{this, (uint8_t)i, pins[i]};
            pinMode(port::apin(pins[i]), INPUT);
#if WB_PLATFORM_STM32
            { Ctx* c = &ctx_[i]; attachInterrupt(port::apin(pins[i]), [c]() { on_edge(c); }, proto == rx::PROTO_PPM ? RISING : CHANGE); }
#else
            attachInterruptArg(pins[i], &RcReceiver::on_edge, &ctx_[i], proto == rx::PROTO_PPM ? RISING : CHANGE);
#endif
        }
        n_pins_ = n;
        if (proto == rx::PROTO_PPM) { ppm_.reset(); kind_ = K_PPM; } else { pwm_.reset(wired); kind_ = K_PWM; }
        return true;
    }

    // nRF24: pins = SCK, MISO, MOSI, CSN, CE on the second SPI bus; c = the pair's address / RF channel (receiver.nrf_*)
    bool beginNrf24(const uint8_t pins[5], const nrf::Config& c, bool bound, uint32_t now_ms) {
        stop_pulses(); dec_ = nullptr; kind_ = K_NONE; fail_ = "";
#if !WB_HAS_NRF24
        (void)pins; (void)c; (void)bound; (void)now_ms; fail_ = "the nRF24 receiver is not in the STM32 builds yet"; return false;
#else
        if (!nrf_bus_.begin(pins[0], pins[1], pins[2], pins[3], pins[4])) { fail_ = "no pin for the nRF24 receiver (Pins tab: nrf_sck, nrf_miso, nrf_mosi, nrf_cs, nrf_ce)"; return false; }
        if (!nrf_.begin(nrf_bus_, c, bound, now_ms)) { fail_ = "the nRF24 module does not answer (wiring, 3.3 V supply, a 10 uF capacitor at its power pins)"; return false; }
        kind_ = K_RADIO; proto_ = rx::PROTO_NRF24; return true;
#endif
    }
    rx::Nrf24Rx* radio() { return kind_ == K_RADIO ? &nrf_ : nullptr; }

    // Drains the UART / the edges / the radio. Returns true when at least one new RC frame arrived.
    bool update(uint32_t now_us, uint32_t now_ms) {
        bool got = false; rx::Frame f;
        if (kind_ == K_SERIAL) {
            // in blocks (serial_block.h): one read() per byte costs several microseconds on Arduino-ESP32 3.x - at 500 Hz CRSF that was
            // enough to fall behind the receiver
            uint8_t buf[128]; size_t total = 0;
            while (total < 2048) {
                int a = uart_->available(); if (a <= 0) break;
                if (a > (int)sizeof(buf)) a = (int)sizeof(buf);
                const size_t n = port::uart_read(*uart_, buf, (size_t)a); if (!n) break;
                for (size_t i = 0; i < n; i++) if (dec_->feed(buf[i], now_us, f)) { link_.on_frame(f, now_ms); got = true; }
                total += n;
            }
        } else if (kind_ == K_RADIO) {
            if (nrf_.update(now_us, now_ms)) { for (int i = 0; i < rx::kChannels; i++) f.ch[i] = nrf_.channels()[i]; link_.on_frame(f, now_ms); got = true; }
        } else if (kind_ == K_PPM || kind_ == K_PWM) {
            rx::Edge e; int budget = 256;                     // (PPM: 9 edges a frame; PWM: 16)
            while (budget-- > 0 && ring_.pop(e)) {
                const bool frame = kind_ == K_PPM ? ppm_.edge(e.us, f) : pwm_.edge(e.input, e.level, e.us, f);
                if (frame) { link_.on_frame(f, now_ms); got = true; }
            }
        }
        link_.tick(now_ms);
        return got;
    }

    // CRSF telemetry back to the receiver: only on CRSF with the UART's TX pin set
    bool telemetryPossible() const { return kind_ == K_SERIAL && tx_ok_ && proto_ == rx::PROTO_CRSF; }
    // MAVLink: the decoder (RADIO_STATUS: flow control), and the UART the MAVLink telemetry goes back on (nullptr: no TX pin)
    const rx::MavlinkDecoder* mavlink() const { return kind_ == K_SERIAL && proto_ == rx::PROTO_MAVLINK ? &mav_ : nullptr; }
    port::SerialPort* mavlinkUart() const { return mavlink() && tx_ok_ ? uart_ : nullptr; }
    // ... and the ground station's frames it passed on (mav::Gcs; false: none waiting)
    bool takeGcsFrame(mav::Parser::Frame& f) { return kind_ == K_SERIAL && proto_ == rx::PROTO_MAVLINK && mav_.take_gcs(f); }
    void sendTelemetry(const uint8_t* frame, size_t n) { if (telemetryPossible() && n) { uart_->write(frame, n); tlm_frames_++; } }
    uint32_t telemetryFrames() const { return tlm_frames_; }
    bool available() const { return kind_ != K_NONE; }
    bool onUart() const { return kind_ == K_SERIAL; }          // false for PPM / PWM: UART2 stays free for something else
    bool linkOk(uint32_t now_ms) const { return kind_ != K_NONE && link_.ok(now_ms); }
    const uint16_t* channels() const { return link_.channels(); }
    uint16_t rateHz() const { return link_.rate_hz(); }
    uint8_t linkQuality() const { return kind_ == K_SERIAL ? dec_->link_quality() : kind_ == K_RADIO ? nrf_.lq() : 0; }
    uint32_t uartOverflows() const { return kind_ == K_SERIAL ? g_uart_ovf : 0; }     // receive buffer / FIFO overflow events since begin()
    uint32_t uartLineErrors() const { return kind_ == K_SERIAL ? g_uart_line : 0; }   // framing / parity errors since begin()
    uint32_t frameErrors() const { return kind_ == K_RADIO ? nrf_.bad() : kind_ == K_SERIAL ? dec_->errors() : kind_ == K_PPM ? ppm_.errors() + ring_.lost() : kind_ == K_PWM ? pwm_.errors() + ring_.lost() : 0; }
    const char* name() const { return kind_ == K_RADIO ? "NRF24" : kind_ == K_SERIAL ? dec_->name() : kind_ == K_PPM ? "PPM" : kind_ == K_PWM ? "PWM" : "none"; }
    int ppmChannels() const { return kind_ == K_PPM ? ppm_.channels() : 0; }      // learnt from the frames, 0 = not yet
    const char* failReason() const { return fail_; }

private:
    static constexpr size_t kRxBuffer = 2048;
    static inline volatile uint32_t g_uart_ovf = 0, g_uart_line = 0;      // (one receiver per firmware)
    enum Kind : uint8_t { K_NONE, K_SERIAL, K_PPM, K_PWM, K_RADIO };
    struct Ctx { RcReceiver* self; uint8_t input, pin; };
    static void WB_FAST on_edge(void* arg) {                  // the GPIO interrupt: the time and the level, nothing else
        const Ctx* c = (const Ctx*)arg;
        c->self->ring_.push(micros(), c->input, (uint8_t)digitalRead(port::apin(c->pin)));
    }
    void stop_pulses() { for (int i = 0; i < n_pins_; i++) if (pins_[i] != 255) detachInterrupt(port::apin(pins_[i])); n_pins_ = 0; }

    rx::CrsfDecoder crsf_; rx::SbusDecoder sbus_; rx::IbusDecoder ibus_; rx::SpektrumDecoder spek_; rx::MavlinkDecoder mav_;
    rx::PpmDecoder ppm_; rx::PwmDecoder pwm_; rx::EdgeRing<64> ring_;
    Ctx ctx_[kMaxPulsePins] = {}; uint8_t pins_[kMaxPulsePins] = {}; int n_pins_ = 0;
    bool tx_ok_ = false; rx::Protocol proto_ = rx::PROTO_CRSF; uint32_t tlm_frames_ = 0;
#if WB_HAS_NRF24
    nrf::ArduinoBus nrf_bus_{(int)SPI_DEVICE_1};
#endif
    rx::Nrf24Rx nrf_;
    rx::Decoder* dec_ = nullptr; Kind kind_ = K_NONE; rx::Link link_; port::SerialPort* uart_ = nullptr; const char* fail_ = "receiver not started";
};
