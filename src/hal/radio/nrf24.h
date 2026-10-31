/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>
#include <string.h>
#include "../../proto/wbp_schema.h"

// ============================================================================
// nRF24L01(+) at register level (datasheet "nRF24L01+ Product Specification" v1.0, chapters 7 - 9), the same for the transmitter
// and the receiver: Enhanced ShockBurst with a 16-bit CRC, auto-ACK, dynamic payload length and an ACK payload (the receiver's
// telemetry rides on the ACK of each packet). Five-byte addresses: the transmitter sends to TX_ADDR and listens for the ACK on
// pipe 0 (= the same address), the receiver listens on pipe 1.
//
// Nothing here waits for the radio: send() loads a packet and pulses CE, poll_tx() reads STATUS once (the next packet period),
// receive() takes one payload if one is there. The caller decides when (the transmitter's packet timer, the flight controller's
// sensor thread). The bus behind it (Bus) is SPI + two GPIOs on the boards, a simulated chip on the PC (test/sim_nrf24.h).
// The start-up order follows the RF24 library's begin() (read for reference, not copied).
// ============================================================================
namespace nrf {

// registers and commands (datasheet 8.3.1, 9.1)
enum Reg : uint8_t { CONFIG = 0x00, EN_AA = 0x01, EN_RXADDR = 0x02, SETUP_AW = 0x03, SETUP_RETR = 0x04, RF_CH = 0x05, RF_SETUP = 0x06, STATUS = 0x07,
                     OBSERVE_TX = 0x08, RPD = 0x09, RX_ADDR_P0 = 0x0A, RX_ADDR_P1 = 0x0B, TX_ADDR = 0x10, FIFO_STATUS = 0x17, DYNPD = 0x1C, FEATURE = 0x1D };
enum Cmd : uint8_t { R_REGISTER = 0x00, W_REGISTER = 0x20, R_RX_PL_WID = 0x60, R_RX_PAYLOAD = 0x61, W_TX_PAYLOAD = 0xA0, W_ACK_PAYLOAD = 0xA8,
                     FLUSH_TX = 0xE1, FLUSH_RX = 0xE2, NOP = 0xFF };
enum StatusBit : uint8_t { ST_RX_DR = 0x40, ST_TX_DS = 0x20, ST_MAX_RT = 0x10, ST_RX_P_NO = 0x0E, ST_TX_FULL = 0x01 };
constexpr uint8_t kMaxPayload = 32;

struct Bus {
    virtual ~Bus() = default;
    virtual void select(bool on) = 0;          // CSN low (on) / high
    virtual uint8_t xfer(uint8_t v) = 0;       // one byte on the SPI bus
    virtual void ce(bool on) = 0;              // CE pin
    virtual uint32_t micros() = 0;
    virtual void wait_us(uint32_t us) = 0;     // only begin() waits (the crystal and the power-up, ~5 ms once)
};

struct Config { uint8_t addr[5] = {}; uint8_t rf_channel = 100; uint8_t rate = wbp::NRF_RATE_R250K; uint8_t power = wbp::NRF_POWER_MAX; uint8_t pps = wbp::NRF_PPS_HZ100; };

inline uint32_t rate_bps(uint8_t rate) { return rate == wbp::NRF_RATE_R2M ? 2000000u : rate == wbp::NRF_RATE_R1M ? 1000000u : 250000u; }
inline uint32_t pps_hz(uint8_t pps) { return pps == wbp::NRF_PPS_HZ50 ? 50u : pps == wbp::NRF_PPS_HZ150 ? 150u : 100u; }
// time on the air of one packet: preamble 1 + address 5 + payload + CRC 2 bytes, plus the 9-bit packet control field
inline uint32_t airtime_us(uint8_t rate, uint8_t payload) { const uint32_t bits = 8u * (1u + 5u + payload + 2u) + 9u, bps = rate_bps(rate); return (bits * 1000000u + bps - 1u) / bps; }
// retransmit delay (ARD, steps of 250 us): long enough for the receiver to turn round (130 us) and send its ACK with the payload
inline uint32_t ard_us(uint8_t rate, uint8_t ack_payload) { const uint32_t need = airtime_us(rate, ack_payload) + 260u; return ((need + 249u) / 250u) * 250u; }
// automatic retransmissions so that every attempt of one packet ends within 80 % of the packet period (a new packet matters more
// than an old one sent again); one attempt = settling 130 us + the packet + ARD
inline uint8_t retries_for(uint8_t rate, uint8_t pps, uint8_t payload, uint8_t ack_payload) {
    const uint32_t budget = 800000u / pps_hz(pps), attempt = 130u + airtime_us(rate, payload) + ard_us(rate, ack_payload);
    const uint32_t n = budget / attempt;
    return n == 0 ? 0 : (uint8_t)(n - 1 > 15 ? 15 : n - 1);
}
// a packet rate is offered only where one retransmission still fits
inline bool pps_allowed(uint8_t rate, uint8_t pps, uint8_t payload, uint8_t ack_payload) { return retries_for(rate, pps, payload, ack_payload) >= 1; }

class Radio {
public:
    enum TxResult { PENDING, ACKED, FAILED };
    static constexpr uint8_t kCtlPayload = 13, kAckPayload = 7;     // the link's packet sizes (src/radio/wb_radio.h) - what the retries are timed for

    bool begin(Bus& bus, bool primary_tx, const Config& c) {
        bus_ = &bus; ptx_ = primary_tx; busy_ = false;
        bus.ce(false); bus.select(false);
        bus.wait_us(5000);                                          // power-on reset: 100 ms after the supply, the caller started long before
        if (!setup(c)) return false;
        bus.wait_us(5000);                                          // Tpd2stby: 1.5 ms, up to 4.5 ms with a slow crystal
        if (!ptx_) bus.ce(true);
        return true;
    }
    bool present() const { return present_; }
    // The chip still holds what it was given? (a brown-out of the module - motor noise on its 3.3 V, a loose wire - resets it to
    // powered down, and it would stay deaf until the next boot). Two register reads; the callers ask every 100 ms.
    bool healthy() { return bus_ && read(CONFIG) == (uint8_t)(0x0E | (ptx_ ? 0 : 0x01)) && read(RF_CH) == cfg_.rf_channel; }
    // set the chip up again with the settings in use, without begin()'s waits (the receiver calls this from the sensor thread;
    // the radio starts listening / the next packet goes out once the crystal runs, ~1.5 ms later)
    bool reinit() { if (!bus_) return false; reinits_++; busy_ = false; bus_->ce(false); if (!setup(cfg_)) return false; if (!ptx_) bus_->ce(true); return true; }
    uint32_t reinits() const { return reinits_; }
    // the last packet came in above -64 dBm (Received Power Detector): the sender is close by
    bool rpd() { return bus_ && (read(RPD) & 1); }
    void configure(const Config& c) {
        if (!bus_) return;
        cfg_ = c; bus_->ce(false);
        write(RF_CH, (uint8_t)(c.rf_channel > 125 ? 125 : c.rf_channel));
        const uint8_t dr = c.rate == wbp::NRF_RATE_R250K ? 0x20 : c.rate == wbp::NRF_RATE_R2M ? 0x08 : 0x00;
        write(RF_SETUP, (uint8_t)(dr | ((c.power & 3u) << 1)));
        const uint32_t ard = ard_us(c.rate, kAckPayload);
        write(SETUP_RETR, (uint8_t)(((ard / 250u - 1u) << 4) | retries_for(c.rate, c.pps, kCtlPayload, kAckPayload)));
        if (ptx_) { write_addr(TX_ADDR, c.addr); write_addr(RX_ADDR_P0, c.addr); write(EN_RXADDR, 0x01); }
        else { write_addr(RX_ADDR_P1, c.addr); write(EN_RXADDR, 0x02); }
        command(FLUSH_TX); command(FLUSH_RX); write(STATUS, 0x70); busy_ = false;
        if (!ptx_ && present_) bus_->ce(true);
    }
    const Config& config() const { return cfg_; }

    // transmitter: load one packet and start it. false: no chip, or the last one is not resolved yet (poll_tx)
    bool send(const uint8_t* p, uint8_t n) {
        if (!present_ || busy_ || !n || n > kMaxPayload) return false;
        bus_->select(true); bus_->xfer(W_TX_PAYLOAD); for (uint8_t i = 0; i < n; i++) bus_->xfer(p[i]); bus_->select(false);
        bus_->ce(true); bus_->wait_us(15); bus_->ce(false);         // a CE pulse > 10 us starts it; the retransmissions run by themselves
        busy_ = true; sent_us_ = bus_->micros(); return true;
    }
    // transmitter: what became of the last packet (one STATUS read). ACKED copies the receiver's ACK payload, if it sent one.
    TxResult poll_tx(uint8_t* ack, uint8_t& ack_n) {
        ack_n = 0;
        if (!busy_) return FAILED;
        const uint8_t st = command(NOP);
        if (st & ST_TX_DS) {
            retr_ = (uint8_t)(read(OBSERVE_TX) & 0x0F);
            // every ACK payload waiting (an old one left behind would keep the telemetry one packet late): the newest is kept
            for (int k = 0; k < 3 && (command(NOP) & ST_RX_P_NO) != ST_RX_P_NO; k++) take(ack, ack_n);
            write(STATUS, 0x70); busy_ = false; return ACKED;
        }
        if ((st & ST_MAX_RT) || (uint32_t)(bus_->micros() - sent_us_) > 50000u) {   // (a chip that never answers: given up after 50 ms)
            retr_ = (uint8_t)(read(OBSERVE_TX) & 0x0F);
            command(FLUSH_TX); write(STATUS, 0x70); busy_ = false; return FAILED;
        }
        return PENDING;
    }
    // receiver: one payload, if there is one (call until false)
    bool receive(uint8_t* p, uint8_t& n) {
        n = 0;
        if (!present_) return false;
        const uint8_t st = command(NOP);
        if ((st & ST_RX_P_NO) == ST_RX_P_NO) return false;          // RX FIFO empty
        return take(p, n);
    }
    // receiver: what the ACK of the next packet carries (only the newest one is kept)
    void set_ack(const uint8_t* p, uint8_t n) {
        if (!present_ || !n || n > kMaxPayload) return;
        command(FLUSH_TX);
        bus_->select(true); bus_->xfer((uint8_t)(W_ACK_PAYLOAD | 1)); for (uint8_t i = 0; i < n; i++) bus_->xfer(p[i]); bus_->select(false);
    }
    uint8_t retransmits() const { return retr_; }

private:
    bool setup(const Config& c) {
        write(CONFIG, 0x0C);                                        // powered down, CRC 2 bytes
        write(SETUP_AW, 0x03);
        present_ = read(SETUP_AW) == 0x03 && read(CONFIG) == 0x0C;     // a floating MISO reads 0xFF / 0x00: no chip
        if (!present_) return false;
        write(EN_AA, 0x03); write(FEATURE, 0x06); write(DYNPD, 0x03); // auto-ACK + dynamic payload on pipes 0 and 1, ACK payload
        if (read(FEATURE) != 0x06) {                                // an nRF24L01 without the + (and some clones): FEATURE needs ACTIVATE first
            bus_->select(true); bus_->xfer(0x50); bus_->xfer(0x73); bus_->select(false);
            write(FEATURE, 0x06); write(DYNPD, 0x03);
        }
        configure(c);
        write(CONFIG, (uint8_t)(0x0E | (ptx_ ? 0 : 0x01)));           // EN_CRC, CRCO (2 bytes), PWR_UP, PRIM_RX on the receiver
        return true;
    }
    uint8_t command(uint8_t c) { bus_->select(true); const uint8_t st = bus_->xfer(c); bus_->select(false); return st; }
    uint8_t read(uint8_t reg) { bus_->select(true); bus_->xfer((uint8_t)(R_REGISTER | reg)); const uint8_t v = bus_->xfer(NOP); bus_->select(false); return v; }
    void write(uint8_t reg, uint8_t v) { bus_->select(true); bus_->xfer((uint8_t)(W_REGISTER | reg)); bus_->xfer(v); bus_->select(false); }
    void write_addr(uint8_t reg, const uint8_t* a) { bus_->select(true); bus_->xfer((uint8_t)(W_REGISTER | reg)); for (int i = 0; i < 5; i++) bus_->xfer(a[i]); bus_->select(false); }
    bool take(uint8_t* p, uint8_t& n) {
        bus_->select(true); bus_->xfer(R_RX_PL_WID); const uint8_t w = bus_->xfer(NOP); bus_->select(false);
        if (!w || w > kMaxPayload) { command(FLUSH_RX); write(STATUS, ST_RX_DR); n = 0; return false; }     // (datasheet: a width > 32 = flush)
        bus_->select(true); bus_->xfer(R_RX_PAYLOAD); for (uint8_t i = 0; i < w; i++) p[i] = bus_->xfer(NOP); bus_->select(false);
        write(STATUS, ST_RX_DR); n = w; return true;
    }
    Bus* bus_ = nullptr; Config cfg_; bool ptx_ = true, present_ = false, busy_ = false; uint32_t sent_us_ = 0, reinits_ = 0; uint8_t retr_ = 0;
};

} // namespace nrf
