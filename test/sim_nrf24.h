// A simulated nRF24L01+ at register level (what src/hal/radio/nrf24.h talks to, byte by byte on SPI) and the air between chips.
// Modelled: the registers the driver uses, the 3-deep RX / TX FIFOs, Enhanced ShockBurst (packet ID, auto-ACK, ACK payload,
// automatic retransmission with ARC, MAX_RT, a duplicate dropped by the receiver but ACKed again), RF channel / data rate /
// address matching. Not modelled: timing inside a packet (an exchange completes at the next Air::step), interrupts, RPD.
#pragma once
#include <stdint.h>
#include <string.h>
#include <deque>
#include <vector>
#include <random>
#include "../src/hal/radio/nrf24.h"

struct SimNrf24 {
    uint8_t reg[0x20] = {};
    uint8_t rx_addr[2][5] = {{0xE7, 0xE7, 0xE7, 0xE7, 0xE7}, {0xC2, 0xC2, 0xC2, 0xC2, 0xC2}}, tx_addr[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
    struct Pkt { std::vector<uint8_t> b; uint8_t pipe = 0; };
    std::deque<Pkt> rx, tx;                 // FIFOs (in a receiver the TX FIFO holds the ACK payloads)
    bool ce_high = false, absent = false;   // absent: nothing on the wires (MISO floats high)
    bool tx_request = false;                // a CE rising edge with a packet loaded (transmitter)
    uint8_t pid = 0, last_pid = 0xFF; uint32_t last_crc = 0;      // ShockBurst packet ID (transmitter / the receiver's duplicate check)
    uint32_t delivered = 0, duplicates = 0;
    bool legacy = false, activated = false;  // an nRF24L01 without the +, or a clone: FEATURE / DYNPD ignored until ACTIVATE 0x73
    bool distant = false;                    // this chip transmits from far away: receivers see RPD = 0 (below -64 dBm)
    uint32_t writes_while_tx = 0;            // registers written while a packet was on its way (the datasheet: only in standby)

    SimNrf24() { reg[nrf::CONFIG] = 0x08; reg[nrf::EN_AA] = 0x3F; reg[nrf::EN_RXADDR] = 0x03; reg[nrf::SETUP_AW] = 0x03; reg[nrf::SETUP_RETR] = 0x03; reg[nrf::RF_CH] = 0x02; reg[nrf::RF_SETUP] = 0x0E; }
    bool prx() const { return reg[nrf::CONFIG] & 0x01; }
    bool powered() const { return reg[nrf::CONFIG] & 0x02; }
    uint8_t rate() const { return (reg[nrf::RF_SETUP] & 0x20) ? wbp::NRF_RATE_R250K : (reg[nrf::RF_SETUP] & 0x08) ? wbp::NRF_RATE_R2M : wbp::NRF_RATE_R1M; }
    uint8_t status() const {
        return (uint8_t)((reg[nrf::STATUS] & 0x70) | ((rx.empty() ? 7 : rx.front().pipe) << 1) | (tx.size() >= 3 ? 1 : 0));
    }

    // ---- SPI, one byte at a time between select(true) and select(false)
    void select(bool on) {
        if (on) { sel = true; idx = 0; cmd = 0; buf.clear(); return; }
        if (!sel) return;
        sel = false;
        if (cmd == 0x50 && !buf.empty() && buf[0] == 0x73) activated = !activated;
        if (cmd == nrf::W_TX_PAYLOAD && !buf.empty() && tx.size() < 3) { tx.push_back({buf, 0}); pid = (uint8_t)((pid + 1) & 3); }
        else if ((cmd & 0xF8) == nrf::W_ACK_PAYLOAD && !buf.empty() && tx.size() < 3) tx.push_back({buf, (uint8_t)(cmd & 7)});
        else if (cmd == nrf::R_RX_PAYLOAD && !rx.empty() && idx > 1) rx.pop_front();
    }
    uint8_t xfer(uint8_t v) {
        if (absent) return 0xFF;
        if (!sel) return 0xFF;
        const int i = idx++;
        if (i == 0) {
            cmd = v; const uint8_t st = status();
            if (v == nrf::FLUSH_TX) tx.clear();
            else if (v == nrf::FLUSH_RX) rx.clear();
            return st;
        }
        const uint8_t r = cmd & 0x1F;
        if (cmd < 0x20) {                                         // R_REGISTER
            if (r == nrf::RX_ADDR_P0 || r == nrf::RX_ADDR_P1) return i - 1 < 5 ? rx_addr[r - nrf::RX_ADDR_P0][i - 1] : 0;
            if (r == nrf::TX_ADDR) return i - 1 < 5 ? tx_addr[i - 1] : 0;
            if (r == nrf::STATUS) return status();
            if (r == nrf::FIFO_STATUS) return (uint8_t)((rx.empty() ? 1 : 0) | (rx.size() >= 3 ? 2 : 0) | (tx.empty() ? 0x10 : 0) | (tx.size() >= 3 ? 0x20 : 0));
            return reg[r];
        }
        if (cmd < 0x40) {                                         // W_REGISTER
            if (r == nrf::RX_ADDR_P0 || r == nrf::RX_ADDR_P1) { if (i - 1 < 5) rx_addr[r - nrf::RX_ADDR_P0][i - 1] = v; return 0; }
            if (r == nrf::TX_ADDR) { if (i - 1 < 5) tx_addr[i - 1] = v; return 0; }
            if (i == 1 && tx_request && !prx()) writes_while_tx++;
            if (i == 1 && legacy && !activated && (r == nrf::FEATURE || r == nrf::DYNPD)) return 0;
            if (i == 1) { if (r == nrf::STATUS) reg[nrf::STATUS] &= (uint8_t)~(v & 0x70); else reg[r] = v; }
            return 0;
        }
        if (cmd == nrf::R_RX_PL_WID) return rx.empty() ? 0 : (uint8_t)rx.front().b.size();
        if (cmd == nrf::R_RX_PAYLOAD) return !rx.empty() && (size_t)(i - 1) < rx.front().b.size() ? rx.front().b[i - 1] : 0;
        if (cmd == nrf::W_TX_PAYLOAD || (cmd & 0xF8) == nrf::W_ACK_PAYLOAD || cmd == 0x50) { if (buf.size() < 32) buf.push_back(v); return 0; }
        return 0;
    }
    void ce(bool on) { if (on && !ce_high && !prx() && !tx.empty()) tx_request = true; ce_high = on; }

    // a packet that passed the hardware CRC put straight into the RX FIFO (the tests' way to deliver bytes the software must reject)
    void inject_rx(const uint8_t* p, uint8_t n, uint8_t pipe = 1) { rx.push_back({std::vector<uint8_t>(p, p + n), pipe}); reg[nrf::STATUS] |= nrf::ST_RX_DR; }

private:
    bool sel = false; int idx = 0; uint8_t cmd = 0; std::vector<uint8_t> buf;
};

// The air: every transmitter's pending packet goes out at step() to the receivers on the same RF channel and data rate whose
// enabled pipe has the packet's address. loss / corrupt: per attempt (a corrupt packet fails the receiver's CRC: never seen).
struct Air {
    double loss = 0, corrupt = 0;
    std::vector<SimNrf24*> chips;
    std::mt19937 rng{777};
    void attach(SimNrf24* c) { chips.push_back(c); }
    void detach(SimNrf24* c) { for (size_t i = 0; i < chips.size(); i++) if (chips[i] == c) { chips.erase(chips.begin() + i); return; } }
    bool roll(double p) { return p > 0 && std::uniform_real_distribution<double>(0, 1)(rng) < p; }

    void step(uint32_t /*now_us*/) { for (SimNrf24* t : chips) if (t->tx_request) send(*t); }

private:
    SimNrf24* receiver_for(const SimNrf24& t, uint8_t& pipe) {
        for (SimNrf24* r : chips) {
            if (r == &t || !r->prx() || !r->powered() || !r->ce_high || r->absent) continue;
            if (r->reg[nrf::RF_CH] != t.reg[nrf::RF_CH] || r->rate() != t.rate() || ((r->reg[nrf::CONFIG] ^ t.reg[nrf::CONFIG]) & 0x0C)) continue;
            for (uint8_t p = 0; p < 2; p++) if ((r->reg[nrf::EN_RXADDR] >> p & 1) && !memcmp(r->rx_addr[p], t.tx_addr, 5)) { pipe = p; return r; }
        }
        return nullptr;
    }
    static uint32_t crc_of(const std::vector<uint8_t>& b) { uint32_t h = 2166136261u; for (uint8_t c : b) { h ^= c; h *= 16777619u; } return h; }
    void send(SimNrf24& t) {
        t.tx_request = false;
        if (!t.powered() || t.prx() || t.tx.empty()) return;
        const SimNrf24::Pkt pkt = t.tx.front();
        const int arc = t.reg[nrf::SETUP_RETR] & 0x0F;
        uint8_t pipe = 0; SimNrf24* r = receiver_for(t, pipe);
        for (int attempt = 0; attempt <= arc; attempt++) {
            t.reg[nrf::OBSERVE_TX] = (uint8_t)((t.reg[nrf::OBSERVE_TX] & 0xF0) | attempt);
            if (!r || roll(loss) || roll(corrupt)) continue;            // never reached the receiver / its CRC failed
            const uint32_t c = crc_of(pkt.b);
            const bool dup = r->last_pid == t.pid && r->last_crc == c;  // (a retransmission whose ACK was lost)
            if (!dup) {
                if (r->rx.size() >= 3) continue;                         // RX FIFO full: no ACK
                r->rx.push_back({pkt.b, pipe}); r->reg[nrf::STATUS] |= nrf::ST_RX_DR; r->last_pid = t.pid; r->last_crc = c; r->delivered++;
                r->reg[nrf::RPD] = t.distant ? 0 : 1;
            } else r->duplicates++;
            if (roll(loss)) continue;                                    // the ACK lost: the transmitter tries again
            t.tx.pop_front();
            for (size_t k = 0; k < r->tx.size(); k++) if (r->tx[k].pipe == pipe) {   // the ACK carries the receiver's ACK payload
                if (t.rx.size() < 3) { t.rx.push_back({r->tx[k].b, 0}); t.reg[nrf::STATUS] |= nrf::ST_RX_DR; }
                r->tx.erase(r->tx.begin() + (long)k); break;
            }
            t.reg[nrf::STATUS] |= nrf::ST_TX_DS; return;
        }
        t.reg[nrf::STATUS] |= nrf::ST_MAX_RT;                            // the packet stays in the TX FIFO (the driver flushes it)
    }
};

// a driver's bus wired straight to a simulated chip, on a test clock
struct SimBus : nrf::Bus {
    SimNrf24* chip; uint32_t* clock;
    SimBus(SimNrf24* c, uint32_t* t) : chip(c), clock(t) {}
    void select(bool on) override { chip->select(on); }
    uint8_t xfer(uint8_t v) override { return chip->xfer(v); }
    void ce(bool on) override { chip->ce(on); }
    uint32_t micros() override { return *clock; }
    void wait_us(uint32_t us) override { *clock += us; }
};
