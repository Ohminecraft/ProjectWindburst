// nRF24: the register driver (src/hal/radio/nrf24.h) against a register-level chip simulator joined by a simulated air
// (test/sim_nrf24.h), and the link packets shared by the transmitter and the receiver (src/radio/wb_radio.h).
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include "sim_nrf24.h"
#include "../src/radio/wb_radio.h"

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, ...) do { const bool ok_ = (cond); if (getenv("V")) { printf("    "); printf(__VA_ARGS__); printf("\n"); } if (ok_) g_pass++; else { g_fail++; printf("  FAIL line %d: %s   ", __LINE__, #cond); printf(__VA_ARGS__); printf("\n"); } } while (0)

static nrf::Config cfg(uint8_t a0, uint8_t ch = 100, uint8_t rate = wbp::NRF_RATE_R250K, uint8_t pps = wbp::NRF_PPS_HZ100) {
    nrf::Config c; const uint8_t a[5] = {a0, 0x12, 0x34, 0x56, 0x78}; memcpy(c.addr, a, 5); c.rf_channel = ch; c.rate = rate; c.power = wbp::NRF_POWER_MAX; c.pps = pps; return c;
}

// a transmitter and a receiver chip in one air, each with its driver
struct Pair {
    uint32_t clock = 0; Air air; SimNrf24 tx_chip, rx_chip; SimBus tx_bus{&tx_chip, &clock}, rx_bus{&rx_chip, &clock}; nrf::Radio tx, rx;
    bool ok = false;
    Pair(const nrf::Config& ct, const nrf::Config& cr) { air.attach(&tx_chip); air.attach(&rx_chip); ok = tx.begin(tx_bus, true, ct) && rx.begin(rx_bus, false, cr); }
    // one packet: send, let the air carry it, poll the result
    nrf::Radio::TxResult exchange(const uint8_t* p, uint8_t n, uint8_t* ack, uint8_t& ack_n) {
        ack_n = 0; if (!tx.send(p, n)) return nrf::Radio::FAILED;
        clock += 2000; air.step(clock);
        return tx.poll_tx(ack, ack_n);
    }
};

static void t_driver() {
    printf("[nrf24 driver]\n");
    { uint32_t clock = 0; SimNrf24 chip; SimBus bus{&chip, &clock}; nrf::Radio r; const bool ok = r.begin(bus, true, cfg(0xC1, 76));
      CHECK(ok && r.present(), "begin finds the chip");
      CHECK((chip.reg[nrf::CONFIG] & 0x0F) == 0x0E, "CONFIG: CRC on, 2 bytes, powered up, transmitter (0x%02X)", chip.reg[nrf::CONFIG]);
      CHECK(chip.reg[nrf::EN_AA] == 0x03 && chip.reg[nrf::SETUP_AW] == 0x03 && chip.reg[nrf::RF_CH] == 76 && chip.reg[nrf::DYNPD] == 0x03 && chip.reg[nrf::FEATURE] == 0x06,
            "auto-ACK on pipes 0 and 1, 5-byte addresses, RF channel 76, dynamic payload + ACK payload");
      CHECK((chip.reg[nrf::RF_SETUP] & 0x28) == 0x20 && (chip.reg[nrf::RF_SETUP] & 0x06) == 0x06, "RF_SETUP: 250 kbps, full power (0x%02X)", chip.reg[nrf::RF_SETUP]);
      CHECK(memcmp(chip.tx_addr, chip.rx_addr[0], 5) == 0 && chip.tx_addr[0] == 0xC1 && chip.reg[nrf::EN_RXADDR] == 0x01, "transmitter: pipe 0 listens on its own address (for the ACK)");
      CHECK((chip.reg[nrf::SETUP_RETR] >> 4) >= 3, "250 kbps with an ACK payload: retransmit delay >= 1000 us (ARD %d)", chip.reg[nrf::SETUP_RETR] >> 4); }
    { uint32_t clock = 0; SimNrf24 chip; SimBus bus{&chip, &clock}; nrf::Radio r; r.begin(bus, false, cfg(0xC1));
      CHECK((chip.reg[nrf::CONFIG] & 0x01) && chip.ce_high && chip.reg[nrf::EN_RXADDR] == 0x02 && chip.rx_addr[1][0] == 0xC1, "receiver: PRIM_RX, CE high (listening), pipe 1 on the address"); }
    { uint32_t clock = 0; SimNrf24 chip; chip.absent = true; SimBus bus{&chip, &clock}; nrf::Radio r;
      CHECK(!r.begin(bus, true, cfg(0xC1)) && !r.present(), "no chip on the wires (MISO floating): begin fails"); }
    { Pair p(cfg(0xC1), cfg(0xC1)); const uint8_t msg[13] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13}, ackmsg[7] = {0x81, 9, 8, 7, 6, 5, 4};
      p.rx.set_ack(ackmsg, 7); uint8_t ack[32], an = 0; const auto res = p.exchange(msg, 13, ack, an);
      uint8_t got[32], gn = 0; const bool rec = p.rx.receive(got, gn);
      CHECK(p.ok && res == nrf::Radio::ACKED && an == 7 && !memcmp(ack, ackmsg, 7), "transmitter: ACKED, with the receiver's ACK payload");
      CHECK(rec && gn == 13 && !memcmp(got, msg, 13) && !p.rx.receive(got, gn), "receiver: the 13 bytes, once"); }
    { Pair p(cfg(0xC1), cfg(0xC2)); const uint8_t msg[13] = {}; uint8_t ack[32], an = 0, got[32], gn = 0;
      CHECK(p.exchange(msg, 13, ack, an) == nrf::Radio::FAILED && !p.rx.receive(got, gn), "another address: never delivered, FAILED after the retries"); }
    { Pair p(cfg(0xC1, 100), cfg(0xC1, 90)); const uint8_t msg[13] = {}; uint8_t ack[32], an = 0, got[32], gn = 0;
      CHECK(p.exchange(msg, 13, ack, an) == nrf::Radio::FAILED && !p.rx.receive(got, gn), "another RF channel: nothing"); }
    { Pair p(cfg(0xC1, 100, wbp::NRF_RATE_R250K), cfg(0xC1, 100, wbp::NRF_RATE_R1M)); const uint8_t msg[13] = {}; uint8_t ack[32], an = 0;
      CHECK(p.exchange(msg, 13, ack, an) == nrf::Radio::FAILED, "another data rate: nothing"); }
    { Pair p(cfg(0xC1), cfg(0xC1)); p.air.loss = 1.0; const uint8_t msg[13] = {}; uint8_t ack[32], an = 0, got[32], gn = 0;
      CHECK(p.exchange(msg, 13, ack, an) == nrf::Radio::FAILED && !p.rx.receive(got, gn) && p.tx.retransmits() == (p.tx_chip.reg[nrf::SETUP_RETR] & 0x0F), "everything lost: FAILED after every retransmission (%u)", p.tx.retransmits()); }
    { Pair p(cfg(0xC1), cfg(0xC1)); p.air.corrupt = 1.0; const uint8_t msg[13] = {}; uint8_t ack[32], an = 0, got[32], gn = 0;
      CHECK(p.exchange(msg, 13, ack, an) == nrf::Radio::FAILED && !p.rx.receive(got, gn), "every packet damaged: the hardware CRC drops them, nothing delivered"); }
    { Pair p(cfg(0xC1), cfg(0xC1)); p.air.loss = 0.5; int acked = 0, rec = 0; uint8_t ack[32], an, got[32], gn;
      for (int i = 0; i < 200; i++) { uint8_t msg[13] = {(uint8_t)i}; acked += p.exchange(msg, 13, ack, an) == nrf::Radio::ACKED; while (p.rx.receive(got, gn)) rec++; }
      CHECK(acked > 110 && rec > 170 && rec >= acked && rec <= 200, "half of everything lost each way, 4 attempts: ~94 %% of the packets arrive, ~68 %% are ACKed (%d acked, %d received), none twice", acked, rec); }
    { Pair p(cfg(0xC1), cfg(0xC1)); const uint8_t msg[13] = {};
      CHECK(p.tx.send(msg, 13) && !p.tx.send(msg, 13), "a second send before the first is resolved is refused"); }
    { // the timing: one attempt = 2 x 130 us settling + the packet + the ACK on the air
      const uint32_t t13 = nrf::airtime_us(wbp::NRF_RATE_R250K, 13), t7 = nrf::airtime_us(wbp::NRF_RATE_R250K, 7);
      CHECK(t13 == 708 && t7 == 516 && nrf::airtime_us(wbp::NRF_RATE_R2M, 13) == 89, "air time: 13 bytes at 250 kbps 708 us, 7 bytes 516 us, 13 bytes at 2 Mbps 89 us (%u %u)", t13, t7);
      CHECK(nrf::ard_us(wbp::NRF_RATE_R250K, 7) == 1000 && nrf::ard_us(wbp::NRF_RATE_R2M, 7) == 500, "retransmit delay: room for the ACK with its payload (1000 us at 250 kbps, 500 us at 2 Mbps)");
      const uint8_t r100 = nrf::retries_for(wbp::NRF_RATE_R250K, wbp::NRF_PPS_HZ100, 13, 7), r150 = nrf::retries_for(wbp::NRF_RATE_R250K, wbp::NRF_PPS_HZ150, 13, 7), r50 = nrf::retries_for(wbp::NRF_RATE_R250K, wbp::NRF_PPS_HZ50, 13, 7);
      CHECK(r100 == 3 && r150 == 1 && r50 == 7, "retransmissions that fit in 80 %% of a period: 100 Hz -> 3, 150 Hz -> 1, 50 Hz -> 7 (%u %u %u)", r100, r150, r50);
      CHECK(nrf::pps_allowed(wbp::NRF_RATE_R250K, wbp::NRF_PPS_HZ150, 13, 7) && nrf::pps_allowed(wbp::NRF_RATE_R2M, wbp::NRF_PPS_HZ150, 13, 7) && !nrf::pps_allowed(wbp::NRF_RATE_R250K, wbp::NRF_PPS_HZ150, 32, 32),
            "150 packets/s fits at every data rate with these packets; two full 32-byte packets at 250 kbps would not"); }
}

static void t_packets() {
    printf("[link packets]\n");
    { const uint16_t ch[8] = {1000, 1500, 2000, 1234, 1000, 1000, 2000, 1500}; uint8_t b[32]; const uint8_t n = wbr::encode_control(42, ch, b);
      uint16_t got[8] = {}; uint8_t seq = 0; const bool ok = wbr::decode_control(b, n, seq, got);
      CHECK(n == 13 && b[0] == wbr::T_CONTROL && ok && seq == 42 && !memcmp(got, ch, sizeof(ch)), "control: 13 bytes, 8 channels round-trip exactly (%u)", n);
      bool all = true; for (int byte = 0; byte < 13; byte++) for (int bit = 0; bit < 8; bit++) { uint8_t c[32]; memcpy(c, b, 13); c[byte] ^= (uint8_t)(1u << bit); uint16_t g[8]; uint8_t sq; if (wbr::decode_control(c, 13, sq, g)) all = false; }
      CHECK(all, "any single flipped bit -> refused");
      CHECK(!wbr::decode_control(b, 12, seq, got) && !wbr::decode_control(b, 14, seq, got), "a wrong length -> refused");
      const uint16_t out_of[8] = {999, 2001, 0, 65535, 1500, 1500, 1500, 1500}; uint8_t c[32]; wbr::encode_control(1, out_of, c); wbr::decode_control(c, 13, seq, got);
      CHECK(got[0] == 1000 && got[1] == 2000 && got[2] == 1000 && got[3] == 2000, "values outside 1000..2000 are sent clamped");
      c[2] = 0xFF; c[3] |= 0x03; c[12] = 0; for (int i = 0; i < 12; i++) c[12] = rx::crc8_dvb_s2(c[12], c[i]);
      CHECK(!wbr::decode_control(c, 13, seq, got), "a 10-bit field of 1023 (> 1000) with a good CRC -> refused"); }
    { wbr::Telemetry t; t.armed = true; t.failsafe = false; t.mode = wbp::FLY_MODE_ALTHOLD; t.batt_cv = 1587; t.lq = 97; uint8_t b[32]; const uint8_t n = wbr::encode_telemetry(t, b);
      wbr::Telemetry g; CHECK(n == 7 && wbr::decode_telemetry(b, n, g) && g.armed && !g.failsafe && g.mode == wbp::FLY_MODE_ALTHOLD && g.batt_cv == 1587 && g.lq == 97, "telemetry: 7 bytes round trip");
      b[3] ^= 1; CHECK(!wbr::decode_telemetry(b, n, g), "damaged telemetry refused"); }
    { wbr::Telemetry t; t.mode = wbp::FLY_MODE_LOITER; uint8_t b[32]; const uint8_t n = wbr::encode_telemetry(t, b); wbr::Telemetry g;
      CHECK(wbr::decode_telemetry(b, n, g) && g.mode == wbp::FLY_MODE_LOITER, "Loiter (0.41) passes on the ACK telemetry"); }
    { wbr::Telemetry t; t.mode = wbp::FLY_MODE_CRASHFLIP; uint8_t b[32]; const uint8_t n = wbr::encode_telemetry(t, b); wbr::Telemetry g;
      CHECK(wbr::decode_telemetry(b, n, g) && g.mode == wbp::FLY_MODE_CRASHFLIP, "crash flip (0.42, the last flight mode) passes on the ACK telemetry"); }
    { wbr::BindOffer o; const uint8_t a[5] = {0x3C, 1, 2, 3, 4}; memcpy(o.addr, a, 5); o.rf_channel = 80; o.rate = wbp::NRF_RATE_R1M; o.pps = wbp::NRF_PPS_HZ150; uint8_t b[32]; const uint8_t n = wbr::encode_bind(o, b);
      wbr::BindOffer g; CHECK(n == 10 && wbr::decode_bind(b, n, g) && !memcmp(g.addr, a, 5) && g.rf_channel == 80 && g.rate == wbp::NRF_RATE_R1M && g.pps == wbp::NRF_PPS_HZ150, "bind offer round trip");
      o.rf_channel = 126; wbr::encode_bind(o, b); CHECK(!wbr::decode_bind(b, n, g), "bind offer with RF channel 126 refused");
      uint8_t k[32]; const uint8_t m = wbr::encode_bind_ack(a, k); uint8_t ga[5]; CHECK(m == 7 && wbr::decode_bind_ack(k, m, ga) && !memcmp(ga, a, 5), "bind ACK echoes the address (7 bytes, as the telemetry: the retries are timed for it)"); }
    { wbr::RadioCfg r; r.seq = 3; r.rf_channel = 90; r.rate = wbp::NRF_RATE_R2M; r.pps = wbp::NRF_PPS_HZ50; uint8_t b[32]; const uint8_t n = wbr::encode_config(r, b); wbr::RadioCfg g;
      CHECK(n == 6 && wbr::decode_config(b, n, g) && g.seq == 3 && g.rf_channel == 90 && g.rate == wbp::NRF_RATE_R2M && g.pps == wbp::NRF_PPS_HZ50, "radio config round trip"); }
    { wbr::SeqFilter f; const uint16_t T = 250;
      const bool a5 = f.accept(5, 0, T), a6 = f.accept(6, 10, T), r6 = f.accept(6, 20, T), r4 = f.accept(4, 30, T), a9 = f.accept(9, 40, T);
      CHECK(a5 && a6 && !r6 && !r4 && a9 && f.lost() == 2, "5, 6 taken; 6 again (a copy whose ACK was lost) and 4 (old) dropped; 9 taken, 7 and 8 counted lost (%u)", f.lost());
      CHECK(f.accept(3, 40 + T + 1, T), "after the timeout anything starts again");
      wbr::SeqFilter w; w.accept(250, 0, T); CHECK(w.accept(2, 10, T) && w.lost() == 7, "the sequence wraps 255 -> 0 (lost %u)", w.lost()); }
    { bool good = true; std::mt19937 r(1);
      for (int i = 0; i < 10000; i++) { uint8_t a[5]; wbr::make_address(r(), r(), a); if (a[0] == 0x00 || a[0] == 0x55 || a[0] == 0xAA || a[0] == 0xFF || !memcmp(a, wbr::kBindAddr, 5)) good = false; }
      uint8_t z[5]; wbr::make_address(0, 0, z); const bool zero_ok = z[0] != 0;
      CHECK(good && zero_ok && wbr::kBindChannel == 83 && !memcmp(wbr::kBindAddr, "WBND!", 5), "made addresses: never preamble-like at the start, never the bind address \"WBND!\""); }
}

static void t_minor() {
    printf("[nrf24 driver: review minors]\n");
    { uint32_t clock = 0; SimNrf24 chip; chip.legacy = true; SimBus bus{&chip, &clock}; nrf::Radio r;
      CHECK(r.begin(bus, true, cfg(0xC1)) && chip.reg[nrf::FEATURE] == 0x06 && chip.reg[nrf::DYNPD] == 0x03, "M5: an nRF24L01 without the + (or a clone): ACTIVATE, then dynamic payload + ACK payload are on (FEATURE 0x%02X)", chip.reg[nrf::FEATURE]); }
    { Pair p(cfg(0xC1), cfg(0xC1)); const uint8_t stale[7] = {0x81, 0, 0, 0, 0, 0, 0}, fresh[7] = {0x81, 1, 2, 3, 4, 5, 6}, msg[13] = {1};
      p.tx_chip.inject_rx(stale, 7, 0); p.rx.set_ack(fresh, 7); uint8_t ack[32], an = 0; p.exchange(msg, 13, ack, an);
      CHECK(an == 7 && !memcmp(ack, fresh, 7) && p.tx_chip.rx.empty(), "M7: an old entry in the transmitter's RX FIFO: the newest ACK payload is taken, the FIFO is left empty"); }
    { Pair p(cfg(0xC1), cfg(0xC1)); const uint8_t msg[13] = {2}; uint8_t ack[32], an, got[32], gn;
      p.exchange(msg, 13, ack, an); p.rx.receive(got, gn); const bool near = p.rx.rpd();
      p.tx_chip.distant = true; const uint8_t m2[13] = {3}; p.exchange(m2, 13, ack, an); p.rx.receive(got, gn);
      CHECK(near && !p.rx.rpd(), "RPD: a transmitter close by reads 1, a far one 0"); }
}

int main() { t_driver(); t_packets(); t_minor(); printf("\n%d passed, %d failed\n", g_pass, g_fail); return g_fail ? 1 : 0; }
