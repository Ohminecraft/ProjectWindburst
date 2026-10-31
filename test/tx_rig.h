// The DIY transmitter on a PC: the real TxApp (src/tx/tx_app.h) with a simulated nRF24 chip in the virtual board's air and four
// joystick readings a test (or wb_sim's tester) sets. step() is called with the simulation clock; the app cycles once per packet
// period like the ESP32-C5's timer would make it. Its settings live in their own store (its own flash).
#pragma once
#include <map>
#include <string>
#include <vector>
#include <stdio.h>
#include "sim_nrf24.h"
#include "../src/tx/tx_app.h"
#include "../src/proto/settings_store.h"

struct TxFlash : wbp::Storage {
    std::map<std::string, std::vector<uint8_t>> kv; int writes = 0;
    int read(const char* k, uint8_t* b, size_t cap) override { auto it = kv.find(k); if (it == kv.end()) return -1; const size_t n = it->second.size() < cap ? it->second.size() : cap; memcpy(b, it->second.data(), n); return (int)n; }
    bool write(const char* k, const uint8_t* d, size_t n) override { kv[k] = std::vector<uint8_t>(d, d + n); writes++; return true; }
};

struct TxRig {
    wbp::Settings s; TxFlash flash; wbp::SettingsStore store{flash};
    SimNrf24 chip; uint32_t clock = 0; SimBus bus{&chip, &clock}; tx::TxApp app; tx::StickInput in;
    Air* air = nullptr; bool on = false, found = false; uint32_t next_us = 0;
    wbp::Server* srv = nullptr;         // the WBP server that answers for it (wb_sim), given to every new TxApp (REPORT)
    bool self_save = true;              // false when a WBP server runs it (wb_sim): the server's tick() keeps what the transmitter changed itself

    TxRig() { store.set_role(wbp::ROLE_TX); store.load(s); }
    // switch it on (again): settings from its flash, radio started
    void power(Air& a, uint32_t now_us) {
        if (air) air->detach(&chip);
        air = &a; a.attach(&chip); chip = SimNrf24(); clock = now_us;
        store.load(s); app = tx::TxApp(); app.configure(s); found = app.begin(bus); on = true; next_us = clock;
        char line[120]; snprintf(line, sizeof(line), "radio: nRF24 %s (simulated), RF channel %u, %s", found ? "found" : "NOT FOUND", (unsigned)s.tx_radio.rf_channel, s.tx_radio.bound ? "bound" : "not bound: BIND in the Radio tab");
        app.add_report(line); if (srv) app.set_server(srv);
    }
    void off() { on = false; if (air) air->detach(&chip); air = nullptr; }
    // bind it to the same pair address / RF channel as a flight controller's settings (as if they had bound earlier)
    void pair_with(wbp::Settings& fc, uint8_t a0 = 0x9A) {
        const uint8_t a[5] = {a0, 0x11, 0x22, 0x33, 0x44};
        memcpy(s.tx_radio.addr, a, 5); s.tx_radio.bound = 1; store.save(s);
        memcpy(fc.receiver.nrf_addr, a, 5); fc.receiver.nrf_channel = s.tx_radio.rf_channel; fc.receiver.nrf_rate = s.tx_radio.rate; fc.receiver.nrf_pps = s.tx_radio.pps; fc.receiver.nrf_bound = 1;
    }
    void stick(int axis, uint16_t raw) { in.raw[axis] = raw; }
    void centre() { for (int i = 0; i < 4; i++) in.raw[i] = 2048; }
    void step(uint32_t now_us) {
        if (!on) return;
        clock = now_us;
        if ((int32_t)(now_us - next_us) < 0) return;
        next_us += app.period_us(); if ((int32_t)(now_us - next_us) > 0) next_us = now_us + app.period_us();
        app.cycle(now_us, in);
        if (self_save) { const uint8_t sec = app.take_autosave(); if (sec) store.save_section(s, sec); }
    }
    wbp::T_tx stream() { uint8_t b[64]; wbp::T_tx t; memset(&t, 0, sizeof(t)); if (app.fill_stream(wbp::STR_TX, b, sizeof(b))) memcpy(&t, b, sizeof(t)); return t; }
    wbp::Err command(uint8_t cmd, uint8_t arg) { uint8_t res[16]; uint16_t n = 0; return app.command(cmd, &arg, 1, res, n); }
};
