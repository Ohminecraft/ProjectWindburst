// The DIY transmitter's portable core: sticks -> channels (src/tx/sticks.h), the link state machine (src/tx/tx_link.h) and the
// app host (src/tx/tx_app.h), against a simulated nRF24 receiver that answers like the flight controller does.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <string>
#include <vector>
#include "sim_nrf24.h"
#include "../src/tx/tx_app.h"

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, ...) do { const bool ok_ = (cond); if (getenv("V")) { printf("    "); printf(__VA_ARGS__); printf("\n"); } if (ok_) g_pass++; else { g_fail++; printf("  FAIL line %d: %s   ", __LINE__, #cond); printf(__VA_ARGS__); printf("\n"); } } while (0)

static wbp::S_tx_sticks stick_defaults() { wbp::S_tx_sticks s; wbp::defaults(s); return s; }
static tx::StickInput centred() { tx::StickInput in; for (int i = 0; i < 4; i++) in.raw[i] = 2048; return in; }

static void t_sticks() {
    printf("[sticks]\n");
    tx::Sticks st; wbp::S_tx_sticks c = stick_defaults(); c.throttle_mode = wbp::TX_THROTTLE_ALTPOS; st.configure(c); uint16_t ch[8];
    tx::StickInput in = centred(); st.update(in, false, 0, ch);
    CHECK(ch[0] == 1500 && ch[1] == 1500 && ch[3] == 1500 && ch[2] == 1500 && ch[4] == 1000 && ch[7] == 1000, "centred: roll, pitch, yaw 1500; ALTPOS throttle 1500 (= hold the height); AUX 1000");
    in.raw[0] = 4095; in.raw[1] = 0; in.raw[3] = 4095; in.raw[2] = 0; st.update(in, false, 10, ch);
    CHECK(ch[0] == 2000 && ch[1] == 1000 && ch[2] == 2000 && ch[3] == 1000, "full deflection: joy1 X -> roll 2000, joy1 Y -> pitch 1000, joy2 Y -> throttle 2000, joy2 X -> yaw 1000");
    c.invert[0] = 1; st.configure(c); st.update(in, false, 20, ch); CHECK(ch[0] == 1000, "invert reverses roll");
    c.invert[0] = 0; st.configure(c); in = centred(); in.raw[0] = 2048 + (uint16_t)(2047 * 0.02); st.update(in, false, 30, ch); CHECK(ch[0] == 1500, "inside the 3 %% dead band: exactly 1500");
    in.raw[0] = 2048 + 1023; st.update(in, false, 40, ch); const int lin = ch[0];
    c.expo = 50; st.configure(c); st.update(in, false, 50, ch); CHECK(ch[0] < lin && ch[0] > 1500, "expo 50 %% at half stick is gentler than linear (%d < %d)", ch[0], lin);
    c.expo = 0; c.map[0] = 2; c.map[3] = 0; st.configure(c); in = centred(); in.raw[2] = 4095; st.update(in, false, 60, ch); CHECK(ch[0] == 2000 && ch[3] == 1500, "the stick map: roll from joy2 X");
    // MANUAL: the throttle stick moves the throttle, a released stick leaves it where it is
    tx::Sticks m; wbp::S_tx_sticks mc = stick_defaults(); m.configure(mc); in = centred();
    m.update(in, true, 0, ch); CHECK(ch[2] == 1000, "MANUAL starts at 1000");
    in.raw[3] = 4095; for (uint32_t t = 10; t <= 1000; t += 10) m.update(in, true, t, ch);
    CHECK(abs((int)ch[2] - 1500) <= 10, "full stick up for 1 s at 50 %%/s: ~1500 (%u)", ch[2]);
    in.raw[3] = 2048; for (uint32_t t = 1010; t <= 2000; t += 10) m.update(in, true, t, ch); CHECK(abs((int)ch[2] - 1500) <= 10, "stick released: it stays (%u)", ch[2]);
    in.raw[3] = 0; for (uint32_t t = 2010; t <= 2500; t += 10) m.update(in, true, t, ch); CHECK(ch[2] < 1300 && ch[2] > 1200, "stick down for 0.5 s: down by ~250 (%u)", ch[2]);
    in.raw[3] = 2048; m.update(in, false, 2510, ch); CHECK(ch[2] == 1000, "the drone not armed: 1000 at once (arming and letting go of the sticks never lifts it)");
    in.raw[3] = 4095; m.update(in, false, 3000, ch); CHECK(ch[2] == 1000, "and it stays 1000 until the drone is armed");
    // calibration: the ends seen while it runs, the centre where the sticks rest at the end
    tx::Sticks k; wbp::S_tx_sticks kc = stick_defaults(); k.configure(kc); k.cal_start(); CHECK(k.calibrating(), "calibrating");
    for (int i = 0; i < 4; i++) { in = centred(); in.raw[i] = 300; k.update(in, false, 0, ch); in.raw[i] = 3800; k.update(in, false, 0, ch); }
    in = centred(); in.raw[0] = 2000; in.raw[1] = 2000; in.raw[2] = 2000; in.raw[3] = 2000; k.update(in, false, 0, ch);
    const int n = k.cal_finish(kc);
    CHECK(n == 4 && kc.cal_min[0] == 300 && kc.cal_max[0] == 3800 && kc.cal_mid[0] == 2000 && kc.cal_min[3] == 300 && !k.calibrating(), "calibration: min 300, max 3800, centre 2000 on all four axes");
    tx::Sticks u; wbp::S_tx_sticks uc = stick_defaults(); u.configure(uc); u.cal_start(); in = centred(); u.update(in, false, 0, ch); in.raw[0] = 3900; u.update(in, false, 0, ch); in.raw[0] = 100; u.update(in, false, 0, ch); in = centred(); u.update(in, false, 0, ch);
    const int n2 = u.cal_finish(uc); CHECK(n2 == 1 && uc.cal_min[0] == 100 && uc.cal_min[1] == 0 && uc.cal_max[1] == 4095, "an axis that was not moved keeps its old calibration (%d axes)", n2);
}

// ---- a simulated flight controller receiver: answers control packets with telemetry, binds, takes radio settings
struct FakeDrone {
    SimNrf24 chip; uint32_t* clock; SimBus bus; nrf::Radio radio; nrf::Config cfg; wbr::Telemetry tel; bool binding = false, switch_after_ack = false;
    bool ignore_config = false, never_move = false; nrf::Config next; uint32_t controls = 0; uint16_t ch[8] = {}; uint8_t seq = 0;
    FakeDrone(Air& air, uint32_t* t, const nrf::Config& c) : clock(t), bus(&chip, t), cfg(c) { air.attach(&chip); radio.begin(bus, false, c); tel.batt_cv = 1600; tel.lq = 100; ack_tel(); }
    void ack_tel() { uint8_t b[32]; radio.set_ack(b, wbr::encode_telemetry(tel, b)); }
    void listen_bind() { nrf::Config b; memcpy(b.addr, wbr::kBindAddr, 5); b.rf_channel = wbr::kBindChannel; b.power = wbp::NRF_POWER_MIN; radio.configure(b); binding = true; }
    void service() {
        if (switch_after_ack && chip.tx.empty() && !never_move) { cfg = next; radio.configure(cfg); switch_after_ack = false; binding = false; ack_tel(); }
        uint8_t p[32], n;
        while (radio.receive(p, n)) {
            wbr::BindOffer o; wbr::RadioCfg rc;
            if (binding && wbr::decode_bind(p, n, o)) { uint8_t b[32]; radio.set_ack(b, wbr::encode_bind_ack(o.addr, b)); next = cfg; memcpy(next.addr, o.addr, 5); next.rf_channel = o.rf_channel; next.rate = o.rate; next.pps = o.pps; switch_after_ack = true; }
            else if (wbr::decode_config(p, n, rc) && ignore_config) ack_tel();
            else if (wbr::decode_config(p, n, rc)) { cfg.rf_channel = rc.rf_channel; cfg.rate = rc.rate; cfg.pps = rc.pps; radio.configure(cfg); ack_tel(); }   // (its ACK went out when it arrived: move now)
            else if (wbr::decode_control(p, n, seq, ch)) { controls++; ack_tel(); }
        }
    }
};

static nrf::Config cfg_of(const wbp::S_tx_radio& r) { nrf::Config c; memcpy(c.addr, r.addr, 5); c.rf_channel = r.rf_channel; c.rate = r.rate; c.power = r.power; c.pps = r.pps; return c; }
static wbp::S_tx_radio bound_radio(uint8_t a0) { wbp::S_tx_radio r; wbp::defaults(r); const uint8_t a[5] = {a0, 9, 8, 7, 6}; memcpy(r.addr, a, 5); r.bound = 1; return r; }

static void t_link() {
    printf("[link]\n");
    { uint32_t clock = 0; Air air; SimNrf24 chip; SimBus bus{&chip, &clock}; air.attach(&chip); wbp::S_tx_radio r = bound_radio(0xB1);
      FakeDrone d(air, &clock, cfg_of(r)); nrf::Radio radio; radio.begin(bus, true, cfg_of(r)); tx::TxLink link; link.begin(radio, r);
      uint16_t ch[8] = {1600, 1500, 1200, 1500, 1000, 1000, 1000, 1000};
      d.tel.armed = true; d.tel.mode = wbp::FLY_MODE_ANGLE; d.ack_tel();
      for (int i = 0; i < 120; i++) { link.tick(clock, ch); air.step(clock); d.service(); clock += 10000; }     // (1.2 s: a full one-second count)
      wbr::Telemetry t; const bool got = link.telemetry(t, clock / 1000);
      CHECK(d.controls >= 118 && d.ch[0] == 1600 && d.ch[2] == 1200, "100 packets/s for 1.2 s: the drone gets them (%u), channels as sent", d.controls);
      CHECK(link.stats().link && link.stats().ack_pct >= 98 && link.stats().pps >= 98 && link.stats().pps <= 101, "link up, ~100 %% ACKed, ~100 packets/s (%u %%, %u/s)", link.stats().ack_pct, link.stats().pps);
      CHECK(got && t.armed && t.mode == wbp::FLY_MODE_ANGLE && t.batt_cv == 1600, "the drone's telemetry arrives on the ACKs");
      air.detach(&d.chip); for (int i = 0; i < 40; i++) { link.tick(clock, ch); air.step(clock); clock += 10000; }
      CHECK(link.stats().link, "drone gone for 400 ms: still a link (the 500 ms rule)");
      for (int i = 0; i < 20; i++) { link.tick(clock, ch); air.step(clock); clock += 10000; }
      CHECK(!link.stats().link, "gone for 600 ms: no link");
      for (int i = 0; i < 180; i++) { link.tick(clock, ch); air.step(clock); clock += 10000; }     // (to 3.2 s: the second 2..3 s had no ACK at all)
      CHECK(!link.telemetry(t, clock / 1000) && link.stats().ack_pct == 0, "and its telemetry is forgotten after 1 s"); }
    { // bind: a new address, then the control packets go there
      uint32_t clock = 0; Air air; SimNrf24 chip; SimBus bus{&chip, &clock}; air.attach(&chip); wbp::S_tx_radio r; wbp::defaults(r);
      nrf::Config any; FakeDrone d(air, &clock, any); d.listen_bind(); nrf::Radio radio; radio.begin(bus, true, cfg_of(r)); tx::TxLink link; link.begin(radio, r);
      CHECK(link.bind_state() == wbp::BIND_STATE_IDLE, "never bound: IDLE");
      link.start_bind(0x11223344u, 0x55u, clock); uint16_t ch[8] = {1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000}; uint8_t addr[5] = {};
      CHECK(chip.reg[nrf::RF_CH] == wbr::kBindChannel && !memcmp(chip.tx_addr, wbr::kBindAddr, 5) && (chip.reg[nrf::RF_SETUP] & 0x06) == 0, "binding: on the bind address / channel at the lowest power");
      bool bound = false; for (int i = 0; i < 50 && !bound; i++) { link.tick(clock, ch); air.step(clock); d.service(); clock += 10000; bound = link.take_bound(addr); }
      CHECK(bound && addr[0] == 0x44 && addr[1] == 0x33 && link.bind_state() == wbp::BIND_STATE_BOUND && !memcmp(chip.tx_addr, addr, 5) && (chip.reg[nrf::RF_SETUP] & 0x06) == 0x06, "bound: the new address from the random words, full power again");
      const uint32_t before = d.controls; for (int i = 0; i < 20; i++) { link.tick(clock, ch); air.step(clock); d.service(); clock += 10000; }
      CHECK(d.controls >= before + 18 && !memcmp(d.cfg.addr, addr, 5), "the drone moved to it too, control packets flow (%u)", d.controls - before); }
    { // nobody answers the bind: given up after 30 s, back where it was
      uint32_t clock = 0; Air air; SimNrf24 chip; SimBus bus{&chip, &clock}; air.attach(&chip); wbp::S_tx_radio r = bound_radio(0xB2);
      nrf::Radio radio; radio.begin(bus, true, cfg_of(r)); tx::TxLink link; link.begin(radio, r); uint16_t ch[8] = {}; uint8_t a[5];
      CHECK(link.bind_state() == wbp::BIND_STATE_BOUND, "bound in the settings: BOUND");
      link.start_bind(1, 2, clock); for (int i = 0; i < 3100; i++) { link.tick(clock, ch); air.step(clock); clock += 10000; }
      CHECK(link.bind_state() == wbp::BIND_STATE_FAILED && !link.take_bound(a) && chip.tx_addr[0] == 0xB2 && chip.reg[nrf::RF_CH] == 76, "no drone for 30 s: FAILED, the old address again"); }
    { // a new RF channel while the drone is on: both move
      uint32_t clock = 0; Air air; SimNrf24 chip; SimBus bus{&chip, &clock}; air.attach(&chip); wbp::S_tx_radio r = bound_radio(0xB3);
      FakeDrone d(air, &clock, cfg_of(r)); nrf::Radio radio; radio.begin(bus, true, cfg_of(r)); tx::TxLink link; link.begin(radio, r); uint16_t ch[8] = {1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000};
      for (int i = 0; i < 10; i++) { link.tick(clock, ch); air.step(clock); d.service(); clock += 10000; }
      wbp::S_tx_radio n = r; n.rf_channel = 80; CHECK(link.request_config(n, clock / 1000), "a new RF channel accepted while the drone is disarmed");
      for (int i = 0; i < 30; i++) { link.tick(clock, ch); air.step(clock); d.service(); clock += 10000; }
      const uint32_t before = d.controls; for (int i = 0; i < 20; i++) { link.tick(clock, ch); air.step(clock); d.service(); clock += 10000; }
      CHECK(chip.reg[nrf::RF_CH] == 80 && d.chip.reg[nrf::RF_CH] == 80 && d.controls >= before + 18 && link.stats().link, "both on RF channel 80, the link up (%u)", d.controls - before);
      d.tel.armed = true; d.ack_tel(); for (int i = 0; i < 5; i++) { link.tick(clock, ch); air.step(clock); d.service(); clock += 10000; }
      n.rf_channel = 60; CHECK(!link.request_config(n, clock / 1000) && chip.reg[nrf::RF_CH] == 80, "refused while the drone says it is armed"); }
}

static void t_link_fallback() {
    printf("[link: settings the drone does not follow]\n");
    uint32_t clock = 0; Air air; SimNrf24 chip; SimBus bus{&chip, &clock}; air.attach(&chip); wbp::S_tx_radio r = bound_radio(0xB4);
    FakeDrone d(air, &clock, cfg_of(r)); d.ignore_config = true; nrf::Radio radio; radio.begin(bus, true, cfg_of(r)); tx::TxLink link; link.begin(radio, r);
    uint16_t ch[8] = {1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000};
    for (int i = 0; i < 10; i++) { link.tick(clock, ch); air.step(clock); d.service(); clock += 10000; }
    wbp::S_tx_radio n = r; n.rf_channel = 70; link.request_config(n, clock / 1000);
    bool failed = false; for (int i = 0; i < 200 && !failed; i++) { link.tick(clock, ch); air.step(clock); d.service(); clock += 10000; failed = link.take_config_failed(); }
    const uint32_t before = d.controls; for (int i = 0; i < 20; i++) { link.tick(clock, ch); air.step(clock); d.service(); clock += 10000; }
    CHECK(failed && link.applied().rf_channel == 76 && chip.reg[nrf::RF_CH] == 76 && d.controls >= before + 18, "no answer on RF channel 70 within 1 s: back on 76, the link works again (%u)", d.controls - before);
    n = r; n.power = wbp::NRF_POWER_LOW; const bool took = link.request_config(n, clock / 1000); link.tick(clock, ch); air.step(clock);
    CHECK(took && (chip.reg[nrf::RF_SETUP] & 0x06) == 0x02, "a new power only: at the next tick (the packet in the air first), nothing sent to the drone");
}

static void t_app() {
    printf("[tx app]\n");
    uint32_t clock = 0; Air air; SimNrf24 chip; SimBus bus{&chip, &clock}; air.attach(&chip);
    wbp::Settings s; wbp::defaults(s); nrf::Config any; FakeDrone d(air, &clock, any); d.listen_bind();
    tx::TxApp app; app.configure(s); CHECK(app.begin(bus) && app.period_us() == 10000, "radio found, a packet every 10 ms");
    uint8_t res[16]; uint16_t rl = 0; const uint8_t one = 1;
    CHECK(app.command(wbp::CMD_BIND, &one, 1, res, rl) == wbp::ERR_OK, "BIND 1 accepted");
    tx::StickInput in = centred(); uint8_t sec = 0;
    for (int i = 0; i < 60 && !sec; i++) { app.cycle(clock, in); air.step(clock); d.service(); clock += 10000; sec = app.take_autosave(); }
    CHECK(sec == wbp::SEC_TX_RADIO && s.tx_radio.bound == 1 && !memcmp(s.tx_radio.addr, d.cfg.addr, 5), "bound: tx_radio (address, bound) is written to flash by itself");
    for (int i = 0; i < 20; i++) { app.cycle(clock, in); air.step(clock); d.service(); clock += 10000; }
    uint8_t buf[64]; const uint16_t n = app.fill_stream(wbp::STR_TX, buf, sizeof(buf)); wbp::T_tx t; memcpy(&t, buf, sizeof(t));
    CHECK(n == sizeof(wbp::T_tx) && t.link && t.ch[0] == 1500 && t.ch[2] == 1000 && t.raw[3] == 2048 && t.bind_state == wbp::BIND_STATE_BOUND && t.d_seen && t.d_batt_cv == 1600, "stream tx: channels, raw readings, link, bound, the drone's battery");
    const uint8_t start = 1, fin = 2;
    CHECK(app.command(wbp::CMD_STICK_CAL, &start, 1, res, rl) == wbp::ERR_OK, "STICK_CAL 1");
    for (int a = 0; a < 4; a++) { in = centred(); in.raw[a] = 250; app.cycle(clock, in); clock += 10000; in.raw[a] = 3850; app.cycle(clock, in); clock += 10000; }
    in = centred(); app.cycle(clock, in); clock += 10000;
    CHECK(app.command(wbp::CMD_STICK_CAL, &fin, 1, res, rl) == wbp::ERR_OK && s.tx_sticks.cal_min[2] == 250 && s.tx_sticks.cal_max[2] == 3850 && app.take_autosave() == wbp::SEC_TX_STICKS, "STICK_CAL 2: the calibration in tx_sticks, kept in flash");
    s.tx_sticks.throttle_mode = wbp::TX_THROTTLE_ALTPOS; CHECK(app.apply_live(wbp::SEC_TX_STICKS), "tx_sticks is applied at once");
    app.cycle(clock, in); app.fill_stream(wbp::STR_TX, buf, sizeof(buf)); memcpy(&t, buf, sizeof(t)); CHECK(t.ch[2] == 1500, "... the throttle mode ALTPOS now (centre = 1500)");
    d.tel.armed = true; d.ack_tel(); for (int i = 0; i < 5; i++) { app.cycle(clock, in); air.step(clock); d.service(); clock += 10000; }
    CHECK(app.command(wbp::CMD_BIND, &one, 1, res, rl) == wbp::ERR_ARMED, "BIND refused while the drone reports it is armed");
}

struct VecTransport : wbp::Transport { std::vector<uint8_t> out; size_t write(const uint8_t* d, size_t n) override { out.insert(out.end(), d, d + n); return n; } };
static void t_report() {
    printf("[tx app: boot report]\n");
    wbp::Settings s; wbp::defaults(s); tx::TxApp app; app.configure(s); VecTransport t;
    wbp::Server srv(s, nullptr, app, t, wbp::ROLE_TX, 0, 38, 0, "Windburst TX", wbp::FEAT_STREAMS);
    app.set_server(&srv); app.add_report("radio: nRF24 NOT FOUND (pins SCK 7, MISO 6, MOSI 8, CSN 24, CE 23)"); app.add_report("sticks: ADC pins 4 5 3 2");
    uint8_t cmd = wbp::CMD_REPORT, f[16]; const size_t n = wbp::encode(f, sizeof(f), wbp::MSG_COMMAND, 3, &cmd, 1); srv.on_bytes(f, n);
    wbp::Parser p; wbp::Frame fr; int logs = 0; bool radio = false;
    for (uint8_t c : t.out) if (p.feed(c, fr)) { if (fr.type == wbp::MSG_LOG) { logs++; if (std::string((const char*)fr.payload + 1, fr.len - 1).find("NOT FOUND") != std::string::npos) radio = true; } }
    CHECK(logs == 2 && radio && app.report_lines() == 2, "REPORT: the transmitter's boot lines as log frames (%d)", logs);
    srv.settings_loaded(); uint8_t b[64]; wbp::T_tx ts; app.fill_stream(wbp::STR_TX, b, sizeof(b)); memcpy(&ts, b, sizeof(ts)); const uint8_t clean = ts.settings_dirty;
    std::vector<uint8_t> set(1 + sizeof(wbp::S_tx_sticks)); set[0] = wbp::SEC_TX_STICKS; wbp::S_tx_sticks k = s.tx_sticks; k.deadband = 9; memcpy(set.data() + 1, &k, sizeof(k));
    uint8_t fb[128]; const size_t m = wbp::encode(fb, sizeof(fb), wbp::MSG_SECTION_SET, 4, set.data(), (uint16_t)set.size()); srv.on_bytes(fb, m);
    app.fill_stream(wbp::STR_TX, b, sizeof(b)); memcpy(&ts, b, sizeof(ts));
    CHECK(clean == 0 && (ts.settings_dirty & wbp::SETTINGS_FLAGS_DIRTY), "stream tx: the server's unsaved flag (the transmitter has no status stream) (%u -> %u)", clean, ts.settings_dirty);
}

// ---- final review (0.38.0): findings C1, C2, I1, I2, I3, I5, each failed before its fix
static void t_review() {
    printf("[review findings]\n");
    { // I2: a stick that rests a little off its centre does not creep the MANUAL throttle up
      tx::Sticks m; wbp::S_tx_sticks c = stick_defaults(); m.configure(c); uint16_t ch[8]; tx::StickInput in = centred();
      in.raw[3] = 4095; for (uint32_t t = 0; t <= 1000; t += 10) m.update(in, tx::ARMED, t, ch); const uint16_t up = ch[2];
      in.raw[3] = 2250; for (uint32_t t = 1010; t <= 4000; t += 10) m.update(in, tx::ARMED, t, ch);
      CHECK(abs((int)ch[2] - (int)up) <= 2, "throttle stick resting 200 counts high for 3 s: the throttle stays (%u -> %u)", up, ch[2]); }
    { // I1: telemetry missing after the drone was armed: the throttle is held, not dropped to 1000 and not integrated
      tx::Sticks m; wbp::S_tx_sticks c = stick_defaults(); m.configure(c); uint16_t ch[8]; tx::StickInput in = centred();
      in.raw[3] = 4095; for (uint32_t t = 0; t <= 1000; t += 10) m.update(in, tx::ARMED, t, ch); const uint16_t up = ch[2];
      for (uint32_t t = 1010; t <= 3000; t += 10) m.update(in, tx::UNKNOWN, t, ch);
      const uint16_t held = ch[2]; m.update(centred(), tx::ARMED, 3010, ch);
      CHECK(held == up && ch[2] == up, "2 s without telemetry: throttle held at %u (%u), then it goes on from there (%u)", up, held, ch[2]);
      m.update(centred(), tx::DISARMED, 3020, ch); CHECK(ch[2] == 1000, "a drone that says it is disarmed: 1000"); }
    { // C1: the microsecond clock wraps after 71.6 min: nothing changes in flight
      uint32_t clock = 0xFFF00000u; Air air; SimNrf24 chip; SimBus bus{&chip, &clock}; air.attach(&chip);
      wbp::Settings s; wbp::defaults(s); s.tx_radio = bound_radio(0xC7); FakeDrone d(air, &clock, cfg_of(s.tx_radio)); d.tel.armed = true; d.ack_tel();
      tx::TxApp app; app.configure(s); app.begin(bus); tx::StickInput in = centred(); in.raw[3] = 4095;
      for (int i = 0; i < 100; i++) { app.cycle(clock, in); air.step(clock); d.service(); clock += 10000; }
      in.raw[3] = 2048; uint16_t lo = 2000, hi = 0; int blinks = 0; uint8_t b[64]; wbp::T_tx st;
      for (int i = 0; i < 200; i++) {
          app.cycle(clock, in); air.step(clock); d.service(); clock += 10000; const uint16_t t = app.channels()[2]; if (t < lo) lo = t; if (t > hi) hi = t;
          app.fill_stream(wbp::STR_TX, b, sizeof(b)); memcpy(&st, b, sizeof(st)); if (!st.link || !st.d_seen) blinks++;
      }
      CHECK(lo > 1400 && hi - lo <= 2, "through the wrap of micros(): the throttle stays where it was (%u .. %u)", lo, hi);
      CHECK(blinks == 0, "... and the link / the drone's telemetry never blink off at the wrap (%d cycles)", blinks); }
    { // ... and a time limit that runs across the wrap: a bind started 1 s before it gives up after its 30 s, not 72 minutes later
      uint32_t clock = 0xFFF0BDC0u; Air air; SimNrf24 chip; SimBus bus{&chip, &clock}; air.attach(&chip); wbp::S_tx_radio r = bound_radio(0xCB);
      nrf::Radio radio; radio.begin(bus, true, cfg_of(r)); tx::TxLink link; link.begin(radio, r); uint16_t ch[8] = {};
      link.tick(clock, ch); link.start_bind(1, 2, clock); for (int i = 0; i < 3200; i++) { link.tick(clock, ch); air.step(clock); clock += 10000; }
      CHECK(link.bind_state() == wbp::BIND_STATE_FAILED, "bind started just before the wrap of micros(): given up after 30 s (state %u)", link.bind_state()); }
    { // C2: never bound: no control packets at all (a fixed default address would let two unbound sets fly each other)
      uint32_t clock = 0; Air air; SimNrf24 chip; SimBus bus{&chip, &clock}; air.attach(&chip); wbp::S_tx_radio r; wbp::defaults(r);
      FakeDrone d(air, &clock, cfg_of(r)); nrf::Radio radio; radio.begin(bus, true, cfg_of(r)); tx::TxLink link; link.begin(radio, r);
      uint16_t ch[8] = {1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000};
      for (int i = 0; i < 50; i++) { link.tick(clock, ch); air.step(clock); d.service(); clock += 10000; }
      CHECK(d.controls == 0 && link.bind_state() == wbp::BIND_STATE_IDLE, "an unbound transmitter sends no control packets (%u)", d.controls); }
    { // I3: the transmitter is "armed" while its drone is; settings, calibration and restarts wait
      uint32_t clock = 0; Air air; SimNrf24 chip; SimBus bus{&chip, &clock}; air.attach(&chip);
      wbp::Settings s; wbp::defaults(s); s.tx_radio = bound_radio(0xC8); FakeDrone d(air, &clock, cfg_of(s.tx_radio)); d.tel.armed = true; d.ack_tel();
      tx::TxApp app; app.configure(s); app.begin(bus); tx::StickInput in = centred();
      for (int i = 0; i < 20; i++) { app.cycle(clock, in); air.step(clock); d.service(); clock += 10000; }
      uint8_t res[8]; uint16_t rl; const uint8_t one = 1;
      CHECK(app.armed() && app.command(wbp::CMD_STICK_CAL, &one, 1, res, rl) == wbp::ERR_ARMED, "drone armed: the transmitter counts as armed (the server refuses settings / restarts), STICK_CAL refused");
      air.detach(&d.chip); for (int i = 0; i < 200; i++) { app.cycle(clock, in); air.step(clock); clock += 10000; }
      CHECK(app.armed(), "drone gone while it was armed: still treated as armed (it may be flying)"); }
    { // I3: radio settings only with the drone there and disarmed; otherwise the values go back at once; confirmed ones are kept in flash
      uint32_t clock = 0; Air air; SimNrf24 chip; SimBus bus{&chip, &clock}; air.attach(&chip);
      wbp::Settings s; wbp::defaults(s); s.tx_radio = bound_radio(0xC9); tx::TxApp app; app.configure(s); app.begin(bus); tx::StickInput in = centred();
      for (int i = 0; i < 20; i++) { app.cycle(clock, in); air.step(clock); clock += 10000; }
      s.tx_radio.rf_channel = 60; const bool ok = app.apply_live(wbp::SEC_TX_RADIO);
      CHECK(ok && s.tx_radio.rf_channel == 76 && chip.reg[nrf::RF_CH] == 76, "no drone heard: a new RF channel is put back at once (nothing waits for a reboot) (%u)", s.tx_radio.rf_channel);
      FakeDrone d(air, &clock, cfg_of(s.tx_radio)); for (int i = 0; i < 20; i++) { app.cycle(clock, in); air.step(clock); d.service(); clock += 10000; }
      s.tx_radio.rf_channel = 60; app.apply_live(wbp::SEC_TX_RADIO); uint8_t sec = 0;
      for (int i = 0; i < 200 && !sec; i++) { app.cycle(clock, in); air.step(clock); d.service(); clock += 10000; sec = app.take_autosave(); }
      CHECK(sec == wbp::SEC_TX_RADIO && s.tx_radio.rf_channel == 60 && d.cfg.rf_channel == 60, "drone there and disarmed: both move, and the transmitter keeps it in flash (%u)", sec);
      s.tx_radio.bound = 0; memset(s.tx_radio.addr, 0, 5); app.apply_live(wbp::SEC_TX_RADIO); const uint32_t before = d.controls;
      for (int i = 0; i < 30; i++) { app.cycle(clock, in); air.step(clock); d.service(); clock += 10000; }
      CHECK(d.controls == before, "defaults (the pair forgotten): the old drone gets nothing more"); }
    { // I5: the radio loses its registers (a brown-out of the module): found and set up again
      uint32_t clock = 0; Air air; SimNrf24 chip; SimBus bus{&chip, &clock}; air.attach(&chip); wbp::S_tx_radio r = bound_radio(0xCA);
      FakeDrone d(air, &clock, cfg_of(r)); nrf::Radio radio; radio.begin(bus, true, cfg_of(r)); tx::TxLink link; link.begin(radio, r);
      uint16_t ch[8] = {1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000};
      for (int i = 0; i < 20; i++) { link.tick(clock, ch); air.step(clock); d.service(); clock += 10000; }
      chip = SimNrf24(); const uint32_t before = d.controls;
      for (int i = 0; i < 200; i++) { link.tick(clock, ch); air.step(clock); d.service(); clock += 10000; }
      CHECK(d.controls > before + 150 && link.stats().link && radio.reinits() >= 1, "chip reset: set up again within a second, the link back (%u packets, %u re-inits)", d.controls - before, radio.reinits()); }
}

static void t_minor() {
    printf("[link: review minors]\n");
    { // M3: the drone echoes the offer but never moves (its echo was ACKed by a copy): the transmitter does not call it bound
      uint32_t clock = 0; Air air; SimNrf24 chip; SimBus bus{&chip, &clock}; air.attach(&chip); wbp::S_tx_radio r; wbp::defaults(r);
      nrf::Config any; FakeDrone d(air, &clock, any); d.listen_bind(); d.never_move = true; nrf::Radio radio; radio.begin(bus, true, cfg_of(r)); tx::TxLink link; link.begin(radio, r);
      link.start_bind(0x11223344u, 0x55u, clock); uint16_t ch[8] = {1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000}; uint8_t a[5]; bool bound = false;
      bool moved = false, back = false;
      for (int i = 0; i < 400; i++) {
          link.tick(clock, ch); air.step(clock); d.service(); clock += 10000; bound |= link.take_bound(a);
          if (i < 100 && chip.reg[nrf::RF_CH] != wbr::kBindChannel) moved = true;
          if (i > 150 && chip.reg[nrf::RF_CH] == wbr::kBindChannel) back = true;      // (until ~tick 202 it is on the new address)
      }
      CHECK(moved && back && !bound && link.bind_state() == wbp::BIND_STATE_BINDING, "echo seen: it tries the new address; nothing answers there for 2 s: back to offering on the bind channel, never BOUND (moved %d back %d bound %d state %u)", moved, back, bound, link.bind_state()); }
    { // M4: every bind picks an RF channel inside the 2.4 GHz band and between WiFi 11 and the band edge (74 .. 82)
      bool ok = true; int distinct = 0, last = -1;
      for (uint32_t k = 0; k < 20; k++) {
          uint32_t clock = 0; Air air; SimNrf24 chip; SimBus bus{&chip, &clock}; air.attach(&chip); wbp::S_tx_radio r; wbp::defaults(r);
          nrf::Config any; FakeDrone d(air, &clock, any); d.listen_bind(); nrf::Radio radio; radio.begin(bus, true, cfg_of(r)); tx::TxLink link; link.begin(radio, r);
          link.start_bind(0x1000u + k * 7919u, k * 104729u, clock); uint16_t ch[8] = {}; uint8_t a[5]; bool b = false;
          for (int i = 0; i < 100 && !b; i++) { link.tick(clock, ch); air.step(clock); d.service(); clock += 10000; b = link.take_bound(a); }
          const int c = link.applied().rf_channel; if (!b || c < 74 || c > 82 || d.cfg.rf_channel != c) ok = false; if (c != last) distinct++; last = c;
      }
      wbp::S_tx_radio r; wbp::defaults(r);
      CHECK(ok && distinct > 3 && r.rf_channel == 76 && wbr::kBindChannel == 83, "binds land on channels 74 .. 82 (both ends), not always the same one; default 76, bind channel 83"); }
    { // M6: no register is written while a packet is on its way (start / stop of a bind, a new power)
      uint32_t clock = 0; Air air; SimNrf24 chip; SimBus bus{&chip, &clock}; air.attach(&chip); wbp::S_tx_radio r = bound_radio(0xCC);
      FakeDrone d(air, &clock, cfg_of(r)); nrf::Radio radio; radio.begin(bus, true, cfg_of(r)); tx::TxLink link; link.begin(radio, r); uint16_t ch[8] = {};
      for (int i = 0; i < 5; i++) { link.tick(clock, ch); air.step(clock); d.service(); clock += 10000; }
      link.tick(clock, ch); link.start_bind(1, 2, clock); air.step(clock); clock += 10000;
      link.tick(clock, ch); link.stop_bind(); air.step(clock); clock += 10000;
      link.tick(clock, ch); wbp::S_tx_radio lo = r; lo.power = wbp::NRF_POWER_LOW; link.request_config(lo, clock / 1000); air.step(clock); clock += 10000;
      for (int i = 0; i < 5; i++) { link.tick(clock, ch); air.step(clock); d.service(); clock += 10000; }
      CHECK(chip.writes_while_tx == 0 && (chip.reg[nrf::RF_SETUP] & 0x06) == 0x02, "settings wait until the packet in the air is done (%u early writes), and then take effect", chip.writes_while_tx); }
}

int main() { t_sticks(); t_link(); t_link_fallback(); t_app(); t_report(); t_review(); t_minor(); printf("\n%d passed, %d failed\n", g_pass, g_fail); return g_fail ? 1 : 0; }
