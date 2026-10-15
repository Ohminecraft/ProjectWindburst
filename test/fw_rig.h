// The complete RX firmware on a PC: virtual board + FlightApp + settings store + WBP server, plus a small frame-level client.
// Shared by test/test_fw.cpp (deterministic firmware-in-the-loop tests) and tools/wb_sim.cpp (simulator the app connects to).
// Single-threaded: where the ESP32 link task would sleep while the flight loop executes a request, the simulation simply advances.
#pragma once
#include "../src/config/version.h"
#include <map>
#include <memory>
#include <string>
#include <vector>
#include "virtual_board.h"
#include "sim_flash.h"
#include "sim_card.h"
#include "../src/blackbox/bb_header.h"
#include "../src/proto/wbp_server.h"
#include "../src/app/link_host.h"

struct MemStorage : wbp::Storage {
    std::map<std::string, std::vector<uint8_t>> kv; bool fail_write = false; int writes = 0;
    int read(const char* k, uint8_t* b, size_t cap) override { auto it = kv.find(k); if (it == kv.end()) return -1; const size_t n = it->second.size() < cap ? it->second.size() : cap; memcpy(b, it->second.data(), n); return (int)n; }
    bool write(const char* k, const uint8_t* d, size_t n) override { if (fail_write) return false; kv[k] = std::vector<uint8_t>(d, d + n); writes++; return true; }
};
struct MemTransport : wbp::Transport {
    std::vector<uint8_t> out; size_t room = 100000;
    size_t write(const uint8_t* d, size_t n) override { out.insert(out.end(), d, d + n); return n; }
    size_t writable() override { return room; }
};

inline bool g_rig_usb_link = false;      // this build's app link is the chip's USB socket (main.cpp: ARDUINO_USB_CDC_ON_BOOT)

// One "power cycle" of the firmware: everything main.cpp creates, in the same order.
inline app::LinkHost::NetInfo g_sim_net;                    // the link stream's WiFi fields of the simulated board (the radio itself is not simulated)
struct Firmware {
    wbp::Settings settings;
    wbp::SettingsStore store;
    wbp::SettingsStore::LoadReport load;
    FlightApp app;
    app::LinkHost host;
    wbp::Server server;
    bb::Service blackbox;
    bb::FirmwareHeader bb_header;
    VirtualBoard& board;
    bool began = false;
    bb::Lock no_lock;                    // one thread here: the "writer task" is a call from Rig::run()
    std::unique_ptr<bb::CardLog> card;
    std::vector<uint8_t> card_buffer;

    // logmem: the flash region of the blackbox (nullptr = a board whose partition table has none)
    // sdcard: the SD card socket (nullptr = a board without one)
    Firmware(VirtualBoard& b, wbp::Storage& st, wbp::Transport& tx, bb::BlockStore* logmem, bb::FileStore* sdcard = nullptr, uint32_t card_buffer_bytes = 512 * 1024)
        : store(st), host(app.link, &Firmware::wait, this, &Firmware::now), server(settings, &store, host, tx, wbp::ROLE_RX, WB_FW_MAJOR, WB_FW_MINOR, WB_FW_PATCH, "Windburst RX (sim)",
                                                                                 wbp::FEAT_STREAMS | wbp::FEAT_CALIBRATION | wbp::FEAT_MOTOR_TEST | wbp::FEAT_GPS | wbp::FEAT_BLACKBOX | wbp::FEAT_BATTERY | wbp::FEAT_BEEPER | wbp::FEAT_WIFI),
          blackbox(app.blackbox, tx), bb_header{&settings, "Windburst RX (sim)", WB_FW_MAJOR, WB_FW_MINOR, WB_FW_PATCH}, board(b) {
        load = store.load(settings);
        app.usb_link = g_rig_usb_link;
        app.can_port = &b.can_port;                              // (the virtual board's CAN controller: used only when the settings switch CAN on)
        app.uart0 = g_rig_usb_link ? &Serial0 : nullptr;       // as main.cpp: with the app on USB, UART0 is free for a sensor
        app.configure(settings);
        began = app.begin();
        { char line[128];
          if (logmem) {
              blackbox.attach_store(logmem);
              snprintf(line, sizeof(line), "blackbox: partition '%.16s' %lu kB, %d sessions, %lu kB used%s", logmem->name(), (unsigned long)(logmem->size() / 1024u), blackbox.log().sessions(),
                       (unsigned long)(blackbox.log().used() / 1024u), blackbox.log().needs_erase() ? " - holds other data: erase it in the app" : "");
          } else snprintf(line, sizeof(line), "blackbox: FAILED - no log partition (needs a data partition named wblog or spiffs)");
          app.add_report(line);
          if (sdcard) { card.reset(new bb::CardLog(*sdcard, no_lock)); card_buffer.assign(card_buffer_bytes, 0); card->set_buffer(card_buffer.data(), (uint32_t)card_buffer.size()); }
          if (settings.blackbox.device == bb::DEV_SDCARD && card) {              // as main.cpp: mounted at boot only when the settings select the card
              card->configure_device(true, settings.blackbox.sd_mode, settings.blackbox.sd_mhz);
              if (card->begin()) snprintf(line, sizeof(line), "blackbox: SD card '%s' %lu MB, %lu MB free, %d log files, buffer %lu kB", sdcard->name(), (unsigned long)(card->capacity_kb() / 1024u),
                                          (unsigned long)((card->capacity_kb() - card->used_kb()) / 1024u), card->sessions(), (unsigned long)(card->buffer_size() / 1024u));
              else snprintf(line, sizeof(line), "blackbox: SD card FAILED - %s (it is looked for again every 3 s)", card->why());
              app.add_report(line);
          }
          if (card) blackbox.attach_card(card.get()); }
        blackbox.set_header_source(&bb::FirmwareHeader::write, &bb_header);
        blackbox.set_loop(app.core.cfg.pid_hz(), app.motors.cfg().count, app.core.cfg.loop_hz);   // a record per control cycle at most
        host.set_blackbox(&blackbox, &settings);
        host.set_net([](app::LinkHost::NetInfo& n) { n = g_sim_net; });       // (what a WiFi wire would report: set by a test)
        host.attach(&server);
        host.set_report(app.report(), app.report_lines());
        for (int i = 0; i < app.report_lines(); i++) server.log(1, app.report()[i]);
    }
    // 1 ms of the link task's sleep = 1 ms of flight loop
    static void wait(void* self) { Firmware* f = static_cast<Firmware*>(self); for (int i = 0; i < 4; i++) f->board.step(f->app); }
    static uint32_t now() { return millis(); }
};

struct Rig {
    VirtualBoard board;
    MemStorage flash;
    MemFlash logmem{1024 * 1024};       // the blackbox partition (1 MB); survives a reboot like the real one
    bool has_logmem = true;
    MemCard sdcard;                     // the SD card: like the flash it keeps its contents over a reboot
    bool has_sdcard = true;
    uint32_t card_buffer_bytes = 512 * 1024;
    bool usb_link = false;              // this board talks to the app over the chip's USB socket: UART0 is free for a sensor (kept over reboots)
    uint32_t card_busy_until_us = 0; bool card_busy = false; uint64_t card_busy_total_us = 0; uint32_t card_longest_busy_us = 0;
    MemTransport tx;
    std::unique_ptr<Firmware> fw;
    int boots = 0;
    wbp::Parser parser; uint8_t seq = 1;
    struct Rx { uint8_t type, seq; std::vector<uint8_t> p; };
    std::vector<Rx> inbox;

    // Pins of the virtual board are stored like a user would do it once in the Pins tab.
    void provision() {
        wbp::Settings s; wbp::SettingsStore st(flash); st.load(s);
        VirtualBoard::wire(s);
        st.save(s);
    }
    void boot() {
        fw.reset();
        ESP.restarts = 0;
        logmem.stall = [this](uint32_t us) { board.stall(us); };
        // a card write blocks only the writer task: the time is noted, and that task is not run again before it has passed
        sdcard.busy = [this](uint32_t us) { card_busy_until_us = (card_busy ? card_busy_until_us : micros()) + us; card_busy = true; card_busy_total_us += us; if (us > card_longest_busy_us) card_longest_busy_us = us; };
        sdcard.unmount(); card_busy = false;
        { const bool was = g_rig_usb_link; g_rig_usb_link = was || usb_link;
          fw.reset(new Firmware(board, flash, tx, has_logmem ? &logmem : nullptr, has_sdcard ? &sdcard : nullptr, card_buffer_bytes));
          g_rig_usb_link = was; }
        boots++; last_ms_ = millis(); fw->host.set_boot_id(0x5EED0000u + (uint32_t)boots);
    }
    FlightApp& app() { return fw->app; }
    const app::Telemetry& tel() { return fw->app.telemetry(); }

    // Advance the whole system. The server ticks every millisecond like the link task; a firmware-requested restart reboots it.
    uint32_t step_us = 250;             // time between two calls of the firmware's loop (250 = the loop gets its 4 kHz; more = a board that is too slow)
    uint32_t streams_off_until_ms = 0;  // a link that stops sending its streams for a while (but still answers): the app must not call it a restart
    void run(double seconds) {
        const uint64_t n = (uint64_t)(seconds * 1e6 / step_us + 0.5);
        for (uint64_t i = 0; i < n; i++) {
            board.step(fw->app, step_us);
            const uint32_t ms = millis();
            if (ms != last_ms_) {
                last_ms_ = ms; if ((int32_t)(ms - streams_off_until_ms) >= 0) fw->server.tick(ms);
                // the SD writer task: a lower-priority task that runs whenever the card is not keeping it waiting
                if (card_busy && (int32_t)(micros() - card_busy_until_us) >= 0) card_busy = false;
                if (fw->card && !card_busy) fw->card->work(ms);
            }
            if (ESP.restarts) boot();
        }
    }

    // ---- frame-level client
    void collect() {
        wbp::Frame f;
        for (uint8_t c : tx.out) if (parser.feed(c, f)) { do inbox.push_back({f.type, f.seq, std::vector<uint8_t>(f.payload, f.payload + f.len)}); while (parser.pump(f)); }
        tx.out.clear();
    }
    void send_raw(uint8_t type, uint8_t sq, const std::vector<uint8_t>& p) {
        uint8_t buf[wbp::kMaxPayload + wbp::kOverhead];
        const size_t n = wbp::encode(buf, sizeof(buf), type, sq, p.data(), (uint16_t)p.size());
        fw->server.on_bytes(buf, n);
    }
    // Sends a request and returns the reply with the same sequence number (type 0 = none arrived).
    // Streams, log lines and blackbox frames number themselves: they are never taken for a reply.
    Rx request(uint8_t type, const std::vector<uint8_t>& p = {}) {
        const uint8_t sq = seq++; if (seq == 0) seq = 1;
        send_raw(type, sq, p);
        collect();
        for (size_t i = 0; i < inbox.size(); i++)
            if (inbox[i].seq == sq && inbox[i].type != wbp::MSG_STREAM && inbox[i].type != wbp::MSG_LOG && !(inbox[i].type >= bb::kMsgHeader && inbox[i].type <= bb::kMsgBlob)) { Rx r = inbox[i]; inbox.erase(inbox.begin() + (long)i); return r; }
        return {0, 0, {}};
    }
    struct CmdResult { wbp::Err err; std::vector<uint8_t> data; bool answered; };
    CmdResult command(uint8_t cmd, const std::vector<uint8_t>& args = {}) {
        std::vector<uint8_t> p; p.push_back(cmd); p.insert(p.end(), args.begin(), args.end());
        const Rx r = request(wbp::MSG_COMMAND, p);
        if (r.type != wbp::MSG_CMD_RESULT || r.p.size() < 2 || r.p[0] != cmd) return {wbp::ERR_FAILED, {}, false};
        return {(wbp::Err)r.p[1], std::vector<uint8_t>(r.p.begin() + 2, r.p.end()), true};
    }
    // ACK error code of a request that answers with ACK (0xFF if something else came back)
    uint8_t ack_of(const Rx& r) const { return r.type == wbp::MSG_ACK && r.p.size() == 3 ? r.p[2] : 0xFF; }
    bool get_section(uint8_t id, void* out, size_t size) {
        const Rx r = request(wbp::MSG_SECTION_GET, {id});
        if (r.type != wbp::MSG_SECTION || r.p.size() != size + 3 || r.p[0] != id) return false;
        memcpy(out, r.p.data() + 3, size); return true;
    }
    Rx set_section(uint8_t id, const void* data, size_t size) {
        std::vector<uint8_t> p; p.push_back(id); p.insert(p.end(), (const uint8_t*)data, (const uint8_t*)data + size);
        return request(wbp::MSG_SECTION_SET, p);
    }
    bool stream(uint8_t id, uint16_t hz) { return ack_of(request(wbp::MSG_STREAM_CFG, {id, (uint8_t)hz, (uint8_t)(hz >> 8)})) == wbp::ERR_OK; }
    // newest frame of a stream collected so far (payload after [id][t_ms])
    template <class T> bool latest(uint8_t id, T& out, uint32_t* t_ms = nullptr) {
        collect();
        for (size_t i = inbox.size(); i-- > 0;)
            if (inbox[i].type == wbp::MSG_STREAM && inbox[i].p.size() == sizeof(T) + 5 && inbox[i].p[0] == id) {
                memcpy(&out, inbox[i].p.data() + 5, sizeof(T));
                if (t_ms) memcpy(t_ms, inbox[i].p.data() + 1, 4);
                return true;
            }
        return false;
    }
    int count(uint8_t type, int stream_id = -1) { collect(); int n = 0; for (const Rx& r : inbox) if (r.type == type && (stream_id < 0 || (!r.p.empty() && r.p[0] == stream_id))) n++; return n; }
    std::vector<std::string> logs() { collect(); std::vector<std::string> v; for (const Rx& r : inbox) if (r.type == wbp::MSG_LOG && !r.p.empty()) v.push_back(std::string(r.p.begin() + 1, r.p.end())); return v; }
    void clear_inbox() { collect(); inbox.clear(); }

private:
    uint32_t last_ms_ = 0;
};
