// ESC passthrough: the 4-way interface (src/hal/motor/esc4way.h) against a simulated BLHeli bootloader on the motor wire, and the
// MSP requests the ESC tools send first (src/app/msp_bridge.h).
//
// The simulated ESC sees only what is really on its wire: the flight controller's pin levels with their times (it decodes them
// as a UART at its own bit time, which can be off by a few percent like an ESC's RC oscillator), and it answers by driving the
// line itself after a turn-around pause. Time is virtual (microseconds); every port call costs a little of it.
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <vector>
#include <deque>
#include <string>
#include "../src/hal/motor/esc4way.h"
#include "../src/app/msp_bridge.h"

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, ...) do { if (cond) g_pass++; else { g_fail++; printf("  FAIL line %d: %s   ", __LINE__, #cond); printf(__VA_ARGS__); printf("\n"); } } while (0)

using namespace esc4way;

struct Edge { double t; bool hi; };

// ---------------------------------------------------------------- a BLHeli bootloader (BLHeli_32 / AM32 style: ARM, 1 kB pages)
struct SimEsc {
    bool present = true;              // false: no bootloader answers (an ESC without power, a wrong pin)
    bool in_boot = false;             // BootInit seen: in the bootloader, frames carry a CRC
    double bit_scale = 1.0;           // its bit time / 52 us (its oscillator)
    double turnaround_us = 40;        // how long after the line is let go its answer starts
    uint8_t sig_hi = 0x1F, sig_lo = 0x06;
    std::vector<uint8_t> flash = std::vector<uint8_t>(0x10000, 0xFF);
    uint32_t addr = 0; int expect_buffer = -1; std::vector<uint8_t> buffer;
    int frames = 0, crc_errors = 0, runs = 0, resets_low = 0;
    std::vector<Edge> tx;             // what it drives (empty = it lets the pull-up hold the line high)
    double tx_end = 0;

    double bit() const { return 52.0 * bit_scale; }
    bool level_at(double t) const {
        if (tx.empty() || t >= tx_end || t < tx.front().t) return true;
        bool v = true; for (const Edge& e : tx) { if (e.t > t) break; v = e.hi; } return v;
    }
    void send(double t0, const std::vector<uint8_t>& bytes) {
        tx.clear(); double t = t0;
        for (uint8_t b : bytes) {
            tx.push_back({t, false}); t += bit();                                   // start bit
            for (int i = 0; i < 8; i++) { tx.push_back({t, ((b >> i) & 1) != 0}); t += bit(); }
            tx.push_back({t, true}); t += bit();                                    // stop bit
            t += bit() * 0.3;                                                       // a little gap between bytes
        }
        tx_end = t;
    }
    static uint16_t crc(const uint8_t* d, size_t n) { uint16_t c = 0; for (size_t i = 0; i < n; i++) c = crc_bl(c, d[i]); return c; }
    // a whole frame the FC sent (from esc_output to esc_input), decoded; `now` = when the FC let go of the line
    void on_frame(const std::vector<uint8_t>& b, double now, double held_low_us) {
        frames++;
        if (held_low_us > 250000) { in_boot = false; resets_low++; return; }        // the line held low 300 ms: a reset
        if (!present) return;
        const double t0 = now + turnaround_us;
        auto ack = [&](uint8_t a, double extra_us = 0) { send(t0 + extra_us, {a}); };
        if (!in_boot) {
            static const uint8_t init[17] = {0, 0, 0, 0, 0, 0, 0, 0, 0x0D, 'B', 'L', 'H', 'e', 'l', 'i', 0xF4, 0x7D};
            if (b.size() >= 17 && memcmp(b.data() + b.size() - 17, init, 17) == 0) { in_boot = true; send(t0, {'4', '7', '1', 'c', sig_hi, sig_lo, 6, 4, BR_SUCCESS}); }
            return;
        }
        if (b.size() < 3 || crc(b.data(), b.size() - 2) != (uint16_t)(b[b.size() - 2] | b[b.size() - 1] << 8)) { crc_errors++; ack(BR_ERROR_CRC); return; }
        const size_t n = b.size() - 2;
        if (expect_buffer > 0) {
            if ((int)n != expect_buffer) { ack(BR_ERROR_COMMAND); expect_buffer = -1; return; }
            buffer.assign(b.begin(), b.begin() + n); expect_buffer = -1; ack(BR_SUCCESS); return;
        }
        switch (b[0]) {
        case BL_SET_ADDRESS: addr = (uint32_t)(b[2] << 8 | b[3]); ack(BR_SUCCESS); break;
        case BL_SET_BUFFER: expect_buffer = (b[2] << 8) | b[3]; break;              // no answer: the data follows
        case BL_READ_FLASH_SIL: { const int len = b[1] ? b[1] : 256; std::vector<uint8_t> r(flash.begin() + addr, flash.begin() + addr + len);
            const uint16_t c = crc(r.data(), r.size()); r.push_back((uint8_t)c); r.push_back((uint8_t)(c >> 8)); r.push_back(BR_SUCCESS); send(t0, r); break; }
        case BL_PROG_FLASH: for (size_t i = 0; i < buffer.size(); i++) flash[addr + i] = buffer[i]; ack(BR_SUCCESS, 3000); break;
        case BL_ERASE_FLASH: for (int i = 0; i < 1024; i++) flash[(addr & ~1023u) + i] = 0xFF; ack(BR_SUCCESS, 25000); break;
        case BL_VERIFY_FLASH_ARM: ack(memcmp(flash.data() + addr, buffer.data(), buffer.size()) == 0 ? BR_SUCCESS : BR_ERROR_VERIFY, 500); break;
        case BL_KEEP_ALIVE: ack(BR_ERROR_COMMAND); break;
        case BL_RUN: in_boot = false; runs++; break;
        default: ack(BR_ERROR_COMMAND); break;
        }
    }
};

// ---------------------------------------------------------------- the board side: virtual time, the PC link, the motor pins
struct VPort {
    double now = 0;
    std::deque<uint8_t> in; std::vector<uint8_t> out; bool hang_up_when_empty = true;
    struct Pin { bool driving = false, out_hi = true; std::vector<Edge> edges; double out_since = 0; SimEsc* esc = nullptr; };
    Pin pin[4]; int locks = 0; double max_lock_us = 0, lock_t = 0;

    uint32_t micros() { now += 0.4; return (uint32_t)now; }
    int host_read() { if (in.empty()) return -1; const int c = in.front(); in.pop_front(); return c; }
    void host_write(const uint8_t* d, size_t n) { out.insert(out.end(), d, d + n); }
    void host_flush() {}
    void host_wait() { now += 1000; }
    bool host_alive() { return !(hang_up_when_empty && in.empty()); }
    void lock() { locks++; lock_t = now; }
    void unlock() { if (now - lock_t > max_lock_us) max_lock_us = now - lock_t; }
    void esc_output(int c) { Pin& p = pin[c]; p.driving = true; p.edges.clear(); p.edges.push_back({now, p.out_hi}); p.out_since = now; }
    void esc_hi(int c) { set(c, true); }
    void esc_lo(int c) { set(c, false); }
    void set(int c, bool v) { Pin& p = pin[c]; p.out_hi = v; if (p.driving) p.edges.push_back({now, v}); }
    bool esc_is_hi(int c) { const Pin& p = pin[c]; if (p.driving) return p.out_hi; return p.esc ? p.esc->level_at(now) : true; }
    void esc_input(int c) {
        Pin& p = pin[c]; if (!p.driving) return; p.driving = false;
        if (!p.esc) return;
        // the longest stretch the line was held low (a reset), and the bytes as the ESC's UART sees them
        double low = 0; for (size_t i = 0; i < p.edges.size(); i++) if (!p.edges[i].hi) { const double e = i + 1 < p.edges.size() ? p.edges[i + 1].t : now; if (e - p.edges[i].t > low) low = e - p.edges[i].t; }
        p.esc->on_frame(decode(p, p.esc->bit()), now, low);
    }
    static bool level(const Pin& p, double t, double end) { if (t >= end) return true; bool v = true; for (const Edge& e : p.edges) { if (e.t > t) break; v = e.hi; } return v; }
    std::vector<uint8_t> decode(const Pin& p, double bit) const {
        std::vector<uint8_t> r; const double end = now; double t = p.out_since;
        for (;;) {
            double ts = -1; bool prev = true;                                        // the next falling edge
            for (const Edge& e : p.edges) { if (e.t < t) { prev = e.hi; continue; } if (prev && !e.hi) { ts = e.t; break; } prev = e.hi; }
            if (ts < 0) break;
            uint8_t b = 0; for (int i = 0; i < 8; i++) if (level(p, ts + bit * (1.5 + i), end)) b |= (uint8_t)(1u << i);
            if (!level(p, ts + bit * 9.5, end)) break;                               // a framing error: the ESC sees garbage from here
            r.push_back(b); t = ts + bit * 9.6;
        }
        return r;
    }
};

// ---------------------------------------------------------------- PC side of the 4-way interface
static std::vector<uint8_t> frame4(uint8_t cmd, uint16_t addr, const std::vector<uint8_t>& param, bool bad_crc = false) {
    std::vector<uint8_t> f = {CMD_LOCAL_ESCAPE, cmd, (uint8_t)(addr >> 8), (uint8_t)addr, (uint8_t)(param.size() & 0xFF)};
    f.insert(f.end(), param.begin(), param.end());
    uint16_t c = 0; for (uint8_t b : f) c = crc_xmodem(c, b); if (bad_crc) c ^= 1;
    f.push_back((uint8_t)(c >> 8)); f.push_back((uint8_t)c); return f;
}
struct Reply { bool ok = false; uint8_t cmd = 0, ack = 0xFF; uint16_t addr = 0; std::vector<uint8_t> data; };
static Reply parse(const std::vector<uint8_t>& o) {
    Reply r; if (o.size() < 8 || o[0] != CMD_REMOTE_ESCAPE) return r;
    const size_t n = o[4] ? o[4] : 256; if (o.size() != 5 + n + 3) return r;
    uint16_t c = 0; for (size_t i = 0; i < 5 + n + 1; i++) c = crc_xmodem(c, o[i]);
    r.ok = (uint16_t)(o[5 + n + 1] << 8 | o[5 + n + 2]) == c; r.cmd = o[1]; r.addr = (uint16_t)(o[2] << 8 | o[3]);
    r.data.assign(o.begin() + 5, o.begin() + 5 + n); r.ack = o[5 + n]; return r;
}
struct Bench {
    VPort port; SimEsc esc[4]; Interface<VPort> itf{port, 4};
    Bench() { for (int i = 0; i < 4; i++) port.pin[i].esc = &esc[i]; itf.begin(); }
    Reply ask(uint8_t cmd, uint16_t addr, const std::vector<uint8_t>& param, bool bad_crc = false, bool* exited = nullptr) {
        port.out.clear(); for (uint8_t b : frame4(cmd, addr, param, bad_crc)) port.in.push_back(b);
        const bool ex = itf.run(); if (exited) *exited = ex;
        return parse(port.out);
    }
};

static void t_interface() {
    printf("[4-way] the interface itself\n");
    Bench b;
    Reply r = b.ask(CMD_INTERFACE_GET_NAME, 0, {0});
    CHECK(r.ok && r.ack == ACK_OK && std::string(r.data.begin(), r.data.end()) == "m4wFCIntf", "name '%s' - what BLHeliSuite looks for", std::string(r.data.begin(), r.data.end()).c_str());
    r = b.ask(CMD_INTERFACE_GET_VERSION, 0, {0}); CHECK(r.ok && r.data.size() == 2 && r.data[0] == 200 && r.data[1] == 6, "interface version 20.0.06, as Betaflight");
    r = b.ask(CMD_PROTOCOL_GET_VERSION, 0, {0}); CHECK(r.ok && r.data.size() == 1 && r.data[0] == 108, "protocol 108");
    r = b.ask(CMD_TEST_ALIVE, 0, {0}); CHECK(r.ok && r.ack == ACK_OK && b.esc[0].frames == 0, "test alive, nothing connected: the ESC wire is not touched");
    r = b.ask(CMD_INTERFACE_GET_NAME, 0, {0}, true); CHECK(r.ok && r.ack == ACK_I_INVALID_CRC && b.itf.stats().bad_crc == 1, "a frame with a broken CRC is answered with INVALID_CRC, not executed");
    r = b.ask(0x55, 0, {0}); CHECK(r.ok && r.ack == ACK_I_INVALID_CMD, "unknown command");
    r = b.ask(CMD_INTERFACE_SET_MODE, 0, {3}); CHECK(r.ack == ACK_I_INVALID_PARAM, "the STK500 'SK' mode is not built in: INVALID_PARAM");
    r = b.ask(CMD_DEVICE_INIT_FLASH, 0, {4}); CHECK(r.ack == ACK_I_INVALID_CHANNEL, "ESC #5 of 4: INVALID_CHANNEL");
    { b.port.in.clear(); for (uint8_t c : {0x11, 0x22, 0x2E, 0x99}) b.port.in.push_back(c);           // noise before a frame: skipped up to the 0x2F
      for (uint8_t c : frame4(CMD_PROTOCOL_GET_VERSION, 0, {0})) b.port.in.push_back(c);
      b.port.out.clear(); b.itf.run();
      CHECK(parse(b.port.out).ok && parse(b.port.out).data[0] == 108, "bytes before the escape byte are ignored"); }
    bool ex = false; r = b.ask(CMD_INTERFACE_EXIT, 0, {0}, false, &ex); CHECK(r.ok && r.ack == ACK_OK && ex, "EXIT: answered, then run() returns true (the firmware restarts)");
    { b.port.in.clear(); b.port.out.clear(); const bool e2 = b.itf.run(); CHECK(!e2 && b.port.out.empty(), "the link is gone: run() returns false, nothing is sent"); }
}

static void t_bootloader() {
    printf("[4-way] the BLHeli bootloader on the wire (simulated ARM ESC)\n");
    for (double scale : {1.0, 0.98, 1.03}) {
        Bench b; for (auto& e : b.esc) e.bit_scale = scale;
        Reply r = b.ask(CMD_DEVICE_INIT_FLASH, 0, {0});
        CHECK(r.ok && r.ack == ACK_OK && r.data.size() == 4 && r.data[0] == 0x06 && r.data[1] == 0x1F && r.data[2] == 'c' && r.data[3] == MODE_ARM_BLB && b.esc[0].in_boot,
              "ESC clock %+.0f %%: connected, device info 06 1F 63 04 (signature 0x1F06 = ARM, BLHeli_32 / AM32; byte 3 is the 4th letter of the boot message, as in Betaflight): %02X %02X %02X %02X", (scale - 1) * 100, r.data.size() > 3 ? r.data[0] : 0, r.data.size() > 3 ? r.data[1] : 0, r.data.size() > 3 ? r.data[2] : 0, r.data.size() > 3 ? r.data[3] : 0);
        for (int i = 0; i < 256; i++) b.esc[0].flash[0x1000 + i] = (uint8_t)(i * 7 + 3);
        r = b.ask(CMD_DEVICE_READ, 0x1000, {0});
        bool same = r.data.size() == 256; for (int i = 0; same && i < 256; i++) same = r.data[i] == (uint8_t)(i * 7 + 3);
        CHECK(r.ok && r.ack == ACK_OK && same && r.addr == 0x1000, "read 256 bytes at 0x1000 (the settings page): all right, CRC checked");
        r = b.ask(CMD_DEVICE_READ, 0x1000, {16}); CHECK(r.ack == ACK_OK && r.data.size() == 16 && r.data[15] == (uint8_t)(15 * 7 + 3), "read 16 bytes");
        std::vector<uint8_t> blob(128); for (int i = 0; i < 128; i++) blob[i] = (uint8_t)(0xA0 ^ i);
        r = b.ask(CMD_DEVICE_WRITE, 0x1400, blob);
        CHECK(r.ack == ACK_OK && memcmp(b.esc[0].flash.data() + 0x1400, blob.data(), 128) == 0, "write 128 bytes at 0x1400: in the ESC's flash");
        r = b.ask(CMD_DEVICE_VERIFY, 0x1400, blob); CHECK(r.ack == ACK_OK, "verify: the same");
        blob[5] ^= 0xFF; r = b.ask(CMD_DEVICE_VERIFY, 0x1400, blob); CHECK(r.ack == ACK_I_VERIFY_ERROR, "verify with one byte different: VERIFY_ERROR");
        r = b.ask(CMD_DEVICE_PAGE_ERASE, 0, {5});
        bool erased = true; for (int i = 0; i < 1024; i++) erased &= b.esc[0].flash[0x1400 + i] == 0xFF;
        CHECK(r.ack == ACK_OK && erased && r.addr == 0x1400 && b.esc[0].flash[0x1000] == 3, "erase page 5: 0x1400 .. 0x17FF are 0xFF (ARM pages are 1 kB), the page before is kept");
        r = b.ask(CMD_TEST_ALIVE, 0, {0}); CHECK(r.ack == ACK_OK && b.esc[0].crc_errors == 0, "test alive while connected: the keep-alive is answered");
        CHECK(b.port.max_lock_us < 800, "the lock is never held longer than one byte on the wire: %.0f us", b.port.max_lock_us);
    }
    {   Bench b; b.ask(CMD_DEVICE_INIT_FLASH, 0, {0});
        Reply r = b.ask(CMD_DEVICE_INIT_FLASH, 0, {2});
        CHECK(r.ack == ACK_OK && b.itf.selected() == 2 && b.esc[2].in_boot && b.esc[0].in_boot, "ESC #3: connected on its own wire, #1 left alone");
        for (int i = 0; i < 16; i++) b.esc[2].flash[i] = 0x40 + i;
        r = b.ask(CMD_DEVICE_READ, 0, {16}); CHECK(r.data.size() == 16 && r.data[3] == 0x43 && b.esc[0].frames == 1, "and read from it, not from #1");
        r = b.ask(CMD_DEVICE_RESET, 0, {2}); CHECK(r.ack == ACK_OK && b.esc[2].runs == 1 && !b.esc[2].in_boot, "reset: the ESC is told to run its firmware");
        r = b.ask(CMD_DEVICE_INIT_FLASH, 0, {2}); r = b.ask(CMD_DEVICE_RESET, 1, {2});
        CHECK(r.ack == ACK_OK && b.esc[2].resets_low == 1, "reset with address 1: the line is also held low for 300 ms (a hard reset)"); }
    {   Bench b; b.esc[1].present = false;
        Reply r = b.ask(CMD_DEVICE_INIT_FLASH, 0, {1});
        CHECK(r.ack == ACK_D_GENERAL_ERROR && r.data.size() == 4 && r.data[0] == 0, "ESC #2 does not answer (no battery?): GENERAL_ERROR, device info zero - three tries");
        r = b.ask(CMD_DEVICE_READ, 0, {16}); CHECK(r.ack == ACK_D_GENERAL_ERROR, "reading from it: GENERAL_ERROR, nothing made up"); }
    {   Bench b; for (auto& e : b.esc) e.bit_scale = 1.12;
        Reply r = b.ask(CMD_DEVICE_INIT_FLASH, 0, {0}); CHECK(r.ack == ACK_D_GENERAL_ERROR, "an ESC 12 %% off the baud rate is not understood: an error, not garbage"); }
}

// The session record for the next boot report (esc4way::Trace): which request failed and what the ESC said
static void t_trace() {
    printf("[4-way] the session record for the boot report\n");
    Bench b; esc4way::Trace t; t.start(); b.itf.set_trace(&t); b.esc[1].present = false;
    b.ask(CMD_INTERFACE_GET_NAME, 0, {0}); b.ask(CMD_DEVICE_INIT_FLASH, 0, {0}); b.ask(CMD_DEVICE_READ, 0x7C00, {48}); b.ask(CMD_DEVICE_INIT_FLASH, 0, {1});
    char l[4][176]; int n = t.lines(l, 4); std::string all; for (int i = 0; i < n; i++) all += l[i];
    if (getenv("V")) for (int i = 0; i < n; i++) printf("      %s\n", l[i]);
    CHECK(t.valid() && n == 1 && all.find("4way: 4 requests, 1 ESC errors:") == 0, "a short session: one line, the count and the ESC errors ('%s')", n ? l[0] : "");
    CHECK(all.find("INIT#0 ok 061F6304") != std::string::npos && all.find("READ 7C00/48 ok FFFFFFFF") != std::string::npos && all.find("INIT#1 ESC-ERR") != std::string::npos,
          "each request: what was asked (command, address, bytes), the answer, the device info / the first bytes read");
    for (int i = 0; i < 40; i++) b.ask(CMD_DEVICE_READ, (uint16_t)(0x1000 + i * 16), {16});
    n = t.lines(l, 4); if (getenv("V")) for (int i = 0; i < n; i++) printf("      %s\n", l[i]); bool fits = true; for (int i = 0; i < n; i++) fits &= strlen(l[i]) < 176;
    std::string last = l[n - 1];
    CHECK(t.total == 44 && n >= 2 && n <= 4 && fits && (std::string(l[0]) + l[1]).find(" ...") != std::string::npos && last.find("READ 1270/16 ESC-ERR") != std::string::npos,
          "44 requests: the first 12 and the last 12 kept, a gap mark between them, at most 4 lines of 175 characters (%d lines)", n);
    { char two[4][176] = {}; CHECK(t.lines(two, 2) == 2 && two[2][0] == 0, "asked for 2 lines: 2 lines, nothing written past them (the boot report has room for 4)"); }
    t.clear(); CHECK(!t.valid() && t.lines(l, 4) == 0, "cleared after it went into the report: nothing the next time");
    esc4way::Trace junk; memset(&junk, 0xA5, sizeof(junk)); CHECK(!junk.valid(), "memory that was never written (RTC memory after power-on) is not a record");
}

static void t_msp() {
    printf("[MSP] what the ESC tools ask before the passthrough\n");
    msp::Bridge br; msp::Info in; in.motors = 4; in.batt_v = 16.4f; in.cells = 4; in.mah = 120; in.amps = 1.25f;
    uint8_t out[256]; size_t n = 0;
    auto v1 = [](uint8_t cmd, std::vector<uint8_t> p = {}) { std::vector<uint8_t> f = {'$', 'M', '<', (uint8_t)p.size(), cmd}; uint8_t ck = (uint8_t)(p.size() ^ cmd); for (uint8_t b : p) { f.push_back(b); ck ^= b; } f.push_back(ck); return f; };
    auto run = [&](const std::vector<uint8_t>& f) { msp::Bridge::Result r = msp::Bridge::NONE; for (uint8_t c : f) { msp::Bridge::Result x = br.feed(c, in, out, sizeof(out), n); if (x != msp::Bridge::NONE) r = x; } return r; };
    CHECK(run(v1(msp::API_VERSION)) == msp::Bridge::REPLY && n == 9 && out[2] == '>' && out[3] == 3 && out[4] == 1 && out[6] == 1 && out[7] == 49, "API_VERSION: $M> 1.49 (Betaflight master)");
    { uint8_t ck = 0; for (size_t i = 3; i < n - 1; i++) ck ^= out[i]; CHECK(ck == out[n - 1], "reply checksum"); }
    CHECK(run(v1(msp::FC_VARIANT)) == msp::Bridge::REPLY && memcmp(out + 5, "BTFL", 4) == 0, "FC_VARIANT BTFL (the tools only talk to firmwares they know)");
    CHECK(run(v1(msp::BATTERY_STATE)) == msp::Bridge::REPLY && out[5] == 4 && out[8] == 164 && out[9] == 120 && (out[11] | out[12] << 8) == 125 && (out[14] | out[15] << 8) == 1640, "BATTERY_STATE: 4 cells, 16.4 V, 120 mAh, 1.25 A");
    in.fw_major = 0; in.fw_minor = 10; in.fw_patch = 3;
    CHECK(run(v1(msp::BUILD_INFO)) == msp::Bridge::REPLY && out[3] == 26 + 2 * (int)(sizeof(msp::kBuildOptions) / 2) && memcmp(out + 5 + 19, "w01003x", 7) == 0, "BUILD_INFO: date, time, and the version as w + major + 2-digit minor + 2-digit patch");
    CHECK(run(v1(msp::MOTOR_CONFIG)) == msp::Bridge::REPLY && out[11] == 4, "MOTOR_CONFIG: 4 motors");
    { msp::Frame f; f.cmd = msp::FC_VERSION; n = br.reply(f, in, out, sizeof(out));
      CHECK(n == 9 && out[3] == 3, "FC_VERSION on the OSD / VTX port: 3 bytes, no version text (goggles read a fixed length)"); }
    CHECK(run(v1(msp::SET_MOTOR, {0xE8, 0x03, 0xE8, 0x03})) == msp::Bridge::REPLY && out[2] == '!' && out[3] == 0, "SET_MOTOR (spin a motor from the ESC tool): refused with $M!");
    CHECK(run(v1(77)) == msp::Bridge::REPLY && out[2] == '!', "an unknown request: $M!, as Betaflight");
    in.bf_motor_protocol = 7; in.pwm_rate = 480; in.idle_x100 = 550;
    CHECK(run(v1(msp::ADVANCED_CONFIG)) == msp::Bridge::REPLY && out[2] == '>' && out[3] == 20 && out[5 + 3] == 7 && (out[5 + 4] | out[5 + 5] << 8) == 480 && (out[5 + 6] | out[5 + 7] << 8) == 550,
          "ADVANCED_CONFIG (90, BLHeliSuite32 asks it - unanswered in 0.9.3, the tool gave up): Betaflight's 20 bytes, motor protocol DShot600");
    // the firmware's own state, in Betaflight's terms
    in.modes_set = (1u << 1) | (1u << 3) | (1u << 9) | (1u << 2);   // ANGLE, ALTHOLD, RESCUE, STABILIZE (Betaflight has no STABILIZE)
    CHECK(run(v1(msp::BOXIDS)) == msp::Bridge::REPLY && out[3] == 4 && out[5] == 0 && out[6] == 1 && out[7] == 3 && out[8] == 46,
          "BOXIDS (119): ARM always, then the modes that have a switch - ANGLE 1, ALTHOLD 3, GPS RESCUE 46 (STABILIZE has no Betaflight box)");
    in.armed = true; in.modes_on = (1u << 0) | (1u << 3); in.health = 1u | (1u << 2) | (1u << 4); in.cpu_pct = 37; in.cycle_us = 250;
    CHECK(run(v1(msp::STATUS)) == msp::Bridge::REPLY && out[3] == 27 && (out[9] | out[10] << 8) == (1 | 2 | 8 | 32) && out[11] == (1 | 4) && (out[16] | out[17] << 8) == 37 && (out[18] | out[19] << 8) == 0,
          "STATUS (master, 27 bytes): sensors ACC + GYRO + BARO + GPS, flight modes ARM + ALTHOLD (bits 0 and 2 of BOXIDS), load 37 %%, gyro cycle 0 (%u bytes)", (unsigned)out[3]);
    in.health |= 1u << 12;   // HEALTH_FLOW
    CHECK(run(v1(msp::STATUS)) == msp::Bridge::REPLY && (out[9] | out[10] << 8) == (1 | 2 | 8 | 32 | 64), "an optical flow sensor: sensor bit 6 (master)");
    in.health &= ~(1u << 12);
    { const uint32_t ms0 = in.modes_set, mo0 = in.modes_on;      // the switch modes of 0.23: ACRO_TRAINER 10, AIRMODE 11, ANTI_GRAVITY 12, BLACKBOX_ERASE 13
      in.modes_set = (1u << 10) | (1u << 11) | (1u << 12) | (1u << 13); in.modes_on = (1u << 10) | (1u << 12);
      CHECK(run(v1(msp::BOXIDS)) == msp::Bridge::REPLY && out[3] == 5 && out[5] == 0 && out[6] == 4 && out[7] == 28 && out[8] == 31 && out[9] == 47,
            "BOXIDS: ANTI GRAVITY 4, AIR MODE 28, BLACKBOX ERASE 31, ACRO TRAINER 47 (Betaflight's permanent ids)");
      CHECK(run(v1(msp::STATUS)) == msp::Bridge::REPLY && out[11] == (1 | 2 | 16), "STATUS: ARM + ANTI GRAVITY + ACRO TRAINER (bits 0, 1, 4 of BOXIDS) (0x%x)", out[11]);
      // 0.44: the later modes - MISSION (14) as Betaflight's AUTOPILOT 56, CRASHFLIP (16) as FLIP OVER AFTER CRASH 35; LOITER (15) and
      // AUTOTUNE (17) have no Betaflight box
      in.modes_set = (1u << 14) | (1u << 15) | (1u << 16) | (1u << 17);
      CHECK(run(v1(msp::BOXIDS)) == msp::Bridge::REPLY && out[3] == 3 && out[5] == 0 && out[6] == 35 && out[7] == 56,
            "BOXIDS: FLIP OVER AFTER CRASH 35, AUTOPILOT 56 (the mission); no box for Loiter and the autotune");
      in.modes_set = ms0; in.modes_on = mo0; }
    in.armed = false; in.blockers = (1u << 2) | (1u << 3) | (1u << 7);   // RX_LOST, THROTTLE_HIGH, BATT_LOW (none in Betaflight)
    CHECK(run(v1(msp::STATUS_EX)) == msp::Bridge::REPLY && out[3] == 27 && out[5 + 16] == 30 && (uint32_t)(out[22] | out[23] << 8 | out[24] << 16 | out[25] << 24) == ((1u << 2) | (1u << 7)),
          "STATUS_EX: why it will not arm, as Betaflight's flags (RX_FAILSAFE, THROTTLE)");
    CHECK(out[5 + 13] == 1 && out[5 + 14] == 0 && out[5 + 22] == 0 && out[5 + 23] == 0 && out[5 + 24] == 1 && out[5 + 25] == 1 && out[5 + 26] == 0,
          "STATUS_EX tail (1.46 / 1.48): one PID profile, rate profile 0; CPU temperature 0; one rate profile; one battery profile, the first");
    in.motor_out[0] = 0.5f; in.motor_out[1] = 1.2f;
    CHECK(run(v1(msp::MOTOR)) == msp::Bridge::REPLY && (out[5] | out[6] << 8) == 1000, "MOTOR disarmed: 1000");
    in.armed = true; CHECK(run(v1(msp::MOTOR)) == msp::Bridge::REPLY && (out[5] | out[6] << 8) == 1500 && (out[7] | out[8] << 8) == 2000 && (out[13] | out[14] << 8) == 0, "MOTOR armed: the outputs, 1000 .. 2000 (motors that do not exist: 0)");
    in.armed = false; in.rx_kind = 1; in.gps = true; in.airmode = true;
    CHECK(run(v1(msp::FEATURE_CONFIG)) == msp::Bridge::REPLY && (uint32_t)(out[5] | out[6] << 8 | out[7] << 16 | out[8] << 24) == ((1u << 0) | (1u << 7) | (1u << 22)), "FEATURE_CONFIG: RX_PPM, GPS, AIRMODE from the settings");
    in.pulse_armed = 1050; in.pole_count = 12; in.bidir = true;
    CHECK(run(v1(msp::MOTOR_CONFIG)) == msp::Bridge::REPLY && out[3] == 12 && (out[5] | out[6] << 8) == 0 && (out[7] | out[8] << 8) == 2000 && out[12] == 12 && out[13] == 1 && (out[15] | out[16] << 8) == 1960,
          "MOTOR_CONFIG (master): min throttle 0 (gone after 4.5), max, poles, bidirectional DShot from the settings, motor KV 1960 at the end (1.49)");
    { std::vector<uint8_t> f = v1(msp::API_VERSION); f.back() ^= 0x5A; CHECK(run(f) == msp::Bridge::NONE && br.parser().bad() == 1, "a broken checksum: no answer"); }
    { std::vector<uint8_t> noise = {0x7E, 0x24, 0x10, 0x24, 0x4D, 0x00}; std::vector<uint8_t> f = v1(msp::API_VERSION); noise.insert(noise.end(), f.begin(), f.end());
      CHECK(run(noise) == msp::Bridge::REPLY && out[7] == 49, "a frame right after noise that contained '$' and '$M'"); }
    {   std::vector<uint8_t> f = {'$', 'X', '<', 0, (uint8_t)msp::API_VERSION, 0, 0, 0}; uint8_t c = 0; for (size_t i = 3; i < f.size(); i++) c = msp::crc8_dvb_s2(c, f[i]); f.push_back(c);
        CHECK(run(f) == msp::Bridge::REPLY && out[1] == 'X' && out[2] == '>' && out[6] == 3 && out[10] == 49, "MSP v2 request: a v2 answer"); }
    CHECK(run(v1(msp::SET_PASSTHROUGH)) == msp::Bridge::PASSTHROUGH, "SET_PASSTHROUGH (legacy, no payload): the caller is asked to open the 4-way interface");
    { const size_t k = br.passthrough_reply(4, out, sizeof(out)); CHECK(k == 7 && out[3] == 1 && out[4] == msp::SET_PASSTHROUGH && out[5] == 4, "... and answers with the number of ESCs"); }
    CHECK(run(v1(msp::SET_PASSTHROUGH, {0xFF, 0})) == msp::Bridge::PASSTHROUGH, "SET_PASSTHROUGH mode 0xFF (4-way)");
    CHECK(run(v1(msp::SET_PASSTHROUGH, {0xFE, 0})) == msp::Bridge::REPLY && out[5] == 0, "serial passthrough (mode 0xFE): not offered, answered 0");
    in.armed = true; CHECK(run(v1(msp::SET_PASSTHROUGH)) == msp::Bridge::REPLY && out[5] == 0 && br.refused() >= 2, "ARMED: no passthrough, 0 ESCs");
}

int main() {
    t_trace();
    t_interface(); t_bootloader(); t_msp();
    printf("%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
