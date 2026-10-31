// The command line (src/app/cli.h): every setting by name, in the app's units, back and forth; what is refused; dump / diff
// lines that paste back; the steps a change takes (wbp::Server).
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <map>
#include <vector>
#include "../src/proto/wbp_server.h"
#include "../src/proto/settings_store.h"
#include "../src/app/cli.h"
using namespace wbp;

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, ...) do { const bool ok_ = (cond); if (getenv("V")) { printf("    "); printf(__VA_ARGS__); printf("\n"); } if (ok_) g_pass++; else { g_fail++; printf("  FAIL line %d: %s   ", __LINE__, #cond); printf(__VA_ARGS__); printf("\n"); } } while (0)

struct MemStorage : Storage {
    std::map<std::string, std::vector<uint8_t>> kv; bool fail = false;
    int read(const char* k, uint8_t* b, size_t cap) override { auto it = kv.find(k); if (it == kv.end()) return -1; const size_t n = it->second.size() < cap ? it->second.size() : cap; memcpy(b, it->second.data(), n); return (int)n; }
    bool write(const char* k, const uint8_t* d, size_t n) override { if (fail) return false; kv[k] = std::vector<uint8_t>(d, d + n); return true; }
};
struct NullTx : Transport { size_t write(const uint8_t*, size_t n) override { return n; } size_t writable() override { return 100000; } };
struct Host1 : Host {
    bool is_armed = false, live = true, rebooted = false; std::vector<uint8_t> applied;
    bool armed() override { return is_armed; }
    uint32_t now_ms() override { return 0; }
    uint16_t fill_stream(uint8_t, uint8_t*, uint16_t) override { return 0; }
    Err command(uint8_t, const uint8_t*, uint16_t, uint8_t*, uint16_t& r) override { r = 0; return ERR_OK; }
    bool apply_live(uint8_t s) override { applied.push_back(s); return live && s != SEC_GENERAL; }
    void reboot() override { rebooted = true; }
};
struct Rig {
    Settings s; MemStorage mem; SettingsStore store{mem}; Host1 host; NullTx tx; Server srv{s, &store, host, tx, ROLE_RX, 0, 1, 0, "t", 0};
    std::string out; app::Cli cli{s, srv, [](const char* t, void* c) { static_cast<Rig*>(c)->out += t; }, this};
    Rig() { defaults(s); srv.settings_loaded(); }
    std::string run(const char* line) { out.clear(); cli.run(line); return out; }
    bool has(const char* text) const { return out.find(text) != std::string::npos; }
};

static void t_table() {
    printf("[field table] every setting, at its place\n");
    int total[32] = {}; bool ok = true;
    for (const FieldInfo& f : kFields) { const int sz = f.type == FT_U16 || f.type == FT_I16 ? 2 : f.type == FT_U32 || f.type == FT_I32 || f.type == FT_F32 ? 4 : 1; if (f.offset != total[f.section]) ok = false; total[f.section] += sz * f.count; }
    for (const SectionInfo& si : kSections) if (total[si.id] != si.size) { ok = false; printf("    %s: %d of %u bytes\n", si.name, total[si.id], si.size); }
    CHECK(ok, "the fields of each section follow each other and add up to its size (%d fields)", kFieldCount);
    { const FieldInfo* p = nullptr; const FieldInfo* idle = nullptr; const FieldInfo* name = nullptr;
      for (const FieldInfo& f : kFields) { if (f.section == SEC_PID && !strcmp(f.name, "p")) p = &f; if (f.section == SEC_MOTORS && !strcmp(f.name, "idle_x10")) idle = &f; if (f.section == SEC_GENERAL && !strcmp(f.name, "usb_name")) name = &f; }
      CHECK(p && idle && name && p->offset == offsetof(S_pid, p) && p->count == 3 && idle->offset == offsetof(S_motors, idle_x10) && name->offset == offsetof(S_general, usb_name) && (name->flags & FF_TEXT),
            "offsets as the structs have them (pid.p, motors.idle_x10, general.usb_name - a text)"); }
}

static void t_cli() {
    printf("[cli] get / set / diff / dump / save, what is refused\n");
    { Rig r; r.cli.enter(); CHECK(r.has("Entering CLI Mode") && r.has("# "), "'#' opens it with a prompt");
      CHECK(app::Cli::opens('#', false) && !app::Cli::opens('#', true) && !app::Cli::opens('$', false), "only '#' between two frames opens it (not in the middle of a frame, not '$' - that is MSP)"); }
    { Rig r; uint8_t f[32]; const uint8_t pl[1] = {SEC_PID}; const size_t n = encode(f, sizeof(f), MSG_SECTION_GET, 1, pl, 1);
      r.srv.on_bytes(f, n - 2); const bool mid = r.srv.rx_mid_frame(); r.srv.on_bytes(f + n - 2, 2);
      CHECK(mid && !r.srv.rx_mid_frame(), "the server says when a frame is under way (a '#' inside it is data, not the command line)"); }
    { Rig r;
      r.run("get loop_hz"); CHECK(r.has("general.loop_hz = 4000") && r.has("allowed: 500 .. 8000"), "get: the value and what it takes:\n%s", r.out.c_str());
      r.run("set general.loop_hz = 8000"); CHECK(r.has("general.loop_hz = 8000") && r.s.general.loop_hz == 8000 && (r.srv.flags() & SF_REBOOT_REQUIRED) && (r.srv.flags() & SF_DIRTY), "set: in RAM, unsaved, and this one needs a reboot");
      r.run("set general.loop_hz = 9000"); CHECK(r.has("Invalid value: out of range") && r.s.general.loop_hz == 8000, "out of range: refused, the value stays");
      r.run("set general.loop_hz = 4000"); r.run("status"); CHECK(r.srv.flags() == 0 && r.has("settings saved") && !r.has("reboot"), "set back as it was: nothing to save, no reboot (%s)", r.out.c_str());
      r.run("set general.loop_hz = 8000");
      r.run("set general.board_trim_roll = -2.5"); CHECK(r.s.general.board_trim_roll == -25 && r.has("general.board_trim_roll = -2.5"), "scaled fields in the app's units (deg: -2.5 is -25 tenths)");
      r.run("set general.airmode = ALWAYS"); CHECK(r.s.general.airmode == AIRMODE_ALWAYS, "enums by name") ; }
    { Rig r;
      r.run("set general.board_align = cw90_flip"); CHECK(r.s.general.board_align == ALIGN_CW90_FLIP && r.has("= CW90_FLIP"), "enum names in any case");
      r.run("set general.board_align = SIDEWAYS"); CHECK(r.has("Invalid value") && r.s.general.board_align == ALIGN_CW90_FLIP, "an unknown name: refused");
      r.run("set general.runaway_takeoff = off"); CHECK(r.s.general.runaway_takeoff == 0 && r.has("= OFF"), "bool: ON / OFF");
      r.run("set pins.motor = 4,5,6,7,255,255,255,255"); CHECK(r.s.pins.motor[0] == 4 && r.s.pins.motor[3] == 7 && r.s.pins.motor[4] == 255, "an array: all values at once");
      r.run("set pins.motor = 4,5"); CHECK(r.has("Needs 8 values"), "too few values: says how many");
      r.run("set pins.motor[2] = 16"); CHECK(r.s.pins.motor[2] == 16 && r.s.pins.motor[1] == 5, "one element: name[i]");
      r.run("set pins.motor[9] = 1"); CHECK(r.has("Index out of range"), "an index past the end");
      r.run("set beeper.conditions = GYRO_CALIBRATED,RX_LOST"); CHECK(r.s.beeper.conditions == ((1u << BEEPER_COND_GYRO_CALIBRATED) | (1u << BEEPER_COND_RX_LOST)) && r.has("GYRO_CALIBRATED") , "a bit set by names: %s", r.out.c_str());
      r.run("set beeper.conditions = none"); CHECK(r.s.beeper.conditions == 0 && r.has("= NONE"), "none");
      r.run("set general.usb_name = \"Nhà Nagumo\""); CHECK(!strcmp((const char*)r.s.general.usb_name, "Nhà Nagumo") && r.has("general.usb_name = \"Nhà Nagumo\""), "a text, quoted (UTF-8)");
      r.run("set general.usb_name = 0123456789012345678901234567890123"); CHECK(r.has("Too long"), "a text longer than its field: refused");
      r.run("set nothing.here = 1"); CHECK(r.has("No such setting"), "an unknown name");
      r.run("set general.loop_hz 8000"); CHECK(r.has("Usage"), "no '=': usage");
      r.run("frobnicate"); CHECK(r.has("Unknown command"), "an unknown command"); }
    { Rig r; r.run("set rates.max_rate = 800,750,500"); r.run("set general.board_trim_roll = 1.5");
      r.run("diff"); CHECK(r.has("set rates.max_rate = 800,750,500") && r.has("set general.board_trim_roll = 1.5") && !r.has("set general.loop_hz"), "diff: only what differs, as set lines:\n%s", r.out.c_str());
      const std::string diff = r.out;
      Rig b; size_t at = 0; int lines = 0;
      while (at < diff.size()) { const size_t e = diff.find('\n', at); std::string l = diff.substr(at, e == std::string::npos ? std::string::npos : e - at); at = e == std::string::npos ? diff.size() : e + 1; while (!l.empty() && (l.back() == '\r')) l.pop_back(); if (l.rfind("set ", 0) == 0 && l.find("<hidden>") == std::string::npos) { b.run(l.c_str()); lines++; } }
      CHECK(lines >= 2 && b.s.rates.max_rate[0] == 800 && b.s.rates.max_rate[1] == 750 && b.s.general.board_trim_roll == 15, "pasted into another board, the diff gives the same settings (%d lines)", lines);
      r.run("diff rates"); CHECK(r.has("set rates.max_rate") && !r.has("board_trim"), "diff <section>");
      r.run("dump"); CHECK(r.has("set general.loop_hz = 4000") && r.has("set servo.max_us") && r.has("set can.node_id = 10"), "dump: every setting"); }
    { Rig r; r.host.is_armed = true;
      r.run("set general.loop_hz = 2000"); CHECK(r.has("Not while armed") && r.s.general.loop_hz == 4000, "armed: nothing is set");
      r.run("save"); CHECK(r.has("Not while armed"), "nor saved"); r.run("defaults"); CHECK(r.has("Not while armed"), "nor reset"); }
    { // 0.38: the transmitter's settings are not the flight controller's: not listed, not settable, not reset here
      Rig r; r.run("get tx_sticks"); CHECK(r.has("Nothing matches"), "get: no transmitter fields on the flight controller");
      r.run("dump"); CHECK(!r.has("tx_sticks") && !r.has("tx_radio") && r.has("receiver.nrf_channel"), "dump: its own sections only (the nRF24 receiver fields are there)");
      r.run("set tx_sticks.deadband = 9"); CHECK(r.s.tx_sticks.deadband == 3, "set: a transmitter field is unknown here");
      r.s.tx_sticks.deadband = 7; r.run("defaults"); CHECK(r.s.tx_sticks.deadband == 7 && r.s.rates.max_rate[0] != 0, "defaults: the transmitter's section is not touched"); }
    { Rig r; r.run("set rates.max_rate = 800,800,500");
      CHECK(r.host.applied.size() == 1 && r.host.applied[0] == SEC_RATES && !(r.srv.flags() & SF_REBOOT_REQUIRED), "a live section: applied at once, no reboot needed");
      r.run("save"); CHECK(r.has("Saved") && !(r.srv.flags() & SF_DIRTY) && r.mem.kv.size() == (size_t)kSectionCount - 2, "save: flash written (the flight controller's sections), nothing unsaved");
      Settings back; SettingsStore st(r.mem); st.load(back); CHECK(back.rates.max_rate[0] == 800 && back.rates.max_rate[2] == 500, "and it is what loads at the next boot");
      r.run("defaults rates"); CHECK(r.s.rates.max_rate[0] == 670 && (r.srv.flags() & SF_DIRTY) && r.has("Defaults loaded"), "defaults <section>: in RAM, unsaved");
      r.run("defaults nope"); CHECK(r.has("No such section"), "an unknown section");
      r.mem.fail = true; r.run("save"); CHECK(r.has("Save FAILED"), "a flash that does not take it: said");
      r.run("reboot"); CHECK(r.host.rebooted, "reboot");
      r.run("version"); CHECK(r.has("# Windburst ") && r.has("schema 0x"), "version");
      r.run("status"); CHECK(r.has("disarmed") && r.has("not saved"), "status"); }
    { Rig r; r.cli.enter(); r.out.clear();
      for (const char* p = "get can.no"; *p; p++) r.cli.feed((uint8_t)*p);
      CHECK(r.out == "get can.no", "typed characters are echoed");
      r.cli.feed(8); r.cli.feed(8); r.cli.feed('n'); r.cli.feed('o'); r.out.clear(); r.cli.feed('\r'); r.cli.feed('\n');
      size_t prompts = 0; for (size_t k = r.out.find("# "); k != std::string::npos; k = r.out.find("# ", k + 1)) prompts++;
      CHECK(r.has("can.node_id = 10") && r.out.rfind("# ") == r.out.size() - 2 && prompts == 1, "backspace, CR LF: the line runs once, a new prompt:\n%s", r.out.c_str());
      r.out.clear(); for (const char* p = "exit\r"; *p; p++) r.cli.feed((uint8_t)*p);
      CHECK(!r.cli.active() && r.has("Leaving CLI mode"), "exit: back to the protocol");
      CHECK(!r.cli.feed('x'), "and it takes nothing more"); }
}

int main() {
    t_table(); t_cli();
    printf("\n%d passed, %d failed\n", g_pass, g_fail); return g_fail ? 1 : 0;
}
