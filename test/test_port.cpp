/*
 * This file is part of Windburst Project
 *
 * The portable pieces of the STM32 port (docs/STM32.md), on a PC:
 *   port/pin_names.h        pin numbers <-> "PB3" / "GPIO12"
 *   proto/kv_flash.h        the settings records in flash with quad-word ECC (the STM32H5's), power cuts included
 *   blackbox/bb_qword.h     the flight log (FlashLog) on the same kind of flash
 * test/sim_qwflash.h plays the flash and refuses what the chip refuses (a second program of a quad-word, partial erases).
 */
#include <stdio.h>
#include <string.h>
#include <string>
#include <vector>
#include "sim_qwflash.h"
#include "wblog_reader.h"
#include "../src/port/pin_names.h"
#include "../src/proto/kv_flash.h"
#include "../src/blackbox/bb_qword.h"
#include "../src/blackbox/bb_service.h"

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, ...) do { if (cond) g_pass++; else { g_fail++; printf("  FAIL line %d: %s   ", __LINE__, #cond); printf(__VA_ARGS__); printf("\n"); } } while (0)

static void t_pins() {
    printf("[pins] names of the one-byte pin numbers\n");
    char b[12];
    port::pin_name(0, true, b, sizeof(b)); std::string a0 = b;
    port::pin_name(19, true, b, sizeof(b)); std::string b3 = b;
    port::pin_name(175, true, b, sizeof(b)); std::string k15 = b;
    port::pin_name(12, false, b, sizeof(b)); std::string g12 = b;
    port::pin_name(255, true, b, sizeof(b)); std::string none = b;
    CHECK(a0 == "PA0" && b3 == "PB3" && k15 == "PK15" && g12 == "GPIO12" && none == "-", "STM32: port * 16 + pin (PA0, PB3, PK15); ESP32: GPIOn; 255: none");
    CHECK(port::pin_parse("PB3", true) == 19 && port::pin_parse("pb03", true) == 19 && port::pin_parse("B3", true) == 19 && port::pin_parse(" PC13 ", true) == 45,
          "STM32 names are read back, with or without the P, any case, a leading zero");
    CHECK(port::pin_parse("PB16", true) == -1 && port::pin_parse("PL0", true) == -1 && port::pin_parse("P", true) == -1 && port::pin_parse("PB3x", true) == -1 && port::pin_parse("12", true) == -1,
          "not STM32 pins: PB16, port L, junk");
    CHECK(port::pin_parse("GPIO12", false) == 12 && port::pin_parse("12", false) == 12 && port::pin_parse("gpio48", false) == 48 && port::pin_parse("PB3", false) == -1 && port::pin_parse("255", false) == -1,
          "ESP32: GPIO12 / 12; a PB name or 255 is not one");
    int both = 0; for (int c = 0; c < 176; c++) { port::pin_name((uint8_t)c, true, b, sizeof(b)); if (port::pin_parse(b, true) == c) both++; }
    CHECK(both == 176, "every STM32 pin name reads back as its number (%d of 176)", both);
}

static void t_kv() {
    printf("[settings in flash with quad-word ECC] records, compaction, power cuts\n");
    {   SimQwFlash f(32 * 1024); wbp::KvFlashStorage kv(f);
        CHECK(kv.begin() && kv.ok() && kv.generation() == 1 && kv.keys() == 0, "a blank region is formatted (half 0, generation 1)");
        uint8_t buf[256]; const uint8_t a[5] = {1, 2, 3, 4, 5};
        CHECK(kv.read("wbs01", buf, sizeof(buf)) == -1, "a key never written: -1");
        CHECK(kv.write("wbs01", a, 5) && kv.read("wbs01", buf, sizeof(buf)) == 5 && memcmp(buf, a, 5) == 0, "write and read back");
        const uint32_t used = kv.used(), progs = f.programs;
        CHECK(kv.write("wbs01", a, 5) && kv.used() == used && f.programs == progs, "writing the same value again programs nothing (a SAVE writes every section)");
        uint8_t b2[40]; for (int i = 0; i < 40; i++) b2[i] = (uint8_t)(100 + i);
        CHECK(kv.write("wbs01", b2, 40) && kv.read("wbs01", buf, sizeof(buf)) == 40 && buf[39] == 139, "a new value of the same key: the newest record counts");
        CHECK(kv.read("wbs01", buf, 10) == -1, "a buffer that is too small: -1, nothing half-copied");
        wbp::KvFlashStorage again(f);
        CHECK(again.begin() && again.read("wbs01", buf, sizeof(buf)) == 40 && buf[0] == 100 && again.keys() == 1, "after a restart: the newest value");
        CHECK(f.violations == 0, "no quad-word was programmed twice (the chip would refuse it)");
    }
    {   // many saves: the half fills up, the newest records move to the other half, generation + 1
        SimQwFlash f(32 * 1024); wbp::KvFlashStorage kv(f); kv.begin();
        uint8_t v[96]; bool ok = true; char key[8];
        for (int round = 0; round < 60 && ok; round++)
            for (int k = 0; k < 19 && ok; k++) { snprintf(key, sizeof(key), "wbs%02d", k); memset(v, round * 19 + k, sizeof(v)); ok = kv.write(key, v, sizeof(v)); }
        CHECK(ok && kv.compactions() > 5 && f.violations == 0, "60 saves of 19 sections of 96 bytes into 2 x 16 kB: %u compactions, no quad-word programmed twice", kv.compactions());
        wbp::KvFlashStorage again(f); uint8_t buf[128]; int good = 0;
        again.begin();
        for (int k = 0; k < 19; k++) { snprintf(key, sizeof(key), "wbs%02d", k); if (again.read(key, buf, sizeof(buf)) == 96 && buf[0] == (uint8_t)(59 * 19 + k) && buf[95] == buf[0]) good++; }
        CHECK(good == 19 && again.generation() == kv.generation(), "after a restart every section has its last value (%d of 19), generation %u", good, again.generation());
    }
    {   // a power cut at every possible moment of a save: the old value or the new one, never garbage, never a lost other key
        int cases = 0, bad = 0, old_v = 0, new_v = 0, cut_new = 0;
        for (int cut = 0; cut < 400; cut++) {
            SimQwFlash f(16 * 1024); wbp::KvFlashStorage kv(f); kv.begin();
            uint8_t a[64], b[64], c[32]; memset(a, 0xA1, sizeof(a)); memset(b, 0xB2, sizeof(b)); memset(c, 0xC3, sizeof(c));
            kv.write("other", c, sizeof(c));
            for (int i = 0; i < 40; i++) { a[0] = (uint8_t)i; kv.write("wbs05", a, sizeof(a)); }   // near a compaction
            f.cut_after = cut % 60;
            kv.write("wbs05", b, sizeof(b));
            const bool was_cut = f.dead(); f.power_on();
            wbp::KvFlashStorage again(f); uint8_t buf[64];
            if (!again.begin()) { bad++; continue; }
            const int n = again.read("wbs05", buf, sizeof(buf)); const int m = again.read("other", buf + 0, 0);
            (void)m; uint8_t o[32];
            const bool other_ok = again.read("other", o, sizeof(o)) == 32 && o[0] == 0xC3 && o[31] == 0xC3;
            uint8_t back[64]; again.read("wbs05", back, sizeof(back));
            const bool is_new = n == 64 && back[0] == 0xB2 && back[63] == 0xB2, is_old = n == 64 && back[0] == 39 && back[1] == 0xA1;
            if (!other_ok || !(is_new || is_old)) bad++;
            if (was_cut) { cases++; old_v += is_old; cut_new += is_new; } else new_v += is_new;
            uint8_t z[64]; memset(z, 0x5C, sizeof(z));
            if (!again.write("wbs05", z, sizeof(z)) || again.read("wbs05", back, sizeof(back)) != 64 || back[0] != 0x5C) bad++;   // and it can be written again
        }
        CHECK(bad == 0 && cases >= 40 && old_v == cases && cut_new == 0 && new_v > 0, "%d power cuts at every step of a save (record and compaction): the old value each time (%d), never a half one; saves that finished: the new value (%d); the other key kept, writable afterwards (%d bad)", cases, old_v, new_v, bad);
    }
    {   SimQwFlash f(16 * 1024); wbp::KvFlashStorage kv(f); kv.begin(); uint8_t big[4096] = {};
        CHECK(!kv.write("toolongkey_abcdefgh", big, 4) && !kv.write("", big, 4) && !kv.write("wbs01", big, sizeof(big)), "refused: a key longer than 15, an empty key, a record bigger than half the region");
        SimQwFlash tiny(8 * 1024); wbp::KvFlashStorage t(tiny);
        CHECK(!t.begin() && !t.ok() && t.read("x", big, 4) == -1 && !t.write("x", big, 4), "a region of one sector (no room for two halves): not usable, says so");
        SimQwFlash junk(16 * 1024); memset(junk.raw(), 0x3C, 16 * 1024); wbp::KvFlashStorage j(junk);
        CHECK(j.begin() && j.keys() == 0 && j.write("wbs01", big, 8), "foreign data in the region: formatted afresh");
    }
}

// FlashLog (the flight log of the ESP32 build) on quad-word flash through QwBlockStore
static std::vector<uint8_t> frame(uint8_t type, uint8_t seq, uint16_t n, uint8_t fill) { uint8_t pay[240], b[300]; for (uint16_t i = 0; i < n; i++) pay[i] = (uint8_t)(fill + i); const size_t k = wbp::encode(b, sizeof(b), type, seq, pay, n); return std::vector<uint8_t>(b, b + k); }
static std::vector<uint8_t> session(bb::FlashLog& log, int frames, uint8_t fill, bool flush_often) {
    std::vector<uint8_t> all;
    if (!log.start_session()) return all;
    const char* text = "WBLOG 1\nsession 1\nend\n"; uint8_t hb[300]; const size_t hn = wbp::encode(hb, sizeof(hb), bb::kMsgHeader, 0, (const uint8_t*)text, (uint16_t)strlen(text));
    auto put = [&](const uint8_t* d, size_t n) { while (!log.append(d, (uint32_t)n)) { if (!log.pump(true)) return false; } all.insert(all.end(), d, d + n); log.pump(flush_often); return true; };
    put(hb, hn);
    for (int i = 0; i < frames; i++) { const std::vector<uint8_t> f = frame(bb::kMsgData, (uint8_t)i, (uint16_t)(37 + i % 200), (uint8_t)(fill + i)); if (!put(f.data(), f.size())) break; }
    log.end_session();
    return all;
}

static void t_blackbox() {
    printf("[flight log on quad-word flash] FlashLog through QwBlockStore\n");
    SimQwFlash f(256 * 1024); bb::QwBlockStore st(f);
    bb::FlashLog log; CHECK(log.begin(&st) && st.size() == 256 * 1024 && log.sessions() == 0, "blank: an empty log (size cut to whole 64 kB blocks: %u)", st.size());
    while (log.verify_step()) {}
    const std::vector<uint8_t> s1 = session(log, 300, 1, false), s2 = session(log, 50, 9, true);
    CHECK(log.sessions() == 2 && f.violations == 0, "two sessions, one of them flushed after every frame (partly filled pages): no quad-word programmed twice (%u refused)", f.violations);
    CHECK(st.pending() == 0, "nothing left waiting in RAM after the session ended");
    std::vector<uint8_t> back(s1.size()); CHECK(log.read(0, back.data(), (uint32_t)back.size()) && back == s1, "the first session reads back byte for byte");
    bb::FlashLog again; again.begin(&st);
    std::vector<uint8_t> b2(s2.size()); CHECK(again.sessions() == 2 && again.read(again.session_start(1), b2.data(), (uint32_t)b2.size()) && b2 == s2, "after a restart both are found, the second reads back too");
    const std::vector<RSession> ss = read_log(f.raw(), again.used());
    CHECK(ss.size() == 2 && ss[0].data_frames == 300 && ss[1].data_frames == 50, "the raw region reads as two sessions (%zu)", ss.size());
    {   // in the middle of a session, before the end: what is in RAM is readable
        bb::FlashLog l; l.begin(&st); while (l.verify_step()) {}
        l.start_session(); const std::vector<uint8_t> fr = frame(bb::kMsgData, 1, 5, 3);
        l.append(fr.data(), (uint32_t)fr.size()); l.pump(true);
        CHECK(st.pending() > 0 && st.pending() < 16, "a flush that ends inside a quad-word keeps those %u bytes in RAM", st.pending());
        { const uint32_t at = l.session_start(l.sessions() - 1); std::vector<uint8_t> got(fr.size());
          CHECK(st.read(at, got.data(), (uint32_t)got.size()) && got == fr, "and reading the session meanwhile gives them too (the flash still has 0xFF there)"); }
        l.end_session(); CHECK(st.pending() == 0 && f.violations == 0, "the end of the session programs them (padded): still nothing programmed twice"); }
    log.erase_start(); while (log.erase_step()) {}
    CHECK(log.sessions() == 0 && f.erases > 0 && f.violations == 0, "erase all: whole 8 kB sectors (%u erased)", f.erases);
}

int main() {
    t_pins(); t_kv(); t_blackbox();
    printf("%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
