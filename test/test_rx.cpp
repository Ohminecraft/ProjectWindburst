// Receiver protocols: CRSF, SBUS, IBUS, Spektrum decoders, the PPM / PWM pulse decoders and the link state. Portable (no Arduino).
// The encoders below are written independently of the decoders (plain bit loops), from the protocol descriptions.
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <vector>
#include <string>
#include <random>
#include "../src/hal/rx/rx_proto.h"
#include "../src/hal/rx/rx_pulse.h"
#include "../src/telemetry/crsf_telemetry.h"
#include "sim_nrf24.h"
#include "../src/hal/rx/rx_nrf24.h"
#include "../src/hal/rx/rx_mavlink.h"
#include "mavlink_vectors.h"

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, ...) do { if (cond) g_pass++; else { g_fail++; printf("  FAIL line %d: %s   ", __LINE__, #cond); printf(__VA_ARGS__); printf("\n"); } } while (0)
typedef std::vector<uint8_t> Bytes;

static void pack11(const uint16_t raw[16], uint8_t out[22]) {
    memset(out, 0, 22);
    for (int ch = 0; ch < 16; ch++) for (int b = 0; b < 11; b++) if ((raw[ch] >> b) & 1) { const int bit = ch * 11 + b; out[bit / 8] |= (uint8_t)(1u << (bit % 8)); }
}
static uint8_t crc8_d5(const uint8_t* p, size_t n) { uint8_t c = 0; for (size_t i = 0; i < n; i++) { c ^= p[i]; for (int b = 0; b < 8; b++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0xD5) : (uint8_t)(c << 1); } return c; }
static Bytes crsf_frame(uint8_t type, const Bytes& payload) { Bytes f = {0xC8, (uint8_t)(payload.size() + 2), type}; f.insert(f.end(), payload.begin(), payload.end()); f.push_back(crc8_d5(f.data() + 2, payload.size() + 1)); return f; }
static Bytes crsf_rc(const uint16_t raw[16]) { uint8_t p[22]; pack11(raw, p); return crsf_frame(0x16, Bytes(p, p + 22)); }
static Bytes sbus(const uint16_t raw[16], uint8_t flags = 0, uint8_t end = 0x00) { uint8_t p[22]; pack11(raw, p); Bytes f = {0x0F}; f.insert(f.end(), p, p + 22); f.push_back(flags); f.push_back(end); return f; }
static Bytes ibus(const uint16_t us[14]) { Bytes f = {0x20, 0x40}; for (int i = 0; i < 14; i++) { f.push_back((uint8_t)us[i]); f.push_back((uint8_t)(us[i] >> 8)); } uint16_t sum = 0xFFFF; for (uint8_t c : f) sum = (uint16_t)(sum - c); f.push_back((uint8_t)sum); f.push_back((uint8_t)(sum >> 8)); return f; }

// feeds bytes; returns the number of frames decoded, last frame in `last`
static int feed(rx::Decoder& d, const Bytes& b, rx::Frame& last, uint32_t now_us = 0) { int n = 0; rx::Frame f; for (uint8_t c : b) if (d.feed(c, now_us, f)) { last = f; n++; } return n; }

static void t_crsf() {
    printf("[CRSF]\n");
    rx::CrsfDecoder d; rx::Frame f;
    CHECK(d.baud() == 420000 && !d.inverted() && !d.even_parity_2stop(), "420000 8N1, not inverted");
    { uint16_t raw[16]; for (int i = 0; i < 16; i++) raw[i] = 992; raw[0] = 172; raw[1] = 1811; raw[2] = 992; raw[15] = 1000;
      CHECK(feed(d, crsf_rc(raw), f) == 1, "one RC frame decoded");
      CHECK(f.ch[0] == 988 && f.ch[1] == 2012 && (f.ch[2] == 1500 || f.ch[2] == 1501), "Betaflight scaling: 172 -> %u (988), 1811 -> %u (2012), 992 -> %u (1500)", f.ch[0], f.ch[1], f.ch[2]);
      CHECK(abs((int)f.ch[15] - (int)lround(0.62477120195241 * 1000 + 881)) <= 1 && !f.failsafe, "last channel sits in the last 11 bits: %u", f.ch[15]); }
    { // every channel distinct: catches any bit-order / channel-order mistake
      uint16_t raw[16]; for (int i = 0; i < 16; i++) raw[i] = (uint16_t)(200 + i * 97);
      feed(d, crsf_rc(raw), f); bool ok = true; for (int i = 0; i < 16; i++) if (abs((int)f.ch[i] - (int)lround(0.62477120195241 * raw[i] + 881)) > 1) ok = false;
      CHECK(ok, "16 distinct channels land in the right slots"); }
    { uint16_t raw[16]; for (int i = 0; i < 16; i++) raw[i] = 0; raw[1] = 2047; feed(d, crsf_rc(raw), f);
      CHECK(f.ch[0] == rx::kUsMin && f.ch[1] == rx::kUsMax, "out-of-range raw values are clamped to %u..%u (got %u %u)", rx::kUsMin, rx::kUsMax, f.ch[0], f.ch[1]); }
    { uint16_t raw[16]; for (int i = 0; i < 16; i++) raw[i] = 992; Bytes b = crsf_rc(raw); b[10] ^= 0x04; const uint32_t e0 = d.errors();
      CHECK(feed(d, b, f) == 0, "a corrupted frame is not accepted");
      b = crsf_rc(raw); for (int i = 0; i < 4; i++) feed(d, b, f);
      CHECK(d.errors() > e0, "...and is counted"); }
    { // link statistics: uplink RSSI ant1/ant2, LQ, SNR, active antenna, rf mode, tx power, downlink RSSI/LQ/SNR
      CHECK(feed(d, crsf_frame(0x14, {62, 55, 87, 9, 1, 4, 2, 70, 99, 6}), f) == 0 && d.link_quality() == 87 && d.rssi_dbm() == -55, "LINK_STATISTICS: LQ %u, RSSI %d dBm (active antenna 2)", d.link_quality(), d.rssi_dbm());
      feed(d, crsf_frame(0x14, {62, 55, 100, 9, 0, 4, 2, 70, 99, 6}), f); CHECK(d.rssi_dbm() == -62, "antenna 1 active: %d", d.rssi_dbm());
      CHECK(feed(d, crsf_frame(0x08, {0, 160, 0, 15, 0, 5, 220, 90}), f) == 0, "other valid frame types (battery) are skipped without error"); }
    { // garbage, a false sync byte with a plausible length, then real frames: nothing real may be lost for long
      rx::CrsfDecoder n; uint16_t raw[16]; for (int i = 0; i < 16; i++) raw[i] = (uint16_t)(300 + i * 10);
      Bytes s = {0x00, 0xC8, 0x30, 0x16, 0x12, 0xC8, 0xC8}; const Bytes good = crsf_rc(raw);
      for (int i = 0; i < 6; i++) s.insert(s.end(), good.begin(), good.end());
      const int got = feed(n, s, f);
      CHECK(got >= 5, "false sync (declaring 48 bytes) in front of 6 real frames: %d recovered", got);
      std::mt19937 rng(7); Bytes noise; for (int i = 0; i < 20000; i++) noise.push_back((uint8_t)rng());
      const int ghosts = feed(n, noise, f);
      CHECK(ghosts == 0, "20000 random bytes produce no RC frame (%d)", ghosts);
      CHECK(feed(n, good, f) + feed(n, good, f) + feed(n, good, f) >= 2, "and the stream is picked up again afterwards"); }
    { // split across reads byte by byte
      rx::CrsfDecoder n; uint16_t raw[16]; for (int i = 0; i < 16; i++) raw[i] = 992; const Bytes b = crsf_rc(raw); int got = 0; rx::Frame g;
      for (uint8_t c : b) got += n.feed(c, 0, g) ? 1 : 0;
      CHECK(got == 1 && n.frames() == 1, "byte-by-byte delivery"); }
}

static void t_sbus() {
    printf("[SBUS]\n");
    rx::SbusDecoder d; rx::Frame f; uint32_t t = 1000000;
    CHECK(d.baud() == 100000 && d.inverted() && d.even_parity_2stop(), "100000 8E2 inverted");
    uint16_t raw[16]; for (int i = 0; i < 16; i++) raw[i] = 992; raw[0] = 172; raw[1] = 1811; raw[7] = 1024;
    CHECK(feed(d, sbus(raw), f, t) == 1 && f.ch[0] == 988 && f.ch[1] == 2012 && f.ch[2] == 1500 && f.ch[7] == 1520 && !f.failsafe && !f.frame_lost,
          "Betaflight scaling 5/8*raw+880: %u %u %u %u", f.ch[0], f.ch[1], f.ch[2], f.ch[7]);
    t += 7000; CHECK(feed(d, sbus(raw, 0x08), f, t) == 1 && f.failsafe, "failsafe flag (bit 3)");
    t += 7000; CHECK(feed(d, sbus(raw, 0x04), f, t) == 1 && f.frame_lost && !f.failsafe, "frame-lost flag (bit 2)");
    t += 7000; CHECK(feed(d, sbus(raw, 0x03), f, t) == 1 && !f.failsafe && !f.frame_lost, "digital channels 17/18 are not failsafe");
    t += 7000; CHECK(feed(d, sbus(raw, 0, 0x14), f, t) == 1, "SBUS2 end byte 0x14 accepted");
    t += 7000; { const uint32_t e = d.errors(); CHECK(feed(d, sbus(raw, 0, 0x55), f, t) == 0 && d.errors() == e + 1, "wrong end byte rejected"); }
    { // a frame cut short, then (after the inter-frame gap) a complete one: the stump must not swallow it
      t += 7000; Bytes cut = sbus(raw); cut.resize(11); CHECK(feed(d, cut, f, t) == 0, "half a frame");
      t += 7000; raw[3] = 300; CHECK(feed(d, sbus(raw), f, t) == 1 && f.ch[3] == (uint16_t)lround(0.625 * 300 + 880), "the next frame after the gap is decoded correctly (ch4 %u)", f.ch[3]); }
    { // a stray byte right in front of a frame (start of reception, line glitch) must not shift the frame
      rx::SbusDecoder n; Bytes b = {0x55}; const Bytes good = sbus(raw); b.insert(b.end(), good.begin(), good.end());
      CHECK(feed(n, b, f, 3000000) == 1 && f.ch[0] == 988, "stray byte before the 0x0F start byte is skipped"); }
    { // data bytes equal to 0x0F must not restart the frame
      rx::SbusDecoder n; uint16_t r2[16]; for (int i = 0; i < 16; i++) r2[i] = 0x00F; r2[5] = 0x78F;
      CHECK(feed(n, sbus(r2), f, 5000000) == 1 && f.ch[5] == rx::clamp_us(0.625f * 0x78F + 880), "0x0F inside the payload"); }
    { // powered up in the middle of a frame: recovers at the next gap
      rx::SbusDecoder n; Bytes tail = sbus(raw); tail.erase(tail.begin(), tail.begin() + 9); uint32_t tt = 9000000;
      feed(n, tail, f, tt); tt += 7000; const int a = feed(n, sbus(raw), f, tt); tt += 7000; const int b = feed(n, sbus(raw), f, tt);
      CHECK(a + b >= 1 && b == 1, "joins a running stream within two frames (%d %d)", a, b); }
}

static void t_ibus() {
    printf("[IBUS]\n");
    rx::IbusDecoder d; rx::Frame f;
    uint16_t us[14]; for (int i = 0; i < 14; i++) us[i] = (uint16_t)(1000 + i * 70);
    CHECK(d.baud() == 115200 && !d.inverted(), "115200 8N1");
    CHECK(feed(d, ibus(us), f) == 1, "frame decoded");
    { bool ok = true; for (int i = 0; i < 14; i++) if (f.ch[i] != us[i]) ok = false; CHECK(ok && f.ch[14] == 1500 && f.ch[15] == 1500, "14 channels in microseconds, 15/16 at mid"); }
    { Bytes b = ibus(us); b[7] ^= 0x10; const uint32_t e = d.errors(); CHECK(feed(d, b, f) == 0, "bad checksum rejected"); feed(d, ibus(us), f); feed(d, ibus(us), f); CHECK(d.errors() > e, "counted"); }
    { rx::IbusDecoder n; Bytes s = {0x20, 0x40, 0x20, 0x11}; const Bytes good = ibus(us); for (int i = 0; i < 4; i++) s.insert(s.end(), good.begin(), good.end());
      CHECK(feed(n, s, f) >= 3, "false header in front of real frames"); }
    { uint16_t hi[14]; for (int i = 0; i < 14; i++) hi[i] = 1500; hi[0] = (uint16_t)(0x3000 | 1234); feed(d, ibus(hi), f); CHECK(f.ch[0] == 1234, "upper nibble (extra channels on newer FlySky) is masked off: %u", f.ch[0]); }
}

static void t_link() {
    printf("[link state]\n");
    rx::Link l; l.timeout_ms = 300; rx::Frame f; for (int i = 0; i < 16; i++) f.ch[i] = (uint16_t)(1100 + i);
    CHECK(!l.ok(0) && !l.ok(100000), "no frame yet => not ok");
    CHECK(l.channels()[2] == 1000 && l.channels()[0] == 1500, "safe defaults before the first frame: throttle low, sticks centred");
    l.on_frame(f, 1000); CHECK(l.ok(1000) && l.ok(1300) && !l.ok(1301), "ok until timeout_ms after the last frame");
    CHECK(l.channels()[5] == 1105, "channels stored");
    rx::Frame fs = f; fs.failsafe = true; fs.ch[2] = 900;
    for (uint32_t t = 1010; t < 2000; t += 10) l.on_frame(fs, t);
    CHECK(!l.ok(2000) && l.channels()[2] == 1102, "frames carrying the receiver's failsafe flag do not keep the link alive and do not replace the sticks");
    CHECK(l.failsafe_frames() == 99, "failsafe frames counted");
    l.on_frame(f, 2000); CHECK(l.ok(2001), "a good frame restores the link");
    { rx::Link r; for (uint32_t t = 0; t < 3000; t += 4) { r.on_frame(f, t); r.tick(t); } CHECK(r.rate_hz() == 250, "frame rate measured: %u Hz", r.rate_hz()); }
    { rx::Link w; w.timeout_ms = 300; w.on_frame(f, 0xFFFFFF00u); CHECK(w.ok(0x00000010u) && !w.ok(0x00000200u), "timeout across the 49-day millis() wrap"); }
}

// ---------------------------------------------------------------- Spektrum: frames built word by word from the layout
// (channel number shifted above the value; big-endian; 0xFFFF = empty slot)
static Bytes spek(bool hires, const int ch[7], const uint16_t val[7], uint8_t fades = 0, uint8_t system = 0xB2) {
    Bytes f = {fades, system};
    for (int k = 0; k < 7; k++) {
        uint16_t w = 0xFFFF;
        if (ch[k] >= 0) { w = 0; for (int b = 0; b < (hires ? 11 : 10); b++) if ((val[k] >> b) & 1) w |= (uint16_t)(1u << b); w |= (uint16_t)(ch[k] << (hires ? 11 : 10)); }
        f.push_back((uint8_t)(w >> 8)); f.push_back((uint8_t)(w & 0xFF));
    }
    return f;
}
static void t_spektrum() {
    printf("[Spektrum] 2048 / 1024, channels split over frames, empty slots, resync from the silence\n");
    rx::SpektrumDecoder d(true); rx::Frame f; d.reset(); uint32_t t = 1000000;
    CHECK(d.baud() == 115200 && !d.inverted() && !d.even_parity_2stop() && !strcmp(d.name(), "SPEKTRUM2048"), "115200 8N1, not inverted");
    const int a[7] = {0, 1, 2, 3, 4, 5, 6}; const uint16_t av[7] = {0, 1024, 2047, 342, 1706, 600, 1400};
    CHECK(feed(d, spek(true, a, av), f, t) == 0, "bytes before the first silence: not taken (could be the middle of a frame)");
    t += 11000; CHECK(feed(d, spek(true, a, av), f, t) == 1, "after a silence: a frame");
    CHECK(f.ch[0] == 988 && f.ch[1] == 1500 && f.ch[2] == 2012 && f.ch[3] == 1159 && f.ch[4] == 1841, "2048: 988 + value / 2 (0 -> %u, 1024 -> %u, 2047 -> %u, 342 -> %u, 1706 -> %u)", f.ch[0], f.ch[1], f.ch[2], f.ch[3], f.ch[4]);
    CHECK(f.ch[7] == 1500 && f.ch[11] == 1500, "channels not sent yet: 1500");
    const int b[7] = {7, 8, 9, 10, 11, -1, -1}; const uint16_t bv[7] = {100, 200, 300, 400, 500, 0, 0};
    Bytes fb = spek(true, b, bv); for (int k = 0; k < 5; k++) fb[2 + 2 * k] |= 0x80;          // bit 15: not part of the channel number
    t += 11000; CHECK(feed(d, fb, f, t) == 1 && f.ch[7] == 1038 && f.ch[11] == 1238 && f.ch[0] == 988 && f.ch[1] == 1500, "the second kind of frame: channels 8..12, the others kept (%u %u)", f.ch[7], f.ch[11]);
    const int c[7] = {12, 13, 15, 2, -1, -1, -1}; const uint16_t cv[7] = {500, 500, 500, 1024, 0, 0, 0};
    t += 11000; CHECK(feed(d, spek(true, c, cv), f, t) == 1 && f.ch[12] == 1500 && f.ch[13] == 1500 && f.ch[2] == 1500, "channel numbers 12..15 are ignored (Betaflight: 12 channels); channel 3 moved to 1500");
    // a frame that stops half way, then the next one after the usual silence
    Bytes half = spek(true, a, av); half.resize(9); t += 11000; CHECK(feed(d, half, f, t) == 0, "a frame cut after 9 bytes: nothing");
    const uint16_t av2[7] = {1000, 1000, 1000, 1000, 1000, 1000, 1000};
    t += 11000; const uint32_t e0 = d.errors(); CHECK(feed(d, spek(true, a, av2), f, t) == 1 && f.ch[0] == 1488 && d.errors() == e0 + 1, "the next whole frame is found again (the cut one counts as an error)");
    // several frames that arrive together (the loop stood still): counted in 16s
    Bytes three; for (int k = 0; k < 3; k++) { const uint16_t v[7] = {(uint16_t)(100 * k), 0, 0, 0, 0, 0, 0}; Bytes x = spek(true, a, v); three.insert(three.end(), x.begin(), x.end()); }
    t += 30000; CHECK(feed(d, three, f, t) == 3 && f.ch[0] == 1088, "three frames read in one go: three frames, the last one wins (%u)", f.ch[0]);
    { rx::SpektrumDecoder d1(false); d1.reset(); const uint16_t v[7] = {0, 512, 1023, 12, 1000, 700, 300}; uint32_t u = 5000000; feed(d1, spek(false, a, v, 0, 0x01), f, u); u += 22000;
      CHECK(!strcmp(d1.name(), "SPEKTRUM1024") && feed(d1, spek(false, a, v, 3, 0x01), f, u) == 1 && f.ch[0] == 988 && f.ch[1] == 1500 && f.ch[2] == 2011 && f.ch[3] == 1000 && f.ch[6] == 1288 && d1.fades() == 3,
            "1024: 988 + value (0 -> %u, 512 -> %u, 1023 -> %u), fades byte kept", f.ch[0], f.ch[1], f.ch[2]); }
    { rx::SpektrumDecoder d2(true); d2.reset(); std::mt19937 rng(7); uint32_t u = 0; int n = 0; rx::Frame g;
      int out = 0; for (int i = 0; i < 20000; i++) { u += (rng() % 50 == 0) ? 6000 : 100; if (d2.feed((uint8_t)rng(), u, g)) { n++; for (int k = 0; k < 16; k++) if (g.ch[k] < rx::kUsMin || g.ch[k] > rx::kUsMax) out++; } }
      CHECK(out == 0 && n > 0, "random bytes (Spektrum has no checksum, so frames do come out: %d): every value stays inside 885..2115 us", n); }
}

// ---------------------------------------------------------------- PPM: edge times built from channel lengths
struct PpmGen {
    uint32_t t = 4000000000u;                                // close to the 32-bit wrap: it must not matter
    void frame(rx::PpmDecoder& d, const std::vector<uint32_t>& ch, int& frames, rx::Frame& last, uint32_t frame_us = 22500) {
        const uint32_t start = t; rx::Frame f;
        if (d.edge(t, f)) { frames++; last = f; }
        for (uint32_t c : ch) { t += c; if (d.edge(t, f)) { frames++; last = f; } }
        t = start + frame_us;
    }
};
static void t_ppm() {
    printf("[PPM] sync, learning the channel count, bad pulses, 4..12 channels, clock wrap\n");
    rx::PpmDecoder d; rx::Frame f; PpmGen g; int n = 0;
    const std::vector<uint32_t> ch8 = {1000, 1100, 1200, 1300, 1400, 1500, 1600, 2000};
    for (int i = 0; i < 26; i++) g.frame(d, ch8, n, f);
    CHECK(n == 0 && d.channels() == 0, "the first 26 frames: the count is being learnt (25 alike in a row), none taken (%d)", n);
    for (int i = 0; i < 4; i++) g.frame(d, ch8, n, f);
    CHECK(d.channels() == 8 && n == 2, "then: 8 channels, frames come out (%d; a frame is judged at the sync that ends it)", n);
    CHECK(f.ch[0] == 1000 && f.ch[3] == 1300 && f.ch[7] == 2000 && f.ch[8] == 1500 && f.ch[15] == 1500, "channel = time between two edges (%u %u %u), 9..16 at 1500", f.ch[0], f.ch[3], f.ch[7]);
    // (each frame is judged when the next one starts: the first call below still delivers the last good frame)
    const int before = n + 1;
    std::vector<uint32_t> bad = ch8; bad[3] = 700; g.frame(d, bad, n, f);
    bad = ch8; bad[5] = 2300; g.frame(d, bad, n, f); CHECK(n == before, "a pulse of 0.70 ms: the frame is dropped (%d)", n - before);
    g.frame(d, {1000, 1100, 1200, 1300, 1400, 1500, 1600}, n, f); CHECK(n == before, "a pulse of 2.30 ms: dropped");
    g.frame(d, ch8, n, f); CHECK(n == before && d.channels() == 8, "one frame with 7 channels: dropped, the learnt count stays 8");
    g.frame(d, ch8, n, f); CHECK(n == before + 1 && f.ch[7] == 2000, "and the next normal one is taken");
    { rx::PpmDecoder d12; PpmGen g2; int m = 0; std::vector<uint32_t> c12; for (int i = 0; i < 12; i++) c12.push_back(1050 + 50 * i);
      for (int i = 0; i < 30; i++) g2.frame(d12, c12, m, f, 30000);
      CHECK(d12.channels() == 12 && m > 0 && f.ch[11] == 1600, "12 channels (30 ms frames): taken (ch 12 = %u)", f.ch[11]);
      rx::PpmDecoder d13; PpmGen g3; int k = 0; std::vector<uint32_t> c13(13, 1500); for (int i = 0; i < 40; i++) g3.frame(d13, c13, k, f, 30000);
      CHECK(k == 0, "13 channels: more than PPM may carry, never taken (the 13th pulse breaks every frame)");
      rx::PpmDecoder d3; PpmGen g4; int j = 0; for (int i = 0; i < 40; i++) g4.frame(d3, {1500, 1500, 1500}, j, f);
      CHECK(j == 0, "3 channels: too few, never taken"); }
    { rx::PpmDecoder dg; PpmGen gg; int m = 0; for (int i = 0; i < 30; i++) gg.frame(dg, ch8, m, f, i % 2 ? 22500 : 19400);
      CHECK(m > 0 && dg.channels() == 8, "frames of changing length (sync 2.7 ms or more): fine"); }
    { rx::PpmDecoder dn; PpmGen gn; int m = 0; for (int i = 0; i < 30; i++) gn.frame(dn, ch8, m, f, ch8.size() * 0 + 13600);
      CHECK(m == 0, "a sync gap of 2.5 ms (frame too short for its channels): never a frame"); }
}

// ---------------------------------------------------------------- PWM
static void t_pwm() {
    printf("[PWM] one input per channel, stale inputs, bad widths, clock wrap\n");
    rx::PwmDecoder d; d.reset(0x0F); rx::Frame f; uint32_t t = 4294900000u; int n = 0;
    auto frame = [&](const uint16_t us[4], uint8_t dead = 0) {        // channels one after the other, every 20 ms
        const uint32_t start = t; rx::Frame g;
        for (int i = 0; i < 4; i++) { if (!(dead & (1u << i))) { if (d.edge((uint8_t)i, 1, t, g)) { n++; f = g; } if (d.edge((uint8_t)i, 0, t + us[i], g)) { n++; f = g; } } t += us[i] + 30; }
        t = start + 20000; };
    const uint16_t a[4] = {1100, 1600, 1900, 1234};
    frame(a); CHECK(n == 0, "first frame: channel 1 finishes before the others ever had a pulse -> nothing yet");
    frame(a); CHECK(n == 1 && f.ch[0] == 1100 && f.ch[1] == 1600 && f.ch[2] == 1900 && f.ch[3] == 1234 && f.ch[4] == 1500, "second frame: all four (%u %u %u %u), channel 5 at 1500", f.ch[0], f.ch[1], f.ch[2], f.ch[3]);
    for (int i = 0; i < 4; i++) frame(a, 1u << 2);
    CHECK(n == 5, "channel 3 stops: frames go on while its last pulse is younger than 100 ms (%d)", n);
    for (int i = 0; i < 4; i++) frame(a, 1u << 2);
    CHECK(n == 6, "and stop after that: no frame on a stale channel (one more: its last pulse was 96.5 ms old then) (%d)", n);
    frame(a); CHECK(n == 6, "back: channel 1 comes first in a frame, so channel 3 counts again from the next frame");
    frame(a); CHECK(n == 7, "frames again");
    const uint16_t bad[4] = {1100, 2400, 1900, 1234}; frame(bad);
    CHECK(n == 8 && f.ch[1] == 1600, "a 2.4 ms pulse on channel 2 is not taken: the last good value stays (%u)", f.ch[1]);
    { rx::Frame g; const uint32_t e0 = d.errors(); d.edge(1, 0, t, g); CHECK(d.errors() == e0, "a falling edge without its rising one: ignored"); }
    { rx::Frame g; CHECK(!d.edge(6, 1, t, g) && !d.edge(6, 0, t + 1500, g), "an input that is not wired: ignored"); }
}

static void t_edge_ring() {
    printf("[edge ring] order, full ring\n");
    rx::EdgeRing<8> r; rx::Edge e;
    for (int i = 0; i < 10; i++) r.push((uint32_t)i, 0, (uint8_t)(i & 1));
    int k = 0; bool order = true; while (r.pop(e)) { if (e.us != (uint32_t)k) order = false; k++; }
    CHECK(k == 8 && order && r.lost() == 2, "8 kept in order, 2 dropped and counted (%d, lost %u)", k, r.lost());
    r.push(99, 3, 1); CHECK(r.pop(e) && e.us == 99 && e.input == 3 && e.level == 1 && !r.pop(e), "and it goes on after being full");
}

// ---------------------------------------------------------------- CRSF telemetry: bytes worked out by hand from Betaflight's layout
static void t_crsf_telemetry() {
    printf("[CRSF telemetry] frame bytes, units, flight mode text, schedule\n");
    uint8_t b[crsf::kMaxFrame]; crsf::TelemetryIn t;
    auto ok = [&](size_t n) { uint8_t c = crc8_d5(b + 2, n - 3); return n >= 4 && b[0] == 0xC8 && b[1] == n - 2 && b[n - 1] == c; };
    t.volts = 16.84f; t.amps = 12.34f; t.mah = 1234.4f; t.remaining = 56;
    size_t n = crsf::build(crsf::FT_BATTERY, t, b);
    CHECK(n == 12 && ok(n) && b[2] == 0x08 && b[3] == 0x00 && b[4] == 168 && b[5] == 0x00 && b[6] == 123 && b[7] == 0x00 && b[8] == 0x04 && b[9] == 0xD2 && b[10] == 56,
          "battery: 16.84 V -> 168, 12.34 A -> 123, 1234 mAh -> 00 04 D2, 56 %%");
    t.volts = -1; t.amps = 1e9f; t.mah = NAN; t.remaining = 150; n = crsf::build(crsf::FT_BATTERY, t, b);
    CHECK(ok(n) && b[3] == 0 && b[4] == 0 && b[5] == 0xFF && b[6] == 0xFF && b[7] == 0 && b[8] == 0 && b[9] == 0 && b[10] == 100, "out-of-range values are clamped, NaN = 0");
    t.roll_deg = 10.0f; t.pitch_deg = -5.0f; t.yaw_deg = 270.0f; n = crsf::build(crsf::FT_ATTITUDE, t, b);
    auto i16 = [&](int o) { return (int)(int16_t)((b[o] << 8) | b[o + 1]); };
    CHECK(n == 10 && ok(n) && b[2] == 0x1E && i16(3) == -873 && i16(5) == 1745 && i16(7) == -15708, "attitude: pitch -5 deg = %d, roll 10 deg = %d, yaw 270 deg sent as -90 = %d (rad x 10000)", i16(3), i16(5), i16(7));
    t.yaw_deg = -190.0f; crsf::build(crsf::FT_ATTITUDE, t, b); CHECK(i16(7) == 29671, "yaw -190 deg -> +170 deg = %d", i16(7));
    auto mode = [&](const crsf::TelemetryIn& x) { const size_t m = crsf::build(crsf::FT_FLIGHT_MODE, x, b); return ok(m) && b[m - 2] == 0 ? std::string((const char*)b + 3) : std::string("?"); };
    crsf::TelemetryIn m; CHECK(mode(m) == "ACRO*", "disarmed, nothing in the way: 'ACRO*' (%s)", mode(m).c_str());
    m.arming_blocked = true; CHECK(mode(m) == "!ERR", "disarmed with something that blocks arming: '!ERR'");
    m = crsf::TelemetryIn(); m.armed = true; m.angle = true; CHECK(mode(m) == "ANGL", "armed, ANGLE");
    m.althold = true; CHECK(mode(m) == "ALTH", "Altitude Hold (ANGLE comes with it)");
    m.poshold = true; CHECK(mode(m) == "POSH", "Position Hold");
    { crsf::TelemetryIn l = m; l.loiter = true; CHECK(mode(l) == "LOIT", "Loiter (0.41), before Position Hold"); }
    m.rescue = true; CHECK(mode(m) == "RTH", "GPS Rescue");
    m.failsafe = true; CHECK(mode(m) == "!FS!", "failsafe wins over everything");
    m = crsf::TelemetryIn(); m.armed = true; m.stabilize = true; CHECK(mode(m) == "STAB", "Stabilize");
    t.gps_fix = true; t.lat = 473977419; t.lon = -85455938; t.gs_cms = 1000; t.course_ddeg = 1234; t.alt_m = 123.4f; t.sats = 14; n = crsf::build(crsf::FT_GPS, t, b);
    auto i32 = [&](int o) { return (int32_t)(((uint32_t)b[o] << 24) | ((uint32_t)b[o + 1] << 16) | ((uint32_t)b[o + 2] << 8) | b[o + 3]); };
    CHECK(n == 19 && ok(n) && b[2] == 0x02 && i32(3) == 473977419 && i32(7) == -85455938 && ((b[11] << 8) | b[12]) == 360 && ((b[13] << 8) | b[14]) == 12340 && ((b[15] << 8) | b[16]) == 1123 && b[17] == 14,
          "GPS: lat / lon, 10 m/s = 360 (km/h x 10), course 123.4 deg = 12340, 123.4 m = 1123 (m + 1000), 14 satellites");
    t.gs_cms = 1002; crsf::build(crsf::FT_GPS, t, b); CHECK(((b[11] << 8) | b[12]) == 361, "10.02 m/s = 36.07 km/h -> 361 (rounded, as Betaflight)");
    t.gps_fix = false; crsf::build(crsf::FT_GPS, t, b); CHECK(i32(3) == 0 && i32(7) == 0 && b[17] == 14, "no fix: position 0, satellites still shown");
    t.vz_ms = -1.23f; n = crsf::build(crsf::FT_VARIO, t, b); CHECK(n == 6 && ok(n) && b[2] == 0x07 && i16(3) == -123, "vario: -1.23 m/s = -123 cm/s");
    // schedule: the whole list ten times a second, one frame at a time, no burst after a pause
    { crsf::Scheduler s; crsf::TelemetryIn x; x.have_gps = true; x.have_vario = true; int cnt[256] = {}, total = 0; uint32_t now = 4294000000u;
      for (int i = 0; i < 20000; i++) { now += 100; const size_t k = s.next(now, x, b); if (k) { cnt[b[2]]++; total++; } }
      CHECK(total == 100 && cnt[0x1E] == 20 && cnt[0x08] == 20 && cnt[0x21] == 20 && cnt[0x02] == 20 && cnt[0x07] == 20, "2 s with GPS and vario: 100 frames, 20 of each (across the 32-bit clock wrap) (%d)", total);
      now += 5000000; int burst = 0; for (int i = 0; i < 100; i++) { now += 100; if (s.next(now, x, b)) burst++; }
      CHECK(burst == 1, "after 5 s without calls: one frame, not a burst of the missed ones (%d in 10 ms)", burst); }
    { crsf::Scheduler s; crsf::TelemetryIn x; int total = 0, cnt[256] = {}; uint32_t now = 0;
      for (int i = 0; i < 10000; i++) { now += 100; if (s.next(now, x, b)) { total++; cnt[b[2]]++; } }
      CHECK(total == 30 && cnt[0x02] == 0 && cnt[0x07] == 0, "no GPS, no altitude: attitude, battery and mode only, still ten rounds a second (%d in 1 s)", total); }
}

// ---- nRF24 (0.38): the flight controller's side of the Windburst link, against a scripted transmitter
struct RxBench {
    uint32_t clock = 0, ms = 0; Air air; SimNrf24 tx_chip, rx_chip; SimBus tx_bus{&tx_chip, &clock}, rx_bus{&rx_chip, &clock}; nrf::Radio tx; rx::Nrf24Rx rx;
    nrf::Config pair; uint8_t seq = 0; uint8_t last_ack[32] = {}; uint8_t last_ack_n = 0;
    RxBench(bool bound) {
        const uint8_t a[5] = {0xA7, 1, 2, 3, 4}; memcpy(pair.addr, a, 5); pair.rf_channel = 90;
        air.attach(&tx_chip); air.attach(&rx_chip); tx.begin(tx_bus, true, bound ? pair : bind_cfg()); rx.begin(rx_bus, pair, bound, ms);
    }
    static nrf::Config bind_cfg() { nrf::Config b; memcpy(b.addr, wbr::kBindAddr, 5); b.rf_channel = wbr::kBindChannel; b.power = wbp::NRF_POWER_MIN; return b; }
    // the transmitter sends p, the air carries it, the receiver reads it: 10 ms later
    bool send(const uint8_t* p, uint8_t n) {
        tx.send(p, n); air.step(clock); last_ack_n = 0; const bool ok = tx.poll_tx(last_ack, last_ack_n) == nrf::Radio::ACKED;
        clock += 10000; ms += 10; rx.update(clock, ms); return ok;
    }
    bool control(uint16_t roll) { uint16_t ch[8] = {roll, 1500, 1000, 1500, 1000, 1000, 1000, 1000}; uint8_t b[32]; return send(b, wbr::encode_control(seq++, ch, b)); }
};

static void t_nrf24() {
    printf("[nRF24 receiver]\n");
    { RxBench b(true);
      CHECK(b.rx.present() && b.rx.bind_state() == wbp::BIND_STATE_BOUND && b.rx_chip.reg[nrf::RF_CH] == 90 && b.rx_chip.rx_addr[1][0] == 0xA7, "bound: listening on its address and RF channel");
      b.control(1700); const uint16_t* c = b.rx.channels();
      CHECK(b.rx.packets() == 1 && c[0] == 1700 && c[1] == 1500 && c[2] == 1000 && c[8] == 1000 && c[15] == 1000, "a control packet: 8 channels, the other 8 at 1000 (%u)", c[0]);
      const uint8_t bad[13] = {wbr::T_CONTROL, 9, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 0x55}; b.rx_chip.inject_rx(bad, 13); b.ms += 10; b.rx.update(b.clock, b.ms);
      CHECK(b.rx.bad() == 1 && b.rx.channels()[0] == 1700 && b.rx.packets() == 1, "a packet the radio passed but whose CRC8 is wrong: dropped, counted, channels kept");
      uint16_t ch[8] = {1200, 1500, 1000, 1500, 1000, 1000, 1000, 1000}; uint8_t p[32]; const uint8_t n = wbr::encode_control((uint8_t)(b.seq - 1), ch, p); b.rx_chip.inject_rx(p, n); b.ms += 10; b.rx.update(b.clock, b.ms);
      CHECK(b.rx.channels()[0] == 1700 && b.rx.packets() == 1, "an old sequence number (a replay): dropped");
      wbr::Telemetry t; t.armed = true; t.mode = wbp::FLY_MODE_POSHOLD; t.batt_cv = 1490; b.rx.set_telemetry(t);
      b.control(1500); b.control(1500); wbr::Telemetry got;
      CHECK(wbr::decode_telemetry(b.last_ack, b.last_ack_n, got) && got.armed && got.mode == wbp::FLY_MODE_POSHOLD && got.batt_cv == 1490, "set_telemetry rides on the ACKs");
      for (int i = 0; i < 120; i++) b.control(1500);
      CHECK(b.rx.lq() >= 98, "100 packets a second of the 100 expected: link quality %u %%", b.rx.lq());
      CHECK(wbr::decode_telemetry(b.last_ack, b.last_ack_n, got) && got.lq >= 98, "and the transmitter is told (%u %%)", got.lq);
      b.air.loss = 0.6; for (int i = 0; i < 200; i++) b.control(1500);
      CHECK(b.rx.lq() > 60 && b.rx.lq() < 95, "60 %% of the attempts lost (3 retransmissions): link quality %u %%", b.rx.lq()); }
    { // never bound: listening for a bind for 60 s
      RxBench b(false); CHECK(b.rx.bind_state() == wbp::BIND_STATE_LISTEN && b.rx_chip.reg[nrf::RF_CH] == wbr::kBindChannel && !memcmp(b.rx_chip.rx_addr[1], wbr::kBindAddr, 5), "never bound: LISTEN on the bind address / channel");
      wbr::BindOffer o; const uint8_t na[5] = {0x6E, 5, 6, 7, 8}; memcpy(o.addr, na, 5); o.rf_channel = 70; uint8_t p[32]; const uint8_t n = wbr::encode_bind(o, p);
      b.send(p, n); uint8_t echo[5];
      CHECK(b.rx_chip.reg[nrf::RF_CH] == wbr::kBindChannel, "the first offer: still on the bind channel (its echo goes out with the next ACK)");
      const bool acked = b.send(p, n);
      CHECK(acked && wbr::decode_bind_ack(b.last_ack, b.last_ack_n, echo) && !memcmp(echo, na, 5), "the second offer is ACKed with the address echoed");
      CHECK(b.rx_chip.reg[nrf::RF_CH] == 70 && b.rx_chip.rx_addr[1][0] == 0x6E && b.rx.bind_state() == wbp::BIND_STATE_BINDING, "then it moves to the new address / channel, not yet BOUND");
      nrf::Config now; CHECK(!b.rx.take_bound(now), "nothing to store before a control packet arrived there");
      nrf::Config moved = b.pair; memcpy(moved.addr, na, 5); moved.rf_channel = 70; b.tx.configure(moved); b.control(1600);
      CHECK(b.rx.bind_state() == wbp::BIND_STATE_BOUND && b.rx.take_bound(now) && now.addr[0] == 0x6E && now.rf_channel == 70 && b.rx.channels()[0] == 1600, "the first control packet there: BOUND, the new settings to store"); }
    { RxBench b(false); wbr::BindOffer o; const uint8_t na[5] = {0x6F, 5, 6, 7, 8}; memcpy(o.addr, na, 5); o.rf_channel = 70; uint8_t p[32]; const uint8_t n = wbr::encode_bind(o, p);
      b.send(p, n); b.send(p, n); for (int i = 0; i < 250; i++) { b.clock += 10000; b.ms += 10; b.rx.update(b.clock, b.ms); }
      CHECK(b.rx.bind_state() == wbp::BIND_STATE_LISTEN && b.rx_chip.reg[nrf::RF_CH] == wbr::kBindChannel, "moved, but no control packet within 2 s (the transmitter missed the echo): back to listening for the bind"); }
    { RxBench b(false); b.ms = 60001; b.clock = 60001000; b.rx.update(b.clock, b.ms);
      wbr::BindOffer o; o.addr[0] = 0x70; uint8_t p[32]; const uint8_t n = wbr::encode_bind(o, p); b.send(p, n); b.send(p, n);
      CHECK(b.rx.bind_state() == wbp::BIND_STATE_IDLE && b.rx_chip.reg[nrf::RF_CH] == 90, "60 s after power-up the bind window closes: IDLE, offers ignored"); }
    { RxBench b(true); CHECK(!b.rx.start_bind(1000, 30000, true), "BIND refused while armed");
      CHECK(b.rx.start_bind(1000, 30000, false) && b.rx.bind_state() == wbp::BIND_STATE_LISTEN, "BIND from the app: LISTEN"); b.rx.stop_bind();
      CHECK(b.rx.bind_state() == wbp::BIND_STATE_BOUND && b.rx_chip.reg[nrf::RF_CH] == 90, "stopped: back on its own address, BOUND"); }
    { // radio settings over the air: move at once, back after 1 s without a control packet there
      RxBench b(true); b.control(1500);
      wbr::RadioCfg rc; rc.rf_channel = 50; uint8_t p[32]; const uint8_t n = wbr::encode_config(rc, p); b.send(p, n);
      CHECK(b.rx_chip.reg[nrf::RF_CH] == 50, "config packet: on RF channel 50 at once (its ACK went out when it arrived)");
      nrf::Config c50 = b.pair; c50.rf_channel = 50; b.tx.configure(c50); b.control(1650);
      CHECK(b.rx.channels()[0] == 1650 && b.rx.config().rf_channel == 50, "control packets there: it stays");
      nrf::Config ch50; CHECK(b.rx.take_changed(ch50) && ch50.rf_channel == 50 && !b.rx.take_changed(ch50), "confirmed there: the new settings to store (once)");
      wbr::RadioCfg r2; r2.rf_channel = 30; const uint8_t n2 = wbr::encode_config(r2, p); b.send(p, n2);
      for (int i = 0; i < 120; i++) { b.clock += 10000; b.ms += 10; b.rx.update(b.clock, b.ms); }
      CHECK(b.rx_chip.reg[nrf::RF_CH] == 50 && b.rx.config().rf_channel == 50, "moved to 30 but nobody came within 1 s: back on 50"); }
}

// ---- final review (0.38.0): findings C2, I4, I5
static void t_nrf24_review() {
    printf("[nRF24 receiver: review findings]\n");
    { RxBench b(false); b.ms = 60001; b.clock = 60001000; b.rx.update(b.clock, b.ms); b.tx.configure(b.pair);
      b.control(1800); CHECK(b.rx.bind_state() == wbp::BIND_STATE_IDLE && b.rx.packets() == 0 && b.rx.channels()[0] == 1500, "never bound: control packets on its (default) address are not taken"); }
    { RxBench b(true); b.control(1500); wbr::Telemetry t; t.armed = true; b.rx.set_telemetry(t);
      wbr::RadioCfg rc; rc.rf_channel = 40; uint8_t p[32]; const uint8_t n = wbr::encode_config(rc, p); b.send(p, n);
      CHECK(b.rx_chip.reg[nrf::RF_CH] == 90 && b.rx.config().rf_channel == 90, "armed: a radio-settings packet is ignored (%u)", b.rx_chip.reg[nrf::RF_CH]); }
    { RxBench b(true); b.control(1500); b.rx_chip = SimNrf24();
      for (int i = 0; i < 30; i++) b.control(1600);
      CHECK(b.rx.channels()[0] == 1600 && b.rx_chip.reg[nrf::RF_CH] == 90 && b.rx.reinits() >= 1, "the chip lost its registers: set up again, packets flow (%u re-inits)", b.rx.reinits()); }
}

static void t_nrf24_minor() {
    printf("[nRF24 receiver: review minors]\n");
    { RxBench b(false); wbr::BindOffer o; const uint8_t na[5] = {0x71, 5, 6, 7, 8}; memcpy(o.addr, na, 5); o.rf_channel = 75; uint8_t p[32]; const uint8_t n = wbr::encode_bind(o, p);
      b.send(p, n); b.send(p, n); const bool binding = b.rx.bind_state() == wbp::BIND_STATE_BINDING; b.rx.stop_bind();
      CHECK(binding && b.rx.config().addr[0] == 0xA7 && b.rx_chip.rx_addr[1][0] == 0xA7, "M1: stopped while confirming a bind: back on its own address, the offered one forgotten (0x%02X)", b.rx.config().addr[0]); }
    { RxBench b(false); b.tx_chip.distant = true; wbr::BindOffer o; o.addr[0] = 0x72; uint8_t p[32]; const uint8_t n = wbr::encode_bind(o, p);
      b.send(p, n); b.send(p, n); b.send(p, n);
      CHECK(b.rx.bind_state() == wbp::BIND_STATE_LISTEN && b.rx_chip.reg[nrf::RF_CH] == wbr::kBindChannel, "M2: an offer from far away (RPD 0, someone else's bind) is not taken"); }
}

// MAVLink frames: the pymavlink ones (test/mavlink_vectors.h) and ones made by mav::pack (checked against pymavlink in test_mavlink)
static Bytes vec_bytes(const char* name) { for (const MavVector& v : kMavVectors) if (!strcmp(v.name, name)) return Bytes(v.bytes, v.bytes + v.n); printf("no vector %s\n", name); return Bytes(); }
static Bytes mav_override(const uint16_t chan[18], uint8_t seq = 0) {
    mav::RcChannelsOverride o; o.target_system = 1; for (int i = 0; i < 18; i++) o.chan[i] = chan[i];
    mav::Encoder e; e.sysid = 255; e.compid = 68; e.set_seq(seq); uint8_t f[mav::kMaxFrame]; const size_t n = mav::pack(e, o, f); return Bytes(f, f + n);
}
static Bytes mav_status(uint8_t rssi, uint8_t remrssi, uint8_t txbuf) {
    mav::RadioStatus r; r.rssi = rssi; r.remrssi = remrssi; r.txbuf = txbuf;
    mav::Encoder e; e.sysid = 255; e.compid = 68; uint8_t f[mav::kMaxFrame]; const size_t n = mav::pack(e, r, f); return Bytes(f, f + n);
}

static void t_mavlink() {
    printf("[MAVLink receiver (ExpressLRS in MAVLink mode)]\n");
    rx::MavlinkDecoder d; rx::Frame f;
    CHECK(!strcmp(d.name(), "MAVLINK") && d.baud() == 460800 && !d.inverted() && !d.even_parity_2stop(), "460800 8N1, not inverted (what ExpressLRS uses)");
    CHECK(!d.radio_status_seen() && d.link_quality() == 0 && d.radio_status_count() == 0, "nothing heard yet: no RADIO_STATUS, link quality 0");
    { const uint16_t v[16] = {1500, 1250, 988, 1750, 2012, 1000, 1500, 2000, 1100, 1200, 1300, 1400, 1600, 1700, 1800, 1900};
      CHECK(feed(d, vec_bytes("rc_channels_override"), f) == 1, "pymavlink's RC_CHANNELS_OVERRIDE: one RC frame");
      bool all = true; for (int i = 0; i < 16; i++) if (f.ch[i] != v[i]) all = false;
      CHECK(all && !f.failsafe && !f.frame_lost, "the 16 channels in microseconds as sent (ch 3 %u, ch 9 %u, ch 16 %u)", f.ch[2], f.ch[8], f.ch[15]); }
    { uint16_t c[18] = {}; for (int i = 0; i < 16; i++) c[i] = 1600; c[0] = 3000; c[3] = 500; c[1] = 0; c[2] = 65535;
      CHECK(feed(d, mav_override(c, 1), f) == 1 && f.ch[0] == 2115 && f.ch[3] == 885 && f.ch[4] == 1600, "out of range: clamped to 885 .. 2115 (%u, %u)", f.ch[0], f.ch[3]);
      CHECK(f.ch[1] == 1250 && f.ch[2] == 988, "0 and 65535 = no change (MAVLink): channels 2 and 3 keep 1250 and 988 (%u, %u)", f.ch[1], f.ch[2]); }
    { const uint32_t errs = d.errors();
      CHECK(feed(d, vec_bytes("radio_status"), f) == 0 && d.radio_status_seen() && d.radio_status_count() == 1 && d.txbuf() == 87, "RADIO_STATUS: not an RC frame; 87 %% of the radio's buffer free");
      CHECK(d.link_quality() == 80 && d.rssi_dbm() == -70 && d.errors() == errs, "rssi 204 = link quality 80 %% (ExpressLRS sends LQ x 2.55), remote rssi 70 = -70 dBm (%u %%, %d dBm)", d.link_quality(), d.rssi_dbm());
      feed(d, mav_status(255, 50, 12), f); CHECK(d.txbuf() == 12 && d.link_quality() == 100 && d.radio_status_count() == 2, "the next one: 12 %% free, 100 %%"); }
    { const uint32_t errs = d.errors(); Bytes b = vec_bytes("heartbeat"); const Bytes s = vec_bytes("statustext"); b.insert(b.end(), s.begin(), s.end());
      CHECK(feed(d, b, f) == 0 && d.errors() == errs, "a ground station's HEARTBEAT and STATUSTEXT: ignored, not errors"); }
    { uint16_t c[18] = {}; for (int i = 0; i < 16; i++) c[i] = 1400; Bytes b = mav_override(c, 2); b[14] ^= 0x10; const uint32_t errs = d.errors();
      Bytes good = mav_override(c, 3); Bytes all = {0x00, 0x13, 0xFE}; all.insert(all.end(), b.begin(), b.end()); all.insert(all.end(), good.begin(), good.end());
      CHECK(feed(d, all, f) == 1 && d.errors() == errs + 1 && f.ch[0] == 1400, "a damaged frame: dropped and counted (%u), the next one read", d.errors() - errs); }
    {   // 0.40: what the ground station sends through the link waits in a small queue for mav::Gcs (FlightApp drains it)
        rx::MavlinkDecoder q; Bytes b = vec_bytes("command_long"); const Bytes c2 = vec_bytes("mission_count"), rs = vec_bytes("radio_status"), ov = vec_bytes("rc_channels_override"), hb = vec_bytes("heartbeat");
        for (const Bytes* x : {&rs, &ov, &c2, &hb}) b.insert(b.end(), x->begin(), x->end());
        feed(q, b, f); mav::Parser::Frame g1, g2, g3, g4;
        CHECK(q.take_gcs(g1) && g1.msgid == mav::MSG_COMMAND_LONG && q.take_gcs(g2) && g2.msgid == mav::MSG_MISSION_COUNT && q.take_gcs(g3) && g3.msgid == mav::MSG_HEARTBEAT && !q.take_gcs(g4),
              "COMMAND_LONG, MISSION_COUNT, HEARTBEAT queued in order; RC_CHANNELS_OVERRIDE and RADIO_STATUS are the decoder's own");
        Bytes many; for (int i = 0; i < rx::MavlinkDecoder::kGcsQueue + 2; i++) many.insert(many.end(), c2.begin(), c2.end());
        feed(q, many, f); int n = 0; while (q.take_gcs(g1)) n++;
        CHECK(n == rx::MavlinkDecoder::kGcsQueue && q.gcs_dropped() == 2, "a full queue (%d) drops what comes after, and counts it (%u)", rx::MavlinkDecoder::kGcsQueue, q.gcs_dropped());
        feed(q, c2, f); q.reset(); CHECK(!q.take_gcs(g1), "reset() empties it"); }
    { d.reset(); uint16_t c[18] = {}; c[5] = 1234;
      CHECK(!d.radio_status_seen() && d.link_quality() == 0 && feed(d, mav_override(c), f) == 1 && f.ch[0] == 1500 && f.ch[5] == 1234, "reset(): channels back to 1500, no RADIO_STATUS"); }
}

int main() {
    t_crsf(); t_sbus(); t_ibus(); t_spektrum(); t_ppm(); t_pwm(); t_edge_ring(); t_crsf_telemetry(); t_link(); t_nrf24(); t_nrf24_review(); t_nrf24_minor(); t_mavlink();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
