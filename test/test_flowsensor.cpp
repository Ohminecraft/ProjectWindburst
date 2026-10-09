// Optical flow drivers on a PC: the MSP v2 parser and the MSP flow / lidar driver against hand-built frames, the PMW3901 driver
// against the register-level chip simulator, the FlowSensor front end against the Arduino stubs.
// Build: g++ -std=c++17 -O2 -DARDUINO=1 -Itest/arduino_stub test/test_flowsensor.cpp -o build/test_flowsensor
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <deque>
#include <vector>
#include "../src/hal/flow/flow_sensor.h"
#include "../src/app/sync.h"
#include "sim_flow.h"

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, ...) do { const bool ok_ = (cond); if (getenv("V")) { printf("    "); printf(__VA_ARGS__); printf("\n"); } if (ok_) g_pass++; else { g_fail++; printf("  FAIL line %d: %s   ", __LINE__, #cond); printf(__VA_ARGS__); printf("\n"); } } while (0)

// ---- Arduino stubs: the second SPI bus goes to the simulated PMW3901, chip select is a GPIO
int g_spi_begin_pins[4];
static Pmw3901Sim* g_chip = nullptr; static uint8_t g_cs_pin = 255;
void digitalWrite(uint8_t pin, uint8_t val) { if (pin < 64) g_stub_gpio[pin] = val; if (g_chip && pin == g_cs_pin) g_chip->cs(val == LOW); }
uint8_t stub_spi_transfer(uint8_t) { return 0xFF; }
bool stub_i2c_probe(uint8_t) { return true; }
int stub_i2c_tx(uint8_t, const uint8_t*, size_t, bool) { return 2; }
int stub_i2c_rx(uint8_t, uint8_t*, size_t) { return 0; }

// ---- an MSP v2 frame, built here from the published format (not with the driver's code)
static uint8_t crc_ref(const uint8_t* p, size_t n) {                // CRC-8/DVB-S2, table-free reference
    uint8_t c = 0;
    for (size_t i = 0; i < n; i++) { c ^= p[i]; for (int b = 0; b < 8; b++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0xD5) : (uint8_t)(c << 1); }
    return c;
}
static std::vector<uint8_t> msp(uint16_t cmd, const std::vector<uint8_t>& payload, char dir = '<', uint8_t flags = 0) {
    std::vector<uint8_t> f = {'$', 'X', (uint8_t)dir, flags, (uint8_t)cmd, (uint8_t)(cmd >> 8), (uint8_t)payload.size(), (uint8_t)(payload.size() >> 8)};
    f.insert(f.end(), payload.begin(), payload.end());
    f.push_back(crc_ref(f.data() + 3, f.size() - 3));
    return f;
}
static std::vector<uint8_t> flow_frame(uint8_t q, int32_t mx, int32_t my) { std::vector<uint8_t> p(9); p[0] = q; memcpy(&p[1], &mx, 4); memcpy(&p[5], &my, 4); return msp(0x1F02, p); }
static std::vector<uint8_t> range_frame(uint8_t q, int32_t mm) { std::vector<uint8_t> p(5); p[0] = q; memcpy(&p[1], &mm, 4); return msp(0x1F01, p); }
struct Wire : flow::BytePort { std::deque<uint8_t> q; uint32_t reads = 0; int read() override { if (q.empty()) return -1; reads++; const uint8_t c = q.front(); q.pop_front(); return c; }
    void put(const std::vector<uint8_t>& f) { for (uint8_t c : f) q.push_back(c); } };

static void t_parser() {
    printf("[msp v2] frames out of a byte stream\n");
    { const uint8_t s[] = "123456789"; uint8_t c = 0; for (int i = 0; i < 9; i++) c = flow::crc8_dvb_s2(c, s[i]);
      CHECK(c == 0xBC, "CRC-8/DVB-S2 of \"123456789\" is 0xBC (0x%02X)", c); }
    { flow::MspV2Parser p; const auto f = flow_frame(200, 1234, -5678); int done = 0, at = -1;
      for (size_t i = 0; i < f.size(); i++) if (p.feed(f[i])) { done++; at = (int)i; }
      int32_t mx, my; memcpy(&mx, p.payload() + 1, 4); memcpy(&my, p.payload() + 5, 4);
      CHECK(done == 1 && at == (int)f.size() - 1 && p.command() == 0x1F02 && p.size() == 9 && p.payload()[0] == 200 && mx == 1234 && my == -5678 && p.frames() == 1 && p.crc_errors() == 0, "a flow frame: complete on its last byte, command 0x%04X, %u bytes", p.command(), p.size()); }
    { flow::MspV2Parser p; auto f = flow_frame(200, 1, 2); f.back() ^= 0x01; int done = 0; for (uint8_t c : f) done += p.feed(c);
      const auto g = range_frame(255, 1500); for (uint8_t c : g) done += p.feed(c);
      CHECK(done == 1 && p.crc_errors() == 1 && p.frames() == 1 && p.command() == 0x1F01, "a frame with a wrong checksum is dropped and counted; the next one is read"); }
    { flow::MspV2Parser p; auto f = flow_frame(200, 1, 2); f[9] ^= 0x40; int done = 0; for (uint8_t c : f) done += p.feed(c);
      CHECK(done == 0 && p.crc_errors() == 1, "one flipped bit in the payload: dropped"); }
    { flow::MspV2Parser p; int done = 0; const uint8_t junk[] = {0x00, 0xFF, '$', 'M', '<', '$', '$', 'X', 'x', '$', 'X'};
      for (uint8_t c : junk) done += p.feed(c);
      const auto f = range_frame(100, 777); for (size_t i = 2; i < f.size(); i++) done += p.feed(f[i]);       // (the junk ended with "$X": the frame carries on from there)
      CHECK(done == 1 && p.command() == 0x1F01 && p.crc_errors() == 0, "noise, an MSP v1 header and a doubled '$' before the frame: found all the same"); }
    { flow::MspV2Parser p; int done = p.feed('$'); for (uint8_t c : flow_frame(7, 1, 2)) done += p.feed(c);
      CHECK(done == 1 && p.payload()[0] == 7, "a stray '$' right before a frame: the frame's own '$' starts it again"); }
    { flow::MspV2Parser p; int done = 0; for (uint8_t c : msp(0x1F01, {1, 2, 3, 4, 5}, '>')) done += p.feed(c); for (uint8_t c : msp(0x1F01, {1, 2, 3, 4, 5}, '!')) done += p.feed(c);
      CHECK(done == 1 && p.frames() == 1, "direction '>' (a reply) is a frame as well; '!' (an error) is not"); }
    { flow::MspV2Parser p; int done = 0; for (uint8_t c : msp(0x0102, {})) done += p.feed(c);
      CHECK(done == 1 && p.size() == 0 && p.command() == 0x0102, "a frame without a payload"); }
    { flow::MspV2Parser p; int done = 0; std::vector<uint8_t> big(65, 0x11); for (uint8_t c : msp(0x1F02, big)) done += p.feed(c);
      const int after_big = done; for (uint8_t c : flow_frame(1, 2, 3)) done += p.feed(c);
      CHECK(after_big == 0 && p.too_long() == 1 && done == 1, "a payload longer than the buffer (65 bytes) is skipped and counted; the parser is in step again for the next frame");
      flow::MspV2Parser q; int d2 = 0; std::vector<uint8_t> fits(64, 0x22); for (uint8_t c : msp(0x1F02, fits)) d2 += q.feed(c);
      CHECK(d2 == 1 && q.size() == 64 && q.payload()[63] == 0x22 && q.too_long() == 0, "64 bytes fit"); }
    { flow::MspV2Parser p; int done = 0; auto f = flow_frame(9, 8, 7); f[3] = 0x5A; f.back() = crc_ref(f.data() + 3, f.size() - 4);
      for (uint8_t c : f) done += p.feed(c);
      CHECK(done == 1, "the flags byte is part of the checksum (a frame with flags 0x5A and the matching checksum is accepted)");
      flow::MspV2Parser q; f[3] = 0x00; int d2 = 0; for (uint8_t c : f) d2 += q.feed(c);
      CHECK(d2 == 0 && q.crc_errors() == 1, "... and the same checksum with other flags is not"); }
}

static void t_accumulator() {
    printf("[accumulator] several coarse samples make one reading\n");
    flow::Accumulator a; flow::Reading r;
    CHECK(!a.ready() && a.count() == 0, "empty: nothing to hand over");
    for (int i = 0; i < 4; i++) a.add(0.005f, -0.010f, 10000, 90);
    CHECK(!a.ready() && a.count() == 4 && fabsf(a.seconds() - 0.040f) < 1e-7f, "four samples of 10 ms: not yet");
    a.add(0.010f, -0.010f, 10000, 60);
    CHECK(a.ready(), "the fifth: 50 ms are covered");
    a.take(r);
    CHECK(fabsf(r.x - 0.6f) < 1e-5f && fabsf(r.y + 1.0f) < 1e-5f && fabsf(r.dt - 0.050f) < 1e-7f && r.quality == 60 && a.count() == 0 && !a.ready(),
          "30 mrad and -50 mrad in 50 ms = %.3f / %.3f rad/s; the worst quality of the five counts (%u); the next sum starts empty", r.x, r.y, r.quality);
    a.add(0.001f, 0, 44999, 100); CHECK(!a.ready(), "44.999 ms: not yet"); a.add(0, 0, 1, 100); CHECK(a.ready(), "45 ms: ready");
    a.reset(); CHECK(!a.ready() && a.count() == 0, "reset() throws a started sum away");
    a.add(0.010f, 0, 120000, 80); CHECK(a.ready(), "one sample that covers more than the window is a reading by itself"); a.take(r); CHECK(fabsf(r.x - 0.010f / 0.12f) < 1e-6f && r.quality == 80, "(%.4f rad/s)", r.x);
    a.add(0.005f, 0, 0, 100); a.add(0.005f, 0, 50000, 100); a.take(r);
    CHECK(fabsf(r.x - 0.2f) < 1e-6f, "a sample without a time of its own (it waited together with the next) still counts: 10 mrad in 50 ms");
}

// feeds `counts.size()` flow frames, one every `step_us`; returns how many of the update() calls delivered a reading
static int feed(Wire& w, flow::MspFlow& d, uint32_t& t, const std::vector<int>& counts, flow::Reading& r, uint32_t step_us = 10000, uint8_t q = 255) {
    int got = 0; for (int c : counts) { t += step_us; w.put(flow_frame(q, c, -c)); got += d.update(t, r); } return got;
}

static void t_msp_flow() {
    printf("[msp flow] MicoAir MTF-01 style sensor: flow and its lidar\n");
    { Wire w; flow::MspFlow d(w); flow::Reading r; uint32_t t = 0;
      CHECK(d.init() && !d.update(1000, r) && strcmp(d.name(), "MSP flow") == 0 && fabsf(d.nominal_rate_hz() - 20.0f) < 1e-6f, "nothing on the wire: no reading");
      const int first = feed(w, d, t, {100}, r), four = feed(w, d, t, {100, 100, 100, 100}, r), fifth = feed(w, d, t, {100}, r);
      CHECK(first == 0 && four == 0 && fifth == 1 && fabsf(r.x - 50.0f) < 1e-3f && fabsf(r.y + 50.0f) < 1e-3f, "the first frame only starts the timing; five more, 10 ms apart, make one reading: 500 counts in 50 ms = %.2f rad/s (motion / 200 / time, as Betaflight)", r.x);
      CHECK(r.quality == 100 && fabsf(r.dt - 0.080f) < 1e-6f && fabsf(r.age_s - 0.045f) < 1e-6f && d.samples() == 6 && d.flow_frames() == 6, "quality 255 = 100 %%; the reading describes 80 ms (five frames of a 40 ms window each) that ended 45 ms ago");
      // what a single frame cannot tell: 0.6 rad/s is 1.2 counts a frame
      CHECK(feed(w, d, t, {1, 1, 1, 1, 2}, r) == 1 && fabsf(r.x - 0.6f) < 1e-5f, "frames of 1, 1, 1, 1, 2 counts: %.3f rad/s (alone they would read 0.5 or 1.0)", r.x);
      // the thread does not look exactly every 10 ms
      { int got = 0; const uint32_t gaps[5] = {9000, 12000, 8000, 11000, 10000}; for (uint32_t g : gaps) { t += g; w.put(flow_frame(255, 2, -2)); got += d.update(t, r); }
        CHECK(got == 1 && fabsf(r.x - 1.0f) < 1e-5f, "frames seen 8 .. 12 ms apart: the sum over 50 ms is still exactly %.3f rad/s (single frames: 0.83 .. 1.25)", r.x); }
      // two frames waiting together
      { w.put(flow_frame(255, 2, -2)); w.put(flow_frame(255, 2, -2)); t += 20000; int got = d.update(t, r); got += feed(w, d, t, {2, 2, 2}, r);
        CHECK(got == 1 && fabsf(r.x - 1.0f) < 1e-5f && d.flow_frames() == 21, "the thread came 10 ms late and found two frames: both counted, no division by zero (%.3f rad/s)", r.x); }
      // quality: the worst frame of the reading
      { t += 10000; w.put(flow_frame(51, 2, -2)); int got = d.update(t, r); got += feed(w, d, t, {2, 2, 2, 2}, r); CHECK(got == 1 && r.quality == 20, "one frame of five with quality 51 / 255: the reading has %u %%", r.quality); }
      // a pause
      { feed(w, d, t, {40, 40}, r); t += 1500000; w.put(flow_frame(255, 40, -40)); const bool pause = d.update(t, r); const int four2 = feed(w, d, t, {2, 2, 2, 2}, r), fifth2 = feed(w, d, t, {2}, r);
        CHECK(!pause && four2 == 0 && fifth2 == 1 && fabsf(r.x - 1.0f) < 1e-5f, "1.5 s of silence: what was summed before is thrown away, the frame after it only restarts the timing (%.3f rad/s from the five that follow)", r.x); }
      CHECK(d.range_frames() == 0, "(no range frame so far)"); }
    { Wire w; flow::MspFlow d(w); flow::Reading r; flow::RangeSample s; d.init();
      CHECK(!d.take_range(s), "no distance yet");
      w.put(range_frame(255, 1234)); const bool flow_new = d.update(5000, r);
      CHECK(!flow_new && d.take_range(s) && fabsf(s.distance_m - 1.234f) < 1e-6f && s.quality_pct == 100 && !d.take_range(s), "range frame 1234 mm: 1.234 m, handed over once (and it is not a flow reading)");
      w.put(range_frame(0, 80)); d.update(6000, r); CHECK(d.take_range(s) && fabsf(s.distance_m - 0.080f) < 1e-6f && s.quality_pct == 0, "quality 0 is passed on as 0");
      w.put(range_frame(255, -1)); d.update(7000, r); CHECK(!d.take_range(s) && d.range_frames() == 3, "a negative distance (out of range) is no measurement");
      w.put(range_frame(255, 900)); w.put(range_frame(255, -1)); d.update(8000, r); CHECK(!d.take_range(s), "out of range after a good one in the same pass: the stale distance is not handed over");
      w.put(msp(0x1F01, {1, 2, 3, 4})); w.put(msp(0x1F02, {1, 2, 3, 4, 5, 6, 7, 8})); w.put(msp(0x1F03, {1, 2, 3, 4, 5, 6, 7, 8, 9})); d.update(9000, r);
      CHECK(!d.take_range(s) && d.range_frames() == 5 && d.flow_frames() == 0 && d.samples() == 0 && d.parser().frames() == 8, "frames that are too short, and commands this driver does not know, are ignored"); }
    { // both kinds mixed, and never more than 256 bytes per call
      Wire w; flow::MspFlow d(w); flow::Reading r; flow::RangeSample s; d.init(); uint32_t t = 0;
      feed(w, d, t, {0, 4, 4, 4, 4}, r);
      t += 10000; w.put(range_frame(255, 500)); w.put(flow_frame(255, 4, 0)); CHECK(d.update(t, r) && fabsf(r.x - 2.0f) < 1e-4f && d.take_range(s) && fabsf(s.distance_m - 0.5f) < 1e-6f, "a range and a flow frame in one pass: both come out");
      for (int i = 0; i < 40; i++) w.put(flow_frame(255, 1, 1));       // 40 x 18 = 720 bytes
      const uint32_t r0 = w.reads; d.update(t + 10000, r);
      CHECK(w.reads - r0 == 256 && w.q.size() == 720 - 256, "a call reads 256 bytes at most (%u): the thread has other sensors to serve", (unsigned)(w.reads - r0)); }
    { // against the simulated sensor: what it sends is what comes out
      MspFlowSim sim; sim.host_baud = 115200; FlowTruth truth; struct P : flow::BytePort { MspFlowSim* s; int read() override { return s->read(); } } port; port.s = &sim;
      flow::MspFlow d(port); d.init(); flow::Reading r; double sx = 0, sy = 0, worst = 0; int n = 0; float range = 0; flow::RangeSample rs;
      for (uint32_t ms = 1; ms <= 2000; ms++) { truth.push(0.30f, -0.20f, 0.5f, 0.25f, 1.5f, 1.0f); sim.tick(ms, truth);
          if (d.update(ms * 1000u, r) && ms > 200) { sx += r.x; sy += r.y; n++; worst = fmax(worst, fmax(fabs(r.x - (0.30 - 0.25 / 1.5)), fabs(r.y - (-0.20 + 0.5 / 1.5)))); } if (d.take_range(rs)) range = rs.distance_m; }
      // x = roll rate - v_right / range = 0.30 - 0.25 / 1.5;  y = pitch rate + v_forward / range = -0.20 + 0.5 / 1.5
      CHECK(n >= 34 && n <= 37 && fabs(sx / n - (0.30 - 0.25 / 1.5)) < 0.01 && fabs(sy / n - (-0.20 + 0.5 / 1.5)) < 0.01 && fabsf(range - 1.5f) < 0.002f && r.quality == 78,
            "2 s of a simulated MTF-01: %d readings, mean rate %.3f / %.3f rad/s (true %.3f / %.3f), lidar %.3f m, quality %u", n, sx / n, sy / n, 0.30 - 0.25 / 1.5, -0.20 + 0.5 / 1.5, range, r.quality);
      CHECK(worst < 0.105, "no single reading is further off than one count in 50 ms (%.3f rad/s; a single frame: up to 0.5)", worst);
      sim.corrupt_next = true; const uint32_t e0 = d.parser().crc_errors(); for (uint32_t ms = 2001; ms <= 2040; ms++) { truth.push(0, 0, 0, 0, 1.5f, 1.0f); sim.tick(ms, truth); d.update(ms * 1000u, r); }
      CHECK(d.parser().crc_errors() == e0 + 1, "a frame damaged on the wire: counted, nothing else happens"); }
}

// ---- PMW3901: the driver's port onto the simulated chip, with everything it does written down
struct ChipPort : flow::PixartPort {
    Pmw3901Sim* chip; std::vector<uint32_t> delays; int selects = 0, releases = 0; uint64_t waited_us = 0; bool nested = false, sel = false;
    void select(bool on) override { if (on == sel) nested = true; sel = on; if (on) selects++; else releases++; chip->cs(on); }
    uint8_t transfer(uint8_t v) override { return chip->xfer(v); }
    void delay_us(uint32_t us) override { delays.push_back(us); waited_us += us; }
};

static void t_pmw() {
    printf("[pmw3901] start-up, identity, motion bursts\n");
    { Pmw3901Sim chip; ChipPort port; port.chip = &chip; flow::Pmw3901 d(port);
      CHECK(d.init() && d.chip_id() == 0x49 && chip.resets == 1 && strcmp(d.name(), "PMW3901") == 0 && fabsf(d.nominal_rate_hz() - 20.0f) < 1e-3f, "the chip answers with id 0x49 and its inverse: started (one reset)");
      CHECK(chip.writes == 74 && chip.bursts == 1 && port.selects == port.releases && !port.nested, "74 register writes (reset + PixArt's two tables of 59 and 14), one burst to empty the motion registers; chip select always released (%u writes)", chip.writes);
      CHECK(chip.reg[0x7F] == 0x00 && chip.reg[0x61] == 0x78 && chip.reg[0x40] == 0x80 && chip.reg[0x5A] == 0x50 && chip.reg[0x4E] == 0xA8 && chip.reg[0x32] == 0x44 && chip.reg[0x6F] == 0xD5 && chip.reg[0x70] == 0x00, "the last value written to each register is the table's");
      // timing the data sheet asks for: 300 us between address and value of a write and 120 us after it (ArduPilot waits the same), 35 / 200 us for a read, 150 us in a burst
      int w300 = 0, w120 = 0, r35 = 0, r200 = 0, b150 = 0, big = 0; for (uint32_t us : port.delays) { w300 += us == 300; w120 += us == 120; r35 += us == 35; r200 += us == 200; b150 += us == 150; big += us >= 50000; }
      CHECK(w300 == 74 && w120 == 74 && r35 == 75 && r200 == 75 && b150 == 1 && big == 3 && port.waited_us >= 200000 && port.waited_us < 300000, "pauses: %d + %d for writes, %d + %d for reads, %d in the burst, three long ones after the reset and between the tables (%.0f ms in all)", w300, w120, r35, r200, b150, port.waited_us * 1e-3);
      flow::Reading r; uint32_t t = 1000000;
      // polls every 10 ms while the picture turns at fx / fy; returns the number of readings that came out
      auto polls = [&](int n, float fx, float fy, uint32_t step_us = 10000) { int got = 0; for (int i = 0; i < n; i++) { chip.integrate(fx, fy, step_us * 1e-6f); t += step_us; got += d.update(t, r); } return got; };
      CHECK(!d.update(t, r) && chip.bursts == 2 && d.samples() == 0, "the first poll only starts the timing");
      CHECK(!d.update(t + 5000, r) && !d.update(t + 9999, r) && chip.bursts == 2, "the chip is left alone until 10 ms have passed");
      const int four = polls(4, 1.0f, -0.5f); port.delays.clear(); const int fifth = polls(1, 1.0f, -0.5f);
      CHECK(four == 0 && fifth == 1 && chip.bursts == 7 && d.samples() == 5 && fabsf(r.x - 1.0f) < 0.03f && fabsf(r.y + 0.5f) < 0.03f && fabsf(r.dt - 0.050f) < 1e-6f && r.age_s == 0.0f && r.quality == 80,
            "five polls, 10 ms apart, make one reading: 50 ms at 1.0 / -0.5 rad/s reads %.3f / %.3f (1 count = 1.26 mrad; one poll alone: steps of 0.126), quality %u", r.x, r.y, r.quality);
      CHECK(port.delays.size() == 1 && port.delays[0] == 150, "one poll = one burst with a single 150 us pause: the bus is busy for a fraction of a millisecond");
      { double sx = 0, sy = 0; int n = 0; for (int i = 0; i < 200; i++) if (polls(1, 0.333f, 2.0f)) { sx += r.x; sy += r.y; n++; }
        CHECK(n == 40 && fabs(sx / n - 0.333) < 0.002 && fabs(sy / n - 2.0) < 0.002, "2 s at 0.333 / 2.0 rad/s: 40 readings, mean %.4f / %.4f (no count is lost between polls)", sx / n, sy / n); }
      CHECK(polls(5, -30.0f, 30.0f) == 1 && fabsf(r.x + 30.0f) < 0.1f && fabsf(r.y - 30.0f) < 0.1f, "fast and negative: %.2f / %.2f rad/s (16-bit counts, sign kept)", r.x, r.y);
      { const int two = polls(2, 1.0f, 1.0f), late = polls(1, 1.0f, 1.0f, 35000);
        CHECK(two == 0 && late == 1 && fabsf(r.x - 1.0f) < 0.03f && fabsf(r.dt - 0.055f) < 1e-6f, "a poll that came 35 ms after the one before: the counts are divided by the time that really passed (%.3f rad/s over %.0f ms)", r.x, r.dt * 1e3); }
      { polls(2, 5.0f, 5.0f); const int after_pause = polls(1, 5.0f, 5.0f, 300000), next4 = polls(4, 1.0f, 1.0f), next5 = polls(1, 1.0f, 1.0f);
        CHECK(after_pause == 0 && next4 == 0 && next5 == 1 && fabsf(r.x - 1.0f) < 0.03f, "after a pause of 0.3 s the counts and what was summed before are thrown away; the five polls after it are a clean reading (%.3f)", r.x); }
      { chip.squal = 30; polls(1, 0, 0); chip.squal = 80; polls(4, 0, 0); const uint8_t q30 = r.quality; chip.squal = 180; polls(5, 0, 0); const uint8_t q180 = r.quality;
        chip.squal = 80; polls(2, 0, 0); chip.shutter_upper = 0x1F; polls(1, 0, 0); chip.shutter_upper = 0x05; const int got = polls(2, 0, 0);
        CHECK(q30 == 30 && q180 == 100 && got == 1 && r.quality == 0 && d.squal() == 80, "quality = the chip's SQUAL, at most 100, the worst poll of the five; 0 when the shutter was at its longest (too dark to see anything)"); }
      // a data line that hangs
      { polls(2, 3.0f, 0); chip.present = false; const uint32_t s0 = d.samples(); const int got = polls(20, 0, 0);
        CHECK(got == 0 && d.dead_bursts() == 20 && d.samples() == s0, "the chip stops answering (every byte 0xFF): no reading - not 'one count backwards at best quality'");
        chip.present = true; const int four2 = polls(4, 0.5f, 0), fifth2 = polls(1, 0.5f, 0);
        CHECK(four2 == 0 && fifth2 == 1 && fabsf(r.x - 0.5f) < 0.03f, "it answers again: readings again, and nothing from before the silence is in them (%.3f)", r.x); }
      { chip.motion_reg = 0xFF; const int got = polls(5, 0.7f, 0); chip.motion_reg = -1;
        CHECK(got == 1 && fabsf(r.x - 0.7f) < 0.03f, "a motion register that happens to read 0xFF in an otherwise good burst is not a dead line (%.3f)", r.x); } }
    { Pmw3901Sim chip; ChipPort port; port.chip = &chip; flow::Pmw3901 d(port); flow::Reading r; d.init();
      chip.integrate(1.0f, 0, 0.15f); const bool got = d.update(150000, r);
      CHECK(!got && d.samples() == 0, "started early after power-up (first poll at 150 ms): the first poll only starts the timing, its counts are not divided by the time since power-up"); }
    { Pmw3901Sim chip; chip.id = 0x30; chip.power_on(); ChipPort port; port.chip = &chip; flow::Pmw3901 d(port); flow::Reading r;
      CHECK(!d.init() && d.chip_id() == 0x30 && chip.writes == 1 && !d.update(1000000, r) && !d.update(2000000, r) && chip.bursts == 0, "another chip (id 0x30): not started, nothing written to it but the reset, never polled"); }
    { Pmw3901Sim chip; ChipPort port; port.chip = &chip; flow::Pmw3901 d(port);
      struct Wrong : ChipPort { uint8_t transfer(uint8_t v) override { const uint8_t o = chip->xfer(v); return (chip->addr == 0x5F && !chip->wr && chip->idx == 2) ? 0x00 : o; } } wrong; wrong.chip = &chip; flow::Pmw3901 e(wrong);
      CHECK(!e.init(), "the id reads right but its inverse does not (a line stuck low): not started");
      chip.present = false; CHECK(!d.init(), "no chip at all (0xFF): not started"); }
    { // a register that does not read back what was written is written five times, then the driver carries on
      Pmw3901Sim chip; struct Stubborn : ChipPort { int w61 = 0; uint8_t transfer(uint8_t v) override { if (chip->idx == 0 && v == (0x61 | 0x80)) w61++; const uint8_t o = chip->xfer(v); return (chip->addr == 0x61 && !chip->wr && chip->idx == 2) ? 0x00 : o; } } port;
      port.chip = &chip; flow::Pmw3901 d(port);
      CHECK(d.init() && port.w61 == 10, "register 0x61 never reads back: tried five times each time it is in the tables (%d writes), start-up goes on", port.w61); }
}

static void t_front() {
    printf("[flow sensor] the front end: sensor type, pins, health\n");
    { FlowSensor f; CHECK(!f.begin() && strstr(f.failReason(), "no optical flow sensor selected") && !f.available() && strcmp(f.name(), "none") == 0 && !f.healthy(1000) && f.rangeDriver() == nullptr, "type NONE: nothing started");
      f.setType(7); CHECK(f.type() == FlowSensor::NONE, "an unknown type is NONE"); }
    { Pmw3901Sim chip; g_chip = &chip; g_cs_pin = 38; g_stub_spi2 = [](uint8_t v) -> uint8_t { return g_chip->xfer(v); }; g_stub_spi2_begins = 0;
      FlowSensor f; f.setType(FlowSensor::PMW3901);
      CHECK(!f.begin() && strstr(f.failReason(), "pins not set") && g_stub_spi2_begins == 0, "PMW3901 without pins: refused before the bus is touched");
      f.setSpiPins(38, 39, 40, 255); CHECK(!f.begin() && strstr(f.failReason(), "pins not set"), "one pin missing: refused");
      f.setSpiPins(38, 39, 40, 41); f.setSpiBusTaken(true); CHECK(!f.begin() && strstr(f.failReason(), "SD card in SPI mode") && g_stub_spi2_begins == 0 && chip.writes == 0, "the bus belongs to the SD card: refused, the bus is not touched");
      f.setSpiBusTaken(false); g_stub_gpio[38] = 0;
      CHECK(f.begin() && f.available() && strcmp(f.name(), "PMW3901") == 0 && g_stub_spi2_begins == 1 && g_stub_spi2_pins[0] == 39 && g_stub_spi2_pins[1] == 40 && g_stub_spi2_pins[2] == 41 && g_stub_gpio[38] == HIGH && f.rangeDriver() == nullptr,
            "started on the second SPI bus (sck 39, miso 40, mosi 41), chip select idle high; this sensor has no lidar");
      CHECK(!f.healthy(0) && f.rateHz() == 0, "not healthy before the first reading");
      uint32_t ms = 0; int got = 0; for (; ms < 1500; ms++) { chip.integrate(0.4f, 0.1f, 0.001f); got += f.update(ms * 1000u, ms); }
      CHECK(got == 29 && f.healthy(ms) && f.rateHz() >= 99 && f.rateHz() <= 100 && fabsf(f.reading().x - 0.4f) < 0.03f, "1.5 s: %d readings from %u polls a second, healthy", got, f.rateHz());
      chip.present = false; for (; ms < 1500 + 400; ms++) f.update(ms * 1000u, ms); const bool at400 = f.healthy(ms); for (; ms < 1500 + 520; ms++) f.update(ms * 1000u, ms);
      CHECK(at400 && !f.healthy(ms), "the chip goes silent: still healthy after 0.4 s, not after 0.52 s");
      for (; ms < 4000; ms++) { f.update(ms * 1000u, ms); }
      CHECK(f.rateHz() == 0, "0 readings per second");
      chip.present = true; chip.id = 0x12; chip.power_on(); CHECK(!f.begin() && strstr(f.failReason(), "no PMW3901 answered") && !f.available(), "begin() again with another chip on the pins: fails, and the old driver is not left in place");
      g_chip = nullptr; g_stub_spi2 = nullptr; }
    { struct Dev : StubUartDevice { std::deque<uint8_t> q; uint32_t baud = 0; int dev_read() override { if (q.empty()) return -1; const uint8_t c = q.front(); q.pop_front(); return c; } void dev_write(const uint8_t*, size_t) override {} void dev_config(uint32_t b, uint32_t, bool) override { baud = b; } } dev;
      HardwareSerial uart; uart.dev = &dev; FlowSensor f; f.setType(FlowSensor::MSP);
      f.setUart(&uart, "UART0", 255, 255, 115200); CHECK(!f.begin() && strstr(f.failReason(), "pin flow_rx not set") && !uart.started, "MSP sensor without its RX pin: refused, the UART is not opened");
      f.setUart(nullptr, "", 42, 255, 115200); CHECK(!f.begin() && strstr(f.failReason(), "no UART left"), "no UART to give: said so");
      f.setUart(&uart, "UART0", 42, 255, 230400);
      CHECK(f.begin() && uart.started && uart.rx_pin == 42 && uart.tx_pin == -1 && dev.baud == 230400 && uart.format == SERIAL_8N1 && strcmp(f.uartName(), "UART0") == 0 && f.rangeDriver() != nullptr && strcmp(f.name(), "MSP flow") == 0,
            "started: 8N1 at the configured baud rate, RX pin 42, no TX pin; it brings a lidar");
      RangeDriver* lidar = f.rangeDriver(); RangeReading rr;
      CHECK(strcmp(lidar->name(), "MSP lidar (flow sensor)") == 0 && lidar->init() && !lidar->healthy() && !lidar->update(0, rr) && fabsf(lidar->max_range_m() - 8.0f) < 1e-6f && fabsf(lidar->min_range_m() - 0.02f) < 1e-6f, "the lidar: not healthy before its first distance");
      auto put = [&](const std::vector<uint8_t>& v) { for (uint8_t c : v) dev.q.push_back(c); };
      put(flow_frame(255, 0, 0)); put(range_frame(255, 640)); const bool f1 = f.update(10000, 10);
      int got = 0; for (uint32_t k = 2; k <= 6; k++) { put(flow_frame(255, 10, 0)); got += f.update(k * 10000, k * 10); }
      CHECK(!f1 && got == 1 && fabsf(f.reading().x - 5.0f) < 1e-3f && f.healthy(60) && lidar->update(60000, rr) && fabsf(rr.distance_m - 0.640f) < 1e-6f && rr.quality_pct == 100 && lidar->healthy() && !lidar->update(61000, rr),
            "a flow reading and a distance come through; the distance is handed to the range finder once");
      for (uint32_t k = 7; k <= 201; k++) { put(flow_frame(255, 10, 0)); f.update(k * 10000, k * 10); }
      CHECK(f.rateHz() == 100, "frames a second, as the page shows them: %u (five of them make a reading)", f.rateHz());
      put(range_frame(0, 700)); f.update(2020000, 2020); CHECK(lidar->update(2020000, rr) && rr.quality_pct == 50, "a sensor that sends no quality for its range (0): taken as 50 %%, not as garbage");
      CHECK(!lidar->update(2020000 + 499000, rr) && lidar->healthy(), "0.499 s without a distance: still healthy"); lidar->update(2020000 + 500000, rr);
      CHECK(!lidar->healthy(), "0.5 s: not healthy (ArduPilot's timeout for MSP range finders)");
      put(range_frame(255, -1)); f.update(2600000, 2600); lidar->update(2600000, rr); CHECK(!lidar->healthy(), "'out of range' frames do not make it healthy"); }
    { // any driver through the test hook
      struct Fake : flow::Driver { bool ok = true; int inits = 0; const char* name() const override { return "fake"; } bool init() override { inits++; return ok; } bool update(uint32_t, flow::Reading& o) override { o.x = 1; return true; } float nominal_rate_hz() const override { return 50; } uint32_t samples() const override { return 0; } } fake;
      FlowSensor f; CHECK(f.useDriver(&fake) && fake.inits == 1 && f.available() && f.update(0, 0) && f.reading().x == 1.0f, "useDriver(): init() is called, readings come through");
      fake.ok = false; CHECK(!f.useDriver(&fake) && !f.available() && !f.useDriver(nullptr), "a driver that does not start is not used"); }
}

static void t_uart_choice() {
    printf("[uarts] which UART is left for an MSP flow sensor\n");
    CHECK(app::flow_uart(false, true, true) == 1 && app::flow_uart(false, false, false) == 1, "no GPS: UART1");
    CHECK(app::flow_uart(true, false, true) == 2 && app::flow_uart(true, false, false) == 2, "GPS on UART1, no serial receiver: UART2");
    CHECK(app::flow_uart(true, true, true) == 0, "both in use, the app on USB: UART0");
    CHECK(app::flow_uart(true, true, false) == -1, "both in use and the app on UART0: none");
    printf("[lidar] when the range sensor is the one inside the flow sensor\n");
    CHECK(app::lidar_of_flow(true, true, true, false, true) && app::lidar_of_flow(true, true, true, false, false), "range model MSP: always the flow sensor's lidar (even if a VL53 answered)");
    CHECK(app::lidar_of_flow(true, true, false, true, false) && !app::lidar_of_flow(true, true, false, true, true), "range model AUTO: the flow sensor's lidar only when no other range sensor answered");
    CHECK(!app::lidar_of_flow(true, true, false, false, false), "range model VL53L1X / VL53L0X: never");
    CHECK(!app::lidar_of_flow(false, true, true, false, false) && !app::lidar_of_flow(true, false, true, false, false), "range sensing switched off, or a flow sensor without a lidar (PMW3901): never");
}

int main() {
    t_parser();
    t_accumulator();
    t_msp_flow();
    t_pmw();
    t_front();
    t_uart_choice();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
