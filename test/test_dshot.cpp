// DShot frame sequencing (src/hal/motor/dshot_direct.h) against a model of the wire.
//
// The model has, per motor: the flight controller's transmitter and pin driver, a pull-up, an ESC that decodes what is really on
// the wire and answers 30 us later, and a receiver that records edges the way an RMT receive channel does. Time is in nanoseconds.
// Every port call costs time and the timer interrupt may arrive late or at a wrong moment - exactly the things that cannot be
// seen by reading the code.
#include <stdio.h>
#include <math.h>
#include <vector>
#include <algorithm>
#include "../src/hal/motor/dshot_direct.h"

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, ...) do { if (cond) g_pass++; else { g_fail++; printf("  FAIL line %d: %s   ", __LINE__, #cond); printf(__VA_ARGS__); printf("\n"); } } while (0)

using namespace motor;
typedef long long ns_t;

struct Wire;
struct SimPort {
    Wire* w = nullptr;
    uint32_t micros();
    uint32_t tick_hz();
    void alarm_in(uint32_t us);
    void drive(int c); void release(int c);
    void tx_load(int c, const uint32_t* words, int n); void tx_start(int c);
    void rx_start(int c); int rx_stop(int c, uint32_t* words, int max);
    void delay_us(uint32_t us);
    void listen(int) {}                           // (the model's receiver always hears the wire)
};

struct Wire {
    // ---- configuration of the experiment
    Protocol proto = PROTO_DSHOT300; bool bidir = true; int count = 4;
    uint32_t tick = 80000000;
    bool esc_answers[4] = {true, true, true, true};
    ns_t esc_delay_ns = 30000;                    // frame end -> first reply edge
    uint16_t esc_value[4] = {0x123, 0x456, 0x789, 0xABC};     // 12-bit value each ESC answers with
    bool esc_noise[4] = {false, false, false, false};         // the reply carries one extra glitch-free but wrong edge
    bool esc_short[4] = {false, false, false, false};         // the ESC stops after 9 bits
    ns_t isr_latency_ns = 4000;                   // timer alarm -> interrupt handler runs
    ns_t cost_load = 1500, cost_start = 400, cost_pin = 100, cost_rx_start = 300, cost_rx_stop = 800, cost_alarm = 2000;
    ns_t filter_ns = 150;                         // receiver glitch filter
    ns_t idle_ns() const { return (ns_t)reply_bit_ns(proto) * 8; }

    // ---- state
    static constexpr int kMem = 48;
    ns_t now = 0;
    DshotDirect<SimPort>* d = nullptr;
    bool alarm_set = false, in_isr = false; ns_t alarm_t = 0;
    struct TxFrame { ns_t t0; uint32_t w[kMem]; bool settled = false, valid = false, replied = false, audited = false; uint16_t packet = 0; ns_t reply_t0 = 0; uint8_t reply_lv[22]; int reply_n = 0; };
    struct Chan {
        uint32_t mem[kMem];                       // the transmitter runs through its memory until it meets an end mark
        Chan() { for (int i = 0; i < kMem; i++) mem[i] = rmt_word(200, 1, 200, 0); }
        std::vector<TxFrame> frames;
        std::vector<std::pair<ns_t, bool>> drv;   // (time, driving)
        bool rx_on = false; ns_t rx_t0 = 0;
        // what the ESC saw
        int good = 0, bad = 0; uint16_t last_value = 0; bool last_telem = false;
        ns_t contention_ns = 0; ns_t worst_release_after_ns = -1;
        int starts_while_released = 0, starts_while_busy = 0;
        bool tx_cut = false, rx_deaf = false;      // board faults: the transmitter does not reach the pin / the receiver does not hear it
    } ch[4];
    int isr_runs = 0;

    // ---- time
    void tick_cost(ns_t cost) {
        now += cost;
        if (alarm_set && !in_isr && alarm_t <= now) fire();
    }
    void fire() { alarm_set = false; in_isr = true; isr_runs++; d->on_alarm(); in_isr = false; }
    void run_to(ns_t t) {
        if (alarm_set && alarm_t <= t) { if (alarm_t > now) now = alarm_t; fire(); }
        if (t > now) now = t;
    }
    void spurious_isr() { in_isr = true; d->on_alarm(); in_isr = false; }     // an old interrupt that was held back

    // ---- waveform helpers
    ns_t tk(uint32_t ticks) const { return (ns_t)ticks * 1000000000ll / tick; }
    ns_t bit_ns() const { return dshot_bit_ns(proto); }
    static void word(uint32_t w, uint32_t& d0, int& l0, uint32_t& d1, int& l1) { d0 = w & 0x7FFF; l0 = (w >> 15) & 1; d1 = (w >> 16) & 0x7FFF; l1 = (int)(w >> 31); }
    ns_t frame_len(const TxFrame& f) const {
        ns_t t = 0;
        for (int i = 0; i < kMem; i++) { uint32_t d0, d1; int l0, l1; word(f.w[i], d0, l0, d1, l1); if (!d0) break; t += tk(d0); if (!d1) break; t += tk(d1); }
        return t;
    }
    int idle_level() const { return bidir ? 1 : 0; }
    int tx_level(const Chan& c, ns_t t) const {                        // what the transmitter puts out
        if (c.tx_cut) return idle_level();
        for (int k = (int)c.frames.size() - 1; k >= 0; k--) {
            const TxFrame& f = c.frames[k];
            if (t < f.t0) continue;
            ns_t x = f.t0;
            for (int i = 0; i < kMem; i++) {
                uint32_t d0, d1; int l0, l1; word(f.w[i], d0, l0, d1, l1);
                if (!d0) return idle_level();
                if (t < x + tk(d0)) return l0;
                x += tk(d0);
                if (!d1) return idle_level();
                if (t < x + tk(d1)) return l1;
                x += tk(d1);
            }
            return idle_level();
        }
        return idle_level();
    }
    bool driving(const Chan& c, ns_t t) const {
        bool on = true;                                                // a transmit channel starts as an output
        for (auto& e : c.drv) { if (e.first > t) break; on = e.second; }
        return on;
    }
    bool esc_level(const Chan& c, ns_t t, int& lv) const {
        for (int k = (int)c.frames.size() - 1; k >= 0; k--) {
            const TxFrame& f = c.frames[k];
            if (!f.replied || t < f.reply_t0) continue;
            const ns_t rb = reply_bit_ns(proto);
            const ns_t i = (t - f.reply_t0) / rb;
            if (i >= f.reply_n) return false;
            lv = f.reply_lv[i]; return true;
        }
        return false;
    }
    int wire(const Chan& c, ns_t t) const {
        if (driving(c, t)) return tx_level(c, t);
        int lv; if (esc_level(c, t, lv)) return lv;
        return 1;                                                      // pull-up
    }

    // The ESC looks at the finished frame as it really was on the wire.
    void settle(int ci) {
        Chan& c = ch[ci];
        audit(ci);
        for (auto& f : c.frames) {
            if (f.settled || now < f.t0 + frame_len(f) + 2 * bit_ns()) continue;
            f.settled = true;
            uint16_t p = 0; const ns_t T = bit_ns();
            for (int b = 0; b < 16; b++) {
                const ns_t ts = f.t0 + b * T + T * 55 / 100;           // between the short (37.5 %) and the long (75 %) pulse
                const int lv = driving(c, ts) ? tx_level(c, ts) : 1;
                const int active = bidir ? !lv : lv;
                p = (uint16_t)((p << 1) | (active ? 1 : 0));
            }
            // the line must also really start with an active edge and return to idle at the end
            const int first = driving(c, f.t0 + T / 10) ? tx_level(c, f.t0 + T / 10) : 1;
            f.packet = p;
            bool rests = true;                                             // an ESC finds the end of a frame by the pause after it
            for (int k = 1; k <= 6; k++) { const ns_t ts = f.t0 + 16 * T + T * k / 4; if ((driving(c, ts) ? tx_level(c, ts) : 1) != idle_level()) rests = false; }
            f.valid = dshot_packet_valid(p, bidir) && first == !idle_level() && rests && frame_len(f) < 17 * T;
            if (!f.valid) { c.bad++; continue; }
            c.good++; c.last_value = p >> 5; c.last_telem = (p >> 4) & 1;
            if (bidir && esc_answers[ci]) {
                f.replied = true; f.reply_t0 = f.t0 + frame_len(f) + esc_delay_ns;
                uint8_t lv[21]; f.reply_n = gcr_to_levels(gcr_encode(esc_value[ci]), lv);
                for (int i = 0; i < 21; i++) f.reply_lv[i] = lv[i];
                if (esc_noise[ci]) f.reply_lv[7] ^= 1;
                if (esc_short[ci]) f.reply_n = 9;
            }
        }
    }
    // Once a reply is over: was the pin free in time, did both ends ever drive different levels?
    void audit(int ci) {
        Chan& c = ch[ci]; const ns_t rb = reply_bit_ns(proto);
        for (auto& f : c.frames) {
            if (!f.replied || f.audited || now < f.reply_t0 + f.reply_n * rb) continue;
            f.audited = true;
            ns_t rel = -1;
            for (auto& e : c.drv) if (e.first >= f.t0 && !e.second) { rel = e.first; break; }
            const ns_t after = rel < 0 ? (ns_t)1e12 : rel - (f.t0 + frame_len(f));
            if (after > c.worst_release_after_ns) c.worst_release_after_ns = after;
            for (ns_t t = f.reply_t0; t < f.reply_t0 + f.reply_n * rb; t += 50)
                if (driving(c, t) && tx_level(c, t) != f.reply_lv[(t - f.reply_t0) / rb]) c.contention_ns += 50;
        }
    }
    void settle_all() { for (int i = 0; i < count; i++) settle(i); }

    // What an RMT receiver records between rx_start and rx_stop.
    int capture(int ci, uint32_t* out, int max) {
        Chan& c = ch[ci];
        if (c.rx_deaf) return 0;
        const ns_t t0 = c.rx_t0, t1 = now;
        std::vector<ns_t> ev;
        for (auto& e : c.drv) if (e.first > t0 && e.first < t1) ev.push_back(e.first);
        for (auto& f : c.frames) {
            ns_t x = f.t0;
            for (int i = 0; i < kMem; i++) { uint32_t d0, d1; int l0, l1; word(f.w[i], d0, l0, d1, l1); if (x > t0 && x < t1) ev.push_back(x); if (!d0) break; x += tk(d0); if (x > t0 && x < t1) ev.push_back(x); if (!d1) break; x += tk(d1); }
            if (x > t0 && x < t1) ev.push_back(x);
            if (f.replied) { const ns_t rb = reply_bit_ns(proto); for (int i = 0; i <= f.reply_n; i++) { const ns_t t = f.reply_t0 + i * rb; if (t > t0 && t < t1) ev.push_back(t); } }
        }
        std::sort(ev.begin(), ev.end());
        // level changes
        std::vector<std::pair<ns_t, int>> edges; int lv = wire(c, t0);
        for (ns_t t : ev) { const int n = wire(c, t); if (n != lv) { edges.push_back({t, n}); lv = n; } }
        // glitch filter: a level that lasts less than the filter time is not seen
        for (size_t i = 0; i + 1 < edges.size();) { if (edges[i + 1].first - edges[i].first < filter_ns) edges.erase(edges.begin() + i, edges.begin() + i + 2); else i++; }
        // runs from the first edge; a run longer than the idle threshold ends the capture
        std::vector<std::pair<int, ns_t>> runs; bool ended = false;
        for (size_t i = 0; i < edges.size() && !ended; i++) {
            const ns_t end = i + 1 < edges.size() ? edges[i + 1].first : t1;
            const ns_t len = end - edges[i].first;
            if (len >= idle_ns()) { runs.push_back({edges[i].second, 0}); ended = true; }
            else if (i + 1 < edges.size()) runs.push_back({edges[i].second, len});
        }
        int n = 0;
        for (size_t i = 0; i < runs.size() && n < max; i += 2, n++) {
            const uint32_t d0 = (uint32_t)(runs[i].second * tick / 1000000000ll);
            const bool second = i + 1 < runs.size();
            const uint32_t d1 = second ? (uint32_t)(runs[i + 1].second * tick / 1000000000ll) : 0;
            out[n] = rmt_word((uint16_t)d0, (uint8_t)runs[i].first, (uint16_t)d1, second ? (uint8_t)runs[i + 1].first : 0);
        }
        return n;
    }
};

uint32_t SimPort::micros() { return (uint32_t)(w->now / 1000); }
uint32_t SimPort::tick_hz() { return w->tick; }
void SimPort::alarm_in(uint32_t us) { w->tick_cost(w->cost_alarm); w->alarm_set = true; w->alarm_t = w->now + (ns_t)us * 1000 + w->isr_latency_ns; }
void SimPort::drive(int c) { w->settle(c); w->ch[c].drv.push_back({w->now, true}); w->tick_cost(w->cost_pin); }
void SimPort::release(int c) { w->settle(c); w->ch[c].drv.push_back({w->now, false}); w->tick_cost(w->cost_pin); }
void SimPort::tx_load(int c, const uint32_t* words, int n) { for (int i = 0; i < n && i < Wire::kMem; i++) w->ch[c].mem[i] = words[i]; w->tick_cost(w->cost_load); }
void SimPort::tx_start(int c) {
    Wire::Chan& ch = w->ch[c];
    w->settle(c);
    if (!w->driving(ch, w->now)) ch.starts_while_released++;
    if (!ch.frames.empty()) {
        const Wire::TxFrame& p = ch.frames.back();
        ns_t busy_until = p.t0 + w->frame_len(p);
        if (p.replied) busy_until = p.reply_t0 + p.reply_n * (ns_t)reply_bit_ns(w->proto);
        if (w->now < busy_until) ch.starts_while_busy++;
    }
    Wire::TxFrame f; f.t0 = w->now; for (int i = 0; i < Wire::kMem; i++) f.w[i] = ch.mem[i];
    ch.frames.push_back(f);
    if (ch.frames.size() > 8) ch.frames.erase(ch.frames.begin());
    if (ch.drv.size() > 64) ch.drv.erase(ch.drv.begin(), ch.drv.begin() + 32);
    w->tick_cost(w->cost_start);
}
void SimPort::delay_us(uint32_t us) { w->run_to(w->now + (ns_t)us * 1000); }
void SimPort::rx_start(int c) { w->settle(c); w->ch[c].rx_on = true; w->ch[c].rx_t0 = w->now; w->tick_cost(w->cost_rx_start); }
int SimPort::rx_stop(int c, uint32_t* words, int max) {
    w->settle(c);
    int n = 0;
    if (w->ch[c].rx_on) n = w->capture(c, words, max);
    w->ch[c].rx_on = false;
    w->tick_cost(w->cost_rx_stop);
    return n;
}

struct Rig {
    Wire w; SimPort port; DshotDirect<SimPort> d;
    uint16_t value[4] = {48, 500, 1047, 2047}; bool telem[4] = {false, false, false, false};
    int got[4] = {}, bad[4] = {}, wrong[4] = {};
    bool begin(Protocol p, bool bidir, int count = 4) {
        w.proto = p; w.bidir = bidir; w.count = count; w.d = &d; port.w = &w;
        return d.begin(port, count, p, bidir);
    }
    // n updates, period_us apart (jitter_us = +- uniform). Values change every cycle.
    void run(int n, double period_us, double jitter_us = 0, bool vary = true) {
        static uint32_t seed = 12345;
        for (int k = 0; k < n; k++) {
            seed = seed * 1664525u + 1013904223u;
            const double j = jitter_us * (((seed >> 8) & 0xFFFF) / 32768.0 - 1.0);
            w.run_to(w.now + (ns_t)((period_us + j) * 1000));
            if (vary) for (int c = 0; c < 4; c++) value[c] = (uint16_t)(48 + (k * 37 + c * 500) % 2000);
            d.send(value, telem);
            for (int c = 0; c < w.count; c++) {
                uint32_t g;
                if (d.take_reply(c, g)) {
                    if (g == kGcrInvalid || gcr_decode(g) == kGcrInvalid) bad[c]++;
                    else if (gcr_decode(g) != w.esc_value[c]) wrong[c]++;
                    else got[c]++;
                }
            }
        }
        w.run_to(w.now + 2000000); w.settle_all();
    }
    int good(int c) { return w.ch[c].good; }
};

static void t_words() {
    printf("[symbol words]\n");
    const uint32_t x = rmt_word(100, 1, 33, 0);
    CHECK((x & 0x7FFF) == 100 && ((x >> 15) & 1) == 1 && ((x >> 16) & 0x7FFF) == 33 && (x >> 31) == 0, "word layout %08x", (unsigned)x);
    CHECK(rmt_word(0x7FFF, 0, 0x7FFF, 1) == 0xFFFF7FFFu, "full durations");
    CHECK(dshot_frame_us(PROTO_DSHOT300) == 54 && dshot_frame_us(PROTO_DSHOT600) == 27, "frame length %u %u", (unsigned)dshot_frame_us(PROTO_DSHOT300), (unsigned)dshot_frame_us(PROTO_DSHOT600));
    // frame + 30 us + 21 reply bits + idle gap + slack
    CHECK(dshot_cycle_us(PROTO_DSHOT300, true) == 54 + 30 + 56 + 22 + 20, "bidirectional cycle DShot300 = %u us", (unsigned)dshot_cycle_us(PROTO_DSHOT300, true));
    CHECK(dshot_cycle_us(PROTO_DSHOT600, true) == 27 + 30 + 28 + 11 + 20, "bidirectional cycle DShot600 = %u us", (unsigned)dshot_cycle_us(PROTO_DSHOT600, true));
    CHECK(dshot_cycle_us(PROTO_DSHOT600, true) < 125, "DShot600 with replies fits an 8 kHz loop");
    CHECK(dshot_cycle_us(PROTO_DSHOT300, true) < 250, "DShot300 with replies fits a 4 kHz loop");
    CHECK(dshot_cycle_us(PROTO_DSHOT300, false) == 54 + 30 && dshot_cycle_us(PROTO_DSHOT150, false) == 107 + 30, "one-way: frame + pause");
    Rig r;
    CHECK(!r.begin(PROTO_ONESHOT125, false) && !r.begin(PROTO_DSHOT300, false, 0) && !r.begin(PROTO_DSHOT300, false, 5), "only DShot, 1..4 motors");
    uint16_t v[4] = {}; CHECK(!r.d.send(v, nullptr) && r.w.ch[0].frames.empty(), "nothing is sent after a failed begin");
}

static void t_oneway() {
    printf("[one-way DShot: what the ESC decodes]\n");
    for (Protocol p : {PROTO_DSHOT150, PROTO_DSHOT300, PROTO_DSHOT600, PROTO_DSHOT1200}) {
        Rig r; CHECK(r.begin(p, false), "begin");
        r.run(500, 500);
        bool ok = true; for (int c = 0; c < 4; c++) ok = ok && r.good(c) == 500 && r.w.ch[c].bad == 0 && r.w.ch[c].last_value == r.value[c];
        CHECK(ok, "DShot%u: 500 frames on 4 motors, all valid, last values right (%d good, %d bad)", (unsigned)dshot_kbps(p), r.good(0), r.w.ch[0].bad);
        CHECK(r.w.isr_runs == 0 && r.w.ch[0].drv.size() == 1, "no timer, the pin is only switched on once");
        CHECK(r.d.stats().frames == 500 && r.d.stats().skipped == 0, "counters");
    }
    { Rig r; r.begin(PROTO_DSHOT300, false); r.telem[2] = true; r.run(3, 500, 0, false);
      CHECK(r.w.ch[2].last_telem && !r.w.ch[1].last_telem, "telemetry-request bit per motor");
      CHECK(r.w.tx_level(r.w.ch[0], r.w.now) == 0 && r.w.wire(r.w.ch[0], r.w.now) == 0, "line idles low"); }
    { Rig r; r.begin(PROTO_DSHOT300, false); r.run(4000, 50);          // 20 kHz updates: faster than a frame
      bool ok = true; for (int c = 0; c < 4; c++) ok = ok && r.w.ch[c].bad == 0 && r.w.ch[c].starts_while_busy == 0;
      CHECK(ok && r.d.stats().skipped > 1000 && r.d.stats().frames + r.d.stats().skipped == 4000, "updates faster than the protocol: frames never overlap (%u sent, %u skipped)", (unsigned)r.d.stats().frames, (unsigned)r.d.stats().skipped);
      CHECK(r.good(0) == (int)r.d.stats().frames, "every frame that went out was valid"); }
    { Rig r; r.begin(PROTO_DSHOT600, false, 2); r.run(100, 250);
      CHECK(r.good(0) == 100 && r.good(1) == 100 && r.w.ch[2].frames.empty(), "2 motors: the other channels stay untouched"); }
    { // all motors start within a couple of microseconds
      Rig r; r.begin(PROTO_DSHOT600, false); r.run(1, 250, 0, false);
      CHECK(r.w.ch[3].frames.back().t0 - r.w.ch[0].frames.back().t0 <= 2000, "4 transmitters start back to back (%lld ns apart)", r.w.ch[3].frames.back().t0 - r.w.ch[0].frames.back().t0); }
}

static void t_bidir() {
    printf("[bidirectional DShot: frame out, pin released, reply in]\n");
    struct Case { Protocol p; double period; } cases[] = {{PROTO_DSHOT150, 500}, {PROTO_DSHOT300, 250}, {PROTO_DSHOT600, 125}, {PROTO_DSHOT1200, 125}};
    for (auto& cs : cases) {
        Rig r; CHECK(r.begin(cs.p, true), "begin");
        r.run(400, cs.period);
        bool ok = true; ns_t cont = 0, worst = 0;
        for (int c = 0; c < 4; c++) {
            ok = ok && r.good(c) == 400 && r.w.ch[c].bad == 0 && r.got[c] >= 399 && r.bad[c] == 0 && r.wrong[c] == 0 && r.w.ch[c].starts_while_released == 0 && r.w.ch[c].starts_while_busy == 0;
            cont += r.w.ch[c].contention_ns; worst = std::max(worst, r.w.ch[c].worst_release_after_ns);
        }
        CHECK(ok, "DShot%u at %.0f us: 400 frames valid at the ESC, every reply decoded (%d good frames, %d replies, %d garbled)", (unsigned)dshot_kbps(cs.p), cs.period, r.good(0), r.got[0], r.bad[0]);
        CHECK(cont == 0, "the two ends never drive against each other (%lld ns)", cont);
        CHECK(worst >= 0 && worst < 25000, "pin released %lld ns after the frame (the ESC starts at 30000)", worst);
        CHECK(r.d.stats().late_switch == 0 && r.d.stats().skipped == 0 && r.d.stats().garbled == 0 && r.d.stats().replies >= 4 * 399, "counters clean");
        CHECK(r.d.responsive(0) && r.d.responsive(3) && !r.d.responsive(4) && !r.d.responsive(-1), "all four answer");
    }
    { Rig r; r.begin(PROTO_DSHOT300, true); r.run(3, 250, 0, false);
      CHECK(r.w.wire(r.w.ch[0], r.w.now) == 1, "line idles high");
      CHECK(r.w.ch[1].last_value == r.value[1] && r.w.ch[3].last_value == 2047, "inverted frames carry the values"); }
    printf("[every 12-bit reply value survives the wire]\n");
    { Rig r; r.begin(PROTO_DSHOT600, true, 1); int okv = 0;
      for (int v = 0; v < 4096; v++) {
          r.w.run_to(r.w.now + 125000); r.w.settle_all(); r.w.esc_value[0] = (uint16_t)v;
          r.d.send(r.value, r.telem); r.w.run_to(r.w.now + 125000); r.d.send(r.value, r.telem);
          uint32_t g; if (r.d.take_reply(0, g) && gcr_decode(g) == (uint32_t)v) okv++; }
      CHECK(okv == 4096, "%d of 4096 values", okv); }
    printf("[ESC that does not answer]\n");
    { Rig r; r.begin(PROTO_DSHOT300, true); for (int c = 0; c < 4; c++) r.w.esc_answers[c] = false;
      r.run(15, 250);
      CHECK(r.d.responsive(0), "not judged on the first few silent frames");
      r.run(10, 250);
      bool ok = true; for (int c = 0; c < 4; c++) ok = ok && !r.d.responsive(c) && r.got[c] == 0 && r.bad[c] == 0 && r.w.ch[c].bad == 0;
      CHECK(ok && r.good(0) == 25, "silent ESCs: reported as not answering after 20 frames, motors still get every frame");
      CHECK(r.d.stats().replies == 0, "nothing captured"); }
    { Rig r; r.begin(PROTO_DSHOT300, true); r.w.esc_answers[1] = false; r.w.esc_answers[2] = false; r.run(100, 250);
      CHECK(r.d.responsive(0) && !r.d.responsive(1) && !r.d.responsive(2) && r.d.responsive(3), "per motor: 1 and 4 answer, 2 and 3 do not");
      CHECK(r.got[0] >= 99 && r.got[3] >= 99 && r.got[1] == 0, "replies only from the ones that answer");
      r.w.esc_answers[1] = true; r.run(5, 250);
      CHECK(r.d.responsive(1) && r.got[1] >= 4, "an ESC that starts to answer is back at once"); }
    printf("[ESC timing spread and a garbled reply]\n");
    for (ns_t delay : {20000, 25000, 30000, 38000, 45000}) {
        Rig r; r.begin(PROTO_DSHOT300, true); r.w.esc_delay_ns = delay; r.run(200, 250);
        CHECK(r.got[0] >= 199 && r.got[3] >= 199 && r.bad[0] == 0 && r.w.ch[0].contention_ns == 0, "ESC answers after %lld us: all decoded (%d)", delay / 1000, r.got[0]);
    }
    { Rig r; r.begin(PROTO_DSHOT300, true); r.w.esc_noise[2] = true; r.run(100, 250);
      CHECK(r.bad[2] >= 99 && r.got[2] == 0 && r.wrong[2] == 0 && r.got[1] >= 99, "a damaged reply is reported as invalid, never as a wrong value (%d invalid)", r.bad[2]);
      CHECK(r.d.responsive(2), "a garbled answer is still an answer");
      CHECK(r.d.stats().garbled == 0, "(21 clean bits with a wrong one: found by the checksum, not by the receiver)"); }
    { Rig r; r.begin(PROTO_DSHOT300, true); r.w.esc_short[1] = true; r.run(100, 250);
      CHECK(r.bad[1] >= 99 && r.got[1] == 0 && r.wrong[1] == 0 && r.got[0] >= 99, "a reply that stops half way is invalid (%d)", r.bad[1]);
      CHECK(r.d.stats().garbled >= 99 && r.d.stats().replies >= 4 * 99, "and counted as garbled (%u)", (unsigned)r.d.stats().garbled); }
    { Rig r; r.w.tick = 40000000; r.begin(PROTO_DSHOT300, true); r.run(100, 250);        // (the clock of the timer is fixed before the driver starts)
      CHECK(r.got[0] >= 99 && r.good(0) == 100, "40 MHz ticks work the same"); }
}

static void t_timing() {
    printf("[loop timing]\n");
    { Rig r; r.begin(PROTO_DSHOT300, true); r.run(2000, 250, 40);
      bool ok = true; for (int c = 0; c < 4; c++) ok = ok && r.w.ch[c].bad == 0 && r.bad[c] == 0 && r.wrong[c] == 0 && r.w.ch[c].contention_ns == 0 && r.w.ch[c].starts_while_busy == 0;
      CHECK(ok && r.got[0] >= 1990, "4 kHz with +-40 us jitter: nothing lost (%d replies, %u skipped)", r.got[0], (unsigned)r.d.stats().skipped); }
    { Rig r; r.begin(PROTO_DSHOT300, true); r.run(3000, 125);          // 8 kHz updates, DShot300 replies need 182 us
      bool ok = true; for (int c = 0; c < 4; c++) ok = ok && r.w.ch[c].bad == 0 && r.bad[c] == 0 && r.w.ch[c].contention_ns == 0 && r.w.ch[c].starts_while_busy == 0;
      CHECK(ok, "updates faster than frame + reply: no frame is started into a reply");
      CHECK(r.d.stats().skipped >= 1400 && r.d.stats().frames >= 1400 && r.got[0] >= (int)r.d.stats().frames - 1, "every second update is skipped, every sent frame gets its reply (%u sent, %u skipped, %d replies)", (unsigned)r.d.stats().frames, (unsigned)r.d.stats().skipped, r.got[0]); }
    printf("[late and misplaced timer interrupts]\n");
    { Rig r; r.begin(PROTO_DSHOT300, true); r.w.isr_latency_ns = 20000; r.run(200, 250);
      CHECK(r.got[0] >= 199 && r.w.ch[0].contention_ns == 0 && r.d.stats().late_switch == 0, "interrupt 20 us late: still before the ESC starts"); }
    { Rig r; r.begin(PROTO_DSHOT300, true); r.w.isr_latency_ns = 50000; r.run(200, 250);
      bool ok = true; for (int c = 0; c < 4; c++) ok = ok && r.good(c) == 200 && r.wrong[c] == 0;
      CHECK(ok, "interrupt 50 us late: the motors still get every frame and no wrong value is ever reported");
      CHECK(r.w.ch[0].contention_ns > 0 && r.got[0] == 0, "(that reply start is lost: %lld ns of both ends driving)", r.w.ch[0].contention_ns); }
    { Rig r; r.begin(PROTO_DSHOT300, true); r.run(50, 250);
      r.w.isr_latency_ns = 400000;                                      // interrupts held off for longer than a whole cycle
      const int before = r.got[0]; r.run(50, 250);
      bool ok = true; for (int c = 0; c < 4; c++) ok = ok && r.good(c) == 100 && r.w.ch[c].bad == 0 && r.wrong[c] == 0 && r.w.ch[c].starts_while_released == 0;
      CHECK(ok, "interrupt later than the next frame: every frame still valid, never started on a released pin");
      CHECK(r.d.stats().late_switch >= 48, "counted as late switches (%u)", (unsigned)r.d.stats().late_switch);
      CHECK(r.got[0] <= before + 1, "no reply while it lasts (%d new; one was still on its way)", r.got[0] - before);
      r.w.isr_latency_ns = 4000; r.run(50, 250);
      CHECK(r.got[0] >= before + 49 && r.d.responsive(0), "recovers by itself when the interrupts are back (%d new replies)", r.got[0] - before); }
    { Rig r; r.begin(PROTO_DSHOT300, true); r.run(20, 250);
      int ok = 0;
      for (int k = 0; k < 100; k++) {                                   // an old interrupt lands inside the next frame
          r.w.run_to(r.w.now + 250000); r.d.send(r.value, r.telem);
          r.w.now += 5000 + (k * 400) % 45000; r.w.spurious_isr();
      }
      r.w.run_to(r.w.now + 1000000); r.w.settle_all();
      for (int c = 0; c < 4; c++) if (r.w.ch[c].bad == 0 && r.good(c) == 120) ok++;
      CHECK(ok == 4, "an interrupt inside a frame does not cut the frame (%d bad frames on motor 1)", r.w.ch[0].bad);
      CHECK(r.d.stats().replies >= 4 * 115, "and the real interrupt still releases the pin afterwards (%u replies)", (unsigned)r.d.stats().replies); }
    { Rig r; r.begin(PROTO_DSHOT300, true);
      r.w.spurious_isr();
      CHECK(r.w.ch[0].drv.size() == 1 && r.w.ch[0].drv[0].second, "an interrupt before the first frame does nothing");
      r.run(10, 250); CHECK(r.got[0] >= 9, "and the sequence runs normally afterwards"); }
    { // interrupt exactly while the next send() is collecting replies
      Rig r; r.begin(PROTO_DSHOT300, true); r.w.isr_latency_ns = 250000 - 57000 + 1200; r.run(300, 250);
      bool ok = true; for (int c = 0; c < 4; c++) ok = ok && r.good(c) == 300 && r.w.ch[c].bad == 0 && r.wrong[c] == 0 && r.w.ch[c].starts_while_released == 0;
      CHECK(ok, "interrupt in the middle of the next send(): frames valid, pins driven (%d good, %d bad)", r.good(0), r.w.ch[0].bad); }
}

static void t_loopback() {
    printf("[start-up check: the receiver hears the frame the transmitter puts on the pin]\n");
    for (Protocol p : {PROTO_DSHOT150, PROTO_DSHOT300, PROTO_DSHOT600, PROTO_DSHOT1200})
      for (uint32_t tick : {80000000u, 40000000u}) {
        Rig r; r.w.tick = tick; CHECK(r.begin(p, true), "begin");
        bool ok = true; int words = 0;
        for (int c = 0; c < 4; c++) { const uint32_t seen = r.d.loopback(c, 0, words); ok = ok && seen == dshot_packet(0, false, true) && words >= 16; }
        for (uint16_t v : {(uint16_t)0, (uint16_t)1047, (uint16_t)2047, (uint16_t)0x555}) { const uint32_t seen = r.d.loopback(1, v, words); ok = ok && seen == dshot_packet(v, false, true); }
        CHECK(ok, "DShot%u, %u MHz ticks: every pin, several values, read back bit for bit (%d words)", (unsigned)dshot_kbps(p), (unsigned)(tick / 1000000), words);
        r.run(200, p == PROTO_DSHOT150 ? 500 : 250);
        ns_t cont = 0; bool good = true; for (int c = 0; c < 4; c++) { cont += r.w.ch[c].contention_ns; good = good && r.w.ch[c].bad == 0 && r.got[c] >= 199; }
        CHECK(cont == 0 && good, "the ESC answers the check frames with the pin released (no fight), and normal frames + replies follow (%lld ns, %d replies)", cont, r.got[0]);
      }
    { Rig r; r.begin(PROTO_DSHOT600, true); r.w.ch[2].tx_cut = true; int words = -1;
      CHECK(r.d.loopback(0, 0, words) == dshot_packet(0, false, true) && r.d.loopback(2, 0, words) == DshotDirect<SimPort>::kLoopNothing && words == 0, "the transmitter of motor 3 does not reach its pin: 'heard nothing'"); }
    { Rig r; r.begin(PROTO_DSHOT600, true); r.w.ch[1].rx_deaf = true; int words = -1;
      CHECK(r.d.loopback(1, 0, words) == DshotDirect<SimPort>::kLoopNothing, "the receiver of motor 2 hears nothing: the same answer (the boot report then names the motor)"); }
    { Rig r; r.begin(PROTO_DSHOT600, false); int words = 0; CHECK(r.d.loopback(0, 0, words) == DshotDirect<SimPort>::kLoopNothing && r.w.ch[0].frames.empty(), "one-way: no check (there is no receiver), nothing sent"); }
}

static void t_proshot() {
    printf("[ProShot1000 through the same sequencer: 4 nibble pulses, one-way]\n");
    Rig r; CHECK(r.begin(PROTO_PROSHOT1000, true), "begin (bidirectional asked for)");
    CHECK(r.d.cycle_us() == 16 + 30 && r.d.frame_us() == 16, "16 us a frame + 30 us pause (%u)", (unsigned)r.d.cycle_us());
    r.value[0] = 1047; r.value[1] = 48; r.value[2] = 0; r.value[3] = 2047; r.run(3, 250, 0, false);
    bool ok = r.w.isr_runs == 0;
    for (int c = 0; c < 4 && ok; c++) {
        const uint16_t pk = dshot_packet(r.value[c], false, false); const auto& f = r.w.ch[c].frames.back();
        for (int i = 0; i < 4; i++) { uint32_t d0, d1; int l0, l1; Wire::word(f.w[i], d0, l0, d1, l1); ok = ok && l0 == 1 && l1 == 0 && d0 + d1 == 320 && d0 == 80 + ((pk >> (12 - 4 * i)) & 0xF) * 10; }
        uint32_t d0, d1; int l0, l1; Wire::word(f.w[4], d0, l0, d1, l1); ok = ok && d0 == 0 && l0 == 0;
    }
    CHECK(ok, "every motor: its packet as 4 pulses, then the end mark, line low; no timer (one-way, whatever was asked for)");
    CHECK(!r.d.responsive(0) && r.d.stats().replies == 0, "no replies expected");
}

int main() {
    t_words();
    t_oneway();
    t_bidir();
    t_loopback();
    t_proshot();
    t_timing();
    printf("\n%d checks, %d failed\n", g_pass + g_fail, g_fail);
    return g_fail ? 1 : 0;
}
