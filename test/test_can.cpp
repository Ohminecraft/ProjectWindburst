// DroneCAN: the transport layer and the data types against frames made by pydronecan (test/dronecan_vectors.h), the
// reassembly of multi-frame transfers with everything that can go wrong on a bus, and the node (dronecan_node.h).
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <vector>
#include "../src/hal/can/dronecan.h"
#include "../src/hal/can/dronecan_msgs.h"
#include "../src/hal/can/dronecan_node.h"
#include <deque>
#include "dronecan_vectors.h"
using namespace dcan;

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, ...) do { const bool ok_ = (cond); if (getenv("V")) { printf("    "); printf(__VA_ARGS__); printf("\n"); } if (ok_) g_pass++; else { g_fail++; printf("  FAIL line %d: %s   ", __LINE__, #cond); printf(__VA_ARGS__); printf("\n"); } } while (0)

static const DcVector& vec(const char* name) { for (const DcVector& v : kDcVectors) if (!strcmp(v.name, name)) return v; printf("no vector %s\n", name); exit(2); }
// our frames == pydronecan's, bit for bit
static bool same(const DcVector& v, const Frame* f, int n) {
    if (n != v.n) { printf("    %s: %d frames, pydronecan %d\n", v.name, n, v.n); return false; }
    for (int i = 0; i < n; i++) {
        if (f[i].id != v.id[i] || f[i].len != v.len[i] || memcmp(f[i].data, v.data[i], f[i].len)) {
            printf("    %s frame %d: id %08X len %u, pydronecan %08X len %u\n      ours ", v.name, i, (unsigned)f[i].id, f[i].len, (unsigned)v.id[i], v.len[i]);
            for (int k = 0; k < f[i].len; k++) printf("%02X ", f[i].data[k]);
            printf("\n      them "); for (int k = 0; k < v.len[i]; k++) printf("%02X ", v.data[i][k]); printf("\n");
            return false;
        }
    }
    return true;
}
static uint64_t sig_all(Kind k, uint16_t id, void*) {
    if (k == MESSAGE) switch (id) { case kNodeStatusId: return kNodeStatusSig; case kFix2Id: return kFix2Sig; case kGnssAuxId: return kGnssAuxSig; case kMag2Id: return kMag2Sig; case kAllocationId: return kAllocationSig; case kPressureId: return kPressureSig; case kTemperatureId: return kTemperatureSig; case kBatteryId: return kBatterySig; case kRawCommandId: return kRawCommandSig; case kEscStatusId: return kEscStatusSig; default: return 0; }
    if (id == kGetNodeInfoId) return kGetNodeInfoSig;
    return 0;
}
// pydronecan's frames back into a transfer
static bool reassemble(const DcVector& v, std::vector<uint8_t>& out, Transfer& t, uint8_t me = 0) {
    Receiver<> rx(&sig_all, nullptr); rx.set_node_id(me);
    for (int i = 0; i < v.n; i++) { Frame f; f.id = v.id[i]; f.len = v.len[i]; memcpy(f.data, v.data[i], f.len); if (rx.feed(f, 1000, t)) { out.assign(t.data, t.data + t.len); return i == v.n - 1; } }
    return false;
}

static void t_codec() {
    printf("[codec] float16, bit packing\n");
    { const float vals[] = {0.0f, 1.0f, -2.5f, 0.3f, 65504.0f, 6.1035e-5f, 1e-6f, -0.125f};
      bool ok = true; for (float v : vals) { const float back = f16_to(f16_from(v)); if (fabsf(back - v) > fabsf(v) * 1e-3f + 1e-7f) { ok = false; printf("    %g -> %g\n", v, back); } }
      CHECK(ok && f16_from(1.0f) == 0x3C00 && f16_from(-2.0f) == 0xC000 && f16_from(1e6f) == 0x7C00 && std::isinf(f16_to(0x7C00)), "float16: values round-trip, 1.0 = 0x3C00, -2 = 0xC000, too large = infinity"); }
    { uint8_t b[8]; Writer w(b, sizeof(b)); w.u(5, 3); w.u(0x1FF, 9); w.i(-3, 5); w.boolean(true);
      Reader r(b, w.bytes()); const uint64_t a = r.u(3), c = r.u(9); const int64_t d = r.i(5); const bool e = r.boolean();
      CHECK(a == 5 && c == 0x1FF && d == -3 && e && w.bits() == 18 && b[0] == 0xBF, "odd-sized fields across byte boundaries: written and read back (first byte %02X)", b[0]); }
    { uint8_t b[2]; Writer w(b, 2); w.u(0xFFFFFF, 24); CHECK(w.overflow(), "a writer that runs out of room says so"); Reader r(b, 2); r.u(24); CHECK(r.too_short(), "and so does a reader"); }
}

static void t_vectors() {
    printf("[vectors] our frames == pydronecan's\n");
    Frame f[24]; uint8_t p[300]; size_t n;
    { NodeStatus s; s.uptime_sec = 123456; s.health = 1; s.mode = 2; s.sub_mode = 5; s.vendor = 0xBEEF; n = encode(s, p, sizeof(p));
      const int k = encode_transfer(message_id(16, kNodeStatusId, 10), kNodeStatusSig, 7, p, n, f, 24);
      CHECK(n == 7 && same(vec("node_status"), f, k), "NodeStatus: single frame"); }
    Fix2 fx; fx.timestamp_us = 1234567890123ull; fx.gnss_timestamp_us = 987654321ull; fx.time_standard = 3; fx.leap_seconds = 18; fx.lon_1e8 = 854559380; fx.lat_1e8 = 4739774190ll;
    fx.height_ellipsoid_mm = 488123; fx.height_msl_mm = 440555; fx.ned_velocity[0] = 1.25f; fx.ned_velocity[1] = -0.5f; fx.ned_velocity[2] = 0.125f; fx.sats_used = 17; fx.status = 3; fx.mode = 1; fx.sub_mode = 1;
    { const float cv[6] = {0.25f, 0.25f, 0.5f, 0.01f, 0.01f, 0.02f}; fx.cov_len = 6; for (int i = 0; i < 6; i++) fx.covariance[i] = cv[i]; } fx.pdop = 1.5f;
    { n = encode(fx, p, sizeof(p)); const int k = encode_transfer(message_id(16, kFix2Id, 125), kFix2Sig, 21, p, n, f, 24);
      CHECK(same(vec("fix2"), f, k), "Fix2 with a covariance: %d frames, the transfer CRC, the toggle bits", k); }
    { Fix2 s; s.lon_1e8 = -12239500000ll; s.lat_1e8 = -3370000000ll; s.height_ellipsoid_mm = -50000; s.height_msl_mm = -12345; s.ned_velocity[0] = -3; s.ned_velocity[1] = 2; s.ned_velocity[2] = -1; s.sats_used = 6; s.status = 2; s.pdop = 3.25f;
      n = encode(s, p, sizeof(p)); const int k = encode_transfer(message_id(20, kFix2Id, 42), kFix2Sig, 31, p, n, f, 24);
      CHECK(same(vec("fix2_south_west"), f, k), "Fix2 south-west (negative 37-bit fields), no covariance"); }
    { GnssAux a; a.gdop = 1.5f; a.pdop = 1.25f; a.hdop = 0.75f; a.vdop = 1; a.tdop = 0.5f; a.ndop = 0.5f; a.edop = 0.5f; a.sats_visible = 24; a.sats_used = 17;
      n = encode(a, p, sizeof(p)); const int k = encode_transfer(message_id(20, kGnssAuxId, 125), kGnssAuxSig, 3, p, n, f, 24); CHECK(same(vec("gnss_aux"), f, k), "gnss.Auxiliary"); }
    { Mag2 m; m.sensor_id = 1; m.field_ga[0] = 0.3f; m.field_ga[1] = -0.05f; m.field_ga[2] = 0.38f;
      n = encode(m, p, sizeof(p)); const int k = encode_transfer(message_id(20, kMag2Id, 125), kMag2Sig, 9, p, n, f, 24); CHECK(n == 7 && same(vec("mag2"), f, k), "MagneticFieldStrength2 (the empty covariance tail)"); }
    { Allocation a; a.node_id = 0; a.first_part = true; a.uid_len = 6; for (int i = 0; i < 6; i++) a.uid[i] = (uint8_t)(0x11 * (i + 1));
      n = encode(a, p, sizeof(p)); const int k = encode_transfer(anonymous_id(30, 0x1A5, kAllocationId), kAllocationSig, 0, p, n, f, 24); CHECK(same(vec("alloc_request"), f, k), "an anonymous Allocation request (discriminator, type id & 3)"); }
    { Allocation a; a.node_id = 125; a.first_part = false; a.uid_len = 16; for (int i = 0; i < 16; i++) a.uid[i] = (uint8_t)(i + 1);
      n = encode(a, p, sizeof(p)); const int k = encode_transfer(message_id(30, kAllocationId, 10), kAllocationSig, 4, p, n, f, 24); CHECK(same(vec("alloc_response"), f, k), "the allocator's answer with the whole unique id"); }
    { NodeInfo ni; ni.status.uptime_sec = 77; ni.sw_minor = 15; ni.hw_major = 1; ni.hw_minor = 2; for (int i = 0; i < 16; i++) ni.uid[i] = (uint8_t)i; strcpy(ni.name, "org.windburst.fc");
      n = encode(ni, p, sizeof(p)); const int k = encode_transfer(service_id(30, kGetNodeInfoId, false, 125, 10), kGetNodeInfoSig, 5, p, n, f, 24); CHECK(same(vec("node_info_response"), f, k), "a GetNodeInfo response (service frame, the name as tail array)"); }
    { const int k = encode_transfer(service_id(30, kGetNodeInfoId, true, 125, 10), kGetNodeInfoSig, 6, p, 0, f, 24); CHECK(same(vec("node_info_request"), f, k), "the empty GetNodeInfo request"); }
}


static void t_vectors2() {
    printf("[vectors] baro, battery, ESC: our frames == pydronecan's, and back\n");
    Frame f[24]; uint8_t p[300]; size_t n; std::vector<uint8_t> pl; Transfer t;
    { Pressure v; v.pa = 97123.5f; v.variance = 4.0f; n = encode(v, p, sizeof(p)); const int k = encode_transfer(message_id(16, kPressureId, 77), kPressureSig, 2, p, n, f, 24);
      Pressure b; CHECK(same(vec("pressure"), f, k) && reassemble(vec("pressure"), pl, t) && decode(pl.data(), pl.size(), b) && b.pa == 97123.5f, "StaticPressure %.1f Pa", b.pa); }
    { Temperature v; v.kelvin = 293.5f; v.variance = 1.0f; n = encode(v, p, sizeof(p)); const int k = encode_transfer(message_id(20, kTemperatureId, 77), kTemperatureSig, 3, p, n, f, 24);
      CHECK(same(vec("temperature"), f, k), "StaticTemperature"); }
    { Battery b; b.temperature_k = 300; b.voltage = 15.75f; b.current = 12.5f; b.power_10s = 190; b.remaining_wh = 20; b.full_wh = 33; b.status_flags = 0x41; b.soh_pct = 97; b.soc_pct = 61; b.soc_stdev = 2; b.model_instance = 12345; strcpy(b.model, "4S 2200");
      n = encode(b, p, sizeof(p)); const int k = encode_transfer(message_id(20, kBatteryId, 78), kBatterySig, 9, p, n, f, 24); Battery d;
      CHECK(same(vec("battery"), f, k) && reassemble(vec("battery"), pl, t) && decode(pl.data(), pl.size(), d) && fabsf(d.voltage - 15.75f) < 0.01f && fabsf(d.current - 12.5f) < 0.01f && d.soc_pct == 61 && !strcmp(d.model, "4S 2200"),
            "BatteryInfo: %.2f V, %.2f A, %u %%, '%s'", d.voltage, d.current, d.soc_pct, d.model); }
    { RawCommand c; c.n = 4; c.cmd[0] = 0; c.cmd[1] = 1000; c.cmd[2] = -8191; c.cmd[3] = 8191; n = encode(c, p, sizeof(p)); const int k = encode_transfer(message_id(4, kRawCommandId, 10), kRawCommandSig, 17, p, n, f, 24);
      RawCommand d; CHECK(n == 7 && k == 1 && same(vec("raw_command"), f, k) && reassemble(vec("raw_command"), pl, t) && decode(pl.data(), pl.size(), d) && d.n == 4 && d.cmd[2] == -8191 && d.cmd[1] == 1000, "RawCommand: four 14-bit commands in one frame"); }
    { EscStatus e; e.error_count = 3; e.voltage = 15.5f; e.current = 4.25f; e.temperature_k = 310; e.rpm = -21000; e.power_pct = 40; e.index = 2; n = encode(e, p, sizeof(p)); const int k = encode_transfer(message_id(16, kEscStatusId, 61), kEscStatusSig, 11, p, n, f, 24);
      EscStatus d; CHECK(same(vec("esc_status"), f, k) && reassemble(vec("esc_status"), pl, t) && decode(pl.data(), pl.size(), d) && d.rpm == -21000 && d.index == 2 && fabsf(d.current - 4.25f) < 0.01f, "esc.Status: ESC %u, %d rpm", d.index, (int)d.rpm); }
}

static void t_decode() {
    printf("[decode] pydronecan's frames -> transfers -> values\n");
    std::vector<uint8_t> pl; Transfer t;
    { CHECK(reassemble(vec("fix2"), pl, t) && t.type_id == kFix2Id && t.src == 125 && t.kind == MESSAGE, "Fix2: reassembled from 10 frames, CRC good");
      Fix2 f; CHECK(decode(pl.data(), pl.size(), f) && f.lat_1e8 == 4739774190ll && f.lon_1e8 == 854559380 && f.height_msl_mm == 440555 && f.sats_used == 17 && f.status == 3 && f.cov_len == 6 && fabsf(f.covariance[2] - 0.5f) < 1e-3f && fabsf(f.pdop - 1.5f) < 1e-3f && f.ned_velocity[1] == -0.5f,
            "Fix2 values: %.7f N %.7f E, %d mm MSL, %u sats, status %u, pdop %.2f", f.lat_1e8 * 1e-8, f.lon_1e8 * 1e-8, f.height_msl_mm, f.sats_used, f.status, f.pdop); }
    { Fix2 f; CHECK(reassemble(vec("fix2_south_west"), pl, t) && decode(pl.data(), pl.size(), f) && f.lat_1e8 == -3370000000ll && f.lon_1e8 == -12239500000ll && f.height_ellipsoid_mm == -50000 && f.cov_len == 0, "negative coordinates sign-extended"); }
    { GnssAux a; CHECK(reassemble(vec("gnss_aux"), pl, t) && decode(pl.data(), pl.size(), a) && fabsf(a.hdop - 0.75f) < 1e-3f && a.sats_visible == 24, "gnss.Auxiliary: hdop %.2f, %u visible", a.hdop, a.sats_visible); }
    { Mag2 m; CHECK(reassemble(vec("mag2"), pl, t) && decode(pl.data(), pl.size(), m) && fabsf(m.field_ga[0] - 0.3f) < 1e-3f && fabsf(m.field_ga[1] + 0.05f) < 1e-4f, "MagneticFieldStrength2: %.3f %.3f %.3f G", m.field_ga[0], m.field_ga[1], m.field_ga[2]); }
    { Allocation a; CHECK(reassemble(vec("alloc_request"), pl, t) && t.anonymous && t.type_id == kAllocationId && decode(pl.data(), pl.size(), a) && a.first_part && a.uid_len == 6 && a.uid[5] == 0x66, "an anonymous Allocation request: first part, 6 bytes of the unique id"); }
    { NodeInfo ni; CHECK(reassemble(vec("node_info_response"), pl, t, 125) && t.kind == RESPONSE && t.src == 10 && t.dst == 125 && decode(pl.data(), pl.size(), ni) && !strcmp(ni.name, "org.windburst.fc") && ni.uid[15] == 15 && ni.sw_minor == 15, "GetNodeInfo response to node 125: name '%s'", ni.name); }
    { CHECK(!reassemble(vec("node_info_response"), pl, t, 99), "a service call for another node is not ours"); }
}

static void t_receiver() {
    printf("[receiver] a busy bus: interleaved transfers, lost frames, damaged ones\n");
    Frame a[24], b[24]; uint8_t p[300]; Fix2 fx; fx.lat_1e8 = 1; fx.cov_len = 6; fx.sats_used = 9; size_t n = encode(fx, p, sizeof(p));
    const int na = encode_transfer(message_id(16, kFix2Id, 20), kFix2Sig, 3, p, n, a, 24);
    fx.sats_used = 11; n = encode(fx, p, sizeof(p)); const int nb = encode_transfer(message_id(16, kFix2Id, 21), kFix2Sig, 3, p, n, b, 24);
    { Receiver<> rx(&sig_all, nullptr); Transfer t; int got = 0; uint8_t sats[2] = {};
      for (int i = 0; i < na || i < nb; i++) { if (i < na && rx.feed(a[i], 0, t)) { Fix2 f; decode(t.data, t.len, f); sats[got++] = f.sats_used; } if (i < nb && rx.feed(b[i], 0, t)) { Fix2 f; decode(t.data, t.len, f); sats[got++] = f.sats_used; } }
      CHECK(got == 2 && sats[0] == 9 && sats[1] == 11, "two GPS modules sending at once, frame by frame interleaved: both arrive (%d)", got); }
    { Receiver<> rx(&sig_all, nullptr); Transfer t; int got = 0; for (int i = 0; i < na; i++) if (i != 4) got += rx.feed(a[i], 0, t);
      CHECK(got == 0 && rx.bad_frames() >= 1, "a frame lost in the middle: the transfer is dropped (toggle out of step)");
      for (int i = 0; i < na; i++) { got += rx.feed(a[i], 10, t); }
      CHECK(got == 1, "the next one arrives whole"); }
    { Receiver<> rx(&sig_all, nullptr); Transfer t; Frame c[24]; memcpy(c, a, sizeof(Frame) * na); c[3].data[2] ^= 0x10; int got = 0; for (int i = 0; i < na; i++) got += rx.feed(c[i], 0, t);
      CHECK(got == 0 && rx.crc_errors() == 1, "one bit flipped: the CRC catches it"); }
    { Receiver<> rx(&sig_all, nullptr); Transfer t; int got = 0; for (int i = 0; i < 3; i++) got += rx.feed(a[i], 0, t); for (int i = 3; i < na; i++) got += rx.feed(a[i], 5000, t);
      CHECK(got == 0, "a transfer that stalls for 5 s is given up"); }
    { Receiver<> rx(&sig_all, nullptr); Transfer t; Frame f; f.id = message_id(16, 1234, 5); f.len = 3; f.data[2] = tail(true, true, false, 0);
      CHECK(!rx.feed(f, 0, t) && rx.transfers() == 0, "a type nobody asked for: ignored"); }
    { Receiver<2> rx(&sig_all, nullptr); Transfer t; int started = 0;
      for (uint8_t src = 1; src <= 3; src++) { Frame g[24]; const int k = encode_transfer(message_id(16, kFix2Id, src), kFix2Sig, 0, p, n, g, 24); (void)k; rx.feed(g[0], 0, t); started++; }
      CHECK(rx.no_slot() == 1, "more transfers at once than slots: the extra one is dropped and counted"); }
}


// ---- a CAN bus with several nodes: what one sends, every other one receives
static uint8_t frame_header_prio(const Frame& f) { return (uint8_t)((f.id >> 24) & 31u); }
struct Hub;
struct Port : Bus {
    Hub* hub = nullptr; std::deque<Frame> in; size_t tx_room = 1000;
    bool send(const Frame& f) override;
    bool receive(Frame& f) override { if (in.empty()) return false; f = in.front(); in.pop_front(); return true; }
};
struct Hub { std::vector<Port*> ports; int frames = 0; void put(const Port* from, const Frame& f) { frames++; for (Port* p : ports) if (p != from) p->in.push_back(f); } };
bool Port::send(const Frame& f) { if (!tx_room) return false; tx_room--; hub->put(this, f); return true; }

// A GPS + compass module as they come (Here3, M10Q-CAN ...): asks for a node id, then sends NodeStatus, Fix2 at 10 Hz, its
// Auxiliary at 2 Hz and the field at 50 Hz, answers GetNodeInfo.
struct Module {
    Port port; uint8_t uid[16]; uint8_t id = 0, want = 0; const char* name; int stage = 0; uint32_t ask_ms = 0; uint8_t tid = 0, tid_s = 0, tid_f = 0, tid_m = 0;
    Fix2 fix; float field[3] = {0.3f, -0.05f, 0.38f}; bool gps = true, compass = true; uint32_t up_ms = 0;
    Module(uint8_t seed, const char* n, uint8_t preferred = 0) : want(preferred), name(n) { for (int i = 0; i < 16; i++) uid[i] = (uint8_t)(seed * 16 + i); fix.status = GNSS_3D; fix.sats_used = 14; fix.lat_1e8 = 4739774190ll; fix.lon_1e8 = 854559380; fix.height_msl_mm = 440000; fix.cov_len = 6; for (int k = 0; k < 6; k++) fix.covariance[k] = k < 3 ? 0.36f : 0.01f; }
    void send(uint32_t id_, uint64_t sig, uint8_t t, const uint8_t* p, size_t n) { Frame f[24]; const int k = encode_transfer(id_, sig, t, p, n, f, 24); for (int i = 0; i < k; i++) port.send(f[i]); }
    void ask(int part) {                                          // the allocatee's request: parts of 6, 6 and 4 bytes of the unique id
        Allocation a; a.node_id = want; a.first_part = part == 0; const int off = part * 6, len = part == 2 ? 4 : 6; a.uid_len = (uint8_t)len; memcpy(a.uid, uid + off, len);
        uint8_t p[16]; const size_t n = encode(a, p, sizeof(p)); send(anonymous_id(30, (uint16_t)(uid[0] * 77 + part), kAllocationId), kAllocationSig, 0, p, n);
    }
    void step(uint32_t ms) {
        Frame f; Transfer t;
        while (port.receive(f)) if (feed(f, ms, t)) handle(t, ms);
        if (!id) { if (ms - ask_ms >= 600u + (uid[0] * 37u) % 400u) { stage = 0; ask(0); ask_ms = ms; } return; }      // (no answer: start again, after a delay of its own)
        if (ms % 1000 == 0) { NodeStatus s; s.uptime_sec = (ms - up_ms) / 1000; uint8_t p[8]; const size_t n = encode(s, p, 8); send(message_id(16, kNodeStatusId, id), kNodeStatusSig, tid_s++, p, n); }
        if (gps && ms % 100 == 0) { uint8_t p[120]; const size_t n = encode(fix, p, sizeof(p)); send(message_id(16, kFix2Id, id), kFix2Sig, tid_f++, p, n);
                                    if (ms % 500 == 0) { GnssAux a; a.hdop = 0.8f; a.sats_used = fix.sats_used; const size_t m = encode(a, p, sizeof(p)); send(message_id(20, kGnssAuxId, id), kGnssAuxSig, tid++, p, m); } }
        if (compass && ms % 20 == 0) { Mag2 m; for (int k = 0; k < 3; k++) m.field_ga[k] = field[k]; uint8_t p[16]; const size_t n = encode(m, p, sizeof(p)); send(message_id(20, kMag2Id, id), kMag2Sig, tid_m++, p, n); }
    }
    Receiver<4, 256> rx_{&sig_all, nullptr};
    bool feed(const Frame& f, uint32_t ms, Transfer& t) { rx_.set_node_id(id); return rx_.feed(f, ms, t); }
    void handle(const Transfer& t, uint32_t ms) {
        if (!id && t.kind == MESSAGE && t.type_id == kAllocationId && !t.anonymous) {
            Allocation a; if (!decode(t.data, t.len, a)) return;
            const int have = stage == 0 ? 6 : stage == 1 ? 12 : 16;
            if (a.uid_len != have || memcmp(a.uid, uid, have)) return;                    // an answer for somebody else
            if (have == 16) { if (a.node_id) { id = a.node_id; up_ms = ms; } return; }
            stage++; ask(stage); ask_ms = ms;
            return;
        }
        if (id && t.kind == REQUEST && t.type_id == kGetNodeInfoId) {
            NodeInfo ni; strcpy(ni.name, name); memcpy(ni.uid, uid, 16); uint8_t p[128]; const size_t n = encode(ni, p, sizeof(p));
            send(service_id(24, kGetNodeInfoId, false, t.src, id), kGetNodeInfoSig, t.tid, p, n);
        }
    }
};

static void t_node() {
    printf("[node] the flight controller on a bus with GPS / compass modules: ids, names, data\n");
    Hub hub; Port fc_port; fc_port.hub = &hub; hub.ports.push_back(&fc_port);
    Module gps1(1, "org.ardupilot.here3", 0), gps2(2, "com.matek.m10q", 100); gps1.port.hub = gps2.port.hub = &hub; hub.ports.push_back(&gps1.port); hub.ports.push_back(&gps2.port);
    gps2.gps = true; gps2.compass = false; gps2.fix.sats_used = 9;
    Node node; Node::Config c; c.node_id = 10; c.sw_minor = 15; node.begin(&fc_port, c, 0);
    uint32_t ms = 1;
    auto run = [&](uint32_t dur) { for (uint32_t end = ms + dur; ms < end; ms++) { gps1.step(ms); node.update(ms); gps2.step(ms); node.update(ms); } };
    run(3000);
    CHECK(gps1.id == 125 && gps2.id == 100 && node.allocations() == 2, "dynamic ids: the module that asked for none got the highest free one, the one that asked for 100 got it (%u, %u)", gps1.id, gps2.id);
    CHECK(node.nodes() == 2 && node.node(0).health == HEALTH_OK, "both in the node table (%d)", node.nodes());
    run(3000);
    bool names = true; for (int i = 0; i < node.nodes(); i++) { const NodeEntry& e = node.node(i); names &= e.has_info && !strcmp(e.name, e.id == gps1.id ? "org.ardupilot.here3" : "com.matek.m10q"); }
    CHECK(names, "their names, asked once each with GetNodeInfo: '%s', '%s'", node.node(0).name, node.node(1).name);
    Fix2 f; GnssAux a; bool aux = false; const bool got = node.take_gnss(f, &a, aux);
    CHECK(got && node.gnss_node() == gps1.id && f.sats_used == 14 && f.lat_1e8 == 4739774190ll && aux && fabsf(a.hdop - 0.8f) < 1e-3f, "the GNSS fix of the first GPS (node %u, %u sats, hdop %.2f); the second one is only listed", node.gnss_node(), f.sats_used, a.hdop);
    float ga[3] = {}; CHECK(node.take_mag(ga) && node.mag_node() == gps1.id && fabsf(ga[2] - 0.38f) < 1e-3f && !node.take_mag(ga), "the compass field (%.2f %.2f %.2f G), taken once per sample", ga[0], ga[1], ga[2]);
    // the module in use goes quiet: after 2 s the other GPS is taken
    gps1.gps = false; run(2500); node.take_gnss(f, &a, aux); run(200);
    CHECK(node.take_gnss(f, &a, aux) && node.gnss_node() == gps2.id && f.sats_used == 9, "the GPS in use goes quiet: the other one takes over (node %u)", node.gnss_node());
    // the flight controller's own status, and its name for a configuration tool
    { Port tool; tool.hub = &hub; hub.ports.push_back(&tool); run(1100); int status = 0; Frame fr; Receiver<4, 256> rx(&sig_all, nullptr); rx.set_node_id(100); Transfer t;
      while (tool.receive(fr)) { Header h; parse(fr, h); if (h.kind == MESSAGE && h.type_id == kNodeStatusId && h.src == 10) status++; }
      CHECK(status == 1, "NodeStatus from node 10 once a second (%d in 1.1 s)", status);
      Frame q[2]; encode_transfer(service_id(30, kGetNodeInfoId, true, 10, 100), kGetNodeInfoSig, 3, nullptr, 0, q, 2); tool.send(q[0]); run(5);
      NodeInfo ni; bool answered = false; while (tool.receive(fr)) if (rx.feed(fr, ms, t) && t.kind == RESPONSE && t.src == 10 && t.tid == 3) answered = decode(t.data, t.len, ni);
      CHECK(answered && !strcmp(ni.name, "org.windburst.fc") && ni.sw_minor == 15, "GetNodeInfo from a tool (node 100): answered, same transfer id, '%s' %u.%u", ni.name, ni.sw_major, ni.sw_minor);
      hub.ports.pop_back(); }
    // a module that restarts gets the same id again
    { Module again(1, "org.ardupilot.here3"); again.port.hub = &hub; hub.ports.push_back(&again.port); for (uint32_t end = ms + 1500; ms < end; ms++) { again.step(ms); node.update(ms); }
      CHECK(again.id == gps1.id && node.allocations() == 2, "the same module asking again (a restart): the same id %u, no new allocation", again.id); hub.ports.pop_back(); }
    // a full transmit queue: counted, nothing blocks
    fc_port.tx_room = 0; run(1100); CHECK(node.tx_dropped() >= 1, "a transmit queue that takes nothing: frames dropped and counted (%u)", node.tx_dropped());
}

// A barometer + power module (fixed id 60) and four ESCs (ids 61..64, index 0..3) next to the flight controller.
static void t_node2() {
    printf("[node] barometer, power module and ESCs on the bus\n");
    Hub hub; Port fc_port, mod; fc_port.hub = mod.hub = &hub; hub.ports.push_back(&fc_port); hub.ports.push_back(&mod);
    Node node; Node::Config c; c.node_id = 10; node.begin(&fc_port, c, 0);
    uint8_t tid = 0;
    auto send = [&](uint32_t id_, uint64_t sig, const uint8_t* p, size_t n) { Frame f[24]; const int k = encode_transfer(id_, sig, tid++, p, n, f, 24); for (int i = 0; i < k; i++) mod.send(f[i]); };
    uint8_t p[64]; size_t n;
    { Pressure v; v.pa = 95000.0f; n = encode(v, p, sizeof(p)); send(message_id(16, kPressureId, 60), kPressureSig, p, n);
      Temperature t; t.kelvin = 300.15f; n = encode(t, p, sizeof(p)); send(message_id(16, kTemperatureId, 60), kTemperatureSig, p, n); }
    node.update(100);
    float pa = 0, tc = 0;
    CHECK(node.take_baro(pa, tc) && pa == 95000.0f && fabsf(tc - 27.0f) < 0.2f && node.baro_node() == 60 && !node.take_baro(pa, tc), "StaticPressure %.0f Pa + StaticTemperature %.1f C from node %u, taken once", pa, tc, node.baro_node());
    { Pressure v; v.pa = 500.0f; n = encode(v, p, sizeof(p)); send(message_id(16, kPressureId, 60), kPressureSig, p, n); node.update(110);
      CHECK(!node.take_baro(pa, tc), "a pressure no barometer on earth reads (500 Pa): ignored"); }
    { Pressure v; v.pa = 90000.0f; n = encode(v, p, sizeof(p)); send(message_id(16, kPressureId, 70), kPressureSig, p, n); node.update(120);
      CHECK(!node.take_baro(pa, tc) && node.baro_node() == 60, "a second barometer (node 70) while the first one talks: listed, not used"); }
    { Battery b; b.voltage = 16.2f; b.current = 8.5f; b.soc_pct = 80; n = encode(b, p, sizeof(p)); send(message_id(20, kBatteryId, 60), kBatterySig, p, n); node.update(130);
      Battery got; CHECK(node.battery(got, 200) && fabsf(got.voltage - 16.2f) < 0.01f && fabsf(got.current - 8.5f) < 0.01f && node.battery_node() == 60, "BatteryInfo: %.2f V %.2f A from node %u", got.voltage, got.current, node.battery_node());
      CHECK(!node.battery(got, 2200), "the power module quiet for 2 s: no battery reading"); }
    for (int i = 0; i < 4; i++) { EscStatus e; e.index = (uint8_t)i; e.rpm = 10000 + i * 100; e.voltage = 16.0f; e.current = 2.0f + i; e.temperature_k = 313.15f; n = encode(e, p, sizeof(p)); send(message_id(16, kEscStatusId, (uint8_t)(61 + i)), kEscStatusSig, p, n); }
    { EscStatus e; e.index = 9; n = encode(e, p, sizeof(p)); send(message_id(16, kEscStatusId, 66), kEscStatusSig, p, n); }
    node.update(300);
    { EscStatus e; bool ok = true; for (int i = 0; i < 4; i++) ok &= node.take_esc(i, e) && e.rpm == 10000 + i * 100 && fabsf(e.current - (2.0f + i)) < 0.01f;
      CHECK(ok && node.esc_mask(300) == 0x0F && !node.take_esc(0, e) && node.esc_mask(1400) == 0, "esc.Status of ESC 0..3 (index 9: beyond the table, ignored); heard-from mask %02X, empty a second later", node.esc_mask(300)); }
    // the commands: one frame for four ESCs, what an ESC on the bus decodes
    { const int16_t cmd[4] = {0, 1, 4096, 9000}; Frame fr; Transfer t; Receiver<4, 256> rx(&sig_all, nullptr); RawCommand d; int got = 0;
      CHECK(node.send_esc(cmd, 4) && node.esc_sent() == 1, "send_esc(): sent");
      while (mod.receive(fr)) if (rx.feed(fr, 300, t) && t.type_id == kRawCommandId) { got++; decode(t.data, t.len, d); CHECK(frame_header_prio(fr) == kPrioHigh, "RawCommand at high priority"); }
      CHECK(got == 1 && d.n == 4 && d.cmd[0] == 0 && d.cmd[1] == 1 && d.cmd[2] == 4096 && d.cmd[3] == 8191, "an ESC reads 4 commands, the one above 8191 clamped (%d %d %d %d)", d.cmd[0], d.cmd[1], d.cmd[2], d.cmd[3]); }
    { const int16_t cmd[8] = {100, 200, 300, 400, 500, 600, 700, 800}; Frame fr; Transfer t; Receiver<4, 256> rx(&sig_all, nullptr); RawCommand d; int frames = 0, got = 0;
      node.send_esc(cmd, 8);
      while (mod.receive(fr)) { frames++; if (rx.feed(fr, 300, t) && t.type_id == kRawCommandId) { got++; decode(t.data, t.len, d); } }
      CHECK(got == 1 && frames == 3 && d.n == 8 && d.cmd[7] == 800, "eight ESCs: 14 bytes + the transfer CRC = a three-frame transfer (%d frames), all eight back", frames); }
    fc_port.tx_room = 0; { const int16_t cmd[4] = {}; CHECK(!node.send_esc(cmd, 4) && node.esc_dropped() == 1, "a full transmit queue: the command is dropped and counted"); }
}

int main() {
    t_codec(); t_vectors(); t_vectors2(); t_decode(); t_receiver(); t_node(); t_node2();
    printf("\n%d passed, %d failed\n", g_pass, g_fail); return g_fail ? 1 : 0;
}
