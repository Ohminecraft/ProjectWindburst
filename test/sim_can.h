// A CAN bus on the virtual board: the flight controller's controller (SimCanPort, a CanPort) and a DroneCAN GPS + compass module
// (SimCanModule: asks for a node id, then sends NodeStatus, Fix2 at 10 Hz, Auxiliary at 2 Hz, the field at 50 Hz, answers
// GetNodeInfo) - what a Here3 / an M10Q-CAN does. Encoded with the firmware's own dronecan_msgs.h (checked against
// pydronecan in test_can).
#pragma once
#include <deque>
#include <string.h>
#include <math.h>
#include "../src/hal/can/can_port.h"
#include "../src/hal/can/dronecan_msgs.h"

struct SimCanWire {
    std::deque<dcan::Frame> to_fc, to_dev; uint32_t frames = 0;
    void (*tap)(const dcan::Frame&, void*) = nullptr; void* tap_ctx = nullptr;     // every frame the flight controller sends, also to the other devices (ESCs, ...)
};

class SimCanPort : public CanPort {
public:
    explicit SimCanPort(SimCanWire* w) : w_(w) {}
    bool start(uint8_t tx, uint8_t rx, uint32_t bps) override { tx_pin = tx; rx_pin = rx; bitrate = bps; if (refuse) { state_ = FAILED; return false; } state_ = RUNNING; return true; }
    const char* why() const override { return "the simulated controller refuses"; }
    bool send(const dcan::Frame& f) override { if (state_ != RUNNING || w_->to_dev.size() >= room) return false; w_->to_dev.push_back(f); w_->frames++; if (w_->tap) w_->tap(f, w_->tap_ctx); return true; }
    bool receive(dcan::Frame& f) override { if (state_ != RUNNING || w_->to_fc.empty()) return false; f = w_->to_fc.front(); w_->to_fc.pop_front(); return true; }
    uint8_t state() const override { return state_; }
    uint32_t bus_offs() const override { return bus_offs_; }
    void bus_off() { state_ = BUS_OFF; bus_offs_++; }           // (a test: the bus breaks down)
    void recover() { state_ = RUNNING; }
    uint8_t tx_pin = 255, rx_pin = 255; uint32_t bitrate = 0; bool refuse = false; size_t room = 64;
private:
    SimCanWire* w_; uint8_t state_ = OFF; uint32_t bus_offs_ = 0;
};

class SimCanModule {
public:
    explicit SimCanModule(SimCanWire* w) : w_(w) { for (int i = 0; i < 16; i++) uid[i] = (uint8_t)(0xA0 + i); fix.status = dcan::GNSS_3D; fix.sats_used = 15; fix.cov_len = 6; for (int k = 0; k < 6; k++) fix.covariance[k] = k < 3 ? 0.25f : 0.0025f; fix.pdop = 1.2f; }
    // what it measures (the virtual board fills these every millisecond)
    dcan::Fix2 fix; float field_frd[3] = {0.3f, 0.0f, 0.38f}; float hdop = 0.7f;
    bool gps = true, compass = true, silent = false; uint8_t want_id = 0; const char* name = "org.ardupilot.here3"; uint8_t uid[16];
    uint8_t id = 0;
    void step(uint32_t ms) {
        dcan::Frame f; dcan::Transfer t;
        while (!w_->to_dev.empty()) { f = w_->to_dev.front(); w_->to_dev.pop_front(); rx_.set_node_id(id); if (rx_.feed(f, ms, t)) handle(t, ms); }
        if (silent) return;
        if (!id) { if (ms - ask_ms_ >= 700u) { stage_ = 0; ask(0); ask_ms_ = ms; } return; }
        if (ms % 1000 == 0) { dcan::NodeStatus s; s.uptime_sec = (ms - up_ms_) / 1000; uint8_t p[8]; send(dcan::message_id(16, dcan::kNodeStatusId, id), dcan::kNodeStatusSig, tid_[0]++, p, dcan::encode(s, p, 8)); }
        if (gps && ms % 100 == 0) {
            fix.time_standard = 3; fix.gnss_timestamp_us = 1400000000000000ull + (uint64_t)ms * 1000ull; fix.timestamp_us = (uint64_t)ms * 1000ull;
            uint8_t p[120]; send(dcan::message_id(16, dcan::kFix2Id, id), dcan::kFix2Sig, tid_[1]++, p, dcan::encode(fix, p, sizeof(p)));
            if (ms % 500 == 0) { dcan::GnssAux a; a.hdop = hdop; a.pdop = fix.pdop; a.sats_used = fix.sats_used; send(dcan::message_id(20, dcan::kGnssAuxId, id), dcan::kGnssAuxSig, tid_[2]++, p, dcan::encode(a, p, sizeof(p))); }
        }
        if (compass && ms % 20 == 0) { dcan::Mag2 m; for (int k = 0; k < 3; k++) m.field_ga[k] = field_frd[k]; uint8_t p[16]; send(dcan::message_id(20, dcan::kMag2Id, id), dcan::kMag2Sig, tid_[3]++, p, dcan::encode(m, p, sizeof(p))); }
    }
private:
    static uint64_t sig(dcan::Kind k, uint16_t id, void*) { if (k == dcan::MESSAGE) return id == dcan::kAllocationId ? dcan::kAllocationSig : 0; return id == dcan::kGetNodeInfoId ? dcan::kGetNodeInfoSig : 0; }
    void send(uint32_t can_id, uint64_t s, uint8_t tid, const uint8_t* p, size_t n) { dcan::Frame f[24]; const int k = dcan::encode_transfer(can_id, s, tid, p, n, f, 24); for (int i = 0; i < k; i++) { w_->to_fc.push_back(f[i]); w_->frames++; } }
    void ask(int part) {
        dcan::Allocation a; a.node_id = want_id; a.first_part = part == 0; const int off = part * 6, len = part == 2 ? 4 : 6; a.uid_len = (uint8_t)len; memcpy(a.uid, uid + off, len);
        uint8_t p[16]; send(dcan::anonymous_id(30, (uint16_t)(0x155 + part), dcan::kAllocationId), dcan::kAllocationSig, 0, p, dcan::encode(a, p, sizeof(p)));
    }
    void handle(const dcan::Transfer& t, uint32_t ms) {
        if (!id && t.kind == dcan::MESSAGE && t.type_id == dcan::kAllocationId && !t.anonymous) {
            dcan::Allocation a; if (!dcan::decode(t.data, t.len, a)) return;
            const int have = stage_ == 0 ? 6 : stage_ == 1 ? 12 : 16;
            if (a.uid_len != have || memcmp(a.uid, uid, have)) return;
            if (have == 16) { if (a.node_id) { id = a.node_id; up_ms_ = ms; } return; }
            stage_++; ask(stage_); ask_ms_ = ms; return;
        }
        if (id && t.kind == dcan::REQUEST && t.type_id == dcan::kGetNodeInfoId) {
            dcan::NodeInfo ni; strncpy(ni.name, name, 80); memcpy(ni.uid, uid, 16); uint8_t p[128];
            send(dcan::service_id(24, dcan::kGetNodeInfoId, false, t.src, id), dcan::kGetNodeInfoSig, t.tid, p, dcan::encode(ni, p, sizeof(p)));
        }
    }
    SimCanWire* w_; dcan::Receiver<4, 256> rx_{&SimCanModule::sig, nullptr};
    int stage_ = 0; uint32_t ask_ms_ = 0, up_ms_ = 0; uint8_t tid_[4] = {};
};


// More devices on the same bus, all with fixed node ids:
//   SimCanEscs     - four ESCs (nodes 61..64, ESC index 0..3): take esc.RawCommand, report esc.Status at 50 Hz; an ESC that hears
//                    no command for 250 ms stops its motor (what real DroneCAN ESCs do)
//   SimCanSensors  - a barometer + power module (node 50): StaticPressure 50 Hz, StaticTemperature 10 Hz, BatteryInfo 10 Hz
struct SimCanEscs {
    explicit SimCanEscs(SimCanWire* w) : w_(w) {}
    int16_t cmd[8] = {}; uint32_t cmd_ms = 0, commands = 0; bool silent = false;
    float rpm[8] = {}, amps[8] = {}, volt = 16.0f, temp_c = 45.0f;            // what each ESC reports (the board fills it)
    uint32_t status_ms = 20;                                                   // esc.Status period (real ESCs: 10 .. 50 Hz)
    float frac(int slot, uint32_t now_ms) const { if (now_ms - cmd_ms > 250u || slot < 0 || slot >= 8 || cmd[slot] <= 0) return 0.0f; return cmd[slot] / 8191.0f; }
    static void on_frame(const dcan::Frame& f, void* ctx) {
        SimCanEscs& e = *static_cast<SimCanEscs*>(ctx); dcan::Transfer t;
        if (e.rx_.feed(f, e.now_, t) && t.type_id == dcan::kRawCommandId) { dcan::RawCommand c; dcan::decode(t.data, t.len, c); for (int i = 0; i < 8; i++) e.cmd[i] = i < c.n ? c.cmd[i] : 0; e.cmd_ms = e.now_; e.commands++; }
    }
    void step(uint32_t ms) {
        now_ = ms;
        if (silent) return;
        for (int i = 0; i < 4; i++) {
            const uint8_t id = (uint8_t)(61 + i);
            if (ms % 1000 == (uint32_t)i) { dcan::NodeStatus s; s.uptime_sec = ms / 1000; uint8_t p[8]; send(dcan::message_id(16, dcan::kNodeStatusId, id), dcan::kNodeStatusSig, tid_[i][0]++, p, dcan::encode(s, p, 8)); }
            if (ms % status_ms == (uint32_t)i) { dcan::EscStatus st; st.index = (uint8_t)i; st.rpm = (int32_t)lroundf(rpm[i]); st.voltage = volt; st.current = amps[i]; st.temperature_k = temp_c + 273.15f; st.power_pct = (uint8_t)(frac(i, ms) * 100.0f);
                uint8_t p[16]; send(dcan::message_id(16, dcan::kEscStatusId, id), dcan::kEscStatusSig, tid_[i][1]++, p, dcan::encode(st, p, sizeof(p))); }
        }
    }
private:
    static uint64_t sig(dcan::Kind k, uint16_t id, void*) { return k == dcan::MESSAGE && id == dcan::kRawCommandId ? dcan::kRawCommandSig : 0; }
    void send(uint32_t can_id, uint64_t s, uint8_t tid, const uint8_t* p, size_t n) { dcan::Frame f[8]; const int k = dcan::encode_transfer(can_id, s, tid, p, n, f, 8); for (int i = 0; i < k; i++) { w_->to_fc.push_back(f[i]); w_->frames++; } }
    SimCanWire* w_; dcan::Receiver<2, 64> rx_{&SimCanEscs::sig, nullptr}; uint32_t now_ = 0; uint8_t tid_[4][2] = {};
};

struct SimCanSensors {
    explicit SimCanSensors(SimCanWire* w) : w_(w) {}
    double pa = 101325.0, temp_c = 25.0; float volt = 16.0f, amps = 0.0f; bool baro = true, power = true, silent = false;
    void step(uint32_t ms) {
        if (silent) return;
        const uint8_t id = 50; uint8_t p[64];
        if (ms % 1000 == 7) { dcan::NodeStatus s; s.uptime_sec = ms / 1000; send(dcan::message_id(16, dcan::kNodeStatusId, id), dcan::kNodeStatusSig, tid_[0]++, p, dcan::encode(s, p, 8)); }
        if (baro && ms % 20 == 3) { dcan::Pressure v; v.pa = (float)pa; v.variance = 4.0f; send(dcan::message_id(16, dcan::kPressureId, id), dcan::kPressureSig, tid_[1]++, p, dcan::encode(v, p, sizeof(p))); }
        if (baro && ms % 100 == 5) { dcan::Temperature v; v.kelvin = (float)(temp_c + 273.15); send(dcan::message_id(20, dcan::kTemperatureId, id), dcan::kTemperatureSig, tid_[2]++, p, dcan::encode(v, p, sizeof(p))); }
        if (power && ms % 100 == 9) { dcan::Battery b; b.voltage = volt; b.current = amps; b.soc_pct = 127; strcpy(b.model, "sim PM");
            send(dcan::message_id(20, dcan::kBatteryId, id), dcan::kBatterySig, tid_[3]++, p, dcan::encode(b, p, sizeof(p))); }
    }
private:
    void send(uint32_t can_id, uint64_t s, uint8_t tid, const uint8_t* p, size_t n) { dcan::Frame f[24]; const int k = dcan::encode_transfer(can_id, s, tid, p, n, f, 24); for (int i = 0; i < k; i++) { w_->to_fc.push_back(f[i]); w_->frames++; } }
    SimCanWire* w_; uint8_t tid_[4] = {};
};
