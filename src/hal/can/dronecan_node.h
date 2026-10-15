/*
 * This file is part of Windburst Project
 */

#pragma once

#include "dronecan.h"
#include "dronecan_msgs.h"

// ============================================================================
// The flight controller as a DroneCAN node (hardware independent; the bus is handed in: twai_bus.h on the ESP32, a virtual
// bus in the tests):
//   - NodeStatus once a second (health OK / WARNING, mode OPERATIONAL), answers GetNodeInfo (name org.windburst.fc)
//   - dynamic node id allocation server (most DroneCAN GPS / compass modules ask for an id at power-up): first-come, the
//     preferred id when free, otherwise the highest free one from 125 down; the same module gets the same id again
//   - a table of the nodes on the bus (from their NodeStatus), with their names (GetNodeInfo, asked once per node)
//   - what the flight controller uses: GNSS fix (Fix2 + Auxiliary), compass (MagneticFieldStrength2), barometer (StaticPressure
//     + StaticTemperature) and battery (BatteryInfo), from the first node that sends them (a second module of the same kind is
//     listed, not used); the ESCs' esc.Status, per ESC index
//   - ESCs on the bus: esc.RawCommand, sent by the flight task (send_esc(); its own transfer id and counters, so it does not
//     race the sensor thread's update())
// Call update() often (the sensor thread does, every millisecond): it drains the receive queue and sends what is due.
// ============================================================================
namespace dcan {

struct Bus {
    virtual ~Bus() = default;
    virtual bool send(const Frame& f) = 0;          // false: the transmit queue is full (the frame is dropped and counted)
    virtual bool receive(Frame& f) = 0;             // false: nothing waiting
};

struct NodeEntry {
    uint8_t id = 0, health = 0, mode = 0; uint32_t uptime = 0, seen_ms = 0;
    bool info_asked = false, has_info = false; uint32_t asked_ms = 0; char name[24] = {};
};

class Node {
public:
    static constexpr int kMaxNodes = 8;
    static constexpr uint32_t kOfflineMs = 3000;

    struct Config { uint8_t node_id = 10; bool dna_server = true; uint8_t sw_major = 0, sw_minor = 0; uint8_t uid[16] = {}; const char* name = "org.windburst.fc"; };
    void begin(Bus* bus, const Config& c, uint32_t now_ms) {
        bus_ = bus; cfg_ = c; rx_.set_node_id(c.node_id); start_ms_ = now_ms; status_ms_ = now_ms - 1000u; n_nodes_ = 0; n_alloc_ = 0; pending_.active = false;
        gnss_src_ = mag_src_ = baro_src_ = batt_src_ = 0; gnss_fresh_ = mag_fresh_ = aux_valid_ = baro_fresh_ = batt_have_ = false;
        for (auto& e : esc_) e = EscSlot();
        esc_tx_ = esc_dropped_ = 0; tid_esc_ = 0;
    }

    void update(uint32_t now_ms) {
        if (!bus_) return;
        Frame f; Transfer t;
        for (int i = 0; i < 64 && bus_->receive(f); i++) { rx_frames_++; if (rx_.feed(f, now_ms, t)) on_transfer(t, now_ms); }
        if ((uint32_t)(now_ms - status_ms_) >= 1000u) { status_ms_ += 1000u; if ((uint32_t)(now_ms - status_ms_) >= 1000u) status_ms_ = now_ms; send_status(now_ms); }
        // names: one GetNodeInfo request per new node (again after 2 s without an answer, at most 3 times)
        for (int i = 0; i < n_nodes_; i++) {
            NodeEntry& e = nodes_[i];
            if (e.has_info || online(e, now_ms) == false) continue;
            if (!e.info_asked || ((uint32_t)(now_ms - e.asked_ms) >= 2000u && asks_[i] < 3)) { ask_info(e.id); e.info_asked = true; e.asked_ms = now_ms; asks_[i]++; break; }
        }
        if (pending_.active && (uint32_t)(now_ms - pending_.ms) > 1500u) pending_.active = false;      // an allocation that stopped half way
    }

    // ---- what the flight controller takes
    bool take_gnss(Fix2& f, GnssAux* aux, bool& aux_ok) { if (!gnss_fresh_) return false; gnss_fresh_ = false; f = fix_; aux_ok = aux_valid_; if (aux) *aux = aux_; return true; }
    bool take_mag(float ga[3]) { if (!mag_fresh_) return false; mag_fresh_ = false; for (int k = 0; k < 3; k++) ga[k] = mag_[k]; return true; }
    uint8_t gnss_node() const { return gnss_src_; }
    uint8_t mag_node() const { return mag_src_; }
    uint32_t gnss_ms() const { return gnss_ms_; }
    uint32_t mag_ms() const { return mag_ms_; }
    // barometer: a new pressure (Pa) and the module's last temperature (deg C; 15 until one came)
    bool take_baro(float& pa, float& temp_c) { if (!baro_fresh_) return false; baro_fresh_ = false; pa = baro_pa_; temp_c = baro_temp_c_; return true; }
    uint8_t baro_node() const { return baro_src_; }
    // battery: the last BatteryInfo while it is younger than 2 s (false: none, or the module went quiet)
    bool battery(Battery& b, uint32_t now_ms) const { if (!batt_have_ || stale(batt_ms_, now_ms)) return false; b = batt_; return true; }
    uint8_t battery_node() const { return batt_src_; }
    // ESC index i (= the RawCommand slot): a new esc.Status since the last call
    bool take_esc(int i, EscStatus& e) { if (i < 0 || i >= kMaxEsc || !esc_[i].fresh) return false; esc_[i].fresh = false; e = esc_[i].st; return true; }
    // a bit per ESC index heard from within the last second
    uint8_t esc_mask(uint32_t now_ms) const { uint8_t m = 0; for (int i = 0; i < kMaxEsc; i++) if (esc_[i].seen && (uint32_t)(now_ms - esc_[i].ms) < 1000u) m |= (uint8_t)(1u << i); return m; }
    // esc.RawCommand: one value per ESC index, -8191..8191 (0 = stop). The flight task's call; a single frame for up to 4 ESCs.
    bool send_esc(const int16_t* cmd, int n) {
        if (!bus_ || n <= 0) return false;
        RawCommand c; c.n = (uint8_t)(n > 20 ? 20 : n); for (int i = 0; i < c.n; i++) c.cmd[i] = cmd[i] > 8191 ? 8191 : cmd[i] < -8191 ? -8191 : cmd[i];
        uint8_t p[40]; const size_t len = encode(c, p, sizeof(p));
        Frame f[8]; const int k = encode_transfer(message_id(kPrioHigh, kRawCommandId, cfg_.node_id), kRawCommandSig, tid_esc_++, p, len, f, 8);
        for (int i = 0; i < k; i++) if (!bus_->send(f[i])) { esc_dropped_++; return false; }
        esc_tx_++; return true;
    }
    uint32_t esc_sent() const { return esc_tx_; }
    uint32_t esc_dropped() const { return esc_dropped_; }
    static constexpr int kMaxEsc = 8;

    // ---- the bus as the app sees it
    int nodes() const { return n_nodes_; }
    const NodeEntry& node(int i) const { return nodes_[i]; }
    bool online(const NodeEntry& e, uint32_t now_ms) const { return (uint32_t)(now_ms - e.seen_ms) < kOfflineMs; }
    uint32_t rx_frames() const { return rx_frames_; }
    uint32_t tx_frames() const { return tx_frames_; }
    uint32_t tx_dropped() const { return tx_dropped_; }
    uint32_t crc_errors() const { return rx_.crc_errors(); }
    uint32_t bad_frames() const { return rx_.bad_frames(); }
    uint32_t allocations() const { return allocations_; }
    uint8_t id() const { return cfg_.node_id; }
    void set_health(uint8_t h) { health_ = h; }

private:
    static uint64_t signature(Kind k, uint16_t id, void*) {
        if (k == MESSAGE) switch (id) {
            case kNodeStatusId: return kNodeStatusSig; case kFix2Id: return kFix2Sig; case kGnssAuxId: return kGnssAuxSig; case kMag2Id: return kMag2Sig; case kAllocationId: return kAllocationSig;
            case kPressureId: return kPressureSig; case kTemperatureId: return kTemperatureSig; case kBatteryId: return kBatterySig; case kEscStatusId: return kEscStatusSig;
            default: return 0; }
        return id == kGetNodeInfoId ? kGetNodeInfoSig : 0;
    }
    void on_transfer(const Transfer& t, uint32_t now_ms) {
        if (t.kind == MESSAGE && t.anonymous) { if (t.type_id == kAllocationId && cfg_.dna_server) on_allocation(t, now_ms); return; }
        if (t.kind == MESSAGE) {
            switch (t.type_id) {
            case kNodeStatusId: { Reader r(t.data, t.len); const NodeStatus s = get_node_status(r); if (!r.too_short()) seen(t.src, s, now_ms); break; }
            case kFix2Id: { Fix2 f; if ((!gnss_src_ || gnss_src_ == t.src || stale(gnss_ms_, now_ms)) && decode(t.data, t.len, f)) { fix_ = f; gnss_src_ = t.src; gnss_fresh_ = true; gnss_ms_ = now_ms; } break; }
            case kGnssAuxId: { GnssAux a; if (t.src == gnss_src_ && decode(t.data, t.len, a)) { aux_ = a; aux_valid_ = true; } break; }
            case kMag2Id: { Mag2 m; if ((!mag_src_ || mag_src_ == t.src || stale(mag_ms_, now_ms)) && decode(t.data, t.len, m)) { for (int k = 0; k < 3; k++) mag_[k] = m.field_ga[k]; mag_src_ = t.src; mag_fresh_ = true; mag_ms_ = now_ms; } break; }
            case kPressureId: { Pressure v; if ((!baro_src_ || baro_src_ == t.src || stale(baro_ms_, now_ms)) && decode(t.data, t.len, v) && v.pa > 1000.0f && v.pa < 120000.0f) { baro_pa_ = v.pa; baro_src_ = t.src; baro_fresh_ = true; baro_ms_ = now_ms; } break; }
            case kTemperatureId: { Temperature v; if (t.src == baro_src_ && decode(t.data, t.len, v) && v.kelvin > 150.0f && v.kelvin < 400.0f) baro_temp_c_ = v.kelvin - 273.15f; break; }
            case kBatteryId: { Battery b; if ((!batt_src_ || batt_src_ == t.src || stale(batt_ms_, now_ms)) && decode(t.data, t.len, b)) { batt_ = b; batt_src_ = t.src; batt_have_ = true; batt_ms_ = now_ms; } break; }
            case kEscStatusId: { EscStatus e; if (decode(t.data, t.len, e) && e.index < kMaxEsc) { EscSlot& x = esc_[e.index]; x.st = e; x.fresh = x.seen = true; x.ms = now_ms; x.node = t.src; } break; }
            default: break;
            }
            return;
        }
        if (t.type_id != kGetNodeInfoId) return;
        if (t.kind == REQUEST) {                                     // somebody (a configuration tool) asks who we are
            NodeInfo ni; ni.status = own_status(now_ms); ni.sw_major = cfg_.sw_major; ni.sw_minor = cfg_.sw_minor; memcpy(ni.uid, cfg_.uid, 16);
            size_t k = 0; for (; cfg_.name[k] && k < 80; k++) ni.name[k] = cfg_.name[k]; ni.name[k] = 0;
            uint8_t p[128]; const size_t n = encode(ni, p, sizeof(p));
            send(service_id(kPrioLow, kGetNodeInfoId, false, t.src, cfg_.node_id), kGetNodeInfoSig, t.tid, p, n);
        } else {                                                     // the answer to our own question: the node's name
            NodeInfo ni; if (!decode(t.data, t.len, ni)) return;
            for (int i = 0; i < n_nodes_; i++) if (nodes_[i].id == t.src) { memcpy(nodes_[i].name, ni.name, sizeof(nodes_[i].name) - 1); nodes_[i].name[sizeof(nodes_[i].name) - 1] = 0; nodes_[i].has_info = true; }
        }
    }
    static bool stale(uint32_t ms, uint32_t now_ms) { return (uint32_t)(now_ms - ms) > 2000u; }   // the module in use went quiet: the next one is taken
    void seen(uint8_t id, const NodeStatus& s, uint32_t now_ms) {
        int i = 0; while (i < n_nodes_ && nodes_[i].id != id) i++;
        if (i == n_nodes_) { if (n_nodes_ == kMaxNodes) { i = oldest(now_ms); if (i < 0) return; } else n_nodes_++; nodes_[i] = NodeEntry(); nodes_[i].id = id; asks_[i] = 0; }
        NodeEntry& e = nodes_[i];
        if (s.uptime_sec + 2 < e.uptime) { e.has_info = false; e.info_asked = false; asks_[i] = 0; }    // it restarted: maybe another firmware now
        e.health = s.health; e.mode = s.mode; e.uptime = s.uptime_sec; e.seen_ms = now_ms;
    }
    int oldest(uint32_t now_ms) const { for (int i = 0; i < n_nodes_; i++) if (!online(nodes_[i], now_ms)) return i; return -1; }
    NodeStatus own_status(uint32_t now_ms) const { NodeStatus s; s.uptime_sec = (now_ms - start_ms_) / 1000u; s.health = health_; s.mode = MODE_OPERATIONAL; return s; }
    void send_status(uint32_t now_ms) {
        uint8_t p[8]; const size_t n = encode(own_status(now_ms), p, sizeof(p));
        send(message_id(kPrioLow, kNodeStatusId, cfg_.node_id), kNodeStatusSig, tid_status_++, p, n);
    }
    void ask_info(uint8_t id) { send(service_id(kPrioLowest, kGetNodeInfoId, true, id, cfg_.node_id), kGetNodeInfoSig, tid_info_++, nullptr, 0); }
    void send(uint32_t can_id, uint64_t sig, uint8_t tid, const uint8_t* p, size_t n) {
        Frame f[24]; const int k = encode_transfer(can_id, sig, tid, p, n, f, 24);
        for (int i = 0; i < k; i++) { if (bus_->send(f[i])) tx_frames_++; else { tx_dropped_++; return; } }   // (the rest of a transfer is useless without this frame)
    }

    // ---- dynamic node id allocation (the server's half of uavcan.protocol.dynamic_node_id.Allocation)
    void on_allocation(const Transfer& t, uint32_t now_ms) {
        Allocation a; if (!decode(t.data, t.len, a) || !a.uid_len) return;
        if (a.first_part) { pending_.active = true; pending_.len = 0; pending_.preferred = a.node_id; }
        else if (!pending_.active) return;                           // a follow-up of a request we never saw (or gave up)
        if (pending_.len + a.uid_len > 16) { pending_.active = false; return; }
        memcpy(pending_.uid + pending_.len, a.uid, a.uid_len); pending_.len = (uint8_t)(pending_.len + a.uid_len); pending_.ms = now_ms;
        Allocation r; r.first_part = false; r.uid_len = pending_.len; memcpy(r.uid, pending_.uid, pending_.len); r.node_id = 0;
        if (pending_.len == 16) { r.node_id = pick_id(pending_.uid, pending_.preferred); pending_.active = false; if (!r.node_id) return; }
        uint8_t p[24]; const size_t n = encode(r, p, sizeof(p));
        send(message_id(kPrioLow, kAllocationId, cfg_.node_id), kAllocationSig, tid_alloc_++, p, n);
    }
    uint8_t pick_id(const uint8_t uid[16], uint8_t preferred) {
        for (int i = 0; i < n_alloc_; i++) if (!memcmp(alloc_[i].uid, uid, 16)) return alloc_[i].id;          // seen it before: the same id again
        auto taken = [&](uint8_t id) {
            if (id == 0 || id > 125 || id == cfg_.node_id) return true;
            for (int i = 0; i < n_alloc_; i++) if (alloc_[i].id == id) return true;
            for (int i = 0; i < n_nodes_; i++) if (nodes_[i].id == id) return true;
            return false;
        };
        uint8_t id = preferred && !taken(preferred) ? preferred : 0;
        for (int c = 125; !id && c > 0; c--) if (!taken((uint8_t)c)) id = (uint8_t)c;
        if (!id || n_alloc_ == kMaxAlloc) return 0;
        memcpy(alloc_[n_alloc_].uid, uid, 16); alloc_[n_alloc_].id = id; n_alloc_++; allocations_++;
        return id;
    }

    Bus* bus_ = nullptr; Config cfg_;
    Receiver<8, 256> rx_{&Node::signature, nullptr};
    uint32_t start_ms_ = 0, status_ms_ = 0, rx_frames_ = 0, tx_frames_ = 0, tx_dropped_ = 0, allocations_ = 0;
    uint8_t tid_status_ = 0, tid_info_ = 0, tid_alloc_ = 0, health_ = HEALTH_OK;
    NodeEntry nodes_[kMaxNodes]; uint8_t asks_[kMaxNodes] = {}; int n_nodes_ = 0;
    static constexpr int kMaxAlloc = 16;
    struct { uint8_t uid[16]; uint8_t id; } alloc_[kMaxAlloc] = {}; int n_alloc_ = 0;
    struct { bool active = false; uint8_t uid[16] = {}; uint8_t len = 0, preferred = 0; uint32_t ms = 0; } pending_;
    Fix2 fix_; GnssAux aux_; bool gnss_fresh_ = false, aux_valid_ = false; uint8_t gnss_src_ = 0; uint32_t gnss_ms_ = 0;
    float mag_[3] = {}; bool mag_fresh_ = false; uint8_t mag_src_ = 0; uint32_t mag_ms_ = 0;
    float baro_pa_ = 0, baro_temp_c_ = 15.0f; bool baro_fresh_ = false; uint8_t baro_src_ = 0; uint32_t baro_ms_ = 0;
    Battery batt_; bool batt_have_ = false; uint8_t batt_src_ = 0; uint32_t batt_ms_ = 0;
    struct EscSlot { EscStatus st; bool fresh = false, seen = false; uint32_t ms = 0; uint8_t node = 0; };
    EscSlot esc_[kMaxEsc];
    uint32_t esc_tx_ = 0, esc_dropped_ = 0; uint8_t tid_esc_ = 0;           // (the flight task's: send_esc())
};

} // namespace dcan
