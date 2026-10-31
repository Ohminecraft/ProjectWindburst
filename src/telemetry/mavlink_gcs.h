/*
 * This file is part of Windburst Project
 */

#pragma once

#include <math.h>
#include "mavlink_telemetry.h"

// ============================================================================
// What a ground station (Mission Planner, QGroundControl) sends, and the answers (0.40). Portable: the craft is behind `Vehicle`
// (FlightApp; a fake in test/test_gcs.cpp), the answers go out through the Scheduler's reply queue. Read from Betaflight's
// telemetry/mavlink.c and telemetry/mavlink_mission.c, written anew.
//
//   COMMAND_LONG / COMMAND_INT  COMPONENT_ARM_DISARM (arming: only with telemetry.remote_arm; disarming in the air only with param2 =
//                               21196, ArduPilot's "force" - Betaflight disarms always), DO_SET_MODE (ArduCopter numbers, as HEARTBEAT
//                               says them), NAV_RETURN_TO_LAUNCH, DO_SET_HOME (here only: home follows the arming), PREFLIGHT_REBOOT_SHUTDOWN
//                               (reboot), MISSION_START, REQUEST_MESSAGE (AUTOPILOT_VERSION, HOME_POSITION, GPS_GLOBAL_ORIGIN,
//                               AVAILABLE_MODES, AVAILABLE_MODES_MONITOR), REQUEST_AUTOPILOT_CAPABILITIES. Anything else: UNSUPPORTED.
//   the mission protocol        MISSION_COUNT -> MISSION_REQUEST_INT -> MISSION_ITEM_INT ... -> MISSION_ACK (upload: the old mission
//                               stays until the last item is in; asked again every 1.5 s, given up after 5 times); MISSION_REQUEST_LIST
//                               -> MISSION_COUNT -> MISSION_REQUEST_INT / MISSION_ITEM_INT (download); MISSION_CLEAR_ALL;
//                               MISSION_SET_CURRENT (only the command it is at: the mission always starts at its first command);
//                               MISSION_ITEM_REACHED as the mission moves on (MISSION_CURRENT: the Scheduler, 1 Hz). Item 0 is the first
//                               command (PX4 / Betaflight), not ArduPilot's home. Fence and rally: none (an empty list).
//   TIMESYNC                    a request (tc1 = 0) is answered with this craft's time.
// A message for another system (target_system not 0 / ours) or another component (not 0 / 1) is ignored.
// The commands a mission holds, both ways (the section `mission`, StoreItem): NAV_WAYPOINT (hold s), NAV_LOITER_TIME (s), NAV_RETURN_TO_LAUNCH,
// NAV_LAND, NAV_TAKEOFF, NAV_DELAY (s), DO_SET_SERVO (servo 1..4, us), DO_CHANGE_SPEED (m/s); heights above home (frame
// GLOBAL_RELATIVE_ALT[_INT]; GLOBAL[_INT] = above sea level, turned into above home when there is a home point).
// Mission Planner (0.40.1, read from its FlightPlanner.cs / mav_mission.cs): it sends the float forms MISSION_ITEM / MISSION_REQUEST
// when it has not seen MISSION_INT in AUTOPILOT_VERSION - both taken and answered in kind; and it drops item 0 of every download as
// "home" (but sends none up to a craft that is not ArduPilot) - with home_item (telemetry.mp_home_item) a download starts with the home
// point as item 0, as ArduPilot's does, and the caller counts it in MISSION_CURRENT / MISSION_ITEM_REACHED (State::mission_seq).
// ============================================================================
namespace mav {

// one command of the mission as the section `mission` keeps it (MAV_CMD numbers here; FlightApp converts)
struct StoreItem { uint16_t cmd = 0; uint8_t arg = 0; int16_t alt_dm = 0; int32_t lat = 0, lon = 0; uint16_t p = 0; };
constexpr int kStoreMax = 32;

// What the ground station may ask of the craft. A command answers a MAV_RESULT, or RESULT_IN_PROGRESS when the answer comes later
// (Gcs::command_done: arming waits for the flight loop).
class Vehicle {
public:
    virtual ~Vehicle() = default;
    virtual uint8_t arm(bool on, bool force) = 0;
    virtual uint8_t set_mode(uint32_t custom_mode) = 0;          // ArduCopter numbers (COPTER_*)
    virtual uint8_t set_home_here() = 0;
    virtual uint8_t reboot() = 0;
    virtual uint8_t mission_start() = 0;
    virtual int mission_count() const = 0;
    virtual StoreItem mission_item(int i) const = 0;
    virtual bool mission_busy() const = 0;                       // the mission flies: it is not replaced
    virtual bool home_alt_mm(int32_t& alt_mm) const = 0;         // false: no home point yet
    virtual bool mission_replace(const StoreItem* items, int n) = 0;   // false: refused
};

// The modes a ground station may show and choose (AVAILABLE_MODES): ArduCopter's numbers, as HEARTBEAT.custom_mode carries them (the
// ELRS transmitter names the modes on the handset by them too). Smart RTL and Land come from the rescue setting / the failsafe.
struct ModeInfo { uint32_t custom; uint8_t standard; uint32_t properties; const char* name; };
inline const ModeInfo* available_modes(int& n) {
    static const ModeInfo m[] = {
        {COPTER_ACRO, STANDARD_MODE_NON_STANDARD, 0, "Acro"}, {COPTER_STABILIZE, STANDARD_MODE_NON_STANDARD, 0, "Angle"},
        {COPTER_ALT_HOLD, STANDARD_MODE_ALTITUDE_HOLD, 0, "Alt Hold"}, {COPTER_POSHOLD, STANDARD_MODE_NON_STANDARD, 0, "Pos Hold"},
        {COPTER_AUTO, STANDARD_MODE_MISSION, 0, "Mission"}, {COPTER_RTL, STANDARD_MODE_SAFE_RECOVERY, 0, "RTL"},
        {COPTER_SMART_RTL, STANDARD_MODE_SAFE_RECOVERY, MODE_PROPERTY_NOT_USER_SELECTABLE, "Smart RTL"}, {COPTER_LAND, STANDARD_MODE_LAND, MODE_PROPERTY_NOT_USER_SELECTABLE, "Land"},
        {COPTER_LOITER, STANDARD_MODE_POSITION_HOLD, 0, "Loiter"},      // (0.41, at the end: the indexes before stay; ArduCopter's POSITION_HOLD is Loiter)
    };
    n = (int)(sizeof(m) / sizeof(m[0])); return m;
}

// a float parameter as a whole number 0..max (a NaN, an infinity, a negative or too big number: false - casting those is undefined)
inline bool param_uint(float f, uint32_t max, uint32_t& out) {
    if (!(f == f) || f < 0.0f || (double)f > (double)max) return false;
    out = (uint32_t)f; return true;
}
inline bool positional(uint16_t c) { return c == CMD_NAV_WAYPOINT || c == CMD_NAV_LOITER_TIME || c == CMD_NAV_LAND || c == CMD_NAV_TAKEOFF; }

// A MISSION_ITEM_INT as the section keeps it. false + result (MAV_MISSION_RESULT) when it cannot be flown.
inline bool store_item_of(const MissionItemInt& it, bool have_home, int32_t home_alt_mm, StoreItem& o, uint8_t& result) {
    o = StoreItem(); o.cmd = it.command;
    const bool yaw_nan_ok = it.command == CMD_NAV_WAYPOINT && it.param[3] != it.param[3];       // (NAV_WAYPOINT: NaN yaw = keep the heading)
    if (!isfinite(it.param[0]) || !isfinite(it.param[1]) || !isfinite(it.param[2]) || (!isfinite(it.param[3]) && !yaw_nan_ok)) { result = MISSION_INVALID; return false; }
    auto seconds = [&](float s, uint16_t& out) { if (s < 0.0f) { result = MISSION_INVALID_PARAM1; return false; } out = (uint16_t)(s >= 65535.0f ? 65535u : (uint32_t)(s + 0.5f)); return true; };
    if (positional(it.command)) {
        float alt_m;
        if (!isfinite(it.z)) { result = MISSION_INVALID_PARAM7; return false; }
        if (it.frame == FRAME_GLOBAL_RELATIVE_ALT || it.frame == FRAME_GLOBAL_RELATIVE_ALT_INT) alt_m = it.z;
        else if (it.frame == FRAME_GLOBAL || it.frame == FRAME_GLOBAL_INT) { if (!have_home) { result = MISSION_INVALID; return false; } alt_m = it.z - (float)home_alt_mm * 0.001f; }
        else { result = MISSION_UNSUPPORTED_FRAME; return false; }
        const float dm = alt_m * 10.0f;
        if (!(dm >= -32767.0f && dm <= 32767.0f)) { result = MISSION_INVALID_PARAM7; return false; }
        if (it.x < -900000000 || it.x > 900000000) { result = MISSION_INVALID_PARAM5_X; return false; }
        if (it.y < -1800000000 || it.y > 1800000000) { result = MISSION_INVALID_PARAM6_Y; return false; }
        o.alt_dm = (int16_t)(dm < 0 ? dm - 0.5f : dm + 0.5f); o.lat = it.x; o.lon = it.y;
    }
    switch (it.command) {
    case CMD_NAV_TAKEOFF: o.lat = o.lon = 0; return true;                            // (straight up where the craft stands)
    case CMD_NAV_LAND: o.alt_dm = 0; return true;                                    // (from the height it has)
    case CMD_NAV_WAYPOINT: case CMD_NAV_LOITER_TIME: return seconds(it.param[0], o.p);
    case CMD_NAV_RETURN_TO_LAUNCH: return true;
    case CMD_NAV_DELAY: return seconds(it.param[0], o.p);                            // (-1 = a time of day: not supported)
    case CMD_DO_SET_SERVO: {
        uint32_t servo; if (!param_uint(it.param[0], 4, servo) || servo < 1 || (float)servo != it.param[0]) { result = MISSION_INVALID_PARAM1; return false; }
        if (!(it.param[1] >= 500.0f && it.param[1] <= 2500.0f)) { result = MISSION_INVALID_PARAM2; return false; }
        o.arg = (uint8_t)servo; o.p = (uint16_t)(it.param[1] + 0.5f); return true; }
    case CMD_DO_CHANGE_SPEED:
        if (!(it.param[1] > 0.0f && it.param[1] <= 6553.0f)) { result = MISSION_INVALID_PARAM2; return false; }
        o.p = (uint16_t)(it.param[1] * 10.0f + 0.5f); return true;                     // (dm/s)
    default: result = MISSION_UNSUPPORTED; return false;
    }
}
// ... and back (download)
inline MissionItemInt mav_item_of(const StoreItem& s, uint16_t seq) {
    MissionItemInt it; it.seq = seq; it.command = s.cmd; it.autocontinue = 1;
    const bool doc = s.cmd == CMD_DO_SET_SERVO || s.cmd == CMD_DO_CHANGE_SPEED;
    it.frame = doc ? FRAME_MISSION : FRAME_GLOBAL_RELATIVE_ALT_INT;
    if (positional(s.cmd)) { it.x = s.lat; it.y = s.lon; it.z = (float)s.alt_dm * 0.1f; }
    if (s.cmd == CMD_NAV_WAYPOINT || s.cmd == CMD_NAV_LOITER_TIME || s.cmd == CMD_NAV_DELAY) it.param[0] = (float)s.p;
    if (s.cmd == CMD_DO_SET_SERVO) { it.param[0] = (float)s.arg; it.param[1] = (float)s.p; }
    if (s.cmd == CMD_DO_CHANGE_SPEED) { it.param[0] = 1.0f; it.param[1] = (float)s.p * 0.1f; }      // (speed type 1 = ground speed)
    return it;
}

class Gcs {
public:
    static constexpr uint32_t kRetryMs = 1500;          // an upload item not come: asked again (Betaflight MISSION_UPLOAD_RETRY_MS)
    static constexpr int kMaxRetry = 5;
    static constexpr uint32_t kIdleMs = 5000;           // a download nobody continues: over
    static constexpr float kForceDisarm = 21196.0f;     // ArduPilot: COMPONENT_ARM_DISARM param2 that disarms in the air

    void configure(uint8_t sysid, uint32_t flight_sw_version, bool home_item = false) {
        sysid_ = sysid ? sysid : 1; version_ = flight_sw_version; home_item_ = home_item; x_ = Xfer(); pending_ = false; track_ = false;
    }

    // one frame from the ground station (or from anything else on the wire: what is not for us is dropped)
    void handle(const Parser::Frame& f, Vehicle& v, Scheduler& out, const State& s, uint32_t now_ms, uint64_t now_us) {
        CommandLong cl; CommandInt ci; MissionRequestList rl; MissionRequestInt ri; MissionCount mc; MissionItemInt it; MissionAck ma; MissionClearAll ca; MissionSetCurrent sc; Timesync ts;
        MissionItem fi; MissionRequest fr;
        if (unpack(f, cl)) { if (to_us(cl.target_system, cl.target_component)) command(f, cl.command, cl.param, v, out, s); }
        else if (unpack(f, ci)) { if (to_us(ci.target_system, ci.target_component)) { float p[7] = {ci.param[0], ci.param[1], ci.param[2], ci.param[3], 0, 0, ci.z}; command(f, ci.command, p, v, out, s); } }
        else if (unpack(f, rl)) { if (to_us(rl.target_system, rl.target_component)) request_list(f, rl.mission_type, v, out, now_ms); }
        else if (unpack(f, ri)) { if (to_us(ri.target_system, ri.target_component)) request_item(f, ri.seq, ri.mission_type, false, v, out, s, now_ms); }
        else if (unpack(f, fr)) { if (to_us(fr.target_system, fr.target_component)) request_item(f, fr.seq, fr.mission_type, true, v, out, s, now_ms); }
        else if (unpack(f, fi)) { if (to_us(fi.target_system, fi.target_component)) item(f, int_of(fi), v, out, now_ms); }
        else if (unpack(f, mc)) { if (to_us(mc.target_system, mc.target_component)) count(f, mc, v, out, now_ms); }
        else if (unpack(f, it)) { if (to_us(it.target_system, it.target_component)) item(f, it, v, out, now_ms); }
        else if (unpack(f, ma)) { if (to_us(ma.target_system, ma.target_component) && x_.state == SENDING && partner(f)) x_.state = IDLE; }
        else if (unpack(f, ca)) { if (to_us(ca.target_system, ca.target_component)) clear_all(f, ca.mission_type, v, out); }
        else if (unpack(f, sc)) { if (to_us(sc.target_system, sc.target_component)) set_current(f, sc.seq, out, s); }
        else if (unpack(f, ts)) { if (ts.tc1 == 0) { Timesync r; r.tc1 = (int64_t)now_us * 1000; r.ts1 = ts.ts1; out.reply(r); } }
    }

    // every pass: the upload's retries, an abandoned download, MISSION_ITEM_REACHED
    void update(Vehicle&, Scheduler& out, const State& s, uint32_t now_ms) {
        if (x_.state == RECEIVING && (uint32_t)(now_ms - x_.last_ms) >= kRetryMs) {
            if (x_.retries >= kMaxRetry) mission_ack(x_.sys, x_.comp, x_.type, MISSION_OPERATION_CANCELLED, out);
            else { x_.retries++; x_.last_ms = now_ms; request(x_.next, out); }
        }
        if (x_.state == SENDING && (uint32_t)(now_ms - x_.last_ms) >= kIdleMs) x_.state = IDLE;
        if (s.mission_state == MISSION_STATE_ACTIVE) {
            if (track_ && s.mission_seq != reached_) { MissionItemReached r; r.seq = reached_; out.reply(r); }
            track_ = true; reached_ = s.mission_seq;
        } else if (track_) {
            if (s.mission_state == MISSION_STATE_COMPLETE) { MissionItemReached r; r.seq = reached_; out.reply(r); }
            track_ = false;
        }
    }

    // the answer to the command that was IN_PROGRESS
    void command_done(uint8_t result, Scheduler& out) { if (!pending_) return; pending_ = false; ack(pending_cmd_, result, pending_sys_, pending_comp_, out); }
    bool pending() const { return pending_; }

private:
    enum XState : uint8_t { IDLE, RECEIVING, SENDING };
    struct Xfer { XState state = IDLE; uint8_t type = 0, sys = 0, comp = 0, retries = 0; uint16_t next = 0, total = 0; uint32_t last_ms = 0; };

    bool to_us(uint8_t ts, uint8_t tc) const { return (ts == 0 || ts == sysid_) && (tc == 0 || tc == 1); }
    bool partner(const Parser::Frame& f) const { return f.sysid == x_.sys && f.compid == x_.comp; }
    void ack(uint16_t cmd, uint8_t result, uint8_t sys, uint8_t comp, Scheduler& out) { CommandAck a; a.command = cmd; a.result = result; a.target_system = sys; a.target_component = comp; out.reply(a); }
    void mission_ack(uint8_t sys, uint8_t comp, uint8_t type, uint8_t result, Scheduler& out) {
        MissionAck a; a.target_system = sys; a.target_component = comp; a.type = result; a.mission_type = type; out.reply(a);
        x_.state = IDLE;
    }
    void request(uint16_t seq, Scheduler& out) { MissionRequestInt r; r.target_system = x_.sys; r.target_component = x_.comp; r.seq = seq; r.mission_type = x_.type; out.reply(r); }
    void mission_current(const State& s, Scheduler& out) {
        MissionCurrent c; c.seq = s.mission_seq; c.total = s.mission_total; c.mission_state = s.mission_state; c.mission_mode = s.mission_mode; out.reply(c);
    }

    void command(const Parser::Frame& f, uint16_t cmd, const float* p, Vehicle& v, Scheduler& out, const State& s) {
        uint32_t u = 0, u2 = 0; uint8_t r = RESULT_UNSUPPORTED;
        switch (cmd) {
        case CMD_COMPONENT_ARM_DISARM:
            if (pending_) r = RESULT_TEMPORARILY_REJECTED;
            else if (!param_uint(p[0], 1, u)) r = RESULT_DENIED;
            else r = v.arm(u == 1, p[1] == kForceDisarm);
            break;
        case CMD_DO_SET_MODE:
            r = !param_uint(p[0], 255, u) || !param_uint(p[1], 0xFFFFFF, u2) || !(u & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED) ? RESULT_DENIED : v.set_mode(u2); break;
        case CMD_NAV_RETURN_TO_LAUNCH: r = v.set_mode(COPTER_RTL); break;
        case CMD_DO_SET_HOME: r = !param_uint(p[0], 1, u) ? RESULT_DENIED : u == 1 ? v.set_home_here() : RESULT_UNSUPPORTED; break;
        case CMD_PREFLIGHT_REBOOT_SHUTDOWN: r = !param_uint(p[0], 3, u) ? RESULT_DENIED : u == 0 ? RESULT_ACCEPTED : u == 1 ? v.reboot() : RESULT_UNSUPPORTED; break;
        case CMD_MISSION_START: r = v.mission_start(); break;
        case CMD_REQUEST_AUTOPILOT_CAPABILITIES:
            if (!param_uint(p[0], 1, u)) r = RESULT_DENIED; else { if (u == 1) version(out); r = RESULT_ACCEPTED; }
            break;
        case CMD_REQUEST_MESSAGE: r = !param_uint(p[0], 0xFFFFFF, u) ? RESULT_DENIED : request_message(u, p[1], out, s); break;
        default: break;
        }
        if (r == RESULT_IN_PROGRESS) { pending_ = true; pending_cmd_ = cmd; pending_sys_ = f.sysid; pending_comp_ = f.compid; return; }
        ack(cmd, r, f.sysid, f.compid, out);
    }
    void version(Scheduler& out) {
        AutopilotVersion a; a.capabilities = CAP_MAVLINK2 | CAP_MISSION_INT | CAP_COMMAND_INT; a.flight_sw_version = version_; out.reply(a);
    }
    uint8_t request_message(uint32_t id, float p2, Scheduler& out, const State& s) {
        switch (id) {
        case MSG_AUTOPILOT_VERSION: version(out); return RESULT_ACCEPTED;
        case MSG_HOME_POSITION: case MSG_GPS_GLOBAL_ORIGIN:
            if (!s.have_home) return RESULT_FAILED;
            if (id == MSG_HOME_POSITION) { HomePosition h; h.latitude = s.home_lat; h.longitude = s.home_lon; h.altitude = s.home_alt_mm; out.reply(h); }
            else { GpsGlobalOrigin g; g.latitude = s.home_lat; g.longitude = s.home_lon; g.altitude = s.home_alt_mm; out.reply(g); }
            return RESULT_ACCEPTED;
        case MSG_AVAILABLE_MODES: {
            int n; const ModeInfo* m = available_modes(n); uint32_t i;
            if (!param_uint(p2, (uint32_t)n, i)) return RESULT_DENIED;                    // (0 = all of them, 1..n = that one)
            for (int k = 0; k < n; k++) {
                if (i && (uint32_t)k + 1 != i) continue;
                AvailableModes a; a.number_modes = (uint8_t)n; a.mode_index = (uint8_t)(k + 1); a.standard_mode = m[k].standard; a.custom_mode = m[k].custom; a.properties = m[k].properties;
                strncpy(a.mode_name, m[k].name, sizeof(a.mode_name) - 1); out.reply(a);
            }
            return RESULT_ACCEPTED; }
        case MSG_AVAILABLE_MODES_MONITOR: { AvailableModesMonitor a; a.seq = 1; out.reply(a); return RESULT_ACCEPTED; }      // (the set never changes)
        default: return RESULT_UNSUPPORTED;
        }
    }

    // ---- the mission protocol (Betaflight mavlink_mission.c)
    void request_list(const Parser::Frame& f, uint8_t type, Vehicle& v, Scheduler& out, uint32_t now_ms) {
        if (type != MISSION_TYPE_MISSION && type != MISSION_TYPE_FENCE && type != MISSION_TYPE_RALLY) { MissionAck a; a.target_system = f.sysid; a.target_component = f.compid; a.type = MISSION_UNSUPPORTED; a.mission_type = type; out.reply(a); return; }
        int n = type == MISSION_TYPE_MISSION ? v.mission_count() : 0;              // (no fence, no rally points: an empty list)
        if (n > 0 && type == MISSION_TYPE_MISSION && home_item_) n++;             // (Mission Planner: the home item first)
        MissionCount c; c.target_system = f.sysid; c.target_component = f.compid; c.count = (uint16_t)n; c.mission_type = type; out.reply(c);
        if (n == 0) return;
        x_ = Xfer(); x_.state = SENDING; x_.type = type; x_.sys = f.sysid; x_.comp = f.compid; x_.total = (uint16_t)n; x_.last_ms = now_ms;
    }
    // one item of a download; `legacy`: asked by MISSION_REQUEST, answered with MISSION_ITEM (degrees)
    void request_item(const Parser::Frame& f, uint16_t seq, uint8_t type, bool legacy, Vehicle& v, Scheduler& out, const State& s, uint32_t now_ms) {
        if (x_.state != SENDING || !partner(f)) return;
        if (type != x_.type) { mission_ack(f.sysid, f.compid, type, MISSION_UNSUPPORTED, out); return; }
        // the next one, or the one before again (its reply was lost); anything else is out of sequence
        const uint16_t h = home_offset();
        if ((seq != x_.next && !(x_.next > 0 && seq == x_.next - 1)) || seq >= x_.total || (int)seq - h >= v.mission_count()) { mission_ack(f.sysid, f.compid, x_.type, MISSION_INVALID_SEQUENCE, out); return; }
        MissionItemInt it;
        if (seq < h) {                                          // the home item (ArduPilot's: above sea level; 0 everywhere without a home)
            it.seq = seq; it.command = CMD_NAV_WAYPOINT; it.frame = FRAME_GLOBAL_INT;
            if (s.have_home) { it.x = s.home_lat; it.y = s.home_lon; it.z = (float)s.home_alt_mm * 0.001f; }
        } else it = mav_item_of(v.mission_item(seq - h), seq);
        it.target_system = f.sysid; it.target_component = f.compid; it.mission_type = x_.type;
        it.current = s.mission_state == MISSION_STATE_ACTIVE && s.mission_seq == seq && seq >= h ? 1 : 0;
        if (legacy) out.reply(float_of(it)); else out.reply(it);
        if (seq == x_.next) x_.next++;
        x_.last_ms = now_ms;
    }
    uint16_t home_offset() const { return home_item_ && x_.type == MISSION_TYPE_MISSION ? 1 : 0; }
    // the float form and back: degrees in the global frames, everything else as it is
    static bool in_degrees(uint8_t frame) { return frame == FRAME_GLOBAL || frame == FRAME_GLOBAL_RELATIVE_ALT || frame == FRAME_GLOBAL_INT || frame == FRAME_GLOBAL_RELATIVE_ALT_INT; }
    static int32_t e7(float deg) { return deg == deg && deg > -214.0f && deg < 214.0f ? (int32_t)llround((double)deg * 1e7) : INT32_MIN; }   // (out of range: refused as such)
    static MissionItemInt int_of(const MissionItem& m) {
        MissionItemInt it; for (int k = 0; k < 4; k++) it.param[k] = m.param[k];
        it.x = in_degrees(m.frame) ? e7(m.x) : (int32_t)0; it.y = in_degrees(m.frame) ? e7(m.y) : (int32_t)0; it.z = m.z;
        it.seq = m.seq; it.command = m.command; it.target_system = m.target_system; it.target_component = m.target_component;
        it.frame = m.frame == FRAME_GLOBAL_INT ? (uint8_t)FRAME_GLOBAL : m.frame == FRAME_GLOBAL_RELATIVE_ALT_INT ? (uint8_t)FRAME_GLOBAL_RELATIVE_ALT : m.frame;
        it.current = m.current; it.autocontinue = m.autocontinue; it.mission_type = m.mission_type;
        return it;
    }
    static MissionItem float_of(const MissionItemInt& i) {
        MissionItem m; for (int k = 0; k < 4; k++) m.param[k] = i.param[k];
        const bool deg = in_degrees(i.frame); m.x = deg ? (float)((double)i.x * 1e-7) : (float)i.x; m.y = deg ? (float)((double)i.y * 1e-7) : (float)i.y; m.z = i.z;
        m.seq = i.seq; m.command = i.command; m.target_system = i.target_system; m.target_component = i.target_component;
        m.frame = i.frame == FRAME_GLOBAL_INT ? (uint8_t)FRAME_GLOBAL : i.frame == FRAME_GLOBAL_RELATIVE_ALT_INT ? (uint8_t)FRAME_GLOBAL_RELATIVE_ALT : i.frame;
        m.current = i.current; m.autocontinue = i.autocontinue; m.mission_type = i.mission_type;
        return m;
    }
    void count(const Parser::Frame& f, const MissionCount& c, Vehicle& v, Scheduler& out, uint32_t now_ms) {
        x_ = Xfer(); x_.sys = f.sysid; x_.comp = f.compid; x_.type = c.mission_type;
        if (c.mission_type != MISSION_TYPE_MISSION) { mission_ack(f.sysid, f.compid, c.mission_type, c.count == 0 && (c.mission_type == MISSION_TYPE_FENCE || c.mission_type == MISSION_TYPE_RALLY) ? (uint8_t)MISSION_ACCEPTED : (uint8_t)MISSION_UNSUPPORTED, out); return; }
        if (v.mission_busy()) { mission_ack(f.sysid, f.compid, c.mission_type, MISSION_DENIED, out); return; }
        if (c.count > kStoreMax) { mission_ack(f.sysid, f.compid, c.mission_type, MISSION_NO_SPACE, out); return; }
        if (c.count == 0) { mission_ack(f.sysid, f.compid, c.mission_type, v.mission_replace(nullptr, 0) ? (uint8_t)MISSION_ACCEPTED : (uint8_t)MISSION_DENIED, out); return; }
        x_.state = RECEIVING; x_.total = c.count; x_.last_ms = now_ms;
        request(0, out);
    }
    void item(const Parser::Frame& f, const MissionItemInt& it, Vehicle& v, Scheduler& out, uint32_t now_ms) {
        if (x_.state != RECEIVING || !partner(f)) return;
        if (it.mission_type != x_.type) { mission_ack(f.sysid, f.compid, x_.type, MISSION_UNSUPPORTED, out); return; }
        if (it.seq != x_.next) { mission_ack(f.sysid, f.compid, x_.type, MISSION_INVALID_SEQUENCE, out); return; }
        int32_t home_alt = 0; const bool home = v.home_alt_mm(home_alt); uint8_t result = MISSION_ACCEPTED;
        if (!store_item_of(it, home, home_alt, staged_[it.seq], result)) { mission_ack(f.sysid, f.compid, x_.type, result, out); return; }
        x_.next++; x_.last_ms = now_ms; x_.retries = 0;
        if (x_.next < x_.total) { request(x_.next, out); return; }
        // all in: now it replaces the mission (not while that one flies)
        mission_ack(f.sysid, f.compid, x_.type, !v.mission_busy() && v.mission_replace(staged_, x_.total) ? (uint8_t)MISSION_ACCEPTED : (uint8_t)MISSION_DENIED, out);
    }
    void clear_all(const Parser::Frame& f, uint8_t type, Vehicle& v, Scheduler& out) {
        if (type == MISSION_TYPE_FENCE || type == MISSION_TYPE_RALLY) { mission_ack(f.sysid, f.compid, type, MISSION_ACCEPTED, out); return; }   // (nothing kept)
        if (type != MISSION_TYPE_MISSION && type != MISSION_TYPE_ALL) { mission_ack(f.sysid, f.compid, type, MISSION_UNSUPPORTED, out); return; }
        mission_ack(f.sysid, f.compid, type, !v.mission_busy() && v.mission_replace(nullptr, 0) ? (uint8_t)MISSION_ACCEPTED : (uint8_t)MISSION_DENIED, out);
    }
    void set_current(const Parser::Frame& f, uint16_t seq, Scheduler& out, const State& s) {
        const bool active = s.mission_state == MISSION_STATE_ACTIVE;
        if ((active && seq == s.mission_seq) || (!active && seq == (home_item_ ? 1 : 0))) mission_current(s, out);
        else { MissionAck a; a.target_system = f.sysid; a.target_component = f.compid; a.type = MISSION_ERROR; out.reply(a); }
    }

    uint8_t sysid_ = 1; uint32_t version_ = 0; bool home_item_ = false;
    Xfer x_; StoreItem staged_[kStoreMax];
    bool pending_ = false; uint16_t pending_cmd_ = 0; uint8_t pending_sys_ = 0, pending_comp_ = 0;
    bool track_ = false; uint16_t reached_ = 0;
};

} // namespace mav
