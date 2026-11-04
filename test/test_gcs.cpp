// The ground station's side of MAVLink (src/telemetry/mavlink_gcs.h): commands, the mission protocol (upload, download, clear,
// current, reached), AVAILABLE_MODES, TIMESYNC - against a fake craft. The frames go through the real encoder and parser both ways.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <string>
#include <vector>
#include "../src/telemetry/mavlink_gcs.h"
using namespace mav;

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, ...) do { const bool ok_ = (cond); if (getenv("V")) { printf("    "); printf(__VA_ARGS__); printf("\n"); } if (ok_) g_pass++; else { g_fail++; printf("  FAIL line %d: %s   ", __LINE__, #cond); printf(__VA_ARGS__); printf("\n"); } } while (0)

struct FakeVehicle : Vehicle {
    std::vector<StoreItem> store; bool busy = false, have_home = true; int32_t home_alt = 488123; bool refuse_replace = false; int replaces = 0;
    uint8_t arm_result = RESULT_IN_PROGRESS, mode_result = RESULT_ACCEPTED, home_result = RESULT_ACCEPTED, reboot_result = RESULT_ACCEPTED, start_result = RESULT_ACCEPTED;
    int arms = 0, modes = 0, homes = 0, reboots = 0, starts = 0; bool arm_on = false, arm_force = false; uint32_t mode = 999;
    uint8_t arm(bool on, bool force) override { arms++; arm_on = on; arm_force = force; return arm_result; }
    uint8_t set_mode(uint32_t custom) override { modes++; mode = custom; return mode_result; }
    uint8_t set_home_here() override { homes++; return home_result; }
    uint8_t reboot() override { reboots++; return reboot_result; }
    uint8_t mission_start() override { starts++; return start_result; }
    int mission_count() const override { return (int)store.size(); }
    StoreItem mission_item(int i) const override { return store[(size_t)i]; }
    bool mission_busy() const override { return busy; }
    bool home_alt_mm(int32_t& a) const override { a = home_alt; return have_home; }
    bool mission_replace(const StoreItem* it, int n) override { if (refuse_replace) return false; replaces++; store.assign(it, it + n); return true; }
    // 0.43: three tuning parameters; the last one stores at most 250 (as a setting's max)
    const char* pname[3] = {"PID_ROLL_P", "PID_ROLL_I", "RATE_MAX_ROLL"}; float pval[3] = {45, 80, 670}; int sets = 0; bool refuse_set = false;
    int param_count() const override { return 3; }
    bool param_get(int i, char id[17], float& v) const override { if (i < 0 || i >= 3) return false; strncpy(id, pname[i], 16); id[16] = 0; v = pval[i]; return true; }
    bool param_set(int i, float v) override { if (refuse_set || i < 0 || i >= 3) return false; sets++; pval[i] = i == 0 && v > 250 ? 250 : v; return true; }
};

// the ground station (system 255, component 190) and the craft (system 1) on one wire
struct Bench {
    Gcs g; FakeVehicle v; Scheduler sch; State s; uint32_t t = 1000; std::vector<Parser::Frame> out; bool keep_current = false;
    Bench() { Rates off; off.attitude = off.position = off.hud = off.status = off.rc = 0; sch.configure(off, 1); g.configure(1, (0u << 24) | (40u << 16) | (0u << 8) | 255u); }
    template <class M> void send(const M& m, uint8_t sys = 255, uint8_t comp = 190) {
        Encoder e; e.sysid = sys; e.compid = comp; uint8_t f[kMaxFrame]; const size_t n = pack(e, m, f);
        Parser p; for (size_t i = 0; i < n; i++) if (p.feed(f[i])) g.handle(p.frame(), v, sch, s, t, (uint64_t)t * 1000u);
        pump();
    }
    void pump() {                                       // the craft's frames that answer (not HEARTBEAT / MISSION_CURRENT: periodic)
        g.update(v, sch, s, t);
        uint8_t buf[4096]; const size_t n = sch.poll(t, s, buf, sizeof(buf)); Parser p;
        for (size_t i = 0; i < n; i++) if (p.feed(buf[i]) && p.frame().msgid != MSG_HEARTBEAT && (keep_current || p.frame().msgid != MSG_MISSION_CURRENT)) out.push_back(p.frame());
    }
    void wait(uint32_t ms) { const uint32_t end = t + ms; while (t < end) { t += 10; pump(); } }
    template <class M> bool last(M& m) { for (auto it = out.rbegin(); it != out.rend(); ++it) if (unpack(*it, m)) return true; return false; }
    template <class M> int count() { int n = 0; M m; for (auto& f : out) if (unpack(f, m)) n++; return n; }
    int ack() { MissionAck a; return last(a) ? a.type : -1; }                  // the last MISSION_ACK's result
    int cmd_ack(uint16_t cmd) { CommandAck a; for (auto it = out.rbegin(); it != out.rend(); ++it) if (unpack(*it, a) && a.command == cmd) return a.result; return -1; }
    int requested() { MissionRequestInt r; return last(r) ? r.seq : -1; }
};
static MissionItemInt item(uint16_t seq, uint16_t cmd, int32_t lat, int32_t lon, float z, float p1 = 0, float p2 = 0, uint8_t frame = FRAME_GLOBAL_RELATIVE_ALT_INT) {
    MissionItemInt it; it.target_system = 1; it.target_component = 1; it.seq = seq; it.command = cmd; it.x = lat; it.y = lon; it.z = z; it.param[0] = p1; it.param[1] = p2; it.frame = frame; return it;
}
static MissionCount count_of(uint16_t n, uint8_t type = MISSION_TYPE_MISSION) { MissionCount c; c.target_system = 1; c.target_component = 1; c.count = n; c.mission_type = type; return c; }
static void upload(Bench& b, const std::vector<MissionItemInt>& items) { b.send(count_of((uint16_t)items.size())); for (const auto& it : items) b.send(it); }
static StoreItem si(uint16_t cmd, int32_t lat, int32_t lon, int16_t alt_dm, uint16_t p = 0, uint8_t arg = 0) { StoreItem s; s.cmd = cmd; s.lat = lat; s.lon = lon; s.alt_dm = alt_dm; s.p = p; s.arg = arg; return s; }

static void t_upload() {
    printf("[mission upload] MISSION_COUNT, MISSION_REQUEST_INT / MISSION_ITEM_INT, MISSION_ACK (Betaflight mavlink_mission.c)\n");
    {   Bench b; b.send(count_of(4));
        CHECK(b.requested() == 0, "MISSION_COUNT 4: item 0 asked for");
        b.send(item(0, CMD_NAV_TAKEOFF, 0, 0, 10.0f)); CHECK(b.requested() == 1, "then item 1");
        b.send(item(1, CMD_NAV_WAYPOINT, 473980000, 85460000, 25.5f, 5.0f)); b.send(item(2, CMD_DO_SET_SERVO, 0, 0, 0, 2.0f, 1900.0f, FRAME_MISSION));
        CHECK(b.v.replaces == 0, "nothing replaced before the last item (the old mission stays while it comes in)");
        b.send(item(3, CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0));
        CHECK(b.ack() == MISSION_ACCEPTED && b.v.replaces == 1 && b.v.store.size() == 4, "the last one: MISSION_ACK ACCEPTED, the mission replaced (%zu commands)", b.v.store.size());
        const auto& st = b.v.store;
        CHECK(st[0].cmd == 22 && st[0].alt_dm == 100 && st[0].lat == 0, "TAKEOFF to 10 m (100 dm), no position");
        CHECK(st[1].cmd == 16 && st[1].lat == 473980000 && st[1].lon == 85460000 && st[1].alt_dm == 255 && st[1].p == 5, "WAYPOINT: the point, 25.5 m above home, hold 5 s");
        CHECK(st[2].cmd == 183 && st[2].arg == 2 && st[2].p == 1900, "DO_SET_SERVO: servo 2 to 1900 us");
        CHECK(st[3].cmd == 20, "RTL"); }
    {   Bench b; upload(b, {item(0, CMD_DO_CHANGE_SPEED, 0, 0, 0, 1, 3.5f, FRAME_MISSION), item(1, CMD_NAV_DELAY, 0, 0, 0, 12.4f, 0, FRAME_MISSION)});
        CHECK(b.ack() == MISSION_ACCEPTED && b.v.store.size() == 2 && b.v.store[0].p == 35 && b.v.store[1].p == 12, "DO_CHANGE_SPEED 3.5 m/s = 35 dm/s; DELAY 12.4 s = 12 s"); }
    {   Bench b; upload(b, {item(0, CMD_NAV_WAYPOINT, 473980000, 85460000, 513.123f, 0, 0, FRAME_GLOBAL_INT)});
        CHECK(b.ack() == MISSION_ACCEPTED && b.v.store.size() == 1 && b.v.store[0].alt_dm == 250, "frame GLOBAL (above sea level): 513.1 m with home at 488.1 m = 25 m above home (%d dm)", b.v.store.empty() ? 0 : b.v.store[0].alt_dm);
        Bench c; c.v.have_home = false; c.v.store = {si(16, 1, 2, 30)}; upload(c, {item(0, CMD_NAV_WAYPOINT, 473980000, 85460000, 513.0f, 0, 0, FRAME_GLOBAL)});
        CHECK(c.ack() == MISSION_INVALID && c.v.store.size() == 1 && c.v.store[0].lat == 1, "... without a home point: INVALID, the old mission kept"); }
    {   Bench b; b.v.store = {si(16, 1, 2, 30)};
        upload(b, {item(0, CMD_NAV_TAKEOFF, 0, 0, 10), item(1, 17 /* LOITER_UNLIM */, 1, 2, 10)});
        CHECK(b.ack() == MISSION_UNSUPPORTED && b.v.store.size() == 1 && b.v.replaces == 0, "a command it cannot fly (LOITER_UNLIM): UNSUPPORTED, nothing replaced"); }
    struct Bad { MissionItemInt it; uint8_t want; const char* what; };
    const Bad bad[] = {
        {item(0, CMD_NAV_WAYPOINT, 950000000, 0, 10), MISSION_INVALID_PARAM5_X, "latitude 95 deg"},
        {item(0, CMD_NAV_WAYPOINT, 0, 1850000000, 10), MISSION_INVALID_PARAM6_Y, "longitude 185 deg"},
        {item(0, CMD_NAV_WAYPOINT, 1, 1, 4000.0f), MISSION_INVALID_PARAM7, "4000 m above home (the section holds +-3276 m)"},
        {item(0, CMD_NAV_WAYPOINT, 1, 1, NAN), MISSION_INVALID_PARAM7, "height NaN"},
        {item(0, CMD_NAV_WAYPOINT, 1, 1, 10, NAN), MISSION_INVALID, "hold time NaN"},
        {item(0, CMD_NAV_WAYPOINT, 1, 1, 10, -1), MISSION_INVALID_PARAM1, "hold time -1 s"},
        {item(0, CMD_NAV_WAYPOINT, 1, 1, 10, 0, 0, FRAME_MISSION), MISSION_UNSUPPORTED_FRAME, "a waypoint in frame MISSION"},
        {item(0, CMD_NAV_DELAY, 0, 0, 0, -1), MISSION_INVALID_PARAM1, "DELAY -1 (time of day)"},
        {item(0, CMD_DO_SET_SERVO, 0, 0, 0, 7, 1500, FRAME_MISSION), MISSION_INVALID_PARAM1, "servo 7 (1..4)"},
        {item(0, CMD_DO_SET_SERVO, 0, 0, 0, 1, 3000, FRAME_MISSION), MISSION_INVALID_PARAM2, "servo pulse 3000 us"},
        {item(0, CMD_DO_CHANGE_SPEED, 0, 0, 0, 1, 0, FRAME_MISSION), MISSION_INVALID_PARAM2, "speed 0 m/s"},
    };
    for (const Bad& x : bad) { Bench b; upload(b, {x.it}); CHECK(b.ack() == x.want && b.v.replaces == 0, "%s: result %d (want %d)", x.what, b.ack(), x.want); }
    {   Bench b; MissionItemInt w = item(0, CMD_NAV_WAYPOINT, 1, 1, 10); w.param[3] = NAN; upload(b, {w});
        CHECK(b.ack() == MISSION_ACCEPTED, "WAYPOINT with yaw NaN (= keep the heading): taken"); }
    {   Bench b; b.send(count_of(2)); b.send(item(1, CMD_NAV_WAYPOINT, 1, 1, 10));
        CHECK(b.ack() == MISSION_INVALID_SEQUENCE && b.v.replaces == 0, "item 1 when 0 was asked for: INVALID_SEQUENCE"); }
    {   Bench b; b.v.busy = true; b.send(count_of(2)); CHECK(b.ack() == MISSION_DENIED && b.requested() < 0, "the mission flies: DENIED");
        Bench c; c.send(count_of(40)); CHECK(c.ack() == MISSION_NO_SPACE, "40 commands (32 fit): NO_SPACE");
        Bench d; d.v.store = {si(16, 1, 2, 30)}; d.send(count_of(0)); CHECK(d.ack() == MISSION_ACCEPTED && d.v.store.empty(), "MISSION_COUNT 0: the mission cleared"); }
    {   Bench b; b.v.store = {si(16, 1, 2, 30)}; b.send(count_of(1)); b.v.busy = true; b.v.refuse_replace = true; b.send(item(0, CMD_NAV_WAYPOINT, 1, 1, 10));
        CHECK(b.ack() == MISSION_DENIED && b.v.store.size() == 1, "started flying while it came in: DENIED, the old one kept"); }
    {   Bench b; b.send(count_of(2)); b.out.clear(); b.wait(1600);
        CHECK(b.count<MissionRequestInt>() == 1 && b.requested() == 0, "no item for 1.5 s: asked again (%d)", b.count<MissionRequestInt>());
        b.wait(8000); CHECK(b.ack() == MISSION_OPERATION_CANCELLED && b.count<MissionRequestInt>() == 5, "5 times, then OPERATION_CANCELLED (%d requests)", b.count<MissionRequestInt>());
        b.send(item(0, CMD_NAV_WAYPOINT, 1, 1, 10)); CHECK(b.v.replaces == 0 && b.ack() == MISSION_OPERATION_CANCELLED, "a late item is ignored"); }
    {   Bench b; b.send(count_of(1)); b.out.clear(); b.send(item(0, CMD_NAV_WAYPOINT, 1, 1, 10), 200, 1);
        CHECK(b.out.empty() && b.v.replaces == 0, "an item from another ground station (system 200) during the upload: ignored");
        MissionCount other = count_of(1); other.target_system = 2; Bench c; c.send(other); CHECK(c.out.empty(), "MISSION_COUNT for system 2: not ours, no answer"); }
}

static void t_download() {
    printf("[mission download] MISSION_REQUEST_LIST, MISSION_COUNT, MISSION_ITEM_INT; clear, set current, reached; fence / rally\n");
    Bench b; b.v.store = {si(22, 0, 0, 100), si(16, 473980000, 85460000, 255, 5), si(183, 0, 0, 0, 1900, 2), si(178, 0, 0, 0, 35), si(93, 0, 0, 0, 12), si(20, 0, 0, 0)};
    MissionRequestList rl; rl.target_system = 1; b.send(rl);
    MissionCount c; CHECK(b.last(c) && c.count == 6 && c.target_system == 255 && c.target_component == 190, "MISSION_COUNT 6 to the asker");
    std::vector<MissionItemInt> got;
    for (uint16_t i = 0; i < 6; i++) { MissionRequestInt r; r.target_system = 1; r.seq = i; b.send(r); MissionItemInt it; if (b.last(it) && it.seq == i) got.push_back(it); }
    CHECK(got.size() == 6, "every item sent on request (%zu)", got.size());
    if (got.size() == 6) {
        CHECK(got[0].command == 22 && got[0].z == 10.0f && got[0].frame == FRAME_GLOBAL_RELATIVE_ALT_INT, "TAKEOFF 10 m above home (frame GLOBAL_RELATIVE_ALT_INT)");
        CHECK(got[1].command == 16 && got[1].x == 473980000 && got[1].y == 85460000 && fabsf(got[1].z - 25.5f) < 1e-4f && got[1].param[0] == 5.0f, "WAYPOINT: point, 25.5 m, hold 5 s");
        CHECK(got[2].command == 183 && got[2].param[0] == 2.0f && got[2].param[1] == 1900.0f && got[2].frame == FRAME_MISSION, "DO_SET_SERVO servo 2, 1900 us (frame MISSION)");
        CHECK(got[3].command == 178 && fabsf(got[3].param[1] - 3.5f) < 1e-4f, "DO_CHANGE_SPEED 3.5 m/s");
        CHECK(got[4].command == 93 && got[4].param[0] == 12.0f, "DELAY 12 s");
        CHECK(got[5].command == 20 && got[5].autocontinue == 1, "RTL");
    }
    { MissionAck a; a.target_system = 1; b.send(a); b.out.clear(); MissionRequestInt r; r.target_system = 1; r.seq = 0; b.send(r);
      CHECK(b.out.empty(), "after the ground station's MISSION_ACK the transfer is over: a late request gets nothing"); }
    {   Bench x; x.v.store = b.v.store; x.send(rl); MissionRequestInt r; r.target_system = 1; r.seq = 0; x.send(r); r.seq = 0; x.send(r);
        MissionItemInt it; CHECK(x.last(it) && it.seq == 0, "the same item asked twice (a lost reply): sent again");
        r.seq = 3; x.send(r); CHECK(x.ack() == MISSION_INVALID_SEQUENCE, "a jump ahead: INVALID_SEQUENCE"); }
    {   // round trip: what goes up comes down the same
        Bench x; std::vector<MissionItemInt> up = {item(0, CMD_NAV_TAKEOFF, 0, 0, 12), item(1, CMD_NAV_WAYPOINT, 473981234, 85461234, 30, 2), item(2, CMD_NAV_LOITER_TIME, 0, 0, 0, 8),
                                                   item(3, CMD_NAV_LAND, 473981000, 85461000, 0)};
        upload(x, up); x.send(rl); bool same = x.v.store.size() == 4;
        for (uint16_t i = 0; same && i < 4; i++) { MissionRequestInt r; r.target_system = 1; r.seq = i; x.send(r); MissionItemInt it; same = x.last(it) && it.command == up[i].command && it.x == up[i].x && it.y == up[i].y && it.z == up[i].z && it.param[0] == up[i].param[0]; }
        CHECK(same, "uploaded and downloaded again: the same commands, points, heights and times"); }
    {   Bench x; x.v.store = {si(16, 1, 1, 10), si(16, 2, 2, 10), si(21, 0, 0, 0)}; x.s.mission_state = MISSION_STATE_ACTIVE; x.s.mission_mode = 1; x.s.mission_seq = 1; x.send(rl);
        MissionRequestInt r; r.target_system = 1; MissionItemInt a, c2; r.seq = 0; x.send(r); x.last(a); r.seq = 1; x.send(r); x.last(c2);
        CHECK(a.current == 0 && c2.current == 1, "the command it flies is marked current"); }
    {   Bench x; MissionRequestList f; f.target_system = 1; f.mission_type = MISSION_TYPE_FENCE; x.send(f); MissionCount cnt;
        CHECK(x.last(cnt) && cnt.count == 0 && cnt.mission_type == MISSION_TYPE_FENCE, "fence list: none (count 0), so the ground station is not left waiting");
        x.send(count_of(0, MISSION_TYPE_RALLY)); CHECK(x.ack() == MISSION_ACCEPTED, "an empty rally upload: ACCEPTED");
        x.send(count_of(2, MISSION_TYPE_FENCE)); CHECK(x.ack() == MISSION_UNSUPPORTED, "a fence with points: UNSUPPORTED"); }
    {   Bench x; x.v.store = {si(16, 1, 1, 10)}; MissionClearAll ca; ca.target_system = 1; ca.mission_type = MISSION_TYPE_ALL;
        x.v.busy = true; x.send(ca); CHECK(x.ack() == MISSION_DENIED && x.v.store.size() == 1, "MISSION_CLEAR_ALL while it flies: DENIED");
        x.v.busy = false; x.send(ca); CHECK(x.ack() == MISSION_ACCEPTED && x.v.store.empty(), "MISSION_CLEAR_ALL: cleared"); }
    {   Bench x; x.v.store = {si(16, 1, 1, 10), si(16, 2, 2, 10)}; MissionSetCurrent sc; sc.target_system = 1; sc.seq = 0;
        x.pump(); x.keep_current = true; x.out.clear(); x.send(sc);                // (the periodic one went out at this time already: what comes is the answer)
        MissionCurrent mc; CHECK(x.last(mc) && mc.seq == 0, "MISSION_SET_CURRENT 0 before the start: answered with MISSION_CURRENT");
        sc.seq = 1; x.send(sc); CHECK(x.ack() == MISSION_ERROR, "MISSION_SET_CURRENT 1: ERROR (the mission always starts at its first command)"); }
    {   Bench x; x.s.mission_state = MISSION_STATE_ACTIVE; x.s.mission_mode = 1; x.s.mission_seq = 0; x.pump();
        x.s.mission_seq = 1; x.pump(); x.s.mission_seq = 2; x.pump();
        int r0 = -1, r1 = -1; MissionItemReached r; for (auto& f : x.out) if (unpack(f, r)) { if (r0 < 0) r0 = r.seq; else r1 = r.seq; }
        CHECK(r0 == 0 && r1 == 1, "MISSION_ITEM_REACHED as the mission moves on: %d, %d", r0, r1);
        x.out.clear(); x.s.mission_state = MISSION_STATE_COMPLETE; x.s.mission_mode = 2; x.pump(); CHECK(x.last(r) && r.seq == 2, "and the last one when it is done"); }
}

static void t_mission_planner() {
    printf("[mission planner] the float forms (MISSION_ITEM / MISSION_REQUEST); a home item at 0 on the way down (telemetry.mp_home_item)\n");
    {   // Mission Planner without MISSION_INT in its view of the craft answers MISSION_REQUEST_INT with MISSION_ITEM (degrees as floats)
        Bench b; b.send(count_of(2));
        MissionItem w; w.target_system = 1; w.seq = 0; w.command = CMD_NAV_WAYPOINT; w.frame = FRAME_GLOBAL_RELATIVE_ALT; w.x = 47.398f; w.y = 8.546f; w.z = 25.5f; w.param[0] = 3; b.send(w);
        MissionItem s2; s2.target_system = 1; s2.seq = 1; s2.command = CMD_DO_SET_SERVO; s2.frame = FRAME_MISSION; s2.param[0] = 1; s2.param[1] = 1200; b.send(s2);
        CHECK(b.ack() == MISSION_ACCEPTED && b.v.store.size() == 2, "MISSION_ITEM answers the requests: taken (%zu)", b.v.store.size());
        if (b.v.store.size() == 2)
            CHECK(abs(b.v.store[0].lat - 473980000) < 30 && abs(b.v.store[0].lon - 85460000) < 30 && b.v.store[0].alt_dm == 255 && b.v.store[0].p == 3 && b.v.store[1].arg == 1 && b.v.store[1].p == 1200,
                  "degrees to degE7 (%d, %d), height, hold; the servo command as it is", b.v.store[0].lat, b.v.store[0].lon);
        Bench c; c.send(count_of(1)); MissionItem g = w; g.frame = FRAME_GLOBAL; g.z = 513.123f; c.send(g);
        CHECK(c.ack() == MISSION_ACCEPTED && !c.v.store.empty() && c.v.store[0].alt_dm == 250, "frame GLOBAL: above home (%d dm)", c.v.store.empty() ? 0 : c.v.store[0].alt_dm); }
    {   Bench b; b.v.store = {si(22, 0, 0, 100), si(16, 473980000, 85460000, 255, 5), si(183, 0, 0, 0, 1900, 2)};
        MissionRequestList rl; rl.target_system = 1; b.send(rl); std::vector<MissionItem> got;
        for (uint16_t i = 0; i < 3; i++) { MissionRequest r; r.target_system = 1; r.seq = i; b.send(r); MissionItem it; if (b.last(it) && it.seq == i) got.push_back(it); }
        CHECK(got.size() == 3, "MISSION_REQUEST: answered with MISSION_ITEM (%zu)", got.size());
        if (got.size() == 3)
            CHECK(got[1].frame == FRAME_GLOBAL_RELATIVE_ALT && fabsf(got[1].x - 47.398f) < 1e-5f && fabsf(got[1].y - 8.546f) < 1e-5f && fabsf(got[1].z - 25.5f) < 1e-4f && got[1].param[0] == 5.0f
                  && got[2].frame == FRAME_MISSION && got[2].param[1] == 1900.0f, "in degrees, frame GLOBAL_RELATIVE_ALT (the float form's), the servo in frame MISSION"); }
    {   // Mission Planner drops item 0 of every download as "home" (FlightPlanner.cs processToScreen): with mp_home_item the list starts with one
        Bench b; b.g.configure(1, 0, true); b.v.store = {si(22, 0, 0, 100), si(16, 473980000, 85460000, 255, 5)};
        b.s.have_home = true; b.s.home_lat = 473977418; b.s.home_lon = 85455939; b.s.home_alt_mm = 488123;
        MissionRequestList rl; rl.target_system = 1; b.send(rl); MissionCount c;
        CHECK(b.last(c) && c.count == 3, "MISSION_COUNT 3: the home item and the 2 commands");
        std::vector<MissionItemInt> got; b.s.mission_state = MISSION_STATE_ACTIVE; b.s.mission_seq = 2;          // (FlightApp counts the home item in: command 1 is seq 2)
        for (uint16_t i = 0; i < 3; i++) { MissionRequestInt r; r.target_system = 1; r.seq = i; b.send(r); MissionItemInt it; if (b.last(it) && it.seq == i) got.push_back(it); }
        CHECK(got.size() == 3, "all three sent (%zu)", got.size());
        if (got.size() == 3) {
            CHECK(got[0].command == CMD_NAV_WAYPOINT && got[0].frame == FRAME_GLOBAL_INT && got[0].x == 473977418 && got[0].y == 85455939 && fabsf(got[0].z - 488.123f) < 1e-3f,
                  "item 0: home, above sea level (frame GLOBAL), as ArduPilot sends it");
            CHECK(got[1].command == CMD_NAV_TAKEOFF && got[2].command == CMD_NAV_WAYPOINT && got[2].x == 473980000, "then the commands, one further on");
            CHECK(got[2].current == 1 && got[1].current == 0, "the command it flies marked current at its own seq"); }
        b.s.mission_state = MISSION_STATE_NOT_STARTED; b.s.mission_seq = 1; b.out.clear();
        MissionSetCurrent sc; sc.target_system = 1; sc.seq = 1; b.pump(); b.keep_current = true; b.out.clear(); b.send(sc); MissionCurrent mc;
        CHECK(b.last(mc) && mc.seq == 1, "MISSION_SET_CURRENT 1 (the first command) before the start: answered");
        b.v.store.clear(); b.s.have_home = false; b.send(rl); CHECK(b.last(c) && c.count == 0, "no mission: count 0, no home item alone");
        Bench u; u.g.configure(1, 0, true); upload(u, {item(0, CMD_NAV_TAKEOFF, 0, 0, 10), item(1, CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0)});
        CHECK(u.ack() == MISSION_ACCEPTED && u.v.store.size() == 2 && u.v.store[0].cmd == 22, "the way up is as before (Mission Planner sends no home item to a craft that is not ArduPilot)"); }
}

static CommandLong cmd(uint16_t c, float p1 = 0, float p2 = 0) { CommandLong m; m.target_system = 1; m.target_component = 1; m.command = c; m.param[0] = p1; m.param[1] = p2; return m; }
static void t_commands() {
    printf("[commands] COMMAND_LONG / COMMAND_INT -> the craft, COMMAND_ACK back; REQUEST_MESSAGE; TIMESYNC\n");
    {   Bench b; b.send(cmd(CMD_COMPONENT_ARM_DISARM, 1));
        CHECK(b.v.arms == 1 && b.v.arm_on && b.cmd_ack(400) < 0 && b.g.pending(), "ARM: asked of the craft, the answer comes when the flight loop has tried (no ACK yet)");
        b.send(cmd(CMD_COMPONENT_ARM_DISARM, 1)); CHECK(b.cmd_ack(400) == RESULT_TEMPORARILY_REJECTED && b.v.arms == 1, "a second one meanwhile: TEMPORARILY_REJECTED");
        b.out.clear(); b.g.command_done(RESULT_ACCEPTED, b.sch); b.pump(); CommandAck a;
        CHECK(b.last(a) && a.command == 400 && a.result == RESULT_ACCEPTED && a.target_system == 255 && a.target_component == 190 && !b.g.pending(), "then COMMAND_ACK ACCEPTED to the asker"); }
    {   Bench b; b.v.arm_result = RESULT_ACCEPTED; b.send(cmd(CMD_COMPONENT_ARM_DISARM, 0, 21196));
        CHECK(!b.v.arm_on && b.v.arm_force && b.cmd_ack(400) == RESULT_ACCEPTED, "DISARM with 21196 (ArduPilot's force): forced");
        b.send(cmd(CMD_COMPONENT_ARM_DISARM, 0, 5)); CHECK(!b.v.arm_force, "any other param2: not forced");
        const int before = b.v.arms; b.send(cmd(CMD_COMPONENT_ARM_DISARM, 2)); b.send(cmd(CMD_COMPONENT_ARM_DISARM, NAN));
        CHECK(b.v.arms == before && b.cmd_ack(400) == RESULT_DENIED, "param1 2 or NaN: DENIED, the craft not asked"); }
    {   Bench b; b.send(cmd(CMD_DO_SET_MODE, 1, 16)); CHECK(b.v.mode == 16 && b.cmd_ack(176) == RESULT_ACCEPTED, "DO_SET_MODE custom 16 (PosHold)");
        b.send(cmd(CMD_DO_SET_MODE, 0, 2)); CHECK(b.v.mode == 16 && b.cmd_ack(176) == RESULT_DENIED, "without MAV_MODE_FLAG_CUSTOM_MODE_ENABLED: DENIED");
        b.v.mode_result = RESULT_TEMPORARILY_REJECTED; b.send(cmd(CMD_NAV_RETURN_TO_LAUNCH)); CHECK(b.v.mode == COPTER_RTL && b.cmd_ack(20) == RESULT_TEMPORARILY_REJECTED, "NAV_RETURN_TO_LAUNCH = mode RTL, the craft's answer passed on");
        CommandInt ci; ci.target_system = 1; ci.command = CMD_DO_SET_MODE; ci.param[0] = 1; ci.param[1] = 2; b.v.mode_result = RESULT_ACCEPTED; b.send(ci);
        CHECK(b.v.mode == 2 && b.cmd_ack(176) == RESULT_ACCEPTED, "COMMAND_INT works the same"); }
    {   Bench b; b.send(cmd(CMD_DO_SET_HOME, 1)); CHECK(b.v.homes == 1 && b.cmd_ack(179) == RESULT_ACCEPTED, "DO_SET_HOME 1: here");
        b.send(cmd(CMD_DO_SET_HOME, 0)); CHECK(b.v.homes == 1 && b.cmd_ack(179) == RESULT_UNSUPPORTED, "DO_SET_HOME at given coordinates: UNSUPPORTED (home follows the arming)");
        b.send(cmd(CMD_PREFLIGHT_REBOOT_SHUTDOWN, 1)); CHECK(b.v.reboots == 1 && b.cmd_ack(246) == RESULT_ACCEPTED, "reboot");
        b.send(cmd(CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0)); CHECK(b.v.reboots == 1 && b.cmd_ack(246) == RESULT_ACCEPTED, "param1 0: nothing to do, ACCEPTED");
        b.send(cmd(CMD_PREFLIGHT_REBOOT_SHUTDOWN, 3)); CHECK(b.v.reboots == 1 && b.cmd_ack(246) == RESULT_UNSUPPORTED, "the bootloader: UNSUPPORTED");
        b.send(cmd(CMD_MISSION_START)); CHECK(b.v.starts == 1 && b.cmd_ack(300) == RESULT_ACCEPTED, "MISSION_START");
        b.send(cmd(31000)); CHECK(b.cmd_ack(31000) == RESULT_UNSUPPORTED, "an unknown command: UNSUPPORTED");
        CommandLong other = cmd(CMD_MISSION_START); other.target_system = 7; b.out.clear(); b.send(other); CHECK(b.out.empty() && b.v.starts == 1, "for system 7: ignored"); }
    {   Bench b; b.send(cmd(CMD_REQUEST_MESSAGE, MSG_AUTOPILOT_VERSION)); AutopilotVersion v;
        CHECK(b.last(v) && v.capabilities == (CAP_MAVLINK2 | CAP_MISSION_INT | CAP_COMMAND_INT) && v.flight_sw_version == ((40u << 16) | 255u) && b.cmd_ack(512) == RESULT_ACCEPTED,
              "REQUEST_MESSAGE AUTOPILOT_VERSION: MAVLink 2, MISSION_INT, COMMAND_INT, version 0.40.0");
        b.out.clear(); b.send(cmd(CMD_REQUEST_AUTOPILOT_CAPABILITIES, 1)); CHECK(b.last(v) && b.cmd_ack(520) == RESULT_ACCEPTED, "REQUEST_AUTOPILOT_CAPABILITIES: the same");
        b.out.clear(); b.send(cmd(CMD_REQUEST_MESSAGE, MSG_AVAILABLE_MODES, 0));
        std::vector<AvailableModes> m; AvailableModes a; for (auto& f : b.out) if (unpack(f, a)) m.push_back(a);
        bool order = m.size() == 9; for (size_t i = 0; order && i < m.size(); i++) order = m[i].mode_index == i + 1 && m[i].number_modes == 9;
        CHECK(order && b.cmd_ack(512) == RESULT_ACCEPTED, "AVAILABLE_MODES, all (param2 0): 9 modes, index 1..9 (%zu)", m.size());
        if (m.size() == 9) {
            CHECK(m[8].custom_mode == COPTER_LOITER && !strcmp(m[8].mode_name, "Loiter") && m[8].standard_mode == STANDARD_MODE_POSITION_HOLD && m[8].properties == 0,
                  "Loiter (0.41) at the end: 5, the standard POSITION_HOLD (ArduCopter gives it to Loiter, not to PosHold)");
            CHECK(m[0].custom_mode == COPTER_ACRO && !strcmp(m[0].mode_name, "Acro") && m[3].custom_mode == COPTER_POSHOLD && m[3].standard_mode == STANDARD_MODE_NON_STANDARD
                  && m[4].custom_mode == COPTER_AUTO && m[4].standard_mode == STANDARD_MODE_MISSION && m[5].custom_mode == COPTER_RTL, "ArduCopter numbers with the standard modes");
            CHECK(m[6].custom_mode == COPTER_SMART_RTL && (m[6].properties & MODE_PROPERTY_NOT_USER_SELECTABLE) && m[7].custom_mode == COPTER_LAND && (m[7].properties & MODE_PROPERTY_NOT_USER_SELECTABLE)
                  && !(m[5].properties & MODE_PROPERTY_NOT_USER_SELECTABLE), "Smart RTL and Land shown, not selectable (they come from the rescue setting / the failsafe)"); }
        b.out.clear(); b.send(cmd(CMD_REQUEST_MESSAGE, MSG_AVAILABLE_MODES, 4)); CHECK(b.count<AvailableModes>() == 1 && b.last(a) && !strcmp(a.mode_name, "Pos Hold"), "one mode by its index");
        b.send(cmd(CMD_REQUEST_MESSAGE, MSG_AVAILABLE_MODES, 10)); CHECK(b.cmd_ack(512) == RESULT_DENIED, "index 10: DENIED");
        b.out.clear(); b.send(cmd(CMD_REQUEST_MESSAGE, MSG_AVAILABLE_MODES_MONITOR)); AvailableModesMonitor mon; CHECK(b.last(mon) && mon.seq == 1, "AVAILABLE_MODES_MONITOR: the set never changes (seq 1)");
        b.s.have_home = true; b.s.home_lat = 11; b.s.home_lon = 22; b.s.home_alt_mm = 33; b.out.clear(); b.send(cmd(CMD_REQUEST_MESSAGE, MSG_HOME_POSITION)); HomePosition h;
        CHECK(b.last(h) && h.latitude == 11 && h.altitude == 33 && b.cmd_ack(512) == RESULT_ACCEPTED, "HOME_POSITION on request");
        b.s.have_home = false; b.send(cmd(CMD_REQUEST_MESSAGE, MSG_HOME_POSITION)); CHECK(b.cmd_ack(512) == RESULT_FAILED, "no home yet: FAILED");
        b.send(cmd(CMD_REQUEST_MESSAGE, MSG_ATTITUDE)); CHECK(b.cmd_ack(512) == RESULT_UNSUPPORTED, "a message it does not send on request: UNSUPPORTED"); }
    {   Bench b; b.t = 5000; Timesync q; q.ts1 = 777; b.send(q); Timesync r;
        CHECK(b.last(r) && r.tc1 == 5000000000ll && r.ts1 == 777, "TIMESYNC: answered with its own time (ns) and the asker's");
        b.out.clear(); q.tc1 = 1; b.send(q); CHECK(b.out.empty(), "a TIMESYNC answer (tc1 set) is not answered"); }
}

// 0.43: the parameter protocol (PARAM_REQUEST_LIST / _READ, PARAM_SET, PARAM_VALUE): the tuning from Mission Planner / QGroundControl
static void t_params() {
    printf("[params] the tuning as MAVLink parameters\n");
    auto pid = [](const ParamValue& v) { return std::string(v.param_id, strnlen(v.param_id, 16)); };
    {   Bench b; ParamRequestList l; l.target_system = 1; l.target_component = 1; b.send(l); b.wait(200);
        std::vector<ParamValue> got; ParamValue v; for (auto& f : b.out) if (unpack(f, v)) got.push_back(v);
        CHECK(got.size() == 3 && pid(got[0]) == "PID_ROLL_P" && got[0].param_index == 0 && got[2].param_index == 2 && got[1].param_count == 3 && got[0].param_value == 45.0f && got[0].param_type == PARAM_TYPE_REAL32,
              "PARAM_REQUEST_LIST: every parameter once, in order, with its index, the count and REAL32 (%zu)", got.size()); }
    {   Bench b; ParamRequestList l; l.target_system = 1; l.target_component = 1; b.send(l); int most = 0;
        for (int k = 0; k < 50; k++) { b.t += 20; b.g.update(b.v, b.sch, b.s, b.t); if (b.sch.pending_replies() > most) most = b.sch.pending_replies(); }
        CHECK(most == 1 && b.sch.dropped_replies() == 0, "a link that takes nothing for a second: the list waits, one PARAM_VALUE queued at most (%d), none dropped", most); }
    {   Bench b; ParamRequestList l; l.target_system = 7; l.target_component = 1; b.send(l); b.wait(200);
        CHECK(b.count<ParamValue>() == 0, "a list asked of another system: nothing"); }
    {   Bench b; ParamRequestRead r; r.target_system = 1; r.target_component = 1; r.param_index = -1; strncpy(r.param_id, "RATE_MAX_ROLL", 16); b.send(r);
        ParamValue v; CHECK(b.last(v) && pid(v) == "RATE_MAX_ROLL" && v.param_index == 2 && v.param_value == 670.0f, "PARAM_REQUEST_READ by name (index -1)");
        b.out.clear(); r.param_index = 1; memset(r.param_id, 0, 16); b.send(r); CHECK(b.last(v) && pid(v) == "PID_ROLL_I", "... by index");
        b.out.clear(); r.param_index = -1; strncpy(r.param_id, "NO_SUCH", 16); b.send(r); CHECK(b.count<ParamValue>() == 0, "an unknown name: no answer (ArduPilot)"); }
    {   Bench b; ParamSet p; p.target_system = 1; p.target_component = 1; strncpy(p.param_id, "PID_ROLL_P", 16); p.param_value = 52; p.param_type = PARAM_TYPE_REAL32; b.send(p);
        ParamValue v; CHECK(b.v.sets == 1 && b.v.pval[0] == 52 && b.last(v) && pid(v) == "PID_ROLL_P" && v.param_value == 52.0f && v.param_count == 3, "PARAM_SET: stored, and PARAM_VALUE says so");
        b.out.clear(); p.param_value = 900; b.send(p); CHECK(b.last(v) && v.param_value == 250.0f, "too big: PARAM_VALUE carries what was really stored (%g)", v.param_value);
        b.out.clear(); p.param_value = NAN; b.send(p); CHECK(b.v.sets == 2 && b.last(v) && v.param_value == 250.0f, "NaN: not stored, the value as it is is sent back");
        b.out.clear(); strncpy(p.param_id, "NO_SUCH", 16); b.send(p); CHECK(b.count<ParamValue>() == 0 && b.v.sets == 2, "an unknown name: ignored");
        b.out.clear(); b.v.refuse_set = true; strncpy(p.param_id, "PID_ROLL_I", 16); p.param_value = 90; b.send(p); CHECK(b.last(v) && v.param_value == 80.0f, "refused by the craft: the old value is sent back"); }
    {   Bench b; ParamSet p; p.target_system = 1; p.target_component = 5; strncpy(p.param_id, "PID_ROLL_P", 16); p.param_value = 60; b.send(p);
        CHECK(b.v.sets == 0, "a PARAM_SET for another component: ignored"); }
}

int main() {
    t_upload(); t_download(); t_mission_planner(); t_commands(); t_params();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
