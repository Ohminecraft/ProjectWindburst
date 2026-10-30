# Nhiệm vụ bay — phần 1 (firmware) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The flight controller stores a waypoint mission (ArduPilot command semantics), flies it by itself (takeoff, waypoints, hover + servo steps, land-wait-takeoff with auto re-arm, RTL), started by a MISSION switch or a WBP command.

**Architecture:** A portable `MissionRunner` (`src/core/nav/mission.h`) turns the plan (already converted to metres from home) and the craft state into the same kind of output the GPS rescue gives (target point + velocity feed-forward + climb rate), plus arm / disarm / takeoff / servo / RTL requests. `FlightCore` flies that output with its existing position controller (`rctl_`) and Altitude Hold; `FlightApp` owns the settings section, the lat/lon → NE conversion, the home point, servos, beeper, telemetry and OSD.

**Tech Stack:** C++17 header-only firmware, PC tests (`make test`, FIL `test_fw`), WBP schema generator `tools/gen_proto.py`, mutation tests `tools/mutate.py`.

**Spec:** `docs/superpowers/specs/2026-10-29-mission-design.md`

## Global Constraints

- Schema: edit `protocol/wbp_schema.json`, run `python tools/gen_proto.py`; enums append-only; new fields at the end of a section; a section that grows gets its old size in `compat` (max 4 entries).
- Code comments in English; docs in Vietnamese. CRLF files stay CRLF (`todo.txt`, `test/test_core.cpp`).
- Version bump to **0.37.0** in `src/config/version.h` and the "Windburst RX (sim) 0.37.0" string in `test/test_fw.cpp`.
- Every feature: unit tests, FIL tests, docs (`docs/MISSION.md`) + HANDOFF row + todo, mutants in `tools/mutate.py` (never edit `src/` while mutate runs).
- Windows build: `export PATH=/c/msys64/ucrt64/bin:$PATH; mingw32-make -k "CXXFLAGS=-std=gnu++17 -O2 -Wall -Wextra -Wno-unused-parameter" build/test_X`.
- Mission size: 32 commands. Accept radius 2 m, height arrival 1 m (takeoff 0.5 m). Ground-wait warning: the last 10 s. Defaults as the rescue: speed 5 m/s (`DO_CHANGE_SPEED` overrides), accel 2 m/s², leash 5 m, landing_alt 4 m, land speed 0.5 m/s, descend 1.5 m/s, ascend 5 m/s... take them from `RescueConfig` (one config, not a second copy).

## Review Focus

1. Mission re-arm after a mid-mission landing must NOT move home / the height origin — test: after land-wait-takeoff the craft reaches the next waypoint within 2 m of its true position and home distance stays right (Task 5, Task 6).
2. ARM switch turned OFF during the ground wait → no automatic take-off ever (Task 5).
3. A plan that starts with something other than TAKEOFF while on the ground → start refused, motors never spin (Task 2, Task 5).
4. Link lost mid-leg → the mission carries on (default) / RTL when `mission_rc_lost = RTL` (Task 5).
5. Battery critical during the mission → the failsafe lands, the mission does not fight it (Task 5).

---

### Task 1: Schema additions

**Files:**
- Modify: `protocol/wbp_schema.json`; regenerated `src/proto/wbp_schema.h`, `src/proto/wbp_fields.h`
- Test: `test/test_proto.cpp`

**Interfaces:**
- Produces enums: `mission_cmd` [NONE, TAKEOFF, WAYPOINT, LOITER_TIME, LAND, DELAY, RTL, SERVO, SPEED]; `mission_state` [IDLE, TAKEOFF, WAYPOINT, LOITER, LANDING, GROUND_WAIT, ARMING, RTL, DONE, REFUSED, FAILED]; `mission_rc_lost` [CONTINUE, RTL]. Appended: `mode` += MISSION; `servo_source` += MISSION; `beeper_cond` += MISSION_WARN (and its bit in the default of `beeper.conditions`).
- Section `mission` (id 26, live): `count` u8 (max 32), `cmd[32]` u8 enum mission_cmd, `arg[32]` u8, `alt[32]` i16 (dm), `lat[32]` i32, `lon[32]` i32, `p[32]` u16.
- Section `servo` appends `angle_min[4]` i16 (d 0), `angle_max[4]` i16 (d 180), `init_us[4]` u16 (d 1500, 500..2500); compat += old size.
- Section `nav` appends `mission_rc_lost` u8 (d CONTINUE); compat updated (drop the oldest if 4).
- Stream `nav` appends `mission_state` u8, `mission_index` u8, `mission_count` u8, `mission_dist` u16 (×0.1 m), `mission_left` u16 (m), `mission_wait` u8 (s).
- Command `MISSION` (args: `action` u8: 0 stop, 1 start).

- [ ] Step 1: test_proto checks: section `mission` id 26 exists with the six arrays and `count`; defaults of `servo.angle_max` = 180 and `init_us` = 1500; `nav.mission_rc_lost` default 0; servo / nav compat accept the old sizes; enum names `MISSION` present in `mode` and `servo_source`.
- [ ] Step 2: run `build/test_proto` → FAIL.
- [ ] Step 3: edit the schema, `python tools/gen_proto.py`.
- [ ] Step 4: `build/test_proto` → PASS; rebuild `build/wb_sim`.

### Task 2: MissionRunner (portable)

**Files:**
- Create: `src/core/nav/mission.h`
- Create: `test/test_mission.cpp`; Modify: `Makefile` (add `build/test_mission` to `test:` and its run line)

**Interfaces:**
- Produces:
  - `enum class MissionCmd : uint8_t` (same order as schema `mission_cmd`), `enum class MissionState : uint8_t` (same order as `mission_state`).
  - `uint16_t mav_cmd_of(MissionCmd)` / `MissionCmd mission_cmd_of_mav(uint16_t)` (16, 19, 20, 21, 22, 93, 178, 183).
  - `struct MissionItem { MissionCmd cmd; uint8_t arg; float n, e; bool has_pos; float alt_m; bool has_alt; uint16_t p; }`, `struct MissionPlan { MissionItem item[32]; uint8_t count; }`, `constexpr int kMissionMax = 32`.
  - `struct MissionInput { bool armed, airborne, pos_valid, alt_valid, landed /* AltHold LANDED_IDLE */, arm_switch_on, rtl_done; float pos_n, pos_e, vel_n, vel_e, z, vz, yaw_deg; }`.
  - `struct MissionOutput { bool active, horizontal; float tgt_n, tgt_e, vel_n, vel_e, climb_mps; bool takeoff, want_arm, want_disarm, warn, want_rtl, yaw_control; float yaw_rate_dps; int8_t servo; uint16_t servo_us; }` (`servo` = -1 when no servo command this step).
  - `class MissionRunner { RescueConfig* cfg; bool start(const MissionPlan&, const MissionInput&); void stop(); MissionOutput update(const MissionInput&, float dt); MissionState state() const; uint8_t index() const; uint8_t count() const; float dist() const; float left() const; uint8_t wait_s() const; bool holds_home() const; }`
    - `start` returns false and sets state REFUSED for: empty plan, on the ground (not airborne) with first NAV ≠ TAKEOFF, no position.
    - `holds_home()`: true from the first mid-mission landing until the mission ends (FlightApp keeps home / origin).

- [ ] Step 1: tests in `test/test_mission.cpp` (point-mass sim like `RescSim` in test_nav: tracks the target with the velocity feed-forward):
  - `t_mav_map`: round trip of all 8 commands.
  - `t_refuse`: empty plan, ground start without TAKEOFF → `start()` false, state REFUSED, output inactive.
  - `t_takeoff_wp`: TAKEOFF 10 → WAYPOINT (40, 0, 10) → WAYPOINT (40, 30, 15): state order TAKEOFF WAYPOINT WAYPOINT DONE; arrives within 2 m; never above 15.5 m; speed ≤ 5.2 m/s; `left()` decreases monotonically on each leg.
  - `t_do_order`: WAYPOINT P, LOITER_TIME(0,0,3 s), SERVO(2, 1800), LOITER_TIME(2 s), SERVO(2, 1100): servo 2 → 1800 at arrival, → 1100 exactly 3 s later (±0.1 s), each emitted once.
  - `t_land_wait_takeoff`: LAND at P → `want_disarm` once landed; DELAY 20 → `warn` only in the last 10 s, `wait_s()` counts down; then TAKEOFF → `want_arm` until armed, then climbs; `holds_home()` true from the landing.
  - `t_rtl_end`: RTL item → `want_rtl` until `rtl_done`, then DONE; end of plan airborne → holds position (horizontal, target = last point).
  - `t_speed_cmd`: SPEED 3 m/s (p = 30 dm/s) → the next leg's cruise ≤ 3.1 m/s.
- [ ] Step 2: run → FAIL (no header).
- [ ] Step 3: implement `mission.h`. Leg follower: target point moves from the stopping point (pos + v·|v|/(2a)) along the leg at `v = min(speed, along + sqrt(2a·max(leash − lead, 0)), sqrt_controller(remaining, pos_p, accel))`; altitude `want_z` interpolated by leg progress, `climb = clamp(sqrt_controller(want_z − z, alt pos p 1.0, accel), −descend, ascend)`. DO commands after NAV N run when N starts (AP_Mission::update). LAND: horizontal hold of the point, climb = −descend above `landing_alt_m`, −`land_speed_mps` below.
- [ ] Step 4: run → PASS.

### Task 3: Altitude Hold automatic take-off

**Files:** Modify `src/core/nav/alt_control.h` (`AltInput::auto_takeoff`); Test: `test/test_nav_ctl.cpp`

- [ ] Step 1: test `t_auto_takeoff` in test_nav_ctl: armed, LANDED_IDLE, stick at 0, `pilot_absent = true`, `auto_takeoff = true` → TAKEOFF then lifted; stick 0 does not abort; without the flag nothing happens.
- [ ] Step 2: FAIL → Step 3: LANDED_IDLE / LANDED_PRE_TAKEOFF go to TAKEOFF when `auto_takeoff` (ignoring need_low / pilot_absent / stick); `takeoff()` skips the stick-low abort when `auto_takeoff` → Step 4: PASS.

### Task 4: Mission arm / disarm

**Files:** Modify `src/core/safety.h` (`Arming`); Test: `test/test_core.cpp` (CRLF!)

**Interfaces:** `void Arming::mission_disarm()` (armed=false, no sticky, prev_switch_ kept true); `bool Arming::mission_arm(bool switch_on, bool link_ok, ArmInput in)` → arms when (`switch_on` || !`link_ok`) and `arm_blockers(in) & ~(AB_THROTTLE | AB_ARM_SWITCH_ON | AB_RX)` == 0 (use the real bit names in safety.h).

- [ ] Step 1: tests: mission_disarm then mission_arm with switch on, throttle high → armed; switch off → not armed; tilt blocker → not armed; after mission_disarm the normal `update(switch on)` does not re-arm by itself.
- [ ] Step 2–4: FAIL → implement → PASS.

### Task 5: FlightCore integration

**Files:** Modify `src/core/rc.h` (`M_MISSION`, forces ANGLE), `src/core/flight_core.h`, `src/core/safety.h` (FsConfig `mission_rc_continue`); Test: `test/test_flight.cpp`

**Interfaces:**
- `void FlightCore::set_mission(const MissionPlan&)` (copied, used at the next start), `void FlightCore::mission_command(bool start)` (app start / stop latch), `const MissionRunner& mission() const`.
- `CoreOutput` adds: `mission_state, mission_index, mission_count, mission_wait` (u8), `mission_dist, mission_left` (float), `mission_warn`, `mission_holds_home` (bool), `servo_cmd` (int8, -1 none) + `servo_us`.
- Mission on = armed-by-pilot-first && (M_MISSION switch || app latch); app latch cleared by stop, by a switch on→off edge, by a pilot disarm. Rescue (switch / failsafe) > mission > Position Hold > nav request.
- RC lost while the mission runs: the failsafe is not raised when `cfg.fs.mission_rc_continue` (set from `nav.mission_rc_lost == CONTINUE`); with RTL it runs as today (rescue).

- [ ] Step 1: closed-loop tests `t_mission` in test_flight (Rig with ngps + mag, five-inch plant): full plan TAKEOFF 10 → WP (40,0) → WP+LOITER+SERVO → LAND (40,30) + DELAY 15 + TAKEOFF 12 → RTL: phases in order, servo command seen, disarmed on the ground at the landing point (< 2 m), re-armed by itself after 15 s with the warning in the last 10 s, home kept (lands within 3 m of the first take-off point at the end); link lost on the first leg → still completes; mission switch off mid-leg → pilot has Angle at once; ARM switch off during the wait → stays disarmed for 30 s; battery critical mid-leg → lands where it is; ground start with a plan beginning with WAYPOINT → refused, motors at 0.
- [ ] Step 2–4: FAIL → implement → PASS (`build/test_flight`, then all of `make test`).

### Task 6: FlightApp, servo, beeper, telemetry, OSD, MAVLink

**Files:** Modify `src/app/flight_app.h`, `src/hal/servo/servo_out.h`, `src/hal/beeper/beeper_core.h`, `src/app/telemetry.h`, `src/osd/osd.h`, `src/telemetry/mavlink_telemetry.h`, `src/proto/settings_apply.h`, `src/hal/gps/gps_nav.h` (home kept while `holds_home`); Test: `test/test_fw.cpp`, `test/test_osd.cpp`, `test/test_mavlink.cpp`

**Interfaces:**
- FlightApp converts section `mission` → `MissionPlan` (lat/lon → NE with `gps::offset_cm` from the home point at start; alt dm → m; zero lat/lon = no position) whenever the section changes or home is set; `core.set_mission()`.
- WBP command `MISSION` → `core.mission_command(action == 1)`; reply ERR_FAILED when start is refused (state REFUSED) or no home.
- `ServoOutputs::set_mission(int i, uint16_t us)`; source 17 (MISSION) outputs `init_us` until a mission command.
- Beeper `MISSION_WARN` (append after ON_USB; sequence `{10, 10, kStop}` fast) while `mission_warn`.
- Home: `gps::Home::update(sol, armed || holds_home)`; FlightApp skips `core.reset_position()` on an arming edge while `mission_holds_home`.
- OSD `osd::mission_text()`: `AUTO 3/9 120M`, `WAIT 45S`, `TAKEOFF 5S`; mode name `AUTO`.
- MAVLink HEARTBEAT `custom_mode = COPTER_AUTO (3)` while the mission runs.

- [ ] Step 1: FIL tests (block `mission` in test_fw): write the section over WBP, save, reboot keeps it; arm + MISSION start over WBP → takes off and flies (virtual board GPS); servo set to MISSION shows `init_us` then the commanded pulse; beeper MISSION_WARN sounds during the last 10 s of a ground wait; GPS home distance unchanged across the mission re-arm; nav stream fields; OSD line; start refused when disarmed (ERR_FAILED). test_osd: `mission_text` cases; test_mavlink: AUTO custom mode.
- [ ] Step 2–4: FAIL → implement → PASS (`test_fw`, `test_osd`, `test_mavlink`, `test_proto`).

### Task 7: App labels (minimum for phase 1)

**Files:** `app/wbclient/web/js/labels.js`, `js/lang/en.js`, `js/tabs/modes.js` / `motors.js` / `nav.js` as needed, `app/tests/test_gui_browser.py`

- [ ] Step 1: the Modes tab lists MISSION; servo source offers "Nhiệm vụ"; the three new servo fields edit; `nav.mission_rc_lost` in the Position tab; i18n test passes.
- [ ] Step 2: run `app/tests` (e2e + i18n + browser) against the rebuilt `wb_sim`.

### Task 8: Docs, mutants, version, builds

- [ ] `docs/MISSION.md` (Vietnamese: what it does, the command table, how to fly it, failsafes, what is not tested on hardware); HANDOFF row; `todo.txt` (CRLF) "Cần test"; spec link.
- [ ] Mutants for mission.h / alt auto_takeoff / Arming mission_* / FlightCore glue / servo / OSD in `tools/mutate.py`; run them; all killed.
- [ ] Version 0.37.0 (+ test_fw string); `pio run` for rx_esp32_s3, rx_esp32, rx_h563, rx_f405; note RAM of F405.
- [ ] Update `docs/MISSION_WORKLOG.md` with what was done and what is open.
