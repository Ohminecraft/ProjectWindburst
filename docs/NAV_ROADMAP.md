# Altitude Hold / Position Hold — rewrite (roadmap)

Rewrite of the navigation code in `src/core/nav/` (namespace `fc::nav`), split into layers: **Estimator → Setpoint → Controller → Mode logic**.
Each phase: builds clean on every MCU, unit tests pass, this file is updated, then **stop and wait for a board test**.

## Decisions (2026-10-24)

| Question | Choice |
|---|---|
| Fix the old code or rewrite | Full rewrite. The old code (`vertical_est.h`, `horizontal_est.h`, `althold.h`, `poshold.h`, `poscontrol.h`) keeps running until the new code can replace each part |
| Which task it runs in | The flight task (stable on every chip: ESP32-S3, ESP32, STM32), divided down to about 200 Hz. Slow sensors (baro, GPS, flow) are still pushed in by the sensor task via `on_baro` / `on_gps` / ... |
| Logging | New blackbox group `NAV_DEBUG`, present in every fast record (default 250 Hz, i.e. above 100 Hz) |
| 15-state EKF (`ekf.h`) | Left as is, still runs in parallel (shadow). The new nav does **not** use it |

General rules:
- Float only, no double; no dynamic allocation in the flight loop.
- SI units, NED frame, origin at home.
- Pure math kept separate for unit testing; every module has `reset()` and handles a bad `dt`.
- Parameters live in the config, with names suffixed by their unit.
- When ALT / POS HOLD is off, flight behaviour is unchanged.
- References: ArduPilot, PX4, Betaflight, INAV. Learn the algorithms only, rewrite them ourselves, credit the source at the top of the file.

## Phases

| | Content | Status |
|---|---|---|
| P0 | Nav logging + how to plot it | done on PC (0.34.0) |
| P1 | `AltEstimator` (`nav/alt_estimator.h`, `nav/state_history.h`) | done on PC (0.35.0) |
| P2 | ALT HOLD: velocity loop + hover-throttle estimate + state machine (`nav/alt_control.h`) | done on PC (0.35.0) |
| P3 | Z position loop, throttle stick = climb/descent rate, altitude lock, takeoff / landing (`nav/alt_control.h`) | done on PC (0.35.0) |
| P4 | `PosEstimator` (`nav/pos_estimator.h`) | done on PC (0.35.0) |
| P5 | POS HOLD (`nav/pos_control.h`, `nav/pos_hold.h`) | done on PC (0.35.0) |
| P6 | Safety: reset handling, fence, degradation on sensor loss | done on PC (0.35.0) |
| P7 | Hooks for RTH / waypoints (`NavRequest`, GPS Rescue running on the new `PosController`) | done on PC (0.35.0) |
| P8 | Rescue rewritten after ArduPilot / PX4: RTL (PX4 cone, stoppable, acceleration-limited flight path, loiter above home), SmartRTL, LAND; OSD shows metres remaining (`rescue.h`, `nav/breadcrumbs.h`, `docs/NAV.md` section 4) | done on PC (0.36.0) |

The user chose to do P1 through P7 in one go and then test on the board once (2026-10-28). **None of it has been tested on the board yet.**

## P0 — nav logging (0.34.0)

`fc::nav::Log` (`src/core/nav/nav_log.h`) is filled by `FlightCore::estimator_step` at every estimator step (500 Hz). The blackbox group `NAV_DEBUG` (bit 15 of `blackbox.fields`, off by default) writes the following fields into every fast record:

| Field | Unit | Meaning |
|---|---|---|
| `nav_acc[0..2]` | m/s² | N, E, D acceleration with gravity removed. At rest all three are about 0; when climbing, D is negative |
| `nav_vib` | m/s² | Vibration: \|a − 2 Hz mean\|, filtered at 5 Hz (ArduPilot / PX4 also assess the accelerometer this way) |
| `nav_clip` | samples | Number of accelerometer samples hitting the limit (> `acc_clip_g` = 15.5 g) since the previous step |
| `nav_baro` | m | Baro altitude relative to the takeoff point (up +) |
| `nav_gps_pos[0..2]` | m | GPS position relative to home: N, E, D |
| `nav_gps_vel[0..2]` | m/s | GPS velocity in NED (D = 0 if the module does not measure it) |
| `nav_gps_hacc`, `nav_gps_sacc` | m, m/s | Position / velocity accuracy reported by the module itself |
| `nav_gps_sats` | | Number of satellites |
| `nav_gps_flags` | bit | bit 0: has a fix; bit 1: has vertical velocity; bit 2: used for XY |
| `nav_gps_seq` | | Increments by 1 on each fix: wherever it steps there is a new fix, so the real GPS rate can be seen |

GPS is only fed into nav once home is set, i.e. after arming (same as the old code).

How to view it: enable the "Detailed navigation data" group in the Blackbox tab, record a session, open the viewer and pick the "Detailed navigation" plot group. This group has 6 plots: NED acceleration, vibration + clip, baro, GPS position, GPS velocity, GPS accuracy. CSV can also be exported from the viewer.

Size: this group adds 38 bytes to every fast record. With every group enabled and 8 motors, a record is 148 bytes (the recorder's buffer is 192 bytes). At the default rate of 250 Hz this group adds about 9.5 kB/s.

### To test on the board (P0)

1. Flash 0.34.0. In the Blackbox tab, also enable the **Detailed navigation data** group; rate 250 Hz; trigger ARMED (or manual recording).
2. **Hand-held, no props**, arm (or record manually). Do in turn:
   - stay still for 10 s;
   - lift about 1 m and lower again, 3 times;
   - tilt about 30° back and forth;
   - walk about 10 m north and back (if there is GPS, do this outdoors).
3. Send the log file. Things to check:
   - At rest: `nav_acc` near 0 on all three axes (off by less than about 0.3 m/s²). If it is off by a lot, the accelerometer is not calibrated or the axes are mounted wrong.
   - When lifting: `nav_acc[2]` negative then positive; `nav_baro` rises about 1 m, baro noise below about 0.3 m.
   - When tilting: `nav_acc` almost unchanged (gravity is removed correctly).
   - `nav_vib` and `nav_clip` at rest. If possible, do one more run **with props, the craft tied down, about 50 % throttle**: `nav_vib` below about 3 m/s², `nav_clip` = 0. If exceeded, vibration must be reduced before moving to P2.
   - GPS: `nav_gps_seq` increments steadily (5 / 10 Hz depending on the module); when walking 10 m north, `nav_gps_pos[0]` rises by about 10; check `nav_gps_hacc` and `nav_gps_sats`.

## P1 - P7 (0.35.0)

Fully replaces the old code. The files `althold.h`, `vertical_est.h`, `horizontal_est.h`, `poshold.h`, `poscontrol.h` have been deleted. Their old unit tests now run against the new modules:
- test_nav (132 checks) uses `nav::PosController`, `nav::PosHoldMode`, `nav::PosEstimator` directly;
- test_core (Altitude Hold) through a small adapter in the test file;
- the old estimator's tests were rewritten in `test_nav_est`.

Order in `FlightCore`: **Estimator → Setpoint → Controller → Mode logic**. Everything runs in the flight task; the estimator at `est_hz` (default 500 Hz).

| Module | Job | What is new compared with the old code |
|---|---|---|
| `state_history.h` | Stores the state of roughly the last 128 steps | Delayed measurements (GPS 150 ms, baro, flow) are compared with the state **at measurement time**. The correction is applied to both present and past (PX4 EKF2, INAV) |
| `alt_estimator.h` | NED altitude: pos_d, vel_d, accel bias, baro offset | NED; baro / GPS / GPS vertical velocity compared with the delayed state; innovation and test ratio per source; resets are **counted**, with the step size |
| `alt_control.h` | `HoverEstimator`, `AltSetpoint` (throttle stick, ceiling), `AltController` (cascade position → velocity → measured acceleration), `AltHoldMode` (state machine) | Split into 4 parts; NED; when the estimator resets, the target shifts with it so the craft does not follow the step; has an altitude ceiling |
| `pos_estimator.h` | N/E: position, velocity, bias | GPS compared with the delayed state; **glitch** handling: if GPS is rejected for `glitch_reset_s` = 2 s, reset to GPS, counted (ArduPilot EKF3) |
| `pos_control.h` | NE cascade from AC_PosControl | Target shifts when the estimator resets |
| `pos_hold.h` | ArduCopter PosHold feel (brake, loiter, wind learning) | Reset handling; radius fence that accounts for stopping distance; pushing the stick toward home gives control back to the pilot |
| `nav_math.h` | sqrt controller, lean angle ↔ acceleration, NE ↔ body axis rotation, `NavRequest` | |

**Degradation on sensor loss** (kept from the old code, with closed-loop tests):
- POS HOLD has no position / velocity → reports "unavailable", pilot flies Angle + ALT HOLD;
- ALT HOLD loses altitude → throttle stick centre = measured hover throttle;
- baro silent → GPS takes over, no step.

**Fence** (`nav.fence_alt_max`, `nav.fence_radius`; 0 = off; Position tab):
- ceiling: climb rate is reduced so the craft stops 0.5 m below the ceiling; above the ceiling it descends by itself;
- radius: wherever the craft can no longer stop before the circle, the stick is no longer followed and the craft holds at the edge;
- while the fence is acting, the `FENCE` bit in `nav_flags` is set.

**P7 hook**: `FlightCore::set_nav_request(NavRequest, ms)` takes a point (N, E), optionally with an altitude, or a velocity. A request is only flown when:
- POS HOLD is on;
- the position is valid;
- the roll / pitch sticks are centred;
- the request is resent within `nav_request_timeout_ms` = 500 ms.

Touching the sticks gives the pilot control back. GPS Rescue (RTH) uses the new `PosController` and `AltHoldMode`.

**Log** (blackbox group `NAV_DEBUG`): besides the P0 fields there are also
- `nav_est_pd`, `nav_est_vd`, `nav_est_bias`, `nav_baro_off`;
- `nav_baro_ratio`, `nav_gps_ratio` (greater than 1 = rejected);
- `nav_resets[2]` (altitude, position);
- `nav_est_pos[2]`, `nav_est_vel[2]`, `nav_tgt_pd`, `nav_tgt_vd`, `nav_hover`, `nav_tgt_pos[2]`;
- `nav_ctl`: bit 0 ALT HOLD, 1 POS HOLD, 2 holding, 3 fence.

### To test on the board (0.35.0), in order, stop as soon as anything looks odd

Enable the blackbox group "Detailed navigation data" for every flight.

1. **Hand-held, no props** (as in P0): `nav_est_pd` must follow `-nav_baro`; at rest `nav_est_vd` near 0 and `nav_baro_ratio` below 1.
2. **ALT HOLD, open area, light wind, low altitude:**
   - take off with the throttle stick (the craft ramps up to about 0.6 m by itself);
   - hold the stick centred for 20 s: altitude error below about 0.3 m;
   - push up then release: smooth stop, no overshoot;
   - lower the stick fully: lands, motors go to idle.
   - In the log check `nav_tgt_pd` against `nav_est_pd`, and `nav_hover` (stable, at the right hover throttle).
3. **Altitude ceiling:** set `nav.fence_alt_max` = 5, ALT HOLD, push the stick fully up: the craft stops at about 4.5 m.
4. **POS HOLD (GPS + compass), outdoors:**
   - enable while hovering: holds within about 1.5 m for 30 s;
   - push pitch then release: brakes, stops, holds;
   - in wind, check that the craft leans into the wind.
   - In the log check `nav_gps_ratio` (mostly below 1), `nav_resets[1]` (should stay constant), and `nav_tgt_pos` against `nav_est_pos`.
5. **Radius fence:** set `nav.fence_radius` = 20, POS HOLD, fly away: the craft stops at the edge at about 20 m, the app reports the fence; pushing the stick toward home lets it fly back.
6. **GPS Rescue 0.36** (switch first, open area, 30 - 50 m, about 10 m high; `docs/NAV.md` section 4):
   - **RTL**: climbs to the return altitude (farther than `rescue_min_start_dist` → goes up to `rescue_return_alt`), flies straight back with the nose along the flight direction, descends to 10 m above home, waits 5 s, lands, disarms by itself. Goggles: `RTL ..M` counting down, then `RTL HOME`, `RTL LAND`.
   - **SmartRTL**: fly an L shape (north then east), enable rescue: the craft must return along the L, not cut diagonally.
   - **LAND**: enable rescue: the craft descends right where it is and stays put (within about 2 - 3 m) even in wind.
   - Only once the switch works correctly set failsafe = Rescue and test switching the transmitter off.
   - In the log check `rescue_phase`, `nav_target_n/e` (the target point running along the flight path) against the craft's position.

Send the log from each step. If ALT HOLD oscillates, lower `althold.acc_p` / `acc_i`. If POS HOLD wobbles, lower `nav.vel_p`.
