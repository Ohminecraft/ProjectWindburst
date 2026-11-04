# Flight core (rewritten from scratch) — `src/core`, namespace `fc`

## Why it was rewritten
The old core (`wb::FlightController`) was written before the drivers existed; once real drivers were in place it diverged in many places:
| Problem with the old core | New core |
|---|---|
| Mixer had its own `idle`/`output_limit`, duplicating `MotorOutputManager` (two owners of idle, no path in for dynamic idle) | Mixer only receives `MotorLimits{idle, limit}` **every loop** from the motor layer; outputs **absolute** values 0..1 within [idle, limit] |
| `FcInput.rpm` was an optional pointer, fixed at 4 motors | `MotorFeedback.rpm[8]` + `rpm_valid`; RPM notch filter for up to 8 motors × 3 harmonics |
| Sensors called `kf.update_*` directly at any time, with no age/confidence | `on_baro/on_range/on_gps/on_mag(value, sigma, time)`; the estimator checks **staleness** itself and uses the sigma measured by the driver |
| Only an altitude Kalman | + horizontal Kalman (GPS position/velocity, 150 ms GPS delay compensation) ready for Position Hold / RTH |
| Compass was only a loose function | Integrated compass: seeds yaw while disarmed, only corrects yaw, only used when `use_mag` + calibrated + field strength is right |
| Arming/failsafe scattered, unaware of each driver's health | `Health` taken directly from the drivers' `valid()/usable()`; `Arming` (rising edge of the switch, switch on at startup is blocked, needs an on-off cycle after failsafe), new blocker `AB_MAG_NOT_CALIBRATED` |
| Mixer table typed by hand | Mixer rows **derived from geometry** (motor angle, spin direction): QUAD_X and QUAD_PLUS match **Betaflight's default motor numbering and spin directions** (props in: QUAD_X 1 rear-right CW, 2 front-right CCW, 3 rear-left CCW, 4 front-left CW), HEX_X; checked against an independent physical plant. `props_out` = Betaflight's "motor direction reversed" |
| Everything ran at 4 kHz | Fast loop: gyro → filters → AHRS → rate PID → mixer; the estimator runs at `est_hz` (500) on averaged accel |

## Data flow
```
IMUAccGyro::read ─┐                                         ┌─ MotorSystem::update(out.motor, out.armed)
Baro/Range/GPS/Mag ┼─ FlightCore::on_*  ─┐                  │
RC (1000..2000us) ─┴─ FlightCore::update(FastInput) ─► CoreOutput ─┘  (motor[8] absolute, armed, blockers, failsafe, estimate, debug)
MotorSystem::idleFraction / cfg().output_limit ──► FastInput.limits (next loop)
MotorSystem::manager().rpm_array ───────────────► FastInput.motors (RPM filter)
```
`src/app/flight_app.h` (`FlightApp`) is the corresponding glue layer; it only forwards and has no flight logic (see `docs/FIRMWARE.md`).

Functions added when integrating the firmware:
- `apply_tuning()` — the app changes PID / rate / Angle / Stabilize while disarmed: pushes `cfg` into the controllers **without** resetting AHRS / estimator (`configure()` resets everything).
- `apply_filters()` — the app changes filters while disarmed: rebuilds the gyro filter chain and the D-term filter.
- `emergency_disarm()` — IMU lost entirely: disarm, the arm switch must be flipped again.
- `pid()`, `gyro_filter()`, `setpoints()`, `angle_ctl()`, `stabilize()`, `althold()` — read only: the running state for the `tuning` stream and for tests.

`FastInput.rc_seq` changes on every **new receiver frame**: feedforward and RC smoothing work per frame, not per loop cycle. `FastInput.motors.rpm[]` is each motor's speed (0 = that motor has no telemetry).

## Files
`types.h` (sign conventions — read the top of the file) · `filters.h` (PT1/2/3, BIQUAD and state-variable notch) · `gyro_filter.h` (gyro filter chain, RPM filter) · `dyn_notch.h` (SDFT + dynamic notch) · `pid.h` (Betaflight rate PID, throttle boost) · `setpoint.h` (frame rate, RC smoothing, feedforward) · `attitude.h` (Betaflight's Angle, ArduPilot's Stabilize) · `simplified_tuning.h` (sliders) · `inflight_trim.h` (in-flight level trim) · `ahrs.h` (Mahony + compass; how far the accelerometer is trusted in flight / at rest: `AccelGainSchedule`) · `vertical_est.h` (4-state Kalman: z, v, accel bias, baro offset) · `horizontal_est.h` · `rc.h` (Actual rates, `RcInterpreter`, `ModeMap`) · `mixer.h` · `althold.h` (ArduCopter-style 5-state machine, `HoverEstimator`) · `safety.h` (arm blockers, `Arming`, `Failsafe`, `RunawayTakeoff`, `LandDetector`) · `flight_core.h`.

Altitude estimator (`vertical_est.h`): the barometer holds `z`; the lidar measures **relative to a ground reference** when there is a barometer (10 consecutive readings outside the gate, or the first reading after 0.5 s of absence: the reference moves, `z` does not jump), and is the absolute altitude when there is no barometer; barometer sigma is clamped to 0.3 … 1.0 m, ×2 right after arming / leaving the ground (1.5 s). GPS (0.8.3, details in `docs/GPS.md`, section "GPS altitude"): the u-blox vertical velocity is always blended into `v` (brought to the present using the acceleration measured over the 0.15 s delay); GPS altitude per `nav.gps_alt` — `FALLBACK` (default): it is the altitude when there is neither barometer nor lidar, `BLEND`: additionally corrects barometer drift through the offset state `o` (relative to a reference taken from the first 20 readings, so only the *change* in GPS altitude has an effect), `OFF`: not used. The prediction step uses real time; each accelerometer sample is weighted by the time span it represents.

Details on the rate loop, filters, the two self-levelling modes, hover throttle: **`docs/TUNING.md`**.

## Modes
- **Acro**: stick → rate (Actual) → RC smoothing + feedforward → PID.
- **Angle** (Betaflight): the stick gives the tilt angle along the rate curve; `ModeMap` turns Angle on automatically when Altitude Hold / Position Hold is on, and failsafe also forces Angle.
- **Stabilize** (ArduPilot): the attitude target is shaped, the nose heading is held. Entering the mode in flight starts from the current attitude; standing on the ground with motors at idle, the target heading follows the craft.
- **Altitude Hold** (needs baro, lidar, or — since 0.8.3, `nav.gps_alt` not `OFF` — GPS altitude): hover throttle is measured automatically, takeoff by raising throttle gradually until leaving the ground. Altitude lost in flight → flag `althold_unavailable`, the throttle stick is read **around the measured hover throttle** (mid stick = hover), and when altitude returns it continues from that throttle level. Pulling the stick down after leaving the ground = powered descent. Throttle at the top / bottom limit: the integrator does not wind up. Tilt beyond 84°: thrust reduced to 0.
- **Failsafe** (`safety.h`): has a **cause** (LINK / SWITCH / BATTERY, output in `CoreOutput.fs_cause`). GUARD (0.5 s; ends immediately when the cause goes away; the time used only "cools down" gradually, so a flickering link still leads to a landing) → DESCENT (with altitude: controlled descent; without: a fraction of the measured hover thrust, and hover throttle is not "learned" meanwhile) → LANDED. During DESCENT: link back continuously for `fs_recover_ms` hands control back; the switch turned off ends it; battery cannot be cancelled but the pilot still keeps roll / pitch / yaw. On the ground when GUARD ends: motors off immediately. The `fs_max_descent_s` limit only counts while the craft is not descending or has no altitude. In both GUARD and DESCENT due to link loss, the old channel's throttle stick is **not** used by Altitude Hold.
- **Runaway on takeoff** (`RunawayTakeoff`, Betaflight port): PID sum ≥ 600 **together with** rotation rate > 15 °/s (roll, pitch) or 50 °/s (yaw) for 75 ms → disarm, blocker `RUNAWAY_TAKEOFF` shown until the ARM switch goes back to OFF. Deactivated (until the next power-up) after 0.5 s of normal flight has been seen: throttle ≥ 20 % with a stick ≥ 15 %, or throttle ≥ 40 %, all PID sums < 100. Not evaluated while motors are at ground idle.
- **Ground / airborne**: `airborne` = throttle has gone up (25 % immediately, or above "low" for 1 s), or Altitude Hold has lifted the craft; ends when `LandDetector` sees the craft still for 1 s with throttle at the lowest. At **ground idle** all motors are equal, I-term = 0, runaway is not evaluated.
- **CALIB**: in-flight level trim, measured from the accelerometer while hovering (`inflight_trim.h`; `FlightApp` applies it after disarm and asks the link task to write flash).
- **Airmode** (`general.airmode`): `AUTO` (default) = full stabilization at lowest throttle **only while airborne**; `ALWAYS` = from the first time throttle passes `airmode_start_throttle` until disarm (Betaflight: on the bench the motors will spin up by themselves); `OFF`.
- Position Hold: the switch exists, **no XY controller yet** (the horizontal estimator exists).

Disarm clears all PID, Angle and throttle boost state; low-throttle TPA is reloaded.

## Verified (on PC)
- `test/test_core.cpp` (296): AHRS, vertical (including lidar reference, barometer sigma) / horizontal estimator, RC, mixer against physics (3 layouts), failsafe (cause, handing back control, limits), runaway, arming, Altitude Hold (state machine, `HoverEstimator`, takeoff, giving up, nobody on the sticks, throttle at the limit, upside down).
- `test/test_control.cpp` (269): each block of the rate loop checked **against its formula**: filters, gyro filter chain, RPM filter, dynamic notch, PID and helpers, stick path (including lost frames), Angle, Stabilize, sliders, in-flight level trim.
- `test/test_nav.cpp` (130, 0.7.0): optical flow → velocity, tilt angle corrected by measured acceleration (`tilt_aid.h`), position controller, Position Hold, GPS Rescue — each block with a physical model written separately in the test; see `docs/NAV.md`.
- `test/test_flight.cpp` (251): the whole `FlightCore` with an independent 6-DOF plant on two craft (5 inch high thrust; 1 kg low thrust): roll / pitch / yaw signs, rate steps, feedforward, filters in closed loop, throttle boost, Angle vs Stabilize, mode switching, airmode and ground idle, compass, Altitude Hold (altitude loss, a table under the lidar, GPS off by 3 m, prop wash near the ground), failsafe, **irregular cycle times**, **runaway with two motor wires swapped**, handover to `MotorOutputManager`; since 0.7.0 also Position Hold (GPS + compass, flow + lidar), GPS Rescue and tilt angle corrected by measured velocity.
- Deliberate faults (mutation): more than 660 faults for the rate loop / filter / mode / Altitude Hold / safety files alone (`tools/mutate.py`).
- `test/test_fw.cpp`: the core runs **inside the real firmware** on a virtual board: data goes through the IMU (SPI), baro (I2C) drivers, the CRSF decoder, DShot frames (including ESC rpm replies) before reaching the plant.

## Not verified / limits
- Version 0.5 was bench-tested (no props) on a real board; what showed up and how it was fixed: `docs/HARDWARE_FEEDBACK_1.md`. **The time of one `update()` on the S3 for this version still has no number** (the Overview tab now shows the "compute" stage separately). How to reduce load: `docs/TUNING.md` section 9.
- Known differences from Betaflight / ArduPilot not yet addressed: `docs/HARDWARE_FEEDBACK_1.md` section 6, part "Known but not done".
- Defaults are Betaflight's numbers for a 5 inch quad; other craft must be tuned (Master Multiplier).
- Simulated plant: thrust proportional to command; horizontal motion has linear drag 0.35 /s; the accelerometer is **physically correct** (thrust + drag, no gravity in flight). A few tests on controller response shape and on blackbox use an "ideal tilt sensor" (`true_accel = false`) and state the reason. It does not replace tuning on a real craft.
- The attitude estimator with gyro + accelerometer is off by a few degrees during sustained acceleration (numbers in `docs/TUNING.md` section 5). Since 0.7.0, when there is a measured velocity (GPS + compass, or optical flow) and the craft is armed, `core/tilt_aid.h` replaces the accelerometer with a comparison of specific force against true acceleration (the ArduPilot DCM / PX4 approach): simulation gives under 1° throughout a 25° tilt held for 4 seconds. Without GPS / flow it is as before.
- Not yet: battery voltage compensation for throttle, crash-flip, thrust linearisation, I-term rotation, waypoint / mission. Position Hold, GPS Rescue, optical flow: since 0.7.0 (`docs/NAV.md`), **not yet flown for real**.

## Bug fixed (0.6.0): the attitude estimator trusted the accelerometer too much
Up to 0.5.0, the AHRS accelerometer gain was 2 /s (0.5 s time constant) and the simulated plant made the accelerometer always point exactly vertical. On a real craft that is wrong: in flight the accelerometer senses thrust, not gravity. When the plant was made physically correct (for Position Hold), the bug showed up immediately: holding half roll stick for 2 seconds in Angle, commanded 12°, the craft actually tilted 27° and swung back 14° after the stick was released.

Fixed in `core/ahrs.h` the way Betaflight does it (`imu.c`), cross-checked with ArduPilot: gain 0.25 /s when armed, × 10 when disarmed at rest, × 100 for half a second right after coming to rest; 2nd-order low-pass at 10 Hz; weight drops to 0 when the magnitude is off by 0.1 g from the at-rest value (learned while disarmed); gyro bias learned at no more than 0.5 °/s per minute. In-flight level trim switched to measuring directly from the accelerometer. Numbers and consequences for pilots: `docs/TUNING.md` sections 5 and 7. Tests: `t_ahrs_trust` (`test_core.cpp`), `t_estimator_in_flight` (`test_flight.cpp`), `t_inflight_trim` (`test_fw.cpp`).

Lesson about testing: a simulation that "idealises" a sensor hides exactly the bugs that live where that sensor is used. The plant now uses a physically correct accelerometer by default.

## Bug fixed: default quad spin direction was reversed relative to Betaflight
The previous version said "matches BF motor numbering" but the spin direction table for QUAD_X / QUAD_PLUS was the inverted one (`{+1,-1,-1,+1}`): with `props_out = false` the firmware treated motor 1 (rear-right) as spinning CCW, i.e. Betaflight's **props out** layout. Anyone mounting props per Betaflight's default diagram got a positive-feedback yaw loop (the craft spins wildly on takeoff). Cause: Betaflight flips the yaw sign twice (the yaw stick in `rc.c`, and `-scaledAxisPidYaw` in `mixer.c`), and the first reading only counted one. The old tests did not catch it because the simulated plant used the same wrong table (exactly the "simulator and code share the same misunderstanding" kind of bug).
Now: `quad_x_spin = quad_p_spin = {-1,+1,+1,-1}` (+1 = CCW prop), the mixer table **equals number for number** Betaflight's `mixerQuadX` / `mixerQuadP` (with a test comparing each coefficient), the simulated plant is a quad built the Betaflight way. HEX_X already had the right directions by position, unchanged.
**Still must be checked on the board** (props off): the GUI's Motors tab draws the direction each motor must spin; run each motor and compare.
