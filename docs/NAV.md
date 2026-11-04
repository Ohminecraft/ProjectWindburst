# Position: optical flow, position estimation, Position Hold, GPS Rescue (firmware 0.7.0)

**None of this has run on real hardware yet.** Everything in this document is verified on the PC: unit tests per block, the real firmware on a virtual board (sensors simulated at register / frame level, a simulated aircraft with wind), deliberate faults, and a build for ESP32-S3. Section 8 lists what has to be tried on the board, in order.

The **Position** tab in the app collects every setting and status for this part.

## 1. What hardware you need

| You want | You need |
|---|---|
| Indoor / low-altitude Position Hold | an optical flow sensor **and** a lidar (measures distance to the floor) |
| Outdoor Position Hold | a GPS with a fix **and** a calibrated compass |
| GPS Rescue (fly home by itself) | a GPS fix at arming (the home point), a calibrated compass, an altitude estimate (barometer) |

GPS without a compass **cannot** hold position: the craft does not know where north is relative to the nose, and pushing in the wrong direction makes it fly away. The firmware refuses rather than guesses (blockers `POSHOLD_NO_GPS`, `RESCUE_NO_COMPASS`).

### Supported optical flow sensors

| Type (`flow.type`) | Wiring | Lidar | Notes |
|---|---|---|---|
| `MSP` — MicoAir MTF-01 / MTF-02, Matek 3901-L0X (MSP v2 protocol) | UART, only needs **sensor TX → pin `flow_rx`** | built into the sensor | the easiest option; put the sensor in MSP mode, 115200 baud |
| `PMW3901` | a **dedicated** SPI (the chip's second SPI bus): `flow_cs`, `flow_sck`, `flow_miso`, `flow_mosi` | needs a separate lidar (VL53L1X / VL53L0X on the I2C bus) | cannot be used together with an SPI-mode SD card (same bus): switch the card to SDMMC |

Why the PMW3901 does not share the IMU bus: each read holds the bus for about 0.3 ms (the datasheet requires delays between bytes); on the IMU bus that would block a 250 µs control cycle.

### Which port a UART sensor gets

The firmware picks a free port by itself, in this order: **UART1** if GPS is not enabled → **UART2** if no serial receiver is used → **UART0** if the app connects through the chip's own USB (env `rx_s3_usb`; the default UART0 RX on ESP32-S3 is GPIO44). A board that connects the app through a USB-UART bridge and has both a GPS and a serial receiver has no port left: the boot report says so explicitly with `flow: FAILED - no UART left ...`.

Your board (N16R8, app over USB): GPS on UART1, CRSF on UART2, the MSP sensor on **GPIO44** (or any other pin you like; ESP32 UARTs can be routed to any pin).

## 2. First-time setup (Position tab)

1. *Pins and ports* tab, "Optical flow" group: fill in the pins. *Position* tab: pick the "Sensor type". **Save and reboot** (sensor type and baud are only read at boot).
2. The "Firmware reports" line must say `MSP flow on UART0` (or `PMW3901 on its own SPI bus`). "Samples received" should be about 100 samples/second. With the craft lying on the desk, "Latest sample" says *skipped: too close to the floor* — correct, below 10 cm the sensor cannot focus.
3. **Check the mounting orientation** (no need to arm, props off): hold the craft level, 0.5 – 1 m above the floor, over a textured floor with enough light.
   - move the craft toward the nose → the arrow on the circle points **up**;
   - move it to the right → the arrow points **right**;
   - off by a quarter / half turn → change "Sensor mounting orientation"; left and right swapped → enable "Image is mirrored". Both take effect immediately, no reboot needed;
   - just rock the craft in place → the arrow stays almost still (the rotation part is subtracted using the gyro). If the arrow follows the rocking: the orientation is still wrong, or the speed scale is far off.
4. "Speed scale" (0.42.1: **automatic calibration**, the "Optical flow scale calibration" block):
   - hold the craft 0.3 – 1 m above a textured floor, press **Start**;
   - tilt the craft back and forth in both roll and pitch for about 10 seconds, keeping the sensor in one spot (rotate around the sensor, do not move the craft);
   - press **Done**. When only rotating, the image moves only because the sensor rotates, so flow must equal gyro. The firmware computes per axis `Σ(flow × gyro) / Σ(flow²)` (least squares, like INAV's `opflow` calibration and ArduPilot's `AP_OpticalFlow_Calibrator`), averages the two axes into `nav.flow_scale` and uses it immediately. Press **Save** to keep it;
   - on an error it keeps collecting: *too few samples* (tilt more; each axis needs 20 samples rotating faster than 0.3 rad/s), *X / Y axis reads reversed* (fix orientation / mirroring first; the scale is left unchanged), *the two axes differ by more than 30 %* (the craft was moved, not just rotated);
   - why this is needed: nobody agrees on the scale of these sensors. PMW3901: ArduPilot uses 1.26 mrad/count, PX4 uses 1/385 ≈ 2.6 mrad/count. MTF-01 over MSP: Betaflight divides by 200 (measured empirically), INAV effectively divides by about 600, ArduPilot flips the sign.
5. *Modes* tab: assign a switch to **Position Hold** (and **GPS Rescue** if used). Position Hold turns on Angle by itself; it is best combined with Altitude Hold.
6. Fly-home failsafe: *Position* tab, the "On signal loss or Failsafe switch" box → *GPS Rescue*.

## 3. What Position Hold does

Written after ArduCopter's `mode_poshold.cpp`, one state machine per axis (roll, pitch):

| State | Meaning |
|---|---|
| pilot in control | the stick commands a tilt angle as in Angle, plus the learned tilt to counter wind |
| braking | stick just released: tilt against the motion, the faster it moves the more it tilts (`ph_brake_rate`, `ph_brake_angle`) |
| braked, waiting for the other axis | this axis has stopped |
| transitioning to position hold | both axes have stopped: 1.5 seconds blending from braking to the position controller |
| holding position | the position controller holds the point; the tilt angle it needs while the craft stands still is exactly the wind |
| handing back to the pilot | stick just moved: 0.5 seconds blending from the controller to the stick |

The position controller is the `AC_PosControl` cascade: position error → velocity (P, with a square-root limit) → velocity error → acceleration (PID, 5 Hz filter) → tilt angle. Default gains are ArduCopter's (`pos_p` 1.0, `vel_p` 2.0, `vel_i` 1.0, `vel_d` 0.25).

Altitude and yaw are not touched: throttle (or Altitude Hold) and the yaw stick stay with the pilot.

**When the position source is lost** (flow sees nothing for more than 0.3 seconds, GPS lost for more than 2 seconds): Position Hold releases by itself, the status says "switch is on but there is nothing to hold position with", and the craft flies like Angle. When the source returns it re-engages by itself (starting with braking).

**Arming with the Position Hold switch already on** is only allowed when there is something to hold with: a healthy flow sensor **with a lidar**, or a GPS fix + a calibrated compass.

### Loiter (0.41) — `src/core/nav/loiter.h`

Written after ArduCopter's `mode_loiter.cpp` and `AC_WPNav/AC_Loiter.cpp`. This is a second way to hold position, different from Position Hold in that **the stick commands acceleration, not tilt angle**, and the position controller always runs:

| | Position Hold | Loiter |
|---|---|---|
| Roll / pitch stick | tilt angle as in Angle | desired acceleration (the tilt angle the stick would give in Angle, at most 2/3 of the angle limit = 20° by default, converted to acceleration) |
| Stick released | fixed-style braking, stops before holding the point | desired velocity decays with "drag", after 1 s extra braking kicks in (at most 2.5 m/s², jerk 5 m/s³) |
| Speed | up to the pilot | full stick = `loiter_speed` (default 12.5 m/s like `LOIT_SPEED_MS`), half stick gives about half (through the rate curve as in Angle) |
| Altitude | follows the Altitude Hold switch | always holds altitude (the LOITER switch turns on Angle + Altitude Hold by itself) |
| Suits | manual flying, FPV | beginners, filming, positioning |

**How it runs** (`AC_Loiter::calc_desired_velocity`):
1. Stick → desired tilt angle → desired acceleration (north / east according to the nose heading), fed straight into the controller (feed-forward).
2. The tilt angle the craft will reach is predicted with a 0.15 s first-order filter (`ATC_INPUT_TC`; ArduPilot uses the attitude controller's command model), then integrated into the desired velocity.
3. Drag: the desired velocity loses (acceleration at maximum angle) × speed / `loiter_speed`, so at full stick it settles at `loiter_speed`.
4. Stick released for more than 1 s → braking: `sqrt_controller(speed, vel_p / 2, jerk)`, at most 2.5 m/s², ramped with a maximum jerk of 5 m/s³.
5. The desired velocity is integrated into the target point. Target point, velocity and acceleration go straight into `PosController` (`set_pva`, like `set_pos_vel_accel_NE_m`); its integrator learns the wind.

**Fence** (`fence_radius`, shared with Position Hold), the way `AC_Avoid` limits velocity: when moving outward the velocity never exceeds what still allows stopping before a line 1 m inside the fence (`sqrt_controller` of the remaining distance), and the target point never crosses that line. On the simulator, charging at full stick into a 20 m fence stops at 19.0 m, without crossing.

**Enabled together with Position Hold:** Loiter takes priority. The "hold position" failsafe still uses Position Hold. Without a position source it behaves like Position Hold: arming is blocked (`POSHOLD_NO_GPS`), and in flight it falls back to Angle + Altitude Hold.

**Different from ArduPilot, on purpose:** the stick goes through the rate curve just like Windburst's Angle and Position Hold, so the same stick position gives the same tilt angle in all three modes (ArduPilot reads the stick linearly). The predicted angle uses a first-order filter. The fence is Position Hold's circle; there is no full `AC_Avoid` yet.

**Ground station:** ArduCopter mode 5 (LOITER). HEARTBEAT reports 5. `DO_SET_MODE` 5 enables it (held until the mode switch is moved). `AVAILABLE_MODES` has "Loiter" at index 9, standard mode `POSITION_HOLD`; Position Hold changes to `NON_STANDARD`, as in ArduCopter. OSD and CRSF show `LOIT`.

## 4. GPS Rescue

Since 0.36 rescue flies like ArduPilot and PX4. The code was rewritten after reading their sources: PX4 `navigator/rtl.cpp` (`computeReturnAltitude`) and `rtl_direct.cpp`; ArduCopter `mode_rtl.cpp`, `mode_smart_rtl.cpp`, `mode_land.cpp` and `libraries/AP_SmartRTL`. The **"Rescue type"** box (`nav.rescue_mode`) has 3 options.

**Return home (RTL)**, the default:

1. **Climb** straight up to the return altitude, holding still at the *stopping point*. If the craft is flying fast, this point lies ahead, where the craft can brake in time at 2 m/s² (ArduPilot `build_path`); the craft is not yanked backwards. The last few metres of the climb slow down so it does not overshoot.
   - Return altitude, **Cone** type (default, PX4 `RTL_CONE_ANG`): equal to "Return altitude" (`RTL_RETURN_ALT`, default 30 m).
     - If closer to home than "Cone distance" (`RTL_MIN_DIST`), the craft only climbs to distance / tan(cone angle). With a 45° angle, 10 m away it climbs to 10 m.
     - Never lower than 2 times the acceptance radius (4 m).
     - Cone angle = 0: always climb to "Return altitude".
   - The old types remain: Fixed, Highest point of the flight + extra climb, Current altitude + extra climb.
   - If the craft is already higher than the return altitude it keeps its altitude and does not descend first (except with the Fixed type).
2. **Fly home** in a straight line at the return altitude.
   - A *target point* runs along the path: it accelerates and then decelerates at 2 m/s² and stops exactly above home. The position controller tracks this point, using its velocity as feed-forward (like `AC_WPNav`).
   - If the craft falls more than 5 m behind (for example in a headwind), the target point waits for it.
   - The nose turns gradually toward the direction of travel, at most 90°/s (PX4 `MPC_YAW_MODE` 0). There is no longer a "turn the nose in place" phase as in Betaflight.
3. **Descend above home** to "Loiter altitude above home" (`RTL_DESCEND_ALT`, default 10 m) if higher. High up it descends fast, up to 3 times the "Descent speed".
4. **Wait** for "Loiter time above home" (`RTL_LAND_DELAY`, default 5 s), holding position. Set to 255 and the craft waits forever and does not land by itself.
5. **Land** straight down, holding position above home. Below "Slow descent altitude" it descends at "Landing speed" (ArduPilot `LAND_SPEED`, default 0.5 m/s). On touchdown the motors stop and the craft disarms.

**Return along the flown path (SmartRTL)**, after ArduPilot SmartRTL:
- From arming, the firmware records the flight path: one point every 2 m, at most 128 points.
  - Loops are pruned: when a new segment passes closer than 0.99 × 2 m to an old segment, the whole loop in between is replaced by one point.
  - When memory is full, the path is simplified with Ramer-Douglas-Peucker with a 1 m tolerance.
- During rescue, the craft flies that path in reverse: around the places it went around, at the altitudes it flew, without climbing first.
  - The last point is home, at 2 m.
  - At turns the craft slows down: the change in velocity is limited to 1 second of acceleration, and the craft looks far enough ahead to decelerate in time.
  - At home it waits and lands as in RTL.
- It falls back to straight-line return (RTL) when:
  - the path is too tangled and memory stays full for 15 s (`SMARTRTL_TIMEOUT`);
  - position is lost for 15 s in flight;
  - or at the start it is closer to home than "Cone distance".

**Land in place (LAND)**, after ArduPilot LAND:
- Descends right at the stopping point, holding position with GPS.
- Without GPS or compass it still descends, but does not hold position (`nogps_run`); a pilot on the sticks can push the craft.
- Only needs an altitude estimate.

**OSD** (MSP DisplayPort) while rescue is flying:
- The mode field shows `RTL`, `SRTL` or `LAND` (during failsafe it is still `!FS!`).
- The warning line shows the metres still to fly home, measured along the path: `RTL 123M` (for SmartRTL this is the path flown, not the straight-line distance).
- Above home this line shows `RTL HOME`, while landing it shows `RTL LAND`.
- The LAND type shows the remaining altitude, for example `LAND 12M`.
- The app's Position tab has a "Distance left to home" line (field `rescue_left` of the `nav` stream).

**When it runs**: the GPS Rescue switch is on, or failsafe (signal loss / Failsafe switch) if the failsafe procedure is Rescue.

**Taking back control**
- Rescue from the switch: turn the switch off.
- Rescue from failsafe: after the signal returns, **move the roll or pitch stick more than 30 %**. If the signal returns but nobody touches the sticks, the craft keeps flying home (the pilot may not be ready yet).

**Rescue gives up** (phase "giving up" + reason) when: GPS is lost, compass heading is lost, the altitude estimate is lost, there are fewer satellites than "Minimum satellites" for 15 seconds, it climbs for 10 seconds without gaining height, or it flies for 20 seconds without getting closer to home. Then:
- from failsafe → land in place like a normal failsafe (nobody is flying);
- from the switch → hand the craft back to the pilot (Angle + Altitude Hold).

**Battery below the minimum always lands in place**, it does not fly home on a drained battery.

**Arming with Rescue configured** (a Rescue switch or failsafe = Rescue): the firmware requires a GPS fix (blocker `RESCUE_NO_FIX`, because without a fix there is no home point) and a compass (`RESCUE_NO_COMPASS`). The "Allow arming without GPS fix" box drops the first requirement; failsafe will then land in place.

## 5. How the estimator uses flow and GPS

- **Flow → velocity** (`core/flow.h`): `v_tiến = +khoảng cách × (flow_y − gyro_y)`, `v_phải = −khoảng cách × (flow_x − gyro_x)` (ArduPilot's convention; forward velocity = +distance × …, rightward velocity = −distance × …). The gyro is averaged over **exactly the time window** the flow sample describes (the MSP sensor reports about 45 ms late for a 40 ms window; the firmware keeps 0.25 seconds of gyro history to cut out exactly that window). A sample is dropped (not treated as "zero velocity") when: surface quality is low, it is outside the height range 0.1 m … "Maximum height for optical flow", tilt exceeds 45°, or the rotation is faster than 200 °/s.
- **Body-frame velocity → north / east** (0.42.1, `flow_to_earth`): flow measures velocity in the **body** plane, not the ground plane. When the craft is tilted, part of the vertical velocity shows up in it, and the horizontal velocity reads short (multiplied by the cosine of the tilt). The firmware takes the vertical velocity from the altitude estimator to compute the z component of the body velocity, then rotates the whole vector with the full attitude matrix. ArduPilot EKF3 (`FuseOptFlow`: `relVelSensor = Tnb × velocity`) and PX4 EKF2 (`predictFlow`) do the inverse: predict flow from the 3D velocity. Before 0.42.1 the firmware only rotated by the nose heading: nose down 20° while climbing at 1 m/s read 0.34 m/s of horizontal velocity that did not exist (test_flight measured 0.35, now below 0.08).
- **Sensor limits** (0.42.1): flow above 4.2 rad/s (before gyro subtraction) is dropped, reason `RATE`. The image smears so the speed reads lower than reality. The 4.2 rad/s threshold comes from ArduPilot's `writeOptFlowMeas`; PX4 uses 5 rad/s for the PMW3901. Flow noise is now 0.15 rad/s at the best quality (PX4's `EKF2_OF_N_MIN`; ArduPilot uses 0.25).
- **PMW3901** (0.42.1): the delta is only taken when the MOT bit is set (like ArduPilot `AP_OpticalFlow_Pixart`). A delta above 240 counts per 10 ms is a corrupt burst (the chip only goes up to about 60), so that whole read is dropped (PX4 treats > 240 as garbage).
- **Sample accumulation** (`hal/flow/flow_driver.h`, `Accumulator`): a single sample from these sensors is very coarse — the MSP sensor counts in 5 mrad steps, the PMW3901 in 1.26 mrad steps; at 100 samples/second that is 0.5 and 0.13 rad/s (half a metre per second at one metre height). The driver sums about 5 samples (50 ms) into one reading: both the quantisation error and the timing error between two samples shrink 5 times. The estimator gets 20 readings per second. (ArduPilot sums PixArt sensors over 100 ms for the same reason.)
- **Horizontal Kalman** (`core/horizontal_est.h`): flow enters as a velocity measurement, GPS as position + velocity. With flow only: velocity is correct, position is only relative (not anchored to the map) — enough to hold a spot. While there are no measurements, the velocity uncertainty grows by itself (only prediction remains), so the first measurement after an outage is trusted almost fully and always passes the gate: no "ghost velocity" after flow is lost and comes back (Betaflight has hit this). The first draft had an extra "restart after 2 seconds" step; deliberate faults showed it changed nothing, so it was removed.
- **Altitude and vertical velocity from GPS** (0.8.3, the "Altitude from GPS" frame of the Position tab): see `docs/GPS.md`, section "GPS altitude in Altitude Hold".
- **GPS without a compass**: position is still displayed but not used for control; and if flow is running, GPS is not blended in (flow velocity is in the craft's axes, which do not match GPS north when the nose heading is unknown).
- **Tilt correction from measured acceleration** (`core/tilt_aid.h`, the "Correct tilt angle…" box): in flight the accelerometer measures thrust, not gravity; every time the craft accelerates or brakes, the estimated tilt is off by a few degrees. With a measured velocity (GPS or flow) the true acceleration is known and can be corrected (the `AP_AHRS_DCM` approach). Only runs while armed.

## 6. Protocol and log

- Section `flow` (id 16, read at boot): `type`, `baud`. Section `nav` (id 17, takes effect immediately): flow orientation / mirroring / scale / thresholds, Position Hold, Rescue, failsafe procedure.
- Stream `nav` (id 13): flags (`POSHOLD`, `POSHOLD_UNAVAILABLE`, `RESCUE`, `RESCUE_AVAILABLE`, `NAV_LEAN`), flow quality and sample-rejection reason, samples/second, measured flow velocity (0.25-second average), Position Hold state of both axes, tilt angle requested by the autopilot, the point being held, wind compensation, Rescue phase and give-up reason, distance to home, return altitude.
- Stream `estimate` gains three bits: `VEL` (ground velocity available), `FLOW` (flow in use), `TILT_AIDED`; 0.8.3 adds `GPS_ALT` (GPS altitude fed into the estimate within the last 2 s) and `GPS_VZ` (GPS vertical velocity). Section `nav` in 0.8.3 adds `gps_alt` (enum `OFF` / `FALLBACK` / `BLEND`, default `FALLBACK`) and `gps_vel_z` (on); old `nav` records (27 bytes) can still be read, the two new fields take their defaults.
- Mode `RESCUE`; blockers `RESCUE_NO_FIX`, `RESCUE_NO_COMPASS`; health `FLOW`; `range_model` gains `MSP`; 6 new pins in section `pins` (0.6 pins records still load, the new pins are left empty).
- **Blackbox**: new group **"Position and navigation"** (`NAV`, on by default, 10 times per second): `pos_n/e`, `vel_n/e`, `est_valid`, `nav_flags`, `flow_quality`, `flow_reject`, `flow_fwd/right`, `nav_roll/pitch`, `nav_target_n/e`, `rescue_phase`. `pos_*` and `vel_*` used to be in the GPS group. Blackbox settings saved since 0.6 do not have this group: tick the "Position and navigation" box again in the Blackbox tab. The log viewer has a "Position" chart group.
- CLI: `python -m wbclient monitor nav,estimate`.

## 7. Trying it on the simulator

`build/wb_sim` comes with both flow sensors (MTF-01 on UART0, PMW3901 on the second SPI bus, pins filled in). The control port (app port + 1) has these extra commands:

```
carry <z> <vn> <ve>    hold the craft level, z metres above the floor, moving at vn / ve (north / east)      carry off   put it down
wind <n> <e>           wind (m/s)
flow quality <0..255>  surface quality reported by the MSP sensor        flow dark 0|1   PMW3901 sees nothing
flow plug 0|1          unplug / plug the MSP sensor                      flow mount <deg> <mirror 0|1> <scale>   how the sensor sits on the craft
gps fix 0|1            GPS loses / has a fix                             gps dead 0|1    GPS module stops sending
```

`state` additionally prints `pn pe vn ve` (true position and velocity of the simulated craft).

## 8. Trying it on the real board, in order

Remove the props for steps 1 – 4.

1. **Boot report**: the `flow:` and `range:` lines match section 2. If it says `flow: MSP flow on UARTx` but "Samples received" is 0: wrong pin, wrong baud, or the sensor is not in MSP mode.
2. **Sample count and quality**: about 100 samples/second (MTF-01); quality over a textured floor. Write down the quality figures over a few kinds of floor — the default threshold of 30 is a guess.
3. **Orientation and scale** (section 2, steps 3 – 4). **This is the step most likely to go wrong**: the axis orientation of the real sensor has not been verified; the default follows ArduPilot.
4. **Rocking in place**: the arrow must stay almost still. If not: report back with a log (groups "Position and navigation" + "Gyro"); the 45 ms delay of your sensor may be different.
5. **CPU load**: Overview tab, "Sensor thread, longest pass" and "Actual control cycle" before and after enabling flow. The PMW3901 adds about 0.3 ms every 10 ms to the sensor thread; the MSP sensor is almost negligible.
6. **Flying**: Angle + Altitude Hold first, low (1 m), indoors with no wind or outdoors in calm air. Enable Position Hold while hovering. Keep a hand ready on the switch. The craft drifting steadily in one direction and then darting off = wrong orientation (switch off immediately). The craft rocking back and forth in place = P gain too high for your craft: lower `vel_p`.
7. **GPS**: outdoors, compass calibrated, the Position tab says "Nose heading: from compass" and the 3D model on Overview turns the same way as the real craft. If the compass is off by 90°, GPS Position Hold will fly in widening circles ("toilet bowl"): switch off immediately and fix "Compass chip orientation".
8. **Rescue**: test with the switch first, at a distance of 30 – 50 m, at 10 m altitude, in an open area. Only once the switch works correctly set failsafe = Rescue and try switching off the transmitter.

After each flight step: send the log (recorded to flash or SD card, with the default groups).

## 9. Verified on the PC

- `test_nav` (130): flow → velocity, gyro history, late-arriving velocity in the horizontal estimator, `TiltAid` (including time-window slicing down to the millisecond), the position controller, the Position Hold state machine (braking, wind learned only when stationary, hand-over), Rescue phases and give-up reasons.
- `test_flowsensor` (94): the MSP v2 frame parser (CRC, broken frames, noise), the MSP driver (scaling, accumulation, late / bunched frames, lidar), the PMW3901 driver with register-level simulation (identification, init table, delays, accumulation, broken data line), the `FlowSensor` layer, UART selection and lidar selection.
- `test_flight` (251): closed loop with the simulated craft — Position Hold with GPS + compass (two craft sizes) and with flow + lidar, wind, rotating in place, loss of position source, silent lidar, GPS without compass next to flow; full Rescue sequence (from failsafe and from the switch, with and without Altitude Hold), starting in a crosswind, failure types (compass, GPS, satellites, battery).
- `test_fw` (real firmware on the virtual board): MSP sensor on UART0 / UART1, PMW3901 on its own bus, UART selection, orientation / mirroring / scale, position-hold flight with each sensor, unplugging the sensor in flight, broken PMW3901 data line, GPS Position Hold with the nose not pointing north, Rescue from failsafe and from the switch, signal returning midway, GPS dying midway, drained battery, the blockers, section `nav` taking effect immediately, the new log group.
- App: Python client (`test_e2e`: flow + Position Hold, Rescue flying home), the real page in a browser (`test_position_tab`: from "no sensor selected" to position-hold flight, the orientation-check circle).

## 10. Not verified / assumptions

- **Axis orientation and scale of the real sensors** (both types). The simulation is written from ArduPilot / Betaflight documentation, independently of the driver, but not a single byte from a real sensor has been seen yet.
- The MSP scale `motion / 200 / dt` and the 45 ms delay + 40 ms window come from Betaflight's MTF-01 driver; the MTF-02 and Matek may differ.
- The PMW3901 init table is copied from ArduPilot; it has not been loaded into a real chip. The delays between bytes use `delayMicroseconds`.
- UART0 as the sensor port on the USB variant: the ESP32 ROM prints its boot log on UART0's TX pin at power-on; the sensor only connects RX so it should not matter, but this is untested.
- Quality threshold 30, flow noise 0.10 rad/s, the 200 °/s limit: values chosen from documentation, not measured.
- The Position Hold gains are ArduCopter's defaults for a ~1 kg craft; a light 5-inch craft with high thrust may need a lower `vel_p`. In simulation both sizes are stable.
- Touchdown detection during Rescue shares Altitude Hold's detector; behaviour on sloped ground or tall grass is unknown.
- Execution time of the navigation part within the control cycle on ESP32-S3 has not been measured (it runs at the estimator rate, 500 Hz, and sits in RAM like the rest of the cycle, except the library's `atanf`). Look at the `t_est` column of the log before and after enabling Position Hold.
- Deliberate faults (mutation): more than 440 deliberate faults for this part alone (firmware) and 41 for the Position tab; every remaining fault is caught by the tests.

## 11. Flight log 2026-10-11 (firmware 0.8.7) and what 0.8.8 fixes

Log `flight_20261011_211839_s2_s0` (loop set to 6000 Hz, GPS NMEA, `gps_alt` BLEND). Position Hold oscillated continuously and drifted ≈ 15 m.

| Seen in the log | Cause | 0.8.8 |
|---|---|---|
| The GPS solution changes every **1.02 s**; each time the position jumps up to 1.8 m, velocity 2.5 m/s; estimated velocity deviates from raw GPS by ~1.5 m/s; the Position Hold tilt angle hits ±15° | The NMEA module was left at its default **1 Hz**. The controller answers every jump with a tilt. Not a tuning issue | The firmware **requests a solution rate** when `gps.auto_config` is on: `$PMTK220`, `$PCAS02`, UBX CFG-RATE (u-blox in NMEA mode), limited by baud (9600: stays at 1 Hz; 19200: 2; 38400: 5; ≥ 57600: 10). Stream `gps.rate` shows the measured rate (Sensors tab). **GPS-only Position Hold needs ≥ `nav.ph_min_gps_hz` (5 Hz)**, otherwise it refuses, flag `POSHOLD_SLOW_GPS`, and the Position tab states the reason. Optical flow is not affected |
| Compass \|B\| changes 0.50 → 0.40 … 0.58 with throttle; `YAW_FROM_MAG` lost from 6.5 s to 15 s (Position Hold has no heading) | Magnetic interference from motor current / battery leads right next to the compass | Hardware: move the compass away from power wiring (GPS mast), twist the battery leads. Current-based compensation (ArduPilot's compassmot) later, needs a log with current |
| At disarm (40.6 s) `est_vz` jumps to ≈ −4 m/s and then `est_z` drifts to **−290 m** while baro stays ≈ 0; the whole second flight (51 … 115 s) has no `ALT` flag, Altitude Hold does not run | The estimate ran far off, every baro measurement was rejected by the 5σ gate, nothing could pull it back; the uncertainty takes minutes to grow on its own | **Reset to baro** after 1.5 s in a row without accepting any baro measurement (like ArduPilot EKF's ResetHeight): z = baro − offset, v = 0; counted in `estimate.z_resets`. **On disarm the vertical velocity is pulled to 0** (zero-velocity update, like ArduPilot on the ground). `test_core`: kicked to −4 m/s then 3 s without baro → reset, error < 0.6 m after 2 s |
| Loop: set to 6000, averaged 4273 Hz; `t_lock` median 236 µs | The flight task waits for the sensor thread to release the lock (the slow telemetry part holds the lock in one long stretch) | Split into short pieces (docs/FIRMWARE.md). Remaining: the IMU SPI read |

### How Betaflight and ArduPilot do Position Hold (for comparison)
- **Betaflight** (master 10/2026: `flight/position_estimator.c`, `pos_hold_multirotor.c`, `autopilot_multirotor.c`): a 3-state Kalman (position, velocity, acceleration) per axis, taking GPS position + velocity (R from hAcc / sAcc), optical flow, IMU acceleration; **requests 10 Hz GPS by default** (`gps_update_rate_hz = 10`, u-blox). One solution is reused at every estimator step within its interval.
- **ArduPilot** (EKF3 + `AC_PosControl`, Loiter / PosHold modes): a 24-state EKF, GPS delay compensation with a time buffer, innovation gates, position / altitude reset when rejected for too long; GPS should be ≥ 5 Hz (u-blox default 5 Hz, `GPS_RATE_MS = 200`).
- **Windburst**: a linear Kalman per axis for horizontal position / velocity with GPS delay compensation (`horizontal_est.h`), tilt aid, a 4-state Kalman for altitude (z, v, accel bias, baro offset). Common to all three: **no filter can make 1 Hz GPS fly smoothly** - solutions arrive too sparsely compared with how fast the craft drifts. That is why 0.8.8 fixes the source (GPS rate) and blocks when the source is too slow, without changing the filter.

### Is an "EKF" needed for Position Hold (todo ★★★★)
The horizontal estimator **already is** a Kalman filter (fusing GPS + accelerometer, with delay). A full nonlinear EKF in the style of EKF3 (attitude + position + velocity + biases + magnetic field in one filter) is a very large job (ArduPilot: tens of thousands of lines, years of test flights); the ESP32-S3 CPU can run it at 100 - 200 Hz on core 1 but it **cannot be verified without a lot of real flying**. This log shows no fault that an EKF would fix better: the faults are 1 Hz GPS, compass interference, and an altitude estimate that does not recover by itself - all three handled directly above. Suggestion: fly again with GPS ≥ 5 Hz and send the log; only if it still oscillates then is the filter worth discussing.

**0.9.0**: the EKF now exists, running in parallel (SHADOW), see `docs/EKF.md`. Replaying the 2026-10-11 log through it: attitude matches the AHRS within 1–2°, velocity error with 1 Hz GPS is almost the same as the old estimator (GPS is the limit), altitude does not drift.
