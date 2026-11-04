# Flight missions (waypoints) — firmware 0.37

The drone flies a loaded plan by itself: takes off, flies through points, at one point holds still and runs a servo, at another lands, stops the motors and waits for someone to pick up or attach a load, then takes off by itself and continues, finally returns home and lands. It works like **ArduCopter's AUTO mode**: the real code was read (`ArduCopter/mode_auto.cpp`, `libraries/AP_Mission/AP_Mission.cpp`, `ArduCopter/events.cpp`, `AP_ServoRelayEvents`) and then rewritten, more compactly.

**Not yet flown on a real board.** Verified on PC: unit tests of the runner, closed-loop simulation (`test_flight`), and the real firmware on a virtual board (`test_fw`, group `mission`).

Design: `docs/superpowers/specs/2026-10-29-mission-design.md`. Decisions made while you were away: `docs/MISSION_WORKLOG.md`.

## Done (part 1: firmware)
- Stores a plan of up to **32 commands** in the settings (section `mission`, saved to flash like every setting), uploaded and read over WBP: USB / UART or the simulator.
- Started with the **MISSION switch** (Modes tab) or the **WBP command `MISSION`** (`1` = start, `0` = stop).
- Mission servo: the `Mission` channel in the Motors tab, with the angle at minimum / maximum pulse and the initial pulse.
- Beeper and motors beep fast during the **last 10 seconds** before the drone takes off again by itself. Cannot be turned off.
- OSD and MAVLink report AUTO mode; the `nav` stream carries the mission state.

## The "Mission" tab in the app (part 2)
Chosen layout C: the map fills the whole tab, with floating panels on top.

- **Map:** satellite imagery (Esri World Imagery) or a street map (OpenStreetMap). On the satellite imagery there is a layer of street, ward and place names like Google's satellite mode (Esri `Reference/World_Transportation` and `Reference/World_Boundaries_and_Places`, transparent PNG images); the "Labels" button turns this layer on / off, on by default. Google tiles are not used the way Betaflight Configurator does (`mt1.google.com/vt`): that is not a public API, and Google's terms forbid both fetching tiles directly and storing them. Map tiles go through the `wbclient` server (`/api/tile`, `wbclient/maptiles.py`) and are stored in `~/Windburst/tiles`. Places viewed once can be opened later without a network; places not yet fetched stay blank.
- **Address search:** type and press Enter (OpenStreetMap Nominatim), click a result and the map jumps there.
- **My position:** the map jumps to where the laptop is, drawing a blue dot and an accuracy circle.
  - The position comes from the Windows location service (the machine's GPS if any, otherwise by scanning nearby Wi-Fi networks). The first time, Windows asks for permission.
  - The app window (`desktop.py`) lets the page ask Windows directly, without an extra WebView2 permission prompt. When opened in a browser, the browser asks first.
  - Blocked or not found: the page says what needs to be turned on, with a button that opens Windows Settings > Privacy & security > Location (`/api/location-settings`).
- **Drawing the plan:**
  - Click on the map to add a point, drag a point to move it.
  - Click a point to open its edit panel: choose **Fly to** (hold N seconds), **Hold · servo** (hold N seconds, then step by step: servo 1–4 to an angle in degrees, wait M seconds), or **Land · wait · take off** (wait N seconds on the ground, take off again to the point's altitude).
  - The distance in metres is shown on each leg.
- **General options:** takeoff altitude, speed (0 = rescue speed), return home and land after the last point, on signal loss continue or failsafe.
- **Bottom line:** what the drone is doing (command number out of how many), distance left to the point, wait time left, altitude, battery, distance from home, and the whole trip (metres, estimated time, commands / 32). Over 32 commands this number turns red and uploading is refused.
- **Drone on the map:** a red arrow following GPS and nose heading, the red trail is where it has flown (since arming).
- **Buttons:**
  - **Upload to the craft:** writes section `mission` and saves to flash. Must be disarmed.
  - **Download from the craft:** reads back the plan stored in the drone.
  - **Delete all points** (options panel): asks for confirmation then deletes every point on the map. The plan in the drone stays until the next Upload.
  - **Start / Stop:** sends the `MISSION` command. The drone must be armed with the switch first.
- Degrees are converted to pulses using the servo's "angle at minimum / maximum pulse" in the Motors tab.
- Code: `web/js/tabs/mission.js` (UI, Leaflet 1.9.4 bundled in `web/vendor/leaflet`, BSD-2 licence), `web/js/mission_plan.js` (converting the plan to and from section `mission`, distance estimate).

**Over MAVLink (0.40, part 3):** Mission Planner / QGroundControl upload and download the plan, start it (`MISSION_START` or AUTO mode), follow the command being flown (`MISSION_CURRENT`, `MISSION_ITEM_REACHED`) over an ELRS link in MAVLink mode or a telemetry radio (`docs/MAVLINK.md`, `docs/ELRS_MAVLINK.md`). A plan uploaded from a ground station lives in the same section `mission` as the app's plan; the app reloads it automatically when the firmware saves it (once disarmed), and the Mission tab shows it right away if the plan on the map has no unsaved edits. The app still connects to the drone by wire (USB / UART), so to follow it in flight use a ground station.

## Commands
| Command | `MAV_CMD` | Uses | What the drone does |
|---|---|---|---|
| TAKEOFF | 22 | `alt` | Climbs straight up to `alt` (relative to home) where it stands. Altitude Hold takes off by itself, no throttle stick needed |
| WAYPOINT | 16 | `lat`, `lon`, `alt`, `p` = hold seconds | Flies to the point and stops right over it, holds `p` seconds |
| LOITER_TIME | 19 | `p` = seconds (`lat` = `lon` = 0: at the point just reached) | Holds still for `p` seconds |
| LAND | 21 | `lat`, `lon` (0 = where it is) | Flies to the point, keeping its altitude, then lands. On touchdown it **stops the motors** |
| DELAY | 93 | `p` = seconds | Waits. After LAND it waits on the ground with the motors stopped |
| RTL | 20 | — | Flies home and lands (uses the 0.36 rescue RTL) |
| SERVO | 183 | `arg` = servo 1..4, `p` = pulse µs | Sets a servo (DO_SET_SERVO) |
| SPEED | 178 | `p` = dm/s | Horizontal speed for the following legs (DO_CHANGE_SPEED) |

Altitude is relative to where it first armed. Coordinates are in 1e-7 degrees. SERVO / SPEED commands (DO commands) run **when the flight command before them starts**, as in `AP_Mission::update`. So a "hold then run a servo" step is written as:

```
WAYPOINT P          fly to P
LOITER_TIME 3 s     at P: ...
SERVO 2 -> 1800 us  ... the servo moves immediately, then wait 3 s
LOITER_TIME 2 s
SERVO 2 -> 1100 us  second step, then wait 2 s
```

A delivery point is `LAND P`, `DELAY 60`, `TAKEOFF 12`.

Default speed, acceleration, the tracking leash, the altitude where slow descent starts and the descent speed are shared with rescue (Position tab): flies at `rescue_ground_speed`, accelerates / decelerates at 2 m/s², below `rescue_landing_alt` descends at `rescue_land_speed`. A point is reached when the horizontal distance is under 2 m and the altitude difference under 1 m (like ArduPilot's `WPNAV_RADIUS`). The drone's nose turns to the direction of flight.

## How to fly
1. Write the plan (section `mission`), press Save.
2. Put the drone in the field, wait for a GPS fix.
3. **Arm with the ARM switch** as usual. The first arm is always done by a person.
4. Turn on the **MISSION** switch, or send `MISSION 1` from the app: the drone takes off and flies by itself.
   - On the ground, if the first flight command is not TAKEOFF the mission is refused (`REFUSED`, like ArduCopter's "Missing Takeoff Cmd").
   - A MISSION switch already on before arming is also refused: flip it off and on again.
5. Taking control back: turn off the MISSION switch, or send `MISSION 0`. The drone immediately returns to the modes the pilot has selected.

## At a delivery point (different from ArduPilot, on purpose)
ArduPilot (`MIS_OPTIONS` CONTINUE_AFTER_LAND) keeps the motors spinning at minimum throttle while waiting. Windburst **stops the motors completely** because people come close:

- After touchdown the motors stop and the `DELAY` time is counted.
- In the **last 10 seconds** the beeper beeps fast, the motors (DShot) beep, the OSD shows `TAKEOFF 7S`.
- When the time is up the drone re-arms by itself and takes off. This arm skips three conditions: throttle stick not low, ARM switch on, RC signal lost. Every other condition still blocks: tilt, battery, IMU...
- **Flipping the ARM switch to OFF while waiting stops the drone from taking off by itself** (the mission ends).
- The home point and altitude reference are kept from the first arm, so the plan does not drift even if the delivery spot is higher or lower than home.
- If there is no TAKEOFF after LAND, the mission ends on the ground (like ArduCopter's `verify_land`).

## When things go wrong
| What happens | What the drone does |
|---|---|
| RC signal lost | Continues the mission (ArduPilot `FS_OPTIONS` continue in AUTO). Choose `mission.rc_lost = RTL` to have failsafe run as usual |
| Low / critical battery | Battery failsafe as before (RTL or LAND per the Position tab). The mission stops |
| GPS position lost for more than 1 s in flight | Abandons the mission (`FAILED`), lands in place (rescue LAND) |
| Pilot disarms | The mission stops |
| Out of commands in flight | Holds still at the last point (like ArduCopter's `exit_mission`) |
| Out of commands on the ground | Stops the motors once |

## App, OSD, MAVLink
- Modes tab: mode **Mission (AUTO)**. Motors tab: servo channel **Mission**, with the angle at minimum / maximum pulse (default 0° and 180°) and the initial pulse (default 1500 µs). Battery and beeper tab: condition **Mission about to take off by itself**.
- Stream `nav`: `mission_state`, `mission_index`, `mission_count`, `mission_dist` (m to the current point), `mission_left` (m left in the whole trip), `mission_wait` (seconds of waiting left).
- OSD: mode `AUTO`, warning lines `AUTO 3/9 120M`, `WAIT 45S`, `TAKEOFF 7S`. While the mission is returning home the OSD shows the rescue display (`RTL 123M`).
- MAVLink HEARTBEAT: `custom_mode` AUTO (3), during RTL it is RTL (6) (Smart RTL 21 / Land 9 per the rescue type). Since 0.40: plan upload / download, `MISSION_CURRENT`, `MISSION_ITEM_REACHED`, `MISSION_START`.

## Code
- `src/core/nav/mission.h`: the runner (portable). Flies leg by leg like the rescue flight path: the target point runs along the leg from where the drone can stop, then it brakes to stop right over the point.
- `src/core/flight_core.h`: mission on/off, priority (rescue > mission > Position Hold), arming/stopping motors at the delivery point (`Arming::mission_arm` / `mission_disarm`, `safety.h`), automatic takeoff (`AltInput::auto_takeoff`, `nav/alt_control.h`), servo.
- `src/proto/settings_apply.h`: `mission_plan_from()` converts lat/lon to metres from home using GPS's own `gps::offset_cm`.
- `src/app/flight_app.h`: the WBP `MISSION` command, keeping home and the baro reference while waiting, servo, beeper, telemetry, OSD.

## Verification (on PC)
- `test_mission` (41): each command, DO/NAV order, refusals, keeping home from the moment the motors stop, countdown, position loss, RTL, SPEED.
- `test_flight` (`t_mission`): a whole delivery trip in simulation. Lands 0 m from the delivery point, waits 14.9 s (set to 15), beeper exactly 10 s, returns to 0.2 m from home, maximum tilt 25°. Additional cases: signal lost mid-leg still continues; `rc_lost = RTL` gives normal failsafe; turning the switch off takes the craft back; flipping ARM to OFF while waiting prevents the automatic takeoff; critical battery gives a failsafe landing; a plan starting with WAYPOINT on the ground is refused; start and stop from the app.
- `test_fw` (group `mission`, 14): lat/lon plan in the settings, virtual GPS. Servo at the initial pulse then up to 1800 µs over the point, lands less than 2.5 m from the delivery point, waits exactly 12 s, beeper 80/80 ms in the last 10 s. The home distance in telemetry matches the real distance both while waiting and after the second arm. RTL returns exactly to the takeoff spot. The `MISSION` command is refused while disarmed and accepted when armed.
- Unit: `test_nav_ctl` (automatic takeoff), `test_core` (arm/disarm by the mission), `test_nav` (rescue in the style the caller specifies), `test_proto` (schema, lat/lon conversion, beeper flag), `test_osd`, `test_mavlink`.

## To try on the board (see `todo.txt`)
Try step by step, the switch first, close to home, with someone holding the transmitter:

1. Assign the MISSION switch. Plan: TAKEOFF 5 m then one WAYPOINT 10 m away. Arm, turn on the switch: the drone must climb to 5 m by itself, fly to the point, hold still. Turn off the switch: your Altitude Hold / Position Hold must take the craft back immediately.
2. Add LOITER_TIME + SERVO: the servo must move while the drone holds over the point.
3. Add `LAND` + `DELAY 30` + `TAKEOFF`: the drone must land, stop the motors, beep fast for the last 10 s, then take off by itself. Also try: flip ARM to OFF while waiting and the drone must stay put.
4. Add RTL at the end: the drone must return to where it first armed, not to the delivery point.

Turn on the blackbox group **"Navigation debug data"** when testing, and send the log if the drone misses a point by more than 3 m or wobbles during the automatic takeoff.
