# MAVLink telemetry to a ground station — section `telemetry` (firmware 0.19)

Sends the craft's state to Mission Planner, QGroundControl, or a MAVLink-capable radio (433 / 915 MHz telemetry radio, ELRS backpack). Since 0.40 the firmware also **receives** commands and missions from the ground station (section "Receiving from the ground station" below); parameters are not supported yet.

Using ExpressLRS (ELRS) hardware as a telemetry radio, or an ELRS link in MAVLink mode for both control and telemetry (0.39): `docs/ELRS_MAVLINK.md`.

## Settings (Pins and ports tab, "Telemetry (MAVLink)" block)
| Field | Default | Meaning |
|---|---|---|
| `protocol` | `NONE` | `MAVLINK` = on (MAVLink 2) |
| `baud` | 57600 | UART speed; 57600 for most 433 / 915 MHz radios, 115200 or more for an ELRS backpack / direct wire |
| `tx_pin` | empty | UART TX pin, wired to the radio's RX. Empty = nothing is sent |
| `rx_pin` | empty | RX pin, wired to the radio's TX: commands and missions from the ground station (0.40). STM32 needs it to start the UART |
| `sysid` | 1 | MAVLink system id; two craft on one ground station need different ids |
| `rate_attitude` / `rate_position` / `rate_hud` / `rate_status` / `rate_rc` | 10 / 5 / 4 / 2 / 2 | times per second, 0 = not sent |
| `remote_arm` | off | lets the ground station arm over MAVLink (0.40; disarm is always allowed) |
| `mp_home_item` | off | Mission Planner: a downloaded mission has the home point on row 0 as in ArduPilot (0.40.1, section "Using with Mission Planner") |

Changes need save and restart.

**Choosing the UART** (`docs/PORTS.md`):
- `ports.assign = AUTO`: the firmware takes the UART left free after GPS, serial receiver, MSP optical flow and ESC telemetry wire, in the order UART1, UART2, UART0. UART0 is only usable when the app connects over the chip's USB.
- `TABLE`: the firmware uses the UART with function `TELEMETRY`.

**On the receiver's UART** (0.39): `receiver.protocol = MAVLINK` (ELRS in MAVLink mode) and `receiver.telemetry` on → telemetry goes out the TX pin of the receiver UART, 460800 baud, without taking any other UART. Section `telemetry` is then only used for `sysid` and the rates; `protocol`, `baud`, `tx_pin`, `rx_pin` are unused (if `tx_pin` is set, the report says `telemetry.tx_pin is not used`). Receiver without a TX pin → `telemetry: FAILED - the receiver's UART has no TX pin (pins.rx_tx)`.

The startup report has the line `telemetry: MAVLink 2, system id 1, UART0 TX GPIO43, 57600 baud` (on the receiver UART: `... on the receiver's UART (UART2 TX GPIO15, 460800 baud)`). If it cannot run there is a line `telemetry: FAILED - ...` with the reason. The TX pin is checked for clashes with other pins: the app reports it right away, the firmware blocks arming.

## What is sent
| Message | Rate | Content |
|---|---|---|
| HEARTBEAT | 1 Hz, always sent | frame type (quad / hexa / octo), armed, mode using ArduCopter numbers (Acro, Stabilize = Angle, Alt Hold, PosHold, RTL = GPS Rescue, Land = failsafe), state (standby / active / calibrating / critical) |
| ATTITUDE | `rate_attitude` | roll, pitch, yaw (rad, MAVLink convention: nose up = positive pitch), rotation rates |
| GLOBAL_POSITION_INT | `rate_position` | GPS position, MSL altitude, altitude above the takeoff point (estimator), NED velocity, heading |
| GPS_RAW_INT | `rate_position`, when there is a GPS | fix type, satellite count, HDOP, speed, course, accuracy |
| VFR_HUD | `rate_hud` | ground speed, heading, throttle %, altitude, climb rate |
| SYS_STATUS | `rate_status` | sensors present / healthy, CPU %, voltage, current, battery %, receiver bad frame count |
| BATTERY_STATUS | `rate_status`, when the battery is measured | per-cell voltage (split evenly), current, mAh, % |
| RC_CHANNELS | `rate_rc` | 16 channels, RSSI |
| STATUSTEXT | when there is text | each line of the startup report (FAILED = error, WARNING = warning); "Arming blocked: ..." on change and repeated every 10 s; "Ready to arm"; "Armed" / "Disarmed" |

## Receiving from the ground station (0.40) — `src/telemetry/mavlink_gcs.h`

Read from Betaflight's `telemetry/mavlink.c` and `telemetry/mavlink_mission.c`, rewritten. The firmware reads frames from the ground station via:
- an ELRS receiver in MAVLink mode: the receiver passes the frames unchanged into its UART (`docs/ELRS_MAVLINK.md`, method 3);
- a separate telemetry radio: the RX wire of the telemetry UART (`telemetry.rx_pin`), up to 64 bytes per pass.

Frames addressed to another craft (`target_system` not 0 and not `sysid`) or another component (not 0 and not 1) are ignored. Replies go before all periodic frames, in order (queue of 12, `Scheduler::reply`). They still wait while the ELRS receiver reports full (`txbuf` ≤ 35 %).

**Commands (`COMMAND_LONG` / `COMMAND_INT`, answered with `COMMAND_ACK`):**

| Command | What it does | Rejected when |
|---|---|---|
| `COMPONENT_ARM_DISARM` (400) | param1 1 = arm, 0 = disarm | arm: `telemetry.remote_arm` off (DENIED, default); an arming-block condition exists, including high throttle or the ARM switch being on (TEMPORARILY_REJECTED). Disarm in flight: only when param2 = 21196 (ArduPilot's force code; Betaflight always disarms) |
| `DO_SET_MODE` (176) | param1 has the `CUSTOM_MODE_ENABLED` flag, param2 = ArduCopter mode number: 1 Acro, 0 Angle, 2 Alt Hold, 16 Pos Hold, 5 Loiter (0.41), 3 Mission, 6 RTL | AUTO and RTL while disarmed; AUTO without a mission or home. 21 (Smart RTL), 9 (Land) cannot be selected: they are decided by the rescue / failsafe settings |
| `NAV_RETURN_TO_LAUNCH` (20) | same as `DO_SET_MODE` RTL | disarmed |
| `MISSION_START` (300) | same as the app's `MISSION 1` command | disarmed, no mission or home |
| `DO_SET_HOME` (179) | param1 = 1: home = current position (like the app's `SET_HOME`) | armed (the position estimate would restart); given coordinates: UNSUPPORTED (home is reset at every arm) |
| `PREFLIGHT_REBOOT_SHUTDOWN` (246) | param1 = 1: restart after 300 ms (ACK goes first) | armed; param1 = 3 (bootloader): UNSUPPORTED |
| `REQUEST_MESSAGE` (512) | `AUTOPILOT_VERSION`, `HOME_POSITION`, `GPS_GLOBAL_ORIGIN`, `AVAILABLE_MODES` (param2 = 0: all, 1..8: one), `AVAILABLE_MODES_MONITOR` | other messages: UNSUPPORTED; no home yet: FAILED |
| `REQUEST_AUTOPILOT_CAPABILITIES` (520) | sends `AUTOPILOT_VERSION`: MAVLink 2, `MISSION_INT`, `COMMAND_INT`, firmware version | |

Other commands: UNSUPPORTED. `TIMESYNC` (tc1 = 0) is answered with the board's time (ns).

**Mode chosen by the ground station** (`FlightCore::set_mode_override`): the flight mode bits (Angle, Stabilize, Alt Hold, Pos Hold, Rescue, Mission) **replace** the pilot's switches until the pilot flips a flight mode switch (ARM switch, beeper, blackbox... do not count) or the ground station picks another mode. Betaflight differs: it ORs the ground station's mode onto the switches and keeps it forever. HEARTBEAT, OSD and the Receiver tab all show the mode actually being flown.

**Arming from the ground station** (`Arming::remote_arm`): every arming-block condition still applies. An ARM switch that is off does not disarm the craft. The pilot takes control back with the next flip of ARM to on (on a live link); after that, flipping it down disarms as usual. Losing the link and getting it back does not count as a flip.

**Mode list (`AVAILABLE_MODES`):** Acro (1), Angle (0), Alt Hold (2), Pos Hold (16), Mission (3), RTL (6), Smart RTL (21, not selectable), Land (9, not selectable), Loiter (5, 0.41, index 9). Each mode carries a MAVLink "standard mode" so QGroundControl understands it. The mode numbers follow ArduCopter because the ELRS TX module names modes for the transmitter by these numbers.

**Mission** (like `mavlink_mission.c`):
- **Upload:** `MISSION_COUNT` → the firmware requests each item with `MISSION_REQUEST_INT` → `MISSION_ITEM_INT` → `MISSION_ACK`.
  - The old mission stays until the last item arrives.
  - No item → request again after 1.5 s; after 5 misses, `OPERATION_CANCELLED`.
  - Wrong order: `INVALID_SEQUENCE`. Mission being flown: `DENIED`. More than 32 commands: `NO_SPACE`.
  - Only accepted from the ground station that started the upload.
- **Download:** `MISSION_REQUEST_LIST` → `MISSION_COUNT` → `MISSION_REQUEST_INT` / `MISSION_ITEM_INT` (re-requesting an earlier item is fine).
- **Other:** `MISSION_CLEAR_ALL`. `MISSION_SET_CURRENT` only accepts the current mission command, since a mission always starts from the first command. `MISSION_CURRENT` is sent at 1 Hz; `MISSION_ITEM_REACHED` is sent when moving to the next command.
- **Fence and rally:** empty lists.
- **Item 0 is the first command** (like PX4 / Betaflight), not home as in ArduPilot.
- **Accepted commands:**

  | MAVLink command | Section `mission` |
  |---|---|
  | `NAV_TAKEOFF` | altitude |
  | `NAV_WAYPOINT` | point, altitude, hold seconds = param1 |
  | `NAV_LOITER_TIME` | point, altitude, seconds = param1 |
  | `NAV_LAND` | point |
  | `NAV_RETURN_TO_LAUNCH` | |
  | `NAV_DELAY` | seconds = param1; -1 (time of day) not accepted |
  | `DO_SET_SERVO` | servo 1..4, 500..2500 µs |
  | `DO_CHANGE_SPEED` | m/s = param2, stored as dm/s |

  Other commands: `UNSUPPORTED`.
- **Altitude:** frame `GLOBAL_RELATIVE_ALT[_INT]` is relative to home. Frame `GLOBAL[_INT]` (above mean sea level) is converted to relative to home, so it needs a home.
- **Where it is stored:** an uploaded mission is written to section `mission`, the same mission the app shows and edits. The firmware saves it to flash by itself once disarmed, then sends the log line `settings: section mission written by the firmware`. When the app sees that line it reloads the section and reports "A ground station just put a new plan into the drone". The Mission tab shows the new plan automatically if the plan on the map has no unsaved edits.

**Also sent:** `HOME_POSITION` and `GPS_GLOBAL_ORIGIN` (when home is set or changed, then every 10 s), `MISSION_CURRENT` (1 Hz). HEARTBEAT reports rescue by the way it is flying: RTL (6), Smart RTL (21), Land (9).

Not yet: parameters (`PARAM_*`, Mission Planner times out when loading parameters), `SET_MESSAGE_INTERVAL`, fence, arm / mode commands via `SET_MODE` (old message, id 11).

## Tuning from the ground station: parameters (0.43)
The tuning is exposed as MAVLink parameters. Use Mission Planner (Config -> Full Parameter List) or QGroundControl (Vehicle Setup -> Parameters).

- **Protocol** (`mavlink_gcs.h`):
  - `PARAM_REQUEST_LIST` sends every `PARAM_VALUE`, one at a time, only when the replies before it have left and after 10 ms plus the link's slowdown. A burst overflows a radio's buffer: ExpressLRS flushes a full one and every value in it is lost. ArduPilot paces its list by the link's free space for the same reason.
  - `PARAM_REQUEST_READ` works by index, or by name with index -1.
  - `PARAM_SET` stores the value and answers with `PARAM_VALUE` carrying what was really stored (clamped to the setting's range). A NaN is not stored.
  - An unknown name gets no answer, as in ArduPilot.
  - Every value is a `REAL32`, in the units the app shows.
- **Which parameters:** the sections `pid`, `rates` and `filters` (114 parameters), from `src/proto/wbp_params.h`, which `tools/gen_proto.py` writes from the schema.
  - The PID gains are named per axis: `PID_ROLL_P`, `PID_PITCH_I`, `PID_YAW_D`, `PID_ROLL_DMAX`, `PID_ROLL_F` ...
  - The others are the field names with a section prefix (`PID_`, `RATE_`, `FLT_`), shortened to MAVLink's 16 characters: `PID_TPA_RATE`, `PID_ANG_STR`, `RATE_MAX_RATE_R`, `FLT_G_LPF1_HZ`, `FLT_DN_Q` ...
- **Applying and saving:** a set value is in use at once (the sections are live) and is written to the flash once the craft is disarmed. A flash write stalls both cores for milliseconds, so it is never done in flight.
- **Sliders:** a PID gain set by hand turns the PID tuning sliders off (`slider_mode` OFF), and a gyro / D-term filter frequency set by hand turns its filter slider off. Otherwise the sliders would compute the old value again, at once and at every boot. Betaflight Configurator does the same.
- **Verified:**
  - `test_gcs` (list, read by name and by index, set, clamped echo, NaN, unknown names, other systems / components);
  - `test_mavlink` (the four messages against pymavlink's bytes);
  - `test_fw` group `params` (the whole list over the ExpressLRS MAVLink link, `PID_ROLL_P` in use at once and kept in the flash, a value past its range clamped);
  - 14 mutants `params:`.
  - **Not tried with a real Mission Planner / QGroundControl yet.**

## Using with Mission Planner (0.40.1)

Read from the Mission Planner source (`GCSViews/FlightPlanner.cs`, `ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs`, `Common.cs`), **not yet tried with a real Mission Planner**:
- Windburst reports `MAV_AUTOPILOT_GENERIC` of type quad. Mission Planner then keeps its default ArduCopter type, so it shows the **correct mode names** for the ArduCopter numbers Windburst sends. Set Mode (Actions) sends `DO_SET_MODE`. Stabilize, Acro, AltHold, PosHold, Auto, RTL work; Loiter, Guided... are rejected.
- **Uploading a mission (Write WPs):** Mission Planner only inserts home at row 0 for ArduPilot. With Windburst, row 0 is the first command, which matches the firmware.
- **Downloading a mission (Read WPs):** Mission Planner **always drops row 0** because it treats it as home. If row 0 has coordinates, it also asks "Reset Home to loaded coords". With `telemetry.mp_home_item` on (app: "Mission Planner: home as the first mission item") the firmware sends the home point at row 0 as ArduPilot does (command `NAV_WAYPOINT`, frame `GLOBAL`, altitude above mean sea level; no home yet → coordinates 0). `MISSION_CURRENT` / `MISSION_ITEM_REACHED` then count that row too: the first command is 1, matching the row number Mission Planner shows. The upload direction does not change. **Turn it off when using QGroundControl:** QGroundControl handles row 0 like PX4, so it would see an extra point.
- **`MISSION_INT`:** on connect, Mission Planner requests `AUTOPILOT_VERSION`. With the `MISSION_INT` flag it uses `MISSION_ITEM_INT` / `MISSION_REQUEST_INT`; if it gets no answer (lost on the link) it uses the float (degrees) `MISSION_ITEM` / `MISSION_REQUEST`. Since 0.40.1 the firmware understands and answers the float form too: coordinates are converted through `double`, off by a few mm at most. It answers with `MISSION_ITEM` when asked with `MISSION_REQUEST`.
- **Parameters:** none. Mission Planner will report it could not load parameters; ignore it.

**Steps:**
1. Connect Mission Planner to the link (ELRS TX module USB 460800, WiFi backpack UDP 14550, or the telemetry radio's COM port). Turn on `mp_home_item`.
2. Flight Plan: set Alt to **Relative**. The first row is `TAKEOFF`, then `WAYPOINT` / `LOITER_TIME` / `LAND` / `DELAY` / `DO_SET_SERVO` / `DO_CHANGE_SPEED` / `RETURN_TO_LAUNCH`. Press Write WPs; Read WPs to check.
3. Arm with the switch (or the Arm button if `remote_arm` is on), then Actions → Auto or Mission Start.

**Betaflight:** Betaflight has a guide for connecting Mission Planner over ELRS MAVLink (`betaflight.com`, MAVLinkELRS), but for telemetry only. Betaflight uses its own mode numbers (Acro 0, Angle 1...) and reports autopilot BETAFLIGHT, while Mission Planner reads mode numbers as ArduCopter ones, so the mode names will be wrong.

## How it runs
- The sensor thread calls `mav_send()` every pass (~1 ms). A frame is only written when the UART has room for the whole frame (`uart_room()`). Frames are never cut and it never waits; a frame that does not fit waits for the next pass and gives back its sequence number, so the ground station sees no lost frames.
- When it falls behind it does not send a burst: after 3 s without room, each type sends one more frame and then returns to its normal rate.
- **Throttling by `RADIO_STATUS`** (0.39, only when telemetry goes over the MAVLink receiver's UART): each `RADIO_STATUS.txbuf` (% of the ELRS receiver's buffer still free, 100 times per second) is fed to `Scheduler::radio_status()`. Like Betaflight: `txbuf` ≤ 35 → nothing is sent. Like ArduPilot (`handle_radio_status`): `txbuf` < 20 → each group's period gets 60 ms longer, < 50 → 20 ms longer (up to about 2 s), > 95 → 40 ms shorter (when above 200 ms), > 90 → 20 ms shorter. HEARTBEAT stays at 1 Hz and STATUSTEXT is not slowed down (ArduPilot sends them outside the streams). Telemetry on its own UART is not slowed by the receiver's `RADIO_STATUS`.
- Up to 0.38.3 it could also send over WiFi (UDP broadcast port 14550). WiFi was dropped in 0.38.4 (the ESP32-S3 does not have enough internal RAM for the WiFi driver); the `telemetry.wifi` byte is kept as `reserved_t`.
- Text longer than 50 characters is split into several STATUSTEXT chunks with the same `id`. A line is at most 150 characters; the queue holds 6 lines, when full the oldest is dropped. One chunk is sent every 100 ms.

Code: `src/telemetry/mavlink.h` (framing, CRC, 9 sent + 2 read messages, parser), `src/telemetry/mavlink_telemetry.h` (send schedule, text), `FlightApp::mav_in()` / `mav_send()` in `src/app/flight_app.h`.

## Verification
- `test_mavlink` (76 checks): frames compared byte by byte with pymavlink (`tools/mavlink_vectors.py` → `test/mavlink_vectors.h`, 15 vectors, including `RC_CHANNELS_OVERRIDE` and `RADIO_STATUS` as ELRS sends them), parser, reading the two ELRS messages, rates, slow UART (no cut frames, contiguous sequence numbers), no bursts, throttling by `txbuf` (each ArduPilot step, the 35 % threshold, HEARTBEAT and text not slowed), text splitting, full queue, HEARTBEAT by mode.
- `test_rx` MAVLink group: decoder (channels, 0 / 65535, clamping, LQ / RSSI / `txbuf`, bad frames, other messages queued for the ground station).
- 0.40: `test_mavlink` adds 18 ground-station messages compared byte by byte with pymavlink (33 vectors) and read back, the reply queue, `HOME_POSITION`, `MISSION_CURRENT`, Smart RTL / Land. `test_gcs` (87): mission protocol (upload, download, order, re-request, give up, in flight, out of space, frame, bad values, another sender cutting in, clear, current, reached, fence / rally) and each command with a fake craft. `test_core` / `test_flight`: remote arm / disarm, mode override until a switch is flipped. `test_fw` group `mavlink gcs`: ground station over the ELRS link and over a telemetry radio; upload a mission then fly all of it with `DO_SET_MODE AUTO`, save to flash, reload, modes, arm, home, restart.
- `test_fw` group `mavlink`: virtual board, values taken from the craft model (attitude, GPS, altitude while flying Alt Hold), startup report as STATUSTEXT, slow UART, settings (baud, sysid, rate 0, hexa), port table, no TX pin, clashing pins.
- `test_fw` group `mavlink receiver`: virtual board with an ELRS receiver in MAVLink mode (1 kB buffer like ELRS, drained at the packet rate's capacity, bytes lost when full): 16 channels, 100 frames/s, LQ, arm and fly, signal loss; telemetry through the buffer at 2375 B/s and at 360 B/s (250Hz: no byte lost, HEARTBEAT still 1 Hz); `receiver.telemetry` off; separate telemetry not slowed; both on together; no TX pin.
- 0.40.1: `test_gcs` adds the float form (`MISSION_ITEM` / `MISSION_REQUEST`) and the home row for Mission Planner (100 checks); `test_fw` uploads with `MISSION_ITEM` over the ELRS link, downloads with home first, `MISSION_CURRENT` counts that row.
- `test_app` (UART choice), mutants `mav:` (30), `mavrx:` (30), `gcs:` (52) in `tools/mutate.py`.
- App: "Telemetry (MAVLink)" block in the Pins and ports tab; `test_gui_browser.py::test_telemetry_block` (not run: no playwright on this machine).
- **Not yet tried on a real board**, with Mission Planner / QGroundControl, or with a real radio.
