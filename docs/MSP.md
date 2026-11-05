# MSP: peripherals and configuration tools (0.44)

Checked on PC: unit tests and firmware running on the virtual board (`test_fw`). **Not tried with real devices, not tried with a real Betaflight App / SpeedyBee.**

MSP (Betaflight's MultiWii Serial Protocol) is used in two places:
1. **MSP port:** a UART that MSP devices push data into.
2. **App port:** Betaflight App, the SpeedyBee app and ESC tools talk to the firmware over this port.

## 1. MSP port: peripherals

Follows ArduPilot (`SERIALn_PROTOCOL = MSP`, `AP_MSP`) and INAV's MSP sensors. One module can carry several sensors, e.g. an MSP GPS with a compass, or an MTF-01 with flow and lidar. So all MSP devices share **one port**, and the firmware routes each frame to the right place (`src/hal/msp_sensors.h`).

| Frame | Command | Used when | Notes |
|---|---|---|---|
| Lidar | `0x1F01` MSP2_SENSOR_RANGEFINDER | MSP optical flow | as before |
| Optical flow | `0x1F02` MSP2_SENSOR_OPTIC_FLOW | `flow.type = MSP` | as before |
| GPS | `0x1F03` MSP2_SENSOR_GPS | `gps.provider = MSP` | 52 bytes. Fix type 2 = 2D, 3 and above = 3D (like INAV / ArduPilot). Accuracy cm -> mm, course 0.01° -> 0.1° |
| Compass | `0x1F04` MSP2_SENSOR_COMPASS | `sensors.mag_model = MSP` (**new**) | milligauss, module axes forward / right / down. Module orientation set with `mag_align` like the DroneCAN compass |
| Barometer | `0x1F05` MSP2_SENSOR_BARO | `sensors.baro_model = MSP` (**new**) | Pa (float), 0.01 °C. Pressure outside 10–120 kPa is dropped |
| Receiver | `200` MSP_SET_RAW_RC (v1 or v2) | `receiver.protocol = MSP` (**new**) | up to 18 channels, 16 kept. Channels not sent keep their old value. No frame for 300 ms counts as **signal loss**, failsafe like any other receiver |

- **Pins and speed:** uses the `flow_rx` / `flow_tx` pins and `flow.baud` (default 115200). Only RX is needed, wired to the device's TX.
- **UART selection:**
  - AUTO: a free UART. UART1 if GPS does not use a UART (MSP GPS needs no UART of its own); UART2 if there is no serial receiver; UART0 when the app connects over the chip's USB.
  - TABLE: the UART with the `FLOW_MSP` function. The app now calls this function "MSP port".
- **The port only opens when something needs it:** MSP flow, MSP GPS, MSP compass, MSP barometer, or MSP receiver.
- **Boot report:**
  - opened: `msp: MSP port on UART1 RX GPIO44 TX -, 115200 baud, for: GPS compass barometer receiver`;
  - could not open: `msp: FAILED - <reason>`, and the items using this port (gps / rx / flow) report the same reason.
- **MSP barometer:** no second barometer runs.
- The **parser** also reads MSP v1 frames (`$M<`), because some devices send MSP_SET_RAW_RC as v1.

## 2. Configuration tools on the app port

Section **`msp`** (System tab, "Other tools over MSP" block):

| Field | Default | Meaning |
|---|---|---|
| `identity` | `BETAFLIGHT` | Answers as Betaflight master: `BTFL`, version 2026.12.0, API 1.49. The Betaflight App, SpeedyBee and the ESC tools only work with firmware they know. `WINDBURST`: reports `WBRX` and the real version; those tools refuse it. BOARD_INFO always carries the board code `WBRX` |
| `writes` | on | Lets those tools edit the configuration and save. Off makes it read-only. Never allows edits while armed |

### 2.1 Connecting (Betaflight App)

- The App's connect sequence is answered in master's layouts: API_VERSION, FC_VARIANT, FC_VERSION (with the version text "2026.12.0"), BUILD_INFO (with the build options that make the App show its tabs), BOARD_INFO (gyro rate, "accelerometer not calibrated" problem), STATUS / STATUS_EX, UID, NAME, MSP2_GET_TEXT, SET_RTC.
- The video transmitter / goggles on the OSD's UART get FC_VERSION without the version text (3 bytes, the length they expect).
- **Arming held off:** while the App is connected it sends MSP_SET_ARMING_DISABLED; the firmware then refuses to arm with the `CONFIGURATOR` blocker. The hold ends when the App sends 0 (disconnect), or after 3 s without any MSP request (cable pulled). Already armed: nothing is cut in the air; only the next arming is held off.

### 2.2 Reads

- **Live state:** RC (105: in function order - roll, pitch, yaw, throttle, then AUX1.. - whatever the channel map, as Betaflight's), RAW_IMU (102: in Betaflight's frame x forward, y left, z up - lying flat reads +1 g on z; acc x2048 / g, gyro x4.1 / deg/s, mag x1090 / G), RAW_GPS (106: with pdop / hdop / vdop and satellites), COMP_GPS (107), ATTITUDE (108: 0.1 deg; positive pitch is nose down), ALTITUDE (109), ANALOG (110), BATTERY_STATE (130).
- **Configuration and Setup tabs:**

| Command | What it shows |
|---|---|
| FEATURE_CONFIG (36) | RX_PPM / RX_SERIAL / RX_PARALLEL_PWM / RX_MSP / RX_SPI (nRF24) from `receiver.protocol`; GPS, RANGEFINDER from `sensors.*_enabled`; AIRMODE when `general.airmode` is not OFF; 3D from `motors.mode3d`; TELEMETRY from `receiver.telemetry`; LED_STRIP when `led.count` is not 0; OSD when `osd.type` is not NONE; ESC_SENSOR from `motors.esc_sensor` |
| MIXER_CONFIG (42) | the frame (`motors.layout`) as Betaflight's mixer: QUADX, QUADP, HEX6X, HEX6, HEX6H, Y6, Y4, OCTOX8, OCTOFLATP, OCTOFLATX, VTAIL4, ATAIL4, QUADX1234; props reversed = `props_out` |
| MOTOR_3D_CONFIG (124) | `deadband3d_low`, `deadband3d_high`, `neutral3d` (us) |
| RC_DEADBAND (125) | `receiver.deadband`, `yaw_deadband`, `deadband3d_throttle` (us; Position Hold's deadband: 0, none here) |
| RX_CONFIG (44) | serial receiver: SPEKTRUM2048 1, SBUS 2, IBUS 7, CRSF 9, SPEKTRUM1024 15, MAVLINK 16 (0 for PPM / PWM / MSP / nRF24); airmode start = `airmode_start_throttle`; RC smoothing on / off and its two factors; the stick thresholds maxcheck / midrc / mincheck = `receiver.max_check`, `mid_rc`, `min_check` |
| ARMING_CONFIG (61) | small angle = `max_arm_angle`; auto disarm 5 s (no such setting here) |
| BEEPER_CONFIG (184) | `beeper.conditions`, `dshot_beacon_tone`, `motor_conditions` in Betaflight's beeper numbering. Betaflight beepers this firmware does not have are shown off |
| SENSOR_CONFIG (96) | what is set up: accelerometer on; barometer on / NONE; compass NONE or its model in Betaflight's list (HMC5883, AK8963, QMC5883L, QMC5883P, DroneCAN; others DEFAULT); rangefinder / flow sensor on (255: a sensor not in Betaflight's list) or NONE; pitot NONE |
| MSP2_SENSOR_CONFIG_ACTIVE (0x300A) | what runs now: gyro, accelerometer, barometer, compass, rangefinder, flow sensor (found or NONE) |
| MSP2_GYRO_SENSOR_ACTIVE (0x300D) | the two gyro places: the first and the second IMU found or not |
| PID_ADVANCED (94) | feedforward (F per axis, transition, averaging, smoothing, boost, max rate limit, jitter), I-term relax (type, cutoff), anti-gravity gain (0 when `pid.anti_gravity` is off), D max (per axis, gain, advance), throttle boost, acro trainer limit, angle limit (`rates.angle_limit`), motor output limit, dynamic idle, TPA (mode, rate, breakpoint: % of the throttle sent as us, 1000 + % x 10). Betaflight settings this firmware has not are 0 |
| BATTERY_CONFIG (32) | cell min / max / warning (0.1 V, then 0.01 V as Betaflight's battery profile), capacity, voltage meter (NONE, ADC, ESC), current meter (NONE, ADC, ESC) |
| VOLTAGE_METERS (128), CURRENT_METERS (129) | battery 1 (id 10): the voltage now (0.1 V); mAh used, the current (mA; negative: 0) |
| VOLTAGE_METER_CONFIG (56) | one ADC divider: `battery.divider` as Betaflight's scale / divider value / multiplier (11.00 = 110 / 10 / 1; above 25.5: scale / 1 / 1) |
| CURRENT_METER_CONFIG (40) | one ADC sensor: `current_scale` (0.1 mV per A), `current_offset` (mA) - Betaflight's own units; no virtual meter |
| FAILSAFE_CONFIG (75) | guard time `fs_guard_ms` (0.1 s), landing time `fs_max_descent_s`, throttle `fs_throttle` (us), procedure from `nav.fs_procedure` (DROP 0, LAND 1, RESCUE 2; HOLD is said LAND). Switch mode and throttle-low delay: Betaflight's defaults |
| RXFAIL_CONFIG (77), RSSI_CONFIG (50) | 16 channels: sticks AUTO, the AUX channels HOLD (the last value); no RSSI channel |
| GPS_RESCUE (135) | 26 bytes: angle (`nav.ph_angle_max`), return altitude, descent distance, ground speed (cm/s), min sats, ascend / descend rate (cm/s), arming without a fix, altitude mode (MAX, FIXED, CURRENT; CONE is said MAX), min start distance, initial climb. Autopilot throttles: Betaflight's, the hover one measured |
| SIMPLIFIED_TUNING (140) | the PID sliders (mode, master, pitch D ratio, I, D, PI, D max, FF, pitch PI) and the D-term / gyro filter sliders (on, multiplier, their cutoffs) |
| CALCULATE_SIMPLIFIED_PID / GYRO / DTERM (142..144) | what the sliders sent would give, worked out by the firmware's own code (`core/simplified_tuning.h`); nothing changes |
| VALIDATE_SIMPLIFIED_TUNING (145) | whether the stored gains / gyro cutoffs / D-term cutoffs are still the sliders' |
| OSD_CONFIG (84) | master's 261 bytes, sent as a v1 jumbo frame: the 32 elements at Betaflight's item numbers (position and shown, converted from `osd.pos`; Betaflight items this firmware has not: 0), the statistics this firmware shows (`osd.stats`), timers / warnings / alarms: Betaflight's defaults |
| OSD_CANVAS (189) | the grid from `osd.canvas` (AUTO and HD 53 x 20, HDZero 50 x 18, SD 30 x 16) |
| MSP2_COMMON_SERIAL_CONFIG (0x1009), CF_SERIAL_CONFIG (54) | UART0..2 as Betaflight's ports 50..52, no functions (as master: they are assigned through the CLI), 115200 |
| GPS_CONFIG (132) | `gps.provider` (NMEA, UBLOX, MSP, VIRTUAL, DRONECAN - Betaflight's table), `sbas`, `auto_config`, `auto_baud`, `set_home_once`, `use_galileo` |
| GPSSVINFO (164) | the satellites the module reports: constellation, number, quality + used, C/N0 (as Betaflight shows a u-blox NAV-SAT) |
| FILTER_CONFIG (92) | gyro LPF 1 / 2 (type, static, dynamic min / max), gyro notches 1 / 2, D-term LPF 1 / 2 (with the dynamic expo), D-term notch, yaw lowpass, dynamic notch (count, Q, min / max), RPM filter (harmonics, min Hz, fade range, Q, the three weights) |
| ADVANCED_CONFIG (90) | the real `pid_denom`, motor protocol (DShot1200 is shown as DShot600, DroneCAN as DShot300), `pulse_hz`, motor idle |
| BOARD_ALIGNMENT_CONFIG (38) | how the whole IMU is mounted: the chip orientation `board_align` and `board_trim_*` together, as Betaflight's angles in whole degrees (roll -179..180, yaw 0..359 clockwise). A chip turned 90 deg reads yaw 90 |
| SENSOR_ALIGNMENT (126) | gyro / acc CW0 (their orientation is inside BOARD_ALIGNMENT), compass = `mag_align`, gyros in use = `gyro_use`, compass angles = `mag_trim_*` |
| COMPASS_CONFIG (133) | `mag_declination` (0.1 deg) |

- Also: NAME (10), MODE_RANGES (34) / MODE_RANGES_EXTRA (238), RX_MAP (64), RC_TUNING (111, rates type **Actual**, 3), PID (112), BOXNAMES (116), PIDNAMES (117), ACC_TRIM (240), BLACKBOX_CONFIG (80), DATAFLASH_SUMMARY (70), SDCARD_SUMMARY (79).
- Unsupported commands return `!`; the App skips them and carries on.

### 2.3 Writes

Writes go through the same steps as the app's Save on a section: clamp values, apply rules, apply immediately or report that a reboot is needed. EEPROM_WRITE (250) saves; REBOOT (68) replies first, then reboots.

| Command | What it changes |
|---|---|
| SET_FEATURE_CONFIG (37) | GPS, rangefinder, airmode (off -> OFF; on while OFF -> AUTO; ALWAYS stays ALWAYS), 3D (`motors.mode3d`), TELEMETRY, ESC_SENSOR, LED_STRIP (off: `led.count` 0; on with no strip: 8 LEDs - set the real length in the Windburst app), OSD (off: NONE; on: MSP DisplayPort), the receiver kind (PPM, PWM, MSP, nRF24). RX_SERIAL from a receiver that is not serial: the serial receiver is chosen by the next SET_RX_CONFIG. Other feature bits are not kept |
| SET_RX_CONFIG (45) | the serial receiver (only when the receiver is serial; a receiver this firmware does not have, e.g. SUMD, is refused), the stick thresholds, the airmode start throttle, RC smoothing. Refused when mincheck >= maxcheck (as Betaflight) |
| SET_ARMING_CONFIG (62) | `max_arm_angle` |
| SET_BEEPER_CONFIG (185) | `beeper.conditions`, `dshot_beacon_tone` (1..5), `motor_conditions`. ARM_REFUSED and MISSION_WARN (no such beepers in Betaflight) are kept as they are |
| SET_ADVANCED_CONFIG (91) | `pid_denom`, motor protocol (DISABLED refused), `pulse_hz`, motor idle. A value read and written back changes nothing (DShot1200 stays DShot1200) |
| SET_BOARD_ALIGNMENT_CONFIG (39) | Betaflight's angles of the whole mounting -> a chip orientation (`board_align`) and `board_trim_*` that give the same rotation, the smallest trims first (roll / pitch within +-45 deg). Refused when none fits (a board standing on its edge). The App's board alignment wizard works with it. Angles written back as read keep the tenths |
| SET_MIXER_CONFIG (43) | `motors.layout`, `props_out` (the frame takes effect after Save and a restart). HEX6X is Betaflight's motor order (HEX_X_BF); a value read and written back keeps the layout |
| SET_MOTOR_3D_CONFIG (217) | `deadband3d_low`, `deadband3d_high`, `neutral3d` |
| SET_RC_DEADBAND (218) | `receiver.deadband`, `yaw_deadband`, `deadband3d_throttle` (Position Hold's deadband is not kept) |
| SET_SENSOR_ALIGNMENT (220) | `mag_align` (DEFAULT and CUSTOM keep it), `gyro_use`, `mag_trim_*`. The gyro's alignment is not written (Betaflight master neither): set `board_align` in the Windburst app |
| SET_COMPASS_CONFIG (224) | `mag_declination` |
| SET_NAME (11), MSP2_SET_TEXT | `osd.name` |
| SET_MODE_RANGE (35), SET_RX_MAP (65) | mode switches, channel map. The 3D switch is Betaflight's box 29 "3D DISABLE" |
| SET_RC_TUNING (204) | rates; only Actual rates are accepted, no guessing at a conversion |
| SET_PID (202) | PIDs; different values turn the tuning sliders off (`pid.slider_mode = OFF`), like the App. Writing back the values just read changes nothing |
| SET_PID_ADVANCED (95) | what PID_ADVANCED shows (anti-gravity gain 0 turns it off). F or D max typed: the PID sliders let go. Values Betaflight has and this firmware has not are not kept |
| SET_FILTER_CONFIG (93) | what FILTER_CONFIG shows. A typed gyro / D-term lowpass value turns that filter slider off (`filters.slider_gyro` / `slider_dterm`) |
| SET_BATTERY_CONFIG (33) | the cell limits (0.1 V, or the 0.01 V long form; min > warning or warning > max is refused, as Betaflight), capacity, the meters. A meter read and written back keeps AUTO / DroneCAN; a virtual or MSP current meter is refused |
| SET_VOLTAGE_METER_CONFIG (57), SET_CURRENT_METER_CONFIG (41) | `battery.divider` = scale x 100 / (divider value x multiplier); `current_scale`, `current_offset`. Meter id 10 only |
| SET_FAILSAFE_CONFIG (76) | guard time, landing time, throttle, procedure (switch mode, throttle-low delay: not kept). A procedure read and written back keeps HOLD |
| SET_GPS_RESCUE (225) | what GPS_RESCUE shows; the autopilot throttles and the sanity checks are not kept. CONE read and written back stays CONE |
| SET_SIMPLIFIED_TUNING (141) | the sliders; the gains / cutoffs they own follow (as when the Windburst app moves them) |
| SET_BLACKBOX_CONFIG (81) | device (NONE, FLASH, SDCARD; SERIAL refused), the sample rate -> `blackbox.rate_hz`. Refused while logging or erasing (Betaflight) |
| SET_OSD_CONFIG (85) | one element's position / shown (`osd.pos`; Betaflight items this firmware has not: accepted, not kept); screen 0: a statistic -> `osd.stats`; -1: the general part (video system, units, alarms, warnings: accepted, not kept) |
| SET_MOTOR (214) | the Motors tab's motor test, through the firmware's own (the Windburst app's Motor tab): disarmed only, `msp.writes` on; a motor at 1000 us stops; the motors stop by themselves 600 ms after the App stops sending |
| SET_GPS_CONFIG (223) | what GPS_CONFIG shows; a provider or SBAS value outside the table is refused. The 4-byte form keeps home once / Galileo |
| SET_SENSOR_CONFIG (97) | barometer on / off, compass on / off and its model, rangefinder on / off, flow sensor off (turning it on: pick the sensor in the Windburst app). The accelerometer stays on. A value read and written back keeps the compass model |


**Not available yet:** the Motors tab's wizards; the OSD tab's font upload; VTX; servos; calibration from the App; MSP_SET_RAW_RC on the app port.

### 2.4 CLI over MSP, the Ports tab

The Betaflight App sends a command line between STX and ETX on the app link and reads the lines that come back between STX and ETX (its CLI tab, and the Ports tab: in API 1.49 that tab is read-only and assigns ports through the CLI). Taken only between frames of both protocols; a line that is not text is dropped.

| Command | What it does |
|---|---|
| `peripherals` | one line per UART: `serial UART1: gps*` - what each one serves (`msp` on UART0 when the app link is there) |
| `get gps_uart` (also `rx_uart`, `opticalflow_uart`, `esc_sensor_uart`, `osd_uart`, `telemetry_uart`) | `gps_uart = UART1` and `Allowed values: NONE, UART1, UART2` (UART0 only when the app link is on the chip's USB) |
| `set gps_uart = UART2` | the serial ports table: `ports.assign` TABLE, the other assignments as they were, this one moved (a device on that UART loses it - the App warns first). Unsaved until `save` |
| `save` | save, then restart (Betaflight's save restarts; the App reconnects) |
| `exit` | restart without saving |
| anything else | this firmware's own CLI (docs/CLI.md): `get`, `set section.field = ...`, `diff`, `dump`, ... |

`opticalflow_uart` is this firmware's MSP port (flow sensor, MSP GPS / compass / barometer / receiver).

## 3. Tests
- `test_msp_sensors` (25): frames built from INAV's definitions, not using driver code. Covers GPS (2D / 3D / RTK fix, truncated frame, excessive speed), compass, barometer (NaN dropped), RC v1 / v2 (value clamping, signal loss after 300 ms), flow forwarding, corrupt frames, byte limit per read.
- `test_msp_config` (219): identity; each read reply compared byte by byte with Betaflight master's packing; each write command; a read written back changes nothing; rejections while armed / with `writes` off / for values this firmware does not have.
- `test_esc4way`: the bridge's answers (API 1.49, STATUS, MOTOR_CONFIG, box ids, FC_VERSION on the OSD port).
- `test_flowsensor`: MSP v1 frames.
- `test_fw`, group `msp` (`WB_ONLY=msp`):
  - GPS + compass + barometer + MSP receiver on UART1: arm with the MSP receiver's switch, take off with its throttle in Altitude Hold on the MSP barometer; the receiver stops sending and failsafe triggers;
  - flow + GPS sharing one port;
  - RX pin not set reports an error;
  - a configurator on the app port: read attitude, read and write PIDs (applied immediately to the flight core), save, reboot, `identity` / `writes`;
  - the Betaflight App's connect sequence: every request answered, arming held off (`CONFIGURATOR`), released on disconnect and after 3 s of silence.
- Mutants: `msp:` (40), `bfapp:` (33), `bfcfg:` (23), `balign:` (8), `bfmix:` (4), `bwiz:` (11), `units:` (13), `bfsens:` (11), `bfpid:` (12), `bffilt:` (11), `bfgps:` (8), `checks:` (12); `bfcfg:` is now 31.

## 4. To try on the board
1. Betaflight App (2026.12) over USB: connect, Setup tab (model rotates with the craft), Configuration tab (change the PID loop divider, airmode, beepers, Save, reconnect and check), Receiver, Modes, PID Tuning, Rates. Then open the Windburst app and check the same values.
2. While the App is connected, try to arm: it must refuse (`CONFIGURATOR`). Disconnect: it arms.
3. SpeedyBee app (Bluetooth) if you have the module.
4. An MSP GPS / compass module (e.g. Matek M10Q-5883 in MSP mode) and an MSP receiver if available.
