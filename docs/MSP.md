# MSP: peripherals and other configuration tools (0.33)

Checked on PC: unit tests and firmware running on the virtual board (`test_fw`). **Not tried with real devices, not tried with a real Betaflight Configurator / SpeedyBee.**

MSP (Betaflight's MultiWii Serial Protocol) is used in two places:
1. **MSP port:** a UART that MSP devices push data into.
2. **App port:** Betaflight Configurator, the SpeedyBee app and ESC tools talk to the firmware over this port.

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

## 2. Other configuration tools on the app port

New section **`msp`** (System tab, "Other tools over MSP" block):

| Field | Default | Meaning |
|---|---|---|
| `identity` | `BETAFLIGHT` | Answers as Betaflight 4.5 (`BTFL`, 4.5.0, API 1.46). Betaflight Configurator, SpeedyBee and ESC tools only work with firmware they know. `WINDBURST`: reports `WBRX` and the real version; the tools above will refuse. BOARD_INFO: board code `WBRX` when claiming to be Betaflight (Betaflight Configurator shows it as the board name), chip code (e.g. `_S3_`) when claiming to be Windburst |
| `writes` | on | Lets those tools edit the configuration and save. Off makes it read-only. Never allows edits while armed |

> The previous build had an uncommitted change that made the firmware always report `WBRX`. That is now the `identity = WINDBURST` option. The default is still Betaflight, so ESC tools (ESC passthrough) and Betaflight Configurator work as before.

**Reads** (exactly Betaflight 4.5's packing, `msp.c`):
- **Live state:** RC (105), RAW_IMU (102: acc ×512 / g, gyro ×4.1 / °/s, mag ×1090 / G), RAW_GPS (106), COMP_GPS (107), ATTITUDE (108: 0.1°; positive pitch is nose down), ALTITUDE (109), ANALOG (110).
- **Configuration:**
  - NAME (10): craft name = `osd.name`;
  - MODE_RANGES (34), MODE_RANGES_EXTRA (238): 20 ranges; the STABILIZE mode has no box in Betaflight so it is not shown;
  - RX_MAP (64);
  - RC_TUNING (111): **Actual** rates;
  - PID (112): roll / pitch / yaw, LEVEL = `angle_strength`;
  - BOXNAMES (116), PIDNAMES (117), ACC_TRIM (240).
- Older commands (STATUS, BOXIDS, MOTOR, BATTERY_STATE, ...) unchanged. Unsupported commands return `!`; Betaflight Configurator skips them and carries on.

**Writes** (go through exactly the app's steps when editing a section: clamp values, apply rules, apply immediately or report that a reboot is needed):
- SET_NAME (11), SET_MODE_RANGE (35), SET_RX_MAP (65), SET_RC_TUNING (204), SET_PID (202).
- EEPROM_WRITE (250): save.
- REBOOT (68): replies first, then reboots.
- **Rates:** only Actual rates are accepted. Other types are rejected, no guessing at a conversion.
- **PID:** entering PIDs different from the old values turns the tuning sliders off (`pid.slider_mode = OFF`), like Betaflight Configurator. Writing back exactly the values just read changes nothing.
- **MSP_SET_MOTOR is still rejected:** use the app's Motor tab for motor tests, since it has the safety checks.

**Not yet available:**
- Betaflight Configurator's Filters, Motors, Ports, Failsafe, OSD, Blackbox tabs: partially readable or not at all;
- calibration from that tool;
- MSP_SET_RAW_RC on the app port;
- MSP2 (MSP2_GET_TEXT, ...).

## 3. Tests
- `test_msp_sensors` (25): frames built from INAV's definitions, not using driver code. Covers GPS (2D / 3D / RTK fix, truncated frame, excessive speed), compass, barometer (NaN dropped), RC v1 / v2 (value clamping, signal loss after 300 ms), flow forwarding, corrupt frames, byte limit per read.
- `test_msp_config` (42): identity; each read reply, compared byte by byte with Betaflight 4.5's packing; each write command; rejections while armed / with `writes` off / when rates are not Actual / when the box does not exist.
- `test_flowsensor`: adds MSP v1 frames.
- `test_fw`, group `msp` (`WB_ONLY=msp`):
  - GPS + compass + barometer + MSP receiver on UART1: arm with the MSP receiver's switch, take off with its throttle in Altitude Hold on the MSP barometer; the receiver stops sending and failsafe triggers;
  - flow + GPS sharing one port;
  - RX pin not set reports an error;
  - Betaflight Configurator on the app port: read attitude, read and write PIDs (applied immediately to the flight core), save, reboot, `identity` / `writes`.
- 40 `msp:` mutants.

## 4. To try on the board
1. Betaflight Configurator 10.10 over USB: Setup tab (model rotates with the craft), Receiver, Modes, PID Tuning, Rates. Edit PIDs and Save, then reopen the Windburst app and check the values.
2. SpeedyBee app (Bluetooth) if you have the module.
3. An MSP GPS / compass module (e.g. Matek M10Q-5883 in MSP mode) and an MSP receiver if available.
