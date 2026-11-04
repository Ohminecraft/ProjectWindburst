# WBP — app ↔ firmware protocol

A single source: **`protocol/wbp_schema.json`**. From this file:
- `tools/gen_proto.py` generates `src/proto/wbp_schema.h` (packed structs, default values, `validate()` functions, section/stream/command tables) for the firmware;
- the Python client (`app/wbclient`) reads **that same json file at runtime**; a future JavaScript app only needs to read this json too.

Both sides carry the same `hash` (crc32 of the json with the `desc` fields removed). The app refuses to talk if the hashes differ, so it never misreads the settings of a different firmware generation. After editing the json: `make proto` (or `python3 tools/gen_proto.py`), then rebuild the firmware.

Layer diagram:

```
app (UI / CLI)  ──  wbclient (Python)  ──  USB / UART0  ──  8 kB transmit queue  ──  wbp::Server (task "wb_link", core 0)
                                                                     │  app::LinkHost
                                                                     ▼
                                                              app::Exchange  (seqlock + 1-request mailbox)
                                                                     ▲
                                                              FlightApp (flight task + sensor thread, core 1)
```

## 1. Framing (`src/proto/wbp_codec.h`, `app/wbclient/framing.py`)

```
| 0xA5 | 0x5A | len_lo | len_hi | type | seq | payload (len bytes, max 512) | crc_lo | crc_hi |
```
- CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over `len, type, seq, payload`. All multi-byte numbers are little-endian.
- `seq`: the app counts up 1..255; a reply carries exactly the `seq` of the request. `STREAM` and `LOG` frames have their own counters.
- **Self-resynchronising**: a corrupt frame is rescanned from its second byte, so garbage on the line (the ESP32 boot log sharing the port) or a fake header never hides a real frame behind it. ~20 ms of silence in the middle of a frame drops the partial frame (`Server::on_idle()`).

| type | name | direction | payload |
|---|---|---|---|
| 0x01 | HELLO | app→fw | — |
| 0x02 | DEVICE_INFO | fw→app | `proto u8, role u8 (0 RX, 1 TX), major, minor, patch, schema_hash u32, features u32, name…` |
| 0x10 | SECTION_GET | app→fw | `id` |
| 0x11 | SECTION | fw→app | `id, size u16, data` |
| 0x12 | SECTION_SET | app→fw | `id, data` → returns `SECTION` with the value **actually stored** (after range clamping) |
| 0x13 | SETTINGS_SAVE | app→fw | — → ACK |
| 0x14 | SETTINGS_DEFAULTS | app→fw | `id` or `0xFF` (all) → ACK |
| 0x15 | REBOOT | app→fw | — → ACK, the firmware reboots ~300 ms later |
| 0x20 | STREAM_CFG | app→fw | `id, rate_hz u16` (0 = off, id 0xFF = all off) → ACK |
| 0x21 | STREAM | fw→app | `id, t_ms u32, the stream's struct` |
| 0x30 | COMMAND | app→fw | `cmd, parameters…` |
| 0x31 | CMD_RESULT | fw→app | `cmd, err, results…` |
| 0x40 | LOG | fw→app | `level, text` |
| 0x7F | ACK | fw→app | `type being answered, seq, err` |

Error codes (`errors` in the json): `OK, UNKNOWN, BAD_ARG, ARMED, BUSY, NOT_SUPPORTED, BAD_SECTION, SIZE_MISMATCH, STORAGE, NO_SENSOR, BAD_STATE, FAILED`.

## 2. Settings

**0.43.0: Flycam / FPV, temperature-compensated baro.** **Backward compatibility:** the app no longer requires the hashes to be equal. It uses the schema of the connected firmware, taken from the `app/wbclient/schemas` library (see docs/APP.md). Nothing changes in the firmware. Section `general` gains `fly_style` (new enum `fly_style`: FPV, FLYCAM) and `flycam_hold` (new enum `flycam_hold`: LOITER, POSHOLD) at the end (57 → 59 bytes). `compat` of `general` is now 57, 56, 24, 22: 23-byte records (0.9.0 – 0.9.4) revert to defaults. Schema hash changes.

**0.42.1: optical flow.** Enum `flow_reject` gains `RATE` at the end. New enum `flow_cal_result`. New command `FLOW_CAL` (26): `step` 1 = start, 2 = done, 0 = cancel. The result contains `reason`, `scale` (%), `scale_x`, `scale_y` (signed %). On OK it writes `nav.flow_scale` (RAM, Save to keep it). Schema hash changes, sections unchanged.

**0.42.0: Crash flip.** Enums `mode` and `fly_mode` gain `CRASHFLIP` at the end. Bits `arm_blocker` gain `CRASHFLIP` (bit 19), `nav_flags` gain `CRASHFLIP` (bit 7). Section `motors` gains `crashflip_rate`, `crashflip_motor_pct`, `crashflip_auto_rearm` at the end (44 → 47 bytes). `compat` of `motors` is now 44, 37, 36, 35: 34-byte records (firmware 0.5) are no longer read, in which case section `motors` reverts to defaults. Schema hash changes.

**0.41.0: Loiter.** Enums `mode` and `fly_mode` gain `LOITER` at the end. Section `nav` gains `loiter_speed` at the end (43 → 44 bytes). `compat` of `nav` is now 43, 38, 34, 31: 30-byte records (firmware around 0.30 and earlier) are no longer read, because `compat` keeps at most 4 sizes; in that case section `nav` reverts to defaults. Schema hash changes.

**0.40.1:** section `telemetry` gains `mp_home_item` at the end (15 → 16 bytes, `compat` 15 and 14). Schema hash changes.

**0.40.0: ground station over MAVLink.** Section `telemetry` gains `remote_arm` at the end (14 → 15 bytes, `compat` 14: old records still load, `remote_arm` = off). Schema hash changes: app and firmware must be the same version. The firmware writes section `mission` by itself when the ground station uploads a plan (like level trim: saved once disarmed, log line `settings: section mission written by the firmware`).

**0.39.0: MAVLink receiver.** Enum `rx_protocol` gains `MAVLINK` (= 9, at the end: old values unchanged, old records load as before). Only the descriptions of `receiver.telemetry` and section `telemetry` change. Schema hash changes: app and firmware must be the same version.

**0.38.4: WiFi removed.** Section `wifi` (id 20) is gone: its old record in NVS just sits there, read by nobody; id 20 is not reused. Field `telemetry.wifi` becomes `reserved_t` (a placeholder so later fields do not shift). Stream `link` drops its last 5 fields (`wifi`, `wifi_ip`, `wifi_rssi`, `wifi_clients`, `via`) and the enums `wifi_mode`, `wifi_state`, `link_via`. Feature bit 8 (`WIFI`) is no longer used. The `secret` field flag (only `wifi.pass` had it) is also dropped. Schema changes: app and firmware must be the same version.

**Upgrading to 0.38**: sections `receiver` (13 → 22 bytes) and `pins` (52 → 57 bytes) grow at the end; old records can still be read through `compat`. `pins` only remembers 4 old sizes (52, 50, 48, 47): the pin table of firmware older than about 0.30 (46 bytes or less) reverts to defaults - set the pins again in the Pins and ports tab after flashing. 0.38.1: the default nRF24 frequency channel changes 100 → 76 (100 = 2500 MHz, outside the permitted 2.4 GHz band); saved records keep the old value until the next bind.

**Two devices, one schema (0.38).** The flight board (role RX) and the DIY transmitter (role TX, `docs/TX.md`) share `protocol/wbp_schema.json` and one hash. Sections / streams with `"role": "tx"` belong to the transmitter (`tx_radio`, `tx_sticks`, stream `tx`); without it they belong to the flight board. Each device only serves its own part: `SECTION_GET` / `SECTION_SET` / `DEFAULTS` of the other device's section → `ERR_BAD_SECTION`, `STREAM_CFG` of the other device's stream → `ERR_BAD_ARG`, `SAVE` / `DEFAULTS` all / flash storage / CLI only touch its own sections. The app knows which device it is through `role` in `DEVICE_INFO` (`Schema.sections_of(role)`).

17 sections, each section is a packed struct (`S_<name>`). For fields with `scale`, `real value = number on the wire × scale`; an `enum` is an index into the list of names; `optional` + 255 = empty slot.

| id | section | bytes | applied | content |
|---|---|---|---|---|
| 1 | general | 22 | reboot | loop rate, board alignment + trim, failsafe (including `fs_fallback_throttle`: % of hover thrust when there is no altitude), airmode + airmode start throttle threshold |
| 2 | receiver | 12 | reboot | RX protocol, channel order, deadband, timeout, RC smoothing (enable, factors for sticks and throttle) |
| 3 | modes | 96 | **live** | 16 slots: mode / aux / µs range (as in Betaflight) |
| 4 | rates | 19 | **live** | Actual rates, throttle mid/expo/limit, angle limits for Angle / Stabilize |
| 5 | pid | 75 | **live** | tuning sliders (`slider_*`), per-axis P I D Dmax F, I-term relax, anti-gravity, D max, TPA, feedforward, throttle boost, Angle, Stabilize; see `docs/TUNING.md` |
| 6 | motors | 35 | reboot | layout, ESC protocol, reversing, motor order, idle, dynamic idle |
| 7 | sensors | 16 | reboot | enable/disable baro, compass, sonar, GPS; compass model + alignment; magnetic declination |
| 8 | gps | 17 | reboot | provider, baud, rate, SBAS, model, home |
| 9 | beeper | 14 | **live** | which situations make the buzzer sound and which make the **motors** beep (two bit fields by enum `beeper_cond`), buzzer type, frequency, volume, ESC beacon tone; see `docs/POWER.md` |
| 10 | ports | 15 | reboot | (stored only — the firmware currently uses a fixed UART1 = GPS, UART2 = RX; a UART optical flow sensor takes a free port, see `docs/NAV.md`) |
| 11 | pins | 33 | reboot | IMU SPI, I2C, GPS, RX, motor, battery ADC, buzzer, **SD card**, **optical flow** (`flow_cs / sck / miso / mosi`, `flow_rx / tx`) pins… loaded into `HardwareConfig` at boot; 255 = use the default from `hardware_config.h` |
| 12 | calibration | 82 | **live** | compass calibration, accelerometer (offset/scale), level trim |
| 13 | battery | 13 | **live** | measurement source, cell count, full / warning / minimum thresholds, delay, land when critical, ADC divider factor |
| 15 | filters | 58 | **live** | two filter sliders, gyro lowpass 1 (static / dynamic) and 2, two static notches, dynamic notch, RPM filter, D-term filtering, yaw lowpass |
| 14 | blackbox | 10 | **live** | destination (`NONE` / `FLASH` / `SDCARD`), recording condition, rate, field groups, SD card wiring mode and speed; see `docs/BLACKBOX.md` |
| 16 | flow | 5 | reboot | optical flow sensor: type (`NONE` / `PMW3901` / `MSP`), UART sensor baud |
| 17 | nav | 27 | **live** | flow orientation / mirroring / scale / quality threshold / maximum height, tilt correction from measured acceleration, Position Hold (maximum angle, braking, position controller gains), failsafe procedure (`LAND` / `RESCUE`), GPS Rescue; see `docs/NAV.md` |

Firmware-side rules (`wbp_server.h`, `settings_store.h`):
- Changes live in **RAM** until `SETTINGS_SAVE`. Flags in stream `status.settings_dirty`:
  - `DIRTY` (unsaved): some section differs from the copy in flash.
  - `REBOOT_REQUIRED`: some non-live section differs from what the firmware used at boot.

  The firmware compares each section with a 32-bit hash (FNV-1a). The baseline is the moment the settings have just finished loading (`Server::settings_loaded()`), then it is updated on every flash write. So **changing a value and then setting it back turns both flags off** (since 0.37.1; before that the flags stayed until the next save). Saving a section that needs a reboot and only then setting it back leaves only `DIRTY`: flash holds the new value while the firmware still runs the old one. A failed flash write turns `DIRTY` on until a save succeeds, because it is unknown what the flash still holds.
- **Live** sections are applied to the running firmware immediately (PID, filters, rates, modes, calibration); the other sections take effect after save + reboot.
- **Relations between fields** (`settings_rules.h`), applied after `validate()` whenever a section is written by the app or loaded from flash: for an axis that follows the slider, that axis's P/I/D/Dmax/F are **recomputed** from the slider; with the filter slider enabled the lowpass cutoffs are recomputed. The SECTION_SET reply carries the computed values, so the app always shows exactly what is in effect.
- **The firmware writes a section by itself**: in-flight level trim changes section `calibration` and then asks the link task to write just that section to flash (only once disarmed; a flash write stalls the loop for a few ms). Success: LOG `settings: section calibration written by the firmware` (the app re-reads the section and reports it); flash refuses: `DIRTY` flag.
- **While armed every change is refused**: SECTION_SET, SAVE, DEFAULTS, REBOOT and most commands return `ERR_ARMED`. Reading is always allowed.
- Every incoming value goes through `validate()`: range clamping, unknown enums, **NaN → default value** (a buggy app cannot push NaN into the PID).
- Flash: one NVS record per section `wbsNN = [format][size][data][crc16]`. A missing record / wrong size (new firmware changed that section) / wrong CRC reverts **only that section** to defaults; the rest is kept. Exception: a section that only **added fields at the end** (or was truncated at the end) compared with the old firmware declares `"compat": <old size>` in the schema — a record of exactly that size is still loaded, the new fields keep their defaults (for example the 23-byte `pins` of version 0.4.0 before the SD card pins existed: no need to retype the pin map after flashing new firmware). The `settings:` line of the boot report shows the mask of sections loaded this way; the next save writes them back at the new size. `compat` is not part of the schema hash.
  Upgrading from 0.5.0 to 0.6.0: `receiver`, `rates`, `motors` (adds `dshot_driver` at the end), `pins`, `blackbox` are kept thanks to `compat`, sections whose size did not change are kept (including accelerometer calibration and level trim); **`general` and `pid` revert to defaults** (the layout changed in the middle: gyro filtering moved to the new `filters` section; `general` gained `airmode`, `runaway_takeoff`) — re-enter board alignment / trim and the failsafe options. A backup (System tab) taken before flashing can restore the parts whose names still match.
  Upgrading to 0.8.0 (from 0.7.x or 0.6): `pins` grows by `servo[4]` (0.7.1), `curr_adc`, `rx_pwm[8]` at the end and can still be read from 37-byte (0.7.1), 33-byte (0.7.0) and 27-byte (0.6) records — `compat` now accepts up to **three** old sizes; 23-byte records from 0.5 revert to defaults. `battery` gains 4 current-measurement fields (`compat` 13), `ports` gains `assign` (`compat` 15; default AUTO so the old table changes nothing). `servo` (0.7.1) is a new section. Enum `rx_protocol` gains `SPEKTRUM1024`, `port_function` gains `FLOW_MSP` (added at the end: old values keep their meaning). App and firmware must be the same version (schema hash changes).
  Upgrading from 0.6 to 0.7.0: `pins` grows by 6 pins at the end and can still be read (both the 27-byte records of 0.6 and the 23-byte records of 0.5: the `compat` table now has two old sizes); `flow` and `nav` are new sections (defaults); section `blackbox` keeps its old values so it **does not yet have the `NAV` log group** (tick it in the Blackbox tab). A mode in section `modes` whose index the firmware does not know becomes an empty slot (it used to be clamped to the last mode).
  **The meaning of enum `align` has changed** (now exactly Betaflight's meaning): `general.board_align` must be re-entered according to the table in `docs/HARDWARE_FEEDBACK_1.md` section 8; `sensors.mag_align` keeps its old number but with the new meaning.

## 3. Telemetry streams

The app subscribes to each stream at the desired rate; the firmware clamps it to `max`. Each tick sends at most 4 frames, never "catching up"; a frame that does not fit the TX buffer is **dropped**, not waited for (a hung app does not slow the firmware).

| id | stream | default / max (Hz) | content |
|---|---|---|---|
| 1 | attitude | 50 / 200 | roll, pitch (pilot convention), **yaw is the heading 0 … 360° (0 = north, clockwise; since 0.8.8, u16; before that −180 … 180)**, filtered gyro, setpoint |
| 2 | rc | 20 / 100 | 16 channels in µs, link, LQ, frames/second |
| 3 | status | 10 / 50 | armed, **arm blockers** (named bits; new: `LOOP_RATE`, `RUNAWAY_TAKEOFF`, `RESCUE_NO_FIX`, `RESCUE_NO_COMPASS`), failsafe, modes, health, AltHold state, loop time, battery/cell, settings flags, throttle, CPU %, RAM %, free RAM / PSRAM (kB), **average time of each stage of the control cycle over the last second `t_imu / t_core / t_motor / t_rest` (µs), longest pass of the sensor thread `t_slow_max`, missed cycles `overruns`, set rate `loop_target`, failsafe cause `fs_cause` (bits LINK / SWITCH / BATTERY)** |
| 4 | motors | 20 / 100 | the commands actually sent to the ESCs (including during motor test), rpm, telemetry errors, temperature, voltage, dynamic idle, **`bidir` (OFF / ACTIVE / UNAVAILABLE), `answers` (bit i = motor i's ESC answered), `skipped`, `late` (skipped frames / missed replies over the last second)** |
| 5 | sensors | 50 / 200 | raw gyro, accel, compass, baro, temperature, sonar |
| 6 | gps | 5 / 25 | fix, satellites, lat/lon/alt, speed, course, HDOP, distance/direction to home, driver status, **`rate` measured solutions per second (0.8.8)** |
| 7 | estimate | 20 / 100 | altitude, vertical velocity, sigma, horizontal position/velocity, validity flags (`ALT`, `POS`, `YAW_FROM_MAG`, **`VEL`, `FLOW`, `TILT_AIDED`**, 0.8.3: **`GPS_ALT`, `GPS_VZ`**), 0.8.8: **`z_resets`** (number of times the altitude was reset to baro) |
| 8 | gyro_fast | 0 / 1000 | raw + filtered gyro for the vibration chart |
| 9 | calibration | 10 / 20 | type, progress %, status, 6-side mask (accel), raw values, **% of the in-flight level trim measurement** (100 = measured, waiting for disarm) |
| 10 | battery | 2 / 20 | voltage, per cell, cell count, status, measurement source, mV at the ADC pin, buzzer tone playing, motor tone playing |
| 11 | blackbox | 2 / 10 | status, capacity, used (bytes, saturating at 4 GB), number of sessions, actual rate, bytes per record, dropped records, erase %, bytes/second, **device, capacity / used in kB (large cards), card buffer fill level (%)** |
| 12 | tuning | 0 / 50 | live state of the filters and PID: current gyro / D-term lowpass 1 cutoffs, 9 dynamic notches (3 per axis), first-harmonic RPM notch per motor, TPA %, anti-gravity, measured receiver frame rate and RC smoothing cutoff, airmode, PID sum per axis, **measured hover throttle** |
| 13 | nav | 10 / 50 | flags (`POSHOLD`, `POSHOLD_UNAVAILABLE`, `RESCUE`, `RESCUE_AVAILABLE`, `NAV_LEAN`, 0.8.8: `POSHOLD_SLOW_GPS`), flow: quality, sample-rejection reason (`OK` … `NO_SAMPLE`), samples/second, measured velocity (forward / right, 0.25 s average); Position Hold: per-axis state, requested tilt angle, held point, wind compensation; Rescue: phase, give-up reason, distance to home, return altitude |
| 14 | link | 1 / 10 | (0.8.8) link health: `uptime` (s, to ms), `boot_id`, frames received / corrupt, `busy`, `tx_lost`, `dropped`, `kicks`, `cmd_max_ms`, `link_gap_ms`, `slow_gap_ms`, `queued` - see docs/FIRMWARE.md "Firmware not responding" |

New section in 0.8.8: **`althold`** (id 19, live): `mid_stick`, `deadband`, `low_stick`, `max_stick` (% of throttle stick travel, not through the throttle curve), `climb_rate`, `descend_rate` (0.1 m/s). `nav` gains `ph_min_gps_hz` (compat 29, 27).
On the same port as WBP the firmware also understands enough **MSP** for ESC tools (docs/ESC_PASSTHROUGH.md); both parsers receive every byte.

The bit names (`bits` in the json) are checked by the firmware at compile time against the core's real enums (`static_assert` in `settings_apply.h`): changing an enum in the core and forgetting to update the json is a compile error, not a wrong label in the app.

## 4. Commands

| id | command | parameters | notes |
|---|---|---|---|
| 1 | GYRO_CAL | — | ~1 s, progress in stream `calibration`; the craft must be still |
| 2 | ACC_CAL_POINT | — | averages ~1 s per side; the firmware **detects which side by itself** (bit in `calibration.mask`) |
| 3 | ACC_CAL_FINISH | — | needs all 6 sides; fits offset + scale, writes into section `calibration` (RAM) |
| 16 | ACC_CAL_CANCEL | — | |
| 24 | BIND | action u8 (1 start, 0 stop) | nRF24, both devices: the transmitter broadcasts a bind invitation, the flight board listens; refused while armed |
| 25 | STICK_CAL | step u8 (1 start, 2 done, 0 cancel) | transmitter: push every stick to both ends then release to center, done = writes `tx_sticks.cal_*` (RAM) |
| 4 | LEVEL_TRIM | — | craft lying flat, still; refused if tilted > 20° |
| 5/6/7 | MAG_CAL_START / FINISH / CANCEL | — | FINISH returns `reason` (enum `mag_cal_result`); on failure it keeps collecting samples |
| 8 | MOTOR_TEST | motor u8 (255 = all), value u16 0..1000 | **must be resent every ≤ 0.5 s**, otherwise the motors stop by themselves; **remove the props** |
| 9 | MOTOR_STOP | — | |
| 10 | MOTOR_DIRECTION | motor, reversed | DShot reverse-direction command (DShot only) |
| 11 | MOTOR_BEACON | motor | |
| 12 | SET_HOME | — | needs usable GPS |
| 13 | ESC_TELEMETRY | enable | extended DShot telemetry |
| 14 | APPLY_SETTINGS | — | re-applies every live section |
| 15 | REPORT | — | resends the boot report (which sensors were found / why not) as LOG frames |
| 17 | BEEP | condition (empty = silence) | plays a test tone on the buzzer and on the motors (if the ESCs use DShot), even tones that are disabled; `NO_SENSOR` when the craft has nothing to beep with |
| 18 | BLACKBOX_ERASE | — | erases the whole log memory of the selected device (flash: erases the region; SD card: deletes the `wb_*.wblog` files); runs in the background (progress in stream `blackbox`), blocks arming until done |
| 19 | BLACKBOX_STREAM | enable, rate_hz (0 = as configured) | sends the log to the app in `BB_HEADER` / `BB_DATA` frames; **allowed while armed**, but refused (`BUSY`) while armed and recording to flash |
| 20 | BLACKBOX_SESSION | index → start, size | location of a session in the log memory (`BAD_ARG` = end of the list, `NO_SENSOR` = no memory / no card). For an SD card: the files are laid end to end as one address range |
| 21 | BLACKBOX_READ | offset, length | sends a chunk of the log memory as `BB_BLOB` frames; length 0 = cancel |

While the app is calibrating, testing motors, or the log memory is being erased, the firmware **blocks arming** (blocker `CALIBRATING`).

The three blackbox frame types (`BB_HEADER` 0x60, `BB_DATA` 0x61, `BB_BLOB` 0x62) are not replies to any request: the client hands them to `on_blackbox` / the log downloader before matching replies by `seq`. The inner format: `docs/BLACKBOX.md`.

"App connected" (used for buzzer silence and for ending the log stream) = a valid frame within the last 3 s; the client sends a `HELLO` with `seq` 0 every 0.3 s when it has nothing else to send.

## 5. Threads and safety (`src/app/exchange.h`, `link_host.h`)

- The flight loop (core 1) **never waits** for the link task: telemetry is `publish()`ed through a seqlock (lock-free writes), and the link task `snapshot()`s it and then packs it.
- Every command and every settings application **runs inside the flight loop**, between two control cycles (`Exchange::service()`), so drivers and the core are only touched by one thread. The link task waits for a result for at most ~200 ms; a request not picked up in time is **withdrawn** (it never runs late — important for motor test).
- `check_request()` (arm state, parameter size, unknown command) is checked on the flight-loop side, with the exact arm state of that cycle.
- Writing NVS flash stalls flash access on both cores for a few ms → only allowed while disarmed.

## 6. Adding a new field / command

1. Edit `protocol/wbp_schema.json` (add a field to a section/stream, or a command). `f32` fields must have `min`/`max`.
2. `make proto` → new header, new hash.
3. Use the field in `src/proto/settings_apply.h` (settings → configuration) or `src/app/telemetry.h` (streams) or `FlightApp::execute()` (commands).
4. `make test && make apptest`. When a section changes size, the old flash record of **that section only** reverts to defaults at the next boot — unless fields are only added at the END of the section and the old size is written into `"compat"` (see the Storage section); adding a field in the middle cannot use `compat`.

## Verified / not verified

Verified on the PC: see `docs/FIRMWARE.md` (test table). The Python client is tested against **the real firmware code itself** running in `wb_sim`.
Not verified: everything on the real board — the ESP32-S3's USB CDC (`availableForWrite`, real throughput), NVS, DTR/RTS behaviour when the port opens (the client holds DTR/RTS low so as not to reset the board, untested), the actual latency of `Exchange` between the two cores.
