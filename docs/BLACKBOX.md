# Blackbox — `src/blackbox`, `app/wbclient/wblog.py`, Blackbox tab

Records a flight so it can be viewed as charts: gyro, setpoint, PID terms, sticks, motor commands, angles, loop timing (fast), and status, modes, battery, altitude, GPS, compass, position and navigation (10 times/second).

## 1. Two paths for the log

| | On-board flash | SD card | Sent straight to the app ("live recording over the cable") |
|---|---|---|---|
| Enabled by | `blackbox.device = FLASH` + `blackbox.trigger` (`ARMED` / `SWITCH` = Blackbox mode / `ALWAYS`) | `blackbox.device = SDCARD` + `trigger` | the "Start recording" button in the tab, or `python -m wbclient blackbox record` |
| Retrieving | download over the app link (`BLACKBOX_READ`) | pull the card and plug it into a computer (files `/wblog/wb_00001.wblog`…), or download through the app like flash | the app writes the file as it receives it |
| Limits | partition size (N16R8: 14 MB); every flash write stalls both cores (section 4) | card capacity; a slow card loses records (they are counted), **the flight loop is not affected** | link speed (230400 baud ≈ 250 records/second with the default groups) |

If the board has an SD card, use the card: it is the only path that does not stall the flight loop (section 4b).

Rules between the two paths (original request: "when connected to the app, blackbox to Storage turns off automatically and transmits over serial"; revised following the feedback **do not switch automatically in mid-flight**):
- The app requests the log stream while the craft is **on the ground**: the running flash session (if any) is closed and the stream starts.
- The app requests the log stream while the craft is **armed and recording to flash**: refused (`BUSY`), the flight log is not cut.
- While the app is receiving the stream, no new flash session is opened. The stream ends when the app says stop or the app is silent for 3 s; after that `trigger` decides again.
- Merely connecting the app does **not** turn off flash recording; you have to press "Start recording".
- **The live stream rate limits itself to the link**: no more than half the bytes per second the port can carry (UART: baud / 10; TinyUSB-style USB CDC: 60 kB/s; USB Serial/JTAG: unlimited). Asking for 1000 records/second over a 230400 baud bridge makes the firmware drop to a rate that fits the link by itself, instead of dropping frames at random. Stream `blackbox.rate_hz` is the actual rate.
- **Downloading the log** (`BLACKBOX_READ`): while erasing, or while armed and recording → `BUSY`. Disarmed but a session is still open (trigger ALWAYS, or just disarmed) → the firmware closes that session, returns `BUSY` for this request, and the client retries within 3 seconds. For 3 seconds after a read, the firmware does not open a new session (arming lifts this limit immediately). GUI: pressing download while live recording stops the live recording session first.

Not available: the board showing up as a USB drive (mass storage), data compression.

## 2. The `.wblog` format

File = a sequence of **WBP frames** (same frames, same CRC as the link, see `docs/PROTOCOL.md`), so the file in flash, the stream sent to the app and the file on the computer are one format:

| Frame | Content |
|---|---|
| `BB_HEADER` 0x60 | text, split into several frames (seq 0, 1, 2 …). Seq 0 opens a new session and starts with `WBLOG 1` |
| `BB_DATA` 0x61 | records concatenated: `'F'` fast `[t_us u32][fields]`, `'S'` slow (10 Hz), `'E'` event `[t_us][code][n][data]` |
| `BB_BLOB` 0x62 | link only: a chunk of the log memory during download (`[offset u32][data]`) |

The header is **self-describing**: each field is written as `name:type:scale:unit` (for example `gyro[0]:i16:0.1:deg/s`), along with the event names, firmware version, schema hash, `loop_hz` (PID rate, since 0.12), `gyro_hz`, `rate_hz`, motor count, and **all of the flight's settings** (`set <section> <hex>`). The reader relies only on the header, so logs from another firmware version can still be read; the attached settings can only be decoded when the schema matches.

Events: `ARM`, `DISARM`, `FAILSAFE`, `MODES`, `DROPPED` (the firmware dropped records because the queue was full), `BATTERY`, `END` (reason for ending), `AH_STATE`. A session without `END` = power lost while recording; the data before that is still usable. A corrupt or missing frame only loses the records inside it (counted in `gaps`).

## 3. Firmware

```
src/blackbox/bb_format.h      field tables (kFast, kSlow), groups, Plan (field selection), record packing        [portable]
src/blackbox/bb_ring.h        lock-free SPSC queue: the flight loop writes, the link task reads                  [portable]
src/blackbox/bb_recorder.h    bb::Recorder: flight-loop side (log_fast every N cycles, log_slow 10 Hz, events)    [portable]
src/blackbox/bb_flash.h       bb::FlashLog: sessions on a raw NOR region (BlockStore)                            [portable]
src/blackbox/bb_storage.h     bb::Storage: common interface of the storage (flash or card)                       [portable]
src/blackbox/bb_card.h        bb::CardLog: one file per session on the SD card; byte queue, dedicated writer task [portable]
src/blackbox/bb_card_esp.h    SdFileStore (Arduino SD / SD_MMC), SdCardTask (task "wb_sd"), RtosLock
src/blackbox/bb_service.h     bb::Service: link-task side - trigger, header, flash / card / stream, download, erase   [portable]
src/blackbox/bb_header.h      the header part added by the firmware (version, settings)
src/blackbox/bb_store_esp.h   EspPartitionStore: the "wblog" partition, otherwise the existing "spiffs" partition
partitions/wblog_{4,8,16}MB.csv   partition tables with a large log partition (platformio.ini: board_build.partitions)
```
- **The flight loop only writes to RAM** (a 16 kB queue). When full it drops records and logs a `DROPPED` event; the flight loop never waits.
- **Flash is a raw NOR region, not a file system.** Reason: a file system would erase sectors by itself in mid-flight (stalls of 45 ms or more). Here erasing only happens when the user presses "Erase log memory"; in flight there are only page writes.
  - Each session starts at the beginning of a 4 kB sector; sessions are found at boot by reading the sector-start markers.
  - Free space is **checked to be all 0xFF** before use. A partition holding other data (old SPIFFS…) → status `NEEDS_ERASE`, the firmware does not overwrite it.
  - At most **one 256-byte page per millisecond**; a partially filled page is written after 0.5 s (limits the data lost on power loss).
  - Erasing runs from the end to the start, block by block, so an interrupted erase (power loss) leaves no "ghost" session; arming is not possible while erasing (blocker `CALIBRATING`), and the buzzer sounds when the erase is done.
- **Recording rate** `blackbox.rate_hz` is rounded to a divisor of the loop rate. `blackbox.fields` selects the field groups; by default 74 bytes per fast record.
- **Units of the PID columns** (`pid_p / pid_i / pid_d / pid_f`): Betaflight's PID sum units, 1000 = full motor range (the sum is limited to ±500, yaw ±400), step 0.1. Up to version 0.6.1 these columns were clamped at ±3.28 because of a wrong scale when writing; old logs can still be read but the PID numbers in them are not usable (`docs/HARDWARE_FEEDBACK_2.md` section 2).
- **"Loop timing" group**: `loop_us` (run time of the recorded cycle), `gap_us` (the longest gap between two cycles since the previous record), and since 0.6.2 five one-byte columns, 4 µs step, each the **maximum** since the previous record: `t_lock` (waiting for the lock), `t_imu` (IMU read), `t_core` (computation, normal cycle), `t_est` (computation, cycle that runs the estimator; 0 = none), `t_motor` (motor output).
- **"Position and navigation" group** (since 0.7.0, `NAV`; 10 times/second like the other slow groups): `pos_n / pos_e / vel_n / vel_e` (moved here from the GPS group: present even with optical flow only), `est_valid` (schema bits: altitude / position / heading from compass / velocity / flow / tilt corrected), `nav_flags` (Position Hold, Rescue, autopilot commanding tilt), `flow_quality`, `flow_reject` (why the latest flow sample was not used), `flow_fwd / flow_right` (measured flow speed, in the craft's axes), `nav_roll / nav_pitch` (tilt angle requested by the autopilot), `nav_target_n / nav_target_e` (the point Position Hold is holding), `rescue_phase`. Settings saved from versions before 0.7.0 lack this group's bit: enable it in the Blackbox tab (0.7.0 defaults already include it).
- **"Detailed navigation data" group** (since 0.34.0, `NAV_DEBUG`, off by default; in **every fast record**): inputs of the new navigator `nav_acc[3]` (N/E/D acceleration with gravity removed), `nav_vib`, `nav_clip`, `nav_baro`, `nav_gps_pos[3]`, `nav_gps_vel[3]`, `nav_gps_hacc`, `nav_gps_sacc`, `nav_gps_sats`, `nav_gps_flags`, `nav_gps_seq` (38 bytes). Later stages add estimates, innovations, setpoints (`docs/NAV_ROADMAP.md`).
- Every change in the `blackbox` section is **live**; a session being recorded keeps the configuration it started with.
- Stream `blackbox` (id 11): status (`OFF / NO_STORAGE / NEEDS_ERASE / IDLE / LOGGING / STREAMING / ERASING / FULL`), selected device, capacity and used (bytes, and kB for cards larger than 4 GB), number of sessions, actual rate, bytes per record, number of dropped records, erase %, highest fill level of the card buffer.
- `device = NONE`: nothing is recorded, but the sessions already in flash can still be listed / downloaded / erased. When logging is off, the firmware no longer read-checks the flash region.

### SD card (`bb_card.h`)

```
flight loop (core 1) --records--> 16 kB queue --every ms--> link task: framing --> ByteFifo (512 kB in PSRAM)
                                                                                         |
task "wb_sd" (core 0, low priority):  CardLog::work()  <---- 4 kB blocks, sync every 250 ms --+
```
- **Why a dedicated task is needed**: the card answers a write in 1 to 5 ms, but occasionally 50 to 300 ms (the card doing internal housekeeping). The link task (telemetry, app) and the flight loop must not wait for that. While recording, the link task only copies bytes into the queue; every `write()` / `sync()` call lives in the `wb_sd` task. Ground-side jobs (open / close a file, list, read back, erase) share one lock so the two tasks never touch the file system at the same time.
- **Does not stall the flight loop**: writing to the card does not disable the flash cache, and it runs on the other core. This is the main difference from recording to on-board flash.
- **Files**: directory `/wblog`, names `wb_00001.wblog`, `wb_00002.wblog`… (increasing numbers, continuing from the largest number on the card). The content is identical to a log in flash or a log the app records over the cable: open it directly with the Blackbox tab ("Add .wblog file…") or `python -m wbclient blackbox info`.
- **Power loss midway** (battery ejected in a crash): at most the data written since the last sync (250 ms) plus what is still in the buffer is lost. The file is still readable up to that point. On-board flash loses less (a few tens of ms): if you need every last millisecond before a crash, consider flash.
- **Buffer**: 512 kB of PSRAM when the module has PSRAM (N16R8), otherwise 32 kB of internal RAM. At 2000 records/second (≈ 150 kB/second) 512 kB survives a card stall of more than 3 seconds; 32 kB only about 0.2 seconds. When the buffer is full records are dropped and **counted** (`DROPPED` event in the log, `dropped` and `buffer_pct = 100` in the stream) — the flight loop is still unaffected.
- **Card not inserted / pulled out / reinserted**: no card → `NO_STORAGE`, it looks again every 3 s, no reboot needed. Pulling the card while recording → the session ends there, the file is readable up to what was written; reinserting continues recording into a new file. When idle the firmware checks every 3 s that the card is still there.
- **Selecting the card in the app needs no reboot** (the card is mounted immediately by the writer task). Changing **pins** needs a save and reboot like any other pin. While another device is selected, the firmware does not touch the card or its pins.
- **Erase**: only erases the `wb_*.wblog` files in `/wblog`; other files on the card are not touched. Fast (deletes files, no overwriting).
- **Download through the app**: the files are laid end to end as one address range, so the app downloads using exactly the same `BLACKBOX_SESSION` / `BLACKBOX_READ` commands as with flash. The newest 128 files are listed (and no more than 4 GB in total); older files stay on the card.
- **Wiring** (`blackbox.sd_mode`, pins `pins.sd_*`):
  - `SPI`: CS, SCK, MOSI, MISO. Uses the ESP32-S3's second SPI bus (the first bus belongs to the IMU): the card never shares wires with the gyro. An SPI barometer also uses the second bus — it cannot be used together with an SPI card (an I2C barometer is fine).
  - `SDMMC_1BIT`: CLK, CMD, D0 (three card pins, the same as SCK, DI, DO). Faster, no busy-waiting. Needs 10 kΩ pull-ups on CMD and D0, and the card's DAT3 / CS pin held high.
  - `blackbox.sd_mhz` (default 20): with long wires where the card is not recognised, lower it to 4 to 10.
- The card must be formatted **FAT32** (FAT16 also works). The FatFs in Arduino-ESP32 **has no exFAT**: cards of 64 GB and up ship as exFAT and must be reformatted to FAT32 on a computer. The firmware does not format the card by itself.

### Viewing the list does not cut the recording (0.8.8)
Bug a user hit: recording blackbox with an AUX switch, opening the app's Blackbox tab closed the running session, 3 s later a new session started by itself, the next list view closed it again... Since 0.8.8:
- `BLACKBOX_SESSION` (viewing the list) **no longer closes** the running session; that session appears in the list with its size so far.
- The session is only closed on **download** (`BLACKBOX_READ`) (on the ground; in flight it is BUSY as before). The app calls `blackbox_claim()` before downloading so the list has the final size.
- A **switch** session that the app closed for downloading: it does not restart by itself while the switch is still on; flip the switch off and on to record a new session. (Trigger ALWAYS still resumes recording 3 s after the last read; when the live stream to the app ends, the switch keeps recording as before.)
Verified: `test_blackbox` (TRIG_SWITCH: viewing the list 5 times in 4 s → one session, still recording; download → closed, 5 s later still no new session; off / on → new session), pytest `test_download_with_trigger_always`.

## 4. What must be measured on the real board: flash stalls the flight loop

On the ESP32, **every flash operation disables the cache and stalls both cores**, including the core running the flight loop. The design above limits each stall to one page (about 0.7 ms per the datasheet), and never erases in flight. In simulation: page write 700 µs, sector erase 45 ms, 64 kB block erase 150 ms — **these are assumed figures, not measured**.

Consequence if the real figures are larger: a 4 kHz loop loses a few cycles on every page write (at 250 records/second that is about 70 writes per second). The firmware records this itself so you can measure it:
- the `gap_us` field in the log ("Loop timing" group) = the longest gap between two cycles since the previous record;
- `status.gap_max_us` in telemetry.

The first thing to do on the board: enable flash recording, arm **without props**, look at `gap_us`. If it is larger than about 1.5 ms, use "live recording over the cable" or lower `rate_hz`, and report back so the design can be changed (for example recording to an external SPI flash or an SD card on its own bus).

## 4b. SD card: what must be measured on the real board

In simulation, one card write takes 3 ms, every 25th write takes 180 ms, sync 6 ms — **assumed figures**. The flight loop in simulation loses no cycles (4000 Hz, `gap_us` always 250), because the writing lives in another task; on the real board this depends on the Arduino-ESP32 SD library not doing anything that blocks core 1. How to measure: select the card, enable the "Loop timing" group, arm **without props** for one minute, look at `gap_us` in the log and `buffer_pct` / `dropped` in the Blackbox tab.

## 5. Capacity

The default Arduino-ESP32 partition table (4 MB) only has a `spiffs` partition of about 1.5 MB ≈ 75 seconds at 250 records/second with the default groups (about 20 kB/second). For more: use a table from `partitions/` (uncomment the `board_build.partitions` line in `platformio.ini`) matching the board's flash size, lower `rate_hz`, or drop some groups. The Blackbox tab precomputes "bytes per record, about … per second, free space is enough for about …".

## 6. App

**Blackbox tab** (`web/js/tabs/blackbox.js`, charts `web/js/gfx/logplot.js`, server side `wbclient/logview.py` + the `/api/blackbox/*`, `/api/log/*` routes in `gui.py`):
- *What to record, when*: destination, condition, rate, field groups, capacity estimate.
- *On-board log memory*: status, used space, session list, download one session or all (with progress), erase (asks for confirmation first). Locked while armed.
- *Live recording over the cable*: the chart follows new data ("Follow new data"); stopping turns it into a file in the library.
  - Since 0.31 the chart window scrolls by the clock, about 30 frames/second (`web/js/gfx/follow.js`), rather than jumping with the rhythm of incoming data. Before that, data through the UART bridge / the computer's USB driver arrived in chunks so the chart jumped about once per second.
  - The window ends slightly behind the newest record (`lag`): lag adjusts itself to the gap between two data bursts (×1.25), minimum 0.1 s, maximum 2 s. Thanks to that the right edge never runs into where there is no data yet.
  - The scroll speed is only adjusted by ±40 % to track the target: it never jumps, never goes backwards.
  - The app fetches new data 5 times/second.
  - Firmware: FIL test `test_fw` group `[blackbox] live stream over a 230400 baud UART` simulates a 230400 baud UART behind an 8 kB queue. Frames go out evenly, at most about 20–30 ms apart, even with extra 50 Hz telemetry; no frames are lost; the newest record is less than 80 ms old when it reaches the app. So the chunking is not in the firmware.
- *Logs on the computer*: directory `~/Windburst/logs` (change with `gui --logs DIR` or the environment variable `WBCLIENT_LOG_DIR`); view, save file, delete, add existing `.wblog` files. **Viewing logs does not need a board connection.**
- *Charts*: several stacked panels sharing the time axis, one unit per panel (gyro and setpoint per axis, PID, motors, sticks, angles, gyro before / after filtering, loop, altitude, battery, GPS, position (estimated and flow velocity, position and the held point, autopilot tilt angle, surface quality), receiver, compass — only groups for which the log has fields are shown). The PID group is drawn in PID sum units (smallest panel ±20); the "Loop" group has two panels: run time / gap between two loops, and each stage of a cycle (logs from 0.6.2). Drag to zoom in, double-click to see the whole log, Ctrl + mouse wheel, Shift + drag to pan; the cursor reads values in every panel; events are labelled dashed lines; CSV export; download the flight's settings as a backup file (restorable in the System tab).
- Logs longer than the number of pixels: the server sends **the minimum and maximum of each pixel column**, so a one-sample-wide spike still shows at every zoom level; when zoomed in far enough it sends the samples themselves. Where the log is missing data the line is broken, not bridged. "Missing" = the gap between two samples is larger than 2.5 times the **log's actual rhythm** (the median sample gap), not the configured rate: a loop running slower than its set rate gives an evenly sparse log, not a holey one (version 0.5 drew such logs as all scattered dots).

**Blackbox Explorer-style playback (0.31)** (`web/js/gfx/playback.js`, `logplot.js`):
- Playback button row:
  - go to the start of the log;
  - back / forward one record;
  - play / pause;
  - speed 5 % … 400 %.
- Whole-flight overview bar: draws the throttle stick (or throttle out to the motors, or roll gyro), with event markers, marking the region in view and the playhead.
  - Click or drag on the bar to jump there.
- The playhead (red) runs through every panel. While playing, the window always keeps the playhead in the middle, and the legend shows the values at the playhead.
- Data is preloaded one window ahead and half a window behind.
- Keys:
  - Space: play / pause;
  - ← →: one record at a time (Shift: ten records);
  - + −: change speed;
  - Home / End: go to the start / end of the log.
- Click on the chart (without dragging) to put the playhead there.
- Craft state panel at the playhead (fetches its own data, independent of the selected chart groups):
  - two mode-2 sticks;
  - motor bars;
  - roll / pitch / yaw;
  - throttle out to the motors.
- Tests: `tests/test_gui_playback.py` (node) and `tests/test_gui_viewer.py` (real browser, no board needed).

**PID analysis from a log (0.31)** (`wbclient/tune.py`, `/api/log/tune`, the "Analyse PID from this log" button, command line `python -m wbclient blackbox tune flight.wblog`):
- **Step response** per axis, like PIDtoolbox:
  - takes 2 s windows with stick inputs above 20 °/s, at most the 12 strongest windows;
  - applies a Hann window then Wiener deconvolution: h = IFFT(G·S̄ / (|S|² + λ)), λ = 1e-5·max|S|²; the step response is the cumulative sum of h over 500 ms;
  - drops windows whose final value is outside 0.5–1.5, then averages.
  - On a known second-order system (test), the error is below 0.04, and below 0.1 with 4 °/s gyro noise.
- **Measurements:**
  - peak (1.0 = no overshoot);
  - time to half step;
  - final value (200–500 ms);
  - number of oscillations across 1.0;
  - gyro noise above about 80 Hz;
  - D's share of the PID sum.
- **Suggestions** (at most ±25 % each time):
  - overshoot above 10 % → more D and D max; if D is already noisy then less P;
  - slow (reaching half step after more than 18 ms) without overshoot → more P / FF;
  - 3 or more oscillations → less P;
  - final value below 0.95 → more I;
  - noisy → do not raise D.
- The "Apply suggestions to the configuration (not saved)" button: only shown when connected to the board and the flight's configuration does not use sliders.
- Needs a log with the "Gyro and setpoint" group, ideally recorded at 1000 records/second or more.
- Does not need numpy; if numpy is available it is used to go faster.

**Library**
```python
from wbclient import wblog
for s in wblog.load("flight.wblog"):                 # one file can contain several sessions
    print(s.meta["firmware"], s.duration, len(s.fast["t"]))
    roll_rate = s.fast["gyro[0]"]                    # array('d'), engineering units; s.fast["t"] = seconds
    print(s.settings(schema)["pid"]["p"])            # that flight's settings
    for t, name, data in s.events: ...
```
`WbClient`: `blackbox_status()`, `blackbox_sessions()`, `blackbox_download(index)`, `blackbox_erase()`, `blackbox_record(seconds, rate_hz)`.

**Command line**
```
python -m wbclient --port COM5 blackbox status | list | download [N] [-o DIR] | erase
python -m wbclient --port COM5 blackbox record bench.wblog --seconds 10 --hz 200
python -m wbclient blackbox info flight.wblog          # no board needed
python -m wbclient blackbox csv flight.wblog [--slow] [-o flight.csv]
```

## 7. Verified (PC)

- `test_blackbox` 165 checks: packing, queue, sessions on simulated flash with correct NOR semantics (writes can only go 1 → 0), power loss midway, foreign data in the partition, interrupted erase, full region, the one-page-per-ms limit, download, streaming. The reader in the test (`test/wblog_reader.h`) is written independently of the firmware.
- `test_fw` (firmware-in-the-loop, 294 checks in total): flying in simulation with a model of loop stalls during flash writes, downloading the log, decoding it, **comparing it with what the virtual craft actually did** (angles, setpoint, motors, altitude, battery, GPS, events); the flash / stream rules; erase blocking arming.
- All 69 deliberate faults in the firmware are caught (`tools/mutate.py`).
- Python: `test_blackbox.py` (reader with logs hand-written byte by byte, fly → flash → download → decode, live streaming, foreign data, CLI), `test_logview.py` (reading a growing log, file library, decimated view not losing extremes), `test_gui_api.py` (download with progress, live recording, refusal while flying), `test_gui_browser.py` (real page: configuration, flying, download, **cursor values versus the log, line positions versus the time axis**, zoom, events, export, erase, live recording, viewing logs without a board, English page).
- SD card: `test_blackbox` (file names, byte queue, 4 kB blocks and 250 ms sync, session table, power loss = only the synced part remains, card pulled / reinserted, unformatted / write-protected / full card, more than 128 files, more than 4 GB, erasing only its own files, select / deselect / change wiring mode) with a simulated card with latency and power loss; `test_fw`: **flying while recording to a card with 180 ms latency, the flight loop loses no cycles, the file on the card = the download through the app = what the virtual craft did**, no card at boot, card pulled in flight, 32 kB buffer with a very slow card (lost records are fully counted). Python: `test_blackbox.py` (select card, fly, download, pull / reinsert, erase, reboot), real page in the browser.
- The firmware builds for ESP32-S3 N16R8 (`partitions/wblog_16MB.csv`) and N8; the Arduino-ESP32 SD / SD_MMC / FATFS libraries are in the ELF file.

## 8. Not verified (needs the real board)

- **Actual stall time when writing flash** (section 4) — the most important.
- `esp_partition_find/read/write/erase_range`: built only, not run. Whether the default `spiffs` partition is found; actual erase time.
- Log download speed over native USB CDC and over the UART bridge; at 230400 baud downloading 1.5 MB takes about 1 to 2 minutes (estimate).
- High-rate live streaming over a real link (in simulation the link has no speed limit).
- The chart page has only run in headless Chromium; not yet tried with logs tens of minutes long (the server decimates the data so it is expected to be fine, not measured).
- **Real SD card**: all of `bb_card_esp.h` (Arduino `SD` / `SD_MMC`, `File`, `flush()`) is built only, not run. Unknown: whether the card mounts at 20 MHz with your wiring; how long `usedBytes()` takes at boot on a 32 GB card; the card's actual write latency; whether `flush()` really updates the file size every 250 ms (pull the battery midway and see how much of the file is left); the SPI-mode SD library busy-waits during transfers — does it slow the app task on core 0; SDMMC 1-bit mode with the chosen pins; whether the stacks of the `wb_link` (12 kB) and `wb_sd` (8 kB) tasks are enough for FatFs.
- PSRAM: `ps_malloc(512 kB)` succeeds on N16R8 (the boot report has a `buffer 512 kB` line).
