# App: graphical configurator, client library, command line — `app/`

## Graphical configurator (GUI)

```
cd app && pip install -r requirements.txt      # only needs pyserial
python -m wbclient gui                         # opens the browser at http://127.0.0.1:8765/ ; pick the COM port in the page
python -m wbclient --port COM5 gui             # connect straight to COM5
python -m wbclient gui --http 9000 --no-browser
python -m wbclient gui --logs D:\\logs               # blackbox log directory (default ~/Windburst/logs)
```
No board: `make sim && build/wb_sim`, then press "Find ports" and pick "Simulator wb_sim".

**The new app works with old firmware** (0.43):
- The app carries the schema of every released firmware version, in `wbclient/schemas/` (`index.json` lists version, hash, file).
- On connect, the firmware reports its schema hash in HELLO. The app picks exactly that schema (`schema.pick_schema`) and reloads the page with it.
- Settings, modes and commands that the old firmware does not have are not shown; buttons for new features are greyed out with the note "This firmware does not have this feature yet".
- There is a notice "Firmware x.y.z is older than the app".
- **Firmware newer than any version the app knows**: the app refuses and says "update the app".
- **Unknown hash** (a development build): the app refuses, as before.
- The library is written every time `tools/gen_proto.py` runs: the entry for the version in `src/config/version.h`; a development version gets overwritten. `python tools/schema_library.py --from-git` rebuilds it from git history (currently 0.11.0 → 0.43.0).
- Verification:
  - `tests/test_compat.py`;
  - `test_gui_browser::test_an_older_firmware` runs against 0.41.0's own wb_sim, path set with `WB_OLD_SIM`, built with `git worktree add <dir> 80590f8` and then `make sim` inside it. Every tab renders, what 0.41.0 lacks is hidden, settings can still be written.
- With very old versions (before 0.37) some tabs may be missing items; the app says so in the tab rather than hanging.

**Long help behind a "?"** (0.42.1): help text for a setting or a mode longer than 140 characters is no longer printed under the field. Instead there is a "?" circle next to the field (in the Modes tab: next to the mode name). Hovering, tabbing to it, or tapping it shows a bubble that always stays inside the window; Esc or moving the mouse away hides it. Short help stays under the field as before. Code: `helpFor` / `helpTip` in `web/js/ui.js`, threshold `kHelpInline`. Test: `test_long_help_behind_a_question_mark`.

**Structure.** The GUI is a locally served web page, needs no Internet, has no build step, uses no external libraries:

```
browser  ──HTTP + server-sent events──  wbclient/gui.py  ──WbClient (WBP)──  USB serial / wb_sim
wbclient/web/index.html, app.css
wbclient/web/js/main.js        connection bar, the "why it cannot arm yet" line, Save button, tab switching
wbclient/web/js/store.js       schema, settings (exactly as the firmware holds them in RAM), telemetry, stream subscriptions
wbclient/web/js/ui.js          widgets bound to a schema field (number / enum / switch), command buttons
wbclient/web/js/labels.js      Vietnamese labels, units, explanations; blocker names; things the firmware does not support yet
wbclient/web/js/i18n.js        tr('Vietnamese string', {values}) → string in the selected language
wbclient/web/js/lang/en.js     the English version: EN (Vietnamese → English) and SAME (strings identical in both languages)
wbclient/web/js/tabs/*.js      one file per tab
wbclient/web/js/gfx/           fwmath.js (formulas identical to the firmware), craft3d.js (3D model), stickpreview.js (preview craft following the sticks), scope.js (scrolling chart), logplot.js (blackbox log chart)
wbclient/logview.py            log directory on the computer, decimated view of a log for the charts (see docs/BLACKBOX.md)
```
- The browser never sees the binary protocol: settings and telemetry arrive as JSON, in engineering units, enums by name (exactly the values the command line prints).
- Editing a field sends it straight into the firmware's **RAM** (0.2 s later), and the page shows back **the value the firmware actually accepted** (range-clamped). The "Save" button appears when the firmware reports `DIRTY`; when an item needing a reboot changed, the button becomes "Save and reboot". Editing back to exactly the old value hides the button by itself: the firmware compares with the copy in flash (since 0.37.1). "Discard changes" = reboot without saving.
- While ARMED: every field and command button is locked (the firmware refuses too).
- The flight controller reboots by itself (reset button, brownout) → `gui.py` notices the silence, asks again, re-subscribes the streams; the page re-reads the settings. Cable pulled → the page goes back to the connect screen with the reason.
- To make an Electron app later: wrap exactly this page (Electron runs `python -m wbclient gui --no-browser` and then opens the URL), or rewrite `gui.py` in Node. The interface does not depend on Python.

**Two languages (Vietnamese / English).** The selector is at the end of the tab bar; the default follows the browser's language, and the choice is remembered. Changing the language reloads the page (keeping the connection to the board).
- Source strings in the code are Vietnamese and always go through `tr()`. The English version lives in one file: `web/js/lang/en.js`.
- **Editing or adding a line of text**: write `tr('Chữ tiếng Việt {n}', { n })` in the code, add a line `"Chữ tiếng Việt {n}": "English text {n}"` to `EN` (or to `SAME` if both languages write it the same way: `Hz`, `Roll`, `DShot600`...). If you forget, `test_gui_i18n.py` reports exactly which strings are missing, which are extra, and which `{n}` do not match.
- **Adding a third language**: add a file `lang/xx.js` of the same shape, declare it in `LANGS` in `i18n.js`.
- `gui.py` contains no user-facing text: errors and status travel as **code + values** (`{"error": "OPEN_FAILED", "port": ..., "detail": ...}`, `note: {"code": "PORT_LOST", "port": ...}`), and the page builds the sentence in the selected language. Text sent by the firmware (boot report, log) is English to begin with and is shown verbatim.
- The documentation in `docs/` is in English; the Vietnamese originals are in `docs/vi/`.

**Serial connection.** The page connects to the board through the computer's serial port (COM on Windows, `/dev/ttyACM*` / `ttyUSB*` on Linux) via pyserial in `gui.py`; `socket://` is only used for the simulator.
- The port list includes the operating system's descriptions; ports with chips common on ESP32 boards (Espressif native USB, CP210x, CH34x / CH910x, FTDI) are sorted to the top and preselected. The list refreshes itself every 2.5 s while not connected, so plugging in the board makes it appear. "Enter another port…" lets you type any port name.
- The **baud** box next to the port box (115200 / 230400 / 460800 / 921600, default 230400 = `WB_LINK_BAUD` in `main.cpp`). Only matters for a USB-UART bridge; the ESP32-S3's native USB works at any baud. The choice is remembered.
- **Board not responding** (wrong port, wrong baud, no firmware flashed, firmware stuck in a boot loop): the page spells out each possibility and prints **what the port sent back** (the ESP32 boot log, "Guru Meditation", or all garbage characters with a wrong baud). Port cannot be opened: suggests closing the serial monitor that is holding the port.
- **One port, one program.** The client opens the port in exclusive mode (pyserial's `exclusive`), so on Linux / macOS just as on Windows: a port already held by a serial monitor or another configurator window reports "cannot open", rather than being shared with each side reading half of the replies. When a connection fails the port is released immediately (to retry, or to flash firmware).
- **Raw data** (System tab): every byte that does not belong to a WBP frame while connected is separated out and shown as text, for example when the firmware prints an error and reboots. The link keeps working normally because the parser resynchronises by itself.
- Cable pulled: the page goes back to the connect screen and says "Lost connection to port …".

**Tabs**

| Tab | What it has | Not yet in the firmware (the page says so) |
|---|---|---|
| Overview | 3D model following the attitude, module table (present / absent + the reason the firmware reports; plus an "Optical flow" row when a sensor is selected and a "Pin table" row when the firmware detects a pin conflict), list of reasons it cannot arm yet (including "control loop not reaching its rate", "just shut down because of runaway"), live readings, **CPU / RAM / PSRAM**, **actual rate / set rate and missed cycles, time of each stage of a cycle, failsafe with its cause** | |
| Receiver | 12 live channels following the channel map, link quality, protocol, channel order (AETR / TAER), RC smoothing, deadbands in us (roll / pitch, yaw, 3D throttle, as Betaflight), stick thresholds (low / centre / high: Betaflight's min_check / mid_rc / max_check), preview craft following the sticks (Angle by default) | SPEKTRUM, nRF24, PPM, PWM |
| Modes | AUX ranges for each mode (drag both ends of the range; bold mark = current switch position), actual on/off state; includes Position Hold and GPS Rescue | |
| Sensors | gyro calibration, 6-side accelerometer, level trim, **in-flight level trim** (measurement state: not measured / measuring % / measured, waiting for disarm; the page re-reads and tells you when the firmware saves it by itself), compass (point cloud); chip orientation; **board mounting wizard** (as the Betaflight App's: hold still lying flat, nose down about 45°, right side down about 45° - the firmware finds the chip orientation and board trims, rounded to 45°, and uses them at once; Save keeps them; a second IMU turns along); airmode + start throttle threshold; module enable/disable; GPS + set home | gyro 1/2 |
| Motors | frame picture with motor numbers and required spin directions, ESC protocol (motor idle only shown with DShot), per-motor test (RPM, telemetry errors), **bidirectional DShot status line: which ESC does not answer, skipped frames, missed replies**, reversing, motor order, raw / filtered gyro vibration chart, **3D mode** (reversible motors; the 3D pulses shown for PWM-type ESCs only) | DShot with more than 4 motors |
| PID | **tuning sliders** (dragging: only the number next to the slider changes; release: one write; the firmware computes P I D Dmax F and returns them, the fields of axes computed by the slider are locked), per-axis PID table, feedforward, I-term relax, anti-gravity, D max, TPA and throttle boost, Angle, Stabilize (per-axis P, acceleration, input time constant); **every other PID-type gain as well** (0.44.0): Altitude Hold's acceleration loop (P, I), Position Hold / Loiter (position P, velocity P I D), dynamic idle (min rpm, P I D, max increase; a WARNING in its block while the motors are not on bidirectional DShot, the only case the firmware runs it) - the Rate, Navigation and Motors tabs say where they went. Laid out in four titled tiers from the inner loop out, each in two columns: 1. rate PID, 2. rate controller, 3. holding the attitude, 4. holding height, position and motor idle; live values: TPA, anti-gravity, airmode, PID sum, **measured hover throttle** | |
| Rates | Actual-style rates + chart with a dot at the stick position, preview craft following the sticks (Acro by default), throttle curve, angle limits for Angle / Stabilize | |
| Filters | **frequency map** (each lowpass and the range it slides over, static notches, the current position of the dynamic notch per axis and the RPM notch per motor), two filter sliders (when enabled the cutoff fields are locked and show the result), gyro lowpass 1 / 2, static notch, RPM filter (says so when bidirectional DShot is not enabled), dynamic notch, D-term filtering, yaw lowpass | |
| **Position** | optical flow sensor: type, the line the firmware reports at boot, samples/second, surface quality, why samples are dropped, measured speed and a **circle with an arrow** to check the mounting orientation (craft nose at the top; orientation / mirroring / scale take effect immediately); position estimate (velocity source, position, nose heading from compass or gyro only, whether the tilt angle is being corrected); Position Hold (state, per axis, requested tilt angle, held point, wind compensation, gains); GPS Rescue and the failsafe procedure (ready or not, phase + give-up reason, distance to home, return altitude, every parameter); see `docs/NAV.md` | |
| **Mission** | automatic flight plan on a map (layout: the map fills the whole tab, floating panels): satellite / street imagery (cached on disk, usable offline in places already viewed), address search, the laptop's location (Windows location service), click to add a point, drag to move, clear all points, a per-point edit box (fly to / hover + servo step in degrees / land - wait - take off), metres per leg, drone + flight trail from GPS, status line (what it is doing, how far to go, time left waiting, battery, distance from home, whole trip), Upload / Download / Start / Stop buttons; see `docs/MISSION.md` | |
| Battery and buzzer | voltage, cell count, single-cell gauge against the thresholds, the thresholds, ADC divider calibration; table of 18 situations with **two columns: buzzer and motors**, tone pattern, number of beeps the motors play, a "Test" button; motor tones; buzzer type (see `docs/POWER.md`) | current measurement, mAh |
| Blackbox | choose destination (**flash / SD card**) / condition / rate / field groups; card wiring mode and speed; log memory (download, erase, card buffer fill level); live recording over the cable; log library on the computer; **charts** with several panels sharing the time axis, zoom, cursor value readout, events, CSV export. Viewing logs needs no board connection (see `docs/BLACKBOX.md`) | USB drive (mass storage) |
| Pins and ports | GPIO per module (including battery ADC, buzzer, SD card, **optical flow**: pins of the sensor type not selected do not count as conflicts, as in the firmware), warnings for duplicate pins / ESP32-S3 strapping / USB / flash / PSRAM pins, pins that cannot read ADC; a **"Fill in the suggested pin set"** button for the DevKitC N16R8 (see `docs/BOARD_N16R8.md`) | remapping UART functions |
| System | failsafe (delay, time the signal must be stable again, limit when it cannot get down), **runaway takeoff prevention**, backup / restore, reset to defaults, firmware log, the port's raw data | |

Top bar: while failsafe is active it says "Failsafe: landing by itself (signal lost)" (phase + cause); the "Loop" box shows `actual / set Hz` and changes colour below 80 %.
Blackbox tab: the log info shows "actual … records/second" when the log is sparser than configured; pressing download while live recording stops the recording session by itself (the file is kept) and then downloads.

Not yet available: a tab for the TX.

How to tune and what each slider means: `docs/TUNING.md`.

**Safety of the local server** (it can spin motors, so it must not obey just any open web page): listens only on `127.0.0.1`; every `/api` call must carry a random token embedded in the page it serves itself; checks `Host` and `Origin` (blocks DNS rebinding and cross-site requests); POSTs must be `application/json`.

**Motor test**: the sliders stay locked until "All props removed" is ticked; the page resends the command every 0.2 s and the firmware stops the motors by itself if nothing arrives for 0.6 s, so if the page hangs or the cable is pulled the motors stop; leaving the tab or hiding the window stops them too.

**Drawings must not drift from the firmware**: `tools/ui_reference.cpp` prints mixer rows, rate curves, throttle curves, deadband as computed by **the firmware code**; `app/tests/test_gui_math.py` compares them with `js/gfx/fwmath.js` (run with node).

## DIY transmitter (0.38)
When the transmitter (firmware `tx_c5`, `docs/TX.md`) is plugged in, the app recognises it through `role` in `DEVICE_INFO` and the tab bar shows only the transmitter's 4 tabs: **Overview** (two stick boxes, 8 channels, link to the drone, drone telemetry, a warning when the throttle mode does not suit the flight mode), **Joystick** (throttle mode, calibration, axis assignment, reversing, deadzone, expo, ADC values), **Radio** (bind, frequency channel, rate, power, packet counts, nRF24 pins), **System** (version, boot report, reset to defaults, reboot). Top bar: drone connected / not connected, packet count, drone battery, Save button driven by the `settings_dirty` flag of the `tx` stream. Plugging the flight board back in brings back the flight board's tabs. The flight board's Receiver tab has an nRF24 panel (bind, status, LQ) when the protocol is NRF24; the Pins and ports tab has an nRF24 Receiver group. Verified: `test_gui_browser.py` (`test_tx_*`, `test_receiver_nrf24_block`).

## Client library and command line

```
app/wbclient/framing.py     WBP frames + self-resynchronising parser (Python version of wbp_codec.h)
app/wbclient/schema.py      reads protocol/wbp_schema.json: layout, scale, enum, bit → "human-readable" values
app/wbclient/client.py      WbClient: hello, get/set/save/defaults/reboot, subscribe, command, report, blackbox_*
app/wbclient/wblog.py       reads blackbox logs (.wblog): docs/BLACKBOX.md
app/wbclient/__main__.py    python -m wbclient ...  (command-line configurator, and the `gui` command)
app/wbclient/gui.py         the GUI's local server;  app/wbclient/web/  the web page
app/tests/                  pytest: framing, schema, and end-to-end tests with build/wb_sim
app/requirements.txt        pyserial
```

## Library

```python
from wbclient import WbClient, WbError

with WbClient("COM5") as fc:                      # or "/dev/ttyACM0", or "socket://127.0.0.1:5760" (wb_sim)
    print(fc.info)                                # name, version, role, schema hash, features
    print(fc.report())                            # which sensors were found / why not

    pid = fc.get("pid")                           # dict, engineering units
    stored = fc.set("pid", p=[0.05, 0.05, 0.012]) # returns the value the firmware ACTUALLY stored (range-clamped)
    fc.set("motors", protocol="DSHOT600")         # enum by name
    fc.save(); fc.reboot()                        # reboot() waits until the firmware answers again

    fc.subscribe("attitude", 50, lambda v: print(v["roll"], v["pitch"]))
    fc.subscribe("status", 10)
    st = fc.wait_stream("status")                 # {'armed': 0, 'blockers': ['RX_LOST'], 'health': ['IMU', ...], ...}

    try:
        fc.command("MOTOR_TEST", motor=0, value=100)       # 10 %, must be resent every ≤ 0.5 s
    except WbError as e:
        print(e.name)                             # 'ARMED', 'BAD_ARG', 'NO_SENSOR', ...
```
- Opening the port sends `HELLO` and compares the **schema hash**; a mismatch → `SchemaMismatch` (settings are never misread).
- One reader thread decodes everything: replies are matched by `seq`, streams go into `latest()` / callbacks, firmware logs into `fc.logs`.
- Settings read/write requests are resent automatically on timeout; **commands are not** (a command with side effects is never repeated).
- Scales, enums and bits are converted already: `0.1°` → degrees, `blockers` → list of names, empty mode slot → `None`.

## Command line

```
cd app && pip install -r requirements.txt
python -m wbclient --port COM5 info
python -m wbclient --port COM5 get pid
python -m wbclient --port COM5 set pins imu_cs=10 imu_miso=13 imu_mosi=11 imu_sck=12 motor=4,5,6,7,255,255,255,255
python -m wbclient --port COM5 set modes mode[0]=ARM aux[0]=0 lo[0]=1700 hi[0]=2100            # mode slot 0: ARM when AUX1 is at 1700..2100
python -m wbclient --port COM5 set modes mode[1]=ANGLE aux[1]=1 lo[1]=1700 hi[1]=2100
python -m wbclient --port COM5 save
python -m wbclient --port COM5 reboot
python -m wbclient --port COM5 monitor attitude,status,rc --hz 20
python -m wbclient --port COM5 calibrate gyro | level | acc | mag
python -m wbclient --port COM5 motor 0 10 --seconds 2 --props-off
python -m wbclient --port COM5 backup my_quad.json      /   restore my_quad.json
python -m wbclient --port COM5 blackbox status | list | download [N] | erase | record FILE     # docs/BLACKBOX.md
python -m wbclient blackbox info FILE | csv FILE        # reads a log file, no board needed
python -m wbclient schema                               # lists sections, streams, commands
```
Array fields: enter the whole array (`motor=4,5,6,7,255,255,255,255`, must have **every element**) or one element (`motor[2]=6`). Changing many things at once: `backup` → edit the json file → `restore`.

No board: `make sim && build/wb_sim`, then use `--port socket://127.0.0.1:5760`.

## Verification

`make apptest` (`pip install -r app/requirements-dev.txt`; browser tests need `playwright` + Chromium, formula and translation tests need `node`, and are skipped if missing):

| File | Tests | Content |
|---|---|---|
| `test_framing.py`, `test_e2e.py` | 29 | framing and parser (noise, fake headers, bit errors), json matching the C++ header, **every field's default = the firmware's**, end-to-end scenarios of the library and CLI with the real firmware in `wb_sim`; battery (ADC, live thresholds), buzzer and silence on the bench; since 0.7.0: optical flow sensor (carrying the craft around, orientation, Position Hold in wind) and GPS Rescue from start to landing |
| `test_blackbox.py` | 8 | `.wblog` reader with logs hand-written byte by byte (clock wraparound, corrupt frames, truncated files), fly → simulated flash → download → decode → compare with the virtual craft, live streaming, foreign data in the partition, CLI |
| `test_logview.py` | 5 | reading a growing log = reading it in one go; the decimated view keeps every extreme; log directory (safe file names, no overwriting, re-read when a file changes); live recording goes to the file and to the chart at once |
| `test_gui_api.py` | 18 | the GUI server and the `gui` command: token / Host / Origin, errors as code + values, set + clamp + save + reboot, streams, refusal while ARMED, firmware resetting by itself, cable pulled, backup / restore; **serial through a real OS device** (pseudo-terminal connected to `wb_sim`): baud, telemetry, stray text between frames, cable pulled, port not answering (with / without data) and released afterwards, port held by another program; **blackbox logs**: download from the board with progress, one job at a time, decimated view, CSV, flight settings, delete / add files, live recording with the chart following, refusal while flying |
| `test_gui_store.py` | 1 | the page's settings store (run with node): two consecutive edits of the same section reach the firmware in the right order, none lost |
| `test_gui_math.py` | 7 | the page's formulas = the numbers the firmware computes (mixer rows for 3 frame types, rates, throttle, deadband, 3D model rotation), buzzer tone patterns |
| `test_gui_i18n.py` | 5 | every string on the page has an English version (or is in `SAME`), no extra entries, `{values}` match, no Vietnamese string outside `tr()`; **every blocker / failsafe cause / driver status the firmware can report has text** |
| `test_gui_browser.py` | 25 | the real page in headless Chromium + the real firmware in `wb_sim`: connect, edit / clamp / save / power loss / reboot / discard changes, Modes tab, motor test (measured at the virtual ESC), vibration chart, accelerometer + level + compass calibration, pins tab, lock while ARMED, port lost, restore / defaults; **English page: all 13 tabs and the special states contain no Vietnamese text**, language change is remembered; connecting through a serial port from the page, explanation when the port does not answer; Battery and buzzer tab (thresholds, ADC calibration, buzzer measured at the virtual GPIO pin); **Blackbox tab**: configure → fly → download → chart, cursor values and line positions versus the log, zoom, events, export, erase, live recording, viewing logs without a board; **Position tab**: the firmware's sensor report, the speed arrow while carrying the craft (measured in pixels), fixing orientation without rebooting, Position Hold in wind, rescue refused without a compass, flow sensor pins |

Every browser test also checks that the page logs no errors to the console. Mutation test (`python3 tools/mutate_gui.py`).

**Not verified**:
- COM ports on Windows and a real ESP32-S3 board (native USB CDC or a USB-UART bridge). The serial path has only been run through a pseudo-terminal on Linux: same pyserial code, but no USB driver, no real baud, no DTR/RTS. Exclusive mode on Windows relies on COM's built-in behaviour, untested. Whether holding DTR/RTS low prevents the board from resetting when the port opens is still unknown.
- Port detection by USB chip VID (the list in `gui.py`) has not been tried with any board.
- The GUI has only run in headless Chromium on Linux. Not yet opened in Chrome / Edge / Firefox on Windows; the Bahnschrift font (digits) only exists on Windows so screenshots during development use a substitute font.
- `wb_sim` on Windows (the winsock code is written but has not been test-compiled).

## Preview craft following the sticks (0.24.1)
The **Receiver** tab and the **Rates** tab have a "Craft following the sticks" block: a 3D model of the craft that moves with the sticks being moved on the transmitter (stream `rc`). This is only a simulation inside the app; nothing is sent to the firmware.
- **Acro** (default in the Rates tab): the craft rotates at exactly the rate the rate curve gives for the current stick position. Release the stick and the craft stays in its current attitude. Angles are accumulated the same way as the firmware's attitude estimator when using gyro only (`acroStep`).
- **Angle** (default in the Receiver tab): the roll / pitch tilt equals `AngleController`'s target angle (angle limit × rate / max rate). The yaw stick turns the nose heading.
- The "Level" button puts the craft back level, nose pointing away from the viewer. On signal loss the craft stands still and the page says "No signal yet".
- **Verification:** `test_gui_math::test_stick_preview_flies_like_the_firmware` compares `acroStep` with the firmware's gyro-only `Ahrs` (6 chained rotation segments), and compares `angleTarget` with `AngleController` (198 cases, including center sensitivity larger than max rate). `test_gui_browser::test_stick_preview` is written but could not be run on this machine yet.

## Standalone program (its own window, no browser)

`dist/Windburst.exe` (about 14 MB) is a single ready-to-run file. It already contains Python, wbclient, the web page and the schema. Opening it shows **its own window**, without opening a browser and without a console window.

- **Inside:** the window uses Windows' Edge WebView2 (the pywebview library). Windows 11 ships WebView2; Windows 10 has it if updated.
- **Server:** runs only on 127.0.0.1, on a free port. Closing the window stops the server and releases the serial port.
- **Page settings** (theme, language, baud): stored in `%LOCALAPPDATA%\Windburst\webview`, so they survive reopening. Flight logs are still stored in the same place (`~/Windburst/logs`).
- **Run from source, no build needed:** `python -m wbclient app` (needs `pip install pywebview`). The `python -m wbclient gui` command still opens in the browser as before.
- **Rebuilding**, run in the `app` directory:
  ```
  pip install pywebview pyinstaller pyserial
  python packaging/build_exe.py
  ```
  The result goes to `app/dist/Windburst.exe`. `dist/` and `build/` are not committed to git.
- **Icon:** `packaging/windburst.ico`, drawn by `packaging/make_icon.py` (needs Pillow).
