# RX firmware: `main.cpp`, `FlightApp`, the app link, simulation on a PC

## 1. Structure

```
src/main.cpp                 setup(): NVS → settings → FlightApp.configure/begin → flight task → sensor task → link task → delete the Arduino loop task
src/app/flight_app.h         firmware body: drivers + FlightCore + telemetry + handling of app commands
src/app/flight_task.h        `wb_flight` task + hardware timer that wakes it every control cycle; shared lock (FreeRTOS mutex)
src/app/sync.h               lock (interface), per-stage timing, "is the loop holding its rate"                 [portable]
src/app/tx_queue.h           8 kB transmit queue shared by UART / USB Serial-JTAG / USB CDC                          [portable]
src/config/fast_code.h       WB_FAST = IRAM_ATTR for control-cycle code (docs/HARDWARE_FEEDBACK_1.md section 2)
src/app/telemetry.h          app::Telemetry (engineering units) → wire struct (scale, saturation, NaN→0)      [portable]
src/app/exchange.h           the only bridge between the flight loop and the link task (seqlock + 1-request mailbox)      [portable]
src/app/link_host.h          wbp::Host for the link task + check_request() for the flight loop                          [portable]
src/app/link_arduino.h       SerialTransport, NvsStorage (Preferences), LinkTask (FreeRTOS, core 0), EspEscPort (ESC passthrough)
src/app/msp_bridge.h         just enough MSP for ESC tools (BLHeliSuite32 ...) on the same port as WBP          [portable]
src/hal/motor/esc4way.h      4-way interface + BLHeli bootloader on the motor wire (docs/ESC_PASSTHROUGH.md)     [portable]
src/proto/*                  WBP framing, server, settings storage, settings → configuration (see docs/PROTOCOL.md)       [portable]
src/hal/rx/*                 serial receivers CRSF / SBUS / IBUS (see docs/RX.md)
src/hal/battery/*, beeper/*  battery measurement (ADC / ESC), beeper and motor beeps (see docs/POWER.md)
src/blackbox/*               flight log: queue, raw flash, SD card, stream to the app (see docs/BLACKBOX.md)
src/config/pin_check.h       pin-table check at boot: duplicate pins, flash / PSRAM / USB / SWD pins  [portable]
src/app/sysload.h            CPU % of the core running the flight loop, RAM, PSRAM                                       [portable]
src/main_stm32.cpp           STM32H5 version of main.cpp (0.10.0, docs/STM32.md)
src/config/version.h         firmware version number, shared by both chips
src/port/*                   chip-specific parts: UART, PWM, GPIO / ADC, FreeRTOS, pin names, quad-word flash (docs/STM32.md section 4)
src/proto/kv_flash.h         settings on the STM32H5 ECC flash (ESP32: NVS)                               [portable]
src/blackbox/bb_qword.h      FlashLog on the STM32H5 ECC flash                                             [portable]
```
The default board is the ESP32-S3 **N16R8**: env, partition table, unusable pins, suggested pin set, SD card → `docs/BOARD_N16R8.md`.

| Core | What runs | Notes |
|---|---|---|
| core 1, task `wb_flight` (priority 20) | `FlightApp::flight_cycle()`: IMU → FlightCore → motors → DShot replies → blackbox record. Woken by a **hardware timer** exactly every `1 / loop_hz`; code lives in RAM (`WB_FAST`) | preempts everything else on core 1; a tick that arrives while the previous cycle is still running is dropped (counted in `status.overruns`), not caught up. The task keeps a **time budget** (`app::TickBudget` / `app::TickLoop` in `sync.h`): the sensor thread always gets ≥ 20 % of the core, even when a cycle is longer than the set period; a cycle that overruns its tick is followed immediately by the next one (catching up at most one tick) as long as the budget allows; the sensor thread never waits longer than two set periods + two control cycles |
| core 1, task `wb_sensor` (priority 1, 16 kB stack; since 0.8.8, before that it was Arduino's `loop()`) | `FlightApp::slow_update()` every ~1 ms. Phase A, **without the lock**: receiver, GPS, battery measurement, **one** I2C sensor (baro / lidar / compass in turn). Phase B, holding the lock: feed results into the core, beeper, level trim, app requests. Phase C (0.8.8), **three short lock holds** with releases in between: per-second statistics + health, slow telemetry, CRSF telemetry + slow blackbox record | a 0.3 ms I2C transaction no longer shifts the control cycle; a cycle that falls due midway only waits for one piece, not for all of B + C |
| core 0, task `wb_link` | `wbp::Server`: read bytes, reply, send streams; `bb::Service`: take blackbox records from the queue, write flash, push into the SD card buffer, or send to the app | may be blocked by USB without affecting the flight loop; **writing flash stalls both cores** (docs/BLACKBOX.md section 4) |
| core 0, task `wb_sd` | `bb::CardLog::work()`: writes the buffer to the SD card in 4 kB blocks, syncs every 250 ms; looks for a card every 3 s when none is present | only active when `blackbox.device = SDCARD`; a slow card only slows this task |

Boot order (`setup()`): `Serial` → NVS → `SettingsStore::load()` (a corrupt/missing section falls back to its own defaults) → `FlightApp::configure(settings)` → `FlightApp::begin()` → flight task → sensor task → send the boot report as LOG (the app fetches it again with the `REPORT` command) → link task, SD card task → `vTaskDelete(NULL)`: the Arduino loop task deletes itself, `loop()` never runs (only if creating the sensor task fails does `loop()` take over that job, and the report has the line `loop: FAILED to start the sensor task`).

## 2. `FlightApp`

- **Pins**: the app sends the `pins` section → the firmware keeps it in RAM (`wbp::Settings::pins`) → `SAVE` writes it to NVS (`wbp::SettingsStore`, key `wbs11`) → on every boot `FlightApp::begin()` reads it back and loads it into **`HardwareConfig::setAppPins()`**. Every driver asks for its pins through `HardwareConfig` getters (`getSPIAccGyroSensorPins()`, `getMotorPins()`, `getSerialRxPins()`...): pins sent by the app take priority, a slot the app leaves empty (255) uses the default in `src/config/hardware_config.h`. Since they are only loaded at boot, changing pins requires save + reboot. The IMU uses SPI when it has a CS pin, otherwise I2C. UART1 = GPS, UART2 = receiver.
- **Loop rate** (gyro rate) is set = min(`general.loop_hz`, the chip's gyro rate); an I2C IMU is limited to 1 kHz. The control part runs every `general.pid_denom` samples (`docs/LOOP_RATES.md`). This value is written in the boot report and used to configure the filters. The **achieved** rate and the per-stage timing (IMU read / core / motors / rest, averaged per second) are in the `status` stream; below 80 % for 2 seconds in a row (not counting seconds in which the app is erasing the log, calibrating…) → blocker `LOOP_RATE`.
- **Two threads, one lock** (`sync.h`): everything the two threads share (core, IMU, motors, telemetry) may only be touched while holding the lock; no device call is made while holding the lock. `test_threads` runs exactly this structure with real threads under ThreadSanitizer.
- The `dt` of each cycle is the real time since the previous cycle (clamped to 0.25 … 8 times the set period); the estimator steps by the total real time.
- **Non-blocking gyro calibration**: runs right after boot (~1 s); if the craft is moved it retries by itself until it succeeds (blocker `GYRO_NOT_CAL`).
- **6-side accelerometer calibration** (`hal/acc_gyro/acc_calibration.h`): place the craft on its 6 sides in any order, the firmware recognises the side by itself; it fits offset + scale so that all 6 readings are exactly 1 g long → a table tilted by a few degrees does not spoil it (the usual (max+min)/2 method does). Stored in the chip's axes, before board alignment.
- **Level trim**: computes alignment roll/pitch angles so that the attitude the craft is resting in reads as level; added to the board trim the user enters. **In-flight level trim** (CALIB mode): measures the accelerometer during a steady hover, applied after disarm, the firmware writes it to flash by itself (`docs/TUNING.md` section 7).
- **Dead IMU**: 32 consecutive samples identical bit for bit (a broken MISO wire reads 0xFF/0x00 = a very plausible "standing still") → treated as a fault; 20 consecutive faulty cycles → actively stop the motors, disarm, blocker `NO_IMU`, the arm switch has to be toggled again.
- **Telemetry**: fast fields are updated every cycle, published at ≤ 1 kHz; slow fields every ms.
- **App commands**: `execute()` — see the command table in `docs/PROTOCOL.md`.
- **Pin-table check** (`check_pins()` → `pin_check.h`): runs once in `begin()` over the pins in use. Duplicate pin → `status.pin_conflict` → blocker `PIN_CONFLICT` (cannot arm) + a `pins: FAILED - …` line in the report; flash / PSRAM pin → a `pins: WARNING - …` line.
- **CPU / RAM** (request: "show how many % of cpu and ram are loaded"): `status.cpu` = percentage of time core 1 was busy (control cycle + per-ms work) in the last second; 100 % means the loop can no longer hold its rate. `status.ram` = internal RAM in use, `heap_free`, `psram_free`, `psram_total` (kB). Core 0 (app, SD card) is not included in the CPU figure.

Known limitations:
- Version 0.5 measured on a real board reached only ~1500 Hz with 4000 Hz set. Three causes and their fixes: `docs/HARDWARE_FEEDBACK_1.md` sections 1 – 2. **Figures for this version on a board: not available yet.**
- Writing flash (blackbox to flash, saving settings) stalls **both cores**, including code in RAM; only specially registered interrupts keep running.
- Libraries the control cycle still calls in flash (libm, Arduino SPI, a few gptimer functions): the list is printed by `tools/iram_report.py`.
- Battery: ADC or ESC telemetry, see `docs/POWER.md`. With no measurement source, arming is not blocked.
- The `ports` section is only stored so far.
- SPI buses on the ESP32-S3: see `docs/BOARD_N16R8.md` section 6 (the bus-number bug was fixed in `bus_spi.h`).
- Blackbox writing to flash stalls the flight loop on every page write: the real numbers have not been measured, see `docs/BLACKBOX.md` section 4 (`status.gap_max_us`, the `gap_us` field in the log).

## 3. Running the firmware on a PC

`test/virtual_board.h` is a virtual FC board: **the real firmware code** (`FlightApp`, drivers, CRSF decoder, core, motor manager, WBP server) runs on register-level simulated chips (ICM-42688-P over SPI, BMP280 + QMC5883L over I2C, u-blox on UART1, CRSF receiver on UART2) and a quadcopter model. The DShot frames the firmware sends are decoded to spin the model's "motors". Only the ESP32 peripherals (SPI/I2C/UART/RMT/NVS/FreeRTOS) are replaced by `test/arduino_stub`.

```
make test        # 20 C++ test programs (including test_fw: firmware-in-the-loop, test_threads: two real threads)
make sim         # build/wb_sim
build/wb_sim     # the app connects to  socket://127.0.0.1:5760 ; port 5761 = "the tester's hand"
make apptest     # Python client ↔ wb_sim (needs python3, pyserial, pytest)
```

The `wb_sim` control port (one command per line, replies `ok`): `rc <channel 1..16> <µs>`, `tx on|off`, `pose <roll> <nose_down> <yaw>`, `shake 0|1`, `batt <volt>`, `logmem garbage`, `sdcard pull|push|slow <n>|put <number> <byte>` (64 MB virtual SD card, held in RAM), `mount <roll> <pitch>`, `accerr ox oy oz sx sy sz`, `magerr hx hy hz`, `vib <axis 1..3> <Hz> <°/s>` (vibration on the gyro), `weak <motor 1..8> <factor>` (weak motor), `swap <motor> <motor>` (two ESC wires swapped: what the runaway check catches), `esc <motor|0 = all> <0|1>` (ESC stops / resumes replying with rpm), since 0.7.0: `carry <z> <vn> <ve>` / `carry off` (carry the craft at height z, m/s north / east), `wind <north> <east>`, `flow quality <0..255>|dark <0|1>|plug <0|1>|mount <degrees> <flip 0|1> <factor>`, `gps fix <0|1>`, `gps dead <0|1>`, since 0.8.0: `adc <gpio 1..20> <mV>` (voltage at an ADC pin, e.g. a current sensor), `escamps <motor> <A>` (ESC current reported via extended DShot telemetry), `rclink crsf|spektrum2048|spektrum1024|ppm|pwm` (receiver on the virtual board: PPM on GPIO40, PWM on GPIO39, 40, 41, 42, 1, 3, 45, 46), `state` (with `pn pe vn ve` = true position / velocity, `m0..m3` = the commands the virtual ESCs actually receive, `tones` = number of ESC beacon commands received, `sd_files`, `sd_writes`), `quit`. The virtual craft has horizontal motion with drag and a **physically correct accelerometer** (in flight it only senses thrust + drag): holding roll stick really makes it fly off, and the horizon tilts a few degrees under acceleration as on a real craft (`docs/TUNING.md` section 5). In `wb_sim` the virtual craft has landing gear: test-running a single motor does not flip it (not so in `make test`, where the closed-loop tests are written for a free body). Options: `--port N --flash FILE --fresh --speed X --no-baro --no-mag --no-imu --no-flow`. The `wb_sim` virtual board connects to the app over USB (UART0 free) and has a MicoAir MTF-01 on UART0 (pin 44) plus a PMW3901 on the second SPI bus; the first boot pre-fills their pins. The virtual flash is saved to a file, so settings survive to the next run; `REBOOT` really restarts the virtual firmware.

Use `wb_sim` to write app UI without a board: arm with `rc 5 2000`, fly with `rc 3 1700`, etc.

## 4. Verification

| Test | Checks | Content |
|---|---|---|
| `test_proto` | 165 | framing, CRC, self-synchronisation, schema, flash storage, server, streams, settings → configuration |
| `test_rx` | 43 | CRSF / SBUS / IBUS with independently written encoders, noise, resync, link state |
| `test_motor` | 183 | DShot frames, DShot commands, PWM pulses, GCR / eRPM / EDT, telemetry, dynamic idle |
| `test_dshot` | 91 | the direct DShot driver on a **signal-wire model** (docs/MOTOR_OUTPUT.md) |
| `test_power` | 123 | battery and beeps |
| `test_blackbox` | 301 | format (real PID values, per-stage timing), queue, simulated NOR flash and SD card, download, streaming, link-bandwidth limit |
| `test_app` | 277 | telemetry packing, Exchange with real threads, transmit queue, lock, loop statistics and **the flight task's time budget on a simulated timer**, calibration, pin table |
| `test_fw` | 662 | **firmware-in-the-loop**: boot, arming over CRSF, flight, failsafe (causes, handing control back), runaway, loop rate with simulated I2C / SPI cost, per-stage timing in the log, settings, calibration, battery, beeper, blackbox flash / SD card, duplicate pins; since 0.7.0 optical flow (MSP on UART0, PMW3901 on the second SPI bus, mounting orientation, UART assignment), Position Hold, GPS + compass, GPS Rescue, NAV log group |
| `test_threads` | 17 | firmware with **two real threads + the link thread under ThreadSanitizer**; no bus transaction while holding the lock |
| remaining tests | 1639 | flight core (`test_core` 296, `test_control` 269, `test_nav` 130, `test_flight` 251: see `docs/CORE.md`, `docs/NAV.md`), IMU, baro, lidar, optical flow (`test_flowsensor` 94), GPS, compass |
| `app/tests` (pytest) | 98 | Python client + CLI + **GUI** (server, formulas, translations, the real page in a browser) with `wb_sim`; see `docs/APP.md` |

In total 20 C++ programs, 3424 checks.

Mutation test (`make mutate`, deliberately breaks the code, the tests must report it): **1721** deliberate faults in `tools/mutate.py` (0.8.8: 17 new faults or faults rewritten for the new code, run separately: all 17 caught; 27 old faults no longer match the code because earlier rounds changed it, not rewritten yet), 253 in `tools/mutate_gui.py`.

**Run on a real board**: version 0.5 (boot, ICM20602 IMU over SPI, arming, motors spinning with one-way DShot, app over UART and over the S3's USB, blackbox recording); version 0.6.1, props off, from the log the user sent (`docs/HARDWARE_FEEDBACK_2.md`): flight task + timer, code in RAM, new DShot driver (one-way DShot600), CRSF receiver, USB transmit queue with seamless live logging at 250 records/second for 87 seconds; measured loop ≈ 3300 Hz out of 4000. **Not yet run on a board**: all of 0.8.0 (current measurement, Spektrum / PPM / PWM receivers, CRSF telemetry, port table), servos (0.7.1), all of 0.7.0 (optical flow, Position Hold, GPS Rescue: `docs/NAV.md` section 8), every change in 0.6.2 (the flight task's time budget, the new log columns), bidirectional DShot, real flight. Also still unchecked: NVS, the real time of one cycle (`status.loop_us` and the four per-stage figures), inverted UART for SBUS, I2C interference with the 4 kHz loop, ADC read time, **the stall time while the blackbox writes flash**, a real SD card, whether ESCs beep on beacon commands, PSRAM. Ordered test list: `docs/BOARD_N16R8.md` section 7.

### The USB Serial/JTAG port stops replying (0.8.5)
User report: with the app connected over USB Serial/JTAG, the app occasionally keeps reporting "Firmware is not responding". One path to exactly that symptom, read from the Arduino core code (`HWCDC.cpp`): the transmit ring buffer is full, and the interrupt that drains it is not enabled (USB just went through a bus reset, the host missed one poll, the core's SOF watchdog mistakenly reported "cable unplugged"). The core only re-enables that interrupt in `write()`; our queue (`tx_queue.h`) only calls `write()` when the port reports free space, so nobody calls it any more: the link is dead until the cable is unplugged. Since 0.8.5:
- If the port reports no space for 50 ms in a row: the queue still hands over **one byte** (its next byte; whether the port accepts it or not, it is neither lost nor duplicated), and the core re-enables the interrupt (`app::StallKick`, `link_arduino.h`). The core's write timeout is set to 0 (`setTxTimeoutMs(0)`): that write never blocks.
- Every time the port has to be "kicked", the app receives a log line: "link: the USB port took nothing for 50 ms and was restarted (N times since boot)".
- The boot report gains a **`reset:`** line: why the chip booted (power-on, software restart, CRASH panic / watchdog, BROWN-OUT, reset pin, USB). The app notices on its own that the firmware restarted and re-subscribes the streams; this line says why.
Verification: `test_app` (USB ring buffer full with nobody draining it: without the "kick" the link is dead for good, with it every byte arrives after 50 ms, in order; no kick earlier than 50 ms). **Not yet tried on a board**, and this is only one possible cause: if the app still reports no response, send me the `reset:` line and the last log lines.

### "Firmware is not responding" and "software restart" (0.8.8)
The user saw the app report that nothing was arriving from the firmware, and the log contained a "software restart" line. Re-reading the code:
- **"software restart" is not caused by that fault**: there is only one place in the firmware that calls `ESP.restart()` on request: the app's REBOOT command (Save + reboot, Discard changes). The `reset:` line is the reason for the **actual most recent** boot. Before 0.8.8, whenever the streams went silent for 1.5 s while the firmware still answered HELLO, the app assumed the firmware had just restarted and re-read the boot report, so it saw the old `reset: software restart` line again.
- Since 0.8.8 the app can tell the difference: the new **`link`** stream (1 Hz, always subscribed by the app) carries `boot_id` (a random number drawn at every boot, `esp_random()` mixed with a counter in RTC and the µs clock) and the uptime. Unchanged `boot_id` = the firmware did **not** restart: the app reports "Data stopped for N seconds but the firmware did NOT restart" and writes the link figures into the journal; changed = a real restart.
- Those figures (System tab, "Link to the app" block): requests not handled in time (BUSY), longest command wait, **longest idle of the link thread**, **longest idle of the sensor thread** (it runs the app's requests), dropped telemetry frames, lost replies, number of USB port "kicks", bytes waiting to be sent. Next time the fault happens, these figures show where it lies: sensor thread idle for tens of ms = core 1 is taken over entirely by the loop (lower `loop_hz`); link thread idle for long = core 0 (flash writes, SD card); "kicks" increasing = USB.
- At the same time a real cause was reduced: the 0.8.7 log showed the flight task waiting for the lock with a median of **236 µs** (`t_lock`): phase B of the sensor thread held the lock in one stretch including the slow telemetry part. 0.8.8 cuts that part into three pieces, releasing the lock between them.
Verification: `test_app` (`link` stream: uptime, kicks, idle gaps of the link task, bytes in the queue), pytest `test_a_pause_of_the_streams_is_not_called_a_restart` (wb_sim command `linkpause`: the streams stop for 3 s but replies keep coming → STALLED, not REBOOTED) and `test_a_restart_behind_our_back_is_noticed`. **Not yet tried on a board.**

## 5. Bugs found during integration (fixed)

- `now_ms = micros()/1000` in the fast loop drifted away from `millis()` after 71.6 minutes (micros overflows 32 bits) → every "stale data" check was wrong. Now uses `millis()`; there is a test that flies through the overflow moment.
- An empty mode slot (255) was clamped by `validate()` to mode 7 (BLACKBOX). Now `optional` slots keep 255.
- `validate()` let NaN through (NaN compares false with everything) → a faulty app could feed NaN into the PID. Now NaN → default, every mandatory `f32` has a min/max.
- IMU not responding: the flight loop stalled silently, the app still saw "no blockers". Now reports `NO_IMU`, stops the motors.
- A broken MISO wire reads valid zeros → added "frozen chip" detection.
- **Wrong SPI bus numbers on the ESP32-S3** (`bus_spi.h` used 2 / 3 from the original ESP32; the S3 only has 0 / 1): an SPI IMU would never start on the N16R8 board. Found while reading the Arduino core code to attach the SD card to the second SPI bus; the old stub accepted any bus number. Details: `docs/BOARD_N16R8.md` section 6.
- **`MotorOutputManager::rpm_array` only filled 4 motors**: on a hexa, motors 5 and 6 never got an RPM notch. Now every motor is filled; the RPM filter works per motor (a silent ESC only disables its own notch).
- **The attitude estimator trusted the accelerometer too much** (gain 2 /s): in Angle, holding half stick for 2 seconds tilted the craft 27° for a 12° command. Exposed when the virtual board's accelerometer was made physically correct. Fix and explanation: end of `docs/CORE.md`.
- **Hover throttle guessed at 40 %** in failsafe without altitude: a high-thrust quad would shoot upward. Hover throttle is now measured (`docs/TUNING.md` section 6).
- The old `main.cpp`/`main_rx.cpp` did not compile (included files that do not exist) → replaced by a real `main.cpp`; deleted `main_rx.cpp`, `hal/hal.h`, `hal/sensor/*`, the old `hal/rx/{crsf,ibus,sbus}.h`, `acc_gyro/mpu6xxx.h` (the old versions are still in the zip you sent earlier).
