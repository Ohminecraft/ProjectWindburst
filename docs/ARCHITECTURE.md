# Architecture — documentation index

| Document | Contents |
|---|---|
| **`docs/HARDWARE_FEEDBACK_1.md`** | first run on the real board (version 0.5): each symptom → cause → what was fixed → what still needs checking on the board; "IMU chip orientation" conversion table; `IRAM_ATTR` |
| **`docs/HARDWARE_FEEDBACK_2.md`** | first log from the real board (version 0.6.1): why motors differ in a props-off test, PID columns in the log being clamped, 83 % loop → flight task time budget, per-stage timing in the log |
| **`docs/HARDWARE_FEEDBACK_3.md`** | bidirectional DShot with 0.8.2 on the real board: ESC accepts no frames, 0 replies; hypothesis that the line is pulled low when the pin is released → 0.8.4 keeps the line high after the reply window, measures the line level, reads back frames while running |
| `docs/FIRMWARE.md` | `main.cpp`, `FlightApp`, the three threads (flight task / sensors / link), virtual board + `wb_sim`, verification table |
| `docs/PROTOCOL.md` | app ↔ firmware protocol (WBP): frames, settings, streams, commands, safety rules |
| `docs/APP.md` | graphical configurator (13 tabs), Python client library, command line |
| `docs/CORE.md` | flight core `src/core` |
| **`docs/NAV.md`** | optical flow (PMW3901, MSP sensors), position estimation, Position Hold, GPS Rescue: required hardware, settings, how it works, order of tests on the board |
| `docs/TUNING.md` | filters, PID, sliders, Angle / Stabilize, Altitude Hold hover throttle, in-flight level trim, tuning order, what has not been measured on the board |
| `docs/RX.md` | CRSF / SBUS / IBUS receivers |
| `docs/MAVLINK.md` | MAVLink telemetry: what is sent, how to choose the UART; from 0.40 commands, modes, arming and missions from a ground station (`telemetry/mavlink_gcs.h`) |
| `docs/ELRS_MAVLINK.md` | MAVLink over ELRS: one ELRS link in MAVLink mode for both control and telemetry (0.39), or a separate ELRS pair as a telemetry radio; wiring, settings, throttling, bandwidth |
| `docs/BOARD_N16R8.md` | ESP32-S3 N16R8 board: build, unusable pins, suggested pin set, ADC, SD card wiring, what needs trying on the board |
| `docs/POWER.md` | battery measurement (ADC / ESC), thresholds, alerts via buzzer and via motors (DShot beacon) |
| `docs/BLACKBOX.md` | flight logs: `.wblog` format, writing to flash / SD card / sending to the app, Blackbox tab, what must be measured on the board |
| `docs/IMU_BRINGUP.md`, `MOTOR_OUTPUT.md`, `BARO.md`, `SONAR.md`, `GPS.md`, `MAG.md` | individual drivers |

```
            ┌──────────────── core 1 ─────────────────────┐        ┌──── core 0: task wb_link ────┐
 timer ───► │ task wb_flight (priority 20, code in RAM)   │        │ wbp::Server ◄──► USB / UART0 │ ◄──► app
 IMU ─────► │   FlightApp::flight_cycle → FlightCore → ESC│        │   (via 8 kB send queue)      │
            │        ▲ shared lock ▼                      │        │   app::LinkHost              │
 RX, GPS,   │ loop() (priority 1): FlightApp::slow_update │        │                              │
 baro, mag ►│   publish(Telemetry) ───────────────────────────►   │   snapshot()                 │
            │   service(request)   ◄───────────────────────────   │   call(request)              │
            └─────────────────────────────────────────────┘ app::Exchange └────────────────────────┘
 settings (NVS) ──► wbp::Settings ──► settings_apply.h ──► core / motor / GPS / sensor configuration
```

Core 0 also runs the `wb_sd` task (lowest priority): it receives log bytes from the link task through a ring buffer and writes them to the SD card; the flight loop never waits for the card.
