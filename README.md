<img width="2358" height="600" alt="windburst-logo-dark" src="https://github.com/user-attachments/assets/1e601fd5-5c6c-48c4-8e2b-622c0c928d50" />


[![CI](https://github.com/Ohminecraft/ProjectWindburst_Re/actions/workflows/ci.yml/badge.svg)](https://github.com/Ohminecraft/ProjectWindburst_Re/actions/workflows/ci.yml)
[![Release](https://img.shields.io/github/v/release/Ohminecraft/ProjectWindburst_Re?include_prereleases)](https://github.com/Ohminecraft/ProjectWindburst_Re/releases)
[![Stars](https://img.shields.io/github/stars/Ohminecraft/ProjectWindburst_Re?style=flat)](https://github.com/Ohminecraft/ProjectWindburst_Re/stargazers)
[![License: GPL-3.0](https://img.shields.io/badge/license-GPL--3.0-blue)](LICENSE)
[![PlatformIO](https://img.shields.io/badge/PlatformIO-compatible-orange?logo=platformio)](https://platformio.org)
[![Arduino framework](https://img.shields.io/badge/Arduino-framework-00979D?logo=arduino)](https://github.com/espressif/arduino-esp32)
[![ESP32](https://img.shields.io/badge/ESP32-S3%20%7C%20classic%20%7C%20C5-E7352C?logo=espressif)](https://www.espressif.com)

**A modern flight controller firmware for the ESP32.** Windburst brings what the established open-source flight stacks do best -
[Betaflight](https://github.com/betaflight/betaflight)'s rate loop and filters, [ArduPilot](https://github.com/ArduPilot/ardupilot)'s
navigation modes, [PX4](https://github.com/PX4/PX4-Autopilot)'s failsafe design - to the ESP32-S3, ported from their real sources
and tested against them, with a configurator that runs in your browser.

[Tiếng Việt](README.vi.md) · [Documents](docs/ARCHITECTURE.md) · [Web flasher](https://ohminecraft.github.io/ProjectWindburst_Re/) · [Releases](https://github.com/Ohminecraft/ProjectWindburst_Re/releases)

> **Status: experimental.** Everything below is tested on a PC (unit tests, closed-loop flight simulation, firmware-in-the-loop
> against register-level sensor simulators, mutation testing) and compiles for every target. Hardware runs so far: ESP32-S3,
> props off. **It has not flown yet.** Read the "Not verified" part of each document before you put propellers on.

## What it does

| | |
|---|---|
| **Flight core** | Betaflight's rate PID (feedforward, I-term relax, anti-gravity, TPA, throttle boost, simplified tuning sliders), gyro filters (dynamic lowpass, RPM filter, dynamic notch), RC smoothing; Angle (Betaflight), Stabilize (ArduPilot), Acro trainer, Airmode; runaway take-off prevention, motor stall detection |
| **Flying styles** | **FPV** - every mode on a switch, Acro included. **Flycam** - like a commercial camera drone: always self-leveling, height and position held, throttle stick centred = hover |
| **Navigation** | Altitude Hold (baro / lidar / GPS, ArduPilot-style controller, learned hover throttle), Position Hold, **Loiter** (ArduCopter), GPS Rescue / RTL / Smart RTL / Land, missions (waypoints, deliveries with servo drops), geofence, optical flow (PMW3901, MicoAir MTF-01) |
| **Safety** | PX4-style failsafe framework (link, switch, battery warning / critical: hold, RTL, land, drop), arming checks with a reason for every refusal, **crash flip** (Betaflight turtle mode) |
| **Sensors** | 14 IMUs (ICM-426xx/456xx, BMI270, MPU6xxx ...; dual IMU), BMP280 / BMP388 / MS5611 barometers, QMC5883 / HMC5883 / RM3100 / AK8963 compasses with automatic calibration, VL53L0X / L1X lidars, u-blox / NMEA GPS, DroneCAN sensors |
| **Motors** | DShot 150 - 1200 (bidirectional, extended telemetry), ProShot, PWM / Oneshot / Multishot, DroneCAN ESCs; 14 frame layouts; ESC passthrough for BLHeliSuite / ESC Configurator |
| **Radio** | CRSF / ELRS (with telemetry, and ELRS MAVLink mode), SBUS, IBUS, Spektrum, PPM, PWM; a DIY ESP32-C5 transmitter with nRF24 |
| **Ground station** | MAVLink 2 telemetry and control (missions, modes, arm / disarm, PID / rate / filter tuning as parameters) - works with Mission Planner and QGroundControl; MSP DisplayPort OSD; CRSF telemetry |
| **Logging** | Blackbox to internal flash or SD card, live log streaming, PID step-response analysis in the app |
| **Configurator** | A browser app (English / Vietnamese): every setting, live sensor views, motor test and wizard, calibrations, mission planner on a map, log viewer. It talks to the firmware it is built with **and to every older release** |

## Get started

**Flash** - from the browser: the [web flasher](https://ohminecraft.github.io/ProjectWindburst_Re/) (ESP32 boards), or with PlatformIO:

```
pio run -e rx_esp32_s3 -t upload        # ESP32-S3 N16R8 flight controller (rx_esp32: classic ESP32, tx_c5: the DIY transmitter)
```

**Configure** - the configurator (Python 3, pyserial):

```
cd app && pip install -r requirements.txt
python -m wbclient gui                   # opens the page; pick the port and the baud rate there
python -m wbclient --port COM5 info      # or the command line: the same operations
```

The link is binary: close any serial monitor first. The pin map is set in the "Pins and ports" tab (there is a suggested set for the DevKitC N16R8).

**No board?** The real firmware runs on a simulated board:

```
make sim && build/wb_sim                 # then choose "Simulator wb_sim" in the configurator's port list
make test                                # every firmware test on the PC (g++), firmware-in-the-loop included
```

## Targets

| Board | Status |
|---|---|
| ESP32-S3 (N16R8 / N8) | **main target** |
| ESP32 classic (4 MB) | builds; fewer features fit |
| ESP32-C5 | the DIY transmitter |
| STM32 F405 / F412 / G4 / H5 / C5 | ⚠️ **experimental** |

> ⚠️ **STM32 is experimental.** The STM32 builds exist so the code stays portable; they have seen far less testing than the ESP32-S3
> build. For an STM32 flight controller you intend to fly, use a firmware with years of flight history:
> [Betaflight](https://betaflight.com), [ArduPilot](https://ardupilot.org), [INAV](https://github.com/iNavFlight/inav) or [PX4](https://px4.io).

## Documents

[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) is the table of contents. The documents are in English; the Vietnamese originals are in [docs/vi](docs/vi).
Each feature's document ends with what has been verified and what has not.

## Contributing

Interested? Help is welcome - flight tests and logs, a board you can test on, reviews against the reference firmware, drivers,
documents and translations. Open an issue or a pull request. The ground rules:

- every change comes with its tests (`make test`; the app's tests: `make apptest`), and a feature's document says what is verified;
- algorithms follow a named reference (Betaflight, ArduPilot, PX4, INAV) - say which file you read;
- the protocol has one source: `protocol/wbp_schema.json` (`python tools/gen_proto.py` writes the firmware's and the app's side).

## Credits

Windburst stands on the work of these projects. Where code was ported, the file says from where.

- [Betaflight](https://github.com/betaflight/betaflight) - rate loop, PID, filters, mixer, DShot, RC smoothing, MSP, crash flip, CRSF, OSD (GPL-3.0)
- [ArduPilot](https://github.com/ArduPilot/ardupilot) - Altitude Hold, Loiter, Position Hold, land detector, baro altitude, EKF3 optical flow, missions (GPL-3.0)
- [PX4 Autopilot](https://github.com/PX4/PX4-Autopilot) - failsafe framework, EKF2 optical flow, sensor drivers (BSD-3-Clause)
- [INAV](https://github.com/iNavFlight/inav) - optical flow calibration, MSP sensors, navigation (GPL-3.0)
- [ExpressLRS](https://github.com/ExpressLRS/ExpressLRS) - CRSF and the MAVLink link mode (GPL-3.0)
- [MAVLink](https://github.com/mavlink/mavlink) and [pymavlink](https://github.com/ArduPilot/pymavlink) - the protocol and its test vectors
- [DroneCAN](https://dronecan.github.io) - CAN sensors and ESCs
- [esp-fc](https://github.com/rtlopez/esp-fc) - ESP32 flight controller ideas and pin layouts
- [Bitcraze Crazyflie firmware](https://github.com/bitcraze/crazyflie-firmware) - PMW3901 handling
- [Bluejay](https://github.com/bird-sanctuary/bluejay), [AM32](https://github.com/am32-firmware/AM32), BLHeli - the ESC side of DShot
- [ESP Web Tools](https://github.com/esphome/esp-web-tools) - the web flasher

## License

[GPL-3.0](LICENSE). Windburst contains code ported from GPL-3.0 projects (Betaflight, ArduPilot, INAV), so the whole firmware is GPL-3.0.
