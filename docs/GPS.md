# GPS

## Which GPS protocols / features Betaflight has (read from `io/gps.c`, `io/gps.h`, `pg/gps.c`)
| Betaflight | Windburst |
|---|---|
| Provider **UBLOX** (UBX binary) | ✅ M5/M6, M7/M8, M9/M10 — the right "dialect" for each generation |
| Provider **NMEA** (GGA, RMC, GSA, GSV) | ✅ + `nmea_custom_commands` sent line by line to the module |
| Provider **MSP** (GPS fed in over MSP) and **VIRTUAL** | ✅ `inject()` (no real MSP source yet — that belongs to the telemetry/app part) |
| Provider **DRONECAN** | ❌ not done yet (needs a CAN bus) |
| `gps_auto_baud` (baud scan, module detection) | ✅ MON-VER 3 times/330 ms then change baud, order 9600 → … → 230400 as in BF |
| `gps_auto_config` | ✅ step-by-step configuration sequence, waits for ACK, retries 2 times then skips that step |
| `gps_baud` (default **115200** since 0.8.0, previously 57600; the app offers 9600 / 19200 / 38400 / 57600 / 115200 / 230400, other values → nearest standard rate), `gps_update_rate_hz` (default 10) | ✅ |
| `gps_ublox_acquire_model` / `flight_model` | ✅ uses the "acquire" model while there is no fix, switches to "flight" (AIRBORNE<4g) on the first 3D fix |
| `gps_ublox_use_galileo`, `sbas_mode` (AUTO/EGNOS/WAAS/MSAS/GAGAN/NONE), `sbas_integrity`, `gps_ublox_utc_standard` | ✅ (CFG-SBAS + CFG-GNSS for M7/M8, CFG-VALSET for M9/M10) |
| NAV-SAT off when **armed**, back on when disarmed (eases UART congestion) | ✅ |
| Home point set at **arm** (needs 3D fix + enough satellites), `gps_set_home_point_once` | ✅ |
| Distance / direction to home, distance flown (`gps_use_3d_speed`, ignores < 15 cm/s) | ✅ |
| "GPS ready" beep when the conditions for setting home are met | ✅ `take_ready_edge()` flag for the buzzer |
| Connection loss detection (2.5 s), automatic re-detection | ✅ |
| **GPS Rescue** (flies home automatically on failsafe or when a switch is flipped) | ✅ since 0.7.0, Betaflight's phases drive our own controller; needs a compass — see `docs/NAV.md` (**not flown for real yet**) |
| AssistNow Autonomous (`gps_ublox_enable_ana`) | ❌ off by default in BF, skipped |

## Structure
`hal/gps/gps_types.h` (data, units as in BF: 1e-7°, cm, cm/s, ddeg) · `ubx_proto.h` (UBX frame, parser, decoding of NAV-PVT/POSLLH/VELNED/SOL/STATUS/DOP/SAT/SVINFO, MON-VER) · `nmea_proto.h` · `gps_engine.h` (state machine) · `gps_nav.h` (geometry + home) · `gps_base.h` (`GpsSensor`, Arduino UART port).

## Usage
```cpp
GpsSensor gnss;
gnss.setPins(RX, TX);                 // or getGpsPins() in hardware_config.h
gnss.config().baud = 115200;          // + provider / update_rate_hz / sbas / use_galileo / models ...
if (!gnss.begin(Serial1)) Serial.println(gnss.failReason());
// fast loop (>= 100 Hz, only drains the UART):
gnss.update(millis(), armed);
if (gnss.newSolution()) { auto& s = gnss.solution(); /* lat, lon, alt_cm, vel_n/e/d_cms, num_sat, h_acc_mm ... */ }
arm.gps_fix = gnss.usable();          // good link + 3D fix + >= 8 satellites + hdop <= 3.0 + hAcc <= 5 m (change with setLimits)
float east, north; gnss.home().offset_from_home_cm(east, north);   // input for Position Hold / RTH
```

## Verified
- **On PC (187 tests, all 21 deliberate bugs caught):** a *generation-strict* u-blox simulator — M9/M10 NACK legacy commands, M5–M8 NACK CFG-VALSET, M5/M6 have no NAV-PVT/NAV-SAT. The driver must send the right dialect to configure anything (`nacks == 0` for all 5 generations). Checks: baud detection from 9600 → 57600 with a `$PUBX,41` sentence byte-identical to Betaflight's table, connection loss/recovery, a module that never ACKs (no hang), the SBAS PRN mask, a 3D fix with `gnssFixOK=0` not counted as a fix, decoding of NED velocity/speed/heading/MSL altitude, NMEA in both hemispheres, home/distance/direction/distance flown.
- **Real build for ESP32-S3** succeeded. This build caught a bug the PC tests did not see: Arduino defines the macro `bit(b)`, so a local function named `bit` was swallowed and the M9/M10 SBAS mask became 0. Renamed; the tests' Arduino stub now has those macros (`bit`, `sq`, `radians`, `word`, ...) to catch a recurrence.

## Deliberate differences from Betaflight
- **Longitude near ±180°:** BF computes `to->lon - from->lon` as `int32_t`; the two points 179.9999° and −179.9999° overflow and give a difference of ~69.5° instead of 0.0002° (we checked with the real wrap arithmetic). Here 64-bit is used.
- M5/M6 do not get CFG-GNSS (the chip does not support it); BF still sends it and receives a NACK.
- On connection loss, fix/satellite count are cleared right at the timeout (not waiting for the next pass).

## Not yet run on a real chip — check in this order
1. Does `gnss.begin()` return true? Connect the module, print `stateName()`, `module()` (M8/M9/M10...), `baud()`; the state must go detect baud → change baud → configure → receiving within a few seconds.
2. `configNacks()` must be 0; if not, send me that number + `module()`.
3. Outdoors: `solution().num_sat`, `rateHz()` ≈ `update_rate_hz`, `fix` comes up after a few tens of seconds.
4. If the module was previously switched to an unusual baud and saved to flash: enable `auto_baud` (on by default) and wait — one full scan takes about 10–20 s.
5. M9/M10 modules (new BN-880, cheap M10) use CFG-VALSET: if `module()` prints UNKNOWN, the module has an unusual `hwVersion` string — we treat it as M8 and report `version_assumed()`.

## Automatic baud switching (0.8.0)
A u-blox module is found at the rate it is currently running (`gps.auto_baud` on: tries each rate in turn, asking MON-VER 3 times at each) and then switched to `gps.baud` with `$PUBX,41` (as in Betaflight); the default is now **115200** (enough room for NAV-PVT + NAV-SAT at 10 Hz). Settings records from older versions keep the old value (57600): to get 115200, select it in the Sensors tab, GPS section, "GPS baud", save and reboot. NMEA modules (protocol `NMEA`) are not switched: they must already be set to the right rate. On the next boot the module (unless it has a backup battery keeping its configuration) returns to its original rate and is switched again. Verified: `test_gps` (100000 → 115200, 20000 → 19200), `test_fw` group `gpsbaud` (virtual modules at 9600 / 230400 / 38400 are switched to 115200 / 57600 / 115200 and get a fix; with auto-detect off, a module at 9600 is not found). **Not yet tested with a real module.**

## GPS altitude in Altitude Hold (0.8.3)
**Before 0.8.3:** GPS altitude was only blended into the estimator when there was neither a barometer nor a lidar, and even then the altitude was still treated as "invalid" — so in practice Altitude Hold **never** used GPS. GPS vertical velocity (u-blox `velD`) was not used anywhere.

**Should it be used?** GPS altitude drifts 1–3 m within a minute (satellites move, the atmosphere changes); a barometer is much more stable over a few seconds but drifts with the weather and the board temperature (typically 0.5–2 m in 10 minutes). Therefore:
- **GPS vertical velocity: use it always** (ArduPilot also uses it by default). It does not drift, has 0.1–0.3 m/s error, and helps exactly where the barometer is weakest: prop wash, fast climbs / descents. On by default (`nav.gps_vel_z`). The module's 0.15 s delay is compensated with the measured acceleration (like the horizontal estimator). Only u-blox modules provide this number; NMEA modules do not (flag `has_vel_d`), so zeros are not blended in.
- **GPS altitude in place of the barometer: yes**, when there is nothing else (`FALLBACK`, the default — like Betaflight's `altitude_source = DEFAULT`). A craft without a barometer can now use Altitude Hold / auto-landing / GPS Rescue on GPS altitude (accurate to about ±1–3 m, enough for flying high outdoors, not enough for flying close to the ground). If the barometer loses its signal mid-flight: GPS takes over **without a jump** (the GPS − z reference is tracked the whole time the barometer is running), Altitude Hold is not interrupted. The `ALTHOLD_NO_BARO` blocker now only applies when there is no barometer **and** no GPS fix.
- **Blending GPS altitude with the barometer: optional** (`BLEND`, not the default). Helps on long flights (10 minutes or more, changing weather), slightly hurts short ones: a metre of GPS drift will be partly followed. How it works: the mean (GPS − z) of the first 20 measurements after arming is taken as the reference, after which GPS altitude relative to that reference corrects the barometer *offset* with sigma ≥ 3 m (time constant about one minute). GPS altitude deviating from the estimate by more than 4 m (satellites lost / gained) is rejected, and after 10 in a row a new reference is taken — it does not pull `z`. GPS lost for more than 3 s also takes a new reference.

Settings are in the **Position** tab, **"Altitude from GPS"** panel (take effect immediately, no reboot needed); that panel also shows whether GPS altitude / vertical velocity are in use (bits `GPS_ALT`, `GPS_VZ` of the `estimate` stream). GPS altitude only exists once a home point is set (GPS fix at arm).

**Fixed alongside:** with the NMEA protocol, north / east velocity used to always be 0 (only UBX filled it), so the horizontal estimator was told "stationary" while the craft was flying — Position Hold / Rescue with an NMEA module would fail. It is now computed from the speed + course of the RMC sentence.

**Verified on PC:**
- `test_core` [vertical estimator: GPS]: GPS vertical velocity halves the climb-rate error (0.035 vs 0.071 m/s RMS) and on its own holds the climb rate when there is no altitude; barometer drifting 1 m/minute for 5 minutes: 5.0 m off with baro only, 0.2 m with BLEND; GPS 25 m off from the start has no effect (reference); GPS jumping 8 m: new reference, estimate shifts < 0.4 m (without the 4 m gate it shifts 6.5 m); barometer silent: GPS takes over, estimate shifts 0.14 m; GPS only: z tracks GPS altitude, a 30 m GPS jump resets z after 10 measurements.
- `test_flight`: GPS only (1 m noise, 150 ms delay), Altitude Hold at 5 m for 20 s: error < 0.3 m; `OFF`: Altitude Hold has no altitude; Altitude Hold for 3 minutes, barometer drifting 1 m/minute: the craft sinks 2.8 m with `FALLBACK`, only 0.65 m with `BLEND`; barometer off at 5 m: Altitude Hold does not drop out, the craft stays within 0.33 m for 20 s; NMEA: vertical velocity is not used.
- `test_fw` group `gpsalt` (full firmware on a virtual board, noise-free virtual GPS): no barometer + GPS fix → can arm with the Altitude Hold switch, holds altitude, bits `GPS_ALT` + `GPS_VZ`; `OFF` → `ALTHOLD_NO_BARO`; with barometer: `FALLBACK` uses vertical velocity, not altitude; `BLEND` uses both; `gps_vel_z` off → not used.
- `test_gps`: RMC → north / east velocity, NMEA has no vertical velocity, UBX does.

**Not yet tested with a real GPS.** The virtual GPS in the tests only has white noise (test_flight) or no noise (test_fw); a real GPS also drifts slowly and jumps when satellites change, so outdoor numbers will be worse. First tests: (1) with a barometer, leave the defaults, fly Altitude Hold as before — the only difference is that the `GPS_VZ` bit is lit; (2) try `BLEND` on a long flight, compare the altitude at landing; (3) only then try a craft without a barometer, in an open area, above 5 m, hand ready on the switch.
