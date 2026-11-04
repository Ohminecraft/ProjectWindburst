# Sonar / Lidar (VL53L1X, VL53L0X)

The prompt says `Lidar: VL53L1X, VL53LX`; I take "VL53LX" to mean **VL53L0X** (ST's real chip). Betaflight has no driver for these two chips, so there is nothing to copy from BF.

| File | Role | Verification |
|---|---|---|
| `hal/range/range_driver.h` | Non-blocking driver interface (distance along the beam + quality) | — |
| `hal/range/range_pololu.h` | VL53L1X / VL53L0X driver: wraps the **Pololu library** (MIT) | **compiles + links for ESP32-S3** (symbols present in the ELF); not run on the board yet |
| `hal/range/range_height.h` | Distance → height: tilt compensation, filtering, self-learning sensor mounting height, sigma | 33 PC tests, 12 deliberate bugs all caught |
| `hal/range/range_base.h` | `RangeFinder` (Sonar tab): chip detection, on/off switch, text error reporting | PC tests with a fake driver |
| `hal/flow/flow_sensor.h` (`FlowRangeDriver`) | Lidar inside an MSP optical flow sensor (MicoAir MTF-01 / MTF-02): MSP v2 frame 0x1F01, `sensors.range_model = MSP` or AUTO when there is no VL53L*; no measurement for 0.5 s = unhealthy (like ArduPilot's `AP_RangeFinder_MSP`) | `test_flowsensor`, `test_fw` (0.7.0), see `docs/NAV.md`; not run on the board yet |

## Why wrap a library instead of writing registers myself
VL53L1X/L0X need ST's proprietary init sequence (reading SPAD calibration from NVM, reference calibration, loading hundreds of tuning registers, 16-bit register addresses on the L1X). Rewriting it from memory without a board to verify could produce distances that "look plausible but are wrong" — more dangerous for the altitude hold loop than reporting "no sensor". The old Windburst code chose the same approach (the prompt said to reuse it).
**`lib_deps` in `platformio.ini` is mandatory** (already added for `rx_s3`). I tried it: without `lib_deps` PlatformIO does not add the library, and the driver silently compiles as "library missing" (`failReason()` will say so).

## Usage
```cpp
RangeFinder sonar;
sonar.attachI2C(sensorBus);          // shares the I2C bus with baro/compass (or setI2cPins)
if (!sonar.RangeInit()) Serial.println(sonar.failReason());
// loop:
if (sonar.update(micros(), millis(), fc.att.up_cos(), armed)) {
    kf.cfg.r_range = sonar.sigmaM();
    kf.update_range(sonar.heightM());      // tilt-compensated, sensor mounting height subtracted
}
arm.range_ok = sonar.valid(millis());
```
- `setModel(AUTO|VL53L1X|VL53L0X)`, `setL1xMode(0 short | 1 medium | 2 long)` (default long = 4 m; in strong sunlight try medium/short), `setEnabled(false)` for the switch in the app.
- Altitude Hold **does not depend** on the sonar: out of range / tilted beyond 30° / signal lost, the Kalman ignores it and the baro carries on.

## Processing (range_height.h)
- Height = distance × cos(tilt); tilt > 30° drops the sample. Only up to 90% of the maximum range is used (VL53L1X long 4 m → 3.6 m; VL53L0X 1.2 m → 1.08 m).
- Impossible jumps (> 8 m/s + 0.3 m) are rejected; 3 consistent consecutive samples accept the new level (roofs, steps). A hand under the drone only lasts 1–2 samples so it is rejected.
- The sensor mounting height (a few cm when parked) is learned while **disarmed**, level, reading < 0.5 m; locked when armed. Holding the drone in the air at power-up does not learn garbage. Set `cfg.ground_clearance_m ≥ 0` to fix it.
- `sigmaM()` = 2 cm + 1.5% × height → `kf.cfg.r_range`.
- Closed-loop test with the real Kalman + drifting baro: altitude error < 12 cm, velocity does not gradually drift (the `-95 cm/s` bug the prompt mentioned); when the lidar drops out from flying too high there is no step.

## On the board (in order)
1. `RangeInit()` true? Print `name()`, `maxRangeM()`, `rateHz()`; if wrong read `failReason()` (wiring, 2.8 V supply, XSHUT pin pulled high).
2. Leave the drone still: `rawM()` ≈ sensor height; `heightM()` goes to ~0 after ~1 s (learned).
3. Lift to 0.5 / 1 m: `heightM()` follows correctly; tilt 40°: `update()` returns false.
4. Outdoors in sun: if `quality()` is low / range is lost, try `setL1xMode(1)` or `(0)`.
5. If a real ultrasonic sensor (HC-SR04/US-100) is added later: only a new `RangeDriver` is needed, the rest is reused.
