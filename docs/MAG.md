# Magnetometer (HMC5883L, QMC5883L/P, AK8963 in the MPU9250)

| File | Role |
|---|---|
| `hal/mag/hmc5883l_mag.h` | HMC5883L @0x1E (id "H43", reads in X,Z,Y order big-endian, 1090 LSB/G) |
| `hal/mag/rm3100_mag.h` | RM3100 @0x20-0x23 or SPI on the IMU bus (0.28, 150 Hz, 7500 LSB/G) - see `docs/SENSORS2.md` |
| `hal/mag/qmc5883_mag.h` | QMC5883L @0x0D (200 Hz, 3000 LSB/G, DOR unlock) and QMC5883P @0x2C (3750 LSB/G) — parameters per Betaflight |
| `hal/mag/ak8963_mpu9250_mag.h` | AK8963 inside the MPU9250 via the MPU's I2C master (runs when the IMU is on SPI), ASA sensitivity adjustment, axes converted to the MPU frame; **init after the IMU** |
| `hal/mag/mag_calibration.h` | Ellipsoid calibration: hard iron + 3×3 soft iron matrix (Betaflight only has offsets); 32-cell sphere coverage; rejects data with insufficient coverage, too much distortion (axis ratio > 2), noise, implausible field |
| `hal/mag/mag_processor.h` | calibration → 8-orientation alignment + trim (same maths as the IMU) → ±30% field strength monitoring |
| `hal/mag/mag_base.h` | `MagSensor`: auto-detect, on/off switch, calibration API for the app, `readyForFusion()` |

## Usage
```cpp
mag.attachI2C(sensorBus); mag.attachImuBus(imu.regBus(), imu.detected());
if (!mag.MagInit()) Serial.println(mag.failReason());
mag.setAlignment(CW90_DEG); mag.setCalibration(saved);
// loop: if (mag.update(micros(), millis())) core.on_mag({b[0],b[1],b[2]}, mag.readyForFusion(millis()), millis());
// calibration: mag.startCalibration(); ...rotate the craft in every direction, read mag.calibrationProgress()... mag.finishCalibration(); save mag.calibration()
```
`mag.rawGauss()` feeds the configurator's 3D view. Calibration is stored in the chip frame (before alignment), so changing alignment does not lose the calibration.

## In the core
The compass only corrects **yaw** (rotation about the earth's Down axis), never tilts the craft; it is only used when `core.cfg.use_mag`, calibrated, the sample is fresh (< 300 ms) and the field strength is within ±30% of the calibrated value (motor current distorts the field → dropped automatically). `declination_deg` (east positive): yaw 0 = true north. While disarmed and stationary, yaw is set to match the compass immediately (seed). Compass enabled but not calibrated → cannot arm (`AB_MAG_NOT_CALIBRATED`).

## Auto calibration (0.18.0, like Betaflight 4.5)
- **How to start** (only when disarmed, compass detected):
  - the "Auto calibrate (hand rotation)" button in the Sensors tab (command `MAG_AUTOCAL`);
  - or in the field, with the sticks: **throttle full up, pitch stick pulled back, roll and yaw centred, hold 1 second** (`app/stick_commands.h`). The same place has the gyro calibration command: throttle low, yaw left, pitch pulled back.
  - On start the beeper sounds like for accelerometer calibration.
- **Then**: pick the craft up and rotate it in every direction. Only gyro rotation above 30 °/s counts as "rotating"; leaving the craft still collects no samples.
- **Computation** (`hal/mag/mag_autocal.h`):
  - Least-squares sphere fit: |u|² = 2 b·u + c, with u = M·raw, M the soft iron matrix in use (identity if none yet).
  - Accumulated into 4×4 normal equations, equivalent to RLS without a forgetting factor.
  - New offset = M⁻¹·b. **The matrix M is kept**: the full (ellipsoid) calibration is still valid, only the hard iron is refreshed (e.g. when a battery strap, GPS mast or screw has moved).
- **Stops by itself** when all four conditions hold:
  - samples cover at least 18/32 direction cells (8 azimuths × 4 elevation bands);
  - at least 60 samples;
  - the craft has been rotated for at least 4 s;
  - the estimate is stable (changes below 0.004 G) for 1.5 s.
  - Usually takes 5 – 15 s. The spacing between two kept samples is 5 % of the field strength, so the 300-sample buffer lasts equally long whether the field is strong or weak.
- **Failure**:
  - nobody rotates it within 15 s: `NO_MOTION`;
  - 30 s elapsed without enough: `TOO_FEW_SAMPLES` / `POOR_COVERAGE`;
  - field strength outside 0.15 – 0.9 G: `BAD_FIELD_STRENGTH`;
  - fit error above 8 %: `BAD_FIT`.
  - On failure the old calibration is kept, the beeper plays the error pattern.
- **Done**: the new calibration is used immediately, the beeper sounds 3 tones, and **the firmware writes the `calibration` section to flash by itself** (like in-flight level trim): in the field there is no app to press Save.
- While it runs, arming is blocked (blocker CALIBRATING) and the LED shows the calibrating colour.
- Stream `calibration`: kind `MAG_AUTO`, state, progress, and a new field `result` (the failure reason, also used for manual calibration).
- Tests:
  - `test_mag` auto calibration group: hand rotation, with / without a soft iron matrix, no rotation, flat-only rotation, 4× stronger field, compass going silent midway, cancel.
  - `test_app` stick commands group.
  - `test_fw` group `magauto`: stick command, rotation, automatic flash save, correct heading afterwards, command from the app, no rotation, cancel, no compass.
  - `test_e2e` `test_automatic_compass_calibration` and the browser test `test_compass_calibration_flow` (new `tumble` command of wb_sim).
  - Mutants: `python3 tools/mutate.py magauto`.
- **Not tried on a real craft yet**: the 30 °/s and 18/32 cell thresholds may need tuning to how people rotate the craft by hand.

## Verified (PC, 90 tests + 17 deliberate bugs all caught)
Drivers vs register-level simulation (including a simulation of the MPU9250's I2C master); calibration recovers offsets and soft iron with cross terms, rejects half sphere / 4:1 distortion / 2.5 G field / 12% noise; full calibration flow through `MagSensor`; yaw fusion (declination, tilt, 175° error, 90°/s rotation) — `test/test_mag.cpp` and `test/test_flight.cpp`.
## Not verified
Not run on a real chip yet. Most error-prone points: the axes of each GY-271 module relative to the mounting frame (use `setAlignment` and observe: rotating the craft about the vertical axis, heading must increase when turning right); AK8963 via the I2C master (SLV0/EXT_SENS_DATA registers per my reading of the datasheet, Betaflight in the zip only uses bypass).
