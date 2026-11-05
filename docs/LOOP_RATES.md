# Separate gyro rate and PID rate (0.12.0)

Before 0.12 every gyro sample ran a whole cycle: gyro filtering, attitude computation, modes, PID, mixer, sending motor commands, blackbox recording. Since 0.12, like Betaflight's `pid_process_denom`:

| Setting | Meaning |
|---|---|
| `general.loop_hz` | **gyro rate**: samples read per second; the hardware timer wakes the loop at this rate. The gyro filters (lowpass, static notch, dynamic notch, RPM filter) run here and see **every** sample, so vibration above the PID rate is not folded (aliased) into the control loop |
| `general.pid_denom` (1 – 8, default 1) | the control part runs every n gyro samples: **PID rate = loop_hz / pid_denom** |

What runs at the PID rate: attitude (AHRS, integrating over the **average** gyro of the whole n-sample window, with the exact elapsed time), RC / arming / failsafe, Altitude Hold, Angle / Stabilize / Acro modes, PID (including D-term filtering, anti-gravity, TPA, feedforward), mixer, runaway, motor commands (DShot / PWM), ESC telemetry requests, blackbox records. The estimator (`est_hz`) keeps its own rate.

A "filter-only" gyro sample (not a control cycle) returns the result of the previous control cycle, replacing only the filtered gyro; the motors keep their previous command.

## Limits
- The PID rate is never below 500 Hz: if `loop_hz / pid_denom < 500`, the firmware lowers `pid_denom` until it is enough (e.g. gyro 2000 Hz, divide by 8 → runs divide by 4 = 500 Hz). The boot report shows both: `imu: SPI ..., gyro 8000 Hz, control 4000 Hz (1/2)`.
- Default `pid_denom = 1`: behaves like previous versions.
- Old settings: the `general` section of 22 bytes (0.9.5 – 0.11) and 23 bytes (0.9.0 – 0.9.4) both load; the old version's 23rd byte lands in the unused `reserved_g` slot, so it does not become a garbage `pid_denom`.

## App and log
- The `status` stream gains `pid_hz` (the real number of control cycles in the last second) and `pid_target`. The Overview tab has an "Actual PID loop" line; the Sensors tab has a "PID rate divider" field and a "Running: gyro … Hz, PID … Hz" line; the "Loop" chip in the top bar adds "· PID …" when PID is slower than the gyro.
- "Per-stage timing": IMU and "computation" are averaged over **every** gyro sample (filter-only cycles are very short); "motor send" only counts control cycles.
- Blackbox: at most one record per control cycle. The `loop_hz` header = PID rate (the log's time base), new line `gyro_hz`. (Also fixes an old bug: the header always said 4000 and the log rate divider was computed from 4000 even when set to 8000.)
- MSP (BLHeliSuite / Betaflight tools): cycle time = PID period, as in Betaflight.

## Loop Hz drops when armed (0.11 feedback: ~5700 Hz → 4400 Stabilize / 4700 Angle / 4900 Acro)
When disarmed, the cycle stops after the RC part; when armed it also runs Altitude Hold / hover estimation, the mode controller, PID, mixer, runaway, and motor commands. Two changes in 0.12:
1. `pid_denom`: e.g. gyro 8000 / divide by 2 means the expensive part only runs 4000 times per second.
2. Fast trigonometry (`types.h`: `sin_approx`, `cos_approx`, `atan2_approx`, `asin_approx`, `acos_approx`, the polynomials from Betaflight's `maths.c`) replaces the C library's `sinf / cosf / atan2f / asinf / acosf` in places that run every cycle: Euler attitude (every cycle, even disarmed), tilt angle, Angle (yaw coupling) and Stabilize (4 sin/cos calls). The library functions live in flash (WB_FAST cannot move them) and cost a few µs each; the polynomials sit in RAM next to the caller. Error: sin/cos < 2e-6, atan2/asin < 1e-6 rad, acos < 7e-5 rad (0.004°).

**Not measured on the board yet.** Numbers needed: Overview tab, "Per-stage timing" and "Actual PID loop" when disarmed / armed in Acro / Angle / Stabilize, with `pid_denom` 1 and 2.

## First measurement on the board (0.17, Acro, set to 6000 Hz)
| | |
|---|---|
| one cycle | average 131 µs, longest 449 µs |
| cycles actually run | 5566 Hz out of 6000 (437 cycles lost per second) |
| per stage | IMU read 37 · computation 66 · motor send 26 · rest 10 µs |
| Angle / Stabilize | about 100 / 200 Hz less than Acro |

Two causes:
1. **Out of time.** A 6000 Hz cycle is 167 µs long, and the flight task must leave ¼ of each cycle's time for the sensor thread (`app::TickBudget`): 131 µs × 1.25 = 164 µs, right at the limit, so every long cycle (the estimator, longest 449 µs) loses a tick.
2. **6000 does not divide the chip's rate.** ICM / MPU chips asked for 6000 Hz run at 8000 Hz; a 6000 Hz loop reads 3 out of every 4 samples at uneven intervals (a quarter of the samples are twice as old as the others), while the filters and PID treat them as evenly spaced.

## Loop rate as an integer divisor of the chip's rate (0.18, after the measurement)
`app::loop_rate_for` (`src/app/sync.h`): the loop takes **every nth sample** from the chip, at most 10 % faster than the setting. Examples:

| setting | chip | loop |
|---|---|---|
| 6000 | 8000 (ICM, MPU) | **4000** |
| 3000 | 3200 (BMI270) | 3200 |
| 4000 / 8000 | 4000 / 8000 | unchanged |

The boot report states the reason: `imu: SPI ICM42688P (20 MHz direct), gyro 4000 Hz (setting 6000 Hz: the chip delivers 8000 Hz), control 4000 Hz (1/1)`. To go faster than 4000 Hz: set 8000 with `pid_denom` 2.

## Faster IMU reads (0.18, after the measurement)
37 µs for 13 – 15 bytes is too much: at 8 MHz the wire only takes about 15 µs; the rest is Arduino's SPI library (two mutexes, resetting the clock each time, `digitalWrite` for the CS pin - all of it in flash).
- **Per-chip clock** (`AccGyroDriver::max_spi_read_hz`): ICM-426xx / 456xx, MPU6000 / 6500 / 9250: 20 MHz; ICM-20601 / 20602, BMI160 / 270: 10 MHz. Previously every chip was at 8 MHz. Register writes still use the slow clock.
- **Direct SPI register reads** (`SPIBus::direct_read`, ESP32-S3 only): writes `data_buf`, `cmd.update`, `cmd.usr` as the library does, CS via `GPIO.out_w1ts/w1tc`; lives in IRAM. Only the IMU uses SPI_DEVICE_0 and only the flight task reads it, so no lock is needed. When the clock in the register has been changed by someone else (slow writes, a second IMU), it resets it through the library. If the controller does not finish within ~1 ms: one failed read (the loop skips that sample), no hang.
- **Automatic fallback on implausible readings** (`IMUAccGyro::finish_init`): tries the chip's clock + direct reads; if the first 20 samples do not give ~1 g, it tries that clock through the library; then 8 MHz through the library. The boot report shows the outcome: `(20 MHz direct)`, `(20 MHz)` or `(8 MHz)`.
- To disable for comparison: `-DWB_SPI_DIRECT=0` (back to the library), `-DWB_IMU_SPI_MAX_HZ=8000000` (back to 8 MHz, e.g. with long wires).

**Not measured on the board yet.** Need to check "Per-stage timing" (IMU read) again and the `imu:` line of the boot report.

## Testing
- `test_flight` group `pid_denom`: which cycles are control cycles, motors holding their command, filtered gyro updated on every sample, the estimator staying at 500 Hz, Angle / 5-inch Acro / Stabilize / attitude through a flip with divide by 1, 2, 4.
- `test_fw` group `loop`: boot report, `status.pid_hz` / `pid_target`, 2000 motor commands per second at 4000 / 2, lowering `pid_denom` when below 500 Hz, the log's time base.
- `test_proto`: layout and loading of old `general` sections. `test_core` group fast trigonometry.
- Mutants: `python3 tools/mutate.py denom`, `python3 tools/mutate.py fastmath`.
- `test_app` group `loop rate`: the `loop_rate_for` table. `test_fw` group `loop`: 6000 Hz on an 8000 Hz chip runs at 4000 Hz and the report gives the reason; the IMU's SPI clock in the report (there is no direct read path on the PC).

## IMU chip details (0.44.0)
- **Detection order** (`classify_spi`, as Betaflight's `gyroSpiDetectFnTable`): WHO_AM_I at 0x75 first (MPU family, ICM-2060x, ICM-426xx), then 0x72 (ICM-456xx), the IDs at 0x00 (BMI270, BMI160, ICM-209xx) last. On the MPU family reg 0x00 holds factory trim bytes and on the ICM-456xx accelerometer data, so any value can show up there.
- **ICM-426xx / ICM-456xx: −32768 is "no valid data"** (the chip's own marker, e.g. right after start-up). A sample with that value on any axis is dropped; the loop treats it like a missed read.
- **ICM-456xx gyro low-pass inside the chip** is chosen by bandwidth, not as a fixed divider of the rate: about 200 Hz at every gyro rate (6400 Hz: ODR/32, 3200 Hz: ODR/16, 1600 Hz: ODR/8, 800 Hz: ODR/4). Betaflight always runs this chip at 6400 Hz with ODR/32 = 200 Hz; before 0.44.0 a lower rate kept ODR/32 and so had 100 Hz (3200) down to 25 Hz (800) and much more delay.
- Tests: `test_accgyro` (`[detect]`, the −32768 group, the ICM-456xx low-pass table). Mutants: `python3 tools/mutate.py audit043`.
