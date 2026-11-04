# Sensors added in 0.28: ICM-20948 / ICM-20649, MS5611, RM3100, two barometers, sensors on the IMU's SPI bus

Everything has only been checked on PC (drivers run against register-level simulated chips, firmware runs on the virtual board). **Not tried on a real board yet.**
Specifications taken from the datasheets and the ArduPilot (`external_source/ardupilot`) and Betaflight drivers.

## 1. ICM-20948 and ICM-20649 IMUs (`hal/acc_gyro/icm20948_accgyro.h`)
- "v2" register map with 4 banks (REG_BANK_SEL 0x7F). WHO_AM_I bank 0 register 0x00: 0xEA (20948), 0xE1 (20649).
- Startup: reset PWR_MGMT_1 = 0x80, wait 100 ms, then 0x01 (awake, best clock). SPI: USER_CTRL = 0x10 (I2C off).
- Range: 2000 dps / 16 g on both chips. The FS codes differ: the 20948 uses 3, the 20649 uses 2 (the 20649 counts from 500 dps / 4 g). ArduPilot leaves the 20649 at 30 g; here it stays at 16 g like Betaflight.
- Rate: SPI with a loop > 1125 Hz drops the on-chip filter (FCHOICE 0): gyro 9 kHz, accel 4.5 kHz. I2C (or ≤ 1125 Hz): DLPF on, 1125 Hz.
- Read: bank 0, 0x2D, 12 bytes big-endian (accel then gyro). The driver always leaves the chip in bank 0. SPI at most 7 MHz.
- Detection:
  - SPI: probed after every other chip (like Betaflight), because in the MPU family register 0x00 is a factory self-test value that may coincide with 0xEA / 0xE1.
  - Chip left in another bank (warm reboot during init): the second attempt writes 0x7F = 0 and probes again.
  - I2C 0x68 / 0x69: read 0x75 first (MPU); if not an MPU, select bank 0 and read 0x00.
- The AK09916 compass inside the ICM-20948 is not used yet.

## 2. MS5611 barometer (`hal/baro/ms5611_baro.h`)
- The chip is command-driven, with no registers. `AgRegBus` gains `command()` and `command_read()`:
  - I2C: a single command byte (`I2CBus::I2CWriteByte`), or command then read.
  - SPI: the command byte is sent as is, without the 0x80 read flag.
- Sequence:
  - reset 0x1E;
  - PROM 0xA0 + 2i: 8 words. There is no chip id: only a correct CRC-4 (per AN520), and a PROM that is not all 0 / all 1, counts as an MS5611;
  - D1 0x48, D2 0x58 (OSR 4096, wait 9.5 ms), read ADC 0x00 (3 bytes). An ADC result of 0 (read too early, conversion not done) is rejected.
- Temperature is converted again after every 4 pressure samples, giving about 80 samples/second.
- Second-order temperature compensation below 20 °C and below −15 °C. The datasheet example comes out at exactly 20.07 °C and 100009 Pa.
- Detection: I2C 0x76 / 0x77 after BMP280 / BMP388, or on SPI.

## 3. RM3100 compass (`hal/mag/rm3100_mag.h`)
- I2C 0x20 .. 0x23 (probed after HMC / QMC), or SPI (see section 5).
- Chip recognition: the three cycle count registers (0x04 .. 0x09) at their default of 200, like ArduPilot. REVID is not checked, because some clones report a different value.
- Configuration: TMRC = 0x94 (150 Hz), CMM = 0x71 (continuous, X Y Z).
- Data: 0x24, 24-bit signed, big-endian, 7500 LSB/gauss. STATUS bit 7 = new data available.
- 0,0,0 samples are dropped. A reading stuck for more than 30 times in a row reports unhealthy (like the QMC5883).
- `sensors.mag_model` adds `RM3100` (the enum is only appended at the end).

## 4. Two barometers (`hal/baro/baro_select.h`, `FlightApp::init_baros`)
- The firmware takes the first two barometers found, in this order:
  1. the chip on SPI (`pins.baro_cs`);
  2. I2C 0x76;
  3. I2C 0x77.
- The second one skips the first one's address (`skipI2cAddress`).
- `sensors.baro_primary` (FIRST / SECOND) selects which one flies, like ArduPilot's BARO_PRIMARY. Switching:
  - The one in use stops being valid while the other is still valid: switch immediately.
  - Going back to the primary:
    - in flight: only when the primary has been continuously valid for 2 seconds (avoids flipping back and forth with a flaky chip);
    - disarmed: immediately.
  - On each switch, the new one gets an offset added so altitude stays continuous, with no step (each chip has its own ground pressure and drift). Disarmed and using the primary, the offset returns to 0.
- With two barometers the sensors are read in 4 rounds instead of 3; each barometer is still polled about 250 times/second, more than the chip rate.
- BARO health bit: one of the two being valid is enough. Telemetry `baro_pa` and `temp` come from the one in use.
- Boot report:
  - `baro: BMP280 (I2C 0x76)`;
  - `baro2: MS5611 (I2C 0x77), stands by (...)`, or `flies (baro_primary 1)` when the second one is selected.

## 5. Barometer and compass on the IMU's SPI bus (`pins.baro_cs`, `pins.mag_cs`)
- Share SCK / MISO / MOSI with the IMU, one CS pin per chip (like `imu2_cs`).
  - `BaroSensor::BaroInitShared`: BMP280 / BMP388 / MS5611, 8 MHz clock.
  - `MagSensor::MagInitShared`: RM3100 only, 1 MHz clock.
- **Locking:**
  - The IMU is read in the flight task (on the S3 through the direct DMA path, not through the SPI library's mutex). The barometer / compass are read in the sensor thread.
  - So every `update()` of a chip on the IMU bus runs under `sync_` (the flight loop lock, a mutex with priority inheritance). The flight task waits at most one short transaction (a few tens of µs).
  - The IMU's direct read path resets the clock itself when it sees someone else has changed it (`bus_spi.h`).
- **Old bug fixed:** the AK8963 in the MPU9250 is also read over the IMU's SPI bus from the sensor thread and previously without a lock. It now also runs under the lock (`MagSensor::onImuBus()`).
- CS set but no chip present:
  - barometer: reports `baro: FAILED - ...` then continues probing on I2C;
  - compass: tries I2C / AK8963 as usual.
- IMU not on SPI: reports the reason.
- `baro_cs` / `mag_cs` are checked for pin conflicts like every other pin.
- Settings:
  - pins gains 2 bytes (`compat` [50, 48, 47, 46]). The 37-byte pin map of 0.7.1 is no longer accepted (only 4 old sizes are kept).
  - sensors gains `baro_primary` (`compat` [19, 18, 16]).
- App:
  - Pins tab: group "Barometer and compass on the IMU's SPI bus", with a warning when the IMU has no CS;
  - Sensors tab: "Primary barometer".

## 6. To try on the board
1. ICM-20948 / ICM-20649 on SPI at 9 kHz:
   - axis values and signs;
   - on the S3, the direct read path with a 7 MHz clock.
2. MS5611 on I2C and SPI (GY-63 module):
   - pressure compared with another barometer;
   - sample rate around 80 Hz in the Sensors tab.
3. RM3100 on I2C and SPI:
   - field strength around 0.25 – 0.65 gauss;
   - axis orientation relative to the craft's nose.
4. Two barometers: unplug the primary barometer during Altitude Hold; the craft must hold altitude, with no jump.
5. Barometer / compass on SPI together with the IMU at a 4 – 8 kHz loop:
   - `gap_max_us` and the missed-loop count in the Overview tab;
   - no extra gyro noise.
