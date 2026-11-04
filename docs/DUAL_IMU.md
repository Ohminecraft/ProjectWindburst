# Two IMUs (0.14.0)

The board has two IMU chips, like Betaflight's `gyro_to_use`. The second chip sits **on the same SPI bus** as the first (shared SCK / MISO / MOSI) and has its own CS pin.

| Setting | Meaning |
|---|---|
| `pins.imu2_cs` | CS of the second IMU (Pins tab, "Second IMU" group). 255 means there is no second IMU |
| `sensors.gyro_use` | `FIRST` (default): fly on IMU 1, IMU 2 as backup. `SECOND`: fly on IMU 2's gyro. `BOTH`: average the two gyros (uncorrelated noise, reduced to about 1/√2) |
| `sensors.imu2_align` | orientation of the second chip on the board (Betaflight `gyro_2_sensor_align`). The board fine trim (`board_trim_*`) applies to both chips |

## How it runs (`src/hal/acc_gyro/imu_select.h`, `imusel::Selector`)
- **FIRST**: every cycle reads IMU 1. IMU 2 is read every 8 cycles to know it is still alive and to calibrate its gyro bias. In any cycle where IMU 1 does not answer, IMU 2 is read within that same cycle.
- **SECOND / BOTH**: both chips are read every cycle. SPI bus time doubles (see "IMU read" in the Overview tab).
- The **accelerometer** always comes from IMU 1, the chip that has the 6-face calibration. IMU 2's accelerometer is only used when IMU 1 does not answer; it then has no calibration of its own (offset 0, scale 1).
- **Failed chip**: a missing sample means that cycle takes the other chip's. 20 missing samples in a row mark the chip **LOST**, and it is not read again as long as the other chip keeps running, to avoid flipping between two chips that disagree. Only when no chip is left is a LOST chip retried. Both silent for 20 cycles triggers an emergency disarm, as with a single IMU.
- **Gyro calibration** at startup and the command from the app run on both chips. Arming only requires the chips the current flight mode uses to have finished calibrating.
- Gyro rate: with `SECOND` / `BOTH`, `loop_hz` is limited by the slower chip.

## Limitations
- Only a shared SPI bus is supported. A second IMU on I2C or on a separate SPI bus is not supported.
- If IMU 1 is absent or runs on I2C, IMU 2 does not run (reported in the boot report).
- The two gyros are not compared to detect a chip that "lies" (still answers but with wrong numbers). Only a chip that stops answering or freezes is detected.
- `bus_spi.h` adds `SPIInitShared()`: the second device reuses the first device's `SPIClass` and does not re-attach the pins. On the ESP32, creating a second `SPIClass` for the same host steals the first one's pins.
- Settings: the `pins` section gains 1 byte. Pin maps from 0.11 (47 bytes), 0.8 – 0.10 (46) and 0.7.1 (37) still load. Pin maps from 0.7.0 and earlier (33 bytes) **no longer load** (each section only keeps three old sizes). The old 16-byte `sensors` section loads.

## App
- Pins tab: "Second IMU" group (CS). Warns when IMU 1 is not on SPI.
- Sensors tab: "Which gyro to use", "Second IMU chip orientation", and the "Second IMU" line. This line shows the status, the chip the flight loop is using, and IMU 2's gyro next to IMU 1's gyro. Rotate the craft by hand: the two sets of numbers must match; if they differ, the chip orientation is wrong.
- Stream `sensors`: `gyro2[3]`, `imu2` (NONE / OK / FAILED / LOST), `gyro_src` (IMU1 / IMU2). Health gains the `IMU2` bit.

## Tests
- `test_accgyro` group "two IMUs": `Selector` with two simulated chips (chip selection, backup, LOST, return).
- `test_fw` group `dualimu` (`WB_ONLY=dualimu`): the second chip rotated 90° on the virtual board. Checks the boot report, bias, IMU 2 matching IMU 1 when rolling, IMU 1 dying mid-flight while still flying level on IMU 2, BOTH / SECOND, CS with no chip, duplicate CS.
- `test_proto`: old sizes of `pins` / `sensors`. Mutants: `python3 tools/mutate.py imu2` (12 mutants).
