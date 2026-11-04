# Barometer (BMP280 / BME280 / BMP388 / BMP390)

| File | Role |
|---|---|
| `hal/baro/bmp280_baro.h` | BMP280 driver (BME280 uses P/T only), I2C 0x76/0x77 or SPI |
| `hal/baro/bmp388_baro.h` | BMP388/BMP390 driver, I2C or SPI (with dummy byte) |
| `hal/baro/baro_altitude.h` | Pressure → altitude relative to the takeoff point, jump rejection, noise measurement, expiry |
| `hal/baro/ms5611_baro.h` | MS5611 driver (0.28), I2C 0x76/0x77 or SPI - see `docs/SENSORS2.md` |
| `hal/baro/baro_select.h` | Two barometers: which one flies (0.28) |
| `hal/baro/baro_base.h` | `BaroSensor`: chip detection, init, `update()`, text error reporting |
| `hal/bus/reg_bus.h`, `reg_bus_arduino.h` | Shared register bus interface (IMU, baro, later mag/lidar) |

## Usage
```cpp
BaroSensor baro;
baro.setI2cPins(SDA, SCL);                  // or fill in getI2CBaroSensorPins() in hardware_config.h
if (!baro.BaroInit(false)) Serial.println(baro.failReason());
// loop (calling it often is fine, the driver times the conversion itself, non-blocking):
if (baro.update(micros(), millis(), armed)) {
    kf.cfg.r_baro = baro.sigmaM();          // noise measured while stationary
    kf.update_baro(baro.altitudeM());       // altitude relative to the ground at power-up (m, positive = up)
}
arm.baro_ok = baro.valid(millis());         // for AB_ALTHOLD_NO_BARO
```
- Sharing I2C with mag/lidar: create one `I2CBus` then `baro.attachI2C(bus)`; the address is selected before each transaction (only safe if one task accesses the bus).
- Baro on/off switch in the app: `baro.setEnabled(false)`.

## Design
- **Forced mode, self-timed**, read when the status register reports done (BMP280: bit `measuring`; BMP388: `drdy_press`+`drdy_temp`). Default ×8 pressure, ×1 temperature, **chip IIR off** (the Kalman filters noise, the IIR only adds delay). Rate ≈ 40 Hz.
- **Ground**: average of the first 40 samples; while **disarmed** the ground level follows slow weather drift (τ = 20 s); while **armed** it is frozen. In flight, baro drift is handled by the Kalman filter (baro offset state + lidar/GPS), not by the driver.
- Samples jumping > 20 m are rejected (accepted after 5 consecutive samples). More than 500 ms without a new sample → `valid()` = false.

## Verified (PC, 63 tests, 20+ deliberate bugs all caught)
- BMP280: matches the sample vector in the Bosch datasheet (adc_T=519888, adc_P=415148 → 25.08 °C, 100653.27 Pa) and 40 random environments against an independent double-precision model.
- BMP388: against Bosch's float algorithm over 40–115 kPa, −20…43 °C (error < 2 Pa, < 0.1 °C); unsigned calibration coefficients > 32767 (T1, T2, P6, P5; BMP280 T1, P1).
- Your real Bus/SPI/I2C runs through stubs; the real ESP32-S3 build succeeds.

## Altitude from pressure (0.43, compared with ArduPilot / PX4)
`altitude = 153.8462 × T_ground × (1 − (p / p_ground)^0.190259)`, exactly like ArduPilot's `AP_Baro::get_altitude_difference_simple`. T_ground (K) is the temperature of the baro chip itself, capped at 35 °C because the chip sits on a warm board and reads higher than the air (ArduPilot `get_external_temperature`). It is averaged together with the ground pressure, follows the air while disarmed, and stays fixed once armed. Betaflight and PX4 (`getAltitudeFromPressure`) assume the air is always 15 °C (the constant 44330). Warm air is thinner: at 35 °C, climbing 100 m reads 93.5 m with the 15 °C formula; at 0 °C it reads 105.5 m. Verified: `test_baro` (35 °C, 0 °C, board 60 °C → 35 °C, tracking and freezing), 4 `baroT:` mutants.

## Notes
- **BMP388 units**: Bosch's integer algorithm returns **Pa × 100**. The file `barometer_bmp388.c` in your Betaflight zip divides by 256 at the last step; testing against Bosch's float model shows dividing by 100 is correct, so this driver divides by 100 (if you compare with BF, this is an intentional difference).
- Not tried on a real chip yet: please run `BaroInit`, print `name()`, `pressurePa()`, `tempC()` (temperature should be near room temperature, pressure ≈ 1000 hPa ± weather) then raise/lower the board ~1 m and watch `altitudeM()`.
- SPI: the baro needs its own SPI device (`SPI_DEVICE_1`); `SPIBus` currently does not share an SPI host with the IMU.
- Cheap GY-91/GY-BMP280 modules often have a BMP280 or a BME280 mixed up; both are recognised.
