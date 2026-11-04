# Barometer (BMP280 / BME280 / BMP388 / BMP390)

| File | Vai trò |
|---|---|
| `hal/baro/bmp280_baro.h` | Driver BMP280 (BME280 chỉ dùng P/T), I2C 0x76/0x77 hoặc SPI |
| `hal/baro/bmp388_baro.h` | Driver BMP388/BMP390, I2C hoặc SPI (có dummy byte) |
| `hal/baro/baro_altitude.h` | Áp suất → độ cao so với điểm cất cánh, chống nhảy, đo nhiễu, hết hạn |
| `hal/baro/ms5611_baro.h` | Driver MS5611 (0.28), I2C 0x76/0x77 hoặc SPI - xem `docs/SENSORS2.md` |
| `hal/baro/baro_select.h` | Hai khí áp kế: cái nào bay (0.28) |
| `hal/baro/baro_base.h` | `BaroSensor`: dò chip, init, `update()`, báo lỗi bằng chữ |
| `hal/bus/reg_bus.h`, `reg_bus_arduino.h` | Giao diện bus thanh ghi dùng chung (IMU, baro, sau này mag/lidar) |

## Cách dùng
```cpp
BaroSensor baro;
baro.setI2cPins(SDA, SCL);                  // hoặc điền getI2CBaroSensorPins() trong hardware_config.h
if (!baro.BaroInit(false)) Serial.println(baro.failReason());
// vòng lặp (gọi thường xuyên cũng được, driver tự canh thời gian chuyển đổi, không chặn):
if (baro.update(micros(), millis(), armed)) {
    kf.cfg.r_baro = baro.sigmaM();          // độ nhiễu đo được lúc đứng yên
    kf.update_baro(baro.altitudeM());       // độ cao so với mặt đất lúc bật nguồn (m, dương = lên)
}
arm.baro_ok = baro.valid(millis());         // cho AB_ALTHOLD_NO_BARO
```
- Dùng chung I2C với mag/lidar: tạo một `I2CBus` rồi `baro.attachI2C(bus)`; địa chỉ được chọn trước mỗi giao dịch (chỉ an toàn nếu một task truy cập bus).
- Công tắc bật/tắt baro trong app: `baro.setEnabled(false)`.

## Thiết kế
- **Forced mode, tự canh thời gian**, đọc khi thanh ghi trạng thái báo xong (BMP280: bit `measuring`; BMP388: `drdy_press`+`drdy_temp`). Mặc định ×8 áp suất, ×1 nhiệt độ, **tắt IIR của chip** (Kalman lọc nhiễu, IIR chỉ thêm trễ). Tốc độ ≈ 40 Hz.
- **Mặt đất**: trung bình 40 mẫu đầu; khi **disarm** mặt đất trôi theo thời tiết chậm (τ = 20 s); khi **arm** thì đóng băng. Trong bay, trôi baro do bộ lọc Kalman xử lý (trạng thái baro offset + lidar/GPS), không do driver.
- Mẫu nhảy > 20 m bị loại (5 mẫu liên tiếp mới chấp nhận). Quá 500 ms không có mẫu mới → `valid()` = false.

## Đã kiểm chứng (PC, 63 test, 20+ lỗi cố ý đều bị bắt)
- BMP280: đúng vector mẫu trong datasheet Bosch (adc_T=519888, adc_P=415148 → 25,08 °C, 100653,27 Pa) và 40 môi trường ngẫu nhiên so với mô hình double-precision độc lập.
- BMP388: so với thuật toán float của Bosch trên 40–115 kPa, −20…43 °C (sai số < 2 Pa, < 0,1 °C); hệ số hiệu chuẩn unsigned > 32767 (T1, T2, P6, P5; BMP280 T1, P1).
- Bus/SPI/I2C thật của bạn chạy qua stub; build thật cho ESP32-S3 thành công.

## Độ cao từ áp suất (0.43, so với ArduPilot / PX4)
`độ cao = 153,8462 × T_mặt_đất × (1 − (p / p_mặt_đất)^0,190259)`, đúng như `AP_Baro::get_altitude_difference_simple` của ArduPilot. T_mặt_đất (K) là nhiệt độ của chính chip baro, giới hạn tối đa 35 °C vì chip nằm trên board nóng và đọc cao hơn không khí (ArduPilot `get_external_temperature`). Nó được lấy trung bình cùng áp suất mặt đất, đi theo không khí khi chưa arm, và đứng yên khi đã arm. Betaflight và PX4 (`getAltitudeFromPressure`) giả định không khí luôn 15 °C (hằng số 44330). Không khí nóng thì loãng hơn: ở 35 °C, leo 100 m thì công thức 15 °C đọc ra 93,5 m; ở 0 °C thì đọc ra 105,5 m. Kiểm chứng: `test_baro` (35 °C, 0 °C, board 60 °C → 35 °C, theo dõi và đóng băng), 4 mutant `baroT:`.

## Lưu ý
- **Đơn vị BMP388**: thuật toán số nguyên của Bosch trả **Pa × 100**. File `barometer_bmp388.c` trong zip Betaflight của bạn chia cho 256 ở bước cuối; test với mô hình float của Bosch cho thấy chia 100 mới đúng, nên driver này chia 100 (nếu bạn đối chiếu với BF thì đây là chỗ khác nhau có chủ đích).
- Chưa thử trên chip thật: nhờ bạn chạy `BaroInit`, in `name()`, `pressurePa()`, `tempC()` (nhiệt độ phải gần nhiệt độ phòng, áp suất ≈ 1000 hPa ± thời tiết) rồi nâng/hạ board ~1 m xem `altitudeM()`.
- SPI: baro cần SPI device riêng (`SPI_DEVICE_1`); `SPIBus` hiện không chia sẻ chung host SPI với IMU.
- Module GY-91/GY-BMP280 rẻ tiền hay có chip BMP280 hoặc BME280 pha nhau; cả hai đều được nhận.
