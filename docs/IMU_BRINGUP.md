# IMU driver — kiểm tra trên mạch thật

## Đã kiểm chứng trên PC (`make test`, 66+148 test cho phần IMU)
- Mỗi chip trong `IMUAccGyroDevice` chạy qua bộ mô phỏng mức thanh ghi: đúng endian, đúng thứ tự trục gyro/accel, đúng scale theo mã range mà driver ghi,
  đúng bank/IREG (ICM426xx/ICM456xx), đúng upload microcode 8192 byte (BMI270), SPI wake (BMI160/270), dummy byte (BMI270).
- `IMUAccGyro` thật + `SPIBus`/`I2CBus` thật của bạn chạy qua stub Arduino: detect, chọn driver, init, sanity check 1 g, alignment, gyro bias, báo lỗi.
- Đã thử phá cố ý 16 chỗ (đổi endian, sai bank, sai địa chỉ microcode...) → test đều bắt được.

## Chưa kiểm chứng được (cần mạch thật)
Bộ mô phỏng viết từ cùng nguồn (Betaflight + datasheet) với driver, nên KHÔNG bắt được chỗ hiểu sai chung. Kiểm chứng thật:
1. `IMUInit()` trả true? Nếu false → `failReason()` nói lý do. In ra `name()`, `gyroRateHz()`.
2. Đặt board nằm yên: accel phải ≈ (0,0,-1 g) sau khi chỉnh alignment (Z chúc xuống = -1). Nghiêng mũi lên → xem trục nào đổi.
3. Xoay tay quanh từng trục: gyro dấu có khớp quy ước FRD không (roll phải = +x, mũi lên = +y, quay phải nhìn từ trên = +z).
4. Nhiễu gyro khi đứng yên (rms nên < ~0.1–0.3 dps). Cao bất thường → nghi SPI clock/dây/nguồn.
5. Đo tốc độ đọc thực (số mẫu/giây); `data_ready()` chỉ là tiện ích polling, dùng chân INT khi bay thật.

## Những điểm mình KHÔNG chắc 100% (kiểm tra đầu tiên nếu có lỗi)
- **Bus SPI trên ESP32-S3**: `SPIBus_device_e` trước đây là 2 / 3 (số của ESP32 đời đầu); trên S3 core Arduino chỉ nhận 0 (FSPI) / 1 (HSPI) nên `SPIInit()` luôn thất bại ("SPI bus init failed"). Đã sửa trong `bus_spi.h`; lần chạy đầu trên N16R8 là lần đầu bus này chạy thật. Xem `docs/BOARD_N16R8.md` mục 6.
- BMI270 microcode: lấy từ gói PyPI `bmi270` 0.4.3 (không có trong zip Betaflight của bạn). Đối chiếu với `BMI270_SensorAPI` của Bosch.
- Thanh ghi status dùng cho `data_ready()` của ICM426xx (0x2D bit3), ICM456xx (0x19 bit2), BMI160/270 (0x1B / 0x03 bit6): không nằm trong code Betaflight (BF dùng ngắt), mình lấy theo datasheet.
- ICM42686P: `FS_SEL=1` cho ±2000 dps / ±16 g (mã 0 là ±4000 dps / ±32 g) — theo bảng datasheet, không có trong BF đã đọc.
- SPI clock của `SPIBus`: ghi 1 MHz, đọc 8 MHz, mode 3. MPU6000 đọc dữ liệu được tới 20 MHz nên ổn; nếu nhiễu thì hạ `bus_clock_fast_hz`.

## Cách dùng
```cpp
#include "hal/acc_gyro/acc_gyro_base.h"
IMUAccGyro imu;
imu.setSpiPins(CS, MISO, MOSI, SCK);        // hoặc điền trong hardware_config.h
if (!imu.IMUInit(true)) Serial.println(imu.failReason());
imu.setAlignment(CW0_DEG);                   // hướng chip trên PCB, + tinh chỉnh roll/pitch/yaw nếu cần
imu.calibrateGyro();                         // để máy đứng yên
AgSample s; imu.read(s);                     // s.gyro_dps[3], s.acc_g[3] — hệ thân máy (FRD)
```

## Bổ sung khi ghép firmware (dùng bởi `FlightApp`, xem `docs/FIRMWARE.md`)
- `startGyroCalibration(n)` / `gyroCalState()` / `gyroCalProgress()`: hiệu chuẩn gyro **không chặn** — `read()` tự gom `n` mẫu; bias chỉ đổi khi máy đứng yên. (`calibrateGyro()` kiểu chặn vẫn còn.)
- `setAccCalibration(AccCal)` / `rawAcc()`: offset + scale của accelerometer, áp ở **hệ trục chip** trước alignment; `rawAcc()` là đầu vào cho `AccCalibrator` (`acc_calibration.h`: hiệu chuẩn 6 mặt, `level_trim()`).
- **Phát hiện chip đông cứng**: 32 mẫu liên tiếp giống hệt từng bit ở cả 6 trục → `read()` trả false (`frozen()`). Chip MEMS thật luôn có nhiễu ở bit thấp; dây MISO đứt đọc ra hằng số. `setFrozenLimit(0)` để tắt.
  Cần kiểm tra trên mạch: khi máy nằm **thật yên**, `frozen()` không được bật (nếu loop nhanh hơn tốc độ mẫu của chip thì mẫu sẽ lặp — `FlightApp` đã giới hạn loop ≤ tốc độ gyro).
