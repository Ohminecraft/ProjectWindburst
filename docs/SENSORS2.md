# Cảm biến thêm ở 0.28: ICM-20948 / ICM-20649, MS5611, RM3100, hai khí áp kế, cảm biến trên bus SPI của IMU

Tất cả mới chỉ kiểm tra trên PC (driver chạy với chip mô phỏng ở mức thanh ghi, firmware chạy trên mạch ảo). **Chưa thử trên mạch thật.**
Thông số lấy từ datasheet và driver của ArduPilot (`external_source/ardupilot`), Betaflight.

## 1. IMU ICM-20948 và ICM-20649 (`hal/acc_gyro/icm20948_accgyro.h`)
- Bản đồ thanh ghi "v2" có 4 bank (REG_BANK_SEL 0x7F). WHO_AM_I bank 0 thanh ghi 0x00: 0xEA (20948), 0xE1 (20649).
- Khởi động: reset PWR_MGMT_1 = 0x80, chờ 100 ms, rồi 0x01 (thức, clock tốt nhất). SPI: USER_CTRL = 0x10 (tắt I2C).
- Thang đo: 2000 dps / 16 g trên cả hai chip. Mã FS khác nhau: 20948 dùng 3, 20649 dùng 2 (20649 đếm từ 500 dps / 4 g). ArduPilot để 20649 ở 30 g; ở đây giữ 16 g như Betaflight.
- Tốc độ: SPI và vòng lặp > 1125 Hz thì bỏ bộ lọc trong chip (FCHOICE 0): gyro 9 kHz, gia tốc 4,5 kHz. I2C (hoặc ≤ 1125 Hz): DLPF bật, 1125 Hz.
- Đọc: bank 0, 0x2D, 12 byte big-endian (gia tốc rồi gyro). Driver luôn để chip ở bank 0. SPI tối đa 7 MHz.
- Dò chip:
  - SPI: tìm sau mọi chip khác (giống Betaflight), vì ở họ MPU thanh ghi 0x00 là giá trị tự kiểm tra của nhà máy, có thể trùng 0xEA / 0xE1.
  - Chip bị bỏ lại ở bank khác (khởi động lại nóng giữa lúc init): lần thử thứ hai ghi 0x7F = 0 rồi dò lại.
  - I2C 0x68 / 0x69: đọc 0x75 trước (MPU), không phải MPU thì chọn bank 0 và đọc 0x00.
- La bàn AK09916 bên trong ICM-20948 chưa dùng.

## 2. Khí áp kế MS5611 (`hal/baro/ms5611_baro.h`)
- Chip điều khiển bằng lệnh, không có thanh ghi. `AgRegBus` có thêm `command()` và `command_read()`:
  - I2C: một byte lệnh (`I2CBus::I2CWriteByte`), hoặc lệnh rồi đọc.
  - SPI: byte lệnh gửi nguyên, không thêm cờ đọc 0x80.
- Trình tự:
  - reset 0x1E;
  - PROM 0xA0 + 2i: 8 word. Không có chip id: CRC-4 (theo AN520) đúng, và PROM không toàn 0 / toàn 1, mới coi là MS5611;
  - D1 0x48, D2 0x58 (OSR 4096, chờ 9,5 ms), đọc ADC 0x00 (3 byte). ADC trả 0 (đọc sớm, chưa đổi) thì bị loại.
- Nhiệt độ đổi lại sau mỗi 4 mẫu áp suất, nên được khoảng 80 mẫu/giây.
- Bù nhiệt bậc hai dưới 20 °C và dưới −15 °C. Ví dụ của datasheet ra đúng 20,07 °C và 100009 Pa.
- Dò chip: I2C 0x76 / 0x77 sau BMP280 / BMP388, hoặc trên SPI.

## 3. La bàn RM3100 (`hal/mag/rm3100_mag.h`)
- I2C 0x20 .. 0x23 (dò sau HMC / QMC), hoặc SPI (xem mục 5).
- Nhận chip: ba thanh ghi cycle count (0x04 .. 0x09) đang ở mặc định 200, giống ArduPilot. REVID không kiểm tra, vì có bản clone báo giá trị khác.
- Cấu hình: TMRC = 0x94 (150 Hz), CMM = 0x71 (liên tục, X Y Z).
- Dữ liệu: 0x24, 24 bit có dấu, big-endian, 7500 LSB/gauss. STATUS bit 7 = có dữ liệu mới.
- Bỏ mẫu 0,0,0. Số đọc đứng yên quá 30 lần liên tiếp thì báo không khoẻ (như QMC5883).
- `sensors.mag_model` thêm `RM3100` (enum chỉ nối thêm ở cuối).

## 4. Hai khí áp kế (`hal/baro/baro_select.h`, `FlightApp::init_baros`)
- Firmware lấy hai khí áp kế tìm thấy đầu tiên, theo thứ tự:
  1. chip trên SPI (`pins.baro_cs`);
  2. I2C 0x76;
  3. I2C 0x77.
- Cái thứ hai bỏ qua địa chỉ của cái thứ nhất (`skipI2cAddress`).
- `sensors.baro_primary` (FIRST / SECOND) chọn cái bay, giống BARO_PRIMARY của ArduPilot. Cách chuyển:
  - Cái đang dùng hết hợp lệ mà cái kia còn hợp lệ: chuyển ngay.
  - Quay về cái chính:
    - đang bay: chỉ khi cái chính hợp lệ liên tục 2 giây (tránh nhảy qua lại với chip chập chờn);
    - đang disarm: ngay.
  - Mỗi lần chuyển, cái mới được cộng một độ lệch để độ cao nối tiếp, không nhảy bậc (mỗi chip có áp suất mặt đất và độ trôi riêng). Disarm và đang dùng cái chính thì độ lệch về 0.
- Hai khí áp kế thì cảm biến được đọc theo 4 lượt thay vì 3; mỗi khí áp kế vẫn được hỏi khoảng 250 lần/giây, nhiều hơn tốc độ chip.
- Health bit BARO: một trong hai hợp lệ là đủ. Telemetry `baro_pa` và `temp` lấy từ cái đang dùng.
- Báo cáo khởi động:
  - `baro: BMP280 (I2C 0x76)`;
  - `baro2: MS5611 (I2C 0x77), stands by (...)`, hoặc `flies (baro_primary 1)` khi chọn cái thứ hai.

## 5. Khí áp kế và la bàn trên bus SPI của IMU (`pins.baro_cs`, `pins.mag_cs`)
- Chung SCK / MISO / MOSI với IMU, mỗi chip một chân CS (giống `imu2_cs`).
  - `BaroSensor::BaroInitShared`: BMP280 / BMP388 / MS5611, clock 8 MHz.
  - `MagSensor::MagInitShared`: chỉ RM3100, clock 1 MHz.
- **Khoá:**
  - IMU được đọc trong flight task (trên S3 qua đường DMA trực tiếp, không qua mutex của thư viện SPI). Khí áp kế / la bàn đọc ở sensor thread.
  - Vì vậy mỗi lần `update()` của chip trên bus IMU chạy dưới `sync_` (khoá của vòng bay, mutex có kế thừa ưu tiên). Flight task chờ nhiều nhất bằng một giao dịch ngắn (vài chục µs).
  - Đường đọc trực tiếp của IMU tự đặt lại clock khi thấy người khác đã đổi clock (`bus_spi.h`).
- **Sửa lỗi cũ:** AK8963 trong MPU9250 cũng được đọc qua bus SPI của IMU từ sensor thread mà trước đây không khoá. Nay nó cũng chạy dưới khoá (`MagSensor::onImuBus()`).
- CS đặt mà không có chip:
  - khí áp kế: báo `baro: FAILED - ...` rồi dò tiếp trên I2C;
  - la bàn: thử I2C / AK8963 như thường.
- IMU không ở SPI: báo lý do.
- `baro_cs` / `mag_cs` được kiểm tra trùng chân như mọi chân khác.
- Settings:
  - pins thêm 2 byte (`compat` [50, 48, 47, 46]). Bản đồ chân 37 byte của 0.7.1 không còn được nhận (chỉ giữ 4 kích thước cũ).
  - sensors thêm `baro_primary` (`compat` [19, 18, 16]).
- App:
  - tab Chân: nhóm "Khí áp kế và la bàn trên bus SPI của IMU", có cảnh báo khi IMU không có CS;
  - tab Cảm biến: "Khí áp kế chính".

## 6. Cần thử trên mạch
1. ICM-20948 / ICM-20649 trên SPI ở 9 kHz:
   - giá trị và dấu các trục;
   - trên S3, đường đọc trực tiếp với clock 7 MHz.
2. MS5611 trên I2C và SPI (module GY-63):
   - áp suất so với một khí áp kế khác;
   - tần số mẫu khoảng 80 Hz trong tab Cảm biến.
3. RM3100 trên I2C và SPI:
   - cường độ từ trường khoảng 0,25 – 0,65 gauss;
   - hướng trục so với hướng mũi máy.
4. Hai khí áp kế: rút dây khí áp kế chính khi đang Altitude Hold; máy phải giữ độ cao, không nhảy.
5. Khí áp kế / la bàn trên SPI cùng IMU ở vòng lặp 4 – 8 kHz:
   - `gap_max_us` và số vòng lỡ trong tab Tổng quan;
   - gyro không nhiễu thêm.
