# Magnetometer (HMC5883L, QMC5883L/P, AK8963 trong MPU9250)

| File | Vai trò |
|---|---|
| `hal/mag/hmc5883l_mag.h` | HMC5883L @0x1E (id "H43", đọc thứ tự X,Z,Y big-endian, 1090 LSB/G) |
| `hal/mag/rm3100_mag.h` | RM3100 @0x20-0x23 hoặc SPI trên bus IMU (0.28, 150 Hz, 7500 LSB/G) - xem `docs/SENSORS2.md` |
| `hal/mag/qmc5883_mag.h` | QMC5883L @0x0D (200 Hz, 3000 LSB/G, mở khóa DOR) và QMC5883P @0x2C (3750 LSB/G) — thông số theo Betaflight |
| `hal/mag/ak8963_mpu9250_mag.h` | AK8963 bên trong MPU9250 qua I2C master của MPU (chạy khi IMU nối SPI), điều chỉnh độ nhạy ASA, đổi trục sang hệ MPU; **init sau IMU** |
| `hal/mag/mag_calibration.h` | Hiệu chuẩn ellipsoid: hard iron + ma trận soft iron 3×3 (Betaflight chỉ có offset); độ phủ 32 ô cầu; từ chối dữ liệu thiếu phủ, méo quá (tỉ trục > 2), nhiễu, từ trường vô lý |
| `hal/mag/mag_processor.h` | hiệu chuẩn → alignment 8 hướng + trim (cùng toán với IMU) → theo dõi cường độ ±30% |
| `hal/mag/mag_base.h` | `MagSensor`: tự dò, công tắc bật/tắt, API hiệu chuẩn cho app, `readyForFusion()` |

## Cách dùng
```cpp
mag.attachI2C(sensorBus); mag.attachImuBus(imu.regBus(), imu.detected());
if (!mag.MagInit()) Serial.println(mag.failReason());
mag.setAlignment(CW90_DEG); mag.setCalibration(saved);
// loop: if (mag.update(micros(), millis())) core.on_mag({b[0],b[1],b[2]}, mag.readyForFusion(millis()), millis());
// hiệu chuẩn: mag.startCalibration(); ...xoay máy mọi hướng, đọc mag.calibrationProgress()... mag.finishCalibration(); lưu mag.calibration()
```
`mag.rawGauss()` cho view 3D của configurator. Hiệu chuẩn lưu ở hệ chip (trước alignment) nên đổi alignment không làm mất hiệu chuẩn.

## Trong core
La bàn chỉ sửa **yaw** (quay quanh trục Down của trái đất), không bao giờ làm nghiêng máy; chỉ dùng khi `core.cfg.use_mag`, đã hiệu chuẩn, mẫu còn mới (< 300 ms) và cường độ trong ±30% giá trị đã hiệu chuẩn (dòng điện motor làm méo từ trường → tự bỏ). `declination_deg` (đông dương): yaw 0 = bắc thật. Lúc disarm và còn đứng yên, yaw được đặt khớp la bàn ngay (seed). Bật la bàn mà chưa hiệu chuẩn → không arm được (`AB_MAG_NOT_CALIBRATED`).

## Tự hiệu chuẩn (0.18.0, giống Betaflight 4.5)
- **Cách bắt đầu** (chỉ khi disarm, la bàn đã được tìm thấy):
  - nút "Tự hiệu chuẩn (xoay tay)" trong tab Cảm biến (lệnh `MAG_AUTOCAL`);
  - hoặc ngoài sân, bằng cần: **ga lên hết (trên `receiver.max_check`), cần pitch kéo về (dưới `min_check`), roll và yaw ở giữa, giữ 1 giây** (`app/stick_commands.h`). Cùng nơi đó có lệnh hiệu chuẩn gyro: ga thấp, yaw trái, pitch kéo về.
  - Lúc bắt đầu beeper kêu như khi hiệu chuẩn gia tốc.
- **Sau đó**: nhấc máy lên, xoay theo mọi hướng. Gyro quay trên 30 °/s mới tính là "đang xoay"; đặt máy nằm yên thì không thu mẫu.
- **Cách tính** (`hal/mag/mag_autocal.h`):
  - Khớp hình cầu bình phương tối thiểu: |u|² = 2 b·u + c, với u = M·raw, M là ma trận soft iron đang dùng (đơn vị nếu chưa có).
  - Cộng dồn thành phương trình chuẩn 4×4, tương đương RLS không có hệ số quên.
  - Offset mới = M⁻¹·b. **Ma trận M giữ nguyên**: lần hiệu chuẩn đầy đủ (ellipsoid) vẫn còn giá trị, chỉ hard iron được làm mới (ví dụ khi dây buộc pin, cột GPS hay con ốc bị dời chỗ).
- **Tự dừng** khi đủ cả bốn điều kiện:
  - mẫu phủ ít nhất 18/32 ô hướng (8 phương vị × 4 dải độ cao);
  - có ít nhất 60 mẫu;
  - máy đã được xoay ít nhất 4 s;
  - ước lượng đứng yên (đổi dưới 0,004 G) suốt 1,5 s.
  - Thường mất 5 – 15 s. Khoảng cách giữa hai mẫu giữ lại bằng 5 % cường độ trường, nên bộ đệm 300 mẫu dùng được lâu như nhau dù trường mạnh hay yếu.
- **Thất bại**:
  - không ai xoay trong 15 s: `NO_MOTION`;
  - hết 30 s mà chưa đủ: `TOO_FEW_SAMPLES` / `POOR_COVERAGE`;
  - cường độ trường ngoài 0,15 – 0,9 G: `BAD_FIELD_STRENGTH`;
  - sai số khớp trên 8 %: `BAD_FIT`.
  - offset trên 1,8 G ở bất kỳ trục nào (nam châm hoặc sắt thép sát chip; `COMPASS_OFFS_MAX` 1800 mG của ArduPilot): `OFFSET_TOO_LARGE` (0.44.0; hiệu chuẩn đầy đủ cũng kiểm tra như vậy).
- **Số đọc không phải từ trường** (0.44.0): toàn số 0 (chip đang reset, mất bus) hoặc NaN / Inf bị bỏ qua, nên la bàn trở thành cũ thay vì lái hướng mũi. Khi đã hiệu chuẩn, số 0 sẽ thành -offset và có thể lọt qua kiểm tra cường độ.
  - Thất bại thì hiệu chuẩn cũ giữ nguyên, beeper kêu kiểu lỗi.
- **Xong**: hiệu chuẩn mới được dùng ngay, beeper kêu 3 tiếng, và **firmware tự ghi section `calibration` vào flash** (giống level trim khi bay): ngoài sân không có app để bấm Lưu.
- Trong lúc chạy, arming bị chặn (blocker CALIBRATING) và LED hiện màu đang hiệu chuẩn.
- Stream `calibration`: kind `MAG_AUTO`, state, progress, và trường mới `result` (lý do thất bại, cũng dùng cho lần hiệu chuẩn bằng tay).
- Kiểm thử:
  - `test_mag` nhóm auto calibration: xoay tay, có / không có ma trận soft iron, không xoay, chỉ xoay phẳng, trường mạnh gấp 4, la bàn im giữa chừng, huỷ.
  - `test_app` nhóm stick commands.
  - `test_fw` nhóm `magauto`: lệnh bằng cần, xoay, tự lưu flash, heading đúng sau đó, lệnh từ app, không xoay, huỷ, không có la bàn.
  - `test_e2e` `test_automatic_compass_calibration` và trình duyệt `test_compass_calibration_flow` (lệnh `tumble` mới của wb_sim).
  - Mutant: `python3 tools/mutate.py magauto`.
- **Chưa thử trên máy thật**: ngưỡng 30 °/s và 18/32 ô có thể cần chỉnh theo cách người cầm máy xoay.

## Đã kiểm chứng (PC, 90 test + 17 lỗi cố ý đều bị bắt)
Driver vs mô phỏng mức thanh ghi (kể cả mô phỏng I2C master của MPU9250); hiệu chuẩn phục hồi offset và soft-iron có thành phần chéo, từ chối nửa cầu / méo 4:1 / từ trường 2,5 G / nhiễu 12%; luồng hiệu chuẩn đầy đủ qua `MagSensor`; hợp nhất yaw (khai báo declination, nghiêng, sai 175°, quay 90°/s) — `test/test_mag.cpp` và `test/test_flight.cpp`.
## Chưa kiểm chứng
Chưa chạy trên chip thật. Điểm dễ sai nhất: trục của từng module GY-271 so với khung gắn (dùng `setAlignment` và quan sát: quay máy quanh trục thẳng đứng, heading phải tăng khi quay phải); AK8963 qua I2C master (thanh ghi SLV0/EXT_SENS_DATA theo hiểu biết datasheet, Betaflight trong zip chỉ dùng bypass).
