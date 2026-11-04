# Tần số gyro và tần số PID tách riêng (0.12.0)

Trước 0.12 mỗi mẫu gyro chạy toàn bộ một chu kỳ: lọc gyro, tính tư thế, mode, PID, mixer, gửi lệnh motor, ghi blackbox. Từ 0.12, giống `pid_process_denom` của Betaflight:

| Setting | Ý nghĩa |
|---|---|
| `general.loop_hz` | **tần số gyro**: số mẫu đọc mỗi giây, timer phần cứng đánh thức vòng lặp ở tần số này. Bộ lọc gyro (lowpass, notch tĩnh, dynamic notch, lọc RPM) chạy ở đây và thấy **mọi** mẫu, nên rung trên tần số PID không bị gập (alias) vào vòng điều khiển |
| `general.pid_denom` (1 – 8, mặc định 1) | phần điều khiển chạy mỗi n mẫu gyro: **tần số PID = loop_hz / pid_denom** |

Phần chạy ở tần số PID: tư thế (AHRS, tích phân trên **trung bình** gyro của cả cửa sổ n mẫu, với đúng thời gian đã trôi qua), RC / arm / failsafe, Altitude Hold, mode Angle / Stabilize / Acro, PID (gồm lọc D-term, anti-gravity, TPA, feedforward), mixer, runaway, lệnh motor (DShot / PWM), hỏi telemetry ESC, bản ghi blackbox. Bộ ước lượng (`est_hz`) vẫn giữ tần số của nó.

Mẫu gyro "chỉ lọc" (không phải chu kỳ điều khiển) trả về kết quả của chu kỳ điều khiển trước, chỉ thay gyro đã lọc; motor giữ lệnh cũ.

## Giới hạn
- Tần số PID không bao giờ dưới 500 Hz: nếu `loop_hz / pid_denom < 500`, firmware hạ `pid_denom` cho đến khi đủ (ví dụ gyro 2000 Hz, chia 8 → chạy chia 4 = 500 Hz). Báo cáo khởi động ghi cả hai: `imu: SPI ..., gyro 8000 Hz, control 4000 Hz (1/2)`.
- Mặc định `pid_denom = 1`: hoạt động như các bản trước.
- Settings cũ: section `general` 22 byte (0.9.5 – 0.11) và 23 byte (0.9.0 – 0.9.4) đều nạp được; byte thứ 23 của bản cũ rơi vào ô `reserved_g` không dùng, nên không thành `pid_denom` rác.

## App và log
- Stream `status` thêm `pid_hz` (số chu kỳ điều khiển thật trong giây vừa rồi) và `pid_target`. Tab Tổng quan có dòng "Vòng PID thực tế"; tab Cảm biến có ô "Chia tần số PID" và dòng "Đang chạy: gyro … Hz, PID … Hz"; chip "Vòng lặp" ở thanh trên thêm "· PID …" khi PID chậm hơn gyro.
- "Thời gian từng khâu": IMU và "tính toán" là trung bình trên **mọi** mẫu gyro (chu kỳ chỉ lọc rất ngắn); "gửi motor" chỉ tính chu kỳ điều khiển.
- Blackbox: một bản ghi tối đa mỗi chu kỳ điều khiển. Header `loop_hz` = tần số PID (cơ sở thời gian của log), dòng mới `gyro_hz`. (Sửa luôn lỗi cũ: header luôn ghi 4000 và bộ chia tần số log tính theo 4000 dù đặt 8000.)
- MSP (BLHeliSuite / Betaflight tool): cycle time = chu kỳ PID, như Betaflight.

## Loop Hz giảm khi arm (phản hồi 0.11: ~5700 Hz → 4400 Stabilize / 4700 Angle / 4900 Acro)
Khi disarm, chu kỳ dừng sau phần RC; khi arm còn chạy Altitude Hold / ước lượng hover, bộ điều khiển mode, PID, mixer, runaway, và lệnh motor. Hai thay đổi trong 0.12:
1. `pid_denom`: ví dụ gyro 8000 / chia 2 thì phần đắt chỉ chạy 4000 lần mỗi giây.
2. Lượng giác nhanh (`types.h`: `sin_approx`, `cos_approx`, `atan2_approx`, `asin_approx`, `acos_approx`, đa thức của Betaflight `maths.c`) thay `sinf / cosf / atan2f / asinf / acosf` của thư viện C ở chỗ chạy mỗi chu kỳ: tư thế Euler (mỗi chu kỳ, kể cả disarm), góc nghiêng, Angle (coupling yaw) và Stabilize (4 lần sin/cos). Hàm thư viện nằm trong flash (WB_FAST không chuyển được) và mất vài µs mỗi lần; đa thức nằm trong RAM cùng chỗ gọi. Sai số: sin/cos < 2e-6, atan2/asin < 1e-6 rad, acos < 7e-5 rad (0.004°).

**Chưa đo trên mạch.** Cần số: tab Tổng quan, "Thời gian từng khâu" và "Vòng PID thực tế" khi disarm / arm Acro / Angle / Stabilize, với `pid_denom` 1 và 2.

## Đo trên mạch lần đầu (0.17, Acro, đặt 6000 Hz)
| | |
|---|---|
| một chu kỳ | trung bình 131 µs, dài nhất 449 µs |
| chu kỳ chạy thật | 5566 Hz trên 6000 (mất 437 chu kỳ mỗi giây) |
| từng khâu | đọc IMU 37 · tính toán 66 · gửi motor 26 · còn lại 10 µs |
| Angle / Stabilize | ít hơn Acro khoảng 100 / 200 Hz |

Hai nguyên nhân:
1. **Hết thời gian.** Chu kỳ 6000 Hz dài 167 µs, mà task bay phải chừa cho luồng cảm biến ¼ thời gian của mỗi chu kỳ (`app::TickBudget`): 131 µs × 1,25 = 164 µs, gần sát giới hạn, nên mỗi chu kỳ dài (bộ ước lượng, dài nhất 449 µs) đều làm mất tick.
2. **6000 không chia hết tốc độ của chip.** Chip ICM / MPU khi được yêu cầu 6000 Hz sẽ chạy 8000 Hz; vòng 6000 Hz đọc 3 trong 4 mẫu, ở những khoảng cách không đều (một phần tư số mẫu cũ gấp đôi những mẫu khác), trong khi bộ lọc và PID coi chúng là đều nhau.

## Tần số vòng lặp chia hết tốc độ của chip (0.18, sau lần đo)
`app::loop_rate_for` (`src/app/sync.h`): vòng lặp lấy **mỗi mẫu thứ n** của chip, nhanh hơn setting tối đa 10 %. Ví dụ:

| setting | chip | vòng lặp |
|---|---|---|
| 6000 | 8000 (ICM, MPU) | **4000** |
| 3000 | 3200 (BMI270) | 3200 |
| 4000 / 8000 | 4000 / 8000 | không đổi |

Báo cáo khởi động ghi lý do: `imu: SPI ICM42688P (20 MHz direct), gyro 4000 Hz (setting 6000 Hz: the chip delivers 8000 Hz), control 4000 Hz (1/1)`. Muốn nhanh hơn 4000 Hz: đặt 8000 với `pid_denom` 2.

## Đọc IMU nhanh hơn (0.18, sau lần đo)
37 µs cho 13 – 15 byte là quá nhiều: ở 8 MHz dây chỉ mất khoảng 15 µs, phần còn lại là thư viện SPI của Arduino (hai mutex, đặt lại clock mỗi lần, `digitalWrite` cho chân CS - tất cả nằm trong flash).
- **Clock theo chip** (`AccGyroDriver::max_spi_read_hz`): ICM-426xx / 456xx, MPU6000 / 6500 / 9250: 20 MHz; ICM-20601 / 20602, BMI160 / 270: 10 MHz. Trước đây mọi chip đều 8 MHz. Ghi thanh ghi vẫn dùng clock chậm.
- **Đọc thẳng thanh ghi SPI** (`SPIBus::direct_read`, chỉ ESP32-S3): ghi `data_buf`, `cmd.update`, `cmd.usr` như thư viện làm, CS bằng `GPIO.out_w1ts/w1tc`; nằm trong IRAM. Chỉ IMU dùng SPI_DEVICE_0 và chỉ task bay đọc nó, nên không cần khoá. Khi clock trên thanh ghi bị người khác đổi (ghi chậm, IMU thứ hai), nó tự đặt lại qua thư viện. Bộ điều khiển không xong trong ~1 ms: một lần đọc lỗi (vòng lặp bỏ mẫu đó), không treo.
- **Tự lùi khi số đọc vô lý** (`IMUAccGyro::finish_init`): thử clock của chip + đọc trực tiếp; nếu 20 mẫu đầu không ra ~1 g thì thử clock đó qua thư viện; rồi 8 MHz qua thư viện. Báo cáo khởi động cho biết kết quả: `(20 MHz direct)`, `(20 MHz)` hoặc `(8 MHz)`.
- Tắt để so sánh: `-DWB_SPI_DIRECT=0` (về thư viện), `-DWB_IMU_SPI_MAX_HZ=8000000` (về 8 MHz, ví dụ khi dây dài).

**Chưa đo trên mạch.** Cần xem lại "Thời gian từng khâu" (đọc IMU) và dòng `imu:` của báo cáo khởi động.

## Kiểm thử
- `test_flight` nhóm `pid_denom`: chu kỳ nào là chu kỳ điều khiển, motor giữ lệnh, gyro lọc mới ở mọi mẫu, ước lượng giữ 500 Hz, Angle / Acro 5 inch / Stabilize / tư thế qua cú lật với chia 1, 2, 4.
- `test_fw` nhóm `loop`: báo cáo khởi động, `status.pid_hz` / `pid_target`, 2000 lệnh motor mỗi giây ở 4000 / 2, hạ `pid_denom` khi dưới 500 Hz, cơ sở thời gian của log.
- `test_proto`: bố cục và nạp section `general` cũ. `test_core` nhóm fast trigonometry.
- Mutant: `python3 tools/mutate.py denom`, `python3 tools/mutate.py fastmath`.
- `test_app` nhóm `loop rate`: bảng `loop_rate_for`. `test_fw` nhóm `loop`: 6000 Hz trên chip 8000 Hz chạy 4000 Hz và báo cáo nói lý do; clock SPI của IMU trong báo cáo (trên PC không có đường đọc trực tiếp).
