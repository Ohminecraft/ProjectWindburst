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

## Kiểm thử
- `test_flight` nhóm `pid_denom`: chu kỳ nào là chu kỳ điều khiển, motor giữ lệnh, gyro lọc mới ở mọi mẫu, ước lượng giữ 500 Hz, Angle / Acro 5 inch / Stabilize / tư thế qua cú lật với chia 1, 2, 4.
- `test_fw` nhóm `loop`: báo cáo khởi động, `status.pid_hz` / `pid_target`, 2000 lệnh motor mỗi giây ở 4000 / 2, hạ `pid_denom` khi dưới 500 Hz, cơ sở thời gian của log.
- `test_proto`: bố cục và nạp section `general` cũ. `test_core` nhóm fast trigonometry.
- Mutant: `python3 tools/mutate.py denom`, `python3 tools/mutate.py fastmath`.
