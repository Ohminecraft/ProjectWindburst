# EKF (0.9.0): tư thế + vận tốc + vị trí + bias của IMU trong một bộ lọc

`src/core/ekf.h`. Chạy **song song** với các bộ ước lượng cũ (`nav.ekf = SHADOW`, mặc định) và chỉ ghi vào blackbox / telemetry để so sánh. `nav.ekf = NAV`: Position Hold và GPS Rescue lấy vị trí, vận tốc, hướng mũi từ EKF khi nó đang có GPS (optical flow vẫn đi đường cũ). Tư thế mà vòng điều khiển dùng **vẫn là của AHRS** ở cả hai chế độ.

## 1. Khác gì bộ Kalman đang có

| | Bộ cũ (`ahrs.h` + `horizontal_est.h` + `vertical_est.h`) | EKF |
|---|---|---|
| Tư thế | Mahony, riêng | trạng thái của cùng bộ lọc |
| Vị trí / vận tốc | Kalman tuyến tính, coi tư thế là đúng | cùng bộ lọc, có hiệp phương sai chéo với tư thế |
| Hướng mũi khi la bàn nhiễu | mất (`YAW_FROM_MAG` tắt), Position Hold không dùng được | vận tốc GPS sửa hướng mỗi lần máy tăng tốc |
| Bias gyro / accel | gyro: tích phân Mahony; accel z: trong bộ lọc độ cao | 6 trạng thái, ước lượng cùng lúc |

"Extended": phép xoay gia tốc sang hệ trái đất không tuyến tính theo tư thế, nên mô hình được tuyến tính hoá quanh ước lượng hiện tại ở mỗi bước. "Error state": bộ lọc ước lượng **sai số nhỏ** của một trạng thái danh nghĩa được tích phân đầy đủ phi tuyến (quaternion vẫn là quaternion). Đây là dạng PX4 EKF2 dùng (Solà 2017, sai số góc toàn cục).

- Trạng thái lỗi 15: góc (3, hệ trái đất), vận tốc NED (3), vị trí NED (3), bias gyro (3), bias accel (3).
- Dự đoán từ tổng gyro / accel của **mọi mẫu IMU** (task bay cộng dồn, ~4 kHz), mỗi 4 ms (250 Hz).
- Đo:
  - GPS vị trí N/E và vận tốc N/E (D nếu u-blox), so với trạng thái của `gps_delay_s` trước (lưu lịch sử 0,5 s);
  - baro (độ cao);
  - la bàn: **chỉ hướng**, ma trận Jacobian theo góc của từ trường ngang (không phải góc yaw Euler: khi máy nghiêng hai cái khác nhau). Tối đa 10 Hz, sigma 0,5 rad như EK3_YAW_M_NSE của ArduPilot: sai số la bàn gần motor là một độ lệch chậm, không phải nhiễu trắng;
  - không có GPS quá 1 s: "đứng yên tại chỗ" (sigma 10 m, EK3_NOAID_M_NSE) để độ nghiêng không trôi;
  - disarm và đứng yên: vận tốc 0.
- Mọi phép đo có cổng (GPS 5σ, baro 5σ, la bàn 3σ). Vị trí GPS / baro bị loại liên tục 5 s: trạng thái nhảy theo (như ResetHeight / ResetPosition của ArduPilot).
- Bias accel X/Y bị buộc chặt (như PX4 EKF2): trên multirotor lực đẩy luôn dọc trục z, hai bias này gần như không quan sát được. Đây là đề phòng cho mạch thật, **test trên PC không chứng minh được nó cần**: để tự do thì test_ekf vẫn đạt (lúc đầu mình thấy hướng lệch 29° không về, nhưng đó là do lỗi ghép vận tốc-tư thế trong F, đã sửa).
- Căn chỉnh trên mặt đất: nghiêng theo accelerometer, hướng theo la bàn (chờ tối đa 3 s), không có la bàn thì lấy hướng của AHRS với độ bất định lớn.

## 2. Chi phí

Bước dự đoán chỉ nhân trên ~40 phần tử khác 0 của F (≈ 1000 phép nhân-cộng, không phải 6750). Trên PC: ~1 µs/bước. Trên ESP32-S3 dự kiến 30–60 µs mỗi bước, chạy trong luồng cảm biến **ngoài khoá**, không đụng tới chu kỳ điều khiển. Con số thật hiện ở tab Vị trí ("Một bước EKF tốn") và stream `estimate.ekf_us`: **chưa đo trên mạch**.

## 3. Kiểm chứng trên PC

`test/test_ekf.cpp` (13 kiểm tra) với một quadcopter mô phỏng biết chính xác trạng thái thật: tư thế là cái multirotor phải có để bay quỹ đạo đó (accelerometer chỉ thấy lực đẩy), IMU 250 Hz có nhiễu và bias, GPS 10 Hz hoặc 1 Hz có trễ. `WB_VERBOSE=1 ./build/test_ekf` in ra các con số:

| Tình huống | Kết quả |
|---|---|
| treo 60 s | nghiêng sai ≤ 0,37°, hướng ≤ 0,30°, vị trí ≤ 0,18 m |
| hình số 8, 30 × 15 m, nghiêng tới ~25° | nghiêng ≤ 0,43°, hướng ≤ 0,53°, vị trí ≤ 0,20 m |
| không la bàn, bắt đầu lệch 29° | bay số 8: hướng về ≤ 0,83° |
| la bàn nhiễu theo lực đẩy (một mình sai 20,7° RMS) | EKF sai 8,9° RMS (lúc treo đều thì không gì phân biệt được độ lệch tĩnh của la bàn) |
| gyro yaw lệch 1°/s | học được 0,99°/s |
| GPS 1 Hz | vị trí ≤ 0,60 m (10 Hz: 0,18 m), nghiêng ≤ 0,43° |
| GPS nhảy 25 m trong 3 s | bị loại, ước lượng lệch ≤ 0,21 m; nhảy luôn thì sau 5 s trạng thái theo |
| treo 60 s không có GPS | nghiêng ≤ 0,2° (bỏ ràng buộc "đứng yên tại chỗ": 0,9°) |
| đứng yên, accel z lệch 0,3 m/s² | học được 0,29 |

`test_fw` nhóm `ekf` (firmware thật trên mạch ảo):
- SHADOW: căn chỉnh đúng hướng 60° trên mặt đất; bay các chặng với Position Hold, hướng EKF lệch thật ≤ 4°, vị trí ≤ 1,5 m, nghiêng ≤ 3°.
- NAV: Position Hold giữ trong 0,8 m suốt 10 s, đẩy cần thì đi đúng theo mũi.
- NAV, **không có la bàn**: lúc đầu Position Hold bị từ chối (bộ cũ không biết hướng); bay vài lần tăng tốc ở Altitude Hold, EKF lấy được hướng từ vận tốc GPS (lệch 0,5°), Position Hold bật được và giữ trong 0,19 m suốt 8 s. Đây là chỗ EKF làm được việc mà bộ cũ không làm được.
- OFF: không chạy.
- `test_threads` (ThreadSanitizer) chạy cùng EKF: không có truy cập nào ngoài khoá.

## 4. Chạy lại log bay thật

`build/ekf_replay fast.csv slow.csv [out.csv] [--acc-noise x] [--gps-delay s] [--no-mag]` (`tools/ekf_replay.cpp`). Với log 2026-10-11 (firmware 0.8.7):

- **Tư thế EKF và AHRS khớp nhau**: roll 1,3°, pitch 1,9°, hướng 3,3° RMS (khoảng 2° trong đó là level trim, AHRS có, accelerometer thô thì không).
- **Vận tốc trừ vận tốc GPS thô, ngay trước mỗi lần fix được dùng**: EKF 1,30 m/s (1,11 với `--acc-noise 1`), firmware 1,23 m/s. Với GPS 1 Hz cả hai đều quanh 1 m/s: tần số GPS là giới hạn, đúng như phân tích ở docs/NAV.md mục 11.
- **Độ cao**: EKF lớn nhất 7,5 m; firmware 0.8.7 trôi tới 290 m (lỗi đã sửa ở 0.8.8).
- **Hướng trong lúc la bàn nhiễu (10–27 s)**: EKF lệch với AHRS tới −9,6°. Log không có hướng thật để biết bên nào đúng.
- Giới hạn của lần chạy lại này:
  - log chỉ có mẫu accelerometer **tức thời** 250 Hz, nên rung động bị gập tần (trên mạch EKF nhận tổng của mọi mẫu 4 kHz);
  - gyro trong log đã lọc và theo quy ước phi công (pitch đảo dấu, công cụ tự đổi lại);
  - toạ độ trong CSV bị làm tròn (11 m), nên chỉ dùng vận tốc GPS.

**Xuất CSV đã sửa ở 0.9.0**: trước đây `"%g"` chỉ giữ 6 chữ số, nên vĩ độ 20.8634 tương đương bước 11 m. Nay mỗi cột có đủ số lẻ theo độ phân giải của nó.

## 5. Bay thử theo thứ tự
1. Để SHADOW, GPS ≥ 5 Hz (0.8.8 tự xin tần số nếu `gps.auto_config` bật; NMEA cần baud ≥ 38400). Bay như mọi khi, có tăng tốc và đổi hướng, gửi log `.wblog` (hoặc CSV xuất từ app 0.9.0).
2. Mình so `ekf_yaw` với `yaw`, `ekf_vn/ve` với vận tốc GPS, `ekf_pn/pe` với `pos_n/pos_e`. Nếu EKF tốt hơn rõ, thử NAV ở độ cao an toàn, chỗ thoáng, sẵn sàng tắt công tắc Position Hold.
