# Altitude Hold / Position Hold viết lại theo ArduPilot (0.29)

Mới kiểm tra trên PC: unit test và firmware chạy trên mạch ảo. **Chưa bay thật.**

## 1. Lý do
Bộ điều khiển trước 0.29 có các điểm sau:
- Chỉ có vòng vị trí P → vận tốc PI. Gia tốc mong muốn được đổi thẳng ra ga qua mô hình ga treo (hover), không có phản hồi gia tốc đo được.
- Cần ga nhảy từ "giữ" sang "leo 2 m/s" thì vận tốc mục tiêu nhảy theo ngay. Sai số vận tốc nhảy 2 m/s, gia tốc yêu cầu nhảy kp_v × 2 = 10 m/s², motor giật.
- Ước lượng ga treo học rất nhanh (0,4 s), cả khi Altitude Hold đang bay. Trong Altitude Hold nó thành thêm một bộ tích phân cãi nhau với tích phân vận tốc, có thể là nguồn dao động chậm lên xuống.

## 2. Bộ điều khiển mới (`src/core/althold.h`, `src/core/kinematics.h`)
Theo `AC_PosControl::D_update_controller` của ArduPilot; lực đẩy được chuẩn hoá theo ga treo như PX4.

1. **Tạo mục tiêu có giới hạn jerk** (port `shape_vel_accel`, `update_pos_vel_accel` từ `AP_Math/control.cpp`):
   - Cần ga cho tốc độ leo mong muốn.
   - Tốc độ mục tiêu tiến tới đó với gia tốc ≤ `althold.accel_z` (2,5 m/s², giống PILOT_ACCEL_Z) và jerk ≤ `althold.jerk_z` (5 m/s³, giống PSC_JERK_D).
   - Vị trí mục tiêu = tích phân của tốc độ mục tiêu.
   - Motor đang bão hoà theo chiều nào thì mục tiêu không chạy tiếp theo chiều đó (limit vector của AP).
2. **Vị trí → vận tốc:** P (`kp_z` 1,2) cộng tốc độ mục tiêu. Mục tiêu không cách máy quá 2 m.
3. **Vận tốc → gia tốc:** P (`kp_v` 5). I mặc định 0, vì tích phân nằm ở vòng gia tốc như AP. Cộng gia tốc mục tiêu làm feed-forward.
4. **Vòng gia tốc đo được (mới):**
   - Sai lệch = gia tốc yêu cầu − gia tốc đo (IMU, hệ trục đất), lọc 20 Hz.
   - Lực đẩy = hover × (1 + (a_yêu_cầu + P·sai_lệch + I) / g) / cos(nghiêng), với P = `althold.acc_p` (0,5) và I = `althold.acc_i` (1,0/s).
   - Tác dụng: gió giật, luồng gió cánh, pin sụt được bù ngay, không phải chờ thành sai số vận tốc. Tích phân I gánh phần ga treo ước lượng sai.
   - Test mô phỏng một luồng gió xuống 3 m/s² trong 1 s: mất độ cao ít hơn rõ rệt so với kiểu cũ (`test_core`, nhóm `[althold 0.28]`).
5. **Over-speed** (`calculate_overspeed_gain` của AP): bật Altitude Hold khi máy đang leo / hạ nhanh hơn giới hạn thì gia tốc và jerk dừng được nhân lên. Máy không trôi thêm vài mét.
6. **Chuyển sang Altitude Hold giữa chuyến bay:**
   - Mục tiêu bắt đầu ở vị trí, tốc độ và gia tốc hiện tại của máy (`D_init_controller`).
   - Tích phân lấy từ ga đang dùng, nên ga không nhảy.
   - Máy đang leo thì dừng êm với gia tốc giới hạn, thay vì bị giật về điểm bật.
7. **Cất cánh:**
   - Vẫn tăng ga từ từ đến khi rời đất như trước.
   - Sau đó leo tới độ cao cất cánh với tốc độ giảm dần theo √(2·a·d), tính cả quãng dừng của bộ tạo mục tiêu, nên không vọt quá độ cao.
8. **Ga treo:**
   - Khi Altitude Hold đang bay: học chậm (3 s), vì ở đây nó chỉ là feed-forward (AP học trong 10 s).
   - Khi bay tay: vẫn học nhanh (0,4 s), vì failsafe hạ cánh không có ước lượng độ cao chỉ có số này.

## 3. Position Hold / GPS Rescue (`src/core/poscontrol.h`)
- Vận tốc mong muốn (Rescue bay về) cũng đi qua bộ tạo mục tiêu 2-D có giới hạn jerk: gia tốc ≤ 2,5 m/s², jerk ≤ 5 m/s³ (`input_vel_accel_NE` của AP).
- Gia tốc mục tiêu được cộng làm feed-forward.
- Vòng vị trí / vận tốc giữ nguyên như AP: sqrt controller, PID có lọc 5 Hz, giới hạn nghiêng.

## 4. Không đổi
Các phần sau không đổi, vì đã theo ArduPilot / EKF và đã có test:
- bộ ước lượng độ cao (Kalman 4 trạng thái: baro / lidar / GPS);
- bộ ước lượng ngang;
- optical flow;
- Position Hold kiểu brake / loiter của ArduCopter.

Muốn sửa các phần này cần log bay thật cho thấy chỗ sai.

## 5. Cài đặt mới (section althold, compat [6])
| Trường | Mặc định | Ý nghĩa |
|---|---|---|
| `accel_z` | 2,5 m/s² | gia tốc tối đa khi đổi tốc độ leo |
| `jerk_z` | 5 m/s³ | jerk tối đa |
| `acc_p` | 0,50 | P vòng gia tốc. 0 = không phản hồi gia tốc |
| `acc_i` | 1,00 /s | I vòng gia tốc |

App: tab Rates, khối "Cần ga trong Altitude Hold".

## 6. Thay đổi cảm giác bay (cố ý)
- Bắt đầu leo / dừng leo mềm hơn: mất khoảng 1 s để đạt 2 m/s.
- Thả cần ở 2 m/s thì máy dừng cao hơn khoảng 1,5 m.
- Rescue tăng tốc tối đa 2,5 m/s².
- Thấy quá mềm thì tăng `accel_z` / `jerk_z`. Thấy dao động lên xuống nhanh thì giảm `acc_p`.

## 7. Cần thử trên mạch
1. Hover Altitude Hold 1 phút.
   - Biên độ lên xuống, so với 0.28.
   - Trong blackbox: tích phân (`integrator`) ổn định, không trôi.
2. Đẩy ga lên hết rồi về giữa: leo / dừng êm, không giật motor.
3. Bay tay rồi bật Altitude Hold khi đang leo: dừng êm, ga không nhảy.
4. Có gió hoặc bay ngang nhanh: độ cao giữ tốt hơn trước.
5. Thử `acc_p` = 0 để so với cách cũ.
