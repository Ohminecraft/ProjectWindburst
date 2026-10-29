# Altitude Hold / Position Hold — viết lại (lộ trình)

Viết lại phần dẫn đường trong `src/core/nav/` (namespace `fc::nav`), chia lớp: **Estimator → Setpoint → Controller → Mode logic**.
Mỗi giai đoạn: build sạch mọi MCU, unit test qua, cập nhật file này, rồi **dừng chờ thử trên mạch**.

## Quyết định (2026-10-24)

| Câu hỏi | Chọn |
|---|---|
| Sửa bản cũ hay viết lại | Viết lại hẳn. Bản cũ (`vertical_est.h`, `horizontal_est.h`, `althold.h`, `poshold.h`, `poscontrol.h`) vẫn chạy cho tới khi bản mới thay được từng phần |
| Chạy ở task nào | Task bay (ổn định trên mọi chip: ESP32-S3, ESP32, STM32), chia tần số còn khoảng 200 Hz. Cảm biến chậm (baro, GPS, flow) vẫn do task cảm biến đẩy vào bằng `on_baro` / `on_gps` / ... |
| Log | Nhóm blackbox mới `NAV_DEBUG`, nằm trong mọi bản ghi nhanh (mặc định 250 Hz, tức là trên 100 Hz) |
| EKF 15 trạng thái (`ekf.h`) | Để nguyên, vẫn chạy song song (shadow). Nav mới **không** dùng nó |

Quy tắc chung:
- Chỉ dùng float, không dùng double; không cấp phát động trong vòng bay.
- Đơn vị SI, hệ NED, gốc tại home.
- Phần toán thuần tách riêng để unit test; mỗi module có `reset()` và xử lý `dt` xấu.
- Tham số nằm trong config, tên có hậu tố đơn vị.
- Khi ALT / POS HOLD tắt, hành vi bay không đổi.
- Nguồn tham khảo: ArduPilot, PX4, Betaflight, INAV. Chỉ học thuật toán, tự viết lại, ghi nguồn ở đầu file.

## Giai đoạn

| | Nội dung | Trạng thái |
|---|---|---|
| P0 | Log nav + cách vẽ đồ thị | xong trên PC (0.34.0) |
| P1 | `AltEstimator` (`nav/alt_estimator.h`, `nav/state_history.h`) | xong trên PC (0.35.0) |
| P2 | ALT HOLD: vòng vận tốc + ước lượng ga treo + máy trạng thái (`nav/alt_control.h`) | xong trên PC (0.35.0) |
| P3 | Vòng vị trí Z, cần ga = tốc độ lên/xuống, khoá độ cao, cất / hạ cánh (`nav/alt_control.h`) | xong trên PC (0.35.0) |
| P4 | `PosEstimator` (`nav/pos_estimator.h`) | xong trên PC (0.35.0) |
| P5 | POS HOLD (`nav/pos_control.h`, `nav/pos_hold.h`) | xong trên PC (0.35.0) |
| P6 | An toàn: xử lý reset, hàng rào (fence), xuống cấp khi mất cảm biến | xong trên PC (0.35.0) |
| P7 | Móc cho RTH / waypoint (`NavRequest`, GPS Rescue chạy trên `PosController` mới) | xong trên PC (0.35.0) |
| P8 | Rescue viết lại theo ArduPilot / PX4: RTL (hình nón PX4, điểm dừng được, đường bay có giới hạn gia tốc, chờ trên home), SmartRTL, LAND; OSD báo số mét còn lại (`rescue.h`, `nav/breadcrumbs.h`, `docs/NAV.md` mục 4) | xong trên PC (0.36.0) |

Người dùng chọn làm liền P1 đến P7 rồi mới thử trên mạch một lần (2026-10-28). **Chưa phần nào được thử trên mạch.**

## P0 — log nav (0.34.0)

`fc::nav::Log` (`src/core/nav/nav_log.h`) được `FlightCore::estimator_step` điền ở mỗi bước ước lượng (500 Hz). Blackbox nhóm `NAV_DEBUG` (bit 15 của `blackbox.fields`, mặc định tắt) ghi các trường sau vào mỗi bản ghi nhanh:

| Trường | Đơn vị | Ý nghĩa |
|---|---|---|
| `nav_acc[0..2]` | m/s² | Gia tốc N, E, D đã bỏ trọng lực. Đứng yên thì cả ba khoảng 0; khi bay lên, D âm |
| `nav_vib` | m/s² | Độ rung: \|a − trung bình 2 Hz\|, lọc 5 Hz (ArduPilot / PX4 cũng đánh giá accelerometer theo cách này) |
| `nav_clip` | mẫu | Số mẫu accelerometer chạm biên (> `acc_clip_g` = 15.5 g) kể từ bước trước |
| `nav_baro` | m | Độ cao baro so với điểm cất cánh (lên +) |
| `nav_gps_pos[0..2]` | m | Vị trí GPS so với home: N, E, D |
| `nav_gps_vel[0..2]` | m/s | Vận tốc GPS theo NED (D = 0 nếu module không đo) |
| `nav_gps_hacc`, `nav_gps_sacc` | m, m/s | Độ chính xác vị trí / vận tốc do module tự báo |
| `nav_gps_sats` | | Số vệ tinh |
| `nav_gps_flags` | bit | bit 0: đã có fix; bit 1: có đo vận tốc dọc; bit 2: được dùng cho XY |
| `nav_gps_seq` | | Tăng 1 ở mỗi fix: chỗ nào nó nhảy là có fix mới, nên xem được tần số GPS thật |

GPS chỉ được đưa vào nav khi đã có home, tức là sau khi arm (giống bản cũ).

Cách xem: bật nhóm "Dữ liệu dẫn đường chi tiết" ở tab Blackbox, ghi một phiên, mở viewer rồi chọn nhóm đồ thị "Dẫn đường chi tiết". Nhóm này có 6 đồ thị: gia tốc NED, rung + clip, baro, vị trí GPS, vận tốc GPS, độ chính xác GPS. Cũng có thể xuất CSV từ viewer.

Kích thước: nhóm này thêm 38 byte vào mỗi bản ghi nhanh. Khi bật mọi nhóm với 8 motor, bản ghi là 148 byte (bộ đệm của recorder là 192 byte). Với tốc độ mặc định 250 Hz, nhóm này thêm khoảng 9.5 kB/s.

### Cần thử trên mạch (P0)

1. Flash 0.34.0. Ở tab Blackbox, bật thêm nhóm **Dữ liệu dẫn đường chi tiết**; rate 250 Hz; trigger ARMED (hoặc ghi tay).
2. **Cầm tay, không lắp cánh**, arm (hoặc ghi tay). Làm lần lượt:
   - đứng yên 10 s;
   - nâng lên khoảng 1 m rồi hạ xuống, 3 lần;
   - nghiêng khoảng 30° qua lại;
   - đi bộ khoảng 10 m về phía bắc rồi quay lại (nếu có GPS, làm ngoài trời).
3. Gửi file log. Những điểm cần xem:
   - Đứng yên: `nav_acc` cả ba trục gần 0 (lệch dưới khoảng 0.3 m/s²). Nếu lệch nhiều thì accelerometer chưa hiệu chỉnh, hoặc trục gắn sai.
   - Khi nâng lên: `nav_acc[2]` âm rồi dương; `nav_baro` tăng khoảng 1 m, nhiễu baro dưới khoảng 0.3 m.
   - Khi nghiêng: `nav_acc` gần như không đổi (trọng lực đã được bỏ đúng).
   - `nav_vib` và `nav_clip` khi đứng yên. Nếu được thì làm thêm một lần **có cánh, buộc chặt máy, ga khoảng 50 %**: `nav_vib` dưới khoảng 3 m/s², `nav_clip` = 0. Nếu vượt thì phải giảm rung trước khi sang P2.
   - GPS: `nav_gps_seq` tăng đều (5 / 10 Hz tuỳ module); khi đi 10 m về bắc, `nav_gps_pos[0]` tăng khoảng 10; xem `nav_gps_hacc` và `nav_gps_sats`.

## P1 - P7 (0.35.0)

Thay hẳn bản cũ. Các file `althold.h`, `vertical_est.h`, `horizontal_est.h`, `poshold.h`, `poscontrol.h` đã bị xoá. Unit test cũ của chúng nay chạy trên module mới:
- test_nav (132 kiểm tra) dùng thẳng `nav::PosController`, `nav::PosHoldMode`, `nav::PosEstimator`;
- test_core (Altitude Hold) qua một adapter nhỏ trong file test;
- test của estimator cũ được viết lại trong `test_nav_est`.

Thứ tự trong `FlightCore`: **Estimator → Setpoint → Controller → Mode logic**. Tất cả chạy trong task bay; estimator ở `est_hz` (mặc định 500 Hz).

| Module | Việc | Điểm mới so với bản cũ |
|---|---|---|
| `state_history.h` | Lưu trạng thái của khoảng 128 bước gần nhất | Đo đạc đến trễ (GPS 150 ms, baro, flow) được so với trạng thái **lúc đo**. Phần hiệu chỉnh áp vào cả hiện tại lẫn quá khứ (PX4 EKF2, INAV) |
| `alt_estimator.h` | Độ cao NED: pos_d, vel_d, bias gia tốc, offset baro | NED; baro / GPS / vận tốc dọc GPS so với trạng thái trễ; innovation và test ratio của từng nguồn; reset được **đếm**, kèm bước nhảy |
| `alt_control.h` | `HoverEstimator`, `AltSetpoint` (cần ga, trần), `AltController` (cascade vị trí → vận tốc → gia tốc đo), `AltHoldMode` (máy trạng thái) | Tách 4 phần; NED; khi estimator reset thì target dời theo, máy không bay theo bước nhảy; có trần độ cao |
| `pos_estimator.h` | N/E: vị trí, vận tốc, bias | GPS so với trạng thái trễ; xử lý **glitch**: GPS bị loại suốt `glitch_reset_s` = 2 s thì đặt lại theo GPS, có đếm (ArduPilot EKF3) |
| `pos_control.h` | Cascade NE của AC_PosControl | Target dời theo khi estimator reset |
| `pos_hold.h` | Cảm giác PosHold của ArduCopter (phanh, loiter, học gió) | Xử lý reset; hàng rào bán kính có tính quãng dừng; đẩy cần về phía home thì phi công lấy lại |
| `nav_math.h` | sqrt controller, góc nghiêng ↔ gia tốc, đổi trục NE ↔ thân, `NavRequest` | |

**Xuống cấp khi mất cảm biến** (giữ như bản cũ, đã có test vòng kín):
- POS HOLD không còn vị trí / vận tốc → báo "unavailable", phi công bay Angle + ALT HOLD;
- ALT HOLD mất độ cao → giữa cần ga = ga treo đo được;
- baro im → GPS thay, không có bước nhảy.

**Hàng rào** (`nav.fence_alt_max`, `nav.fence_radius`; 0 = tắt; tab Vị trí):
- trần: tốc độ leo được giảm để dừng dưới trần 0.5 m; cao hơn trần thì tự hạ;
- bán kính: chỗ nào máy không còn kịp dừng trước vòng tròn thì cần không còn được theo, máy giữ ở mép;
- khi hàng rào đang tác động, bit `FENCE` trong `nav_flags` được bật.

**Móc P7**: `FlightCore::set_nav_request(NavRequest, ms)` nhận một điểm (N, E), có thể kèm độ cao, hoặc một vận tốc. Yêu cầu chỉ được bay khi:
- POS HOLD đang bật;
- vị trí hợp lệ;
- cần roll / pitch ở giữa;
- yêu cầu được gửi lại trong `nav_request_timeout_ms` = 500 ms.

Phi công chạm cần là lấy lại máy. GPS Rescue (RTH) dùng `PosController` và `AltHoldMode` mới.

**Log** (nhóm blackbox `NAV_DEBUG`): ngoài các trường của P0 còn có
- `nav_est_pd`, `nav_est_vd`, `nav_est_bias`, `nav_baro_off`;
- `nav_baro_ratio`, `nav_gps_ratio` (lớn hơn 1 = bị loại);
- `nav_resets[2]` (độ cao, vị trí);
- `nav_est_pos[2]`, `nav_est_vel[2]`, `nav_tgt_pd`, `nav_tgt_vd`, `nav_hover`, `nav_tgt_pos[2]`;
- `nav_ctl`: bit 0 ALT HOLD, 1 POS HOLD, 2 đang giữ, 3 hàng rào.

### Cần thử trên mạch (0.35.0), theo thứ tự, dừng ngay khi có gì lạ

Bật nhóm blackbox "Dữ liệu dẫn đường chi tiết" cho mọi lần bay.

1. **Cầm tay, không cánh** (như P0): `nav_est_pd` phải theo `-nav_baro`; đứng yên thì `nav_est_vd` gần 0 và `nav_baro_ratio` dưới 1.
2. **ALT HOLD, chỗ rộng, gió nhẹ, độ cao thấp:**
   - cất cánh bằng cần ga (máy tự ramp lên khoảng 0.6 m);
   - giữ cần giữa 20 s: độ cao lệch dưới khoảng 0.3 m;
   - đẩy lên rồi thả: dừng mượt, không vọt;
   - hạ cần hết: hạ cánh, motor về idle.
   - Trong log xem `nav_tgt_pd` với `nav_est_pd`, và `nav_hover` (ổn định, đúng mức ga treo).
3. **Trần độ cao:** đặt `nav.fence_alt_max` = 5, ALT HOLD, đẩy cần lên hết: máy dừng khoảng 4.5 m.
4. **POS HOLD (GPS + la bàn), ngoài trời:**
   - bật trong lúc treo: giữ trong khoảng 1.5 m trong 30 s;
   - đẩy pitch rồi thả: phanh, dừng, giữ;
   - có gió thì xem máy nghiêng ngược gió.
   - Trong log xem `nav_gps_ratio` (phần lớn dưới 1), `nav_resets[1]` (nên đứng yên), và `nav_tgt_pos` với `nav_est_pos`.
5. **Hàng rào bán kính:** đặt `nav.fence_radius` = 20, POS HOLD, bay ra xa: máy dừng ở mép khoảng 20 m, app báo hàng rào; đẩy cần về phía home thì bay lại được.
6. **GPS Rescue 0.36** (công tắc trước, chỗ trống, 30 - 50 m, khoảng 10 m cao; `docs/NAV.md` mục 4):
   - **RTL**: leo tới độ cao bay về (cách xa hơn `rescue_min_start_dist` thì lên `rescue_return_alt`), bay thẳng về với mũi theo hướng bay, hạ xuống 10 m trên home, chờ 5 s, hạ cánh, tự disarm. Kính: `RTL ..M` giảm dần, rồi `RTL HOME`, `RTL LAND`.
   - **SmartRTL**: bay hình chữ L (ra bắc rồi sang đông), bật rescue: máy phải về theo chữ L, không cắt chéo.
   - **LAND**: bật rescue: máy hạ ngay tại chỗ, đứng yên (trong khoảng 2 - 3 m) cả khi có gió.
   - Chỉ khi công tắc chạy đúng mới đặt failsafe = Rescue và thử tắt tay điều khiển.
   - Trong log xem `rescue_phase`, `nav_target_n/e` (điểm đích chạy dọc đường bay) với vị trí máy.

Gửi log của từng bước. Nếu ALT HOLD dao động, giảm `althold.acc_p` / `acc_i`. Nếu POS HOLD lắc, giảm `nav.vel_p`.
