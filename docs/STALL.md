# Disarm khi một motor dừng (0.16.0)

`motors.stall_disarm` (mặc định **tắt**). Khi cánh chạm vật, motor bị kẹt hay ESC mất đồng bộ, firmware tắt hết motor, trước khi các motor còn lại lật máy xuống đất hoặc motor kẹt bị cháy. Cần biết tốc độ của từng motor, lấy từ **DShot hai chiều** hoặc **dây telemetry ESC**. Motor nào không có tốc độ thì không bị xét: thiếu trả lời là lỗi telemetry, không phải motor dừng.

Khi đang arm, mỗi motor có lệnh ≥ `stall_min_cmd` (mặc định 25 %) được xét hai điều kiện:
- **dừng**: tốc độ < `stall_min_rpm` (1500 rpm);
- **tụt**: tốc độ < `stall_drop` (35 %) của tốc độ cao nhất kể từ lần gần nhất lệnh của motor đó giảm. Khi lệnh giảm thì motor chậm lại là chuyện bình thường, nên mốc so sánh được đặt lại.

Một trong hai điều kiện kéo dài `stall_ms` (100 ms) thì disarm:
- blocker `MOTOR_STALL` giữ cho tới khi gạt ARM về OFF;
- stream `motors` cho biết motor nào (`stall_motor`);
- tab Motor có khối "Motor dừng giữa chừng".

Code: `src/core/safety.h` (`StallGuard`), chạy trong `FlightCore` (bước 9b, sau runaway takeoff).

## Kiểm thử
- `test_core`: các quy tắc (glitch 50 ms không tính; tụt khi lệnh không đổi thì bị bắt; lệnh giảm thì không; idle, thiếu telemetry, disarm, tắt thì không xét).
- `test_flight`, quad 5 inch với tốc độ từng motor lấy theo lực đẩy:
  - 6 vòng lật, lăn, xoay yaw, ga max rồi cắt ga: **không lần nào báo nhầm**;
  - motor 3 dừng giữa không: disarm sau 118 ms.
- `test_fw` nhóm `stall`, bảng mạch ảo với DShot hai chiều:
  - đang bay thì motor 2 dừng: disarm trong 250 ms, stream báo motor 2;
  - khi setting tắt: không có gì disarm.
- Mutant: `python3 tools/mutate.py stall:` (8).
- **Chưa thử trên mạch.** Cần thử với cánh thật: chạm nhẹ khi đang hover thấp, và bay mạnh tay để chắc chắn không báo nhầm. Ngưỡng có thể phải chỉnh theo từng máy.
