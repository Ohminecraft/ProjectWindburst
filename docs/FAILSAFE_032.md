# Failsafe viết lại theo PX4 / ArduPilot (0.32)

Đã kiểm tra trên PC: unit test (`test_core`) và firmware chạy trên mạch ảo (`test_fw`). **Chưa bay thật.**

## 1. Cách làm
Theo khung failsafe của PX4 (Commander/failsafe) và các failsafe của ArduCopter. Code ở `src/core/safety.h`.
- **Mỗi tình huống chọn một hành động.** Nhiều tình huống cùng lúc thì làm hành động **nặng nhất**.
- **Không bao giờ nhẹ đi** khi nguyên nhân vẫn còn.
- **Có đường lui:** hành động không làm được thì chuyển sang hành động làm được.

### Tình huống và cài đặt tương ứng
| Tình huống | Cài đặt | Mặc định |
|---|---|---|
| Mất sóng (LINK) | `nav.fs_procedure` | LAND |
| Công tắc Failsafe (SWITCH) | `nav.fs_procedure` | LAND |
| Pin ở mức cảnh báo (BATTERY_LOW, **mới**) | `nav.batt_low_action`: WARN / RTL / LAND (ArduPilot BATT_FS_LOW_ACT, PX4 COM_LOW_BAT_ACT) | WARN |
| Pin ở mức tối thiểu (BATTERY) | `nav.batt_crit_action`: LAND / RTL (ArduPilot BATT_FS_CRT_ACT) | LAND |

`fs_procedure` có thêm hai lựa chọn mới (enum chỉ nối thêm ở cuối):
- **HOLD**: giữ vị trí, xem bảng hành động bên dưới. Thời gian giữ là `nav.fs_hold_s` (mặc định 30 s).
- **DROP**: tắt motor (procedure DROP của Betaflight).

### Hành động (từ nhẹ tới nặng)
| Hành động | Máy làm gì |
|---|---|
| WARN | Không bay gì, chỉ báo (còi / OSD, `fs_cause` = BATTERY_LOW). Giai đoạn vẫn là NONE |
| HOLD | Giữ độ cao bằng Altitude Hold; giữ vị trí bằng Position Hold nếu có vị trí (GPS + la bàn, hoặc optical flow). Cần để giữa. Hết `fs_hold_s` thì chuyển sang LAND. Không có độ cao thì LAND ngay |
| RTL | GPS Rescue bay về rồi hạ cánh. Không bay được (không fix / home / la bàn) thì hạ cánh tại chỗ |
| LAND | Hạ cánh tại chỗ như trước. Với LINK / SWITCH: tự cân bằng, cần để giữa. Với BATTERY: phi công vẫn lái roll / pitch / yaw |
| DROP | Tắt motor, sau thời gian chờ (guard) |

## 2. Các quy tắc giữ nguyên từ trước
- Thời gian chờ (`fs_guard_ms`) áp dụng cho LINK / SWITCH / BATTERY. Các lần mất sóng ngắn được cộng dồn.
- Sóng về phải liền `fs_recover_ms` mới trả máy cho phi công. Nếu Rescue đang bay về, phi công phải đánh cần mới lấy lại máy.
- Hạ cánh vì pin tối thiểu không bị huỷ.
- Lưới thời gian `fs_max_descent_s` chỉ đếm khi **đang hạ cánh** mà không xuống. Khi đang giữ vị trí, hoặc khi Rescue đang bay về, thì không đếm.
- Đứng dưới đất khi hết thời gian chờ thì tắt motor.

## 3. Mới
- **BATTERY_LOW không cần thời gian chờ**, vì bộ theo dõi pin đã có độ trễ riêng: hành động bắt đầu ngay.
- **Phi công lấy lại máy (user takeover của PX4):**
  - Với hành động do pin cảnh báo (RTL / LAND), khi sóng vẫn tốt, phi công đánh cần quá 0,30 là lấy lại máy.
  - Hành động đó không lặp lại cho tới khi pin xuống mức tối thiểu.
  - Pin tối thiểu thì không lấy lại được.
- **Chọn hành động RTL cho pin** thì firmware coi như đã cài Rescue: GPS phải fix mới arm được.
- **Telemetry:**
  - stream `status` có thêm trường `fs_action`;
  - `fs_cause` có thêm bit BATTERY_LOW;
  - thanh trạng thái hiện thêm hành động, ví dụ "Failsafe: đang tự hạ cánh - giữ vị trí (mất sóng)".
- **App:** tab Vị trí có thêm "Thời gian giữ", "Khi pin xuống mức cảnh báo", "Khi pin xuống mức tối thiểu".
- **Settings:** nav compat là [31, 30, 29]. Bản nav 27 byte của các bản rất cũ không còn được nhận.

## 4. Test
- `test_core` nhóm `[failsafe 0.32]`:
  - thứ tự mức nặng;
  - pin cảnh báo với WARN / RTL / LAND;
  - phi công lấy lại máy, và việc đó không lặp lại;
  - chuyển sang LAND khi pin xuống tối thiểu;
  - không nhẹ đi khi mất sóng thêm;
  - RTL không bay được thì hạ cánh tại chỗ;
  - HOLD rồi LAND, HOLD khi không có độ cao;
  - DROP sau thời gian chờ.
- `test_fw` nhóm `[failsafe 0.32]`:
  - mất sóng với HOLD: trong 5 s máy giữ chỗ trong vòng 4 m và giữ độ cao trong vòng 0,8 m, sau 8 s thì hạ cánh và tắt motor;
  - pin cảnh báo với RTL: bay về; phi công đánh cần thì lấy lại máy, và Rescue không tự chạy lại.
- Các test failsafe cũ đều qua mà không phải sửa gì.

## 5. Cần thử trên mạch
1. Công tắc Failsafe với `fs_procedure = HOLD`, có GPS + la bàn: máy đứng yên, hết thời gian giữ thì hạ cánh.
2. `batt_low_action = RTL`: cho pin xuống mức cảnh báo khi đang bay xa. Kiểm tra máy bay về, và đánh cần thì lấy lại được máy.
3. `fs_procedure = DROP`: chỉ thử khi máy ở độ cao an toàn trên nệm, hoặc thử trên bàn, không gắn cánh.
