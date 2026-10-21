# Telemetry MAVLink tới ground station — section `telemetry` (firmware 0.19)

Gửi trạng thái máy bay cho Mission Planner, QGroundControl, hoặc một radio có MAVLink (radio telemetry 433 / 915 MHz, ELRS backpack). **Chỉ gửi**: firmware chưa đọc lệnh, tham số hay mission từ ground station.

## Cài đặt (tab Chân và cổng, khối "Telemetry (MAVLink)")
| Trường | Mặc định | Ý nghĩa |
|---|---|---|
| `protocol` | `NONE` | `MAVLINK` = bật (MAVLink 2) |
| `baud` | 57600 | tốc độ UART; 57600 cho đa số radio 433 / 915 MHz, 115200 trở lên cho ELRS backpack / dây thẳng |
| `tx_pin` | trống | chân TX của UART, nối vào RX của radio. Trống = không dùng UART (chỉ WiFi) |
| `rx_pin` | trống | chân RX; chưa đọc. STM32 cần nó để khởi động UART |
| `sysid` | 1 | system id MAVLink; hai máy bay trên một ground station cần id khác nhau |
| `wifi` | bật | gửi thêm qua WiFi khi `wifi.mode` bật: UDP broadcast tới cổng 14550 (ground station tự nhận) |
| `rate_attitude` / `rate_position` / `rate_hud` / `rate_status` / `rate_rc` | 10 / 5 / 4 / 2 / 2 | số lần mỗi giây, 0 = không gửi |

Đổi cần lưu và khởi động lại.

**Chọn UART** (`docs/PORTS.md`):
- `ports.assign = AUTO`: firmware lấy UART còn trống sau GPS, receiver nối tiếp, optical flow MSP và dây telemetry ESC, theo thứ tự UART1, UART2, UART0. UART0 chỉ dùng được khi app nối qua USB của chip.
- `TABLE`: firmware dùng UART có chức năng `TELEMETRY`.

Báo cáo khởi động có dòng `telemetry: MAVLink 2, system id 1, UART0 TX GPIO43, 57600 baud + WiFi UDP 14550`. Không chạy được thì có dòng `telemetry: FAILED - ...` kèm lý do. Chân TX được kiểm tra trùng với các chân khác: app báo ngay, firmware chặn arm.

## Gửi gì
| Message | Tần số | Nội dung |
|---|---|---|
| HEARTBEAT | 1 Hz, luôn gửi | loại khung (quad / hexa / octo), armed, mode theo số của ArduCopter (Acro, Stabilize = Angle, Alt Hold, PosHold, RTL = GPS Rescue, Land = failsafe), trạng thái (standby / active / calibrating / critical) |
| ATTITUDE | `rate_attitude` | roll, pitch, yaw (rad, quy ước MAVLink: mũi lên = pitch dương), tốc độ quay |
| GLOBAL_POSITION_INT | `rate_position` | vị trí GPS, độ cao MSL, độ cao so với điểm cất cánh (bộ ước lượng), vận tốc NED, hướng |
| GPS_RAW_INT | `rate_position`, khi có GPS | loại fix, số vệ tinh, HDOP, tốc độ, hướng đi, độ chính xác |
| VFR_HUD | `rate_hud` | tốc độ mặt đất, hướng, % ga, độ cao, tốc độ leo |
| SYS_STATUS | `rate_status` | cảm biến có / khoẻ, % CPU, điện áp, dòng, % pin, số khung receiver hỏng |
| BATTERY_STATUS | `rate_status`, khi đo pin | điện áp từng cell (chia đều), dòng, mAh, % |
| RC_CHANNELS | `rate_rc` | 16 kênh, RSSI |
| STATUSTEXT | khi có chữ | từng dòng của báo cáo khởi động (FAILED = lỗi, WARNING = cảnh báo); "Arming blocked: ..." khi đổi và nhắc lại mỗi 10 s; "Ready to arm"; "Armed" / "Disarmed" |

## Cách chạy
- Thread cảm biến gọi `mav_send()` mỗi lượt (~1 ms). Một khung chỉ được ghi khi UART còn đủ chỗ cho cả khung (`uart_room()`). Không bao giờ cắt khung, không bao giờ chờ; khung chưa vừa sẽ đợi lượt sau và trả lại số thứ tự, nên ground station không thấy khung mất.
- Bị chậm thì không gửi dồn: sau 3 s không có chỗ, mỗi loại gửi tiếp một khung rồi về nhịp thường.
- WiFi: các khung đi qua `mav::Ring` (2 kB, một bên ghi một bên đọc) sang task link. Task này gửi chúng thành datagram UDP broadcast cổng 14550 (`wifi_link.h`).
- Chữ dài hơn 50 ký tự được chia thành nhiều chunk STATUSTEXT cùng `id`. Một dòng tối đa 150 ký tự; hàng đợi giữ 6 dòng, đầy thì bỏ dòng cũ nhất. Mỗi 100 ms gửi một chunk.

Mã: `src/telemetry/mavlink.h` (đóng khung, CRC, 9 message, parser, ring), `src/telemetry/mavlink_telemetry.h` (lịch gửi, chữ), `FlightApp::mav_in()` / `mav_send()` trong `src/app/flight_app.h`.

## Kiểm chứng
- `test_mavlink` (55 kiểm tra): khung so từng byte với pymavlink (`tools/mavlink_vectors.py` → `test/mavlink_vectors.h`, 13 vector), parser, tần số, UART chậm (không cắt khung, số thứ tự liền), không gửi dồn, chia chữ, hàng đợi đầy, HEARTBEAT theo mode.
- `test_fw` nhóm `mavlink`: mạch ảo, giá trị lấy từ mô hình máy bay (tư thế, GPS, độ cao khi bay Alt Hold), báo cáo khởi động thành STATUSTEXT, UART chậm, cài đặt (baud, sysid, tần số 0, hexa), bảng cổng, chỉ WiFi, chân trùng.
- `test_app` (chọn UART), 30 mutant `mav:` trong `tools/mutate.py`.
- App: khối "Telemetry (MAVLink)" ở tab Chân và cổng; `test_gui_browser.py::test_telemetry_block` (chưa chạy: máy này không có playwright).
- **Chưa thử trên mạch thật**, với Mission Planner / QGroundControl, với radio thật, hay qua WiFi.
