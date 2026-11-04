# Cổng nối tiếp (UART) — section `ports` (firmware 0.8)

ESP32-S3 có ba UART; mỗi UART nối được vào bất kỳ chân GPIO nào, nên **chân** của từng thiết bị vẫn đặt ở nhóm của nó (GPS, receiver nối tiếp, optical flow) trong tab **Chân và cổng**. Section `ports` chỉ quyết định **UART nào** phục vụ thiết bị nào.

- `ports.assign = AUTO` (mặc định, như các bản trước): GPS ở UART1, receiver nối tiếp (CRSF / SBUS / IBUS / Spektrum) ở UART2, cảm biến optical flow MSP lấy cổng còn trống (telemetry MAVLink cũng vậy, sau optical flow và dây telemetry ESC) (UART1 nếu không có GPS, UART2 nếu receiver là PPM / PWM hoặc không có, UART0 khi app nối qua USB của chip). Bảng chức năng **không** được đọc: bản ghi cài đặt cũ có gì trong bảng cũng không đổi gì.
- `ports.assign = TABLE`: `ports.function[0..2]` = việc của UART0, UART1, UART2: `NONE`, `SERIAL_RX`, `GPS`, `FLOW_MSP`, `ESC_SENSOR`, `TELEMETRY` (từ 0.19: telemetry MAVLink, `docs/MAVLINK.md`; telemetry CRSF vẫn đi trên chân TX của receiver), `DISPLAYPORT` (từ 0.21: OSD cho kính digital, `docs/OSD.md`). Một việc chọn cho hai UART → UART đầu tiên. Thiết bị không có UART trong bảng → báo cáo khởi động `gps: FAILED - no UART has the function GPS ...` (tương tự `rx`, `flow`). UART0 chỉ dùng được khi app nối qua cổng USB của chip; trên mạch có cầu USB-UART, UART0 là đường nối app → `ports: WARNING - UART0 carries the app link ...`, thiết bị đó không được UART.
- `ports.baud` không dùng: mỗi thiết bị tự chọn baud (GPS tự dò, receiver theo giao thức, optical flow `flow.baud`). App không còn hiện cột baud.
- Đổi cần lưu và khởi động lại. App kiểm tra trước: việc trùng, thiếu UART cho GPS (khi GPS bật) hoặc cho receiver nối tiếp.

Mã: `app::plan_uarts()` trong `src/app/sync.h` (portable), dùng trong `FlightApp::begin()`.

Kiểm chứng (PC): `test_app` `[uart plan]` (AUTO, bảng, UART0 khi app dùng / không dùng, việc trùng, TELEMETRY); `test_fw` nhóm `ports` (`WB_ONLY=ports`): mạch ảo cắm GPS vào UART2, receiver vào UART1, cảm biến flow MSP vào UART0 (app qua USB) — cả ba chạy; bảng thiếu GPS; GPS đặt vào UART0 khi app dùng UART0; AUTO bỏ qua bảng. Trang thật: `test_serial_ports_table`. **Chưa thử trên mạch thật.**
