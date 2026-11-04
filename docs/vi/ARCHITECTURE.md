# Kiến trúc — mục lục tài liệu

| Tài liệu | Nội dung |
|---|---|
| **`docs/HARDWARE_FEEDBACK_1.md`** | lần chạy đầu trên mạch thật (bản 0.5): từng hiện tượng → nguyên nhân → đã sửa gì → còn phải xem gì trên mạch; bảng đổi "Hướng chip IMU"; `IRAM_ATTR` |
| **`docs/HARDWARE_FEEDBACK_2.md`** | log đầu tiên từ mạch thật (bản 0.6.1): vì sao motor lệch nhau khi thử không cánh, cột PID trong log bị kẹp, vòng lặp 83 % → quỹ thời gian của task bay, thời gian từng khâu trong log |
| **`docs/HARDWARE_FEEDBACK_3.md`** | DShot hai chiều với 0.8.2 trên mạch thật: ESC không nhận khung nào, 0 trả lời; giả thuyết dây bị kéo xuống khi nhả chân → 0.8.4 giữ dây cao sau cửa sổ trả lời, đo mức dây, đọc lại khung khi đang chạy |
| `docs/FIRMWARE.md` | `main.cpp`, `FlightApp`, ba luồng (task bay / cảm biến / link), mạch ảo + `wb_sim`, bảng kiểm chứng |
| `docs/PROTOCOL.md` | giao thức app ↔ firmware (WBP): khung, settings, stream, lệnh, quy tắc an toàn |
| `docs/APP.md` | configurator đồ họa (13 tab), thư viện client Python, dòng lệnh |
| `docs/CORE.md` | lõi bay `src/core` |
| **`docs/NAV.md`** | optical flow (PMW3901, cảm biến MSP), ước lượng vị trí, Position Hold, GPS Rescue: phần cứng cần, cài đặt, cách hoạt động, thứ tự thử trên mạch |
| `docs/TUNING.md` | bộ lọc, PID, thanh trượt, Angle / Stabilize, ga treo của Altitude Hold, level trim khi bay, thứ tự tune, điều chưa đo trên mạch |
| `docs/RX.md` | receiver CRSF / SBUS / IBUS |
| `docs/MAVLINK.md` | telemetry MAVLink: gửi gì, cách chọn UART; từ 0.40 lệnh, chế độ, arm và mission từ ground station (`telemetry/mavlink_gcs.h`) |
| `docs/ELRS_MAVLINK.md` | MAVLink qua ELRS: một link ELRS chế độ MAVLink cho cả điều khiển lẫn telemetry (0.39), hoặc cặp ELRS riêng làm radio telemetry; nối dây, cài đặt, điều tiết, băng thông |
| `docs/BOARD_N16R8.md` | mạch ESP32-S3 N16R8: build, chân không dùng được, bộ chân gợi ý, ADC, nối thẻ SD, việc cần thử trên mạch |
| `docs/POWER.md` | đo pin (ADC / ESC), ngưỡng, tiếng báo bằng còi và bằng motor (DShot beacon) |
| `docs/BLACKBOX.md` | log chuyến bay: định dạng `.wblog`, ghi flash / thẻ SD / gửi về app, tab Blackbox, điều phải đo trên mạch |
| `docs/IMU_BRINGUP.md`, `MOTOR_OUTPUT.md`, `BARO.md`, `SONAR.md`, `GPS.md`, `MAG.md` | từng driver |

```
            ┌──────────────── core 1 ─────────────────────┐        ┌──── core 0: task wb_link ────┐
 timer ───► │ task wb_flight (ưu tiên 20, mã trong RAM)   │        │ wbp::Server ◄──► USB / UART0 │ ◄──► app
 IMU ─────► │   FlightApp::flight_cycle → FlightCore → ESC│        │   (qua hàng đợi phát 8 kB)   │
            │        ▲ khoá chung ▼                       │        │   app::LinkHost              │
 RX, GPS,   │ loop() (ưu tiên 1): FlightApp::slow_update  │        │                              │
 baro, mag ►│   publish(Telemetry) ───────────────────────────►   │   snapshot()                 │
            │   service(request)   ◄───────────────────────────   │   call(request)              │
            └─────────────────────────────────────────────┘ app::Exchange └────────────────────────┘
 settings (NVS) ──► wbp::Settings ──► settings_apply.h ──► cấu hình core / motor / GPS / cảm biến
```

Core 0 còn có task `wb_sd` (ưu tiên thấp nhất): nhận byte log từ task link qua bộ đệm vòng và ghi ra thẻ SD; vòng bay không bao giờ chờ thẻ.
