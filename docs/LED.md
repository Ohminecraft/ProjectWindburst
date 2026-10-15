# Dải LED WS2812 / SK6812 (0.16.0)

Section `led`:
- `count`: số LED, 0 – 32 (0 = tắt);
- `brightness`: 5 – 100 %;
- `layout`: `ALL` (mọi LED cùng màu) hoặc `NAV` (đèn hàng không).

Chân dữ liệu là `pins.led` (tab Chân, nhóm Khác).

Firmware gửi dữ liệu LED qua chân MOSI của **bus SPI thứ hai** (HSPI) ở 3,2 MHz. Mỗi bit dữ liệu là 4 bit SPI (`1000` = 0, `1110` = 1), nên thời gian chính xác lấy từ xung SPI chứ không phụ thuộc CPU. Lý do không dùng RMT: cả 4 bộ phát RMT của ESP32-S3 đều đã dành cho DShot khi có 4 motor. Hệ quả là LED **không dùng được cùng lúc với thẻ SD kiểu SPI hoặc PMW3901** (hai thứ này cũng dùng bus đó); khi trùng, báo cáo khởi động sẽ nói rõ. Chưa có trên STM32.

Luồng cảm biến vẽ 50 khung hình mỗi giây (`app/led_logic.h`), gửi ngoài lock. Mỗi LED mất khoảng 30 µs trên bus.

| Trạng thái | Màu |
|---|---|
| failsafe | đỏ, nháy 5 Hz |
| pin cạn (critical) | đỏ, nháy đôi mỗi giây |
| motor dừng / runaway (đã disarm) | đỏ – vàng xen kẽ |
| đang hiệu chuẩn | xanh lơ, sáng mờ dần lên xuống |
| chưa arm, còn điều gì chặn arm | vàng, nháy 1 Hz |
| chưa arm, sẵn sàng | xanh lá |
| đang bay | màu theo mode: Acro trắng, Angle xanh dương, Stabilize xanh lơ, Altitude Hold tím, Position Hold xanh ngọc, GPS Rescue cam nhấp nháy. Pin yếu (warning): thêm một nháy vàng mỗi 2 s |
| `NAV`, đang bay | nửa đầu dải đỏ (trái), nửa sau xanh (phải), LED giữa là màu mode, LED cuối chớp trắng |

Kiểm thử:
- `test_app` nhóm led strip: các hình theo trạng thái; bit WS2812 được mã hoá rồi giải mã ngược lại để so.
- `test_fw` nhóm `led`: màu theo trạng thái thật của máy (sẵn sàng, Altitude Hold, failsafe); bus bị chiếm; trùng chân.
- Mutant: `python3 tools/mutate.py led:` (5).
- **Chưa thử với dải LED thật.**
