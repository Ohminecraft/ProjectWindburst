# Dải LED WS2812 / SK6812 (0.16.0)

Section `led`:
- `count`: số LED, 0 – 32 (0 = tắt);
- `brightness`: 5 – 100 %;
- `layout`: `ALL` (mọi LED cùng màu), `NAV` (đèn hàng không) hoặc `MANUAL` (tự gán từng LED, từ 0.22).

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

## Kiểu `MANUAL` (0.22)
Mỗi LED (1 – `count`) một chức năng `led.fn[i]` và một màu `led.color[i]` (0xRRGGBB). Ở app: tab **Pin và còi**, khối Đèn LED, chọn kiểu "Tự gán từng LED"; bảng có một dòng cho mỗi LED, nút "Áp cho mọi LED" gán cùng chức năng và màu cho cả dải, và một dải xem thử chạy đúng công thức của firmware (chọn trạng thái xem thử: chưa arm / đang bay / failsafe).

| Chức năng | Hiện gì |
|---|---|
| `OFF` | tắt |
| `STATUS` | hình trạng thái như kiểu `ALL` (bảng trên) |
| `COLOR` | màu của LED, sáng liên tục |
| `MODE` | màu của mode bay (bảng trên), cả khi chưa arm |
| `ARMED` | màu của LED khi đã arm, tắt khi chưa arm |
| `BLINK` | màu của LED, nửa vòng sáng nửa vòng tắt |
| `STROBE` | chớp 60 ms, hai lần mỗi vòng |
| `BREATHE` | sáng dần rồi tối dần (không tắt hẳn, thấp nhất 8 %) |
| `RAINBOW` | cầu vồng chạy; các LED `RAINBOW` chia nhau vòng màu theo thứ tự trên dải |
| `LARSON` | một điểm sáng quét qua lại trên các LED `LARSON` (LED kề bên sáng 20 %) |
| `BATTERY` | đầy: xanh lá, cạn: đỏ, ở giữa thì pha (theo % pin còn lại); không có pin: tắt |
| `THROTTLE` | cần ga thấp: xanh dương, cao: đỏ |

- `led.speed` 1 – 10: một vòng hiệu ứng dài 10 s / speed (5 = 2 s).
- `led.warnings` (mặc định bật): khi failsafe, pin cạn (critical) hoặc motor dừng đột ngột, cả dải hiện hình cảnh báo, bất kể LED gán gì. Tắt đi thì chỉ các LED `STATUS` hiện cảnh báo.
- Mặc định mọi LED là `STATUS` màu trắng, nên chuyển sang `MANUAL` mà chưa gán gì thì giống `ALL`.
- Bản ghi section led cũ (3 byte, 0.16 – 0.21) vẫn đọc được, các trường mới lấy mặc định.

Kiểm thử:
- `test_app` nhóm led strip: các hình theo trạng thái; bit WS2812 được mã hoá rồi giải mã ngược lại để so.
- `test_fw` nhóm `led`: màu theo trạng thái thật của máy (sẵn sàng, Altitude Hold, failsafe); bus bị chiếm; trùng chân.
- `test_app`: từng chức năng MANUAL, cầu vồng / quét theo thời gian, cảnh báo chiếm dải, độ sáng, tốc độ.
- `test_fw` nhóm `led`: MANUAL trên mạch ảo (màu cố định, ga lên thì LED ga đỏ dần, failsafe).
- `test_gui_math`: dải xem thử của app so với firmware ở 288 trường hợp.
- Mutant: `python3 tools/mutate.py led:` (24).
- **Chưa thử với dải LED thật.**
