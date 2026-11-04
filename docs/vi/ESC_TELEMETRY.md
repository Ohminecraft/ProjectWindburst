# Telemetry ESC qua dây riêng (0.11.0)

ESC BLHeli_32, AM32, KISS có một chân **T** (telemetry). Nối chân T của mọi ESC chung vào **RX của một UART** trên flight controller. Betaflight gọi tính năng này là "ESC sensor". Code: `src/hal/motor/esc_sensor.h`.

## 1. Cài đặt
1. Tab Motor: giao thức **DShot** (bit hỏi telemetry nằm trong khung DShot; PWM / Oneshot không có), bật **"Dây telemetry ESC (KISS)"**.
2. Tab Chân: nhóm **Telemetry ESC**, ô RX: chân nối với dây T.
3. Cổng nối tiếp:
   - AUTO: firmware lấy UART còn trống sau GPS (UART1), receiver (UART2) và optical flow MSP. Trên ESP32-S3 khi app nối qua USB của chip thường là **UART0**.
   - Bảng: chọn chức năng "Telemetry ESC (dây riêng)" cho một UART.
4. Lưu và khởi động lại. Báo cáo khởi động: `esc tlm: KISS telemetry on UART0 RX GPIO44, 115200 baud`.

## 2. Chạy thế nào
- 115200 baud, chỉ nhận. 5 giây đầu không hỏi (ESC còn khởi động), giống Betaflight.
- Mỗi tick 1 ms của luồng cảm biến: khung DShot kế tiếp tới **một** motor mang bit telemetry. ESC đó trả 10 byte: nhiệt độ (°C), điện áp (0,01 V), dòng (0,01 A), mAh, eRPM / 100, CRC8 (đa thức 0x07). Có câu trả lời thì sang motor sau. Sau 100 ms không trả lời thì bỏ qua (đếm là "không trả lời"). Sai CRC thì bỏ (đếm).
- 4 ESC: một vòng khoảng 4 - 5 ms. Đủ cho:
  - pin: nguồn dòng = ESC, tổng 4 ESC; điện áp lấy giá trị cao nhất;
  - nhiệt độ và dòng từng ESC (bảng "Từng motor" ở tab Motor);
  - RPM để xem.
  **Không** đủ nhanh cho bộ lọc RPM và dynamic idle (cần DShot hai chiều, mỗi khung một lần trả lời). Khi DShot hai chiều đang bật, RPM vẫn lấy từ DShot hai chiều.
- Tab Motor, dòng "Dây telemetry ESC, mỗi giây": số trả lời tốt, hỏng CRC, ESC không trả lời. Bình thường: vài trăm trả lời, 0 hỏng, 0 không trả lời. Một ESC không trả lời làm mất 100 ms mỗi vòng, tức khoảng 10 lần mỗi giây.
- Health `ESC_TELEMETRY`: bật khi đã nhận được câu trả lời tốt trong giây vừa rồi.

## 3. Kiểm chứng trên PC
- `test_motor` nhóm "ESC telemetry wire":
  - CRC8 và giải mã khung;
  - vòng hỏi (chờ 5 s, hết giờ, hỏng CRC, byte lạ lúc ESC khởi động);
  - bit telemetry vào đúng slot phần cứng của motor logic, mỗi lần một khung, cả lúc disarm; Oneshot không có bit.
- `test_fw` nhóm `escwire`: mạch ảo có dây telemetry chung (ESC ảo trả lời khi khung của nó có bit). Kiểm:
  - báo cáo khởi động, UART còn trống;
  - không hỏi trong 5 s đầu;
  - khoảng 780 trả lời mỗi giây;
  - nhiệt độ / dòng từng ESC theo thứ tự motor, dòng pin, RPM khi motor test;
  - một ESC câm: 10 lần không trả lời mỗi giây;
  - khung hỏng;
  - các trường hợp sai: Oneshot, không chân, bảng cổng thiếu, trùng chân.
- 5 mutant (hỏi ngay không chờ, không hết giờ, bit sai slot, câu trả lời không vào đâu, chọn sai UART): đều bị bắt.

**Chưa thử trên mạch.**
