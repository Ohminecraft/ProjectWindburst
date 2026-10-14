# Danh sách việc (todo.txt của người dùng): trạng thái sau 0.8.8 và đánh giá khả thi

Đối chiếu với mã Betaflight (clone `master` ngày 2026-10-14) và ArduPilot. "Xong" = viết + kiểm chứng trên PC; **chưa có mục nào chạy trên mạch thật**.

| ★ | Việc | Trạng thái | Ghi chú / đánh giá |
|---|---|---|---|
| 5 | ESC passthrough | **xong** | docs/ESC_PASSTHROUGH.md. 4-way + bootloader BLHeli (SiLabs, Atmel, ARM), MSP như Betaflight 4.5, nội dung lấy từ firmware. BLHeliSuite32 chạy được trên mạch (0.9.4) |
| 5 | Bỏ `loop()` của Arduino | **xong** | task `wb_sensor` (core 1, ưu tiên 1, stack 16 kB), `vTaskDelete(NULL)` cuối `setup()` |
| 5 | Firmware thỉnh thoảng không trả lời | **chẩn đoán + sửa một nguyên nhân** | "software restart" là lý do của lần khởi động thật gần nhất, không phải do lỗi này; app trước đây gọi mọi lần stream im là "khởi động lại". Stream `link` (boot_id, số liệu đường truyền), app phân biệt STALLED / REBOOTED; khoá của luồng cảm biến cắt nhỏ. Lần sau gặp: gửi các số ở tab Hệ thống → Đường truyền |
| 5 | Yaw 0 … 360 | **xong** | telemetry `attitude.yaw` và cột `yaw` của blackbox là hướng 0 … 360 (0 = bắc). Bên trong core vẫn −180 … 180 (không ảnh hưởng toán). Log cũ vẫn đọc được (header tự mô tả kiểu) |
| 5 | Position Hold lắc | **phân tích + sửa nguồn** | docs/NAV.md mục 11: GPS 1 Hz là nguyên nhân chính; lệnh tốc độ NMEA, chặn Position Hold khi GPS < 5 Hz, ước lượng độ cao tự đặt lại theo baro. So sánh Betaflight / ArduPilot ở đó |
| 4 | ESC telemetry dây riêng (KISS / BLHeli32) | chưa | khả thi: 115200 baud, chỉ RX, 10 byte mỗi lần hỏi. Cần một UART trống: với app qua USB thì UART0 trống. Đợt sau |
| 4 | Altitude Hold: điểm hover / min / max / vùng chết của cần ga, tốc độ leo / hạ | **xong** | section `althold` (live), tab Rate. "Ga treo" (mức ga máy treo được) vẫn do firmware tự đo: nhập tay thì sai ngay khi đổi pin / cánh. Đường cong ga không áp dụng (đúng như yêu cầu, và vốn đã vậy) |
| 4 | EKF cho Position Hold | **xong ở 0.9.0 (chạy song song)** | `docs/EKF.md`. Lời đánh giá ở bản 0.8.8 ("không làm") là sai một nửa: EKF sửa được hướng mũi khi la bàn nhiễu, thứ log có. Chạy song song + ghi log, chỉ dùng cho Position Hold khi chọn NAV |
| 4 | GPS Rescue đòi fix dù công tắc đang tắt | **giải thích + chữ rõ hơn** | Giống hệt Betaflight (`gps_rescue_allow_arming_without_fix`): công tắc có gán là coi như đã cài rescue. Muốn khỏi chờ: bật "Cho arm khi GPS chưa fix" (tab Vị trí); khi đó chuyến bay đó không có home. Thông báo chặn arm nay nói rõ cách này |
| 3 | Tách tần số PID và tần số đọc gyro | chưa | khả thi (Betaflight `pid_process_denom`). Đi cùng việc đọc SPI trực tiếp thanh ghi |
| 3 | Bỏ driver DShot ESP-IDF; ẩn motor idle với giao thức analog | chưa | khả thi, nhỏ. Đợt sau |
| 3 | Dual gyro / acc | chưa | khả thi (SPI, hai chân CS), trung bình |
| 3 | WiFi | chưa | khả thi trên S3 (cấu hình qua WiFi khi disarm). **Cảnh báo**: radio 2,4 GHz đặt cạnh receiver ELRS 2,4 GHz làm giảm tầm; tốn ~50 kB RAM; ngắt WiFi ở core 0 |
| 3 | DroneCAN | **không làm khi chưa có phần cứng** | ESP32-S3 có bộ TWAI (CAN 2.0), cần thêm transceiver (SN65HVD230…). Ngăn xếp DroneCAN (libcanard + DSDL) lớn; không có thiết bị CAN để thử thì viết ra là đoán mò |
| 3 | Blackbox: bật bằng AUX, mở tab Blackbox là bị cắt rồi ghi phiên mới | **xong** | xem danh sách không cắt phiên; chỉ tải mới cắt; phiên của công tắc chỉ bắt đầu lại sau khi gạt tắt / bật |
| 2 | CLI | chưa | khả thi nhưng app đã có mọi thứ; giá trị thấp |
| 2 | Blackbox > 1 kHz | một phần khả thi | **Flash trong: không** - mỗi trang 256 byte ghi làm đứng cả hai core (~0,5 - 1 ms), 2 kHz × 40 byte = ~300 trang/s là mất 15 - 30 % thời gian bay. **Thẻ SD: được** (task riêng, không đứng core). Đợt sau: cho phép > 1 kHz chỉ khi ghi thẻ SD |
| 2 | Vì sao STM32 Betaflight chạy 8 kHz mà ESP32 dừng ở ~4 - 5,5 kHz | trả lời | xem dưới |
| 2 | Đồ thị rung 3 trục 3 màu | chưa | chỉ phần app, dễ. Đợt sau |
| 2 | Disarm khi cánh / motor bị chạm | chưa | khả thi **chỉ với DShot hai chiều** (cần RPM): so lệnh với RPM thật |
| 2 | LED | chưa | WS2812 cần RMT, mà quad DShot đã dùng cả 4 kênh phát RMT của S3 → phải phát qua SPI (MOSI) hoặc I2S. Khả thi, cần một chân |
| 2 | Hiệu chuẩn la bàn tự động | chưa | khả thi (kiểu học offset khi bay của ArduPilot) |
| 1 | MAVLink, OSD, công tắc telemetry RX, mode mới (acro trainer, xoá blackbox, airmode bằng công tắc, antigravity) | chưa | đều khả thi; OSD analog cần chip MAX7456 trên mạch, OSD HD qua MSP DisplayPort |
| 1 | ESP32 thường | có thể | 2 nhân 240 MHz, 8 kênh RMT; không có USB gốc (chỉ UART qua chip cầu), thiếu chân. Cần một đợt riêng |
| 1 | STM32 (F405, H523/H750/H743, G431/G474) | **đang làm: bản đầu ở 0.10.0** (STM32H563, `docs/STM32.md`; chưa DShot / SD) | Toàn bộ `hal/` viết trên Arduino-ESP32 (RMT, LEDC, gptimer, NVS, FreeRTOS). Port sang STM32 = viết lại một HAL ngang Betaflight, mà trên STM32 thì Betaflight / ArduPilot đã có sẵn, đã bay hàng triệu giờ. Phần `core/` portable có thể mang theo, nhưng phần cứng mới là chỗ tốn công |

## Vì sao Betaflight trên STM32 chạy được 8 kHz
- **DMA làm hết việc vào / ra**: gyro đọc bằng SPI + DMA khi chip báo có mẫu (ngắt EXTI), DShot phát bằng timer + DMA, không tốn CPU. Trên ESP32-S3 bản này: SPI qua driver (vài chục µs mỗi lần đọc), DShot qua RMT có tốn ít CPU để nạp.
- **Không có RTOS, không có khoá**: Betaflight là một vòng lặp với bộ lập lịch riêng. Ở đây có FreeRTOS, hai luồng, một khoá: log 0.8.7 cho thấy task bay chờ khoá trung vị 236 µs - đủ để mất cả một chu kỳ 4 kHz. 0.8.8 cắt nhỏ phần giữ khoá; cần đo lại.
- **Cortex-M7 (F7 / H7) 216 - 480 MHz** có FPU + DSP, mã chạy từ ITCM/flash có ART accelerator, độ trễ ngắt rất nhỏ. Xtensa LX7 240 MHz không chậm hơn nhiều về tính toán, nhưng mã ngoài IRAM chạy qua cache flash.
- **Vì sao "đặt 5000 thì kẹt 4000, trên 5500 thì không lên"**: chu kỳ đo được (~150 µs trung vị, có lúc 300 - 560 µs) cộng thời gian chờ khoá vượt chu kỳ đặt (200 µs ở 5 kHz), task bay theo luật quỹ thời gian bỏ tick → tụt xuống mức chia hết gần nhất.
- **Con đường thực tế** (đợt sau): đọc IMU SPI bằng thanh ghi trực tiếp (bỏ driver, tiết kiệm vài chục µs), tách tần số gyro / PID (gyro 8 kHz + lọc nhẹ, PID 4 kHz như Betaflight `pid_process_denom = 2`). 8 kHz cả gyro lẫn PID trên ESP32-S3 với chuỗi lọc đầy đủ: **không hứa**.

## LacWing_FC (đã đọc)
Mã ESP32 Angle + Alt Hold, ~1800 dòng. Ước lượng độ cao: Kalman 2 trạng thái (z, vz), không có bias gia tốc, không cổng loại số đo, hiệu chỉnh baro ở **mỗi** vòng 500 Hz với số đo cũ (tác giả tự ghi: chỉ hiệu chỉnh khi có mẫu mới thì trôi vô hạn - triệu chứng của bộ lọc thiếu trạng thái bias). Alt Hold: chỉ một vòng PI tốc độ leo (không giữ vị trí độ cao), cần ga giữa = 0 m/s, ±1 m/s, không vùng chết.
Windburst đã có nhiều hơn ở mọi điểm: Kalman 4 trạng thái (bias gia tốc, offset baro), cổng innovation, hiệu ứng mặt đất, hai vòng (độ cao → tốc độ), cất cánh / hạ cánh, ga treo tự đo. Thứ đáng lấy là cấu hình BMP388 (normal mode, ODR 50 Hz, IIR) - bạn đã đưa vào `bmp388_baro.h` (0.8.8 giữ nguyên, thêm kiểm tra cảm biến im lặng, test cập nhật). **Không có gì khác đáng chép.**
