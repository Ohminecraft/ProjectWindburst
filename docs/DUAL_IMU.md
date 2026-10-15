# Hai IMU (0.14.0)

Mạch có hai chip IMU, giống `gyro_to_use` của Betaflight. Chip thứ hai nằm **trên cùng bus SPI** với chip đầu (chung SCK / MISO / MOSI) và có chân CS riêng.

| Setting | Ý nghĩa |
|---|---|
| `pins.imu2_cs` | CS của IMU thứ hai (tab Chân, nhóm "IMU thứ hai"). Để 255 nghĩa là không có IMU thứ hai |
| `sensors.gyro_use` | `FIRST` (mặc định): bay bằng IMU 1, IMU 2 dự phòng. `SECOND`: bay bằng gyro của IMU 2. `BOTH`: lấy trung bình hai gyro (nhiễu không tương quan, còn khoảng 1/√2) |
| `sensors.imu2_align` | hướng chip thứ hai trên mạch (Betaflight `gyro_2_sensor_align`). Phần chỉnh tinh của mạch (`board_trim_*`) áp dụng cho cả hai chip |

## Cách chạy (`src/hal/acc_gyro/imu_select.h`, `imusel::Selector`)
- **FIRST**: mỗi chu kỳ đọc IMU 1. IMU 2 được đọc mỗi 8 chu kỳ để biết nó còn sống và để hiệu chuẩn bias gyro của nó. Chu kỳ nào IMU 1 không trả lời thì đọc IMU 2 ngay trong chu kỳ đó.
- **SECOND / BOTH**: đọc cả hai chip mỗi chu kỳ. Thời gian bus SPI tăng gấp đôi (xem "đọc IMU" ở tab Tổng quan).
- **Gia tốc kế** luôn lấy từ IMU 1, là chip được hiệu chuẩn 6 mặt. Gia tốc của IMU 2 chỉ dùng khi IMU 1 không trả lời; lúc đó nó không có hiệu chuẩn riêng (offset 0, scale 1).
- **Chip hỏng**: thiếu một mẫu thì chu kỳ đó lấy từ chip kia. Thiếu 20 mẫu liên tiếp thì chip bị đánh dấu **LOST** và không được đọc lại chừng nào chip kia còn chạy, để tránh nhảy qua lại giữa hai chip lệch nhau. Chỉ khi không còn chip nào thì chip LOST mới được thử lại. Cả hai cùng im 20 chu kỳ thì disarm khẩn cấp, giống khi chỉ có một IMU.
- **Hiệu chuẩn gyro** lúc khởi động và lệnh từ app chạy trên cả hai chip. Arm chỉ cần các chip mà mode đang bay đã hiệu chuẩn xong.
- Tần số gyro: với `SECOND` / `BOTH`, `loop_hz` bị giới hạn theo chip chậm hơn.

## Giới hạn
- Chỉ hỗ trợ SPI dùng chung bus. Không hỗ trợ IMU thứ hai trên I2C hoặc trên bus SPI riêng.
- IMU 1 không có hoặc chạy I2C thì IMU 2 không chạy (báo trong báo cáo khởi động).
- Không so hai gyro để tự phát hiện chip "nói sai" (chip vẫn trả lời nhưng số sai). Chỉ phát hiện chip ngừng trả lời hoặc bị đơ.
- `bus_spi.h` thêm `SPIInitShared()`: thiết bị thứ hai dùng lại `SPIClass` của thiết bị đầu và không gắn lại chân. Trên ESP32, tạo `SPIClass` thứ hai cho cùng host sẽ giành chân của cái đầu.
- Settings: section `pins` thêm 1 byte. Pin map của 0.11 (47 byte), 0.8 – 0.10 (46) và 0.7.1 (37) vẫn nạp được. Pin map của 0.7.0 trở về trước (33 byte) **không còn nạp được** (mỗi section chỉ giữ ba kích thước cũ). Section `sensors` 16 byte cũ nạp được.

## App
- Tab Chân: nhóm "IMU thứ hai" (CS). Có cảnh báo khi IMU 1 không ở SPI.
- Tab Cảm biến: "Dùng gyro nào", "Hướng chip IMU thứ hai", và dòng "IMU thứ hai". Dòng này cho biết trạng thái, chip mà vòng bay đang dùng, và gyro IMU 2 cạnh gyro IMU 1. Xoay máy bằng tay: hai bộ số phải giống nhau, nếu khác là sai hướng chip.
- Stream `sensors`: `gyro2[3]`, `imu2` (NONE / OK / FAILED / LOST), `gyro_src` (IMU1 / IMU2). Health thêm bit `IMU2`.

## Kiểm thử
- `test_accgyro` nhóm "two IMUs": `Selector` với hai chip giả lập (chọn chip, dự phòng, LOST, quay lại).
- `test_fw` nhóm `dualimu` (`WB_ONLY=dualimu`): chip thứ hai xoay 90° trên bảng mạch ảo. Kiểm báo cáo khởi động, bias, IMU 2 khớp IMU 1 khi lăn, IMU 1 chết giữa chuyến bay mà vẫn bay thăng bằng bằng IMU 2, BOTH / SECOND, CS không có chip, CS trùng.
- `test_proto`: kích thước cũ của `pins` / `sensors`. Mutant: `python3 tools/mutate.py imu2` (12 mutant).
