# DroneCAN (0.15.0, thêm khí áp kế, pin và ESC trong 0.17.0)

ESP32 (S3 / thường) dùng bộ điều khiển TWAI có sẵn cộng với một bộ thu phát CAN 3,3 V bên ngoài (SN65HVD230, TJA1051T/3 …). Hai chân nối như sau: `pins.can_tx` vào chân D (TXD) của bộ thu phát, `pins.can_rx` lấy từ chân R (RXD). Bus cần điện trở đầu cuối 120 Ω ở hai đầu; phần lớn module GPS CAN đã có sẵn một điện trở.

| Setting (section `can`, cần khởi động lại) | Ý nghĩa |
|---|---|
| `mode` | `OFF` / `DRONECAN` |
| `bitrate` | 125k / 250k / 500k / **1M** (module DroneCAN dùng 1 Mbit/s) |
| `node_id` | node id của mạch, mặc định 10 (giống ArduPilot) |
| `dna_server` | cấp node id cho module nào xin (dynamic node id allocation), mặc định bật |
| `gps.provider = DRONECAN` | GPS lấy fix từ bus (Fix2 + gnss.Auxiliary) |
| `sensors.mag_model = DRONECAN` | la bàn lấy từ bus (MagneticFieldStrength2). Hướng: `mag_align` CW0 nghĩa là mũi tên trên module chỉ về phía trước |
| `sensors.baro_model = DRONECAN` (0.17) | khí áp kế lấy từ bus (air_data.StaticPressure, nhiệt độ từ StaticTemperature). `AUTO` = BMP280 / BMP388 trên I2C như cũ |
| `battery.source = DRONECAN` / `battery.current_source = DRONECAN` (0.17) | điện áp / dòng pin lấy từ module nguồn trên bus (power.BatteryInfo). mAh tính bằng tích phân dòng, như với cảm biến ADC. `AUTO` không bao giờ tự chọn DRONECAN |
| `motors.protocol = DRONECAN` (0.17) | ESC trên bus: esc.RawCommand, giá trị 0 (dừng) … 8191, ESC index = slot motor. Không cần chân motor |

## Cài đặt trong firmware
- `src/hal/can/dronecan.h`: tầng vận chuyển (CAN id, tail byte, chia frame, CRC theo chữ ký kiểu dữ liệu) và mã hoá bit, làm giống libcanard. Không phụ thuộc phần cứng.
- `dronecan_msgs.h`: NodeStatus, GetNodeInfo, dynamic_node_id.Allocation, gnss.Fix2, gnss.Auxiliary, ahrs.MagneticFieldStrength2; từ 0.17 thêm air_data.StaticPressure / StaticTemperature, power.BatteryInfo, esc.RawCommand, esc.Status. Chữ ký và bố cục lấy từ DSDL của pydronecan.
- `dronecan_node.h`: mạch đóng vai một node.
  - Gửi NodeStatus 1 Hz và trả lời GetNodeInfo (tên `org.windburst.fc`).
  - Máy chủ cấp node id: ưu tiên id module xin nếu còn trống, không thì lấy id cao nhất còn trống từ 125 trở xuống. Một module khởi động lại vẫn được đúng id cũ (trong cùng một lần chạy firmware; bảng cấp id không lưu vào flash).
  - Bảng tối đa 8 node, kèm tên (hỏi GetNodeInfo một lần cho mỗi node).
  - GNSS, la bàn, khí áp kế và pin lấy từ node đầu tiên gửi tới. Node đó im 2 s thì chuyển sang node khác.
  - Áp suất ngoài khoảng 1 000 … 120 000 Pa bị bỏ. Nhiệt độ chỉ lấy từ chính node đang cấp áp suất.
  - esc.Status lưu theo ESC index (0 … 7), kèm mặt nạ "ESC nào vừa nói trong 1 s".
  - `send_esc()`: flight task gọi, có transfer id và bộ đếm riêng, nên không tranh chấp với sensor thread đang gọi `update()`. Driver TWAI tự an toàn khi nhiều task cùng gửi.
- `twai_bus.h`: driver TWAI của ESP-IDF, không chặn, tự khôi phục khi bus-off.
- Sensor thread đọc bus mỗi 1 ms. Fix được đưa vào lớp GPS như một nguồn ngoài (giống MSP), field đưa vào la bàn qua `ExternalMag`, áp suất vào `BaroSensor` qua `ExternalBaro` (`hal/baro/external_baro.h`), BatteryInfo vào `BatterySensor::setExternal()`.

### ESC qua CAN (`hal/motor/can_esc.h`)
- `CanEscHw` là một backend `MotorHw` như DShot / PWM. `MotorOutputManager` vẫn lo idle, giới hạn ga, thứ tự motor, test motor. Khác ở thang giá trị: disarm 0, đã arm thì từ 1 + idle × 8190 đến 8191 × giới hạn, không bao giờ 0 khi đang bay.
- Mỗi vòng điều khiển một lần ghi, nhưng chỉ gửi một RawCommand mỗi `round(pid_hz / pulse_hz)` vòng, mặc định khoảng 400 Hz (`motors.pulse_hz` để 0). Bốn ESC = 7 byte = một frame, khoảng 130 µs ở 1 Mbit/s. Tám ESC = ba frame.
- esc.Status (rpm, V, A, °C) vào `feed_can()` theo slot, đổi sang motor logic theo bảng thứ tự. Telemetry còn hiệu lực 500 ms (esc.Status thường chỉ 10 … 50 Hz). Vì vậy: bảng motor hiện rpm, nguồn pin "ESC" vẫn dùng được, **bộ phát hiện motor dừng (stall) chạy được** với ESC CAN. Bộ lọc RPM của gyro thì không (cần DShot hai chiều).
- Không có lệnh DShot: beacon, đổi chiều, bật telemetry mở rộng đều bị từ chối. Beeper bằng motor không có. Chiều quay chỉnh trong cấu hình của ESC.
- Bỏ qua ESC passthrough (BLHeli): `escPin()` trả về "không có chân".
- Đổi Fix2 sang `gps::Solution`:
  - độ chính xác ngang / đứng / vận tốc lấy từ covariance (6 hoặc 36 giá trị);
  - HDOP lấy từ Auxiliary; không có thì dùng PDOP thay;
  - giờ GPS (time of week) lấy từ `gnss_timestamp`.

### Sửa luôn hai lỗi cũ của nguồn GPS "ngoài" (MSP / VIRTUAL dùng chung đường này)
- Lỗi 1: một solution được đưa vào không bao giờ được tính là "mới", nên Position Hold và bộ ước lượng không bao giờ nhận được nó.
- Lỗi 2: sau khi mất tín hiệu, engine nhảy sang "dò baud" và kẹt ở đó mãi.

## App
- Tab Chân: nhóm "CAN bus" (TX / RX).
- Tab Cảm biến, khối "CAN bus (DroneCAN)": các ô setting, trạng thái bus (khung/s, lỗi TX/RX, số lần bus-off, số id đã cấp; từ 0.17 thêm số RawCommand đã gửi / bỏ và ESC nào đang nói), và danh sách node (id, tên, health, uptime, node nào đang cấp GNSS / la bàn / khí áp kế / pin). Khối Module thêm ô "Loại khí áp kế".
- Tab Motor: giao thức "DroneCAN (ESC trên bus CAN)" ẩn các ô xung / DShot, giữ ô Motor idle, có một dòng giải thích. Cột rpm hiện từ esc.Status.
- Tab Nguồn: nguồn đo điện áp / dòng "Module nguồn DroneCAN".
- Stream `can` (2 Hz) thêm `baro_node`, `batt_node`, `esc_mask`, `esc_sent`, `esc_dropped`; health thêm bit `CAN`.

## Giới hạn
- Chưa có: cấu hình tham số của module (param GetSet), cập nhật firmware module, esc.RPMCommand (chỉ RawCommand), ESC index khác slot motor (ESC phải được cấu hình index 0 … n-1 theo slot).
- Chưa có trên STM32 (FDCAN của H5 sẽ làm sau).
- Ngắt của driver TWAI chạy ở core 1, cùng core với vòng bay, mất vài µs mỗi frame. **Chưa đo trên mạch.**
- 0.17: section `sensors` thêm 1 byte (`baro_model`); bản 18 byte (0.13 – 0.16) và 16 byte (trước 0.13) vẫn nạp được. Enum `esc_protocol`, `batt_source`, `curr_source` thêm giá trị DRONECAN ở cuối (số cũ không đổi).
- Settings: section `pins` thêm 2 byte. Pin map của 0.14 (48 byte), 0.11 (47), 0.8 – 0.10 (46), 0.7.1 (37) vẫn nạp được, vì từ bản này mỗi section giữ được bốn kích thước cũ.

## Kiểm thử
- `test_can`:
  - frame do encoder của firmware tạo ra **trùng từng bit** với frame pydronecan tạo (`tools/dronecan_vectors.py` → `test/dronecan_vectors.h`); decoder đọc lại đúng frame của pydronecan;
  - bus bận: hai GPS gửi xen kẽ, mất frame, lật bit (CRC bắt được), transfer bị bỏ dở, hết slot;
  - node trên bus ảo cùng hai module: cấp id, tên, GNSS / la bàn, chuyển sang GPS khác, module khởi động lại vẫn giữ id, hàng đợi truyền đầy.
- `test_fw` nhóm `dronecan`: bảng mạch ảo có module GPS + la bàn DroneCAN.
  - module được cấp id 125; stream `can` hiện đúng;
  - hướng mũi lấy từ la bàn của module; bay Position Hold bằng GPS DroneCAN, giữ trong 0,8 m suốt 10 s;
  - module im lặng rồi nói lại;
  - các trường hợp lỗi: thiếu chân, controller không khởi động, trùng chân, tắt máy chủ cấp id, CAN tắt.
- `test_can` nhóm node thứ hai (0.17): module khí áp + nguồn và bốn ESC; áp suất vô lý bị bỏ, khí áp kế thứ hai chỉ được liệt kê, BatteryInfo cũ 2 s là hết hiệu lực, esc.Status với index ngoài bảng bị bỏ, RawCommand một frame (4 ESC) / ba frame (8 ESC), giá trị trên 8191 bị kẹp, hàng đợi đầy được đếm.
- `test_motor` nhóm `DroneCAN ESCs`: endpoint (idle, giới hạn, không bao giờ 0 khi arm), thứ tự motor, không có lệnh DShot, esc.Status vào đúng motor, timeout 500 ms, nhịp 400 Hz ở vòng 4 kHz, ESC nghe đúng giá trị.
- `test_fw` nhóm `dronecan2`: cả máy trên CAN (GPS + la bàn, khí áp + nguồn, bốn ESC, không có khí áp kế I2C, không có chân motor). Báo cáo khởi động; khoảng 400 RawCommand/s, ESC nghe đủ; bay Altitude Hold bằng khí áp kế CAN; rpm / nhiệt độ / dòng từ esc.Status; dòng và mAh của module nguồn; gãy cánh motor 2 → stall guard disarm, ESC nhận 0; CAN tắt thì motor / khí áp kế / pin đều nói lý do; module nguồn im rồi nói lại.
- Mutant: `python3 tools/mutate.py dcan` (13), cộng 3 mutant của lớp GPS / la bàn; 0.17: `python3 tools/mutate.py can2` (23).
- **Phần TWAI chạy trên chip thật chưa thử được**: mới chỉ biên dịch cho ESP32-S3. ESC CAN thật (Zubax, Vertiq, Holybro Kotleta, AM32 CAN …) chưa thử: cần kiểm ESC index, thang 0 … 8191, ESC tự dừng khi mất lệnh.
