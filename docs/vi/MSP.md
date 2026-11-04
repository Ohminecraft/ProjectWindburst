# MSP: thiết bị ngoại vi và công cụ cấu hình khác (0.33)

Đã kiểm tra trên PC: unit test và firmware chạy trên mạch ảo (`test_fw`). **Chưa thử với thiết bị thật, chưa thử với Betaflight Configurator / SpeedyBee thật.**

MSP (MultiWii Serial Protocol của Betaflight) được dùng ở hai chỗ:
1. **Cổng MSP:** một UART mà các thiết bị MSP tự gửi dữ liệu vào.
2. **Cổng của app:** Betaflight Configurator, app SpeedyBee và các công cụ ESC nói chuyện với firmware qua cổng này.

## 1. Cổng MSP: thiết bị ngoại vi

Làm theo ArduPilot (`SERIALn_PROTOCOL = MSP`, `AP_MSP`) và các cảm biến MSP của INAV. Một module có thể mang nhiều cảm biến, ví dụ GPS MSP có la bàn, hoặc MTF-01 có flow và lidar. Vì vậy mọi thiết bị MSP dùng **chung một cổng**, và firmware tự chia từng khung tới đúng chỗ (`src/hal/msp_sensors.h`).

| Khung | Lệnh | Dùng khi | Ghi chú |
|---|---|---|---|
| Lidar | `0x1F01` MSP2_SENSOR_RANGEFINDER | optical flow MSP | như trước |
| Optical flow | `0x1F02` MSP2_SENSOR_OPTIC_FLOW | `flow.type = MSP` | như trước |
| GPS | `0x1F03` MSP2_SENSOR_GPS | `gps.provider = MSP` | 52 byte. Fix type 2 = 2D, từ 3 trở lên = 3D (như INAV / ArduPilot). Độ chính xác cm -> mm, hướng bay 0,01° -> 0,1° |
| La bàn | `0x1F04` MSP2_SENSOR_COMPASS | `sensors.mag_model = MSP` (**mới**) | milligauss, trục trước / phải / dưới của module. Hướng module chỉnh bằng `mag_align` như la bàn DroneCAN |
| Khí áp kế | `0x1F05` MSP2_SENSOR_BARO | `sensors.baro_model = MSP` (**mới**) | Pa (float), 0,01 °C. Áp suất ngoài khoảng 10–120 kPa thì bỏ |
| Receiver | `200` MSP_SET_RAW_RC (v1 hoặc v2) | `receiver.protocol = MSP` (**mới**) | tối đa 18 kênh, giữ 16. Kênh không gửi thì giữ giá trị cũ. Không có khung nào trong 300 ms thì coi là **mất sóng**, failsafe như receiver khác |

- **Chân và tốc độ:** dùng chân `flow_rx` / `flow_tx` và `flow.baud` (mặc định 115200). Chỉ cần RX, nối với TX của thiết bị.
- **Chọn UART:**
  - AUTO: UART còn trống. UART1 nếu GPS không dùng UART (GPS MSP thì không cần UART riêng); UART2 nếu không có receiver nối tiếp; UART0 khi app nối qua USB của chip.
  - TABLE: UART có chức năng `FLOW_MSP`. App giờ gọi chức năng này là "Cổng MSP".
- **Cổng chỉ mở khi có thứ cần nó:** flow MSP, GPS MSP, la bàn MSP, khí áp kế MSP, hoặc receiver MSP.
- **Báo cáo khởi động:**
  - mở được: `msp: MSP port on UART1 RX GPIO44 TX -, 115200 baud, for: GPS compass barometer receiver`;
  - không mở được: `msp: FAILED - <lý do>`, và mục dùng cổng này (gps / rx / flow) cũng báo lý do đó.
- **Khí áp kế MSP:** không chạy khí áp kế thứ hai.
- **Parser** đọc cả khung MSP v1 (`$M<`), vì có thiết bị gửi MSP_SET_RAW_RC bằng v1.

## 2. Công cụ cấu hình khác trên cổng của app

Section mới **`msp`** (tab Hệ thống, khối "Công cụ khác qua MSP"):

| Trường | Mặc định | Ý nghĩa |
|---|---|---|
| `identity` | `BETAFLIGHT` | Trả lời như Betaflight 4.5 (`BTFL`, 4.5.0, API 1.46). Betaflight Configurator, SpeedyBee và công cụ ESC chỉ làm việc với firmware mà chúng biết. `WINDBURST`: báo `WBRX` và phiên bản thật; các công cụ trên sẽ từ chối. BOARD_INFO: mã board `WBRX` khi xưng là Betaflight (Betaflight Configurator hiện nó là tên board), mã chip (ví dụ `_S3_`) khi xưng là Windburst |
| `writes` | bật | Cho các công cụ đó sửa cấu hình và lưu. Tắt thì chỉ đọc được. Đang arm thì không bao giờ cho sửa |

> Bản trước có thay đổi chưa commit làm firmware báo `WBRX` cố định. Giờ việc đó là lựa chọn `identity = WINDBURST`. Mặc định vẫn là Betaflight, để công cụ ESC (ESC passthrough) và Betaflight Configurator dùng được như trước.

**Đọc** (đúng cách đóng gói của Betaflight 4.5, `msp.c`):
- **Trạng thái đang đo:** RC (105), RAW_IMU (102: acc ×512 / g, gyro ×4,1 / °/s, mag ×1090 / G), RAW_GPS (106), COMP_GPS (107), ATTITUDE (108: 0,1°; pitch dương là chúi mũi), ALTITUDE (109), ANALOG (110).
- **Cấu hình:**
  - NAME (10): tên máy bay = `osd.name`;
  - MODE_RANGES (34), MODE_RANGES_EXTRA (238): 20 khoảng; mode STABILIZE không có box trong Betaflight nên không hiện;
  - RX_MAP (64);
  - RC_TUNING (111): rate kiểu **Actual**;
  - PID (112): roll / pitch / yaw, LEVEL = `angle_strength`;
  - BOXNAMES (116), PIDNAMES (117), ACC_TRIM (240).
- Các lệnh cũ (STATUS, BOXIDS, MOTOR, BATTERY_STATE, ...) giữ nguyên. Lệnh chưa có thì trả `!`; Betaflight Configurator bỏ qua và chạy tiếp.

**Ghi** (đi qua đúng các bước của app khi sửa một section: kẹp giá trị, áp luật, áp dụng ngay hoặc báo cần khởi động lại):
- SET_NAME (11), SET_MODE_RANGE (35), SET_RX_MAP (65), SET_RC_TUNING (204), SET_PID (202).
- EEPROM_WRITE (250): lưu.
- REBOOT (68): trả lời trước, rồi mới khởi động lại.
- **Rate:** chỉ nhận rate kiểu Actual. Kiểu khác bị từ chối, không tự đoán cách đổi.
- **PID:** gõ PID khác giá trị cũ thì thanh trượt tinh chỉnh tắt (`pid.slider_mode = OFF`), giống Betaflight Configurator. Ghi lại đúng giá trị vừa đọc thì không đổi gì.
- **MSP_SET_MOTOR vẫn bị từ chối:** thử motor dùng tab Motor của app, vì ở đó có kiểm tra an toàn.

**Chưa có:**
- Các tab Filters, Motors, Ports, Failsafe, OSD, Blackbox của Betaflight Configurator: đọc được một phần hoặc không;
- calibrate từ công cụ đó;
- MSP_SET_RAW_RC trên cổng của app;
- MSP2 (MSP2_GET_TEXT, ...).

## 3. Test
- `test_msp_sensors` (25): khung dựng theo định nghĩa của INAV, không dùng code của driver. Gồm GPS (fix 2D / 3D / RTK, khung bị cắt, tốc độ quá lớn), la bàn, khí áp kế (NaN bị bỏ), RC v1 / v2 (kẹp giá trị, mất sóng sau 300 ms), flow chuyển tiếp, khung hỏng, giới hạn byte mỗi lần đọc.
- `test_msp_config` (42): identity; từng câu trả lời đọc, so từng byte với cách đóng gói của Betaflight 4.5; từng lệnh ghi; từ chối khi đang arm / khi `writes` tắt / khi rate không phải Actual / khi box không có.
- `test_flowsensor`: thêm khung MSP v1.
- `test_fw`, nhóm `msp` (`WB_ONLY=msp`):
  - GPS + la bàn + khí áp kế + receiver MSP trên UART1: arm bằng công tắc của receiver MSP, cất cánh bằng ga của nó với Altitude Hold trên khí áp kế MSP; receiver ngừng gửi thì failsafe;
  - flow + GPS chung một cổng;
  - chưa đặt chân RX thì báo lỗi;
  - Betaflight Configurator trên cổng app: đọc tư thế, đọc và ghi PID (áp dụng ngay vào flight core), lưu, khởi động lại, `identity` / `writes`.
- 40 mutant `msp:`.

## 4. Cần thử trên mạch
1. Betaflight Configurator 10.10 nối qua USB: tab Setup (mô hình quay theo máy), Receiver, Modes, PID Tuning, Rates. Sửa PID rồi Save, sau đó mở lại app Windburst xem giá trị.
2. App SpeedyBee (Bluetooth) nếu có module.
3. Một module GPS / la bàn MSP (ví dụ Matek M10Q-5883 ở chế độ MSP) và một receiver MSP nếu có.
