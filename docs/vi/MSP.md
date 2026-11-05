# MSP: thiết bị ngoại vi và công cụ cấu hình (0.44)

Đã kiểm tra trên PC: unit test và firmware chạy trên mạch ảo (`test_fw`). **Chưa thử với thiết bị thật, chưa thử với Betaflight App / SpeedyBee thật.**

MSP (MultiWii Serial Protocol của Betaflight) được dùng ở hai chỗ:
1. **Cổng MSP:** một UART mà các thiết bị MSP tự gửi dữ liệu vào.
2. **Cổng của app:** Betaflight App, app SpeedyBee và các công cụ ESC nói chuyện với firmware qua cổng này.

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

## 2. Công cụ cấu hình trên cổng của app

Section **`msp`** (tab Hệ thống, khối "Công cụ khác qua MSP"):

| Trường | Mặc định | Ý nghĩa |
|---|---|---|
| `identity` | `BETAFLIGHT` | Trả lời như Betaflight master: `BTFL`, phiên bản 2026.12.0, API 1.49. Betaflight App, SpeedyBee và công cụ ESC chỉ làm việc với firmware mà chúng biết. `WINDBURST`: báo `WBRX` và phiên bản thật; các công cụ đó sẽ từ chối. BOARD_INFO luôn mang mã board `WBRX` |
| `writes` | bật | Cho các công cụ đó sửa cấu hình và lưu. Tắt thì chỉ đọc được. Đang arm thì không bao giờ cho sửa |

### 2.1 Kết nối (Betaflight App)

- Chuỗi lệnh lúc App kết nối được trả lời theo cách đóng gói của master: API_VERSION, FC_VARIANT, FC_VERSION (kèm chuỗi phiên bản "2026.12.0"), BUILD_INFO (kèm các build option để App hiện các tab), BOARD_INFO (tần số gyro, lỗi "accelerometer chưa calibrate"), STATUS / STATUS_EX, UID, NAME, MSP2_GET_TEXT, SET_RTC.
- Bộ phát video / kính trên UART của OSD nhận FC_VERSION không kèm chuỗi phiên bản (3 byte, đúng độ dài chúng chờ).
- **Giữ không cho arm:** khi App đang kết nối, nó gửi MSP_SET_ARMING_DISABLED; firmware từ chối arm với lý do `CONFIGURATOR`. Hết giữ khi App gửi 0 (ngắt kết nối), hoặc sau 3 s không có lệnh MSP nào (rút cáp). Nếu đang arm: không cắt gì trên không, chỉ chặn lần arm tiếp theo.

### 2.2 Đọc

- **Trạng thái đang đo:** RC (105: theo thứ tự chức năng - roll, pitch, yaw, ga, rồi AUX1.. - dù thứ tự kênh là gì, như Betaflight), RAW_IMU (102: theo khung của Betaflight x tới trước, y sang trái, z lên trên - nằm phẳng đọc +1 g ở z; acc x2048 / g, gyro x4,1 / độ/s, mag x1090 / G), RAW_GPS (106: kèm pdop / hdop / vdop và số vệ tinh), COMP_GPS (107), ATTITUDE (108: 0,1 độ; pitch dương là chúi mũi), ALTITUDE (109), ANALOG (110), BATTERY_STATE (130).
- **Tab Configuration và Setup:**

| Lệnh | Hiện gì |
|---|---|
| FEATURE_CONFIG (36) | RX_PPM / RX_SERIAL / RX_PARALLEL_PWM / RX_MSP / RX_SPI (nRF24) theo `receiver.protocol`; GPS, RANGEFINDER theo `sensors.*_enabled`; AIRMODE khi `general.airmode` khác OFF; 3D theo `motors.mode3d`; TELEMETRY theo `receiver.telemetry`; LED_STRIP khi `led.count` khác 0; OSD khi `osd.type` khác NONE; ESC_SENSOR theo `motors.esc_sensor` |
| MIXER_CONFIG (42) | khung (`motors.layout`) theo mixer của Betaflight: QUADX, QUADP, HEX6X, HEX6, HEX6H, Y6, Y4, OCTOX8, OCTOFLATP, OCTOFLATX, VTAIL4, ATAIL4, QUADX1234; đảo chiều cánh = `props_out` |
| MOTOR_3D_CONFIG (124) | `deadband3d_low`, `deadband3d_high`, `neutral3d` (us) |
| RC_DEADBAND (125) | `receiver.deadband`, `yaw_deadband`, `deadband3d_throttle` (us; deadband của Position Hold: 0, ở đây không có) |
| RX_CONFIG (44) | receiver serial: SPEKTRUM2048 1, SBUS 2, IBUS 7, CRSF 9, SPEKTRUM1024 15, MAVLINK 16 (0 với PPM / PWM / MSP / nRF24); ngưỡng airmode = `airmode_start_throttle`; RC smoothing bật / tắt và hai hệ số; ngưỡng cần maxcheck / midrc / mincheck = `receiver.max_check`, `mid_rc`, `min_check` |
| ARMING_CONFIG (61) | small angle = `max_arm_angle`; auto disarm 5 s (ở đây không có setting này) |
| BEEPER_CONFIG (184) | `beeper.conditions`, `dshot_beacon_tone`, `motor_conditions` theo cách đánh số beeper của Betaflight. Beeper Betaflight có mà firmware này không có thì hiện là tắt |
| SENSOR_CONFIG (96) | cái gì được cài: accelerometer bật; khí áp kế bật / NONE; la bàn NONE hoặc model theo danh sách của Betaflight (HMC5883, AK8963, QMC5883L, QMC5883P, DroneCAN; còn lại DEFAULT); rangefinder / cảm biến flow bật (255: cảm biến không có trong danh sách của Betaflight) hoặc NONE; pitot NONE |
| MSP2_SENSOR_CONFIG_ACTIVE (0x300A) | cái gì đang chạy: gyro, accelerometer, khí áp kế, la bàn, rangefinder, cảm biến flow (tìm thấy hay NONE) |
| MSP2_GYRO_SENSOR_ACTIVE (0x300D) | hai chỗ gyro: IMU thứ nhất và thứ hai có tìm thấy không |
| PID_ADVANCED (94) | feedforward (F từng trục, transition, averaging, smoothing, boost, max rate limit, jitter), I-term relax (kiểu, cutoff), anti-gravity gain (0 khi `pid.anti_gravity` tắt), D max (từng trục, gain, advance), throttle boost, giới hạn acro trainer, giới hạn góc (`rates.angle_limit`), giới hạn công suất motor, dynamic idle, TPA (mode, rate, breakpoint: % ga gửi thành us, 1000 + % x 10). Setting Betaflight có mà firmware này không có thì là 0 |
| BATTERY_CONFIG (32) | cell min / max / cảnh báo (0,1 V, rồi 0,01 V như profile pin của Betaflight), dung lượng, nguồn đo áp (NONE, ADC, ESC), đo dòng (NONE, ADC, ESC) |
| VOLTAGE_METERS (128), CURRENT_METERS (129) | pin 1 (id 10): điện áp lúc này (0,1 V); mAh đã dùng, dòng (mA; âm: 0) |
| VOLTAGE_METER_CONFIG (56) | một cầu chia ADC: `battery.divider` thành scale / divider value / multiplier của Betaflight (11,00 = 110 / 10 / 1; trên 25,5: scale / 1 / 1) |
| CURRENT_METER_CONFIG (40) | một cảm biến ADC: `current_scale` (0,1 mV mỗi A), `current_offset` (mA) - đúng đơn vị của Betaflight; không có meter ảo |
| FAILSAFE_CONFIG (75) | thời gian chờ `fs_guard_ms` (0,1 s), thời gian hạ cánh `fs_max_descent_s`, ga `fs_throttle` (us), thủ tục theo `nav.fs_procedure` (DROP 0, LAND 1, RESCUE 2; HOLD báo là LAND). Switch mode và throttle-low delay: mặc định của Betaflight |
| RXFAIL_CONFIG (77), RSSI_CONFIG (50) | 16 kênh: cần AUTO, các kênh AUX HOLD (giữ giá trị cuối); không có kênh RSSI |
| GPS_RESCUE (135) | 26 byte: góc (`nav.ph_angle_max`), độ cao bay về, khoảng cách bắt đầu hạ, tốc độ (cm/s), số vệ tinh tối thiểu, tốc độ lên / xuống (cm/s), arm khi chưa fix, kiểu độ cao (MAX, FIXED, CURRENT; CONE báo là MAX), khoảng cách tối thiểu, độ cao leo ban đầu. Ga của autopilot: của Betaflight, ga treo lấy số đã đo |
| SIMPLIFIED_TUNING (140) | thanh trượt PID (mode, master, tỉ lệ D pitch, I, D, PI, D max, FF, PI pitch) và thanh trượt lọc D-term / gyro (bật, hệ số, các tần số cắt) |
| CALCULATE_SIMPLIFIED_PID / GYRO / DTERM (142..144) | thanh trượt App gửi sẽ ra số nào, tính bằng chính code của firmware (`core/simplified_tuning.h`); không đổi gì |
| VALIDATE_SIMPLIFIED_TUNING (145) | gain / tần số cắt gyro / D-term đang lưu có còn đúng là của thanh trượt không |
| OSD_CONFIG (84) | 261 byte của master, gửi thành khung jumbo v1: 32 phần tử ở đúng số thứ tự item của Betaflight (vị trí và hiện / ẩn, đổi từ `osd.pos`; item Betaflight có mà firmware không có: 0), các thống kê firmware hiện (`osd.stats`), timer / cảnh báo / ngưỡng: mặc định của Betaflight |
| OSD_CANVAS (189) | lưới chữ theo `osd.canvas` (AUTO và HD 53 x 20, HDZero 50 x 18, SD 30 x 16) |
| MSP2_COMMON_SERIAL_CONFIG (0x1009), CF_SERIAL_CONFIG (54) | UART0..2 thành cổng 50..52 của Betaflight, không có chức năng (như master: gán qua CLI), 115200 |
| GPS_CONFIG (132) | `gps.provider` (NMEA, UBLOX, MSP, VIRTUAL, DRONECAN - bảng của Betaflight), `sbas`, `auto_config`, `auto_baud`, `set_home_once`, `use_galileo` |
| GPSSVINFO (164) | các vệ tinh module báo: hệ vệ tinh, số hiệu, chất lượng + đang dùng, C/N0 (như Betaflight hiện NAV-SAT của u-blox) |
| FILTER_CONFIG (92) | LPF gyro 1 / 2 (kiểu, tĩnh, động min / max), notch gyro 1 / 2, LPF D-term 1 / 2 (cả expo động), notch D-term, lowpass yaw, dynamic notch (số notch, Q, min / max), bộ lọc RPM (harmonic, Hz tối thiểu, fade range, Q, ba trọng số) |
| ADVANCED_CONFIG (90) | `pid_denom` thật, giao thức motor (DShot1200 hiện là DShot600, DroneCAN là DShot300), `pulse_hz`, idle motor |
| BOARD_ALIGNMENT_CONFIG (38) | cả IMU lắp thế nào: hướng chip `board_align` và `board_trim_*` gộp lại, thành góc của Betaflight theo độ chẵn (roll -179..180, yaw 0..359 theo chiều kim đồng hồ). Chip xoay 90 độ thì đọc yaw 90 |
| SENSOR_ALIGNMENT (126) | gyro / acc CW0 (hướng của chúng nằm trong BOARD_ALIGNMENT), la bàn = `mag_align`, gyro đang dùng = `gyro_use`, góc la bàn = `mag_trim_*` |
| COMPASS_CONFIG (133) | `mag_declination` (0,1 độ) |

- Ngoài ra: NAME (10), MODE_RANGES (34) / MODE_RANGES_EXTRA (238), RX_MAP (64), RC_TUNING (111, kiểu rate **Actual**, 3), PID (112), BOXNAMES (116), PIDNAMES (117), ACC_TRIM (240), BLACKBOX_CONFIG (80), DATAFLASH_SUMMARY (70), SDCARD_SUMMARY (79).
- Lệnh chưa có thì trả `!`; App bỏ qua và chạy tiếp.

### 2.3 Ghi

Lệnh ghi đi qua đúng các bước như khi Save một section trong app: kẹp giá trị, áp luật, áp dụng ngay hoặc báo cần khởi động lại. EEPROM_WRITE (250) lưu; REBOOT (68) trả lời trước rồi mới khởi động lại.

| Lệnh | Đổi gì |
|---|---|
| SET_FEATURE_CONFIG (37) | GPS, rangefinder, airmode (tắt -> OFF; bật khi đang OFF -> AUTO; ALWAYS giữ ALWAYS), 3D (`motors.mode3d`), TELEMETRY, ESC_SENSOR, LED_STRIP (tắt: `led.count` 0; bật khi chưa có dải: 8 LED - đặt độ dài thật trong app Windburst), OSD (tắt: NONE; bật: MSP DisplayPort), loại receiver (PPM, PWM, MSP, nRF24). RX_SERIAL khi receiver đang không phải serial: receiver serial cụ thể do SET_RX_CONFIG gửi sau chọn. Các bit feature khác không lưu |
| SET_RX_CONFIG (45) | receiver serial (chỉ khi receiver là serial; receiver firmware không có, ví dụ SUMD, bị từ chối), ngưỡng cần, ngưỡng ga bật airmode, RC smoothing. Từ chối khi mincheck >= maxcheck (giống Betaflight) |
| SET_ARMING_CONFIG (62) | `max_arm_angle` |
| SET_BEEPER_CONFIG (185) | `beeper.conditions`, `dshot_beacon_tone` (1..5), `motor_conditions`. ARM_REFUSED và MISSION_WARN (Betaflight không có) giữ nguyên |
| SET_ADVANCED_CONFIG (91) | `pid_denom`, giao thức motor (DISABLED bị từ chối), `pulse_hz`, idle motor. Ghi lại đúng giá trị vừa đọc thì không đổi gì (DShot1200 vẫn là DShot1200) |
| SET_BOARD_ALIGNMENT_CONFIG (39) | góc Betaflight của cả cách lắp -> hướng chip (`board_align`) và `board_trim_*` cho cùng phép xoay, ưu tiên trim nhỏ nhất (roll / pitch trong +-45 độ). Không có cách nào ra (board dựng đứng) thì từ chối. Wizard board alignment của App dùng được. Góc ghi lại đúng như đọc thì giữ phần lẻ 0,1 độ |
| SET_MIXER_CONFIG (43) | `motors.layout`, `props_out` (khung có hiệu lực sau khi lưu và khởi động lại). HEX6X là thứ tự motor của Betaflight (HEX_X_BF); ghi lại đúng giá trị vừa đọc thì giữ khung |
| SET_MOTOR_3D_CONFIG (217) | `deadband3d_low`, `deadband3d_high`, `neutral3d` |
| SET_RC_DEADBAND (218) | `receiver.deadband`, `yaw_deadband`, `deadband3d_throttle` (deadband của Position Hold không lưu) |
| SET_SENSOR_ALIGNMENT (220) | `mag_align` (DEFAULT và CUSTOM thì giữ nguyên), `gyro_use`, `mag_trim_*`. Hướng gyro không được ghi (Betaflight master cũng vậy): đặt `board_align` trong app Windburst |
| SET_COMPASS_CONFIG (224) | `mag_declination` |
| SET_NAME (11), MSP2_SET_TEXT | `osd.name` |
| SET_MODE_RANGE (35), SET_RX_MAP (65) | công tắc mode, thứ tự kênh. Công tắc 3D là box 29 "3D DISABLE" của Betaflight |
| SET_RC_TUNING (204) | rate; chỉ nhận rate Actual, không tự đoán cách đổi |
| SET_PID (202) | PID; gõ giá trị khác thì thanh trượt tinh chỉnh tắt (`pid.slider_mode = OFF`), giống App. Ghi lại đúng giá trị vừa đọc thì không đổi gì |
| SET_PID_ADVANCED (95) | những gì PID_ADVANCED hiện (anti-gravity gain 0 là tắt). Gõ F hay D max: thanh trượt PID nhả ra. Giá trị Betaflight có mà firmware này không có thì không lưu |
| SET_FILTER_CONFIG (93) | những gì FILTER_CONFIG hiện. Gõ giá trị lowpass gyro / D-term thì thanh trượt bộ lọc tương ứng tắt (`filters.slider_gyro` / `slider_dterm`) |
| SET_BATTERY_CONFIG (33) | giới hạn cell (0,1 V, hoặc dạng dài 0,01 V; min lớn hơn mức cảnh báo, hay cảnh báo lớn hơn max thì từ chối, như Betaflight), dung lượng, nguồn đo. Đọc rồi ghi lại thì giữ AUTO / DroneCAN; meter dòng ảo hay MSP bị từ chối |
| SET_VOLTAGE_METER_CONFIG (57), SET_CURRENT_METER_CONFIG (41) | `battery.divider` = scale x 100 / (divider value x multiplier); `current_scale`, `current_offset`. Chỉ meter id 10 |
| SET_FAILSAFE_CONFIG (76) | thời gian chờ, thời gian hạ cánh, ga, thủ tục (switch mode, throttle-low delay: không lưu). Đọc rồi ghi lại thì giữ HOLD |
| SET_GPS_RESCUE (225) | những gì GPS_RESCUE hiện; ga autopilot và sanity check không lưu. Đọc rồi ghi lại thì CONE vẫn là CONE |
| SET_SIMPLIFIED_TUNING (141) | các thanh trượt; gain / tần số cắt chúng quản lý đổi theo (như khi kéo trong app Windburst) |
| SET_BLACKBOX_CONFIG (81) | thiết bị (NONE, FLASH, SDCARD; SERIAL bị từ chối), sample rate -> `blackbox.rate_hz`. Đang ghi hay đang xoá thì từ chối (Betaflight) |
| SET_OSD_CONFIG (85) | vị trí / hiện của một phần tử (`osd.pos`; item Betaflight có mà firmware không có: nhận, không lưu); màn hình 0: một thống kê -> `osd.stats`; -1: phần chung (hệ video, đơn vị, ngưỡng, cảnh báo: nhận, không lưu) |
| SET_MOTOR (214) | thử motor ở tab Motors, qua đúng lệnh thử motor của firmware (tab Motor của app Windburst): chỉ khi chưa arm, `msp.writes` bật; motor ở 1000 us là dừng; App ngừng gửi thì motor tự dừng sau 600 ms |
| SET_GPS_CONFIG (223) | những gì GPS_CONFIG hiện; provider hay SBAS ngoài bảng thì bị từ chối. Bản 4 byte giữ nguyên home once / Galileo |
| SET_SENSOR_CONFIG (97) | khí áp kế bật / tắt, la bàn bật / tắt và model, rangefinder bật / tắt, cảm biến flow tắt (bật: chọn cảm biến trong app Windburst). Accelerometer luôn bật. Đọc rồi ghi lại thì giữ model la bàn |


**Chưa có:** wizard của tab Motors; tải font ở tab OSD; VTX; servo; calibrate từ App; MSP_SET_RAW_RC trên cổng của app.

### 2.4 CLI qua MSP, tab Ports

Betaflight App gửi một dòng lệnh giữa STX và ETX trên cổng app và đọc các dòng trả về giữa STX và ETX (tab CLI của nó, và tab Ports: ở API 1.49 tab này chỉ xem, gán cổng qua CLI). Chỉ nhận khi không có khung nào của hai giao thức đang dở; dòng không phải chữ thì bỏ.

| Lệnh | Làm gì |
|---|---|
| `peripherals` | mỗi UART một dòng: `serial UART1: gps*` - UART đó đang phục vụ gì (`msp` ở UART0 khi cổng app nằm ở đó) |
| `get gps_uart` (cả `rx_uart`, `opticalflow_uart`, `esc_sensor_uart`, `osd_uart`, `telemetry_uart`) | `gps_uart = UART1` và `Allowed values: NONE, UART1, UART2` (UART0 chỉ khi cổng app dùng USB của chip) |
| `set gps_uart = UART2` | bảng cổng serial: `ports.assign` TABLE, các gán khác giữ nguyên, cái này chuyển sang (thiết bị đang ở UART đó mất cổng - App cảnh báo trước). Chưa lưu cho tới `save` |
| `save` | lưu rồi khởi động lại (save của Betaflight khởi động lại; App tự kết nối lại) |
| `exit` | khởi động lại, không lưu |
| lệnh khác | CLI của chính firmware (docs/CLI.md): `get`, `set section.field = ...`, `diff`, `dump`, ... |

`opticalflow_uart` là cổng MSP của firmware này (cảm biến flow, GPS / la bàn / khí áp kế / receiver MSP).

## 3. Test
- `test_msp_sensors` (25): khung dựng theo định nghĩa của INAV, không dùng code của driver. Gồm GPS (fix 2D / 3D / RTK, khung bị cắt, tốc độ quá lớn), la bàn, khí áp kế (NaN bị bỏ), RC v1 / v2 (kẹp giá trị, mất sóng sau 300 ms), flow chuyển tiếp, khung hỏng, giới hạn byte mỗi lần đọc.
- `test_msp_config` (219): identity; từng câu trả lời đọc so từng byte với cách đóng gói của Betaflight master; từng lệnh ghi; ghi lại giá trị vừa đọc thì không đổi gì; từ chối khi đang arm / khi `writes` tắt / với giá trị firmware không có.
- `test_esc4way`: câu trả lời của bridge (API 1.49, STATUS, MOTOR_CONFIG, box id, FC_VERSION trên cổng OSD).
- `test_flowsensor`: khung MSP v1.
- `test_fw`, nhóm `msp` (`WB_ONLY=msp`):
  - GPS + la bàn + khí áp kế + receiver MSP trên UART1: arm bằng công tắc của receiver MSP, cất cánh bằng ga của nó với Altitude Hold trên khí áp kế MSP; receiver ngừng gửi thì failsafe;
  - flow + GPS chung một cổng;
  - chưa đặt chân RX thì báo lỗi;
  - công cụ cấu hình trên cổng app: đọc tư thế, đọc và ghi PID (áp dụng ngay vào flight core), lưu, khởi động lại, `identity` / `writes`;
  - chuỗi lệnh kết nối của Betaflight App: mọi lệnh đều được trả lời, không cho arm (`CONFIGURATOR`), hết giữ khi ngắt kết nối và sau 3 s im lặng.
- Mutant: `msp:` (40), `bfapp:` (33), `bfcfg:` (23), `balign:` (8), `bfmix:` (4), `bwiz:` (11), `units:` (13), `bfsens:` (11), `bfpid:` (12), `bffilt:` (11), `bfgps:` (8), `checks:` (12); `bfcfg:` giờ là 31.

## 4. Cần thử trên mạch
1. Betaflight App (2026.12) nối qua USB: kết nối, tab Setup (mô hình quay theo máy), tab Configuration (đổi PID loop divider, airmode, beeper, Save, kết nối lại và kiểm tra), Receiver, Modes, PID Tuning, Rates. Sau đó mở app Windburst xem cùng các giá trị đó.
2. Khi App đang kết nối, thử arm: phải bị từ chối (`CONFIGURATOR`). Ngắt kết nối: arm được.
3. App SpeedyBee (Bluetooth) nếu có module.
4. Một module GPS / la bàn MSP (ví dụ Matek M10Q-5883 ở chế độ MSP) và một receiver MSP nếu có.
