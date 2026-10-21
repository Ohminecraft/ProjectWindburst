# ESC passthrough (0.8.8): BLHeliSuite32, ESC Configurator, AM32 Configurator

Giống Betaflight: công cụ cấu hình ESC trên máy tính nói chuyện với ESC **xuyên qua flight controller**, theo dây tín hiệu motor. Không phải tháo ESC, không cần cáp riêng.

## 1. Dùng thế nào

1. Cắm pin (ESC phải có điện). **Tháo cánh** cho chắc.
2. Trong app Windburst, tab Motor, khối "Cấu hình ESC": bấm **"Nhả cổng cho BLHeliSuite32"** (hoặc Disconnect). Một cổng chỉ một chương trình mở được.
3. Mở BLHeliSuite32 / [esc-configurator.com](https://esc-configurator.com) / AM32 Configurator, chọn cổng COM của mạch, bấm Connect rồi Read Setup.
4. Xong thì bấm Disconnect trong công cụ đó: firmware tự khởi động lại, motor chạy bình thường. Công cụ bị đóng ngang mà không Disconnect: sau 5 phút không có byte nào firmware tự khởi động lại (hoặc rút pin / USB).

Không làm được khi đang ARM (firmware trả lời "0 ESC"). Trong lúc passthrough, motor không nhận lệnh nào nữa; có ARM lại cũng không quay cho tới khi khởi động lại.

### Lỗi "Connection to Flightcontroller failed" (0.8.8 - 0.9.3, sửa ở 0.9.4)
Nguyên nhân thật: BLHeliSuite32 32.10 (interface Betaflight passthrough) hỏi firmware các lệnh MSP `1 1 2 3 4 5 90 36 119` trước khi xin passthrough. Firmware chưa biết **90 (MSP_ADVANCED_CONFIG)** và **119 (MSP_BOXIDS)**, trả `$M!`, và công cụ bỏ cuộc. Từ 0.9.4 firmware trả lời cả hai. Thử trên mạch (2026-10-14): chạy được, cả qua USB Serial/JTAG (`rx_s3_usb`).

Lúc đầu mình đoán sai, cho rằng do RTS của cổng USB Serial/JTAG reset chip. Ba bản 0.9.0 - 0.9.2 xoay quanh hướng đó: một bit RTC không có tác dụng, và một bản TinyUSB riêng (`rx_s3_otg`). Tất cả đã bỏ ở 0.9.5, cùng setting `general.usb_rts_reset` và lệnh `ESC_TOOL_MODE`. Dòng `reset: USB` thấy lúc đó nhiều khả năng là từ lần nạp firmware. Thứ tìm ra lỗi là bản ghi chẩn đoán của 0.9.3: nó cho biết công cụ đã gửi lệnh MSP nào. Bản ghi này cũng đã bỏ.

### Các câu trả lời MSP lấy từ chính firmware (0.9.5)
Định dạng giống Betaflight 4.5 (`msp.c`); nội dung là trạng thái thật của firmware:

| Lệnh | Gửi gì |
|---|---|
| API_VERSION, FC_VARIANT, FC_VERSION | 1.46, "BTFL", 4.5.0. **Cố định có chủ ý**: các công cụ chỉ chịu nói chuyện với firmware chúng biết (esp-fc, INAV làm giống vậy). |
| BOARD_INFO | "WBRX", chip (ESP32S3), "WINDBURST" |
| BUILD_INFO | ngày giờ build, phiên bản firmware |
| UID | địa chỉ MAC của chip |
| STATUS, STATUS_EX | chu kỳ vòng điều khiển, cảm biến đang chạy (accelerometer / gyro, baro, la bàn, GPS, cảm biến khoảng cách), mode đang bật, tải CPU, lý do không ARM (đổi sang bit của Betaflight) |
| BOXIDS | ARM, cộng các mode đã gán công tắc trong tab Mode (ANGLE 1, ALTHOLD 3, POS HOLD 11, BEEPER 13, CALIB 17, BLACKBOX 26, FAILSAFE 27, GPS RESCUE 46; STABILIZE không có ở Betaflight) |
| FEATURE_CONFIG | RX_SERIAL / RX_PPM / RX_PARALLEL_PWM theo receiver, GPS, RANGEFINDER, AIRMODE theo setting |
| ADVANCED_CONFIG | giao thức motor, tần số PWM, idle theo `motors.*` |
| MOTOR_CONFIG | min throttle, max throttle, min command, số motor, số cực, DShot hai chiều |
| MOTOR | đầu ra từng motor, 1000 .. 2000 (disarm: 1000) |
| BATTERY_STATE | số cell, điện áp, mAh, dòng |
| MOTOR_3D_CONFIG | giá trị mặc định của Betaflight (firmware không có 3D) |

## 2. Bên trong

```
PC tool ──MSP──▶ app/msp_bridge.h    (cùng cổng với WBP: hai parser cùng nhận mọi byte, mỗi bên bỏ qua khung không phải của mình)
         "$M<" API_VERSION, FC_VARIANT, BOARD_INFO, BUILD_INFO, UID, STATUS, MOTOR_CONFIG, BATTERY_STATE ... → trả lời
         "$M<" SET_PASSTHROUGH ──▶ app::MspLink ──Exchange──▶ FlightApp (REQ_ESC_PASSTHROUGH, luồng cảm biến, có khoá):
                                   từ chối nếu ARM; dừng motor test, phát mức disarm lần cuối, MotorSystem::release()
                       ◀── số ESC
PC tool ──4-way──▶ hal/motor/esc4way.h   (Interface<EspEscPort>, chạy trong task link, core 0)
         0x2F cmd addr n param crc16     → lệnh bootloader BLHeli trên MỘT dây motor: UART bán song công 19200 baud bit-bang
         0x2E cmd addr n param ack crc16 ← (FC và ESC thay nhau kéo dây; giữa các byte dây là input có pull-up)
         cmd_InterfaceExit → ESP.restart()
```

- **Nhận dạng**: các công cụ chỉ nói chuyện với firmware chúng biết, nên phần MSP trả lời giống Betaflight 4.5 (variant `BTFL`, API 1.46, FC version 4.5.0), như esp-fc cũng làm; tên board là `WINDBURST`. Chỉ trả lời yêu cầu **đọc**; `MSP_SET_MOTOR` (quay motor từ công cụ ESC) bị từ chối (`$M!`): quay thử motor thì dùng motor test của app, có kiểm tra an toàn.
- **Bootloader hỗ trợ**: BLHeli của SiLabs / Atmel (BLHeli, BLHeli_S, Bluejay) và ARM (BLHeli_32, AM32), giống `serial_4way.c` / `serial_4way_avrootloader.c` của Betaflight (interface 20.0.06, protocol 108, tên `m4wFCIntf`). **Không có** bootloader STK500v2 "SK" của ESC Atmel đời cũ (Betaflight cũng chỉ có khi bật `USE_SERIAL_4WAY_SK_BOOTLOADER`).
- **Định thời**: một byte trên dây 572 µs, các bit cách nhau 52 µs. Trên mạch, `lock()` là critical section (tắt ngắt core 0). Lúc chờ ESC trả lời, khoá chỉ giữ từng lát 200 µs (một lần xoá trang có thể chờ tới 3 s; giữ khoá liền chừng đó thì watchdog ngắt 300 ms của chip sẽ reset).
- **Chân**: chân motor theo **thứ tự motor của app** (motor 1 = ESC #1 trong công cụ). `gpio_reset_pin()` lấy chân khỏi RMT / LEDC; pull-up trong chip, hoặc không, theo `motors.ext_pullup` (board có điện trở 2k kéo lên ngoài).
- **Sau passthrough luôn khởi động lại**: driver RMT / timer / DShot hai chiều khởi tạo lại từ đầu, đơn giản và chắc hơn trả chân lại cho driver đang chạy (Betaflight thì trả lại).

## 3. Kiểm chứng trên PC

`test/test_esc4way.cpp` (60 kiểm tra): một **bootloader BLHeli_32 mô phỏng ở mức chân**: nó chỉ thấy mức điện trên dây của nó kèm thời gian, tự giải mã như một UART với đồng hồ riêng (lệch −2 % … +3 %), trả lời bằng cách tự kéo dây sau một khoảng nghỉ; có flash 64 kB, trang 1 kB.
- interface: tên, phiên bản, CRC hỏng → `INVALID_CRC`, lệnh lạ, kênh không có, chế độ SK → `INVALID_PARAM`, byte rác trước khung, EXIT, mất link;
- bootloader: kết nối (thông tin `06 1F 63 04` = ARM), đọc 256 / 16 byte, ghi 128 byte, verify đúng / sai, xoá trang 5 (đúng 0x1400 … 0x17FF), keep-alive, chọn ESC khác trên dây khác, reset (lệnh RUN) và reset cứng (giữ dây thấp 300 ms), ESC không trả lời → `GENERAL_ERROR`, ESC lệch 12 % baud → báo lỗi chứ không trả rác; khoá không bao giờ giữ quá một byte;
- MSP: v1 và v2, checksum, nhiễu có `$`, `$M!` cho lệnh lạ và `SET_MOTOR`, `SET_PASSTHROUGH` (không payload / mode 0xFF / 0xFE), ARM → 0.

`test_fw` nhóm `esc` (firmware thật trên mạch ảo): MSP qua `MspLink`, ARM thì từ chối và motor vẫn chạy, disarm thì 4 ESC và **không còn khung motor nào** sau đó, ARM lại cũng không, yêu cầu lần hai → 0, chân của 4 motor; WBP vẫn chạy bình thường khi có byte MSP lẫn vào.

## 3b. Bản ghi phiên (0.18.1)
AM32 Configurator kết nối được nhưng không đọc được ESC (`docs/HARDWARE_FEEDBACK_4.md`, mục 2). Để biết nó dừng ở đâu, mỗi phiên passthrough được ghi lại (`esc4way::Trace`):
- Ghi gì: mỗi lệnh 4-way, kèm ESC / địa chỉ / số byte, câu trả lời, và 4 byte đầu của thông tin thiết bị hoặc dữ liệu đọc được.
- Giữ ở đâu: RAM RTC, nên còn nguyên sau lần khởi động lại khi kết thúc phiên. 12 lệnh đầu và 12 lệnh cuối được giữ.
- Đọc ở đâu: lần khởi động sau, báo cáo khởi động có tối đa 4 dòng `4way:`, ví dụ:

```
4way: 9 requests, 1 ESC errors: NAME ok IFVER ok PROTO ok MODE ok INIT#0 ok 061F6304 READ 7C00/48 ESC-ERR ...
```

Ý nghĩa: `INIT#0 ok 061F6304` = ESC 1 đã vào bootloader (chữ ký 0x1F06, ARM). `READ 7C00/48 ESC-ERR` = đọc 48 byte ở 0x7C00 mà ESC không trả lời đúng.

## 4. Chưa kiểm chứng (cần mạch thật)

- Chưa chạy với BLHeliSuite32 / ESC Configurator thật, chưa với ESC thật. Thứ tự các yêu cầu MSP của từng công cụ lấy theo mã nguồn Betaflight; công cụ nào hỏi thêm lệnh khác sẽ nhận `$M!` (Betaflight cũng trả như vậy với lệnh nó không có) - nếu công cụ dừng ở đó, gửi mình log của công cụ.
- Thời gian thật của `gpio_set_level / gpio_get_level` trong critical section (dự kiến < 1 µs, thừa so với bit 52 µs).
- Mở cổng USB Serial/JTAG bằng công cụ khác: một số công cụ bật DTR/RTS khi mở cổng; trên ESP32-S3 một chuỗi DTR/RTS nhất định làm chip reset vào chế độ nạp. Nếu mạch khởi động lại ngay khi công cụ mở cổng, báo mình.
