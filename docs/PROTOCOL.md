# WBP — giao thức app ↔ firmware

Một nguồn duy nhất: **`protocol/wbp_schema.json`**. Từ file này:
- `tools/gen_proto.py` sinh `src/proto/wbp_schema.h` (struct packed, giá trị mặc định, hàm `validate()`, bảng section/stream/command) cho firmware;
- client Python (`app/wbclient`) đọc **chính file json đó lúc chạy**; app JavaScript sau này cũng chỉ cần đọc json này.

Hai bên mang cùng một `hash` (crc32 của json đã bỏ các trường `desc`). App từ chối nói chuyện nếu hash khác nhau, nên không bao giờ đọc nhầm settings của firmware đời khác. Sau khi sửa json: `make proto` (hoặc `python3 tools/gen_proto.py`), build lại firmware.

Sơ đồ tầng:

```
app (UI / CLI)  ──  wbclient (Python)  ──  USB / UART0  ──  hàng đợi phát 8 kB  ──  wbp::Server (task "wb_link", core 0)
                                                                     │  app::LinkHost
                                                                     ▼
                                                              app::Exchange  (seqlock + hộp thư 1 yêu cầu)
                                                                     ▲
                                                              FlightApp (task bay + luồng cảm biến, core 1)
```

## 1. Khung truyền (`src/proto/wbp_codec.h`, `app/wbclient/framing.py`)

```
| 0xA5 | 0x5A | len_lo | len_hi | type | seq | payload (len byte, tối đa 512) | crc_lo | crc_hi |
```
- CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) trên `len, type, seq, payload`. Mọi số nhiều byte là little-endian.
- `seq`: app tăng dần 1..255; câu trả lời mang đúng `seq` của yêu cầu. Khung `STREAM` và `LOG` có bộ đếm riêng.
- **Tự đồng bộ lại**: khung hỏng được quét lại từ byte thứ hai, nên rác trên đường truyền (log khởi động của ESP32 đi chung cổng) hay một header giả không bao giờ che mất khung thật phía sau. Im lặng ~20 ms giữa chừng một khung thì bỏ khung dở (`Server::on_idle()`).

| type | tên | hướng | payload |
|---|---|---|---|
| 0x01 | HELLO | app→fw | — |
| 0x02 | DEVICE_INFO | fw→app | `proto u8, role u8 (0 RX, 1 TX), major, minor, patch, schema_hash u32, features u32, tên…` |
| 0x10 | SECTION_GET | app→fw | `id` |
| 0x11 | SECTION | fw→app | `id, size u16, dữ liệu` |
| 0x12 | SECTION_SET | app→fw | `id, dữ liệu` → trả `SECTION` với giá trị **thực sự đã lưu** (sau khi kẹp miền) |
| 0x13 | SETTINGS_SAVE | app→fw | — → ACK |
| 0x14 | SETTINGS_DEFAULTS | app→fw | `id` hoặc `0xFF` (tất cả) → ACK |
| 0x15 | REBOOT | app→fw | — → ACK, ~300 ms sau firmware khởi động lại |
| 0x20 | STREAM_CFG | app→fw | `id, rate_hz u16` (0 = tắt, id 0xFF = tắt hết) → ACK |
| 0x21 | STREAM | fw→app | `id, t_ms u32, struct của stream` |
| 0x30 | COMMAND | app→fw | `cmd, tham số…` |
| 0x31 | CMD_RESULT | fw→app | `cmd, err, kết quả…` |
| 0x40 | LOG | fw→app | `level, text` |
| 0x7F | ACK | fw→app | `type được trả lời, seq, err` |

Mã lỗi (`errors` trong json): `OK, UNKNOWN, BAD_ARG, ARMED, BUSY, NOT_SUPPORTED, BAD_SECTION, SIZE_MISMATCH, STORAGE, NO_SENSOR, BAD_STATE, FAILED`.

## 2. Settings

17 section, mỗi section là một struct packed (`S_<tên>`). Trường có `scale` thì `giá trị thật = số trên dây × scale`; `enum` là chỉ số trong danh sách tên; `optional` + 255 = ô trống.

| id | section | byte | áp dụng | nội dung |
|---|---|---|---|---|
| 1 | general | 22 | reboot | tần số vòng lặp, board alignment + trim, failsafe (kể cả `fs_fallback_throttle`: % lực treo khi không có độ cao), airmode + ngưỡng ga bắt đầu airmode |
| 2 | receiver | 12 | reboot | giao thức RX, thứ tự kênh, deadband, timeout, RC smoothing (bật, hệ số cho cần và cho ga) |
| 3 | modes | 96 | **live** | 16 ô: mode / aux / dải µs (như Betaflight) |
| 4 | rates | 19 | **live** | Actual rates, throttle mid/expo/limit, giới hạn góc của Angle / Stabilize |
| 5 | pid | 75 | **live** | thanh trượt tune (`slider_*`), P I D Dmax F từng trục, I-term relax, anti-gravity, D max, TPA, feedforward, throttle boost, Angle, Stabilize; xem `docs/TUNING.md` |
| 6 | motors | 35 | reboot | layout, giao thức ESC, đảo chiều, thứ tự motor, idle, dynamic idle |
| 7 | sensors | 16 | reboot | bật/tắt baro, la bàn, sonar, GPS; model + alignment la bàn; độ lệch từ |
| 8 | gps | 17 | reboot | provider, baud, tần số, SBAS, model, home |
| 9 | beeper | 14 | **live** | tình huống nào thì còi kêu và tình huống nào thì **motor** kêu (hai trường bit theo enum `beeper_cond`), loại còi, tần số, âm lượng, âm beacon của ESC; xem `docs/POWER.md` |
| 10 | ports | 15 | reboot | (chỉ lưu — firmware hiện dùng cố định UART1 = GPS, UART2 = RX; cảm biến optical flow qua UART lấy cổng còn trống, xem `docs/NAV.md`) |
| 11 | pins | 33 | reboot | chân IMU SPI, I2C, GPS, RX, motor, ADC pin, còi, **thẻ SD**, **optical flow** (`flow_cs / sck / miso / mosi`, `flow_rx / tx`)… nạp vào `HardwareConfig` lúc khởi động; 255 = dùng mặc định trong `hardware_config.h` |
| 12 | calibration | 82 | **live** | hiệu chuẩn la bàn, accelerometer (offset/scale), level trim |
| 13 | battery | 13 | **live** | nguồn đo, số cell, ngưỡng đầy / cảnh báo / tối thiểu, thời gian chờ, hạ cánh khi nguy hiểm, hệ số chia ADC |
| 15 | filters | 58 | **live** | hai thanh trượt lọc, gyro lowpass 1 (tĩnh / động) và 2, hai notch tĩnh, dynamic notch, RPM filter, lọc D-term, yaw lowpass |
| 14 | blackbox | 10 | **live** | nơi ghi (`NONE` / `FLASH` / `SDCARD`), điều kiện ghi, tốc độ, nhóm trường, kiểu nối và tốc độ thẻ SD; xem `docs/BLACKBOX.md` |
| 16 | flow | 5 | reboot | cảm biến optical flow: loại (`NONE` / `PMW3901` / `MSP`), baud của cảm biến UART |
| 17 | nav | 27 | **live** | hướng lắp / lật gương / hệ số / ngưỡng chất lượng / độ cao tối đa của flow, chỉnh góc nghiêng theo gia tốc đo được, Position Hold (góc tối đa, phanh, hệ số bộ điều khiển vị trí), thủ tục failsafe (`LAND` / `RESCUE`), GPS Rescue; xem `docs/NAV.md` |

Quy tắc phía firmware (`wbp_server.h`, `settings_store.h`):
- Thay đổi nằm trong **RAM** cho tới `SETTINGS_SAVE`. Cờ trong stream `status.settings_dirty`:
  - `DIRTY` (chưa lưu): có section khác với bản trong flash.
  - `REBOOT_REQUIRED`: có section không live khác với bản firmware dùng lúc khởi động.

  Firmware so từng section bằng một mã băm 32 bit (FNV-1a). Mốc là lúc cài đặt vừa nạp xong (`Server::settings_loaded()`), sau đó cập nhật mỗi lần ghi flash. Vì vậy **đổi một giá trị rồi đặt lại như cũ thì cả hai cờ đều tắt** (từ 0.37.1; trước đó cờ giữ nguyên tới lần lưu sau). Lưu một section cần khởi động lại xong rồi mới đặt nó về như cũ thì chỉ còn `DIRTY`: flash đang giữ giá trị mới, còn firmware vẫn chạy giá trị cũ. Một lần ghi flash thất bại thì `DIRTY` bật cho tới khi lưu thành công, vì không biết flash còn giữ gì.
- Section **live** được áp vào firmware đang chạy ngay (PID, bộ lọc, rates, modes, calibration); section còn lại có hiệu lực sau save + reboot.
- **Quan hệ giữa các field** (`settings_rules.h`), áp sau `validate()` mỗi khi một section được app ghi hoặc được nạp từ flash: trục nào đang theo thanh trượt thì P/I/D/Dmax/F của trục đó được **tính lại** từ thanh trượt; thanh trượt lọc đang bật thì các tần số cắt lowpass được tính lại. Câu trả lời của SECTION_SET mang giá trị đã tính, nên app luôn hiện đúng thứ đang có hiệu lực.
- **Firmware tự ghi một section**: level trim khi bay đổi section `calibration` rồi yêu cầu link task ghi riêng section đó vào flash (chỉ khi đã disarm; ghi flash làm vòng lặp đứng vài ms). Thành công: LOG `settings: section calibration written by the firmware` (app đọc lại section và báo); flash từ chối: cờ `DIRTY`.
- **Đang arm thì từ chối mọi thay đổi**: SECTION_SET, SAVE, DEFAULTS, REBOOT và hầu hết command trả `ERR_ARMED`. Đọc thì luôn được.
- Mọi giá trị nhận vào đều qua `validate()`: kẹp miền, enum lạ, **NaN → giá trị mặc định** (app lỗi không thể đưa NaN vào PID).
- Flash: mỗi section một bản ghi NVS `wbsNN = [format][size][dữ liệu][crc16]`. Bản ghi thiếu / sai kích thước (firmware mới đổi section đó) / sai CRC thì **chỉ section đó** về mặc định, phần còn lại giữ nguyên. Ngoại lệ: section chỉ **thêm field ở cuối** (hoặc bị cắt ở cuối) so với firmware cũ thì khai báo `"compat": <kích thước cũ>` trong schema — bản ghi có đúng kích thước đó vẫn được nạp, field mới giữ mặc định (ví dụ `pins` 23 byte của bản 0.4.0 trước khi có các chân thẻ SD: không phải gõ lại sơ đồ chân sau khi nạp firmware mới). Dòng `settings:` của báo cáo khởi động ghi mask những section được nạp theo cách này; lần lưu kế tiếp ghi lại theo kích thước mới. `compat` không nằm trong schema hash.
  Nâng từ 0.5.0 lên 0.6.0: `receiver`, `rates`, `motors` (thêm `dshot_driver` ở cuối), `pins`, `blackbox` được giữ nhờ `compat`, các section không đổi kích thước giữ nguyên (kể cả hiệu chuẩn gia tốc kế và level trim); **`general` và `pid` về mặc định** (bố cục đổi ở giữa: lọc gyro chuyển sang section `filters` mới; `general` thêm `airmode`, `runaway_takeoff`) — nhập lại board alignment / trim và các tuỳ chọn failsafe. Sao lưu (tab Hệ thống) trước khi nạp thì khôi phục được phần còn khớp tên.
  Nâng lên 0.8.0 (từ 0.7.x hoặc 0.6): `pins` dài thêm `servo[4]` (0.7.1), `curr_adc`, `rx_pwm[8]` ở cuối và được đọc tiếp từ bản ghi 37 byte (0.7.1), 33 byte (0.7.0) và 27 byte (0.6) — `compat` nay nhận tới **ba** kích thước cũ; bản ghi 23 byte của 0.5 về mặc định. `battery` thêm 4 field đo dòng (`compat` 13), `ports` thêm `assign` (`compat` 15; mặc định AUTO nên bảng cũ không đổi gì). `servo` (0.7.1) là section mới. Enum `rx_protocol` thêm `SPEKTRUM1024`, `port_function` thêm `FLOW_MSP` (thêm ở cuối: giá trị cũ giữ nghĩa). App và firmware phải cùng bản (schema hash đổi).
  Nâng từ 0.6 lên 0.7.0: `pins` dài thêm 6 chân ở cuối và được đọc tiếp (cả bản ghi 27 byte của 0.6 lẫn 23 byte của 0.5: bảng `compat` nay có hai kích thước cũ); `flow` và `nav` là section mới (mặc định); section `blackbox` giữ nguyên giá trị cũ nên **chưa có nhóm log `NAV`** (tích ở tab Blackbox). Mode trong section `modes` có số thứ tự mà firmware không biết thì ô đó thành trống (trước đây bị kẹp về mode cuối cùng).
  **Ý nghĩa của enum `align` đã đổi** (nay đúng nghĩa Betaflight): `general.board_align` phải nhập lại theo bảng ở `docs/HARDWARE_FEEDBACK_1.md` mục 8; `sensors.mag_align` giữ con số cũ nhưng mang nghĩa mới.

## 3. Telemetry stream

App đăng ký từng stream với tần số mong muốn; firmware kẹp theo `max`. Mỗi tick gửi tối đa 4 khung, không bao giờ "dồn bù"; khung không vừa bộ đệm TX thì **bỏ**, không chờ (app treo không làm chậm firmware).

| id | stream | mặc định / tối đa (Hz) | nội dung |
|---|---|---|---|
| 1 | attitude | 50 / 200 | roll, pitch (quy ước phi công), **yaw là hướng 0 … 360° (0 = bắc, theo chiều kim đồng hồ; từ 0.8.8, u16; trước đó −180 … 180)**, gyro đã lọc, setpoint |
| 2 | rc | 20 / 100 | 16 kênh µs, link, LQ, số khung/giây |
| 3 | status | 10 / 50 | armed, **arm blockers** (bit có tên; mới: `LOOP_RATE`, `RUNAWAY_TAKEOFF`, `RESCUE_NO_FIX`, `RESCUE_NO_COMPASS`), failsafe, modes, health, AltHold state, thời gian vòng lặp, pin/cell, cờ settings, throttle, % CPU, % RAM, RAM / PSRAM còn trống (kB), **thời gian trung bình từng khâu của chu kỳ điều khiển trong giây vừa rồi `t_imu / t_core / t_motor / t_rest` (µs), lượt dài nhất của luồng cảm biến `t_slow_max`, số chu kỳ bỏ lỡ `overruns`, tần số đặt `loop_target`, nguyên nhân failsafe `fs_cause` (bit LINK / SWITCH / BATTERY)** |
| 4 | motors | 20 / 100 | lệnh thực gửi ESC (kể cả khi motor test), rpm, lỗi telemetry, nhiệt, điện áp, dynamic idle, **`bidir` (OFF / ACTIVE / UNAVAILABLE), `answers` (bit i = ESC của motor i trả lời), `skipped`, `late` (khung bỏ qua / lượt trả lời lỡ trong giây vừa rồi)** |
| 5 | sensors | 50 / 200 | gyro thô, accel, la bàn, baro, nhiệt độ, sonar |
| 6 | gps | 5 / 25 | fix, vệ tinh, lat/lon/alt, tốc độ, hướng, HDOP, khoảng cách/hướng về home, trạng thái driver, **`rate` số lời giải mỗi giây đo được (0.8.8)** |
| 7 | estimate | 20 / 100 | độ cao, vận tốc đứng, sigma, vị trí/vận tốc ngang, cờ hợp lệ (`ALT`, `POS`, `YAW_FROM_MAG`, **`VEL`, `FLOW`, `TILT_AIDED`**, 0.8.3: **`GPS_ALT`, `GPS_VZ`**), 0.8.8: **`z_resets`** (số lần độ cao bị đặt lại theo baro) |
| 8 | gyro_fast | 0 / 1000 | gyro thô + đã lọc cho đồ thị rung |
| 9 | calibration | 10 / 20 | loại, tiến độ %, trạng thái, mặt nạ 6 mặt (accel), giá trị thô, **% của lần đo level trim khi bay** (100 = đã đo xong, chờ disarm) |
| 10 | battery | 2 / 20 | điện áp, mỗi cell, số cell, trạng thái, nguồn đo, mV ở chân ADC, âm còi đang kêu, âm motor đang phát |
| 11 | blackbox | 2 / 10 | trạng thái, dung lượng, đã dùng (byte, bão hòa ở 4 GB), số phiên, tốc độ thật, byte mỗi bản ghi, số bản ghi bị bỏ, % xoá, byte/giây, **thiết bị, dung lượng / đã dùng theo kB (thẻ lớn), mức đầy bộ đệm thẻ (%)** |
| 12 | tuning | 0 / 50 | trạng thái đang chạy của bộ lọc và PID: tần số cắt gyro / D-term lowpass 1 lúc này, 9 dynamic notch (3 mỗi trục), RPM notch hài 1 của từng motor, TPA %, anti-gravity, tốc độ khung receiver đo được và tần số cắt RC smoothing, airmode, PID sum từng trục, **ga treo đo được** |
| 13 | nav | 10 / 50 | cờ (`POSHOLD`, `POSHOLD_UNAVAILABLE`, `RESCUE`, `RESCUE_AVAILABLE`, `NAV_LEAN`, 0.8.8: `POSHOLD_SLOW_GPS`), flow: chất lượng, lý do bỏ mẫu (`OK` … `NO_SAMPLE`), số mẫu/giây, vận tốc đo được (tiến / phải, trung bình 0,25 s); Position Hold: trạng thái từng trục, góc nghiêng yêu cầu, điểm đang giữ, bù gió; Rescue: giai đoạn, lý do bỏ cuộc, khoảng cách tới home, độ cao bay về |
| 14 | link | 1 / 10 | (0.8.8) sức khoẻ đường truyền: `uptime` (s, tới ms), `boot_id`, khung nhận / hỏng, `busy`, `tx_lost`, `dropped`, `kicks`, `cmd_max_ms`, `link_gap_ms`, `slow_gap_ms`, `queued` - xem docs/FIRMWARE.md "Firmware không trả lời" |

Section mới 0.8.8: **`althold`** (id 19, live): `mid_stick`, `deadband`, `low_stick`, `max_stick` (% hành trình cần ga, không qua đường cong ga), `climb_rate`, `descend_rate` (0,1 m/s). `nav` thêm `ph_min_gps_hz` (compat 29, 27).
Cùng cổng với WBP firmware còn hiểu **MSP** đủ cho công cụ ESC (docs/ESC_PASSTHROUGH.md); hai parser cùng nhận mọi byte.

Tên các bit (`bits` trong json) được firmware kiểm tra lúc biên dịch với enum thật của core (`static_assert` trong `settings_apply.h`): đổi enum trong core mà quên sửa json là lỗi biên dịch, không phải nhãn sai trên app.

## 4. Command

| id | lệnh | tham số | ghi chú |
|---|---|---|---|
| 1 | GYRO_CAL | — | ~1 s, tiến độ ở stream `calibration`; máy phải đứng yên |
| 2 | ACC_CAL_POINT | — | lấy trung bình ~1 s một mặt; firmware **tự nhận ra mặt nào** (bit trong `calibration.mask`) |
| 3 | ACC_CAL_FINISH | — | cần đủ 6 mặt; khớp offset + scale, ghi vào section `calibration` (RAM) |
| 16 | ACC_CAL_CANCEL | — | |
| 4 | LEVEL_TRIM | — | máy nằm phẳng, đứng yên; từ chối nếu lệch > 20° |
| 5/6/7 | MAG_CAL_START / FINISH / CANCEL | — | FINISH trả `reason` (enum `mag_cal_result`); thất bại thì vẫn tiếp tục thu mẫu |
| 8 | MOTOR_TEST | motor u8 (255 = tất cả), value u16 0..1000 | **phải gửi lại ≤ 0,5 s/lần**, nếu không motor tự dừng; **tháo cánh** |
| 9 | MOTOR_STOP | — | |
| 10 | MOTOR_DIRECTION | motor, reversed | lệnh DShot đảo chiều (chỉ DShot) |
| 11 | MOTOR_BEACON | motor | |
| 12 | SET_HOME | — | cần GPS dùng được |
| 13 | ESC_TELEMETRY | enable | extended DShot telemetry |
| 14 | APPLY_SETTINGS | — | áp lại mọi section live |
| 15 | REPORT | — | gửi lại báo cáo khởi động (cảm biến nào tìm thấy / vì sao không) dưới dạng khung LOG |
| 17 | BEEP | condition (trống = im lặng) | phát thử một âm trên còi và trên motor (nếu ESC dùng DShot), kể cả âm đang tắt; `NO_SENSOR` khi máy không có gì để kêu |
| 18 | BLACKBOX_ERASE | — | xoá cả bộ nhớ log của thiết bị đang chọn (flash: xoá vùng; thẻ SD: xoá các file `wb_*.wblog`); chạy nền (tiến độ ở stream `blackbox`), chặn arm cho tới khi xong |
| 19 | BLACKBOX_STREAM | enable, rate_hz (0 = theo cấu hình) | gửi log về app bằng khung `BB_HEADER` / `BB_DATA`; **được phép khi arm**, nhưng bị từ chối (`BUSY`) khi đang arm và đang ghi flash |
| 20 | BLACKBOX_SESSION | index → start, size | vị trí một phiên trong bộ nhớ log (`BAD_ARG` = hết danh sách, `NO_SENSOR` = không có bộ nhớ / không có thẻ). Với thẻ SD: các file xếp nối nhau thành một dải địa chỉ |
| 21 | BLACKBOX_READ | offset, length | gửi một đoạn bộ nhớ log bằng các khung `BB_BLOB`; length 0 = huỷ |

Trong lúc app đang hiệu chuẩn, test motor, hoặc bộ nhớ log đang được xoá, firmware **chặn arm** (blocker `CALIBRATING`).

Ba loại khung của blackbox (`BB_HEADER` 0x60, `BB_DATA` 0x61, `BB_BLOB` 0x62) không phải câu trả lời của yêu cầu nào: client chuyển chúng cho `on_blackbox` / bộ tải log trước khi ghép trả lời theo `seq`. Định dạng bên trong: `docs/BLACKBOX.md`.

"App đang kết nối" (dùng cho im lặng của còi và việc kết thúc luồng log) = có khung hợp lệ trong 3 s gần nhất; client gửi một `HELLO` với `seq` 0 mỗi 0,3 s khi không có gì khác để gửi.

## 5. Luồng và an toàn (`src/app/exchange.h`, `link_host.h`)

- Vòng bay (core 1) **không bao giờ chờ** task link: telemetry được `publish()` qua seqlock (ghi không khóa), task link `snapshot()` rồi đóng gói.
- Mọi command và mọi lần áp settings **chạy bên trong vòng bay**, giữa hai chu kỳ điều khiển (`Exchange::service()`), nên driver và core chỉ bị một luồng chạm vào. Task link chờ kết quả tối đa ~200 ms; yêu cầu không được nhận kịp sẽ bị **rút lại** (không bao giờ chạy muộn — quan trọng với motor test).
- `check_request()` (trạng thái arm, kích thước tham số, lệnh lạ) được kiểm tra bên vòng bay, với đúng trạng thái arm của chu kỳ đó.
- Ghi flash NVS làm đứng truy cập flash cả hai core vài ms → chỉ cho phép khi disarm.

## 6. Thêm một trường / lệnh mới

1. Sửa `protocol/wbp_schema.json` (thêm field vào section/stream, hoặc command). Field `f32` bắt buộc có `min`/`max`.
2. `make proto` → header mới, hash mới.
3. Dùng field trong `src/proto/settings_apply.h` (settings → cấu hình) hoặc `src/app/telemetry.h` (stream) hoặc `FlightApp::execute()` (command).
4. `make test && make apptest`. Section đổi kích thước thì bản ghi flash cũ của **riêng section đó** về mặc định ở lần khởi động sau — trừ khi chỉ thêm field ở CUỐI section và ghi kích thước cũ vào `"compat"` (xem mục Lưu trữ); thêm field vào giữa thì không dùng được `compat`.

## Đã kiểm chứng / chưa

Đã kiểm chứng trên PC: xem `docs/FIRMWARE.md` (bảng test). Client Python được test với **chính mã firmware thật** chạy trong `wb_sim`.
Chưa kiểm chứng: mọi thứ trên mạch thật — USB CDC của ESP32-S3 (`availableForWrite`, tốc độ thực), NVS, hành vi DTR/RTS khi mở cổng (client giữ DTR/RTS thấp để không reset mạch, chưa thử), độ trễ thực của `Exchange` giữa hai core.
