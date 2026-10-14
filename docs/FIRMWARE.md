# Firmware RX: `main.cpp`, `FlightApp`, liên kết app, mô phỏng trên PC

## 1. Cấu trúc

```
src/main.cpp                 setup(): NVS → settings → FlightApp.configure/begin → task bay → task cảm biến → task link → xoá task loop của Arduino
src/app/flight_app.h         thân firmware: driver + FlightCore + telemetry + xử lý lệnh của app
src/app/flight_task.h        task `wb_flight` + timer phần cứng đánh thức mỗi chu kỳ điều khiển; khoá dùng chung (FreeRTOS mutex)
src/app/sync.h               khoá (giao diện), thời gian từng khâu, "vòng lặp có giữ được tần số không"                 [portable]
src/app/tx_queue.h           hàng đợi phát 8 kB dùng chung cho UART / USB Serial-JTAG / USB CDC                          [portable]
src/config/fast_code.h       WB_FAST = IRAM_ATTR cho mã của chu kỳ điều khiển (docs/HARDWARE_FEEDBACK_1.md mục 2)
src/app/telemetry.h          app::Telemetry (đơn vị kỹ thuật) → struct trên dây (scale, bão hòa, NaN→0)      [portable]
src/app/exchange.h           cầu nối duy nhất giữa vòng bay và task link (seqlock + hộp thư 1 yêu cầu)      [portable]
src/app/link_host.h          wbp::Host cho task link + check_request() cho vòng bay                          [portable]
src/app/link_arduino.h       SerialTransport, NvsStorage (Preferences), LinkTask (FreeRTOS, core 0), EspEscPort (ESC passthrough)
src/app/msp_bridge.h         MSP vừa đủ cho công cụ ESC (BLHeliSuite32 ...) trên cùng cổng với WBP          [portable]
src/hal/motor/esc4way.h      giao diện 4-way + bootloader BLHeli trên dây motor (docs/ESC_PASSTHROUGH.md)     [portable]
src/proto/*                  khung WBP, server, lưu settings, settings → cấu hình (xem docs/PROTOCOL.md)       [portable]
src/hal/rx/*                 receiver nối tiếp CRSF / SBUS / IBUS (xem docs/RX.md)
src/hal/battery/*, beeper/*  đo pin (ADC / ESC), tiếng báo bằng còi và bằng motor (xem docs/POWER.md)
src/blackbox/*               log chuyến bay: hàng đợi, flash thô, thẻ SD, luồng về app (xem docs/BLACKBOX.md)
src/config/pin_check.h       kiểm tra bảng chân lúc khởi động: trùng chân, chân của flash / PSRAM / USB / SWD  [portable]
src/app/sysload.h            % CPU của nhân chạy vòng bay, RAM, PSRAM                                       [portable]
src/main_stm32.cpp           bản STM32H5 của main.cpp (0.10.0, docs/STM32.md)
src/config/version.h         số phiên bản firmware, chung cho hai chip
src/port/*                   phần khác nhau giữa chip: UART, PWM, GPIO / ADC, FreeRTOS, tên chân, flash quad-word (docs/STM32.md mục 4)
src/proto/kv_flash.h         settings trên flash ECC của STM32H5 (ESP32: NVS)                               [portable]
src/blackbox/bb_qword.h      FlashLog trên flash ECC của STM32H5                                             [portable]
```
Mạch mặc định là ESP32-S3 **N16R8**: env, bảng phân vùng, chân không dùng được, bộ chân gợi ý, thẻ SD → `docs/BOARD_N16R8.md`.

| Core | Chạy gì | Ghi chú |
|---|---|---|
| core 1, task `wb_flight` (ưu tiên 20) | `FlightApp::flight_cycle()`: IMU → FlightCore → motor → trả lời DShot → bản ghi blackbox. Được **timer phần cứng** đánh thức đúng mỗi `1 / loop_hz`; mã nằm trong RAM (`WB_FAST`) | chen ngang mọi việc khác trên core 1; tick đến lúc chu kỳ trước chưa xong thì bị bỏ (đếm ở `status.overruns`), không chạy bù. Task giữ một **quỹ thời gian** (`app::TickBudget` / `app::TickLoop` trong `sync.h`): luồng cảm biến luôn có ≥ 20 % nhân, kể cả khi chu kỳ dài hơn chu kỳ đặt; một chu kỳ chạy quá tick được nối tiếp ngay bằng chu kỳ sau (bù tối đa một tick) chừng nào quỹ còn; luồng cảm biến không chờ quá hai chu kỳ đặt + hai chu kỳ điều khiển |
| core 1, task `wb_sensor` (ưu tiên 1, stack 16 kB; từ 0.8.8, trước đó là `loop()` của Arduino) | `FlightApp::slow_update()` mỗi ~1 ms. Pha A, **không giữ khoá**: receiver, GPS, đo pin, **một** cảm biến I2C (baro / lidar / la bàn luân phiên). Pha B, giữ khoá: đưa kết quả vào core, còi, level trim, yêu cầu của app. Pha C (0.8.8), **ba lần giữ khoá ngắn** có nhả ở giữa: thống kê mỗi giây + health, telemetry chậm, telemetry CRSF + bản ghi chậm của blackbox | một giao dịch I2C 0,3 ms không còn dời chu kỳ điều khiển; chu kỳ đến hạn giữa chừng chỉ chờ một mẩu, không chờ cả B + C |
| core 0, task `wb_link` | `wbp::Server`: đọc byte, trả lời, phát stream; `bb::Service`: lấy bản ghi blackbox từ hàng đợi, ghi flash, đưa vào bộ đệm thẻ SD, hoặc gửi về app | có thể bị USB chặn mà không ảnh hưởng vòng bay; **ghi flash thì làm đứng cả hai nhân** (docs/BLACKBOX.md mục 4) |
| core 0, task `wb_sd` | `bb::CardLog::work()`: ghi bộ đệm ra thẻ SD từng khối 4 kB, sync mỗi 250 ms; tìm thẻ mỗi 3 s khi chưa có | chỉ làm việc khi `blackbox.device = SDCARD`; thẻ chậm chỉ làm chậm task này |

Thứ tự khởi động (`setup()`): `Serial` → NVS → `SettingsStore::load()` (section hỏng/thiếu về mặc định riêng nó) → `FlightApp::configure(settings)` → `FlightApp::begin()` → task bay → task cảm biến → gửi báo cáo khởi động dạng LOG (app lấy lại bằng lệnh `REPORT`) → task link, task thẻ SD → `vTaskDelete(NULL)`: task loop của Arduino tự xoá, `loop()` không bao giờ chạy (chỉ khi tạo task cảm biến thất bại thì `loop()` gánh việc đó, báo cáo có dòng `loop: FAILED to start the sensor task`).

## 2. `FlightApp`

- **Chân**: app gửi section `pins` → firmware giữ trong RAM (`wbp::Settings::pins`) → `SAVE` ghi vào NVS (`wbp::SettingsStore`, khóa `wbs11`) → mỗi lần khởi động `FlightApp::begin()` đọc lại và nạp vào **`HardwareConfig::setAppPins()`**. Mọi driver hỏi chân qua getter của `HardwareConfig` (`getSPIAccGyroSensorPins()`, `getMotorPins()`, `getSerialRxPins()`...): chân app gửi được ưu tiên, ô app để trống (255) thì dùng mặc định ghi trong `src/config/hardware_config.h`. Vì chỉ nạp lúc khởi động nên đổi chân cần lưu + khởi động lại. IMU dùng SPI khi có chân CS, không thì I2C. UART1 = GPS, UART2 = receiver.
- **Tần số vòng lặp** đặt = min(`general.loop_hz`, tốc độ gyro của chip); IMU I2C bị giới hạn 1 kHz. Giá trị này ghi trong báo cáo khởi động và dùng để cấu hình bộ lọc. Tần số **đạt được** và thời gian từng khâu (đọc IMU / core / motor / còn lại, trung bình mỗi giây) nằm trong stream `status`; dưới 80 % trong 2 giây liền (không tính giây có app đang xoá log, hiệu chuẩn…) → blocker `LOOP_RATE`.
- **Hai luồng, một khoá** (`sync.h`): mọi thứ hai luồng dùng chung (core, IMU, motor, telemetry) chỉ được đụng tới khi giữ khoá; không có lời gọi thiết bị nào khi đang giữ khoá. `test_threads` chạy đúng cấu trúc này với luồng thật dưới ThreadSanitizer.
- `dt` của mỗi chu kỳ là thời gian thật từ chu kỳ trước (kẹp 0,25 … 8 lần chu kỳ đặt); bộ ước lượng bước theo tổng thời gian thật.
- **Hiệu chuẩn gyro không chặn**: chạy ngay sau khi khởi động (~1 s); nếu máy bị động thì tự thử lại cho tới khi được (blocker `GYRO_NOT_CAL`).
- **Hiệu chuẩn accelerometer 6 mặt** (`hal/acc_gyro/acc_calibration.h`): đặt máy lên 6 mặt theo thứ tự bất kỳ, firmware tự nhận mặt; khớp offset + scale sao cho cả 6 số đo dài đúng 1 g → mặt bàn nghiêng vài độ không làm sai (phương pháp (max+min)/2 thông thường thì sai). Lưu ở hệ trục chip, trước board alignment.
- **Level trim**: tính góc roll/pitch căn chỉnh sao cho tư thế đang nằm đọc thành phẳng; cộng với board trim người dùng nhập. **Level trim khi bay** (mode CALIB): đo gia tốc kế lúc treo yên, áp dụng sau khi disarm, firmware tự ghi flash (`docs/TUNING.md` mục 7).
- **IMU chết**: 32 mẫu liên tiếp giống hệt nhau từng bit (dây MISO đứt đọc ra 0xFF/0x00 = "đứng yên" rất hợp lý) → coi là lỗi; lỗi liên tiếp 20 chu kỳ → dừng motor chủ động, disarm, blocker `NO_IMU`, phải gạt lại công tắc arm.
- **Telemetry**: trường nhanh cập nhật mỗi chu kỳ, công bố ≤ 1 kHz; trường chậm mỗi ms.
- **Lệnh của app**: `execute()` — xem bảng lệnh trong `docs/PROTOCOL.md`.
- **Kiểm tra bảng chân** (`check_pins()` → `pin_check.h`): chạy một lần trong `begin()` trên các chân đang dùng. Trùng chân → `status.pin_conflict` → blocker `PIN_CONFLICT` (không arm được) + dòng `pins: FAILED - …` trong báo cáo; chân của flash / PSRAM → dòng `pins: WARNING - …`.
- **CPU / RAM** (yêu cầu "hiển thị cpu, ram đang load bao nhiêu %"): `status.cpu` = phần trăm thời gian nhân 1 bận (chu kỳ điều khiển + phần việc mỗi ms) trong giây vừa qua; 100 % nghĩa là vòng lặp không còn giữ được tần số. `status.ram` = RAM trong đang dùng, `heap_free`, `psram_free`, `psram_total` (kB). Nhân 0 (app, thẻ SD) không nằm trong con số CPU.

Giới hạn đã biết:
- Bản 0.5 đo trên mạch thật chỉ đạt ~1500 Hz ở 4000 Hz đặt. Ba nguyên nhân và cách sửa: `docs/HARDWARE_FEEDBACK_1.md` mục 1 – 2. **Con số của bản này trên mạch: chưa có.**
- Ghi flash (blackbox ra flash, lưu settings) làm đứng **cả hai core**, kể cả mã nằm trong RAM; chỉ ngắt đăng ký riêng mới chạy tiếp.
- Thư viện mà chu kỳ điều khiển còn gọi trong flash (libm, SPI của Arduino, vài hàm gptimer): danh sách do `tools/iram_report.py` in ra.
- Pin: ADC hoặc telemetry ESC, xem `docs/POWER.md`. Không có nguồn đo nào thì không chặn arm.
- Section `ports` mới chỉ được lưu.
- Bus SPI trên ESP32-S3: xem `docs/BOARD_N16R8.md` mục 6 (lỗi số bus đã sửa trong `bus_spi.h`).
- Blackbox ghi flash làm đứng vòng bay mỗi lần ghi trang: số thật chưa đo, xem `docs/BLACKBOX.md` mục 4 (`status.gap_max_us`, trường `gap_us` trong log).

## 3. Chạy firmware trên PC

`test/virtual_board.h` là một mạch FC ảo: **mã firmware thật** (`FlightApp`, driver, decoder CRSF, core, motor manager, server WBP) chạy trên các chip mô phỏng mức thanh ghi (ICM-42688-P qua SPI, BMP280 + QMC5883L qua I2C, u-blox qua UART1, receiver CRSF qua UART2) và một mô hình quadcopter. Khung DShot firmware phát ra được giải mã để quay "motor" của mô hình. Chỉ có ngoại vi ESP32 (SPI/I2C/UART/RMT/NVS/FreeRTOS) bị thay bằng `test/arduino_stub`.

```
make test        # 20 chương trình test C++ (kể cả test_fw: firmware-in-the-loop, test_threads: hai luồng thật)
make sim         # build/wb_sim
build/wb_sim     # app nối vào  socket://127.0.0.1:5760 ; cổng 5761 = "bàn tay người thử"
make apptest     # client Python ↔ wb_sim (cần python3, pyserial, pytest)
```

Cổng điều khiển của `wb_sim` (mỗi dòng một lệnh, trả `ok`): `rc <kênh 1..16> <µs>`, `tx on|off`, `pose <roll> <nose_down> <yaw>`, `shake 0|1`, `batt <volt>`, `logmem garbage`, `sdcard pull|push|slow <n>|put <số> <byte>` (thẻ SD ảo 64 MB, nằm trong RAM), `mount <roll> <pitch>`, `accerr ox oy oz sx sy sz`, `magerr hx hy hz`, `vib <trục 1..3> <Hz> <°/s>` (rung trên gyro), `weak <motor 1..8> <hệ số>` (motor yếu), `swap <motor> <motor>` (cắm nhầm hai dây ESC: thứ mà chặn runaway bắt), `esc <motor|0 = tất cả> <0|1>` (ESC ngừng / lại trả lời rpm), từ 0.7.0: `carry <z> <vn> <ve>` / `carry off` (cầm máy đi ở độ cao z, m/s bắc / đông), `wind <bắc> <đông>`, `flow quality <0..255>|dark <0|1>|plug <0|1>|mount <độ> <lật 0|1> <hệ số>`, `gps fix <0|1>`, `gps dead <0|1>`, từ 0.8.0: `adc <gpio 1..20> <mV>` (điện áp ở chân ADC, ví dụ cảm biến dòng), `escamps <motor> <A>` (dòng ESC báo qua extended DShot telemetry), `rclink crsf|spektrum2048|spektrum1024|ppm|pwm` (receiver trên mạch ảo: PPM ở GPIO40, PWM ở GPIO39, 40, 41, 42, 1, 3, 45, 46), `state` (kèm `pn pe vn ve` = vị trí / vận tốc thật, `m0..m3` = lệnh mà ESC ảo thật sự nhận, `tones` = số lệnh beacon ESC đã nhận, `sd_files`, `sd_writes`), `quit`. Máy ảo có chuyển động ngang với lực cản và **gia tốc kế đúng vật lý** (khi bay chỉ cảm nhận lực đẩy + lực cản): giữ cần roll thì nó bay đi thật, và đường chân trời lệch vài độ khi tăng tốc như trên máy thật (`docs/TUNING.md` mục 5). Trong `wb_sim` máy ảo có càng đáp: chạy thử một motor không làm nó lật (trong `make test` thì không, các test vòng kín viết cho vật thể tự do). Tùy chọn: `--port N --flash FILE --fresh --speed X --no-baro --no-mag --no-imu --no-flow`. Mạch ảo của `wb_sim` nối app qua USB (UART0 rảnh) và có sẵn một MicoAir MTF-01 trên UART0 (chân 44) cùng một PMW3901 trên bus SPI thứ hai; lần khởi động đầu tiên điền sẵn chân của chúng. Flash ảo lưu ra file nên settings sống qua lần chạy sau; `REBOOT` khởi động lại firmware ảo thật sự.

Dùng `wb_sim` để viết giao diện app mà không cần mạch: arm bằng `rc 5 2000`, bay bằng `rc 3 1700`, v.v.

## 4. Kiểm chứng

| Test | Số kiểm tra | Nội dung |
|---|---|---|
| `test_proto` | 165 | khung, CRC, tự đồng bộ, schema, lưu flash, server, stream, settings → cấu hình |
| `test_rx` | 43 | CRSF / SBUS / IBUS với encoder viết độc lập, nhiễu, đồng bộ lại, trạng thái link |
| `test_motor` | 183 | khung DShot, lệnh DShot, xung PWM, GCR / eRPM / EDT, telemetry, dynamic idle |
| `test_dshot` | 91 | driver DShot trực tiếp trên **mô hình dây tín hiệu** (docs/MOTOR_OUTPUT.md) |
| `test_power` | 123 | pin và tiếng báo |
| `test_blackbox` | 301 | định dạng (giá trị PID thật, thời gian từng khâu), hàng đợi, flash NOR và thẻ SD mô phỏng, tải về, luồng, giới hạn theo đường truyền |
| `test_app` | 277 | đóng gói telemetry, Exchange với luồng thật, hàng đợi phát, khoá, thống kê vòng lặp và **quỹ thời gian của task bay trên timer mô phỏng**, hiệu chuẩn, bảng chân |
| `test_fw` | 662 | **firmware-in-the-loop**: khởi động, arm qua CRSF, bay, failsafe (nguyên nhân, trả quyền), runaway, tần số vòng lặp với chi phí I2C / SPI mô phỏng, thời gian từng khâu trong log, settings, hiệu chuẩn, pin, còi, blackbox flash / thẻ SD, trùng chân; từ 0.7.0 optical flow (MSP trên UART0, PMW3901 trên bus SPI thứ hai, hướng lắp, cấp UART), Position Hold, GPS + la bàn, GPS Rescue, nhóm log NAV |
| `test_threads` | 17 | firmware với **hai luồng thật + luồng link dưới ThreadSanitizer**; không có giao dịch bus nào khi giữ khoá |
| các test còn lại | 1639 | lõi bay (`test_core` 296, `test_control` 269, `test_nav` 130, `test_flight` 251: xem `docs/CORE.md`, `docs/NAV.md`), IMU, baro, lidar, optical flow (`test_flowsensor` 94), GPS, la bàn |
| `app/tests` (pytest) | 98 | client Python + CLI + **GUI** (server, công thức, bản dịch, trang thật trong trình duyệt) với `wb_sim`; xem `docs/APP.md` |

Tổng cộng 20 chương trình C++, 3424 kiểm tra.

Mutation test (`make mutate`, cố ý làm hỏng mã, test phải báo lỗi): **1721** lỗi cố ý trong `tools/mutate.py` (0.8.8: 17 lỗi mới hoặc viết lại theo mã mới, chạy riêng: cả 17 bị bắt; 27 lỗi cũ không còn khớp mã vì các đợt trước đổi mã, chưa viết lại), 253 trong `tools/mutate_gui.py`. Kết quả lượt chạy gần nhất: `docs/HANDOFF.md`.

**Đã chạy trên mạch thật**: bản 0.5 (khởi động, IMU ICM20602 qua SPI, arm, motor quay bằng DShot một chiều, app qua UART và qua USB của S3, blackbox ghi được); bản 0.6.1, không cánh, theo log người dùng gửi (`docs/HARDWARE_FEEDBACK_2.md`): task bay + timer, mã trong RAM, driver DShot mới (DShot600 một chiều), receiver CRSF, hàng đợi phát USB với log trực tiếp 250 bản ghi/giây liền mạch trong 87 giây; vòng lặp đo được ≈ 3300 Hz trên 4000. **Chưa chạy trên mạch**: toàn bộ 0.8.0 (đo dòng, receiver Spektrum / PPM / PWM, telemetry CRSF, bảng cổng), servo (0.7.1), toàn bộ 0.7.0 (optical flow, Position Hold, GPS Rescue: `docs/NAV.md` mục 8), mọi thay đổi của 0.6.2 (quỹ thời gian của task bay, các cột mới trong log), DShot hai chiều, bay thật. Ngoài ra vẫn chưa kiểm: NVS, thời gian thực của một chu kỳ (`status.loop_us` và bốn số theo khâu), UART đảo mức cho SBUS, nhiễu I2C lên vòng 4 kHz, thời gian đọc ADC, **thời gian đứng khi ghi flash của blackbox**, thẻ SD thật, ESC có kêu theo lệnh beacon không, PSRAM. Danh sách thử theo thứ tự: `docs/BOARD_N16R8.md` mục 7.

### Cổng USB Serial/JTAG thôi trả lời (0.8.5)
Người dùng báo: nối app qua USB Serial/JTAG, thỉnh thoảng app báo mãi "Firmware không trả lời". Một đường dẫn tới đúng triệu chứng đó, đọc ra từ mã của lõi Arduino (`HWCDC.cpp`): vòng đệm phát đầy, và ngắt rút nó ra không được bật (USB vừa bị reset bus, host lỡ một lần hỏi, bộ canh SOF của lõi báo "rút cáp" nhầm). Lõi chỉ bật lại ngắt đó trong `write()`; hàng đợi của mình (`tx_queue.h`) lại chỉ gọi `write()` khi cổng báo còn chỗ, nên không ai gọi nữa: link chết hẳn cho tới lúc rút cáp. Từ 0.8.5:
- Cổng báo hết chỗ 50 ms liền: hàng đợi vẫn đưa **một byte** (byte kế tiếp của nó, cổng nhận hay không thì cũng không mất, không lặp), lõi bật lại ngắt (`app::StallKick`, `link_arduino.h`). Thời gian chờ ghi của lõi đặt 0 (`setTxTimeoutMs(0)`): lần ghi đó không bao giờ đứng.
- Mỗi lần phải "đá" cổng, app nhận một dòng log: "link: the USB port took nothing for 50 ms and was restarted (N times since boot)".
- Báo cáo khởi động thêm dòng **`reset:`**: lý do chip khởi động (bật nguồn, khởi động lại bằng phần mềm, CRASH panic / watchdog, BROWN-OUT, chân reset, USB). App tự nhận ra firmware khởi động lại và đăng ký stream lại; dòng này cho biết tại sao.
Kiểm chứng: `test_app` (vòng đệm USB đầy không ai rút: không có "đá" thì link chết hẳn, có thì sau 50 ms mọi byte tới đủ, đúng thứ tự; không đá sớm hơn 50 ms). **Chưa thử trên mạch**, và đó mới là một nguyên nhân có thể: nếu app vẫn báo không trả lời, gửi mình dòng `reset:` và các dòng log cuối cùng.

### "Firmware không trả lời" và "software restart" (0.8.8)
Người dùng thấy app báo không nhận được gì từ firmware, và trong log có dòng "software restart". Đọc lại mã thì:
- **"software restart" không phải do lỗi đó**: chỉ có một chỗ trong firmware gọi `ESP.restart()` theo yêu cầu: lệnh REBOOT của app (Lưu + khởi động lại, Huỷ thay đổi). Dòng `reset:` là lý do của lần khởi động **gần nhất thật sự**. Trước 0.8.8, cứ hễ stream im 1,5 s mà firmware vẫn trả lời HELLO là app coi như firmware vừa khởi động lại, đọc lại báo cáo khởi động, nên lại thấy dòng `reset: software restart` cũ.
- Từ 0.8.8 app phân biệt được: stream mới **`link`** (1 Hz, app luôn đăng ký) mang `boot_id` (số ngẫu nhiên rút mỗi lần khởi động, `esp_random()` trộn với bộ đếm trong RTC và đồng hồ µs) và thời gian chạy. `boot_id` không đổi = firmware **không** khởi động lại: app báo "Dữ liệu ngừng N giây nhưng firmware KHÔNG khởi động lại" và ghi vào nhật ký các con số của link; đổi = khởi động lại thật.
- Các con số đó (tab Hệ thống, khối "Đường truyền với app"): yêu cầu không được xử lý kịp (BUSY), lệnh chờ lâu nhất, **luồng link nghỉ lâu nhất**, **luồng cảm biến nghỉ lâu nhất** (nó chạy yêu cầu của app), khung telemetry bỏ, trả lời bị mất, số lần "đá" cổng USB, byte đang chờ gửi. Lần sau gặp lỗi, mấy con số này chỉ ra nằm ở đâu: luồng cảm biến nghỉ hàng chục ms = core 1 bị vòng lặp chiếm hết (hạ `loop_hz`); luồng link nghỉ lâu = core 0 (ghi flash, thẻ SD); "đá" cổng tăng = USB.
- Đồng thời giảm một nguyên nhân có thật: log 0.8.7 cho thấy task bay chờ khoá trung vị **236 µs** (`t_lock`): pha B của luồng cảm biến giữ khoá liền một mạch cả phần telemetry chậm. 0.8.8 cắt phần đó thành ba mẩu, nhả khoá giữa các mẩu.
Kiểm chứng: `test_app` (stream `link`: uptime, kick, khoảng nghỉ của task link, byte trong hàng đợi), pytest `test_a_pause_of_the_streams_is_not_called_a_restart` (wb_sim lệnh `linkpause`: stream ngừng 3 s nhưng vẫn trả lời → STALLED, không phải REBOOTED) và `test_a_restart_behind_our_back_is_noticed`. **Chưa thử trên mạch.**

## 5. Lỗi tìm ra trong lúc ghép (đã sửa)

- `now_ms = micros()/1000` trong vòng nhanh lệch khỏi `millis()` sau 71,6 phút (micros tràn 32 bit) → mọi kiểm tra "dữ liệu cũ" sai. Nay dùng `millis()`; có test bay xuyên qua thời điểm tràn.
- Ô mode trống (255) bị `validate()` kẹp thành mode 7 (BLACKBOX). Nay ô `optional` giữ 255.
- `validate()` để lọt NaN (NaN so sánh với gì cũng sai) → app lỗi có thể đưa NaN vào PID. Nay NaN → mặc định, mọi `f32` bắt buộc có min/max.
- IMU không trả lời: vòng bay đứng im lặng, app vẫn thấy "không có blocker". Nay báo `NO_IMU`, dừng motor.
- Dây MISO đứt đọc ra số 0 hợp lệ → thêm phát hiện "chip đông cứng".
- **Số bus SPI sai trên ESP32-S3** (`bus_spi.h` dùng 2 / 3 của ESP32 đời đầu; S3 chỉ có 0 / 1): IMU qua SPI sẽ không bao giờ khởi động trên mạch N16R8. Tìm ra khi đọc mã core Arduino để gắn thẻ SD vào bus SPI thứ hai; stub cũ chấp nhận mọi số bus. Chi tiết: `docs/BOARD_N16R8.md` mục 6.
- **`MotorOutputManager::rpm_array` chỉ điền 4 motor**: trên hexa, motor 5 và 6 không bao giờ có RPM notch. Nay điền mọi motor; RPM filter làm việc theo từng motor (một ESC im lặng chỉ tắt notch của chính nó).
- **Bộ ước lượng tư thế tin gia tốc kế quá nhiều** (gain 2 /s): trong Angle, giữ nửa cần 2 giây thì máy nghiêng 27° cho lệnh 12°. Lộ ra khi gia tốc kế của mạch ảo được làm đúng vật lý. Sửa và giải thích: cuối `docs/CORE.md`.
- **Ga treo đoán 40 %** trong failsafe không có độ cao: quad lực đẩy lớn sẽ bay vọt lên. Nay ga treo được đo (`docs/TUNING.md` mục 6).
- `main.cpp`/`main_rx.cpp` cũ không biên dịch được (include file không tồn tại) → thay bằng `main.cpp` thật; xóa `main_rx.cpp`, `hal/hal.h`, `hal/sensor/*`, `hal/rx/{crsf,ibus,sbus}.h` cũ, `acc_gyro/mpu6xxx.h` (bản cũ vẫn còn trong zip bạn gửi trước).
