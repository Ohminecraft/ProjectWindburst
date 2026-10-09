# Phản hồi phần cứng lần 1 (firmware 0.5 trên mạch thật) và những gì đã đổi

Mạch thử: ESP32-S3 N16R8, IMU ICM20602 qua SPI, ESC Aikon AK32 (BLHeli_32), nối app qua cổng USB của chính ESP32-S3.

Tài liệu này đi theo từng điều bạn báo: hiện tượng → nguyên nhân tìm được → đã sửa gì → kiểm chứng tới đâu. **Không có gì ở đây được chạy trên mạch**: mọi kiểm chứng là test trên PC, mô hình dây tín hiệu, và đọc file ELF đã build cho ESP32-S3. Cột "cần bạn xem trên mạch" là phần còn thiếu, gom lại ở mục 10.

**Bản 0.6.2** = 0.6.0 + ba sửa đổi, hai trong số đó đến từ log đầu tiên của mạch thật (`docs/HARDWARE_FEEDBACK_2.md`):

- task bay luôn nhường thời gian cho luồng đọc receiver / cảm biến / app (đoạn "Khi một chu kỳ vẫn dài hơn chu kỳ đặt" ở mục 1). Với 0.6.0, nếu một chu kỳ điều khiển dài hơn 250 µs thì app mất kết nối và receiver không được đọc. Bản 0.6.1 (gửi giữa chừng) nhường theo từng chu kỳ và vì thế mất tick không cần thiết: trên mạch chỉ còn 3300 Hz. 0.6.2 tính theo quỹ thời gian;
- các cột PID trong log không còn bị kẹp ở ±3,28;
- log có thêm thời gian từng khâu của một chu kỳ.

**Hãy nạp 0.6.2** (app hiện phiên bản ở thanh trên cùng sau khi kết nối).

## 1. Vòng lặp chỉ đạt ~1500 Hz (đặt 4000 Hz), khi arm còn ~1300 Hz

Tìm được **ba** nguyên nhân, cộng dồn lên nhau.

| Nguyên nhân | Sửa |
|---|---|
| Cảm biến I2C, receiver, GPS, đo pin được đọc **trong cùng luồng** với vòng điều khiển: mỗi giao dịch I2C chặn ~0,3 ms | Vòng điều khiển có **task riêng** (`wb_flight`, ưu tiên 20, core 1), được timer phần cứng đánh thức đúng mỗi chu kỳ. Mọi việc chậm chạy ở `loop()` của Arduino (ưu tiên 1): task bay chen ngang bất cứ lúc nào. Hai bên dùng chung một khoá; **không bao giờ giữ khoá trong lúc chờ thiết bị** (`src/app/flight_task.h`, `sync.h`) |
| IMU đọc qua SPI **từng byte một** (15 lần gọi `transfer()` cho 14 byte) | Một lần `transferBytes()` cho cả gói (`src/hal/bus/bus_spi.h`) |
| Mã của vòng điều khiển nằm trong flash và chạy qua **instruction cache 16 kB dùng chung cho cả hai core** (cấu hình có sẵn của Arduino core cho ESP32-S3). Một chu kỳ đi qua ~30 kB mã, nên chu kỳ nào cũng phải nạp lại mã từ flash, vài µs cho mỗi 32 byte | Đưa mã của chu kỳ điều khiển vào **RAM trong** (`IRAM_ATTR`), xem mục 2 |

**Khi một chu kỳ vẫn dài hơn chu kỳ đặt** (ví dụ 800 µs như bản 0.5 đo được, trong khi 4000 Hz chỉ cho 250 µs): task bay có ưu tiên cao nhất trên nhân của nó, nên nếu cứ thấy tick đang chờ là chạy tiếp thì luồng đọc receiver, cảm biến và yêu cầu của app **không bao giờ được chạy** - máy vẫn tự cân bằng nhưng không còn nghe phi công, app mất kết nối. Vì vậy task bay giữ một **quỹ thời gian** (`app::TickBudget` / `app::TickLoop`, `src/app/sync.h`): luồng cảm biến được nợ ¼ thời gian của mỗi chu kỳ dưới dạng thời gian nghỉ (⅕ nhân); nghỉ nhiều hơn thế thì "có", tối đa nửa chu kỳ đặt; còn "có" thì một chu kỳ chạy quá tick được nối tiếp ngay (bù tối đa một tick); số dư âm thì task nghỉ tới khi trả xong. Kết quả ở 4000 Hz: chu kỳ 135 µs với một trong tám là 290 µs (số đo của mạch thật) → đủ 4000 Hz; 240 µs đều → 3200 Hz; 800 µs đều → 1000 Hz. Trong mọi trường hợp luồng cảm biến có ít nhất 20 % nhân và app vẫn hiện số đo từng khâu cùng blocker `LOOP_RATE`, thay vì treo. Bảng so sánh và các bảo đảm: `docs/HARDWARE_FEEDBACK_2.md` mục 3.

Thêm: `dt` đưa vào bộ ước lượng là **thời gian thật** giữa hai chu kỳ (trước đây: "chu kỳ cuối × 8"), mỗi mẫu gia tốc kế được tính theo đúng khoảng thời gian nó đại diện. Chu kỳ đến trễ không được "chạy bù" dồn dập.

**Tab Tổng quan có số đo mới** (stream `status`): thời gian từng khâu của một chu kỳ (đọc IMU · tính toán · gửi motor · còn lại), lượt dài nhất của luồng cảm biến, tần số thực / tần số đặt, số chu kỳ bị bỏ lỡ mỗi giây. Vòng lặp dưới 80 % tần số đặt trong 2 giây liền thì **không cho arm** (blocker `LOOP_RATE`): mọi bộ lọc và hệ số PID được tính cho tần số đã đặt.

Kiểm chứng trên PC: `test_threads` chạy firmware với hai luồng thật + luồng link dưới ThreadSanitizer (mọi truy cập ngoài khoá làm test hỏng, và một cái móc chứng minh không có giao dịch bus nào xảy ra khi đang giữ khoá); `test_fw` mô phỏng chi phí thời gian của I2C / SPI và kiểm tra tần số, số chu kỳ lỡ, blocker.
Chưa biết: con số thật trên mạch. Đây là thứ đầu tiên cần bạn gửi lại.

## 2. `IRAM_ATTR` (yêu cầu của bạn)

- Macro `WB_FAST` trong `src/config/fast_code.h` = `IRAM_ATTR` trên ESP32, rỗng trên PC. Tắt để so sánh: thêm `-DWB_FAST_CODE=0` vào `build_flags`.
- Đã đánh dấu **86 hàm, 30,3 kB**: task bay, `FlightApp::fast_cycle`, đọc IMU (driver + bus SPI), `FlightCore::update`, chuỗi lọc gyro, RPM filter, dynamic notch, AHRS, PID, mixer, Angle / Stabilize / Altitude Hold, bộ ước lượng, lớp motor, DShot (gửi, đổi chiều chân, giải mã trả lời), ghi bản ghi blackbox. Tổng mã trong RAM: 73,5 → 105,2 kB; heap giảm tương ứng ~32 kB (số heap còn lại hiện ở tab Tổng quan, dòng RAM).
- `tools/iram_report.py <firmware.elf>` đọc file ELF và in: hàm nào đã nằm trong RAM, và **hàm nào vòng điều khiển còn gọi trong flash**. Hiện còn 25 hàm, 2,6 kB. 21 trong số đó là thư viện không đánh dấu được từ đây: `sinf / cosf / atan2f / asinf / acosf / sqrtf / tanf`, `fminf / fmaxf`, `lroundf / lrintf`, SPI và `digitalWrite` của Arduino, `gptimer_set_alarm_action`, `micros / millis`; 4 hàm của firmware chỉ chạy lúc khởi động, lúc hiệu chuẩn hoặc với IMU khác (BMI270). Từ 0.6.2 báo cáo đọc lời gọi thẳng từ mã máy (objdump bỏ sót vài hàm do trình biên dịch tách ra, trong đó có chính chu kỳ điều khiển). Phép chia số thực (`__divsf3`, 76 chỗ gọi) nằm trong ROM của chip, không qua cache.
- **Từ 0.7.0** (điều hướng chạy trong chu kỳ có bộ ước lượng): **98 hàm, 38,6 kB** trong RAM (thêm optical flow → vận tốc, `TiltAid`, bộ điều khiển vị trí, Position Hold, GPS Rescue, phần ngang của bộ ước lượng); tổng mã trong RAM 115 kB. Còn 26 hàm, 3 kB trong flash; mới là `atanf` (436 byte, đổi gia tốc → góc nghiêng, chỉ khi Position Hold / Rescue đang bay, ở tần số bộ ước lượng). Báo cáo nay còn liệt kê hàm nào của chu kỳ dùng **số thực kép** (FPU của S3 chỉ có số thực đơn, số kép là phần mềm trong ROM): bản nháp đầu của `TiltAid` dùng số kép ở 500 Hz, đã viết lại; còn lại `bb::put_int` / `pack_fast` (nhánh 32 bit của bản ghi blackbox, thực tế không chạy tới).
- Không đánh dấu được constructor / destructor (GCC báo lỗi section); khoá của chu kỳ bay vì thế được viết thành `lock()` / `unlock()` thay cho đối tượng `Guard`.

Điều `IRAM_ATTR` **không** làm: không làm một lệnh chạy nhanh hơn (mã đã nằm trong cache thì nhanh như trong RAM), và không giữ vòng bay chạy lúc ghi flash (ghi flash làm đứng cả hai core, dù mã ở đâu).

Tiện thể tìm ra khi đọc ELF: giải mã trả lời DShot hai chiều làm **một phép chia 64 bit cho mỗi cạnh xung** (tới ~100 lần mỗi chu kỳ với 4 ESC). Nay tính bằng tick của timer, số nguyên 32 bit.

Kiểm chứng: file ELF (địa chỉ hàm trong vùng 0x4037xxxx), build cho cả bốn cấu hình. **Tốc độ thật: chưa đo được ở đây.**

## 3. DShot hai chiều không chạy, và làm mất luôn lệnh motor

| Nguyên nhân | Sửa |
|---|---|
| Sau mỗi khung, mã cũ gọi `pinMode(pin, INPUT_PULLUP)` để nghe ESC trả lời. Trên ESP-IDF 5.5 lệnh này **tách chân khỏi bộ phát RMT**: từ khung thứ hai trở đi không còn gì đi ra chân nữa | Driver mới ghi thẳng thanh ghi RMT (`src/hal/motor/dshot_direct.h`): chân ở chế độ đẩy-kéo khi phát; một timer bắn đúng lúc khung kết thúc (+3 µs) để **nhả chân** và bật thu; khung sau thu hoạch dữ liệu đã thu rồi mới phát |
| Bộ giải mã đòi đủ 20 bit GCR; trả lời kết thúc bằng mức cao thì bit cuối lẫn vào đường dây rảnh, nên ~37 % trả lời hợp lệ bị loại | Chấp nhận từ 18 bit như Betaflight |

- Một chu kỳ gửi + nghe: DShot300 hai chiều **182 µs**, DShot600 **116 µs** (một chiều: 84 / 57 µs). Vòng điều khiển gọi nhanh hơn thế thì khung bị bỏ qua, và được đếm.
- Tab Motor có dòng trạng thái: DShot hai chiều đang chạy hay không, **ESC nào không trả lời**, số khung bỏ qua và số lượt trả lời bị lỡ trong giây vừa rồi.
- Có lối lùi: mục "Cách phát DShot" = **ESP-IDF** dùng driver có sẵn, chỉ một chiều. Chọn nó nếu với cách "Trực tiếp" motor không nhận lệnh.

Kiểm chứng: `test_dshot` là mô hình dây tín hiệu (ESC giải mã khung như ESC thật, kiểm cả khoảng lặng giữa hai khung, trả lời sau 30 µs, bộ thu có lọc nhiễu và ngưỡng rảnh, ngắt đến trễ / đến thừa), 91 kiểm tra.
**Chưa chạy trên chip.** Thứ dễ sai nhất: thời điểm nhả chân so với lúc ESC bắt đầu trả lời, và cực tính / trở kéo trên dây tín hiệu.

## 4. Đồ thị blackbox giật, đứt đoạn khi arm; qua USB chỉ thấy chấm; tải log từ flash thất bại

| Hiện tượng | Nguyên nhân | Sửa |
|---|---|---|
| Đường đồ thị đứt, chỉ còn chấm | App coi "khoảng cách giữa hai mẫu lớn hơn 2,5 lần chu kỳ **cấu hình**" là chỗ mất dữ liệu. Vòng lặp chạy 1300 Hz thay vì 4000 Hz nên mẫu nào cũng "mất" | Khoảng cách được so với **nhịp thật** của log (trung vị); thông tin log có thêm "thực tế … bản ghi/giây" |
| Giật khi ghi trực tiếp qua cáp | Firmware gửi nhiều hơn đường truyền chở được, khung bị bỏ lung tung | Tốc độ ghi trực tiếp tự giới hạn ở **nửa** dung lượng đường truyền |
| Tải log thất bại | Không phải "mass storage đè lên": mạch không hiện thành ổ USB, log đi qua chính đường truyền của app. Giao diện **từ chối tải khi đang ghi trực tiếp**, và firmware trả `BUSY` khi phiên ghi chưa đóng xong | Bấm tải thì phiên ghi trực tiếp tự dừng (file được giữ) rồi mới tải; client tự thử lại `BUSY` trong 3 giây; firmware đóng phiên đang mở khi app xin đọc lúc đã disarm |
| Cổng USB | Ba kiểu cổng của ESP32-S3 trả lời "ghi được bao nhiêu byte" rất khác nhau (USB CDC kiểu TinyUSB: tối đa 64) | Một hàng đợi phát 8 kB dùng chung cho mọi kiểu cổng (`src/app/tx_queue.h`) |

Build cho cổng USB: env `rx_s3_usb` (`ARDUINO_USB_CDC_ON_BOOT=1`, `ARDUINO_USB_MODE=1` = USB Serial/JTAG). Kiểu TinyUSB (`ARDUINO_USB_MODE=0`) cũng biên dịch được và có đường mã riêng, chưa ai thử. Dùng cổng USB thì GPIO19 / 20 không gán được cho việc khác (firmware cảnh báo lúc khởi động).

Nếu tải log vẫn lỗi: gửi nguyên văn dòng lỗi app hiện ra.

## 5. Arm xong, để ga thấp nhất: motor tự tăng tốc dần. Lắc cần rồi về ga min: motor cứ tăng, phải arm lại mới hết

Đây là **I-term tích luỹ khi máy không trả lời được** (đứng trên bàn, không cánh). Betaflight với cấu hình mặc định làm đúng như vậy: airmode giữ PID chạy ở ga thấp nhất kể từ lúc ga vượt 25 % lần đầu, cho tới khi disarm. Sai số gyro 0,3 °/s đủ làm đầy I của yaw sau ~22 giây.

Sửa: mục **Airmode** (tab Cảm biến) có ba lựa chọn.
- **Khi đang bay** (mặc định): firmware tự nhận ra máy đang đứng trên mặt đất (ga ở thấp nhất, không quay, gia tốc kế đọc 1 g, suốt 1 giây) và đưa mọi motor về idle, I-term về 0. Trên không thì vẫn ổn định đầy đủ ở ga thấp nhất.
- **Luôn bật tới khi disarm**: cách của Betaflight (motor sẽ tự tăng tốc trên bàn).
- **Tắt**.

Ở idle mặt đất, gạt roll / pitch / yaw không làm motor phản ứng (kiểm tra: tab Motor, vòng lệnh của bốn motor bằng nhau).

Đi kèm: chỉ arm bằng **cạnh lên của công tắc trên đường truyền đang sống**. Bật nguồn (hoặc sóng quay lại lúc chưa arm) mà công tắc ARM đang bật thì không arm; phải thấy công tắc về OFF trước.

## 6. Rà soát thuật toán bay (đối chiếu Betaflight, ArduPilot / ArduCopter, esp-fc)

Ba lượt rà độc lập (vòng rate; mode / Altitude Hold / failsafe; bộ ước lượng). Ghi chú gốc của từng lượt nằm ngoài bản giao; kết quả:

### Đã sửa

| Chỗ | Trước | Nay | Theo |
|---|---|---|---|
| Hướng chip IMU (`board_align`, `mag_align`) | Bảng xoay của Betaflight nhưng áp vào hệ trục khác: chip nằm ngửa ở `CW0` đọc ra "lật úp" | Đúng nghĩa Betaflight. **Giá trị cũ phải đổi**, xem mục 8 | Betaflight `boardalignment.c` |
| Failsafe | Đã vào hạ cánh thì không bao giờ trả quyền; pin yếu và công tắc Failsafe cũng khoá cần; cắt motor sau 60 s dù đang ở trên cao | Có **nguyên nhân** (mất sóng / công tắc / pin). Sóng trở lại liên tục 1 s thì trả quyền; công tắc tắt là hết ngay; hạ cánh vì pin thì bạn vẫn lái roll / pitch / yaw. Giới hạn thời gian chỉ đếm khi máy **không** đi xuống | Betaflight `failsafe.c`, ArduCopter `events.cpp` |
| Failsafe lúc đứng trên mặt đất | "Hạ cánh" ở 85 % ga treo | Tắt motor | Betaflight |
| Runaway lúc cất cánh | Không có | PID dồn hết lực mà máy vẫn quay (sai thứ tự motor, lắp ngược cánh, sai chiều, sai hướng IMU) → tắt motor sau 75 ms, blocker `RUNAWAY_TAKEOFF`. Hết hiệu lực sau nửa giây bay bình thường. Tắt được ở tab Hệ thống | Betaflight `core.c` |
| Altitude Hold mất độ cao giữa chừng | Ga nhảy sang đường cong cần ga (giữa cần = 47 %, gấp 3,5 lần ga treo của quad 5 inch) | Giữa cần = **ga treo đã đo**, đáy = 0, đỉnh = 100 %; có độ cao lại thì tiếp tục từ mức ga đó | ArduCopter |
| Altitude Hold: kéo cần xuống ngay sau khi rời đất | Cắt motor ở độ cao đang có | Hạ xuống có lực, như lệnh "đi xuống" | ArduCopter `takeoff.cpp` |
| Altitude Hold: mất sóng lúc đang tăng ga cất cánh | Vẫn cất cánh | Về idle ngay, phải hạ cần rồi mới cất cánh lại. Trong thời gian chờ failsafe, cần ga **không** được dùng (máy giữ độ cao, không bay vọt theo giá trị kênh cũ) | — |
| Altitude Hold: ga kịch trần / kịch đáy | Bộ tích phân cứ dồn | Chỉ được tích về phía rời khỏi giới hạn | ArduPilot `AC_PosControl` |
| Altitude Hold: máy nghiêng quá 84° / lật ngửa | Tăng ga ×2 (đẩy máy **xuống**) | Giảm dần về 0 | ArduPilot "inverted factor" |
| Failsafe không có độ cao | Ga treo bị "học" lại trong lúc đang rơi có lực cản: càng rơi càng nhanh | Không học trong lúc hạ kiểu này | — |
| Lidar và độ cao | Lidar được coi là độ cao tuyệt đối: bay qua cái bàn thì ước lượng nhảy bằng chiều cao cái bàn | Có barometer thì lidar đo **so với mốc mặt đất**; 10 số đo liền không khớp, hoặc đo lại sau khi mất số: mốc dời, độ cao không đổi | Betaflight `position_estimator.c`, ArduCopter |
| Barometer | Tin số "nhiễu" tự đo lúc đứng yên (5 cm sau một phút yên tĩnh, 80 cm khi vừa được bê đi) | Kẹp trong 0,3 … 1,0 m; gần mặt đất lúc motor đang chạy thì tin một nửa | ArduPilot EKF3 |
| Độ cao GPS | Nhập vào như độ cao tuyệt đối (GPS lệch 3 m kéo ước lượng đi 2 m sau một phút) | Không dùng khi có barometer hoặc lidar | ArduPilot, Betaflight |
| Feedforward | Mất một khung → khung sau mang quãng đường của hai chu kỳ → feedforward gấp đôi | Khung đến trễ từ 1,5 chu kỳ: chia cho thời gian thật; sau khoảng lặng dài: không có "tốc độ" | Betaflight `rc.c` |
| Throttle boost, anti-gravity | Chạy trên cả ga do Altitude Hold / failsafe điều khiển (khuếch đại nhiễu barometer vào motor) | Chỉ theo tay người lái | Betaflight `mixer.c` |
| Level trim khi bay | "Đang bay" = cần ga trên 15 % | = firmware thấy máy đang trên không | — |

### Biết nhưng chưa làm (ghi lại để không quên)

- Gyro bão hoà (trên 2000 °/s) không được xử lý riêng; không có yaw-spin recovery của Betaflight.
- Bias gia tốc theo trục thẳng đứng là một trạng thái trong hệ trục thế giới: nghiêng lâu để lại sai số tốc độ đứng 0,1 – 0,2 m/s trong hơn 10 giây.
- Vòng PID dùng chu kỳ **danh nghĩa** (như Betaflight): nếu vòng lặp không giữ được tần số thì hệ số D / I và tần số cắt lệch theo. Vì vậy mới có blocker `LOOP_RATE`.
- HEX_X: hệ số pitch chuẩn hoá về ±1 (Betaflight ±0,866): bộ số tune của Betaflight cho hexa mạnh hơn 15 % trên pitch.
- Dynamic idle khác Betaflight vài chi tiết (chỉ khi bật DShot hai chiều + dynamic idle).
- Xung PWM / Oneshot lúc chưa arm mặc định là 0 (không phát xung) theo đặc tả của bạn; nhiều ESC analog cần 1000 µs mới khởi động (có ghi trong trợ giúp của ô đó).
- Runaway: máy treo dưới 20 % ga mà không gạt cần thì phép kiểm tra vẫn còn hiệu lực (ngưỡng của Betaflight).
- Hạ cánh failsafe không có độ cao: không nhận biết được chạm đất, dựa vào giới hạn thời gian.

## 7. 3D preview lag qua UART

Đúng như bạn tìm ra: cầu USB-UART ở 230400 baud không chở nổi. Qua cổng USB gốc thì hết. Không đổi gì ngoài hàng đợi phát ở mục 4.

## 8. Việc phải làm sau khi nạp bản này

1. **Cấu hình `general` và `pid` về mặc định** (bố cục hai section này đổi). Receiver, motor, chân, blackbox, rates, hiệu chuẩn gia tốc kế và level trim được giữ. Sao lưu ở tab Hệ thống trước khi nạp thì khôi phục được phần còn khớp tên.
2. **Nhập lại "Hướng chip IMU"**, theo bảng. Kiểm tra bằng hình 3D ở tab Tổng quan: nghiêng máy sang phải thì hình nghiêng phải, chúi mũi thì hình chúi.

| Giá trị ở 0.5 | Giá trị tương đương bây giờ |
|---|---|
| `CW0` | `CW180_FLIP` |
| `CW90` | `CW90_FLIP` |
| `CW180` | `CW0_FLIP` |
| `CW270` | `CW270_FLIP` |
| `CW0_FLIP` | `CW180` |
| `CW90_FLIP` | `CW90` |
| `CW180_FLIP` | `CW0` |
| `CW270_FLIP` | `CW270` |

   `sensors.mag_align` (la bàn) **giữ nguyên con số nhưng đổi nghĩa** theo cùng bảng: nếu dùng la bàn thì đổi theo bảng rồi hiệu chuẩn lại.
3. Tháo cánh cho mọi phép thử dưới đây.

## 9. Kiểm chứng của lượt này

- `make test`: 18 chương trình C++, 3000 kiểm tra (bản 0.6.2). `make apptest`: client Python, CLI và trang thật trong trình duyệt với firmware mô phỏng.
- Mutation test (cố ý làm hỏng mã, test phải báo): số liệu của lượt chạy cuối ở `docs/HANDOFF.md`.
- Build ESP32-S3: `rx_s3`, `rx_s3_n8`, `rx_s3_usb`, và kiểu TinyUSB.

## 10. Cần bạn xem trên mạch, theo thứ tự (không cánh)

1. Nạp `rx_s3_usb`. Tab Tổng quan: **"Thời gian từng khâu"** (bốn số), "Chu kỳ điều khiển thực tế", số chu kỳ bỏ lỡ; lúc chưa arm và lúc arm. Nếu được, build thêm một bản với `-DWB_FAST_CODE=0` và gửi cả hai bộ số.
2. Hình 3D có đúng chiều không (sau khi nhập lại hướng chip).
3. Tab Motor, DShot một chiều: test từng motor. Không chạy thì đổi "Cách phát DShot" sang ESP-IDF và báo lại.
4. Bật DShot hai chiều: dòng trạng thái ghi gì? ESC nào không trả lời? "Khung bỏ qua" và "Lượt trả lời bị lỡ" là bao nhiêu? RPM có hiện không?
5. Arm, để ga thấp nhất 1 phút; rồi lắc cần và đưa ga về thấp nhất: motor có còn tự tăng không (Airmode = "Khi đang bay").
6. Blackbox: ghi trực tiếp qua USB, đồ thị có liền không; ghi vào flash rồi tải về, lỗi gì nếu có.
7. Cho biết: `ARDUINO_USB_MODE` bạn dùng là 0 hay 1; DShot300 hay 600; cảm biến nào đang nối I2C.
