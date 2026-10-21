# Phản hồi từ mạch thật lần 4 (0.18, ESC AM32): 7 mục khẩn cấp → 0.18.1

Tất cả được thử bằng motor test trong tab Motor, ESC chạy AM32. Tài liệu tham chiếu:
- Betaflight: `dshot*.c`, `pwm_output*.c`, `motor.c`, `pwm_motor_esp32.c` (thư mục `dshotandpwm/`), `io/beeper.c`, `io/serial_4way*.c`;
- AM32 bản hiện hành: `Src/signal.c`, `Src/dshot.c`.

| # | Hiện tượng | Nguyên nhân | Sửa trong 0.18.1 | Trạng thái |
|---|---|---|---|---|
| 1 | DShot hai chiều: rpm thường về 0, thỉnh thoảng mới có số; điện áp / dòng (EDT) vẫn có | Bộ giải mã và cách phân loại eRPM / EDT giống Betaflight. Điện áp được giữ 1 s, nhiệt độ giữ mãi, rpm chỉ hợp lệ 100 ms: vài phần trăm reply đúng là đủ để điện áp vẫn hiện, nên phần lớn reply đang bị mất. Nguyên nhân tìm được khi đọc lại trình tự (0.18.2): đường vào của chân bật lúc nhả chân, bước 0 → 1 qua bộ lọc nhiễu tới bộ thu sau khi nó đã chạy; bộ thu đo 27 µs mức cao trước reply, quá ngưỡng rảnh, và dừng trước khi ESC trả lời | **0.18.2**: đường vào bật một lần và để nguyên, bộ thu chạy trước khi nhả chân (không còn cạnh nào trước bit start); giải mã kiểu Betaflight (tìm bit start trong cửa sổ, 18 – 21 bit); đếm lý do hỏng (`no_start`, `bad_bits`, `bad_crc`); app hiện "–" khi rpm cũ (`docs/MOTOR_OUTPUT.md`, mục 0.18.2) | **cần board** |
| 2 | ESC passthrough: AM32 Configurator kết nối được nhưng không đọc được ESC | Chưa tìm ra. Lớp 4-way và bootloader khớp với Betaflight: nhận dạng ARM, `INIT_FLASH`, đọc, CRC, `RESET`, thời gian bit 52 µs | Thêm **bản ghi phiên**: mỗi lệnh 4-way kèm ACK và dữ liệu đầu, ghi vào RAM RTC (còn sau lần khởi động lại khi kết thúc phiên), hiện trong báo cáo khởi động lần sau, dòng `4way: …` | **cần board** |
| 3 | Oneshot / Multishot: kéo test motor thì motor kêu liên tục, không quay | (a) **AM32 chỉ nhận DShot và Servo PWM 800 – 2200 µs** (`signal.c: detectInput` chỉ có `checkDshot` / `checkServo`): Oneshot125 / 42 / Multishot không chạy được với AM32, firmware nào cũng vậy. (b) Lỗi firmware: chưa arm thì **không phát xung** (`pulse_disarmed = 0`), mà AM32 chỉ arm sau khi đếm đủ xung ga 0 (`zero_input_count`). Thấy ga cao trước (motor test > 1500 µs) thì nó vào **hiệu chuẩn ga** | `pulse_disarmed = 0` giờ là 1000 µs (`mincommand` của Betaflight; thấp hơn nếu `pulse_armed` < 1000). Lúc chưa arm luôn có xung | PC đã kiểm; **cần board**: PWM với AM32 |
| 4 | Pin 4S ở 16 V bị nhận là 1S | Số cell chỉ đếm một lần, khi điện áp vừa "ổn định" trong vùng 3,0 – 4,3 V hoặc trên 6 V (giống Betaflight). Điện áp từ telemetry ESC lên từng bậc, dừng một lúc quanh 4 V, đủ "ổn định" để chốt 1S | Một cell không vượt được `cell_max_v`: khi điện áp ổn định vượt `số cell × cell_max_v` thì đếm lại. Chỉ tăng, không giảm (sụt áp khi bay không làm mất cell). Số cell đặt tay thì không đếm lại | PC đã kiểm (test tái hiện lỗi: không có sửa thì ra 1S) |
| 5 | Đảo chiều motor: lúc đầu chạy, khởi động lại FC thì mất, phải bật tắt lại | `dir_cmd_extended` (lệnh 20 / 21, mà tài liệu cũ khuyên dùng cho AM32) chỉ đổi biến tạm `forward` của AM32, **không bao giờ được lưu**, và còn là chiều *so với chiều đã lưu*. FC khởi động lại thì ESC mất tín hiệu, khởi động lại theo chiều đã lưu. Lệnh 7 / 8 + 12 thì đổi `dir_reversed` và lưu flash (AM32, BLHeli_S, Bluejay, BLHeli_32 đều vậy) | Luôn gửi 7 / 8 + 12 (save). `dir_cmd_extended` không còn dùng, bỏ khỏi tab Motor | PC đã kiểm; **cần board** |
| 6 | Beacon DShot phát đúng pattern nhưng nốt nào cũng dài như nhau | DShot chỉ có lệnh "beacon 1 – 5", không có độ dài: ESC tự quyết (Betaflight: "the DShot Beacon tone duration is determined by the ESC, and should not exceed 250ms"). Firmware cũ còn ép mọi phần "bật" thành đúng 450 ms nên lệch nhịp | Phần "bật" ngắn hơn 450 ms vẫn kéo lên 450 ms (để tiếng sau không bị bỏ), phần dài hơn giữ nguyên độ dài, nên nhịp giống còi. Ví dụ RX_LOST 500 / 500 ms: một tiếng mỗi 1001 ms (trước là 951 ms) | PC đã kiểm. Độ dài từng nốt thì không đổi được: giới hạn của giao thức |
| 7 | Thay LEDC API bằng `ledc_ll` | `ledcWrite()` của Arduino là hàm trong flash, lấy khoá và tìm chân mỗi lần ghi. Motor PWM ghi mỗi chu kỳ | `port/pwm.h` (ESP32): ghi thẳng thanh ghi LEDC low-speed (8 kênh, 4 timer dùng chung theo tần số + độ phân giải) như `pwm_motor_esp32.c` của Betaflight. Motor, servo, còi thụ động cùng dùng. `PulseHw::write` vào IRAM | Build ESP32 OK; **cần board**: PWM motor, servo, còi |

Ngoài ra: Makefile thêm `-MT $@`. Trên Windows, file phụ thuộc trước đây có tên `x.exe` nên đổi header không làm test build lại. Một test (`test_dshot`) không biên dịch được từ 0.17.0 (thiếu `dshot_kbps`) mà vẫn "pass" bằng binary cũ; đã sửa.

## Cần đo trên mạch
1. **Mục 1, DShot hai chiều.** Tab Motor, motor test ở 10 %, 30 %, 50 %, ghi lại:
   - cột rpm và "lỗi %" từng motor;
   - dòng replies / garbled mỗi giây;
   - thử với EDT bật và tắt, DShot300 và DShot600;
   - pull-up trong và ngoài (`motors.ext_pullup` + điện trở 2 kΩ);
   - dòng `motors:` trong báo cáo khởi động (kết quả "start-up check").

   Đọc số liệu (0.18.2, dòng dưới "Trả lời nhận được / hỏng"):
   - cột RPM "–": không có rpm mới trong 100 ms; 0: ESC báo "motor dừng" (`0xFFF`, AM32 khi `running == 0`);
   - `không có bit start` cao: vẫn là thời gian / đổi chân (gửi kèm tần số vòng và DShot300 / 600);
   - `sai số bit` cao: dây nhiễu hoặc cạnh chậm (thử điện trở kéo lên ngoài);
   - `sai checksum` cao: lệch xung nhịp hoặc nhiễu trong reply.
   - Nếu bật DShot hai chiều mà motor không quay / không kêu nữa: đường vào bật cả lúc phát không hợp với mạch, báo lại ngay.
2. **Mục 2, passthrough.** Kết nối AM32 Configurator, bấm đọc ESC, đóng lại. Mở app Windburst, gửi các dòng `4way: …` trong báo cáo khởi động (tab Tổng quan).
3. **Mục 3.** Với AM32 dùng `motors.protocol = PWM`: motor test phải quay, và ESC kêu tiếng sẵn sàng ngay khi có nguồn.
4. **Mục 5.** Đảo chiều motor 2, khởi động lại FC (nút Reboot trong app): motor 2 vẫn quay ngược.
5. **Mục 4.** Pin 4S, nguồn điện áp chỉ từ ESC (`battery.source = ESC`): app hiện 4 cell.

## Kiểm thử
- `test_motor`: xung lúc chưa arm (0 → 1000 µs, đặt tay, `pulse_armed` thấp hơn), lệnh đổi chiều luôn 7 / 8 + 12 kể cả khi `dir_cmd_extended` bật.
- `test_power`: nguồn dừng ở 4 V rồi lên 16,4 V ra 4S, không cảnh báo trên đường lên; sụt áp không làm mất cell; số cell đặt tay không đổi; nhịp beacon mới.
- `test_fw`: nhịp beacon khi mất sóng (1001 ms).
- `test_esc4way`: bản ghi phiên (nội dung, vòng giữ 12 đầu + 12 cuối, giới hạn số dòng, bộ nhớ chưa ghi không bị coi là bản ghi).
- Mutant: `python3 tools/mutate.py fix181`, 13 mutant đều bị bắt.
