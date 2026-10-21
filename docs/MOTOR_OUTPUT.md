# Motor output

## Cấu trúc
| File | Vai trò | Kiểm chứng |
|---|---|---|
| `hal/motor/motor_protocol.h` | Khung DShot + CRC, sóng RMT, GCR/eRPM/EDT, độ rộng xung PWM/Oneshot/Multishot, endpoints | 164 test PC, đối chiếu công thức Betaflight |
| `hal/motor/motor_output.h` | `MotorOutputManager`: disarm/idle/limit, reorder, lệnh DShot (đảo chiều, save, beacon), test slider, telemetry, dynamic idle | test PC + 12 lỗi cố ý đều bị bắt |
| `hal/motor/dshot_direct.h` | `DshotDirect<Port>`: DShot một chiều / hai chiều trên bộ nhớ và thanh ghi của RMT, không phụ thuộc phần cứng (mô phỏng được) | `test_dshot`: mô hình dây tín hiệu, 91 kiểm tra |
| `hal/motor/dshot.h` | `RmtDirectPort` (ESP32-S3: thanh ghi RMT, GPIO, một gptimer) + `DshotHw` (lối lùi qua driver ESP-IDF đã bỏ ở 0.12) | **biên dịch + link cho ESP32-S3**; chưa chạy trên mạch |
| `hal/motor/pwm_oneshot.h` | PWM / Oneshot125 / Oneshot42 / Multishot trên LEDC (tối đa 8 motor) | biên dịch + link S3; chưa chạy trên mạch |
| `hal/motor/motor.h` | `MotorSystem`: ghép manager + backend theo protocol | biên dịch S3 |

## Quy ước quan trọng
- Mixer xuất giá trị **tuyệt đối 0..1** (đã gồm idle). Đặt `mixer.idle = motors.idleFraction(...)` mỗi vòng và `mixer.output_limit = cfg.output_limit`. Manager là nơi duy nhất sở hữu motor idle (tĩnh + động).
- Đang arm thì motor **không bao giờ xuống dưới idle** (mixer lỡ ra 0 cũng không dừng motor giữa không trung). Disarm: DShot = 0, xung = giá trị "không arm" trong app (mặc định 0 = không phát xung).
- `reversed[]` không đổi mixer (giống Betaflight): nó gửi lệnh đảo chiều cho ESC, **luôn là 7/8 + save 12** (ESC lưu vào flash; AM32, BLHeli_S, Bluejay, BLHeli_32 đều nhận). Từ 0.18.1 không dùng 20/21 nữa (`dir_cmd_extended` bị bỏ qua): hai lệnh đó không được lưu và còn là chiều so với chiều đã lưu, nên AM32 quên đảo chiều sau mỗi lần FC khởi động lại (`docs/HARDWARE_FEEDBACK_4.md`). Chỉ gửi khi **disarm**, đúng nhịp Betaflight (chờ 10 ms, lặp 10 lần cách 1 ms). Arm thì mọi lệnh đang chờ bị huỷ.
- Motor reorder: `order[logical] = slot phần cứng`, sai hoán vị thì tự về mặc định. Chân của slot k = `HardwareConfig::getMotorPins()[k]`: chân app gửi (tab Chân và cổng), hoặc mặc định ghi trong `hardware_config.h`.
- Slider test: chỉ khi disarm, tự tắt sau 0,5 s nếu app ngừng refresh (app gửi ~10 Hz).
- Mất telemetry ≠ mất motor: rpm quá 100 ms thì RPM notch của **motor đó** mờ đi, dynamic idle về idle tĩnh.

## DShot trên ESP32-S3: driver "trực tiếp" (từ 0.6.0)

Bản 0.5 trên mạch thật: bật DShot hai chiều thì **motor không nhận lệnh nữa**. Hai nguyên nhân, cả hai đã tái hiện bằng mô hình:
1. Sau mỗi khung, mã gọi `pinMode(pin, INPUT_PULLUP)` để nghe ESC. Trên ESP-IDF 5.5, `gpio_output_disable()` đặt `oen_sel = 1` và xoá bit enable: chân bị **tách khỏi bộ phát RMT**, từ khung thứ hai không còn gì đi ra.
2. Bộ giải mã (`runs_to_gcr`) đòi 20 bit. Trả lời kết thúc bằng mức cao thì đoạn cuối lẫn vào đường dây rảnh và bộ thu báo "hết" không kèm độ dài: ~37 % trả lời hợp lệ bị loại. Nay nhận từ 18 bit (Betaflight).

Cách làm mới (`dshot_direct.h`), một chu kỳ gọi `send()`:
```
send():   thu hoạch trả lời của khung trước (đọc bộ nhớ RX của RMT → độ dài từng mức, tính bằng tick → GCR 20 bit)
          ghi 16 ký hiệu + dấu kết thúc vào bộ nhớ TX, chân ở chế độ đẩy-kéo, bắt đầu phát cả 4 kênh
          hai chiều: hẹn timer sau (thời gian khung + 3 µs)
ngắt timer (IRAM): nhả chân (ESC kéo đường dây), bật thu
```
| | Khung | Chu kỳ một chiều | Chu kỳ hai chiều (khung + 30 µs + trả lời + khoảng rảnh) |
|---|---|---|---|
| DShot150 | 107 µs | 137 µs | 312 µs |
| DShot300 | 54 µs | 84 µs | **182 µs** |
| DShot600 | 27 µs | 57 µs | **116 µs** |

`send()` được gọi sớm hơn một chu kỳ thì khung đó bị **bỏ qua** (đếm ở `stats().skipped`), không bao giờ ghi đè lên khung đang phát. Vòng 4 kHz (250 µs) hợp với DShot300 và DShot600 hai chiều; DShot150 hai chiều thì không.
Ngắt đến trễ (khung sau đã tới hạn mà chân chưa được nhả): trả lời đó bị bỏ, đếm ở `late_switch`. ESC không trả lời 20 khung liền: `rx_responsive(slot) = false`.

**0.12:** lối lùi qua `rmt_transmit` của ESP-IDF (`motors.dshot_driver = IDF`, chỉ một chiều, ~130 µs mỗi chu kỳ cho 4 motor) đã bị bỏ theo yêu cầu: tốn tài nguyên và không cần nữa. Byte đó trong settings giờ là `reserved1` để các trường sau giữ nguyên vị trí. Settings cũ đặt IDF thì nay chạy cách trực tiếp.

**Motor idle với giao thức analog (0.12):** PWM / Oneshot / Multishot / brushed không có "Motor idle" (giống Betaflight): ga thấp nhất khi đã arm là `pulse_armed` (min_throttle của Betaflight). Firmware bỏ qua `idle_x10` với các giao thức này, tab Motor ẩn ô đó. Trước 0.12 idle được cộng thêm trên `pulse_armed`, nên một máy analog cũ giờ idle thấp hơn: nếu motor dừng hoặc mất đồng bộ ở ga thấp, tăng `pulse_armed`.

Số liệu lên app (stream `motors`): `bidir` (OFF / ACTIVE / UNAVAILABLE), `answers` (bit i = ESC của motor i đang trả lời), `skipped`, `late` (trong giây vừa rồi).

**Chưa kiểm trên chip.** Mô hình (`test/test_dshot.cpp`) có ESC giải mã khung như ESC thật (độ rộng xung, khoảng lặng giữa hai khung, CRC đảo), trả lời GCR sau 30 µs, bộ thu có lọc nhiễu và ngưỡng rảnh, hai thiết bị cùng kéo đường dây thì bị bắt. Thứ mô hình không biết: điện trở kéo, độ trễ thật của ngắt timer trên core đang chạy vòng bay, ESC lệch nhịp.

## Kiểm tra lúc khởi động của DShot hai chiều (0.8.1)

Bản 0.8.0 trên mạch thật (ESC Aikon AK32, BLHeli_32): bật DShot hai chiều thì **không có telemetry và motor cũng không nhận lệnh**. Trên PC (mô hình dây tín hiệu, `test_dshot`) cả chuỗi chạy đúng, nên phần hỏng nằm ở thứ PC không mô phỏng được: định tuyến chân của bộ phát / bộ thu RMT qua GPIO matrix trên ESP-IDF 5.5, hoặc thời điểm của ngắt timer. Để tìm ra trên chính mạch đó:

- Lúc khởi động, với từng motor, firmware cho bộ thu nghe chân trong lúc bộ phát gửi một khung "0" (chân ESP32 đọc lại được mức nó đang phát), rồi nhả chân như lúc bay và giải mã 16 bit thu được. Đúng từng bit trên mọi chân → `motors: DShot/RMT direct, bidirectional (start-up check OK on 4 pins) x4`.
- Sai → **tự chuyển về DShot một chiều** (motor vẫn quay, vẫn kêu được) và dòng `motors:` ghi lý do: `bidirectional OFF: start-up check, the receiver of motor N heard nothing (k words)` (bộ phát không ra tới chân hoặc bộ thu không nghe được chân), `... not a DShot frame`, `... sent 0x...., heard 0x....`, hoặc `bidirectional OFF: <hàm ESP-IDF> failed (<mã lỗi>)` khi tạo kênh / timer bị lỗi.
- Tab Motor thêm "Trả lời nhận được / hỏng mỗi giây". Đọc cùng dòng trạng thái: kiểm tra khởi động OK nhưng **0 trả lời** → ESC không trả lời (firmware ESC, hoặc ESC chưa nhận ra tín hiệu đảo); nhiều trả lời **hỏng** → thời điểm / bộ lọc của bộ thu; kiểm tra OK, có trả lời mà motor không quay → báo lại kèm hai dòng đó.
- Kiểm chứng trên PC: `test_dshot` `[start-up check]` (4 tốc độ DShot × 2 độ phân giải: đọc lại đúng từng bit trên mọi chân; ESC trả lời khung kiểm tra khi chân đã nhả, không giành dây; bộ phát không ra chân / bộ thu điếc → "heard nothing"). **Kết quả trên mạch thật: chưa có.**

### 0.8.2: đổi chân kiểu esp-fc
So với driver ESC của esp-fc (`lib/EscDriver/src/EscDriverEsp32.cpp`, chạy được DShot hai chiều trên ESP32-S3), khác biệt chính là cách đổi chiều chân. esp-fc **nối lại tín hiệu qua GPIO matrix mỗi lần đổi**: khi phát thì tắt đường vào, nối tín hiệu của bộ phát RMT, bật output (`enableTx`); khi nghe thì tắt output, bật đường vào, nối tín hiệu của bộ thu (`enableRx`). Bản 0.6 - 0.8.1 để cả hai tín hiệu nối thường trực và chỉ bật / tắt output enable. Từ 0.8.2 `RmtDirectPort::drive()` / `release()` làm đúng như esp-fc (thêm: output enable lấy từ bit GPIO_ENABLE, `OEN_SEL = 1`, để tắt output chắc chắn nhả dây). DShot **một chiều giữ nguyên** như cũ (đường đã chạy được trên mạch). Các thứ khác của esp-fc mình đã có tương đương: khung đảo + checksum đảo, chân kéo lên khi ESC nói, bộ lọc / ngưỡng rảnh của bộ thu, giải mã GCR (`value ^ (value >> 1)`). Khác còn lại: esp-fc bật bộ thu từ ngắt "phát xong" của RMT (driver cũ `driver/rmt.h`, không dùng chung được với driver mới mà Arduino-ESP32 3.x dùng); ở đây là một timer bắn 3 µs sau khung.

### 0.8.4 (đã gỡ ở 0.8.6)
0.8.4 thêm một ngắt thứ hai mỗi khung để đẩy chân lên cao lại sau cửa sổ trả lời, đo mức dây và đọc lại khung khi đang chạy (theo `docs/HARDWARE_FEEDBACK_3.md`). Trên mạch, vòng lặp tụt từ ~5000 xuống ~4300 Hz, nên **0.8.6 trả `dshot_direct.h` về trình tự của 0.8.2 / 0.8.3** (đổi chân kiểu esp-fc, một ngắt mỗi khung, nhả chân tới khung sau); các trường `line_low` / `rb_ok` / `rb_bad` và hai dòng chẩn đoán ở tab Motor cũng gỡ theo. Thay cho việc giữ dây cao bằng phần mềm: điện trở kéo lên ngoài (mục dưới), dây nhả ra là lên cao ngay.

### 0.8.5: điện trở kéo lên ngoài
`motors.ext_pullup` (tab Motor, "Có điện trở kéo lên ngoài"; lưu và khởi động lại): mạch đã có điện trở kéo dây tín hiệu lên 3,3 V (thường 1 – 4,7 kΩ), firmware **tắt** pull-up nội (~45 kΩ) của chân motor khi chạy DShot hai chiều. Dòng `motors:` của báo cáo khởi động ghi "internal / external pull-up". Với 2 kΩ: kéo dây xuống thấp tốn 1,65 mA, ESC kéo được dễ dàng; và dây nhả ra lên mức cao trong khoảng một phần mười micro giây thay vì vài micro giây. Kiểm chứng: `test_fw` nhóm `rxdiag` (cài đặt tới được driver). **Chưa thử trên mạch.**

### 0.18.2: bộ thu không được thấy cạnh nào trước bit start
Hiện tượng trên mạch (AM32): điện áp / dòng từ EDT vẫn có, rpm hầu như luôn 0, thỉnh thoảng mới có số. Điện áp được giữ 1 s, nhiệt độ giữ mãi, còn rpm chỉ hợp lệ 100 ms sau lần nhận cuối: chỉ cần vài phần trăm reply nhận đúng là điện áp vẫn hiện, còn rpm thì tắt liên tục. Vậy phần lớn reply bị mất, eRPM và EDT như nhau.

Nguyên nhân tìm được khi đọc lại trình tự (chưa đo trên mạch): đến 0.18.1, đường vào của chân bị tắt lúc phát (`PIN_INPUT_DISABLE`, chân đọc ra 0) và bật lại lúc nhả chân, rồi bộ thu khởi động ngay sau đó. Bước 0 → 1 đó đi qua bộ lọc nhiễu của RMT (¼ bit trả lời, 333 – 667 ns) nên tới bộ thu **sau** khi nó đã chạy, nhất là ở các kênh khởi động sau cùng. Bộ thu coi đó là cạnh đầu tiên, đo mức cao 27 µs trước reply, dài hơn ngưỡng rảnh (8 bit = 10,7 µs ở DShot600), và **dừng trước khi ESC kịp trả lời**. esp-fc cũng làm theo kiểu này; có thể nó chạy được nhờ ngưỡng rảnh khác, nhưng không chắc.

Sửa (khâu thu viết lại theo cách Betaflight làm trên STM32: thu cả cửa sổ, tìm bit start trong đó):
- Đường vào của chân bật **một lần** ở `begin()` (`listen()`) và để nguyên. Chân ESP32 đọc lại mức nó đang phát; bộ thu dừng trong lúc mình phát nên không ảnh hưởng. `drive()` vẫn nối lại tín hiệu phát và bật output như esp-fc (bài học 0.8.2), `release()` chỉ tắt output.
- Bộ thu khởi động **trước** khi nhả chân, lúc dây vẫn đang được giữ ở mức cao: lúc nhả chân cũng không có cạnh nào.
- Giải mã `reply_from_runs()` (`motor_protocol.h`), theo `dshot_bitbang_decode.c` / `pwm_output_dshot_shared.c`: bỏ qua mọi thứ trước bit start (mức rảnh, xung nhiễu ngắn hơn nửa bit), các đoạn phải cộng lại 18 – 21 bit; chỉ đoạn mức **cao** được vượt bit 21 (đó là dây rảnh sau reply), đoạn mức thấp vượt bit 21 thì không phải reply (Betaflight: `bits != 21`). Một reply đúng luôn kết thúc ở mức cao (checksum cố định số lần đổi mức).
- Đếm riêng lý do hỏng, gửi lên app mỗi giây (stream `motors`): `no_start` (cửa sổ không có bit start: vấn đề thời gian / đổi chân), `bad_bits` (có bit start nhưng số bit sai: nhiễu, cạnh chậm), `bad_crc` (bit sạch, checksum sai). Một lần thu không có bit start không tính là ESC trả lời.
- App: cột RPM hiện "–" khi rpm cũ hơn 100 ms (`rpm_ok`), không hiện 0 nữa (0 dễ đọc thành "motor đứng").

Rủi ro cần thử trên mạch: đường vào bật cả lúc phát là khác với esp-fc. Về điện thì nó chỉ bật bộ đệm vào, và kiểm tra lúc khởi động (`loopback()`) đã phát với đường vào bật từ 0.8.1. Nếu ESC không nhận khung nữa khi bật DShot hai chiều (motor không quay, không kêu) thì nguyên nhân là thay đổi này.

## Motor wizard (0.20, như Betaflight Configurator 4.3+)
Tab Motor, khối "Motor wizard": thứ tự motor và chiều quay được tìm bằng cách nhìn motor quay, không phải gõ số slot. Cột "Slot phần cứng" và công tắc "Đảo chiều" trong bảng "Từng motor" vẫn còn để sửa tay.

- **Điều kiện**: đã kết nối, chưa arm, đã đánh dấu "Đã tháo hết cánh quạt", và không có cài đặt nào đang chờ khởi động lại. Wizard chạy trên cấu hình motor đang chạy; đổi giao thức hay kiểu khung thì phải lưu và khởi động lại trước.
- **Sắp xếp lại thứ tự**: lần lượt từng đầu ra quay chậm (ga đặt được, mặc định 8 %). Bạn bấm vào vị trí của motor đang quay trên hình. Sau motor cuối, app ghi `motors.order` mới. Firmware dùng thứ tự đó **ngay**, không cần khởi động lại; bấm "Lưu" để giữ.
- **Chiều quay** (chỉ DShot): bấm vào một motor để nó quay (hoặc chọn "Quay tất cả"), rồi so với mũi tên trên hình. Quay ngược thì bấm "Đảo chiều motor này" (ở chế độ "Quay tất cả" thì bấm thẳng vào motor). App gửi lệnh 7 / 8 + 12 cho ESC và đặt `motors.reversed_mask`.
- **An toàn**: lệnh test được làm mới mỗi 200 ms, firmware tự dừng motor nếu 0,5 s không được làm mới. Motor dừng khi bấm "Dừng", khi rời tab, khi cửa sổ bị ẩn, hoặc khi máy arm.

Firmware: `FlightApp::apply_motor_order()`. Khi section `motors` được ghi, nếu so với lúc khởi động chỉ có `order` / `reversed_mask` đổi, thì thứ tự và chiều được áp ngay (`MotorOutputManager::set_order()`). Thứ tự phải là hoán vị của các motor, nếu không thì giữ thứ tự cũ. Có field khác đổi thì không áp gì cả và báo cần khởi động lại, như trước. Khi thứ tự thật sự đổi, lệnh test và lệnh DShot đang chờ (nhắm vào slot cũ) bị bỏ.
Kiểm chứng: `test_motor` (`set_order`), `test_fw` nhóm motor test (thứ tự mới chạy ngay, slot đúng quay, thứ tự sai bị từ chối, đổi giao thức thì không áp), 7 mutant `wizard:`. Trang thật: `test_gui_browser.py::test_motor_wizard` (viết nhưng chưa chạy được trên máy này). **Chưa thử trên mạch.**

## Giao thức (0.8.2)
- **ProShot1000** (Betaflight `loadDmaBufferProshot`): cùng gói 16 bit như DShot, gửi thành 4 xung nibble (nibble cao trước), mỗi xung trong ô 4 µs, mức cao 1 µs + nibble × 125 ns; 16 µs một khung. Chỉ một chiều. Mang được lệnh DShot (beacon, chiều quay...), như Betaflight coi ProShot là một giao thức DShot. Cần ESC hỗ trợ (BLHeli_32 có; BLHeli_S / Bluejay không).
- **Brushed**: motor chổi than qua MOSFET, LEDC 16 kHz, 1000..2000 = 0..100 % duty.
- DShot / ProShot: tối đa **4 motor** (4 bộ phát RMT của ESP32-S3), kể cả trên PC (`MotorSystem::kMaxDigitalMotors`); khung 6 / 8 motor dùng PWM / Oneshot / Multishot (LEDC, 8 kênh). Báo cáo khởi động nói rõ lý do.

## Khung (layout, 0.8.2)
Ngoài Quad X, Quad +, Hexa X (của bản này): các bảng của Betaflight (`mixer_init.c`, thứ tự motor và chiều cánh như Betaflight): Hexa + (HEX6), Hexa X thứ tự Betaflight (HEX6X), Hexa H, Y6, Y4, Octo X8, Octo phẳng + / X, V-tail, A-tail, và Quad X thứ tự 1234 (esp-fc). Mỗi trục chuẩn hoá để hệ số lớn nhất = 1 như ba khung cũ. `test_core` kiểm từng khung có hình học (không có V-tail / A-tail: motor nghiêng) với mô hình vật lý dựng từ vị trí motor và chiều cánh: lệnh roll / pitch / yaw quay đúng chiều, không lẫn sang trục khác, không đổi tổng lực đẩy (Y4: Betaflight để lẫn một chút sang yaw, chỉ kiểm chiều và tổng lực). Sơ đồ ở tab Motor vẽ motor dưới của cặp đồng trục lệch vào trong, có dấu ↓. **Chưa bay thử khung nào ngoài quad.**

## Giới hạn hiện tại (nói thẳng)
1. **DShot tối đa 4 motor** trên S3 (RMT có 4 kênh TX, 4 kênh RX). Hexa / octo cần backend xuất song song (I2S / LCD-CAM), chưa làm. Xung PWM / Oneshot / Multishot (LEDC) tới 8 motor.
2. **DShot hai chiều chưa được thử trên silicon** với driver mới (xem trên). Nếu không ra rpm, motor vẫn chạy; nếu motor không chạy, chọn driver ESP-IDF.
3. Multishot 5 – 25 µs, Oneshot42 = giá trị / 24 là định nghĩa giao thức phổ biến; file pwm_output của Betaflight trong zip không có hằng số này để đối chiếu.
4. Pin qua ESC (EDT voltage) cần ESC hỗ trợ và bật "Extended DShot telemetry".
5. DShot300 là mặc định (dễ tính hơn 600 với dây dài).
6. Xung lúc chưa arm của PWM / Oneshot / Multishot: `pulse_disarmed`, **0 = 1000 µs** (từ 0.18.1, `mincommand` của Betaflight; thấp hơn nếu `pulse_armed` < 1000). Không bao giờ "không phát": ESC chỉ arm sau khi thấy xung ga 0, và AM32 thấy ga cao trước (motor test) thì vào hiệu chuẩn ga.
7. **AM32 chỉ nhận DShot và Servo PWM** (800 – 2200 µs), không nhận Oneshot125 / Oneshot42 / Multishot (`signal.c: detectInput`). Các giao thức đó dùng với BLHeli_S / BLHeli_32 / KISS.
8. LEDC ghi thẳng thanh ghi (`port/pwm.h`, `hal/ledc_ll.h`, 0.18.1) thay vì `ledcAttach` / `ledcWrite` của Arduino: không còn gọi hàm trong flash mỗi chu kỳ. Các đầu ra cùng tần số + độ phân giải dùng chung một trong 4 timer.

## Kiểm tra trên mạch (theo thứ tự, không lắp cánh)
1. Tab Tổng quan, dòng Motor / ESC: `DShot/RMT direct` hay `DShot/RMT idf` hay `Pulse/LEDC`.
2. Test từng motor ở ~5 %: đúng motor, đúng chiều? Nút "Kêu" cho biết motor nào là slot nào.
3. Đảo chiều bằng công tắc "Đảo chiều" (lệnh DShot, chỉ khi chưa arm): BLHeli_S / Bluejay / BLHeli_32 dùng lệnh 7 / 8, AM32 dùng được cả 20 / 21.
4. Bật DShot hai chiều, lưu, khởi động lại: dòng trạng thái dưới bảng motor ghi gì; cột RPM và "Lỗi telemetry"; "Khung bỏ qua", "Lượt trả lời bị lỡ". Gửi lại các số đó nếu không phải "đang chạy, mọi ESC đều trả lời".
