# Motor output

## Cấu trúc
| File | Vai trò | Kiểm chứng |
|---|---|---|
| `hal/motor/motor_protocol.h` | Khung DShot + CRC, sóng RMT, GCR/eRPM/EDT, độ rộng xung PWM/Oneshot/Multishot, endpoints | 164 test PC, đối chiếu công thức Betaflight |
| `hal/motor/motor_output.h` | `MotorOutputManager`: disarm/idle/limit, reorder, lệnh DShot (đảo chiều, save, beacon), test slider, telemetry, dynamic idle | test PC + 12 lỗi cố ý đều bị bắt |
| `hal/motor/dshot_direct.h` | `DshotDirect<Port>`: DShot một chiều / hai chiều trên bộ nhớ và thanh ghi của RMT, không phụ thuộc phần cứng (mô phỏng được) | `test_dshot`: mô hình dây tín hiệu, 91 kiểm tra |
| `hal/motor/dshot.h` | `RmtDirectPort` (ESP32-S3: thanh ghi RMT, GPIO, một gptimer) + `DshotHw`; lối lùi dùng driver RMT của ESP-IDF (một chiều) | **biên dịch + link cho ESP32-S3**; chưa chạy trên mạch |
| `hal/motor/pwm_oneshot.h` | PWM / Oneshot125 / Oneshot42 / Multishot trên LEDC (tối đa 8 motor) | biên dịch + link S3; chưa chạy trên mạch |
| `hal/motor/motor.h` | `MotorSystem`: ghép manager + backend theo protocol | biên dịch S3 |

## Quy ước quan trọng
- Mixer xuất giá trị **tuyệt đối 0..1** (đã gồm idle). Đặt `mixer.idle = motors.idleFraction(...)` mỗi vòng và `mixer.output_limit = cfg.output_limit`. Manager là nơi duy nhất sở hữu motor idle (tĩnh + động).
- Đang arm thì motor **không bao giờ xuống dưới idle** (mixer lỡ ra 0 cũng không dừng motor giữa không trung). Disarm: DShot = 0, xung = giá trị "không arm" trong app (mặc định 0 = không phát xung).
- `reversed[]` không đổi mixer (giống Betaflight): nó gửi lệnh đảo chiều cho ESC (mặc định lệnh 7/8 + save 12, đặt `dir_cmd_extended` để dùng 20/21). Chỉ gửi khi **disarm**, đúng nhịp Betaflight (chờ 10 ms, lặp 10 lần cách 1 ms). Arm thì mọi lệnh đang chờ bị huỷ.
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

Lối lùi: `motors.dshot_driver = IDF` (tab Motor, "Cách phát DShot") dùng `rmt_transmit` của ESP-IDF, **chỉ một chiều**; bật hai chiều với driver này thì trạng thái là "đã bật nhưng cách phát đang chọn không hỗ trợ".

Số liệu lên app (stream `motors`): `bidir` (OFF / ACTIVE / UNAVAILABLE), `answers` (bit i = ESC của motor i đang trả lời), `skipped`, `late` (trong giây vừa rồi).

**Chưa kiểm trên chip.** Mô hình (`test/test_dshot.cpp`) có ESC giải mã khung như ESC thật (độ rộng xung, khoảng lặng giữa hai khung, CRC đảo), trả lời GCR sau 30 µs, bộ thu có lọc nhiễu và ngưỡng rảnh, hai thiết bị cùng kéo đường dây thì bị bắt. Thứ mô hình không biết: điện trở kéo, độ trễ thật của ngắt timer trên core đang chạy vòng bay, ESC lệch nhịp.

## Kiểm tra lúc khởi động của DShot hai chiều (0.8.1)

Bản 0.8.0 trên mạch thật (ESC Aikon AK32, BLHeli_32): bật DShot hai chiều thì **không có telemetry và motor cũng không nhận lệnh**. Trên PC (mô hình dây tín hiệu, `test_dshot`) cả chuỗi chạy đúng, nên phần hỏng nằm ở thứ PC không mô phỏng được: định tuyến chân của bộ phát / bộ thu RMT qua GPIO matrix trên ESP-IDF 5.5, hoặc thời điểm của ngắt timer. Để tìm ra trên chính mạch đó:

- Lúc khởi động, với từng motor, firmware cho bộ thu nghe chân trong lúc bộ phát gửi một khung "0" (chân ESP32 đọc lại được mức nó đang phát), rồi nhả chân như lúc bay và giải mã 16 bit thu được. Đúng từng bit trên mọi chân → `motors: DShot/RMT direct, bidirectional (start-up check OK on 4 pins) x4`.
- Sai → **tự chuyển về DShot một chiều** (motor vẫn quay, vẫn kêu được) và dòng `motors:` ghi lý do: `bidirectional OFF: start-up check, the receiver of motor N heard nothing (k words)` (bộ phát không ra tới chân hoặc bộ thu không nghe được chân), `... not a DShot frame`, `... sent 0x...., heard 0x....`, hoặc `bidirectional OFF: <hàm ESP-IDF> failed (<mã lỗi>)` khi tạo kênh / timer bị lỗi.
- Tab Motor thêm "Trả lời nhận được / hỏng mỗi giây". Đọc cùng dòng trạng thái: kiểm tra khởi động OK nhưng **0 trả lời** → ESC không trả lời (firmware ESC, hoặc ESC chưa nhận ra tín hiệu đảo); nhiều trả lời **hỏng** → thời điểm / bộ lọc của bộ thu; kiểm tra OK, có trả lời mà motor không quay → báo lại kèm hai dòng đó.
- Kiểm chứng trên PC: `test_dshot` `[start-up check]` (4 tốc độ DShot × 2 độ phân giải: đọc lại đúng từng bit trên mọi chân; ESC trả lời khung kiểm tra khi chân đã nhả, không giành dây; bộ phát không ra chân / bộ thu điếc → "heard nothing"). **Kết quả trên mạch thật: chưa có.**

## Giới hạn hiện tại (nói thẳng)
1. **DShot tối đa 4 motor** trên S3 (RMT có 4 kênh TX, 4 kênh RX). Hexa / octo cần backend xuất song song (I2S / LCD-CAM), chưa làm. Xung PWM / Oneshot / Multishot (LEDC) tới 8 motor.
2. **DShot hai chiều chưa được thử trên silicon** với driver mới (xem trên). Nếu không ra rpm, motor vẫn chạy; nếu motor không chạy, chọn driver ESP-IDF.
3. Multishot 5 – 25 µs, Oneshot42 = giá trị / 24 là định nghĩa giao thức phổ biến; file pwm_output của Betaflight trong zip không có hằng số này để đối chiếu.
4. Pin qua ESC (EDT voltage) cần ESC hỗ trợ và bật "Extended DShot telemetry".
5. DShot300 là mặc định (dễ tính hơn 600 với dây dài).
6. Xung lúc chưa arm của PWM / Oneshot mặc định là **0 (không phát)** theo đặc tả. Nhiều ESC analog chỉ khởi động khi thấy xung thấp nhất: đặt `pulse_disarmed = 1000`.

## Kiểm tra trên mạch (theo thứ tự, không lắp cánh)
1. Tab Tổng quan, dòng Motor / ESC: `DShot/RMT direct` hay `DShot/RMT idf` hay `Pulse/LEDC`.
2. Test từng motor ở ~5 %: đúng motor, đúng chiều? Nút "Kêu" cho biết motor nào là slot nào.
3. Đảo chiều bằng công tắc "Đảo chiều" (lệnh DShot, chỉ khi chưa arm): BLHeli_S / Bluejay / BLHeli_32 dùng lệnh 7 / 8, AM32 dùng được cả 20 / 21.
4. Bật DShot hai chiều, lưu, khởi động lại: dòng trạng thái dưới bảng motor ghi gì; cột RPM và "Lỗi telemetry"; "Khung bỏ qua", "Lượt trả lời bị lỡ". Gửi lại các số đó nếu không phải "đang chạy, mọi ESC đều trả lời".
