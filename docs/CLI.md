# Command line - CLI (0.16.0)

Giống CLI của Betaflight, dùng cổng serial của app (USB / UART0). Mở một terminal (PuTTY, `screen`, Serial Monitor của Arduino IDE …), chọn đúng cổng và baud của link (USB thì baud nào cũng được; UART0 là 230400), gõ `#` rồi Enter để vào. Khi đang dùng CLI thì đóng app lại, vì app và terminal không dùng chung một cổng được.

```
get [một phần tên]          get loop_hz   → general.loop_hz = 4000, allowed: 500 .. 8000
set <section.field> = <giá trị>
    set general.loop_hz = 8000
    set general.board_trim_roll = -2.5     (đơn vị như trong app: độ, không phải 0.1 độ)
    set general.board_align = CW90_FLIP    (enum: theo tên, không phân biệt hoa thường)
    set general.runaway_takeoff = OFF      (bool: ON / OFF)
    set pins.motor = 4,5,6,7,255,255,255,255   |   set pins.motor[2] = 16
    set beeper.conditions = GYRO_CALIBRATED,RX_LOST   (tập bit: theo tên, hoặc NONE)
    set wifi.ssid = "Nhà tôi"               (chuỗi: để trong dấu ngoặc kép, UTF-8)
diff [section]   các dòng set của những gì khác mặc định (dán lại vào CLI là có lại đúng cấu hình đó)
dump             mọi setting, dạng dòng set
defaults [section]   về mặc định (chỉ trong RAM: gõ save để giữ)
save   reboot   status   version   help   exit
```

- Một thay đổi đi qua đúng các bước như khi đổi từ app (`wbp::Server::commit_section`): kẹp vào giới hạn, tính lại các giá trị suy ra, áp dụng ngay hoặc đánh dấu "cần khởi động lại". Sau `set`, CLI in lại giá trị mà firmware thật sự giữ.
  - Ví dụ: `pid.p` bị thanh trượt tính đè lên nếu `pid.slider_mode` không phải chỉnh tay, như trong app.
- Khi máy đang arm thì không `set` / `save` / `defaults` được.
- `wifi.pass` (trường bí mật) luôn in ra thành `<hidden>`, kể cả trong `diff` / `dump`. Muốn dán lại thì phải gõ mật khẩu bằng tay.
- Chân (pins) hiện bằng số: GPIO trên ESP32; trên STM32 là cổng × 16 + chân (PB3 = 19).
- Bảng tên field được sinh ra từ schema (`tools/gen_proto.py` → `src/proto/wbp_fields.h`). Thêm setting mới vào schema thì CLI có luôn.
- Chỉ mở CLI khi `#` đến **giữa hai khung** (giao thức WBP hay MSP), nên app và BLHeliSuite không bao giờ vô tình mở nó.
- Khi đang trong CLI, firmware không gửi stream ra cổng. `exit` để trở lại giao thức của app.

## Kiểm thử
- `test_cli`:
  - bảng field: offset của từng field khớp đúng struct;
  - `get` / `set` theo từng kiểu (số có scale, enum, bool, mảng, phần tử mảng, tập bit, chuỗi, trường bí mật);
  - các giá trị bị từ chối;
  - `diff` dán sang một mạch khác ra đúng cấu hình;
  - `dump`; khi đang arm; section live; `save` rồi nạp lại;
  - `defaults`; flash lỗi;
  - echo, backspace, CR LF.
- Mutant: `python3 tools/mutate.py cli:` (9).
- Phần đi qua cổng serial thật (`LinkTask`) mới chỉ biên dịch, **chưa thử với terminal trên mạch**.
