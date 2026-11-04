# Pin và tiếng báo (còi / motor) — `src/hal/battery`, `src/hal/beeper`

Hành vi lấy theo Betaflight (`sensors/battery.c`, `sensors/voltage.c`, `io/beeper.c`) để các con số phi công đã quen có cùng ý nghĩa.

## Pin

```
src/hal/battery/battery_monitor.h   batt::Monitor: điện áp → điện áp đã lọc, số cell, trạng thái      [portable]
src/hal/battery/battery_base.h      BatterySensor: chọn nguồn (ADC / ESC), đọc ADC, gọi Monitor 50 Hz
```
- **Nguồn đo** (`battery.source`): `AUTO` (ADC nếu có chân `pins.batt_adc`, không thì telemetry ESC), `ADC`, `ESC`, `NONE`.
  ADC đọc bằng `analogReadMilliVolts()` (đã hiệu chỉnh của Arduino-ESP32) rồi nhân `battery.divider` (= điện áp pin / điện áp ở chân; cầu 10k : 1k là 11).
- **Lọc**: PT1 1/3 Hz ở 50 Hz, nên một cú đạp ga không trông như pin cạn.
- **Có pin hay không**: điện áp đã lọc ổn định (10 trong 11 khoảng 100 ms không lệch quá 0,1 V) và hợp lý: 3,0 V … một cell đầy, hoặc trên 6,0 V. Khoảng giữa là nguồn USB 5 V, không phải pin.
- **Số cell**: theo `battery.cells`, hoặc tự nhận lúc cắm pin: `điện áp / cell_max + 1` (tối đa 8). Pin cắm lúc gần cạn bị nhận thiếu một cell (4S còn 12,8 V = 3S đầy): biết số cell thì nên nhập. Từ 0.18.1, khi điện áp ổn định vượt `số cell × cell_max` (một cell không thể cao hơn) thì đếm lại: nguồn lên từng bậc, như điện áp từ telemetry ESC dừng một lúc quanh 4 V, không còn bị chốt 1S. Chỉ đếm tăng, không bao giờ giảm.
- **Trạng thái**: `NOT_PRESENT → OK → WARNING → CRITICAL`, có trễ 10 mV và thời gian chờ (`warning_delay`, `critical_delay`; thời gian chờ nguy hiểm đếm thời gian **liên tục** dưới ngưỡng).
- **Tác dụng**: `WARNING` trở xuống chặn arm (blocker `BATT_LOW`); `CRITICAL` lúc đang bay → failsafe hạ cánh nếu `land_on_critical` bật. Lõi bay không còn ngưỡng pin riêng: nó nhận `fc::Health{batt_low, batt_critical}` từ `FlightApp`.
- Cả section `battery` là **live**: sửa ngưỡng có hiệu lực ngay.

### Dòng, mAh đã dùng, phần trăm còn lại (firmware 0.8)

- **Nguồn đo dòng** (`battery.current_source`): `AUTO` (cảm biến dòng nếu chân `pins.curr_adc` dùng được, không thì ESC), `ADC`, `ESC`, `NONE`.
- **Cảm biến dòng ADC**: `dòng (mA) = mV ở chân × 10000 / current_scale + current_offset`, đúng công thức và đơn vị của Betaflight (`ibata_scale` 0,1 mV/A, `ibata_offset` mA): số của nhà sản xuất board dùng được luôn. Mặc định 400 = 40 mV/A. Hệ số âm cho cảm biến lắp ngược.
- **ESC**: tổng dòng các ESC báo qua *extended DShot telemetry* (cần `motors.bidirectional` và `motors.edt`). ESC chỉ gửi khi được bật bằng lệnh DShot 13: firmware gửi lệnh này **khi các ESC bắt đầu trả lời** (lúc cắm pin, kể cả cắm sau khi đã cắm USB), gửi lại khi chúng im hết rồi trả lời lại (ESC khởi động lại). **Không bao giờ gửi khi đang arm** (lệnh DShot dừng motor trong lúc lặp lệnh); nếu ESC khởi động lại giữa chuyến bay thì lệnh được gửi ngay sau khi disarm. Số đo của một ESC còn tính trong 1 s sau lần báo cuối.
- **Lọc**: dòng hiển thị qua PT1 1 Hz (như Betaflight); số mAh cộng dồn số đo chưa lọc, 50 lần/giây, tính từ lúc cấp nguồn (không reset khi arm, như Betaflight). Một khoảng hở ≥ 1 s (tác vụ cảm biến bị treo) không được cộng.
- **Còn lại (%)**: có `battery.capacity_mah` và có số đo dòng → `(dung lượng − mAh đã dùng) / dung lượng`; không thì theo điện áp giữa `cells × cell_min` và `cells × cell_max`.
- Báo cáo khởi động: `current: ADC sensor on GPIO3, 400 x 0.1 mV/A`, `current: from the ESCs (extended DShot telemetry)`, `current: none (...)`, hoặc `current: FAILED - ...` (chọn ADC mà chưa có chân, chân không phải ADC, chọn ESC mà chưa bật extended telemetry). Chân trùng: `pins: FAILED ...`, không cho arm.
- Stream `battery`: `current` (0,01 A), `mah`, `remaining` (%), `current_source`, `current_mv` (điện áp ở chân cảm biến, để hiệu chỉnh). Blackbox: cột `current`, `mah` (nhóm battery).
- `current_scale`, `current_offset`, `capacity_mah`, `current_source` có hiệu lực ngay; đổi chân thì lưu và khởi động lại.

## Còi

```
src/hal/beeper/beeper_core.h   beep::Beeper (bảng mẫu âm, ưu tiên), beep::Conditions (tình huống → âm)   [portable]
src/hal/beeper/beeper_base.h   BuzzerOutput: còi chủ động (mức logic) hoặc thụ động (PWM qua LEDC)
```
- 18 tình huống (enum `beeper_cond` trong schema): gyro hiệu chuẩn xong, mất sóng, đang hạ cánh failsafe (SOS), arm / disarm, arm có / chưa có GPS fix, pin thấp / nguy hiểm, công tắc Beeper, hiệu chuẩn accelerometer, GPS sẵn sàng, đang arm ga thấp, bật nguồn, không arm được, xoá xong blackbox, và `USB`.
- Mẫu âm theo đơn vị 10 ms, lấy từ Betaflight; thêm hai mẫu riêng: bật nguồn và "không arm được".
- **Ưu tiên** = thứ tự trong enum; âm đang kêu không bị âm ưu tiên bằng hoặc thấp hơn chen vào.
  Khác Betaflight có chủ đích: tình huống **bị tắt** thì coi như không tồn tại (không chặn âm ưu tiên thấp hơn).
- **Im lặng trên bàn**: app đang kết nối + không đo được pin + tình huống `USB` tắt (mặc định) → **còi** không kêu, trừ nút "Thử" trong app. Từ 0.8.1 luật này **không áp cho motor**: ESC chỉ kêu được khi có điện, tức là đã cắm pin, dù firmware không đo được pin (không có chân ADC, ESC không báo điện áp). Trước 0.8.1, máy không đo được pin mà đang nối app thì motor không bao giờ kêu. "App đang kết nối" = có khung hợp lệ trong 3 s gần nhất (client Python gửi keepalive mỗi 0,3 s).
- Lệnh `BEEP {condition}`: phát thử một âm trên còi và motor (kể cả âm đang tắt, kể cả đang "im lặng trên bàn"); `condition` trống = im lặng.

### Motor kêu thay còi (máy không gắn còi)

ESC nhận lệnh DShot "beacon" (lệnh 1 đến 5 = năm âm khác nhau) và cho motor rung thành tiếng bíp. Firmware dùng cách này làm **bộ phát tiếng thứ hai**, chạy song song với còi và có danh sách tình huống riêng (`beeper.motor_conditions`). Máy không gán chân còi thì đây là tiếng báo duy nhất, không cần cài gì thêm: mặc định đã bật.

```
beep::Beeper beeper        mẫu âm cho còi (như cũ)
beep::Beeper motor_beeper  cùng các tình huống, slot_ms = 450: mỗi đoạn "bật" của mẫu âm = đúng MỘT tiếng của ESC
beep::Conditions           đưa yêu cầu cho cả hai; motor chỉ nhận khi được phép (dưới đây)
MotorOutputManager::beacon_all(tone)   một khung DShot, mọi motor cùng lúc (Betaflight: dshotCommandWrite(ALL_MOTORS))
```
- Motor phát **số tiếng** của mẫu âm, theo nhịp của mẫu (0.18.1): gyro hiệu chuẩn xong = 3 tiếng, vừa disarm = 2 tiếng, không arm được = 3 tiếng, mất sóng = 1 tiếng mỗi 1 s, pin dưới mức tối thiểu = một tiếng mỗi 0,52 s. Một phần "bật" ngắn hơn 450 ms (`DSHOT_BEACON_MODE_INTERVAL_US` của Betaflight) được kéo lên 450 ms, phần dài hơn giữ nguyên. **Độ dài của từng tiếng do ESC quyết định**: DShot chỉ có lệnh "beacon 1 – 5", không có độ dài (Betaflight: "the DShot Beacon tone duration is determined by the ESC").
- **Chỉ khi chưa arm**, và không sớm hơn 1,2 s sau khi disarm (`DSHOT_BEACON_GUARD_DELAY_US`). Tiếng "vừa disarm" vì thế đến sau 1,2 s.
- **Lúc đang bay motor không báo được gì**: pin yếu, failsafe đang hạ cánh, "đang arm, ga thấp" không có tiếng. Các tình huống đó (`ARMING*`, `ARMED`, `RX_LOST_LANDING`) bị firmware loại khỏi danh sách của motor dù settings có bật. Muốn có cảnh báo lúc bay: gắn còi, hoặc xem điện áp trên tay điều khiển (telemetry CRSF, phần việc sau).
- Chỉ với giao thức **DShot** (PWM / Oneshot / Multishot không có lệnh). Báo cáo khởi động ghi rõ: `beeper: motors (DShot beacon, disarmed only)`, `beeper: active buzzer + motors (...)`, hoặc `beeper: FAILED - no buzzer pin set, and the motors can only beep with a DShot protocol`.
- Không chen vào việc khác: đang có lệnh DShot khác (đảo chiều…) hoặc app đang chạy thử motor thì tiếng đó bị bỏ, không xếp hàng.
- Tiếng "bật nguồn" mặc định tắt cho motor (ESC tự phát nhạc khởi động của nó). Tiếng "gyro hiệu chuẩn xong" đến khoảng 1 s sau khi cấp điện: ESC còn đang khởi động thì có thể không phát.
- Quy tắc "im lặng trên bàn" và "mất sóng không kêu khi app đang kết nối" áp dụng cho motor như cho còi.
- `beeper.dshot_beacon_tone` (1 đến 5): âm nào của ESC. Stream `battery` có thêm `motor_beeping` (tình huống motor đang phát + 1).

## App

Tab **Pin và còi**: số đo trực tiếp, thang một cell (đỏ / vàng / xanh theo ngưỡng), các ngưỡng, hiệu chỉnh hệ số chia (nhập điện áp đo bằng đồng hồ, app tính từ điện áp đang có ở chân). Khối **Đo dòng và dung lượng**: dòng, mAh đã dùng, phần trăm còn lại; nguồn đo, hệ số / độ lệch cảm biến, dung lượng pin; hiệu chỉnh hệ số cảm biến (nhập dòng đo bằng ampe kìm lúc motor quay đều, app tính từ điện áp đang có ở chân cảm biến). Chân `ADC đo dòng` ở tab Chân và cổng. Bảng tình huống có **hai cột dấu chọn** (còi, motor), hình mẫu âm, số tiếng motor sẽ phát, nút "Thử"; ô ghi chú đầu bảng nói máy đang có gì để kêu (còi, motor, cả hai, hoặc không có gì). Hình mẫu âm, số tiếng của motor và danh sách tình huống motor không phát được đều được so với firmware (`tools/ui_reference.cpp` → `test_gui_math.py`).

## Đã kiểm chứng (PC)

- `test_power` 119 kiểm tra; trong `test_fw`: pin thật của mạch ảo (ADC + cầu chia, sụt áp khi tải), chặn arm, failsafe khi pin nguy hiểm, còi đo ở chân GPIO ảo, im lặng trên bàn.
- Motor kêu: `test_power` (số tiếng, khoảng cách, không kêu khi arm / trong 1,2 s sau disarm / trên bàn / với ESC không có DShot, qua lúc `millis()` tràn), `test_motor` (một khung cho mọi motor, không xếp hàng sau lệnh khác, không chen vào test motor), và `test_fw` kịch bản **máy không có còi**: đếm chính các khung lệnh DShot đi ra chân motor (thời điểm, âm, đủ 4 motor trong một khung), lúc arm không có khung lệnh nào.
- Lỗi cố ý (`tools/mutate.py`).
- App: `test_e2e.py` (pin, còi, im lặng trên bàn, motor kêu), trang thật trong trình duyệt (`test_battery_and_beeper_tab`).
- Đo dòng: `test_power` nhóm `[current]` (công thức Betaflight với số tính tay, chọn nguồn, lọc 1 Hz, mAh, khoảng hở, NaN, phần trăm còn lại); `test_fw` nhóm `current` (`WB_ONLY=current`): cảm biến ADC trên mạch ảo (10 A → 10 mAh trong 3,6 s, đổi hệ số / độ lệch khi đang chạy), ESC ảo có extended telemetry (chỉ bật sau khi nghe lệnh 13 sáu lần; dòng cộng của 4 ESC; tốc độ motor không bị khung dòng làm sai; ESC im rồi khởi động lại → hỏi lại; khởi động lại lúc đang arm → chỉ hỏi sau khi disarm), các cài đặt sai; trang thật (`test_current_sensor_and_servos_on_the_page`).

## Chưa kiểm chứng (cần mạch thật)

- Thời gian một lần `analogReadMilliVolts()` trong vòng bay (đọc 50 lần/giây; xem `status.loop_max_us`).
- Độ chính xác ADC của ESP32-S3 ở hai đầu dải; điện áp ở chân không được vượt ~3,1 V khi pin đầy.
- Còi thụ động dùng một kênh LEDC: chưa thử cùng lúc với motor PWM / Oneshot (cũng dùng LEDC).
- **Đo dòng trên mạch thật**: cảm biến dòng thật (nhiễu ADC khi motor quay, cần lọc thêm không); ESC thật có gửi extended telemetry sau lệnh 13 không, bao lâu một lần, đơn vị dòng của từng loại ESC (BLHeli_32 / AM32 / Bluejay). ESC 4-in-1 báo dòng tổng trên mỗi kênh thì tổng bị nhân 4: báo lại loại ESC nếu số đo gấp 4 lần.
- Tự nhận số cell với pin thật; ngưỡng mặc định (4,30 / 3,50 / 3,30 V) là của Betaflight cho LiPo.
- **Motor kêu trên ESC thật**: mô phỏng chỉ kiểm tra khung lệnh đi ra đúng lúc, đúng âm. ESC (BLHeli_S / Bluejay / BLHeli_32 / AM32) có phát tiếng với mỗi lệnh không, một tiếng dài bao lâu, năm âm nghe ra sao, có cần lặp lệnh không: phải nghe trên mạch. Nếu ESC bỏ tiếng, báo lại loại ESC và firmware của nó.
- Motor nóng lên khi kêu lâu (dòng qua cuộn dây): "mất sóng" và "pin nguy hiểm" kêu liên tục cho tới khi hết điều kiện. Betaflight cũng làm vậy; chưa đo nhiệt.
