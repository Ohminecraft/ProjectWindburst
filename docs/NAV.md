# Vị trí: optical flow, ước lượng vị trí, Position Hold, GPS Rescue (firmware 0.7.0)

**Chưa có dòng nào ở đây chạy trên mạch thật.** Mọi thứ trong tài liệu này được kiểm chứng trên PC: test từng khối, firmware thật trên mạch ảo (cảm biến mô phỏng mức thanh ghi / mức khung, máy bay mô phỏng có gió), lỗi cố ý, và biên dịch cho ESP32-S3. Mục 8 ghi những gì phải thử trên mạch, theo thứ tự.

Tab **Vị trí** trong app gom mọi cài đặt và trạng thái của phần này.

## 1. Cần phần cứng gì

| Muốn | Cần |
|---|---|
| Position Hold trong nhà / bay thấp | một cảm biến optical flow **và** một lidar (đo khoảng cách tới sàn) |
| Position Hold ngoài trời | GPS đã fix **và** la bàn đã hiệu chuẩn |
| GPS Rescue (tự bay về) | GPS đã fix lúc arm (điểm home), la bàn đã hiệu chuẩn, ước lượng độ cao (barometer) |

GPS không có la bàn thì **không** giữ vị trí được: máy không biết hướng bắc nằm ở đâu so với mũi, đẩy sai hướng là bay đi. Firmware từ chối chứ không đoán (blocker `POSHOLD_NO_GPS`, `RESCUE_NO_COMPASS`).

### Cảm biến optical flow được hỗ trợ

| Loại (`flow.type`) | Nối | Lidar | Ghi chú |
|---|---|---|---|
| `MSP` — MicoAir MTF-01 / MTF-02, Matek 3901-L0X (giao thức MSP v2) | UART, chỉ cần **TX của cảm biến → chân `flow_rx`** | có sẵn trong cảm biến | cách dễ nhất; đặt cảm biến ở chế độ MSP, 115200 baud |
| `PMW3901` | SPI **riêng** (bus SPI thứ hai của chip): `flow_cs`, `flow_sck`, `flow_miso`, `flow_mosi` | cần lidar riêng (VL53L1X / VL53L0X trên bus I2C) | không dùng được cùng lúc với thẻ SD kiểu SPI (cùng một bus): chuyển thẻ sang SDMMC |

Vì sao PMW3901 không chung bus với IMU: mỗi lần đọc nó giữ bus khoảng 0,3 ms (datasheet bắt chờ giữa các byte); trên bus của IMU thì một chu kỳ điều khiển 250 µs sẽ bị chặn.

### Cảm biến UART lấy cổng nào

Firmware tự chọn cổng còn trống, theo thứ tự: **UART1** nếu không bật GPS → **UART2** nếu không dùng receiver nối tiếp → **UART0** nếu app nối qua USB của chip (env `rx_s3_usb`; RX mặc định của UART0 trên ESP32-S3 là GPIO44). Mạch nối app qua cầu USB-UART, có cả GPS lẫn receiver nối tiếp thì không còn cổng nào: báo cáo khởi động ghi rõ `flow: FAILED - no UART left ...`.

Mạch của bạn (N16R8, app qua USB): GPS ở UART1, CRSF ở UART2, cảm biến MSP vào **GPIO44** (hoặc chân khác tuỳ bạn, UART của ESP32 đặt được chân bất kỳ).

## 2. Cài đặt lần đầu (tab Vị trí)

1. Tab *Chân và cổng*, nhóm "Optical flow": điền chân. Tab *Vị trí*: chọn "Loại cảm biến". **Lưu và khởi động lại** (loại cảm biến và baud chỉ đọc lúc khởi động).
2. Dòng "Firmware báo" phải ghi `MSP flow on UART0` (hoặc `PMW3901 on its own SPI bus`). "Số mẫu nhận được" khoảng 100 mẫu/giây. Máy nằm trên bàn thì "Mẫu mới nhất" ghi *bỏ qua: quá gần sàn* — đúng, dưới 10 cm cảm biến không lấy nét được.
3. **Kiểm tra hướng lắp** (không cần arm, tháo cánh): cầm máy phẳng, cách sàn 0,5 – 1 m, sàn có hoa văn và đủ sáng.
   - đưa máy về phía mũi → mũi tên trên vòng tròn chỉ **lên**;
   - đưa sang phải → mũi tên chỉ **sang phải**;
   - lệch một phần tư / nửa vòng → đổi "Hướng lắp cảm biến"; trái phải ngược → bật "Ảnh bị lật gương". Hai ô này có tác dụng ngay, không cần khởi động lại;
   - chỉ lắc máy tại chỗ → mũi tên gần như đứng yên (phần quay đã được gyro trừ đi). Nếu mũi tên chạy theo khi lắc: hướng lắp còn sai, hoặc hệ số tốc độ lệch nhiều.
4. "Hệ số tốc độ": đi một quãng đã biết với tốc độ đều, so tốc độ đọc được. 100 = đúng như cảm biến báo.
5. Tab *Mode*: gán công tắc cho **Position Hold** (và **GPS Rescue** nếu dùng). Position Hold tự bật Angle; nên bật cùng Altitude Hold.
6. Failsafe bay về: tab *Vị trí*, ô "Khi mất sóng hoặc bật công tắc Failsafe" → *GPS Rescue*.

## 3. Position Hold làm gì

Viết theo `mode_poshold.cpp` của ArduCopter, mỗi trục (roll, pitch) một máy trạng thái:

| Trạng thái | Ý nghĩa |
|---|---|
| phi công điều khiển | cần cho góc nghiêng như Angle, cộng phần nghiêng đã học để chống gió |
| đang phanh | vừa buông cần: nghiêng ngược chiều chuyển động, càng nhanh nghiêng càng nhiều (`ph_brake_rate`, `ph_brake_angle`) |
| phanh xong, chờ trục kia | trục này đã dừng |
| đang chuyển sang giữ vị trí | cả hai trục đã dừng: 1,5 giây chuyển từ phanh sang bộ điều khiển vị trí |
| đang giữ vị trí | bộ điều khiển vị trí giữ điểm; góc nghiêng nó cần khi máy đứng yên chính là gió |
| đang trả lại cho phi công | cần vừa động: 0,5 giây chuyển từ bộ điều khiển sang cần |

Bộ điều khiển vị trí là tầng của `AC_PosControl`: sai lệch vị trí → vận tốc (P, có giới hạn căn bậc hai) → sai lệch vận tốc → gia tốc (PID, lọc 5 Hz) → góc nghiêng. Hệ số mặc định là của ArduCopter (`pos_p` 1.0, `vel_p` 2.0, `vel_i` 1.0, `vel_d` 0.25).

Độ cao và yaw không bị đụng tới: ga (hoặc Altitude Hold) và cần yaw vẫn của phi công.

**Khi nguồn vị trí mất** (flow không thấy gì quá 0,3 giây, GPS mất quá 2 giây): Position Hold tự nhả, trạng thái ghi "công tắc đang bật nhưng chưa có gì để giữ vị trí", máy bay như Angle. Có lại thì tự vào lại (bắt đầu bằng phanh).

**Arm với công tắc Position Hold bật sẵn** chỉ được khi có thứ để giữ: cảm biến flow khoẻ **và có lidar**, hoặc GPS fix + la bàn đã hiệu chuẩn.

## 4. GPS Rescue

Trình tự của Betaflight (`flight/gps_rescue.c`):

1. **Leo** tới độ cao bay về: điểm cao nhất của chuyến bay + "Leo thêm" (mặc định), hoặc độ cao cố định, hoặc độ cao hiện tại + "Leo thêm". Vài mét cuối leo chậm lại để không vọt qua.
2. **Quay mũi** về điểm cất cánh (máy đứng tại chỗ trong lúc quay).
3. **Bay về** với "Tốc độ bay về", chậm dần trong 2 × "Bắt đầu hạ khi cách home" cuối.
4. **Hạ dần** khi còn cách home dưới mức đó: trên cao hạ nhanh (tới 3 lần "Tốc độ hạ"), gần đất hạ chậm.
5. **Hạ cánh**: dưới "Độ cao bắt đầu hạ chậm"; chạm đất thì motor tắt và máy disarm.

Bắt đầu gần home hơn "Khoảng cách tối thiểu để bay về": leo lên rồi hạ tại chỗ.

**Chạy khi nào**: công tắc GPS Rescue bật, hoặc failsafe (mất sóng / công tắc Failsafe) nếu thủ tục failsafe là Rescue.

**Lấy lại quyền điều khiển**
- Rescue từ công tắc: tắt công tắc.
- Rescue từ failsafe: sau khi sóng trở lại, **gạt cần roll hoặc pitch quá 30 %**. Sóng trở lại mà không ai động cần thì máy vẫn bay về tiếp (phi công có thể chưa sẵn sàng).

**Rescue bỏ cuộc** (giai đoạn "bỏ cuộc" + lý do) khi: mất GPS, mất hướng la bàn, mất ước lượng độ cao, ít vệ tinh hơn "Số vệ tinh tối thiểu" suốt 15 giây, leo 10 giây mà không lên, bay 20 giây mà không gần home hơn. Khi đó:
- từ failsafe → hạ cánh tại chỗ như failsafe thường (không ai cầm lái);
- từ công tắc → trả máy cho phi công (Angle + Altitude Hold).

**Pin dưới mức tối thiểu luôn hạ cánh tại chỗ**, không bay về bằng một viên pin cạn.

**Arm khi đã cài Rescue** (có công tắc Rescue hoặc failsafe = Rescue): firmware đòi GPS đã fix (blocker `RESCUE_NO_FIX`, vì chưa fix thì chưa có điểm home) và có la bàn (`RESCUE_NO_COMPASS`). Ô "Cho arm khi GPS chưa fix" bỏ yêu cầu thứ nhất; khi đó failsafe sẽ hạ cánh tại chỗ.

## 5. Bộ ước lượng dùng flow và GPS thế nào

- **Flow → vận tốc** (`core/flow.h`): `v_tiến = +khoảng cách × (flow_y − gyro_y)`, `v_phải = −khoảng cách × (flow_x − gyro_x)` (quy ước của ArduPilot). Gyro được lấy trung bình **đúng khoảng thời gian** mà mẫu flow mô tả (cảm biến MSP báo trễ khoảng 45 ms về một cửa sổ 40 ms; firmware giữ 0,25 giây lịch sử gyro để cắt đúng đoạn đó). Mẫu bị bỏ (không phải "vận tốc bằng 0") khi: chất lượng bề mặt thấp, ngoài tầm độ cao 0,1 m … "Độ cao tối đa dùng optical flow", nghiêng quá 45°, quay nhanh hơn 200 °/s.
- **Gộp mẫu** (`hal/flow/flow_driver.h`, `Accumulator`): một mẫu đơn của các cảm biến này rất thô — cảm biến MSP đếm theo bước 5 mrad, PMW3901 theo bước 1,26 mrad; ở 100 mẫu/giây đó là 0,5 và 0,13 rad/s (nửa mét mỗi giây ở độ cao một mét). Driver cộng khoảng 5 mẫu (50 ms) thành một lần đọc: sai số lượng tử và sai số thời gian giữa hai mẫu đều nhỏ đi 5 lần. Bộ ước lượng nhận 20 lần đọc mỗi giây. (ArduPilot cộng cảm biến PixArt trong 100 ms vì cùng lý do.)
- **Kalman ngang** (`core/horizontal_est.h`): flow vào như một phép đo vận tốc, GPS như vị trí + vận tốc. Chỉ có flow: vận tốc đúng, vị trí chỉ là tương đối (không neo vào bản đồ) — đủ để giữ chỗ. Trong lúc không có phép đo, độ bất định của vận tốc tự lớn dần (chỉ còn phép dự đoán), nên phép đo đầu tiên sau quãng mất được tin gần như trọn vẹn và luôn qua cổng lọc: không có "vận tốc ma" sau khi flow mất rồi có lại (Betaflight từng gặp). Bản nháp đầu có thêm bước "bắt đầu lại sau 2 giây"; lỗi cố ý cho thấy nó không làm thay đổi gì, nên đã bỏ.
- **GPS không có la bàn**: vị trí vẫn hiển thị, nhưng không dùng để điều khiển; và nếu flow đang chạy thì GPS không được trộn vào (vận tốc của flow tính theo trục của máy, không khớp với hướng bắc của GPS khi không biết hướng mũi).
- **Chỉnh góc nghiêng theo gia tốc đo được** (`core/tilt_aid.h`, ô "Chỉnh góc nghiêng…"): khi bay, accelerometer đo lực đẩy chứ không đo trọng lực; mỗi lần máy tăng tốc hay phanh, góc nghiêng ước lượng lệch vài độ. Có vận tốc đo được (GPS hoặc flow) thì biết gia tốc thật, và sửa được (cách của `AP_AHRS_DCM`). Chỉ chạy khi đang arm.

## 6. Giao thức và log

- Section `flow` (id 16, đọc lúc khởi động): `type`, `baud`. Section `nav` (id 17, có tác dụng ngay): hướng lắp / lật / hệ số / ngưỡng của flow, Position Hold, Rescue, thủ tục failsafe.
- Stream `nav` (id 13): cờ (`POSHOLD`, `POSHOLD_UNAVAILABLE`, `RESCUE`, `RESCUE_AVAILABLE`, `NAV_LEAN`), chất lượng và lý do bỏ mẫu của flow, số mẫu/giây, vận tốc flow đo (trung bình 0,25 giây), trạng thái hai trục của Position Hold, góc nghiêng autopilot yêu cầu, điểm đang giữ, bù gió, giai đoạn và lý do bỏ cuộc của Rescue, khoảng cách tới home, độ cao bay về.
- Stream `estimate` thêm ba bit: `VEL` (có vận tốc trên mặt đất), `FLOW` (flow đang được dùng), `TILT_AIDED`.
- Mode `RESCUE`; blocker `RESCUE_NO_FIX`, `RESCUE_NO_COMPASS`; health `FLOW`; `range_model` thêm `MSP`; 6 chân mới trong section `pins` (bản ghi pins của 0.6 được đọc tiếp, các chân mới để trống).
- **Blackbox**: nhóm mới **"Vị trí và điều hướng"** (`NAV`, bật mặc định, 10 lần mỗi giây): `pos_n/e`, `vel_n/e`, `est_valid`, `nav_flags`, `flow_quality`, `flow_reject`, `flow_fwd/right`, `nav_roll/pitch`, `nav_target_n/e`, `rescue_phase`. `pos_*` và `vel_*` trước đây nằm trong nhóm GPS. Cài đặt blackbox lưu từ 0.6 không có nhóm này: tích lại ô "Vị trí và điều hướng" ở tab Blackbox. Trình xem log có nhóm đồ thị "Vị trí".
- CLI: `python -m wbclient monitor nav,estimate`.

## 7. Thử trên máy mô phỏng

`build/wb_sim` có sẵn cả hai cảm biến flow (MTF-01 trên UART0, PMW3901 trên bus SPI thứ hai, chân đã điền). Cổng điều khiển (cổng app + 1) có thêm:

```
carry <z> <vn> <ve>    cầm máy phẳng, cách sàn z mét, đi với vận tốc vn / ve (bắc / đông)      carry off   đặt xuống
wind <n> <e>           gió (m/s)
flow quality <0..255>  chất lượng bề mặt cảm biến MSP báo        flow dark 0|1   PMW3901 không thấy gì
flow plug 0|1          rút / cắm cảm biến MSP                    flow mount <độ> <gương 0|1> <hệ số>   cách cảm biến nằm trên máy
gps fix 0|1            GPS mất / có fix                          gps dead 0|1    module GPS ngừng gửi
```

`state` in thêm `pn pe vn ve` (vị trí và vận tốc thật của máy mô phỏng).

## 8. Thử trên mạch thật, theo thứ tự

Tháo cánh cho các bước 1 – 4.

1. **Báo cáo khởi động**: dòng `flow:` và `range:` đúng như mục 2. Nếu `flow: MSP flow on UARTx` mà "Số mẫu nhận được" là 0: sai chân, sai baud, hoặc cảm biến chưa ở chế độ MSP.
2. **Số mẫu và chất lượng**: khoảng 100 mẫu/giây (MTF-01); chất lượng trên sàn có hoa văn. Ghi lại con số chất lượng trên vài loại sàn — ngưỡng mặc định 30 là đoán.
3. **Hướng lắp và hệ số** (mục 2, bước 3 – 4). **Đây là bước dễ sai nhất**: hướng trục của cảm biến thật chưa được kiểm chứng, mặc định theo ArduPilot.
4. **Lắc tại chỗ**: mũi tên phải gần như đứng yên. Nếu không: báo lại kèm log (nhóm "Vị trí và điều hướng" + "Gyro"), có thể độ trễ 45 ms của cảm biến bạn dùng khác.
5. **Tải CPU**: tab Tổng quan, "Luồng cảm biến, lượt dài nhất" và "Chu kỳ điều khiển thực tế" trước và sau khi bật flow. PMW3901 thêm khoảng 0,3 ms mỗi 10 ms vào luồng cảm biến; cảm biến MSP gần như không đáng kể.
6. **Bay**: Angle + Altitude Hold trước, thấp (1 m), trong nhà không gió hoặc ngoài trời lặng gió. Bật Position Hold khi máy đang treo. Tay để sẵn trên công tắc. Máy trôi đều một hướng rồi vọt đi = hướng lắp sai (tắt ngay). Máy lắc qua lại tại chỗ = hệ số P quá cao với máy của bạn: giảm `vel_p`.
7. **GPS**: ngoài trời, la bàn hiệu chuẩn xong, tab Vị trí ghi "Hướng mũi: theo la bàn" và hình 3D ở Tổng quan quay đúng hướng thật. La bàn lệch 90° thì Position Hold bằng GPS sẽ bay vòng tròn rộng dần ("toilet bowl"): tắt ngay và sửa "Hướng chip la bàn".
8. **Rescue**: thử bằng công tắc trước, ở khoảng cách 30 – 50 m, độ cao 10 m, chỗ trống. Chỉ khi công tắc chạy đúng mới đặt failsafe = Rescue và thử tắt tay điều khiển.

Sau mỗi bước bay: gửi log (ghi vào flash hoặc thẻ SD, đủ nhóm mặc định).

## 9. Đã kiểm chứng trên PC

- `test_nav` (130): flow → vận tốc, lịch sử gyro, vận tốc đến trễ trong bộ ước lượng ngang, `TiltAid` (cả phép cắt quãng thời gian tới từng mili giây), bộ điều khiển vị trí, máy trạng thái Position Hold (phanh, gió chỉ học khi đứng yên, bàn giao), các giai đoạn và lý do bỏ cuộc của Rescue.
- `test_flowsensor` (94): bộ đọc khung MSP v2 (CRC, khung hỏng, nhiễu), driver MSP (tỉ lệ, gộp mẫu, khung đến trễ / đến chùm, lidar), driver PMW3901 với mô phỏng mức thanh ghi (nhận dạng, bảng khởi tạo, thời gian chờ, gộp mẫu, đường dữ liệu đứt), lớp `FlowSensor`, chọn UART và chọn lidar.
- `test_flight` (251): vòng kín với máy mô phỏng — Position Hold bằng GPS + la bàn (hai cỡ máy) và bằng flow + lidar, gió, quay tại chỗ, mất nguồn vị trí, lidar im lặng, GPS không la bàn cạnh flow; Rescue đủ trình tự (từ failsafe và từ công tắc, có và không có Altitude Hold), bắt đầu trong gió ngang, các kiểu thất bại (la bàn, GPS, vệ tinh, pin).
- `test_fw` (firmware thật trên mạch ảo): cảm biến MSP trên UART0 / UART1, PMW3901 trên bus riêng, chọn UART, hướng lắp / gương / hệ số, bay giữ vị trí bằng từng cảm biến, rút cảm biến khi đang bay, dây dữ liệu PMW3901 đứt, Position Hold bằng GPS với mũi không hướng bắc, Rescue từ failsafe và từ công tắc, sóng trở lại giữa chừng, GPS chết giữa chừng, pin cạn, các blocker, section `nav` có tác dụng ngay, nhóm log mới.
- App: client Python (`test_e2e`: flow + Position Hold, Rescue bay về), trang thật trong trình duyệt (`test_position_tab`: từ "chưa chọn cảm biến" tới bay giữ vị trí, vòng tròn kiểm tra hướng lắp).

## 10. Chưa kiểm chứng / giả định

- **Hướng trục và tỉ lệ của cảm biến thật** (cả hai loại). Mô phỏng viết theo tài liệu của ArduPilot / Betaflight, độc lập với driver, nhưng chưa có byte nào từ cảm biến thật.
- Tỉ lệ MSP `motion / 200 / dt` và độ trễ 45 ms + cửa sổ 40 ms lấy từ driver MTF-01 của Betaflight; MTF-02 và Matek có thể khác.
- Bảng khởi tạo PMW3901 chép từ ArduPilot; chưa nạp vào chip thật. Thời gian chờ giữa các byte dùng `delayMicroseconds`.
- UART0 làm cổng cảm biến trên bản USB: ROM của ESP32 in log khởi động ra chân TX của UART0 lúc bật nguồn; cảm biến chỉ nối RX nên không ảnh hưởng, nhưng chưa thử.
- Ngưỡng chất lượng 30, nhiễu flow 0,10 rad/s, giới hạn 200 °/s: số chọn theo tài liệu, chưa đo.
- Hệ số Position Hold là mặc định của ArduCopter cho máy cỡ 1 kg; máy 5 inch nhẹ, lực đẩy lớn có thể cần `vel_p` thấp hơn. Trong mô phỏng cả hai cỡ đều ổn định.
- Phát hiện chạm đất khi Rescue dùng chung bộ phát hiện của Altitude Hold; trên mặt đất dốc hoặc cỏ cao chưa biết.
- Thời gian chạy của phần điều hướng trong chu kỳ điều khiển trên ESP32-S3 chưa đo (nó chạy ở tần số bộ ước lượng, 500 Hz, và nằm trong RAM như phần còn lại của chu kỳ, trừ `atanf` của thư viện). Nhìn cột `t_est` của log trước và sau khi bật Position Hold.
- Lỗi cố ý (mutation): hơn 440 lỗi cố ý cho riêng phần này (firmware) và 41 cho tab Vị trí; mọi lỗi còn lại đều bị test bắt (chi tiết: `docs/HANDOFF.md` mục 5, đợt C).
