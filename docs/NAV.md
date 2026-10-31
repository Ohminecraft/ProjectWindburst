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

### Loiter (0.41) — `src/core/nav/loiter.h`

Viết theo `mode_loiter.cpp` và `AC_WPNav/AC_Loiter.cpp` của ArduCopter. Đây là cách giữ vị trí thứ hai, khác Position Hold ở chỗ **cần cho gia tốc, không cho góc nghiêng**, và bộ điều khiển vị trí luôn chạy:

| | Position Hold | Loiter |
|---|---|---|
| Cần roll / pitch | góc nghiêng như Angle | gia tốc mong muốn (góc nghiêng mà cần sẽ cho ở Angle, tối đa 2/3 góc giới hạn = 20° mặc định, đổi ra gia tốc) |
| Buông cần | phanh kiểu cố định, dừng rồi mới giữ điểm | vận tốc mong muốn giảm dần theo "lực cản", sau 1 s thì phanh thêm (tối đa 2,5 m/s², jerk 5 m/s³) |
| Tốc độ | tuỳ phi công | hết cần = `loiter_speed` (mặc định 12,5 m/s như `LOIT_SPEED_MS`), nửa cần cho khoảng nửa (qua đường cong rate như Angle) |
| Độ cao | theo công tắc Altitude Hold | luôn giữ độ cao (công tắc LOITER tự bật Angle + Altitude Hold) |
| Hợp với | bay tay, FPV | người mới, quay phim, căn vị trí |

**Cách chạy** (`AC_Loiter::calc_desired_velocity`):
1. Cần → góc nghiêng mong muốn → gia tốc mong muốn (bắc / đông theo hướng mũi), đưa thẳng vào bộ điều khiển (feed-forward).
2. Góc nghiêng máy sẽ có được dự đoán bằng bộ lọc bậc một 0,15 s (`ATC_INPUT_TC`; ArduPilot dùng mô hình lệnh của bộ điều khiển tư thế), rồi tích phân thành vận tốc mong muốn.
3. Lực cản: vận tốc mong muốn mất (gia tốc ở góc tối đa) × tốc độ / `loiter_speed`, nên hết cần thì dừng lại ở `loiter_speed`.
4. Buông cần quá 1 s thì phanh: `sqrt_controller(tốc độ, vel_p / 2, jerk)`, tối đa 2,5 m/s², lên xuống với jerk tối đa 5 m/s³.
5. Vận tốc mong muốn tích phân thành điểm đích. Điểm đích, vận tốc và gia tốc đưa thẳng vào `PosController` (`set_pva`, như `set_pos_vel_accel_NE_m`); khâu tích phân của nó học gió.

**Hàng rào** (`fence_radius`, dùng chung với Position Hold), theo cách `AC_Avoid` giới hạn vận tốc: đi ra ngoài thì vận tốc không quá mức còn dừng kịp trước vạch cách hàng rào 1 m (`sqrt_controller` của khoảng còn lại), và điểm đích không bao giờ vượt vạch đó. Trên máy mô phỏng, lao hết cần vào hàng rào 20 m thì dừng ở 19,0 m, không vượt.

**Bật cùng Position Hold:** Loiter được ưu tiên. Failsafe "giữ vị trí" vẫn dùng Position Hold. Thiếu nguồn vị trí thì như Position Hold: chặn arm (`POSHOLD_NO_GPS`), đang bay thì nhả về Angle + Altitude Hold.

**Khác ArduPilot, có chủ ý:** cần đi qua đường cong rate giống Angle và Position Hold của Windburst, nên cùng một vị trí cần ra cùng góc nghiêng ở cả ba chế độ (ArduPilot đọc cần tuyến tính). Góc dự đoán dùng bộ lọc bậc một. Hàng rào là vòng tròn của Position Hold, chưa có `AC_Avoid` đầy đủ.

**Ground station:** chế độ 5 (LOITER) của ArduCopter. HEARTBEAT báo 5. `DO_SET_MODE` 5 bật được (giữ tới khi gạt công tắc chế độ). `AVAILABLE_MODES` có "Loiter" ở chỉ số 9, standard mode `POSITION_HOLD`; Position Hold đổi sang `NON_STANDARD`, như ArduCopter. OSD và CRSF ghi `LOIT`.

## 4. GPS Rescue

Từ 0.36 rescue bay như ArduPilot và PX4. Code viết lại sau khi đọc nguồn của họ: PX4 `navigator/rtl.cpp` (`computeReturnAltitude`) và `rtl_direct.cpp`; ArduCopter `mode_rtl.cpp`, `mode_smart_rtl.cpp`, `mode_land.cpp` và `libraries/AP_SmartRTL`. Ô **"Kiểu rescue"** (`nav.rescue_mode`) có 3 lựa chọn.

**Bay về (RTL)**, mặc định:

1. **Leo** thẳng đứng tới độ cao bay về, giữ yên tại *điểm dừng được*. Nếu máy đang bay nhanh, điểm này nằm phía trước, ở chỗ máy phanh kịp với gia tốc 2 m/s² (ArduPilot `build_path`); máy không bị giật ngược lại. Vài mét cuối leo chậm lại để không vọt qua.
   - Độ cao bay về, kiểu **Hình nón** (mặc định, PX4 `RTL_CONE_ANG`): bằng "Độ cao bay về" (`RTL_RETURN_ALT`, mặc định 30 m).
     - Nếu gần home hơn "Khoảng cách dùng hình nón" (`RTL_MIN_DIST`), máy chỉ lên tới khoảng cách / tan(góc nón). Với góc 45°, cách 10 m thì lên 10 m.
     - Không bao giờ thấp hơn 2 lần bán kính chấp nhận (4 m).
     - Góc nón = 0: luôn lên "Độ cao bay về".
   - Các kiểu cũ vẫn còn: Cố định, Điểm cao nhất của chuyến bay + leo thêm, Độ cao hiện tại + leo thêm.
   - Máy đang cao hơn độ cao bay về thì giữ nguyên độ cao, không hạ trước (trừ kiểu Cố định).
2. **Bay về** theo đường thẳng ở độ cao bay về.
   - Một *điểm đích* chạy dọc đường bay: tăng tốc rồi giảm tốc với 2 m/s² và dừng đúng trên home. Bộ điều khiển vị trí bám theo điểm này, lấy vận tốc của nó làm feed-forward (giống `AC_WPNav`).
   - Nếu máy tụt lại sau quá 5 m (ví dụ gió ngược), điểm đích chờ máy.
   - Mũi quay dần theo hướng bay, tối đa 90°/s (PX4 `MPC_YAW_MODE` 0). Không còn pha "quay mũi đứng tại chỗ" như Betaflight.
3. **Hạ xuống trên home** tới "Độ cao chờ trên home" (`RTL_DESCEND_ALT`, mặc định 10 m) nếu đang cao hơn. Trên cao hạ nhanh, tới 3 lần "Tốc độ hạ".
4. **Chờ** "Thời gian chờ trên home" (`RTL_LAND_DELAY`, mặc định 5 s), giữ vị trí. Đặt 255 thì máy chờ mãi và không tự hạ cánh.
5. **Hạ cánh** thẳng xuống, giữ vị trí trên home. Dưới "Độ cao bắt đầu hạ chậm" thì hạ bằng "Tốc độ hạ cánh" (ArduPilot `LAND_SPEED`, mặc định 0,5 m/s). Chạm đất thì motor tắt và máy disarm.

**Bay về theo đường cũ (SmartRTL)**, theo ArduPilot SmartRTL:
- Từ lúc arm, firmware ghi lại đường bay: mỗi 2 m một điểm, tối đa 128 điểm.
  - Vòng lặp được cắt bỏ: khi đoạn mới đi qua gần một đoạn cũ dưới 0,99 × 2 m, cả vòng ở giữa thay bằng một điểm.
  - Khi bộ nhớ đầy, đường được rút gọn bằng Ramer-Douglas-Peucker với sai số 1 m.
- Lúc rescue, máy bay ngược lại đường đó: vòng qua những chỗ đã vòng, ở độ cao đã bay, không leo trước.
  - Điểm cuối là home, cao 2 m.
  - Ở các khúc quẹo máy chậm lại: độ đổi vận tốc không quá 1 giây gia tốc, và máy nhìn trước đủ xa để kịp giảm tốc.
  - Tới home thì chờ và hạ cánh như RTL.
- Rơi về bay thẳng (RTL) khi:
  - đường quá rối, bộ nhớ đầy suốt 15 s (`SMARTRTL_TIMEOUT`);
  - mất vị trí 15 s khi đang bay;
  - hoặc lúc bắt đầu đang gần home hơn "Khoảng cách dùng hình nón".

**Hạ cánh tại chỗ (LAND)**, theo ArduPilot LAND:
- Hạ ngay tại điểm dừng được, giữ vị trí bằng GPS.
- Không có GPS hoặc la bàn thì vẫn hạ, nhưng không giữ vị trí (`nogps_run`); phi công đang cầm điều khiển có thể đẩy máy.
- Chỉ cần ước lượng độ cao.

**OSD** (MSP DisplayPort) khi rescue đang bay:
- Ô chế độ hiện `RTL`, `SRTL` hoặc `LAND` (khi failsafe vẫn là `!FS!`).
- Dòng cảnh báo hiện số mét còn phải bay về, tính dọc theo đường bay: `RTL 123M` (với SmartRTL là đường đã đi, không phải đường chim bay).
- Trên home dòng này hiện `RTL HOME`, lúc hạ cánh hiện `RTL LAND`.
- Kiểu LAND hiện độ cao còn lại, ví dụ `LAND 12M`.
- Tab Vị trí của app có dòng "Còn phải bay về" (trường `rescue_left` của stream `nav`).

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
- **Độ cao và vận tốc đứng từ GPS** (0.8.3, khung "Độ cao từ GPS" của tab Vị trí): xem `docs/GPS.md`, mục "Độ cao GPS trong Altitude Hold".
- **GPS không có la bàn**: vị trí vẫn hiển thị, nhưng không dùng để điều khiển; và nếu flow đang chạy thì GPS không được trộn vào (vận tốc của flow tính theo trục của máy, không khớp với hướng bắc của GPS khi không biết hướng mũi).
- **Chỉnh góc nghiêng theo gia tốc đo được** (`core/tilt_aid.h`, ô "Chỉnh góc nghiêng…"): khi bay, accelerometer đo lực đẩy chứ không đo trọng lực; mỗi lần máy tăng tốc hay phanh, góc nghiêng ước lượng lệch vài độ. Có vận tốc đo được (GPS hoặc flow) thì biết gia tốc thật, và sửa được (cách của `AP_AHRS_DCM`). Chỉ chạy khi đang arm.

## 6. Giao thức và log

- Section `flow` (id 16, đọc lúc khởi động): `type`, `baud`. Section `nav` (id 17, có tác dụng ngay): hướng lắp / lật / hệ số / ngưỡng của flow, Position Hold, Rescue, thủ tục failsafe.
- Stream `nav` (id 13): cờ (`POSHOLD`, `POSHOLD_UNAVAILABLE`, `RESCUE`, `RESCUE_AVAILABLE`, `NAV_LEAN`), chất lượng và lý do bỏ mẫu của flow, số mẫu/giây, vận tốc flow đo (trung bình 0,25 giây), trạng thái hai trục của Position Hold, góc nghiêng autopilot yêu cầu, điểm đang giữ, bù gió, giai đoạn và lý do bỏ cuộc của Rescue, khoảng cách tới home, độ cao bay về.
- Stream `estimate` thêm ba bit: `VEL` (có vận tốc trên mặt đất), `FLOW` (flow đang được dùng), `TILT_AIDED`; 0.8.3 thêm `GPS_ALT` (độ cao GPS vào ước lượng trong 2 s vừa qua) và `GPS_VZ` (vận tốc đứng GPS). Section `nav` 0.8.3 thêm `gps_alt` (enum `OFF` / `FALLBACK` / `BLEND`, mặc định `FALLBACK`) và `gps_vel_z` (bật); bản ghi `nav` cũ (27 byte) đọc tiếp được, hai ô mới lấy mặc định.
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

## 11. Log bay 2026-10-11 (firmware 0.8.7) và những gì 0.8.8 sửa

Log `flight_20261011_211839_s2_s0` (loop đặt 6000 Hz, GPS NMEA, `gps_alt` BLEND). Position Hold lắc liên tục, trôi ≈ 15 m.

| Thấy trong log | Nguyên nhân | 0.8.8 |
|---|---|---|
| Lời giải GPS đổi mỗi **1,02 s**; mỗi lần vị trí nhảy tới 1,8 m, vận tốc 2,5 m/s; vận tốc ước lượng lệch GPS thô ~1,5 m/s; góc nghiêng Position Hold chạm ±15° | Module NMEA để mặc định **1 Hz**. Bộ điều khiển trả lời mỗi bước nhảy bằng một cú nghiêng. Không phải do tune | Firmware **xin tốc độ lời giải** khi `gps.auto_config` bật: `$PMTK220`, `$PCAS02`, UBX CFG-RATE (u-blox ở chế độ NMEA), có giới hạn theo baud (9600: để 1 Hz; 19200: 2; 38400: 5; ≥ 57600: 10). Stream `gps.rate` hiển thị tần số đo được (tab Cảm biến). **Position Hold chỉ bằng GPS cần ≥ `nav.ph_min_gps_hz` (5 Hz)**, không thì từ chối, cờ `POSHOLD_SLOW_GPS`, tab Vị trí nói rõ lý do. Optical flow không bị ảnh hưởng |
| \|B\| la bàn đổi 0,50 → 0,40 … 0,58 theo ga; `YAW_FROM_MAG` mất từ 6,5 s tới 15 s (Position Hold không có hướng) | Nhiễu từ dòng motor / dây pin ngay cạnh la bàn | Phần cứng: đưa la bàn ra xa dây nguồn (cột GPS), xoắn dây pin. Bù theo dòng (compassmot của ArduPilot) để sau, cần log có dòng điện |
| Lúc disarm (40,6 s) `est_vz` nhảy ≈ −4 m/s rồi `est_z` trôi tới **−290 m**, baro vẫn ≈ 0; cả chuyến bay thứ hai (51 … 115 s) không có cờ `ALT`, Altitude Hold không chạy | Ước lượng chạy lệch xa, mọi số đo baro bị cổng 5σ loại, không gì kéo về được; độ bất định tự lớn dần mất hàng phút | **Đặt lại theo baro** sau 1,5 s liền không nhận số đo baro nào (như ResetHeight của ArduPilot EKF): z = baro − offset, v = 0; đếm ở `estimate.z_resets`. **Disarm thì vận tốc đứng được kéo về 0** (zero-velocity update, như ArduPilot trên mặt đất). `test_core`: bị đá −4 m/s rồi 3 s không có baro → đặt lại, sai < 0,6 m sau 2 s |
| Vòng lặp: đặt 6000, đạt trung bình 4273 Hz; `t_lock` trung vị 236 µs | Task bay chờ luồng cảm biến nhả khoá (phần telemetry chậm giữ khoá liền một mạch) | Cắt thành các mẩu ngắn (docs/FIRMWARE.md). Còn lại: đọc SPI của IMU |

### Betaflight, ArduPilot làm Position Hold thế nào (để so)
- **Betaflight** (master 10/2026: `flight/position_estimator.c`, `pos_hold_multirotor.c`, `autopilot_multirotor.c`): Kalman 3 trạng thái (vị trí, vận tốc, gia tốc) mỗi trục, nhận GPS vị trí + vận tốc (R theo hAcc / sAcc), optical flow, gia tốc IMU; **mặc định xin GPS 10 Hz** (`gps_update_rate_hz = 10`, u-blox). Một lời giải được dùng lại ở mọi bước ước lượng trong khoảng của nó.
- **ArduPilot** (EKF3 + `AC_PosControl`, mode Loiter / PosHold): EKF 24 trạng thái, bù trễ GPS bằng bộ đệm thời gian, cổng innovation, đặt lại vị trí / độ cao khi bị loại quá lâu; GPS nên ≥ 5 Hz (mặc định u-blox 5 Hz, `GPS_RATE_MS = 200`).
- **Windburst**: Kalman tuyến tính mỗi trục cho vị trí / vận tốc ngang với bù trễ GPS (`horizontal_est.h`), tilt aid, Kalman 4 trạng thái cho độ cao (z, v, bias gia tốc, offset baro). Điểm chung của cả ba: **không bộ lọc nào làm GPS 1 Hz bay êm được** - lời giải đến quá thưa so với tốc độ máy trôi. Vì thế 0.8.8 sửa ở nguồn (tần số GPS) và chặn khi nguồn quá chậm, không đổi bộ lọc.

### Có cần "EKF" cho Position Hold không (todo ★★★★)
Bộ ước lượng ngang **đã là** bộ lọc Kalman (hợp nhất GPS + gia tốc kế, có trễ). Một EKF phi tuyến đầy đủ kiểu EKF3 (tư thế + vị trí + vận tốc + bias + từ trường trong cùng một bộ lọc) là việc rất lớn (ArduPilot: hàng chục nghìn dòng, nhiều năm bay thử), CPU của ESP32-S3 chạy được ở 100 - 200 Hz trên core 1 nhưng **không kiểm chứng được nếu không bay thật nhiều**. Log này không cho thấy lỗi nào mà EKF sửa được hơn: lỗi là GPS 1 Hz, nhiễu la bàn, và ước lượng độ cao không tự hồi phục - cả ba đã xử lý trực tiếp ở trên. Đề nghị: bay lại với GPS ≥ 5 Hz, gửi log; chỉ khi khi đó vẫn lắc mới đáng bàn bộ lọc.

**0.9.0**: EKF đã có, chạy song song (SHADOW), xem `docs/EKF.md`. Chạy lại log 2026-10-11 qua nó: tư thế khớp AHRS trong 1–2°, sai số vận tốc với GPS 1 Hz gần như bằng bộ cũ (GPS là giới hạn), độ cao không trôi.
