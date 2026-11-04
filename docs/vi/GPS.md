# GPS

## Betaflight có những protocol / tính năng GPS nào (đọc từ `io/gps.c`, `io/gps.h`, `pg/gps.c`)
| Betaflight | Windburst |
|---|---|
| Provider **UBLOX** (UBX binary) | ✅ M5/M6, M7/M8, M9/M10 — đúng "phương ngữ" từng thế hệ |
| Provider **NMEA** (GGA, RMC, GSA, GSV) | ✅ + `nmea_custom_commands` gửi từng dòng cho module |
| Provider **MSP** (GPS đưa vào qua MSP) và **VIRTUAL** | ✅ `inject()` (chưa có nguồn MSP thật — thuộc phần telemetry/app) |
| Provider **DRONECAN** | ❌ chưa làm (cần CAN bus) |
| `gps_auto_baud` (quét baud, dò module) | ✅ MON-VER 3 lần/330 ms rồi đổi baud, thứ tự 9600 → … → 230400 như BF |
| `gps_auto_config` | ✅ chuỗi cấu hình từng bước, chờ ACK, thử lại 2 lần rồi bỏ qua bước đó |
| `gps_baud` (mặc định **115200** từ 0.8.0, trước đó 57600; app cho chọn 9600 / 19200 / 38400 / 57600 / 115200 / 230400, giá trị khác → tốc độ chuẩn gần nhất), `gps_update_rate_hz` (mặc định 10) | ✅ |
| `gps_ublox_acquire_model` / `flight_model` | ✅ lúc chưa có fix dùng model "acquire", có fix 3D đầu tiên thì đổi sang "flight" (AIRBORNE<4g) |
| `gps_ublox_use_galileo`, `sbas_mode` (AUTO/EGNOS/WAAS/MSAS/GAGAN/NONE), `sbas_integrity`, `gps_ublox_utc_standard` | ✅ (CFG-SBAS + CFG-GNSS cho M7/M8, CFG-VALSET cho M9/M10) |
| NAV-SAT tắt khi **arm**, bật lại khi disarm (đỡ nghẽn UART) | ✅ |
| Home point đặt lúc **arm** (cần 3D fix + đủ vệ tinh), `gps_set_home_point_once` | ✅ |
| Khoảng cách / hướng về home, quãng đường đã bay (`gps_use_3d_speed`, bỏ qua < 15 cm/s) | ✅ |
| Tiếng "GPS ready" khi đủ điều kiện đặt home | ✅ cờ `take_ready_edge()` cho buzzer |
| Phát hiện mất kết nối (2,5 s), tự dò lại | ✅ |
| **GPS Rescue** (tự bay về home khi failsafe hoặc bật công tắc) | ✅ từ 0.7.0, các pha của Betaflight lái bộ điều khiển của mình; cần la bàn — xem `docs/NAV.md` (**chưa bay thật**) |
| AssistNow Autonomous (`gps_ublox_enable_ana`) | ❌ mặc định tắt trong BF, bỏ qua |

## Cấu trúc
`hal/gps/gps_types.h` (dữ liệu, đơn vị như BF: 1e-7°, cm, cm/s, ddeg) · `ubx_proto.h` (khung UBX, parser, giải mã NAV-PVT/POSLLH/VELNED/SOL/STATUS/DOP/SAT/SVINFO, MON-VER) · `nmea_proto.h` · `gps_engine.h` (state machine) · `gps_nav.h` (hình học + home) · `gps_base.h` (`GpsSensor`, cổng UART Arduino).

## Cách dùng
```cpp
GpsSensor gnss;
gnss.setPins(RX, TX);                 // hoặc getGpsPins() trong hardware_config.h
gnss.config().baud = 115200;          // + provider / update_rate_hz / sbas / use_galileo / models ...
if (!gnss.begin(Serial1)) Serial.println(gnss.failReason());
// vòng lặp nhanh (>= 100 Hz, chỉ rút UART):
gnss.update(millis(), armed);
if (gnss.newSolution()) { auto& s = gnss.solution(); /* lat, lon, alt_cm, vel_n/e/d_cms, num_sat, h_acc_mm ... */ }
arm.gps_fix = gnss.usable();          // link tốt + fix 3D + >= 8 vệ tinh + hdop <= 3.0 + hAcc <= 5 m (đổi bằng setLimits)
float east, north; gnss.home().offset_from_home_cm(east, north);   // đầu vào cho Position Hold / RTH
```

## Đã kiểm chứng
- **Trên PC (187 test, 21 lỗi cố ý đều bị bắt):** bộ mô phỏng u-blox *nghiêm khắc theo thế hệ* — M9/M10 trả NACK cho lệnh legacy, M5–M8 NACK cho CFG-VALSET, M5/M6 không có NAV-PVT/NAV-SAT. Driver phải gửi đúng phương ngữ mới cấu hình được (`nacks == 0` cho cả 5 thế hệ). Kiểm tra: dò baud từ 9600 → 57600 với câu `$PUBX,41` đúng từng byte như bảng của Betaflight, mất kết nối/khôi phục, module không bao giờ ACK (không treo), mặt nạ PRN của SBAS, fix 3D nhưng `gnssFixOK=0` không được tính là fix, giải mã vận tốc NED/tốc độ/hướng/độ cao MSL, NMEA 2 bán cầu, home/khoảng cách/hướng/quãng bay.
- **Build thật cho ESP32-S3** thành công. Lần build này bắt được một lỗi test PC không thấy: Arduino định nghĩa macro `bit(b)` nên hàm cục bộ tên `bit` bị nuốt và mặt nạ SBAS của M9/M10 thành 0. Đã đổi tên; stub Arduino của test giờ có các macro đó (`bit`, `sq`, `radians`, `word`, ...) để bắt tái diễn.

## Chỗ khác Betaflight có chủ đích
- **Kinh độ gần ±180°:** BF tính `to->lon - from->lon` bằng `int32_t`; hai điểm 179,9999° và −179,9999° tràn số và ra chênh lệch ~69,5° thay vì 0,0002° (mình kiểm bằng phép tính wrap thật). Ở đây dùng 64-bit.
- M5/M6 không gọi CFG-GNSS (chip không hỗ trợ), BF vẫn gọi rồi nhận NACK.
- Khi mất kết nối, fix/số vệ tinh xóa ngay lúc timeout (không đợi vòng sau).

## Chưa chạy trên chip thật — kiểm tra theo thứ tự
1. `gnss.begin()` true? Nối module, in `stateName()`, `module()` (M8/M9/M10...), `baud()`; trạng thái phải đi detect baud → change baud → configure → receiving trong vài giây.
2. `configNacks()` phải = 0; nếu khác 0 gửi mình số đó + `module()`.
3. Ngoài trời: `solution().num_sat`, `rateHz()` ≈ `update_rate_hz`, `fix` lên sau vài chục giây.
4. Nếu module đã đổi baud sang giá trị lạ và lưu flash từ trước: bật `auto_baud` (mặc định bật) và chờ — quét một vòng hết khoảng 10–20 s.
5. Module M9/M10 (BN-880 mới, M10 giá rẻ) dùng CFG-VALSET: nếu `module()` in UNKNOWN thì module có chuỗi `hwVersion` lạ — mình coi là M8 và báo `version_assumed()`.

## Tự chuyển baud (0.8.0)
Module u-blox được tìm ở tốc độ nó đang chạy (`gps.auto_baud` bật: thử lần lượt các tốc độ, mỗi tốc độ hỏi MON-VER 3 lần) rồi được chuyển sang `gps.baud` bằng `$PUBX,41` (như Betaflight); mặc định nay là **115200** (đủ chỗ cho NAV-PVT + NAV-SAT ở 10 Hz). Bản ghi cài đặt của bản cũ giữ giá trị cũ (57600): muốn 115200 thì chọn ở tab Cảm biến, mục GPS, "Baud GPS", lưu và khởi động lại. Module NMEA (giao thức `NMEA`) không được chuyển: phải đặt sẵn đúng tốc độ. Lần khởi động sau, module (nếu không có pin dự phòng giữ cấu hình) lại về tốc độ gốc và được chuyển lại. Kiểm chứng: `test_gps` (100000 → 115200, 20000 → 19200), `test_fw` nhóm `gpsbaud` (module ảo ở 9600 / 230400 / 38400 được chuyển sang 115200 / 57600 / 115200 và có fix; tắt tự dò thì module ở 9600 không được tìm). **Chưa thử với module thật.**

## Độ cao GPS trong Altitude Hold (0.8.3)
**Trước 0.8.3:** độ cao GPS chỉ được trộn vào bộ ước lượng khi không có barometer lẫn lidar, và ngay cả khi đó độ cao vẫn bị coi là "không hợp lệ" — nên trên thực tế Altitude Hold **không bao giờ** dùng GPS. Vận tốc đứng của GPS (u-blox `velD`) thì không dùng ở đâu cả.

**Có nên dùng không?** Độ cao GPS trôi 1–3 m trong một phút (vệ tinh di chuyển, tầng khí quyển thay đổi); barometer ổn định hơn nhiều trong vài giây nhưng trôi theo thời tiết và nhiệt độ của mạch (thường 0,5–2 m trong 10 phút). Vì vậy:
- **Vận tốc đứng của GPS: nên dùng luôn** (ArduPilot cũng mặc định dùng). Nó không trôi, sai số 0,1–0,3 m/s, và đúng chỗ barometer yếu nhất: gió cánh quạt, leo / hạ nhanh. Mặc định bật (`nav.gps_vel_z`). Trễ 0,15 s của module được bù bằng gia tốc đo được (như bộ ước lượng ngang). Chỉ module u-blox có số này; module NMEA không có (cờ `has_vel_d`), nên không bị trộn số 0 vào.
- **Độ cao GPS thay cho barometer: nên**, khi không có gì khác (`FALLBACK`, mặc định — như `altitude_source = DEFAULT` của Betaflight). Máy không lắp barometer giờ dùng được Altitude Hold / hạ cánh tự động / GPS Rescue theo độ cao GPS (chính xác cỡ ±1–3 m, đủ để bay ngoài trời ở trên cao, không đủ để bay sát đất). Barometer mất tín hiệu giữa chuyến bay: GPS nhận việc **không giật** (mốc GPS − z được theo dõi suốt lúc barometer còn chạy), Altitude Hold không bị ngắt quãng. Blocker `ALTHOLD_NO_BARO` chỉ còn khi không có barometer **và** không có GPS fix.
- **Trộn độ cao GPS với barometer: tuỳ** (`BLEND`, không mặc định). Lợi cho chuyến bay dài (10 phút trở lên, thời tiết đổi), hại nhẹ với chuyến ngắn: một mét trôi của GPS thì sẽ bị theo một phần. Cách làm: lấy trung bình (GPS − z) của 20 số đo đầu sau khi arm làm mốc, sau đó độ cao GPS so với mốc đó sửa *offset* của barometer với sigma ≥ 3 m (hằng số thời gian cỡ một phút). Độ cao GPS lệch khỏi ước lượng quá 4 m (mất / thêm vệ tinh) thì bị bỏ, 10 lần liền thì lấy mốc mới — không kéo `z`. Mất GPS quá 3 s cũng lấy mốc mới.

Cài đặt ở tab **Vị trí**, khung **"Độ cao từ GPS"** (có hiệu lực ngay, không cần khởi động lại); khung đó cũng cho biết độ cao / vận tốc đứng GPS có đang được dùng không (bit `GPS_ALT`, `GPS_VZ` của stream `estimate`). Độ cao GPS chỉ có khi đã có điểm home (GPS fix lúc arm).

**Sửa kèm:** với giao thức NMEA, vận tốc bắc / đông trước đây luôn bằng 0 (chỉ UBX điền nó), nên bộ ước lượng ngang được báo "đứng yên" khi máy đang bay — Position Hold / Rescue với module NMEA sẽ hỏng. Nay tính từ tốc độ + hướng của câu RMC.

**Kiểm chứng trên PC:**
- `test_core` [vertical estimator: GPS]: vận tốc đứng GPS làm sai số tốc độ leo giảm một nửa (0,035 so với 0,071 m/s RMS) và tự nó giữ được tốc độ leo khi không có độ cao; barometer trôi 1 m/phút trong 5 phút: lệch 5,0 m nếu chỉ có baro, 0,2 m khi BLEND; GPS lệch 25 m từ đầu không ảnh hưởng (mốc); GPS nhảy 8 m: lấy mốc mới, ước lượng dịch < 0,4 m (không có cổng 4 m thì dịch 6,5 m); barometer im lặng: GPS nhận việc, ước lượng dịch 0,14 m; chỉ có GPS: z bám độ cao GPS, GPS nhảy 30 m thì sau 10 số đo z được đặt lại.
- `test_flight`: chỉ GPS (nhiễu 1 m, trễ 150 ms), Altitude Hold ở 5 m trong 20 s: lệch < 0,3 m; `OFF`: Altitude Hold không có độ cao; Altitude Hold 3 phút, barometer trôi 1 m/phút: máy tụt 2,8 m với `FALLBACK`, chỉ 0,65 m với `BLEND`; barometer tắt ở 5 m: Altitude Hold không ngắt, máy trong 0,33 m suốt 20 s; NMEA: vận tốc đứng không được dùng.
- `test_fw` nhóm `gpsalt` (firmware đầy đủ trên mạch ảo, GPS ảo không nhiễu): không barometer + GPS fix → arm được với công tắc Altitude Hold, giữ độ cao, bit `GPS_ALT` + `GPS_VZ`; `OFF` → `ALTHOLD_NO_BARO`; có barometer: `FALLBACK` dùng vận tốc đứng, không dùng độ cao; `BLEND` dùng cả hai; `gps_vel_z` tắt → không dùng.
- `test_gps`: RMC → vận tốc bắc / đông, NMEA không có vận tốc đứng, UBX có.

**Chưa thử với GPS thật.** GPS ảo trong test chỉ có nhiễu trắng (test_flight) hoặc không nhiễu (test_fw); GPS thật còn trôi chậm và nhảy khi vệ tinh đổi, nên con số ngoài trời sẽ kém hơn. Lần đầu thử: (1) có barometer, để mặc định, bay Altitude Hold như cũ — chỉ khác là bit `GPS_VZ` sáng; (2) thử `BLEND` trong một chuyến dài, so độ cao lúc hạ cánh; (3) chỉ khi đó mới thử máy không barometer, chỗ thoáng, trên 5 m, tay sẵn sàng tắt công tắc.
