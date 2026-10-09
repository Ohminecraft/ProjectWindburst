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
