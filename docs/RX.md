# Receiver: CRSF, SBUS, IBUS, Spektrum, PPM, PWM — `src/hal/rx`

| File | Nội dung |
|---|---|
| `rx_proto.h` | **portable**: `rx::CrsfDecoder`, `rx::SbusDecoder`, `rx::IbusDecoder` (luồng byte → 16 kênh µs), `rx::Link` (link ok / mất, giữ kênh cuối) |
| `rx_pulse.h` | **portable**: `rx::PpmDecoder`, `rx::PwmDecoder` (thời điểm cạnh xung → kênh), `rx::EdgeRing` (hàng đợi ngắt → luồng cảm biến) |
| `rx_base.h` | `RcReceiver`: decoder + `HardwareSerial` (baud, 8E2, đảo mức), hoặc ngắt GPIO cho PPM / PWM |

```cpp
RcReceiver rc;
rc.setTimeoutMs(500);
rc.begin(rx::PROTO_CRSF, Serial2, rxPin, txPin);     // false + failReason() nếu giao thức chưa có / chưa đặt chân
// ~1 kHz:
rc.update(micros(), millis());
rc.channels();  rc.linkOk(millis());  rc.rateHz();  rc.linkQuality();
```
`FlightApp` đã làm sẵn việc này (UART2, giao thức từ `receiver.protocol`, chân từ `pins.rx_rx / rx_tx`).

| Giao thức | UART | Khung | Đổi sang µs (giống Betaflight) |
|---|---|---|---|
| CRSF (Crossfire, **ExpressLRS**) | 420000 8N1 | `C8, len, type, payload, crc8 DVB-S2`; `0x16` = 16 kênh × 11 bit; `0x14` = link statistics (LQ, RSSI) | `0,62477·raw + 881` (172→988, 992→1500, 1811→2012) |
| SBUS | 100000 **8E2, đảo mức** (UART ESP32 tự đảo, không cần mạch đảo) | `0F, 22 byte, flags, 00`; flags bit2 = mất khung, bit3 = failsafe | `5/8·raw + 880` |
| IBUS (FlySky) | 115200 8N1 | `20 40, 14 × u16, checksum = 0xFFFF − tổng` | giá trị là µs |
| Spektrum 2048 (`SPEKTRUM`: DSMX, DSM2 11 ms) | 115200 8N1 | 16 byte: `fades, system, 7 × u16 big-endian`; bit 14..11 = số kênh, 10..0 = giá trị; `FFFF` = ô trống | `988 + giá trị / 2` |
| Spektrum 1024 (`SPEKTRUM1024`: DSM2 22 ms) | 115200 8N1 | như trên, bit 13..10 = số kênh, 9..0 = giá trị | `988 + giá trị` |
| PPM | chân `pins.ppm`, ngắt cạnh lên | thời gian giữa hai cạnh cùng loại = một kênh; khoảng > 2,7 ms = đồng bộ | là µs |
| PWM | chân `pins.rx_pwm[0..7]`, ngắt hai cạnh | cạnh lên → cạnh xuống trên từng chân = một kênh | là µs |

Kênh bị kẹp trong 885..2115 µs. Thứ tự kênh mặc định AETR (roll, pitch, throttle, yaw, aux1…), đổi ở section `receiver`.

**Đồng bộ khung**: UART được đọc theo khối (~1 kHz) nên không dựa vào thời gian từng byte.
- CRSF, IBUS: dựa vào checksum; khung hỏng được quét lại từ byte thứ hai (header giả không che khung thật).
- SBUS không có checksum: dùng khoảng lặng ≥ 4 ms giữa các khung (ngưỡng 2,5 ms) + byte đầu `0x0F` + byte cuối.

**Mất sóng**: `Link::ok()` = có khung **không mang cờ failsafe của receiver** trong `rx_timeout_ms`. Khung failsafe của SBUS không giữ link sống và không ghi đè cần điều khiển. CRSF/ELRS báo mất sóng bằng cách ngừng gửi khung kênh. Khi link mất, core chạy failsafe (xem `docs/CORE.md`).

Lưu ý ELRS: receiver ELRS nói CRSF qua UART → chọn `CRSF`.

**Spektrum** (satellite DSMX / DSM2, firmware 0.8): không có checksum → khung tìm từ khoảng lặng > 4 ms giữa các khung (một khung dài 1,4 ms), rồi đếm từng 16 byte (đọc dồn sau khi vòng lặp bị treo vẫn đúng). Hơn 7 kênh thì các khung thay phiên nhau mang kênh: decoder giữ giá trị cuối của từng kênh. Kênh 0..11 (như Betaflight), kênh chưa nhận = 1500. Thứ tự kênh của Spektrum là **TAER**: chọn TAER ở tab Receiver. **Chưa làm bind** (`spektrum_sat_bind` của Betaflight): bind satellite bằng receiver chính hoặc tay phát trước.

**PPM / PWM** (firmware 0.8): ngắt GPIO chỉ ghi thời điểm (`micros()`) và mức vào `rx::EdgeRing` (64 cạnh); giải mã chạy ở luồng cảm biến. Giới hạn theo Betaflight (`rx_pwm.c`): xung hợp lệ 0,75..2,25 ms; PPM 4..12 kênh, số kênh tự học từ 25 khung giống nhau liên tiếp (một khung hỏng không đổi số kênh, cũng không lọt qua); PPM dương hay âm đều được (chỉ xét một loại cạnh). PWM: một khung được báo khi kênh 1 xong một xung **và** mọi kênh đã nối có xung tốt trong 100 ms (một dây tuột → mất sóng sau `rx_timeout_ms`, không bay tiếp với một kênh cũ). Kênh không có = 1500. PPM / PWM không dùng UART2: cảm biến optical flow MSP lấy được UART2. Đặt failsafe của receiver PWM / PPM là "không phát xung": receiver phát xung failsafe thì firmware coi như vẫn có sóng.

## Telemetry CRSF (firmware 0.8) — `crsf_telemetry.h`

Có chân `pins.rx_tx` và giao thức `CRSF` → firmware gửi khung telemetry vào RX của receiver (ELRS / Crossfire nối UART hai dây, full duplex); tay phát EdgeTX / OpenTX "Discover sensors" thấy RxBt, Curr, Capa, Bat%, Ptch / Roll / Yaw, FM, GPS, Alt, Sats, VSpd. Bố cục và đơn vị như `telemetry/crsf.c` của Betaflight:

| Khung | Nội dung |
|---|---|
| `0x08` pin | điện áp u16 (0,1 V), dòng u16 (0,1 A), đã dùng u24 (mAh), còn lại u8 (%) |
| `0x1E` tư thế | pitch, roll, yaw i16 (rad × 10000; yaw gửi trong −π..π) |
| `0x21` chế độ bay | chuỗi: `!FS!` failsafe, `RTH` rescue, `POSH`, `ALTH`, `ANGL`, `STAB`, `ACRO`; chưa arm: thêm `*`, hoặc `!ERR` khi có thứ chặn arm |
| `0x02` GPS (khi có module GPS) | lat, lon i32 (độ × 1e7; 0 khi chưa fix), tốc độ u16 (km/h × 10), hướng u16 (độ × 100), độ cao u16 (m + 1000, độ cao ước lượng), số vệ tinh |
| `0x07` vario (khi có ước lượng độ cao) | tốc độ lên xuống i16 (cm/s) |

Cả danh sách gửi 10 lần mỗi giây, từng khung một (Betaflight: `CRSF_CYCLETIME_US` / số khung). Khung được dựng trong luồng cảm biến (giữ khoá, vài µs), ghi ra UART sau khi nhả khoá. Mất sóng thì không gửi. Báo cáo khởi động: `rx: CRSF on UART2, telemetry on its TX pin`. Kiểm chứng: `test_rx` `[CRSF telemetry]` (byte tính tay, kẹp giá trị, chuỗi chế độ, lịch gửi qua lúc đồng hồ tràn, không gửi dồn sau một lúc dừng), `test_fw` nhóm `crsf` (khung firmware ghi ra chân TX của mạch ảo được đọc lại bằng CRC riêng, so với telemetry của chính firmware và với mạch ảo: nghiêng 12°, 12 A, GPS fix, đang lên cao; không có TX / SBUS / mất sóng: không gửi). **Chưa thử với receiver và tay phát thật** (dấu của pitch trên màn hình tay phát, ELRS có chuyển hết các khung không).

## Kiểm chứng

`test/test_rx.cpp`: encoder viết độc lập (vòng lặp từng bit), các điểm chuẩn của Betaflight, 16 kênh khác nhau (bắt lỗi thứ tự bit/kênh), khung hỏng, header giả, 20 000 byte ngẫu nhiên không sinh khung ma, đọc từng byte, cờ SBUS, SBUS2, khung cụt + khoảng lặng, timeout qua thời điểm tràn `millis()`.
`test/test_fw.cpp`: firmware thật nhận CRSF từ "tay phát" mô phỏng, arm, bay, tắt tay phát → failsafe.
Spektrum / PPM / PWM: `test_rx` (khung dựng từng bit theo mô tả, 2048 / 1024, kênh chia hai loại khung, số kênh 12..15 bị bỏ, khung cụt, nhiều khung đọc dồn, byte ngẫu nhiên; PPM: học số kênh, xung quá ngắn / quá dài, 3 / 12 / 13 kênh, khung dài ngắn khác nhau, đồng hồ tràn 32 bit; PWM: kênh cũ quá 100 ms, xung sai, cạnh xuống lẻ; hàng đợi đầy) và `test_fw` nhóm `receivers` (`WB_ONLY=receivers`): firmware thật trên mạch ảo với satellite Spektrum ảo trên UART2 và receiver PPM / PWM ảo phát cạnh xung vào ngắt GPIO ảo đúng thời điểm: kênh khớp tới 1 µs, tốc độ khung, arm và bay, mất sóng, đổi số kênh PPM, một dây PWM tuột, cài đặt sai, trùng chân, UART2 được nhường cho optical flow. Trang thật: `test_ppm_and_spektrum_receivers_on_the_page`.

**Chưa kiểm chứng trên mạch**: UART 420 kbaud của ESP32 có rớt byte khi vòng bay bận không (bộ đệm RX đã đặt 512 byte), đảo mức SBUS bằng phần cứng.
**Chưa kiểm chứng trên mạch (Spektrum / PPM / PWM)**: độ trễ ngắt GPIO thật (ngắt khác, ghi flash tắt cache → mất một khung), receiver 5 V cần chia áp, satellite thật (byte `system`, thứ tự khung), bind.
**Chưa làm**: nRF24 (WireEnvelope) — `begin()` từ chối kèm lý do; bind Spektrum.
