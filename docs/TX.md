# Tay phát DIY (ESP32-C5 + nRF24) và receiver nRF24 — 0.38

Tay phát tự làm cho Windburst: một mạch ESP32-C5 DevKitC-1, hai joystick, một module nRF24L01(+). Phía drone, mạch bay nhận bằng một module nRF24 cắm vào bus SPI thứ hai (giao thức receiver `NRF24`). Cả hai đầu là firmware của dự án này; app nối được với cả hai (cắm tay phát thì chỉ hiện các tab của tay phát).

Thiết kế: `docs/superpowers/specs/2026-10-30-tx-nrf24-design.md`. Kế hoạch và các quyết định khi làm: `docs/superpowers/plans/2026-10-30-tx-nrf24.md`.

**Chưa thử trên phần cứng.** Mọi thứ dưới đây đã chạy trên PC (driver nRF24 với chip mô phỏng ở mức thanh ghi, hai firmware thật trong cùng một "không khí" mô phỏng có mất gói). Các bước thử trên mạch: cuối file.

## 1. Phần cứng

### Tay phát (ESP32-C5 DevKitC-1)
Chân mặc định (lấy theo dự án cũ `Desktop/ESP32/ProjectWindburst`, đổi được trong app):

| Phần | Chân |
|---|---|
| joy1 X / Y (roll / pitch) | GPIO4 / GPIO5 |
| joy2 X / Y (yaw / ga) | GPIO3 / GPIO2 |
| nRF24 SCK / MISO / MOSI | GPIO7 / GPIO6 / GPIO8 |
| nRF24 CSN / CE | GPIO24 / GPIO23 |

- Joystick: chân giữa của biến trở vào ADC, hai đầu vào 3,3 V và GND.
- nRF24: cấp **3,3 V** (không phải 5 V), thêm **tụ 10 µF** sát hai chân nguồn của module (module nRF24 rất kén nguồn; thiếu tụ là nguyên nhân mất sóng hay gặp nhất). Module có PA/LNA cần nguồn 3,3 V khỏe.
- App nối qua **cổng nào cũng được** trên DevKit: cổng "UART" (chip cầu USB-UART CH343 / CP2102, **230400 baud** = mặc định của app) hoặc cổng "USB" (USB Serial/JTAG của chính C5, baud nào cũng được). Firmware nghe cả hai và trả lời trên cổng app vừa nói.
- App không thấy cổng nào: đó là chuyện của Windows, không phải firmware. Mở Device Manager khi cắm board: thiết bị "USB Serial" có dấu chấm than (không nằm trong nhóm Ports) = chưa có driver. Chip WCH (CH340 / CH341 / CH343, VID 1A86) cần driver của WCH (CH341SER / CH343SER) từ wch-ic.com; cài xong cắm lại sẽ có cổng COM. Cổng "USB" của C5 (VID 303A) dùng driver có sẵn của Windows. Cáp chỉ có dây nguồn (không có dây dữ liệu) cũng làm cổng không hiện.

### Mạch bay (ESP32-S3)
Module nRF24 nằm trên **bus SPI thứ hai** (HSPI), với 5 chân riêng: `nrf_sck`, `nrf_miso`, `nrf_mosi`, `nrf_cs`, `nrf_ce` (tab Chân và cổng, nhóm "Receiver nRF24"). Bus này cũng là bus của thẻ SD kiểu SPI, cảm biến PMW3901 và dải LED:
- có thẻ SD kiểu SPI hoặc PMW3901 → receiver nRF24 không chạy, boot report: `rx: FAILED - the second SPI bus is taken (...)`;
- có dải LED → receiver chạy, dải LED không chạy (`led: FAILED - the second SPI bus is taken (the nRF24 receiver)`).

Các bản STM32 chưa có receiver nRF24 (`rx: FAILED - the nRF24 receiver is not in the STM32 builds yet`).

## 2. Nạp firmware tay phát
```
pio run -e tx_c5 -t upload          (từ PowerShell lần đầu: bộ cài toolchain RISC-V của pioarduino không chạy trong Git Bash)
```
Boot report (tab Hệ thống của tay phát): `radio: nRF24 found (...)` hoặc `NOT FOUND` (dây, nguồn, tụ), `sticks: ADC pins ...`, `settings: NVS`.

## 3. Dùng lần đầu
1. **Hiệu chỉnh cần** (tab Joystick → Bắt đầu hiệu chỉnh): đẩy từng cần ra hết bốn phía, thả về giữa, bấm Xong. Trục không được đẩy đủ xa (dưới 500 đơn vị ADC) giữ hiệu chỉnh cũ. Tay tự lưu vào flash.
2. **Gán trục** nếu cần (mặc định: roll = joy1 X, pitch = joy1 Y, ga = joy2 Y, yaw = joy2 X), **đảo chiều** trục nào chạy ngược (xem kênh ở tab Tổng quan: đẩy roll sang phải phải ra gần 2000).
3. **Chế độ cần ga** (tab Joystick) — joystick tự về giữa:
   - **ALT / POS**: ga = vị trí cần, giữa = 1500. Dùng khi drone bay ALT HOLD / POS HOLD (giữa = giữ độ cao).
   - **Thủ công** (mặc định): đẩy cần lên thì ga tăng dần (tốc độ: `throttle_rate` % mỗi giây), buông thì ga giữ nguyên, kéo xuống thì giảm. Trong 15 % quanh giữa cần không làm ga đổi (cần rẻ thường nằm lệch tâm vài %). Dùng cho ANGLE / ACRO. Khi drone báo chưa arm (hoặc chưa từng nghe thấy drone), ga luôn là 1000: arm bằng cần rồi buông tay không làm drone bay lên. Đang bay mà mất tin của drone (drone đã báo arm, rồi im): ga **giữ nguyên** chỗ cũ, không về 1000, không tăng giảm, cho tới khi nghe lại drone.
   - Tab Tổng quan cảnh báo khi chế độ ga không hợp chế độ bay drone báo về.
4. **Mạch bay**: tab Receiver → giao thức `nRF24 (tay phát DIY Windburst)`, đặt 5 chân nRF24 (tab Chân và cổng), lưu và khởi động lại. Arm bằng cần: tab Mode → "Arm bằng cần" (ga và pitch xuống hết, yaw và roll về hai phía ngược nhau, giữ 1 giây). Không có công tắc thì chế độ bay chọn bằng một dải luôn bật trên AUX1 (kênh 5 luôn 1000: dải 900..1100).
5. **Bind**: đặt tay sát drone.
   - Drone chưa bind lần nào: bật drone, trong 60 giây bấm "Bind với drone" ở tab Radio của tay phát.
   - Drone đã bind với tay khác: bấm "Bind nRF24" ở tab Receiver của drone (nghe 30 giây), rồi "Bind với drone" ở tay phát.
   - Xong: tab Radio của tay ghi "đã bind", tab Receiver của drone ghi "đã bind"; cả hai tự lưu địa chỉ mới. Boot report drone: `rx: bound to ...`.

## 4. Giao thức sóng (`src/radio/wb_radio.h`)
- nRF24 Enhanced ShockBurst: CRC 16 bit phần cứng, auto-ACK, dynamic payload, **ACK payload** (telemetry của drone đi nhờ gói ACK).
- Mặc định 250 kbps, công suất tối đa, **100 gói/s**; chọn được 50 / 100 / 150 gói/s, 250 kbps / 1 / 2 Mbps. Số lần phát lại tự động tính sao cho mọi lần phát của một gói xong trong 80 % chu kỳ (100 gói/s ở 250 kbps: 3 lần phát lại).
- Gói điều khiển 13 byte: loại, số thứ tự, 8 kênh × 10 bit (1000..2000 µs), CRC8 DVB-S2. Gói hỏng / sai CRC8 / kênh ngoài khoảng bị bỏ cả gói (`rc.bad_frames`); gói số thứ tự cũ (bản phát lại) bị bỏ.
- Telemetry 7 byte (ACK): đã arm, failsafe, chế độ bay (chế độ đang chạy, hoặc chế độ phi công đã chọn khi còn dưới đất), pin (cV), chất lượng sóng phía drone.
- Bind: địa chỉ cặp 5 byte và kênh tần số ngẫu nhiên (tay tạo; địa chỉ không bắt đầu bằng 0x00 / 0x55 / 0xAA / 0xFF; kênh 74..82 = 2474..2482 MHz, giữa WiFi kênh 11 và mép trên dải 2,4 GHz), trao đổi trên địa chỉ "WBND!" kênh 83 (2483 MHz), công suất thấp nhất. Drone chỉ nhận lời mời mạnh hơn -64 dBm (RPD: tay phải đặt sát drone, lời mời của bộ khác ở xa bị bỏ qua). Drone đáp bằng cách trả lại địa chỉ trong ACK; cả hai chuyển sang địa chỉ / kênh mới; mỗi bên chỉ coi là đã bind khi có gói điều khiển qua lại ở đó (không có trong 2 giây → quay lại mời / nghe bind).
- Đổi kênh tần số / tốc độ / số gói (tab Radio của tay): chỉ khi đang nghe thấy drone và drone chưa arm (không thì các ô quay lại giá trị đang chạy ngay). Gửi trên cài đặt cũ, cả hai đổi sau ACK; không có gói nào ở cài đặt mới trong 1 giây → cả hai quay về cài đặt cũ. Drone bỏ qua gói đổi cài đặt khi đang arm. Cả hai tự lưu vào flash cài đặt đang thực sự chạy (đã xác nhận, hoặc cài đặt cũ đã quay về).
- Chưa bind: tay không gửi gì, drone không nhận gói điều khiển nào (hai bộ chưa bind không lái được drone của nhau qua địa chỉ mặc định).
- Khi drone đang arm (hoặc đã báo arm rồi mất tin), tay phát coi như đang arm: app không đổi được cài đặt, không về mặc định, không khởi động lại tay, không hiệu chỉnh cần.
- Mỗi 100 ms cả hai đầu đọc lại thanh ghi của module nRF24; module bị sụt nguồn (mất cấu hình) thì được cài lại ngay.
- Mất sóng: như mọi receiver, `receiver.rx_timeout_ms` → mất link; dưới đất chặn arm (`RX_LOST`), trên không vào failsafe.

## 5. Giới hạn
- Một kênh tần số cố định (chưa nhảy tần). Dải 2,4 GHz được phép chỉ tới 2483,5 MHz: kênh 0..83. Kênh 84..125 (2484..2525 MHz) chip vẫn phát được nhưng nằm ngoài dải được phép ở hầu hết các nước (gồm Việt Nam): không dùng. Bản 0.38.0 mặc định kênh 100 (ngoài dải) - đã sửa ở 0.38.1 (mặc định 76, bind chọn 74..82).
- Trường hợp hiếm: đổi kênh qua sóng mà mọi ACK đều mất trong 1 giây thì drone đã sang kênh mới (và lưu) còn tay quay về kênh cũ → mất sóng. Bind lại là xong.
- Module nRF24L01 đời cũ (không có dấu +) và một số hàng nhái cần lệnh ACTIVATE: firmware tự gửi. Loại này không có tốc độ 250 kbps: chọn 1 Mbps.
- Tay chưa có công tắc, màn hình, pin, LED; kênh 5..8 luôn 1000.
- Receiver nRF24 chưa có trên STM32.
- Tầm xa: chưa đo (module thường không PA ~ vài chục mét; module PA/LNA xa hơn).

## 6. Kiểm chứng trên PC
- `test_radio` (driver + chip nRF24 mô phỏng + không khí: thanh ghi, gói, sai địa chỉ / kênh / tốc độ, mất gói, gói hỏng, thời gian phát lại; mã hoá gói, CRC, số thứ tự, địa chỉ).
- `test_tx` (cần, hiệu chỉnh, hai chế độ ga, đường sóng: link / mất link / telemetry / bind / bind hết giờ / đổi kênh / drone không theo; app host: lệnh, stream, boot report, cờ chưa lưu).
- `test_rx` (receiver: gói, gói hỏng, gói cũ, LQ, bind, cửa sổ 60 s, đổi cài đặt, quay về).
- `test_fw` nhóm `nrf24` và `txfly`: TxApp thật lái FlightApp thật: cần tới kênh, arm bằng cần, ga Thủ công buông tay không bay lên, ALT / POS giữ độ cao, tắt tay → mất link / failsafe, mất 30 % gói vẫn bay, hai bộ cạnh nhau không nghe lẫn, bind lần đầu, đổi kênh qua sóng (kể cả sau khởi động lại), telemetry về tay, bus SPI bị chiếm, chân chưa đặt.
- App: `test_e2e.py::test_the_simulated_transmitter`, `test_gui_browser.py` (`test_tx_tabs_only`, `test_tx_sticks_live_and_calibrate`, `test_tx_bind_and_throttle_warning`, `test_receiver_nrf24_block`). `wb_sim` có tay phát mô phỏng ở cổng +2 (`socket://127.0.0.1:5762`), lệnh tester `stick <1..4> <0..4095>`, `txpower on|off`.

## 7. Thử trên mạch (chưa làm)
0. (thêm sau review) Rút nguồn riêng module nRF24 của drone 1 giây rồi cắm lại khi đang bind: sóng phải tự về; để cần ga Thủ công nằm hơi lệch tâm: ga không tự bò; đang hover cắt sóng 2 giây: ga không về 1000.
1. Tay phát trên bàn: boot report `radio: nRF24 found`; tab Tổng quan: chấm di theo cần, hiệu chỉnh.
2. Drone trên bàn (tháo cánh): receiver NRF24, bind lần đầu trong 60 giây, tab Receiver: LQ ~100 %, 100 gói/s.
3. Arm bằng cần ở chế độ ga Thủ công, buông tay: motor chỉ ở idle.
4. Đi xa dần (tháo cánh, có người giữ) tới khi mất sóng: tab Receiver báo mất link, quay lại thì có lại.
5. Gần router WiFi: so LQ ở kênh bind đã chọn (74..82) với kênh 40 (giữa WiFi kênh 6).
6. Bay thử ALT HOLD với ga ALT / POS, rồi ANGLE với ga Thủ công.
