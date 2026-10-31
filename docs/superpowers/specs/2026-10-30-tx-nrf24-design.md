# Tay phát DIY (ESP32-C5) + receiver nRF24 — thiết kế

Ngày: 2026-10-30. Người dùng duyệt trong chat: mục tiêu "chạy ổn định là được, không cầu kì", app khi cắm tay phát chỉ hiện các tab của tay phát, chia **nhiều tab nhỏ** (phương án A: Tổng quan / Joystick / Radio / Hệ thống), các lựa chọn mặc định tôi nêu (chỉ ESP32-S3 trước, 100 gói/s, 250 kbps, địa chỉ riêng mỗi lần bind, chưa nhảy tần) — "ok rồi tiến hành đi". Những chỗ ghi **(quyết định của tôi)** là tôi tự chọn, người dùng chưa xét riêng.

## 1. Ý định

Người dùng bay thật bằng RadioMaster Pocket (ELRS → CRSF). Tay phát DIY là dự án riêng cho đúng tinh thần tự làm: một mạch ESP32-C5 + module nRF24L01(+), hiện chỉ có **2 joystick** (tay test, chưa có công tắc, màn hình, pin). Phía drone, mạch bay Windburst nhận bằng một module nRF24 cắm vào SPI (giao thức receiver `NRF24`, hiện `begin()` từ chối "chưa làm").

"Xong" = **bay được ổn định bằng tay này**: lái mượt, không giật, không mất sóng vô cớ; mất sóng thì mạch bay vào failsafe như với mọi receiver khác. Không cần màn hình đẹp hay nhiều tính năng.

Dự án cũ (`Desktop/ESP32/ProjectWindburst`, `src/common/radio_link.*`, `wire_protocol.h`, `radio_packet.h`, `src/tx/*`, `src/rx/rc_link_nrf24.h`) là điểm xuất phát: cùng phần cứng tay, cùng ý tưởng (gói điều khiển TX → RX, telemetry RX → TX đi nhờ gói ACK tự động của nRF24, cài đặt radio đẩy qua sóng). Không cần tương thích với định dạng gói cũ — cả hai đầu viết lại.

## 2. Mục tiêu, không làm, phải tránh

Mục tiêu
- Firmware tay phát (env `tx_c5`, hiện chỉ in một dòng) đọc 2 joystick, gửi tối thiểu 8 kênh qua nRF24, nhận telemetry của drone, nói chuyện với app bằng WBP như mạch bay.
- Mạch bay (ESP32-S3) nhận giao thức `NRF24` như một receiver thật: 16 kênh RC vào `FlightApp` như CRSF, `linkOk` / failsafe theo `receiver.rx_timeout_ms`, chất lượng sóng hiện ở tab Receiver.
- App: cắm tay phát → chỉ 4 tab của tay phát. Tab Receiver của mạch bay có thêm phần nRF24 (bind, trạng thái). Tab Pins của mạch bay có chân nRF24.
- Arm / disarm bằng cần gạt (`general.stick_arm`, đã có trong firmware mạch bay) — tay 2 joystick không cần công tắc.
- Kiểm chứng trên PC như phần còn lại của dự án: chip nRF24 mô phỏng, "không khí" mô phỏng giữa hai chip (mất gói, nhiễu), FIL tay phát ↔ mạch bay, test app với tay phát mô phỏng.

Không làm (lần này)
- Receiver nRF24 trên các bản STM32 (viết driver qua lớp `port` để thêm sau dễ, nhưng không nối vào `main_stm32.cpp`).
- Nhảy tần (hopping). Một kênh tần số cố định, chọn lúc bind, đổi được trong app.
- Màn OLED, LED neopixel, còi, pin của tay phát; công tắc (định dạng gói và cài đặt chừa chỗ cho kênh AUX từ chân công tắc, nhưng tay hiện không có).
- Tay phát làm "module" cho máy khác (xuất PPM / CRSF ra ngoài), chế độ học sinh / huấn luyện, nhiều model.
- Tương thích ngược với gói `WireEnvelope` / `RadioPacket` của dự án cũ.

Phải tránh
- **Lái nhầm drone khác**: hai bộ Windburst cùng bật không được nghe lẫn nhau (địa chỉ cố định `0xF0F0F0F0E1` của dự án cũ là đúng lỗi này).
- Motor quay vì một gói hỏng / gói cũ: mọi gói kiểm CRC, gói số thứ tự cũ bị bỏ, kênh ngoài khoảng bị bỏ cả gói.
- Mất sóng mà mạch bay vẫn giữ kênh cuối như còn sóng: hết `rx_timeout_ms` không có gói đúng → `linkOk = false`, failsafe như các receiver khác.
- Vòng điều khiển 4 kHz bị chậm vì radio: driver nRF24 chạy trong luồng sensor (`slow_update`, ~1 kHz), không trong `flight_cycle`, và không chờ bận trên SPI lâu.

## 3. Phần cứng (theo dự án cũ)

Tay phát ESP32-C5 (`esp32-c5-devkitc1-n8r4`), chân mặc định của dự án cũ (`src/common/pins.h`): joystick 1 X/Y = GPIO4/5, joystick 2 X/Y = GPIO3/2 (ADC1), nRF24 MISO 6, SCK 7, MOSI 8, CE 23, CSN 24. Công tắc 26/25, OLED 10/9, neopixel 27 — có trong dự án cũ, ngoài phạm vi lần này. Mọi chân là cài đặt của tay phát (sửa được trong app), không hằng số biên dịch **(quyết định của tôi, theo cách tab Pins của mạch bay)**.

Mạch bay ESP32-S3: module nRF24 trên **bus SPI thứ hai** (HSPI, bus mà thẻ SD kiểu SPI, PMW3901 và dải LED đang dùng) với chân riêng: `nrf_sck`, `nrf_miso`, `nrf_mosi`, `nrf_cs`, `nrf_ce` (+ `nrf_irq` tuỳ chọn, không bắt buộc). Bus đã bị chiếm (SD SPI / PMW3901 / LED) → receiver không chạy, boot report nói rõ vì sao (như `spi2_taken` của dải LED). Dùng chung bus với thiết bị khác: để người làm quyết định, tối thiểu phải báo lỗi rõ thay vì hỏng âm thầm.

## 4. Sóng

### 4.1 Lớp vật lý (nRF24L01+)
- CRC phần cứng 16 bit, auto-ACK, dynamic payload, ACK payload — bật ở cả hai đầu (như dự án cũ).
- Mặc định **250 kbps** (tầm xa nhất), công suất MAX, **100 gói/s** (10 ms). Chọn được: tốc độ 250 k / 1 M / 2 M, công suất MIN..MAX, 50 / 100 / 150 gói/s **(150 là quyết định của tôi; người làm kiểm thời gian phát + ACK + số lần phát lại ở 250 kbps có lọt trong chu kỳ không, không lọt thì không cho chọn tổ hợp đó)**.
- Số lần phát lại tự động: đủ nhỏ để một gói không lấn sang chu kỳ sau — gói mới quan trọng hơn gói cũ phát lại.

### 4.2 Bind (quyết định của tôi — người dùng không trả lời câu hỏi bind, nói "tiến hành")
- Cặp tay/drone dùng **một địa chỉ 5 byte ngẫu nhiên + một kênh tần số**, tạo ở tay phát lúc bind, lưu ở cả hai đầu.
- Bind đi trên một địa chỉ và kênh bind cố định, công suất thấp (bind sát nhau).
- Tay phát: nút "Bind với drone" ở tab Radio của app → tay phát phát gói bind trong vài chục giây, dừng khi drone xác nhận (ACK payload).
- Mạch bay: nút "Bind nRF24" ở tab Receiver của app; **và** mạch bay chưa từng bind thì tự nghe bind trong khoảng 60 s đầu sau khi bật (tay test không có nút). Đã bind thì không tự nghe nữa. Bind xong: ghi section receiver vào flash, báo trong boot report / app.
- Không bao giờ bind khi đang arm.

### 4.3 Gói tay phát → drone (định dạng do người làm chốt, theo các ràng buộc)
- ≤ 32 byte, có loại gói (điều khiển / bind / cài đặt radio), số thứ tự, **tối thiểu 8 kênh** ở độ phân giải ≥ 10 bit, kiểm tra thêm bằng phần mềm ngoài CRC phần cứng (như checksum của dự án cũ) — gói sai bị bỏ cả gói.
- Kênh theo thang 1000..2000 µs như CRSF đưa vào `FlightApp` (0 = chưa có); 4 kênh đầu là roll / pitch / ga / yaw theo cài đặt gán kênh của tay; kênh AUX chưa có nguồn = 1000 **(quyết định của tôi)**.
- Không có bit "armed" như dự án cũ: arm là việc của mạch bay (cần gạt hoặc công tắc AUX).
- Đổi kênh tần số / tốc độ / công suất trong app của tay phát → tay gửi gói cài đặt trên cài đặt cũ, đợi ACK, rồi cả hai đổi (như `radio_tx_send_config` cũ). Chỉ khi drone chưa arm (drone báo trạng thái arm trong telemetry).

### 4.4 Telemetry drone → tay phát (ACK payload)
- Pin (V), đã arm, chế độ bay, chất lượng sóng phía drone (% gói nhận được), cờ failsafe; người làm có thể thêm. Tay phát giữ bản mới nhất, đưa ra app (tab Tổng quan) qua stream.

### 4.5 Phía mạch bay
- Driver đọc nRF24 trong `slow_update` (luồng sensor): đọc hết FIFO, giải gói, chuẩn bị ACK payload cho gói sau. Không ngắt bắt buộc.
- `linkOk` theo `receiver.rx_timeout_ms` hiện có; mất gói / gói hỏng / tỉ lệ nhận được hiện ở tab Receiver như với CRSF (bộ đếm `frameErrors()` v.v.). Kênh RC vào `rc_` như mọi receiver (16 kênh, kênh không có = 0 hoặc 1500 theo cách CRSF đang làm).
- Cài đặt nRF24 phía drone (địa chỉ, kênh tần số, tốc độ...) nằm trong schema (thêm vào section `receiver` hoặc section mới — người làm chọn, theo luật append-only của `protocol/wbp_schema.json`).

## 5. Firmware tay phát

- Cùng kiểu tổ chức với mạch bay: `src/main.cpp` nhánh `_WINDBURST_TX` (hoặc một `main_tx.cpp` — người làm chọn), phần logic thuần (giải/đóng gói, hiệu chỉnh joystick, máy trạng thái bind) là header portable test được trên PC; phần ESP32-C5 mỏng (ADC, SPI, NVS, USB).
- Đọc joystick: ADC 12 bit, lấy trung bình vài mẫu mỗi chu kỳ; hiệu chỉnh mỗi trục: min / tâm / max (nút "Hiệu chỉnh" trong app: đẩy cần ra các góc), đảo chiều, vùng chết quanh tâm, expo (tuỳ chọn); gán trục vật lý → roll / pitch / ga / yaw (mặc định như dự án cũ: ga = joy2 Y, yaw = joy2 X, roll = joy1 X, pitch = joy1 Y). - **Ga trên joystick tự hồi tâm — 2 chế độ** (người dùng chọn: "1 cái cho alt và pos, 1 cái manual"):
  - **ALT / POS**: đọc thẳng, cần ở giữa = 1500 = giữ độ cao (đúng cách ALT HOLD / POS HOLD của mạch bay hiểu cần ga).
  - **Thủ công**: ga cộng dồn — đẩy cần lên thì ga tăng dần, kéo xuống thì giảm dần, buông thì giữ nguyên; dùng cho ANGLE / ACRO. Bắt đầu ở mức thấp nhất (1000) mỗi khi drone chưa arm (biết qua telemetry; không có telemetry thì cũng về 1000), nên buông tay sau khi arm bằng cần gạt không làm drone vọt lên. Tốc độ tăng/giảm là một cài đặt.
  - **Cách chuyển giữa hai chế độ (cách tôi hiểu câu trả lời — người dùng sửa nếu sai)**: một cài đặt của tay phát, chọn trong tab Joystick (tay chưa có công tắc). Tab Tổng quan cảnh báo khi chế độ ga không khớp chế độ bay drone báo về (ví dụ ga ALT / POS mà drone đang ANGLE). Tự đổi theo chế độ bay drone báo về **không** làm lần này: lúc đổi, ga sẽ nhảy.
- Gửi gói theo nhịp timer (không theo `loop()`), số thứ tự tăng mỗi gói.
- App link: WBP qua USB của C5 (USB Serial/JTAG), cùng `wbp::Server`, `SettingsStore` (NVS), lưu / lưu và khởi động lại / chưa lưu như mạch bay. HELLO trả `ROLE_TX` (đã có trong `wbp_server.h`).
- RAM: struct `Settings` sinh từ schema có mặt ở mọi firmware, kể cả F405 (đang 88,8 % RAM). Section của tay phát giữ nhỏ (cỡ trăm byte); nếu F405 vượt ngưỡng thì loại chúng khỏi bản build mạch bay (người làm chọn cách).
- Cài đặt của tay phát: section riêng trong cùng schema (một schema, một hash). **Mạch bay không hiện / không lưu section của tay phát và ngược lại** — cách đánh dấu section theo vai trò do người làm chọn (ví dụ thuộc tính `role` trong schema, `gen_proto.py` sinh ra, server bỏ qua section không thuộc vai trò của nó).

## 6. App

- HELLO role `TX` → thanh tab chỉ còn **Tổng quan · Joystick · Radio · Hệ thống**; role `RX` → như hiện tại. Đổi cổng giữa tay phát và mạch bay không cần mở lại app.
- **Tổng quan**: hai ô vẽ vị trí 2 joystick (sau hiệu chỉnh) + giá trị 8 kênh; sóng tới drone (đang nối / mất, % gói được ACK, gói/s); drone báo về (pin, arm, chế độ bay, chất lượng sóng phía drone).
- **Joystick**: chế độ ga (ALT / POS hoặc Thủ công, tốc độ cộng dồn), hiệu chỉnh (đẩy cần ra 4 góc, thả về tâm, xong), gán trục → kênh, đảo chiều, vùng chết, expo; giá trị thô ADC từng trục để soi.
- **Radio**: bind (trạng thái: chưa bind / đang bind / đã bind với ...), kênh tần số, tốc độ, công suất, gói/s, chân nRF24 của tay.
- **Hệ thống**: phiên bản firmware, khôi phục mặc định, khởi động lại (như phần tương ứng của tab System mạch bay).
- Mạch bay: tab Receiver có phần nRF24 khi giao thức là `NRF24` (nút Bind, trạng thái bind, kênh tần số, chất lượng sóng); tab Pins có các chân nRF24.
- Tiếng Việt + bản dịch `en.js` như các tab khác.

## 7. Kiểm chứng (theo luật của dự án)

- `test/sim_nrf24.h`: chip nRF24 mô phỏng ở mức thanh ghi SPI (đủ cho driver: CONFIG, EN_AA, SETUP_RETR, RF_CH, RF_SETUP, STATUS, FIFO, W_TX_PAYLOAD, R_RX_PAYLOAD, W_ACK_PAYLOAD, R_RX_PL_WID, FLUSH, CE) + "không khí" nối hai chip theo địa chỉ / kênh / tốc độ, có mất gói và hỏng gói theo xác suất. **Driver tự viết (thanh ghi), không dùng thư viện RF24** **(quyết định của tôi: chạy được trên PC như các driver chip khác của dự án; thư viện RF24 vẫn là nguồn tham khảo cách khởi động chip)**.
- Unit test: đóng/giải gói, CRC, số thứ tự cũ, kênh ngoài khoảng, hiệu chỉnh joystick (tâm, vùng chết, đảo, expo), hai chế độ ga (ALT / POS đọc thẳng; Thủ công cộng dồn, về 1000 khi chưa arm, giữ nguyên khi buông), máy trạng thái bind.
- FIL: tay phát mô phỏng + mạch bay trên `virtual_board`: đẩy cần → kênh RC đúng ở mạch bay; arm bằng cần gạt rồi buông cần ở chế độ ga Thủ công → drone không rời đất; cắt sóng → failsafe sau `rx_timeout_ms`; sóng về → hết failsafe; mất 30 % gói vẫn bay; hai bộ đã bind khác nhau không nghe lẫn; bind lần đầu tự động khi chưa bind; đổi kênh tần số qua sóng.
- `wb_sim`: thêm tay phát mô phỏng trên một cổng TCP khác để app nối được (test app cho 4 tab TX, bind từ app, tab Receiver nRF24).
- Mutant trong `tools/mutate.py`, build `tx_c5` và `rx_esp32_s3` bằng PlatformIO, F405 vẫn build (không đổi hành vi STM32).
- Tài liệu: `docs/TX.md` (mới: phần cứng, cách nối, bind, hiệu chỉnh, giới hạn), `docs/RX.md` (nRF24), `docs/PROTOCOL.md` (role / section tay phát), dòng HANDOFF, `todo.txt`, tăng phiên bản (`src/config/version.h` + chuỗi trong `test/test_fw.cpp`); tay phát có phiên bản riêng hay chung — người làm chọn.
- Không tuyên bố chạy trên phần cứng khi chưa thử trên mạch: `todo.txt` liệt kê các bước thử (bind lần đầu, đi xa đến khi mất sóng → failsafe, nhiễu WiFi gần).

## 8. Để người làm quyết định

- Định dạng byte cụ thể của các gói, cách nén kênh, số kênh (≥ 8), bộ đếm thống kê.
- Section nào chứa cài đặt nRF24 phía drone; cách đánh dấu section theo vai trò.
- Cách chia giai đoạn (gợi ý: driver + sim + gói → receiver mạch bay → firmware tay phát → app).
- Có dùng chân IRQ của nRF24 hay chỉ đọc STATUS.
