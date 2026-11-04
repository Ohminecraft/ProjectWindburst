# MAVLink qua ExpressLRS (ELRS)

Cách đưa telemetry MAVLink của Windburst lên Mission Planner / QGroundControl bằng phần cứng ELRS. Viết theo tài liệu ELRS (`expresslrs.org/software/mavlink`), mã nguồn ELRS (receiver: `src/src/rx-serial/SerialMavlink.cpp`; TX: `src/lib/MAVLink/MAVLink.cpp`), Betaflight (`rx/mavlink.c`, `telemetry/mavlink.c`) và ArduPilot (`GCS_MAVLINK::handle_radio_status`), đối chiếu với firmware Windburst 0.40.0. **Chưa thử trên mạch thật.**

## Tóm tắt: dùng được cách nào

| Cách | Lệnh điều khiển đi bằng | Telemetry tới | Với Windburst |
|---|---|---|---|
| 1. ELRS bình thường (CRSF) | ELRS, CRSF | màn hình tay phát (EdgeTX "Discover new") | **Dùng được** (từ 0.8.0, `docs/RX.md`) |
| 2. Một cặp ELRS thứ hai làm radio telemetry | receiver chính (ELRS CRSF hay loại khác) | Mission Planner / QGroundControl | **Dùng được, chỉ xem** (chưa thử trên mạch) |
| 3. ELRS chế độ MAVLink: một link cho cả điều khiển lẫn telemetry | ELRS, MAVLink | tay phát + ground station | **Dùng được từ 0.39.0** (chưa thử trên mạch) |

Cách 3 là cách nên dùng nếu cần ground station: một receiver, một UART, không cần radio thứ hai.

## Cảnh báo: firmware và cài đặt phải khớp với Link Mode

Ở chế độ MAVLink, receiver ELRS **không gửi CRSF nữa**. Nó gửi lệnh điều khiển bằng message `RC_CHANNELS_OVERRIDE`. Vì vậy:
- Firmware **cũ hơn 0.39.0** không đọc được message này: đổi Link Mode sang MAVLink là **mất điều khiển** (không arm được, đang bay thì vào failsafe).
- Từ 0.39.0: đổi Link Mode sang MAVLink thì **phải** đổi `receiver.protocol` thành `MAVLINK` cùng lúc, và ngược lại. Giao thức không khớp thì receiver vẫn gửi nhưng mạch bay không nhận ra khung nào. Trước khi bay, xem tab Receiver: "Tín hiệu: có", khoảng 100 khung/s.

## Cách 1: ELRS bình thường, telemetry về tay phát

Receiver ELRS để giao thức CRSF (mặc định), nối hai dây vào một UART. Ở app, tab Receiver: giao thức CRSF, có chân TX, `receiver.telemetry` bật. Trên tay phát EdgeTX: Telemetry → "Discover new" sẽ thấy pin, dòng, mAh, tư thế, GPS, độ cao, vario, chế độ bay. Chi tiết: `docs/RX.md`, mục "Telemetry CRSF".

Cách này không có ground station trên máy tính.

## Cách 3: một link ELRS cho cả điều khiển lẫn telemetry (từ 0.39.0)

### Cần có
- **TX module và receiver ELRS chạy ESP** (loại có WiFi). Receiver dùng chip STM32 không chạy được chế độ MAVLink.
- **Firmware ELRS 3.5.0 trở lên** cho cả hai. Nếu dùng WiFi của backpack thì TX Backpack phải từ 1.5.0 trở lên.
- **Windburst 0.39.0 trở lên** (nạp firmware và build lại app, vì schema đã đổi).
- Nên dùng hàng 2,4 GHz hoặc LR1121; hàng 900 MHz đời cũ (SX127x) chở được rất ít telemetry.

### Bước 1: cài ELRS
1. Bật tay phát và receiver, đợi ELRS Lua script hiện chữ **C** ở góc phải trên (đã nối với nhau).
2. Đặt giao thức serial của receiver thành **MAVLink**.
   - ELRS 3.x: Lua script → Other Devices → chọn receiver → giao thức serial = MAVLink.
   - ELRS 4.x: bỏ qua bước này, receiver tự đổi khi TX đổi Link Mode.
3. Tắt receiver (Link Mode không đổi được khi đang nối), rồi đặt **Link Mode = MAVLink** ở menu chính của Lua script. Tỉ lệ telemetry tự thành 1:2 và không đổi được.
4. Chọn packet rate theo bảng băng thông ở dưới.

### Bước 2: nối dây
Giống hệt receiver CRSF: hai dây vào UART của receiver (chân `rx_rx`, `rx_tx` ở tab Chân và cổng, nhóm "Receiver nối tiếp").

| Mạch bay (Windburst) | Receiver ELRS |
|---|---|
| `pins.rx_rx` (RX của UART) | TX |
| `pins.rx_tx` (TX của UART) | RX |
| 5 V, GND | 5 V, GND |

Lần này **cần cả hai dây**: dây vào mang cần điều khiển và `RADIO_STATUS`, dây ra mang telemetry.

### Bước 3: cài Windburst
- Tab Receiver: **Giao thức = MAVLink (ELRS ở chế độ MAVLink)**, **Telemetry về tay điều khiển = bật**. Thứ tự kênh vẫn như ELRS CRSF (AETR).
- Tab Chân và cổng, khối "Telemetry (MAVLink)": chỉ còn dùng **System ID** (để 1, là Target SysID mặc định của ELRS) và **tốc độ gửi** từng nhóm. Giao thức, baud, chân TX / RX của khối này không dùng; giao thức để NONE cũng được.
- Baud của UART receiver tự là **460800** (tốc độ ELRS dùng ở chế độ MAVLink), không cần đặt.

Lưu rồi khởi động lại. Báo cáo khởi động phải có hai dòng:
```
rx: MAVLINK on UART2, MAVLink telemetry on its TX pin
telemetry: MAVLink 2, system id 1, on the receiver's UART (UART2 TX GPIOnn, 460800 baud)
```
Nếu thấy `telemetry: FAILED - the receiver's UART has no TX pin (pins.rx_tx)`: chưa đặt chân TX của receiver. Cần điều khiển vẫn chạy, chỉ không có telemetry.

### Bước 4: tay phát
Cần, công tắc, mode đặt như bình thường trên EdgeTX. TX module ELRS tự đổi MAVLink thành telemetry CRSF cho tay phát: Telemetry → "Discover new" thấy pin, GPS, tư thế, độ cao / vario, chế độ bay. Tên chế độ theo mã của ArduCopter, có thêm `*` khi chưa arm (ví dụ `STAB*`). ELRS chỉ đọc khung có component id 1, đúng là cái Windburst gửi.

### Bước 5: ground station
Như bước 4 của cách 2 bên dưới: cắm USB vào TX module (460800 baud), hoặc qua WiFi của backpack (UDP 14550).

### Firmware tự điều tiết tốc độ gửi
Receiver ELRS có bộ đệm 1 kB cho telemetry. Bộ đệm đầy thì byte mới bị mất, và khung MAVLink chứa chúng hỏng theo. Mỗi 10 ms receiver gửi `RADIO_STATUS`, trong đó `txbuf` = % bộ đệm còn trống. Windburst dùng nó theo hai cách (`telemetry/mavlink_telemetry.h`):
- **Như Betaflight** (`mavlink_min_txbuff` = 35): khi `txbuf` ≤ 35 % thì không gửi gì, đợi receiver gửi bớt.
- **Như ArduPilot** (`stream_slowdown_ms`): `txbuf` < 20 thì chu kỳ mỗi nhóm dài thêm 60 ms, < 50 thì thêm 20 ms (tối đa khoảng 2 s); > 95 thì bớt 40 ms, > 90 thì bớt 20 ms. Riêng HEARTBEAT giữ 1 Hz và STATUSTEXT không bị làm chậm, như ArduPilot. Thiếu HEARTBEAT quá lâu thì ground station báo mất kết nối.

Trên máy tính đã thử với bộ đệm giả lập: packet rate 250Hz (khoảng 360 B/s đi xuống) với tốc độ mặc định (khoảng 1400 B/s) thì 10 giây không mất byte nào, bộ đệm cao nhất khoảng 870 / 1024 byte. HEARTBEAT vẫn đủ 1 Hz, các nhóm khác chậm lại. Muốn ATTITUDE mượt hơn ở packet rate thấp, đặt tốc độ gửi theo bảng ở dưới để không phải chờ điều tiết.

### Giới hạn
- Từ 0.40 ground station **nạp / tải mission, đổi chế độ, RTL, bắt đầu mission, disarm** qua chính link này; **arm** khi bật `telemetry.remote_arm` (`docs/MAVLINK.md`, mục "Nhận từ ground station"). Tham số thì chưa: Mission Planner sẽ thử tải tham số rồi hết thời gian chờ, bỏ qua thông báo đó.
- Không kiểm `target_system` của `RC_CHANNELS_OVERRIDE` (như Betaflight): trên dây này chỉ có một người gửi là receiver.
- Mất sóng: receiver ngừng gửi `RC_CHANNELS_OVERRIDE`, sau `receiver.rx_timeout_ms` thì mạch bay vào failsafe như mọi giao thức khác.

## Cách 2: cặp ELRS thứ hai làm radio telemetry

Giống một bộ radio telemetry 433 / 915 MHz (SiK), nhưng dùng phần cứng ELRS. Cặp này **chỉ chở telemetry**. Lệnh điều khiển vẫn đi qua receiver chính như bình thường. Dùng khi receiver chính không phải ELRS, hoặc muốn giữ link điều khiển ở CRSF.

### Cần có
- **TX module ELRS chạy ESP32** (có WiFi). Nó cắm USB vào máy tính, không cần gắn vào tay phát. Module trong tay phát (ví dụ TX16S có ELRS bên trong) không cắm USB được: phải dùng WiFi của backpack.
- **Receiver ELRS chạy ESP**, **firmware ELRS 3.5.0 trở lên**, như cách 3.
- **Một UART còn trống** trên mạch bay.
- **Binding phrase riêng** cho cặp này, khác cặp điều khiển.

### Bước 1: cài ELRS
1. Gắn TX module vào tay phát, chọn **Model ID 0** trong EdgeTX. Khi chạy không có tay phát, ELRS dùng cài đặt của Model ID 0.
2. Làm như bước 1 của cách 3 (giao thức serial MAVLink, Link Mode MAVLink, packet rate).

### Bước 2: nối dây
| Mạch bay (Windburst) | Receiver ELRS thứ hai |
|---|---|
| `telemetry.tx_pin` (TX của UART) | RX |
| `telemetry.rx_pin` (RX của UART) | TX |
| 5 V, GND | 5 V, GND |

Dây RX mang lệnh và mission của ground station (0.40); không nối thì chỉ xem được. Trên STM32 phải đặt `rx_pin`, vì firmware cần chân này để khởi động UART. `RC_CHANNELS_OVERRIDE` và `RADIO_STATUS` của receiver này bị bỏ qua: cặp này không điều khiển được máy bay, và **không có điều tiết tốc độ** - phải tự chọn tốc độ gửi theo bảng băng thông.

### Bước 3: cài Windburst (tab Chân và cổng, khối "Telemetry (MAVLink)")
- Giao thức: **MAVLink**. Baud: **460800**. Chân TX / RX: theo bước 2.
- System ID: **1**. Tốc độ gửi: theo bảng băng thông.
- Nếu receiver chính cũng là ELRS ở chế độ MAVLink (cách 3) với `receiver.telemetry` bật, telemetry đi theo receiver chính và khối này không dùng chân TX nữa (báo cáo khởi động ghi `telemetry.tx_pin is not used`).

Lưu rồi khởi động lại. Báo cáo khởi động phải có dòng `telemetry: MAVLink 2, system id 1, UARTx TX GPIOnn, 460800 baud`. Cách chọn UART và chân: `docs/MAVLINK.md`.

### Bước 4: nối ground station (dùng chung cho cách 2 và 3)
- **Qua USB (nên thử trước):** cắm TX module vào máy tính, bật máy bay.
  - Mission Planner: chọn cổng COM của TX module, baud **460800**, bấm Connect.
  - QGroundControl: tắt hết auto-connect, tạo Comm Link kiểu Serial, baud 460800. **Trên Windows phải tick "Force DTR Low"** (Advanced Settings); không tick thì TX module khởi động lại hoặc treo. Tuỳ chọn này mới có, nên ELRS khuyên dùng bản QGC daily build.
  - Module trong tay phát không cắm USB được: dùng WiFi.
- **Qua WiFi (cần TX Backpack):** trong Lua script, Backpack → **Telemetry = WiFi**. **Không** bật WiFi bằng mục "Enable Backpack WiFi": mục đó đưa backpack vào chế độ nạp firmware, không phải MAVLink.
  - Máy tính vào mạng `ExpressLRS TX Backpack XXXXXX`, mật khẩu `expresslrs` (backpack là 10.0.0.1).
  - Mission Planner: chọn UDP, cổng **14550**. QGroundControl: bật auto-connect UDP.
- Mission Planner: bật `telemetry.mp_home_item` để Read WPs không mất lệnh đầu (`docs/MAVLINK.md`, mục "Dùng với Mission Planner").

### Bước 5: thấy gì trên ground station
- **Hiện được:** HUD (tư thế, độ cao, tốc độ), vị trí GPS trên bản đồ, pin, chế độ bay (theo mã chế độ của ArduCopter), các dòng STATUSTEXT (báo cáo khởi động, "Arming blocked: ...", "Armed").
- **Cách 3 và cách 2 (khi nối dây RX của UART telemetry):** nạp / tải mission, đổi chế độ, RTL, bắt đầu mission, disarm, arm nếu bật `remote_arm` (`docs/MAVLINK.md`).
- **Chưa có:** tham số.

## Băng thông: chọn tốc độ gửi

Cách 3 tự điều tiết, nhưng tốc độ đặt vừa với sức chở thì dữ liệu đều hơn (không bị dồn rồi chờ). Cách 2 **không** điều tiết, nên tổng lượng gửi phải thấp hơn rõ sức chở. Nếu không, bộ đệm của receiver đầy và các khung bị bỏ (trang MAVLink trên WebUI của backpack có số "Buffer overflows").

**Windburst gửi bao nhiêu** (MAVLink 2: 12 byte khung + nội dung):

| Message | Byte / khung | Tốc độ (trường) | Mặc định | B/s ở mặc định |
|---|---|---|---|---|
| HEARTBEAT | 21 | luôn 1 Hz | 1 | 21 |
| ATTITUDE | 40 | `rate_attitude` | 10 | 400 |
| GLOBAL_POSITION_INT + GPS_RAW_INT | 40 + 64 | `rate_position` | 5 | 520 |
| VFR_HUD | 32 | `rate_hud` | 4 | 128 |
| SYS_STATUS + BATTERY_STATUS | 55 + 66 | `rate_status` | 2 | 242 |
| RC_CHANNELS | 54 | `rate_rc` | 2 | 108 |
| **Tổng** | | | | **≈ 1420** |

Byte / khung là mức tối đa (GPS_RAW_INT, SYS_STATUS, BATTERY_STATUS gửi cả phần mở rộng của MAVLink 2). MAVLink 2 cắt các byte 0 ở cuối, nên thực tế thường gửi ít hơn. GPS_RAW_INT chỉ gửi khi có GPS, BATTERY_STATUS chỉ khi có đo pin. Ở cách 3, RC_CHANNELS chỉ lặp lại cần mà tay phát đã biết: có thể để `rate_rc` = 0.

**ELRS chở được bao nhiêu** chiều máy bay → đất, theo bảng của ELRS ở tỉ lệ 1:2. Cột "gợi ý" là bộ tốc độ giữ dưới khoảng 70 % sức chở:

| Packet rate (2,4 GHz) | Sức chở | Gợi ý `attitude / position / hud / status / rc` | Windburst gửi |
|---|---|---|---|
| F1000 (SX128x), K1000 (LR1121) | ~2375 B/s | mặc định 10 / 5 / 4 / 2 / 2 | ≤ 1420 B/s |
| 333Hz Full | ~1470 B/s | 5 / 2 / 2 / 1 / 1 | ≤ 670 B/s |
| 500Hz | ~1185 B/s | 5 / 2 / 2 / 1 / 1 | ≤ 670 B/s |
| 250Hz | ~590 B/s | 2 / 1 / 1 / 1 / 0 | ≤ 360 B/s |
| 100Hz Full | ~435 B/s | 1 / 1 / 0 / 1 / 0 | ≤ 290 B/s |
| 150Hz | ~350 B/s | không nên: bộ nhỏ nhất (1 / 1 / 0 / 1 / 0) đã tới 82 % | – |
| 50Hz | ~110 B/s | không đủ | – |

Packet rate cao chở nhiều nhưng tầm ngắn hơn. Ở cách 3, packet rate cũng là tốc độ gửi cần điều khiển: chọn theo việc bay trước (tầm, độ trễ), rồi mới chỉnh tốc độ telemetry cho vừa. Ở cách 2, cặp telemetry mất sóng thì chỉ mất hình trên ground station, nên 333Hz Full là lựa chọn cân bằng. Hàng 900 MHz: xem bảng trên trang ELRS; ở 200Hz (~470 B/s) dùng bộ 1 / 1 / 1 / 1 / 0 (≤ 320 B/s).

## Khi chạy thật nên kiểm tra
- Cách 3:
  - Báo cáo khởi động có hai dòng `rx: MAVLINK ...` và `telemetry: MAVLink 2 ... on the receiver's UART`.
  - Tab Receiver: "Tín hiệu: có", khoảng 100 khung/s, các kênh theo cần.
  - "Chất lượng" ở tab Receiver bằng LQ trên Lua script (lấy từ `RADIO_STATUS`).
  - Tắt tay phát: tín hiệu mất, failsafe đúng như cài đặt.
- Ground station: HUD chạy theo khi nghiêng máy bay, GPS và pin đúng, HEARTBEAT không đứt (không có thông báo mất kết nối).
- Tay phát (cách 3): "Discover new" thấy pin, GPS, chế độ bay.
- Nếu dùng backpack: trang MAVLink trên WebUI (http://10.0.0.1) có "Downlink packets" tăng dần và "Buffer overflows" đứng yên ở 0. Ở cách 2, overflow tăng thì giảm tốc độ gửi; ở cách 3, overflow tăng là lỗi điều tiết: gửi lại cho tôi số liệu và packet rate.
- Cách 2, hai link ELRS cùng dải 2,4 GHz trên một máy bay: đặt hai receiver và anten cách xa nhau, xem LQ của link điều khiển có giảm khi bật cặp telemetry không.

## Còn thiếu
- Trả lời yêu cầu tham số (`PARAM_REQUEST_LIST`) để ground station không phải chờ hết giờ.
