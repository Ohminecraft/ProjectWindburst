# OSD cho kính FPV digital — section `osd` (firmware 0.21)

OSD hiện trong kính của các hệ FPV digital: DJI O3 / O4, Walksnail Avatar, HDZero. Firmware gửi qua **MSP DisplayPort**, giống Betaflight (`io/displayport_msp.c`, `osd/osd_elements.c`). Kính tự vẽ bằng font Betaflight có sẵn trong nó, nên firmware dùng đúng mã ký tự của Betaflight (`drivers/osd_symbols.h`). OSD analog (chip MAX7456) chưa có.

## Nối dây và cài đặt
- **Dây:** TX của flight controller vào RX của bộ phát hình, RX vào TX của nó, nối chung GND. Bộ phát hỏi trạng thái arm qua dây RX để bắt đầu quay video; thiếu dây RX thì OSD vẫn hiện nhưng kính không tự quay.
- **Cài đặt:** ở tab **OSD** (khối "Kết nối"):

  | Trường | Mặc định | Ý nghĩa |
  |---|---|---|
  | `type` | `NONE` | `MSP_DISPLAYPORT` = bật |
  | `canvas` | `AUTO` | lưới ký tự. AUTO: HDZero / Walksnail tự báo lưới (`MSP_SET_OSD_CANVAS`); không báo thì 53 x 20 (DJI). Hoặc chọn cố định: 53 x 20, 50 x 18, 30 x 16 |
  | `baud` | 115200 | tốc độ UART |
  | `tx_pin`, `rx_pin` | trống | chân UART |
  | `rate` | 10 | số lần vẽ mỗi giây (1 – 30) |
  | `name` | trống | tên máy bay (mục "Tên máy bay", và trả lời `MSP_NAME`) |
  | `pos[24]` | xem dưới | vị trí từng mục, cách mã hoá của Betaflight: cột = bit 0-5, hàng = bit 6-10, hiện = bit 11 |
- **Chọn UART:**
  - AUTO: UART còn trống sau GPS, receiver, optical flow, dây telemetry ESC. OSD được chọn **trước** telemetry MAVLink: nếu chỉ còn một UART thì OSD lấy, MAVLink báo không còn UART.
  - TABLE: UART có chức năng `DISPLAYPORT` (tab Chân và cổng).
- **Trong kính:** chọn OSD kiểu "Custom" / "Betaflight" (DJI: canvas mode / custom OSD).
- Đổi cài đặt cần lưu và khởi động lại.

## Các mục (thứ tự trong `pos[]`, thêm vào cuối)
RSSI / LQ, điện áp pin (ký hiệu pin 7 mức theo khoảng cell; nhấp nháy khi pin ở mức critical), điện áp trung bình mỗi cell, dòng, mAh, mode bay (ACRO / ANGL / STAB / ALTH / POSH / RESC, `!FS!` khi failsafe), tên máy bay (chữ in hoa), thời gian bay (chỉ đếm khi arm), thời gian bật nguồn, số vệ tinh (nhấp nháy khi chưa có fix 3D), tốc độ GPS (km/h), hướng về nhà (mũi tên so với mũi máy bay, 16 hướng), khoảng cách về nhà, độ cao (bộ ước lượng), tốc độ lên / xuống, ga, tâm ngắm, đường chân trời giả (giới hạn pitch 20°, roll 40° như mặc định của Betaflight), cảnh báo, hướng mũi, góc pitch, góc roll, nhiệt độ ESC (cao nhất), chữ DISARMED.

**Cảnh báo:** chỉ hiện một, theo thứ tự ưu tiên:
1. `FAILSAFE`
2. `LAND NOW` (pin critical)
3. `LOW BATTERY`
4. `RXLOSS`
5. khi chưa arm: lý do không arm được đầu tiên, ví dụ `GYRO NOT CAL`.

Mặc định đang hiện: LQ, pin, dòng, mAh, mode, tên, thời gian bay, vệ tinh, tốc độ, về nhà, độ cao, ga, tâm ngắm, cảnh báo, DISARMED. Bố cục mặc định cho lưới 53 x 20.

**App:** ở tab OSD, kéo một mục trên màn hình xem trước để dời nó (hoặc bấm vào mục rồi dùng phím mũi tên), công tắc trong bảng "Các mục" để ẩn / hiện. Ký hiệu trong màn hình xem trước chỉ là hình minh hoạ cùng độ rộng.

## Trên dây
- **Mỗi màn hình:** heartbeat (mỗi giây một lần), clear, mỗi mục một lệnh "write string" (hàng, cột, thuộc tính, tối đa 30 ký tự), rồi draw. Tất cả là khung MSP v1 kiểu trả lời (`$M>`), lệnh 182. Thuộc tính `0x40` = nhấp nháy, kính tự làm.
- **Dung lượng:** một màn hình khoảng 0,5 kB, tức 5 kB/s ở 10 màn hình mỗi giây (115200 baud chở được 11 kB/s).
- **Không bao giờ chặn:** firmware không giao cho UART nhiều hơn chỗ trống của nó, và chỉ dựng màn hình mới khi màn hình trước đã gửi hết.
- **Bộ phát hình hỏi lại** trên cùng dây:
  - `MSP_STATUS` / `MSP_STATUS_EX`: trả lời như Betaflight 4.5 (`msp::Bridge`). Box ARM bật khi đã arm, nên kính bắt đầu quay khi arm.
  - `MSP_SET_OSD_CANVAS`: lưới của bộ phát, chỉ dùng khi `canvas = AUTO`, và chỉ nhận lưới hợp lý (20 – 63 cột, 10 – 31 hàng).
  - `MSP_NAME`: trả lời tên máy bay.
  - `MSP_FC_VARIANT` ("BTFL"), `MSP_API_VERSION` ...: trả lời như Betaflight 4.5.
  - ESC passthrough luôn bị từ chối trên cổng này.

Báo cáo khởi động: `osd: MSP DisplayPort on UART0 TX GPIO43 RX GPIO44, 115200 baud, 53 x 20 (or what the video transmitter asks for), 10 screens/s`, hoặc `osd: FAILED - <lý do>`. Chân TX / RX được kiểm tra trùng với các chân khác.

Mã nguồn: `src/osd/osd.h` (portable), `FlightApp::osd_in()` / `osd_send()` trong `src/app/flight_app.h`.

## Kiểm chứng
- `test_osd` (57 kiểm tra): từng mục so với cách Betaflight vẽ (ký hiệu pin, mũi tên, số lẻ, nhấp nháy), đường chân trời giả theo công thức số nguyên của Betaflight, vẽ trong lưới, byte trên dây, UART chậm (16 byte mỗi lần, khung không bao giờ bị cắt), các yêu cầu của bộ phát hình.
- `test_fw` nhóm `osd` (`WB_ONLY=osd`): mạch ảo, 2 s ra 20 màn hình, bộ phát hỏi trạng thái arm và đổi lưới, khi arm thì mode, timer và ARM box thay đổi, chia UART cùng MAVLink, bảng cổng, chân trùng.
- 28 mutant `osd:`; `test_gui_browser.py::test_osd_tab` (viết nhưng chưa chạy được ở máy này).
- **Chưa thử với kính thật.** Cần thử: DJI O3 (canvas mode), Walksnail, HDZero. Kính có hiện OSD không, các mục có đúng chỗ không, có quay video khi arm không.

## 0.33: thêm mục, bảng thống kê, màn hình xem trước mới

**Mục mới** (nối vào cuối `pos[]`, nên `pos` giờ có 32 ô; cài đặt từ bản cũ vẫn giữ, 8 mục mới mặc định ẩn). Cách vẽ theo `osd_elements.c` của Betaflight:

| Mục | Ví dụ | Ghi chú |
|---|---|---|
| POWER | `198W` | dòng × áp; cần đo pin và đo dòng |
| FLIGHT_DIST | `⟷1235m` | quãng đường bay từ lúc arm, theo tốc độ GPS khi có fix 3D |
| GPS_LAT / GPS_LON | `10.7654321` | ký hiệu, độ, 7 số lẻ, có dấu âm (`osdFormatCoordinate`) |
| COMPASS_BAR | `W·┴·N·┴·E` | 9 ký tự, hướng mũi ở giữa, mỗi bước 1/16 vòng (`osdElementCompassBar`) |
| SIDEBARS | | hai thanh dọc 7 hàng, cách tâm 7 cột, dấu ngang ở cột ±6 (`osdElementHorizonSidebars`) |
| G_FORCE | `1.0G` | độ lớn gia tốc đo được |
| BATT_USAGE | `[■■■■■■■◧□□]` | 10 ô, mỗi ô 10 % pin còn lại; cần `battery.capacity_mah`; nhấp nháy khi pin yếu |

**Bảng thống kê sau chuyến bay** (`osd.stats`, mặc định bật; giống `osdShowStats` của Betaflight):
- Disarm xong thì kính hiện bảng thống kê thay cho các mục, trong 20 giây hoặc tới lần arm sau. Lần arm mới thì số liệu tính lại từ đầu.
- Bảng có tiêu đề `--- STATS ---` và các dòng:
  - FLY TIME;
  - MAX SPEED, MAX DISTANCE, FLIGHT DISTANCE (khi có GPS);
  - MAX ALTITUDE;
  - MIN BATTERY;
  - MAX CURRENT, USED MAH (khi có đo dòng).

**App, tab OSD:**
- **Xem trước như trong kính:** nền là ảnh camera (trời và đất) nghiêng theo máy. Có lưới ký tự. Mỗi ký tự nằm đúng một ô.
- **Ký hiệu:** vẽ lại thành hình vector cùng nghĩa với font Betaflight, trắng viền đen như kính hiện (`js/gfx/osdglyph.js`). Không chép font bitmap của Betaflight.
- **Số hiện là số máy đang báo:** pin, dòng, GPS, độ cao, tư thế, ga, mode. Khi chưa kết nối thì hiện số ví dụ.
- **Đường chân trời giả và thanh bên** vẽ theo đúng công thức của firmware.
- **Bố cục:** khối "Màn hình" lên đầu cột trái; danh sách mục ở cột phải.

**Test:**
- `test_osd`: thêm nhóm `[elements 0.33]` và `[statistics]`, tổng 84 kiểm tra.
- `test_fw` nhóm `osd`: bay, hạ cánh, disarm thì kính hiện bảng thống kê.
- `test_proto`: bản ghi `osd` 73 byte của bản cũ đọc được.
- 15 mutant `osd33:`.
