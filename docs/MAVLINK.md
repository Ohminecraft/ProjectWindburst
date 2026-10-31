# Telemetry MAVLink tới ground station — section `telemetry` (firmware 0.19)

Gửi trạng thái máy bay cho Mission Planner, QGroundControl, hoặc một radio có MAVLink (radio telemetry 433 / 915 MHz, ELRS backpack). Từ 0.40 firmware cũng **nhận** lệnh và mission từ ground station (mục "Nhận từ ground station" ở dưới); tham số thì chưa.

Dùng phần cứng ExpressLRS (ELRS) làm radio telemetry, hoặc một link ELRS chế độ MAVLink cho cả điều khiển lẫn telemetry (0.39): `docs/ELRS_MAVLINK.md`.

## Cài đặt (tab Chân và cổng, khối "Telemetry (MAVLink)")
| Trường | Mặc định | Ý nghĩa |
|---|---|---|
| `protocol` | `NONE` | `MAVLINK` = bật (MAVLink 2) |
| `baud` | 57600 | tốc độ UART; 57600 cho đa số radio 433 / 915 MHz, 115200 trở lên cho ELRS backpack / dây thẳng |
| `tx_pin` | trống | chân TX của UART, nối vào RX của radio. Trống = không gửi gì |
| `rx_pin` | trống | chân RX, nối vào TX của radio: lệnh và mission từ ground station (0.40). STM32 cần nó để khởi động UART |
| `sysid` | 1 | system id MAVLink; hai máy bay trên một ground station cần id khác nhau |
| `rate_attitude` / `rate_position` / `rate_hud` / `rate_status` / `rate_rc` | 10 / 5 / 4 / 2 / 2 | số lần mỗi giây, 0 = không gửi |
| `remote_arm` | tắt | cho ground station arm qua MAVLink (0.40; disarm thì luôn được) |
| `mp_home_item` | tắt | Mission Planner: mission tải về có điểm home ở dòng 0 như ArduPilot (0.40.1, mục "Dùng với Mission Planner") |

Đổi cần lưu và khởi động lại.

**Chọn UART** (`docs/PORTS.md`):
- `ports.assign = AUTO`: firmware lấy UART còn trống sau GPS, receiver nối tiếp, optical flow MSP và dây telemetry ESC, theo thứ tự UART1, UART2, UART0. UART0 chỉ dùng được khi app nối qua USB của chip.
- `TABLE`: firmware dùng UART có chức năng `TELEMETRY`.

**Trên UART của receiver** (0.39): `receiver.protocol = MAVLINK` (ELRS chế độ MAVLink) và `receiver.telemetry` bật → telemetry đi ra chân TX của UART receiver, 460800 baud, không lấy UART nào khác. Section `telemetry` chỉ còn cho `sysid` và các tốc độ; `protocol`, `baud`, `tx_pin`, `rx_pin` không dùng (nếu đặt `tx_pin`, báo cáo ghi `telemetry.tx_pin is not used`). Receiver không có chân TX → `telemetry: FAILED - the receiver's UART has no TX pin (pins.rx_tx)`.

Báo cáo khởi động có dòng `telemetry: MAVLink 2, system id 1, UART0 TX GPIO43, 57600 baud` (trên UART receiver: `... on the receiver's UART (UART2 TX GPIO15, 460800 baud)`). Không chạy được thì có dòng `telemetry: FAILED - ...` kèm lý do. Chân TX được kiểm tra trùng với các chân khác: app báo ngay, firmware chặn arm.

## Gửi gì
| Message | Tần số | Nội dung |
|---|---|---|
| HEARTBEAT | 1 Hz, luôn gửi | loại khung (quad / hexa / octo), armed, mode theo số của ArduCopter (Acro, Stabilize = Angle, Alt Hold, PosHold, RTL = GPS Rescue, Land = failsafe), trạng thái (standby / active / calibrating / critical) |
| ATTITUDE | `rate_attitude` | roll, pitch, yaw (rad, quy ước MAVLink: mũi lên = pitch dương), tốc độ quay |
| GLOBAL_POSITION_INT | `rate_position` | vị trí GPS, độ cao MSL, độ cao so với điểm cất cánh (bộ ước lượng), vận tốc NED, hướng |
| GPS_RAW_INT | `rate_position`, khi có GPS | loại fix, số vệ tinh, HDOP, tốc độ, hướng đi, độ chính xác |
| VFR_HUD | `rate_hud` | tốc độ mặt đất, hướng, % ga, độ cao, tốc độ leo |
| SYS_STATUS | `rate_status` | cảm biến có / khoẻ, % CPU, điện áp, dòng, % pin, số khung receiver hỏng |
| BATTERY_STATUS | `rate_status`, khi đo pin | điện áp từng cell (chia đều), dòng, mAh, % |
| RC_CHANNELS | `rate_rc` | 16 kênh, RSSI |
| STATUSTEXT | khi có chữ | từng dòng của báo cáo khởi động (FAILED = lỗi, WARNING = cảnh báo); "Arming blocked: ..." khi đổi và nhắc lại mỗi 10 s; "Ready to arm"; "Armed" / "Disarmed" |

## Nhận từ ground station (0.40) — `src/telemetry/mavlink_gcs.h`

Đọc từ `telemetry/mavlink.c` và `telemetry/mavlink_mission.c` của Betaflight, viết lại. Firmware đọc khung từ ground station qua:
- receiver ELRS chế độ MAVLink: receiver chuyển nguyên khung vào UART của nó (`docs/ELRS_MAVLINK.md`, cách 3);
- radio telemetry riêng: dây RX của UART telemetry (`telemetry.rx_pin`), tới 64 byte mỗi lượt.

Khung gửi cho máy khác (`target_system` khác 0 và khác `sysid`) hoặc component khác (khác 0 và 1) bị bỏ qua. Câu trả lời đi trước mọi khung định kỳ, theo đúng thứ tự (hàng đợi 12, `Scheduler::reply`). Chúng vẫn chờ khi receiver ELRS báo đầy (`txbuf` ≤ 35 %).

**Lệnh (`COMMAND_LONG` / `COMMAND_INT`, trả `COMMAND_ACK`):**

| Lệnh | Làm gì | Từ chối khi |
|---|---|---|
| `COMPONENT_ARM_DISARM` (400) | param1 1 = arm, 0 = disarm | arm: `telemetry.remote_arm` tắt (DENIED, mặc định); có điều kiện chặn arm, kể cả ga cao hay công tắc ARM đang bật (TEMPORARILY_REJECTED). Disarm khi đang bay: chỉ khi param2 = 21196 (mã force của ArduPilot; Betaflight thì luôn disarm) |
| `DO_SET_MODE` (176) | param1 có cờ `CUSTOM_MODE_ENABLED`, param2 = số chế độ ArduCopter: 1 Acro, 0 Angle, 2 Alt Hold, 16 Pos Hold, 5 Loiter (0.41), 3 Mission, 6 RTL | AUTO và RTL khi chưa arm; AUTO khi chưa có mission hoặc home. 21 (Smart RTL), 9 (Land) không chọn được: chúng do cài đặt rescue / failsafe quyết |
| `NAV_RETURN_TO_LAUNCH` (20) | như `DO_SET_MODE` RTL | chưa arm |
| `MISSION_START` (300) | như lệnh `MISSION 1` của app | chưa arm, chưa có mission hoặc home |
| `DO_SET_HOME` (179) | param1 = 1: home = chỗ đang đứng (như `SET_HOME` của app) | đang arm (vị trí ước lượng sẽ bắt đầu lại); toạ độ cho trước: UNSUPPORTED (home đặt lại mỗi lần arm) |
| `PREFLIGHT_REBOOT_SHUTDOWN` (246) | param1 = 1: khởi động lại sau 300 ms (ACK đi trước) | đang arm; param1 = 3 (bootloader): UNSUPPORTED |
| `REQUEST_MESSAGE` (512) | `AUTOPILOT_VERSION`, `HOME_POSITION`, `GPS_GLOBAL_ORIGIN`, `AVAILABLE_MODES` (param2 = 0: tất cả, 1..8: một), `AVAILABLE_MODES_MONITOR` | message khác: UNSUPPORTED; home chưa có: FAILED |
| `REQUEST_AUTOPILOT_CAPABILITIES` (520) | gửi `AUTOPILOT_VERSION`: MAVLink 2, `MISSION_INT`, `COMMAND_INT`, phiên bản firmware | |

Lệnh khác: UNSUPPORTED. `TIMESYNC` (tc1 = 0) được trả lời bằng giờ của mạch (ns).

**Chế độ do ground station chọn** (`FlightCore::set_mode_override`): các bit chế độ bay (Angle, Stabilize, Alt Hold, Pos Hold, Rescue, Mission) **thay** cho công tắc của phi công, cho tới khi phi công gạt một công tắc chế độ bay (công tắc ARM, còi, blackbox... không tính) hoặc ground station chọn chế độ khác. Betaflight khác: nó OR chế độ của ground station lên công tắc và giữ mãi. HEARTBEAT, OSD, tab Receiver đều thấy chế độ đang bay thật.

**Arm từ ground station** (`Arming::remote_arm`): mọi điều kiện chặn arm vẫn áp dụng. Công tắc ARM đang tắt không disarm máy. Phi công lấy lại quyền bằng lần gạt ARM lên kế tiếp (trên link còn sống); sau đó gạt xuống là disarm như thường. Mất link rồi có lại không được tính là một lần gạt.

**Danh sách chế độ (`AVAILABLE_MODES`):** Acro (1), Angle (0), Alt Hold (2), Pos Hold (16), Mission (3), RTL (6), Smart RTL (21, không chọn được), Land (9, không chọn được), Loiter (5, 0.41, chỉ số 9). Mỗi chế độ kèm "standard mode" của MAVLink để QGroundControl hiểu. Số chế độ giữ theo ArduCopter vì TX module ELRS đặt tên chế độ cho tay phát theo số này.

**Mission** (như `mavlink_mission.c`):
- **Nạp lên:** `MISSION_COUNT` → firmware xin từng item bằng `MISSION_REQUEST_INT` → `MISSION_ITEM_INT` → `MISSION_ACK`.
  - Mission cũ giữ nguyên tới khi item cuối tới.
  - Không có item thì xin lại sau 1,5 s; 5 lần không có thì `OPERATION_CANCELLED`.
  - Sai thứ tự: `INVALID_SEQUENCE`. Mission đang bay: `DENIED`. Quá 32 lệnh: `NO_SPACE`.
  - Chỉ nhận từ ground station đã bắt đầu lần nạp.
- **Tải xuống:** `MISSION_REQUEST_LIST` → `MISSION_COUNT` → `MISSION_REQUEST_INT` / `MISSION_ITEM_INT` (xin lại item trước cũng được).
- **Khác:** `MISSION_CLEAR_ALL`. `MISSION_SET_CURRENT` chỉ nhận lệnh mission đang ở, vì mission luôn bắt đầu từ lệnh đầu. `MISSION_CURRENT` gửi 1 Hz; `MISSION_ITEM_REACHED` gửi khi chuyển sang lệnh kế.
- **Fence và rally:** danh sách rỗng.
- **Item 0 là lệnh đầu tiên** (như PX4 / Betaflight), không phải home như ArduPilot.
- **Lệnh nhận được:**

  | Lệnh MAVLink | Section `mission` |
  |---|---|
  | `NAV_TAKEOFF` | độ cao |
  | `NAV_WAYPOINT` | điểm, độ cao, giây dừng = param1 |
  | `NAV_LOITER_TIME` | điểm, độ cao, giây = param1 |
  | `NAV_LAND` | điểm |
  | `NAV_RETURN_TO_LAUNCH` | |
  | `NAV_DELAY` | giây = param1; -1 (giờ trong ngày) không nhận |
  | `DO_SET_SERVO` | servo 1..4, 500..2500 µs |
  | `DO_CHANGE_SPEED` | m/s = param2, lưu dm/s |

  Lệnh khác: `UNSUPPORTED`.
- **Độ cao:** frame `GLOBAL_RELATIVE_ALT[_INT]` là so với home. Frame `GLOBAL[_INT]` (so với mực nước biển) được đổi sang so với home, nên cần có home.
- **Nơi lưu:** mission nạp lên ghi vào section `mission`, chính là mission app hiện và sửa. Firmware tự lưu flash khi đã disarm, rồi gửi dòng log `settings: section mission written by the firmware`. App thấy dòng đó thì tải lại section và báo "Ground station vừa nạp kế hoạch mới". Tab Nhiệm vụ tự hiện kế hoạch mới nếu kế hoạch trên bản đồ không có chỗ sửa dở.

**Gửi thêm:** `HOME_POSITION` và `GPS_GLOBAL_ORIGIN` (khi home được đặt hoặc đổi, rồi 10 s một lần), `MISSION_CURRENT` (1 Hz). HEARTBEAT báo rescue theo kiểu đang bay: RTL (6), Smart RTL (21), Land (9).

Chưa có: tham số (`PARAM_*`, Mission Planner chờ hết giờ khi tải tham số), `SET_MESSAGE_INTERVAL`, fence, lệnh arm / chế độ qua `SET_MODE` (message cũ, id 11).

## Dùng với Mission Planner (0.40.1)

Đọc từ mã nguồn Mission Planner (`GCSViews/FlightPlanner.cs`, `ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs`, `Common.cs`), **chưa thử với Mission Planner thật**:
- Windburst báo `MAV_AUTOPILOT_GENERIC` kiểu quad. Mission Planner khi đó giữ kiểu mặc định ArduCopter, nên hiện **đúng tên chế độ** theo số ArduCopter mà Windburst gửi. Set Mode (Actions) gửi `DO_SET_MODE`. Stabilize, Acro, AltHold, PosHold, Auto, RTL được; Loiter, Guided... bị từ chối.
- **Nạp mission (Write WPs):** Mission Planner chỉ chèn home vào dòng 0 cho ArduPilot. Với Windburst, dòng 0 là lệnh đầu tiên, khớp với firmware.
- **Tải mission về (Read WPs):** Mission Planner **luôn bỏ dòng 0** vì coi đó là home. Nếu dòng 0 có toạ độ, nó còn hỏi "Reset Home to loaded coords". Bật `telemetry.mp_home_item` (app: "Mission Planner: dòng home đầu mission") thì firmware gửi điểm home ở dòng 0 như ArduPilot (lệnh `NAV_WAYPOINT`, frame `GLOBAL`, độ cao so với mực nước biển; chưa có home thì toạ độ 0). Khi đó `MISSION_CURRENT` / `MISSION_ITEM_REACHED` cũng tính dòng đó: lệnh đầu là 1, đúng số dòng Mission Planner hiện. Chiều nạp lên không đổi. **Tắt khi dùng QGroundControl:** QGroundControl xử lý dòng 0 như PX4, nên sẽ thấy dư một điểm.
- **`MISSION_INT`:** lúc nối, Mission Planner xin `AUTOPILOT_VERSION`. Có cờ `MISSION_INT` thì nó dùng `MISSION_ITEM_INT` / `MISSION_REQUEST_INT`; không nhận được câu trả lời (mất trên link) thì nó dùng `MISSION_ITEM` / `MISSION_REQUEST` dạng float (độ). Từ 0.40.1 firmware hiểu và trả lời cả dạng float: toạ độ đổi qua lại `double`, sai tối đa vài mm. Trả lời bằng `MISSION_ITEM` khi được hỏi bằng `MISSION_REQUEST`.
- **Tham số:** không có. Mission Planner sẽ báo không tải được tham số, bỏ qua.

**Các bước:**
1. Nối Mission Planner với link (USB TX module ELRS 460800, WiFi backpack UDP 14550, hoặc cổng COM của radio telemetry). Bật `mp_home_item`.
2. Flight Plan: Alt chọn **Relative**. Dòng đầu là `TAKEOFF`, rồi `WAYPOINT` / `LOITER_TIME` / `LAND` / `DELAY` / `DO_SET_SERVO` / `DO_CHANGE_SPEED` / `RETURN_TO_LAUNCH`. Bấm Write WPs; Read WPs để kiểm.
3. Arm bằng công tắc (hoặc nút Arm nếu bật `remote_arm`), rồi Actions → Auto hoặc Mission Start.

**Betaflight:** Betaflight có hướng dẫn nối Mission Planner qua ELRS MAVLink (`betaflight.com`, MAVLinkELRS), nhưng chỉ cho telemetry. Betaflight dùng số chế độ riêng (Acro 0, Angle 1...) và báo autopilot BETAFLIGHT, mà Mission Planner đọc số chế độ theo ArduCopter, nên tên chế độ sẽ lệch.

## Cách chạy
- Thread cảm biến gọi `mav_send()` mỗi lượt (~1 ms). Một khung chỉ được ghi khi UART còn đủ chỗ cho cả khung (`uart_room()`). Không bao giờ cắt khung, không bao giờ chờ; khung chưa vừa sẽ đợi lượt sau và trả lại số thứ tự, nên ground station không thấy khung mất.
- Bị chậm thì không gửi dồn: sau 3 s không có chỗ, mỗi loại gửi tiếp một khung rồi về nhịp thường.
- **Điều tiết theo `RADIO_STATUS`** (0.39, chỉ khi telemetry đi trên UART của receiver MAVLink): mỗi `RADIO_STATUS.txbuf` (% bộ đệm còn trống của receiver ELRS, 100 lần mỗi giây) được đưa vào `Scheduler::radio_status()`. Như Betaflight: `txbuf` ≤ 35 thì không gửi gì. Như ArduPilot (`handle_radio_status`): `txbuf` < 20 → chu kỳ mỗi nhóm dài thêm 60 ms, < 50 → thêm 20 ms (đến khoảng 2 s), > 95 → bớt 40 ms (khi trên 200 ms), > 90 → bớt 20 ms. HEARTBEAT giữ 1 Hz và STATUSTEXT không bị làm chậm (ArduPilot gửi chúng ngoài các stream). Telemetry trên UART riêng không bị `RADIO_STATUS` của receiver làm chậm.
- Đến 0.38.3 còn gửi được qua WiFi (UDP broadcast cổng 14550). WiFi bị bỏ ở 0.38.4 (ESP32-S3 không đủ RAM trong chip cho driver WiFi, xem HANDOFF); byte `telemetry.wifi` giữ chỗ thành `reserved_t`.
- Chữ dài hơn 50 ký tự được chia thành nhiều chunk STATUSTEXT cùng `id`. Một dòng tối đa 150 ký tự; hàng đợi giữ 6 dòng, đầy thì bỏ dòng cũ nhất. Mỗi 100 ms gửi một chunk.

Mã: `src/telemetry/mavlink.h` (đóng khung, CRC, 9 message gửi + 2 message đọc, parser), `src/telemetry/mavlink_telemetry.h` (lịch gửi, chữ), `FlightApp::mav_in()` / `mav_send()` trong `src/app/flight_app.h`.

## Kiểm chứng
- `test_mavlink` (76 kiểm tra): khung so từng byte với pymavlink (`tools/mavlink_vectors.py` → `test/mavlink_vectors.h`, 15 vector, có `RC_CHANNELS_OVERRIDE` và `RADIO_STATUS` như ELRS gửi), parser, đọc hai message của ELRS, tần số, UART chậm (không cắt khung, số thứ tự liền), không gửi dồn, điều tiết theo `txbuf` (từng bước của ArduPilot, ngưỡng 35 %, HEARTBEAT và chữ không bị làm chậm), chia chữ, hàng đợi đầy, HEARTBEAT theo mode.
- `test_rx` nhóm MAVLink: decoder (kênh, 0 / 65535, kẹp, LQ / RSSI / `txbuf`, khung hỏng, message khác vào hàng đợi cho ground station).
- 0.40: `test_mavlink` thêm 18 message của ground station so từng byte với pymavlink (33 vector) và đọc lại, hàng đợi trả lời, `HOME_POSITION`, `MISSION_CURRENT`, Smart RTL / Land. `test_gcs` (87): giao thức mission (nạp, tải, thứ tự, xin lại, bỏ cuộc, đang bay, quá chỗ, frame, giá trị sai, người khác chen vào, xoá, current, reached, fence / rally) và từng lệnh với máy bay giả. `test_core` / `test_flight`: arm / disarm từ xa, chế độ ghi đè tới khi gạt công tắc. `test_fw` nhóm `mavlink gcs`: ground station qua link ELRS và qua radio telemetry; nạp mission rồi bay hết nó bằng `DO_SET_MODE AUTO`, lưu flash, tải lại, chế độ, arm, home, khởi động lại.
- `test_fw` nhóm `mavlink`: mạch ảo, giá trị lấy từ mô hình máy bay (tư thế, GPS, độ cao khi bay Alt Hold), báo cáo khởi động thành STATUSTEXT, UART chậm, cài đặt (baud, sysid, tần số 0, hexa), bảng cổng, không có chân TX, chân trùng.
- `test_fw` nhóm `mavlink receiver`: mạch ảo có receiver ELRS chế độ MAVLink (bộ đệm 1 kB như ELRS, xả theo sức chở của packet rate, đầy thì mất byte): 16 kênh, 100 khung/s, LQ, arm và bay, mất sóng; telemetry qua bộ đệm ở 2375 B/s và ở 360 B/s (250Hz: không mất byte nào, HEARTBEAT vẫn 1 Hz); `receiver.telemetry` tắt; telemetry riêng không bị làm chậm; cả hai cùng bật; không có chân TX.
- 0.40.1: `test_gcs` thêm dạng float (`MISSION_ITEM` / `MISSION_REQUEST`) và dòng home cho Mission Planner (100 kiểm tra); `test_fw` nạp bằng `MISSION_ITEM` qua link ELRS, tải về có home đứng đầu, `MISSION_CURRENT` tính dòng đó.
- `test_app` (chọn UART), mutant `mav:` (30), `mavrx:` (30), `gcs:` (52) trong `tools/mutate.py`.
- App: khối "Telemetry (MAVLink)" ở tab Chân và cổng; `test_gui_browser.py::test_telemetry_block` (chưa chạy: máy này không có playwright).
- **Chưa thử trên mạch thật**, với Mission Planner / QGroundControl hay với radio thật.
