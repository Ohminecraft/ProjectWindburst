# Nhiệm vụ bay (waypoint) — thiết kế

Ngày: 2026-10-29. Người dùng duyệt từng phần trong chat (bố cục C, cách hạ cánh-chờ-cất cánh, cách bắt đầu, failsafe, servo, chia 3 phần). Người dùng vắng vài giờ và dặn: cứ làm, vướng thì ghi vào `docs/MISSION_WORKLOG.md` rồi làm tiếp.

## 1. Ý định

Drone tự bay một chuyến giao/nhận đồ: cất cánh ở sân nhà, bay qua các điểm, ở một số điểm **hạ cánh, tắt motor, chờ người gắn/lấy đồ, rồi tự cất cánh bay tiếp**, ở điểm khác **đứng yên trên không và chạy servo** (thả móc, mở hộp...), cuối cùng về nhà hạ cánh. Tầm 1–2 km. Người dùng ở nhà với laptop và RadioMaster Pocket (ELRS 2.4 GHz, module trong có backpack).

Kế hoạch vẽ trên một **tab "Nhiệm vụ" trong app Windburst** (không dùng Mission Planner), xem vị trí drone trực tiếp trên bản đồ.

## 2. Mục tiêu, không làm, phải tránh

Mục tiêu
- Kế hoạch là danh sách lệnh theo **ngữ nghĩa nhiệm vụ của ArduPilot** (MAVLink `MISSION_ITEM_INT`, các `MAV_CMD_*` cùng số), để sau này Mission Planner / QGroundControl nạp được và giao thức MAVLink chỉ là một lớp dịch.
- Drone tự làm hết nhiệm vụ kể cả khi mất sóng tay điều khiển và mất app.
- Mọi thứ kiểm chứng được trên PC (unit test, FIL trên mạch ảo, test app với `wb_sim`).

Không làm (lần này)
- Bay theo địa hình (terrain following), spline, vòng tròn, ROI, camera, nhảy lệnh (`DO_JUMP`), nhiều kế hoạch.
- Sửa kế hoạch giữa chuyến bay (ArduPilot cho phép; ở đây kế hoạch mới chỉ áp dụng ở lần bắt đầu sau — xem 4.6).
- Mission Planner / QGroundControl qua MAVLink (phần 3).

Phải tránh
- Motor tự quay khi có người đứng sát mà không báo trước.
- Bắt đầu nhiệm vụ do một cú bấm nhầm trên laptop khi drone chưa được người arm.
- Kế hoạch "bay lạc" vì gốc tọa độ / độ cao bị đặt lại ở lần arm thứ hai (sau khi hạ cánh giữa chuyến).

## 3. Ba phần, theo thứ tự

1. **Firmware chạy nhiệm vụ** (tài liệu này, phần lớn): lưu kế hoạch, bộ chạy, tích hợp vào `FlightCore`, servo nhiệm vụ, còi/LED, telemetry, OSD, lệnh WBP bắt đầu/dừng.
2. **App: tab "Nhiệm vụ"** (phương án C): bản đồ toàn tab, ô nổi sửa điểm, nạp/tải, theo dõi. Nối bằng WBP như mọi tab khác: USB, WiFi của ESP32 (`socket://<ip>:5760`, đã có), hoặc `wb_sim`.
3. **ELRS chế độ MAVLink**: nhận RC qua `RC_CHANNELS_OVERRIDE`, `RADIO_STATUS` làm điều tiết, MAVLink hai chiều trên cổng receiver, giao thức nhiệm vụ MAVLink; app nối UDP 14550 tới backpack. Chỉ thử thật được khi có mạch + Pocket + receiver ESP.

Ghi chú: trong chat tôi đã nói phần 1 "nhận và lưu kế hoạch theo giao thức nhiệm vụ MAVLink". Quyết định khi viết thiết kế (của tôi): phần 1 lưu kế hoạch **theo đúng ngữ nghĩa MAVLink** nhưng nạp qua **WBP** (đường app đang có, chạy được ngay qua USB/WiFi/sim). Lớp giao thức nhiệm vụ MAVLink (`MISSION_COUNT` / `MISSION_REQUEST_INT` / `MISSION_ITEM_INT` / `MISSION_ACK`) thuộc phần 3, vì hiện firmware chưa có đường nhận MAVLink nào (UART MAVLink và UDP WiFi chỉ gửi).

## 4. Phần 1 — firmware

### 4.1 Nguồn tham khảo (đã đọc mã thật, `master`)
- `ArduCopter/mode_auto.cpp`: `start_command`, `verify_*`, `takeoff_start`, `wp_start`, `do_land`/`verify_land`, `do_loiter_time`/`verify_loiter_time`, `do_nav_delay`/`verify_nav_delay`, `exit_mission`, `init` ("Missing Takeoff Cmd").
- `libraries/AP_Mission/AP_Mission.cpp`: `update()` — lệnh DO nằm sau lệnh NAV N được chạy khi N **bắt đầu**; `continue_after_land_check_for_takeoff` (`MIS_OPTIONS` bit 2).
- `libraries/AP_ServoRelayEvents`: `DO_SET_SERVO` chỉ điều khiển servo không gắn chức năng khác.
- `ArduCopter/events.cpp`: `FS_OPTIONS` "continue if in AUTO" khi mất RC.
- `libraries/GCS_MAVLink/MissionItemProtocol.cpp`: giao thức nạp (phần 3).

### 4.2 Lệnh hỗ trợ (cùng số `MAV_CMD`)
| Lệnh | Số | Tham số dùng | Ý nghĩa |
|---|---|---|---|
| `NAV_TAKEOFF` | 22 | alt | cất cánh thẳng đứng tại chỗ tới alt (so với nhà). Trên đất: tự arm nếu là lệnh tiếp sau một lần hạ cánh giữa chuyến (4.4) |
| `NAV_WAYPOINT` | 16 | lat, lon, alt, p1 = giây giữ | bay tới điểm; tới nơi giữ p1 giây |
| `NAV_LOITER_TIME` | 19 | lat, lon (0 = chỗ hiện tại), alt (0 = giữ), p1 = giây | đứng yên tại điểm p1 giây (đếm khi đã tới) |
| `NAV_LAND` | 21 | lat, lon (0 = tại chỗ) | bay tới điểm (giữ độ cao) rồi hạ cánh; chạm đất thì **tắt motor** |
| `NAV_DELAY` | 93 | p1 = giây | chờ (dùng sau `NAV_LAND`: chờ dưới đất, motor tắt) |
| `NAV_RETURN_TO_LAUNCH` | 20 | — | về nhà và hạ cánh bằng rescue RTL (0.36) |
| `DO_SET_SERVO` | 183 | p1 = servo 1..4, p2 = xung µs | đặt servo nhiệm vụ |
| `DO_CHANGE_SPEED` | 178 | p2 = m/s | tốc độ ngang cho các chặng sau |

Lệnh khác: bỏ qua (ArduPilot: "unable to use the command, try the next").

App ghép lệnh thành "điểm" (thiết kế phần 2):
- Bay tới: `NAV_WAYPOINT`.
- Đứng yên · servo: `NAV_WAYPOINT(P)` rồi với mỗi bước k: `NAV_LOITER_TIME(P, chờ_k)` + `DO_SET_SERVO(servo_k, xung_k)` (DO đi sau LOITER nên chạy lúc LOITER bắt đầu = lúc tới nơi / lúc bước trước hết giờ — đúng thứ tự ArduPilot).
- Hạ cánh · chờ · cất cánh: `NAV_LAND(P)` + `NAV_DELAY(t)` + `NAV_TAKEOFF(alt)`.
- Cuối: `NAV_RETURN_TO_LAUNCH`.

### 4.3 Lưu trữ
- Section settings mới `mission` (id 26, live, mảng song song, tối đa **32 lệnh**): `count`, `cmd[32]` u8, `arg[32]` u8, `alt[32]` i16 (dm, so với nhà), `lat[32]`/`lon[32]` i32 (1e-7 độ), `p[32]` u16. 449 byte < 512 (`kMaxPayload`). Lưu flash như mọi section (NVS / flash STM32).
- `cmd` là mã nội bộ (enum `mission_cmd` trong schema: NONE, TAKEOFF, WAYPOINT, LOITER_TIME, LAND, DELAY, RTL, SERVO, SPEED); bảng đổi sang/từ `MAV_CMD` ở một chỗ.
- `arg`: số servo (SERVO). `p`: giây (WAYPOINT, LOITER_TIME, DELAY), xung µs (SERVO), dm/s (SPEED).

### 4.4 Bộ chạy (`src/core/nav/mission.h`, portable)
Đầu vào mỗi bước ước lượng: đã arm, trên không, vị trí NE / vận tốc / độ cao (từ nhà), hướng, vị trí hợp lệ, trạng thái Altitude Hold (LANDED_IDLE), thời gian. Kế hoạch đưa vào đã đổi sang NE mét (FlightApp đổi lat/lon bằng cùng hàm `offset_cm` của GPS home lúc bắt đầu).

Đầu ra (`MissionOutput`): `active`, `horizontal` + `tgt_n/e` + `vel_n/e` (như `RescueOutput`, bay bằng `rctl_`), `climb_mps` (Altitude Hold `ext_climb`), `takeoff` (cất cánh tự động), `want_disarm`, `want_arm`, `warn` (còi/LED báo sắp cất cánh), `servo` (số + xung, một lần), `want_rtl`, `index`, `phase`, `dist_m` (tới đích hiện tại), `left_m` (còn lại cả chuyến).

Hành vi (ArduCopter, trừ chỗ ghi "lệch"):
- Bắt đầu: trên đất mà lệnh NAV đầu không phải TAKEOFF → từ chối ("Missing Takeoff Cmd"). Bắt đầu lúc đang bay: chạy từ lệnh 0.
- TAKEOFF: Altitude Hold cất cánh (cờ mới `AltInput::auto_takeoff`: LANDED_IDLE → TAKEOFF không cần cần ga; đoạn dốc ga không bị cần ga thấp hủy), rồi leo bằng `ext_climb` tới alt. Xong khi tới alt ±0,5 m. Ngang: giữ chỗ cất cánh.
- WAYPOINT: chặng thẳng từ điểm dừng (vị trí + v²/2a) tới đích; điểm đích chạy dọc chặng với tốc độ ≤ tốc độ bay, giới hạn gia tốc, phanh bằng `sqrt_controller` theo khoảng còn lại, có xích (leash) như rescue. Độ cao: nội suy theo tiến độ chặng + `sqrt_controller` (như rescue FLY_HOME). Tới nơi: ngang < `accept_m` (2 m, WPNAV_RADIUS) và đứng < 1 m; rồi giữ p1 giây.
- LOITER_TIME: giữ điểm; đếm khi tới nơi.
- LAND: (nếu có điểm) bay tới giữ độ cao, rồi hạ: nhanh tới `landing_alt_m` (4 m) rồi `land_speed` (0,5 m/s), giữ ngang. Altitude Hold báo LANDED_IDLE → `want_disarm` (**lệch có chủ ý**: ArduPilot với `CONTINUE_AFTER_LAND` để motor quay ở ground idle; ở đây tắt hẳn vì có người lại gần). Nếu lệnh NAV sau không phải TAKEOFF: nhiệm vụ kết thúc trên đất (như `verify_land` ArduPilot: disarm, dừng).
- DELAY: đếm p1 giây (trên đất, motor tắt). 10 s cuối: `warn`. Hết giờ → lệnh sau.
- TAKEOFF sau khi đã hạ cánh giữa chuyến: `want_arm` (+ `warn`) cho tới khi đã arm, rồi như TAKEOFF thường.
- RTL: `want_rtl` → FlightCore cho rescue bay RTL (kiểu RTL bất kể `rescue_mode`). Nhiệm vụ xong khi rescue đã hạ cánh.
- Hết lệnh: đang bay → giữ vị trí (ArduPilot `exit_mission` → loiter); trên đất → disarm.
- Hướng mũi: theo hướng chặng khi bay nhanh hơn 1 m/s (như rescue), giữ nguyên khi đứng.

### 4.5 Tích hợp `FlightCore`
- Mode mới `M_MISSION` (thêm cuối enum `mode`, append-only; kéo theo ANGLE như các mode giữ). Nhiệm vụ chạy khi: đã arm (lần đầu do người arm bằng công tắc), và (công tắc MISSION bật **hoặc** app gửi lệnh `MISSION` start). Tắt công tắc / lệnh stop → thoát, trả lại cho các mode người lái đang chọn (ArduPilot: rời AUTO là dừng nhiệm vụ).
- Thứ tự ưu tiên: rescue (công tắc hoặc failsafe) > nhiệm vụ > Position Hold > yêu cầu ngoài (P7).
- Failsafe trong nhiệm vụ:
  - Mất RC: **tiếp tục** (ArduPilot `FS_OPTIONS` continue-in-AUTO). Tùy chọn mới `nav.mission_rc_lost` = CONTINUE (mặc định) / RTL.
  - Pin yếu / nguy hiểm: theo `fs_batt_low` / `fs_batt_crit` như hiện nay (RTL / LAND) — được ưu tiên hơn nhiệm vụ.
  - Mất vị trí GPS giữa chặng: hạ cánh tại chỗ (giữ ngang nếu còn vận tốc) — `MissionPhase::FAILED` + rescue LAND.
- Arm/disarm do nhiệm vụ (`Arming::mission_disarm` / `mission_arm`): disarm không gắn "sticky", không đòi gạt công tắc; arm lại chỉ khi công tắc ARM của người vẫn ở ON (hoặc đang mất sóng — đã chốt "tiếp tục khi mất sóng"), bỏ qua blocker "cần ga chưa thấp", "công tắc ARM đang bật", "mất RC"; mọi blocker khác vẫn chặn (IMU, nghiêng, pin nguy hiểm, cảm biến...). Người gạt ARM về OFF trong lúc chờ → nhiệm vụ dừng, không tự cất cánh.
- Lần arm lại do nhiệm vụ **không** đặt lại: GPS home, `reset_position()`, gốc độ cao của ước lượng (vẫn là chỗ cất cánh đầu tiên).

### 4.6 Nạp, bắt đầu, dừng (WBP)
- Kế hoạch: section `mission` (đọc/ghi/lưu như mọi section). Kế hoạch đổi trong lúc nhiệm vụ đang chạy: lần chạy hiện tại giữ bản đã nạp lúc bắt đầu.
- Lệnh WBP mới `MISSION` (arg: 0 dừng, 1 bắt đầu). Bắt đầu bị từ chối nếu: chưa arm, kế hoạch rỗng/sai, không có home / vị trí, trên đất mà lệnh NAV đầu không phải TAKEOFF.
- Trước khi bắt đầu: app ước tính quãng đường/thời gian (phần 2); firmware không tự từ chối vì pin.

### 4.7 Servo nhiệm vụ
- `servo_source` thêm `MISSION` (cuối enum). Section `servo` thêm (cuối): `angle_min[4]`, `angle_max[4]` (độ ứng với `min_us` / `max_us`, mặc định 0 / 180), `init_us[4]` (xung lúc chưa có lệnh, mặc định 1500). `DO_SET_SERVO` mang xung µs (giống MAVLink); app đổi độ ↔ µs bằng `angle_min/max` + `min/max_us` (+ `reverse`).
- Servo `MISSION` không bao giờ đi theo kênh RC; giữ xung cuối cùng sau nhiệm vụ cho tới khi tắt nguồn hoặc lệnh khác.

### 4.8 Báo hiệu, telemetry, OSD
- Còi + LED: điều kiện beeper mới `MISSION_WARN` (thêm cuối `beeper_cond`): kêu nhanh trong 10 s trước khi nhiệm vụ tự arm. LED: nháy trắng/đỏ nếu có dải LED.
- Stream `nav` thêm (cuối): `mission_state` (enum), `mission_index`, `mission_count`, `mission_dist` (dm tới đích hiện tại, u16), `mission_left` (m cả chuyến còn lại, u16), `mission_wait` (s còn lại khi chờ, u8).
- OSD: tên mode `AUTO`; dòng cảnh báo `WP 3/9 120M`, `LAND WAIT 45S`, `TAKEOFF 5S` (đếm ngược).
- MAVLink telemetry (gửi): HEARTBEAT `custom_mode` / base mode AUTO khi nhiệm vụ chạy, `MISSION_CURRENT`.

### 4.9 Kiểm chứng
- Unit (`test_nav`): bộ chạy từng lệnh, thứ tự DO/NAV, từ chối thiếu TAKEOFF, đổi lat/lon → NE, ước tính quãng đường.
- Closed-loop (`test_flight`): nhiệm vụ hoàn chỉnh trên mô phỏng (cất cánh → 2 điểm → servo → hạ cánh-chờ-cất cánh → RTL), gió ngang; mất RC giữa chặng vẫn tiếp tục; tắt công tắc giữa chừng thì dừng; công tắc ARM OFF khi chờ thì không cất cánh.
- FIL (`test_fw`): nạp section qua WBP, lệnh MISSION, servo nhiệm vụ đổi xung, còi báo trước khi tự arm, home không đổi sau lần arm thứ hai, OSD.
- Mutants trong `tools/mutate.py`; docs `docs/MISSION.md` + HANDOFF + todo; bump version.

## 5. Phần 2 — app (tóm tắt, viết chi tiết khi tới)
- Tab `mission` theo phương án C (bản phác: `.superpowers` trong scratchpad, `layout-c-v2.html`): bản đồ toàn tab, ô tìm địa chỉ, vệ tinh / đường phố, điểm đánh số + khoảng cách từng chặng, kéo để dời, ô nổi sửa việc, drone + vệt bay, HUD dưới, ước tính quãng đường/thời gian, nút Tải / Nạp / Bắt đầu.
- Bản đồ: Leaflet (đóng gói sẵn trong app, không CDN); ảnh nền qua backend Python có cache đĩa (OSM đường phố, Esri World Imagery vệ tinh; có ghi công nhận); tìm địa chỉ bằng Nominatim. Không có mạng: dùng ảnh đã cache.
- Nạp = ghi section `mission` + lưu; Tải = đọc section.

## 6. Phần 3 — ELRS MAVLink (tóm tắt)
- Receiver ELRS ở chế độ MAVLink trên một UART: firmware đọc `RC_CHANNELS_OVERRIDE` làm nguồn RC (kiểu receiver mới), tôn trọng `RADIO_STATUS.txbuf`, gửi telemetry MAVLink trên cùng UART.
- Giao thức nhiệm vụ MAVLink + `COMMAND_LONG` (`MAV_CMD_MISSION_START`, `DO_SET_MODE`) dịch sang section `mission` / lệnh MISSION.
- App: kiểu kết nối "ELRS (MAVLink UDP 14550)" chỉ cho tab Nhiệm vụ.

## 7. Để lại cho người làm
Tên biến, cách chia file, tham số mặc định chưa ghi ở trên (tốc độ bay mặc định 5 m/s, gia tốc 2 m/s², xích 5 m — lấy như rescue), chi tiết vẽ OSD/LED.
