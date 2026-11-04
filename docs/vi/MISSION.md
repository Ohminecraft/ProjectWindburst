# Nhiệm vụ bay (waypoint) — firmware 0.37

Drone tự bay một kế hoạch đã nạp: cất cánh, bay qua các điểm, ở điểm này đứng yên rồi chạy servo, ở điểm kia hạ cánh, tắt motor chờ người lấy hoặc gắn đồ, rồi tự cất cánh bay tiếp, cuối cùng về nhà hạ cánh. Cách chạy theo chế độ **AUTO của ArduCopter**: đã đọc mã thật (`ArduCopter/mode_auto.cpp`, `libraries/AP_Mission/AP_Mission.cpp`, `ArduCopter/events.cpp`, `AP_ServoRelayEvents`) rồi viết lại, gọn hơn.

**Chưa bay thử trên mạch thật.** Đã kiểm chứng trên PC: unit test bộ chạy, mô phỏng vòng kín (`test_flight`), và firmware thật trên mạch ảo (`test_fw`, nhóm `mission`).

Thiết kế: `docs/superpowers/specs/2026-10-29-mission-design.md`. Những quyết định đưa ra lúc bạn vắng: `docs/MISSION_WORKLOG.md`.

## Đã có (phần 1: firmware)
- Lưu kế hoạch tối đa **32 lệnh** trong cài đặt (section `mission`, lưu flash như mọi cài đặt), nạp và đọc qua WBP: USB / UART hoặc bản mô phỏng.
- Bắt đầu bằng **công tắc MISSION** (tab Mode) hoặc **lệnh WBP `MISSION`** (`1` = bắt đầu, `0` = dừng).
- Servo nhiệm vụ: kênh `Nhiệm vụ` ở tab Motor, kèm góc tại xung nhỏ nhất / lớn nhất và xung ban đầu.
- Còi và motor kêu nhanh trong **10 giây cuối** trước khi drone tự cất cánh lại. Không tắt được.
- OSD và MAVLink báo chế độ AUTO; stream `nav` có trạng thái nhiệm vụ.

## Tab "Nhiệm vụ" trong app (phần 2)
Bố cục C đã chọn: bản đồ chiếm hết tab, các ô nổi bên trên.

- **Bản đồ:** ảnh vệ tinh (Esri World Imagery) hoặc bản đồ đường phố (OpenStreetMap). Trên ảnh vệ tinh có lớp tên đường, phường, địa danh như chế độ vệ tinh của Google (Esri `Reference/World_Transportation` và `Reference/World_Boundaries_and_Places`, ảnh PNG trong suốt); nút "Tên đường" bật / tắt lớp này, mặc định bật. Không lấy tile của Google như Betaflight Configurator (`mt1.google.com/vt`): đó không phải API công khai, và điều khoản của Google cấm cả lấy trực tiếp lẫn lưu tile. Ô bản đồ đi qua server `wbclient` (`/api/tile`, `wbclient/maptiles.py`) và được lưu vào `~/Windburst/tiles`. Chỗ đã xem một lần thì lần sau mở được khi không có mạng; chỗ chưa có thì để trống.
- **Tìm địa chỉ:** gõ rồi bấm Enter (OpenStreetMap Nominatim), bấm vào kết quả là bản đồ nhảy tới đó.
- **Vị trí của tôi:** bản đồ nhảy tới chỗ đặt laptop, vẽ chấm xanh và vòng sai số.
  - Vị trí lấy từ dịch vụ vị trí của Windows (GPS của máy nếu có, nếu không thì dò theo các mạng Wi-Fi xung quanh). Lần đầu Windows hỏi quyền.
  - Cửa sổ app (`desktop.py`) cho trang hỏi thẳng Windows, không hiện thêm hộp hỏi riêng của WebView2. Mở bằng trình duyệt thì trình duyệt hỏi trước.
  - Bị chặn hoặc không tìm được: trang ghi cần bật gì, và có nút mở trang Cài đặt > Quyền riêng tư và bảo mật > Vị trí của Windows (`/api/location-settings`).
- **Vẽ kế hoạch:**
  - Bấm lên bản đồ để thêm điểm, kéo điểm để dời.
  - Bấm vào một điểm để mở ô sửa: chọn **Bay tới** (giữ N giây), **Đứng yên · servo** (đứng N giây, rồi từng bước: servo 1–4 tới góc tính bằng độ, chờ M giây), hoặc **Hạ cánh · chờ · cất cánh** (chờ N giây dưới đất, cất cánh lại lên độ cao của điểm).
  - Số mét được ghi trên từng chặng.
- **Tuỳ chọn chung:** độ cao cất cánh, tốc độ (0 = tốc độ của rescue), về nhà hạ cánh sau điểm cuối, mất sóng thì bay tiếp hay failsafe.
- **Dòng dưới cùng:** drone đang làm gì (lệnh mấy trên bao nhiêu), còn bao xa tới điểm, còn chờ bao lâu, độ cao, pin, cách nhà, và cả chuyến (số mét, thời gian ước tính, số lệnh / 32). Quá 32 lệnh thì số này chuyển màu đỏ và không cho nạp.
- **Drone trên bản đồ:** mũi tên đỏ theo GPS và hướng mũi, vệt đỏ là chỗ đã bay qua (từ lúc arm).
- **Các nút:**
  - **Nạp vào drone:** ghi section `mission` rồi lưu flash. Phải lúc chưa arm.
  - **Tải từ drone:** đọc lại kế hoạch đang lưu trong drone.
  - **Xoá hết điểm** (ô tuỳ chọn): hỏi lại rồi xoá mọi điểm trên bản đồ. Kế hoạch trong drone giữ nguyên tới lần Nạp sau.
  - **Bắt đầu / Dừng:** gửi lệnh `MISSION`. Drone phải được arm bằng công tắc trước.
- Độ đổi sang xung theo "góc ở xung nhỏ nhất / lớn nhất" của servo ở tab Motor.
- Code: `web/js/tabs/mission.js` (giao diện, Leaflet 1.9.4 đóng gói sẵn trong `web/vendor/leaflet`, giấy phép BSD-2), `web/js/mission_plan.js` (đổi kế hoạch qua lại với section `mission`, ước tính quãng đường).

**Qua MAVLink (0.40, phần 3):** Mission Planner / QGroundControl nạp và tải kế hoạch, bắt đầu nó (`MISSION_START` hoặc chế độ AUTO), theo dõi lệnh đang bay (`MISSION_CURRENT`, `MISSION_ITEM_REACHED`) qua link ELRS chế độ MAVLink hoặc radio telemetry (`docs/MAVLINK.md`, `docs/ELRS_MAVLINK.md`). Kế hoạch nạp từ ground station nằm cùng section `mission` với kế hoạch của app; app tự tải lại khi firmware lưu nó (lúc đã disarm), tab Nhiệm vụ hiện ngay nếu kế hoạch trên bản đồ không có chỗ sửa dở. App vẫn nối tới drone bằng dây (USB / UART), nên theo dõi khi đang bay thì dùng ground station.

## Các lệnh
| Lệnh | `MAV_CMD` | Dùng | Drone làm gì |
|---|---|---|---|
| TAKEOFF | 22 | `alt` | Lên thẳng tới `alt` (so với nhà) tại chỗ đang đứng. Altitude Hold tự cất cánh, không cần cần ga |
| WAYPOINT | 16 | `lat`, `lon`, `alt`, `p` = giây giữ | Bay tới điểm và dừng đúng trên điểm, giữ `p` giây |
| LOITER_TIME | 19 | `p` = giây (`lat` = `lon` = 0: tại điểm vừa tới) | Đứng yên `p` giây |
| LAND | 21 | `lat`, `lon` (0 = tại chỗ) | Bay tới điểm, giữ nguyên độ cao, rồi hạ cánh. Chạm đất thì **tắt motor** |
| DELAY | 93 | `p` = giây | Chờ. Sau LAND là chờ dưới đất với motor tắt |
| RTL | 20 | — | Bay về nhà và hạ cánh (dùng rescue RTL của 0.36) |
| SERVO | 183 | `arg` = servo 1..4, `p` = xung µs | Đặt servo (DO_SET_SERVO) |
| SPEED | 178 | `p` = dm/s | Tốc độ ngang cho các chặng sau (DO_CHANGE_SPEED) |

Độ cao tính so với chỗ arm lần đầu. Toạ độ tính bằng 1e-7 độ. Lệnh SERVO / SPEED (lệnh DO) chạy **khi lệnh bay đứng trước nó bắt đầu**, giống `AP_Mission::update`. Vì vậy một bước "đứng yên rồi chạy servo" được viết là:

```
WAYPOINT P          bay tới P
LOITER_TIME 3 s     tới P: ...
SERVO 2 -> 1800 us  ... servo chạy ngay, rồi chờ 3 s
LOITER_TIME 2 s
SERVO 2 -> 1100 us  bước thứ hai, rồi chờ 2 s
```

Một điểm giao hàng là `LAND P`, `DELAY 60`, `TAKEOFF 12`.

Tốc độ mặc định, gia tốc, xích bám, độ cao bắt đầu hạ chậm và tốc độ hạ dùng chung với rescue (tab Vị trí): bay `rescue_ground_speed`, tăng / giảm tốc 2 m/s², dưới `rescue_landing_alt` hạ bằng `rescue_land_speed`. Tới điểm khi cách điểm ngang dưới 2 m và chênh độ cao dưới 1 m (giống `WPNAV_RADIUS` của ArduPilot). Mũi drone quay theo hướng bay.

## Cách bay
1. Ghi kế hoạch (section `mission`), bấm Lưu.
2. Đặt drone ở sân, chờ GPS fix.
3. **Arm bằng công tắc ARM** như thường. Lần arm đầu tiên luôn do người làm.
4. Bật công tắc **MISSION**, hoặc gửi `MISSION 1` từ app: drone tự cất cánh và bay.
   - Trên mặt đất, nếu lệnh bay đầu tiên không phải TAKEOFF thì nhiệm vụ bị từ chối (`REFUSED`, như ArduCopter "Missing Takeoff Cmd").
   - Công tắc MISSION đã bật sẵn từ trước khi arm cũng bị từ chối: gạt tắt rồi bật lại.
5. Lấy lại quyền điều khiển: tắt công tắc MISSION, hoặc gửi `MISSION 0`. Drone trả về các mode người lái đang chọn ngay lập tức.

## Ở điểm giao hàng (khác ArduPilot, có chủ ý)
ArduPilot (`MIS_OPTIONS` CONTINUE_AFTER_LAND) để motor quay ở ga thấp nhất trong lúc chờ. Windburst **tắt hẳn motor** vì có người lại gần:

- Chạm đất xong thì motor tắt và đếm thời gian `DELAY`.
- **10 giây cuối** còi kêu nhanh, motor (DShot) kêu bíp, OSD hiện `TAKEOFF 7S`.
- Hết giờ thì drone tự arm lại và cất cánh. Lần arm này bỏ qua ba điều kiện: cần ga chưa ở thấp, công tắc ARM đang bật, mất sóng RC. Mọi điều kiện khác vẫn chặn: nghiêng, pin, IMU...
- **Gạt công tắc ARM về OFF trong lúc chờ là drone không tự cất cánh nữa** (nhiệm vụ kết thúc).
- Điểm home và gốc độ cao được giữ từ lần arm đầu tiên, nên kế hoạch không bị lệch dù chỗ giao hàng cao thấp khác nhà.
- Nếu sau LAND không còn TAKEOFF nào thì nhiệm vụ kết thúc dưới đất (như ArduCopter `verify_land`).

## Khi trục trặc
| Chuyện gì | Drone làm gì |
|---|---|
| Mất sóng RC | Bay tiếp nhiệm vụ (ArduPilot `FS_OPTIONS` continue in AUTO). Chọn `mission.rc_lost = RTL` để failsafe chạy như thường |
| Pin yếu / pin nguy hiểm | Failsafe pin như cũ (RTL hoặc LAND theo tab Vị trí). Nhiệm vụ dừng |
| Mất vị trí GPS quá 1 s khi đang bay | Bỏ nhiệm vụ (`FAILED`), hạ cánh tại chỗ (rescue LAND) |
| Người lái disarm | Nhiệm vụ dừng |
| Hết lệnh khi đang bay | Đứng yên ở điểm cuối (như ArduCopter `exit_mission`) |
| Hết lệnh dưới đất | Tắt motor một lần |

## App, OSD, MAVLink
- Tab Mode: mode **Nhiệm vụ (AUTO)**. Tab Motor: kênh servo **Nhiệm vụ**, cùng góc ở xung nhỏ nhất / lớn nhất (mặc định 0° và 180°) và xung ban đầu (mặc định 1500 µs). Tab Pin và còi: điều kiện **Nhiệm vụ sắp tự cất cánh**.
- Stream `nav`: `mission_state`, `mission_index`, `mission_count`, `mission_dist` (m tới điểm hiện tại), `mission_left` (m cả chuyến còn lại), `mission_wait` (giây chờ còn lại).
- OSD: chế độ `AUTO`, dòng cảnh báo `AUTO 3/9 120M`, `WAIT 45S`, `TAKEOFF 7S`. Lúc nhiệm vụ đang về nhà thì OSD hiện của rescue (`RTL 123M`).
- MAVLink HEARTBEAT: `custom_mode` AUTO (3), lúc RTL thì RTL (6) (Smart RTL 21 / Land 9 theo kiểu rescue). Từ 0.40: nạp / tải kế hoạch, `MISSION_CURRENT`, `MISSION_ITEM_REACHED`, `MISSION_START`.

## Mã
- `src/core/nav/mission.h`: bộ chạy (portable). Bay theo từng chặng giống đường bay của rescue: điểm đích chạy dọc chặng từ chỗ drone dừng được, rồi phanh để dừng ngay trên điểm.
- `src/core/flight_core.h`: bật/tắt nhiệm vụ, ưu tiên (rescue > nhiệm vụ > Position Hold), arm/tắt motor ở điểm giao hàng (`Arming::mission_arm` / `mission_disarm`, `safety.h`), cất cánh tự động (`AltInput::auto_takeoff`, `nav/alt_control.h`), servo.
- `src/proto/settings_apply.h`: `mission_plan_from()` đổi lat/lon sang mét so với home bằng chính `gps::offset_cm` của GPS.
- `src/app/flight_app.h`: lệnh WBP `MISSION`, giữ home và gốc baro trong lúc chờ, servo, còi, telemetry, OSD.

## Kiểm chứng (trên PC)
- `test_mission` (41): từng lệnh, thứ tự DO/NAV, từ chối, giữ home ngay từ lúc tắt motor, đếm ngược, mất vị trí, RTL, SPEED.
- `test_flight` (`t_mission`): cả chuyến giao hàng trên mô phỏng. Hạ cánh 0 m cách điểm giao, chờ 14,9 s (đặt 15), còi đúng 10 s, về cách nhà 0,2 m, nghiêng tối đa 25°. Thêm các trường hợp: mất sóng giữa chặng vẫn bay tiếp; `rc_lost = RTL` thì failsafe như thường; tắt công tắc là lấy lại máy; gạt ARM về OFF lúc chờ thì không tự cất cánh; pin nguy hiểm thì failsafe hạ cánh; kế hoạch bắt đầu bằng WAYPOINT dưới đất bị từ chối; bắt đầu và dừng từ app.
- `test_fw` (nhóm `mission`, 14): kế hoạch lat/lon trong cài đặt, GPS ảo. Servo ở xung ban đầu rồi lên 1800 µs trên điểm, hạ cánh cách điểm giao dưới 2,5 m, chờ đúng 12 s, còi bíp 80/80 ms trong 10 s cuối. Khoảng cách home trong telemetry khớp khoảng cách thật cả lúc chờ lẫn sau lần arm thứ hai. RTL về đúng chỗ cất cánh. Lệnh `MISSION` bị từ chối khi chưa arm, nhận khi đã arm.
- Unit: `test_nav_ctl` (cất cánh tự động), `test_core` (arm/disarm do nhiệm vụ), `test_nav` (rescue theo kiểu bộ gọi chỉ định), `test_proto` (schema, đổi lat/lon, cờ còi), `test_osd`, `test_mavlink`.

## Cần thử trên mạch (xem `todo.txt`)
Thử từng bước, công tắc trước, gần nhà, có người giữ tay điều khiển:

1. Gán công tắc MISSION. Kế hoạch: TAKEOFF 5 m rồi một WAYPOINT cách 10 m. Arm, bật công tắc: drone phải tự lên 5 m, bay tới điểm, đứng yên. Tắt công tắc: Altitude Hold / Position Hold của bạn phải lấy lại máy ngay.
2. Thêm LOITER_TIME + SERVO: servo phải chạy khi drone đứng trên điểm.
3. Thêm `LAND` + `DELAY 30` + `TAKEOFF`: drone phải hạ cánh, tắt motor, 10 s cuối còi kêu nhanh, rồi tự cất cánh. Thử thêm: gạt ARM về OFF lúc chờ thì drone phải nằm yên.
4. Thêm RTL ở cuối: drone phải về đúng chỗ arm lần đầu, không phải về chỗ giao hàng.

Bật nhóm blackbox **"Dữ liệu dẫn đường chi tiết"** khi thử, và gửi log nếu drone lệch điểm hơn 3 m hoặc lắc khi tự cất cánh.
