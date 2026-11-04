# Flight core (viết lại từ đầu) — `src/core`, namespace `fc`

## Vì sao viết lại
Core cũ (`wb::FlightController`) được viết trước khi có driver; sau khi có driver thật nó lệch ở nhiều chỗ:
| Vấn đề của core cũ | Core mới |
|---|---|
| Mixer có `idle`/`output_limit` riêng, trùng với `MotorOutputManager` (hai nơi sở hữu idle, dynamic idle không có đường vào) | Mixer chỉ nhận `MotorLimits{idle, limit}` **mỗi vòng** từ lớp motor; xuất giá trị **tuyệt đối** 0..1 nằm trong [idle, limit] |
| `FcInput.rpm` là con trỏ tùy chọn, cố định 4 motor | `MotorFeedback.rpm[8]` + `rpm_valid`; bộ lọc notch RPM tới 8 motor × 3 hài |
| Cảm biến gọi thẳng `kf.update_*` bất cứ lúc nào, không tuổi/độ tin cậy | `on_baro/on_range/on_gps/on_mag(giá trị, sigma, thời gian)`; estimator tự kiểm tra **độ cũ**, dùng sigma do driver đo |
| Chỉ có Kalman độ cao | + Kalman ngang (GPS vị trí/vận tốc, bù trễ GPS 150 ms) sẵn cho Position Hold / RTH |
| La bàn chỉ là hàm rời | La bàn tích hợp: seed yaw lúc disarm, chỉ sửa yaw, chỉ dùng khi `use_mag` + đã hiệu chuẩn + từ trường đúng cường độ |
| Arming/failsafe rải rác, không biết sức khỏe từng driver | `Health` lấy thẳng từ `valid()/usable()` của driver; `Arming` (cạnh lên của công tắc, công tắc bật lúc khởi động bị chặn, cần bật-tắt lại sau failsafe), blocker mới `AB_MAG_NOT_CALIBRATED` |
| Mixer gõ bảng tay | Hàng mixer **suy ra từ hình học** (góc motor, chiều quay): QUAD_X và QUAD_PLUS đúng **số motor và chiều quay mặc định của Betaflight** (props in: QUAD_X 1 sau-phải CW, 2 trước-phải CCW, 3 sau-trái CCW, 4 trước-trái CW), HEX_X; kiểm tra bằng plant vật lý độc lập. `props_out` = "motor direction reversed" của Betaflight |
| Mọi thứ chạy ở 4 kHz | Vòng nhanh: gyro → lọc → AHRS → rate PID → mixer; estimator chạy `est_hz` (500) trên accel trung bình |

## Luồng dữ liệu
```
IMUAccGyro::read ─┐                                         ┌─ MotorSystem::update(out.motor, out.armed)
Baro/Range/GPS/Mag ┼─ FlightCore::on_*  ─┐                  │
RC (1000..2000us) ─┴─ FlightCore::update(FastInput) ─► CoreOutput ─┘  (motor[8] tuyệt đối, armed, blockers, failsafe, estimate, debug)
MotorSystem::idleFraction / cfg().output_limit ──► FastInput.limits (vòng sau)
MotorSystem::manager().rpm_array ───────────────► FastInput.motors (lọc RPM)
```
`src/app/flight_app.h` (`FlightApp`) là lớp keo tương ứng, chỉ chuyển tiếp, không có logic bay (xem `docs/FIRMWARE.md`).

Các hàm thêm khi ghép firmware:
- `apply_tuning()` — app đổi PID / rate / Angle / Stabilize lúc disarm: đẩy `cfg` vào các bộ điều khiển mà **không** reset AHRS / estimator (`configure()` thì reset hết).
- `apply_filters()` — app đổi bộ lọc lúc disarm: dựng lại chuỗi lọc gyro và lọc D-term.
- `emergency_disarm()` — IMU mất hẳn: disarm, phải gạt lại công tắc arm.
- `pid()`, `gyro_filter()`, `setpoints()`, `angle_ctl()`, `stabilize()`, `althold()` — chỉ đọc: trạng thái đang chạy cho stream `tuning` và cho test.

`FastInput.rc_seq` đổi mỗi khi có **khung receiver mới**: feedforward và RC smoothing làm việc theo khung, không theo chu kỳ vòng lặp. `FastInput.motors.rpm[]` là tốc độ từng motor (0 = motor đó không có telemetry).

## Các file
`types.h` (quy ước dấu — đọc phần đầu file) · `filters.h` (PT1/2/3, BIQUAD và notch dạng state-variable) · `gyro_filter.h` (chuỗi lọc gyro, RPM filter) · `dyn_notch.h` (SDFT + dynamic notch) · `pid.h` (rate PID của Betaflight, throttle boost) · `setpoint.h` (tốc độ khung, RC smoothing, feedforward) · `attitude.h` (Angle của Betaflight, Stabilize của ArduPilot) · `simplified_tuning.h` (thanh trượt) · `inflight_trim.h` (level trim khi bay) · `ahrs.h` (Mahony + la bàn; gia tốc kế được tin tới đâu khi bay / khi nằm yên: `AccelGainSchedule`) · `vertical_est.h` (Kalman 4 trạng thái: z, v, bias accel, offset baro) · `horizontal_est.h` · `rc.h` (rates Actual, `RcInterpreter`, `ModeMap`) · `mixer.h` · `althold.h` (máy trạng thái 5 trạng thái kiểu ArduCopter, `HoverEstimator`) · `safety.h` (arm blockers, `Arming`, `Failsafe`, `RunawayTakeoff`, `LandDetector`) · `flight_core.h`.

Bộ ước lượng độ cao (`vertical_est.h`): barometer giữ `z`; lidar đo **so với một mốc mặt đất** khi có barometer (10 số đo liền ngoài cổng, hoặc số đo đầu tiên sau 0,5 s vắng: mốc dời, `z` không nhảy), và là độ cao tuyệt đối khi không có barometer; sigma của barometer kẹp trong 0,3 … 1,0 m, ×2 khi vừa arm / vừa rời đất (1,5 s). GPS (0.8.3, chi tiết ở `docs/GPS.md`, mục "Độ cao GPS"): vận tốc đứng của u-blox luôn được trộn vào `v` (đưa tới hiện tại bằng gia tốc đo trong 0,15 s trễ); độ cao GPS theo `nav.gps_alt` — `FALLBACK` (mặc định): là độ cao khi không có barometer lẫn lidar, `BLEND`: thêm vào đó sửa trôi barometer qua trạng thái offset `o` (so với một mốc lấy từ 20 số đo đầu, nên chỉ phần *thay đổi* của độ cao GPS có tác dụng), `OFF`: không dùng. Bước dự đoán dùng thời gian thật, mỗi mẫu gia tốc kế tính theo khoảng thời gian nó đại diện.

Chi tiết vòng rate, bộ lọc, hai mode tự cân bằng, ga treo: **`docs/TUNING.md`**.

## Chế độ
- **Acro**: cần → rate (Actual) → RC smoothing + feedforward → PID.
- **Angle** (Betaflight): cần cho góc nghiêng theo đường cong rate; `ModeMap` tự bật Angle khi bật Altitude Hold / Position Hold, và failsafe cũng ép Angle.
- **Stabilize** (ArduPilot): mục tiêu tư thế được định hình, giữ hướng mũi. Vào mode lúc đang bay thì bắt đầu từ tư thế hiện tại; đứng dưới đất với motor idle thì hướng mục tiêu bám theo máy.
- **Altitude Hold** (cần baro, lidar, hoặc — từ 0.8.3, `nav.gps_alt` khác `OFF` — độ cao GPS): ga treo tự đo, cất cánh bằng cách tăng ga dần tới khi rời đất. Mất độ cao trong chuyến bay → cờ `althold_unavailable`, cần ga được đọc **quanh ga treo đã đo** (giữa cần = treo), có độ cao lại thì tiếp tục từ mức ga đó. Kéo cần xuống sau khi rời đất = hạ xuống có lực. Ga kịch trần / đáy: bộ tích phân không dồn. Nghiêng quá 84°: giảm lực đẩy về 0.
- **Failsafe** (`safety.h`): có **nguyên nhân** (LINK / SWITCH / BATTERY, xuất ở `CoreOutput.fs_cause`). GUARD (0,5 s; hết nguyên nhân thì hết ngay; thời gian đã dùng chỉ "nguội" dần nên sóng chập chờn vẫn dẫn tới hạ cánh) → DESCENT (có độ cao: hạ có kiểm soát; không có: một phần của lực treo đã đo, và trong lúc đó không "học" ga treo) → LANDED. Trong DESCENT: sóng trở lại liên tục `fs_recover_ms` thì trả quyền; công tắc tắt là hết; pin thì không huỷ được nhưng người lái vẫn giữ roll / pitch / yaw. Đứng trên mặt đất khi hết GUARD: tắt motor luôn. Giới hạn `fs_max_descent_s` chỉ đếm khi máy không đi xuống hoặc không có độ cao. Trong cả GUARD lẫn DESCENT vì mất sóng, cần ga của kênh cũ **không** được Altitude Hold dùng.
- **Runaway lúc cất cánh** (`RunawayTakeoff`, port của Betaflight): PID sum ≥ 600 **cùng lúc** với tốc độ quay > 15 °/s (roll, pitch) hoặc 50 °/s (yaw) suốt 75 ms → disarm, blocker `RUNAWAY_TAKEOFF` hiện tới khi công tắc ARM về OFF. Hết hiệu lực (tới lần cấp nguồn sau) khi đã thấy 0,5 s bay bình thường: ga ≥ 20 % kèm một cần ≥ 15 %, hoặc ga ≥ 40 %, mọi PID sum < 100. Không xét khi motor đang ở idle mặt đất.
- **Mặt đất / trên không**: `airborne` = ga đã lên (25 % ngay, hoặc trên "thấp" 1 s), hoặc Altitude Hold đã nhấc máy; kết thúc khi `LandDetector` thấy máy đứng yên 1 s với ga ở thấp nhất. Ở **idle mặt đất** mọi motor bằng nhau, I-term = 0, runaway không xét.
- **CALIB**: level trim khi bay, đo từ gia tốc kế lúc treo (`inflight_trim.h`; `FlightApp` áp dụng sau khi disarm và nhờ link task ghi flash).
- **Airmode** (`general.airmode`): `AUTO` (mặc định) = ổn định đầy đủ ở ga thấp nhất **chỉ khi đang trên không**; `ALWAYS` = từ lần đầu ga vượt `airmode_start_throttle` tới khi disarm (Betaflight: trên bàn motor sẽ tự tăng tốc); `OFF`.
- Position Hold: công tắc có, **chưa có bộ điều khiển XY** (estimator ngang đã có).

Disarm xoá hết trạng thái của PID, Angle và throttle boost; low-throttle TPA được nạp lại.

## Đã kiểm chứng (trên PC)
- `test/test_core.cpp` (296): AHRS, estimator dọc (kể cả mốc lidar, sigma barometer) / ngang, RC, mixer so với vật lý (3 layout), failsafe (nguyên nhân, trả quyền, giới hạn), runaway, arming, Altitude Hold (máy trạng thái, `HoverEstimator`, cất cánh, bỏ cuộc, không có người cầm cần, ga kịch giới hạn, lật ngửa).
- `test/test_control.cpp` (269): từng khối của vòng rate đối chiếu **với công thức của nó**: bộ lọc, chuỗi lọc gyro, RPM filter, dynamic notch, PID và các phụ trợ, đường cần (kể cả khung bị mất), Angle, Stabilize, thanh trượt, level trim khi bay.
- `test/test_nav.cpp` (130, 0.7.0): optical flow → vận tốc, góc nghiêng chỉnh theo gia tốc đo được (`tilt_aid.h`), bộ điều khiển vị trí, Position Hold, GPS Rescue — từng khối với mô hình vật lý viết riêng trong test; xem `docs/NAV.md`.
- `test/test_flight.cpp` (251): toàn bộ `FlightCore` với plant 6-DOF độc lập trên hai máy (5 inch lực đẩy lớn; 1 kg lực đẩy yếu): dấu roll / pitch / yaw, bước rate, feedforward, bộ lọc trong vòng kín, throttle boost, Angle so với Stabilize, chuyển mode, airmode và idle mặt đất, la bàn, Altitude Hold (mất độ cao, cái bàn dưới lidar, GPS lệch 3 m, gió cánh quạt sát đất), failsafe, **chu kỳ không đều**, **runaway với hai dây motor cắm nhầm**, bàn giao cho `MotorOutputManager`; từ 0.7.0 cả Position Hold (GPS + la bàn, flow + lidar), GPS Rescue và góc nghiêng chỉnh theo vận tốc đo được.
- Lỗi cố ý (mutation): hơn 660 lỗi cho riêng các file của vòng rate / bộ lọc / mode / Altitude Hold / an toàn (`tools/mutate.py`).
- `test/test_fw.cpp`: core chạy **bên trong firmware thật** trên mạch ảo: dữ liệu đi qua driver IMU (SPI), baro (I2C), decoder CRSF, khung DShot (kể cả ESC trả lời rpm) rồi mới tới plant.

## Chưa kiểm chứng / giới hạn
- Bản 0.5 đã bay thử trên bàn (không cánh) trên mạch thật; những gì lộ ra và cách sửa: `docs/HARDWARE_FEEDBACK_1.md`. **Thời gian một lần `update()` trên S3 của bản này vẫn chưa có số** (tab Tổng quan nay hiện riêng khâu "tính toán"). Cách giảm tải: `docs/TUNING.md` mục 9.
- Những khác biệt với Betaflight / ArduPilot đã biết mà chưa làm: `docs/HARDWARE_FEEDBACK_1.md` mục 6, phần "Biết nhưng chưa làm".
- Mặc định là bộ số của Betaflight cho quad 5 inch; máy khác phải tune (Master Multiplier).
- Plant mô phỏng: lực đẩy tỉ lệ thẳng với lệnh; chuyển động ngang có lực cản tuyến tính 0,35 /s; gia tốc kế **đúng vật lý** (lực đẩy + lực cản, không có trọng lực khi bay). Vài test về hình dạng đáp ứng của bộ điều khiển và về blackbox dùng "cảm biến nghiêng lý tưởng" (`true_accel = false`) và ghi rõ lý do. Không thay thế việc tune trên máy thật.
- Bộ ước lượng tư thế với gyro + gia tốc kế lệch vài độ khi tăng tốc kéo dài (số đo ở `docs/TUNING.md` mục 5). Từ 0.7.0, khi có vận tốc đo được (GPS + la bàn, hoặc optical flow) và máy đang arm, `core/tilt_aid.h` thay gia tốc kế bằng phép so lực riêng với gia tốc thật (cách của ArduPilot DCM / PX4): mô phỏng cho dưới 1° trong cả lần nghiêng 25° kéo dài 4 giây. Không có GPS / flow thì vẫn như cũ.
- Chưa có: bù điện áp pin cho throttle, crash-flip, thrust linearisation, I-term rotation, waypoint / mission. Position Hold, GPS Rescue, optical flow: từ 0.7.0 (`docs/NAV.md`), **chưa bay thật**.

## Lỗi đã sửa (0.6.0): bộ ước lượng tư thế tin gia tốc kế quá nhiều
Tới 0.5.0, gain gia tốc kế của AHRS là 2 /s (hằng số thời gian 0,5 s) và plant mô phỏng cho gia tốc kế luôn chỉ đúng phương thẳng đứng. Trên máy thật điều đó sai: khi bay, gia tốc kế cảm nhận lực đẩy chứ không phải trọng lực. Khi plant được sửa cho đúng vật lý (để làm Position Hold), lỗi lộ ra ngay: giữ nửa cần roll 2 giây trong Angle, lệnh 12°, máy nghiêng thật 27° và lắc ngược 14° sau khi thả cần.

Đã sửa trong `core/ahrs.h` theo cách Betaflight làm (`imu.c`), có đối chiếu ArduPilot: gain 0,25 /s khi arm, × 10 khi disarm nằm yên, × 100 nửa giây sau khi vừa nằm yên; lọc thông thấp bậc 2 ở 10 Hz; trọng số giảm về 0 khi độ lớn lệch 0,1 g so với lúc nằm yên (học khi disarm); bias gyro học không quá 0,5 °/s mỗi phút. Level trim khi bay chuyển sang đo thẳng từ gia tốc kế. Bảng số và hệ quả cho người bay: `docs/TUNING.md` mục 5 và 7. Test: `t_ahrs_trust` (`test_core.cpp`), `t_estimator_in_flight` (`test_flight.cpp`), `t_inflight_trim` (`test_fw.cpp`).

Bài học về cách thử: mô phỏng "lý tưởng hoá" một cảm biến sẽ che đúng những lỗi nằm ở chỗ dùng cảm biến đó. Plant bây giờ mặc định dùng gia tốc kế đúng vật lý.

## Lỗi đã sửa: chiều quay mặc định của quad bị ngược so với Betaflight
Bản trước ghi "đúng số motor BF" nhưng bảng chiều quay của QUAD_X / QUAD_PLUS lại là bản đảo (`{+1,-1,-1,+1}`): với `props_out = false` firmware coi motor 1 (sau-phải) quay CCW, tức là kiểu **props out** của Betaflight. Ai lắp cánh theo hình mặc định của Betaflight thì vòng yaw thành hồi tiếp dương (máy xoay tít khi cất cánh). Nguyên nhân: Betaflight đổi dấu yaw hai lần (cần yaw trong `rc.c`, và `-scaledAxisPidYaw` trong `mixer.c`), lần đọc đầu chỉ tính một. Test cũ không bắt được vì plant mô phỏng dùng cùng bảng sai đó (đúng loại lỗi "simulator và code cùng hiểu sai").
Nay: `quad_x_spin = quad_p_spin = {-1,+1,+1,-1}` (+1 = cánh CCW), bảng mixer **bằng đúng từng số** `mixerQuadX` / `mixerQuadP` của Betaflight (có test so từng hệ số), plant mô phỏng là một quad lắp kiểu Betaflight. HEX_X vốn đã đúng chiều theo vị trí, không đổi.
**Vẫn phải kiểm tra trên mạch** (tháo cánh): tab Motor của GUI vẽ chiều mỗi motor phải quay; chạy từng motor và so.
