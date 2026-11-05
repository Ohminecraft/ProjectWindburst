# Các mode gán công tắc (0.23), arm bằng cần (0.24)

Bốn mode mới trong tab **Mode**, thêm vào cuối enum `mode` (số cũ không đổi): `ACRO_TRAINER` (10), `AIRMODE` (11), `ANTI_GRAVITY` (12), `BLACKBOX_ERASE` (13). Trên MSP, firmware báo chúng bằng box id cố định của Betaflight: ACRO TRAINER 47, AIR MODE 28, ANTI GRAVITY 4, BLACKBOX ERASE 31. Kính hoặc tay điều khiển đọc được mode nào đang bật.

## Acro trainer (`ACRO_TRAINER`)
Giống `applyAcroTrainer()` trong `flight/pid.c` của Betaflight 4.5 (`src/core/acro_trainer.h`).
- **Khi nào chạy:** chỉ trong Acro. Angle, Stabilize, các mode giữ (hold) và GPS Rescue đều đã tự giới hạn góc, nên trainer không làm gì ở đó.
- **Cần vẫn điều khiển tốc độ quay như Acro.** Trainer chỉ thay setpoint roll / pitch khi máy sắp vượt góc giới hạn:
  - Đã vượt góc mà cần vẫn đẩy về phía đó: setpoint = (giới hạn − góc) × gain. Máy được giữ ở giới hạn tới khi cần đổi chiều.
  - Sắp vượt (góc + tốc độ quay × thời gian nhìn trước): setpoint = (giới hạn − góc dự đoán) × gain. Thời gian nhìn trước tăng theo tốc độ quay: 0 khi đứng yên, đủ `lookahead` khi quay từ 500 °/s.
  - Mỗi lần trainer giành quyền, I-term của trục đó về 0.
  - Setpoint sửa góc giới hạn ở ±1000 °/s.
- **Cài đặt** (tab PID, khối "Acro trainer"; section `pid`, có hiệu lực ngay):

  | Trường | Mặc định | Khoảng | Ý nghĩa |
  |---|---|---|---|
  | `acro_trainer_angle_limit` | 20 ° | 10 – 80 | góc nghiêng tối đa |
  | `acro_trainer_lookahead_ms` | 50 ms | 10 – 200 | thời gian nhìn trước |
  | `acro_trainer_gain` | 75 | 25 – 255 | độ mạnh (Betaflight: chia 10) |

- **Trên máy mô phỏng** (`test_flight`): giữ cần roll hết cỡ 1,5 s thì máy dừng ở khoảng 20 °. Vượt nhiều nhất khoảng 34 °, vì cần hết cỡ là 670 °/s. Không có trainer, máy lật qua 170 °.

## Airmode bằng công tắc (`AIRMODE`)
- **Có gán công tắc:** công tắc quyết định airmode.
  - Công tắc tắt: không có airmode. Ga về min thì mọi motor về idle, dù `general.airmode` đặt gì.
  - Công tắc bật: airmode chạy theo kiểu `general.airmode` đặt (`AUTO` hoặc `ALWAYS`). Nếu `general.airmode` là `OFF` thì dùng `AUTO`.
- **Không gán công tắc:** như trước, chỉ `general.airmode` quyết định.
- **Khác Betaflight:** ở Betaflight, box AIR MODE chỉ có tác dụng khi tắt feature AIRMODE (bật hoặc là đủ). Ở đây công tắc đã gán là thắng, nên không cần đổi cài đặt nào khác.

## Anti Gravity bằng công tắc (`ANTI_GRAVITY`)
- **Có gán công tắc:** Anti Gravity chỉ chạy khi công tắc bật, bất kể `pid.anti_gravity` đặt gì. Các số gain / cutoff vẫn lấy từ section `pid`.
- **Không gán công tắc:** `pid.anti_gravity` quyết định, như trước.

## Xoá blackbox bằng công tắc (`BLACKBOX_ERASE`)
Giống box BLACKBOX ERASE của Betaflight.
- **Cách xoá:** gạt công tắc từ tắt sang bật khi chưa arm. Firmware xoá như nút "Xoá" trong tab Blackbox: xoá flash, hoặc xoá các file log trên thẻ SD. Xoá xong thì còi kêu tiếng `BLACKBOX_ERASE`. Trong lúc xoá không arm được.
- **Chỉ một lần gạt mới tính:**
  - Công tắc đang bật sẵn lúc khởi động thì không xoá.
  - Giữ công tắc bật thì chỉ xoá một lần.
  - Gạt lúc đang arm thì bị bỏ qua.
- **Lần gạt bị từ chối thì không thử lại:** ví dụ đang ghi log trên mặt đất (kiểu ghi `ALWAYS`) hoặc không có bộ nhớ log. Muốn xoá thì gạt lại.
- Mã nguồn: `Service::poll()` trong `src/blackbox/bb_service.h`, dùng `Inputs::erase_switch`.

## Kiểu bay: FPV và Flycam (`general.fly_style`, 0.43)
Tab **Mode**, khối "Kiểu bay". Cần lưu và khởi động lại.

- **FPV** (mặc định): như cũ, mọi mode theo công tắc, kể cả Acro.
- **Flycam**: cho người quay phim, cách bay giống máy DJI.
  - Luôn bật **Angle + Altitude Hold + giữ vị trí**, tức `flycam_hold`: **Loiter** (mặc định, cần = vận tốc, buông là tự phanh rồi đứng, như chế độ P của DJI) hoặc **Position Hold**. Công tắc Acro, Stabilize, Acro Trainer bị bỏ qua. Công tắc GPS Rescue, Nhiệm vụ, Crash flip, Failsafe vẫn chạy.
  - **`flycam_attitude`** (0.44): bộ điều khiển tư thế mà tất cả những mode đó bay trên - **Angle** (Betaflight, mặc định, `angle_strength`) hoặc **Stabilize** (ArduPilot, `stab_p`, target được tạo hình bởi `stab_input_tc` và giới hạn gia tốc, như Loiter / Altitude Hold của ArduCopter). Khi chọn Stabilize, các mode giữ, failsafe, rescue và mission đều lái qua nó, công tắc Angle bị bỏ qua, và autotune tune thêm Stabilize P. Position Hold dừng chậm hơn một chút trên Stabilize (còn 0,3 m/s sau 4 s buông cần khi đang bay 3,4 m/s, hết hẳn sau 7 s; Loiter như trên Angle). FPV không bị ảnh hưởng: các mode giữ vẫn trên Angle.
  - **Cần ga ở giữa = giữ độ cao.** Arm được khi cần ga **không cao hơn giữa** (ở giữa hoặc ở đáy, nên arm bằng cần kiểu DJI vẫn dùng được). Cần ga cao hơn giữa thì chặn arm (`THROTTLE_HIGH`).
  - **Cất cánh:** từ giữa đẩy lên là cất cánh, không phải kéo xuống đáy trước như ở FPV. Ở FPV vẫn phải kéo xuống đáy trước, để máy không tự bay lên khi lỡ arm với cần ga đang cao. Nếu lúc arm hoặc lúc vừa đáp cần ga đang cao hơn giữa, thì phải về giữa trước rồi mới cất cánh được.
  - **Hạ cánh:** kéo cần xuống thì máy hạ. Đáp rồi mà giữ cần ở đáy 3 giây thì motor tắt (DJI). Muốn bay lại thì gạt ARM về OFF rồi bật lại.
  - **Không có GPS / optical flow:** vẫn arm được, chỉ giữ độ cao (như ATTI của DJI). App hiện "giữ vị trí: không có gì để giữ". **Không có baro thì không arm** (`ALTHOLD_NO_BARO`).
- **Mã:** `CoreConfig::flycam` / `flycam_loiter` trong `src/core/flight_core.h`, `AltConfig::centred_stick` trong `src/core/nav/alt_control.h`.
- **Kiểm chứng:**
  - `test_flight` nhóm flycam: chế độ bị ép, arm ở giữa, đẩy lên cất cánh, giữ độ cao và vị trí, đáp rồi tắt motor sau 3 s, Position Hold, không có flow, không có baro, FPV giữ nguyên;
  - `test_fw` nhóm `flycam`;
  - `test_proto` (section `general` 59 byte; giữ được 57, 56, 24, 22; bỏ 23 byte của 0.9.0 – 0.9.4);
  - app `test_fly_style_setting`;
  - 12 mutant `flycam:`.
  - **Chưa bay thử.**

## Crash flip (`CRASHFLIP`, 0.42)
Turtle mode của Betaflight, viết theo mã thật: `mixer.c` (`applyCrashFlipModeToMotors`) và `core.c` (`tryArm`, `updateArmingStatus`, `disarm`). Máy rơi ngửa thì lật lại bằng cần, không phải đi nhặt.

**Cách dùng:** gán công tắc CRASHFLIP ở tab **Mode**. Máy nằm ngửa: bật CRASHFLIP, rồi arm. Motor quay ngược chiều, mỗi cần chọn motor chạy:

| Cần | Motor chạy (Quad X, cánh "props in") |
|---|---|
| pitch tới | hai motor trước |
| pitch lùi | hai motor sau |
| roll phải / trái | hai motor bên phải / trái |
| gần đường chéo (trong 30°) | một motor ở góc đó |
| yaw (khi lớn nhất trong ba cần) | một cặp chéo: yaw phải là hai cánh quay theo chiều kim đồng hồ (props out: cặp kia). Dùng khi một cánh bị kẹt |

- Đẩy cần càng xa thì motor càng mạnh (độ dài của roll + pitch). Trong 15 % quanh giữa: không motor nào chạy.
- Motor không được chọn **đứng yên**, không chạy idle. Lệnh dưới 2 % hành trình cũng là đứng yên.
- Gạt CRASHFLIP về tắt: máy disarm, lý do `CRASHFLIP` hiện cho tới khi gạt ARM về OFF. Gạt ARM OFF rồi ON để bay như thường.
- Công tắc chỉ được xét **lúc arm**. Đang bay mà bật CRASHFLIP thì không có gì xảy ra.

**Cài đặt** (tab **Motor**, khối "Crash flip"; cần lưu và khởi động lại), tên và mặc định như Betaflight:

| Cài đặt | Betaflight | Mặc định | Ý nghĩa |
|---|---|---|---|
| `motors.crashflip_rate` | `crashflip_rate` (x10) | 0 | °/s. Máy xoay nhanh hơn nửa mức này thì lực giảm dần, tới mức này thì motor dừng. Máy đổi tư thế quá cạnh (cos góc nghiêng đổi hơn 1) thì motor cũng dừng: đã lật xong. 0 = không giảm. Nên thử 350 |
| `motors.crashflip_motor_pct` | `crashflip_motor_percent` | 0 | các motor phía bên kia chạy bằng % này của lực. 0 = đứng yên |
| `motors.crashflip_auto_rearm` | `crashflip_auto_rearm` | tắt | bật: gạt CRASHFLIP về tắt thì máy vẫn arm và bay tiếp (motor đứng khoảng 1 s để đổi lại chiều quay) |

**Đổi chiều quay:** lúc arm vào crash flip, mọi ESC nhận lệnh DShot 21 (quay ngược so với chiều đã lưu trong ESC, không lưu lại), 10 lần như Betaflight. Motor chỉ chạy sau lệnh cuối cùng. Hết crash flip (disarm, hoặc tắt công tắc khi có auto re-arm): lệnh DShot 20 (trả lại chiều đã lưu), gửi sau khi motor đã quay chậm lại (1 s kể từ khung cuối có lệnh quay). Lệnh 7 / 8 + 12 của nút "Đảo chiều" ở tab Motor không bị đụng tới.

**Điều kiện arm:**
- Có công tắc CRASHFLIP bật: không chặn vì nằm nghiêng / ngửa (`ANGLE_TOO_HIGH`), không chặn vì GPS Rescue thiếu fix (`RESCUE_NO_FIX`), như Betaflight. Các điều kiện khác giữ nguyên (ga thấp, gyro đã hiệu chuẩn, có sóng...).
- ESC không phải DShot (PWM, Oneshot, Multishot, DroneCAN): bật CRASHFLIP thì **chặn arm** (`CRASHFLIP`). Betaflight trong trường hợp này vẫn bỏ qua góc nghiêng và arm với motor quay xuôi chiều. Ở đây chặn luôn cho an toàn.

**Khi đang crash flip:**
- Mất sóng: cần coi như ở giữa nên motor đứng. Failsafe làm như cũ.
- Không chạy PID, Altitude Hold, runaway takeoff, phát hiện motor dừng. Motor kẹt trong cỏ là chuyện bình thường lúc lật.
- OSD và CRSF ghi `FLIP`. Cờ `CRASHFLIP` có trong `nav.flags`, `fly_mode` của tay phát DIY là `CRASHFLIP`.

**Khác Betaflight:**
- Chặn arm khi không có DShot (xem trên).
- Lệnh 20 khi kết thúc chờ motor quay chậm lại rồi mới gửi. Trong lúc đó motor đứng. Betaflight bỏ lệnh nếu motor chưa ở 0, nên auto re-arm có thể để motor quay ngược.
- Arm bình thường (không crash flip) **không** gửi lệnh 20. Betaflight gửi 20 ở mọi lần arm.
- Không có tiếng beeper riêng cho crash flip.
- Cần đi thẳng, không qua đường cong rate. Betaflight cũng đọc độ lệch cần như vậy.

**Mã nguồn:**
- `src/core/crash_flip.h` (lệnh motor);
- `FlightCore` (lúc nào bật / tắt);
- `MotorOutputManager::update(..., flip)` (DShot 21 / 20, motor đứng).

**Kiểm chứng:**
- `test_core` (các giá trị tính tay theo công thức Betaflight cho Quad X, hexa);
- `test_motor` (21 x10 trước khi bay, motor 0 đứng yên, 20 sau khi quay chậm, auto re-arm, arm lại trong lúc chờ, giao thức xung);
- `test_flight` (arm khi ngửa, công tắc, auto re-arm, mất sóng, tốc độ xoay và tư thế qua AHRS thật);
- `test_fw` nhóm `crashflip` (ESC ảo nghe 21 / 20, cần, cài đặt % và tốc độ, Oneshot125 bị chặn);
- `test_osd`, `test_rx`, `test_radio`, `test_proto`, app (`test_crash_flip_on_the_page`);
- 36 mutant `crashflip:`.
- **Chưa thử trên máy thật.** Cũng chưa biết ESC nào nhận lệnh 20 / 21: BLHeli_32, AM32, Bluejay thì có; BLHeli_S thì cần bản 16.7 trở lên.

## 3D: motor quay hai chiều (`motors.mode3d`, công tắc `3D`, 0.44)
Feature 3D của Betaflight, viết theo code thật của nó (`rc.c`, `rc_controls.c`, `mixer.c`, `dshot.c`, `pwm_output.c`). ESC quay được cả hai chiều; cần ga chia đôi ở giữa.

**Cách dùng:**
- Tab **Motor**, khối "Mode 3D": bật **Mode 3D**. Cần lưu và khởi động lại.
- ESC cũng phải được cài 3D (BLHeli / Bluejay / AM32: bật 3D mode).
- Không dùng được với ESC DroneCAN hay brushed (setting tự tắt).

| Cần ga | Motor |
|---|---|
| trên giữa + deadband | quay thuận, càng lên càng nhanh |
| giữa +- `receiver.deadband3d_throttle` (us, tab Receiver, mặc định 50) | dừng; đây là "ga thấp" khi arm |
| dưới giữa - deadband | quay ngược, càng xuống càng nhanh |

- Về lại trong deadband: motor giữ chiều đang quay, ở mức 0.
- Disarm xong, lần arm sau bắt đầu quay thuận.
- **Arm:** cần ga phải nằm trong deadband. Betaflight cho máy 3D arm ở mọi mức ga; ở đây motor không được khởi động khi đang có tốc độ.
- **Công tắc 3D** (tab Chế độ, box 29 "3D DISABLE" của Betaflight):
  - **3D theo công tắc** (`motors.switched_mode3d`) tắt: bật công tắc là tắt 3D, cần ga như thường (dưới cùng là dừng, chỉ quay thuận).
  - 3D theo công tắc bật (`3d_switched_mode` của Betaflight): cần ga như thường, công tắc chọn chiều, bật = quay ngược.
- **Đổi chiều khi đang bay:**
  - lệnh roll / pitch / yaw đổi dấu (lực đẩy hướng ngược lại);
  - các thành phần I giữ ở 0 trong 250 ms, như Betaflight.

**Setting** (tên như Betaflight):

| Setting | Betaflight | Mặc định | Ý nghĩa |
|---|---|---|---|
| `motors.mode3d` | feature 3D | tắt | bật 3D |
| `motors.switched_mode3d` | `3d_switched_mode` | tắt | công tắc 3D đổi chiều thay vì tắt 3D |
| `receiver.deadband3d_throttle` | `3d_deadband_throttle` | 50 us | khoảng dừng quanh giữa cần ga |
| `motors.deadband3d_low` | `3d_deadband_low` | 1406 us | chỉ với PWM / Oneshot / Multishot: xung bắt đầu quay ngược |
| `motors.deadband3d_high` | `3d_deadband_high` | 1514 us | ... xung bắt đầu quay thuận |
| `motors.neutral3d` | `3d_neutral` | 1460 us | ... xung dừng (disarm, đang dừng) |
| `motors.limit3d_low` / `limit3d_high` | `3d_limit_low` / `3d_limit_high` | 1000 / 2000 us | ... ngược hết cỡ / thuận hết cỡ |

- DShot không cần các xung trên: 48..1047 là quay ngược, 1048..2047 quay thuận.
- Thử motor ở tab Motor cho motor quay thuận.

**3D tắt những gì:**
- Chỉ bay Acro, Angle và Stabilize. Altitude / Position Hold, Loiter, GPS Rescue, Nhiệm vụ, Autotune và crash flip bị chặn: lực đẩy có thể hướng cả hai phía nên không giữ được độ cao và vị trí.
- Failsafe tắt motor (disarm), như Betaflight làm với 3D. Nhiệm vụ đang chạy thì dừng.
- Runaway lúc cất cánh, dynamic idle và Flycam tắt.

**Code:** `src/core/throttle3d.h` (cần ga), `FlightCore` (mode, failsafe, dấu của mixer, thành phần I), `MotorOutputManager` (`to_value3d`).

**Kiểm chứng:**
- `test_mode3d` (chia cần ga, chiều trong deadband, theo công tắc / công tắc, giữ I, giá trị DShot và PWM);
- `test_flight` (bay thuận và ngược, failsafe disarm, mode bị chặn);
- `test_fw` nhóm `mode3d` (giá trị DShot trên dây ESC ảo; DroneCAN thì 3D tắt);
- `test_proto`, `test_msp_config` (feature bit 12, MOTOR_3D_CONFIG, RC_DEADBAND, box 29), app;
- 23 mutant `m3d:`.
- **Chưa thử trên máy thật** hay với ESC 3D thật.

## Arm bằng cần (`general.stick_arm`, 0.24)
Bật ở tab **Mode**, ô "Arm bằng cần". Cần lưu và khởi động lại. Mặc định tắt.
- **Động tác:** kéo cần ga và cần pitch xuống hết, đồng thời kéo cần yaw và cần roll sang hai phía ngược nhau (yaw trái + roll phải, hoặc yaw phải + roll trái). Với tay điều khiển mode 2, đó là hai cần cùng xuống dưới và cùng hướng ra ngoài, hoặc cùng hướng vào trong (giống "combination stick" của DJI). Mỗi cần phải gạt quá 80 % hành trình.
- **Arm:** giữ động tác 1 giây khi chưa arm. Các điều kiện chặn arm vẫn áp dụng như với công tắc.
- **Disarm:** giữ đúng động tác đó 1 giây khi máy đang nằm trên đất. Khi đang bay thì không bao giờ disarm, vì phi công có thể gạt cần đúng như vậy lúc bay.
- **Phải thả cần giữa hai lần,** và cả sau khi cấp nguồn: giữ sẵn động tác lúc bật máy thì không arm.
- **Có gán công tắc ARM:** công tắc thành nút cho phép và ngắt khẩn cấp.
  - Công tắc phải bật thì động tác mới arm được.
  - Gạt tắt là disarm ngay.
  - Gạt công tắc bật lại thì không tự arm; phải làm lại động tác.
- **Không gán công tắc ARM:** chỉ dùng động tác để arm và disarm. Nên gán công tắc ARM để có cách tắt motor ngay lập tức.
- **Bị chặn arm:** công tắc ảo (do động tác bật) tự về tắt khi thả cần, nên không còn báo "công tắc ARM bật sẵn". Lý do bị buộc disarm (runaway, motor dừng) vẫn hiện cho tới khi bắt đầu động tác kế tiếp.
- **Mất sóng:** động tác không có tác dụng gì. Lúc đó failsafe xử lý như cũ.
- Báo cáo khởi động: `arming: sticks: throttle + pitch down, yaw and roll to opposite sides, 1 s (disarm: the same, on the ground)`, thêm `; the ARM switch must be on` nếu có gán công tắc ARM.
- Mã nguồn: `StickArm` trong `src/core/safety.h`. Nó tạo một công tắc ảo đưa vào `Arming`.

## Kiểm chứng
- `test_control` nhóm acro trainer: từng nhánh so với code Betaflight (giữ, nhìn trước, đổi chiều cần, dấu của 0, giới hạn 1000 °/s).
- `test_flight` nhóm switch modes: trainer khi bay (roll, pitch, đổi chiều cần, không chạy trong Angle, giới hạn 45 °), airmode và anti-gravity theo công tắc.
- `test_blackbox`: công tắc xoá (bật sẵn lúc khởi động, gạt, giữ, lúc arm, lúc đang ghi).
- `test_fw` nhóm `switch`: bảng mode, `status.modes`, cài đặt trainer có hiệu lực ngay, xoá blackbox bằng công tắc trên mạch ảo.
- `test_esc4way`: BOXIDS / STATUS kiểu Betaflight.
- `test_proto`: section `pid` cũ (75 byte) vẫn đọc được.
- Arm bằng cần: `test_flight` nhóm stick arming (động tác, giữ 1 s, thả cần, giữ sẵn lúc cấp nguồn, đang bay, bị chặn, công tắc ARM làm nút ngắt), `test_core` (giữ lý do disarm), `test_fw` nhóm `stickarm`, `test_proto` (section general cũ), 12 mutant `stickarm:`, `test_gui_browser::test_stick_arm_setting` (chưa chạy được ở máy này).
- 17 mutant `modes:`. `test_gui_browser::test_switch_modes` đã viết nhưng chưa chạy được ở máy này.
- **Chưa thử trên máy thật.**
