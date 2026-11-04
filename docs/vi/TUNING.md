# Tune: bộ lọc, PID, Angle / Stabilize, Altitude Hold

Vòng trong (rate) và chuỗi lọc là bản port của **Betaflight 4.5** (`flight/pid.c`, `sensors/gyro_filter_impl.c`, `flight/dyn_notch_filter.c`, `fc/rc.c`, `config/simplified_tuning.c`): cùng tên, cùng đơn vị, cùng mặc định, nên số từ một bản tune Betaflight dùng được thẳng và kinh nghiệm tune Betaflight áp dụng được nguyên. Stabilize là bản port của **ArduPilot** (`AC_AttitudeControl`, `AP_Math/control.cpp`).

Mọi thứ ở đây mới được kiểm chứng **trên PC** (mô phỏng). Chưa có số nào đo trên máy thật: xem mục cuối.

## 1. Tín hiệu đi qua những gì

```
gyro thô ─ lowpass 2 ─ RPM notch ─ notch tĩnh 1, 2 ─ lowpass 1 (tĩnh / động theo ga) ─ dynamic notch ─┬─► P, I  (sai lệch = setpoint − gyro)
                                                                                                      └─ D-term notch ─ D lowpass 1 (tĩnh / động) ─ D lowpass 2 ─► D
cần ─ rate (Actual) ─┬─ RC smoothing (PT3, cắt theo tốc độ khung đo được) ─► setpoint
                     └─ feedforward (tốc độ của setpoint, tính theo từng khung) ─ RC smoothing ─► F
P + I + D + F  ─ giới hạn ±500 (roll, pitch) / ±400 (yaw) ─ /1000 ─► mixer
```

| Khối | File | Ghi chú |
|---|---|---|
| lowpass, notch | `core/filters.h` | PT1 / PT2 / PT3 (cắt −3 dB đúng tại tần số nhập) và BIQUAD (Butterworth bậc 2, dạng state-variable nên đổi tần số lúc đang chạy không giật) |
| chuỗi lọc gyro | `core/gyro_filter.h` | đúng thứ tự của Betaflight |
| RPM filter | `core/gyro_filter.h` | 1 notch cho mỗi motor × mỗi hài (tối đa 3), đặt đúng tần số ESC báo về. **Cần DShot hai chiều.** Làm việc theo từng motor: ESC nào mất telemetry thì notch của riêng motor đó tắt dần |
| dynamic notch | `core/dyn_notch.h` | SDFT 72 mẫu → 36 bin, tìm tối đa 7 đỉnh mỗi trục, mỗi đỉnh một notch. Cần vòng lặp từ 2000 Hz |
| PID | `core/pid.h` | I-term relax, anti-gravity, D max, TPA (cả phần ga thấp), yaw lowpass |
| cần → setpoint | `core/setpoint.h` | đo tốc độ khung receiver, RC smoothing, feedforward (jitter, smooth, boost, max rate limit, averaging, yaw hold) |
| Angle, Stabilize | `core/attitude.h` | |
| thanh trượt | `core/simplified_tuning.h`, `proto/settings_rules.h` | firmware tính, app chỉ hiển thị kết quả |

Hệ số: `Kp = 0,032029 × P`, `Ki = 0,244381 × I` (yaw × 2,5), `Kd = 0,000529 × D`, `Kf = 0,013754 × F / 100`. PID sum 1000 = toàn bộ dải motor.

## 2. Mặc định dành cho máy nào

Bộ số mặc định (P 45/47/45, I 80/84/80, D 30/34/0, D max 40/46/0, F 120/125/120; lowpass gyro 250–500 Hz, D-term 75–150 Hz) là của Betaflight cho **quad 5 inch lực đẩy lớn** (lực đẩy gấp ~7 lần trọng lượng). Trong mô phỏng:

| Máy mô phỏng | Bước roll hết cần trong 60 ms | Ghi chú |
|---|---|---|
| 5 inch, 650 g, 1,2 kg lực đẩy mỗi motor | vọt lố ~4 %, đạt 90 % trong < 110 ms | đúng đối tượng của bộ số |
| 1 kg, lực đẩy chỉ gấp 2 trọng lượng | vọt lố ~27 % | bay được nhưng mềm: motor cần 140 ms chạy hết vi sai mới tới 670 °/s |
| như trên, **Master Multiplier 2,0** | vọt lố ~17 % | |

Máy nặng hoặc lực đẩy yếu: tăng **Master Multiplier** từng nấc 0,1 (thường dừng ở 1,3 – 2,0). Sau mỗi lần đổi: bay thử 30 giây rồi **sờ motor**. Motor nóng = D quá cao hoặc lọc quá ít.

## 3. Thanh trượt (tab PID)

100 = 1,00 = mặc định; dải 0 – 2,00. Tích được **cắt** thành số nguyên đúng như mã C của Betaflight (ví dụ pitch D max ở Dynamic Damping 2,0 là 57, không phải 58).

| Thanh trượt | Field | Tác dụng lên |
|---|---|---|
| Damping | `slider_d` | D và D max mọi trục |
| Tracking | `slider_pi` | P và I mọi trục |
| Stick Response | `slider_ff` | feedforward |
| Dynamic Damping | `slider_d_max` | khoảng cách D max so với D (0: D max = D, không còn boost) |
| Drift – Wobble | `slider_i` | chỉ I |
| Pitch Damping | `slider_pitch_d` | D, D max của pitch so với roll |
| Pitch Tracking | `slider_pitch_pi` | P, I, feedforward của pitch so với roll |
| Master Multiplier | `slider_master` | tất cả |

`slider_mode`: `RPY` (mặc định, cả ba trục do thanh trượt tính), `RP` (yaw nhập tay), `OFF` (mọi ô nhập tay). Trục nào do thanh trượt tính thì **firmware ghi đè** P/I/D/D max/F của trục đó mỗi khi nhận hoặc nạp section `pid`, và app đọc lại đúng số đang có hiệu lực (ô bị khoá).

Bộ lọc có hai thanh trượt riêng (tab Bộ lọc): `slider_gyro_mult` và `slider_dterm_mult` (0,10 – 2,00) nhân các tần số cắt lowpass; lọc nào đang tắt (0 Hz) thì vẫn tắt; kết quả không vượt 1000 Hz. Notch, dynamic notch, RPM filter không thuộc thanh trượt.

## 4. Bộ lọc

- **Lowpass 1 động**: tần số cắt chạy từ `dyn_min` (ga 0) lên `dyn_max` (ga đầy) theo đường cong `dyn_expo`; cập nhật nhiều nhất mỗi 5 ms và chỉ khi ga đổi 1 %. `dyn_min = 0` → dùng tần số tĩnh.
- **RPM filter**: không có DShot hai chiều thì không chạy (tab Bộ lọc ghi rõ). `rpm_min_hz` (100): notch không xuống dưới mức này và mờ dần trong `rpm_fade_range_hz` (50) phía trên nó. Trọng số từng hài 0 – 100 %: cánh 2 lá thường chỉ cần hài 1 và 2.
- **Dynamic notch**: `count` 0 = tắt. Một đỉnh phải cao gấp đôi nền quanh nó mới được bám; notch được **lái** tới đỉnh (PT1 4 Hz, nhanh gấp tới 10 lần khi đỉnh cao), không nhảy; các notch của một trục luôn xếp theo tần số. Tần số dưới `min_hz` (chuyển động của chính máy) không kéo notch.
- Tab Bộ lọc vẽ **bản đồ tần số**: từng lowpass (kèm vùng nó trượt), notch tĩnh, vị trí dynamic notch từng trục và RPM notch từng motor **lúc này** (stream `tuning`).

Nguyên tắc: lọc nhiều hơn → motor mát hơn, máy trễ hơn; lọc ít hơn → nhạy hơn, motor dễ nóng. Có RPM filter chạy tốt thì mới nên kéo thanh trượt gyro lên trên 1,0.

## 5. Angle và Stabilize

| | Angle (Betaflight) | Stabilize (ArduPilot) |
|---|---|---|
| Góc theo cần | đi theo **đường cong rate**: nửa cần cho ít hơn nửa góc tối đa (0,5 → 12,4° với mặc định) | **tỉ lệ thẳng** (0,5 → 22,5°) |
| Cảm giác | trực tiếp; `angle_strength` 50 = 5 °/s cho mỗi độ lệch, có feedforward theo cần | mục tiêu được **định hình** (giới hạn gia tốc và jerk, `stab_input_tc` 0,15 s): lướt tới, không vọt lố trên cả hai máy mô phỏng |
| Yaw | theo rate; `angle_earth_ref` 100 %: đang nghiêng mà gạt yaw thì máy quay quanh trục thẳng đứng | **giữ hướng mũi** khi cần yaw ở giữa (bị gió xoay thì quay về); mục tiêu không chạy trước máy quá 45° |
| Hệ số | `angle_strength`, `angle_feedforward`, `angle_ff_smoothing`, `angle_earth_ref` | `stab_p[3]` (ATC_ANG_RLL/PIT/YAW_P, mặc định 4,5), `stab_accel_rp` 1100, `stab_accel_yaw` 270 °/s², `stab_rate_ff` |

Góc tối đa chung: `rates.angle_limit`. Altitude Hold, Position Hold và failsafe luôn dùng Angle. Trong hai mode tự cân bằng, I-term relax nhạy gấp 5 (ngưỡng 8 °/s thay vì 40).

### Góc nghiêng mà firmware "tin" khi đang bay (`core/ahrs.h`)

Gia tốc kế của một máy đang bay **không chỉ ra phương thẳng đứng**: nó cảm nhận lực đẩy (luôn dọc theo trục đứng của chính máy, nghiêng bao nhiêu cũng vậy) và lực cản không khí, không cảm nhận trọng lực. Máy vừa nghiêng vừa tăng tốc thì gia tốc kế vẫn báo "đang cân bằng"; phải vài giây sau, khi lực cản đã lớn bằng lực kéo ngang, nó mới cho thấy góc nghiêng. Vì vậy khi bay, gyro giữ tư thế, gia tốc kế chỉ được kéo chậm:

| | Giá trị | Ghi chú |
|---|---|---|
| Gain gia tốc kế khi đã arm | 0,25 /s (hằng số thời gian 4 s) | số của Betaflight (`dcm_kp`); ArduPilot DCM và PX4 dùng 0,2 |
| Khi disarm, nằm yên | × 10 | lúc này số đọc đúng là trọng lực |
| Vừa đặt xuống / vừa cấp điện | × 100 trong 0,5 s (sau 0,25 s yên) | đường chân trời được đặt lại sau va chạm, lật máy. Đang bị cầm xoay (> 15 °/s): × 1 |
| Lọc trước khi dùng | thông thấp bậc 2 ở 10 Hz | rung motor 100 Hz trở lên còn dưới 1 % |
| Độ lớn số đọc lệch khỏi lúc nằm yên | trọng số giảm dần, **0 từ 0,1 g** | nghiêng giữ độ cao 25° đã là 1,1 g; leo, rơi, va chạm đều bị bỏ qua. "Lúc nằm yên" được học khi disarm, nên gia tốc kế chưa hiệu chuẩn (ví dụ đọc 1,08 g) vẫn dùng được |
| Học bias gyro | `ki` 0,01; tối đa 0,5 °/s mỗi phút, không quá 3 °/s | vài giây tăng tốc không dạy được một bias không có thật |
| La bàn (chỉ sửa yaw) | 0,5 /s; nhân cùng hệ số × 10 / × 100 khi disarm | đặt máy xuống, hướng mũi đúng lại trong khoảng một giây |

Hệ quả người bay cần biết (đo trên mô phỏng có lực cản, máy 5 inch):
- Giữ **nửa cần 2 giây** (lệnh 12,4°): firmware giữ đúng 12° theo ước lượng, máy nghiêng thật tới ~15° (lệch 3,4°). Thả cần, 3 giây sau cân lại trong 0,5°.
- Giữ **0,8 cần 4 giây** (lệnh 30°, tới 13 m/s): nghiêng thật tới ~34°.
- Sau một quãng bay nhanh rồi dừng, đường chân trời có thể lệch 2 – 4° trong vài giây rồi tự về. Đây là giới hạn chung của mọi bộ ước lượng chỉ có gyro + gia tốc kế (Betaflight cũng vậy); khi có GPS hoặc optical flow, gia tốc thật sẽ được bù (đợt Position Hold).
- Bản 0.5.0 trở về trước dùng gain 2 /s: cùng bài thử, máy nghiêng **27° cho lệnh 12°** và lắc ngược 14° sau khi thả cần. **Đừng bay Angle / Altitude Hold với firmware cũ hơn 0.6.0.**

## 6. Altitude Hold: ga treo tự đo

Ga treo (đúng ra: **mức ra motor** mà tại đó máy treo) khác nhau tới 5 lần giữa máy nặng (≈ 50 %) và quad đua (≈ 13 %), nên **không phải là một setting**:

- `HoverEstimator` (`core/althold.h`) đo liên tục trong **mọi mode bay**: `mức ra × cos(nghiêng) / (1 + gia tốc lên / g)`. Bỏ qua lúc nghiêng quá 45°, motor gần như tắt, hoặc tăng / giảm tốc mạnh. Hội tụ trong ~2,5 s bay tay.
- **Cất cánh trong Altitude Hold**: ga tăng dần 40 %/s cho tới khi máy rời đất (đang lên > 0,15 m/s hoặc cao hơn chỗ đứng 8 cm). Mức ga lúc đó, sau khi trừ phần gia tốc đang có, là ga treo. Tới 90 % mà vẫn chưa rời đất (thiếu cánh, bị giữ): về idle, phải hạ cần ga rồi mới thử lại.
- **Failsafe không có độ cao** (không baro, không lidar): motor về `fs_fallback_throttle` % của **lực treo đã đo**, nên máy chắc chắn đi xuống. Bản 0.5.0 trở về trước dùng số đoán 40 %: máy treo ở dưới ~16 % sẽ bay vọt lên. **Đừng bay Altitude Hold / dựa vào failsafe với firmware cũ hơn 0.6.0 trên máy lực đẩy lớn.**
- Giảm ga mạnh nhất mà Altitude Hold yêu cầu là 0,8 g, để vòng tư thế còn motor mà điều khiển.
- **Mất độ cao giữa chừng** (barometer ngừng trả lời quá 1 s): công tắc Altitude Hold vẫn bật thì cần ga được đọc quanh ga treo đã đo: giữa cần = treo, đáy = 0, đỉnh = 100 %. Ga không nhảy; có độ cao lại thì Altitude Hold tiếp tục từ mức ga đang có. Tab Tổng quan báo "chưa hợp lệ" ở dòng độ cao.
- **Kéo cần xuống ngay sau khi rời đất** = lệnh đi xuống (hạ có lực tới khi đứng yên), không phải tắt motor. **Mất sóng lúc ga đang tăng để cất cánh**: về idle ngay; muốn cất cánh lại phải hạ cần trước.
- Ga do Altitude Hold / failsafe điều khiển không đi qua throttle boost và không kích anti-gravity (hai thứ đó chỉ theo tay bạn).
- Lidar: có barometer thì lidar không còn "kéo" độ cao theo mặt đất bên dưới. Bay qua bàn, bậc thềm: máy giữ độ cao theo barometer, không nhảy theo vật cản.
- Tab PID hiện "Ga treo đo được" (trước chuyến bay đầu là số đoán).

## 7. Level trim khi bay

Máy vẫn trôi khi cần ở giữa trong Angle / Stabilize: gán mode **CALIB** ("Level trim khi bay") cho một công tắc.
1. Bay ở chỗ **lặng gió**, giữ máy **đứng yên một chỗ** bằng cần (không phải chỉ giữ cho nó "trông cân bằng": nó phải hết trôi).
2. Bật công tắc ít nhất 2 giây rồi tắt: hai tiếng bíp ngắn = đã đo (tiếng khác = bị từ chối: chưa đủ 2 giây treo yên).
3. Hạ cánh, disarm: kết quả thành "cân bằng" mới, firmware **tự ghi flash** (không cần app).

Thứ được đo là **gia tốc kế lúc treo**: máy đang treo yên thì gia tốc kế chỉ cảm nhận lực đẩy, nên số đọc của nó cho biết lực đẩy lệch bao nhiêu so với mạch — đúng bằng sai số cân bằng cần sửa (cách Betaflight / MultiWii làm với "in-flight ACC calibration"). Kết quả không phụ thuộc đường chân trời mà firmware đang tin, nên đo ngay sau một cú cơ động vẫn đúng. Hai thứ làm sai:
- **Máy còn đang trôi**: lực cản lọt vào số đọc, khoảng 2° cho mỗi 1 m/s. Trôi 0,25 m/s → sai 0,5°.
- **Gió**: đứng yên trong gió nghĩa là đang nghiêng vào gió. Đo trong nhà hoặc lúc lặng gió.

Chỉ tính thời gian đang arm, firmware thấy máy **đang trên không** (không phải "cần ga trên 15 %": quad lực đẩy lớn treo ở dưới mức đó), quay chậm hơn 15 °/s, nghiêng dưới 10°. Không gì thay đổi lúc đang bay. Lần đo quá ngắn không xoá lần đo tốt trước đó.

## 7b. Ga ở thấp nhất, mặt đất, an toàn lúc cất cánh

- **Airmode** (tab Cảm biến). "Khi đang bay" (mặc định): trên không thì máy vẫn được giữ ổn định đầy đủ khi cần ga ở thấp nhất; đứng yên trên mặt đất 1 giây thì mọi motor về idle và I-term về 0, nên máy đặt trên bàn không tự tăng tốc. "Luôn bật tới khi disarm" là hành vi của Betaflight: dùng khi bạn tắt ga để rơi tự do rất lâu rồi bắt lại (máy rơi phẳng ở tốc độ tới hạn đọc gia tốc 1 g y như đang đứng yên, không có độ cao thì firmware không phân biệt được).
- **Chặn runaway lúc cất cánh** (tab Hệ thống, mặc định bật): lắp sai gì đó làm PID đẩy máy lật thì motor tắt sau 75 ms, lý do hiện ở "Vì sao chưa arm được". Trên bàn không cánh nó không kích (PID dồn lực nhưng máy không quay).
- Arm chỉ được bằng cách **gạt công tắc từ OFF sang ON khi đang có sóng**.

Chi tiết và lý do: `docs/HARDWARE_FEEDBACK_1.md` mục 5 – 6.

## 7c. Thử trên bàn, không cánh: motor không đều là bình thường

Máy không quay được thì vòng PID không có phản hồi. Chỉ cần lệch 0,2° ở chế độ Angle là có sai số 1 °/s không bao giờ hết; I-term cộng dồn khoảng 20 đơn vị mỗi giây, sau 10 – 15 giây ở ga cao các motor lệch nhau vài chục phần trăm. Gạt một cần rồi thả ra: I-term của trục đó chạm trần và **giữ nguyên**, nên máy như "nhớ" lệnh vừa rồi. Sau lần đầu ga vượt 25 %, airmode cho PID toàn quyền cả ở ga rất thấp: gạt yaw ở ga 5 % có thể đưa hai motor lên 80 %. Để cần ga ở đáy 1 giây thì mọi thứ về idle và I-term về 0. Betaflight và ArduPilot cũng vậy; khi bay, máy quay theo lệnh và I-term tự xả.

- Kiểm tra motor: tab Motor, không arm.
- Kiểm tra chiều phản ứng: arm, ga 10 – 15 %, nghiêng máy bằng tay vài giây; motor phía bị hạ thấp phải tăng.
- Muốn motor trở về đều nhau khi thả cần: tạm đặt I = 0 cho ba trục, và trả lại trước khi bay.

Phân tích một log thật với số liệu: `docs/HARDWARE_FEEDBACK_2.md` mục 1.

## 8. Thứ tự tune đề nghị trên máy thật

0. Tab Tổng quan: "Chu kỳ điều khiển thực tế" phải bằng tần số đặt, "bỏ lỡ 0 chu kỳ". Không đạt thì đừng tune: mọi tần số cắt và hệ số đang lệch theo.
1. Tháo cánh: kiểm tra chiều motor (tab Motor), hướng gyro (hình 3D), bật DShot hai chiều nếu ESC hỗ trợ.
2. Lắp cánh, bay Angle ngắn với mặc định. Ghi blackbox (nhóm gyro + gyro thô + PID + motor).
3. Xem motor có nóng không và phổ gyro (tab Blackbox). Có đỉnh cố định → notch tĩnh; nhiễu theo ga mà không có RPM filter → giữ dynamic notch 3 – 5.
4. Máy mềm, vọt lố → Master Multiplier lên từng 0,1. Rung nhanh sau cú đạp ga → Damping lên hoặc Dynamic Damping lên. Trôi chậm sau khi lật → Drift – Wobble.
5. Level trim khi bay, rồi mới thử Altitude Hold.

## 9. Đã kiểm chứng trên PC / chưa kiểm chứng

Đã có (`make test`): từng khối đối chiếu với công thức (`test/test_control.cpp`), bay vòng kín trên hai máy mô phỏng (`test/test_flight.cpp`), chạy trong firmware thật trên mạch ảo kể cả ESC trả lời DShot hai chiều (`test/test_fw.cpp`); lỗi cố ý (mutation) cho mọi dòng có ý nghĩa của các file trên.

**Cần mạch thật, chưa có số nào:**
- Thời gian một vòng lặp với cả chuỗi lọc + dynamic notch + PID ở 4 kHz trên ESP32-S3: tab Tổng quan, "Thời gian từng khâu" (khâu "tính toán"). Bản 0.5 không giữ được 4 kHz trên mạch thật; bản này đã đổi cấu trúc vòng lặp (`docs/HARDWARE_FEEDBACK_1.md` mục 1 – 2) nhưng **chưa có số đo**. Nếu vẫn không kịp: giảm `dyn_notch_count`, giảm số hài RPM, hoặc hạ `loop_hz` xuống 2000.
- RPM filter với telemetry thật (tỉ lệ lỗi khung, độ trễ).
- Bộ số mặc định trên khung của bạn; ga treo đo bằng đường cong lực đẩy thật (lực đẩy không tỉ lệ thẳng với mức ra như mô hình).
- Baro bị gió cánh quạt thổi lúc sát đất có làm việc phát hiện rời đất sớm / muộn không.
- Bộ ước lượng tư thế với gia tốc kế thật: mức rung của khung, hệ số lực cản thật của máy (mô phỏng dùng 0,35 /s: nghiêng 30° thì đạt 16 m/s). Các con số "lệch 3,4°" ở mục 5 là của mô hình đó.
