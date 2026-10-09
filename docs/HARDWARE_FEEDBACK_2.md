# Phản hồi phần cứng lần 2: log đầu tiên từ mạch thật (firmware 0.6.1)

Log: `live_20261007_221658` (ghi trực tiếp qua USB, 250 bản ghi/giây, 87 giây), mạch ESP32-S3 N16R8 + ICM20602 + ESC Aikon AK32, DShot600 một chiều, vòng lặp đặt 4000 Hz, chế độ Angle.

Người dùng báo hai điều:

1. Đẩy ga lên cao thì motor tăng theo, nhưng một lúc sau vài motor tự giảm.
2. Gạt cần nào thì máy "nhớ" luôn kiểu hoạt động đó.

Tài liệu này ghi lại: log nói gì, vì sao, và hai lỗi của firmware mà log làm lộ ra. **Mọi kết luận ở đây đến từ số trong log và từ test trên PC; chưa có lần chạy nào của bản 0.6.2 trên mạch.**

## 1. Hai hiện tượng là I-term tích luỹ khi máy không quay được

### Log nói gì

- Suốt 87 giây gyro chỉ có rung (±10 °/s, trung bình 0); roll / pitch ước lượng đứng yên ở −0,06° / +0,21°. Máy không quay: không gắn cánh, hoặc bị giữ.
- Chế độ Angle muốn đưa máy về 0°. Sai lệch 0,2° cho ra setpoint pitch −1,1 °/s và roll +0,4 °/s **liên tục**, dù các cần ở giữa (hệ số Angle 5 /s × 0,21° ≈ 1,05 °/s).
- Máy không quay nên sai số ấy không bao giờ hết. I-term cộng dồn: pitch `Ki = 84 × 0,244381` ⇒ khoảng −21 đơn vị PID mỗi giây, roll +7.
- Sau 13 giây ở ga 100 %: pitch −240 trên 1000 (1000 = toàn dải motor). Motor 4 đã ở 100 %, mixer chỉ còn cách hạ các motor kia: motor 1 từ 0,95 xuống 0,39, motor 3 xuống 0,53.

### Kiểm chứng bằng số

Từ bốn cột motor suy ra phần chênh lệch theo từng trục (QUAD_X: roll = (−m1 −m2 +m3 +m4)/4, pitch = (m1 −m2 +m3 −m4)/4, chia cho 0,945 vì idle 5,5 %), rồi so với tích phân của `(setpoint − gyro) × Ki` lấy từ chính log:

| Thời điểm | Pitch: motor đo được | Pitch: tính lại | Roll: motor đo được | Roll: tính lại |
|---|---|---|---|---|
| giây 8 | −76,9 | −72,0 | 17,4 | 17,7 |
| giây 11 | −133,7 | −127,7 | 40,8 | 36,2 |
| giây 14 | −187,6 | −185,7 | 57,3 | 55,0 |
| giây 17 | −240,0 | −236,1 | 75,0 | 72,4 |

Hai đường trùng nhau (hình `log_221658_iterm_windup.png` gửi kèm). Chú ý: phải tích phân theo **chu kỳ danh nghĩa** (16 chu kỳ × 250 µs cho mỗi bản ghi) mới khớp; tích phân theo thời gian thật thì lệch đúng 17 % - vì vòng lặp lúc đó chỉ chạy 83 % tần số đặt (mục 3) và vòng PID dùng chu kỳ danh nghĩa như Betaflight.

### "Gạt cần nào nó cũng nhớ"

Cùng một cơ chế. Đẩy cần yaw hết cỡ: firmware đòi 400 °/s, máy không quay, I-term yaw chạm trần (320) trong chưa đầy 0,1 giây. Thả cần về giữa: sai số bằng 0, I-term **giữ nguyên** vì chưa có chuyển động nào trả lại nó. Khi bay, máy quay theo lệnh, sai số đổi dấu và I-term tự xả.

Thêm hai điều dễ gây giật mình khi thử trên bàn:

- Sau lần đầu ga vượt 25 % (`airmode_start_throttle`), airmode "Khi đang bay" coi như máy đã cất cánh. Từ đó ga chỉ cần nhích trên mức thấp nhất là PID có toàn quyền: ở ga ~5 % mà gạt yaw thì hai motor lên 82 % (giây 35,6 trong log).
- Khi cả ba I-term cùng chạm trần, một motor ở 100 % và ba motor ở idle **bất kể ga** (giây 58 – 63): mixer ưu tiên giữ chênh lệch giữa các motor.

Để cần ga ở đáy khoảng 1 giây thì firmware nhận là đã hạ cánh: motor về idle, I-term về 0 (giây 19, 26, 34, 39).

### Vì sao không đổi luật điều khiển

Betaflight (I-term chỉ bị xoá khi ga ở đáy và airmode chưa bật) và ArduCopter (`reset_rate_controller_I_terms_smoothly()` chỉ ở GROUND_IDLE) làm đúng như vậy. I-term phải được phép tích luỹ ngay khi ga rời đáy, nếu không máy lệch trọng tâm sẽ lật lúc cất cánh. "Runaway takeoff" không can thiệp vì điều kiện của nó cần gyro quay nhanh; trên bàn gyro đứng yên.

### Cách thử trên bàn

| Muốn kiểm tra | Làm thế nào |
|---|---|
| Thứ tự, chiều quay, motor có tăng đều theo lệnh không | tab Motor, **không arm**, tháo cánh |
| Chiều phản ứng của PID | arm, ga 10 – 15 %, nghiêng máy bằng tay vài giây: motor phía bị hạ thấp phải tăng. Không đánh giá độ "đều" của motor |
| Motor trở về đều nhau khi thả cần | tạm đặt I = 0 cho cả ba trục, thử, rồi **trả lại trước khi bay** |
| Phản ứng thật của vòng điều khiển | chỉ có khi máy được quay tự do (bay, hoặc treo trên giá quay một trục) |

## 2. Lỗi của firmware: các cột PID trong log bị kẹp ở ±3,28

Trong log, `pid_p`, `pid_i`, `pid_d`, `pid_f` không bao giờ vượt ±3,28. Nguyên nhân: bảng trường ghi các thành phần PID với bước 0,0001 trong số nguyên 16 bit ("phần của toàn dải motor"), nhưng giá trị đưa vào là **đơn vị PID sum** của Betaflight (tới ±500). Mọi thứ trên 3,2767 bị kẹp.

Vì sao test không bắt được: test định dạng dùng số nhỏ (0,01 … 0,12), test firmware chỉ kiểm tra "có PID lớn hơn 0,005 trong log".

Đã sửa (0.6.2):

- Bước ghi là **0,1 đơn vị PID sum**, dải ±3276. Đồ thị PID của app dùng cùng đơn vị (khung nhỏ nhất ±20).
- Test định dạng dùng giá trị thật (144,5; −236,1; 576 …). Test firmware so P lớn nhất trong log với P lớn nhất mà core thật sự tính ra trong chuyến bay mô phỏng (135,6 so với 135,5). Test Python và test trình duyệt kiểm tra P của một cú roll lớn hơn 20.
- Log cũ vẫn đọc được (tỉ lệ nằm trong header của từng log), nhưng các cột PID của nó là số đã bị kẹp.

## 3. Lỗi của firmware: vòng lặp chỉ chạy 83 % tần số đặt

### Số đo thật đầu tiên

| Đại lượng | Giá trị trong log |
|---|---|
| Khoảng cách giữa hai bản ghi (16 chu kỳ) | 4,5 – 6,0 ms, giữa 4,75 ms. Đúng ra là 4,0 ms |
| ⇒ tần số vòng lặp thật | khoảng 3300 Hz trên 4000 |
| `loop_us` của chu kỳ được ghi | giữa 144 µs; 83 % số chu kỳ trong 118 – 180 µs |
| | 14 % số chu kỳ trong 250 – 400 µs (hai cụm tách hẳn nhau: chỉ 3 % nằm ở 180 – 250) |
| `gap_us` | luôn ≥ 500 µs: mỗi 16 chu kỳ có ít nhất một tick bị mất; 15 % số bản ghi có 750 µs |

Chu kỳ dài tập trung ở hai vị trí trong mỗi mili giây: 21 % số chu kỳ bắt đầu ở phần tư thứ hai của mili giây và 29 % ở phần tư thứ ba là chu kỳ dài, so với 6 % ở hai phần tư còn lại. Luồng cảm biến chạy một lượt mỗi mili giây, nên **chu kỳ dài có liên quan tới luồng cảm biến**: hoặc chu kỳ điều khiển phải chờ khoá, hoặc mã của nó bị đẩy khỏi cache và phải nạp lại từ flash. Log này không đủ để phân biệt hai khả năng.

Bộ ước lượng (500 Hz) chạy theo số đếm chu kỳ, cùng nhịp với việc ghi log, nên trong một phiên ghi nó hoặc luôn rơi vào chu kỳ được ghi, hoặc không bao giờ: `loop_us` không cho biết nó tốn bao nhiêu.

### Phần thuộc về firmware 0.6.1

Quy tắc thêm ở 0.6.1 ("chu kỳ kết thúc khi còn dưới ¼ chu kỳ đặt thì nhường luôn tick kế tiếp") biến **mỗi** chu kỳ dài hơn 188 µs thành một tick mất. Với số đo trên: thường là ba tick mất trong mỗi 19, tức khoảng 83 %.

### Đã đổi (0.6.2): quỹ thời gian thay cho quy tắc theo từng chu kỳ

`app::TickBudget` + `app::TickLoop` trong `src/app/sync.h`; `flight_task.h` và test dùng **cùng một đoạn mã**.

- Luồng cảm biến được "nợ" ¼ thời gian của mỗi chu kỳ (làm tròn lên) dưới dạng thời gian nghỉ, tức ⅕ nhân.
- Thời gian nghỉ nhiều hơn mức đó là "có", tối đa nửa chu kỳ đặt. Còn "có" thì chu kỳ chạy quá tick được nối tiếp ngay bằng chu kỳ sau (bù tối đa một tick; nhiều tick nổ trong một chu kỳ thì gộp làm một).
- Khi số dư âm, task bay nghỉ cho tới tick đầu tiên mà khoản nợ đã được trả.

Bảng dưới là kết quả **mô phỏng lịch chạy** (`test_app`), chưa đo trên mạch; dòng đầu dùng một mẫu chu kỳ xấp xỉ số đo ở trên.

| Chu kỳ điều khiển (timer 4000 Hz) | 0.6.1 | 0.6.2 |
|---|---|---|
| 135 µs, một trong tám là 290 µs, một trong mười sáu là 200 µs | ≈ 3300 Hz (đo trên mạch) | **4000 Hz**, còn 37 % nhân, luồng cảm biến chờ tối đa 425 µs |
| 240 µs đều | 2000 Hz | 3200 Hz |
| một trong tám là 400 µs | 3550 Hz | 4000 Hz (hai chu kỳ sau bắt đầu trễ rồi đuổi kịp) |
| 800 µs đều (như bản 0.5) | 1000 Hz | 1000 Hz, còn 20 % nhân |
| một lần đứng 20 ms (ghi flash) | mất 80 tick | mất 80 tick trong lúc đứng + 20 tick nghỉ |

Bảo đảm, kiểm tra bằng mô phỏng với timer 2 / 4 / 8 kHz và chu kỳ dài từ 1 µs tới 12 chu kỳ đặt:

- luồng cảm biến không bao giờ có dưới 20 % nhân;
- chu kỳ dài tới 80 % chu kỳ đặt dùng đủ mọi tick;
- tần số đạt ít nhất 93 % mức mà "⅕ thời gian nghỉ" cho phép;
- luồng cảm biến không bao giờ chờ lâu hơn hai chu kỳ đặt cộng hai chu kỳ điều khiển (1 ms ở 4 kHz khi chu kỳ vừa trong chu kỳ đặt).

Chu kỳ bắt đầu trễ vài chục µs sau một chu kỳ dài: bộ ước lượng dùng thời gian thật giữa hai chu kỳ, vòng PID dùng chu kỳ danh nghĩa (như Betaflight).

### Log 0.6.2 nói thêm thời gian nằm ở đâu

Nhóm trường "Thời gian vòng lặp" có thêm năm cột, mỗi cột một byte, bước 4 µs, tối đa 1020 µs. Mỗi cột là giá trị **lớn nhất** của khâu đó trong các chu kỳ kể từ bản ghi trước:

| Cột | Khâu | Nếu lớn thì |
|---|---|---|
| `t_lock` | chờ luồng cảm biến nhả khoá | pha "bàn giao" của luồng cảm biến giữ khoá quá lâu → chia nhỏ pha đó |
| `t_imu` | đọc IMU qua SPI | mã SPI của Arduino nằm trong flash → ghi thẳng thanh ghi SPI |
| `t_core` | lọc, PID, mixer, trong chu kỳ **không** chạy bộ ước lượng | mã / hàm toán còn trong flash |
| `t_est` | như trên, trong chu kỳ **có** chạy bộ ước lượng (0 = không có chu kỳ nào như vậy trong khoảng này) | `sinf / cosf / atan2f / asinf / sqrtf` của thư viện nằm trong flash → thay bằng bản trong RAM, hoặc chia bộ ước lượng ra nhiều chu kỳ |
| `t_motor` | gửi DShot + đọc trả lời ESC | driver RMT |

Bản ghi mặc định dài thêm 5 byte (69 → 74). App vẽ năm cột này ở khung thứ hai của nhóm "Vòng lặp".

## 4. Việc tiếp theo cần số đo của bạn

1. Nạp **0.6.2** (app hiện phiên bản ở thanh trên cùng).
2. Ghi một log trực tiếp như lần này, khoảng 30 giây, arm và để ga khoảng 30 %, không cần gạt cần. Nhóm "Thời gian vòng lặp" phải được chọn (mặc định có).
3. Gửi file CSV. Tôi cần: khoảng cách giữa các bản ghi (kỳ vọng 4,0 ms), `gap_us` (kỳ vọng dưới 500: sau một chu kỳ dài, chu kỳ kế tiếp bắt đầu ngay nên khoảng cách dài nhất chỉ bằng chính chu kỳ dài đó), và năm cột `t_*`.
4. Tab Tổng quan lúc arm: "Chu kỳ điều khiển thực tế" và "Thời gian từng khâu".

Từ đó mới quyết định tối ưu chỗ nào; tối ưu trước khi có số đo là đoán.
