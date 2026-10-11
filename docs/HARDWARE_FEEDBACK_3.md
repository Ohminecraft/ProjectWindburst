# Phản hồi phần cứng lần 3: DShot hai chiều với firmware 0.8.2

Báo cáo của người dùng (tab Motor, DShot hai chiều bật, ESC Aikon AK32, BLHeli_32 / AM32, **có cắm pin**):

| Dòng | Giá trị |
|---|---|
| DShot hai chiều | đang chạy (kiểm tra lúc khởi động qua) |
| ESC không trả lời | 1, 2, 3, 4 |
| Khung DShot bỏ qua trong giây vừa rồi | 0 |
| Lượt trả lời bị lỡ | 0 |
| Trả lời nhận được / hỏng mỗi giây | 0 / 0 |
| Điện áp ESC báo | không có |
| Motor quay ở tab Motor | **không** |
| esp-fc trên cùng mạch | chưa thử |

## Đọc ra được gì

- Kiểm tra lúc khởi động qua: bộ phát RMT ra tới chân, và bộ thu nghe đúng một khung DShot **đảo** (idle cao, xung thấp). Đổi cực sai thì kiểm tra này đã báo lỗi.
- Khung bỏ qua 0, lượt bị lỡ 0: khung được phát mỗi chu kỳ, timer nhả chân đúng giờ.
- 0 trả lời **và** 0 trả lời hỏng: trong cửa sổ nghe, bộ thu không thấy dây đổi mức lần nào.
- Motor không quay: ESC không giải mã được khung nào. Vấn đề không nằm ở phần nghe telemetry; ESC không nhận ra tín hiệu.

## Giả thuyết chính

Giữa hai khung, chân bị nhả (đầu vào + pull-up nội ~45 kΩ) khoảng 220 trên 250 µs. Nếu đầu vào ESC có điện trở kéo xuống hoặc mạch lọc, dây nằm thấp gần như suốt thời gian đó: với ESC, đó không phải DShot đảo. Điều này khớp cả ba quan sát: kiểm tra lúc khởi động qua (chân đang được đẩy-kéo), không có cạnh nào trong cửa sổ nghe (dây đã xuống thấp trước khi bộ thu bật), motor không quay. Mô hình dây trong `test_dshot` cho đúng triệu chứng đó với trình tự của 0.8.3.

Giả thuyết khác vẫn còn: ESC chỉ dò kiểu tín hiệu lúc nó khởi động. Nếu ESC khởi động trước khi FC phát khung đảo, nó có thể không nhận ra.

## Đã làm (0.8.4)

Xem `docs/MOTOR_OUTPUT.md`, mục "0.8.4". Tóm tắt:
- chân được đẩy lên cao lại ngay sau cửa sổ trả lời;
- mức dây được đo ở cuối mỗi cửa sổ (`line_low`);
- khung đang phát được đọc lại mỗi giây khi đang chạy (`rb_ok` / `rb_bad`).

## 0.8.6
Người dùng gắn điện trở kéo lên 2 kΩ trên dây tín hiệu motor, và thấy 0.8.4 làm vòng lặp tụt từ ~5000 xuống ~4300 Hz. 0.8.6 trả trình tự DShot về như 0.8.2 / 0.8.3 và giữ setting `motors.ext_pullup`. Với 2 kΩ, dây nhả ra luôn ở mức cao, nên giả thuyết "dây bị kéo xuống" được thử thẳng: nếu ESC vẫn không nhận khung thì nguyên nhân nằm chỗ khác.

## Cần báo lại (0.8.4, không còn áp dụng các dòng chẩn đoán)

1. Motor có quay ở tab Motor với DShot hai chiều không?
2. Hai dòng mới ở tab Motor: "Khung đang phát, đọc lại ở chân" và "Dây tín hiệu nằm ở mức thấp khi nhả chân".
3. "Trả lời nhận được / hỏng mỗi giây" và RPM.
4. Nếu vẫn không quay: thử cắm USB trước, đợi FC chạy xong rồi mới cắm pin.
5. Nếu có scope / logic analyzer: mức dây giữa hai khung, và có xung trả lời khoảng 30 µs sau khung không.
