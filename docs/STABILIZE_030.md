# Stabilize viết lại theo AC_AttitudeControl (0.30)

Mới kiểm tra trên PC (unit test + firmware trên mạch ảo). **Chưa bay thật.**

## 1. Khác gì so với trước
- **Trước 0.30:** sai số được tính theo từng góc Euler (roll mục tiêu − roll, pitch mục tiêu − pitch, hướng mục tiêu − hướng), rồi mới đổi sang tốc độ thân máy.
  - Chỉ đúng khi nghiêng ít.
  - Khi vừa nghiêng ngang vừa chúi, hoặc nghiêng lớn, các trục lẫn vào nhau.
  - Sửa hướng có thể kéo lệch độ nghiêng.
- **Từ 0.30:** dùng bộ điều khiển quaternion của ArduPilot (`AC_AttitudeControl::attitude_controller_run_quat`), xem `src/core/attitude.h`.

## 2. Các bước
1. **Lệnh phi công** (`input_euler_angle_roll_pitch_euler_rate_yaw`), giữ nguyên như trước:
   - cần roll / pitch cho **góc**, cần yaw cho **tốc độ quay**;
   - mỗi trục được tạo hình (`attitude_command_model`): gia tốc giới hạn, input time constant;
   - kết quả là mục tiêu Euler cùng tốc độ và gia tốc của nó.
2. **Mục tiêu và máy đều là quaternion.** Sai số được tách như `thrust_vector_rotation_angles`:
   - **trước tiên**: phép quay đưa vector lực đẩy ("hướng lên" của máy) về trùng vector lực đẩy mục tiêu, cho thành phần x / y;
   - **sau đó**: phần hướng còn lại quanh trục đó, cho thành phần z.
   - Vì vậy sai số hướng không bao giờ làm lệch độ nghiêng.
3. **Sai số hướng tối đa 45°.** Máy không theo kịp (bị giữ, bão hoà) thì mục tiêu được kéo theo máy.
4. **Mỗi trục thân máy:** square-root controller với hệ số Stabilize P và nửa giới hạn gia tốc (`update_ang_vel_target_from_att_error`).
5. **Feed-forward:**
   - tốc độ Euler của mục tiêu được đổi thành tốc độ thân **tại tư thế mục tiêu** (`euler_rate_to_ang_vel`), rồi quay sang hệ của máy;
   - khi sai lệch lực đẩy từ 30° đến 60°, feed-forward giảm dần;
   - trên 60°, yaw được để nguyên theo gyro (như ArduPilot: lấy lại lực đẩy trước).
6. **Gia tốc mục tiêu** (feed-forward cho vòng rate) cũng được tính trong hệ thân máy.

Bên trong code dùng hệ của ArduPilot (pitch + = ngóc mũi). Giao diện ra ngoài vẫn theo quy ước của firmware (pitch + = chúi mũi), nên các phần khác không đổi.

## 3. Cài đặt
Không đổi: `stab_p` (Roll / Pitch / Yaw P), `stab_accel_rp`, `stab_accel_yaw`, `stab_input_tc`, `stab_rate_ff`. Cùng nghĩa, cùng mặc định như ArduPilot.

## 4. Test (`test_control`, nhóm `[stabilize 0.29]`)
- Hàm tách sai số:
  - chỉ lệch hướng → toàn bộ sai số nằm ở z;
  - chỉ lệch roll → chỉ có x;
  - đúng mục tiêu khi nghiêng và chúi cùng lúc → sai số 0.
- Nghiêng 40°, lệch hướng 40°: quay về đúng hướng, độ nghiêng thay đổi dưới 3° trong lúc quay.
- Lệch lớn cả ba trục (60 / 50 / 120°): về thăng bằng; lực đẩy được sửa trước, sau 0,5 s sai lệch lực đẩy còn dưới 20°.
- Sai lệch lực đẩy trên 60°: yaw để nguyên theo gyro.
- Ngóc mũi 60°, lệch hướng 10°: phần lớn lệnh là roll, vì máy quay quanh trục thẳng đứng của đất. Bộ cũ tính theo hiệu Euler sẽ dồn hết vào yaw.
- Mục tiêu chúi 30° và đang quay yaw 60°/s: tốc độ thân máy đúng bằng 60·sin30 cho roll và 60·cos30 cho yaw. Feed-forward gia tốc cũng có phần roll tương ứng.
- Máy lệch roll 30° so với mục tiêu đang quay: tốc độ quay của mục tiêu được quay sang trục của máy (có phần pitch).
- Các test bay trên plant vật lý (`test_flight`: bước nghiêng, yaw, gió) vẫn qua không đổi ngưỡng.
- Có 25 mutant `stab:` viết lại cho code mới, tất cả đều bị test bắt.

## 5. Cần thử trên mạch
1. Stabilize, cần roll hoặc pitch nhanh rồi về giữa: không vọt, cảm giác giống trước.
2. Nghiêng lớn kèm quay yaw (bay vòng): độ nghiêng giữ đều.
3. Gió giật làm lệch hướng: quay về hướng mà máy không chao.
