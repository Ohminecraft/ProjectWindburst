# Servo (firmware 0.7.1, sửa lỗi ở 0.7.2)

Tối đa **4 servo**, mỗi servo đi theo một kênh RC (gimbal nghiêng camera trên một núm xoay, kẹp gắp trên một công tắc...).

**Chưa chạy trên mạch thật.** Đã kiểm chứng trên PC: firmware thật trên mạch ảo (xung PWM đo ở bộ PWM giả lập), biên dịch cho ESP32-S3.

**0.7.2:** sửa lỗi của 0.7.1: firmware đọc cài đặt servo qua con trỏ có thể không thẳng hàng, trên ESP32-S3 có thể gây lỗi (crash) khi khởi động hoặc khi đổi cài đặt servo. Nếu đã nạp 0.7.1 thì nạp lại 0.7.2.

## Cách dùng
1. Tab **Chân và cổng**, nhóm "Servo": nhập GPIO cho servo 1..4. **Lưu và khởi động lại.**
2. Tab **Motor**, khối "Servo": chọn kênh RC cho từng servo (CH1..CH16, "Tắt" = không phát xung), xung nhỏ nhất / lớn nhất, đảo chiều. Có tác dụng ngay, không cần khởi động lại. Cột "Đang phát" hiện xung đang gửi ra.
3. "Tần số xung servo": 50 Hz cho servo analog (mặc định), tới 330 Hz cho servo digital. Đổi tần số cần lưu và khởi động lại.

## Firmware làm gì
- Kênh RC ở 1000 µs → xung "nhỏ nhất", 2000 µs → xung "lớn nhất", tuyến tính, kẹp ở hai đầu; "đảo chiều" đổi chỗ hai đầu.
- Chưa nhận khung RC nào (vừa bật nguồn, chưa có receiver): servo đứng ở giữa khoảng. **Mất sóng: servo giữ nguyên vị trí cuối.**
- Phần cứng: bộ LEDC PWM của ESP32 (14 bit). ESP32-S3 có **8 kênh LEDC dùng chung**: motor ở giao thức PWM / Oneshot và còi thụ động cũng dùng chúng. Với DShot, cả 8 kênh rảnh. Hết kênh: báo cáo khởi động ghi `servo: FAILED - no free PWM (LEDC) channel ...`.
- Chân servo trùng chân khác (motor, ADC...): `pins: FAILED ...` và không cho arm.
- Mã: `src/hal/servo/servo_out.h`; settings: section `servo` (id 18, live), `pins.servo[4]`; stream `motors` có thêm `servo[4]` (µs).

## Lưu ý phần cứng
- Nguồn servo lấy từ BEC 5 / 6 V riêng, **nối chung GND** với mạch. Không cấp servo từ 3,3 V của ESP32.
- Xung 3,3 V của ESP32 đủ cho hầu hết servo. Servo nào không nhận thì cần mạch nâng mức 5 V.
- Nhiều servo chịu được 500..2500 µs, nhưng hãy bắt đầu với 1000..2000 µs rồi nới dần: servo bị ép vào chặn cơ khí sẽ nóng và tốn dòng.

## Kiểm chứng
`test_fw` (nhóm `servo`, `WB_ONLY=servo ./build/test_fw`): báo cáo khởi động, tần số / độ phân giải PWM, ánh xạ kênh → xung (có đảo chiều), kẹp ở hai đầu, đổi khoảng xung khi đang chạy, giữ vị trí khi mất sóng, tắt servo, xung ở giữa trước khung RC đầu tiên ở 200 Hz, trùng chân với motor, hết kênh PWM. Chưa có test trình duyệt riêng cho khối Servo ở tab Motor.
