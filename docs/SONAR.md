# Sonar / Lidar (VL53L1X, VL53L0X)

Prompt ghi `Lidar: VL53L1X, VL53LX`; "VL53LX" mình hiểu là **VL53L0X** (chip thật của ST). Betaflight không có driver cho hai chip này, nên không có gì để chép từ BF.

| File | Vai trò | Kiểm chứng |
|---|---|---|
| `hal/range/range_driver.h` | Giao diện driver không chặn (khoảng cách theo tia + chất lượng) | — |
| `hal/range/range_pololu.h` | Driver VL53L1X / VL53L0X: bọc **thư viện Pololu** (MIT) | **biên dịch + link cho ESP32-S3** (symbol có trong ELF); chưa chạy trên mạch |
| `hal/range/range_height.h` | Khoảng cách → độ cao: bù nghiêng, lọc, tự học độ cao lắp cảm biến, sigma | 33 test PC, 12 lỗi cố ý đều bị bắt |
| `hal/range/range_base.h` | `RangeFinder` (tab Sonar): dò chip, công tắc bật/tắt, báo lỗi bằng chữ | test PC với driver giả |
| `hal/flow/flow_sensor.h` (`FlowRangeDriver`) | Lidar nằm trong cảm biến optical flow MSP (MicoAir MTF-01 / MTF-02): khung MSP v2 0x1F01, `sensors.range_model = MSP` hoặc AUTO khi không có VL53L*; mất số đo 0,5 s = không khỏe (như ArduPilot `AP_RangeFinder_MSP`) | `test_flowsensor`, `test_fw` (0.7.0), xem `docs/NAV.md`; chưa chạy trên mạch |

## Vì sao bọc thư viện thay vì tự viết thanh ghi
VL53L1X/L0X cần chuỗi khởi tạo độc quyền của ST (đọc hiệu chuẩn SPAD từ NVM, hiệu chuẩn tham chiếu, nạp hàng trăm thanh ghi tuning, địa chỉ thanh ghi 16-bit ở L1X). Tự viết lại từ trí nhớ mà không có mạch kiểm chứng có thể cho ra khoảng cách "trông hợp lý nhưng sai" — nguy hiểm hơn việc báo "không có cảm biến" cho vòng giữ độ cao. Code Windburst cũ cũng chọn cách này (prompt bảo tận dụng lại).
**Bắt buộc có `lib_deps` trong `platformio.ini`** (đã thêm cho `rx_s3`). Mình đã thử: thiếu `lib_deps` thì PlatformIO không thêm thư viện, driver âm thầm biên dịch thành "library missing" (`failReason()` sẽ nói rõ).

## Cách dùng
```cpp
RangeFinder sonar;
sonar.attachI2C(sensorBus);          // dùng chung bus I2C với baro/compass (hoặc setI2cPins)
if (!sonar.RangeInit()) Serial.println(sonar.failReason());
// vòng lặp:
if (sonar.update(micros(), millis(), fc.att.up_cos(), armed)) {
    kf.cfg.r_range = sonar.sigmaM();
    kf.update_range(sonar.heightM());      // đã bù nghiêng, đã trừ độ cao lắp cảm biến
}
arm.range_ok = sonar.valid(millis());
```
- `setModel(AUTO|VL53L1X|VL53L0X)`, `setL1xMode(0 short | 1 medium | 2 long)` (mặc định long = 4 m; ngoài nắng gắt hãy thử medium/short), `setEnabled(false)` cho công tắc trong app.
- Altitude Hold **không phụ thuộc** sonar: ngoài tầm đo / nghiêng quá 30° / mất tín hiệu thì Kalman bỏ qua và baro tiếp tục.

## Xử lý (range_height.h)
- Độ cao = khoảng cách × cos(nghiêng); nghiêng > 30° thì bỏ mẫu. Chỉ dùng tới 90% tầm tối đa (VL53L1X long 4 m → 3,6 m; VL53L0X 1,2 m → 1,08 m).
- Nhảy bất khả thi (> 8 m/s + 0,3 m) bị loại; 3 mẫu liên tiếp nhất quán thì chấp nhận mức mới (mái nhà, bậc thang). Tay chặn dưới drone chỉ 1–2 mẫu nên bị loại.
- Độ cao lắp cảm biến (vài cm khi đỗ) tự học khi **disarm**, nằm ngang, đo < 0,5 m; khoá khi arm. Cầm drone trên không lúc bật nguồn thì không học bừa. Đặt `cfg.ground_clearance_m ≥ 0` để cố định.
- `sigmaM()` = 2 cm + 1,5% × độ cao → `kf.cfg.r_range`.
- Test đóng vòng với Kalman thật + baro trôi: sai số độ cao < 12 cm, vận tốc không bị lệch dần (lỗi `-95 cm/s` mà prompt nêu); khi lidar mất vì bay quá cao thì không có bước nhảy.

## Trên mạch (theo thứ tự)
1. `RangeInit()` true? In `name()`, `maxRangeM()`, `rateHz()`; sai thì đọc `failReason()` (dây, nguồn 2,8 V, chân XSHUT lên cao).
2. Đặt drone nằm yên: `rawM()` ≈ độ cao cảm biến; `heightM()` về ~0 sau ~1 s (đã học).
3. Nhấc lên 0,5 / 1 m: `heightM()` theo đúng; nghiêng 40°: `update()` trả false.
4. Ngoài nắng: nếu `quality()` thấp / mất tầm, thử `setL1xMode(1)` hoặc `(0)`.
5. Nếu thêm cảm biến siêu âm thật (HC-SR04/US-100) sau này: chỉ cần một `RangeDriver` mới, phần còn lại dùng lại.
