# Windburst Reworked

Firmware flight controller cho ESP32-S3 (RX/FC) + lớp giao tiếp với app. Tài liệu: `docs/ARCHITECTURE.md` (mục lục).

## Dùng nhanh

```
# 1. Nạp firmware (PlatformIO). rx_s3 = module ESP32-S3 N16R8, app qua cầu USB-UART; rx_s3_usb = cùng module, app qua cổng USB của chính
#    ESP32-S3 (nhanh hơn nhiều); module N8 không PSRAM: -e rx_s3_n8. Xem docs/BOARD_N16R8.md
pio run -e rx_s3 -t upload

# 2. Configurator đồ họa (liên kết là NHỊ PHÂN - đóng serial monitor trước)
cd app && pip install -r requirements.txt
python -m wbclient gui                                   # mở trình duyệt; chọn cổng COM và baud trong trang (giao diện Việt / Anh)

# 3. Hoặc dòng lệnh, cùng các thao tác đó
python -m wbclient --port COM5 info                      # cảm biến nào tìm thấy / vì sao không
python -m wbclient --port COM5 set pins imu_cs=10 imu_miso=13 imu_mosi=11 imu_sck=12 i2c_sda=8 i2c_scl=9 rx_rx=16 gps_rx=17 gps_tx=18
python -m wbclient --port COM5 set pins motor[0]=4 motor[1]=5 motor[2]=6 motor[3]=7
python -m wbclient --port COM5 set modes mode[0]=ARM aux[0]=0 lo[0]=1700 hi[0]=2100
python -m wbclient --port COM5 save
python -m wbclient --port COM5 reboot
python -m wbclient --port COM5 monitor status,attitude,rc   # hoặc: monitor nav,estimate (optical flow, Position Hold, GPS Rescue)
python -m wbclient --port COM5 calibrate acc             # rồi: calibrate level, calibrate mag
python -m wbclient --port COM5 blackbox download         # log chuyến bay (.wblog) về máy; xem bằng tab Blackbox của GUI
```
(Số chân ở trên chỉ là ví dụ — nhập chân thật của mạch bạn. Tab "Chân và cổng" có nút "Điền bộ chân gợi ý" cho DevKitC N16R8.)

## Không có mạch

```
make test        # toàn bộ test firmware trên PC (g++), kể cả firmware-in-the-loop
make sim         # build/wb_sim: firmware thật trên mạch ảo
build/wb_sim &
cd app && python -m wbclient gui                          # bấm "Tìm cổng", chọn "Mô phỏng wb_sim"
cd app && python -m wbclient --port socket://127.0.0.1:5760 info
make apptest     # client Python <-> wb_sim (cần pyserial, pytest)
```

## Trạng thái

Có: lõi bay với vòng rate và chuỗi lọc của Betaflight (PID đầy đủ, thanh trượt tune, lowpass động, RPM filter, dynamic notch, feedforward, RC smoothing), Angle (Betaflight) và Stabilize (ArduPilot), Altitude Hold tự đo ga treo, failsafe có nguyên nhân và trả quyền, chặn runaway lúc cất cánh, level trim khi bay, 14 loại IMU, motor (DShot một / hai chiều, PWM), baro, lidar, GPS, la bàn, receiver CRSF / SBUS / IBUS / Spektrum / PPM / PWM (bản 0.8.0), telemetry CRSF về tay phát (0.8.0), đo pin, đo dòng / mAh (0.8.0), servo theo kênh RC (0.7.1, `docs/SERVO.md`), bảng cổng UART (0.8.0, `docs/PORTS.md`), tiếng báo bằng còi hoặc bằng motor, blackbox ra flash hoặc thẻ SD, vòng điều khiển trong task riêng với số đo thời gian từng khâu, **optical flow (PMW3901, cảm biến MSP như MTF-01), Position Hold (flow + lidar, hoặc GPS + la bàn) và GPS Rescue (bản 0.7.0, xem `docs/NAV.md`)**, configurator 13 tab. Cách tune: `docs/TUNING.md`. Tay phát DIY ESP32-C5 + receiver nRF24 trên bus SPI thứ hai (0.38, `docs/TX.md`, chưa thử trên mạch). Chưa có: bind Spektrum, receiver nRF24 trên STM32.

Bản 0.5 đã chạy trên mạch thật (ESP32-S3 N16R8, không cánh); các lỗi lộ ra ở lần đó và cách bản 0.6 xử lý: **`docs/HARDWARE_FEEDBACK_1.md`** (đọc mục 8 trước khi nạp: hai mục cấu hình phải nhập lại). Log đầu tiên của bản 0.6.1 trên mạch (thử không cánh: vì sao motor lệch nhau, và hai lỗi nó làm lộ ra): **`docs/HARDWARE_FEEDBACK_2.md`**. Bản 0.6.1 đã chạy trên mạch đó (không cánh): arm, receiver CRSF, DShot600 ra bốn motor và ghi log trực tiếp qua USB hoạt động; vòng lặp đo được khoảng 3300 Hz trên 4000. Những gì 0.6.2 đổi sau log đó, toàn bộ bản 0.7.0 (optical flow, Position Hold, GPS Rescue), servo (0.7.1 / 0.7.2), toàn bộ 0.8.0 (đo dòng, receiver Spektrum / PPM / PWM, telemetry CRSF, bảng cổng), và mọi thứ chưa được nhắc ở đây (DShot hai chiều, ghi log vào flash / thẻ SD, Altitude Hold, failsafe, **bay thật**), mới được kiểm chứng trên PC và biên dịch cho ESP32-S3. Xem `docs/HANDOFF.md` (bảng tiến trình) và mục "Chưa kiểm chứng" trong từng tài liệu trước khi lắp cánh.

## English (short)

Windburst is a flight controller firmware for the ESP32-S3 with a configurator that runs in your browser (Vietnamese / English).

```
pio run -e rx_s3 -t upload                 # flash the firmware (ESP32-S3 N16R8 module; -e rx_s3_usb = app over the chip's own USB; -e rx_s3_n8 = 8 MB module without PSRAM)
cd app && pip install -r requirements.txt   # pyserial
python -m wbclient gui                      # opens the configurator; pick the COM port and baud rate in the page
```
No board: `make sim && build/wb_sim`, then choose "Simulator wb_sim" in the port list. The link is binary: close any serial monitor first.
Tests: `make test` (firmware on the PC), `make apptest` (app and GUI against the simulated firmware). The documents in `docs/` are in Vietnamese.
Status: version 0.5 has run on a real ESP32-S3 board (props off); what that showed and what 0.6 changed is in `docs/HARDWARE_FEEDBACK_1.md`. The first log from 0.6.1 on the board is analysed in `docs/HARDWARE_FEEDBACK_2.md`. Version 0.6.1 has run on that board with the props off (arming, CRSF receiver, DShot600 output, live logging over USB; the loop reached about 3300 Hz of 4000). What 0.6.2 changed after that log, all of 0.7.0 (optical flow, Position Hold, GPS Rescue: `docs/NAV.md`), the servos (0.7.1 / 0.7.2) and all of 0.8.0 (current sensing, Spektrum / PPM / PWM receivers, CRSF telemetry, the serial port table), and everything else (bidirectional DShot, logging to flash / SD card, Altitude Hold, failsafe, **actual flight**), is tested on the PC and compiled for the ESP32-S3 only.
