# Windburst

[![CI](https://github.com/Ohminecraft/ProjectWindburst_Re/actions/workflows/ci.yml/badge.svg)](https://github.com/Ohminecraft/ProjectWindburst_Re/actions/workflows/ci.yml)
[![Release](https://img.shields.io/github/v/release/Ohminecraft/ProjectWindburst_Re?include_prereleases)](https://github.com/Ohminecraft/ProjectWindburst_Re/releases)
[![Stars](https://img.shields.io/github/stars/Ohminecraft/ProjectWindburst_Re?style=flat)](https://github.com/Ohminecraft/ProjectWindburst_Re/stargazers)
[![License: GPL-3.0](https://img.shields.io/badge/license-GPL--3.0-blue)](LICENSE)
[![PlatformIO](https://img.shields.io/badge/PlatformIO-compatible-orange?logo=platformio)](https://platformio.org)
[![Arduino framework](https://img.shields.io/badge/Arduino-framework-00979D?logo=arduino)](https://github.com/espressif/arduino-esp32)
[![ESP32](https://img.shields.io/badge/ESP32-S3%20%7C%20classic%20%7C%20C5-E7352C?logo=espressif)](https://www.espressif.com)

**Firmware flight controller hiện đại cho ESP32.** Windburst mang những gì các firmware bay mã nguồn mở làm tốt nhất -
vòng rate và bộ lọc của [Betaflight](https://github.com/betaflight/betaflight), các chế độ dẫn đường của
[ArduPilot](https://github.com/ArduPilot/ardupilot), cách thiết kế failsafe của [PX4](https://github.com/PX4/PX4-Autopilot) - lên
ESP32-S3. Mọi thứ được viết lại từ mã nguồn thật của các dự án đó và kiểm tra đối chiếu với chúng. Đi kèm là một app cấu hình chạy
trong trình duyệt.

[English](README.md) · [Tài liệu](docs/vi/ARCHITECTURE.md) · [Nạp firmware trên web](https://ohminecraft.github.io/ProjectWindburst_Re/) · [Các bản phát hành](https://github.com/Ohminecraft/ProjectWindburst_Re/releases)

> **Trạng thái: thử nghiệm.** Mọi thứ dưới đây đã được kiểm tra trên PC (unit test, mô phỏng bay vòng kín, firmware-in-the-loop
> với mô phỏng cảm biến ở mức thanh ghi, mutation test) và biên dịch được cho mọi mạch. Đã chạy trên phần cứng: ESP32-S3, tháo cánh.
> **Chưa bay thật.** Đọc mục "Chưa kiểm chứng" của từng tài liệu trước khi lắp cánh.

## Làm được gì

| | |
|---|---|
| **Lõi bay** | PID vòng rate của Betaflight (feedforward, I-term relax, anti-gravity, TPA, throttle boost, thanh trượt tune), bộ lọc gyro (lowpass động, RPM filter, dynamic notch), RC smoothing; Angle (Betaflight), Stabilize (ArduPilot), Acro trainer, Airmode; chặn runaway lúc cất cánh, phát hiện motor dừng |
| **Kiểu bay** | **FPV** - mọi chế độ theo công tắc, kể cả Acro. **Flycam** - như drone thương mại: luôn tự cân bằng, giữ độ cao và vị trí, cần ga ở giữa = đứng yên |
| **Dẫn đường** | Altitude Hold (baro / lidar / GPS, bộ điều khiển kiểu ArduPilot, tự học ga treo), Position Hold, **Loiter** (ArduCopter), GPS Rescue / RTL / Smart RTL / Land, nhiệm vụ (waypoint, giao hàng thả bằng servo), hàng rào ảo, optical flow (PMW3901, MicoAir MTF-01) |
| **An toàn** | Failsafe kiểu PX4 (mất sóng, công tắc, pin yếu / cạn: giữ chỗ, RTL, hạ cánh, tắt motor), kiểm tra trước khi arm có lý do cho từng lần từ chối, **crash flip** (turtle mode của Betaflight) |
| **Cảm biến** | 14 loại IMU (ICM-426xx/456xx, BMI270, MPU6xxx ...; hai IMU), baro BMP280 / BMP388 / MS5611, la bàn QMC5883 / HMC5883 / RM3100 / AK8963 có tự hiệu chuẩn, lidar VL53L0X / L1X, GPS u-blox / NMEA, cảm biến DroneCAN |
| **Motor** | DShot 150 - 1200 (hai chiều, extended telemetry), ProShot, PWM / Oneshot / Multishot, ESC DroneCAN; 14 kiểu khung; ESC passthrough cho BLHeliSuite / ESC Configurator |
| **Radio** | CRSF / ELRS (có telemetry, và chế độ MAVLink của ELRS), SBUS, IBUS, Spektrum, PPM, PWM; tay phát DIY ESP32-C5 dùng nRF24 |
| **Ground station** | Telemetry và điều khiển MAVLink 2 (nhiệm vụ, chế độ, arm / disarm, chỉnh PID / rate / bộ lọc bằng tham số) - dùng được với Mission Planner và QGroundControl; OSD qua MSP DisplayPort; telemetry CRSF |
| **Ghi log** | Blackbox vào flash trong chip hoặc thẻ SD, truyền log trực tiếp, phân tích đáp ứng bước PID ngay trong app |
| **App cấu hình** | Chạy trong trình duyệt (tiếng Anh / tiếng Việt): mọi cài đặt, xem cảm biến trực tiếp, thử motor và wizard, hiệu chuẩn, lập kế hoạch nhiệm vụ trên bản đồ, xem log. Nói chuyện được với firmware cùng phiên bản **và mọi bản cũ hơn** |

## Bắt đầu

**Nạp firmware** - ngay trên trình duyệt: [trang nạp firmware](https://ohminecraft.github.io/ProjectWindburst_Re/) (mạch ESP32), hoặc bằng PlatformIO:

```
pio run -e rx_esp32_s3 -t upload        # flight controller ESP32-S3 N16R8 (rx_esp32: ESP32 đời đầu, tx_c5: tay phát DIY)
```

**Cấu hình** - app cấu hình (Python 3, pyserial):

```
cd app && pip install -r requirements.txt
python -m wbclient gui                   # mở trang; chọn cổng và baud ngay trong trang
python -m wbclient --port COM5 info      # hoặc dòng lệnh: cùng những thao tác đó
```

Đường truyền là nhị phân: đóng serial monitor trước. Sơ đồ chân đặt ở tab "Chân và cổng" (có bộ chân gợi ý cho DevKitC N16R8).

**Chưa có mạch?** Firmware thật chạy trên một mạch ảo:

```
make sim && build/wb_sim                 # rồi chọn "Mô phỏng wb_sim" trong danh sách cổng của app
make test                                # toàn bộ test firmware trên PC (g++), kể cả firmware-in-the-loop
```

## Mạch hỗ trợ

| Mạch | Trạng thái |
|---|---|
| ESP32-S3 (N16R8 / N8) | **mạch chính** |
| ESP32 đời đầu (4 MB) | build được; ít tính năng vừa hơn |
| ESP32-C5 | tay phát DIY |
| STM32 F405 / F412 / G4 / H5 / C5 | ⚠️ **thử nghiệm** |

> ⚠️ **STM32 còn ở trong giai đoạn thử nghiệm.** Các bản STM32 có để mã nguồn giữ được tính di động, nhưng được kiểm tra ít hơn nhiều so với bản
> ESP32-S3. Nếu bạn định bay một flight controller STM32, hãy dùng firmware đã có nhiều năm bay thực tế:
> [Betaflight](https://betaflight.com), [ArduPilot](https://ardupilot.org), [INAV](https://github.com/iNavFlight/inav) hoặc [PX4](https://px4.io).

## Tài liệu

[docs/vi/ARCHITECTURE.md](docs/vi/ARCHITECTURE.md) là mục lục. Tài liệu tiếng Việt nằm ở [docs/vi](docs/vi), bản tiếng Anh (bản chính) ở [docs](docs).
Cuối mỗi tài liệu tính năng có phần đã kiểm chứng và chưa kiểm chứng.

## Đóng góp

Thấy thú vị? Rất mong được chung tay để firmware tốt hơn - bay thử và gửi log, mạch để thử, đối chiếu với firmware gốc, driver,
tài liệu và bản dịch. Mở issue hoặc pull request. Vài quy ước:

- mỗi thay đổi đi kèm test của nó (`make test`; test của app: `make apptest`), và tài liệu tính năng ghi rõ cái gì đã kiểm chứng;
- thuật toán theo một nguồn có tên (Betaflight, ArduPilot, PX4, INAV) - ghi rõ đã đọc file nào;
- giao thức chỉ có một nguồn: `protocol/wbp_schema.json` (`python tools/gen_proto.py` sinh phần của firmware và của app).

## Ghi công

Windburst đứng trên công sức của các dự án sau. Chỗ nào chuyển mã sang thì file đó ghi rõ nguồn.

- [Betaflight](https://github.com/betaflight/betaflight) - vòng rate, PID, bộ lọc, mixer, DShot, RC smoothing, MSP, crash flip, CRSF, OSD (GPL-3.0)
- [ArduPilot](https://github.com/ArduPilot/ardupilot) - Altitude Hold, Loiter, Position Hold, phát hiện hạ cánh, độ cao baro, optical flow của EKF3, nhiệm vụ (GPL-3.0)
- [PX4 Autopilot](https://github.com/PX4/PX4-Autopilot) - khung failsafe, optical flow của EKF2, driver cảm biến (BSD-3-Clause)
- [INAV](https://github.com/iNavFlight/inav) - hiệu chuẩn optical flow, cảm biến MSP, dẫn đường (GPL-3.0)
- [ExpressLRS](https://github.com/ExpressLRS/ExpressLRS) - CRSF và chế độ link MAVLink (GPL-3.0)
- [MAVLink](https://github.com/mavlink/mavlink) và [pymavlink](https://github.com/ArduPilot/pymavlink) - giao thức và vector kiểm tra
- [DroneCAN](https://dronecan.github.io) - cảm biến và ESC trên bus CAN
- [esp-fc](https://github.com/rtlopez/esp-fc) - ý tưởng flight controller trên ESP32 và cách bố trí chân
- [Bitcraze Crazyflie firmware](https://github.com/bitcraze/crazyflie-firmware) - cách xử lý PMW3901
- [Bluejay](https://github.com/bird-sanctuary/bluejay), [AM32](https://github.com/am32-firmware/AM32), BLHeli - phía ESC của DShot
- [ESP Web Tools](https://github.com/esphome/esp-web-tools) - trang nạp firmware trên web

## Giấy phép

[GPL-3.0](LICENSE). Windburst có mã chuyển từ các dự án GPL-3.0 (Betaflight, ArduPilot, INAV), nên toàn bộ firmware theo GPL-3.0.
