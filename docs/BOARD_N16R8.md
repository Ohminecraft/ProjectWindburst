# Mạch ESP32-S3 N16R8 — build, chân, thẻ SD, đo pin, lần chạy đầu

N16R8 = module ESP32-S3-WROOM-1 có **16 MB flash** (quad SPI) và **8 MB PSRAM** (octal SPI). Firmware đã đặt đây là mạch mặc định.

Bản 0.5 đã được bạn chạy trên mạch này (IMU ICM20602 qua SPI, ESC BLHeli_32, app qua USB của S3); những gì lộ ra và đã đổi: `docs/HARDWARE_FEEDBACK_1.md`. **Chưa có thay đổi nào của bản 0.6 được chạy trên mạch thật**: chúng đã biên dịch bằng toolchain ESP32-S3 thật và chạy trên mạch ảo (PC); phần "phải thử trên mạch" ở cuối là danh sách việc cần bạn làm.

## 1. Build và nạp

```
pio run -e rx_s3 -t upload          # N16R8 (mặc định, chỉ cần `pio run -t upload`)
pio run -e rx_s3_usb -t upload      # N16R8, app nối qua cổng USB của chính ESP32-S3 (USB Serial/JTAG)
pio run -e rx_s3_n8 -t upload       # module N8: 8 MB flash, không PSRAM
```
| | `rx_s3` (N16R8) | `rx_s3_n8` |
|---|---|---|
| board PlatformIO | `esp32-s3-devkitc1-n16r8` (flash QIO 16 MB, PSRAM octal) | `esp32-s3-devkitc-1` |
| bảng phân vùng | `partitions/wblog_16MB.csv`: app 2 MB, **log 14,2 MB**, NVS (settings) | mặc định của Arduino (phân vùng `spiffs` 1,5 MB làm log) |
| firmware (0.7.0, `rx_s3_usb`) | 676 kB flash (32 % của 2 MB), 100 kB RAM tĩnh, 115 kB mã trong RAM (trong đó 38,6 kB là chu kỳ điều khiển, xem `src/config/fast_code.h`) | tương đương (666 kB) |

- Nạp sai loại module không làm hỏng gì: firmware không khởi động được (sai kích thước flash / loại PSRAM), nạp lại đúng env là xong.
- Đổi bảng phân vùng làm mất log cũ trong flash; settings (NVS) giữ nguyên vị trí nên không mất.
- App nối qua **UART0** (cổng USB có chữ "UART" / "COM" trên DevKitC, qua chip cầu CP210x / CH34x) ở 230400 baud với `rx_s3`: đủ cho cấu hình, **không đủ** cho hình 3D mượt và log trực tiếp tốc độ cao.
- Cổng "USB" gốc của S3: env `rx_s3_usb` (`ARDUINO_USB_CDC_ON_BOOT=1`, `ARDUINO_USB_MODE=1`). Nhanh hơn nhiều; khi đó GPIO19 / 20 là của USB (gán chức năng khác lên hai chân này thì báo cáo khởi động có dòng cảnh báo). Kiểu TinyUSB (`ARDUINO_USB_MODE=0`) cũng biên dịch được, chưa ai thử.
- So sánh có / không có mã trong RAM: thêm `-DWB_FAST_CODE=0` vào `build_flags` của `[rx_common]`. Xem mã nào đang ở đâu: `python3 tools/iram_report.py .pio/build/rx_s3/firmware.elf` (cần `pip install pyelftools`).
- `lib_ldf_mode = deep` (không phải `deep+`): xem ghi chú trong `platformio.ini`.

## 2. Chân nào KHÔNG dùng được trên N16R8

| GPIO | Vì sao |
|---|---|
| 26 – 32 | flash SPI của module |
| **35, 36, 37** | PSRAM octal của module R8. (Module không có PSRAM octal thì dùng được.) |
| 0, 3, 45, 46 | chân strapping: mức lúc cấp nguồn quyết định chế độ khởi động |
| 19, 20 | USB D- / D+ |
| 43, 44 | UART0 TX / RX = đường nối app qua cầu USB-UART |
| 48 (DevKitC-1 v1.0) / 38 (v1.1) | LED RGB trên mạch |

Tab **Chân và cổng** cảnh báo các chân này ngay khi gõ. Firmware kiểm tra lần nữa lúc khởi động:
- hai chức năng trùng một GPIO → dòng `pins: FAILED - GPIO6 is set for both motor 3 and buzzer` trong báo cáo, và **không cho arm** (blocker `PIN_CONFLICT`). Lý do: ADC hay còi đặt nhầm lên chân motor sẽ lặng lẽ chiếm chân đó, motor ấy đứng yên lúc arm.
- chân thuộc flash / PSRAM của module → dòng `pins: WARNING - GPIO36 (sd_clk) belongs to the module's own flash / PSRAM`.
- Chỉ kiểm tra các chân **đang dùng**: chân GPS khi GPS bật, chân thẻ SD khi chọn ghi ra thẻ, số motor theo kiểu khung.

## 3. Bộ chân gợi ý (nút "Điền bộ chân gợi ý" ở tab Chân và cổng)

Không đụng chân nào ở bảng trên. Chỉ là điểm bắt đầu: **dây nối thật trên mạch của bạn mới quyết định**; nhập xong bấm "Lưu và khởi động lại".

| Chức năng | GPIO | Ghi chú |
|---|---|---|
| IMU SPI: CS / MOSI / SCK / MISO | 10 / 11 / 12 / 13 | bus SPI thứ nhất (SPI2 / FSPI) |
| I2C cảm biến: SDA / SCL | 8 / 9 | barometer, lidar, la bàn |
| GPS (UART1): RX / TX | 18 / 17 | RX của ESP32 nối TX của GPS |
| Receiver (UART2): RX / TX | 16 / 15 | CRSF / SBUS / IBUS |
| Motor 1 – 4 | 4, 5, 6, 7 | DShot dùng RMT, chân nào cũng được |
| ADC đo pin | **1** | ADC1 kênh 0; sau cầu chia áp |
| Thẻ SD: CS / SCK / MOSI / MISO | 42 / 39 / 40 / 41 | bus SPI thứ hai (SPI3) |
| Còi | (để trống) | motor kêu thay, xem `docs/POWER.md` |
| Optical flow (không có trong bộ gợi ý) | MSP qua UART: RX = **44** khi app nối qua USB của chip (`rx_s3_usb`: UART0 rảnh); PMW3901: 4 chân SPI riêng, ví dụ CS / SCK / MISO / MOSI = 2 / 47 / 21 / 14 (tránh 38 và 48: LED RGB của DevKitC; không chung được với thẻ SD kiểu SPI) | xem `docs/NAV.md` mục 1 |

## 4. Đo pin bằng ADC

- Cầu chia áp từ cực dương pin xuống GND, điểm giữa vào **GPIO1** (hoặc chân nào trong GPIO1 – 10 = ADC1; GPIO11 – 20 là ADC2 cũng đo được). Thêm tụ 100 nF từ điểm giữa xuống GND cho số đo đỡ nhảy.
- Điện áp ở chân **không được vượt khoảng 3,1 V** khi pin đầy. Cầu 10 kΩ : 1 kΩ (hệ số 11) dùng được tới 6S (25,2 V → 2,29 V). 
- Trong app: tab Chân và cổng nhập `ADC đo pin = 1`, lưu và khởi động lại; tab Pin và còi, mục "Hiệu chỉnh số đo": đo pin bằng đồng hồ, nhập số, bấm "Tính hệ số chia".
- Không muốn nối ADC: bật DShot hai chiều + "Extended DShot telemetry" ở tab Motor, firmware lấy điện áp do ESC báo (nguồn `AUTO` tự chọn ADC khi có chân, không thì ESC).

## 5. Thẻ SD

Chi tiết thiết kế ở `docs/BLACKBOX.md`. Việc cần làm:
1. Thẻ định dạng **FAT32** (thẻ ≥ 64 GB xuất xưởng là exFAT: phải định dạng lại).
2. Nối dây, kiểu **SPI** (hợp mọi module thẻ): CS, SCK, MOSI (chân DI của thẻ), MISO (chân DO), 3,3 V, GND. Dây càng ngắn càng tốt.
   Kiểu **SDMMC 1 bit** (3 dây CLK / CMD / D0, nhanh hơn): cần trở kéo lên 10 kΩ ở CMD và D0, chân DAT3 (CS) của thẻ nối lên 3,3 V qua trở.
3. Tab Chân và cổng: nhập 4 chân (hoặc 3 chân cho SDMMC), "Lưu và khởi động lại".
4. Tab Blackbox: "Ghi vào đâu" = Thẻ SD. Trạng thái phải thành "Sẵn sàng" trong vòng 3 giây; nếu "Không có bộ nhớ log" thì lý do nằm ở tab Tổng quan, dòng Blackbox. Dây dài: hạ "Tốc độ thẻ SD" xuống 4 – 10 MHz.
5. File log: `/wblog/wb_00001.wblog`… Rút thẻ cắm vào máy tính rồi "Thêm file .wblog…" ở tab Blackbox, hoặc tải qua cáp.

PSRAM 8 MB chỉ dùng cho bộ đệm ghi thẻ (512 kB). Vòng bay không bao giờ đụng PSRAM. Báo cáo khởi động có dòng `memory: … kB internal RAM free of …, PSRAM 8192 kB`; tab Tổng quan hiện CPU (nhân chạy vòng bay), RAM, PSRAM.

## 6. Lỗi đã sửa liên quan trực tiếp tới ESP32-S3

`src/hal/bus/bus_spi.h` (file của bạn) đặt số bus SPI là 2 và 3 cho mọi chip. Đó là số của ESP32 đời đầu (HSPI = 2, VSPI = 3). Trên ESP32-S3, core Arduino chỉ có hai bus đánh số **0 (FSPI) và 1 (HSPI)**; `SPIClass(2).begin()` trả về lỗi (`spiStartBus: spi_num >= SPI_COUNT`), nghĩa là **IMU qua SPI không bao giờ khởi động được trên S3**. Đã sửa thành `FSPI` / `HSPI` theo từng dòng chip (giữ nguyên 2 / 3 cho ESP32 đời đầu). Stub Arduino trong test trước đây chấp nhận mọi số bus nên không bắt được; nay nó từ chối số bus không tồn tại như core thật, và `test_fw` kiểm tra bus đã khởi động.

## 7. Phải thử trên mạch (theo thứ tự)

Danh sách của lượt này (vòng lặp, DShot, airmode, blackbox qua USB): **`docs/HARDWARE_FEEDBACK_1.md` mục 10**. Còn lại từ trước:

1. Tab Tổng quan: dòng `memory:` có `PSRAM 8192 kB` không.
2. Tháo cánh. Tab Pin và còi, bấm "Thử" ở "Gyro hiệu chuẩn xong": motor có kêu 3 tiếng không? Thử 5 âm.
3. Điện áp pin đọc đúng không (sau khi hiệu chỉnh hệ số chia).
4. Thẻ SD: trạng thái "Sẵn sàng"; arm không cánh 1 phút ở 1000 bản ghi/giây; xem `gap_us` (nhóm "Thời gian vòng lặp") có đứng yên ở 250 µs không, "Bộ đệm ghi thẻ" cao nhất bao nhiêu %, có bản ghi nào bị bỏ không. Rút pin giữa lúc ghi, cắm thẻ vào máy tính: file dài tới đâu.
5. So với ghi flash trên mạch: cùng phép thử, xem `gap_us`.
