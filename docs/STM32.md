# STM32H5 (0.10.0): bản đầu tiên

Cùng mã nguồn với bản ESP32-S3, vẫn trên Arduino (STM32duino 3.x), không viết lại bằng HAL thuần. Phần không phụ thuộc chip (FlightCore, EKF, bộ lọc, settings, giao thức với app, blackbox, MSP / ESC passthrough) dùng y nguyên. Chỉ phần chạm vào phần cứng có bản riêng cho STM32, gom trong `src/port/`.

**Chưa thử trên mạch.** Bản này mới biên dịch được (`pio run -e rx_h563`) và các phần dùng chung qua test trên PC.

## 1. Chạy được gì, chưa được gì

| | STM32H5 (0.10.0) | Ghi chú |
|---|---|---|
| Vòng điều khiển | có | TIM7 đánh thức task `wb_flight`, như timer của ESP32 |
| Kết nối app | USB CDC của chip (PA11 / PA12) | app chọn cổng COM như mọi khi |
| Settings | flash bank 2, 64 kB đầu | `proto/kv_flash.h`, chịu được mất điện giữa chừng (test) |
| Blackbox | flash bank 2, phần còn lại (~960 kB trên H563) | `blackbox/bb_qword.h`, cùng định dạng log |
| IMU SPI, baro / la bàn / lidar I2C | có | chân SPI / I2C phải thuộc cùng một khối SPI / I2C (xem mục 3) |
| Receiver nối tiếp, GPS, optical flow MSP | có | UART theo chân |
| Receiver PPM / PWM | có | ngắt ngoài trên chân |
| Motor PWM / Oneshot / Multishot / brushed | có | timer của chân (HardwareTimer) |
| **Motor DShot / ProShot** | **chưa** | cần timer + DMA; báo cáo khởi động ghi "DShot (not on STM32 yet ...)" và không ARM được |
| DShot hai chiều, ESC telemetry | chưa | đi theo DShot |
| Thẻ SD | chưa | |
| Servo, còi | có | timer của chân |
| ESC passthrough (BLHeliSuite32) | có trong mã | bit-bang bằng `digitalWriteFast`, chưa thử |
| Đo pin / dòng (ADC) | có | đo kèm VREFINT nên không phụ thuộc 3,3 V chính xác |

Chip: **STM32H563** trước (board NUCLEO-H563ZI, cũng là chip Betaflight có target). H523 / H533 dùng được cùng mã; chỉ cần board khác trong `platformio.ini` (flash bank 2 của chúng là 256 kB nên log nhỏ hơn).

## 2. Build và nạp
```
pio run -e rx_h563 -t upload        # NUCLEO-H563ZI: qua ST-LINK có sẵn trên board
```
Sau đó cắm cổng USB **user** của Nucleo (CN13, nối với PA11 / PA12) vào máy tính, app chọn cổng COM đó. Báo cáo khởi động (tab Tổng quan) có:
- `link: USB CDC`
- `settings: flash ok (gen 1, ...)` lần đầu: bank 2 trắng nên được định dạng
- `blackbox: flash bank 2 960 kB, 0 sessions`
- `motors: ...` và `pins: ...` nếu chân không đúng chức năng
- `reset: ...` lấy từ cờ RCC (bật nguồn / brown-out, watchdog, reset mềm, chân reset)

## 3. Chân
Trong settings, một chân là một byte, giống ESP32. Trên STM32 byte đó là **cổng × 16 + chân**: PA0 = 0, PA15 = 15, PB0 = 16, PB3 = 19 ... PK15 = 175 (giống `PinName` của STM32duino). Tab Chân của app tự đổi sang tên khi firmware báo là STM32 (cờ `STM32` trong thông tin thiết bị): gõ `PB3`, `pc13` hay `B3` đều được.

Khác ESP32:
- **UART, SPI, I2C không đặt được lên chân bất kỳ.** Mỗi khối chỉ ra được một số chân nhất định (bảng `PinMap_*` của STM32duino theo datasheet). Firmware chọn khối theo chân. RX và TX của một UART, ba chân SCK / MISO / MOSI, hai chân SDA / SCL phải cùng một khối; sai thì module đó báo lỗi trong báo cáo khởi động.
- Receiver chỉ nối RX (không telemetry): driver STM32 vẫn cần một chân TX lúc khởi động. Firmware mượn một chân TX của cùng UART mà không chức năng nào trong tab Chân dùng (và không phải USB / SWD), rồi tắt bộ phát và trả chân đó về input. Nếu mọi chân TX của UART đó đều đang dùng, receiver báo lỗi.
- Một số chân có chức năng "thay thế" thứ hai (ví dụ PA5 vừa là SPI1 vừa là SPI6; trong STM32duino là `PA_5_ALT1`). Một byte chân không ghi được lựa chọn đó, nên firmware luôn dùng khối đầu tiên trong bảng của STM32duino.
- **Motor, servo và còi thụ động dùng kênh timer của chân.** Các kênh trên cùng một timer chạy cùng một tần số. Servo 50 Hz không chung timer với motor 480 Hz được: firmware từ chối thay vì đổi tần số của output kia. Báo cáo ghi rõ.
- PA11 / PA12 là USB (app nói chuyện qua đó). PA13 / PA14 là cổng debug SWD: dùng được, nhưng khi đó không gắn mạch nạp / debug được lúc firmware đang chạy. Tab Chân cảnh báo cả hai.

## 4. Bên trong (`src/port/`)
| File | Việc | ESP32 | STM32 |
|---|---|---|---|
| `platform.h` | chip nào | `WB_PLATFORM_ESP32` | `WB_PLATFORM_STM32` |
| `rtos.h` | tạo task, độ ưu tiên, critical section, restart | 2 nhân, stack theo byte | 1 nhân, stack theo word, scheduler do `setup()` khởi động |
| `uart.h` | UART theo chân | HardwareSerial | `Uart` + chọn khối theo chân |
| `pwm.h` | PWM trên chân | LEDC | HardwareTimer, kiểm tra trùng tần số |
| `gpio.h` | số chân, ADC | GPIO | PinName, VREFINT |
| `pin_names.h` | tên chân | GPIO12 | PB3 (portable, có test) |
| `qw_flash.h` | flash ghi theo quad-word | | |
| `stm32/flash_h5.h` | flash bank 2 | | HAL, ECC, ICACHE |
| `stm32/flight_task_stm32.h` | task điều khiển | (`app/flight_task.h`) | TIM7 + FreeRTOS |

Flash của H5 có ECC trên từng quad-word (16 byte): mỗi quad-word chỉ ghi được **một lần** sau khi xoá, và xoá theo sector 8 kB. NVS và FlashLog của ESP32 dựa vào việc ghi thêm bit lên NOR flash, nên có hai lớp mới:
- `kv_flash.h`: settings là các bản ghi nối tiếp trong một nửa vùng. Hết chỗ thì chép bản mới nhất của mỗi key sang nửa kia, rồi mới ghi đầu nửa đó (đó là lúc "commit"). Mất điện ở bất kỳ bước nào: còn giá trị cũ hoặc giá trị mới, không bao giờ nửa vời. `test_port` cắt điện ở 42 thời điểm khác nhau để kiểm.
- `bb_qword.h`: FlashLog y như ESP32; quad-word chưa đầy được giữ trong RAM (tối đa 15 byte) cho tới khi có thêm byte hoặc phiên kết thúc.
- Quad-word bị mất điện lúc đang ghi có thể đọc ra lỗi ECC kép. H5 báo bằng NMI: `flash_h5.h` bắt NMI đó, đếm lại, và lần đọc đó trả về "không đọc được". Nhờ vậy firmware không treo mỗi lần khởi động.

Một nhân, ba task FreeRTOS: `wb_flight` (cao nhất) > `wb_sensor` > `wb_link`. Ngắt UART / USB nằm trên mức FreeRTOS (không gọi FreeRTOS). Ngắt TIM7 ở mức 6, ngay dưới giới hạn 5 của FreeRTOS.

## 5. Kiểm chứng trên PC
- `test_port`: tên chân hai chiều, settings trên flash quad-word (ghi lại, nén vùng, 42 lần cắt điện), FlashLog trên flash quad-word (không quad-word nào bị ghi hai lần, đọc lại từng byte, sau khởi động lại vẫn thấy phiên).
- `app/tests/test_gui_pins.py`: tên chân trong app (`js/pinname.js`) giống hệt firmware (`port/pin_names.h`) cho cả 256 giá trị.
- `test_gui_browser.py::test_pins_tab_stm32`: tab Chân với simulator đóng vai STM32 (`wb_sim --stm32`).
- Mọi test cũ chạy lại sau khi `hal/` chuyển sang `port/` (ESP32 vẫn build 5 cấu hình).

## 6. Thử trên mạch theo thứ tự
1. Nạp, cắm USB user: app kết nối, đọc báo cáo khởi động, lưu settings, khởi động lại thì settings còn.
2. IMU SPI (ví dụ ICM-42688-P trên SPI1: PA5 SCK, PA6 MISO, PB5 MOSI, CS bất kỳ): tab Cảm biến có số, tab Tổng quan "loop: flight task, TIM7 every 250 us", CPU % hợp lý.
3. Receiver CRSF trên một UART (ví dụ USART2: PD5 TX, PD6 RX trên Nucleo), kênh chạy trong tab Receiver.
4. Motor PWM hoặc Oneshot125 **tháo cánh**: motor test trong app.
5. Blackbox: ghi một phiên, tải về.

## 7. Tiếp theo
DShot qua timer + DMA (cách của Betaflight, cũng là cách duy nhất đủ nhanh trên STM32), rồi DShot hai chiều, thẻ SD. DroneCAN (FDCAN của H5) sau khi bản này bay được.
