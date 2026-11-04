# STM32 (H5, G4, F4, C5) và ESP32 thường: các MCU ngoài ESP32-S3 (0.25, DShot 0.26, đầy đủ 0.27)

Cùng mã nguồn với bản ESP32-S3, vẫn trên Arduino (STM32duino 4.x), không viết lại bằng HAL thuần. Phần không phụ thuộc chip (FlightCore, EKF, bộ lọc, settings, giao thức với app, blackbox, MSP / ESC passthrough) dùng y nguyên. Chỉ phần chạm vào phần cứng có bản riêng cho STM32, gom trong `src/port/`.

**Chưa thử trên mạch.** Các bản dưới đây mới biên dịch được (`pio run -e <env>`). Các phần dùng chung đã qua test trên PC.

## 1. Các MCU và trạng thái

| env | MCU (vỏ) | build | RAM tĩnh / tổng | Settings | Blackbox trong flash của chip | Ghi chú |
|---|---|---|---|---|---|---|
| `rx_esp32_s3` | ESP32-S3 (N16R8) | được | | NVS | partition `wblog` | bản chính |
| `rx_esp32` | ESP32 (WROOM-32) | được | 79 / 320 kB | NVS | partition (bảng 4 MB) | xem mục 7 |
| `rx_h563` | STM32H563ZI (144 chân, Nucleo) | được | 112 / 640 kB | bank 2 | bank 2, ~960 kB | |
| `rx_h562` | STM32H562RG (64 chân) | được | 112 / 640 kB | bank 2 | bank 2, ~448 kB | |
| `rx_h523` | STM32H523CE (48 chân) | được | 112 / 272 kB | 32 kB cuối bank 2 | không: firmware (~340 kB) lớn hơn bank 1 (256 kB), chạy cả từ bank 2 | linker script mẫu của ST (variant không có) |
| `rx_g473` | STM32G473CE (48 chân) | được | 94 / 128 kB | 16 kB cuối bank 2 | không: như H523 (2 x 256 kB) | RAM ít: xem mục 5 |
| `rx_g491` | STM32G491RE (64 chân) | được | 94 / 112 kB | 16 kB cuối flash | không | **RAM rất sát**: 21 kB còn lại cho 22,5 kB stack của 3 task, có thể không khởi động được task |
| `rx_f405` | STM32F405RG (64 chân) | được | 94 / 128 kB | 2 sector 128 kB cuối | không | thạch anh 8 MHz (`WB_HSE_MHZ`), xem mục 5 |
| `rx_f412` | STM32F412RG (64 chân) | được | 112 / 256 kB | 2 sector 128 kB cuối | không | 96 MHz (để USB đúng 48 MHz) |
| `rx_g431` | STM32G431CB | **không** | | | | chỉ 128 kB flash, 32 kB RAM: firmware cần ~340 kB flash, ~94 kB RAM |
| `rx_c552` | STM32C552CE | **không** | | | | STM32duino 4.30000 chưa biên dịch được lõi Arduino cho C5 (HAL2): lỗi ngay trong `uart.h`, `stm32c5xx_hal.c`. Mã của Windburst cho C5 (flash bank 2, quad-word) đã viết sẵn, chờ STM32duino |

Mỗi chip một env. Board của PlatformIO thiếu thì có file trong `boards/` (G473CE, G491RE, H523CE, H562RG, C552CE). Muốn dùng vỏ khác cùng chip thì sửa `build.variant`, `build.mcu`, `build.arduino.board` trong file board đó, theo tên thư mục trong `framework-arduinoststm32/variants/`.

Phần chung trên mọi STM32:
- Vòng điều khiển: TIM7 đánh thức task `wb_flight`.
- Kết nối app: USB CDC của chip (PA11 / PA12). Riêng C5 phải dùng UART.
- IMU SPI, baro / la bàn / lidar I2C: có.
- Receiver nối tiếp, PPM / PWM, GPS, optical flow MSP: có.
- Motor PWM / Oneshot / Multishot / brushed: có, dùng timer của chân.
- Motor DShot150 / 300 / 600 / 1200: một chiều (0.26) và hai chiều có RPM (0.27), mục 12.
- ESC telemetry trên dây riêng (KISS, UART RX): có, như ESP32.
- Servo, còi: có.
- ESC passthrough (BLHeliSuite / ESC Configurator / AM32): có. Bit-bang trên chân motor bằng `digitalWriteFast`, giống ESP32.
- Đo pin / dòng: có.
- Thẻ SD cho blackbox (kiểu SPI, thư viện SdFat, FAT16 / FAT32 / exFAT): có (0.27, mục 13).
- Dải LED WS2812 trên kênh timer + DMA: có (0.27, mục 13).
- DroneCAN: FDCAN (G4 / H5) hoặc bxCAN (F4), có (0.27, mục 13).
- **Chưa có:**
  - ProShot.
  - Thẻ SD kiểu SDMMC / SDIO (chỉ kiểu SPI).
  - Flash SPI ngoài cho blackbox.
  - Lưu nhật ký phiên ESC passthrough qua lần khởi động lại (ESP32 dùng RTC RAM; STM32 chưa).

## 2. MCU trong app
- **Firmware báo tên MCU** trong DEVICE_INFO (`[tên]\0[MCU]\0[32 byte]`, `wbp::Server::set_chip`). 32 byte cuối là mặt nạ: bit n = có chân số n.
  - Với STM32, danh sách chân lấy từ `digitalPin[]` của variant STM32duino, tức đúng các chân của vỏ chip đó.
  - Với ESP32, danh sách viết trong `port/chip.h`.
- **App:**
  - Thanh trên cùng có nhãn MCU (`#mcu`) cạnh phiên bản firmware.
  - Tab Chân và cổng chỉ cho chọn chân có trên chip.
  - Chân không có trên chip bị cảnh báo: "... không có trên STM32F405".
- **Firmware cũ** (trước 0.25) không báo MCU: app ghi "STM32" hoặc "ESP32" và cho chọn mọi chân.
- **Báo cáo khởi động** có dòng `mcu: STM32F405, 168 MHz, crystal , flash 1024 kB (firmware 324 kB), settings in the end of the flash at 0x080c0000`. Bản ESP32: `mcu: ESP32 rev 3, 240 MHz, flash 4 MB, PSRAM 0 kB`.

## 3. Chân
Trong settings, một chân là một byte. Trên STM32 byte đó là **cổng × 16 + chân**: PA0 = 0, PA15 = 15, PB0 = 16, PB3 = 19 ... PK15 = 175 (giống `PinName` của STM32duino).

**Tab Chân trên STM32: chọn chân theo hai bước.**
1. Chọn cổng (PA, PB, ...). Chỉ có các cổng mà vỏ chip có.
2. Chọn chân của cổng đó. Chọn cổng thôi thì chưa ghi gì; ghi khi đã chọn chân.

Đổi sang cổng khác thì chân cũ bỏ trống cho tới khi chọn chân mới. Chọn "trống" ở ô cổng là bỏ chân.

Khác ESP32:
- **UART, SPI, I2C không đặt được lên chân bất kỳ.** Mỗi khối chỉ ra được một số chân nhất định (bảng `PinMap_*` của STM32duino theo datasheet). Firmware chọn khối theo chân. RX và TX của một UART, ba chân SCK / MISO / MOSI, hai chân SDA / SCL phải cùng một khối; sai thì module đó báo lỗi trong báo cáo khởi động.
- **Receiver chỉ nối RX (không telemetry):** driver STM32 vẫn cần một chân TX lúc khởi động. Firmware mượn một chân TX của cùng UART mà không chức năng nào trong tab Chân dùng (và không phải USB / SWD), rồi tắt bộ phát và trả chân đó về input. Nếu mọi chân TX của UART đó đều đang dùng, receiver báo lỗi.
- **Chức năng "thay thế" thứ hai của chân** (ví dụ PA5 vừa là SPI1 vừa là SPI6, trong STM32duino là `PA_5_ALT1`): một byte chân không ghi được lựa chọn đó, nên firmware luôn dùng khối đầu tiên trong bảng.
- **Motor, servo và còi thụ động dùng kênh timer của chân.** Các kênh trên cùng một timer chạy cùng một tần số, nên servo 50 Hz không chung timer với motor 480 Hz được. Firmware từ chối thay vì đổi tần số của output kia, và báo cáo ghi rõ.
- **PA11 / PA12 là USB** (app nói chuyện qua đó). **PA13 / PA14 là cổng debug SWD:** dùng được, nhưng khi đó không gắn mạch nạp được lúc firmware đang chạy. Tab Chân cảnh báo cả hai.

## 4. Flash: settings và blackbox (`port/stm32/flash_stm32.h`)

| họ | bank | vùng dùng | blackbox | đơn vị ghi | đơn vị xoá |
|---|---|---|---|---|---|
| H5 | 2 | bank 2 (settings 64 kB đầu nếu bank 1 MB, 32 kB nếu nhỏ hơn; còn lại là log) | có | quad-word 16 B | sector 8 kB |
| C5 | 2 | bank 2 | có | quad-word | trang 8 kB |
| G4 | G473: 2 (DBANK, mặc định), G491: 1 | G473: bank 2; G491: 16 kB cuối | chỉ G473 | 2 double-word | trang 2 kB |
| F4 | 1 | 2 sector 128 kB cuối | không | 4 word | sector 128 kB |

- **Hai bank:** firmware chạy từ bank 1 trong lúc ghi / xoá bank 2, nên vòng bay không phải chờ và log được phép nằm ở đó.
- **Một bank:** mỗi lần ghi / xoá làm CPU đứng (xoá một sector F4 mất 1 – 2 giây). Vì vậy vùng đó chỉ giữ settings, vốn chỉ ghi khi ở dưới đất.
- **Hai bank nhưng firmware lớn hơn bank 1** (H523, G473, C552: 2 x 256 kB, firmware ~340 kB): code chạy cả từ bank 2, nên bank 2 cũng như một bank. Firmware chỉ lấy 32 kB cuối (G4: 16 kB) cho settings, và không có log (`FlashStm32::shares_bank_with_code`). Báo cáo khởi động ghi lý do. Blackbox trên các chip này cần thẻ SD hoặc flash SPI (chưa làm). Báo cáo khởi động ghi `blackbox: FAILED - no room for a log in the flash of the STM32F405 (one flash bank: ...)`.
- **Firmware chạm vùng settings** (ảnh firmware dài quá đầu vùng, `port::image_end()`): firmware không ghi / xoá gì, và báo `settings: flash FAILED - the firmware is too big`.
- **ECC** (H5, C5, G4): một lần ghi bị mất điện giữa chừng có thể đọc ra lỗi ECC kép, và chip báo bằng NMI. `NMI_Handler` bắt NMI đó, đếm lại, và lần đọc đó trả về "không đọc được". Nhờ vậy firmware không treo mỗi lần khởi động. Flash của F4 không có ECC.
- **Cách ghi:** flash có ECC chỉ ghi được một lần cho mỗi quad-word sau khi xoá, nên hai lớp ở trên dùng chung giao diện `QwFlash` cho mọi họ:
  - `kv_flash.h`: settings là các bản ghi nối tiếp, mất điện ở bất kỳ bước nào cũng còn giá trị cũ hoặc giá trị mới (`test_port` cắt điện ở 42 thời điểm).
  - `bb_qword.h`: log.

## 5. Chip ít RAM (`config/ram_profile.h`, `WB_LOW_RAM`)
F405 (128 kB, cộng 64 kB CCM mà DMA không với tới), G473 (128 kB), G491 (112 kB). Trên các chip này firmware dùng bộ đệm nhỏ hơn để còn chỗ cho stack của ba task:
- hàng đợi log 16 → 4 kB;
- hàng đợi gửi của kết nối app 8 → 2 kB;
- stack `wb_flight` / `wb_sensor` / `wb_link` 24 / 16 / 14 kB → 8 / 8 / 6 kB. **Chưa đo trên mạch.** Cortex-M cần stack ít hơn Xtensa của ESP32 nhiều, nhưng phải xem lại trên board.

Không tạo được task thì báo cáo khởi động ghi `loop: FAILED to start the flight task` hoặc `FAILED to start the sensor task`.

## 6. Clock
- **G4 / H5:** clock của variant generic (HSI). USB lấy HSI48.
- **F4:** `SystemClock_Config` riêng trong `main_stm32.cpp`, dùng thạch anh `WB_HSE_MHZ` (mặc định 8 MHz). USB cần đúng 48 MHz, mà dao động trong ±1 % không đảm bảo được. F405 chạy 168 MHz, F412 chạy 96 MHz. Không có thạch anh thì quay về HSI, và báo cáo khởi động ghi `NO CRYSTAL`.

## 7. ESP32 thường (`rx_esp32`)
- **Kết nối app:** UART0 qua mạch USB-UART của board, 230400 baud. ESP32 thường không có USB riêng.
- **Chân:** GPIO 0 – 39, trừ 6 – 11 (flash) và các số chip không có (20, 24, 28 – 31). GPIO 34 – 39 chỉ là chân vào; app cảnh báo nếu gán motor, servo, TX, ... vào đó. Đo điện áp chỉ ở GPIO 32 – 39 (ADC1).
- **RAM:** lõi Arduino giữ phần đầu RAM dữ liệu cho Bluetooth, nên `FlightApp` (~75 kB) nằm trong heap nội (`heap_caps_malloc(MALLOC_CAP_INTERNAL)`), không phải biến tĩnh.
- **Code:** để trong flash (`-DWB_FAST_CODE=0`), vì IRAM không đủ.
- **DShot:** RMT 8 kênh, mỗi kênh vừa phát vừa thu; mã DShot hai chiều đã sửa cho cách đánh số kênh này (`kRxOffset`). Chưa thử.

## 8. Bên trong (`src/port/`)

| File | Việc | ESP32 | STM32 |
|---|---|---|---|
| `platform.h` | chip nào | `WB_PLATFORM_ESP32` | `WB_PLATFORM_STM32` |
| `chip.h` | tên MCU, các chân có trên chip | danh sách S3 / ESP32 | `digitalPin[]` của variant |
| `rtos.h` | tạo task, độ ưu tiên, critical section, restart | 2 nhân, stack theo byte | 1 nhân, stack theo word, scheduler do `setup()` khởi động |
| `uart.h` | UART theo chân | HardwareSerial | `Uart` + chọn khối theo chân |
| `pwm.h` | PWM trên chân | LEDC | HardwareTimer, kiểm tra trùng tần số |
| `gpio.h` | số chân, ADC | GPIO | PinName, VREFINT |
| `pin_names.h` | tên chân | GPIO12 | PB3 (portable, có test) |
| `qw_flash.h` | flash ghi theo quad-word | | |
| `stm32/flash_stm32.h` | flash của chip, UID, lý do reset | | H5 / C5 / G4 / F4 |
| `stm32/flight_task_stm32.h` | task điều khiển | (`app/flight_task.h`) | TIM7 + FreeRTOS |

Một nhân, ba task FreeRTOS: `wb_flight` (cao nhất) > `wb_sensor` > `wb_link`. Ngắt UART / USB nằm trên mức FreeRTOS (không gọi FreeRTOS). Ngắt TIM7 ở mức 6, ngay dưới giới hạn 5 của FreeRTOS.

`WB_FAST` (`config/fast_code.h`) trên STM32 mặc định để trống: F4 / G4 / H5 chạy code từ flash qua bộ tăng tốc (ART / ICACHE). Trước 0.25 nó đặt code vào section `.tcm_code`, nhưng linker script không có section đó, nên mọi bản STM32 đều không biên dịch được ("section type conflict"). Muốn đặt vào RAM thì cần linker script có section đó, rồi đặt `WB_FAST_SECTION`.

## 9. Kiểm chứng trên PC
- `test_port`:
  - tên chân hai chiều;
  - settings trên flash quad-word: ghi lại, nén vùng, 42 lần cắt điện;
  - FlashLog trên flash quad-word.
- `test_proto`: DEVICE_INFO có MCU và mặt nạ chân; danh sách chân của S3 (N16R8 và không có PSRAM), ESP32 thường (28 GPIO), STM32 giả của simulator.
- `test_fw`: mạch ảo báo ESP32-S3 và các chân của nó.
- `app/tests/test_gui_pins.py`:
  - tên chân trong app giống hệt firmware;
  - phân tích DEVICE_INFO mới và cũ.
- `test_gui_browser.py::test_pins_tab_stm32`: chọn chân hai bước, nhãn MCU (simulator đóng vai STM32H563, `wb_sim --stm32`). **Chưa chạy được ở máy này** (không có playwright).
- Biên dịch bằng toolchain thật: tất cả env trong bảng mục 1.

## 10. Thử trên mạch theo thứ tự
1. **Nạp và kết nối:** nạp, cắm USB. App kết nối và hiện đúng tên MCU. Đọc báo cáo khởi động: dòng `mcu:`, `settings:`, `loop:`. Lưu settings, khởi động lại thì settings còn.
2. **Tab Chân:** chỉ có các cổng / chân của vỏ chip.
3. **IMU SPI:** tab Cảm biến có số, CPU % hợp lý.
4. **Receiver** trên một UART.
5. **Motor:** PWM hoặc Oneshot125, **tháo cánh**.
6. **Blackbox** (H5 / G473): ghi một phiên, tải về.

## 12. DShot trên STM32 (0.26, `port/stm32/dshot_stm32.h`, `hal/motor/dshot_burst.h`)
Theo cách "DMA burst" của Betaflight (`DSHOT_DMAR`). Mỗi chân motor phải là một kênh timer (CH1 – CH4, không phải CHxN). Các motor cùng một timer dùng chung một lượt DMA.
- **Timer:** chạy đúng tốc độ bit (PSC 0, ARR = clock timer / tốc độ bit − 1; DShot600 trên TIM1 của F405 là 280 tick một bit), kênh ở chế độ PWM 1 có preload.
- **DMA:** mỗi lần timer update, DMA ghi một hàng giá trị so sánh vào `TIMx->DMAR`, timer chia ra CCR của các kênh motor.
  - 18 hàng mỗi frame: 16 bit (bit 1 cao 14/20 chu kỳ, bit 0 cao 7/20, như Betaflight), rồi 2 hàng 0 để dây nằm thấp sau frame.
  - CPU chỉ điền 18 hàng và khởi động DMA, không có ngắt.
  - Frame trước chưa xong mà tới lượt frame mới thì bỏ frame mới và đếm.
- **DMA theo họ:**
  - F4: stream / kênh cố định của reference manual cho TIMx_UP (TIM1: DMA2 stream 5 kênh 6, TIM2: DMA1 stream 1 kênh 3, TIM3: DMA1 stream 2 kênh 5, TIM4: DMA1 stream 6 kênh 2, TIM5: DMA1 stream 0 kênh 6, TIM8: DMA2 stream 1 kênh 7).
  - G4: kênh DMA1 1 – 4, DMAMUX `DMA_REQUEST_TIMx_UP`.
  - H5: kênh GPDMA1 0 – 3, `GPDMA1_REQUEST_TIMx_UP`, request ở đích (`DREQ`).
- **Timer của motor thuộc về DShot hoàn toàn:** servo hoặc còi đặt trên kênh khác của cùng timer bị từ chối (`port::pwm_reserve_timer`).
- **Báo lỗi:** báo cáo khởi động ghi lý do khi không bắt đầu được, ví dụ "a motor pin has no timer channel", "a motor's timer also runs a servo / the buzzer", "no DMA for the motor timer's update request".
- **Một chiều** (DShot hai chiều tắt): cách "DMA burst" ở trên.
- **Hai chiều (0.27):** dùng đúng bộ điều khiển trình tự của ESP32 (`hal/motor/dshot_direct.h`, đã test trên PC), chỉ đổi phần chạm phần cứng (`stm32dshot::TimDmaPort`):
  - Mỗi motor một kênh DMA (request TIMx_CHy). Dây đảo mức (nằm cao, bit là xung thấp, CCxP = 1).
  - TIM6 một lần, vài µs sau frame: ngắt của nó trả dây cho ESC.
  - Kênh chuyển sang input capture cả hai sườn, timer chạy tự do (ARR 0xFFFF). Cùng kênh DMA chép giá trị bộ đếm ở mỗi sườn.
  - Frame sau: các sườn thành các đoạn mức (`motor::capture_to_words`, test với cả 4096 giá trị trả lời, cả khi bộ đếm quay vòng), giải mã như ESP32, RPM vào bộ lọc RPM.
  - Đây là cách của Betaflight khi dùng timer (`pwm_output_dshot.c`, input capture + DMA).
  - F4: TIM4 CH4 không có DMA, nên chân đó không dùng được DShot hai chiều.
  - Không có kiểm tra lúc khởi động như ESP32 (chân STM32 không vừa phát vừa nghe được). ESC không trả lời thì tab Motor ghi "không trả lời", như ESP32.
- **Kiểm chứng:**
  - `test_dshot` nhóm burst: các hàng của frame đọc ngược lại ra đúng packet, kênh không dùng nằm 0, đuôi thấp, chu kỳ theo clock.
  - 9 mutant `burst:`.
  - Phần chạm thanh ghi (timer, DMA) chỉ biên dịch được; **chưa thử trên mạch**, cần xem dạng sóng bằng máy hiện sóng / logic analyzer trước khi gắn ESC.

## 13. Thẻ SD, dải LED, DroneCAN trên STM32 (0.27)
- **Thẻ SD** (`blackbox/bb_card_stm32.h`):
  - Kiểu SPI, thư viện SdFat (thêm vào `lib_deps`). Bốn chân sd_cs / sd_clk / sd_cmd (MOSI) / sd_d0 (MISO); SCK / MOSI / MISO phải là chân của cùng một khối SPI, sai thì báo lý do.
  - Cùng thư mục và tên file như ESP32 (`/wblog/wb_00001.wblog`), thẻ exFAT dùng được.
  - Task ghi `wb_sd` ưu tiên thấp nhất, bộ đệm 32 kB (chip ít RAM: 8 kB).
  - Chip ít RAM chỉ chạy task này khi chọn thẻ SD lúc khởi động; chọn thẻ sau đó trong app thì cần khởi động lại.
  - `blackbox.sd_mode = SDMMC` bị từ chối kèm lý do (chưa làm SDMMC trên STM32).
- **Dải LED** (`port/stm32/ws2812_stm32.h`):
  - Chân LED phải là kênh timer. Timer chạy 800 kHz, DMA ghi giá trị so sánh cho từng bit (`led::timer_rows`, bit 1 cao 2/3, bit 0 cao 1/3, thứ tự GRB).
  - Timer dành riêng cho dải LED: không được chung với motor, servo, còi.
  - Bộ nhớ hàng (3 kB) chỉ cấp khi có dải LED.
- **DroneCAN** (`hal/can/stm32_can.h`):
  - G4 / H5: FDCAN ở chế độ CAN thường. Clock nhân: PCLK1 trên G4, PLL1Q trên H5.
  - F4: bxCAN CAN1 / CAN2.
  - Controller theo chân (`PinMap_CAN_TD / RD`). Frame 29 bit, không ngắt (vòng cảm biến hỏi). Bus-off tự phục hồi và được đếm.
  - Thời gian bit (`hal/can/can_timing.h`, test trong `test_can`): 8 – 25 quanta, điểm lấy mẫu 87,5 %, sai số dưới 0,5 %.
  - Cần module CAN của HAL: `platformio.ini` thêm `-DHAL_FDCAN_MODULE_ENABLED` (G4 / H5) hoặc `-DHAL_CAN_MODULE_ENABLED` (F4).
- **RAM:** các tính năng này thêm khoảng 8 kB RAM tĩnh. G473 / F405 còn khoảng 29 kB cho stack và heap; G491 chỉ còn khoảng 12 kB, ít hơn 22,5 kB stack của 3 task, nên gần như chắc chắn không khởi động được task.

## 11. Tiếp theo
- Thẻ SD kiểu SDMMC / SDIO, flash SPI ngoài cho blackbox trên chip một bank.
- Lưu nhật ký phiên ESC passthrough qua lần khởi động lại.
- Đo stack thật của ba task; giảm RAM cho G491.
- C5 khi STM32duino biên dịch được.
