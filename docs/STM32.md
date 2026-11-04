# STM32 (H5, G4, F4, C5) and plain ESP32: MCUs other than the ESP32-S3 (0.25, DShot 0.26, complete 0.27)

The same source code as the ESP32-S3 build, still on Arduino (STM32duino 4.x), not rewritten in plain HAL. The chip-independent parts (FlightCore, EKF, filters, settings, the app protocol, blackbox, MSP / ESC passthrough) are used unchanged. Only the parts that touch hardware have STM32-specific versions, gathered in `src/port/`.

**Not yet tried on a board.** The builds below only compile so far (`pio run -e <env>`). The shared parts have passed tests on the PC.

## 1. MCUs and status

| env | MCU (package) | build | Static RAM / total | Settings | Blackbox in the chip's flash | Notes |
|---|---|---|---|---|---|---|
| `rx_esp32_s3` | ESP32-S3 (N16R8) | yes | | NVS | partition `wblog` | main build |
| `rx_esp32` | ESP32 (WROOM-32) | yes | 79 / 320 kB | NVS | partition (4 MB table) | see section 7 |
| `rx_h563` | STM32H563ZI (144 pins, Nucleo) | yes | 112 / 640 kB | bank 2 | bank 2, ~960 kB | |
| `rx_h562` | STM32H562RG (64 pins) | yes | 112 / 640 kB | bank 2 | bank 2, ~448 kB | |
| `rx_h523` | STM32H523CE (48 pins) | yes | 112 / 272 kB | last 32 kB of bank 2 | no: the firmware (~340 kB) is larger than bank 1 (256 kB), runs from bank 2 too | ST's sample linker script (the variant has none) |
| `rx_g473` | STM32G473CE (48 pins) | yes | 94 / 128 kB | last 16 kB of bank 2 | no: like H523 (2 x 256 kB) | little RAM: see section 5 |
| `rx_g491` | STM32G491RE (64 pins) | yes | 94 / 112 kB | last 16 kB of flash | no | **RAM very tight**: 21 kB left for the 22.5 kB of stack of the 3 tasks, the tasks may fail to start |
| `rx_f405` | STM32F405RG (64 pins) | yes | 94 / 128 kB | last 2 sectors of 128 kB | no | 8 MHz crystal (`WB_HSE_MHZ`), see section 5 |
| `rx_f412` | STM32F412RG (64 pins) | yes | 112 / 256 kB | last 2 sectors of 128 kB | no | 96 MHz (so USB gets exactly 48 MHz) |
| `rx_g431` | STM32G431CB | **no** | | | | only 128 kB flash, 32 kB RAM: the firmware needs ~340 kB flash, ~94 kB RAM |
| `rx_c552` | STM32C552CE | **no** | | | | STM32duino 4.30000 cannot yet compile the Arduino core for C5 (HAL2): errors right in `uart.h`, `stm32c5xx_hal.c`. Windburst's C5 code (flash bank 2, quad-word) is already written, waiting for STM32duino |

One env per chip. Where PlatformIO lacks a board, there is a file in `boards/` (G473CE, G491RE, H523CE, H562RG, C552CE). To use a different package of the same chip, edit `build.variant`, `build.mcu`, `build.arduino.board` in that board file, following the directory names in `framework-arduinoststm32/variants/`.

Common to every STM32:
- Control loop: TIM7 wakes the `wb_flight` task.
- App connection: the chip's USB CDC (PA11 / PA12). C5 alone must use a UART.
- SPI IMU, I2C baro / compass / lidar: yes.
- Serial receivers, PPM / PWM, GPS, MSP optical flow: yes.
- PWM / Oneshot / Multishot / brushed motors: yes, using the pin's timer.
- DShot150 / 300 / 600 / 1200 motors: one-way (0.26) and bidirectional with RPM (0.27), section 12.
- ESC telemetry on a separate wire (KISS, UART RX): yes, as on ESP32.
- Servos, beeper: yes.
- ESC passthrough (BLHeliSuite / ESC Configurator / AM32): yes. Bit-banged on the motor pin with `digitalWriteFast`, like ESP32.
- Battery / current measurement: yes.
- SD card for blackbox (SPI mode, SdFat library, FAT16 / FAT32 / exFAT): yes (0.27, section 13).
- WS2812 LED strip on a timer channel + DMA: yes (0.27, section 13).
- DroneCAN: FDCAN (G4 / H5) or bxCAN (F4), yes (0.27, section 13).
- **Not yet:**
  - ProShot.
  - SDMMC / SDIO SD card (SPI mode only).
  - External SPI flash for blackbox.
  - Keeping the ESC passthrough session journal across reboots (ESP32 uses RTC RAM; STM32 not yet).

## 2. The MCU in the app
- **The firmware reports the MCU name** in DEVICE_INFO (`[name]\0[MCU]\0[32 bytes]`, `wbp::Server::set_chip`). The last 32 bytes are a mask: bit n = pin number n exists.
  - On STM32, the pin list is taken from the STM32duino variant's `digitalPin[]`, i.e. exactly the pins of that chip package.
  - On ESP32, the list is written in `port/chip.h`.
- **App:**
  - The top bar has an MCU label (`#mcu`) next to the firmware version.
  - The Pins and ports tab only offers pins that exist on the chip.
  - Pins that do not exist on the chip get a warning: "... does not exist on the STM32F405".
- **Old firmware** (before 0.25) does not report the MCU: the app shows "STM32" or "ESP32" and allows any pin.
- **The boot report** has the line `mcu: STM32F405, 168 MHz, crystal , flash 1024 kB (firmware 324 kB), settings in the end of the flash at 0x080c0000`. ESP32 version: `mcu: ESP32 rev 3, 240 MHz, flash 4 MB, PSRAM 0 kB`.

## 3. Pins
In the settings, a pin is one byte. On STM32 that byte is **port × 16 + pin**: PA0 = 0, PA15 = 15, PB0 = 16, PB3 = 19 ... PK15 = 175 (same as STM32duino's `PinName`).

**Pins tab on STM32: pins are chosen in two steps.**
1. Choose the port (PA, PB, ...). Only the ports the chip package has are offered.
2. Choose a pin of that port. Choosing only the port writes nothing yet; it is written once a pin is chosen.

Switching to another port leaves the old pin empty until a new pin is chosen. Choosing "empty" in the port field clears the pin.

Differences from ESP32:
- **UART, SPI, I2C cannot be placed on just any pin.** Each peripheral can only reach certain pins (STM32duino's `PinMap_*` tables, per the datasheet). The firmware picks the peripheral from the pin. The RX and TX of one UART, the three SCK / MISO / MOSI pins, the two SDA / SCL pins must belong to the same peripheral; otherwise that module reports an error in the boot report.
- **Receiver wired on RX only (no telemetry):** the STM32 driver still needs a TX pin at start-up. The firmware borrows a TX pin of the same UART that no function in the Pins tab uses (and that is not USB / SWD), then disables the transmitter and returns that pin to input. If every TX pin of that UART is in use, the receiver reports an error.
- **A pin's second "alternate" function** (for example PA5 is both SPI1 and SPI6, `PA_5_ALT1` in STM32duino): a one-byte pin cannot record that choice, so the firmware always uses the first peripheral in the table.
- **Motors, servos and the passive buzzer use the pin's timer channel.** Channels on the same timer run at the same frequency, so a 50 Hz servo cannot share a timer with a 480 Hz motor. The firmware refuses rather than changing the other output's frequency, and the report says so.
- **PA11 / PA12 are USB** (the app talks over them). **PA13 / PA14 are the SWD debug port:** usable, but then a programmer cannot be attached while the firmware is running. The Pins tab warns about both.

## 4. Flash: settings and blackbox (`port/stm32/flash_stm32.h`)

| family | banks | area used | blackbox | write unit | erase unit |
|---|---|---|---|---|---|
| H5 | 2 | bank 2 (settings in the first 64 kB if the bank is 1 MB, 32 kB if smaller; the rest is the log) | yes | 16 B quad-word | 8 kB sector |
| C5 | 2 | bank 2 | yes | quad-word | 8 kB page |
| G4 | G473: 2 (DBANK, default), G491: 1 | G473: bank 2; G491: last 16 kB | G473 only | 2 double-words | 2 kB page |
| F4 | 1 | last 2 sectors of 128 kB | no | 4 words | 128 kB sector |

- **Two banks:** the firmware runs from bank 1 while bank 2 is written / erased, so the flight loop does not have to wait and the log may live there.
- **One bank:** every write / erase stalls the CPU (erasing an F4 sector takes 1 – 2 seconds). So that area only holds the settings, which are only written while on the ground.
- **Two banks but the firmware is larger than bank 1** (H523, G473, C552: 2 x 256 kB, firmware ~340 kB): code also runs from bank 2, so bank 2 behaves like a single bank. The firmware only takes the last 32 kB (G4: 16 kB) for settings, and there is no log (`FlashStm32::shares_bank_with_code`). The boot report states the reason. Blackbox on these chips needs an SD card or SPI flash (not done yet). The boot report says `blackbox: FAILED - no room for a log in the flash of the STM32F405 (one flash bank: ...)`.
- **Firmware reaching into the settings area** (the firmware image runs past the start of the area, `port::image_end()`): the firmware writes / erases nothing, and reports `settings: flash FAILED - the firmware is too big`.
- **ECC** (H5, C5, G4): a write interrupted by power loss can read back as a double ECC error, which the chip reports with an NMI. `NMI_Handler` catches that NMI, counts it, and that read returns "unreadable". Thanks to this the firmware does not hang on every boot. F4 flash has no ECC.
- **How writing works:** ECC flash can only be written once per quad-word after an erase, so the two layers above share the `QwFlash` interface for every family:
  - `kv_flash.h`: settings are sequential records, power loss at any step still leaves either the old value or the new one (`test_port` cuts power at 42 points).
  - `bb_qword.h`: the log.

## 5. Low-RAM chips (`config/ram_profile.h`, `WB_LOW_RAM`)
F405 (128 kB, plus 64 kB of CCM that DMA cannot reach), G473 (128 kB), G491 (112 kB). On these chips the firmware uses smaller buffers to leave room for the stacks of the three tasks:
- log queue 16 → 4 kB;
- app connection transmit queue 8 → 2 kB;
- `wb_flight` / `wb_sensor` / `wb_link` stacks 24 / 16 / 14 kB → 8 / 8 / 6 kB. **Not yet measured on a board.** Cortex-M needs much less stack than the ESP32's Xtensa, but it must be re-checked on the board.

If a task cannot be created, the boot report says `loop: FAILED to start the flight task` or `FAILED to start the sensor task`.

## 6. Clock
- **G4 / H5:** the generic variant's clock (HSI). USB uses HSI48.
- **F4:** a dedicated `SystemClock_Config` in `main_stm32.cpp`, using the `WB_HSE_MHZ` crystal (default 8 MHz). USB needs exactly 48 MHz, which an oscillator within ±1 % cannot guarantee. F405 runs at 168 MHz, F412 at 96 MHz. Without a crystal it falls back to HSI, and the boot report says `NO CRYSTAL`.

## 7. Plain ESP32 (`rx_esp32`)
- **App connection:** UART0 through the board's USB-UART chip, 230400 baud. The plain ESP32 has no native USB.
- **Pins:** GPIO 0 – 39, except 6 – 11 (flash) and the numbers the chip does not have (20, 24, 28 – 31). GPIO 34 – 39 are input-only; the app warns if a motor, servo, TX, ... is assigned to them. Voltage measurement only on GPIO 32 – 39 (ADC1).
- **RAM:** the Arduino core reserves the start of data RAM for Bluetooth, so `FlightApp` (~75 kB) lives in the internal heap (`heap_caps_malloc(MALLOC_CAP_INTERNAL)`), not as a static variable.
- **Code:** kept in flash (`-DWB_FAST_CODE=0`), because IRAM is not big enough.
- **DShot:** 8-channel RMT, each channel both transmits and receives; the bidirectional DShot code was fixed for this channel numbering (`kRxOffset`). Not yet tried.

## 8. Internals (`src/port/`)

| File | Job | ESP32 | STM32 |
|---|---|---|---|
| `platform.h` | which chip | `WB_PLATFORM_ESP32` | `WB_PLATFORM_STM32` |
| `chip.h` | MCU name, pins present on the chip | S3 / ESP32 list | the variant's `digitalPin[]` |
| `rtos.h` | task creation, priorities, critical sections, restart | 2 cores, stack in bytes | 1 core, stack in words, scheduler started by `setup()` |
| `uart.h` | UART by pin | HardwareSerial | `Uart` + peripheral chosen by pin |
| `pwm.h` | PWM on a pin | LEDC | HardwareTimer, checks for frequency clashes |
| `gpio.h` | pin numbers, ADC | GPIO | PinName, VREFINT |
| `pin_names.h` | pin names | GPIO12 | PB3 (portable, tested) |
| `qw_flash.h` | quad-word-write flash | | |
| `stm32/flash_stm32.h` | the chip's flash, UID, reset reason | | H5 / C5 / G4 / F4 |
| `stm32/flight_task_stm32.h` | control task | (`app/flight_task.h`) | TIM7 + FreeRTOS |

One core, three FreeRTOS tasks: `wb_flight` (highest) > `wb_sensor` > `wb_link`. UART / USB interrupts sit above the FreeRTOS level (they do not call FreeRTOS). The TIM7 interrupt is at level 6, just below FreeRTOS's limit of 5.

`WB_FAST` (`config/fast_code.h`) is empty by default on STM32: F4 / G4 / H5 run code from flash through the accelerator (ART / ICACHE). Before 0.25 it put code into a `.tcm_code` section, but the linker script has no such section, so no STM32 build compiled ("section type conflict"). To place code in RAM you need a linker script with that section, then set `WB_FAST_SECTION`.

## 9. Verification on the PC
- `test_port`:
  - pin names both ways;
  - settings on quad-word flash: rewriting, area compaction, 42 power cuts;
  - FlashLog on quad-word flash.
- `test_proto`: DEVICE_INFO with MCU and pin mask; pin lists of the S3 (N16R8 and without PSRAM), plain ESP32 (28 GPIO), the simulator's fake STM32.
- `test_fw`: the virtual board reports an ESP32-S3 and its pins.
- `app/tests/test_gui_pins.py`:
  - pin names in the app identical to the firmware;
  - parsing of new and old DEVICE_INFO.
- `test_gui_browser.py::test_pins_tab_stm32`: two-step pin selection, MCU label (the simulator plays an STM32H563, `wb_sim --stm32`). **Cannot run on this machine yet** (no playwright).
- Compiled with the real toolchain: every env in the table of section 1.

## 10. Board tests in order
1. **Flash and connect:** flash, plug in USB. The app connects and shows the right MCU name. Read the boot report: the `mcu:`, `settings:`, `loop:` lines. Save settings, reboot, and the settings are still there.
2. **Pins tab:** only the ports / pins of the chip package.
3. **SPI IMU:** the Sensors tab shows numbers, CPU % is reasonable.
4. **Receiver** on a UART.
5. **Motors:** PWM or Oneshot125, **props off**.
6. **Blackbox** (H5 / G473): record a session, download it.

## 12. DShot on STM32 (0.26, `port/stm32/dshot_stm32.h`, `hal/motor/dshot_burst.h`)
Follows Betaflight's "DMA burst" approach (`DSHOT_DMAR`). Each motor pin must be a timer channel (CH1 – CH4, not CHxN). Motors on the same timer share one DMA transfer.
- **Timer:** runs at exactly the bit rate (PSC 0, ARR = timer clock / bit rate − 1; DShot600 on the F405's TIM1 is 280 ticks per bit), channels in PWM mode 1 with preload.
- **DMA:** on every timer update, DMA writes one row of compare values into `TIMx->DMAR`, and the timer distributes it to the motor channels' CCRs.
  - 18 rows per frame: 16 bits (bit 1 high for 14/20 of the period, bit 0 high for 7/20, as in Betaflight), then 2 rows of 0 so the line stays low after the frame.
  - The CPU only fills the 18 rows and starts the DMA, no interrupts.
  - If a new frame is due before the previous one has finished, the new frame is dropped and counted.
- **DMA per family:**
  - F4: the fixed stream / channel from the reference manual for TIMx_UP (TIM1: DMA2 stream 5 channel 6, TIM2: DMA1 stream 1 channel 3, TIM3: DMA1 stream 2 channel 5, TIM4: DMA1 stream 6 channel 2, TIM5: DMA1 stream 0 channel 6, TIM8: DMA2 stream 1 channel 7).
  - G4: DMA1 channels 1 – 4, DMAMUX `DMA_REQUEST_TIMx_UP`.
  - H5: GPDMA1 channels 0 – 3, `GPDMA1_REQUEST_TIMx_UP`, request at the destination (`DREQ`).
- **A motor timer belongs entirely to DShot:** a servo or buzzer placed on another channel of the same timer is refused (`port::pwm_reserve_timer`).
- **Error reporting:** the boot report gives the reason when it cannot start, for example "a motor pin has no timer channel", "a motor's timer also runs a servo / the buzzer", "no DMA for the motor timer's update request".
- **One-way** (bidirectional DShot off): the "DMA burst" approach above.
- **Bidirectional (0.27):** uses exactly the ESP32's sequencing controller (`hal/motor/dshot_direct.h`, tested on the PC), only the hardware-facing part changes (`stm32dshot::TimDmaPort`):
  - One DMA channel per motor (request TIMx_CHy). Inverted line (idles high, bits are low pulses, CCxP = 1).
  - TIM6 one-shot, a few µs after the frame: its interrupt hands the line to the ESC.
  - The channel switches to input capture on both edges, the timer free-runs (ARR 0xFFFF). The same DMA channel copies the counter value on every edge.
  - Next frame: the edges become level segments (`motor::capture_to_words`, tested with all 4096 reply values, including when the counter wraps), decoded as on ESP32, RPM goes into the RPM filter.
  - This is Betaflight's approach when using timers (`pwm_output_dshot.c`, input capture + DMA).
  - F4: TIM4 CH4 has no DMA, so that pin cannot use bidirectional DShot.
  - There is no start-up check as on ESP32 (an STM32 pin cannot transmit and listen at the same time). If an ESC does not reply, the Motor tab says "not replying", as on ESP32.
- **Verification:**
  - `test_dshot` burst group: the frame's rows read back into the right packet, unused channels stay 0, low tail, period follows the clock.
  - 9 `burst:` mutants.
  - The register-facing part (timer, DMA) only compiles; **not yet tried on a board**, the waveform needs to be checked with an oscilloscope / logic analyzer before connecting ESCs.

## 13. SD card, LED strip, DroneCAN on STM32 (0.27)
- **SD card** (`blackbox/bb_card_stm32.h`):
  - SPI mode, SdFat library (added to `lib_deps`). Four pins sd_cs / sd_clk / sd_cmd (MOSI) / sd_d0 (MISO); SCK / MOSI / MISO must be pins of the same SPI peripheral, otherwise the reason is reported.
  - Same directory and file names as on ESP32 (`/wblog/wb_00001.wblog`), exFAT cards work.
  - The `wb_sd` writer task has the lowest priority, 32 kB buffer (low-RAM chips: 8 kB).
  - Low-RAM chips only run this task when the SD card is selected at boot; selecting the card later in the app requires a reboot.
  - `blackbox.sd_mode = SDMMC` is refused with a reason (SDMMC not done on STM32 yet).
- **LED strip** (`port/stm32/ws2812_stm32.h`):
  - The LED pin must be a timer channel. The timer runs at 800 kHz, DMA writes a compare value for each bit (`led::timer_rows`, bit 1 high 2/3, bit 0 high 1/3, GRB order).
  - The timer is reserved for the LED strip: it cannot be shared with motors, servos, the buzzer.
  - The row memory (3 kB) is only allocated when there is an LED strip.
- **DroneCAN** (`hal/can/stm32_can.h`):
  - G4 / H5: FDCAN in classic CAN mode. Kernel clock: PCLK1 on G4, PLL1Q on H5.
  - F4: bxCAN CAN1 / CAN2.
  - Controller chosen by pin (`PinMap_CAN_TD / RD`). 29-bit frames, no interrupts (polled by the sensor loop). Bus-off recovers by itself and is counted.
  - Bit timing (`hal/can/can_timing.h`, tested in `test_can`): 8 – 25 quanta, 87.5 % sample point, error below 0.5 %.
  - Needs the HAL CAN module: `platformio.ini` adds `-DHAL_FDCAN_MODULE_ENABLED` (G4 / H5) or `-DHAL_CAN_MODULE_ENABLED` (F4).
- **RAM:** these features add about 8 kB of static RAM. G473 / F405 have about 29 kB left for stacks and heap; G491 only about 12 kB, less than the 22.5 kB of stack of the 3 tasks, so the tasks almost certainly cannot start.

## 11. Next
- SDMMC / SDIO SD card, external SPI flash for blackbox on single-bank chips.
- Keep the ESC passthrough session journal across reboots.
- Measure the real stacks of the three tasks; reduce RAM for G491.
- C5 once STM32duino can compile it.
