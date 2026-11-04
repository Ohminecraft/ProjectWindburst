# ESC passthrough (0.8.8): BLHeliSuite32, ESC Configurator, AM32 Configurator

Same as Betaflight: the ESC configuration tool on the computer talks to the ESCs **through the flight controller**, over the motor signal wires. No need to remove the ESC, no special cable needed.

## 1. How to use it

1. Plug in the battery (the ESCs must be powered). **Remove the props** to be safe.
2. In the Windburst app, Motor tab, "ESC configuration" block: press **"Release port for BLHeliSuite32"** (or Disconnect). Only one program can open a port at a time.
3. Open BLHeliSuite32 / [esc-configurator.com](https://esc-configurator.com) / AM32 Configurator, select the board's COM port, press Connect then Read Setup.
4. When done, press Disconnect in that tool: the firmware reboots by itself and the motors run normally. If the tool is closed abruptly without Disconnect: after 5 minutes without any byte the firmware reboots by itself (or unplug the battery / USB).

Not possible while ARMED (the firmware answers "0 ESC"). During passthrough the motors receive no more commands; even if ARMED again they will not spin until a reboot.

### Error "Connection to Flightcontroller failed" (0.8.8 - 0.9.3, fixed in 0.9.4)
The real cause: BLHeliSuite32 32.10 (Betaflight passthrough interface) asks the firmware for MSP commands `1 1 2 3 4 5 90 36 119` before requesting passthrough. The firmware did not know **90 (MSP_ADVANCED_CONFIG)** and **119 (MSP_BOXIDS)**, answered `$M!`, and the tool gave up. Since 0.9.4 the firmware answers both. Tested on the board (2026-10-14): works, including over USB Serial/JTAG (`rx_s3_usb`).

At first we guessed wrong, thinking the USB Serial/JTAG port's RTS was resetting the chip. Three versions 0.9.0 - 0.9.2 went down that path: an RTC bit that had no effect, and a separate TinyUSB build (`rx_s3_otg`). All of it was dropped in 0.9.5, together with the setting `general.usb_rts_reset` and the command `ESC_TOOL_MODE`. The `reset: USB` line seen then most likely came from the firmware flashing. What found the bug was the diagnostic record in 0.9.3: it showed which MSP commands the tool had sent. That record has also been dropped.

### MSP replies taken from the firmware itself (0.9.5)
Format as in Betaflight 4.5 (`msp.c`); the content is the firmware's real state:

| Command | What is sent |
|---|---|
| API_VERSION, FC_VARIANT, FC_VERSION | 1.46, "BTFL", 4.5.0. **Fixed on purpose**: the tools only agree to talk to firmware they know (esp-fc and INAV do the same). |
| BOARD_INFO | "WBRX", chip (ESP32S3), "WINDBURST" |
| BUILD_INFO | build date and time, firmware version |
| UID | the chip's MAC address |
| STATUS, STATUS_EX | control loop period, sensors running (accelerometer / gyro, baro, compass, GPS, rangefinder), active modes, CPU load, reasons arming is blocked (converted to Betaflight's bits) |
| BOXIDS | ARM, plus the modes assigned to switches in the Mode tab (ANGLE 1, ALTHOLD 3, POS HOLD 11, BEEPER 13, CALIB 17, BLACKBOX 26, FAILSAFE 27, GPS RESCUE 46; STABILIZE does not exist in Betaflight) |
| FEATURE_CONFIG | RX_SERIAL / RX_PPM / RX_PARALLEL_PWM according to the receiver, GPS, RANGEFINDER, AIRMODE according to the settings |
| ADVANCED_CONFIG | motor protocol, PWM frequency, idle according to `motors.*` |
| MOTOR_CONFIG | min throttle, max throttle, min command, motor count, pole count, bidirectional DShot |
| MOTOR | each motor's output, 1000 .. 2000 (disarmed: 1000) |
| BATTERY_STATE | cell count, voltage, mAh, current |
| MOTOR_3D_CONFIG | Betaflight's default values (the firmware has no 3D) |

## 2. Internals

```
PC tool ──MSP──▶ app/msp_bridge.h    (same port as WBP: both parsers receive every byte, each ignores frames that are not its own)
         "$M<" API_VERSION, FC_VARIANT, BOARD_INFO, BUILD_INFO, UID, STATUS, MOTOR_CONFIG, BATTERY_STATE ... → reply
         "$M<" SET_PASSTHROUGH ──▶ app::MspLink ──Exchange──▶ FlightApp (REQ_ESC_PASSTHROUGH, sensor thread, locked):
                                   refuse if ARMED; stop motor test, send the disarm level one last time, MotorSystem::release()
                       ◀── ESC count
PC tool ──4-way──▶ hal/motor/esc4way.h   (Interface<EspEscPort>, runs in the link task, core 0)
         0x2F cmd addr n param crc16     → BLHeli bootloader command on ONE motor wire: bit-banged half-duplex UART at 19200 baud
         0x2E cmd addr n param ack crc16 ← (FC and ESC take turns driving the wire; between bytes the wire is an input with pull-up)
         cmd_InterfaceExit → ESP.restart()
```

- **Identification**: the tools only talk to firmware they know, so the MSP part answers like Betaflight 4.5 (variant `BTFL`, API 1.46, FC version 4.5.0), as esp-fc also does; the board name is `WINDBURST`. Only **read** requests are answered; `MSP_SET_MOTOR` (spinning motors from the ESC tool) is refused (`$M!`): to test-spin motors use the app's motor test, which has safety checks.
- **Supported bootloaders**: SiLabs / Atmel BLHeli (BLHeli, BLHeli_S, Bluejay) and ARM (BLHeli_32, AM32), as in Betaflight's `serial_4way.c` / `serial_4way_avrootloader.c` (interface 20.0.06, protocol 108, name `m4wFCIntf`). The STK500v2 "SK" bootloader of old Atmel ESCs is **not** supported (Betaflight also only has it when `USE_SERIAL_4WAY_SK_BOOTLOADER` is enabled).
- **Timing**: one byte on the wire is 572 µs, bits are 52 µs apart. On the board, `lock()` is a critical section (interrupts disabled on core 0). While waiting for the ESC to answer, the lock is only held in 200 µs slices (a page erase can wait up to 3 s; holding the lock that long continuously would make the chip's 300 ms interrupt watchdog reset it).
- **Pins**: motor pins follow **the app's motor order** (motor 1 = ESC #1 in the tool). `gpio_reset_pin()` takes the pin away from RMT / LEDC; on-chip pull-up, or not, according to `motors.ext_pullup` (the board has external 2k pull-up resistors).
- **Always reboot after passthrough**: the RMT / timer / bidirectional DShot drivers are reinitialised from scratch, simpler and more reliable than handing the pins back to a running driver (Betaflight hands them back).

## 3. Verification on PC

`test/test_esc4way.cpp` (60 checks): a **pin-level simulated BLHeli_32 bootloader**: it only sees the level on its own wire with timestamps, decodes it itself like a UART with its own clock (off by −2 % … +3 %), answers by driving the wire itself after a pause; it has 64 kB of flash, 1 kB pages.
- interface: name, version, bad CRC → `INVALID_CRC`, unknown command, nonexistent channel, SK mode → `INVALID_PARAM`, garbage bytes before a frame, EXIT, link loss;
- bootloader: connect (info `06 1F 63 04` = ARM), read 256 / 16 bytes, write 128 bytes, verify pass / fail, erase page 5 (exactly 0x1400 … 0x17FF), keep-alive, selecting another ESC on another wire, reset (RUN command) and hard reset (holding the wire low for 300 ms), ESC not answering → `GENERAL_ERROR`, ESC with 12 % baud error → reports an error rather than returning garbage; the lock is never held for more than one byte;
- MSP: v1 and v2, checksum, noise containing `$`, `$M!` for unknown commands and `SET_MOTOR`, `SET_PASSTHROUGH` (no payload / mode 0xFF / 0xFE), ARMED → 0.

`test_fw` group `esc` (real firmware on a virtual board): MSP via `MspLink`, ARMED → refused and the motors keep running, disarmed → 4 ESCs and **no more motor frames** afterwards, not even when ARMED again, a second request → 0, the pins of the 4 motors; WBP keeps running normally when MSP bytes are mixed in.

## 3b. Session record (0.18.1)
AM32 Configurator could connect but could not read the ESCs (`docs/HARDWARE_FEEDBACK_4.md`, section 2). To find out where it stops, every passthrough session is recorded (`esc4way::Trace`):
- What is recorded: each 4-way command, with ESC / address / byte count, the reply, and the first 4 bytes of the device info or of the data read.
- Where it is kept: RTC RAM, so it survives the reboot at the end of the session. The first 12 and last 12 commands are kept.
- Where to read it: on the next boot, the boot report has up to 4 `4way:` lines, for example:

```
4way: 9 requests, 1 ESC errors: NAME ok IFVER ok PROTO ok MODE ok INIT#0 ok 061F6304 READ 7C00/48 ESC-ERR ...
```

Meaning: `INIT#0 ok 061F6304` = ESC 1 has entered its bootloader (signature 0x1F06, ARM). `READ 7C00/48 ESC-ERR` = reading 48 bytes at 0x7C00 and the ESC did not answer correctly.

## 4. Not yet verified (needs a real board)

- Not yet run with the real BLHeliSuite32 / ESC Configurator, nor with real ESCs. The order of each tool's MSP requests is taken from the Betaflight source; a tool that asks for other commands will receive `$M!` (Betaflight answers the same for commands it does not have) - if the tool stops there, send me the tool's log.
- The real duration of `gpio_set_level / gpio_get_level` inside the critical section (expected < 1 µs, plenty compared with a 52 µs bit).
- Opening the USB Serial/JTAG port with another tool: some tools toggle DTR/RTS when opening the port; on the ESP32-S3 a particular DTR/RTS sequence resets the chip into download mode. If the board reboots as soon as the tool opens the port, tell me.
