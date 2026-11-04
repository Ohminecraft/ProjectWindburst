# Receiver: CRSF, SBUS, IBUS, Spektrum, MAVLink, PPM, PWM — `src/hal/rx`

| File | Contents |
|---|---|
| `rx_proto.h` | **portable**: `rx::CrsfDecoder`, `rx::SbusDecoder`, `rx::IbusDecoder` (byte stream → 16 channels in µs), `rx::Link` (link ok / lost, holds the last channels) |
| `rx_mavlink.h` | **portable**: `rx::MavlinkDecoder` (ELRS in MAVLink mode: `RC_CHANNELS_OVERRIDE` → 16 channels in µs; `RADIO_STATUS` → LQ, RSSI, `txbuf` for telemetry throttling) |
| `rx_pulse.h` | **portable**: `rx::PpmDecoder`, `rx::PwmDecoder` (pulse edge times → channels), `rx::EdgeRing` (interrupt → sensor thread queue) |
| `rx_base.h` | `RcReceiver`: decoder + `HardwareSerial` (baud, 8E2, inversion), or GPIO interrupts for PPM / PWM |

```cpp
RcReceiver rc;
rc.setTimeoutMs(500);
rc.begin(rx::PROTO_CRSF, Serial2, rxPin, txPin);     // false + failReason() if the protocol is not available / pins not set
// ~1 kHz:
rc.update(micros(), millis());
rc.channels();  rc.linkOk(millis());  rc.rateHz();  rc.linkQuality();
```
`FlightApp` already does this (UART2, protocol from `receiver.protocol`, pins from `pins.rx_rx / rx_tx`).

| Protocol | UART | Frame | Conversion to µs (same as Betaflight) |
|---|---|---|---|
| CRSF (Crossfire, **ExpressLRS**) | 420000 8N1 | `C8, len, type, payload, crc8 DVB-S2`; `0x16` = 16 channels × 11 bits; `0x14` = link statistics (LQ, RSSI) | `0,62477·raw + 881` (172→988, 992→1500, 1811→2012) |
| SBUS | 100000 **8E2, inverted** (the ESP32 UART inverts by itself, no inverter circuit needed) | `0F, 22 byte, flags, 00`; flags bit2 = frame lost, bit3 = failsafe | `5/8·raw + 880` |
| IBUS (FlySky) | 115200 8N1 | `20 40, 14 × u16, checksum = 0xFFFF − sum` | values are µs |
| Spektrum 2048 (`SPEKTRUM`: DSMX, DSM2 11 ms) | 115200 8N1 | 16 bytes: `fades, system, 7 × u16 big-endian`; bits 14..11 = channel number, 10..0 = value; `FFFF` = empty slot | `988 + value / 2` |
| Spektrum 1024 (`SPEKTRUM1024`: DSM2 22 ms) | 115200 8N1 | as above, bits 13..10 = channel number, 9..0 = value | `988 + value` |
| MAVLink (`MAVLINK`: **ExpressLRS** in MAVLink mode, 0.39) | 460800 8N1 | MAVLink 2; `RC_CHANNELS_OVERRIDE` (id 70) every 10 ms = 16 channels; `RADIO_STATUS` (id 109) every 10 ms | values are µs; 0 or 65535 = keep the old channel value |
| PPM | pin `pins.ppm`, rising-edge interrupt | time between two edges of the same kind = one channel; gap > 2.7 ms = sync | is µs |
| PWM | pins `pins.rx_pwm[0..7]`, both-edge interrupt | rising edge → falling edge on each pin = one channel | is µs |

Channels are clamped to 885..2115 µs. Default channel order is AETR (roll, pitch, throttle, yaw, aux1…), changed in section `receiver`.

**Frame sync**: the UART is read in blocks (~1 kHz), so per-byte timing is not relied on.
- CRSF, IBUS: based on the checksum; a bad frame is rescanned from its second byte (a fake header does not hide a real frame).
- SBUS has no checksum: uses an idle gap ≥ 4 ms between frames (2.5 ms threshold) + first byte `0x0F` + last byte.

**Signal loss**: `Link::ok()` = a frame **without the receiver's failsafe flag** arrived within `rx_timeout_ms`. SBUS failsafe frames do not keep the link alive and do not overwrite the sticks. CRSF/ELRS signal loss by stopping channel frames. When the link is lost, the core runs failsafe (see `docs/CORE.md`).

ELRS note: an ELRS receiver speaks CRSF over UART → choose `CRSF`. ELRS set to MAVLink mode (Link Mode = MAVLink) → choose `MAVLINK` (below).

**MAVLink** (firmware 0.39, `rx_mavlink.h`, usage: `docs/ELRS_MAVLINK.md`): like Betaflight's `rx/mavlink.c`. Frames are found by the start byte `0xFD` and the CRC (`mav::Parser`, like `mavlink_parse_char`: a bad frame is dropped, it waits for the next start byte). `RC_CHANNELS_OVERRIDE`: channels 1..16, value 0 or 65535 keeps the old channel value (MAVLink's "ignore this field"), channels not yet received = 1500; `target_system` is not checked. `RADIO_STATUS`: LQ = `rssi` × 100 / 255 (ELRS sends LQ × 2.55), RSSI = −`remrssi` dBm, `txbuf` = % of the receiver's buffer still free. All other messages (from the ground station) are ignored, not counted as errors. Signal loss: ELRS stops sending channels → after `rx_timeout_ms`, failsafe. MAVLink telemetry goes back out the TX pin of this same UART when `receiver.telemetry` is on (`docs/MAVLINK.md`).

**Spektrum** (DSMX / DSM2 satellite, firmware 0.8): no checksum → frames are found from an idle gap > 4 ms between frames (a frame lasts 1.4 ms), then counted in 16-byte steps (reading a backlog after the loop stalled still works). With more than 7 channels, frames take turns carrying channels: the decoder keeps each channel's last value. Channels 0..11 (as in Betaflight), channels not yet received = 1500. Spektrum's channel order is **TAER**: choose TAER in the Receiver tab. **Bind not implemented** (Betaflight's `spektrum_sat_bind`): bind the satellite with a main receiver or the transmitter first.

**PPM / PWM** (firmware 0.8): the GPIO interrupt only records the time (`micros()`) and level into `rx::EdgeRing` (64 edges); decoding runs in the sensor thread. Limits as in Betaflight (`rx_pwm.c`): valid pulse 0.75..2.25 ms; PPM 4..12 channels, channel count learned from 25 identical consecutive frames (one bad frame neither changes the channel count nor gets through); positive or negative PPM both work (only one edge type is considered). PWM: a frame is reported when channel 1 finishes a pulse **and** every connected channel had a good pulse within 100 ms (a loose wire → signal loss after `rx_timeout_ms`, no flying on with a stale channel). Missing channels = 1500. PPM / PWM do not use UART2: the MSP optical flow sensor can take UART2. Set the PWM / PPM receiver's failsafe to "no pulses": if the receiver sends failsafe pulses the firmware thinks the signal is still there.

## CRSF telemetry (firmware 0.8) — `crsf_telemetry.h`

With pin `pins.rx_tx` set and protocol `CRSF` → the firmware sends telemetry frames into the receiver's RX (ELRS / Crossfire on a two-wire UART, full duplex); an EdgeTX / OpenTX transmitter's "Discover sensors" finds RxBt, Curr, Capa, Bat%, Ptch / Roll / Yaw, FM, GPS, Alt, Sats, VSpd. Layout and units as in Betaflight's `telemetry/crsf.c`:

| Frame | Contents |
|---|---|
| `0x08` battery | voltage u16 (0.1 V), current u16 (0.1 A), used u24 (mAh), remaining u8 (%) |
| `0x1E` attitude | pitch, roll, yaw i16 (rad × 10000; yaw sent in −π..π) |
| `0x21` flight mode | string: `!FS!` failsafe, `FLIP` crash flip, `RTH` rescue, `LOIT`, `POSH`, `ALTH`, `ANGL`, `STAB`, `ACRO`; disarmed: `*` appended, or `!ERR` when something blocks arming |
| `0x02` GPS (when there is a GPS module) | lat, lon i32 (degrees × 1e7; 0 before a fix), speed u16 (km/h × 10), heading u16 (degrees × 100), altitude u16 (m + 1000, estimated altitude), satellite count |
| `0x07` vario (when there is an altitude estimate) | vertical speed i16 (cm/s) |

The whole list is sent 10 times per second, one frame at a time (Betaflight: `CRSF_CYCLETIME_US` / frame count). Frames are built in the sensor thread (holding the lock, a few µs) and written to the UART after the lock is released. Nothing is sent while the signal is lost. Startup report: `rx: CRSF on UART2, telemetry on its TX pin`. Verification: `test_rx` `[CRSF telemetry]` (bytes computed by hand, value clamping, mode strings, schedule across clock wraparound, no burst after a pause), `test_fw` group `crsf` (frames the firmware writes to the virtual board's TX pin are read back with a separate CRC, compared with the firmware's own telemetry and with the virtual board: 12° tilt, 12 A, GPS fix, climbing; no TX / SBUS / signal lost: nothing sent). **Not yet tried with a real receiver and transmitter** (sign of pitch on the transmitter screen, whether ELRS passes all frames).

## Diagnosing a frame rate drop (0.8.5)
A user reported the receiver frame rate dropping when bidirectional DShot was on (version 0.8.2). Reading the code shows no direct path between the two (the receiver runs in the sensor thread, DShot in the flight task + a timer interrupt). The only indirect path is CPU time on core 1: bidirectional DShot adds work per cycle and one (since 0.8.4: two) interrupts per frame, so the sensor thread runs less. Therefore:
- **More headroom**: the receiver UART's receive buffer 512 → **2048 bytes** (48 ms of CRSF at top speed), the hardware FIFO is moved to the buffer every **64 bytes** (Arduino's default at this speed is 120 / 128: only 190 µs left for the UART interrupt), each `update()` reads up to 1024 bytes (previously 256).
- **Count what is lost**, per second, in the `rc` stream and the Receiver tab: `bad_frames` (frames dropped by the decoder: checksum, length, sync), `uart_ovf` (UART driver out of room: the firmware cannot keep up reading), `uart_line` (bit framing / parity errors: noise on the wire, wrong baud, wrong inversion). All three at 0 while the rate is still low: the receiver is sending fewer frames.
- The Arduino serial event task (which receives those errors) runs on **core 0, priority 2** (`platformio.ini`): its default is the highest priority on any core, which would cut into the flight loop.

Verification: `test_fw` group `rxdiag` (2 kB buffer and 64-byte FIFO are set; 30 bad frames, 2 overflows, 1 bit framing error counted correctly and shown in the `rc` stream). **The real cause on the board is not yet known**: these three numbers are needed with bidirectional DShot on / off, along with `loop_hz` in the System tab.

## Reading the UART in blocks (0.8.7)
The first board measurement with the 0.8.5 counters: 149 frames/s, **~100 UART buffer overflows per second**, usually 1 bad frame, the transmitter "updating slowly". Overflows with almost no lost frames mean the firmware **reads slower than bytes arrive**: the driver's buffer is always nearly full, each frame is read late by exactly the amount of bytes waiting in it (2 kB of CRSF is hundreds of milliseconds), and each time the driver has no room it counts one overflow.

Cause: the old code read **byte by byte** (`available()` + `read()`). On Arduino-ESP32 3.x each such byte locks the port's mutex twice, then the driver's mutex, goes through the ring buffer and checks for overflow: a few microseconds of flash code per byte, while the sensor thread only gets the CPU share left on core 1 after the flight task. Since 0.8.7 the receiver reads **in blocks** (`read(buf, n)`, up to 128 bytes at a time, 2 kB per pass): that cost is paid once per block. GPS and the MSP flow sensor also read in blocks (`src/hal/serial_block.h`).

Verification: `test_fw` group `rxdiag` (286 frames in 2 s, 286 read calls, never a single-byte read; GPS likewise). **Not yet measured on the board**: after this version "UART buffer overflows" should drop to 0 and the frame rate should equal the packet rate set on the transmitter.

## Verification

`test/test_rx.cpp`: independently written encoder (bit-by-bit loop), Betaflight's reference points, 16 distinct channels (catches bit/channel order bugs), bad frames, fake headers, 20 000 random bytes producing no ghost frames, byte-by-byte reads, SBUS flags, SBUS2, truncated frame + idle gap, timeout across `millis()` wraparound.
`test/test_fw.cpp`: the real firmware receives CRSF from a simulated "transmitter", arms, flies, transmitter off → failsafe.
Spektrum / PPM / PWM: `test_rx` (frames built bit by bit from the description, 2048 / 1024, channels split across two frame types, channel numbers 12..15 dropped, truncated frames, many frames read as a backlog, random bytes; PPM: learning the channel count, pulses too short / too long, 3 / 12 / 13 channels, frames of varying length, 32-bit clock wraparound; PWM: channel stale beyond 100 ms, bad pulses, stray falling edges; full queue) and `test_fw` group `receivers` (`WB_ONLY=receivers`): the real firmware on a virtual board with a virtual Spektrum satellite on UART2 and virtual PPM / PWM receivers emitting pulse edges into virtual GPIO interrupts at the right times: channels match to 1 µs, frame rate, arm and fly, signal loss, PPM channel count change, one loose PWM wire, wrong settings, pin clashes, UART2 handed over to optical flow. Real page: `test_ppm_and_spektrum_receivers_on_the_page`.

**Not yet verified on the board**: whether the ESP32's 420 kbaud UART drops bytes while the flight loop is busy (RX buffer set to 512 bytes), hardware SBUS inversion.
**Not yet verified on the board (Spektrum / PPM / PWM)**: real GPIO interrupt latency (other interrupts, flash writes disabling the cache → one frame lost), 5 V receivers needing a voltage divider, a real satellite (`system` byte, frame order), bind.
**Not done**: Spektrum bind.

## nRF24 (0.38)
Protocol `NRF24`: an nRF24L01+ module on the second SPI bus (pins `nrf_*`), receiving the Windburst DIY transmitter (`docs/TX.md`). `RcReceiver::beginNrf24` + `rx::Nrf24Rx` (`src/hal/rx/rx_nrf24.h`): reads every pending packet in the sensor thread, 8 channels (9..16 = 1000), bad packets go to `rc.bad_frames`, `rc.lq` = % of packets received out of those the transmitter sent, `rc.bind_state`. Bind: listens automatically for 60 s after power-on if never bound, or via the `BIND` command (Receiver tab, 30 s), never while armed. Telemetry back to the transmitter via ACK. Not yet on STM32. Verification: `test_rx` (`t_nrf24`), `test_fw` groups `nrf24` / `txfly`.

## Telemetry to the transmitter: on / off (0.20)
`receiver.telemetry` (Receiver tab, on by default): when on, the firmware sends CRSF telemetry on the receiver's TX pin (battery, attitude, GPS, vario, mode). When off the TX pin is silent and the receiver still receives channels normally; the startup report says `rx: CRSF on UART2, telemetry off (receiver.telemetry)`. Takes effect after save and restart. Old-version receiver settings records (12 bytes) are still readable, with telemetry kept on. Verification: `test_fw` group `crsf`, `test_proto`.
