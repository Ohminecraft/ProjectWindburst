# Motor output

## Structure
| File | Role | Verification |
|---|---|---|
| `hal/motor/motor_protocol.h` | DShot frame + CRC, RMT waveform, GCR/eRPM/EDT, PWM/Oneshot/Multishot pulse widths, endpoints | 164 PC tests, checked against Betaflight's formulas |
| `hal/motor/motor_output.h` | `MotorOutputManager`: disarm/idle/limit, reorder, DShot commands (reverse, save, beacon), test slider, telemetry, dynamic idle | PC tests + 12 deliberate faults, all caught |
| `hal/motor/dshot_direct.h` | `DshotDirect<Port>`: one-way / bidirectional DShot on the RMT memory and registers, hardware-independent (can be simulated) | `test_dshot`: signal-wire model, 91 checks |
| `hal/motor/dshot.h` | `RmtDirectPort` (ESP32-S3: RMT registers, GPIO, one gptimer) + `DshotHw` (fallback via the ESP-IDF driver, removed in 0.12) | **compiles + links for ESP32-S3**; not yet run on a board |
| `hal/motor/pwm_oneshot.h` | PWM / Oneshot125 / Oneshot42 / Multishot on LEDC (up to 8 motors) | compiles + links on S3; not yet run on a board |
| `hal/motor/motor.h` | `MotorSystem`: combines manager + backend according to the protocol | compiles on S3 |

## Important conventions
- The mixer outputs **absolute 0..1** values (idle included). Set `mixer.idle = motors.idleFraction(...)` every loop and `mixer.output_limit = cfg.output_limit`. The manager is the only owner of motor idle (static + dynamic).
- While armed, the motors **never go below idle** (even if the mixer slips to 0, a motor does not stop in mid-air). Disarm: DShot = 0, pulse = the "disarmed" value in the app (default 0 = no pulse).
- `reversed[]` does not change the mixer (like Betaflight): it sends a reverse command to the ESC, **always 7/8 + save 12** (the ESC stores it in flash; AM32, BLHeli_S, Bluejay, BLHeli_32 all accept it). Since 0.18.1, 20/21 are no longer used (`dir_cmd_extended` is ignored): those two commands are not saved and are also relative to the saved direction, so AM32 forgot the reversal after every FC reboot (`docs/HARDWARE_FEEDBACK_4.md`). Only sent while **disarmed**, at Betaflight's exact rhythm (wait 10 ms, repeat 10 times 1 ms apart). Arming cancels every pending command.
- Motor reorder: `order[logical] = hardware slot`, an invalid permutation falls back to the default. The pin of slot k = `HardwareConfig::getMotorPins()[k]`: the pin sent by the app (Pins and ports tab), or the default in `hardware_config.h`.
- Test slider: only while disarmed, turns itself off after 0.5 s if the app stops refreshing (the app sends ~10 Hz).
- Losing telemetry ≠ losing the motor: rpm older than 100 ms fades out the RPM notch of **that motor**, dynamic idle falls back to static idle.

## DShot on the ESP32-S3: the "direct" driver (since 0.6.0)

Version 0.5 on a real board: enabling bidirectional DShot made **the motors stop accepting commands**. Two causes, both reproduced with the model:
1. After each frame, the code called `pinMode(pin, INPUT_PULLUP)` to listen to the ESC. On ESP-IDF 5.5, `gpio_output_disable()` sets `oen_sel = 1` and clears the enable bit: the pin is **detached from the RMT transmitter**, from the second frame on nothing goes out.
2. The decoder (`runs_to_gcr`) required 20 bits. A reply ending at a high level has its last segment merge into the idle line and the receiver reports "done" without a length: ~37 % of valid replies were rejected. Now accepted from 18 bits (Betaflight).

The new approach (`dshot_direct.h`), one cycle calls `send()`:
```
send():   harvest the previous frame's reply (read RMT RX memory → length of each level, in ticks → 20-bit GCR)
          write 16 symbols + end marker into TX memory, pin in push-pull mode, start transmitting on all 4 channels
          bidirectional: schedule the timer after (frame time + 3 µs)
timer ISR (IRAM): release the pin (the ESC drives the line), enable receiving
```
| | Frame | One-way cycle | Bidirectional cycle (frame + 30 µs + reply + idle gap) |
|---|---|---|---|
| DShot150 | 107 µs | 137 µs | 312 µs |
| DShot300 | 54 µs | 84 µs | **182 µs** |
| DShot600 | 27 µs | 57 µs | **116 µs** |

If `send()` is called sooner than one cycle, that frame is **skipped** (counted in `stats().skipped`), never overwriting a frame being transmitted. A 4 kHz loop (250 µs) fits bidirectional DShot300 and DShot600; bidirectional DShot150 does not.
Late interrupt (the next frame is already due and the pin has not been released): that reply is dropped, counted in `late_switch`. An ESC not replying for 20 frames in a row: `rx_responsive(slot) = false`.

**0.12:** the fallback through ESP-IDF's `rmt_transmit` (`motors.dshot_driver = IDF`, one-way only, ~130 µs per cycle for 4 motors) was removed on request: it cost resources and is no longer needed. That byte in the settings is now `reserved1` so the following fields keep their positions. Old settings that chose IDF now run the direct method.

**Motor idle with analog protocols (0.12):** PWM / Oneshot / Multishot / brushed have no "Motor idle" (like Betaflight): the lowest throttle when armed is `pulse_armed` (Betaflight's min_throttle). The firmware ignores `idle_x10` for these protocols, and the Motor tab hides that field. Before 0.12 idle was added on top of `pulse_armed`, so an old analog craft now idles lower: if the motors stop or desync at low throttle, raise `pulse_armed`.

Figures sent to the app (`motors` stream): `bidir` (OFF / ACTIVE / UNAVAILABLE), `answers` (bit i = the ESC of motor i is replying), `skipped`, `late` (in the last second).

**Not yet checked on the chip.** The model (`test/test_dshot.cpp`) has an ESC that decodes frames like a real ESC (pulse width, gap between two frames, inverted CRC), replies in GCR after 30 µs, a receiver with noise filtering and an idle threshold, and catches two devices driving the line at once. What the model does not know: pull resistors, the real latency of the timer interrupt on the core running the flight loop, ESCs drifting off timing.

## Start-up check of bidirectional DShot (0.8.1)

Version 0.8.0 on a real board (Aikon AK32 ESCs, BLHeli_32): enabling bidirectional DShot gave **no telemetry and the motors did not accept commands either**. On the PC (signal-wire model, `test_dshot`) the whole chain works, so the broken part lies in something the PC cannot simulate: the routing of the RMT transmitter / receiver pins through the GPIO matrix on ESP-IDF 5.5, or the timing of the timer interrupt. To find it on that very board:

- At boot, for each motor, the firmware has the receiver listen to the pin while the transmitter sends a "0" frame (an ESP32 pin can read back the level it is driving), then releases the pin as in flight and decodes the 16 bits received. Every bit correct on every pin → `motors: DShot/RMT direct, bidirectional (start-up check OK on 4 pins) x4`.
- Wrong → **falls back to one-way DShot by itself** (the motors still spin, can still beep) and the `motors:` line gives the reason: `bidirectional OFF: start-up check, the receiver of motor N heard nothing (k words)` (the transmitter does not reach the pin or the receiver cannot hear the pin), `... not a DShot frame`, `... sent 0x...., heard 0x....`, or `bidirectional OFF: <ESP-IDF function> failed (<error code>)` when creating a channel / timer fails.
- The Motor tab adds "Replies received / corrupt per second". Read together with the status line: start-up check OK but **0 replies** → the ESC is not replying (ESC firmware, or the ESC has not recognised the inverted signal); many **corrupt** replies → receiver timing / filter; check OK, replies coming but motors not spinning → report back with those two lines.
- PC verification: `test_dshot` `[start-up check]` (4 DShot speeds × 2 resolutions: every bit read back correctly on every pin; the ESC replies to the check frame once the pin is released, no fighting over the line; transmitter not reaching the pin / deaf receiver → "heard nothing"). **Result on a real board: not available yet.**

### 0.8.2: esp-fc-style pin switching
Compared with esp-fc's ESC driver (`lib/EscDriver/src/EscDriverEsp32.cpp`, which runs bidirectional DShot on the ESP32-S3), the main difference is how the pin direction is switched. esp-fc **re-routes the signal through the GPIO matrix on every switch**: when transmitting it disables the input path, connects the RMT transmitter's signal, enables output (`enableTx`); when listening it disables output, enables the input path, connects the receiver's signal (`enableRx`). Versions 0.6 - 0.8.1 left both signals permanently connected and only toggled output enable. Since 0.8.2 `RmtDirectPort::drive()` / `release()` do exactly what esp-fc does (plus: output enable taken from the GPIO_ENABLE bit, `OEN_SEL = 1`, so disabling the output surely releases the line). **One-way** DShot **stays as before** (the path already works on the board). The other esp-fc pieces already have equivalents here: inverted frame + inverted checksum, pin pulled up while the ESC talks, receiver filter / idle threshold, GCR decoding (`value ^ (value >> 1)`). The remaining difference: esp-fc enables the receiver from the RMT "transmit done" interrupt (old driver `driver/rmt.h`, which cannot be shared with the new driver that Arduino-ESP32 3.x uses); here it is a timer firing 3 µs after the frame.

### 0.8.4 (removed in 0.8.6)
0.8.4 added a second interrupt per frame to drive the pin high again after the reply window, measure the line level and read back the frame while running (following `docs/HARDWARE_FEEDBACK_3.md`). On the board the loop dropped from ~5000 to ~4300 Hz, so **0.8.6 returned `dshot_direct.h` to the sequence of 0.8.2 / 0.8.3** (esp-fc-style pin switching, one interrupt per frame, pin released until the next frame); the `line_low` / `rb_ok` / `rb_bad` fields and the two diagnostic lines in the Motor tab were removed with it. Instead of holding the line high in software: an external pull-up resistor (section below), so the line goes high as soon as it is released.

### 0.8.5: external pull-up resistor
`motors.ext_pullup` (Motor tab, "External pull-up resistor fitted"; save and reboot): the board already has a resistor pulling the signal wire up to 3.3 V (typically 1 – 4.7 kΩ), and the firmware **disables** the internal pull-up (~45 kΩ) of the motor pins when running bidirectional DShot. The `motors:` line of the boot report says "internal / external pull-up". With 2 kΩ: pulling the line low costs 1.65 mA, which the ESC drives easily; and a released line rises to high in about a tenth of a microsecond instead of a few microseconds. Verification: `test_fw` group `rxdiag` (the setting reaches the driver). **Not yet tried on a board.**

### 0.18.2: the receiver must not see any edge before the start bit
Symptom on the board (AM32): voltage / current from EDT still present, rpm almost always 0, only occasionally a number. Voltage is held for 1 s, temperature held forever, while rpm is only valid for 100 ms after the last reception: just a few percent of replies received correctly keep the voltage showing, while rpm keeps dropping out. So most replies are lost, eRPM and EDT alike.

Cause found while re-reading the sequence (not yet measured on a board): up to 0.18.1, the pin's input path was disabled while transmitting (`PIN_INPUT_DISABLE`, the pin reads 0) and re-enabled when the pin was released, with the receiver starting right after. That 0 → 1 step goes through the RMT noise filter (¼ of a reply bit, 333 – 667 ns) and so reaches the receiver **after** it has started, especially on the channels started last. The receiver takes it as the first edge, measures a 27 µs high level before the reply, longer than the idle threshold (8 bits = 10.7 µs at DShot600), and **stops before the ESC has a chance to reply**. esp-fc also works this way; it may get away with it thanks to a different idle threshold, but that is not certain.

Fix (the receive stage rewritten the way Betaflight does it on STM32: capture the whole window, find the start bit in it):
- The pin's input path is enabled **once** in `begin()` (`listen()`) and left alone. An ESP32 pin reads back the level it drives; the receiver is stopped while we transmit, so it has no effect. `drive()` still re-routes the transmit signal and enables output like esp-fc (the 0.8.2 lesson), `release()` only disables output.
- The receiver starts **before** the pin is released, while the line is still held high: releasing the pin produces no edge either.
- Decoding in `reply_from_runs()` (`motor_protocol.h`), following `dshot_bitbang_decode.c` / `pwm_output_dshot_shared.c`: ignore everything before the start bit (idle level, glitches shorter than half a bit), the segments must add up to 18 – 21 bits; only a **high** segment may run past bit 21 (that is the idle line after the reply), a low segment running past bit 21 is not a reply (Betaflight: `bits != 21`). A valid reply always ends at a high level (the checksum fixes the number of level changes).
- Failure reasons are counted separately and sent to the app every second (`motors` stream): `no_start` (window has no start bit: a timing / pin switching problem), `bad_bits` (start bit present but wrong number of bits: noise, slow edges), `bad_crc` (clean bits, wrong checksum). A capture without a start bit does not count as the ESC replying.
- App: the RPM column shows "–" when rpm is older than 100 ms (`rpm_ok`), no longer showing 0 (0 is easily read as "motor stopped").

Risk to try on the board: keeping the input path enabled while transmitting differs from esp-fc. Electrically it only enables the input buffer, and the start-up check (`loopback()`) has transmitted with the input path enabled since 0.8.1. If the ESCs stop accepting frames when bidirectional DShot is enabled (motors do not spin, do not beep), this change is the cause.

## Motor wizard (0.20, like Betaflight Configurator 4.3+)
Motor tab, "Motor wizard" block: motor order and spin direction are found by watching the motors spin, not by typing slot numbers. The "Hardware slot" column and the "Reverse" switch in the "Per motor" table are still there for manual editing.

- **Conditions**: connected, not armed, "All propellers removed" ticked, and no setting waiting for a reboot. The wizard works on the running motor configuration; changing the protocol or frame type requires saving and rebooting first.
- **Reorder**: each output in turn spins slowly (adjustable throttle, default 8 %). You click the position of the spinning motor in the picture. After the last motor, the app writes the new `motors.order`. The firmware uses that order **immediately**, no reboot needed; press "Save" to keep it.
- **Spin direction** (DShot only): click a motor to spin it (or choose "Spin all"), then compare with the arrow in the picture. If it spins the wrong way, press "Reverse this motor" (in "Spin all" mode, click the motor directly). The app sends commands 7 / 8 + 12 to the ESC and sets `motors.reversed_mask`.
- **Safety**: the test command is refreshed every 200 ms, and the firmware stops the motor by itself if it is not refreshed for 0.5 s. The motor stops when "Stop" is pressed, when leaving the tab, when the window is hidden, or when the craft arms.

Firmware: `FlightApp::apply_motor_order()`. When the `motors` section is written, if compared with boot only `order` / `reversed_mask` changed, the order and direction are applied immediately (`MotorOutputManager::set_order()`). The order must be a permutation of the motors, otherwise the old order is kept. If another field changed, nothing is applied and a reboot is reported as required, as before. When the order really changes, pending test commands and DShot commands (aimed at the old slots) are dropped.
Verification: `test_motor` (`set_order`), `test_fw` motor-test group (new order takes effect immediately, the right slot spins, a wrong order is rejected, a protocol change is not applied), 7 `wizard:` mutants. Real page: `test_gui_browser.py::test_motor_wizard` (written but cannot run on this machine yet). **Not yet tried on a board.**

### Crash flip (0.42)
When arming into crash flip, `MotorOutputManager` sends DShot 21 to every motor (10 times), and DShot 20 when it ends. These commands spin the motors relative to the direction saved in the ESC and are not saved. This is the only case where DShot commands are sent while the craft is armed. Until the commands have all gone out, the motors receive nothing else. During crash flip, a motor commanded to 0 stands still rather than running at idle. See `docs/MODES.md`, Crash flip section.

## Protocols (0.8.2)
- **ProShot1000** (Betaflight `loadDmaBufferProshot`): the same 16-bit packet as DShot, sent as 4 nibble pulses (high nibble first), each pulse in a 4 µs slot, high level 1 µs + nibble × 125 ns; 16 µs per frame. One-way only. Can carry DShot commands (beacon, spin direction...), as Betaflight treats ProShot as a DShot protocol. Needs ESC support (BLHeli_32 has it; BLHeli_S / Bluejay do not).
- **Brushed**: brushed motors through a MOSFET, LEDC 16 kHz, 1000..2000 = 0..100 % duty.
- DShot / ProShot: up to **4 motors** (the ESP32-S3's 4 RMT transmitters), also on the PC (`MotorSystem::kMaxDigitalMotors`); 6 / 8 motor frames use PWM / Oneshot / Multishot (LEDC, 8 channels). The boot report states the reason.

## Frames (layout, 0.8.2)
Besides Quad X, Quad +, Hexa X (this project's own): Betaflight's tables (`mixer_init.c`, motor order and prop direction as in Betaflight): Hexa + (HEX6), Hexa X in Betaflight order (HEX6X), Hexa H, Y6, Y4, Octo X8, flat Octo + / X, V-tail, A-tail, and Quad X in 1234 order (esp-fc). Each axis is normalised so the largest coefficient = 1, like the three old frames. `test_core` checks every frame with a geometry (not V-tail / A-tail: tilted motors) against a physical model built from motor positions and prop directions: roll / pitch / yaw commands rotate the right way, do not leak into another axis, do not change total thrust (Y4: Betaflight leaks a little into yaw, only direction and total thrust are checked). The diagram in the Motor tab draws the lower motor of a coaxial pair offset inwards, with a ↓ mark. **No frame other than a quad has been test-flown.**

## Current limitations (frankly)
1. **DShot up to 4 motors** on the S3 (RMT has 4 TX channels, 4 RX channels). Hexa / octo need a parallel output backend (I2S / LCD-CAM), not done yet. PWM / Oneshot / Multishot pulses (LEDC) up to 8 motors.
2. **Bidirectional DShot has not been tried on silicon** with the new driver (see above). If no rpm comes out, the motors still run; if the motors do not run, choose the ESP-IDF driver.
3. Multishot 5 – 25 µs, Oneshot42 = value / 24 are the common protocol definitions; the Betaflight pwm_output file in the zip does not have these constants to compare against.
4. Battery via ESC (EDT voltage) needs ESC support and "Extended DShot telemetry" enabled.
5. DShot300 is the default (more forgiving than 600 with long wires).
6. Disarmed pulse for PWM / Oneshot / Multishot: `pulse_disarmed`, **0 = 1000 µs** (since 0.18.1, Betaflight's `mincommand`; lower if `pulse_armed` < 1000). Never "no output": an ESC only arms after seeing a zero-throttle pulse, and an AM32 that sees high throttle first (motor test) enters throttle calibration.
7. **AM32 only accepts DShot and Servo PWM** (800 – 2200 µs), not Oneshot125 / Oneshot42 / Multishot (`signal.c: detectInput`). Those protocols are for BLHeli_S / BLHeli_32 / KISS.
8. LEDC writes registers directly (`port/pwm.h`, `hal/ledc_ll.h`, 0.18.1) instead of Arduino's `ledcAttach` / `ledcWrite`: no more calls into flash functions every cycle. Outputs with the same frequency + resolution share one of the 4 timers.

## Checks on the board (in order, props off)
1. Overview tab, Motor / ESC line: `DShot/RMT direct` or `DShot/RMT idf` or `Pulse/LEDC`.
2. Test each motor at ~5 %: right motor, right direction? The "Beep" button tells which motor is which slot.
3. Reverse with the "Reverse" switch (DShot command, only while disarmed): BLHeli_S / Bluejay / BLHeli_32 use commands 7 / 8, AM32 accepts 20 / 21 as well.
4. Enable bidirectional DShot, save, reboot: what does the status line under the motor table say; the RPM column and "Telemetry errors"; "Skipped frames", "Missed replies". Send those numbers back if it is not "running, every ESC replying".
