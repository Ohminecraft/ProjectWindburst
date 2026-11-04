# Battery and alerts (beeper / motors) — `src/hal/battery`, `src/hal/beeper`

Behaviour follows Betaflight (`sensors/battery.c`, `sensors/voltage.c`, `io/beeper.c`) so that the numbers pilots are used to mean the same thing.

## Battery

```
src/hal/battery/battery_monitor.h   batt::Monitor: voltage → filtered voltage, cell count, state      [portable]
src/hal/battery/battery_base.h      BatterySensor: picks the source (ADC / ESC), reads the ADC, calls Monitor at 50 Hz
```
- **Measurement source** (`battery.source`): `AUTO` (ADC if pin `pins.batt_adc` is set, otherwise ESC telemetry), `ADC`, `ESC`, `NONE`.
  The ADC is read with `analogReadMilliVolts()` (calibrated by Arduino-ESP32) and multiplied by `battery.divider` (= battery voltage / voltage at the pin; a 10k : 1k divider is 11).
- **Filtering**: PT1 1/3 Hz at 50 Hz, so a throttle punch does not look like an empty battery.
- **Battery present or not**: the filtered voltage is stable (10 out of 11 intervals of 100 ms within 0.1 V) and plausible: 3.0 V … one full cell, or above 6.0 V. The range in between is 5 V USB power, not a battery.
- **Cell count**: from `battery.cells`, or detected when the battery is plugged in: `voltage / cell_max + 1` (at most 8). A battery plugged in nearly empty is detected one cell short (a 4S at 12.8 V = a full 3S): if you know the cell count, enter it. Since 0.18.1, when the stable voltage exceeds `cell count × cell_max` (a cell cannot be higher) it is recounted: power coming up in steps, such as ESC telemetry voltage pausing for a while around 4 V, no longer gets stuck at 1S. It only counts up, never down.
- **State**: `NOT_PRESENT → OK → WARNING → CRITICAL`, with 10 mV hysteresis and delays (`warning_delay`, `critical_delay`; the critical delay counts time **continuously** below the threshold).
- **Effect**: `WARNING` or worse blocks arming (blocker `BATT_LOW`); `CRITICAL` in flight → failsafe landing if `land_on_critical` is on. The flight core no longer has its own battery thresholds: it receives `fc::Health{batt_low, batt_critical}` from `FlightApp`.
- The whole `battery` section is **live**: threshold changes take effect immediately.

### Current, mAh used, percent remaining (firmware 0.8)

- **Current source** (`battery.current_source`): `AUTO` (current sensor if pin `pins.curr_adc` is usable, otherwise ESC), `ADC`, `ESC`, `NONE`.
- **ADC current sensor**: `current (mA) = mV at the pin × 10000 / current_scale + current_offset`, exactly Betaflight's formula and units (`ibata_scale` 0.1 mV/A, `ibata_offset` mA): the board manufacturer's numbers work as they are. Default 400 = 40 mV/A. A negative scale for a sensor mounted backwards.
- **ESC**: sum of the currents the ESCs report via *extended DShot telemetry* (needs `motors.bidirectional` and `motors.edt`). ESCs only send it when enabled with DShot command 13: the firmware sends this command **when the ESCs start answering** (when the battery is plugged in, including after USB was plugged in), and sends it again when they all go silent and then answer again (ESC restart). **Never sent while armed** (DShot commands stop the motors while the command repeats); if an ESC restarts mid-flight the command is sent right after disarm. An ESC's reading still counts for 1 s after its last report.
- **Filtering**: displayed current goes through a 1 Hz PT1 (as in Betaflight); mAh accumulates the unfiltered readings, 50 times per second, from power-up (not reset on arming, as in Betaflight). A gap ≥ 1 s (sensor task stalled) is not added.
- **Remaining (%)**: with `battery.capacity_mah` and a current reading → `(capacity − mAh used) / capacity`; otherwise by voltage between `cells × cell_min` and `cells × cell_max`.
- Startup report: `current: ADC sensor on GPIO3, 400 x 0.1 mV/A`, `current: from the ESCs (extended DShot telemetry)`, `current: none (...)`, or `current: FAILED - ...` (ADC chosen with no pin set, pin is not an ADC, ESC chosen without extended telemetry on). Clashing pins: `pins: FAILED ...`, arming not allowed.
- Stream `battery`: `current` (0.01 A), `mah`, `remaining` (%), `current_source`, `current_mv` (voltage at the sensor pin, for calibration). Blackbox: columns `current`, `mah` (battery group).
- `current_scale`, `current_offset`, `capacity_mah`, `current_source` take effect immediately; changing pins needs save and restart.

## Beeper

```
src/hal/beeper/beeper_core.h   beep::Beeper (tone pattern table, priority), beep::Conditions (condition → tone)   [portable]
src/hal/beeper/beeper_base.h   BuzzerOutput: active buzzer (logic level) or passive (PWM via LEDC)
```
- 18 conditions (enum `beeper_cond` in the schema): gyro calibration done, signal lost, failsafe landing in progress (SOS), arm / disarm, arm with / without GPS fix, battery low / critical, Beeper switch, accelerometer calibration, GPS ready, armed at low throttle, power-on, cannot arm, blackbox erase done, and `USB`.
- Tone patterns in 10 ms units, taken from Betaflight; plus two of our own: power-on and "cannot arm".
- **Priority** = order in the enum; a tone that is playing is not interrupted by a tone of equal or lower priority.
  Deliberately different from Betaflight: a condition that is **turned off** is treated as non-existent (it does not block lower-priority tones).
- **Silent on the bench**: app connected + battery not measurable + condition `USB` off (default) → the **buzzer** does not sound, except for the "Try" button in the app. Since 0.8.1 this rule **does not apply to the motors**: ESCs can only beep when powered, i.e. the battery is plugged in, even if the firmware cannot measure the battery (no ADC pin, ESC does not report voltage). Before 0.8.1, a craft that could not measure the battery and was connected to the app never beeped its motors. "App connected" = a valid frame in the last 3 s (the Python client sends a keepalive every 0.3 s).
- Command `BEEP {condition}`: plays a test tone on the buzzer and motors (even a tone that is turned off, even while "silent on the bench"); empty `condition` = silence.

### Motors beeping instead of a buzzer (craft without a buzzer)

ESCs accept the DShot "beacon" commands (commands 1 to 5 = five different tones) and make the motors vibrate into a beep. The firmware uses this as a **second sound output**, running in parallel with the buzzer and with its own condition list (`beeper.motor_conditions`). On a craft with no buzzer pin assigned this is the only alert sound, with nothing extra to set up: it is on by default.

```
beep::Beeper beeper        tone patterns for the buzzer (as before)
beep::Beeper motor_beeper  same conditions, slot_ms = 450: each "on" segment of the pattern = exactly ONE ESC beep
beep::Conditions           passes requests to both; the motors only take them when allowed (below)
MotorOutputManager::beacon_all(tone)   one DShot frame, all motors at once (Betaflight: dshotCommandWrite(ALL_MOTORS))
```
- The motors play the pattern's **number of beeps**, at the pattern's rhythm (0.18.1): gyro calibration done = 3 beeps, just disarmed = 2 beeps, cannot arm = 3 beeps, signal lost = 1 beep every 1 s, battery below minimum = one beep every 0.52 s. An "on" part shorter than 450 ms (Betaflight's `DSHOT_BEACON_MODE_INTERVAL_US`) is stretched to 450 ms, longer parts stay as they are. **Each beep's length is decided by the ESC**: DShot only has "beacon 1 – 5" commands, with no duration (Betaflight: "the DShot Beacon tone duration is determined by the ESC").
- **Only while disarmed**, and no sooner than 1.2 s after disarm (`DSHOT_BEACON_GUARD_DELAY_US`). The "just disarmed" beep therefore comes after 1.2 s.
- **In flight the motors cannot signal anything**: low battery, failsafe landing, "armed, low throttle" have no sound. Those conditions (`ARMING*`, `ARMED`, `RX_LOST_LANDING`) are removed from the motor list by the firmware even if the settings turn them on. For in-flight warnings: fit a buzzer, or watch the voltage on the transmitter (CRSF telemetry, later work).
- Only with the **DShot** protocol (PWM / Oneshot / Multishot have no commands). The startup report says it clearly: `beeper: motors (DShot beacon, disarmed only)`, `beeper: active buzzer + motors (...)`, or `beeper: FAILED - no buzzer pin set, and the motors can only beep with a DShot protocol`.
- It does not cut into other work: when another DShot command (reversing…) is in progress or the app is running a motor test, the beep is dropped, not queued.
- The "power-on" beep is off by default for the motors (the ESC plays its own startup tune). The "gyro calibration done" beep comes about 1 s after power-up: if the ESC is still starting up it may not play.
- The "silent on the bench" and "no signal-lost beep while the app is connected" rules apply to the motors as to the buzzer.
- `beeper.dshot_beacon_tone` (1 to 5): which ESC tone. Stream `battery` also has `motor_beeping` (condition the motors are playing + 1).

## App

**Battery and beeper** tab: live readings, a one-cell gauge (red / yellow / green by threshold), the thresholds, divider calibration (enter the voltage measured with a meter, the app computes from the voltage currently at the pin). **Current and capacity** block: current, mAh used, percent remaining; measurement source, sensor scale / offset, battery capacity; sensor scale calibration (enter the current measured with a clamp meter while the motors spin steadily, the app computes from the voltage currently at the sensor pin). The `Current ADC` pin is in the Pins and ports tab. The condition table has **two checkbox columns** (buzzer, motors), the tone pattern picture, the number of beeps the motors will play, a "Try" button; the note at the top of the table says what the craft currently has to beep with (buzzer, motors, both, or nothing). The tone pattern pictures, motor beep counts and the list of conditions the motors cannot play are all compared against the firmware (`tools/ui_reference.cpp` → `test_gui_math.py`).

## Verified (PC)

- `test_power` 119 checks; in `test_fw`: the virtual board's real battery (ADC + divider, voltage sag under load), arming block, failsafe on critical battery, buzzer measured on the virtual GPIO pin, silent on the bench.
- Motor beeps: `test_power` (beep count, spacing, no beeps when armed / within 1.2 s after disarm / on the bench / with non-DShot ESCs, across `millis()` wraparound), `test_motor` (one frame for all motors, not queued behind other commands, not cutting into motor tests), and the `test_fw` **craft without a buzzer** scenario: counting the actual DShot command frames going out on the motor pins (timing, tone, all 4 motors in one frame), no command frames while armed.
- Deliberate faults (`tools/mutate.py`).
- App: `test_e2e.py` (battery, beeper, silent on the bench, motor beeps), the real page in a browser (`test_battery_and_beeper_tab`).
- Current measurement: `test_power` group `[current]` (Betaflight's formula with hand-computed numbers, source choice, 1 Hz filter, mAh, gaps, NaN, percent remaining); `test_fw` group `current` (`WB_ONLY=current`): ADC sensor on the virtual board (10 A → 10 mAh in 3.6 s, changing scale / offset while running), virtual ESCs with extended telemetry (only enabled after hearing command 13 six times; summed current of 4 ESCs; motor speed not corrupted by current frames; ESC silent then restarting → asked again; restart while armed → asked only after disarm), wrong settings; the real page (`test_current_sensor_and_servos_on_the_page`).

## Not verified (needs a real board)

- Time of one `analogReadMilliVolts()` in the flight loop (read 50 times per second; see `status.loop_max_us`).
- ESP32-S3 ADC accuracy at both ends of the range; the pin voltage must not exceed ~3.1 V with a full battery.
- The passive buzzer uses an LEDC channel: not yet tried together with PWM / Oneshot motors (which also use LEDC).
- **Current measurement on a real board**: a real current sensor (ADC noise with motors spinning, whether more filtering is needed); whether real ESCs send extended telemetry after command 13, how often, the current unit of each ESC type (BLHeli_32 / AM32 / Bluejay). A 4-in-1 ESC that reports the total current on every channel gets multiplied by 4: report the ESC type if the reading is 4 times too high.
- Cell count detection with a real battery; the default thresholds (4.30 / 3.50 / 3.30 V) are Betaflight's for LiPo.
- **Motor beeps on real ESCs**: the simulation only checks that the command frames go out at the right time with the right tone. Whether the ESC (BLHeli_S / Bluejay / BLHeli_32 / AM32) beeps on every command, how long one beep lasts, what the five tones sound like, whether the command needs repeating: must be heard on the board. If the ESC skips beeps, report the ESC type and its firmware.
- Motors heat up when beeping for a long time (current through the windings): "signal lost" and "critical battery" beep continuously until the condition ends. Betaflight does the same; temperature not yet measured.
