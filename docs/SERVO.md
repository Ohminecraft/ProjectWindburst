# Servo (firmware 0.7.1, bug fix in 0.7.2)

Up to **4 servos**, each following an RC channel (camera tilt gimbal on a knob, gripper on a switch...).

**Not run on a real board yet.** Verified on PC: real firmware on the virtual board (PWM pulses measured at a simulated PWM unit), compiled for ESP32-S3.

**0.7.2:** fixes a 0.7.1 bug: the firmware read the servo settings through a possibly unaligned pointer, which on the ESP32-S3 could cause a crash at boot or when changing servo settings. If you flashed 0.7.1, flash 0.7.2.

## Usage
1. **Pins and ports** tab, "Servo" group: enter the GPIO for servos 1..4. **Save and reboot.**
2. **Motor** tab, "Servo" block: choose the RC channel for each servo (CH1..CH16, "Off" = no pulses), minimum / maximum pulse, reverse. Takes effect immediately, no reboot needed. The "Output" column shows the pulse being sent.
3. "Servo pulse rate": 50 Hz for analog servos (default), up to 330 Hz for digital servos. Changing the rate needs save and reboot.

## What the firmware does
- RC channel at 1000 µs → "minimum" pulse, 2000 µs → "maximum" pulse, linear, clamped at both ends; "reverse" swaps the two ends.
- No RC frame received yet (just powered up, no receiver yet): the servo sits in the middle of the range. **Signal loss: the servo holds its last position.**
- Hardware: the ESP32's LEDC PWM unit (14 bits). The ESP32-S3 has **8 shared LEDC channels**: motors using the PWM / Oneshot protocol and the passive buzzer also use them. With DShot, all 8 channels are free. Out of channels: the boot report says `servo: FAILED - no free PWM (LEDC) channel ...`.
- A servo pin conflicting with another pin (motor, ADC...): `pins: FAILED ...` and arming is not allowed.
- Code: `src/hal/servo/servo_out.h`; settings: section `servo` (id 18, live), `pins.servo[4]`; the `motors` stream gains `servo[4]` (µs).

## Hardware notes
- Power the servos from a separate 5 / 6 V BEC, **with a common GND** to the board. Do not power servos from the ESP32's 3.3 V.
- The ESP32's 3.3 V pulses are enough for most servos. A servo that does not accept them needs a 5 V level shifter.
- Many servos tolerate 500..2500 µs, but start with 1000..2000 µs and widen gradually: a servo forced against its mechanical stop gets hot and draws current.

## Verification
`test_fw` (group `servo`, `WB_ONLY=servo ./build/test_fw`): boot report, PWM rate / resolution, channel → pulse mapping (with reverse), clamping at both ends, changing the pulse range while running, holding position on signal loss, servo off, centred pulse before the first RC frame at 200 Hz, pin conflict with a motor, out of PWM channels. There is no dedicated browser test yet for the Servo block in the Motor tab.
