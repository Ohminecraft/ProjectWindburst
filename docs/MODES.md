# Switch-assigned modes (0.23), stick arming (0.24)

Four new modes in the **Modes** tab, added at the end of the `mode` enum (old numbers unchanged): `ACRO_TRAINER` (10), `AIRMODE` (11), `ANTI_GRAVITY` (12), `BLACKBOX_ERASE` (13). Over MSP, the firmware reports them with Betaflight's fixed box ids: ACRO TRAINER 47, AIR MODE 28, ANTI GRAVITY 4, BLACKBOX ERASE 31. Goggles or the transmitter can read which mode is on.

## Acro trainer (`ACRO_TRAINER`)
Same as `applyAcroTrainer()` in Betaflight 4.5's `flight/pid.c` (`src/core/acro_trainer.h`).
- **When it runs:** only in Acro. Angle, Stabilize, the hold modes and GPS Rescue already limit the angle themselves, so the trainer does nothing there.
- **The sticks still control rotation rate as in Acro.** The trainer only replaces the roll / pitch setpoint when the craft is about to exceed the limit angle:
  - Already past the angle and the stick still pushes that way: setpoint = (limit − angle) × gain. The craft is held at the limit until the stick reverses.
  - About to pass it (angle + rotation rate × lookahead time): setpoint = (limit − predicted angle) × gain. The lookahead time grows with rotation rate: 0 when still, the full `lookahead` when rotating at 500 °/s or more.
  - Each time the trainer takes over, that axis's I-term goes to 0.
  - The angle-correcting setpoint is limited to ±1000 °/s.
- **Settings** (PID tab, "Acro trainer" block; section `pid`, takes effect immediately):

  | Field | Default | Range | Meaning |
  |---|---|---|---|
  | `acro_trainer_angle_limit` | 20 ° | 10 – 80 | maximum tilt angle |
  | `acro_trainer_lookahead_ms` | 50 ms | 10 – 200 | lookahead time |
  | `acro_trainer_gain` | 75 | 25 – 255 | strength (Betaflight: divided by 10) |

- **On the simulated craft** (`test_flight`): holding full roll stick for 1.5 s, the craft stops at about 20 °. The largest overshoot is about 34 °, because full stick is 670 °/s. Without the trainer, the craft flips past 170 °.

## Airmode by switch (`AIRMODE`)
- **Switch assigned:** the switch decides airmode.
  - Switch off: no airmode. Throttle at min puts every motor at idle, whatever `general.airmode` is set to.
  - Switch on: airmode runs in the style set by `general.airmode` (`AUTO` or `ALWAYS`). If `general.airmode` is `OFF`, `AUTO` is used.
- **No switch assigned:** as before, only `general.airmode` decides.
- **Different from Betaflight:** in Betaflight, the AIR MODE box only has an effect when the AIRMODE feature is off (either one being on is enough). Here an assigned switch wins, so no other setting needs to change.

## Anti Gravity by switch (`ANTI_GRAVITY`)
- **Switch assigned:** Anti Gravity only runs while the switch is on, regardless of `pid.anti_gravity`. The gain / cutoff numbers still come from section `pid`.
- **No switch assigned:** `pid.anti_gravity` decides, as before.

## Blackbox erase by switch (`BLACKBOX_ERASE`)
Same as Betaflight's BLACKBOX ERASE box.
- **How to erase:** flip the switch from off to on while disarmed. The firmware erases like the "Delete" button in the Blackbox tab: erases the flash, or deletes the log files on the SD card. When done the beeper plays the `BLACKBOX_ERASE` tone. Arming is not possible while erasing.
- **Only a new flip counts:**
  - A switch already on at startup does not erase.
  - Holding the switch on erases only once.
  - A flip while armed is ignored.
- **A rejected flip is not retried:** for example while logging on the ground (log mode `ALWAYS`) or when there is no log storage. To erase, flip again.
- Source: `Service::poll()` in `src/blackbox/bb_service.h`, using `Inputs::erase_switch`.

## Flying style: FPV and Flycam (`general.fly_style`, 0.43)
**Modes** tab, "Flying style" block. Needs save and restart.

- **FPV** (default): as before, every mode follows the switches, including Acro.
- **Flycam**: for camera operators, flies like a DJI craft.
  - **Angle + Altitude Hold + position hold** always on, per `flycam_hold`: **Loiter** (default, stick = velocity, release to brake and stop, like DJI's P mode) or **Position Hold**. The Acro, Stabilize, Acro Trainer switches are ignored. The GPS Rescue, Mission, Crash flip, Failsafe switches still work.
  - **Throttle stick at centre = hold altitude.** Arming is allowed when the throttle stick is **not above centre** (at centre or at the bottom, so DJI-style stick arming still works). Throttle above centre blocks arming (`THROTTLE_HIGH`).
  - **Takeoff:** push up from centre to take off, no need to pull to the bottom first as in FPV. In FPV you still have to pull to the bottom first, so the craft does not climb by itself if armed by mistake with the throttle high. If the throttle is above centre at arming or right after landing, it must go back to centre before takeoff is possible.
  - **Landing:** pull the stick down and the craft descends. After touchdown, holding the stick at the bottom for 3 seconds stops the motors (DJI). To fly again, flip ARM to OFF and back on.
  - **No GPS / optical flow:** can still arm, only holds altitude (like DJI's ATTI). The app shows "position hold: nothing to hold". **No baro means no arming** (`ALTHOLD_NO_BARO`).
- **Code:** `CoreConfig::flycam` / `flycam_loiter` in `src/core/flight_core.h`, `AltConfig::centred_stick` in `src/core/nav/alt_control.h`.
- **Verification:**
  - `test_flight` flycam group: forced mode, arming at centre, push up to take off, altitude and position hold, motors off 3 s after landing, Position Hold, no flow, no baro, FPV unchanged;
  - `test_fw` group `flycam`;
  - `test_proto` (section `general` 59 bytes; 57, 56, 24, 22 still accepted; the 23 bytes of 0.9.0 – 0.9.4 dropped);
  - app `test_fly_style_setting`;
  - 12 mutants `flycam:`.
  - **Not yet flown.**

## Crash flip (`CRASHFLIP`, 0.42)
Betaflight's Turtle mode, written from the real code: `mixer.c` (`applyCrashFlipModeToMotors`) and `core.c` (`tryArm`, `updateArmingStatus`, `disarm`). A craft that lands upside down is flipped back with the sticks instead of walking over to pick it up.

**How to use:** assign a CRASHFLIP switch in the **Modes** tab. Craft upside down: turn CRASHFLIP on, then arm. The motors spin in reverse, and each stick picks which motors run:

| Stick | Motors running (Quad X, "props in") |
|---|---|
| pitch forward | the two front motors |
| pitch back | the two rear motors |
| roll right / left | the two right / left motors |
| near a diagonal (within 30°) | the one motor in that corner |
| yaw (when it is the largest of the three sticks) | a diagonal pair: yaw right is the two clockwise props (props out: the other pair). Use it when one prop is stuck |

- The further the stick, the stronger the motors (length of roll + pitch). Within 15 % of centre: no motor runs.
- Unselected motors **stand still**, they do not run at idle. Commands below 2 % of travel also mean standing still.
- Flip CRASHFLIP off: the craft disarms, reason `CRASHFLIP` shown until ARM is flipped to OFF. Flip ARM OFF then ON to fly normally.
- The switch is only checked **at arming**. Turning CRASHFLIP on in flight does nothing.

**Settings** (**Motors** tab, "Crash flip" block; needs save and restart), names and defaults as in Betaflight:

| Setting | Betaflight | Default | Meaning |
|---|---|---|---|
| `motors.crashflip_rate` | `crashflip_rate` (x10) | 0 | °/s. When the craft rotates faster than half this value the power fades, at this value the motors stop. When the attitude has changed past the edge (cos of tilt changed by more than 1) the motors also stop: the flip is done. 0 = no fade. Try 350 |
| `motors.crashflip_motor_pct` | `crashflip_motor_percent` | 0 | the motors on the opposite side run at this % of power. 0 = stand still |
| `motors.crashflip_auto_rearm` | `crashflip_auto_rearm` | off | on: flipping CRASHFLIP off keeps the craft armed and it flies on (motors stand for about 1 s to restore spin direction) |

**Reversing spin direction:** when arming into crash flip, every ESC receives DShot command 21 (spin opposite to the direction stored in the ESC, not saved), 10 times as in Betaflight. The motors only run after the last command. When crash flip ends (disarm, or switch off with auto re-arm): DShot command 20 (restore the stored direction), sent once the motors have slowed down (1 s after the last frame with a spin command). Commands 7 / 8 + 12 from the "Reversed" button in the Motors tab are not touched.

**Arming conditions:**
- CRASHFLIP switch on: arming is not blocked for lying tilted / upside down (`ANGLE_TOO_HIGH`), nor for GPS Rescue lacking a fix (`RESCUE_NO_FIX`), as in Betaflight. Other conditions stay (low throttle, gyro calibrated, link present...).
- Non-DShot ESCs (PWM, Oneshot, Multishot, DroneCAN): turning CRASHFLIP on **blocks arming** (`CRASHFLIP`). Betaflight in this case still skips the tilt check and arms with the motors spinning forward. Here it is simply blocked for safety.

**During crash flip:**
- Signal loss: the sticks are treated as centred so the motors stand. Failsafe works as before.
- No PID, Altitude Hold, runaway takeoff, motor stop detection. Motors stuck in grass are normal while flipping.
- OSD and CRSF show `FLIP`. The `CRASHFLIP` flag is in `nav.flags`, the DIY transmitter's `fly_mode` is `CRASHFLIP`.

**Different from Betaflight:**
- Arming blocked without DShot (see above).
- Command 20 at the end waits for the motors to slow down before being sent. Meanwhile the motors stand. Betaflight drops the command if the motor is not at 0, so auto re-arm can leave the motors spinning in reverse.
- Normal arming (not crash flip) does **not** send command 20. Betaflight sends 20 on every arm.
- No dedicated beeper tone for crash flip.
- The sticks go straight through, not through the rate curve. Betaflight reads stick deflection the same way.

**Source:**
- `src/core/crash_flip.h` (motor commands);
- `FlightCore` (when it turns on / off);
- `MotorOutputManager::update(..., flip)` (DShot 21 / 20, motors standing).

**Verification:**
- `test_core` (values computed by hand from Betaflight's formulas for Quad X, hexa);
- `test_motor` (21 x10 before flying, motor 0 standing still, 20 after slowing down, auto re-arm, re-arming while waiting, pulse protocols);
- `test_flight` (arming upside down, switch, auto re-arm, signal loss, rotation rate and attitude through the real AHRS);
- `test_fw` group `crashflip` (virtual ESC listens for 21 / 20, sticks, % and rate settings, Oneshot125 blocked);
- `test_osd`, `test_rx`, `test_radio`, `test_proto`, app (`test_crash_flip_on_the_page`);
- 36 mutants `crashflip:`.
- **Not yet tried on a real craft.** It is also not yet known which ESCs accept commands 20 / 21: BLHeli_32, AM32, Bluejay do; BLHeli_S needs version 16.7 or newer.

## Stick arming (`general.stick_arm`, 0.24)
Turned on in the **Modes** tab, "Arm with the sticks" box. Needs save and restart. Off by default.
- **Gesture:** pull the throttle and pitch sticks all the way down, while pushing the yaw and roll sticks to opposite sides (yaw left + roll right, or yaw right + roll left). On a mode 2 transmitter, that is both sticks down and both pointing outward, or both pointing inward (like DJI's "combination stick"). Each stick must go past 80 % of its travel.
- **Arm:** hold the gesture for 1 second while disarmed. The arming-block conditions still apply as with a switch.
- **Disarm:** hold the same gesture for 1 second while the craft is on the ground. It never disarms in flight, because a pilot may move the sticks exactly like that while flying.
- **The sticks must be released between two gestures,** and also after power-up: holding the gesture at power-on does not arm.
- **ARM switch assigned:** the switch becomes an enable and emergency cut-off.
  - The switch must be on for the gesture to arm.
  - Flipping it off disarms immediately.
  - Flipping the switch back on does not arm by itself; the gesture must be repeated.
- **No ARM switch assigned:** only the gesture arms and disarms. Assigning an ARM switch is recommended so the motors can be stopped instantly.
- **Arming blocked:** the virtual switch (turned on by the gesture) goes back off when the sticks are released, so "ARM switch already on" is no longer reported. The reason for a forced disarm (runaway, motor stop) is still shown until the next gesture begins.
- **Signal loss:** the gesture has no effect. Failsafe handles it as before.
- Startup report: `arming: sticks: throttle + pitch down, yaw and roll to opposite sides, 1 s (disarm: the same, on the ground)`, plus `; the ARM switch must be on` if an ARM switch is assigned.
- Source: `StickArm` in `src/core/safety.h`. It creates a virtual switch fed into `Arming`.

## Verification
- `test_control` acro trainer group: each branch compared with Betaflight's code (hold, lookahead, stick reversal, sign of 0, 1000 °/s limit).
- `test_flight` switch modes group: trainer in flight (roll, pitch, stick reversal, not running in Angle, 45 ° limit), airmode and anti-gravity by switch.
- `test_blackbox`: erase switch (already on at startup, flip, hold, while armed, while logging).
- `test_fw` group `switch`: mode table, `status.modes`, trainer settings taking effect immediately, blackbox erase by switch on the virtual board.
- `test_esc4way`: Betaflight-style BOXIDS / STATUS.
- `test_proto`: the old `pid` section (75 bytes) is still readable.
- Stick arming: `test_flight` stick arming group (gesture, 1 s hold, stick release, held at power-up, in flight, blocked, ARM switch as cut-off), `test_core` (keeping the disarm reason), `test_fw` group `stickarm`, `test_proto` (old general section), 12 mutants `stickarm:`, `test_gui_browser::test_stick_arm_setting` (cannot run on this machine yet).
- 17 mutants `modes:`. `test_gui_browser::test_switch_modes` is written but cannot run on this machine yet.
- **Not yet tried on a real craft.**
