# Tuning: filters, PID, Angle / Stabilize, Altitude Hold

The inner (rate) loop and the filter chain are a port of **Betaflight 4.5** (`flight/pid.c`, `sensors/gyro_filter_impl.c`, `flight/dyn_notch_filter.c`, `fc/rc.c`, `config/simplified_tuning.c`): same names, same units, same defaults, so the numbers from a Betaflight tune can be used directly and Betaflight tuning experience applies as is. Stabilize is a port of **ArduPilot** (`AC_AttitudeControl`, `AP_Math/control.cpp`).

Everything here has only been verified **on the PC** (simulation). No figure has been measured on a real craft yet: see the last section.

## 1. What the signal goes through

```
raw gyro ─ lowpass 2 ─ RPM notch ─ static notch 1, 2 ─ lowpass 1 (static / dynamic with throttle) ─ dynamic notch ─┬─► P, I  (error = setpoint − gyro)
                                                                                                                   └─ D-term notch ─ D lowpass 1 (static / dynamic) ─ D lowpass 2 ─► D
stick ─ rate (Actual) ─┬─ RC smoothing (PT3, cutoff from the measured frame rate) ─► setpoint
                       └─ feedforward (setpoint speed, computed per frame) ─ RC smoothing ─► F
P + I + D + F  ─ limit ±500 (roll, pitch) / ±400 (yaw) ─ /1000 ─► mixer
```

| Block | File | Notes |
|---|---|---|
| lowpass, notch | `core/filters.h` | PT1 / PT2 / PT3 (−3 dB cutoff exactly at the entered frequency) and BIQUAD (2nd-order Butterworth, state-variable form so changing the frequency while running does not glitch) |
| gyro filter chain | `core/gyro_filter.h` | exactly Betaflight's order |
| RPM filter | `core/gyro_filter.h` | 1 notch per motor × per harmonic (up to 3), placed exactly at the frequency the ESC reports. **Needs bidirectional DShot.** Works per motor: an ESC that loses telemetry only fades out its own motor's notches |
| dynamic notch | `core/dyn_notch.h` | 72-sample SDFT → 36 bins, finds up to 7 peaks per axis, one notch per peak. Needs a loop of 2000 Hz or more |
| PID | `core/pid.h` | I-term relax, anti-gravity, D max, TPA (including the low-throttle part), yaw lowpass |
| stick → setpoint | `core/setpoint.h` | measures the receiver frame rate, RC smoothing, feedforward (jitter, smooth, boost, max rate limit, averaging, yaw hold) |
| Angle, Stabilize | `core/attitude.h` | |
| sliders | `core/simplified_tuning.h`, `proto/settings_rules.h` | computed by the firmware, the app only displays the result |

Gains: `Kp = 0.032029 × P`, `Ki = 0.244381 × I` (yaw × 2.5), `Kd = 0.000529 × D`, `Kf = 0.013754 × F / 100`. PID sum 1000 = the full motor range.

## 2. Which craft the defaults are for

The default set (P 45/47/45, I 80/84/80, D 30/34/0, D max 40/46/0, F 120/125/120; gyro lowpass 250–500 Hz, D-term 75–150 Hz) is Betaflight's for a **high-thrust 5-inch quad** (thrust ~7 times its weight). In simulation:

| Simulated craft | Full-stick roll step in 60 ms | Notes |
|---|---|---|
| 5 inch, 650 g, 1.2 kg thrust per motor | ~4 % overshoot, reaches 90 % in < 110 ms | exactly the target of this set |
| 1 kg, thrust only 2 times its weight | ~27 % overshoot | flies but soft: the motors need 140 ms of full differential to reach 670 °/s |
| as above, **Master Multiplier 2.0** | ~17 % overshoot | |

Heavy or low-thrust craft: raise **Master Multiplier** in steps of 0.1 (usually stopping at 1.3 – 2.0). After each change: fly for 30 seconds then **touch the motors**. Hot motors = D too high or too little filtering.

## 3. Sliders (PID tab)

100 = 1.00 = default; range 0 – 2.00. Products are **truncated** to integers exactly as in Betaflight's C code (for example pitch D max at Dynamic Damping 2.0 is 57, not 58).

| Slider | Field | Affects |
|---|---|---|
| Damping | `slider_d` | D and D max on all axes |
| Tracking | `slider_pi` | P and I on all axes |
| Stick Response | `slider_ff` | feedforward |
| Dynamic Damping | `slider_d_max` | the gap between D max and D (0: D max = D, no more boost) |
| Drift – Wobble | `slider_i` | I only |
| Pitch Damping | `slider_pitch_d` | pitch D, D max relative to roll |
| Pitch Tracking | `slider_pitch_pi` | pitch P, I, feedforward relative to roll |
| Master Multiplier | `slider_master` | everything |

`slider_mode`: `RPY` (default, all three axes computed by the sliders), `RP` (yaw entered by hand), `OFF` (every field entered by hand). For an axis computed by the sliders, **the firmware overwrites** that axis's P/I/D/D max/F whenever it receives or loads the `pid` section, and the app reads back exactly the values in effect (the fields are locked).

The filters have two sliders of their own (Filters tab): `slider_gyro_mult` and `slider_dterm_mult` (0.10 – 2.00) multiply the lowpass cutoff frequencies; a filter that is off (0 Hz) stays off; the result does not exceed 1000 Hz. Notches, dynamic notch, and RPM filter are not governed by the sliders.

## 4. Filters

- **Dynamic lowpass 1**: the cutoff runs from `dyn_min` (throttle 0) up to `dyn_max` (full throttle) along the `dyn_expo` curve; updated at most every 5 ms and only when throttle changes by 1 %. `dyn_min = 0` → use the static frequency.
- **RPM filter**: does not run without bidirectional DShot (the Filters tab says so). `rpm_min_hz` (100): the notch does not go below this level and fades out over `rpm_fade_range_hz` (50) above it. Per-harmonic weight 0 – 100 %: 2-blade props usually only need harmonics 1 and 2.
- **Dynamic notch**: `count` 0 = off. A peak must be twice as high as the floor around it to be tracked; the notch is **steered** towards the peak (PT1 4 Hz, up to 10 times faster when the peak is high), it does not jump; the notches of one axis are always ordered by frequency. Frequencies below `min_hz` (the craft's own motion) do not pull a notch.
- The Filters tab draws a **frequency map**: each lowpass (with the range it slides over), the static notches, the position of the dynamic notches per axis and the RPM notches per motor **right now** (`tuning` stream).

Principle: more filtering → cooler motors, more delay; less filtering → more responsive, motors heat up more easily. Only push the gyro slider above 1.0 when the RPM filter is working well.

## 5. Angle and Stabilize

| | Angle (Betaflight) | Stabilize (ArduPilot) |
|---|---|---|
| Angle from stick | follows the **rate curve**: half stick gives less than half the maximum angle (0.5 → 12.4° with defaults) | **linear** (0.5 → 22.5°) |
| Feel | direct; `angle_strength` 50 = 5 °/s per degree of error, with stick feedforward | the target is **shaped** (acceleration and jerk limits, `stab_input_tc` 0.15 s): glides in, no overshoot on either simulated craft |
| Yaw | rate-based; `angle_earth_ref` 100 %: yawing while tilted makes the craft turn about the vertical axis | **holds the nose heading** with the yaw stick centred (turned by wind, it comes back); the target does not run more than 45° ahead of the craft |
| Gains | `angle_strength`, `angle_feedforward`, `angle_ff_smoothing`, `angle_earth_ref` | `stab_p[3]` (ATC_ANG_RLL/PIT/YAW_P, default 4.5), `stab_accel_rp` 1100, `stab_accel_yaw` 270 °/s², `stab_rate_ff` |

Common maximum angle: `rates.angle_limit`. Altitude Hold, Position Hold and failsafe always use Angle. In the two self-levelling modes, I-term relax is 5 times more sensitive (threshold 8 °/s instead of 40).

### The tilt angle the firmware "believes" in flight (`core/ahrs.h`)

The accelerometer of a flying craft **does not point to the vertical**: it senses thrust (always along the craft's own vertical axis, however much it tilts) and air drag, not gravity. A craft that tilts and accelerates still reads "level" on the accelerometer; only a few seconds later, when drag has grown as large as the horizontal pull, does it reveal the tilt. So in flight the gyro holds the attitude, and the accelerometer is only allowed to pull slowly:

| | Value | Notes |
|---|---|---|
| Accelerometer gain when armed | 0.25 /s (4 s time constant) | Betaflight's number (`dcm_kp`); ArduPilot DCM and PX4 use 0.2 |
| When disarmed, resting | × 10 | at this point the reading really is gravity |
| Just set down / just powered up | × 100 for 0.5 s (after 0.25 s at rest) | the horizon is reset after crashes, flips. While being turned by hand (> 15 °/s): × 1 |
| Filtering before use | 2nd-order lowpass at 10 Hz | motor vibration at 100 Hz and above stays below 1 % |
| Reading magnitude deviating from the resting value | weight decreases, **0 from 0.1 g** | a 25° altitude-holding tilt is already 1.1 g; climbs, falls, crashes are all ignored. The "resting value" is learned while disarmed, so an uncalibrated accelerometer (e.g. reading 1.08 g) still works |
| Gyro bias learning | `ki` 0.01; at most 0.5 °/s per minute, no more than 3 °/s | a few seconds of acceleration cannot teach a bias that does not exist |
| Compass (corrects yaw only) | 0.5 /s; multiplied by the same × 10 / × 100 factors when disarmed | set the craft down and the nose heading is correct again within about a second |

Consequences the pilot needs to know (measured in a simulation with drag, 5-inch craft):
- Holding **half stick for 2 seconds** (12.4° command): the firmware holds exactly 12° by its estimate, the craft really tilts to ~15° (3.4° off). Release the stick and 3 seconds later it is level again within 0.5°.
- Holding **0.8 stick for 4 seconds** (30° command, reaching 13 m/s): it really tilts to ~34°.
- After a fast stretch followed by a stop, the horizon may be 2 – 4° off for a few seconds and then recover by itself. This is a common limit of every estimator with only gyro + accelerometer (Betaflight too); with GPS or optical flow the real acceleration will be compensated (the Position Hold round).
- Version 0.5.0 and earlier used a gain of 2 /s: in the same test, the craft tilted **27° for a 12° command** and swung back 14° after the stick was released. **Do not fly Angle / Altitude Hold with firmware older than 0.6.0.**

## 6. Altitude Hold: self-measured hover throttle

Hover throttle (strictly: the **motor output level** at which the craft hovers) differs by up to 5 times between a heavy craft (≈ 50 %) and a racing quad (≈ 13 %), so **it is not a setting**:

- `HoverEstimator` (`core/althold.h`) measures continuously in **every flight mode**: `output level × cos(tilt) / (1 + upward acceleration / g)`. Skipped when tilted beyond 45°, with motors nearly off, or during strong acceleration / deceleration. Converges in ~2.5 s of manual flight.
- **Takeoff in Altitude Hold**: throttle ramps up at 40 %/s until the craft leaves the ground (climbing > 0.15 m/s or 8 cm above where it stood). The throttle level at that moment, after subtracting the current acceleration part, is the hover throttle. Reaching 90 % without leaving the ground (missing props, held down): back to idle, the throttle stick must be lowered before trying again.
- **Failsafe without altitude** (no baro, no lidar): the motors go to `fs_fallback_throttle` % of the **measured hover thrust**, so the craft surely descends. Version 0.5.0 and earlier used a guessed 40 %: a craft hovering below ~16 % would shoot upward. **Do not fly Altitude Hold / rely on failsafe with firmware older than 0.6.0 on a high-thrust craft.**
- The strongest throttle reduction Altitude Hold requests is 0.8 g, so the attitude loop still has motor authority to work with.
- **Altitude lost midway** (barometer not replying for more than 1 s): if the Altitude Hold switch is still on, the throttle stick is read around the measured hover throttle: mid stick = hover, bottom = 0, top = 100 %. Throttle does not jump; when altitude comes back, Altitude Hold continues from the current throttle level. The Overview tab reports "not valid" on the altitude line.
- **Pulling the stick down right after leaving the ground** = a descend command (powered descent until standing still), not motor cut. **Signal lost while throttle is rising for takeoff**: back to idle immediately; to take off again the stick must be lowered first.
- Throttle controlled by Altitude Hold / failsafe does not go through throttle boost and does not trigger anti-gravity (those two only follow your hand).
- Lidar: with a barometer, the lidar no longer "drags" the altitude along with the ground below. Flying over a table, a step: the craft holds altitude by the barometer, it does not jump with the obstacle.
- The PID tab shows "Measured hover throttle" (a guess before the first flight).

## 7. In-flight level trim

The craft still drifts with sticks centred in Angle / Stabilize: assign the **CALIB** mode ("In-flight level trim") to a switch.
1. Fly in a **windless** spot, hold the craft **stationary in one place** with the sticks (not just keep it "looking level": it must stop drifting).
2. Turn the switch on for at least 2 seconds then off: two short beeps = measured (a different sound = rejected: not 2 seconds of steady hover yet).
3. Land, disarm: the result becomes the new "level", and the firmware **writes it to flash by itself** (no app needed).

What is measured is **the accelerometer during the hover**: when the craft is hovering steadily the accelerometer only senses thrust, so its reading tells how far the thrust is tilted relative to the board — exactly the level error to correct (the way Betaflight / MultiWii do "in-flight ACC calibration"). The result does not depend on the horizon the firmware currently believes, so measuring right after a manoeuvre is still correct. Two things spoil it:
- **The craft is still drifting**: drag leaks into the reading, about 2° per 1 m/s. A 0.25 m/s drift → 0.5° error.
- **Wind**: standing still in wind means leaning into the wind. Measure indoors or when there is no wind.

Only time while armed counts, with the firmware seeing the craft **airborne** (not "throttle stick above 15 %": a high-thrust quad hovers below that level), rotating slower than 15 °/s, tilted below 10°. Nothing changes in flight. A measurement that is too short does not erase an earlier good measurement.

## 7b. Minimum throttle, the ground, safety at takeoff

- **Airmode** (Sensors tab). "While flying" (default): in the air the craft is still fully stabilised with the throttle stick at minimum; standing still on the ground for 1 second brings every motor to idle and the I-term to 0, so a craft placed on a bench does not spin up by itself. "Always on until disarm" is Betaflight's behaviour: use it when you cut throttle to free-fall for a long time and then catch it (a craft falling flat at terminal velocity reads 1 g on the accelerometer exactly as if standing still; without altitude the firmware cannot tell the difference).
- **Runaway takeoff prevention** (System tab, on by default): if something mounted wrong makes the PID flip the craft, the motors cut after 75 ms, and the reason appears under "Why it cannot arm". On a bench without props it does not trigger (the PID pushes but the craft does not rotate).
- Arming only works by **flipping the switch from OFF to ON while there is a signal**.

Details and reasons: `docs/HARDWARE_FEEDBACK_1.md` sections 5 – 6.

## 7c. Bench test without props: uneven motors are normal

If the craft cannot rotate, the PID loop gets no feedback. Just 0.2° off in Angle mode gives a 1 °/s error that never goes away; the I-term accumulates about 20 units per second, and after 10 – 15 seconds at high throttle the motors differ by tens of percent. Move a stick and release it: that axis's I-term hits its ceiling and **stays there**, so the craft seems to "remember" the last command. After throttle first exceeds 25 %, airmode gives the PID full authority even at very low throttle: moving yaw at 5 % throttle can push two motors to 80 %. Leaving the throttle stick at the bottom for 1 second brings everything to idle and the I-term to 0. Betaflight and ArduPilot behave the same; in flight, the craft rotates as commanded and the I-term unwinds by itself.

- Motor check: Motor tab, not armed.
- Reaction direction check: arm, 10 – 15 % throttle, tilt the craft by hand for a few seconds; the motors on the lowered side must speed up.
- To get the motors back to even when the stick is released: temporarily set I = 0 on all three axes, and restore it before flying.

Analysis of a real log with figures: `docs/HARDWARE_FEEDBACK_2.md` section 1.

## 8. Suggested tuning order on a real craft

0. Overview tab: "Actual control cycle" must equal the set rate, "0 cycles missed". If not, do not tune: every cutoff frequency and gain is off accordingly.
1. Props off: check motor direction (Motor tab), gyro orientation (3D view), enable bidirectional DShot if the ESC supports it.
2. Props on, a short Angle flight with defaults. Record blackbox (gyro + raw gyro + PID + motor groups).
3. Check whether the motors are hot and look at the gyro spectrum (Blackbox tab). A fixed peak → static notch; throttle-dependent noise without RPM filter → keep the dynamic notch at 3 – 5.
4. Soft craft, overshoot → raise Master Multiplier by 0.1 at a time. Fast oscillation after a throttle punch → raise Damping or Dynamic Damping. Slow drift after a flip → Drift – Wobble.
5. In-flight level trim, and only then try Altitude Hold.

## 9. Verified on the PC / not verified

Done (`make test`): each block checked against its formula (`test/test_control.cpp`), closed-loop flight on two simulated craft (`test/test_flight.cpp`), running inside the real firmware on the virtual board including ESCs replying with bidirectional DShot (`test/test_fw.cpp`); deliberate faults (mutation) for every meaningful line of the files above.

**Needs a real board, no figures yet:**
- The time of one loop with the full filter chain + dynamic notch + PID at 4 kHz on the ESP32-S3: Overview tab, "Per-stage timing" (the "computation" stage). Version 0.5 could not hold 4 kHz on a real board; this version has changed the loop structure (`docs/HARDWARE_FEEDBACK_1.md` sections 1 – 2) but **there are no measurements yet**. If it still cannot keep up: reduce `dyn_notch_count`, reduce the number of RPM harmonics, or lower `loop_hz` to 2000.
- RPM filter with real telemetry (frame error rate, latency).
- The default set on your frame; hover throttle measured with a real thrust curve (thrust is not linear in the output level as in the model).
- Whether prop wash blowing on the baro close to the ground makes takeoff detection early / late.
- The attitude estimator with a real accelerometer: the frame's vibration level, the craft's real drag coefficient (the simulation uses 0.35 /s: a 30° tilt reaches 16 m/s). The "3.4° off" figures in section 5 come from that model.
