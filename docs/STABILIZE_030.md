# Stabilize rewritten after AC_AttitudeControl (0.30)

Only checked on PC so far (unit tests + firmware on the virtual board). **Not flown for real yet.**

## 1. What changed
- **Before 0.30:** the error was computed per Euler angle (target roll − roll, target pitch − pitch, target heading − heading), and only then converted to body rates.
  - Only correct at small tilt.
  - When rolling and pitching at the same time, or at large tilt, the axes mix.
  - Correcting heading could pull the tilt off.
- **From 0.30:** uses ArduPilot's quaternion controller (`AC_AttitudeControl::attitude_controller_run_quat`), see `src/core/attitude.h`.

## 2. Steps
1. **Pilot command** (`input_euler_angle_roll_pitch_euler_rate_yaw`), unchanged from before:
   - the roll / pitch sticks give an **angle**, the yaw stick gives a **rotation rate**;
   - each axis is shaped (`attitude_command_model`): acceleration limit, input time constant;
   - the result is an Euler target together with its rate and acceleration.
2. **Both the target and the craft are quaternions.** The error is split like `thrust_vector_rotation_angles`:
   - **first**: the rotation that brings the thrust vector (the craft's "up") onto the target thrust vector, giving the x / y components;
   - **then**: the remaining heading part about that axis, giving the z component.
   - So a heading error never disturbs the tilt.
3. **Heading error at most 45°.** When the craft cannot keep up (held, saturated), the target is dragged along with the craft.
4. **Each body axis:** square-root controller with the Stabilize P gain and half the acceleration limit (`update_ang_vel_target_from_att_error`).
5. **Feed-forward:**
   - the target's Euler rates are converted to body rates **at the target attitude** (`euler_rate_to_ang_vel`), then rotated into the craft's frame;
   - when the thrust error is between 30° and 60°, feed-forward is reduced gradually;
   - above 60°, yaw is left to follow the gyro (like ArduPilot: recover thrust first).
6. **Target acceleration** (feed-forward for the rate loop) is also computed in the body frame.

Internally the code uses ArduPilot's frame (pitch + = nose up). The outward interface still follows the firmware convention (pitch + = nose down), so the other parts are unchanged.

## 3. Settings
Unchanged: `stab_p` (Roll / Pitch / Yaw P), `stab_accel_rp`, `stab_accel_yaw`, `stab_input_tc`, `stab_rate_ff`. Same meaning, same defaults as ArduPilot.

## 4. Tests (`test_control`, group `[stabilize 0.29]`)
- Error split function:
  - heading error only → all of the error is in z;
  - roll error only → x only;
  - on target while rolled and pitched at once → zero error.
- Tilted 40°, heading off by 40°: returns to the right heading, tilt changes by less than 3° while turning.
- Large errors on all three axes (60 / 50 / 120°): returns to level; thrust is corrected first, after 0.5 s the thrust error is below 20°.
- Thrust error above 60°: yaw left to follow the gyro.
- Nose up 60°, heading off by 10°: most of the command is roll, because the craft turns about the earth's vertical axis. The old Euler-difference scheme would put it all into yaw.
- Target pitched down 30° and yawing at 60°/s: body rates exactly 60·sin30 for roll and 60·cos30 for yaw. The acceleration feed-forward has the corresponding roll part too.
- Craft rolled 30° off a rotating target: the target's rotation rate is rotated into the craft's axes (has a pitch part).
- The flight tests on the physical plant (`test_flight`: tilt step, yaw, wind) still pass without changing thresholds.
- 25 `stab:` mutants rewritten for the new code, all caught by the tests.

## 5. To try on the board
1. Stabilize, flick the roll or pitch stick quickly and return to centre: no overshoot, feels the same as before.
2. Large tilt combined with yaw rotation (flying circles): tilt stays steady.
3. A gust knocks the heading off: returns to heading without the craft wobbling.
