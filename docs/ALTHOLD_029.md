# Altitude Hold / Position Hold rewritten after ArduPilot (0.29)

Only checked on PC so far: unit tests and firmware running on the virtual board. **Not flown for real yet.**

## 1. Why
The controller before 0.29 had these issues:
- There was only a position P → velocity PI loop. Desired acceleration was converted straight to throttle through the hover throttle model, with no feedback from measured acceleration.
- When the throttle stick jumped from "hold" to "climb 2 m/s", the target velocity jumped immediately. The velocity error jumped by 2 m/s, the requested acceleration jumped by kp_v × 2 = 10 m/s², and the motors jerked.
- The hover throttle estimate learned very fast (0.4 s), even while Altitude Hold was flying. In Altitude Hold it became one more integrator fighting the velocity integrator, a possible source of slow up-and-down oscillation.

## 2. The new controller (`src/core/althold.h`, `src/core/kinematics.h`)
After ArduPilot's `AC_PosControl::D_update_controller`; thrust is normalised to hover throttle like PX4.

1. **Jerk-limited target generation** (port of `shape_vel_accel`, `update_pos_vel_accel` from `AP_Math/control.cpp`):
   - The throttle stick gives the desired climb rate.
   - The target velocity approaches it with acceleration ≤ `althold.accel_z` (2.5 m/s², like PILOT_ACCEL_Z) and jerk ≤ `althold.jerk_z` (5 m/s³, like PSC_JERK_D).
   - Target position = integral of the target velocity.
   - When the motors saturate in one direction, the target does not keep moving in that direction (AP's limit vector).
2. **Position → velocity:** P (`kp_z` 1.2) plus the target velocity. The target is never more than 2 m from the craft.
3. **Velocity → acceleration:** P (`kp_v` 5). I defaults to 0, because the integrator lives in the acceleration loop like AP. The target acceleration is added as feed-forward.
4. **Measured acceleration loop (new):**
   - Error = requested acceleration − measured acceleration (IMU, earth frame), filtered at 20 Hz.
   - Thrust = hover × (1 + (a_requested + P·error + I) / g) / cos(tilt), with P = `althold.acc_p` (0.5) and I = `althold.acc_i` (1.0/s).
   - Effect: gusts, prop wash and battery sag are compensated immediately, without waiting for them to become a velocity error. The I term absorbs a wrong hover throttle estimate.
   - A simulated 3 m/s² downdraft for 1 s: noticeably less altitude lost than the old scheme (`test_core`, group `[althold 0.28]`).
5. **Over-speed** (AP's `calculate_overspeed_gain`): engaging Altitude Hold while the craft climbs / descends faster than the limit multiplies the stopping acceleration and jerk. The craft does not drift a few extra metres.
6. **Switching to Altitude Hold mid-flight:**
   - The target starts at the craft's current position, velocity and acceleration (`D_init_controller`).
   - The integrator is taken from the throttle in use, so throttle does not jump.
   - A climbing craft stops smoothly with limited acceleration, instead of being yanked back to the engage point.
7. **Takeoff:**
   - Throttle still ramps up gradually until it leaves the ground, as before.
   - Then it climbs to the takeoff altitude with a speed decreasing as √(2·a·d), including the target generator's stopping distance, so it does not overshoot the altitude.
8. **Hover throttle:**
   - While Altitude Hold is flying: learns slowly (3 s), since here it is only feed-forward (AP learns over 10 s).
   - While flying manually: still learns fast (0.4 s), because the failsafe landing without an altitude estimate only has this number.

## 3. Position Hold / GPS Rescue (`src/core/poscontrol.h`)
- The desired velocity (Rescue flying home) also goes through a jerk-limited 2-D target generator: acceleration ≤ 2.5 m/s², jerk ≤ 5 m/s³ (AP's `input_vel_accel_NE`).
- The target acceleration is added as feed-forward.
- The position / velocity loops are unchanged, as in AP: sqrt controller, PID with a 5 Hz filter, tilt limit.

## 4. Unchanged
The following are unchanged, since they already follow ArduPilot / EKF and have tests:
- the altitude estimator (4-state Kalman: baro / lidar / GPS);
- the horizontal estimator;
- optical flow;
- ArduCopter-style brake / loiter Position Hold.

Changing these needs real flight logs showing what is wrong.

## 5. New settings (section althold, compat [6])
| Field | Default | Meaning |
|---|---|---|
| `accel_z` | 2.5 m/s² | maximum acceleration when changing climb rate |
| `jerk_z` | 5 m/s³ | maximum jerk |
| `acc_p` | 0.50 | acceleration loop P. 0 = no acceleration feedback |
| `acc_i` | 1.00 /s | acceleration loop I |

App: Rates tab, block "Throttle stick in Altitude Hold".

## 6. Change in flight feel (intentional)
- Starting / stopping a climb is softer: about 1 s to reach 2 m/s.
- Releasing the stick at 2 m/s, the craft stops about 1.5 m higher.
- Rescue accelerates at most 2.5 m/s².
- If it feels too soft, increase `accel_z` / `jerk_z`. If it oscillates up and down quickly, decrease `acc_p`.

## 7. To try on the board
1. Hover in Altitude Hold for 1 minute.
   - Up-and-down amplitude, compared with 0.28.
   - In the blackbox: the integrator (`integrator`) is stable, not drifting.
2. Push the throttle fully up then back to centre: smooth climb / stop, no motor jerk.
3. Fly manually then engage Altitude Hold while climbing: stops smoothly, throttle does not jump.
4. In wind or fast horizontal flight: altitude held better than before.
5. Try `acc_p` = 0 to compare with the old scheme.
