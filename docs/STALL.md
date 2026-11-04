# Disarm when a motor stalls (0.16.0)

`motors.stall_disarm` (default **off**). When a prop hits something, a motor jams or an ESC desyncs, the firmware shuts down all motors, before the remaining motors flip the craft into the ground or the jammed motor burns out. It needs each motor's speed, from **bidirectional DShot** or the **ESC telemetry wire**. A motor without a speed reading is not judged: missing replies are a telemetry fault, not a stalled motor.

While armed, each motor with a command ≥ `stall_min_cmd` (default 25 %) is checked against two conditions:
- **stopped**: speed < `stall_min_rpm` (1500 rpm);
- **dropped**: speed < `stall_drop` (35 %) of the highest speed since that motor's command last decreased. When the command decreases the motor naturally slows down, so the reference is reset.

Either condition lasting `stall_ms` (100 ms) disarms:
- the `MOTOR_STALL` blocker holds until ARM is switched OFF;
- the `motors` stream says which motor (`stall_motor`);
- the Motor tab has a "Motor stalled mid-flight" block.

Code: `src/core/safety.h` (`StallGuard`), running in `FlightCore` (step 9b, after runaway takeoff).

## Tests
- `test_core`: the rules (a 50 ms glitch does not count; a drop with an unchanged command is caught; with a decreasing command it is not; idle, missing telemetry, disarmed, disabled are not judged).
- `test_flight`, 5-inch quad with per-motor speed derived from thrust:
  - 6 flips, rolls, yaw spins, full throttle then throttle cut: **not a single false alarm**;
  - motor 3 stopping in mid-air: disarm after 118 ms.
- `test_fw` group `stall`, virtual board with bidirectional DShot:
  - motor 2 stopping in flight: disarm within 250 ms, the stream reports motor 2;
  - with the setting off: nothing disarms.
- Mutants: `python3 tools/mutate.py stall:` (8).
- **Not tried on the board yet.** Needs testing with real props: a light touch while hovering low, and aggressive flying to make sure there are no false alarms. Thresholds may need tuning per craft.
