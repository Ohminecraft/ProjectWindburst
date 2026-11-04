# Failsafe rewritten after PX4 / ArduPilot (0.32)

Checked on PC: unit tests (`test_core`) and firmware running on the virtual board (`test_fw`). **Not flown for real yet.**

## 1. Approach
Follows PX4's failsafe framework (Commander/failsafe) and ArduCopter's failsafes. Code in `src/core/safety.h`.
- **Each situation selects one action.** With several situations at once, the **most severe** action is taken.
- **Never becomes less severe** while the cause is still present.
- **Has a fallback:** if an action cannot be carried out, switch to one that can.

### Situations and corresponding settings
| Situation | Setting | Default |
|---|---|---|
| Signal loss (LINK) | `nav.fs_procedure` | LAND |
| Failsafe switch (SWITCH) | `nav.fs_procedure` | LAND |
| Battery at warning level (BATTERY_LOW, **new**) | `nav.batt_low_action`: WARN / RTL / LAND (ArduPilot BATT_FS_LOW_ACT, PX4 COM_LOW_BAT_ACT) | WARN |
| Battery at minimum level (BATTERY) | `nav.batt_crit_action`: LAND / RTL (ArduPilot BATT_FS_CRT_ACT) | LAND |

`fs_procedure` gains two new options (the enum is only appended at the end):
- **HOLD**: hold position, see the action table below. The hold time is `nav.fs_hold_s` (default 30 s).
- **DROP**: motors off (Betaflight's DROP procedure).

### Actions (from least to most severe)
| Action | What the craft does |
|---|---|
| WARN | No flight action, only warns (buzzer / OSD, `fs_cause` = BATTERY_LOW). The phase stays NONE |
| HOLD | Holds altitude with Altitude Hold; holds position with Position Hold if position is available (GPS + compass, or optical flow). Sticks centred. After `fs_hold_s` it switches to LAND. Without altitude it LANDs immediately |
| RTL | GPS Rescue flies home then lands. If it cannot fly (no fix / home / compass) it lands in place |
| LAND | Lands in place as before. With LINK / SWITCH: self-levelling, sticks centred. With BATTERY: the pilot still controls roll / pitch / yaw |
| DROP | Motors off, after the guard time |

## 2. Rules kept from before
- The guard time (`fs_guard_ms`) applies to LINK / SWITCH / BATTERY. Short signal losses accumulate.
- The signal must be back continuously for `fs_recover_ms` before the craft is handed back to the pilot. If Rescue is flying home, the pilot must move the sticks to take the craft back.
- A landing because of minimum battery is not cancelled.
- The `fs_max_descent_s` timeout only counts while **landing** without descending. While holding position, or while Rescue is flying home, it does not count.
- On the ground when the guard time expires, the motors are turned off.

## 3. New
- **BATTERY_LOW needs no guard time**, since the battery monitor already has its own delay: the action starts immediately.
- **Pilot takeover (PX4's user takeover):**
  - For actions caused by the battery warning (RTL / LAND), while the signal is still good, moving a stick beyond 0.30 takes the craft back.
  - That action is not repeated until the battery reaches the minimum level.
  - At minimum battery takeover is not possible.
- **Choosing the RTL action for the battery** makes the firmware treat Rescue as configured: GPS must have a fix to arm.
- **Telemetry:**
  - the `status` stream gains the field `fs_action`;
  - `fs_cause` gains the BATTERY_LOW bit;
  - the status bar also shows the action, e.g. "Failsafe: auto landing - holding position (signal lost)".
- **App:** the Position tab gains "Hold time", "When the battery reaches the warning level", "When the battery reaches the minimum level".
- **Settings:** nav compat is [31, 30, 29]. The 27-byte nav of very old versions is no longer accepted.

## 4. Tests
- `test_core` group `[failsafe 0.32]`:
  - severity order;
  - battery warning with WARN / RTL / LAND;
  - pilot takeover, and it not repeating;
  - switching to LAND when the battery reaches the minimum;
  - not becoming less severe on further signal loss;
  - RTL unable to fly lands in place;
  - HOLD then LAND, HOLD without altitude;
  - DROP after the guard time.
- `test_fw` group `[failsafe 0.32]`:
  - signal loss with HOLD: for 5 s the craft holds position within 4 m and altitude within 0.8 m, after 8 s it lands and turns the motors off;
  - battery warning with RTL: flies home; the pilot moves the sticks and takes the craft back, and Rescue does not restart by itself.
- All the old failsafe tests pass without any changes.

## 5. To try on the board
1. Failsafe switch with `fs_procedure = HOLD`, with GPS + compass: the craft stays put, and lands when the hold time runs out.
2. `batt_low_action = RTL`: let the battery reach the warning level while flying far away. Check that the craft flies home, and that moving the sticks takes it back.
3. `fs_procedure = DROP`: only test with the craft at a safe height over a mattress, or on the bench without props.
