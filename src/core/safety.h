#pragma once
#include "../config/fast_code.h"
#include "types.h"

// ============================================================================
// Safety: arming checks, failsafe, runaway take-off prevention, land detection, the arming state machine.
// The failsafe shares state with Altitude Hold (controlled descent from baro / range) instead of cutting the motors - a
// filming craft carries a gimbal. Each part is described where it stands.
// ============================================================================
namespace fc {
enum ArmBlock : uint32_t {
    AB_NONE = 0, AB_NO_IMU = 1u << 0, AB_GYRO_NOT_CAL = 1u << 1, AB_RX_LOST = 1u << 2, AB_THROTTLE_HIGH = 1u << 3,
    AB_ANGLE_TOO_HIGH = 1u << 4, AB_FAILSAFE = 1u << 5, AB_ALTHOLD_NO_BARO = 1u << 6, AB_BATT_LOW = 1u << 7,
    AB_ARM_SWITCH_ON = 1u << 8, AB_CONFIG_UNSAVED = 1u << 9, AB_CALIBRATING = 1u << 10, AB_POSHOLD_NO_GPS = 1u << 11, AB_MAG_NOT_CALIBRATED = 1u << 12,
    AB_PIN_CONFLICT = 1u << 13, AB_LOOP_RATE = 1u << 14, AB_RUNAWAY_TAKEOFF = 1u << 15, AB_RESCUE_NO_FIX = 1u << 16, AB_RESCUE_NO_COMPASS = 1u << 17, AB_MOTOR_STALL = 1u << 18,
    AB_CRASHFLIP = 1u << 19,          // the crash flip switch with motors that cannot turn round (no DShot), or a crash flip ended by its switch (the arm switch must go off)
};

struct ArmInput {
    bool imu_ok = false, gyro_calibrated = false, rx_ok = false, failsafe_active = false, baro_ok = false, gps_fix = false;
    bool althold_switch = false, poshold_switch = false, arm_switch_was_on_at_boot = false, calibrating = false;
    bool use_mag = false, mag_calibrated = false;       // compass enabled in the settings but not calibrated => refuse to arm
    bool batt_low = false;                              // the battery monitor says: warning level or worse
    bool pin_conflict = false;                          // two functions share a GPIO (found at boot): an output may be dead
    bool loop_slow = false;                             // the control loop does not reach its configured rate: every filter and gain assumes that rate
    bool throttle_low = false;                          // the throttle stick is below min_check
    bool pos_source = false;                            // something Position Hold can work with: a GPS fix (and a heading), or an optical-flow sensor that answers
    bool rescue_configured = false, rescue_allow_no_fix = false;       // GPS Rescue is set up (a switch, or the failsafe procedure): home is where the craft is ARMED - with a fix
    bool rescue_no_compass = false;                     // ... but the compass is switched off: the rescue could never fly
    float tilt_deg = 0, max_arm_angle_deg = 25.0f;
    bool crashflip = false, crashflip_ok = false;       // the CRASHFLIP switch is on; the motors can be turned round (DShot): then upside down and without a GPS fix it arms (Betaflight)
};
inline uint32_t arm_blockers(const ArmInput& a) {
    uint32_t b = 0;
    if (!a.imu_ok) b |= AB_NO_IMU;
    if (!a.gyro_calibrated) b |= AB_GYRO_NOT_CAL;
    if (!a.rx_ok) b |= AB_RX_LOST;
    if (!a.throttle_low) b |= AB_THROTTLE_HIGH;
    const bool flip = a.crashflip && a.crashflip_ok;
    if (a.tilt_deg > a.max_arm_angle_deg && !flip) b |= AB_ANGLE_TOO_HIGH;
    if (a.crashflip && !a.crashflip_ok) b |= AB_CRASHFLIP;
    if (a.failsafe_active) b |= AB_FAILSAFE;
    if (a.althold_switch && !a.baro_ok) b |= AB_ALTHOLD_NO_BARO;       // spec: Altitude Hold requires baro, warn immediately
    if (a.poshold_switch && !a.pos_source) b |= AB_POSHOLD_NO_GPS;
    if (a.rescue_configured && !a.gps_fix && !a.rescue_allow_no_fix && !flip) b |= AB_RESCUE_NO_FIX;
    if (a.rescue_no_compass) b |= AB_RESCUE_NO_COMPASS;
    if (a.batt_low) b |= AB_BATT_LOW;
    if (a.arm_switch_was_on_at_boot) b |= AB_ARM_SWITCH_ON;
    if (a.calibrating) b |= AB_CALIBRATING;
    if (a.use_mag && !a.mag_calibrated) b |= AB_MAG_NOT_CALIBRATED;
    if (a.pin_conflict) b |= AB_PIN_CONFLICT;
    if (a.loop_slow) b |= AB_LOOP_RATE;
    return b;
}

// ---------------------------------------------------------------------------------------------------------------------
// Failsafe: what the craft does when the pilot can no longer fly it - since 0.32 built the way PX4's failsafe framework
// (Commander/failsafe) and ArduCopter's failsafes are: every CONDITION asks for an ACTION, the most severe action asked for
// is what is flown, an action is never softened while its cause lasts, and an action that cannot be flown falls back to the
// next one that can.
//
//   conditions  LINK         no receiver frames                                     -> nav.fs_procedure
//               SWITCH       the pilot's failsafe switch (a test of the above)      -> nav.fs_procedure
//               BATTERY_LOW  the pack at its warning level                          -> nav.batt_low_action  (WARN, RTL, LAND)
//               BATTERY      the pack at its minimum                                -> nav.batt_crit_action (LAND, RTL)
//   actions     (least to most severe)
//               WARN   nothing is flown; the beeper / OSD say so
//               HOLD   the craft holds its place (Position Hold, if it has a position) and its height, for fs_hold_s; then LAND
//               RTL    GPS Rescue flies home and lands there
//               LAND   comes down where it is (with a height: at a controlled rate; without: at a fixed fraction of the hover
//                      throttle). LINK / SWITCH: levelled, sticks neutral. BATTERY: the pilot keeps roll / pitch / yaw - he can
//                      still choose where it comes down (ArduCopter's LAND).
//               DROP   the motors stop (Betaflight's failsafe procedure DROP: a craft that must not fly on by itself)
//   fallbacks   RTL that cannot fly (no fix, no home, no compass: the caller says so with rescue_active false) -> LAND;
//               HOLD without a height -> LAND (the open-loop descent)
//
//   GUARD     the first guard_s of a LINK / SWITCH / BATTERY cause: the sticks read neutral and the craft is levelled (LINK /
//             SWITCH), the throttle stays where it was. The cause goes away -> back to normal AT ONCE (Betaflight's stage 1).
//             Short dropouts add up: the guard time already used drains only as fast as it was filled, so a link that is gone
//             half of the time still ends in a descent. BATTERY_LOW needs no guard: the battery monitor has its own delays.
//   DESCENT   an action is flown (the name is older than HOLD and RTL: any action but WARN). The causes end by themselves:
//             the switch at once, the link after recover_s without a break (Betaflight: failsafe_recovery_delay). A battery
//             landing is not called off: the voltage comes back as soon as the motors are unloaded, and would be gone again
//             with the next climb. While a rescue flies the craft home, a link that is back is not enough: the pilot gets the
//             craft when he MOVES A STICK (Betaflight; PX4's COM_RC_OVERRIDE). The same takes back a BATTERY_LOW RTL / LAND
//             (PX4's user takeover): it is not asked for again until the pack reaches its minimum.
//             Standing on the ground when the guard time ends: no descent, the motors stop.
//   LANDED    disarmed; back to NONE when the switch is off on a live link.
//
// The time limit max_descent_s is a net for a craft that does not come down (or cannot tell): it counts only while LANDING and
// the craft is NOT sinking by its own height estimate - a descent from 100 m is never cut short in the air - and always when
// there is no height estimate. A hold or a rescue flying home is not a landing that makes no progress.
// ---------------------------------------------------------------------------------------------------------------------
enum class FsStage : uint8_t { NONE, GUARD, DESCENT, LANDED };
enum FsCause : uint8_t { FS_LINK = 1, FS_SWITCH = 2, FS_BATTERY = 4, FS_BATTERY_LOW = 8 };
enum FsProcedure : uint8_t { FSP_LAND = 0, FSP_RESCUE = 1, FSP_HOLD = 2, FSP_DROP = 3 };
enum FsAction : uint8_t { FA_NONE = 0, FA_WARN = 1, FA_HOLD = 2, FA_RTL = 3, FA_LAND = 4, FA_DROP = 5 };      // severity order
enum FsBattLow : uint8_t { FBL_WARN = 0, FBL_RTL = 1, FBL_LAND = 2 };
enum FsBattCrit : uint8_t { FBC_LAND = 0, FBC_RTL = 1 };
struct FsConfig { float guard_s = 0.5f, recover_s = 1.0f, max_descent_s = 60.0f; float fallback_throttle_scale = 0.85f;
                  uint8_t procedure = FSP_LAND;          // LINK / SWITCH: what to fly (FsProcedure)
                  float stick_threshold = 0.30f;         // the pilot takes the craft back by moving a stick this far
                  float hold_s = 30.0f;                  // FSP_HOLD: how long before it lands
                  uint8_t batt_low = FBL_WARN, batt_crit = FBC_LAND;
                  bool mission_rc_continue = true;       // the link lost while the mission flies: no failsafe, it flies on (mission.rc_lost)
};
struct FsInput { bool rx_ok = true, batt_critical = false, alt_valid = false, landed = false, armed = false, switch_on = false;
                 bool batt_low = false;        // the pack at its warning level
                 bool airborne = true;         // false: the craft stands on the ground - there is nothing to land, the failsafe disarms
                 float vz = 0.0f;              // climb rate by the height estimate (m/s, up positive); only looked at when alt_valid
                 bool rescue_active = false;   // GPS Rescue is flying the craft (as asked for by FsOutput::rescue)
                 float stick_move = 0.0f; };   // the largest deflection of roll / pitch / yaw right now (0..1)
struct FsOutput {
    FsStage stage = FsStage::NONE;
    uint8_t cause = 0;                // FsCause bits: why (0 when NONE and nothing is warned about)
    uint8_t action = FA_NONE;         // FsAction: what is flown (GUARD: what will be)
    bool warn = false;                // a condition only warned about (BATTERY_LOW with WARN, or one the pilot took back)
    bool neutral_sticks = false;      // roll/pitch/yaw = 0, Angle forced
    bool alt_descent = false;         // feed AltHold failsafe_descent
    bool hold = false;                // HOLD: Altitude Hold (and Position Hold, if possible) with neutral sticks
    bool throttle_fallback = false;   // no altitude estimate: use hover*scale
    bool disarm = false;
    bool rescue = false;              // the action is RTL: the caller starts GPS Rescue; while that flies, alt_descent / throttle_fallback do not count
};

class Failsafe {
public:
    FsConfig cfg;
    static constexpr float kSinking = 0.2f;      // m/s: coming down at least this fast = the descent makes progress

    // The action a set of causes asks for: the most severe of them (PX4: actions are ordered, the worst wins).
    WB_FAST uint8_t action_of(uint8_t causes) const {
        uint8_t a = FA_NONE;
        if (causes & (FS_LINK | FS_SWITCH)) {
            static const uint8_t kProc[4] = {FA_LAND, FA_RTL, FA_HOLD, FA_DROP};
            a = up(a, cfg.procedure < 4 ? kProc[cfg.procedure] : (uint8_t)FA_LAND);
        }
        if (causes & FS_BATTERY) a = up(a, cfg.batt_crit == FBC_RTL ? FA_RTL : FA_LAND);
        if (causes & FS_BATTERY_LOW) a = up(a, cfg.batt_low == FBL_RTL ? FA_RTL : cfg.batt_low == FBL_LAND ? FA_LAND : FA_WARN);
        return a;
    }

    WB_FAST FsOutput update(const FsInput& in, float dt) {
        FsOutput o;
        uint8_t now = 0;
        if (in.armed) {
            if (!in.rx_ok) now |= FS_LINK;
            else if (in.switch_on) now |= FS_SWITCH;                 // (without a link the switch channel is not the pilot's hand)
            if (in.batt_critical) now |= FS_BATTERY;
            else if (in.batt_low) now |= FS_BATTERY_LOW;
        }
        // the pilot took a BATTERY_LOW action back: it stays a warning until the pack reaches its minimum (or recovers)
        if (!(now & FS_BATTERY_LOW)) low_taken_back_ = false;          // (at the minimum the cause is BATTERY: the takeover is over too)
        uint8_t acting = now;
        if (low_taken_back_) acting &= (uint8_t)~FS_BATTERY_LOW;
        const uint8_t want = action_of(acting);
        switch (stage) {
        case FsStage::NONE:
            t = t > dt ? t - dt : 0.0f;                              // the guard time used by earlier dropouts drains
            if (want > FA_WARN) { stage = FsStage::GUARD; cause = acting; action_ = want; }
            break;
        case FsStage::GUARD:
            if (want <= FA_WARN) { stage = FsStage::NONE; cause = 0; action_ = FA_NONE; break; }
            cause = acting; action_ = want; t += dt;
            // BATTERY_LOW alone needs no guard (the battery monitor's delays did that): the action starts at once
            if (t >= cfg.guard_s || !(acting & (FS_LINK | FS_SWITCH | FS_BATTERY))) {
                if (action_ == FA_DROP || !in.airborne) { stage = FsStage::LANDED; o.disarm = true; }   // on the ground: motors off, not a "descent"
                else { stage = FsStage::DESCENT; t = 0; good_t = 0; stuck_t = 0; hold_t = 0; }
            }
            break;
        case FsStage::DESCENT: {
            // each cause ends by itself: the link after recover_s without a break, the switch at once; a battery landing stays
            if (now & FS_LINK) { cause |= FS_LINK; good_t = 0; }
            else {
                if (cause & FS_LINK) { good_t += dt; if (good_t >= cfg.recover_s && (!in.rescue_active || in.stick_move > cfg.stick_threshold)) cause &= (uint8_t)~FS_LINK; }
                if (now & FS_SWITCH) cause |= FS_SWITCH; else cause &= (uint8_t)~FS_SWITCH;        // (only a live link says where the switch is)
            }
            if (now & FS_BATTERY) cause |= FS_BATTERY;               // stays: a pack recovers as soon as the load is gone, and would sag again
            if (acting & FS_BATTERY_LOW) cause |= FS_BATTERY_LOW;
            // the pilot takes a battery-low action back with a stick (on a live link, nothing else asking for an action)
            if ((cause & FS_BATTERY_LOW) && in.rx_ok && in.stick_move > cfg.stick_threshold && !(cause & (FS_LINK | FS_SWITCH | FS_BATTERY))) {
                cause &= (uint8_t)~FS_BATTERY_LOW; low_taken_back_ = true;
            }
            const uint8_t a = action_of(cause);
            if (a <= FA_WARN) { stage = FsStage::NONE; cause = 0; action_ = FA_NONE; t = 0; break; }
            if (a > action_) { action_ = a; hold_t = 0; stuck_t = 0; }            // worse news: the more severe action (never the other way)
            if (action_ == FA_DROP) { stage = FsStage::LANDED; o.disarm = true; break; }
            if (action_ == FA_HOLD) {
                hold_t += dt;
                if (hold_t >= cfg.hold_s || !in.alt_valid) action_ = FA_LAND;   // held long enough (or nothing to hold a height with): land
            }
            const bool landing = action_ == FA_LAND || (action_ == FA_RTL && !in.rescue_active);
            if (!landing) stuck_t = 0;                                         // holding or on its way home: the net is for a landing that makes no progress
            else if (!in.alt_valid || in.vz > -kSinking) stuck_t += dt;
            if (!in.armed || in.landed || stuck_t > cfg.max_descent_s) { stage = FsStage::LANDED; o.disarm = true; }
            break;
        }
        case FsStage::LANDED:
            o.disarm = true;
            if (!in.armed && in.rx_ok && !in.switch_on) { stage = FsStage::NONE; cause = 0; action_ = FA_NONE; t = 0; }
            break;
        }
        o.stage = stage;
        const bool active = stage == FsStage::GUARD || stage == FsStage::DESCENT;
        o.cause = stage == FsStage::NONE ? (uint8_t)(now & FS_BATTERY_LOW) : cause;     // (a battery-low warning shows its cause too)
        o.action = stage == FsStage::NONE ? (uint8_t)((now & FS_BATTERY_LOW) ? FA_WARN : FA_NONE) : action_;
        o.warn = stage == FsStage::NONE && (now & FS_BATTERY_LOW);
        o.neutral_sticks = active && (cause & (FS_LINK | FS_SWITCH)) != 0;
        const bool flying = stage == FsStage::DESCENT;
        o.hold = flying && action_ == FA_HOLD && in.alt_valid;
        o.rescue = flying && action_ == FA_RTL;
        o.alt_descent = flying && (action_ == FA_LAND || action_ == FA_RTL) && in.alt_valid;
        o.throttle_fallback = flying && !in.alt_valid && action_ != FA_DROP;
        return o;
    }
    FsStage get_stage() const { return stage; }
    uint8_t get_cause() const { return cause; }
    uint8_t get_action() const { return action_; }
private:
    static uint8_t up(uint8_t a, uint8_t b) { return a > b ? a : b; }
    FsStage stage = FsStage::NONE; uint8_t cause = 0, action_ = FA_NONE; float t = 0, good_t = 0, stuck_t = 0, hold_t = 0; bool low_taken_back_ = false;
};

// ---------------------------------------------------------------------------------------------------------------------
// Runaway take-off prevention - Betaflight's (fc/core.c, RUNAWAY_TAKEOFF_*), on by default there and here.
// A wrong motor order, a prop on the wrong motor, a motor turning the wrong way or a reversed gyro axis make the rate loop a
// POSITIVE feedback: the PID sum runs to its limit while the craft turns faster and faster. Until a normal flight has been
// seen once since power-up -
//      throttle >= 20 % with a stick moved, or >= 40 % (75 % at most), and every PID sum below 100,
//      for 0.5 s in total (half of that above 75 % throttle)
// - a PID sum of 600 or more on any axis TOGETHER WITH a turn rate above 15 deg/s (roll, pitch) or 50 deg/s (yaw), for 75 ms,
// disarms the craft. The turn-rate condition keeps the bench quiet: without props the PID sum also saturates, but nothing turns.
// ---------------------------------------------------------------------------------------------------------------------
struct RunawayConfig {
    bool enabled = true;
    float pidsum_threshold = 600.0f, gyro_limit_rp_dps = 15.0f, gyro_limit_yaw_dps = 50.0f, activate_delay_s = 0.075f;
    float deactivate_throttle = 0.20f, deactivate_stick = 0.15f, deactivate_pidsum = 100.0f, deactivate_delay_s = 0.5f, high_throttle = 0.75f;
};
class RunawayTakeoff {
public:
    RunawayConfig cfg;
    struct Input {
        bool armed = false;
        bool motors_mixed = false;       // the PID output really reaches the motors (not standing at ground idle)
        float throttle = 0;              // pilot's throttle 0..1
        float stick = 0;                 // the largest of |roll|, |pitch|, |yaw| stick, 0..1
        float pid_sum[3] = {};           // P + I + D + F per axis, PID units (the mixer gets 1/1000 of it)
        float gyro_dps[3] = {};          // filtered rates
    };
    void on_arm() { stable_t_ = 0.0f; trigger_t_ = 0.0f; }
    // true: disarm now
    WB_FAST bool update(const Input& in, float dt) {
        if (!cfg.enabled || !in.armed || flown_ || !in.motors_mixed) { trigger_t_ = 0.0f; return false; }
        const float sr = fabsf(in.pid_sum[0]), sp = fabsf(in.pid_sum[1]), sy = fabsf(in.pid_sum[2]);
        // ---- has it flown normally?
        const float mid = minf(cfg.deactivate_throttle * 2.0f, cfg.high_throttle);
        const bool stable = ((in.throttle >= cfg.deactivate_throttle && in.stick >= cfg.deactivate_stick) || in.throttle >= mid)
                         && sr < cfg.deactivate_pidsum && sp < cfg.deactivate_pidsum && sy < cfg.deactivate_pidsum;
        if (stable) {
            stable_t_ += dt;
            if (stable_t_ > (in.throttle >= cfg.high_throttle ? 0.5f * cfg.deactivate_delay_s : cfg.deactivate_delay_s)) { flown_ = true; trigger_t_ = 0.0f; return false; }
        }
        // ---- is it running away?
        const bool saturated = sr >= cfg.pidsum_threshold || sp >= cfg.pidsum_threshold || sy >= cfg.pidsum_threshold;
        const bool turning = fabsf(in.gyro_dps[0]) > cfg.gyro_limit_rp_dps || fabsf(in.gyro_dps[1]) > cfg.gyro_limit_rp_dps || fabsf(in.gyro_dps[2]) > cfg.gyro_limit_yaw_dps;
        if (saturated && turning) { trigger_t_ += dt; if (trigger_t_ > cfg.activate_delay_s) { trigger_t_ = 0.0f; return true; } }
        else trigger_t_ = 0.0f;
        return false;
    }
    bool flown() const { return flown_; }        // a normal flight was seen since power-up: the check is off
    void reset() { flown_ = false; stable_t_ = trigger_t_ = 0.0f; }
private:
    bool flown_ = false; float stable_t_ = 0.0f, trigger_t_ = 0.0f;
};

// ---------------------------------------------------------------------------------------------------------------------
// A motor that stops (a prop strike, a motor blocked by grass, a desync): disarm, before the other motors flip the craft into
// the ground or a stuck motor burns. Needs the motor's speed: bidirectional DShot or the ESC telemetry wire. Per motor, while
// it is commanded at least min_command and its speed is known:
//   stalled - below min_rpm;
//   dropped - below drop_ratio of the highest speed it had since its command last fell (a motor whose command goes down
//             slows down for that reason: the reference starts again then).
// Either one for delay_s: the craft is disarmed (blocker MOTOR_STALL until the arm switch is cycled). A motor without speed
// telemetry is never judged - a missing reply is a telemetry fault, not a stall. Off by default (motors.stall_disarm).
// ---------------------------------------------------------------------------------------------------------------------
struct StallConfig { bool enabled = false; float min_command = 0.25f, min_rpm = 1500.0f, drop_ratio = 0.35f, delay_s = 0.10f; };
class StallGuard {
public:
    StallConfig cfg;
    struct Input { bool armed = false; int motors = 0; const float* command = nullptr; const float* rpm = nullptr; uint8_t valid_mask = 0; };
    // the motor (1..8) that stopped, 0 = none
    WB_FAST int update(const Input& in, float dt) {
        if (!cfg.enabled || !in.armed || !in.command || !in.rpm) { reset(); return 0; }
        for (int m = 0; m < in.motors && m < kMaxMotors; m++) {
            const float c = in.command[m], r = in.rpm[m];
            if (!((in.valid_mask >> m) & 1u) || c < cfg.min_command) { t_[m] = 0.0f; peak_[m] = 0.0f; peak_cmd_[m] = c; continue; }
            if (c < 0.8f * peak_cmd_[m]) { peak_[m] = r; peak_cmd_[m] = c; }                 // asked to slow down: a new reference
            if (r > peak_[m]) { peak_[m] = r; peak_cmd_[m] = c; }
            const bool stalled = r < cfg.min_rpm, dropped = r < cfg.drop_ratio * peak_[m];
            if (stalled || dropped) { t_[m] += dt; if (t_[m] >= cfg.delay_s) { last_ = m + 1; reset(); return last_; } }
            else t_[m] = 0.0f;
        }
        return 0;
    }
    WB_FAST void reset() { for (int m = 0; m < kMaxMotors; m++) { t_[m] = 0.0f; peak_[m] = 0.0f; peak_cmd_[m] = 0.0f; } }
    int last() const { return last_; }               // the motor that stopped last (1..8), 0 = none since boot
private:
    float t_[kMaxMotors] = {}, peak_[kMaxMotors] = {}, peak_cmd_[kMaxMotors] = {}; int last_ = 0;
};

// ---------------------------------------------------------------------------------------------------------------------
// "It stands on the ground": the pilot's throttle is down, the craft does not turn, its accelerometer reads 1 g (it neither
// falls nor is pushed) and - when a height is known - it does not sink, all of it for a full second. ArduCopter's land detector
// asks the same questions (land_detector.cpp). A craft that falls level at its terminal speed reads 1 g as well: without a
// height that cannot be told from standing, which is why the caller offers "airmode until disarm" for pilots who do that.
// ---------------------------------------------------------------------------------------------------------------------
class LandDetector {
public:
    struct Config { float still_s = 1.0f, max_gyro_dps = 15.0f, max_sink_mps = 0.5f; } cfg;
    struct Input {
        bool throttle_down = false;      // the pilot's throttle is low (or the autopilot asks for a descent)
        bool accel_1g = false;           // the low-passed accelerometer reads what it reads at rest (Ahrs::acc_weight() > 0)
        float gyro_dps = 0;              // the largest of the three filtered rates, absolute
        bool alt_valid = false; float vz = 0;
    };
    bool update(const Input& in, float dt) {
        const bool still = in.throttle_down && in.accel_1g && in.gyro_dps < cfg.max_gyro_dps && (!in.alt_valid || fabsf(in.vz) < cfg.max_sink_mps);
        t_ = still ? t_ + dt : 0.0f;
        return t_ >= cfg.still_s;
    }
    void reset() { t_ = 0.0f; }
    float still_for() const { return t_; }
private:
    float t_ = 0.0f;
};

// ---------------------------------------------------------------------------------------------------------------------
// Arming state machine. Arms only on the RISING EDGE of the arm switch with no blocker active; disarms the moment the switch goes off.
// Arming needs an EDGE of the arm switch: after a failsafe / forced disarm the switch must be cycled before it can arm again.
// The switch is only believed while the receiver delivers frames (link_ok). What the channel reads before the first frame (a
// default) or while the link is down (held, or the receiver's failsafe value) is not the pilot's hand:
//   - it never arms: after power-up, and after every loss of the link while disarmed, the switch must first be SEEN OFF on a live
//     link. A craft powered up - or coming back into range - with the arm switch on stays disarmed (Betaflight: NOT_DISARMED);
//   - it never disarms: an armed craft that loses its link is the failsafe's business.
// ---------------------------------------------------------------------------------------------------------------------
class Arming {
public:
    // keep_reason: the forced-disarm reason (sticky) is not cleared by the switch being off this cycle (stick arming: the
    // virtual switch is off whenever the sticks rest, so the reason stays until the pilot starts the next arming gesture).
    WB_FAST bool update(bool switch_on, bool link_ok, ArmInput in, bool keep_reason = false) {
        if (!link_ok) {
            if (!armed_) seen_off_ = false;
            switch_on = armed_;                                      // not believed: as if it had not moved
        } else if (!switch_on) seen_off_ = true;
        in.arm_switch_was_on_at_boot = !seen_off_ && link_ok && switch_on;
        blockers_ = arm_blockers(in);
        if (armed_) {
            if (remote_) { if (link_ok && switch_on && !prev_switch_) remote_ = false; }      // (armed by a ground station: the switch takes it with its next edge on a live link)
            else if (!switch_on) { armed_ = false; disarm_reason_ = 1; }
        } else if (switch_on && !prev_switch_) {                    // the edge of a switch that is believed
            if (blockers_ == 0) armed_ = true; else refused_++;      // (a switch not yet seen off on a live link is a blocker: AB_ARM_SWITCH_ON)
        }
        // A switch that is ON while disarmed can no longer arm: its edge was refused, or a forced disarm asks for a cycle.
        // Say so, otherwise the pilot sees "nothing blocks arming" with the switch on and motors that stay off.
        if (!armed_ && switch_on) blockers_ |= AB_ARM_SWITCH_ON;
        // Why the craft was disarmed against the pilot's will stays on show until he has switched off (Betaflight keeps
        // RUNAWAY_TAKEOFF as an arming-disabled flag the same way).
        if (link_ok && !switch_on && !keep_reason) sticky_ = 0;
        if (!armed_) blockers_ |= sticky_;
        prev_switch_ = switch_on;
        return armed_;
    }
    // After this the switch is still on, so there is no edge until the pilot has moved it off and on again.
    // why: a blocker bit to show until then (0 = none).
    WB_FAST void force_disarm(uint32_t why = 0) { if (armed_) { armed_ = false; disarm_reason_ = 2; } sticky_ |= why; remote_ = false; }
    // The mission (nav/mission.h) puts the craft down with the motors OFF at a delivery stop and arms it again after the wait:
    // no switch edge for either (the pilot's ARM switch stays on all the while) and no sticky reason. Arming again ignores what
    // a take-off without a pilot cannot satisfy - the throttle stick, the switch being on, the link (the mission carries on
    // without it); every other blocker still holds it off. switch_on: the ARM switch as read (not believed while the link is lost).
    WB_FAST void mission_disarm() { if (armed_) { armed_ = false; disarm_reason_ = 3; } prev_switch_ = true; remote_ = false; }
    WB_FAST bool mission_arm(bool switch_on, bool link_ok, const ArmInput& in) {
        if (armed_) return true;
        if (link_ok && !switch_on) return false;
        if (((arm_blockers(in) & ~(AB_THROTTLE_HIGH | AB_ARM_SWITCH_ON | AB_RX_LOST)) | sticky_) != 0) return false;
        armed_ = true; prev_switch_ = true;
        return true;
    }
    // A ground station (MAVLink COMPONENT_ARM_DISARM, telemetry.remote_arm) arms without the switch: every blocker of this cycle holds
    // it off (a switch left on included - Betaflight's tryArm path). The switch, off all the while, does not disarm it; the pilot
    // takes it over with the switch's next edge (on), and then off disarms as usual. Call after update().
    WB_FAST bool remote_arm() {
        if (armed_) return true;
        if ((blockers_ | sticky_) != 0) { refused_++; return false; }
        armed_ = true; remote_ = true; return true;
    }
    WB_FAST void remote_disarm() { if (armed_) { armed_ = false; disarm_reason_ = 4; } remote_ = false; }
    bool armed() const { return armed_; }
    uint32_t blockers() const { return blockers_; }
    uint32_t refused() const { return refused_; }          // arm attempts rejected by a blocker
    uint8_t last_disarm_reason() const { return disarm_reason_; }   // 1 switch, 2 forced (failsafe / crash), 3 the mission (a delivery stop), 4 a ground station
private:
    bool armed_ = false, prev_switch_ = false, seen_off_ = false, remote_ = false; uint32_t blockers_ = 0, refused_ = 0, sticky_ = 0; uint8_t disarm_reason_ = 0;
};
// ---------------------------------------------------------------------------------------------------------------------
// Arming with the sticks (general.stick_arm): a virtual arm switch for Arming above.
//   the gesture  throttle down, pitch down (stick pulled back), yaw and roll to OPPOSITE sides: yaw left + roll right, or yaw
//                right + roll left (on a mode 2 transmitter both sticks down and out, or down and in - DJI's "combination stick")
//   arm          the gesture held arm_s while disarmed: the switch turns on (Arming arms on that edge, or refuses it)
//   disarm       the gesture held disarm_s while armed AND on the ground (never in the air: the same sticks can be flown)
// The gesture must be released between two of them (and after power-up). A switch that turned on but did not arm (refused,
// forced off later) goes off when the sticks are released, so the next gesture is a new edge. enable = false (an ARM switch
// is assigned and off) holds the switch off: Arming disarms at once. Without a link nothing changes (the failsafe's business).
// ---------------------------------------------------------------------------------------------------------------------
class StickArm {
public:
    struct Config { float arm_s = 1.0f, disarm_s = 1.0f, threshold = 0.8f; } cfg;
    static bool gesture(float roll, float pitch, float yaw, bool throttle_low, float k) {
        return throttle_low && pitch <= -k && ((yaw <= -k && roll >= k) || (yaw >= k && roll <= -k));
    }
    WB_FAST bool update(bool gesture_now, bool link_ok, bool enable, bool armed, bool on_ground, float dt) {
        if (!link_ok) { t_ = 0.0f; return sw_; }
        if (!enable) { sw_ = false; t_ = 0.0f; held_ = gesture_now; return false; }
        if (!gesture_now) { t_ = 0.0f; held_ = false; if (!armed) sw_ = false; return sw_; }
        if (held_) return sw_;
        if (armed && !on_ground) { t_ = 0.0f; return sw_; }          // in the air these sticks are flying, never a disarm
        t_ += dt;
        if (!armed && t_ >= cfg.arm_s) { sw_ = true; held_ = true; t_ = 0.0f; }
        else if (armed && t_ >= cfg.disarm_s) { sw_ = false; held_ = true; t_ = 0.0f; }
        return sw_;
    }
    bool holding() const { return t_ > 0.0f; }          // the gesture is being held towards an arm / disarm
    float held_s() const { return t_; }
private:
    bool sw_ = false, held_ = true; float t_ = 0.0f;      // held_: released first (a gesture held at power-up does nothing)
};
} // namespace fc
