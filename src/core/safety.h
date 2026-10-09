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
    AB_PIN_CONFLICT = 1u << 13, AB_LOOP_RATE = 1u << 14, AB_RUNAWAY_TAKEOFF = 1u << 15, AB_RESCUE_NO_FIX = 1u << 16, AB_RESCUE_NO_COMPASS = 1u << 17,
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
};
inline uint32_t arm_blockers(const ArmInput& a) {
    uint32_t b = 0;
    if (!a.imu_ok) b |= AB_NO_IMU;
    if (!a.gyro_calibrated) b |= AB_GYRO_NOT_CAL;
    if (!a.rx_ok) b |= AB_RX_LOST;
    if (!a.throttle_low) b |= AB_THROTTLE_HIGH;
    if (a.tilt_deg > a.max_arm_angle_deg) b |= AB_ANGLE_TOO_HIGH;
    if (a.failsafe_active) b |= AB_FAILSAFE;
    if (a.althold_switch && !a.baro_ok) b |= AB_ALTHOLD_NO_BARO;       // spec: Altitude Hold requires baro, warn immediately
    if (a.poshold_switch && !a.pos_source) b |= AB_POSHOLD_NO_GPS;
    if (a.rescue_configured && !a.gps_fix && !a.rescue_allow_no_fix) b |= AB_RESCUE_NO_FIX;
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
// Failsafe: what the craft does when the pilot can no longer fly it.
//
//   cause     LINK     no receiver frames
//             SWITCH   the pilot's failsafe switch (a test of the above on a live link)
//             BATTERY  the pack is at its minimum
//   GUARD     the first guard_s. LINK / SWITCH: the sticks read neutral and the craft is levelled, the throttle stays where it was.
//             BATTERY: nothing yet. The cause goes away -> back to normal AT ONCE (Betaflight's stage 1).
//             Short dropouts add up: the guard time already used drains only as fast as it was filled, so a link that is gone
//             half of the time still ends in a descent.
//   DESCENT   the craft comes down by itself (with a height: at a controlled rate; without: at a fixed fraction of the hover
//             throttle). LINK / SWITCH: levelled, sticks neutral. BATTERY: the pilot keeps roll / pitch / yaw - he can still
//             choose where it comes down (ArduCopter's LAND).
//             The cause goes away -> control returns: at once for the switch, after recover_s of unbroken link for a lost
//             link (Betaflight: failsafe_recovery_delay). A battery landing is not called off: the voltage comes back as
//             soon as the motors are unloaded, and would be gone again with the next climb.
//             Standing on the ground when the guard time ends: no descent, the motors stop.
//   LANDED    disarmed; back to NONE when the switch is off on a live link.
//
//   procedure RESCUE (LINK / SWITCH only): instead of coming down where it is, the craft is flown home and landed there by
//             GPS Rescue (rescue.h). The failsafe only asks for it (FsOutput::rescue); when the rescue cannot fly - no fix, no
//             home, no compass - the caller says so (FsInput::rescue_active stays false) and the descent on the spot happens as
//             described above. While the rescue flies, the time limit does not count (flying home takes minutes, and starts
//             with a climb), and a pilot whose link is back for recover_s gets the craft when he MOVES A STICK (Betaflight's
//             rule: nobody is surprised by a craft that suddenly follows sticks left in a corner). A critical battery lands on
//             the spot whatever the procedure: there may be no energy left to fly home.
//
// The time limit max_descent_s is a net for a craft that does not come down (or cannot tell): it counts only while the craft
// is NOT sinking by its own height estimate - a descent from 100 m is never cut short in the air - and always when there is
// no height estimate.
// ---------------------------------------------------------------------------------------------------------------------
enum class FsStage : uint8_t { NONE, GUARD, DESCENT, LANDED };
enum FsCause : uint8_t { FS_LINK = 1, FS_SWITCH = 2, FS_BATTERY = 4 };
enum FsProcedure : uint8_t { FSP_LAND = 0, FSP_RESCUE = 1 };
struct FsConfig { float guard_s = 0.5f, recover_s = 1.0f, max_descent_s = 60.0f; float fallback_throttle_scale = 0.85f;
                  uint8_t procedure = FSP_LAND;          // FSP_RESCUE: fly home and land there; falls back to the landing on the spot when the rescue cannot fly
                  float stick_threshold = 0.30f; };      // after a rescue from a lost link the pilot gets the craft back when a stick is moved this far
struct FsInput { bool rx_ok = true, batt_critical = false, alt_valid = false, landed = false, armed = false, switch_on = false;
                 bool airborne = true;         // false: the craft stands on the ground - there is nothing to land, the failsafe disarms
                 float vz = 0.0f;              // climb rate by the height estimate (m/s, up positive); only looked at when alt_valid
                 bool rescue_active = false;   // GPS Rescue is flying the craft (as asked for by FsOutput::rescue)
                 float stick_move = 0.0f; };   // the largest deflection of roll / pitch / yaw right now (0..1)
struct FsOutput {
    FsStage stage = FsStage::NONE;
    uint8_t cause = 0;                // FsCause bits: why (0 when NONE)
    bool neutral_sticks = false;      // roll/pitch/yaw = 0, Angle forced
    bool alt_descent = false;         // feed AltHold failsafe_descent
    bool throttle_fallback = false;   // no altitude estimate: use hover*scale
    bool disarm = false;
    bool rescue = false;              // DESCENT with the procedure RESCUE: the caller starts GPS Rescue; while that flies, alt_descent / throttle_fallback do not count
};

class Failsafe {
public:
    FsConfig cfg;
    static constexpr float kSinking = 0.2f;      // m/s: coming down at least this fast = the descent makes progress

    WB_FAST FsOutput update(const FsInput& in, float dt) {
        FsOutput o;
        uint8_t now = 0;
        if (in.armed) {
            if (!in.rx_ok) now |= FS_LINK;
            else if (in.switch_on) now |= FS_SWITCH;                 // (without a link the switch channel is not the pilot's hand)
            if (in.batt_critical) now |= FS_BATTERY;
        }
        switch (stage) {
        case FsStage::NONE:
            t = t > dt ? t - dt : 0.0f;                              // the guard time used by earlier dropouts drains
            if (now) { stage = FsStage::GUARD; cause = now; }
            break;
        case FsStage::GUARD:
            if (!now) { stage = FsStage::NONE; cause = 0; break; }
            cause = now; t += dt;
            if (t >= cfg.guard_s) {
                if (in.airborne) { stage = FsStage::DESCENT; t = 0; good_t = 0; stuck_t = 0; }
                else { stage = FsStage::LANDED; o.disarm = true; }   // on the ground: motors off, not a "descent" at 85 % of the hover throttle
            }
            break;
        case FsStage::DESCENT:
            // each cause ends by itself: the link after recover_s without a break, the switch and the battery at once
            if (now & FS_LINK) { cause |= FS_LINK; good_t = 0; }
            else {
                // (while the rescue flies the craft home, a link that is back is not enough: the pilot shows he is there)
                if (cause & FS_LINK) { good_t += dt; if (good_t >= cfg.recover_s && (!in.rescue_active || in.stick_move > cfg.stick_threshold)) cause &= (uint8_t)~FS_LINK; }
                if (now & FS_SWITCH) cause |= FS_SWITCH; else cause &= (uint8_t)~FS_SWITCH;        // (only a live link says where the switch is)
            }
            if (now & FS_BATTERY) cause |= FS_BATTERY;               // stays: a pack recovers as soon as the load is gone, and would sag again
            if (!cause) { stage = FsStage::NONE; t = 0; break; }
            if (in.rescue_active) stuck_t = 0;                       // on its way home: the net is for a landing that makes no progress
            else if (!in.alt_valid || in.vz > -kSinking) stuck_t += dt;
            if (!in.armed || in.landed || stuck_t > cfg.max_descent_s) { stage = FsStage::LANDED; o.disarm = true; }
            break;
        case FsStage::LANDED:
            o.disarm = true;
            if (!in.armed && in.rx_ok && !in.switch_on) { stage = FsStage::NONE; cause = 0; t = 0; }
            break;
        }
        o.stage = stage;
        o.cause = cause;                                             // (0 whenever the stage is NONE: every way there clears it)
        const bool active = stage == FsStage::GUARD || stage == FsStage::DESCENT;
        o.neutral_sticks = active && (cause & (FS_LINK | FS_SWITCH)) != 0;
        o.alt_descent = stage == FsStage::DESCENT && in.alt_valid;
        o.throttle_fallback = stage == FsStage::DESCENT && !in.alt_valid;
        o.rescue = stage == FsStage::DESCENT && cfg.procedure == FSP_RESCUE && (cause & (FS_LINK | FS_SWITCH)) != 0 && !(cause & FS_BATTERY);
        return o;
    }
    FsStage get_stage() const { return stage; }
    uint8_t get_cause() const { return cause; }
private:
    FsStage stage = FsStage::NONE; uint8_t cause = 0; float t = 0, good_t = 0, stuck_t = 0;
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
        const float mid = fminf(cfg.deactivate_throttle * 2.0f, cfg.high_throttle);
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
    WB_FAST bool update(bool switch_on, bool link_ok, ArmInput in) {
        if (!link_ok) {
            if (!armed_) seen_off_ = false;
            switch_on = armed_;                                      // not believed: as if it had not moved
        } else if (!switch_on) seen_off_ = true;
        in.arm_switch_was_on_at_boot = !seen_off_ && link_ok && switch_on;
        blockers_ = arm_blockers(in);
        if (armed_) {
            if (!switch_on) { armed_ = false; disarm_reason_ = 1; }
        } else if (switch_on && !prev_switch_) {                    // the edge of a switch that is believed
            if (blockers_ == 0) armed_ = true; else refused_++;      // (a switch not yet seen off on a live link is a blocker: AB_ARM_SWITCH_ON)
        }
        // A switch that is ON while disarmed can no longer arm: its edge was refused, or a forced disarm asks for a cycle.
        // Say so, otherwise the pilot sees "nothing blocks arming" with the switch on and motors that stay off.
        if (!armed_ && switch_on) blockers_ |= AB_ARM_SWITCH_ON;
        // Why the craft was disarmed against the pilot's will stays on show until he has switched off (Betaflight keeps
        // RUNAWAY_TAKEOFF as an arming-disabled flag the same way).
        if (link_ok && !switch_on) sticky_ = 0;
        if (!armed_) blockers_ |= sticky_;
        prev_switch_ = switch_on;
        return armed_;
    }
    // After this the switch is still on, so there is no edge until the pilot has moved it off and on again.
    // why: a blocker bit to show until then (0 = none).
    WB_FAST void force_disarm(uint32_t why = 0) { if (armed_) { armed_ = false; disarm_reason_ = 2; } sticky_ |= why; }
    bool armed() const { return armed_; }
    uint32_t blockers() const { return blockers_; }
    uint32_t refused() const { return refused_; }          // arm attempts rejected by a blocker
    uint8_t last_disarm_reason() const { return disarm_reason_; }   // 1 switch, 2 forced (failsafe / crash)
private:
    bool armed_ = false, prev_switch_ = false, seen_off_ = false; uint32_t blockers_ = 0, refused_ = 0, sticky_ = 0; uint8_t disarm_reason_ = 0;
};
} // namespace fc
