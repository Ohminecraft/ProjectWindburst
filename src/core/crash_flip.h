#pragma once
#include "../config/fast_code.h"
#include "types.h"
#include "mixer.h"

// ============================================================================
// Crash flip - Betaflight's "turtle mode" (flight/mixer.c applyCrashFlipModeToMotors, fc/core.c tryArm / updateArmingStatus).
// A craft lying upside down after a crash is armed with the CRASHFLIP switch on: the motors turn the other way (DShot 21, the
// motor layer), so their thrust lifts the side they are on, and the sticks pick which motors run:
//   pitch forward  the front motors      roll right  the right motors      yaw  one diagonal pair (a motor stuck in grass)
//   the larger of roll / pitch wins unless the stick is within 30 deg of a diagonal (then the one motor of that corner);
//   yaw wins only when it is the largest of the three.
//   power = how far the stick is pushed (roll and pitch together), nothing within the 15 % dead band; a motor whose command is
//   below 2 % of the range STANDS (no idle: a stopped prop does not dig in).
// crashflip_motor_percent: the motors on the other side run at this share of the power (0 = they stand).
// crashflip_rate (deg/s; 0 = off): the power fades out as the craft turns faster than half of it (none at the rate itself), and
// as its attitude moves away from where it was at the start - none once it has turned past its side (cos of the tilt changed by 1):
// it has flipped, the motors stop by themselves.
// The FlightCore decides when (the switch at the arming; its end) - here only the motor commands.
// ============================================================================
namespace fc {

struct CrashFlipConfig {
    bool available = false;            // the motors can be turned round by command (DShot): without it the switch refuses the arming
    float rate_limit_dps = 0.0f;       // Betaflight crashflip_rate x 10; 0 = no fading
    float motor_pct = 0.0f;            // Betaflight crashflip_motor_percent
    bool auto_rearm = false;           // Betaflight crashflip_auto_rearm: the switch off flies on (instead of a disarm)
};

class CrashFlip {
public:
    CrashFlipConfig cfg;
    static constexpr float kStickDeadband = 0.15f, kMotorDeadband = 0.02f;
    void start() { tilt_set_ = false; }               // a new crash flip: the attitude of its first cycle is the reference
    // Sticks -1..1 (roll right +, pitch forward +, yaw right +); cos_tilt: +1 upright, -1 upside down; rates in deg/s.
    // out[] = absolute 0..1: 0 (stand) or inside [idle, limit].
    WB_FAST void update(const Mixer& mix, float roll, float pitch, float yaw, float cos_tilt, float rate_roll_dps, float rate_pitch_dps,
                        const MotorLimits& lim, float out[kMaxMotors]) {
        const float ap = fabsf(pitch), ar = fabsf(roll), ay = fabsf(yaw);
        // Betaflight's signs: its yaw deflection is positive to the LEFT; its mixer columns are this firmware's rows
        float sp = pitch < 0 ? 1.0f : -1.0f, sr = roll < 0 ? 1.0f : -1.0f;
        float sy = (yaw > 0 ? 1.0f : -1.0f) * (mix.cfg.props_out ? 1.0f : -1.0f);
        float len = sqrtf(ap * ap + ar * ar);
        if (ay > maxf(ap, ar)) { len = ay; sr = sp = 0.0f; }          // yaw the largest: yaw alone
        else sy = 0.0f;
        const float cos_phi = len > 0.0f ? (ap + ar) / (1.41421356f * len) : 0.0f;
        if (cos_phi < 0.86602540f) { if (ar > ap) sp = 0.0f; else sr = 0.0f; }      // more than 30 deg off a diagonal: roll or pitch alone
        float power = len > kStickDeadband ? len : 0.0f;
        if (cfg.rate_limit_dps > 0.0f) {
            if (!tilt_set_) { tilt0_ = cos_tilt; tilt_set_ = true; }
            const float need = maxf(1.0f - fabsf(cos_tilt - tilt0_), 0.0f);           // 1 at the start .. 0 past the side
            const float att = need > 0.5f ? 1.0f : need / 0.5f;
            const float change = minf(maxf(fabsf(rate_roll_dps), fabsf(rate_pitch_dps)) / cfg.rate_limit_dps, 1.0f);
            const float rate = change < 0.5f ? 1.0f : (1.0f - change) / 0.5f;
            power *= att * rate;
        }
        const int n = mix.motor_count();
        for (int i = 0; i < n; i++) {
            const MixRow& r = mix.row(i);
            float m = sp * r.pitch + sr * r.roll + sy * r.yaw;
            if (m < 0.0f) m = cfg.motor_pct > 0.0f ? -m * cfg.motor_pct * 0.01f : 0.0f;
            m = minf(1.0f, power * m);
            out[i] = m < kMotorDeadband ? 0.0f : lim.idle + m * (lim.limit - lim.idle);
        }
        for (int i = n; i < kMaxMotors; i++) out[i] = 0.0f;
    }
private:
    bool tilt_set_ = false; float tilt0_ = 1.0f;
};
} // namespace fc
