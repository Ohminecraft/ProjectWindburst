#pragma once
#include "../config/fast_code.h"
#include "types.h"

// ============================================================================
// Acro trainer (Betaflight flight/pid.c applyAcroTrainer, 4.5): in Acro, with the ACRO_TRAINER switch on, roll and pitch are
// held below an angle - for learning Acro without flipping over. The stick still gives the rate; only when the craft would pass
// the limit is the setpoint replaced:
//   past the limit (and the stick still pushes that way)  setpoint = (limit - angle) x gain       held until the stick reverses
//   about to pass it (angle + gyro x lookahead)            setpoint = (limit - projected) x gain  the lookahead grows with the
//                                                                                                   rate: 0 at 0, full at 500 deg/s
// Each time it takes over, that axis' I-term is cleared (reset_i). Angles, rates and setpoints: user convention (types.h).
// ============================================================================
namespace fc {

struct AcroTrainerConfig {
    float angle_limit_deg = 20.0f;      // acro_trainer_angle_limit
    float lookahead_s = 0.05f;          // acro_trainer_lookahead_ms / 1000
    float gain = 7.5f;                  // acro_trainer_gain / 10
};

class AcroTrainer {
public:
    AcroTrainerConfig cfg;
    static constexpr float kLookaheadRateLimit = 500.0f, kSetpointLimit = 1000.0f;

    // Switched on (Betaflight pidAcroTrainerInit): nothing is held yet.
    void reset() { state_[0] = state_[1] = 0; }
    int state(int axis) const { return state_[axis]; }       // -1 / +1: held at the negative / positive limit, 0: free

    // One axis (ROLL or PITCH): the setpoint to fly. reset_i: clear that axis' I-term.
    WB_FAST float apply(int axis, float setpoint, float angle_deg, float gyro_dps, bool& reset_i) {
        reset_i = false;
        const float lim = cfg.angle_limit_deg;
        const int sp_sign = sign(setpoint), angle_sign = sign(angle_deg);
        int& st = state_[axis];
        if (st != 0 && st != sp_sign) st = 0;                                         // the stick reversed: let go
        if (fabsf(angle_deg) > lim && st == 0 && angle_sign == sp_sign) { st = angle_sign; reset_i = true; }
        if (st != 0) return clampf((lim * angle_sign - angle_deg) * cfg.gain, -kSetpointLimit, kSetpointLimit);
        const float look = clampf(fabsf(gyro_dps) / kLookaheadRateLimit, 0.0f, 1.0f) * cfg.lookahead_s;
        const float projected = gyro_dps * look + angle_deg;
        const int p_sign = sign(projected);
        if (fabsf(projected) > lim && p_sign == sp_sign) { reset_i = true; return (lim * p_sign - projected) * cfg.gain; }
        return setpoint;
    }

private:
    static int sign(float x) { return x > 0.0f ? 1 : -1; }      // (Betaflight acroTrainerSign: 0 counts as negative)
    int state_[2] = {0, 0};
};

} // namespace fc
