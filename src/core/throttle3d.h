/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>
#include <math.h>
#include "rc.h"

// ============================================================================
// 3D (motors.mode3d, Betaflight's feature 3D): reversible ESCs, the throttle stick split at its centre.
//
// Betaflight rc.c (the 3D throttle), rc_controls.c calculateThrottleStatus, mixer.c calculateThrottleAndCurrentMotorEndpoints:
//   not switched  centre +- 3d_deadband_throttle = stopped ("throttle low": arming allowed); above = forward, below = reversed,
//                 each half scaled to 0..1. Back into the deadband the motors keep the direction they had, at zero
//                 (INVERTED_TO_DEADBAND / NORMAL_TO_DEADBAND). The 3D switch ON here turns 3D off: the whole stick forward.
//   switched      (3d_switched_mode) the whole stick as usual (below min_check = low), the 3D switch reverses the motors.
//   disarmed      the next arming starts forward (Betaflight keeps mid_rc as the "previous" throttle while disarmed).
//   a reversal    the I-terms are held at zero for 250 ms (Betaflight pidResetIterm while reversalTimeUs is fresh).
// The throttle curve is not applied to the 3D magnitude (linear). Arming needs the stick inside the deadband (Betaflight lets a
// 3D craft arm at any throttle; here the motors must not start at speed).
// Portable: unit-tested in test/test_mode3d.cpp.
// ============================================================================
namespace fc {

struct Mode3dConfig {
    bool enabled = false, switched = false; uint16_t deadband_us = 50;     // receiver.deadband3d_throttle
    uint16_t mid_us = 1500, min_check_us = kRcMinCheck;                  // the centre (Betaflight midrc), "low" in switched mode (min_check)
};
struct Throttle3d { float thr = 0; bool reversed = false; bool low = true; };              // magnitude 0..1, direction, "throttle low"

class Mode3d {
public:
    static constexpr float kItermHoldS = 0.25f;
    Mode3dConfig cfg;

    // thr_us: the throttle channel; box: the 3D switch; dt: the control period (s)
    Throttle3d update(uint16_t thr_us, bool box, bool armed, float dt) {
        Throttle3d o;
        const float us = thr_us < 1000 ? 1000.0f : thr_us > 2000 ? 2000.0f : (float)thr_us;
        if (!armed) prev_rev_ = false;
        if (cfg.switched || box) {                                       // the stick as usual; switched: the switch reverses
            const float mc = (float)cfg.min_check_us;
            o.low = us < mc;
            o.thr = o.low ? 0.0f : (us - mc) / (2000.0f - mc);
            o.reversed = cfg.switched && box;
        } else {
            const float d = (float)cfg.deadband_us, m = (float)cfg.mid_us, lo = m - d, hi = m + d;
            o.low = fabsf(us - m) < d;
            if (us <= lo) { o.reversed = true; o.thr = (lo - us) / (lo - 1000.0f); }
            else if (us >= hi) { o.reversed = false; o.thr = (us - hi) / (2000.0f - hi); }
            else { o.reversed = prev_rev_; o.thr = 0.0f; }
        }
        if (o.reversed != prev_rev_) since_ = 0.0f; else if (since_ < 1e6f) since_ += dt;
        prev_rev_ = o.reversed;
        return o;
    }
    bool iterm_hold() const { return since_ < kItermHoldS; }       // a reversal less than 250 ms ago
    bool reversed() const { return prev_rev_; }

private:
    bool prev_rev_ = false; float since_ = 1e9f;
};

} // namespace fc
