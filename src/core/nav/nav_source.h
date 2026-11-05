#pragma once
#include <stdint.h>

// ============================================================================
// Which horizontal position the navigation flies on - the usual estimator (PosEstimator) or the EKF - and ONE reset counter
// for the controllers that hold a position target (PosHold, Loiter: on_estimator_reset).
//
// The position the controllers see jumps when
//   * the source changes (the EKF starts or stops aiding): the two estimates are never exactly the same - the jump is the
//     difference between them at that moment (ArduPilot: a lane switch is reported as a reset, NavEKF3::getLastPosNorthEastReset
//     returns the difference between the old and the new lane);
//   * the source in use resets itself (its own reset counter moves): the jump is the step it reports.
// Both are reported the same way: count() changes, step_n() / step_e() say by how much. A controller that moves its target by
// the step keeps holding the same place on the ground instead of lurching to where the old target now lies.
// ============================================================================
namespace fc {
namespace nav {

class NavSource {
public:
    struct Src { float pos_n = 0, pos_e = 0; uint8_t reset_count = 0; float reset_n = 0, reset_e = 0; };
    // Once per navigation step, with both estimates (the one not in use as well) and which of them is flown on.
    void update(bool use_ekf, const Src& est, const Src& ekf) {
        if (!init_) { init_ = true; use_ekf_ = use_ekf; est_seen_ = est.reset_count; ekf_seen_ = ekf.reset_count; return; }
        const Src& now = use_ekf ? ekf : est;
        if (use_ekf != use_ekf_) {
            const Src& was = use_ekf_ ? ekf : est;
            report(now.pos_n - was.pos_n, now.pos_e - was.pos_e);
        } else if (now.reset_count != (use_ekf ? ekf_seen_ : est_seen_)) {
            report(now.reset_n, now.reset_e);
        }
        use_ekf_ = use_ekf; est_seen_ = est.reset_count; ekf_seen_ = ekf.reset_count;
    }
    uint8_t count() const { return count_; }
    float step_n() const { return step_[0]; }
    float step_e() const { return step_[1]; }
    bool ekf() const { return use_ekf_; }
private:
    void report(float dn, float de) {
        if (!(dn == dn) || !(de == de)) dn = de = 0.0f;       // (a NaN step would poison the target)
        step_[0] = dn; step_[1] = de; count_++;
    }
    bool init_ = false, use_ekf_ = false;
    uint8_t est_seen_ = 0, ekf_seen_ = 0, count_ = 0;
    float step_[2] = {0, 0};
};

} // namespace nav
} // namespace fc
