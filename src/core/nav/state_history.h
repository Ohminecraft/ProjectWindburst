/*
 * This file is part of Windburst Project
 */

#pragma once
#include "../../config/fast_code.h"
#include "../types.h"

// ============================================================================
// What a filter's states were a short while ago, so that a measurement that arrives late can be compared with the state at the
// time it was TAKEN (GPS solutions are 100 - 200 ms old when they arrive, a barometer some tens of ms, optical flow its own
// window). Comparing a late measurement with the present state makes the filter answer a climb or a stop with a lag error that
// looks like a real one; comparing it with the past state and applying the correction to the present is what PX4's estimator
// (ECL / EKF2, "delayed fusion horizon") and INAV (`posEstimator` history) do. Here in its simplest form: the correction of the
// delayed state is applied to the present state as it is, and to everything in the history as well, so the next late
// measurement sees a history that agrees with it.
//
// Fixed size, no allocation: N steps of the filter (128 at 500 Hz = 256 ms; at 200 Hz 640 ms). A delay longer than what the
// buffer holds is answered with the oldest entry. Pure, unit-tested in test_nav_est.
// ============================================================================
namespace fc {
namespace nav {

template <int K, int N = 128>
class StateHistory {
public:
    void reset() { n_ = 0; head_ = 0; }
    // the state after one step of dt seconds
    WB_FAST void push(const float (&x)[K], float dt) {
        head_ = (head_ + 1) % N;
        for (int k = 0; k < K; k++) x_[head_][k] = x[k];
        dt_[head_] = dt > 0.0f ? dt : 0.0f;
        if (n_ < N) n_++;
    }
    // The state `delay_s` ago (exactly: the step whose end is closest at or before that moment). false: nothing stored yet.
    WB_FAST bool at(float delay_s, float (&out)[K]) const {
        if (n_ == 0) return false;
        int i = head_; float t = 0.0f;
        for (int c = 1; c < n_ && t + 0.5f * dt_[i] < delay_s; c++) { t += dt_[i]; i = (i + N - 1) % N; }
        for (int k = 0; k < K; k++) out[k] = x_[i][k];
        return true;
    }
    // a correction made to the present state: the stored past moves with it
    WB_FAST void shift(const float (&dx)[K]) {
        for (int c = 0, i = head_; c < n_; c++, i = (i + N - 1) % N) for (int k = 0; k < K; k++) x_[i][k] += dx[k];
    }
    int size() const { return n_; }
private:
    float x_[N][K] = {}, dt_[N] = {};
    int n_ = 0, head_ = 0;
};

}  // namespace nav
}  // namespace fc
