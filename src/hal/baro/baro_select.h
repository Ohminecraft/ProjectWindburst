/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>

// ============================================================================
// Two barometers: which one the altitude comes from (ArduPilot AP_Baro: BARO_PRIMARY, and another healthy one while the primary
// is not). Portable (test_baro).
//   - the preferred one (sensors.baro_primary) flies while it is valid;
//   - when the one in use stops being valid and the other is valid, the other takes over at once;
//   - back to the preferred one only after it has been valid for kBackMs without a break (no back and forth on a flaky sensor);
//     disarmed at once (on the ground nothing hangs on the altitude; at boot the faster chip is ready first);
//   - each switch keeps the altitude continuous: the new sensor gets an offset so that its next value continues from the last
//     one given out (each barometer has its own ground pressure and drift - a step would be a climb or a sink to the hold).
// The offsets are cleared while disarmed and on the preferred sensor: on the ground every barometer reads its own 0 again.
// ============================================================================
namespace barosel {

class Selector {
public:
    static constexpr uint32_t kBackMs = 2000;
    void configure(uint8_t primary, bool have2) { pref_ = (have2 && primary == 1) ? 1 : 0; have2_ = have2; cur_ = -1; off_[0] = off_[1] = 0; switches_ = 0; good_since_ = 0; good_ = false; seen_[0] = seen_[1] = false; have_out_ = false; }

    // One pass of the sensor thread. valid[i]: BaroSensor::valid(); fresh[i]: a new sample this pass, alt[i] / sigma[i] its values.
    // true: a new altitude for the estimator in alt_out / sigma_out.
    bool update(const bool valid[2], const bool fresh[2], const float alt[2], const float sigma[2], bool armed, uint32_t now_ms, float& alt_out, float& sigma_out) {
        const int n = have2_ ? 2 : 1;
        for (int i = 0; i < n; i++) if (fresh[i]) { last_[i] = alt[i]; seen_[i] = true; }
        // how long the preferred one has been valid without a break
        if (valid[pref_]) { if (!good_) { good_ = true; good_since_ = now_ms; } } else good_ = false;
        int want = cur_;
        if (cur_ < 0) want = valid[pref_] ? pref_ : (n == 2 && valid[1 - pref_]) ? 1 - pref_ : -1;
        else if (!valid[cur_] && n == 2 && valid[1 - cur_]) want = 1 - cur_;
        else if (cur_ != pref_ && good_ && (!armed || (now_ms - good_since_) >= kBackMs)) want = pref_;
        if (want != cur_ && want >= 0) {
            if (cur_ >= 0 && have_out_ && seen_[want]) { off_[want] = out_ - last_[want]; switches_++; }
            cur_ = want;
        }
        if (!armed && cur_ == pref_) off_[cur_] = 0;
        if (cur_ < 0 || !fresh[cur_] || !valid[cur_]) return false;
        out_ = alt[cur_] + off_[cur_]; have_out_ = true;
        alt_out = out_; sigma_out = sigma[cur_];
        return true;
    }
    int in_use() const { return cur_; }                    // -1 none yet, 0 / 1
    uint16_t switches() const { return switches_; }
    float offset(int i) const { return off_[i & 1]; }

private:
    float last_[2] = {0, 0}, off_[2] = {0, 0}, out_ = 0;
    bool seen_[2] = {false, false}, have2_ = false, have_out_ = false, good_ = false;
    int pref_ = 0, cur_ = -1;
    uint32_t good_since_ = 0;
    uint16_t switches_ = 0;
};

} // namespace barosel
