/*
 * This file is part of Windburst Project
 */

#pragma once

#include "mag_driver.h"

// ============================================================================
// A compass whose samples come from elsewhere: a DroneCAN module (MagneticFieldStrength2, handed in by the CAN node).
// push() takes the field in the module's frame (x forward, y right, z down - the DroneCAN convention); it is turned into the
// chip convention of the other drivers (Betaflight's CW0: x forward, y LEFT, z UP), so that mag_align CW0 means "the module's
// arrow points forward". Healthy while samples keep coming (a module that falls silent for 500 ms is not).
// ============================================================================
class ExternalMag : public MagDriver {
public:
    explicit ExternalMag(const char* name) : name_(name) {}
    const char* name() const override { return name_; }
    bool init() override { have_ = fresh_ = false; n_ = 0; return true; }
    void push(const float frd_ga[3], uint32_t now_us) {
        g_[0] = frd_ga[0]; g_[1] = -frd_ga[1]; g_[2] = -frd_ga[2];
        if (have_ && now_us != last_us_) { const float dt = (now_us - last_us_) * 1e-6f; if (dt > 0 && dt < 1.0f) rate_ = rate_ * 0.9f + 0.1f / dt; }
        last_us_ = now_us; have_ = fresh_ = true; n_++;
    }
    bool update(uint32_t now_us, MagSample& out) override {
        now_us_ = now_us;
        if (!fresh_) return false;
        fresh_ = false; for (int k = 0; k < 3; k++) out.gauss[k] = g_[k];
        return true;
    }
    float rate_hz() const override { return rate_; }
    bool healthy() const override { return have_ && (int32_t)(now_us_ - last_us_) < 500000; }        // (signed: a sample pushed after the last update() is newer, not 71 minutes old)
    uint32_t samples() const { return n_; }
private:
    const char* name_; float g_[3] = {0, 0, 0}, rate_ = 0; bool have_ = false, fresh_ = false; uint32_t last_us_ = 0, now_us_ = 0, n_ = 0;
};
