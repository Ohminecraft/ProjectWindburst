/*
 * This file is part of Windburst Project
 */

#pragma once

#include "baro_driver.h"

// ============================================================================
// A barometer whose samples come from elsewhere: a DroneCAN module (StaticPressure + StaticTemperature, handed in by the CAN
// node). push() takes the pressure in Pa and the temperature in deg C; update() hands each pushed sample out once. Healthy while
// samples keep coming (a module that falls silent for 500 ms is not). The rate is measured from the pushes.
// ============================================================================
class ExternalBaro : public BaroDriver {
public:
    explicit ExternalBaro(const char* name) : name_(name) {}
    const char* name() const override { return name_; }
    bool init(const BaroInitConfig&) override { have_ = fresh_ = false; n_ = 0; rate_ = 0; return true; }
    void push(float pa, float temp_c, uint32_t now_us) {
        s_.pressure_pa = pa; s_.temp_c = temp_c;
        if (have_ && now_us != last_us_) { const float dt = (now_us - last_us_) * 1e-6f; if (dt > 0 && dt < 1.0f) rate_ = rate_ > 0 ? rate_ * 0.9f + 0.1f / dt : 1.0f / dt; }
        last_us_ = now_us; have_ = fresh_ = true; n_++;
    }
    bool update(uint32_t now_us, BaroSample& out) override {
        now_us_ = now_us;
        if (!fresh_) return false;
        fresh_ = false; out = s_; return true;
    }
    float nominal_rate_hz() const override { return rate_; }
    bool healthy() const override { return have_ && (int32_t)(now_us_ - last_us_) < 500000; }
    uint32_t samples() const { return n_; }
private:
    const char* name_; BaroSample s_; float rate_ = 0; bool have_ = false, fresh_ = false; uint32_t last_us_ = 0, now_us_ = 0, n_ = 0;
};
