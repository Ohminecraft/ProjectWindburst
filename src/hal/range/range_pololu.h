/*
 * This file is part of Windburst Project
 */

#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "range_driver.h"

// ============================================================================
// VL53L1X / VL53L0X drivers = thin, NON-BLOCKING adapters on Pololu's MIT-licensed Arduino libraries.
//
// Why a library and not a hand-written register driver (like the IMU / baro): these ST chips need a proprietary bring-up (factory SPAD
// calibration read from NVM, reference calibration, a long tuning-register load, 16-bit register addressing for the L1X) that ST only
// publishes as compiled API code. A from-memory re-implementation that silently reports plausible-but-wrong distances would be far worse
// for an altitude loop than a missing sensor. Pololu's libraries are that validated sequence (VL53L0X 1.3.1, VL53L1X 1.3.1).
//
// platformio.ini:   lib_deps = pololu/VL53L0X@^1.3.1
//                              pololu/VL53L1X@^1.3.1
// If a library is absent the matching driver compiles to "library missing" and init() returns false - nothing else breaks.
// ============================================================================
#if defined(__has_include)
  #if __has_include(<VL53L1X.h>)
    #include <VL53L1X.h>
    #define WB_HAS_VL53L1X 1
  #endif
  #if __has_include(<VL53L0X.h>)
    #include <VL53L0X.h>
    #define WB_HAS_VL53L0X 1
  #endif
#endif

#ifdef WB_HAS_VL53L1X
class Vl53l1xRange : public RangeDriver {
public:
    enum Mode : uint8_t { SHORT_MODE, MEDIUM_MODE, LONG_MODE };
    Vl53l1xRange() = default;
    Vl53l1xRange(TwoWire& wire, Mode mode = LONG_MODE, uint16_t budget_ms = 33) : wire_(&wire), mode_(mode), budget_ms_(budget_ms) {}
    void bind(TwoWire& wire, Mode mode, uint16_t budget_ms = 33) { wire_ = &wire; mode_ = mode; budget_ms_ = budget_ms; }
    const char* name() const override { return "VL53L1X"; }

    bool init() override {
        ok_ = false; errors_ = 0;
        if (!wire_) return false;
        s_.setBus(wire_);
        s_.setTimeout(100);
        if (!s_.init()) return false;                                   // model id 0xEACC + full ST bring-up
        s_.setDistanceMode(mode_ == SHORT_MODE ? VL53L1X::Short : (mode_ == MEDIUM_MODE ? VL53L1X::Medium : VL53L1X::Long));
        s_.setMeasurementTimingBudget((uint32_t)budget_ms_ * 1000u);
        s_.startContinuous(budget_ms_);
        ok_ = true;
        return true;
    }
    bool update(uint32_t, RangeReading& out) override {
        if (!ok_) return false;
        if (!s_.dataReady()) return false;
        s_.read(false);                                                  // results already latched: does not wait
        if (s_.timeoutOccurred()) { if (errors_ < 255) errors_++; return false; }
        const auto st = s_.ranging_data.range_status;
        errors_ = 0;
        // valid statuses: 0 RangeValid, 3 MinRangeClipped (at the sensor's minimum), 6 NoWrapCheckFail (valid, wrap check skipped)
        const bool good = st == VL53L1X::RangeValid, soft = st == VL53L1X::RangeValidMinRangeClipped || st == VL53L1X::RangeValidNoWrapCheckFail;
        if (!good && !soft) return false;
        out.distance_m = s_.ranging_data.range_mm * 0.001f;
        const float sig = s_.ranging_data.peak_signal_count_rate_MCPS;   // Mcps; ~>1 strong, <0.1 weak
        float q = sig >= 2.0f ? 100.0f : (sig / 2.0f) * 100.0f;
        if (soft) q *= 0.5f;
        out.quality_pct = (uint8_t)(q < 1.0f ? 1.0f : q);
        return true;
    }
    float max_range_m() const override { return mode_ == SHORT_MODE ? 1.3f : (mode_ == MEDIUM_MODE ? 3.0f : 4.0f); }
    float min_range_m() const override { return 0.04f; }
    float nominal_rate_hz() const override { return 1000.0f / budget_ms_; }
    bool healthy() const override { return ok_ && errors_ < 10; }
private:
    TwoWire* wire_ = nullptr; Mode mode_ = LONG_MODE; uint16_t budget_ms_ = 33; VL53L1X s_; bool ok_ = false; uint8_t errors_ = 0;
};
#endif

#ifdef WB_HAS_VL53L0X
class Vl53l0xRange : public RangeDriver {
public:
    Vl53l0xRange() = default;
    explicit Vl53l0xRange(TwoWire& wire, uint16_t budget_ms = 33) : wire_(&wire), budget_ms_(budget_ms) {}
    void bind(TwoWire& wire, uint16_t budget_ms = 33) { wire_ = &wire; budget_ms_ = budget_ms; }
    const char* name() const override { return "VL53L0X"; }

    bool init() override {
        ok_ = false; errors_ = 0;
        if (!wire_) return false;
        s_.setBus(wire_);
        s_.setTimeout(100);
        if (!s_.init()) return false;                                   // model id 0xEE + SPAD / reference calibration + tuning
        s_.setMeasurementTimingBudget((uint32_t)budget_ms_ * 1000u);
        s_.startContinuous((uint32_t)budget_ms_);
        ok_ = true;
        return true;
    }
    // Same register sequence as Pololu's readRangeContinuousMillimeters(), minus the wait: if no result is latched yet, return at once.
    bool update(uint32_t, RangeReading& out) override {
        if (!ok_) return false;
        if ((s_.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) == 0) return false;
        const uint16_t mm = s_.readReg16Bit(VL53L0X::RESULT_RANGE_STATUS + 10);
        s_.writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 0x01);
        if (mm >= 8190) { return false; }                                // 8190/8191 = no target / out of range
        out.distance_m = mm * 0.001f;
        out.quality_pct = 70;                                            // this chip's library exposes no signal figure
        errors_ = 0;
        return true;
    }
    float max_range_m() const override { return 1.2f; }                  // reliable indoors; ~2 m in the dark
    float min_range_m() const override { return 0.03f; }
    float nominal_rate_hz() const override { return 1000.0f / budget_ms_; }
    bool healthy() const override { return ok_; }
private:
    TwoWire* wire_ = nullptr; uint16_t budget_ms_ = 33; VL53L0X s_; bool ok_ = false; uint8_t errors_ = 0;
};
#endif
