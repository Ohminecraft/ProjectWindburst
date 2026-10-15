/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>
#include <math.h>

// ============================================================================
// Battery monitor (portable, no Arduino includes): pack voltage -> filtered voltage, cell count, state.
// Same behaviour as Betaflight sensors/battery.c + sensors/voltage.c, so the numbers a pilot knows from there mean the same here:
//
//   * the voltage is low-pass filtered (0.33 Hz, Betaflight vbat_display_lpf_period = 30) - a throttle punch does not look like an empty pack
//   * a battery is "present" once the filtered voltage is stable (within 0.1 V for ~1 s) and plausible:
//       3.0 V .. one full cell, or more than 6.0 V. In between is a USB supply (5 V), not a battery.
//   * the cell count is the configured one, or detected at that moment:  cells = voltage / cell_max + 1   (max 8)
//       A pack plugged in nearly empty is detected one cell short (4S at 12.8 V reads as a full 3S): set the cell count when you can.
//   * OK -> WARNING below cells * cell_warning, WARNING -> CRITICAL below cells * cell_min, each after its delay and with 10 mV
//     hysteresis; CRITICAL -> WARNING -> OK as soon as the voltage is back above the level.
//
// Units: volts, seconds, milliseconds. Call update() at a steady rate (the filter is tuned for kRateHz).
// ============================================================================
namespace batt {

enum class State : uint8_t { NOT_PRESENT, OK, WARNING, CRITICAL };

struct Config {
    uint8_t cells = 0;                                  // 0 = detect when the battery is connected
    float cell_max_v = 4.30f, cell_warning_v = 3.50f, cell_min_v = 3.30f;
    float warning_delay_s = 0.5f, critical_delay_s = 2.0f;
};

constexpr float kRateHz = 50.0f;                        // Betaflight SLOW_VOLTAGE_TASK_FREQ_HZ
constexpr float kDisplayLpfHz = 1.0f / 3.0f;            // vbat_display_lpf_period 30 -> 1 / (30 / 10) Hz
constexpr float kNotPresentCellV = 3.0f;                // vbatnotpresentcellvoltage
constexpr float kStableMaxDeltaV = 0.10f;               // VOLTAGE_STABLE_MAX_DELTA
constexpr uint32_t kStableTickMs = 100;                 // VOLTAGE_STABLE_TICK_MS
constexpr int kStableBitsTotal = 11, kStableBitsThreshold = 10;
constexpr float kHysteresisV = 0.01f;                   // vbathysteresis
constexpr int kMaxAutoCells = 8;

class Monitor {
public:
    Config cfg;

    void reset() { *this = Monitor(cfg); }
    Monitor() {}
    explicit Monitor(const Config& c) : cfg(c) {}

    // volts = pack voltage from the source, valid = the source delivered a reading (false: treated as 0 V)
    void update(uint32_t now_ms, float volts, bool valid) {
        const float v = (valid && volts == volts && volts > 0.0f) ? volts : 0.0f;
        raw_ = v;
        if (!seeded_) { filt_ = v; seeded_ = true; stable_ref_ = v; stable_tick_ms_ = now_ms; change_ms_ = now_ms; }
        else filt_ += kGain * (v - filt_);

        // ---- stability: a shift register, one bit per 100 ms, cleared when the voltage left the 0.1 V window in that period
        if (fabsf(stable_ref_ - filt_) > kStableMaxDeltaV) { stable_ref_ = filt_; stable_bits_ &= ~1u; }
        if ((uint32_t)(now_ms - stable_tick_ms_) >= kStableTickMs) { stable_bits_ = (uint16_t)((stable_bits_ << 1) | 1u); stable_tick_ms_ = now_ms; }

        // ---- presence
        if (state_ == State::NOT_PRESENT && from_battery() && stable()) {
            state_ = State::OK; change_ms_ = now_ms;
            int c = (int)(filt_ / cfg.cell_max_v) + 1; if (c > kMaxAutoCells) c = kMaxAutoCells;
            auto_cells_ = (uint8_t)c;                                  // what the voltage says, kept for when the setting is 0 = detect
        } else if (state_ != State::NOT_PRESENT && stable() && !from_battery()) {
            state_ = State::NOT_PRESENT; cells_ = 0;
        }
        if (state_ == State::NOT_PRESENT) return;
        cells_ = cfg.cells ? cfg.cells : auto_cells_;                  // the setting wins, also when it is typed while the battery is connected

        // ---- level
        const float warn = cells_ * cfg.cell_warning_v, crit = cells_ * cfg.cell_min_v;
        switch (state_) {
        case State::OK:
            if (filt_ <= warn - kHysteresisV) { if ((uint32_t)(now_ms - change_ms_) >= (uint32_t)(cfg.warning_delay_s * 1000.0f)) { state_ = State::WARNING; change_ms_ = now_ms; } }
            else change_ms_ = now_ms;
            break;
        case State::WARNING:
            if (filt_ <= crit - kHysteresisV) { if ((uint32_t)(now_ms - change_ms_) >= (uint32_t)(cfg.critical_delay_s * 1000.0f)) state_ = State::CRITICAL; }
            else { if (filt_ > warn) state_ = State::OK; change_ms_ = now_ms; }
            break;
        case State::CRITICAL:
            if (filt_ > crit) { state_ = State::WARNING; change_ms_ = now_ms; }
            break;
        default: break;
        }
    }

    State state() const { return state_; }
    bool present() const { return state_ != State::NOT_PRESENT; }
    bool low() const { return state_ == State::WARNING || state_ == State::CRITICAL; }      // do not take off on this pack
    bool critical() const { return state_ == State::CRITICAL; }                             // land now
    float voltage() const { return filt_; }                                                 // filtered pack voltage
    float unfiltered() const { return raw_; }
    uint8_t cells() const { return cells_; }                                                // 0 while no battery is present
    float cell_voltage() const { return cells_ ? filt_ / (float)cells_ : 0.0f; }
    bool cells_detected() const { return cells_ != 0 && cfg.cells == 0; }
    bool stable() const {
        int n = 0; for (int i = 0; i < kStableBitsTotal; i++) n += (stable_bits_ >> i) & 1u;
        return n >= kStableBitsThreshold;
    }

private:
    // PT1 gain for kDisplayLpfHz at kRateHz: dt / (RC + dt)
    static constexpr float kDt = 1.0f / kRateHz, kRc = 1.0f / (6.283185307f * kDisplayLpfHz), kGain = kDt / (kRc + kDt);
    bool from_battery() const { return (filt_ >= kNotPresentCellV && filt_ <= cfg.cell_max_v) || filt_ > 2.0f * kNotPresentCellV; }

    State state_ = State::NOT_PRESENT;
    float filt_ = 0, raw_ = 0, stable_ref_ = 0; bool seeded_ = false;
    uint16_t stable_bits_ = 0; uint32_t stable_tick_ms_ = 0, change_ms_ = 0; uint8_t cells_ = 0, auto_cells_ = 0;
};

// ---------------------------------------------------------------------------------------------------------------------
// Which reading feeds the monitor. AUTO: the ADC when a pin exists, otherwise the ESC telemetry.
// ---------------------------------------------------------------------------------------------------------------------
enum Source : uint8_t { SRC_AUTO, SRC_ADC, SRC_ESC, SRC_NONE, SRC_DRONECAN };        // == schema enum batt_source (DRONECAN appended in 0.17)
inline Source pick_source(Source wanted, bool adc_ok) {
    if (wanted == SRC_AUTO) return adc_ok ? SRC_ADC : SRC_ESC;
    return wanted;
}
// Pin voltage (mV, already calibrated by the ADC driver) -> pack voltage (V).  divider = Vbattery / Vpin.
inline float adc_to_volts(uint32_t pin_mv, float divider) { return (float)pin_mv * 0.001f * divider; }

// ---------------------------------------------------------------------------------------------------------------------
// Current and consumption (Betaflight sensors/current.c): an ADC current sensor, or the sum of what the ESCs report.
//   ADC:  amps = (pin_mV x 10000 / scale + offset_mA) / 1000      scale in 0.1 mV per A (Betaflight ibata_scale: 400 = 40 mV/A,
//         the Allegro ACS758-100U), offset in mA (ibata_offset). The same two numbers as in Betaflight, so a board maker's values fit.
//   ESC:  the sum of the motors' currents from extended DShot telemetry (each ESC reports its own; a 4-in-1 reports per channel).
// The current shown is low-passed at 1 Hz (Betaflight ibat_lpf_period 10); the consumption adds up the unfiltered readings.
// ---------------------------------------------------------------------------------------------------------------------
enum CurrentSource : uint8_t { CUR_AUTO, CUR_ADC, CUR_ESC, CUR_NONE, CUR_DRONECAN };           // == schema enum curr_source (DRONECAN: 0.17)
inline CurrentSource pick_current_source(CurrentSource wanted, bool adc_ok) {
    if (wanted == CUR_AUTO) return adc_ok ? CUR_ADC : CUR_ESC;
    return wanted;
}
inline float adc_to_amps(uint32_t pin_mv, int16_t scale, int16_t offset_ma) {
    return scale ? ((float)pin_mv * 10000.0f / (float)scale + (float)offset_ma) * 0.001f : 0.0f;
}

class CurrentMeter {
public:
    void reset() { *this = CurrentMeter(); }
    // amps: this step's reading; valid: the source delivered one; dt: seconds since the previous call (a gap of a second or more
    // - the sensor task stood still - is not added up: the consumption would jump)
    void update(float amps, bool valid, float dt) {
        valid_ = valid && amps == amps;
        if (!valid_) return;
        latest_ = amps;
        if (!seeded_) { filt_ = amps; seeded_ = true; } else filt_ += kGain * (amps - filt_);
        if (dt > 0.0f && dt < 1.0f) mah_ += amps * dt * (1000.0f / 3600.0f);
    }
    bool valid() const { return valid_; }
    float amps() const { return valid_ ? filt_ : 0.0f; }       // filtered, for showing
    float latest() const { return valid_ ? latest_ : 0.0f; }
    float mah_drawn() const { return mah_; }                    // since power-up (Betaflight does not reset it on arming either)
private:
    static constexpr float kDt = 1.0f / kRateHz, kRc = 1.0f / (6.283185307f * 1.0f), kGain = kDt / (kRc + kDt);
    float latest_ = 0, filt_ = 0, mah_ = 0; bool valid_ = false, seeded_ = false;
};

// What is left of the battery (%), Betaflight calculateBatteryPercentageRemaining(): from the consumption when the capacity is
// known, otherwise from where the voltage stands between empty (cells x cell_min) and full (cells x cell_max). 0 without a battery.
inline uint8_t remaining_percent(float mah_drawn, uint16_t capacity_mah, float volts, uint8_t cells, float cell_min_v, float cell_max_v) {
    if (cells == 0) return 0;
    float p;
    if (capacity_mah > 0) p = ((float)capacity_mah - mah_drawn) * 100.0f / (float)capacity_mah;
    else { const float lo = cells * cell_min_v, hi = cells * cell_max_v; p = hi > lo ? (volts - lo) * 100.0f / (hi - lo) : 0.0f; }
    return p <= 0.0f ? 0 : p >= 100.0f ? 100 : (uint8_t)(p + 0.5f);
}

} // namespace batt
