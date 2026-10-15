#pragma once
#include "../config/fast_code.h"
#include "types.h"

// ============================================================================
// Low-pass and notch filters.
// The forms and the constants are Betaflight's (common/filter.c), so that a cutoff typed in the app does what a pilot who knows
// Betaflight expects: PT1 / PT2 / PT3 (cascaded first-order sections, cutoff corrected so that -3 dB is AT the given frequency),
// and state-variable filters (SVF, "TPT" form) for the second-order low-pass ("BIQUAD" in the app: Butterworth, Q = 0.707) and
// for the notches. An SVF keeps its state meaningful when its frequency is moved every few milliseconds (dynamic low-pass,
// RPM notches, dynamic notches); a direct-form biquad does not.
//
// Every filter starts from its first input (no step from zero at power-on / after reset()).
// ============================================================================
namespace fc {

inline float pt1_gain(float hz, float dt) { const float w = 2.0f * kPi * hz * dt; return w / (w + 1.0f); }
// gain of a PT1 that reaches 63 % of a step after `delay` seconds
inline float pt1_gain_from_delay(float delay, float dt) { return delay <= 0.0f ? 1.0f : dt / (dt + delay); }
constexpr float kPt2Correction = 1.553773974f;    // 1 / sqrt(2^(1/2) - 1)
constexpr float kPt3Correction = 1.961459177f;    // 1 / sqrt(2^(1/3) - 1)

struct Pt1 {                       // first-order low-pass; cutoff <= 0 => bypass
    float k = 1.0f, y = 0.0f; bool init = false;
    void set_cutoff(float hz, float dt) { k = hz > 0.0f ? pt1_gain(hz, dt) : 1.0f; }
    void set_gain(float gain) { k = gain; }
    WB_FAST float apply(float x) { if (!init) { y = x; init = true; } y += k * (x - y); return y; }
    void reset() { init = false; }
    void reset(float v) { y = v; init = true; }
};
struct Pt2 {
    float k = 1.0f, y1 = 0.0f, y = 0.0f; bool init = false;
    void set_cutoff(float hz, float dt) { k = hz > 0.0f ? pt1_gain(hz * kPt2Correction, dt) : 1.0f; }
    WB_FAST float apply(float x) { if (!init) { y1 = y = x; init = true; } y1 += k * (x - y1); y += k * (y1 - y); return y; }
    void reset() { init = false; }
    void reset(float v) { y1 = y = v; init = true; }
};
struct Pt3 {
    float k = 1.0f, y1 = 0.0f, y2 = 0.0f, y = 0.0f; bool init = false;
    void set_cutoff(float hz, float dt) { k = hz > 0.0f ? pt1_gain(hz * kPt3Correction, dt) : 1.0f; }
    void set_gain(float gain) { k = gain; }
    WB_FAST float apply(float x) { if (!init) { y1 = y2 = y = x; init = true; } y1 += k * (x - y1); y2 += k * (y1 - y2); y += k * (y2 - y); return y; }
    void reset() { init = false; }
    void reset(float v) { y1 = y2 = y = v; init = true; }
};

// Second-order Butterworth low-pass as a state-variable filter. hz must be below fs / 2.
struct SvfLowpass {
    float f = 0.0f, a1 = 1.0f, a2 = 0.0f, ic1 = 0.0f, ic2 = 0.0f; bool init = false;
    WB_FAST void set(float hz, float dt) {
        f = tan_approx(kPi * hz * dt);
        const float inv = 1.0f / (1.0f + f * (f + 1.41421356237f));          // 1 / Q = sqrt(2)
        a1 = inv; a2 = f * inv;
    }
    WB_FAST float apply(float x) {
        if (!init) { ic1 = 0.0f; ic2 = x; init = true; }                     // settled on the first input
        const float v3 = x - ic2, v1 = a1 * ic1 + a2 * v3, v2 = ic2 + f * v1;
        ic1 = 2.0f * v1 - ic1; ic2 = 2.0f * v2 - ic2;
        return v2;
    }
    void reset() { init = false; }
};

// Notch as a state-variable filter. Q = centre / bandwidth (see notch_q()).
struct SvfNotch {
    float a1 = 1.0f, a2q = 0.0f, fq = 0.0f, ic1q = 0.0f, ic2 = 0.0f; bool init = false;
    WB_FAST void set(float hz, float dt, float q) {
        const float f = tan_approx(kPi * hz * dt), iq = 1.0f / q, inv = 1.0f / (1.0f + f * (f + iq));
        a1 = inv; a2q = f * inv * iq; fq = f * q;
    }
    WB_FAST float apply(float x) {
        if (!init) { ic1q = 0.0f; ic2 = x; init = true; }
        const float v3 = x - ic2, v1q = a1 * ic1q + a2q * v3, v2 = ic2 + fq * v1q;
        ic1q = 2.0f * v1q - ic1q; ic2 = 2.0f * v2 - ic2;
        return x - v1q;
    }
    void reset() { init = false; }
};
// Q of a notch from its centre and its lower -3 dB frequency ("cutoff" in the app), as Betaflight defines it:
// the upper edge is centre^2 / cutoff, Q = centre / (upper - lower).
inline float notch_q(float center_hz, float cutoff_hz) { return center_hz * cutoff_hz / (center_hz * center_hz - cutoff_hz * cutoff_hz); }

// One RPM notch for the three gyro axes at once (they share the frequency), with a weight: 1 = full notch, 0 = no effect.
// The weight fades the notch out near its minimum frequency instead of switching it.
struct RpmNotch {
    float f = 0.0f, a1 = 1.0f, a2 = 0.0f, wq = 0.0f, ic1[3] = {0, 0, 0}, ic2[3] = {0, 0, 0};
    WB_FAST void set(float hz, float dt, float q, float weight) { set_inv_q(hz, dt, 1.0f / q, weight); }
    WB_FAST void set_inv_q(float hz, float dt, float iq, float weight) {      // (iq = 1 / q: the RPM filter moves a few notches every cycle and keeps it)
        f = tan_approx(kPi * hz * dt);
        a1 = 1.0f / (1.0f + f * (f + iq)); a2 = f * a1; wq = iq * weight;
    }
    void apply(float v[3]) {
        for (int i = 0; i < 3; i++) {
            const float v3 = v[i] - ic2[i], v1 = a1 * ic1[i] + a2 * v3, v2 = ic2[i] + f * v1;
            ic1[i] = 2.0f * v1 - ic1[i]; ic2[i] = 2.0f * v2 - ic2[i];
            v[i] -= wq * v1;
        }
    }
    void reset() { for (int i = 0; i < 3; i++) ic1[i] = ic2[i] = 0.0f; }
};

// A low-pass whose kind is a setting (gyro low-pass 1 / 2, D-term low-pass 1 / 2).
enum LpfType : uint8_t { LPF_PT1, LPF_BIQUAD, LPF_PT2, LPF_PT3, LPF_TYPE_COUNT };
class Lowpass {
public:
    // hz 0 = off. A BIQUAD at or above 95 % of the Nyquist frequency is switched off as well (it would be unstable).
    void configure(uint8_t type, float hz, float dt) {
        type_ = type < LPF_TYPE_COUNT ? type : (uint8_t)LPF_PT1; dt_ = dt; on_ = hz > 0.0f && dt > 0.0f;
        if (on_ && type_ == LPF_BIQUAD && hz >= 0.475f / dt) on_ = false;
        if (on_) set_cutoff(hz);
        reset();
    }
    // Moves the cutoff of a running filter (dynamic low-pass). No effect on a filter that is off.
    WB_FAST void set_cutoff(float hz) {
        if (!on_ || hz <= 0.0f) return;
        hz_ = hz;
        switch (type_) {
        case LPF_BIQUAD: svf_.set(hz < 0.475f / dt_ ? hz : 0.475f / dt_, dt_); break;
        case LPF_PT2: k_ = pt1_gain(hz * kPt2Correction, dt_); break;
        case LPF_PT3: k_ = pt1_gain(hz * kPt3Correction, dt_); break;
        default: k_ = pt1_gain(hz, dt_); break;
        }
    }
    WB_FAST float apply(float x) {
        if (!on_) return x;
        if (type_ == LPF_BIQUAD) return svf_.apply(x);
        if (!init_) { s_[0] = s_[1] = s_[2] = x; init_ = true; }
        s_[0] += k_ * (x - s_[0]);
        if (type_ == LPF_PT1) return s_[0];
        s_[1] += k_ * (s_[0] - s_[1]);
        if (type_ == LPF_PT2) return s_[1];
        s_[2] += k_ * (s_[1] - s_[2]);
        return s_[2];
    }
    void reset() { init_ = false; svf_.reset(); }
    bool active() const { return on_; }
    float cutoff_hz() const { return on_ ? hz_ : 0.0f; }
private:
    uint8_t type_ = LPF_PT1; bool on_ = false, init_ = false; float dt_ = 0.00025f, hz_ = 0.0f, k_ = 1.0f, s_[3] = {0, 0, 0}; SvfLowpass svf_;
};

// A static notch given the way the app gives it: centre and lower cutoff (0 = off). Centre above 95 % of Nyquist: clamped there
// if the cutoff is still below it, otherwise off (Betaflight's calculateNyquistAdjustedNotchHz).
class StaticNotch {
public:
    void configure(float center_hz, float cutoff_hz, float dt) {
        on_ = false; n_.reset();
        if (dt <= 0.0f) return;
        const float nyq = 0.475f / dt;
        if (center_hz > nyq) center_hz = cutoff_hz < nyq ? nyq : 0.0f;
        if (center_hz <= 0.0f || cutoff_hz <= 0.0f || cutoff_hz >= center_hz) return;
        n_.set(center_hz, dt, notch_q(center_hz, cutoff_hz)); on_ = true;
    }
    WB_FAST float apply(float x) { return on_ ? n_.apply(x) : x; }
    bool active() const { return on_; }
private:
    SvfNotch n_; bool on_ = false;
};

// Mean of the last N inputs (N <= 4): feedforward averaging.
struct MovingAverage {
    float buf[4] = {0, 0, 0, 0}, sum = 0.0f; uint8_t size = 1, idx = 0; bool primed = false;
    void init(uint8_t n) { size = n < 1 ? 1 : (n > 4 ? 4 : n); idx = 0; sum = 0.0f; primed = false; for (float& b : buf) b = 0.0f; }
    WB_FAST float apply(float x) {
        sum -= buf[idx]; buf[idx] = x; sum += x;
        if (++idx == size) { idx = 0; primed = true; }
        return sum / (float)(primed ? size : idx);
    }
};
} // namespace fc
