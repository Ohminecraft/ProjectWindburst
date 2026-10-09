#pragma once
#include <stdint.h>

// ============================================================================
// The tuning sliders (Betaflight's "simplified tuning", config/simplified_tuning.c): a handful of multipliers from which the
// PID gains and the filter cutoffs are computed, instead of typing each number. 100 = the default, 0..200 %.
//
//   Damping            slider_d          D (and D max) of every axis
//   Tracking           slider_pi         P and I of every axis
//   Stick response     slider_ff         feedforward
//   Dynamic damping    slider_d_max      how far D max sits above D (0 = D max equals D: no boost)
//   Drift - wobble     slider_i          I only
//   Pitch damping      slider_pitch_d    pitch D relative to roll D
//   Pitch tracking     slider_pitch_pi   pitch P, I and feedforward relative to roll
//   Master multiplier  slider_master     everything
//
// The products are truncated to whole numbers exactly as the C code of Betaflight does (constrain() takes ints).
// ============================================================================
namespace fc {

constexpr uint8_t kPidDefaultP[3] = {45, 47, 45}, kPidDefaultI[3] = {80, 84, 80}, kPidDefaultD[3] = {30, 34, 0}, kPidDefaultDMax[3] = {40, 46, 0};
constexpr uint16_t kPidDefaultF[3] = {120, 125, 120};
constexpr int kPidGainMax = 250, kFGainMax = 1000;

enum SliderMode : uint8_t { SLIDERS_OFF, SLIDERS_RP, SLIDERS_RPY };

struct PidSliders { uint8_t mode = SLIDERS_RPY, d = 100, pi = 100, ff = 100, d_max = 100, i = 100, pitch_d = 100, pitch_pi = 100, master = 100; };

inline int slider_clamp(float v, int lo, int hi) { const int x = (int)v; return x < lo ? lo : (x > hi ? hi : x); }

// Overwrites the gains of the axes that are under the sliders (roll + pitch, or all three); the others are left alone.
inline void sliders_to_pid(const PidSliders& s, uint8_t p[3], uint8_t i[3], uint8_t d[3], uint8_t d_max[3], uint16_t f[3]) {
    if (s.mode == SLIDERS_OFF) return;
    const float master = s.master / 100.0f, pi_gain = s.pi / 100.0f, d_gain = s.d / 100.0f, ff_gain = s.ff / 100.0f, i_gain = s.i / 100.0f;
    const int last = s.mode == SLIDERS_RP ? 1 : 2;
    for (int a = 0; a <= last; a++) {
        const float pitch_d = a == 1 ? s.pitch_d / 100.0f : 1.0f, pitch_pi = a == 1 ? s.pitch_pi / 100.0f : 1.0f;
        p[a] = (uint8_t)slider_clamp(kPidDefaultP[a] * master * pi_gain * pitch_pi, 0, kPidGainMax);
        i[a] = (uint8_t)slider_clamp(kPidDefaultI[a] * master * pi_gain * i_gain * pitch_pi, 0, kPidGainMax);
        d[a] = (uint8_t)slider_clamp(kPidDefaultD[a] * master * d_gain * pitch_d, 0, kPidGainMax);
        f[a] = (uint16_t)slider_clamp(kPidDefaultF[a] * master * pitch_pi * ff_gain, 0, kFGainMax);
        // slider 100: D max at its default; 0: D max = D; 200: twice as far above D
        const float dmax_gain = kPidDefaultDMax[a] > 0 ? s.d_max / 100.0f + (1.0f - s.d_max / 100.0f) * kPidDefaultD[a] / kPidDefaultDMax[a] : 1.0f;
        d_max[a] = (uint8_t)slider_clamp(kPidDefaultDMax[a] * master * d_gain * pitch_d * dmax_gain, 0, kPidGainMax);
    }
}

// ---- filter sliders: the low-pass cutoffs scale with the multiplier (10..200 %); a filter that is off (0) stays off.
constexpr int kGyroLpf1DynMinDefault = 250, kGyroLpf1DynMaxDefault = 500, kGyroLpf2Default = 500;
constexpr int kDtermLpf1DynMinDefault = 75, kDtermLpf1DynMaxDefault = 150, kDtermLpf2Default = 150;
constexpr int kLpfMaxHz = 1000;

inline uint16_t slider_hz(int def, uint8_t mult) { const int v = def * mult / 100; return (uint16_t)(v < 0 ? 0 : (v > kLpfMaxHz ? kLpfMaxHz : v)); }
inline void gyro_filter_slider(uint8_t mult, uint16_t& lpf1_static, uint16_t& lpf1_dyn_min, uint16_t& lpf1_dyn_max, uint16_t& lpf2) {
    if (lpf1_dyn_min) { lpf1_dyn_min = slider_hz(kGyroLpf1DynMinDefault, mult); lpf1_dyn_max = slider_hz(kGyroLpf1DynMaxDefault, mult); }
    if (lpf1_static) lpf1_static = slider_hz(kGyroLpf1DynMinDefault, mult);
    if (lpf2) lpf2 = slider_hz(kGyroLpf2Default, mult);
}
inline void dterm_filter_slider(uint8_t mult, uint16_t& lpf1_static, uint16_t& lpf1_dyn_min, uint16_t& lpf1_dyn_max, uint16_t& lpf2) {
    if (lpf1_dyn_min) { lpf1_dyn_min = slider_hz(kDtermLpf1DynMinDefault, mult); lpf1_dyn_max = slider_hz(kDtermLpf1DynMaxDefault, mult); }
    if (lpf1_static) lpf1_static = slider_hz(kDtermLpf1DynMinDefault, mult);
    if (lpf2) lpf2 = slider_hz(kDtermLpf2Default, mult);
}
} // namespace fc
