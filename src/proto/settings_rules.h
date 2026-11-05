#pragma once
#include "wbp_schema.h"
#include "../core/simplified_tuning.h"

// ============================================================================
// Relations between settings that a range check cannot express. Applied after every range validation of a section that came
// from outside (written by the app, loaded from flash), so the RAM copy the firmware runs on is always consistent and the app
// reads back exactly what is in effect:
//
//   pid      while an axis is under the tuning sliders its p / i / d / d_max / f are COMPUTED from the sliders (what the app wrote
//            into those fields is replaced) - Betaflight's simplified tuning, config/simplified_tuning.c
//   filters  while a filter slider is on, the low-pass cutoffs it owns are computed from its multiplier
//   motors   3D needs a protocol that carries a reversed range (DShot, PWM / Oneshot / Multishot): off for DroneCAN and brushed
// ============================================================================
namespace wbp {

inline void apply_rules(S_pid& p) {
    fc::PidSliders s;
    s.mode = p.slider_mode; s.d = p.slider_d; s.pi = p.slider_pi; s.ff = p.slider_ff; s.d_max = p.slider_d_max; s.i = p.slider_i;
    s.pitch_d = p.slider_pitch_d; s.pitch_pi = p.slider_pitch_pi; s.master = p.slider_master;
    uint8_t pp[3], ii[3], dd[3], dm[3]; uint16_t ff[3];                // (the struct is packed: work on aligned copies)
    for (int a = 0; a < 3; a++) { pp[a] = p.p[a]; ii[a] = p.i[a]; dd[a] = p.d[a]; dm[a] = p.d_max[a]; ff[a] = p.f[a]; }
    fc::sliders_to_pid(s, pp, ii, dd, dm, ff);
    for (int a = 0; a < 3; a++) { p.p[a] = pp[a]; p.i[a] = ii[a]; p.d[a] = dd[a]; p.d_max[a] = dm[a]; p.f[a] = ff[a]; }
}

inline void apply_rules(S_filters& f) {
    if (f.slider_gyro) {
        uint16_t st = f.gyro_lpf1_static_hz, mn = f.gyro_lpf1_dyn_min_hz, mx = f.gyro_lpf1_dyn_max_hz, l2 = f.gyro_lpf2_hz;
        fc::gyro_filter_slider(f.slider_gyro_mult, st, mn, mx, l2);
        f.gyro_lpf1_static_hz = st; f.gyro_lpf1_dyn_min_hz = mn; f.gyro_lpf1_dyn_max_hz = mx; f.gyro_lpf2_hz = l2;
    }
    if (f.slider_dterm) {
        uint16_t st = f.dterm_lpf1_static_hz, mn = f.dterm_lpf1_dyn_min_hz, mx = f.dterm_lpf1_dyn_max_hz, l2 = f.dterm_lpf2_hz;
        fc::dterm_filter_slider(f.slider_dterm_mult, st, mn, mx, l2);
        f.dterm_lpf1_static_hz = st; f.dterm_lpf1_dyn_min_hz = mn; f.dterm_lpf1_dyn_max_hz = mx; f.dterm_lpf2_hz = l2;
    }
}

inline void apply_rules(S_motors& m) {
    if (m.protocol == ESC_PROTOCOL_DRONECAN || m.protocol == ESC_PROTOCOL_BRUSHED) m.mode3d = 0;
}

inline void apply_rules(Settings& s, uint8_t section) {
    if (section == SEC_PID) apply_rules(s.pid);
    else if (section == SEC_FILTERS) apply_rules(s.filters);
    else if (section == SEC_MOTORS) apply_rules(s.motors);
}

} // namespace wbp
