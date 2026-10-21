// Unit tests of the control stack that was ported from Betaflight / ArduPilot: filters, the gyro filter chain (RPM notches, dynamic
// low-pass, dynamic notch), the rate PID with its helpers, the stick path (RC smoothing, feedforward), Angle and Stabilize, the
// tuning sliders. Every expected number is worked out from the formula it tests (written next to it), not copied from a run.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <random>
#include "../src/core/filters.h"
#include "../src/core/dyn_notch.h"
#include "../src/core/gyro_filter.h"
#include "../src/core/pid.h"
#include "../src/core/setpoint.h"
#include "../src/core/acro_trainer.h"
#include "../src/core/attitude.h"
#include "../src/core/simplified_tuning.h"
#include "../src/core/inflight_trim.h"
using namespace fc;

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, ...) do { const bool ok_ = (cond); if (getenv("V")) { printf("    "); printf(__VA_ARGS__); printf("\n"); } if (ok_) g_pass++; else { g_fail++; printf("  FAIL line %d: %s   ", __LINE__, #cond); printf(__VA_ARGS__); printf("\n"); } } while (0)
static std::mt19937 rng(4321); static float gauss(float s) { static std::normal_distribution<float> n(0, 1); return n(rng) * s; }
static const float FS = 4000.0f, DT = 1.0f / FS;

// amplitude ratio of a filter for a sine of `hz` (after it has settled), by correlation with sin / cos
template <typename F> static float gain_at(F&& apply, float hz, float fs = FS, float seconds = 1.0f) {
    const int n = (int)(fs * seconds), skip = n / 2; double s = 0, c = 0; int m = 0;
    for (int i = 0; i < n; i++) { const float ph = 2 * kPi * hz * (float)i / fs, y = apply(sinf(ph)); if (i >= skip) { s += y * sinf(ph); c += y * cosf(ph); m++; } }
    return (float)(2.0 * sqrt(s * s + c * c) / m);
}
static bool near(float a, float b, float tol) { return fabsf(a - b) <= tol; }
// What `order` cascaded first-order sections y += k (x - y) do to a sine of `hz`, exactly (the discrete filter, not the analogue ideal)
static float pt_exact(float k, float hz, int order) { const float w = 2 * kPi * hz * DT; return powf(k / sqrtf(1 + (1 - k) * (1 - k) - 2 * (1 - k) * cosf(w)), (float)order); }
static float lpf_exact(int type, float cutoff, float hz) {
    if (type == LPF_BIQUAD) { const float r = hz / cutoff; (void)r; const float wa = tanf(kPi * hz * DT) / tanf(kPi * cutoff * DT); return 1.0f / sqrtf(1 + wa * wa * wa * wa); }       // Butterworth with the frequency axis warped
    if (type == LPF_PT2) return pt_exact(pt1_gain(cutoff * kPt2Correction, DT), hz, 2);
    if (type == LPF_PT3) return pt_exact(pt1_gain(cutoff * kPt3Correction, DT), hz, 3);
    return pt_exact(pt1_gain(cutoff, DT), hz, 1);
}

// ---------------------------------------------------------------------------------------------------------------------------------
static void t_filters() {
    printf("[filters] cutoffs are where they are said to be\n");
    { Pt1 a; a.set_cutoff(100, DT); Pt2 b; b.set_cutoff(100, DT); Pt3 c; c.set_cutoff(100, DT);
      const float g1 = gain_at([&](float x) { return a.apply(x); }, 100), g2 = gain_at([&](float x) { return b.apply(x); }, 100), g3 = gain_at([&](float x) { return c.apply(x); }, 100);
      CHECK(near(g1, lpf_exact(LPF_PT1, 100, 100), 0.005f) && near(g2, lpf_exact(LPF_PT2, 100, 100), 0.005f) && near(g3, lpf_exact(LPF_PT3, 100, 100), 0.005f) && g1 > 0.64f && g1 < 0.72f && g2 > 0.64f && g3 > 0.63f && g3 < 0.72f,
            "PT1 / PT2 / PT3 at 100 Hz: about -3 dB at 100 Hz for every order (%.3f %.3f %.3f; Betaflight's simple discretisation reads a little low, 0.68 instead of 0.71 at 4 kHz)", g1, g2, g3);
      a.reset(); b.reset(); c.reset();
      const float h1 = gain_at([&](float x) { return a.apply(x); }, 400), h2 = gain_at([&](float x) { return b.apply(x); }, 400), h3 = gain_at([&](float x) { return c.apply(x); }, 400);
      CHECK(h1 > 0.2f && h1 < 0.3f && h2 < 0.6f * h1 && h3 < 0.75f * h2, "two octaves above: a higher order cuts more (%.3f > %.3f > %.3f)", h1, h2, h3);
      a.reset(); CHECK(a.apply(7.5f) == 7.5f && near(a.apply(7.5f), 7.5f, 1e-6f), "a filter starts from its first input - no step from zero at power-on");
      Pt1 off; off.set_cutoff(0, DT); CHECK(off.apply(3) == 3 && off.apply(-9) == -9, "cutoff 0 = bypass");
      CHECK(near(pt1_gain(100, DT), 0.13575f, 1e-4f) && near(pt1_gain_from_delay(0.01f, 0.004f), 0.2857f, 1e-3f) && pt1_gain_from_delay(0, 0.004f) == 1.0f, "pt1 gain = w / (w + 1), w = 2 pi f dt; from a delay: dt / (dt + delay)"); }
    { SvfLowpass f; f.set(100, DT); const float g = gain_at([&](float x) { return f.apply(x); }, 100); f.reset(); const float g2 = gain_at([&](float x) { return f.apply(x); }, 200); f.reset(); const float g0 = gain_at([&](float x) { return f.apply(x); }, 10);
      CHECK(near(g, 0.7071f, 0.02f) && near(g2, 0.2425f, 0.02f) && near(g0, 1.0f, 0.01f), "second-order Butterworth (\"BIQUAD\"): -3 dB at the cutoff, 1 / sqrt(1 + 2^4) one octave above (%.3f %.3f %.3f)", g, g2, g0); }
    { SvfNotch n; const float q = notch_q(200, 150); n.set(200, DT, q);
      const float gc = gain_at([&](float x) { return n.apply(x); }, 200); n.reset(); const float ge = gain_at([&](float x) { return n.apply(x); }, 150); n.reset(); const float gf = gain_at([&](float x) { return n.apply(x); }, 40);
      CHECK(near(q, 200.0f * 150.0f / (200.0f * 200.0f - 150.0f * 150.0f), 1e-5f) && gc < 0.02f && near(ge, 0.7071f, 0.04f) && gf > 0.97f, "notch 200 Hz, cutoff 150: nothing left at 200 (%.3f), -3 dB at the cutoff (%.3f), untouched at 40 (%.3f)", gc, ge, gf); }
    { Lowpass l; l.configure(LPF_PT1, 0, DT); CHECK(!l.active() && l.apply(5) == 5 && l.cutoff_hz() == 0, "Lowpass: 0 Hz = off");
      bool ok = true; for (int t = 0; t < LPF_TYPE_COUNT; t++) { Lowpass f; f.configure((uint8_t)t, 120, DT); const float g = gain_at([&](float x) { return f.apply(x); }, 120); ok &= f.active() && near(g, lpf_exact(t, 120, 120), 0.005f) && g > 0.63f && g < 0.72f && f.cutoff_hz() == 120;
        f.reset(); const float g2 = gain_at([&](float x) { return f.apply(x); }, 360); ok &= near(g2, lpf_exact(t, 120, 360), 0.005f); }
      CHECK(ok && near(lpf_exact(LPF_BIQUAD, 120, 120), 0.7071f, 1e-4f), "every type (PT1, BIQUAD, PT2, PT3) is about -3 dB at its cutoff (the BIQUAD exactly), and does what its formula says an octave and a half above");
      Lowpass m; m.configure(LPF_PT2, 100, DT); m.set_cutoff(300); const float g = gain_at([&](float x) { return m.apply(x); }, 300); CHECK(near(g, lpf_exact(LPF_PT2, 300, 300), 0.005f) && m.cutoff_hz() == 300, "a running filter is moved (dynamic low-pass): now a 300 Hz PT2 (%.3f at 300 Hz)", g);
      Lowpass b; b.configure(LPF_BIQUAD, 1950, DT); CHECK(!b.active(), "a BIQUAD at 97 %% of Nyquist would be unstable: off");
      Lowpass u; u.configure(99, 100, DT); const float gu = gain_at([&](float x) { return u.apply(x); }, 100); CHECK(u.active() && near(gu, lpf_exact(LPF_PT1, 100, 100), 0.005f), "an unknown type is a PT1");
      l.set_cutoff(200); CHECK(!l.active(), "moving a filter that is off does not switch it on");
      bool first = true; for (int t = 0; t < LPF_TYPE_COUNT; t++) { Lowpass f; f.configure((uint8_t)t, 50, DT); first &= near(f.apply(7.5f), 7.5f, 1e-5f) && near(f.apply(7.5f), 7.5f, 1e-4f); f.reset(); first &= near(f.apply(-3.0f), -3.0f, 1e-5f); }
      CHECK(first, "every type starts from its first input, and again after a reset"); }
    { StaticNotch n; n.configure(0, 0, DT); CHECK(!n.active() && n.apply(4) == 4, "static notch: centre 0 = off"); n.configure(200, 250, DT); CHECK(!n.active(), "cutoff above the centre: off (not a notch)");
      n.configure(200, 0, DT); CHECK(!n.active(), "cutoff 0: off");
      n.configure(300, 200, DT); const float g = gain_at([&](float x) { return n.apply(x); }, 300); CHECK(n.active() && g < 0.02f, "centre 300 / cutoff 200: on (%.3f at 300)", g);
      n.configure(2500, 1000, DT); const float gn = gain_at([&](float x) { return n.apply(x); }, 1900); CHECK(n.active() && gn < 0.05f, "a centre above Nyquist is moved to 95 %% of it (1900 Hz at 4 kHz): %.3f there", gn);
      n.configure(2500, 1950, DT); CHECK(!n.active(), "... unless the cutoff is above that too: off"); }
    { RpmNotch n; n.set(200, DT, 5.0f, 1.0f); float worst = 0; double s = 0, c = 0; int m = 0;
      for (int i = 0; i < 4000; i++) { const float ph = 2 * kPi * 200 * i * DT; float v[3] = {sinf(ph), 2 * sinf(ph), -sinf(ph)}; n.apply(v); if (i > 2000) { worst = fmaxf(worst, fmaxf(fabsf(v[0]), fmaxf(fabsf(v[1]), fabsf(v[2])))); s += v[0] * sinf(ph); c += v[0] * cosf(ph); m++; } }
      CHECK(worst < 0.03f, "RPM notch, weight 1: the tone is gone on all three axes (%.4f left)", worst);
      RpmNotch h; h.set(200, DT, 5.0f, 0.5f); const float gh = gain_at([&](float x) { float v[3] = {x, 0, 0}; h.apply(v); return v[0]; }, 200);
      RpmNotch z; z.set(200, DT, 5.0f, 0.0f); const float gz = gain_at([&](float x) { float v[3] = {x, 0, 0}; z.apply(v); return v[0]; }, 200);
      CHECK(near(gh, 0.5f, 0.03f) && near(gz, 1.0f, 1e-4f), "weight 0.5 leaves half of it, weight 0 leaves it alone (%.3f, %.4f): a notch fades, it does not switch", gh, gz); }
    { MovingAverage a; a.init(3); const float y1 = a.apply(3), y2 = a.apply(6), y3 = a.apply(9), y4 = a.apply(12);
      CHECK(y1 == 3 && y2 == 4.5f && y3 == 6 && y4 == 9, "moving average over 3: mean of what there is, then of the last three (%.1f %.1f %.1f %.1f)", y1, y2, y3, y4);
      MovingAverage one; one.init(0); CHECK(one.size == 1 && one.apply(5) == 5 && one.apply(8) == 8, "size 0 / 1 = no averaging"); MovingAverage big; big.init(9); CHECK(big.size == 4, "at most 4 points"); }
    // ---- created on a steady signal, or switched off after it has run: nothing is disturbed, nothing is left behind
    { SvfNotch n; n.set(200, DT, 3.0f); CHECK(n.apply(100.0f) == 100.0f && near(n.apply(100.0f), 100.0f, 1e-4f), "a notch starts settled on its first input: a steady gyro value passes unchanged from the first sample");
      Lowpass l; l.configure(LPF_PT2, 100, DT); for (int i = 0; i < 50; i++) l.apply(1.0f); l.configure(LPF_PT2, 0, DT);
      CHECK(!l.active() && l.cutoff_hz() == 0 && l.apply(5.0f) == 5.0f && l.apply(-3.0f) == -3.0f, "a low-pass set to 0 Hz after it has run: off, reports 0 Hz, passes everything");
      StaticNotch s; s.configure(300, 200, DT); for (int i = 0; i < 50; i++) s.apply(1.0f); s.configure(0, 0, DT);
      CHECK(!s.active() && s.apply(1.0f) == 1.0f && s.apply(5.0f) == 5.0f, "a static notch set to 0 after it has run: off, passes everything"); }
}

// ---------------------------------------------------------------------------------------------------------------------------------
static void run_chain(GyroFilter& f, const float* rpm, float seconds, float sig_hz, float sig_amp, const float noise_hz[], const float noise_amp[], int nn, double& res, double& sig_out, int axis = 0) {
    const int n = (int)(FS * seconds); double e = 0, so = 0, si = 0; int m = 0;
    for (int i = 0; i < n; i++) {
        const float t = i * DT, sig = sig_amp * sinf(2 * kPi * sig_hz * t); float x = sig; for (int k = 0; k < nn; k++) x += noise_amp[k] * sinf(2 * kPi * noise_hz[k] * t + k);
        float v[3] = {0, 0, 0}; v[axis] = x; f.apply(v, rpm);
        if (i > n / 2) { so += v[axis] * sig; si += sig * sig; m++; }
        (void)e;
    }
    // residual noise = output minus the (scaled, in-phase part of the) signal: measured on a second pass with the gain known
    const float g = si > 0 ? (float)(so / si) : 0; sig_out = g;
    double r = 0, tot = 0; int mm = 0;
    for (int i = 0; i < n / 2; i++) {
        const float t = (n + i) * DT, sig = sig_amp * sinf(2 * kPi * sig_hz * t); float noise = 0; for (int k = 0; k < nn; k++) noise += noise_amp[k] * sinf(2 * kPi * noise_hz[k] * t + k);
        float v[3] = {0, 0, 0}; v[axis] = sig + noise; f.apply(v, rpm);
        if (i > n / 8) { const float sig_late = sig_amp * sinf(2 * kPi * sig_hz * (t - 0.0f)); (void)sig_late; r += (double)(v[axis]) * v[axis]; tot += (double)(sig + noise) * (sig + noise); mm++; }
    }
    res = tot > 0 ? r / tot : 0;
}

static void t_gyro_chain() {
    printf("[gyro filter] low-passes, RPM notches, static notches - in Betaflight's order\n");
    GyroFilterConfig def;
    CHECK(def.lpf1_dyn_min_hz == 250 && def.lpf1_dyn_max_hz == 500 && def.lpf2_hz == 500 && def.dyn.count == 3 && def.rpm.harmonics == 3 && def.rpm.q == 5.0f && def.rpm.min_hz == 100 && def.notch1_hz == 0, "defaults = Betaflight's");
    // ---- dynamic low-pass 1: the cutoff follows the throttle
    { CHECK(near(dyn_lpf_cutoff(0, 250, 500, 5), 250, 0.01f) && near(dyn_lpf_cutoff(1, 250, 500, 5), 500, 0.01f) && near(dyn_lpf_cutoff(0.5f, 250, 500, 5), 250 + 250 * (0.5f * 0.5f * 0.5f + 0.5f), 0.01f),
            "curve with expo 5: min at zero throttle, max at full, (max - min) x (t (1 - t) 0.5 + t) + min in between (%.1f at half)", dyn_lpf_cutoff(0.5f, 250, 500, 5));
      CHECK(near(dyn_lpf_cutoff(0.5f, 250, 500, 0), 0.5f * (1 - 0.25f / 3) * 1.5f * 500, 0.01f) && near(dyn_lpf_cutoff(0.1f, 250, 500, 0), 250, 0.01f) && near(dyn_lpf_cutoff(1, 250, 500, 0), 500, 0.01f),
            "expo 0 = the older curve: t (1 - t^2 / 3) x 1.5 x max, never below min");
      GyroFilter f; f.configure(def, FS, 4); CHECK(near(f.lpf1_cutoff_hz(), 250, 0.01f), "starts at the minimum"); f.set_throttle(1.0f); CHECK(near(f.lpf1_cutoff_hz(), 500, 0.01f), "full throttle: 500 Hz");
      GyroFilterConfig c = def; c.dyn.count = 0; c.rpm.harmonics = 0; c.lpf2_hz = 0; GyroFilter g; g.configure(c, FS, 4);
      const float lo = gain_at([&](float x) { float v[3] = {x, 0, 0}; g.apply(v, nullptr); return v[0]; }, 250); g.set_throttle(1.0f); const float hi = gain_at([&](float x) { float v[3] = {x, 0, 0}; g.apply(v, nullptr); return v[0]; }, 250);
      CHECK(near(lo, lpf_exact(LPF_PT1, 250, 250), 0.005f) && near(hi, lpf_exact(LPF_PT1, 500, 250), 0.005f) && hi > lo + 0.15f, "and the filter really moves: a 250 Hz tone passes at %.2f with the throttle down, %.2f with it up", lo, hi);
      { GyroFilterConfig d2 = c; d2.lpf1_dyn_min_hz = 200; d2.lpf1_dyn_max_hz = 400; d2.lpf1_static_hz = 90; GyroFilter gd; gd.configure(d2, FS, 4);
        const float g200 = gain_at([&](float x) { float v[3] = {x, 0, 0}; gd.apply(v, nullptr); return v[0]; }, 200); CHECK(near(gd.lpf1_cutoff_hz(), 200, 0.01f) && near(g200, lpf_exact(LPF_PT1, 200, 200), 0.005f), "dynamic on: it starts at the dynamic minimum (200), not at the static cutoff (90)"); }
      c.lpf1_dyn_min_hz = 0; c.lpf1_static_hz = 120; GyroFilter s; s.configure(c, FS, 4); s.set_throttle(1.0f); const float gs = gain_at([&](float x) { float v[3] = {x, 0, 0}; s.apply(v, nullptr); return v[0]; }, 120);
      CHECK(near(s.lpf1_cutoff_hz(), 120, 0.01f) && near(gs, lpf_exact(LPF_PT1, 120, 120), 0.005f), "dynamic off (min 0): the static cutoff, whatever the throttle");
      c.lpf1_static_hz = 0; GyroFilter o; o.configure(c, FS, 4); float v[3] = {1, 2, 3}; o.apply(v, nullptr); CHECK(v[0] == 1 && v[1] == 2 && v[2] == 3 && o.lpf1_cutoff_hz() == 0, "everything off: the gyro passes untouched"); }
    // ---- RPM filter
    { GyroFilterConfig c = def; c.dyn.count = 0; c.lpf1_dyn_min_hz = 0; c.lpf1_static_hz = 0; c.lpf2_hz = 0;                      // the RPM notches alone
      GyroFilter f; f.configure(c, FS, 4); float rpm[kMaxMotors] = {12000, 12000, 12000, 12000};
      const float nh[3] = {200, 400, 600}, na[3] = {50, 30, 20}; double res, sig; run_chain(f, rpm, 2.0f, 20, 30, nh, na, 3, res, sig);
      const double want = 30.0 * 30.0 / (30.0 * 30.0 + 50.0 * 50.0 + 30.0 * 30.0 + 20.0 * 20.0);       // only the signal's power is left
      CHECK(near(sig, 1.0f, 0.03f) && res < want * 1.05 && res > want * 0.9, "motors at 12000 rpm = 200 Hz: the tone and its 2nd and 3rd harmonic are removed, the 20 Hz signal stays (gain %.3f, power left %.3f, the signal alone is %.3f)", sig, res, want);
      CHECK(near(f.rpm_filter().notch_hz(0, 0), 200, 2) && near(f.rpm_filter().notch_hz(2, 1), 400, 4) && near(f.rpm_filter().notch_hz(3, 2), 600, 6) && near(f.rpm_filter().notch_weight(1, 0), 1.0f, 0.01f), "notch frequencies = harmonic x rpm / 60");
      // every motor at its own speed
      GyroFilter g; g.configure(c, FS, 4); float r2[kMaxMotors] = {9000, 10500, 12000, 13800}; const float n2[4] = {150, 175, 200, 230}, a2[4] = {40, 40, 40, 40}; run_chain(g, r2, 2.0f, 20, 30, n2, a2, 4, res, sig);
      CHECK(res < 0.18 && near(sig, 1.0f, 0.05f), "four motors at four speeds: four tones removed (power left %.3f; the signal alone is %.3f)", res, 900.0 / (900.0 + 4 * 1600.0));
      // one harmonic switched off by its weight
      GyroFilterConfig w = c; w.rpm.weights[1] = 0; GyroFilter h; h.configure(w, FS, 4); const float gw = gain_at([&](float x) { float v[3] = {x, 0, 0}; h.apply(v, rpm); return v[0]; }, 400);
      GyroFilter h3; h3.configure(c, FS, 4); const float g3 = gain_at([&](float x) { float v[3] = {x, 0, 0}; h3.apply(v, rpm); return v[0]; }, 400);
      CHECK(gw > 0.8f && g3 < 0.05f, "weight of harmonic 2 = 0: the 400 Hz tone passes (%.2f - the neighbours at 200 and 600 take a little), with the default weights it does not (%.3f)", gw, g3);
      GyroFilterConfig half = c; half.rpm.harmonics = 1; half.rpm.weights[0] = 50; GyroFilter hh; hh.configure(half, FS, 1); float one_motor[kMaxMotors] = {12000};
      const float g50 = gain_at([&](float x) { float v[3] = {x, 0, 0}; hh.apply(v, one_motor); return v[0]; }, 200); CHECK(near(g50, 0.5f, 0.04f), "weight 50 (one motor): half of the tone is left (%.3f)", g50);
      GyroFilterConfig one = c; one.rpm.harmonics = 1; GyroFilter h1; h1.configure(one, FS, 4); const float g1 = gain_at([&](float x) { float v[3] = {x, 0, 0}; h1.apply(v, rpm); return v[0]; }, 400);
      CHECK(g1 > 0.93f, "1 harmonic: only the fundamental is filtered (400 Hz passes at %.2f)", g1);
      // no telemetry: the notches fade out instead of sitting somewhere
      GyroFilter nt; nt.configure(c, FS, 4); for (int i = 0; i < 4000; i++) { float v[3] = {1, 1, 1}; nt.apply(v, rpm); } for (int i = 0; i < 4000; i++) { float v[3] = {1, 1, 1}; nt.apply(v, nullptr); }
      const float gn = gain_at([&](float x) { float v[3] = {x, 0, 0}; nt.apply(v, nullptr); return v[0]; }, 100);
      CHECK(nt.rpm_filter().notch_weight(0, 0) < 0.01f && near(nt.rpm_filter().notch_hz(0, 0), 100, 0.5f) && gn > 0.99f, "rpm telemetry lost: the notch runs down to the minimum frequency and fades to nothing there (weight %.3f, a 100 Hz tone passes at %.3f)", nt.rpm_filter().notch_weight(0, 0), gn);
      // fade range: 110 Hz with min 100 and a fade range of 50 -> weight 0.2
      GyroFilter fd; fd.configure(c, FS, 4); float slow[kMaxMotors] = {6600, 6600, 6600, 6600}; for (int i = 0; i < 8000; i++) { float v[3] = {0, 0, 0}; fd.apply(v, slow); }
      CHECK(near(fd.rpm_filter().notch_weight(0, 0), 0.2f, 0.02f) && near(fd.rpm_filter().notch_weight(0, 1), 1.0f, 0.01f), "110 Hz, 10 above the minimum of 100 with a fade range of 50: weight 0.2 (%.3f); its harmonic at 220 is at full weight", fd.rpm_filter().notch_weight(0, 0));
      GyroFilter off; GyroFilterConfig oc = c; oc.rpm.harmonics = 0; off.configure(oc, FS, 4); float v[3] = {5, 6, 7}; off.apply(v, rpm); CHECK(!off.rpm_filter().active() && v[0] == 5 && v[2] == 7, "0 harmonics = RPM filter off");
      GyroFilter six; six.configure(c, FS, 6); float r6[kMaxMotors] = {9000, 9000, 9000, 12000, 12000, 12000}; const float n6[2] = {150, 200}, a6[2] = {40, 40}; run_chain(six, r6, 2.0f, 20, 30, n6, a6, 2, res, sig);
      CHECK(res < 0.25 && near(sig, 1.0f, 0.05f), "six motors (hex): both groups of tones removed (power left %.3f, the signal alone %.3f)", res, 900.0 / (900.0 + 2 * 1600.0)); }
    // ---- static notches and the order of the chain
    { GyroFilterConfig c = def; c.dyn.count = 0; c.rpm.harmonics = 0; c.lpf1_dyn_min_hz = 0; c.lpf1_static_hz = 0; c.lpf2_hz = 0; c.notch1_hz = 180; c.notch1_cutoff = 140; c.notch2_hz = 320; c.notch2_cutoff = 280;
      GyroFilter f; f.configure(c, FS, 4); const float g1 = gain_at([&](float x) { float v[3] = {0, x, 0}; f.apply(v, nullptr); return v[1]; }, 180), g2 = gain_at([&](float x) { float v[3] = {0, 0, x}; f.apply(v, nullptr); return v[2]; }, 320),
                                        g3 = gain_at([&](float x) { float v[3] = {x, 0, 0}; f.apply(v, nullptr); return v[0]; }, 60);
      CHECK(g1 < 0.03f && g2 < 0.03f && g3 > 0.95f, "two static notches (180 and 320 Hz) on every axis: %.3f, %.3f; 60 Hz passes at %.3f", g1, g2, g3); }
    // ---- low-pass 2 and low-pass 1 are two filters in a row
    { GyroFilterConfig c = def; c.dyn.count = 0; c.rpm.harmonics = 0; c.lpf1_dyn_min_hz = 0; c.lpf1_static_hz = 0; c.lpf2_hz = 200; GyroFilter f; f.configure(c, FS, 4);
      const float g2 = gain_at([&](float x) { float v[3] = {x, 0, 0}; f.apply(v, nullptr); return v[0]; }, 200);
      c.lpf1_static_hz = 100; c.lpf2_type = LPF_PT2; GyroFilter b; b.configure(c, FS, 4); const float gb = gain_at([&](float x) { float v[3] = {x, 0, 0}; b.apply(v, nullptr); return v[0]; }, 150);
      CHECK(near(g2, lpf_exact(LPF_PT1, 200, 200), 0.005f) && near(gb, lpf_exact(LPF_PT1, 100, 150) * lpf_exact(LPF_PT2, 200, 150), 0.005f), "low-pass 2 alone (PT1 200 Hz): %.3f at 200 Hz; with low-pass 1 (PT1 100) behind a PT2 200: the product of the two at 150 Hz (%.3f)", g2, gb);
      GyroFilterConfig q = def; q.lpf1_type = LPF_BIQUAD; q.lpf1_dyn_min_hz = 0; q.lpf1_static_hz = 1950; GyroFilter o; o.configure(q, FS, 4); CHECK(o.lpf1_cutoff_hz() == 0, "a low-pass 1 that cannot run (BIQUAD at 1950 Hz of a 2000 Hz Nyquist) is reported as off, not as 1950 Hz"); }
    // ---- the RPM notches follow the motor speed through a low-pass (rpm telemetry is noisy and arrives in steps)
    { GyroFilterConfig c = def; c.dyn.count = 0; c.rpm.harmonics = 1; GyroFilter f; f.configure(c, FS, 1); float r[kMaxMotors] = {12000};
      for (int i = 0; i < 400; i++) { float v[3] = {0, 0, 0}; f.apply(v, r); }
      r[0] = 18000; for (int i = 0; i < 8; i++) { float v[3] = {0, 0, 0}; f.apply(v, r); } const float mid = f.rpm_filter().notch_hz(0, 0);
      for (int i = 0; i < 80; i++) { float v[3] = {0, 0, 0}; f.apply(v, r); }
      const float want = 300 - 100 * powf(1 - pt1_gain(150, DT), 8);
      CHECK(near(mid, want, 1.0f) && near(f.rpm_filter().notch_hz(0, 0), 300, 0.5f), "the motor jumps from 12000 to 18000 rpm: 2 ms later the notch is at %.0f Hz (a 150 Hz low-pass: %.0f), then at 300", mid, want); }
}

// ---------------------------------------------------------------------------------------------------------------------------------
static void t_dyn_notch() {
    printf("[dynamic notch] finds the peaks of the noise and follows them\n");
    DynNotchConfig c;
    auto feed = [&](DynNotch& d, float seconds, float hz0, float hz1, float hz_pitch, double* in_pow = nullptr, double* out_pow = nullptr, float t0 = 0) {
        const int n = (int)(FS * seconds); double pi = 0, po = 0; double ph = 0;
        for (int i = 0; i < n; i++) {
            const float t = t0 + i * DT, hz = hz0 + (hz1 - hz0) * (float)i / (float)n; ph += 2 * kPi * hz * DT;
            const float x[3] = {20 * sinf((float)ph) + gauss(1.0f), hz_pitch > 0 ? 20 * sinf(2 * kPi * hz_pitch * t) + gauss(1.0f) : gauss(1.0f), gauss(1.0f)};
            float y0 = 0; for (int a = 0; a < 3; a++) { d.push(a, x[a]); const float y = d.apply(a, x[a]); if (a == 0) y0 = y; }
            d.update();
            if (i > n * 3 / 4) { pi += x[0] * x[0]; po += y0 * y0; }
        }
        if (in_pow) *in_pow = pi;
        if (out_pow) *out_pow = po;
    };
    auto nearest = [](const DynNotch& d, int axis, float hz) { float best = 1e9f; for (int p = 0; p < d.count(); p++) best = fminf(best, fabsf(d.center_hz(axis, p) - hz)); return best; };
    { DynNotch d; d.configure(c, FS); CHECK(d.count() == 3 && near(d.center_hz(0, 0), 100 + 500.0f / 6, 0.5f) && near(d.center_hz(2, 2), 100 + 500.0f * 5 / 6, 0.5f), "three notches per axis, spread over 100 .. 600 Hz to start with (%.0f %.0f %.0f)", d.center_hz(0, 0), d.center_hz(0, 1), d.center_hz(0, 2));
      double pi, po; feed(d, 1.5f, 250, 250, 430, &pi, &po);
      CHECK(nearest(d, 0, 250) < 8 && po < 0.08 * pi, "a 250 Hz resonance on roll: a notch sits on it within a second (%.1f Hz off), %.1f %% of the power is left", nearest(d, 0, 250), 100 * po / pi);
      CHECK(nearest(d, 1, 430) < 8 && nearest(d, 1, 250) > 50, "pitch has its own notches: on its 430 Hz, not on roll's 250 (%.1f Hz off)", nearest(d, 1, 430));
      feed(d, 1.0f, 250, 340, 430, nullptr, nullptr, 1.5f); feed(d, 0.5f, 340, 340, 430, &pi, &po, 2.5f);
      CHECK(nearest(d, 0, 340) < 10 && po < 0.10 * pi, "the resonance moves to 340 Hz in a second (throttle up): the notch follows (%.1f Hz off, %.1f %% left)", nearest(d, 0, 340), 100 * po / pi); }
    { DynNotchConfig z = c; z.count = 0; DynNotch d; d.configure(z, FS); CHECK(d.count() == 0 && d.apply(0, 5.0f) == 5.0f, "count 0 = off"); d.update();
      DynNotch slow; slow.configure(c, 1000.0f); CHECK(slow.count() == 0, "a 1 kHz loop is too slow for it: off (needs 2 kHz)");
      DynNotchConfig m = c; m.count = 20; DynNotch big; big.configure(m, FS); CHECK(big.count() == kDynNotchMax, "at most 7 per axis");
      DynNotchConfig lim = c; lim.min_hz = 150; lim.max_hz = 400; DynNotch d2; d2.configure(lim, FS); feed(d2, 1.5f, 520, 520, 0);
      bool inside = true; for (int a = 0; a < 3; a++) for (int p = 0; p < d2.count(); p++) inside &= d2.center_hz(a, p) >= 150 - 0.01f && d2.center_hz(a, p) <= 400 + 0.01f;
      CHECK(inside, "a notch never leaves the min .. max range (a 520 Hz tone with max 400)"); }
    { GyroFilterConfig g; g.rpm.harmonics = 0; GyroFilter f; f.configure(g, FS, 4); double pi = 0, po = 0;                         // inside the whole chain, on pitch
      for (int i = 0; i < 8000; i++) { const float t = i * DT; float v[3] = {gauss(1), 25 * sinf(2 * kPi * 310 * t) + gauss(1), gauss(1)}; const float in = v[1]; f.apply(v, nullptr); if (i > 6000) { pi += in * in; po += v[1] * v[1]; } }
      CHECK(po < 0.05 * pi && f.dyn_notch().count() == 3, "in the gyro chain: a 310 Hz frame resonance on pitch is taken out (%.1f %% of the power left)", 100 * po / pi); }
    // ---- the details, on clean tones (roll only; no noise, so every number below is the algorithm's own)
    auto tone = [](DynNotch& d, int cycles, float a1, float hz1, float a2 = 0, float hz2 = 0, int i0 = 0, double* in_pow = nullptr, double* out_pow = nullptr) {
        double pi = 0, po = 0;
        for (int i = 0; i < cycles; i++) { const float t = (i0 + i) * DT, x = a1 * sinf(2 * kPi * hz1 * t) + a2 * sinf(2 * kPi * hz2 * t); float y = 0;
            for (int a = 0; a < 3; a++) { d.push(a, a == 0 ? x : 0.0f); const float r = d.apply(a, a == 0 ? x : 0.0f); if (a == 0) y = r; } d.update(); if (i >= cycles / 2) { pi += x * x; po += y * y; } }
        if (in_pow) *in_pow = pi;
        if (out_pow) *out_pow = po; };
    DynNotchConfig one = c; one.count = 1; DynNotchConfig two = c; two.count = 2;
    { float worst = 0; for (float hz : {245.4f, 254.6f, 319.4f}) { DynNotch d; d.configure(one, FS); tone(d, 4000, 20, hz); worst = fmaxf(worst, fabsf(d.center_hz(0, 0) - hz)); }
      CHECK(worst < 3.5f, "the bins are 18.5 Hz apart, a tone a quarter of a bin off its bin is still found within %.1f Hz: the peak is interpolated between the (windowed) bins", worst); }
    { DynNotch d; d.configure(one, FS); const float start = d.center_hz(0, 0); float first = 0;
      for (int i = 0; i < 160; i++) { tone(d, 1, 20, 200, 0, 0, i); if (first == 0 && d.center_hz(0, 0) != start) first = (start - d.center_hz(0, 0)) / (start - 200); }
      CHECK(near(start, 350, 0.01f) && first > 0.2f && first < 0.5f && fabsf(d.center_hz(0, 0) - 200) < 3.0f, "a notch does not jump onto a peak, it is steered there: the first step covers %.0f %% of the way (350 -> 200 Hz), a strong peak is reached within 40 ms (%.1f Hz off)", 100 * first, d.center_hz(0, 0) - 200); }
    { DynNotch d; d.configure(two, FS); double pi, po; tone(d, 6000, 10, 200, 20, 400, 0, &pi, &po);
      CHECK(fabsf(d.center_hz(0, 0) - 200) < 6 && fabsf(d.center_hz(0, 1) - 400) < 6 && po < 0.02 * pi, "two tones, the weaker one lower: one notch on each, and they are kept in order of frequency (%.0f, %.0f Hz; %.2f %% of the power left)", d.center_hz(0, 0), d.center_hz(0, 1), 100 * po / pi); }
    { DynNotch weak, clear; weak.configure(two, FS); clear.configure(two, FS); tone(weak, 6000, 20, 250, 2.2f, 430); tone(clear, 6000, 20, 250, 3.5f, 430);
      CHECK(fabsf(weak.center_hz(0, 1) - 430) > 50 && fabsf(clear.center_hz(0, 1) - 430) < 6 && fabsf(weak.center_hz(0, 0) - 250) < 3, "a second bump must stand out twice above what is left around the main peak: 430 Hz at 11 %% of the main tone's amplitude is not chased (notch at %.0f Hz), at 17.5 %% it is (%.0f Hz)", weak.center_hz(0, 1), clear.center_hz(0, 1)); }
    { DynNotch d; d.configure(c, FS); tone(d, 6000, 20, 55); CHECK(d.center_hz(0, 0) > 150 && d.center_hz(0, 1) > 150 && d.center_hz(0, 2) > 150, "a 55 Hz wobble (the craft's own motion, below the 100 Hz minimum) attracts no notch (%.0f %.0f %.0f Hz)", d.center_hz(0, 0), d.center_hz(0, 1), d.center_hz(0, 2));
      DynNotchConfig hi = c; hi.max_hz = 5000; DynNotch h; h.configure(hi, FS); CHECK(h.center_hz(0, h.count() - 1) < 2000, "a maximum above Nyquist is cut to it (highest notch at %.0f Hz of 2000)", h.center_hz(0, h.count() - 1)); }
    { DynNotchConfig qc = one; qc.q = 3.0f; DynNotch d; d.configure(qc, FS); SvfNotch ref; ref.set(350, DT, 3.0f);
      const float g = gain_at([&](float x) { return d.apply(0, x); }, 300), want = gain_at([&](float x) { return ref.apply(x); }, 300);
      tone(d, 3000, 20, 250); SvfNotch ref2; ref2.set(d.center_hz(0, 0), DT, 3.0f); const float g2 = gain_at([&](float x) { return d.apply(0, x); }, 220), want2 = gain_at([&](float x) { return ref2.apply(x); }, 220);
      CHECK(near(g, want, 0.01f) && g > 0.5f && near(g2, want2, 0.01f) && g2 > 0.4f, "Q 3: a notch at 350 Hz leaves %.2f of a 300 Hz tone; after it has moved to 250 Hz, %.2f of a 220 Hz one (a narrow notch: little delay)", g, g2); }
}

// ---------------------------------------------------------------------------------------------------------------------------------
// A PID with nothing but the term under test: no D-term filters, no relax, no anti-gravity, no TPA, unless a test turns it on.
static PidConfig plain() {
    PidConfig c; c.iterm_relax = RELAX_OFF; c.anti_gravity = false; c.tpa_rate = 0; c.tpa_low_rate = 0; c.dterm_lpf1_static_hz = 0; c.dterm_lpf1_dyn_min_hz = 0; c.dterm_lpf2_hz = 0; c.yaw_lowpass_hz = 0;
    for (int a = 0; a < 3; a++) c.d_max[a] = c.d[a];
    return c;
}
static void t_pid() {
    printf("[pid] Betaflight's units and terms\n");
    { PidConfig d; CHECK(d.p[0] == 45 && d.p[1] == 47 && d.p[2] == 45 && d.i[0] == 80 && d.i[1] == 84 && d.d[0] == 30 && d.d[1] == 34 && d.d[2] == 0 && d.d_max[0] == 40 && d.d_max[1] == 46 && d.f[0] == 120 && d.f[1] == 125 && d.f[2] == 120, "default gains = Betaflight 4.5's");
      CHECK(d.iterm_relax == RELAX_RP && d.iterm_relax_type == RELAX_SETPOINT && d.iterm_relax_cutoff == 15 && d.iterm_windup == 80 && d.anti_gravity && d.anti_gravity_gain == 80 && d.anti_gravity_cutoff_hz == 5 && d.anti_gravity_p_gain == 100 &&
            d.d_max_gain == 0 && d.d_max_advance == 35 && d.tpa_mode == TPA_D && d.tpa_rate == 65 && near(d.tpa_breakpoint, 0.35f, 1e-6f) && d.tpa_low_rate == 20 && near(d.tpa_low_breakpoint, 0.05f, 1e-6f) && d.pidsum_limit == 500 && d.pidsum_limit_yaw == 400 &&
            d.dterm_lpf1_dyn_min_hz == 75 && d.dterm_lpf1_dyn_max_hz == 150 && d.dterm_lpf2_hz == 150 && d.yaw_lowpass_hz == 100, "and its options"); }
    // ---- P
    { PidController pc; pc.configure(plain(), FS); PidInput in; in.setpoint[0] = 100; in.setpoint[1] = -50; in.setpoint[2] = 20; in.i_enabled = false; float u[3]; pc.update_throttle(0.3f, nullptr); pc.update(in, u);
      CHECK(near(pc.p_term[0], 0.032029f * 45 * 100, 0.01f) && near(pc.p_term[1], -0.032029f * 47 * 50, 0.01f) && near(pc.p_term[2], 0.032029f * 45 * 20, 0.01f), "P = 0.032029 x p x error: 100 dps of roll error with p 45 = %.2f", pc.p_term[0]);
      CHECK(near(u[0], pc.p_term[0] / 1000 + pc.f_term[0] / 1000, 1e-5f) && near(u[0], 0.14413f, 1e-4f), "the mixer gets the sum / 1000: %.4f of the motor range", u[0]);
      in.setpoint[0] = 0; in.gyro[0] = 100; pc.update(in, u); CHECK(pc.p_term[0] < -100, "error = setpoint - gyro: turning right uncommanded gives a correction to the left"); }
    // ---- I
    { PidController pc; pc.configure(plain(), FS); PidInput in; in.setpoint[0] = 100; in.setpoint[2] = 100; float u[3]; pc.update_throttle(0.3f, nullptr); for (int i = 0; i < 40; i++) pc.update(in, u);
      CHECK(near(pc.i_term[0], 0.244381f * 80 * 100 * 0.01f, 0.01f) && near(pc.i_term[2], 2.5f * 0.244381f * 80 * 100 * 0.01f, 0.03f), "I grows by 0.244381 x i x error per second: 100 dps for 10 ms = %.2f; yaw's gain is x 2.5 (%.2f)", pc.i_term[0], pc.i_term[2]);
      for (int i = 0; i < 8000; i++) { pc.update(in, u); }
      CHECK(near(pc.i_term[0], 400, 0.01f) && near(pc.i_term[2], 320, 0.01f), "and stops at iterm_windup %% of the PID sum limit: 80 %% of 500 / of 400 (%.0f, %.0f)", pc.i_term[0], pc.i_term[2]);
      in.i_enabled = false; pc.update(in, u); CHECK(pc.i_term[0] == 0 && pc.i_term[2] == 0, "I not enabled (on the ground, throttle down): held at zero");
      PidConfig w = plain(); w.iterm_windup = 30; PidController p2; p2.configure(w, FS); in.i_enabled = true; for (int i = 0; i < 8000; i++) p2.update(in, u); CHECK(near(p2.i_term[0], 150, 0.01f), "iterm_windup 30: limit 150"); }
    // ---- D on the measurement
    { PidController pc; pc.configure(plain(), FS); PidInput in; float u[3]; pc.update_throttle(0.3f, nullptr);
      for (int i = 0; i < 200; i++) { in.gyro[0] = 1000.0f * i * DT; in.gyro[1] = -500.0f * i * DT; pc.update(in, u); }
      CHECK(near(pc.d_term[0], -0.000529f * 30 * 1000, 0.05f) && near(pc.d_term[1], 0.000529f * 34 * 500, 0.05f) && pc.d_term[2] == 0, "D = -0.000529 x d x gyro speed: 1000 dps/s with d 30 = %.2f (it brakes); yaw has no D", pc.d_term[0]);
      PidController p2; p2.configure(plain(), FS); PidInput s; p2.update(s, u); s.setpoint[0] = 500; p2.update(s, u); CHECK(p2.d_term[0] == 0, "a stick step makes no D kick: D only looks at the gyro");
      PidController p3; p3.configure(plain(), FS); PidInput g; g.gyro[0] = 300; p3.update(g, u); CHECK(p3.d_term[0] == 0, "and the first sample after a reset is not a jump"); }
    // ---- F and the limits
    { PidController pc; pc.configure(plain(), FS); PidInput in; in.feedforward[0] = 1000; in.feedforward[1] = -2000; in.feedforward[2] = 500; float u[3]; pc.update_throttle(0.3f, nullptr); pc.update(in, u);
      CHECK(near(pc.f_term[0], 0.013754f * 1.20f * 1000, 0.01f) && near(pc.f_term[1], -0.013754f * 1.25f * 2000, 0.01f) && near(pc.f_term[2], 0.013754f * 1.20f * 500, 0.01f), "F = 0.013754 x f / 100 x setpoint speed: 1000 dps/s with f 120 = %.2f", pc.f_term[0]);
      in.setpoint[0] = 2000; in.setpoint[2] = -2000; pc.update(in, u); CHECK(near(u[0], 0.5f, 1e-6f) && near(u[2], -0.4f, 1e-6f), "the sum is limited to +-500 (roll, pitch) and +-400 (yaw): %.2f, %.2f of the motor range", u[0], u[2]); }
    // ---- yaw low-pass on P
    { PidConfig c = plain(); c.yaw_lowpass_hz = 100; PidController pc; pc.configure(c, FS); PidInput in; in.i_enabled = false; float u[3]; in.setpoint[0] = 100; in.setpoint[2] = 100; pc.update(PidInput(), u); pc.update(in, u);
      CHECK(near(pc.p_term[0], 144.13f, 0.01f) && pc.p_term[2] < 30, "yaw P is low-passed (100 Hz), roll P is not: after one cycle of a step %.1f against %.1f", pc.p_term[2], pc.p_term[0]);
      for (int i = 0; i < 400; i++) { pc.update(in, u); }
      CHECK(near(pc.p_term[2], 144.13f, 0.05f), "and arrives at the same value"); }
    // ---- D-term filters: the default chain lets slow gyro changes through and takes the noise out
    { PidConfig c; c.iterm_relax = RELAX_OFF; c.anti_gravity = false; c.tpa_rate = 0; c.tpa_low_rate = 0; for (int a = 0; a < 3; a++) c.d_max[a] = c.d[a];
      PidController pc; pc.configure(c, FS); PidInput in; float u[3]; pc.update_throttle(0.0f, nullptr); double noisy = 0, clean = 0; int n = 0;
      for (int i = 0; i < 4000; i++) { const float t = i * DT, slow = 100 * sinf(2 * kPi * 5 * t); in.gyro[0] = slow + 5 * sinf(2 * kPi * 600 * t); pc.update(in, u);
        const float ideal = -0.000529f * 30 * 100 * 2 * kPi * 5 * cosf(2 * kPi * 5 * t), raw_noise = 0.000529f * 30 * 5 * 2 * kPi * 600; if (i > 2000) { clean += (pc.d_term[0] - ideal) * (pc.d_term[0] - ideal); noisy += raw_noise * raw_noise * 0.5; n++; } }
      CHECK(clean < 0.02 * noisy && near(pc.dterm_lpf1_cutoff_hz(), 75, 0.01f), "D of a 5 Hz movement with 600 Hz noise on it: the noise an unfiltered D would show is cut to %.1f %% of its power", 100 * clean / noisy);
      pc.update_throttle(1.0f, nullptr); for (int i = 0; i < 30; i++) pc.update_throttle(1.0f, nullptr); CHECK(near(pc.dterm_lpf1_cutoff_hz(), 150, 0.01f), "dynamic D-term low-pass: 75 Hz at zero throttle, 150 at full");
      { PidConfig dc = plain(); dc.dterm_lpf1_dyn_min_hz = 75; dc.dterm_lpf1_dyn_max_hz = 150; dc.dterm_lpf1_static_hz = 20; PidController lo, hi; lo.configure(dc, FS); hi.configure(dc, FS); PidInput gi; double pl = 0, ph = 0;
        for (int i = 0; i < 40; i++) { lo.update_throttle(0.0f, nullptr); hi.update_throttle(1.0f, nullptr); }
        for (int i = 0; i < 4000; i++) { gi.gyro[0] = 10 * sinf(2 * kPi * 150 * i * DT); lo.update(gi, u); hi.update(gi, u); if (i > 2000) { pl += lo.d_term[0] * lo.d_term[0]; ph += hi.d_term[0] * hi.d_term[0]; } }
        const float want = lpf_exact(LPF_PT1, 150, 150) / lpf_exact(LPF_PT1, 75, 150);
        CHECK(near((float)sqrt(ph / pl), want, 0.03f), "and the filter itself moves: D of a 150 Hz wobble is %.2f times larger at full throttle than at zero (%.2f for a PT1 at 150 instead of 75 Hz)", sqrt(ph / pl), want); }
      PidConfig st = c; st.dterm_lpf1_dyn_min_hz = 0; st.dterm_lpf1_static_hz = 90; PidController ps; ps.configure(st, FS); for (int i = 0; i < 30; i++) ps.update_throttle(1.0f, nullptr); CHECK(near(ps.dterm_lpf1_cutoff_hz(), 90, 0.01f), "dynamic off: the static cutoff");
      PidConfig nt = plain(); nt.dterm_notch_hz = 260; nt.dterm_notch_cutoff = 160; PidController pn; pn.configure(nt, FS); PidController p0; p0.configure(plain(), FS); double a = 0, b = 0;
      for (int i = 0; i < 4000; i++) { in.gyro[0] = 10 * sinf(2 * kPi * 260 * i * DT); pn.update(in, u); p0.update(in, u); if (i > 2000) { a += pn.d_term[0] * pn.d_term[0]; b += p0.d_term[0] * p0.d_term[0]; } }
      CHECK(a < 0.01 * b, "D-term notch at 260 Hz: that tone is gone from D (%.2f %% left)", 100 * a / b); }
    // ---- each D-term low-pass on its own
    { auto d_rms = [](const PidConfig& cfg, float hz) { PidController pc; pc.configure(cfg, FS); PidInput gi; float u[3]; double s = 0; for (int i = 0; i < 4000; i++) { gi.gyro[0] = 10 * sinf(2 * kPi * hz * i * DT); pc.update(gi, u); if (i > 2000) s += pc.d_term[0] * pc.d_term[0]; } return (float)sqrt(s); };
      PidConfig l2 = plain(); l2.dterm_lpf2_hz = 100; PidConfig l1 = plain(); l1.dterm_lpf1_static_hz = 80; l1.dterm_lpf1_type = LPF_PT2; const float raw = d_rms(plain(), 100), a2 = d_rms(l2, 100) / raw, a1 = d_rms(l1, 100) / raw;
      CHECK(near(a2, lpf_exact(LPF_PT1, 100, 100), 0.01f) && near(a1, lpf_exact(LPF_PT2, 80, 100), 0.01f), "low-pass 2 alone (PT1 100 Hz) leaves %.3f of the D of a 100 Hz wobble, low-pass 1 alone (PT2 80 Hz, static) %.3f", a2, a1);
      PidConfig dc = plain(); dc.dterm_lpf1_dyn_min_hz = 75; dc.dterm_lpf1_dyn_max_hz = 150; dc.dterm_lpf1_static_hz = 20; PidController fresh; fresh.configure(dc, FS); CHECK(near(fresh.dterm_lpf1_cutoff_hz(), 75, 0.01f), "dynamic: from the first cycle at the dynamic minimum (75), not at the static cutoff (20)");
      PidConfig bq = plain(); bq.dterm_lpf1_type = LPF_BIQUAD; bq.dterm_lpf1_dyn_min_hz = 1950; bq.dterm_lpf1_dyn_max_hz = 1990; PidController off; off.configure(bq, FS); for (int i = 0; i < 40; i++) off.update_throttle(0.5f, nullptr);
      CHECK(off.dterm_lpf1_cutoff_hz() == 0, "a D-term low-pass that cannot run (BIQUAD at Nyquist) is reported as off"); }
    // ---- the dynamic low-passes are moved on a schedule: not more often than every 5 ms, and only for a 1 % throttle change
    { GyroFilterConfig gc; GyroFilter gf; gf.configure(gc, FS, 4); PidController pc; pc.configure(PidConfig(), FS);
      for (int i = 0; i < 40; i++) pc.update_throttle(0.0f, &gf);
      pc.update_throttle(0.504f, &gf); const float first = gf.lpf1_cutoff_hz(); for (int i = 0; i < 18; i++) pc.update_throttle(0.9f, &gf); const float held = gf.lpf1_cutoff_hz(); for (int i = 0; i < 4; i++) pc.update_throttle(0.9f, &gf);
      CHECK(near(first, dyn_lpf_cutoff(0.5f, 250, 500, 5), 0.01f) && held == first && near(gf.lpf1_cutoff_hz(), dyn_lpf_cutoff(0.9f, 250, 500, 5), 0.01f), "throttle to 50 %%: gyro low-pass 1 at %.0f Hz (the value for 0.50: steps of 1 %%); the next move waits 5 ms (%.0f, then %.0f)", first, held, gf.lpf1_cutoff_hz());
      const float before = gf.lpf1_cutoff_hz(); for (int i = 0; i < 100; i++) pc.update_throttle(0.903f, &gf); CHECK(gf.lpf1_cutoff_hz() == before, "a change below 1 %% moves nothing"); }
    // ---- TPA
    { PidConfig c = plain(); c.tpa_rate = 65; c.tpa_breakpoint = 0.35f; c.tpa_low_rate = 0; PidController pc; pc.configure(c, FS); PidInput in; float u[3];
      pc.update_throttle(0.2f, nullptr); const float below = pc.tpa_factor(); pc.update_throttle(0.35f, nullptr); const float at = pc.tpa_factor(); pc.update_throttle(0.675f, nullptr); const float mid = pc.tpa_factor(); pc.update_throttle(1.0f, nullptr); const float full = pc.tpa_factor();
      CHECK(below == 1.0f && at == 1.0f && near(mid, 1 - 0.325f, 1e-4f) && near(full, 0.35f, 1e-4f), "TPA 65 %% from 35 %% throttle: full gains up to the breakpoint, then a straight line down to 35 %% of them at full throttle (%.3f half-way)", mid);
      for (int i = 0; i < 200; i++) { in.gyro[0] = 1000.0f * i * DT; in.setpoint[0] = in.gyro[0] + 100; pc.update(in, u); }
      CHECK(near(pc.d_term[0], -15.87f * 0.35f, 0.05f) && near(pc.p_term[0], 144.13f, 0.02f), "mode D (the default): D is reduced, P is not (%.2f, %.2f)", pc.d_term[0], pc.p_term[0]);
      PidConfig pd = c; pd.tpa_mode = TPA_PD; PidController p2; p2.configure(pd, FS); p2.update_throttle(1.0f, nullptr); for (int i = 0; i < 200; i++) { in.gyro[0] = 1000.0f * i * DT; in.setpoint[0] = in.gyro[0] + 100; p2.update(in, u); }
      CHECK(near(p2.d_term[0], -15.87f * 0.35f, 0.05f) && near(p2.p_term[0], 144.13f * 0.35f, 0.02f), "mode PD: both");
      // the low-throttle part: more damping where the props have little grip, until the throttle first passes the low breakpoint
      PidConfig lo = plain(); lo.tpa_rate = 65; lo.tpa_low_rate = 20; lo.tpa_low_breakpoint = 0.05f; PidController p3; p3.configure(lo, FS);
      p3.update_throttle(0.0f, nullptr); const float f0 = p3.tpa_factor(); p3.update_throttle(0.025f, nullptr); const float f1 = p3.tpa_factor(); p3.update_throttle(0.2f, nullptr); const float f2 = p3.tpa_factor(); p3.update_throttle(0.0f, nullptr); const float f3 = p3.tpa_factor();
      CHECK(near(f0, 0.8f, 1e-4f) && near(f1, 0.9f, 1e-4f) && f2 == 1.0f && f3 == 1.0f, "low TPA 20 %% below 5 %% throttle: 0.80 at zero, 0.90 half-way - and gone for good once the throttle has passed it (%.2f %.2f %.2f %.2f)", f0, f1, f2, f3);
      PidConfig al = lo; al.tpa_low_always = true; PidController p4; p4.configure(al, FS); p4.update_throttle(0.2f, nullptr); p4.update_throttle(0.0f, nullptr); CHECK(near(p4.tpa_factor(), 0.8f, 1e-4f), "\"always\": it comes back whenever the throttle is that low");
      p3.reset(); p3.update_throttle(0.0f, nullptr); CHECK(near(p3.tpa_factor(), 0.8f, 1e-4f), "after a disarm it is armed again"); }
    // ---- I-term relax
    { auto wound = [](uint8_t relax, uint8_t type, int axis, bool angle = false, float cutoff = 15) { PidConfig c = plain(); c.iterm_relax = relax; c.iterm_relax_type = type; c.iterm_relax_cutoff = (uint8_t)cutoff; PidController pc; pc.configure(c, FS);
          PidInput in; in.angle_mode = angle; float u[3]; pc.update_throttle(0.3f, nullptr); for (int i = 0; i < 40; i++) pc.update(in, u);
          // a flip: the setpoint rises to 500 dps in 50 ms, the craft follows 30 ms behind
          for (int i = 0; i < 600; i++) { const float t = i * DT; in.setpoint[axis] = 500 * fminf(t / 0.05f, 1.0f); in.gyro[axis] = 500 * fminf(fmaxf(t - 0.03f, 0.0f) / 0.05f, 1.0f); pc.update(in, u); }
          return pc.i_term[axis]; };
      const float off = wound(RELAX_OFF, RELAX_SETPOINT, 0), sp = wound(RELAX_RP, RELAX_SETPOINT, 0), gy = wound(RELAX_RP, RELAX_GYRO, 0);
      CHECK(near(off, 0.244381f * 80 * 500 * 0.03f, 6.0f), "without relax, the 30 ms the craft lags behind a 500 dps stick move wind the I-term up by %.0f (ki x 500 dps x 0.03 s = %.0f): it would push on after the flip", off, 0.244381f * 80 * 500 * 0.03f);
      CHECK(sp < 0.1f * off && sp >= 0 && gy < 0.4f * off && gy > sp, "with relax it does not: %.1f (type setpoint), %.1f (type gyro - the gentler of the two)", sp, gy);
      const float yaw_rp = wound(RELAX_RP, RELAX_SETPOINT, 2), yaw_rpy = wound(RELAX_RPY, RELAX_SETPOINT, 2);
      CHECK(near(yaw_rp, fminf(2.5f * off, 320.0f), 20.0f) && yaw_rpy < 0.3f * yaw_rp, "axes RP: yaw is not relaxed (%.0f); axes RPY: it is (%.0f)", yaw_rp, yaw_rpy);
      const float hi_cut = wound(RELAX_RP, RELAX_SETPOINT, 0, false, 40); CHECK(hi_cut > sp, "a higher cutoff relaxes for a shorter time: more I (%.1f against %.1f)", hi_cut, sp);
      // a slow turn: below the threshold nothing is held back
      auto slow = [](uint8_t relax, bool angle) { PidConfig c = plain(); c.iterm_relax = relax; PidController pc; pc.configure(c, FS); PidInput in; in.angle_mode = angle; float u[3];
          for (int i = 0; i < 4000; i++) { const float t = i * DT; in.setpoint[0] = 300 * t; in.gyro[0] = 300 * t - 10; pc.update(in, u); } return pc.i_term[0]; };
      const float s_off = slow(RELAX_OFF, false), s_on = slow(RELAX_RP, false), s_angle = slow(RELAX_RP, true);
      const float hpf = 300.0f / (2 * kPi * 15);                         // a ramp of 300 dps/s runs 300 x tau ahead of its 15 Hz low-pass: 3.2 dps
      CHECK(near(s_off, 0.244381f * 80 * 10, 2.0f) && near(s_on, s_off * (1 - hpf / 40), 0.02f * s_off), "a setpoint that moves steadily (300 dps/s, 3.2 dps ahead of its low-pass) is integrated at 1 - 3.2 / 40 = 92 %% (%.0f of %.0f)", s_on, s_off);
      CHECK(near(s_angle, s_off * (1 - hpf / 8), 0.03f * s_off), "self-levelling modes relax five times earlier (threshold 8 dps instead of 40): %.0f = 60 %%", s_angle);
      // "increment only": a wound-up I-term may always shrink, even while the stick moves
      auto shrink = [](uint8_t relax) { PidConfig c = plain(); c.iterm_relax = relax; PidController pc; pc.configure(c, FS); PidInput in; float u[3];
          in.setpoint[0] = 50; for (int i = 0; i < 2000; i++) pc.update(in, u); const float before = pc.i_term[0];
          for (int i = 0; i < 200; i++) { in.setpoint[0] = 50 + 4000 * i * DT; in.gyro[0] = in.setpoint[0] + 100; pc.update(in, u); } return before - pc.i_term[0]; };
      const float d_rp = shrink(RELAX_RP), d_inc = shrink(RELAX_RP_INC);
      CHECK(d_inc > 5 * fmaxf(d_rp, 1.0f) && d_inc > 50, "RP_INC: while the stick moves the I-term cannot grow, but it can still come down (%.0f against %.0f with RP)", d_inc, d_rp); }
    { auto wound_yaw = [](uint8_t relax) { PidConfig c = plain(); c.iterm_relax = relax; PidController pc; pc.configure(c, FS); PidInput in; float u[3]; pc.update_throttle(0.3f, nullptr); for (int i = 0; i < 40; i++) pc.update(in, u);
          for (int i = 0; i < 600; i++) { const float t = i * DT; in.setpoint[2] = 500 * fminf(t / 0.05f, 1.0f); in.gyro[2] = 500 * fminf(fmaxf(t - 0.03f, 0.0f) / 0.05f, 1.0f); pc.update(in, u); } return pc.i_term[2]; };
      const float rp_inc = wound_yaw(RELAX_RP_INC), rpy_inc = wound_yaw(RELAX_RPY_INC); CHECK(rp_inc > 200 && rpy_inc < 0.3f * rp_inc, "RP_INC leaves yaw alone (%.0f), RPY_INC relaxes it too (%.0f)", rp_inc, rpy_inc); }
    // ---- anti-gravity
    { PidConfig c = plain(); c.anti_gravity = true; PidController pc; pc.configure(c, FS); PidInput in; in.setpoint[0] = 20; in.setpoint[1] = 300; in.setpoint[2] = 20; float u[3];
      PidController ref; ref.configure(plain(), FS);
      for (int i = 0; i < 400; i++) { pc.update_throttle(0.6f, nullptr); ref.update_throttle(0.6f, nullptr); pc.update(in, u); ref.update(in, u); }
      CHECK(pc.anti_gravity() < 1e-3f && near(pc.i_term[0], ref.i_term[0], 1e-3f) && near(pc.p_term[0], ref.p_term[0], 1e-3f), "steady throttle: anti-gravity does nothing");
      float peak = 0, p_peak = 0, p_peak_turning = 0; const float i0 = pc.i_term[0], r0 = ref.i_term[0], y0 = pc.i_term[2], ry0 = ref.i_term[2];
      for (int i = 0; i < 400; i++) { const float thr = fmaxf(0.6f - 0.5f * i / 80.0f, 0.1f);                // throttle chopped from 60 % to 10 % in 20 ms
        pc.update_throttle(thr, nullptr); ref.update_throttle(thr, nullptr); pc.update(in, u); ref.update(in, u); peak = fmaxf(peak, pc.anti_gravity());
        p_peak = fmaxf(p_peak, pc.p_term[0] / ref.p_term[0]); p_peak_turning = fmaxf(p_peak_turning, pc.p_term[1] / ref.p_term[1]); }
      const float di = pc.i_term[0] - i0, dr = ref.i_term[0] - r0;
      CHECK(peak > 300 && di > 1.5f * dr, "throttle chopped 60 %% -> 10 %% in 20 ms: the I-term works harder while it happens (boost up to %.0f; I grew %.2f instead of %.2f) - the nose does not drop", peak, di, dr);
      CHECK(p_peak > 1.5f && p_peak_turning < 1.0f + (p_peak - 1.0f) / 5.0f, "and P is boosted too (x %.2f) - less so on an axis that is being turned by the stick (x %.2f at 300 dps)", p_peak, p_peak_turning);
      CHECK(near(pc.i_term[2] - y0, ref.i_term[2] - ry0, 1e-2f), "yaw is left alone");
      for (int i = 0; i < 4000; i++) { pc.update_throttle(0.1f, nullptr); } CHECK(pc.anti_gravity() < 0.5f, "it is over a second later (%.2f)", pc.anti_gravity());
      // the same chop commanded by Altitude Hold / the failsafe descent
      { PidController ap; ap.configure(c, FS); for (int i = 0; i < 400; i++) { ap.update_throttle(0.6f, nullptr, false); } float pk = 0;
        for (int i = 0; i < 400; i++) { ap.update_throttle(fmaxf(0.6f - 0.5f * i / 80.0f, 0.1f), nullptr, false); pk = fmaxf(pk, ap.anti_gravity()); }
        const float tpa_auto = ap.tpa_factor(); ap.update_throttle(0.1f, nullptr, true); const float handover = ap.anti_gravity();
        CHECK(pk == 0.0f && handover == 0.0f, "the same chop coming from Altitude Hold: no anti-gravity (%.1f) - it answers a hand on the stick, not a controller working against barometer noise; and nothing kicks when the pilot takes over at that throttle (%.1f)", pk, handover);
        PidConfig t = plain(); t.tpa_rate = 65; PidController a, b; a.configure(t, FS); b.configure(t, FS); a.update_throttle(0.8f, nullptr, false); b.update_throttle(0.8f, nullptr, true);
        CHECK(a.tpa_factor() == b.tpa_factor() && a.tpa_factor() < 0.6f, "TPA follows the throttle that is flown, whoever commands it (x %.2f at 80 %%)", a.tpa_factor()); (void)tpa_auto;
        ap.update_throttle(0.5f, nullptr, true); for (int i = 0; i < 40; i++) { ap.update_throttle(0.5f, nullptr, true); pk = fmaxf(pk, ap.anti_gravity()); } CHECK(pk > 10.0f, "the pilot's own throttle jump right after: boosted as always (%.0f)", pk); }
      // going up counts less than going down, and less at high throttle
      auto boost = [](float from, float to) { PidConfig c2 = plain(); c2.anti_gravity = true; PidController p; p.configure(c2, FS); for (int i = 0; i < 400; i++) p.update_throttle(from, nullptr); float pk = 0;
          for (int i = 0; i < 400; i++) { p.update_throttle(from + (to - from) * fminf(i / 80.0f, 1.0f), nullptr); pk = fmaxf(pk, p.anti_gravity()); } return pk; };
      const float down = boost(0.6f, 0.1f), up = boost(0.1f, 0.6f), high = boost(1.0f, 0.5f);
      CHECK(up < 0.6f * down && high < 0.5f * down && up > 0, "a throttle cut counts more than a punch, and little happens near full throttle (down %.0f, up %.0f, 100 -> 50 %%: %.0f)", down, up, high);
      PidConfig g2 = c; g2.anti_gravity_gain = 160; PidController p2; p2.configure(g2, FS); PidController p1; p1.configure(c, FS); float a = 0, b = 0;
      for (int i = 0; i < 400; i++) { p1.update_throttle(0.6f, nullptr); p2.update_throttle(0.6f, nullptr); } for (int i = 0; i < 200; i++) { const float thr = fmaxf(0.6f - 0.5f * i / 80.0f, 0.1f); p1.update_throttle(thr, nullptr); p2.update_throttle(thr, nullptr); a = fmaxf(a, p1.anti_gravity()); b = fmaxf(b, p2.anti_gravity()); }
      CHECK(near(b, 2 * a, 0.01f * b), "twice the gain, twice the boost"); }
    { auto chop = [](uint8_t p_gain, uint8_t cutoff, float& p_roll, float& p_yaw, float& i_gain) { PidConfig c = plain(); c.anti_gravity = true; c.anti_gravity_p_gain = p_gain; c.anti_gravity_cutoff_hz = cutoff; PidController pc, ref; pc.configure(c, FS); ref.configure(plain(), FS);
          PidInput in; in.setpoint[0] = 20; in.setpoint[2] = 20; float u[3], pk = 0; p_roll = p_yaw = 0; for (int i = 0; i < 400; i++) { pc.update_throttle(0.6f, nullptr); ref.update_throttle(0.6f, nullptr); pc.update(in, u); ref.update(in, u); }
          const float i0 = pc.i_term[0], r0 = ref.i_term[0];
          for (int i = 0; i < 400; i++) { const float thr = fmaxf(0.6f - 0.5f * i / 80.0f, 0.1f); pc.update_throttle(thr, nullptr); ref.update_throttle(thr, nullptr); pc.update(in, u); ref.update(in, u);
            pk = fmaxf(pk, pc.anti_gravity()); p_roll = fmaxf(p_roll, pc.p_term[0] / ref.p_term[0]); p_yaw = fmaxf(p_yaw, fabsf(pc.p_term[2] / ref.p_term[2] - 1.0f)); }
          i_gain = (pc.i_term[0] - i0) / (ref.i_term[0] - r0); return pk; };
      float p100, p0, p200, py, ig, ig20, dummy; const float pk5 = chop(100, 5, p100, py, ig); chop(0, 5, p0, dummy, dummy); chop(200, 5, p200, dummy, dummy); const float pk20 = chop(100, 20, dummy, dummy, ig20);
      CHECK(p0 == 1.0f && near(p200 - 1.0f, 2 * (p100 - 1.0f), 0.02f * (p200 - 1.0f)) && py < 1e-6f, "the P boost has its own gain: 0 = none, 200 = twice that of 100 (x %.2f, x %.2f); yaw P is never boosted", p100, p200);
      CHECK(pk20 > 1.5f * pk5, "the cutoff decides how sharp the reaction is: 20 Hz peaks at %.0f where 5 Hz peaks at %.0f", pk20, pk5); }
    // ---- D max ("dynamic damping")
    { PidConfig c = plain(); c.d[0] = 30; c.d_max[0] = 40; c.d_max_gain = 37; c.d_max_advance = 0; PidController pc; pc.configure(c, FS); PidInput in; float u[3]; pc.update_throttle(0.3f, nullptr);
      for (int i = 0; i < 2000; i++) { in.gyro[0] = 20.0f * sinf(2 * kPi * 1.0f * i * DT); pc.update(in, u); }           // a gentle movement
      const float calm = pc.d_term[0], calm_want = -0.000529f * 30 * 20 * 2 * kPi * cosf(2 * kPi * 0.5f);
      CHECK(near(calm, calm_want, 0.02f * fabsf(calm_want) + 0.01f), "calm air: D is the base value (d = 30): %.3f (%.3f for a pure d of 30)", calm, calm_want);
      float worst = 0; for (int i = 0; i < 800; i++) { const float t = i * DT; in.gyro[0] = 150 * sinf(2 * kPi * 25 * t); pc.update(in, u); if (i > 400) { const float amp = 0.000529f * 30 * 150 * 2 * kPi * 25, base = -amp * cosf(2 * kPi * 25 * t); if (fabsf(base) > 0.95f * amp) worst = fmaxf(worst, pc.d_term[0] / base); } }   // (compared at the crests)
      CHECK(worst > 1.25f && worst < 1.3334f + 0.01f, "the craft is shaken (prop wash): D rises towards D max - x %.2f, at most 40 / 30", worst);
      PidConfig a = plain(); a.d[0] = 30; a.d_max[0] = 40; a.d_max_gain = 0; a.d_max_advance = 35; PidController pa; pa.configure(a, FS); PidInput s; float ratio = 0;
      for (int i = 0; i < 400; i++) { s.gyro[0] = 1000.0f * i * DT; s.feedforward[0] = 9000; pa.update(s, u); ratio = pa.d_term[0] / (-0.000529f * 30 * 1000); }
      CHECK(near(ratio, 1 + (40.0f / 30 - 1) * 9000 * 0.00008f * 35 / 35, 0.01f), "advance 35: a stick moving the setpoint at 9000 dps/s raises D by 9000 x 0.00008 = 72 %% of the way to D max (x %.2f) - before the craft has even reacted", ratio);
      for (int i = 0; i < 400; i++) { s.gyro[0] = 1000.0f * (400 + i) * DT; s.feedforward[0] = 30000; pa.update(s, u); ratio = pa.d_term[0] / (-0.000529f * 30 * 1000); } CHECK(near(ratio, 40.0f / 30, 0.005f), "a faster move: all the way, and no further (x %.3f)", ratio); s.feedforward[0] = 0;
      PidConfig n = plain(); n.d[0] = 30; n.d_max[0] = 30; n.d_max_gain = 100; n.d_max_advance = 200; PidController pn; pn.configure(n, FS); for (int i = 0; i < 400; i++) { s.gyro[0] = 1000.0f * i * DT; pn.update(s, u); }
      CHECK(near(pn.d_term[0], -15.87f, 0.05f), "D max = D: no boost, whatever the gains say");
      PidConfig z = plain(); z.d[0] = 30; z.d_max[0] = 40; z.d_max_gain = 0; z.d_max_advance = 0; PidController pz; pz.configure(z, FS); float wz = 0; PidInput q;
      for (int i = 0; i < 800; i++) { const float t = i * DT; q.gyro[0] = 150 * sinf(2 * kPi * 25 * t); pz.update(q, u); if (i > 400) { const float amp = 0.000529f * 30 * 150 * 2 * kPi * 25, base = -amp * cosf(2 * kPi * 25 * t); if (fabsf(base) > 0.95f * amp) wz = fmaxf(wz, pz.d_term[0] / base); } }
      CHECK(near(wz, 1.0f, 0.03f), "both gains 0: D max is never reached (x %.2f)", wz); }
    { PidConfig a = plain(); a.d[0] = 30; a.d_max[0] = 40; a.d_max_gain = 0; a.d_max_advance = 35; PidController pa; pa.configure(a, FS); PidInput s; float u[3], r4 = 0;
      for (int i = 0; i < 40; i++) { s.gyro[0] = 1000.0f * i * DT; pa.update(s, u); }
      for (int i = 40; i < 44; i++) { s.gyro[0] = 1000.0f * i * DT; s.feedforward[0] = 30000; pa.update(s, u); r4 = pa.d_term[0] / (-0.000529f * 30 * 1000); }
      CHECK(r4 > 1.0f && r4 < 1.05f, "the boost comes in smoothly (a 35 Hz low-pass), not as a step in D: x %.3f one millisecond after the stick starts to race", r4); }
    // ---- reset: nothing carries over a disarm
    { PidController pc; pc.configure(PidConfig(), FS); PidInput in; in.setpoint[0] = 200; in.gyro[0] = 50; in.feedforward[0] = 500; float u[3]; for (int i = 0; i < 2000; i++) { pc.update_throttle(0.5f, nullptr); pc.update(in, u); }
      pc.reset(); CHECK(pc.i_term[0] == 0 && pc.p_term[0] == 0 && pc.d_term[0] == 0 && pc.f_term[0] == 0 && pc.tpa_factor() == 1.0f && pc.anti_gravity() == 0, "reset clears every term"); }
    { PidController pc; pc.configure(plain(), FS); PidInput in; float u[3]; for (int i = 0; i < 100; i++) { in.gyro[0] = 0.5f * i; pc.update(in, u); }
      pc.reset(); in.gyro[0] = 300; pc.update(in, u); CHECK(pc.d_term[0] == 0, "and the gyro history: the first sample after a re-arm is no D spike (%.1f)", pc.d_term[0]); }
    // ---- throttle boost
    { ThrottleBoost b; b.configure(5, 15, FS); float out = 0; for (int i = 0; i < 4000; i++) out = b.apply(0.3f); CHECK(near(out, 0.3f, 1e-4f), "throttle boost: a steady throttle is passed on unchanged");
      const float jump = b.apply(0.5f); CHECK(near(jump, 0.5f + 0.5f * 0.2f * (1 - pt1_gain(15, DT)), 2e-3f), "a jump of 20 %% is passed on with half of it added on top (boost 5 = x 0.5 of what a 15 Hz low-pass has not let through yet): %.3f", jump);
      for (int i = 0; i < 4000; i++) { out = b.apply(0.5f); }
      CHECK(near(out, 0.5f, 1e-4f), "and the extra fades");
      { ThrottleBoost a; a.configure(5, 15, FS); for (int i = 0; i < 4000; i++) { a.apply(0.3f); }
        const float jump_auto = a.apply(0.5f, false); float worst = 0; for (int i = 0; i < 4000; i++) { worst = fmaxf(worst, fabsf(a.apply(0.5f + 0.02f * sinf(i * 0.3f), false) - (0.5f + 0.02f * sinf(i * 0.3f)))); }
        CHECK(jump_auto == 0.5f && worst == 0.0f, "a throttle that comes from Altitude Hold or the failsafe is passed on as it is (a jump: %.3f) - its ripple is not amplified into the motors", jump_auto);
        for (int i = 0; i < 4000; i++) { a.apply(0.6f, false); }
        const float takeover = a.apply(0.6f, true); CHECK(near(takeover, 0.6f, 1e-4f), "the pilot takes over where the autopilot left the throttle: no kick (%.4f; a low-pass that had not followed would add %.2f)", takeover, 0.5f * 0.3f); }
      ThrottleBoost z; z.configure(0, 15, FS); CHECK(z.apply(0.2f) == 0.2f && z.apply(0.9f) == 0.9f, "boost 0 = off");
      ThrottleBoost big; big.configure(100, 15, FS); big.apply(0.0f); CHECK(big.apply(1.0f) == 1.0f && big.apply(0.0f) == 0.0f, "the result stays inside 0 .. 1"); }
}

// ---------------------------------------------------------------------------------------------------------------------------------
// Acro trainer (Betaflight applyAcroTrainer): the setpoint it gives back
static void t_acro_trainer() {
    printf("[acro trainer] the angle limit in Acro\n");
    AcroTrainer t; bool ri = false;
    CHECK(t.cfg.angle_limit_deg == 20.0f && t.cfg.lookahead_s == 0.05f && t.cfg.gain == 7.5f, "defaults: Betaflight's (20 deg, 50 ms, gain 75)");
    CHECK(t.apply(ROLL, 300, 0, 0, ri) == 300 && !ri && t.state(ROLL) == 0, "level: the stick's rate goes through");
    float v = t.apply(ROLL, 100, 25, 0, ri); CHECK(near(v, -37.5f, 1e-4f) && ri && t.state(ROLL) == 1, "25 deg, stick still right: back to the limit, (20 - 25) x 7.5 = %.1f, I-term cleared", v);
    v = t.apply(ROLL, 100, 22, 0, ri); CHECK(near(v, -15.0f, 1e-4f) && !ri && t.state(ROLL) == 1, "held: (20 - 22) x 7.5, the I-term only cleared when it takes over");
    v = t.apply(ROLL, 100, 18, 0, ri); CHECK(near(v, 15.0f, 1e-4f), "below the limit while held: pushed up to it (%.1f)", v);
    v = t.apply(ROLL, -50, 22, 0, ri); CHECK(v == -50 && t.state(ROLL) == 0, "the stick reverses: let go");
    v = t.apply(PITCH, 400, 15, 500, ri); CHECK(near(v, -150.0f, 1e-3f) && ri && t.state(PITCH) == 0, "15 deg at 500 deg/s: 25 deg ahead in 50 ms -> 40 projected, (20 - 40) x 7.5 = %.1f", v);
    v = t.apply(PITCH, 400, 15, 250, ri); CHECK(near(v, -9.375f, 1e-3f), "at 250 deg/s the lookahead is half: 21.25 projected (%.3f)", v);
    v = t.apply(PITCH, 400, 15, 100, ri); CHECK(v == 400 && !ri, "at 100 deg/s: 16 projected, free");
    v = t.apply(ROLL, -100, -25, 0, ri); CHECK(near(v, 37.5f, 1e-4f) && t.state(ROLL) == -1, "the left side the same (%.1f)", v);
    v = t.apply(ROLL, 0, -25, 0, ri); CHECK(near(v, 37.5f, 1e-4f) && t.state(ROLL) == -1, "the stick centred is 'negative' (Betaflight's sign of 0): still held on the left");
    t.reset(); CHECK(t.state(ROLL) == 0, "reset (switched on): nothing held");
    v = t.apply(ROLL, 0, 25, 0, ri); CHECK(v == 0.0f && t.state(ROLL) == 0, "the stick centred at 25 deg right: not held, rate 0");
    t.cfg.gain = 25.5f; v = t.apply(ROLL, 100, 80, 0, ri); CHECK(v == -1000.0f, "the correction is limited to 1000 deg/s");
    t.reset(); t.cfg.gain = 7.5f; t.cfg.angle_limit_deg = 45; v = t.apply(ROLL, 100, 30, 0, ri); CHECK(v == 100, "limit 45: 30 deg is free");
    PidController pc; PidConfig c; pc.configure(c, 4000); pc.i_term[1] = 50; pc.reset_i(1); CHECK(pc.i_term[1] == 0, "reset_i clears one axis' I-term");
}

static void t_setpoint() {
    printf("[stick path] frame rate, RC smoothing, feedforward\n");
    // ---- receiver frame rate
    { RxRate r; r.reset(); bool upd = false; for (int i = 0; i < 40; i++) upd |= r.on_frame(0.004f);
      CHECK(upd && near(r.smoothed_hz(), 250, 2.5f) && near(r.current_hz(), 250, 0.01f), "a 250 Hz link is recognised (%.1f Hz)", r.smoothed_hz());
      r.on_frame(0.012f); CHECK(near(r.smoothed_hz(), 250, 2.5f), "one late frame (a dropped packet) does not change the rate");
      for (int i = 0; i < 4; i++) { r.on_frame(0.02f); }
      CHECK(near(r.smoothed_hz(), 50, 0.5f), "three frames in a row at the new interval do: the link switched to 50 Hz (%.1f)", r.smoothed_hz());
      r.on_frame(0.5f); CHECK(near(r.interval_s(), kRxIntervalMaxS, 1e-6f) && near(r.smoothed_hz(), 50, 0.5f), "a gap is not a rate: intervals outside 0.8 .. 65.5 ms are ignored (and clamped for the maths)");
      RxRate j; j.reset(150); bool any = false; for (int i = 0; i < 300; i++) any |= j.on_frame((i % 2) ? 0.006f : 0.007f); CHECK(near(j.smoothed_hz(), 154, 6.0f), "frames polled at 1 kHz arrive 6 / 7 / 6 / 7 ms apart: the smoothed rate is steady (%.1f Hz) - that is the one the speed maths use", j.smoothed_hz()); (void)any; }
    { RxRate r; r.reset(50); for (int i = 0; i < 6; i++) r.on_frame(0.5f); CHECK(near(r.smoothed_hz(), 50, 0.01f), "a link that only gets a frame through every half second is not a 15 Hz link: the rate stays (%.1f)", r.smoothed_hz());
      RxRate s; s.reset(150); int asked = 0; for (int i = 0; i < 30; i++) asked += s.on_frame(1.0f / 160) ? 1 : 0; CHECK(asked == 10 && near(s.smoothed_hz(), 160, 1.0f), "a steady link drifting from 150 to 160 Hz: the cutoffs are recomputed every third good frame (%d times in 30)", asked);
      RxRate a; a.reset(250); for (int i = 0; i < 8; i++) a.on_frame((i % 2) ? 0.002f : 0.010f); CHECK(near(a.smoothed_hz(), 250, 0.01f), "frames arriving alternately late and early (10 ms, 2 ms) are jitter, not a new rate: only three outliers on the SAME side switch it (%.0f)", a.smoothed_hz()); }
    // what a stick movement is divided by to get its speed
    { RxRate r; r.reset(250); for (int i = 0; i < 20; i++) { r.on_frame(0.004f); }
      CHECK(r.speed_hz() == r.smoothed_hz(), "a frame on time: the smoothed rate");
      r.on_frame(0.0045f); const bool jitter = r.speed_hz() == r.smoothed_hz(); r.on_frame(0.004f); r.on_frame(0.0059f);
      CHECK(jitter && r.speed_hz() == r.smoothed_hz(), "4.5 or 5.9 ms instead of 4 (the 1 ms the polling adds, a little more): still the smoothed rate");
      r.on_frame(0.004f); r.on_frame(0.0062f); const float late = r.speed_hz(); r.on_frame(0.004f); r.on_frame(0.008f);
      CHECK(near(late, 1.0f / 0.0062f, 0.1f) && near(r.speed_hz(), 125.0f, 0.01f), "from one and a half frame periods on a frame was lost on the way: the movement it carries took that long (6.2 ms -> %.0f Hz, 8 ms -> %.0f Hz)", late, r.speed_hz());
      r.on_frame(0.004f); CHECK(r.speed_hz() == r.smoothed_hz() && near(r.smoothed_hz(), 250, 4.0f), "the next one on time: as before (%.1f)", r.smoothed_hz());
      r.on_frame(0.0655f); const float edge = r.speed_hz(); r.on_frame(0.0656f); const float gap = r.speed_hz(); r.on_frame(0.004f);
      CHECK(near(edge, 1.0f / 0.0655f, 0.01f) && gap == 0.0f && r.speed_hz() == r.smoothed_hz(), "65.5 ms is the longest frame interval there is (%.1f Hz); after a longer gap a stick delta has no speed (0), and the frame after it is normal again", edge);
      r.on_frame(0.3f); r.reset(250); CHECK(r.speed_hz() == 250.0f, "a restart (re-arming) right after a gap starts clean"); }
    // ---- RC smoothing
    { SetpointGenerator g; FeedforwardConfig ff; RcSmoothingConfig sm; g.configure(ff, sm, FS); RateProfile rates; const float st0[3] = {0, 0, 0}, st1[3] = {1, 0, 0};
      for (int f = 0; f < 60; f++) { g.on_frame(st0, 0, rates); for (int k = 0; k < 16; k++) g.update(); }                        // 250 Hz
      CHECK(near(g.rx_rate().smoothed_hz(), 250, 3) && near(g.setpoint_cutoff_hz(), 250 * 1.5f / 4, 1.5f) && near(g.throttle_cutoff_hz(), 250 * 1.5f / 4, 1.5f), "cutoff = frame rate x 1.5 / (1 + factor / 10): 250 Hz and factor 30 -> %.1f Hz", g.setpoint_cutoff_hz());
      float prev = 0, max_step = 0; StickSetpoint o;
      for (int f = 0; f < 25; f++) { g.on_frame(st1, 0.8f, rates); for (int k = 0; k < 16; k++) { o = g.update(); max_step = fmaxf(max_step, o.rate[0] - prev); prev = o.rate[0]; } }
      CHECK(near(o.rate[0], 670, 1.0f) && near(o.stick[0], 1.0f, 0.002f) && near(o.throttle, 0.8f, 0.002f) && max_step < 67 && max_step > 10, "a stick step of 670 dps reaches the craft as a smooth curve: never more than %.0f dps per cycle, 670 dps and throttle 0.8 at the end", max_step);
      SetpointGenerator slow; slow.configure(ff, sm, FS); for (int f = 0; f < 40; f++) { slow.on_frame(st0, 0, rates); for (int k = 0; k < 80; k++) slow.update(); }      // 50 Hz
      CHECK(near(slow.setpoint_cutoff_hz(), 50 * 1.5f / 4, 0.5f), "a slow link is smoothed more (50 Hz -> %.2f Hz)", slow.setpoint_cutoff_hz());
      SetpointGenerator vslow; vslow.configure(ff, sm, FS); for (int f = 0; f < 20; f++) { vslow.on_frame(st0, 0, rates); for (int k = 0; k < 200; k++) vslow.update(); }  // 20 Hz
      CHECK(near(vslow.setpoint_cutoff_hz(), 15, 0.01f), "but never below 15 Hz");
      RcSmoothingConfig thr = sm; thr.auto_factor = 0; thr.auto_factor_throttle = 90; SetpointGenerator t2; t2.configure(ff, thr, FS); for (int f = 0; f < 60; f++) { t2.on_frame(st0, 0, rates); for (int k = 0; k < 16; k++) t2.update(); }
      CHECK(near(t2.setpoint_cutoff_hz(), 375, 4) && near(t2.throttle_cutoff_hz(), 37.5f, 0.5f), "sticks and throttle have their own factor (0 -> %.0f Hz, 90 -> %.1f Hz)", t2.setpoint_cutoff_hz(), t2.throttle_cutoff_hz());
      RcSmoothingConfig off = sm; off.enabled = false; SetpointGenerator raw; raw.configure(ff, off, FS); raw.on_frame(st1, 0.5f, rates); const StickSetpoint r = raw.update();
      CHECK(r.rate[0] == 670 && r.throttle == 0.5f && r.stick[0] == 1.0f, "smoothing off: the frame's values as they are");
      g.neutral(); for (int k = 0; k < 2000; k++) o = g.update(); CHECK(fabsf(o.rate[0]) < 0.01f && fabsf(o.feedforward[0]) < 0.01f && near(o.throttle, 0.8f, 0.002f), "neutral() (failsafe): the sticks go to centre, the throttle is left to the caller"); }
    // ---- feedforward
    { FeedforwardConfig c; CHECK(c.transition == 0 && c.averaging == 1 && c.smooth_factor == 65 && c.jitter_factor == 7 && c.boost == 15 && c.max_rate_limit == 90 && c.yaw_hold_gain == 15 && c.yaw_hold_time == 100, "defaults = Betaflight's");
      // a steady stick sweep: feedforward = the speed of the setpoint
      auto sweep = [](FeedforwardConfig cfg, int axis, float dps_per_s, float hz, float start_stick = -0.5f, int frames = 60) { FeedforwardAxis f; f.configure(cfg, axis); RxRate rx; rx.reset(hz); f.set_rx_rate(hz); RateProfile rp; rp.center_sens_dps[axis] = rp.max_rate_dps[axis] = 500;      // linear: 500 dps at full stick
          float out = 0; for (int i = 0; i < frames; i++) { const float sp = start_stick * 500 + dps_per_s * i / hz, stick = sp / 500; out = f.on_frame(stick * 500, sp, 500, fabsf(stick), rx); } return out; };
      FeedforwardConfig plain_ff; plain_ff.boost = 0; plain_ff.max_rate_limit = 0; plain_ff.averaging = 0; plain_ff.jitter_factor = 0; plain_ff.yaw_hold_gain = 0;
      const float v = sweep(plain_ff, ROLL, 600, 250); CHECK(near(v, 600, 3.0f), "the stick sweeps the setpoint at 600 dps per second: feedforward is 600 (%.1f)", v);
      const float v50 = sweep(plain_ff, ROLL, 600, 50, -0.5f, 40); CHECK(near(v50, 600, 6.0f), "the same on a 50 Hz link (%.1f): the speed does not depend on the frame rate", v50);
      const float vy = sweep(plain_ff, YAW, -300, 250); CHECK(near(vy, -300, 3.0f), "and on yaw (%.1f)", vy);
      // jitter reduction: a creeping stick (finger tremor) gets less
      FeedforwardConfig jit = plain_ff; jit.jitter_factor = 7; const float creep = sweep(jit, ROLL, 250, 250, 0.0f), fast = sweep(jit, ROLL, 2500, 250, -1.0f, 40);
      CHECK(near(creep, 250 * (1.0f + 1.0f) / 8.0f, 6.0f) && near(fast, 2500, 30), "jitter factor 7: a stick creeping 1 step per frame gets (1 + 1) / (1 + 7) of its feedforward (%.0f of 250), a real move all of it (%.0f)", creep, fast);
      // boost: the acceleration of the setpoint, at the start of a move
      FeedforwardConfig bo = plain_ff; bo.boost = 15; bo.smooth_factor = 0; auto first_frames = [](FeedforwardConfig cfg) { FeedforwardAxis f; f.configure(cfg, ROLL); RxRate rx; rx.reset(250); f.set_rx_rate(250); float pk = 0;
          for (int i = 0; i < 6; i++) { const float sp = i == 0 ? 0.0f : 4.0f * (i - 0), stick = sp / 500; pk = fmaxf(pk, f.on_frame(stick * 500, sp, 500, fabsf(stick), rx)); } return pk; };
      FeedforwardConfig nb = bo; nb.boost = 0; const float with = first_frames(bo), without = first_frames(nb);
      CHECK(near(without, 1000, 5) && near(with, 1000 + 1000 * 250 * 0.015f, 60), "boost 15: when the stick starts to move (0 -> 1000 dps/s within one frame) the feedforward peaks at %.0f instead of %.0f - it pushes the motors over their lag", with, without);
      // max rate limit: no feedforward into the end stop
      FeedforwardConfig lim = plain_ff; lim.max_rate_limit = 90; { FeedforwardAxis f; f.configure(lim, ROLL); RxRate rx; rx.reset(250); f.set_rx_rate(250); float at_end = 0, mid = 0;
        for (int i = 0; i <= 50; i++) { const float sp = 10.0f * i, stick = sp / 500; const float o = f.on_frame(stick * 500, sp, 500, stick, rx); if (i == 25) mid = o; if (i == 50) at_end = o; }
        CHECK(near(mid, 2500, 30) && at_end == 0, "max rate limit 90: full feedforward in the middle of the travel (%.0f), none as the setpoint arrives at the top rate (%.0f) - no overshoot past it", mid, at_end);
        float back = 0; for (int i = 49; i >= 45; i--) { const float sp = 10.0f * i, stick = sp / 500; back = f.on_frame(stick * 500, sp, 500, stick, rx); } CHECK(back < -1000, "coming back from the end stop is not limited (%.0f)", back); }
      // transition: less around centre
      FeedforwardConfig tr = plain_ff; tr.transition = 40; const float centre = sweep(tr, ROLL, 500, 250, 0.0f, 51), outer = sweep(tr, ROLL, 500, 250, 0.5f, 51);
      CHECK(near(centre, 500 * (0.2f / 0.4f), 12) && near(outer, 500, 5), "transition 40: at 20 %% stick half of the feedforward (%.0f), beyond 40 %% all of it (%.0f)", centre, outer);
      // averaging
      { FeedforwardConfig a0 = plain_ff; a0.smooth_factor = 0; FeedforwardConfig a2 = a0; a2.averaging = 1; FeedforwardAxis f0, f2; f0.configure(a0, ROLL); f2.configure(a2, ROLL); RxRate rx; rx.reset(250); f0.set_rx_rate(250); f2.set_rx_rate(250); float o0 = 0, o2 = 0;
        const float sps[4] = {0, 4, 4.4f, 8.4f}; for (int i = 0; i < 4; i++) { o0 = f0.on_frame(sps[i], sps[i], 500, sps[i] / 500, rx); o2 = f2.on_frame(sps[i], sps[i], 500, sps[i] / 500, rx); }
        CHECK(near(o0, 1000, 1) && near(o2, (1000 + 100) / 2.0f, 1), "2-point averaging: the mean of the last two frames (%.0f; %.0f without) - steadies a noisy link", o2, o0); }
      // smoothing
      { FeedforwardConfig s0 = plain_ff; s0.smooth_factor = 0; FeedforwardConfig s65 = plain_ff; s65.smooth_factor = 65; FeedforwardAxis f0, f1; f0.configure(s0, ROLL); f1.configure(s65, ROLL); RxRate rx; rx.reset(250); f0.set_rx_rate(250); f1.set_rx_rate(250);
        f0.on_frame(0, 0, 500, 0, rx); f1.on_frame(0, 0, 500, 0, rx); const float a = f0.on_frame(4, 4, 500, 0.008f, rx), b = f1.on_frame(4, 4, 500, 0.008f, rx);
        CHECK(near(a, 1000, 1) && near(b, 1000 * (1 - 0.65f), 2), "smoothness 65 on a 250 Hz link: the first frame of a move shows 35 %% of the speed (%.0f of %.0f)", b, a); }
      // a lost frame: the next one carries two frame periods of stick movement
      { FeedforwardConfig c0 = plain_ff; c0.smooth_factor = 0; FeedforwardAxis f; f.configure(c0, ROLL); RxRate rx; rx.reset(250); f.set_rx_rate(250); float before = 0, at = 0, after = 0, sp = 0;
        for (int i = 0; i < 12; i++) { const bool lost = i == 8; if (lost) continue;                                  // the stick sweeps 1000 dps/s all along
          sp = 4.0f * i; rx.on_frame(i == 9 ? 0.008f : 0.004f); const float o = f.on_frame(sp, sp, 500, sp / 500, rx); if (i == 7) before = o; if (i == 9) at = o; if (i == 10) after = o; }
        CHECK(near(before, 1000, 1) && near(at, 1000, 1) && near(after, 1000, 1), "a frame is lost in the middle of a move: the feedforward stays the stick's speed (%.0f, %.0f, %.0f) - taken at the nominal rate it would have doubled for one frame", before, at, after);
        FeedforwardConfig cb = c0; cb.boost = 15; FeedforwardAxis fb; fb.configure(cb, ROLL); RxRate r2; r2.reset(250); fb.set_rx_rate(250); float worst = 0;
        for (int i = 0; i < 12; i++) { if (i == 8) continue; sp = 4.0f * i; r2.on_frame(i == 9 ? 0.008f : 0.004f); const float o = fb.on_frame(sp, sp, 500, sp / 500, r2); if (i >= 4) worst = fmaxf(worst, fabsf(o - 1000)); }
        CHECK(worst < 1.0f, "... and the boost sees no acceleration in it either (off by %.1f at most)", worst);
        // the link was gone for a while and the stick is somewhere else when it returns
        FeedforwardAxis g; g.configure(c0, ROLL); RxRate r3; r3.reset(250); g.set_rx_rate(250); for (int i = 0; i < 6; i++) { r3.on_frame(0.004f); g.on_frame(0, 0, 500, 0, r3); }
        r3.on_frame(0.25f); const float back = g.on_frame(200, 200, 500, 0.4f, r3); r3.on_frame(0.004f); const float next = g.on_frame(204, 204, 500, 0.408f, r3);
        CHECK(back == 0.0f && near(next, 1000, 1), "a quarter of a second without frames, the stick 200 dps further on: that jump is not a speed (feedforward %.0f); the frame after it is (%.0f)", back, next);
        FeedforwardConfig ci = c0; ci.interpolate = true; FeedforwardAxis gi; gi.configure(ci, ROLL); RxRate r4; r4.reset(250); gi.set_rx_rate(250); for (int i = 0; i < 6; i++) { r4.on_frame(0.004f); gi.on_frame(0, 0, 500, 0, r4); }
        r4.on_frame(0.25f); CHECK(gi.on_frame(200, 200, 500, 0.4f, r4) == 0.0f, "the same with the repeat handling on (a frame held twice before the gap)"); }
      // the same through the whole stick path: frames every 16 control cycles, one of them lost
      { auto peak_ff = [](int lost_frame) { SetpointGenerator g; FeedforwardConfig ff; ff.smooth_factor = 0; ff.averaging = 0; ff.boost = 0; ff.jitter_factor = 0; RcSmoothingConfig sm; sm.enabled = false; g.configure(ff, sm, FS);
            RateProfile rates; rates.center_sens_dps[ROLL] = rates.max_rate_dps[ROLL] = 500; rates.expo[ROLL] = 0; float pk = 0;
            for (int f = 0; f < 60; f++) { for (int k = 0; k < 16; k++) g.update();
                if (f == lost_frame) continue;
                const float st[3] = {f < 30 ? 0.0f : 0.004f * (f - 30), 0, 0}; g.on_frame(st, 0, rates); if (f > 32) pk = fmaxf(pk, g.update().feedforward[ROLL]); }
            return pk; };
        const float clean = peak_ff(-1), lossy = peak_ff(40);
        CHECK(near(clean, 0.004f * 500 * 250, 20) && lossy < clean * 1.05f, "through the stick path at 250 Hz: feedforward %.0f dps/s, %.0f at most with one frame lost", clean, lossy); }
      // repeated frames (links that send a frame twice): the speed is carried over the repeat
      { FeedforwardConfig in = plain_ff; in.smooth_factor = 0; in.interpolate = true; FeedforwardConfig no = in; no.interpolate = false; FeedforwardAxis fi, fn; fi.configure(in, ROLL); fn.configure(no, ROLL); RxRate rx; rx.reset(250); fi.set_rx_rate(250); fn.set_rx_rate(250);
        const float sps[8] = {4, 8, 12, 16, 20, 20, 28, 32}; float oi[8], on[8]; for (int i = 0; i < 8; i++) { oi[i] = fi.on_frame(sps[i], sps[i], 500, sps[i] / 500, rx); on[i] = fn.on_frame(sps[i], sps[i], 500, sps[i] / 500, rx); }
        CHECK(on[5] == 0 && near(on[6], 2000, 1) && near(oi[5], 1000, 1) && near(oi[6], 1000, 1) && near(oi[7], 1000, 1), "a frame sent twice in the middle of a move: without the repeat handling feedforward drops to 0 and then doubles (%.0f, %.0f); with it the speed stays (%.0f, %.0f)", on[5], on[6], oi[5], oi[6]);
        float o = 0; for (int i = 0; i < 3; i++) o = fi.on_frame(32, 32, 500, 0.064f, rx); CHECK(o == 0, "two equal frames in a row: the stick has stopped"); }
      // yaw hold: a sustained push while the yaw stick is held over
      { FeedforwardConfig y = plain_ff; y.yaw_hold_gain = 15; y.yaw_hold_time = 100; FeedforwardAxis f; f.configure(y, YAW); RxRate rx; rx.reset(250); f.set_rx_rate(250); f.on_frame(0, 0, 400, 0, rx);
        const float first = f.on_frame(100, 80, 400, 0.2f, rx); float later = 0; for (int i = 0; i < 25; i++) later = f.on_frame(100, 80, 400, 0.2f, rx); float end = 0; for (int i = 0; i < 250; i++) end = f.on_frame(100, 80, 400, 0.2f, rx);
        CHECK(first > 15 * 80 * 0.9f && near(later, 15 * 80 * expf(-1.0f), 0.12f * 15 * 80) && fabsf(end) < 5, "yaw hold 15 / 100 ms: a yaw step of 80 dps adds 15 x 80 at once, 37 %% of that 100 ms later (%.0f), nothing in the end - yaw starts at once although its motors are slow", later); } }
    // ---- the same pieces, the cases a pilot would notice
    { FeedforwardConfig pf; pf.boost = 0; pf.max_rate_limit = 0; pf.averaging = 0; pf.jitter_factor = 0; pf.yaw_hold_gain = 0; pf.smooth_factor = 0;
      auto axis = [](const FeedforwardConfig& cfg, int ax, float hz, RxRate& rx, FeedforwardAxis& f) { f.configure(cfg, ax); rx.reset(hz); f.set_rx_rate(hz); };
      // leaving the end stop
      { FeedforwardConfig lim = pf; lim.max_rate_limit = 90; FeedforwardAxis f; RxRate rx; axis(lim, ROLL, 250, rx, f); for (int i = 0; i <= 50; i++) f.on_frame(10.0f * i, 10.0f * i, 500, i / 50.0f, rx); for (int i = 0; i < 3; i++) f.on_frame(500, 500, 500, 1, rx);
        const float back = f.on_frame(490, 490, 500, 0.98f, rx); CHECK(near(back, -2500, 1.0f), "the stick leaves the end stop: the very first frame has its full feedforward (%.0f) - the limit only holds back a move TOWARDS the top rate", back); }
      // yaw hold with a short hold time, and with a long one
      { FeedforwardConfig y = pf; y.yaw_hold_gain = 15; y.yaw_hold_time = 50; FeedforwardAxis f; RxRate rx; axis(y, YAW, 250, rx, f); f.on_frame(0, 0, 400, 0, rx); const float first = f.on_frame(100, 80, 400, 0.2f, rx);
        const float want = 80 * 250 + 15 * 1.5f * 80 * (1 - pt1_gain_from_delay(0.05f, 0.004f));
        FeedforwardConfig y2 = pf; y2.yaw_hold_gain = 15; y2.yaw_hold_time = 200; FeedforwardAxis f2; RxRate r2; axis(y2, YAW, 250, r2, f2); f2.on_frame(0, 0, 400, 0, r2); f2.on_frame(100, 80, 400, 0.2f, r2); float later = 0; for (int i = 0; i < 25; i++) later = f2.on_frame(100, 80, 400, 0.2f, r2);
        CHECK(near(first, want, 2.0f) && near(later, 15 * 80 * expf(-0.5f), 0.06f * 15 * 80), "yaw hold time 50 ms: the gain is raised x 150 / (50 + 50) = 1.5 to make up for the shorter push (%.0f); 200 ms: 61 %% is still there after 100 ms (%.0f)", first, later); }
      // the speed is taken with the SMOOTHED frame rate
      { FeedforwardAxis f; RxRate rx; axis(pf, ROLL, 154, rx, f); float lo = 1e9f, hi = -1e9f;
        for (int i = 1; i <= 40; i++) { rx.on_frame((i % 2) ? 0.006f : 0.007f); const float o = f.on_frame(2.0f * i, 2.0f * i, 500, 2.0f * i / 500, rx); if (i > 20) { lo = fminf(lo, o); hi = fmaxf(hi, o); } }
        CHECK(hi < 1.03f * lo && near(lo, 2 * 154, 12), "frames 6 / 7 ms apart and a stick moving evenly: the feedforward is even too (%.0f .. %.0f) - the single intervals would make it jump by 15 %%", lo, hi); }
      // jitter reduction looks at two frames
      { FeedforwardConfig j = pf; j.jitter_factor = 7; FeedforwardAxis f; RxRate rx; axis(j, ROLL, 250, rx, f); float sp = 0, at3 = 0, at1 = 0; f.on_frame(0, 0, 500, 0, rx);
        for (int i = 0; i < 20; i++) { const float step = (i % 2) ? 1.0f : 3.0f; sp += step; const float o = f.on_frame(sp, sp, 500, sp / 500, rx); if (step == 3.0f) at3 = o; else at1 = o; }
        CHECK(near(at3, 3 * 250 * (2 + 1) / 8.0f, 1.0f) && near(at1, 1 * 250 * (2 + 1) / 8.0f, 1.0f), "a noisy link (steps of 3, 1, 3, 1): the reduction uses the mean of the last two steps, (2 + 1) / 8 for both (%.0f, %.0f)", at3, at1); }
      // boost is computed on the smoothed speed, and smoothed again
      { FeedforwardConfig b = pf; b.boost = 15; b.smooth_factor = 65; FeedforwardAxis f; RxRate rx; axis(b, ROLL, 250, rx, f); float pk = 0, want = 0; f.on_frame(0, 0, 500, 0, rx);
        { const float k = pt1_gain_from_delay(0.004f * 0.65f / 0.35f, 0.004f); float sp = 0, prev = 0, dl = 0; for (int i = 1; i <= 6; i++) { sp += k * (1000 - sp); dl += k * ((sp - prev) - dl); prev = sp; want = fmaxf(want, sp + dl * 250 * 0.015f); } }
        for (int i = 1; i <= 6; i++) pk = fmaxf(pk, f.on_frame(4.0f * i, 4.0f * i, 500, 4.0f * i / 500, rx));
        CHECK(near(pk, want, 2.0f) && want > 1250 && want < 1400, "boost with smoothness 65: both the speed and its change go through the low-pass - the peak is %.0f (a boost on the raw change would be 1660)", pk); }
      // smoothness is relative to the link rate
      { FeedforwardConfig s = pf; s.smooth_factor = 65; FeedforwardAxis f; RxRate rx; axis(s, ROLL, 50, rx, f); f.on_frame(0, 0, 500, 0, rx); const float o = f.on_frame(20, 20, 500, 0.04f, rx); const float tau = 0.004f * 0.65f / 0.35f;
        CHECK(near(o, 1000 * 0.02f / (0.02f + tau), 2.0f) && o > 700, "smoothness 65 on a 50 Hz link: its frames are 20 ms apart, the same low-pass lets %.0f of 1000 through in the first frame (35 %% at 250 Hz)", o); }
      // repeated frames
      { FeedforwardConfig in = pf; in.interpolate = true; FeedforwardAxis f; RxRate rx; axis(in, ROLL, 250, rx, f); float o = 0; for (int i = 0; i <= 48; i++) o = f.on_frame(10.0f * i, 10.0f * i, 500, i / 50.0f, rx); const float moving = o; o = f.on_frame(480, 480, 500, 0.96f, rx);
        CHECK(near(moving, 2500, 1) && o == 0, "a repeated frame close to the end of the stick travel (96 %%) is NOT carried on: the stick may really have stopped there, and the craft must not be pushed past its top rate (%.0f)", o);
        FeedforwardConfig bo = in; bo.boost = 15; FeedforwardAxis g; RxRate r2; axis(bo, ROLL, 250, r2, g); const float sps[6] = {4, 8, 12, 12, 12, 12}; float last[6]; for (int i = 0; i < 6; i++) last[i] = g.on_frame(sps[i], sps[i], 500, sps[i] / 500, r2);
        CHECK(last[4] == 0 && last[5] == 0, "the stick has stopped (two equal frames): feedforward is exactly zero, with the boost on as well - no kick backwards (%.0f, %.0f)", last[4], last[5]);
        FeedforwardConfig jt = in; jt.jitter_factor = 7; FeedforwardAxis h; RxRate r3; axis(jt, ROLL, 250, r3, h); const float cr[7] = {1, 2, 3, 4, 4, 6, 7}; float oc[7]; for (int i = 0; i < 7; i++) oc[i] = h.on_frame(cr[i], cr[i], 500, cr[i] / 500, r3);
        CHECK(near(oc[4], oc[3], 0.01f * oc[3]) && near(oc[3], 250 * 2 / 8.0f, 0.5f), "a creeping stick with one frame repeated: the repeat gets the same jitter reduction as the frames around it (%.1f, %.1f)", oc[3], oc[4]); } }
    // ---- the generator: feedforward of a real stick move, through the PT3
    { SetpointGenerator g; FeedforwardConfig ff; RcSmoothingConfig sm; g.configure(ff, sm, FS); RateProfile rates; float st[3] = {0, 0, 0}; StickSetpoint o; float peak = 0;
      for (int f = 0; f < 40; f++) { g.on_frame(st, 0, rates); for (int k = 0; k < 16; k++) g.update(); }
      for (int f = 0; f < 50; f++) { st[0] = 0.5f * fminf(f / 25.0f, 1.0f); g.on_frame(st, 0, rates); for (int k = 0; k < 16; k++) { o = g.update(); peak = fmaxf(peak, o.feedforward[0]); } }
      const float mean_speed = apply_actual_rate(rates, ROLL, 0.5f) / 0.1f;
      CHECK(peak > 0.8f * mean_speed && peak < 2.6f * mean_speed && fabsf(o.feedforward[0]) < 0.05f * mean_speed && near(o.rate[0], 185, 1), "half stick in 100 ms: feedforward of the order of the setpoint's speed (peak %.0f, mean speed %.0f dps/s), zero once the stick rests", peak, mean_speed); }
    // ---- what the PT3s do, exactly; and the feedforward is told the link rate
    { SetpointGenerator g; FeedforwardConfig ff; RcSmoothingConfig sm; sm.auto_factor_throttle = 60; g.configure(ff, sm, FS); RateProfile rates; const float st0[3] = {0, 0, 0}, st1[3] = {1, 0, 0};
      for (int f = 0; f < 60; f++) { g.on_frame(st0, 0, rates); for (int k = 0; k < 16; k++) g.update(); }
      Pt3 ref, refs, reft; ref.set_cutoff(g.setpoint_cutoff_hz(), DT); ref.reset(0.0f); refs.set_cutoff(g.setpoint_cutoff_hz(), DT); refs.reset(0.0f); reft.set_cutoff(g.throttle_cutoff_hz(), DT); reft.reset(0.0f);
      const float half[3] = {0.5f, 0, 0}, R = apply_actual_rate(rates, ROLL, 0.5f); (void)st1;
      g.on_frame(half, 1.0f, rates); bool same = true; StickSetpoint o; float ff1 = 0;
      for (int k = 0; k < 16; k++) { o = g.update(); if (k == 0) ff1 = o.feedforward[0]; same &= near(o.rate[0], ref.apply(R), 0.01f) && near(o.throttle, reft.apply(1.0f), 1e-5f) && near(o.stick[0], refs.apply(0.5f), 1e-5f); }
      CHECK(same && o.rate[0] > 0.45f * R && o.rate[0] < 0.9f * R && o.throttle < 0.8f * o.rate[0] / R, "RC smoothing is a PT3 at the cutoff shown (%.1f Hz sticks, %.1f Hz throttle): 4 ms after a stick step the setpoint is at %.0f of %.0f dps, the throttle at %.2f", g.setpoint_cutoff_hz(), g.throttle_cutoff_hz(), o.rate[0], R, o.throttle);
      CHECK(ff1 > 0 && ff1 < 0.05f * o.feedforward[0], "feedforward goes through the same smoothing: %.0f in the first cycle after the frame, %.0f at its end", ff1, o.feedforward[0]);
      g.neutral(); for (int k = 0; k < 2000; k++) o = g.update(); CHECK(fabsf(o.stick[0]) < 1e-4f && fabsf(o.rate[0]) < 0.01f, "neutral() centres the stick positions too (Stabilize flies on those)");
      FeedforwardConfig pf; pf.boost = 0; pf.max_rate_limit = 0; pf.averaging = 0; pf.jitter_factor = 0; pf.yaw_hold_gain = 0; RcSmoothingConfig off; off.enabled = false; SetpointGenerator r; r.configure(pf, off, FS);
      for (int f = 0; f < 60; f++) { r.on_frame(st0, 0, rates); for (int k = 0; k < 16; k++) r.update(); }
      const float st2[3] = {0.01f, 0, 0}; r.on_frame(st2, 0, rates); const StickSetpoint q = r.update(); const float speed = apply_actual_rate(rates, ROLL, 0.01f) * r.rx_rate().smoothed_hz();
      CHECK(near(r.rx_rate().smoothed_hz(), 250, 1.0f) && near(q.feedforward[0], 0.35f * speed, 0.02f * speed), "the feedforward's own low-pass follows the measured link rate: smoothness 65 at 250 Hz shows 35 %% in the first frame (%.1f of %.1f)", q.feedforward[0], speed); }
}

// ---------------------------------------------------------------------------------------------------------------------------------
static void t_attitude() {
    printf("[angle / stabilize] the two self-levelling controllers\n");
    // ---- ArduPilot's square-root controller
    { CHECK(sqrt_controller(2.0f, 4.5f, 0, DT) == 9.0f && sqrt_controller(-2.0f, 4.5f, 0, 0) == -9.0f, "no limit: a plain P");
      const float lin = 100.0f / (4.5f * 4.5f);                                                                                       // 4.94 deg
      CHECK(near(sqrt_controller(lin * 0.5f, 4.5f, 100, 0), 4.5f * lin * 0.5f, 1e-4f) && near(sqrt_controller(20, 4.5f, 100, 0), sqrtf(2 * 100 * (20 - lin / 2)), 1e-3f) && near(sqrt_controller(-20, 4.5f, 100, 0), -sqrtf(2 * 100 * (20 - lin / 2)), 1e-3f),
            "with a limit of 100 deg/s^2 and P 4.5: linear inside %.2f deg, sqrt(2 a (e - lin / 2)) outside (20 deg -> %.1f dps, not 90)", lin, sqrt_controller(20, 4.5f, 100, 0));
      CHECK(near(sqrt_controller(lin, 4.5f, 100, 0), 4.5f * lin, 1e-3f) && near(sqrt_controller(lin * 1.001f, 4.5f, 100, 0), 4.5f * lin, 0.05f), "the two pieces meet without a step");
      CHECK(near(sqrt_controller(20, 0, 100, 0), sqrtf(2 * 100 * 20.0f), 1e-3f) && sqrt_controller(0, 0, 100, 0) == 0, "P 0: the pure square root");
      CHECK(near(sqrt_controller(0.001f, 4.5f, 0, 0.01f), 0.0045f, 1e-6f) && near(sqrt_controller(0.001f, 400, 0, 0.01f), 0.1f, 1e-6f), "never past the target within one step (error / dt)");
      // a body braked by this rule with exactly the allowed deceleration stops ON the target
      float x = 30, v = 0, vmax = 0, amax = 0; for (int i = 0; i < 40000 && x > 0.01f; i++) { const float want = sqrt_controller(x, 4.5f, 100, 0.0005f); const float a = clampf((want - v) / 0.0005f, -100, 100); v += a * 0.0005f; x -= v * 0.0005f; vmax = fmaxf(vmax, v); if (x < 20) amax = fmaxf(amax, -a); }
      CHECK(x <= 0.011f && x > -0.05f && vmax > 45 && amax <= 100.0f, "closing 30 deg with at most 100 deg/s^2: top speed %.0f dps, arrives without overshoot (%.3f)", vmax, x); }
    { CHECK(sqrt_controller_accel(5, 10, -3, 2, 100) == 0 && sqrt_controller_accel(5, 10, 0, 2, 100) == 0, "sqrt_controller_accel: nothing while moving away from the target");
      CHECK(near(sqrt_controller_accel(1, 2, 3, 2, 0), -6, 1e-5f) && near(sqrt_controller_accel(1, 2, 3, 2, 100), -6, 1e-5f) && near(sqrt_controller_accel(50, 80, 3, 2, 100), -(100.0f / 80) * 3, 1e-4f) && near(sqrt_controller_accel(50, 80, 3, 0, 100), -(100.0f / 80) * 3, 1e-4f),
            "linear range: -p x rate; square-root range: -(limit / command) x rate"); }
    // ---- the command model: critically damped, never above the acceleration limit
    { float rate = 0, accel = 0, target = 0, peak_acc = 0, peak_rate = 0, over = 0, t63 = -1; const float dt = 0.0025f;
      for (int i = 0; i < 1200; i++) { attitude_command_model(10.0f - target, 0, rate, accel, 0, 1100, 0.15f, dt); target += rate * dt; peak_acc = fmaxf(peak_acc, fabsf(accel)); peak_rate = fmaxf(peak_rate, rate); over = fmaxf(over, target - 10.0f); if (t63 < 0 && target > 6.32f) t63 = i * dt; }
      CHECK(near(target, 10, 0.01f) && over < 0.02f && peak_acc <= 1100.01f && t63 > 0.2f && t63 < 0.45f, "a 10 deg stick step with input time constant 0.15 s: the target glides in - 63 %% after %.2f s, overshoot %.3f deg, peak %.0f deg/s^2 (limit 1100)", t63, over, peak_acc);
      rate = accel = target = peak_acc = over = 0; for (int i = 0; i < 2400; i++) { attitude_command_model(90.0f - target, 0, rate, accel, 0, 1100, 0.15f, dt); target += rate * dt; peak_acc = fmaxf(peak_acc, fabsf(accel)); over = fmaxf(over, target - 90.0f); }
      CHECK(near(target, 90, 0.05f) && over < 0.3f && near(peak_acc, 1100, 1.0f), "a 90 deg step: runs at the acceleration limit (%.0f deg/s^2) and still does not overshoot (%.3f)", peak_acc, over);
      rate = accel = 0; float jerk_peak = 0, prev = 0; for (int i = 0; i < 40; i++) { attitude_command_model(60.0f, 0, rate, accel, 0, 1100, 0.15f, dt); jerk_peak = fmaxf(jerk_peak, (accel - prev) / dt); prev = accel; }
      CHECK(near(jerk_peak, 1100 / 0.15f, 5.0f), "the acceleration itself builds up at accel / tc = %.0f deg/s^3 (no jolt)", jerk_peak);
      rate = accel = 0; for (int i = 0; i < 400; i++) attitude_command_model(0, 120, rate, accel, 0, 270, 0, dt); CHECK(near(rate, 120, 0.5f), "rate input (yaw): the target rate goes to the commanded 120 dps");
      rate = accel = 0; float pk = 0; for (int i = 0; i < 100; i++) { attitude_command_model(0, 500, rate, accel, 0, 270, 0, dt); pk = fmaxf(pk, accel); } CHECK(near(pk, 270, 0.5f) && near(rate, 270 * 0.25f, 5.0f), "... with the yaw acceleration limit: 270 deg/s^2 (after 0.25 s at %.0f dps)", rate);
      rate = accel = 0; for (int i = 0; i < 800; i++) { attitude_command_model(100, 0, rate, accel, 30, 1100, 0.15f, dt); } CHECK(near(rate, 30, 0.5f), "a rate limit caps the target's speed (%.1f)", rate);
      float r0 = 5, a0 = 7; attitude_command_model(10, 0, r0, a0, 0, 1100, 0.15f, 0); CHECK(r0 == 5 && a0 == 7, "dt 0: untouched"); }
    // ---- the limits of the shaper, one at a time (ArduPilot's limit_total switch: the limits bound the correction only, or the total)
    { const float dt = 0.0025f; auto run = [&](float pos_err, float vel_des, float acc_des, float vmax, bool total, float& peak_acc) { float v = 0, a = 0; peak_acc = 0;
          for (int i = 0; i < 2000; i++) { shape_pos_vel_accel(pos_err, vel_des, acc_des, v, a, vmax, 300, 3000, dt, total); v += a * dt; peak_acc = fmaxf(peak_acc, fabsf(a)); } return v; };
      float pk; const float v_corr = run(1000, 0, 0, 30, false, pk); run(100000, 0, 0, 0, false, pk);
      CHECK(near(v_corr, 30, 0.3f) && pk <= 300.01f && pk > 299, "correction only: a huge error is closed at the velocity limit (%.1f of 30); without one, at the acceleration limit and not above (%.0f of 300)", v_corr, pk);
      const float v_tot = run(0, 50, 0, 30, true, pk), v_free = run(0, 50, 0, 30, false, pk); CHECK(near(v_tot, 30, 0.3f) && near(v_free, 50, 0.5f), "a desired velocity above the limit: cut to it when the total is limited (%.1f), followed when only the correction is (%.1f)", v_tot, v_free);
      run(1000, 0, 500, 0, true, pk); const float pk_tot = pk; run(1000, 0, 500, 0, false, pk); CHECK(pk_tot <= 300.01f && pk > 700, "a desired acceleration on top: the total stays inside the limit (%.0f) or is added to the limited correction (%.0f)", pk_tot, pk);
      float rate = 0, accel = 0, target = 0, pk_a = 0; for (int i = 0; i < 1200; i++) { attitude_command_model(90.0f - target, 0, rate, accel, 0, 0, 0.15f, dt); target += rate * dt; pk_a = fmaxf(pk_a, fabsf(accel)); }
      CHECK(near(target, 90, 0.05f) && pk_a > 1500 && pk_a <= 1800.01f, "no acceleration limit set (0): ArduPilot's 1800 deg/s^2 is used (%.0f) - the target still moves", pk_a); }
    // ---- Betaflight Angle
    { AngleConfig c; CHECK(c.limit_deg == 45 && c.strength == 50 && c.feedforward == 50 && c.ff_smoothing_ms == 80 && c.earth_ref == 100, "Angle defaults");
      AngleController a; a.configure(c, FS); const float mr[2] = {670, 670}, ff0[2] = {0, 0}; float sp[3] = {0, 0, 0};
      for (int i = 0; i < 400; i++) { sp[0] = sp[1] = sp[2] = 0; a.update(sp, ff0, 10.0f, -4.0f, mr); }
      CHECK(near(sp[0], -50, 0.1f) && near(sp[1], 20, 0.1f) && sp[2] == 0, "strength 50: 5 dps per degree - 10 deg right-wing-down gives -50 dps, 4 deg nose-up +20 (%.1f, %.1f)", sp[0], sp[1]);
      for (int i = 0; i < 400; i++) { sp[0] = 670; sp[1] = -335; sp[2] = 0; a.update(sp, ff0, 45.0f, -22.5f, mr); }
      CHECK(near(a.target_deg(0), 45, 1e-3f) && near(a.target_deg(1), -22.5f, 1e-3f) && fabsf(sp[0]) < 0.1f && fabsf(sp[1]) < 0.1f, "the stick's rate setpoint, as a fraction of the top rate, is the tilt: full = 45 deg, half of the top rate = 22.5");
      for (int i = 0; i < 400; i++) { sp[0] = 2000; sp[1] = 0; sp[2] = 0; a.update(sp, ff0, 0, 0, mr); } CHECK(near(a.target_deg(0), 45, 1e-3f), "never beyond the limit");
      // feedforward: the stick's speed tilts the craft at once
      const float full_ff = 45 * 3000 * 0.5f / 670;
      AngleController f; f.configure(c, FS); float peak = 0; for (int i = 0; i < 4000; i++) { const float ff[2] = {i < 400 ? 3000.0f : 0.0f, 0}; sp[0] = sp[1] = sp[2] = 0; f.update(sp, ff, 0, 0, mr); peak = fmaxf(peak, sp[0]); }
      CHECK(peak > 0.45f * full_ff && peak < 0.75f * full_ff && fabsf(sp[0]) < 0.5f, "feedforward 50: a 100 ms stick move at 3000 dps/s tilts the craft with up to %.0f dps (of the 45 x 3000 x 0.5 / 670 = %.0f a long move would give: it is smoothed over 80 ms), gone when the stick rests", peak, full_ff);
      AngleController fl; fl.configure(c, FS); for (int i = 0; i < 4000; i++) { const float ff[2] = {3000.0f, 0}; sp[0] = sp[1] = sp[2] = 0; fl.update(sp, ff, 0, 0, mr); } CHECK(near(sp[0], full_ff, 0.5f), "a sustained move: %.1f", sp[0]);
      AngleConfig nf = c; nf.feedforward = 0; AngleController g; g.configure(nf, FS); float pk0 = 0; for (int i = 0; i < 400; i++) { const float ff[2] = {3000, 0}; sp[0] = sp[1] = sp[2] = 0; g.update(sp, ff, 0, 0, mr); pk0 = fmaxf(pk0, fabsf(sp[0])); } CHECK(pk0 == 0, "feedforward 0: off");
      // earth reference: yaw while tilted
      AngleController e; e.configure(c, FS); for (int i = 0; i < 800; i++) { sp[0] = 670 * 30.0f / 45; sp[1] = 0; sp[2] = 100; e.update(sp, ff0, 30, 0, mr); }
      CHECK(near(sp[2], 100 * cosf(30 * kDeg2Rad), 0.1f) && near(sp[1], -100 * sinf(30 * kDeg2Rad), 0.2f) && fabsf(sp[0]) < 0.2f, "banked 30 deg with 100 dps of yaw stick: the turn is about the vertical - body yaw 100 cos 30 = %.1f, body pitch -100 sin 30 = %.1f (the nose is held on the horizon)", sp[2], sp[1]);
      AngleConfig ne = c; ne.earth_ref = 0; AngleController b; b.configure(ne, FS); for (int i = 0; i < 800; i++) { sp[0] = 670 * 30.0f / 45; sp[1] = 0; sp[2] = 100; b.update(sp, ff0, 30, 0, mr); }
      CHECK(sp[2] == 100 && fabsf(sp[1]) < 0.01f, "earth reference 0: the yaw stick turns the craft about its own axis");
      // an autopilot's target instead of the sticks
      const float ext[2] = {12.0f, -8.0f}; AngleController x; x.configure(c, FS); for (int i = 0; i < 400; i++) { sp[0] = 670; sp[1] = 670; sp[2] = 0; const float ff[2] = {5000, 5000}; x.update(sp, ff, 0, 0, mr, ext, 10.0f); }
      CHECK(near(x.target_deg(0), 10, 1e-3f) && near(x.target_deg(1), -8, 1e-3f) && near(sp[0], 50, 0.1f) && near(sp[1], -40, 0.1f), "external target (position hold, rescue): the sticks and their feedforward are ignored, the target is cut to the autopilot's own limit (12 -> 10 deg)"); }
    { AngleConfig c; AngleController a; a.configure(c, FS); const float mr[2] = {670, 670}, ff0[2] = {0, 0}; float sp[3] = {0, 0, 0}; a.update(sp, ff0, 0, 0, mr);
      sp[0] = sp[1] = sp[2] = 0; a.update(sp, ff0, 10.0f, 0, mr); const float one = sp[0]; for (int i = 0; i < 40; i++) { sp[0] = sp[1] = sp[2] = 0; a.update(sp, ff0, 10.0f, 0, mr); }
      Pt3 ref; ref.set_cutoff(kAttitudeCutoffHz, DT); ref.reset(0.0f); float want = 0; for (int i = 0; i < 41; i++) want = ref.apply(-50.0f);
      CHECK(one < 0 && one > -0.1f * 50 && near(sp[0], want, 0.01f), "the levelling rate is smoothed (PT3 50 Hz: the attitude estimate moves in steps): %.2f dps in the first cycle of a 10 deg error, %.1f after 10 ms, 50 in the end", one, sp[0]);
      AngleConfig q = c; q.ff_smoothing_ms = 20; AngleController f; f.configure(q, FS); float peak = 0; for (int i = 0; i < 400; i++) { const float ff[2] = {3000.0f, 0}; sp[0] = sp[1] = sp[2] = 0; f.update(sp, ff, 0, 0, mr); peak = fmaxf(peak, sp[0]); }
      CHECK(peak > 0.9f * 45 * 3000 * 0.5f / 670, "feedforward smoothing 20 ms instead of 80: a 100 ms stick move gets nearly all of it (%.0f of %.0f)", peak, 45 * 3000 * 0.5f / 670); }
    // ---- ArduPilot Stabilize
    { StabilizeConfig c; CHECK(c.kp[0] == 4.5f && c.kp[1] == 4.5f && c.kp[2] == 4.5f && c.accel_rp_dss == 1100 && c.accel_yaw_dss == 270 && near(c.input_tc, 0.15f, 1e-6f) && c.rate_ff && c.limit_deg == 45, "Stabilize defaults = ArduPilot's (ATC_ANG_*_P 4.5, ATC_ACCEL_*_MAX 110000 / 27000 cdeg/s^2, ATC_INPUT_TC 0.15)");
      // an ideal craft (its rates are the setpoints): the attitude follows the stick without overshoot
      StabilizeController s; s.configure(c); s.reset(0, 0, 0); float att[3] = {0, 0, 0}, sp[3], peak = 0, t90 = -1; const float dt = DT, st[2] = {0.8f, 0};
      for (int i = 0; i < 8000; i++) { s.update(st, 0, att, dt, sp); att[0] += sp[0] * dt; att[1] += sp[1] * dt; att[2] += sp[2] * dt; peak = fmaxf(peak, att[0]); if (t90 < 0 && att[0] > 0.9f * 36) t90 = i * dt; }
      CHECK(near(att[0], 36, 0.05f) && peak < 36.05f && t90 > 0.3f && t90 < 0.7f && fabsf(att[1]) < 0.01f && fabsf(att[2]) < 0.01f, "stick 0.8 -> 36 deg (linear in the stick): there after %.2f s, peak %.2f, pitch and heading untouched", t90, peak);
      // the per-axis P gains: a disturbance is corrected with kp x error
      StabilizeController k; StabilizeConfig kc = c; kc.kp[0] = 3; kc.kp[1] = 6; kc.kp[2] = 2; k.configure(kc); k.reset(0, 0, 0); const float a2[3] = {-2, 1, 3}, st0[2] = {0, 0}; k.update(st0, 0, a2, dt, sp);
      CHECK(near(sp[0], 3 * 2 + (2 * -3) * sinf(1 * kDeg2Rad), 0.02f) && near(sp[1], 6 * -1 * cosf(-2 * kDeg2Rad) - (2 * -3) * cosf(1 * kDeg2Rad) * sinf(-2 * kDeg2Rad), 0.02f) && near(sp[2], 6 * -1 * sinf(-2 * kDeg2Rad) + (2 * -3) * cosf(1 * kDeg2Rad) * cosf(-2 * kDeg2Rad), 0.02f),
            "Stabilize Roll / Pitch / Yaw P = 3 / 6 / 2: pushed 2 deg left, 1 deg nose-down and 3 deg right of its heading, it answers %.2f / %.2f / %.2f dps (kp x error, turned into body rates)", sp[0], sp[1], sp[2]);
      // a big error is answered with the square root, not with kp x error
      StabilizeController b; b.configure(c); b.reset(0, 0, 0); const float a3[3] = {-40, 0, 0}; b.update(st0, 0, a3, dt, sp); const float lin = 550.0f / (4.5f * 4.5f);
      CHECK(near(sp[0], sqrtf(2 * 550 * (40 - lin / 2)), 0.5f) && sp[0] < 4.5f * 40, "40 deg off: %.0f dps (sqrt with half the acceleration limit), not 180", sp[0]);
      // heading hold: the target does not run away from a craft that cannot follow
      StabilizeController y; y.configure(c); y.reset(0, 0, 0); float ya[3] = {0, 0, 0}; for (int i = 0; i < 8000; i++) y.update(st0, 200, ya, dt, sp);
      CHECK(near(wrap180(y.target_deg(2) - ya[2]), 45, 0.5f), "yaw stick held while the craft is stuck: the heading target leads by 45 deg at most (%.1f)", wrap180(y.target_deg(2) - ya[2]));
      y.hold_yaw(170); CHECK(y.target_deg(2) == 170 && y.target_rate(2) == 0, "on the ground the heading target is the craft's heading");
      StabilizeController w; w.configure(c); w.reset(0, 0, 179); float wa[3] = {0, 0, 179}; for (int i = 0; i < 4000; i++) { w.update(st0, 60, wa, dt, sp); wa[2] = wrap180(wa[2] + sp[2] * dt); }
      CHECK(fabsf(wrap180(wa[2] - (179 + 60))) < 9 && near(sp[2], 60, 1.5f), "turning through north / south (+-180): no jump (heading %.1f after 1 s at 60 dps from 179)", wa[2]);
      // without the shaping
      StabilizeConfig nf = c; nf.rate_ff = false; StabilizeController d; d.configure(nf); d.reset(0, 0, 0); const float a0[3] = {0, 0, 0}; d.update(st, 0, a0, dt, sp);
      CHECK(near(d.target_deg(0), 36, 1e-4f) && d.target_rate(0) == 0 && near(sp[0], sqrtf(2 * 550 * (36 - lin / 2)), 0.5f), "rate feedforward off: the target jumps with the stick (%.0f dps asked at once)", sp[0]);
      // the limit and the input time constant
      StabilizeConfig lc = c; lc.limit_deg = 30; lc.input_tc = 0.05f; StabilizeController l; l.configure(lc); l.reset(0, 0, 0); float la[3] = {0, 0, 0}; const float full[2] = {1.5f, -1}; float lt90 = -1;
      for (int i = 0; i < 8000; i++) { l.update(full, 0, la, dt, sp); la[0] += sp[0] * dt; la[1] += sp[1] * dt; if (lt90 < 0 && la[0] > 27) lt90 = i * dt; }
      CHECK(near(la[0], 30, 0.05f) && near(la[1], -30, 0.05f) && lt90 < 0.35f, "tilt limit 30: full stick (and beyond) = 30 deg; input time constant 0.05 s is quicker (90 %% after %.2f s)", lt90);
      s.reset(5, -3, 90); CHECK(s.target_deg(0) == 5 && s.target_deg(1) == -3 && s.target_deg(2) == 90 && s.target_rate(0) == 0 && s.target_accel(0) == 0, "reset: the target is where the craft is, at rest");
      float keep[3] = {1, 2, 3}; s.update(st, 0, att, 0, keep); CHECK(keep[0] == 1 && keep[2] == 3, "dt 0: nothing happens"); }
    { StabilizeConfig c; const float dt = DT, st0[2] = {0, 0}; float sp[3];
      StabilizeController y; y.configure(c); y.reset(0, 0, 0); const float still[3] = {0, 0, 0}; for (int i = 0; i < 1000; i++) y.update(st0, 500, still, dt, sp);
      CHECK(near(y.target_rate(2), 270 * 0.25f, 5.0f), "yaw has its own acceleration limit: 0.25 s into a hard yaw command the target turns at %.0f dps (270 deg/s^2), not at the 275 the roll limit would give", y.target_rate(2));
      StabilizeController e; e.configure(c); e.reset(0, 0, 0); const float off[3] = {0, 0, -40}; e.update(st0, 0, off, dt, sp); const float lin = 120.0f / (4.5f * 4.5f);
      CHECK(near(sp[2], sqrtf(2 * 120 * (40 - lin / 2)), 1.0f), "40 deg off its heading: %.1f dps - the square root with 120 deg/s^2 (half the yaw limit, at most 120)", sp[2]);
      StabilizeController w; w.configure(c); w.reset(0, 0, 150); const float stuck[3] = {0, 0, 150}; for (int i = 0; i < 8000; i++) w.update(st0, 200, stuck, dt, sp);
      CHECK(near(wrap180(w.target_deg(2) - 150), 45, 0.5f) && sp[2] > 80, "stuck at heading 150 with the yaw stick held: the target leads by 45 deg ACROSS +-180 (target %.0f), and the craft is still asked to turn the same way (%.0f dps)", w.target_deg(2), sp[2]);
      StabilizeConfig nf = c; nf.rate_ff = false; StabilizeController d; d.configure(nf); d.reset(0, 0, 0); float att[3] = {0, 0, 0}; for (int i = 0; i < 4000; i++) { d.update(st0, 100, att, dt, sp); att[2] = wrap180(att[2] + sp[2] * dt); }
      CHECK(att[2] > 60 && att[2] < 100, "rate feedforward off: the yaw stick still turns the heading target (%.0f deg after 1 s at 100 dps; the craft follows a little behind)", att[2]); }
}

// ---------------------------------------------------------------------------------------------------------------------------------
static void t_sliders() {
    printf("[tuning sliders] Betaflight's simplified tuning\n");
    { PidSliders s; uint8_t p[3] = {1, 2, 3}, i[3] = {4, 5, 6}, d[3] = {7, 8, 9}, dm[3] = {10, 11, 12}; uint16_t f[3] = {13, 14, 15}; sliders_to_pid(s, p, i, d, dm, f);
      CHECK(p[0] == 45 && p[1] == 47 && p[2] == 45 && i[0] == 80 && i[1] == 84 && i[2] == 80 && d[0] == 30 && d[1] == 34 && d[2] == 0 && dm[0] == 40 && dm[1] == 46 && dm[2] == 0 && f[0] == 120 && f[1] == 125 && f[2] == 120, "every slider at 100: exactly the default gains");
      PidConfig def; bool same = true; for (int a = 0; a < 3; a++) same &= def.p[a] == kPidDefaultP[a] && def.i[a] == kPidDefaultI[a] && def.d[a] == kPidDefaultD[a] && def.d_max[a] == kPidDefaultDMax[a] && def.f[a] == kPidDefaultF[a];
      CHECK(same, "one set of default gains: the PID controller's and the sliders'"); }
    { PidSliders s; s.mode = SLIDERS_OFF; uint8_t p[3] = {1, 2, 3}, i[3] = {4, 5, 6}, d[3] = {7, 8, 9}, dm[3] = {10, 11, 12}; uint16_t f[3] = {13, 14, 15}; s.master = 150; sliders_to_pid(s, p, i, d, dm, f);
      CHECK(p[0] == 1 && i[1] == 5 && d[2] == 9 && dm[0] == 10 && f[2] == 15, "mode OFF: nothing is touched");
      s.mode = SLIDERS_RP; sliders_to_pid(s, p, i, d, dm, f); CHECK(p[0] == 67 && p[1] == 70 && p[2] == 3 && i[2] == 6 && d[2] == 9 && dm[2] == 12 && f[2] == 15 && f[0] == 180, "mode RP: roll and pitch computed (master 1.5: P 45 -> 67), yaw as typed"); }
    { PidSliders s; s.master = 0; uint8_t p[3], i[3], d[3], dm[3]; uint16_t f[3]; sliders_to_pid(s, p, i, d, dm, f); CHECK(p[0] == 0 && i[1] == 0 && d[0] == 0 && dm[1] == 0 && f[2] == 0, "master 0: all zero (the craft would not fly - the app warns)");
      s.master = 200; s.pi = 200; s.i = 200; s.ff = 200; s.d = 200; s.pitch_d = 200; s.pitch_pi = 200; s.d_max = 200; sliders_to_pid(s, p, i, d, dm, f);
      CHECK(p[1] == 250 && i[0] == 250 && d[1] == 250 && dm[1] == 250 && f[1] == 1000 && f[0] == 480, "everything at 200: the results stop at the gain limits (250, feedforward 1000)"); }
    { auto calc = [](PidSliders s, uint8_t p[3], uint8_t i[3], uint8_t d[3], uint8_t dm[3], uint16_t f[3]) { sliders_to_pid(s, p, i, d, dm, f); };
      uint8_t p[3], i[3], d[3], dm[3]; uint16_t f[3]; PidSliders s;
      s = PidSliders(); s.pi = 150; calc(s, p, i, d, dm, f); CHECK(p[0] == 67 && p[1] == 70 && i[0] == 120 && i[1] == 126 && d[0] == 30 && f[0] == 120, "Tracking 1.5: P and I of every axis (45 -> 67, 80 -> 120), nothing else");
      s = PidSliders(); s.i = 150; calc(s, p, i, d, dm, f); CHECK(i[0] == 120 && i[2] == 120 && p[0] == 45, "Drift - wobble 1.5: I alone");
      s = PidSliders(); s.pitch_d = 150; calc(s, p, i, d, dm, f); CHECK(d[1] == 51 && dm[1] == 69 && d[0] == 30 && dm[0] == 40, "Pitch damping 1.5: pitch D and D max (34 -> 51, 46 -> 69), roll untouched");
      s = PidSliders(); s.pitch_pi = 150; calc(s, p, i, d, dm, f); CHECK(p[1] == 70 && i[1] == 126 && f[1] == 187 && p[0] == 45 && i[0] == 80 && f[0] == 120, "Pitch tracking 1.5: pitch P, I and feedforward, roll untouched");
      s = PidSliders(); s.d_max = 0; calc(s, p, i, d, dm, f); const uint8_t dm0 = dm[0]; s.d_max = 200; calc(s, p, i, d, dm, f); CHECK(dm0 == 30 && dm[0] == 50 && d[0] == 30, "Dynamic damping: 0 puts D max on D (30), 2.0 twice as far above it as the default (30 + 2 x 10 = 50)");
      s = PidSliders(); s.master = 250; s.ff = 200; s.pitch_pi = 200; calc(s, p, i, d, dm, f); CHECK(f[1] == 1000 && f[0] == 600, "feedforward stops at 1000 (125 x 2.5 x 2 x 2 would be 2500)"); }
    { uint16_t st = 250, mn = 250, mx = 500, l2 = 500; gyro_filter_slider(100, st, mn, mx, l2); CHECK(st == 250 && mn == 250 && mx == 500 && l2 == 500, "gyro filter slider at 100: the default cutoffs");
      gyro_filter_slider(150, st, mn, mx, l2); CHECK(st == 375 && mn == 375 && mx == 750 && l2 == 750, "at 150: every cutoff x 1.5 = less filtering, less delay");
      gyro_filter_slider(10, st, mn, mx, l2); CHECK(st == 25 && mn == 25 && mx == 50 && l2 == 50, "at 10: x 0.1");
      uint16_t s2 = 0, m2 = 0, x2 = 333, q2 = 0; gyro_filter_slider(200, s2, m2, x2, q2); CHECK(s2 == 0 && m2 == 0 && x2 == 333 && q2 == 0, "what is switched off stays off");
      uint16_t ds = 75, dn = 75, dx = 150, d2 = 150; dterm_filter_slider(120, ds, dn, dx, d2); CHECK(ds == 90 && dn == 90 && dx == 180 && d2 == 180, "D-term filter slider at 120: 75 / 150 / 150 -> 90 / 180 / 180");
      CHECK(slider_hz(500, 200) == 1000 && slider_hz(900, 200) == 1000 && slider_hz(75, 133) == 99, "a cutoff is cut to a whole Hz and never passes 1000"); }
}

// ---------------------------------------------------------------------------------------------------------------------------------
static void t_inflight_trim() {
    printf("[in-flight level trim] the attitude of a steady hover becomes \"level\"\n");
    InflightTrim t; InflightTrim::Input in; in.armed = true; in.flying = true; in.roll_deg = 2.0f; in.pitch_deg = -1.5f; const float dt = 0.001f;
    auto run = [&](float seconds) { InflightTrim::Event last = InflightTrim::NONE; for (int i = 0; i < (int)(seconds / dt + 0.5f); i++) { const InflightTrim::Event e = t.update(in, dt); if (e != InflightTrim::NONE) last = e; } return last; };
    CHECK(run(1.0f) == InflightTrim::NONE && !t.collecting() && !t.pending(), "switch off: nothing is measured");
    in.switch_on = true; CHECK(run(1.0f) == InflightTrim::NONE && t.collecting() && near(t.progress(), 0.5f, 0.01f), "switch on in a steady hover: collecting (half-way after 1 s of the 2 s it needs)");
    in.gyro_dps = 40; run(0.5f); CHECK(near(t.progress(), 0.5f, 0.01f), "while the craft is being turned (40 dps) the time does not count");
    in.gyro_dps = 2; in.roll_deg = 2.0f; CHECK(run(1.2f) == InflightTrim::NONE && t.progress() == 1.0f, "steady again: complete");
    in.switch_on = false; CHECK(run(0.01f) == InflightTrim::MEASURED && t.pending() && near(t.roll_deg(), 2.0f, 0.02f) && near(t.pitch_deg(), -1.5f, 0.02f), "switch off: the mean attitude is kept (%.2f, %.2f) - not applied in the air, the horizon must not jump", t.roll_deg(), t.pitch_deg());
    CHECK(run(1.0f) == InflightTrim::NONE && t.pending(), "it waits for the landing");
    in.armed = false; in.flying = false; CHECK(run(0.01f) == InflightTrim::APPLY && !t.pending(), "disarmed: now it is to be applied (once)"); CHECK(run(0.5f) == InflightTrim::NONE, "... once");
    // too short
    { InflightTrim s; InflightTrim::Input i2; i2.armed = i2.flying = i2.switch_on = true; i2.roll_deg = 3; for (int k = 0; k < 800; k++) s.update(i2, dt); i2.switch_on = false; const InflightTrim::Event e = s.update(i2, dt);
      i2.armed = false; const InflightTrim::Event e2 = s.update(i2, dt); CHECK(e == InflightTrim::FAILED && !s.pending() && e2 == InflightTrim::NONE, "switched off after 0.8 s: too short to trust - refused, nothing is changed"); }
    // tilted far / on the ground / still on at the disarm
    { InflightTrim s; InflightTrim::Input i2; i2.armed = i2.flying = i2.switch_on = true; i2.roll_deg = 15; for (int k = 0; k < 3000; k++) s.update(i2, dt); CHECK(s.progress() == 0, "leaning 15 deg is not a hover (limit 10): not counted");
      i2.roll_deg = 0; i2.pitch_deg = -15; for (int k = 0; k < 3000; k++) s.update(i2, dt); CHECK(s.progress() == 0, "nor is 15 deg nose-up"); i2.pitch_deg = 0;
      i2.roll_deg = 1; i2.flying = false; for (int k = 0; k < 3000; k++) s.update(i2, dt); CHECK(s.progress() == 0, "standing on the ground does not count either: the gear decides the attitude there");
      i2.flying = true; for (int k = 0; k < 2500; k++) s.update(i2, dt); i2.armed = false; i2.flying = false; const InflightTrim::Event e = s.update(i2, dt);
      CHECK(e == InflightTrim::APPLY && near(s.roll_deg(), 1.0f, 0.02f), "landed and disarmed with the switch still on: applied as well"); }
    // a new measurement replaces an old one
    { InflightTrim s; InflightTrim::Input i2; i2.armed = i2.flying = i2.switch_on = true; i2.roll_deg = 1; for (int k = 0; k < 2500; k++) s.update(i2, dt); i2.switch_on = false; s.update(i2, dt);
      i2.switch_on = true; i2.roll_deg = 4; for (int k = 0; k < 2500; k++) s.update(i2, dt); i2.switch_on = false; s.update(i2, dt); CHECK(s.pending() && near(s.roll_deg(), 4.0f, 0.02f), "switched on a second time: the newer measurement counts");
      i2.switch_on = true; for (int k = 0; k < 300; k++) s.update(i2, dt); i2.switch_on = false; const InflightTrim::Event e = s.update(i2, dt); CHECK(e == InflightTrim::FAILED && s.pending() && near(s.roll_deg(), 4.0f, 0.02f), "a third, too short try is refused and the good one is kept"); }
    // the average is over the whole window
    { InflightTrim s; InflightTrim::Input i2; i2.armed = i2.flying = i2.switch_on = true; for (int k = 0; k < 4000; k++) { i2.roll_deg = 1.0f + 0.5f * sinf(2 * kPi * 0.8f * k * dt); i2.pitch_deg = (k % 2) ? 0.3f : -0.3f; s.update(i2, dt); } i2.switch_on = false; s.update(i2, dt);
      CHECK(near(s.roll_deg(), 1.0f, 0.06f) && near(s.pitch_deg(), 0.0f, 0.01f), "a craft that sways around its hover attitude: the mean (%.3f, %.3f)", s.roll_deg(), s.pitch_deg()); }
}

int main() {
    t_filters(); t_gyro_chain(); t_dyn_notch(); t_pid(); t_acro_trainer(); t_setpoint(); t_attitude(); t_sliders(); t_inflight_trim();
    printf("\n%d passed, %d failed\n", g_pass, g_fail); return g_fail ? 1 : 0;
}
