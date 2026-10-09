#pragma once
#include "../config/fast_code.h"
#include "filters.h"

// ============================================================================
// Dynamic notch filter: finds the strongest peaks of the gyro noise spectrum while flying and puts a notch on each of them.
// A port of Betaflight's flight/dyn_notch_filter.c + common/sdft.c (sliding DFT, 72 samples -> 36 bins, Hann window applied in
// the frequency domain, parabolic interpolation of the peak, PT1-smoothed centre frequencies that move fast towards a big
// peak and slowly away from it). Same constants, same four-step state machine spread over the control cycles, so the cost
// per cycle is small and even.
//
//   once per control cycle:   for axis 0..2: push(axis, gyro);  gyro = apply(axis, gyro);     then  update();
//
// Needs a loop of 2 kHz or faster (below that the filters would be moved too rarely): count() is 0 otherwise.
// ============================================================================
namespace fc {

constexpr int kSdftSamples = 72;
constexpr int kSdftBins = kSdftSamples / 2;
constexpr int kDynNotchMax = 7;

// Sliding discrete Fourier transform over the last 72 samples; only the bins start..end are kept up to date.
class Sdft {
public:
    void init(int start_bin, int end_bin, int batches) {
        const float r = 0.9999f;                                 // damping factor: keeps the recursion stable
        r_pow_n_ = powf(r, (float)kSdftSamples);
        for (int i = 0; i < kSdftBins; i++) { const float phi = 2.0f * kPi * (float)i / (float)kSdftSamples; tw_re_[i] = r * cosf(phi); tw_im_[i] = r * sinf(phi); }
        idx_ = 0;
        start_ = start_bin < 0 ? 0 : (start_bin > kSdftBins - 1 ? kSdftBins - 1 : start_bin);
        end_ = end_bin < start_ ? start_ : (end_bin > kSdftBins - 1 ? kSdftBins - 1 : end_bin);
        batches_ = batches < 1 ? 1 : batches;
        batch_size_ = (end_ - start_ + 1) / batches_;
        for (float& s : samples_) s = 0.0f;
        for (int i = 0; i < kSdftBins; i++) re_[i] = im_[i] = 0.0f;
    }
    // Adds one sample, updating one part of the bins per call: call with batch 0 .. batches-1 for the same sample.
    WB_FAST void push_batch(float sample, int batch) {
        const int b0 = batch_size_ * batch + start_;
        const float delta = sample - r_pow_n_ * samples_[idx_];
        int b1;
        if (batch == batches_ - 1) { samples_[idx_] = sample; idx_ = (idx_ + 1) % kSdftSamples; b1 = end_ + 1; }
        else b1 = b0 + batch_size_;
        for (int i = b0; i < b1; i++) rotate(i, delta);
        // the bins just outside the range are needed for the window at its edges
        if (start_ > 0 && batch == 0) rotate(start_ - 1, delta);
        if (end_ < kSdftBins - 1 && batch == batches_ - 1) rotate(end_ + 1, delta);
    }
    // Squared magnitude with a Hann window (X[k] - (X[k-1] + X[k+1]) / 2), bins start..end.
    WB_FAST void window_squared(float* out) const {
        for (int i = start_; i <= end_; i++) {
            float re, im;
            if (i == 0) { re = re_[0] - re_[1]; im = im_[0] - im_[1]; }
            else if (i == kSdftBins - 1) { re = re_[i] - re_[i - 1]; im = im_[i] - im_[i - 1]; }
            else { re = re_[i] - 0.5f * (re_[i - 1] + re_[i + 1]); im = im_[i] - 0.5f * (im_[i - 1] + im_[i + 1]); }
            out[i] = re * re + im * im;
        }
    }
    int start_bin() const { return start_; }
    int end_bin() const { return end_; }
private:
    WB_FAST void rotate(int i, float delta) {                            // data[i] = twiddle[i] * (data[i] + delta)
        const float re = re_[i] + delta, im = im_[i];
        re_[i] = tw_re_[i] * re - tw_im_[i] * im; im_[i] = tw_re_[i] * im + tw_im_[i] * re;
    }
    int idx_ = 0, start_ = 1, end_ = kSdftBins - 1, batches_ = 1, batch_size_ = kSdftBins - 1;
    float r_pow_n_ = 1.0f, samples_[kSdftSamples] = {}, re_[kSdftBins] = {}, im_[kSdftBins] = {}, tw_re_[kSdftBins] = {}, tw_im_[kSdftBins] = {};
};

struct DynNotchConfig {
    uint8_t count = 3;             // notches per axis, 0 = off
    float q = 3.0f;                // Betaflight dyn_notch_q / 100
    float min_hz = 100.0f, max_hz = 600.0f;
};

class DynNotch {
public:
    static constexpr float kSmoothHz = 4.0f;       // PT1 on the centre frequencies (x1 .. x10 with the height of the peak)
    static constexpr float kFloorMinHz = 60.0f;    // below this the spectrum is the craft's own motion: not part of the noise floor
    static constexpr float kMinLoopHz = 2000.0f;

    void configure(const DynNotchConfig& c, float loop_hz) {
        count_ = 0;
        if (c.count == 0 || loop_hz < kMinLoopHz) return;
        dt_ = 1.0f / loop_hz;
        const float nyquist = loop_hz * 0.5f;
        q_ = c.q; min_hz_ = c.min_hz; max_hz_ = fmaxf(c.min_hz, c.max_hz); if (max_hz_ > nyquist) max_hz_ = nyquist;
        count_ = c.count > kDynNotchMax ? kDynNotchMax : c.count;

        sample_count_ = (int)(nyquist / max_hz_); if (sample_count_ < 1) sample_count_ = 1;     // 4 kHz, max 600 Hz -> 3: the SDFT sees 1333 Hz
        sample_rate_ = loop_hz / (float)sample_count_;
        resolution_ = sample_rate_ / (float)kSdftSamples;                                       // Hz per bin
        start_bin_ = (int)lrintf(min_hz_ / resolution_); if (start_bin_ < 1) start_bin_ = 1;        // bin 0 is DC
        end_bin_ = (int)lrintf(max_hz_ / resolution_); if (end_bin_ > kSdftBins - 1) end_bin_ = kSdftBins - 1;
        floor_start_ = (int)lrintf(kFloorMinHz / resolution_); if (floor_start_ < start_bin_) floor_start_ = start_bin_;
        if (floor_start_ > end_bin_ - 3) floor_start_ = start_bin_;
        calc_period_s_ = (float)kCalcTicks / loop_hz;

        sample_index_ = 0; tick_ = 0; step_ = 0; axis_ = 0; noise_threshold_ = 0.0f;
        for (int a = 0; a < 3; a++) {
            acc_[a] = avg_[a] = 0.0f;
            sdft_[a].init(start_bin_, end_bin_, sample_count_);
            for (int p = 0; p < count_; p++) {                   // spread over the range: they find their peaks sooner
                center_[a][p] = ((float)p + 0.5f) * (max_hz_ - min_hz_) / (float)count_ + min_hz_;
                notch_[a][p].reset(); notch_[a][p].set(center_[a][p], dt_, q_);
            }
        }
        for (int p = 0; p < kDynNotchMax; p++) { peak_bin_[p] = 0; peak_val_[p] = 0.0f; }
    }

    int count() const { return count_; }
    float center_hz(int axis, int p) const { return center_[axis][p]; }
    void push(int axis, float sample) { acc_[axis] += sample; }
    float apply(int axis, float v) { for (int p = 0; p < count_; p++) v = notch_[axis][p].apply(v); return v; }

    // Once per control cycle, after the three push() calls.
    WB_FAST void update() {
        if (!count_) return;
        if (sample_index_ == sample_count_) {
            sample_index_ = 0;
            for (int a = 0; a < 3; a++) { avg_[a] = acc_[a] / (float)sample_count_; acc_[a] = 0.0f; }
            tick_ = kCalcTicks;                                  // a new sample is in: one round of the state machine over all axes
        }
        for (int a = 0; a < 3; a++) sdft_[a].push_batch(avg_[a], sample_index_);
        sample_index_++;
        if (tick_ > 0) { process(); tick_--; }
    }

private:
    static constexpr int kCalcTicks = 12;          // 3 axes x 4 steps

    WB_FAST void process() {
        switch (step_) {
        case 0: {                                  // window, total power for the noise floor
            sdft_[axis_].window_squared(data_);
            noise_threshold_ = 0.0f;
            for (int b = floor_start_; b <= end_bin_; b++) noise_threshold_ += data_[b];
            break;
        }
        case 1: {                                  // the `count` highest peaks, then sorted by frequency
            for (int p = 0; p < count_; p++) { peak_bin_[p] = 0; peak_val_[p] = 0.0f; }
            for (int b = start_bin_ + 1; b < end_bin_; b++) {
                if (data_[b] > data_[b - 1] && data_[b] > data_[b + 1]) {
                    for (int p = 0; p < count_; p++) {
                        if (data_[b] > peak_val_[p]) {
                            for (int k = count_ - 1; k > p; k--) { peak_bin_[k] = peak_bin_[k - 1]; peak_val_[k] = peak_val_[k - 1]; }
                            peak_bin_[p] = b; peak_val_[p] = data_[b];
                            break;
                        }
                    }
                    b++;                           // the bin after a peak cannot be one
                }
            }
            for (int p = count_ - 1; p > 0; p--)
                for (int k = 0; k < p; k++)
                    if (peak_bin_[k] > peak_bin_[k + 1] && peak_bin_[k + 1] != 0) {
                        const int tb = peak_bin_[k]; const float tv = peak_val_[k];
                        peak_bin_[k] = peak_bin_[k + 1]; peak_val_[k] = peak_val_[k + 1]; peak_bin_[k + 1] = tb; peak_val_[k + 1] = tv;
                    }
            break;
        }
        case 2: {                                  // noise floor without the peaks; centre frequencies follow the peaks above it
            int peaks = 0;
            for (int p = 0; p < count_; p++) {
                if (peak_bin_[p] >= floor_start_) {
                    noise_threshold_ -= 0.75f * data_[peak_bin_[p] - 1] + data_[peak_bin_[p]] + 0.75f * data_[peak_bin_[p] + 1];
                    peaks++;
                }
            }
            { const int bins = end_bin_ - floor_start_ - peaks + 1; noise_threshold_ /= (float)(bins > 0 ? bins : 1); }
            noise_threshold_ *= 2.0f;              // a peak must stand out twice above the floor
            for (int p = 0; p < count_; p++) {
                if (peak_bin_[p] == 0 || !(peak_val_[p] > noise_threshold_)) continue;
                float mean_bin = (float)peak_bin_[p];
                const float y0 = data_[peak_bin_[p] - 1], y1 = data_[peak_bin_[p]], y2 = data_[peak_bin_[p] + 1];
                const float denom = 2.0f * (y0 - 2.0f * y1 + y2);                          // vertex of the parabola through the three bins
                if (denom != 0.0f) mean_bin += (y0 - y2) / denom;
                const float f = clampf(mean_bin * resolution_, min_hz_, max_hz_);
                const float mult = clampf(peak_val_[p] / noise_threshold_, 1.0f, 10.0f);
                center_[axis_][p] += pt1_gain(kSmoothHz * mult, calc_period_s_) * (f - center_[axis_][p]);
            }
            break;
        }
        default: {                                 // move the notches that were updated
            for (int p = 0; p < count_; p++)
                if (peak_bin_[p] != 0 && peak_val_[p] > noise_threshold_) notch_[axis_][p].set(center_[axis_][p], dt_, q_);
            axis_ = (axis_ + 1) % 3;
            break;
        }
        }
        step_ = (step_ + 1) % 4;
    }

    int count_ = 0, sample_count_ = 1, sample_index_ = 0, start_bin_ = 1, end_bin_ = kSdftBins - 1, floor_start_ = 1, tick_ = 0, step_ = 0, axis_ = 0;
    float dt_ = 0.00025f, q_ = 3.0f, min_hz_ = 100.0f, max_hz_ = 600.0f, sample_rate_ = 1333.0f, resolution_ = 18.5f, calc_period_s_ = 0.003f, noise_threshold_ = 0.0f;
    float acc_[3] = {}, avg_[3] = {}, data_[kSdftBins] = {}, center_[3][kDynNotchMax] = {}, peak_val_[kDynNotchMax] = {};
    int peak_bin_[kDynNotchMax] = {};
    Sdft sdft_[3]; SvfNotch notch_[3][kDynNotchMax];
};
} // namespace fc
