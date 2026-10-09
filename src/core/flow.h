#pragma once
#include "types.h"

// ============================================================================
// Optical flow -> velocity over the ground.
//
// A flow sensor looks straight down and reports how fast the picture moves, as an angular rate about its own X and Y axes
// (rad/s; a right-handed rotation of the SENSOR about an axis reads positive - ArduPilot's convention, AP_OpticalFlow).
// The picture moves for two reasons: the craft turns (roll / pitch rate) and the craft travels. Taking the gyro's part away
// leaves the travel, and the distance to the ground turns that angle into metres:
//
//      v_forward = +range x (flow_y - gyro_y)            v_right = -range x (flow_x - gyro_x)          range = height / cos(tilt)
//
// The gyro must cover the SAME stretch of time as the flow sample (sensors report 10 .. 65 ms late and integrate over 10 .. 100 ms):
// GyroHistory keeps the last quarter second of roll / pitch rates so that exactly that window can be averaged.
//
// A sample is not used when the picture cannot be trusted: surface quality too low, too close to the ground or too far from it
// (out of focus / no texture), tilted so far that the beam of the range sensor and the camera no longer look at the same spot,
// or turning so fast that the gyro part dwarfs the travel. Not used means NOT FUSED - never "velocity zero".
// ============================================================================
namespace fc {

struct FlowConfig {
    float scale = 1.0f;            // correction of the sensor's own scale (lens): 1 = as the driver reports
    float yaw_deg = 0.0f;          // the sensor's X axis points this far to the right of the craft's nose (ArduPilot FLOW_ORIENT_YAW)
    bool flip = false;             // the picture is mirrored: the flow about the sensor's X axis has the opposite sign
    uint8_t quality_min = 30;      // 0..100
    float min_height_m = 0.10f, max_height_m = 4.0f;
    float max_gyro_dps = 200.0f;   // roll / pitch rate above which a sample is skipped
    float min_up_cos = 0.71f;      // tilt limit (45 deg)
    float noise_rad = 0.10f;       // rad/s of flow noise at full quality: velocity sigma = noise x range (at least 0.10 m/s)
};

struct FlowSample {
    float x = 0, y = 0;            // rad/s about the sensor's X / Y axis, averaged over dt
    float dt = 0.02f;              // length of the stretch of time it covers (s)
    float age_s = 0.0f;            // how long ago that stretch ENDED when the sample was handed over
    uint8_t quality = 0;           // 0..100
};

// Roll / pitch rates (board frame FRD, rad/s) of the last kSlots x step seconds.
class GyroHistory {
public:
    static constexpr int kSlots = 128;
    void configure(float step_s) { step_ = step_s > 1e-5f ? step_s : 0.002f; reset(); }
    void reset() { head_ = 0; count_ = 0; }
    WB_FAST void push(float gx_rad, float gy_rad) { x_[head_] = gx_rad; y_[head_] = gy_rad; head_ = (head_ + 1) % kSlots; if (count_ < kSlots) count_++; }
    float span_s() const { return count_ * step_; }
    // Mean rate over the window that ended `age_s` ago and lasted `dt_s`. false: the history does not reach back that far.
    WB_FAST bool average(float age_s, float dt_s, float& gx, float& gy) const {
        if (age_s < 0.0f) age_s = 0.0f;
        int first = (int)(age_s / step_ + 0.5f), n = (int)(dt_s / step_ + 0.5f);
        if (n < 1) n = 1;
        if (first + n > count_) return false;
        float sx = 0, sy = 0;
        for (int i = 0; i < n; i++) { const int k = (head_ - 1 - first - i + 2 * kSlots) % kSlots; sx += x_[k]; sy += y_[k]; }
        gx = sx / (float)n; gy = sy / (float)n;
        return true;
    }
private:
    float x_[kSlots] = {}, y_[kSlots] = {}, step_ = 0.002f; int head_ = 0, count_ = 0;
};

struct FlowVelocity {
    bool valid = false;
    float forward = 0, right = 0;  // m/s over the ground, along the craft's own forward / right axes (in the plane of the board)
    float sigma = 0;               // m/s
    uint8_t reject = 0;            // why not valid: FlowReject
};
enum FlowReject : uint8_t { FLOW_OK, FLOW_QUALITY, FLOW_HEIGHT, FLOW_TILT, FLOW_TURNING, FLOW_NO_GYRO, FLOW_NO_SAMPLE };      // NO_SAMPLE: there is nothing to judge (no sensor, or it went silent)

// height_m: height above the ground (already tilt-corrected, as the range sensor's manager gives it). gyro: mean roll / pitch rate
// (FRD, rad/s) over the sample's own window.
inline WB_FAST FlowVelocity flow_velocity(const FlowConfig& c, const FlowSample& s, float gyro_x, float gyro_y, float height_m, float up_cos) {
    FlowVelocity o;
    if (s.quality < c.quality_min) { o.reject = FLOW_QUALITY; return o; }
    if (!(height_m >= c.min_height_m && height_m <= c.max_height_m)) { o.reject = FLOW_HEIGHT; return o; }
    if (up_cos < c.min_up_cos) { o.reject = FLOW_TILT; return o; }
    const float lim = c.max_gyro_dps * kDeg2Rad;
    if (fabsf(gyro_x) > lim || fabsf(gyro_y) > lim) { o.reject = FLOW_TURNING; return o; }
    // the sensor's axes -> the craft's (only the flow is turned: the gyro is already in the craft's axes)
    const float yr = c.yaw_deg * kDeg2Rad, cy = cosf(yr), sy = sinf(yr);
    const float sx = c.flip ? -s.x : s.x;
    const float fx = (sx * cy - s.y * sy) * c.scale, fy = (sx * sy + s.y * cy) * c.scale;
    const float range = height_m / up_cos;
    o.forward = range * (fy - gyro_y);
    o.right = -range * (fx - gyro_x);
    const float qn = clampf((float)(s.quality - c.quality_min) / (float)(100 - c.quality_min > 1 ? 100 - c.quality_min : 1), 0.05f, 1.0f);
    o.sigma = fmaxf(c.noise_rad * range, 0.10f) / sqrtf(qn);
    o.valid = true;
    return o;
}

} // namespace fc
