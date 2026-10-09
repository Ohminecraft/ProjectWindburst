#pragma once
#include "../config/fast_code.h"
#include "types.h"

// Motor mixer. Rows are derived from the frame GEOMETRY, not typed in:
//   motor at angle theta (clockwise from the nose, seen from above):   roll  = -sin(theta)   (right motors down => right wing down = +roll)
//                                                                      pitch = -cos(theta)   (front motors down => nose down = +pitch)
//                                                                      yaw   = +1 for a CCW prop (its reaction torque turns the frame clockwise)
//   each of roll / pitch is then normalised so its largest coefficient is 1.
// Motor numbering AND prop directions follow Betaflight's default ("props in") for the quads:
//   QUAD_X    1 rear-right CW, 2 front-right CCW, 3 rear-left CCW, 4 front-left CW
//   QUAD_PLUS 1 rear CW,       2 right CCW,       3 left CCW,      4 front CW
//   (Betaflight mixer_init.c lists yaw = -1, +1, +1, -1 for these motors and mixer.c negates the yaw PID sum, whose positive
//    direction is nose LEFT there. Here +yaw is nose RIGHT, so a CCW prop - reaction torque clockwise - gets +1: the same builds.)
// HEX_X: motors every 60 degrees starting at 30 degrees (front-right), clockwise: 1..6, alternating CCW/CW starting with CCW
//   (same directions per position as Betaflight's hex X; the numbering is this firmware's own).
// props_out = true is Betaflight's "motor direction reversed": every prop turns the other way, so every yaw sign flips.
// The other frames (0.8.2) are Betaflight's tables (mixer_init.c: roll, pitch, yaw per motor, motor order and prop directions as
// Betaflight's), each axis normalised the same way (largest |coefficient| = 1). Betaflight's roll / pitch columns have this
// firmware's signs already (+roll = right wing down, +pitch = nose down, yaw +1 = CCW prop); test_core checks every geometric
// frame against the physical plant built from motor positions:
//   HEX_PLUS  (HEX6)       1 rear-right, 2 front-right, 3 rear-left, 4 front-left, 5 front, 6 rear
//   HEX_X_BF  (HEX6X)      1 rear-right, 2 front-right, 3 rear-left, 4 front-left, 5 right, 6 left
//   HEX_H     (HEX6H)      1 rear-right, 2 front-right, 3 rear-left, 4 front-left, 5 right, 6 left (5, 6: lift only)
//   Y6                     top 1 rear, 2 right, 3 left; under 4 rear, 5 right, 6 left
//   Y4                     1 rear top (CW), 2 front-right (CCW), 3 rear bottom (CCW), 4 front-left (CW): yaw from the rear pair
//   OCTO_X8                top 1 rear-right, 2 front-right, 3 rear-left, 4 front-left; under 5..8 the same places
//   OCTO_FLAT_PLUS / _X    Betaflight OCTOFLATP / OCTOFLATX order
//   VTAIL4 / ATAIL4        quad with tilted rear motors (Betaflight's numbers, not derived from a geometry)
//   QUAD_X_1234            quad X numbered 1 front-left, 2 front-right, 3 rear-right, 4 rear-left (esp-fc / Betaflight "1234")
// DShot / ProShot drive at most 4 motors on the ESP32-S3: the 6- and 8-motor frames need PWM / Oneshot / Multishot.
//
// The mixer outputs ABSOLUTE 0..1 values already inside [idle, limit]. idle and limit are owned by the motor layer
// (MotorOutputManager: static + dynamic idle, output limit) and handed in every call - there is no second copy of them here.
// "reversed" motors (spin direction) are an ESC setting handled by the output layer and never change the mix; props_out flips every yaw sign.
namespace fc {
enum Layout : uint8_t { QUAD_X, QUAD_PLUS, HEX_X, HEX_PLUS, HEX_X_BF, HEX_H, Y6, Y4, OCTO_X8, OCTO_FLAT_PLUS, OCTO_FLAT_X, VTAIL4, ATAIL4, QUAD_X_1234, LAYOUT_COUNT };

// Betaflight's tables (roll, pitch, yaw) for the frames after HEX_X. n = motor count.
struct LayoutTable { uint8_t n; float r[8][3]; };
inline const LayoutTable& layout_table(Layout l) {
    static const LayoutTable t[LAYOUT_COUNT - 3] = {
        {6, {{-0.866025f, 0.5f, 1}, {-0.866025f, -0.5f, -1}, {0.866025f, 0.5f, 1}, {0.866025f, -0.5f, -1}, {0, -1, 1}, {0, 1, -1}}},                   // HEX6
        {6, {{-0.5f, 0.866025f, 1}, {-0.5f, -0.866025f, 1}, {0.5f, 0.866025f, -1}, {0.5f, -0.866025f, -1}, {-1, 0, -1}, {1, 0, 1}}},                   // HEX6X
        {6, {{-1, 1, -1}, {-1, -1, 1}, {1, 1, 1}, {1, -1, -1}, {0, 0, 0}, {0, 0, 0}}},                                                                // HEX6H
        {6, {{0, 1.333333f, 1}, {-1, -0.666667f, -1}, {1, -0.666667f, -1}, {0, 1.333333f, -1}, {-1, -0.666667f, 1}, {1, -0.666667f, 1}}},             // Y6
        {4, {{0, 1, -1}, {-1, -1, 0}, {0, 1, 1}, {1, -1, 0}}},                                                                                         // Y4
        {8, {{-1, 1, -1}, {-1, -1, 1}, {1, 1, 1}, {1, -1, -1}, {-1, 1, 1}, {-1, -1, -1}, {1, 1, -1}, {1, -1, 1}}},                                   // OCTOX8
        {8, {{0.707107f, -0.707107f, 1}, {-0.707107f, -0.707107f, 1}, {-0.707107f, 0.707107f, 1}, {0.707107f, 0.707107f, 1},
             {0, -1, -1}, {-1, 0, -1}, {0, 1, -1}, {1, 0, -1}}},                                                                                       // OCTOFLATP
        {8, {{1, -0.414178f, 1}, {-0.414178f, -1, 1}, {-1, 0.414178f, 1}, {0.414178f, 1, 1},
             {0.414178f, -1, -1}, {-1, -0.414178f, -1}, {-0.414178f, 1, -1}, {1, 0.414178f, -1}}},                                                     // OCTOFLATX
        {4, {{-0.58f, 0.58f, 1}, {-0.46f, -0.39f, -0.5f}, {0.58f, 0.58f, -1}, {0.46f, -0.39f, 0.5f}}},                                                 // VTAIL4
        {4, {{0, 1, 1}, {-1, -1, 0}, {0, 1, -1}, {1, -1, 0}}},                                                                                         // ATAIL4
        {4, {{1, -1, -1}, {-1, -1, 1}, {-1, 1, -1}, {1, 1, 1}}},                                                                                       // QUAD X 1234
    };
    return t[l - 3];
}

struct MixerConfig {
    Layout layout = QUAD_X;
    bool props_out = false;
};
struct MotorLimits { float idle = 0.055f, limit = 1.0f; };
struct MixRow { float roll, pitch, yaw; };

inline int mixer_motor_count(Layout l) {
    if (l >= LAYOUT_COUNT) return 4;
    return l == HEX_X ? 6 : l < HEX_X ? 4 : layout_table(l).n;
}

inline MixRow mixer_row(Layout l, int m) {
    if (l > HEX_X && l < LAYOUT_COUNT) {                // Betaflight's table, each axis scaled so that its largest |coefficient| is 1
        const LayoutTable& t = layout_table(l);
        float mx[3] = {0, 0, 0};
        for (int i = 0; i < t.n; i++) for (int k = 0; k < 3; k++) mx[k] = fmaxf(mx[k], fabsf(t.r[i][k]));
        MixRow r;
        r.roll = mx[0] > 0 ? t.r[m][0] / mx[0] : 0.0f; r.pitch = mx[1] > 0 ? t.r[m][1] / mx[1] : 0.0f; r.yaw = mx[2] > 0 ? t.r[m][2] / mx[2] : 0.0f;
        return r;
    }
    if (l >= LAYOUT_COUNT) l = QUAD_X;
    static const float quad_x[4] = {135, 45, 225, 315}, quad_p[4] = {180, 90, 270, 0};
    static const int8_t quad_x_spin[4] = {-1, +1, +1, -1}, quad_p_spin[4] = {-1, +1, +1, -1};     // +1 = CCW prop
    float th = 0; int spin = 1;
    switch (l) {
    case QUAD_X: th = quad_x[m]; spin = quad_x_spin[m]; break;
    case QUAD_PLUS: th = quad_p[m]; spin = quad_p_spin[m]; break;
    default: th = 30.0f + 60.0f * (float)m; spin = (m % 2 == 0) ? +1 : -1; break;
    }
    const float t = th * kDeg2Rad;
    float rmax = 0, pmax = 0;                         // normalisation: largest |coefficient| over the layout
    const int n = mixer_motor_count(l);
    for (int i = 0; i < n; i++) {
        const float ti = (l == QUAD_X ? quad_x[i] : l == QUAD_PLUS ? quad_p[i] : 30.0f + 60.0f * (float)i) * kDeg2Rad;
        rmax = fmaxf(rmax, fabsf(sinf(ti))); pmax = fmaxf(pmax, fabsf(cosf(ti)));
    }
    MixRow r;
    r.roll = -sinf(t) / rmax; r.pitch = -cosf(t) / pmax; r.yaw = (float)spin;
    if (fabsf(r.roll) < 1e-4f) r.roll = 0.0f;
    if (fabsf(r.pitch) < 1e-4f) r.pitch = 0.0f;
    return r;
}

// Mixer with the rows computed once in configure() (the trig is not repeated at 4 kHz).
class Mixer {
public:
    MixerConfig cfg;
    void configure() { n_ = mixer_motor_count(cfg.layout); for (int i = 0; i < n_; i++) row_[i] = mixer_row(cfg.layout, i); }
    int motor_count() const { return n_; }
    // thr 0..1 (stick/controller), roll/pitch/yaw = authority as a fraction of the usable motor range [idle, limit] (the PID sum / 1000,
    // as in Betaflight's mixer: motor = min + range * (throttle + mix)). out[] = absolute 0..1.
    // The pattern always keeps its full authority: at zero throttle it is slid up until no motor is below idle (Betaflight's mixer
    // with airmode). Whether a craft with the throttle down is stabilised at all is the caller's decision - ground_idle = every
    // motor at idle, nothing mixed (standing on the ground, or airmode switched off: ArduCopter's GROUND_IDLE).
    WB_FAST void mix(const MotorLimits& lim, float thr, float r, float p, float y, float out[kMaxMotors], bool ground_idle = false) const {
        float raw[kMaxMotors], lo = 1e9f, hi = -1e9f;
        const float ys = cfg.props_out ? -1.0f : 1.0f;
        const float top = lim.limit, bot = lim.idle, avail = top - bot;
        for (int i = 0; i < n_; i++) {
            raw[i] = (r * row_[i].roll + p * row_[i].pitch + y * row_[i].yaw * ys) * avail;
            lo = fminf(lo, raw[i]); hi = fmaxf(hi, raw[i]);
        }
        if (ground_idle) { for (int i = 0; i < n_; i++) out[i] = bot; for (int i = n_; i < kMaxMotors; i++) out[i] = 0.0f; return; }
        const float range = hi - lo;
        if (range > avail && range > 1e-6f) { const float k = avail / range; for (int i = 0; i < n_; i++) raw[i] *= k; lo *= k; hi *= k; }   // desaturate: scale ALL axes equally
        const float t = bot + clampf(thr, 0.0f, 1.0f) * avail;
        const float shift = clampf(t, bot - lo, top - hi);          // slide the whole pattern so nothing clips
        for (int i = 0; i < n_; i++) out[i] = clampf(raw[i] + shift, 0.0f, top);
        for (int i = n_; i < kMaxMotors; i++) out[i] = 0.0f;
    }
private:
    MixRow row_[kMaxMotors]; int n_ = 4;
};
} // namespace fc
