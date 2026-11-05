/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>
#include <math.h>
#include "../hal/acc_gyro/acc_gyro_util.h"

// ============================================================================
// Board alignment in Betaflight's terms (MSP_BOARD_ALIGNMENT_CONFIG, the Betaflight App's board alignment wizard) <-> this
// firmware's chip orientation (general.board_align, Betaflight's eight sensor_align values) + board trims (general.board_trim_*).
//
// Betaflight (the App's model, test/js/boardAlignment.test.ts): out = E(roll, pitch, -yaw) * sensor-aligned, in its frame
// x forward, y LEFT, z UP; E = Rz * Ry * Rx (magAlignment.ts eulerToMatrix), yaw clockwise positive.
// Here (IMUAccGyro::read): out_frd = Rb^T * F * D * chip - D the chip orientation (Betaflight's table), F = diag(1, -1, -1) to
// this firmware's frame (x forward, y right, z down), Rb = accgyro::build_rotation(trims) (apply_rotation multiplies by Rb^T).
// In Betaflight's frame: out = F * out_frd = (F Rb^T F) * D * chip, so E = F Rb^T F.
//
// A Betaflight angle set may need another chip orientation (the wizard snaps to 45 deg: a board turned 90 deg, upside down):
// from_bf() keeps the total rotation and tries the eight orientations (the current one first) for trims inside their limits
// (roll / pitch +-45 deg, yaw +-180 deg). None fits (a board standing on its edge): false, nothing changes.
// Portable: unit-tested in test/test_msp_config.cpp (t_board_wizard: the App's wizard ported, boards mounted many ways).
// ============================================================================
namespace fc { namespace balign {

typedef float M3[3][3];
constexpr float kTrimLimitDeg = 45.0f;

// Betaflight's chip orientation table as a matrix (columns: where each chip axis goes)
inline void discrete(M3 m, uint8_t a) {
    for (int i = 0; i < 3; i++) { float e[3] = {0, 0, 0}; e[i] = 1; accgyro::align_discrete(e, (AccGyroAlign)a); for (int k = 0; k < 3; k++) m[k][i] = e[k]; }
}
inline void mul(M3 o, const M3 a, const M3 b) { M3 t; for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) t[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j]; for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) o[i][j] = t[i][j]; }
inline float deg(float rad) { return rad * 57.29577951308232f; }
inline const float* flu() { static const float f[3] = {1, -1, -1}; return f; }

// E(roll, pitch, yaw counter-clockwise) - magAlignment.ts eulerToMatrix
inline void euler(M3 m, float r, float p, float y) {
    const float d = 0.017453292519943295f, cr = cosf(r * d), sr = sinf(r * d), cp = cosf(p * d), sp = sinf(p * d), cy = cosf(y * d), sy = sinf(y * d);
    m[0][0] = cy * cp; m[0][1] = cy * sp * sr - sy * cr; m[0][2] = cy * sp * cr + sy * sr;
    m[1][0] = sy * cp; m[1][1] = sy * sp * sr + cy * cr; m[1][2] = sy * sp * cr - cy * sr;
    m[2][0] = -sp;     m[2][1] = cp * sr;                m[2][2] = cp * cr;
}

// The chip orientation + trims (deg) -> Betaflight's board alignment of the whole mounting, the chip taken as CW0 (deg: roll,
// pitch, yaw clockwise): the App shows how the board sits (a chip turned 90 deg reads yaw 90), as a Betaflight board would.
inline void bf_of(uint8_t align, float tr, float tp, float ty, float out[3]) {
    M3 b; accgyro::build_rotation(b, tr, tp, ty); const float* f = flu();
    M3 e; for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) e[i][j] = f[i] * b[j][i] * f[j];                   // E = F Rb^T F
    M3 d; discrete(d, align); mul(e, e, d);                                                                             // the whole mounting
    const float sp = e[2][0] < -1 ? 1 : e[2][0] > 1 ? -1 : -e[2][0];
    out[0] = deg(atan2f(e[2][1], e[2][2])); out[1] = deg(asinf(sp)); out[2] = -deg(atan2f(e[1][0], e[0][0]));
}

// E (Betaflight's frame) -> the trims that make it (deg); Rb = (F E F)^T
inline void trims_of(const M3 e, float t[3]) {
    const float* f = flu(); M3 b; for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) b[i][j] = f[j] * e[j][i] * f[i];
    const float sy = b[0][2] < -1 ? -1 : b[0][2] > 1 ? 1 : b[0][2];
    t[0] = deg(atan2f(-b[1][2], b[2][2])); t[1] = deg(asinf(sy)); t[2] = deg(atan2f(-b[0][1], b[0][0]));
}

// Betaflight's board alignment of the whole mounting (deg; yaw clockwise, any turn; the chip taken as CW0) -> a chip orientation
// and trims (deg) that give the same rotation. `cur`: the orientation now (it wins a tie). false: none fits the trim limits.
inline bool from_bf(float r, float p, float y, uint8_t cur, uint8_t& align, float trims[3]) {
    M3 tot; euler(tot, r, p, -y);                                                                                       // chip -> out
    bool found = false; float best = 1e9f;
    for (int k = 0; k < 8; k++) {
        const uint8_t a = (uint8_t)((cur + k) % 8);                                                                     // (the current one first)
        M3 da, dt, ea; discrete(da, a); for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) dt[i][j] = da[j][i];
        mul(ea, tot, dt);                                                                                               // E' = total * D(a)^T
        float t[3]; trims_of(ea, t);
        const float worst = fmaxf(fabsf(t[0]), fabsf(t[1]));
        // the smallest trims win: roll / pitch first, then yaw (a chip turned 90 deg is its orientation, not a 90 deg yaw trim -
        // and the level trims, added to the same rotation, keep their meaning); the current orientation wins a tie
        const float score = worst * 1000.0f + fabsf(t[2]);
        if (worst <= kTrimLimitDeg + 0.05f && score < best - 0.01f) { best = score; found = true; align = a; for (int i = 0; i < 3; i++) trims[i] = t[i]; }
    }
    return found;
}

// The Betaflight App's board alignment wizard (src/js/utils/boardAlignment.ts, detectBoardAlignment), here for CMD_BOARD_ALIGN.
// flat / nose / right: gravity (g, Betaflight's frame x forward, y left, z up - as MSP_RAW_IMU) lying flat, nose down, right side
// down - read through the mounting now in use, whose Betaflight angles are `cur` (deg, yaw clockwise). The horizontal part of
// gravity points to the raised end: nose down -> it points aft. out: the whole mounting, snapped to 45 deg (roll, pitch in
// (-180, 180], yaw clockwise 0..359). The App's yaw gesture (a confidence grade only) is not taken.
// The results are wbp's board_align_result (OK, NOT_STILL, NO_GRAVITY, NO_PITCH, NO_ROLL, DIRECTION, NO_FIT).
enum Detect : uint8_t { DET_OK = 0, DET_NOT_STILL = 1, DET_NO_GRAVITY = 2, DET_NO_PITCH = 3, DET_NO_ROLL = 4, DET_DIRECTION = 5, DET_NO_FIT = 6 };
inline float snap45(float d) { float s = roundf(d / 45.0f) * 45.0f; while (s > 180.0f) s -= 360.0f; while (s <= -180.0f) s += 360.0f; return s == 0.0f ? 0.0f : s; }
inline uint8_t detect(const float flat[3], const float nose[3], const float right[3], const float cur[3], int out[3]) {
    const auto dot = [](const float* a, const float* b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; };
    const auto unit = [&](float* v) { const float m = sqrtf(dot(v, v)); if (m < 1e-9f) { v[0] = v[1] = v[2] = 0; return; } for (int k = 0; k < 3; k++) v[k] /= m; };
    const auto perp = [&](float* o, const float* v, const float* ax) { const float d = dot(v, ax); for (int k = 0; k < 3; k++) o[k] = v[k] - ax[k] * d; };
    float up[3] = {flat[0], flat[1], flat[2]}; unit(up);
    if (sqrtf(dot(up, up)) < 0.5f) return DET_NO_GRAVITY;
    float hp[3]; perp(hp, nose, up); if (sqrtf(dot(hp, hp)) < 0.05f) return DET_NO_PITCH;
    float fr[3] = {-hp[0], -hp[1], -hp[2]}; unit(fr);
    float hr[3]; perp(hr, right, up); if (sqrtf(dot(hr, hr)) < 0.05f) return DET_NO_ROLL;
    unit(hr);
    float fw[3]; perp(fw, fr, up); unit(fw);
    const float rt[3] = {up[1] * fw[2] - up[2] * fw[1], up[2] * fw[0] - up[0] * fw[2], up[0] * fw[1] - up[1] * fw[0]};     // up x forward: +Y (left)
    if (dot(rt, hr) < 0.0f) return DET_DIRECTION;                                       // one of the two gestures the wrong way (which: unknown)
    M3 m = {{fw[0], fw[1], fw[2]}, {rt[0], rt[1], rt[2]}, {up[0], up[1], up[2]}}, c;
    euler(c, cur[0], cur[1], -cur[2]); mul(m, m, c);                                    // the residual found on top of the mounting in use
    const float sp = m[2][0] < -1 ? 1 : m[2][0] > 1 ? -1 : -m[2][0], pitch = deg(asinf(sp));
    float roll, yaw;
    if (fabsf(m[2][0]) < 0.99999f) { roll = deg(atan2f(m[2][1], m[2][2])); yaw = deg(atan2f(m[1][0], m[0][0])); }
    else { roll = deg(atan2f(-m[0][1], m[1][1])); yaw = 0.0f; }                         // (pitch 90: all of it is roll)
    out[0] = (int)snap45(roll); out[1] = (int)snap45(pitch); out[2] = ((-(int)snap45(yaw) % 360) + 360) % 360;
    return DET_OK;
}

// A second chip on the same board turned along with the first: the orientation b' with D(b') = D(a_new) D(a_old)^T D(b).
// (The eight orientations are a group: there always is one.)
inline uint8_t follow(uint8_t a_new, uint8_t a_old, uint8_t b) {
    M3 dn, dob, db, t; discrete(dn, a_new); discrete(dob, a_old); discrete(db, b);
    M3 dt; for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) dt[i][j] = dob[j][i];
    mul(t, dn, dt); mul(t, t, db);
    for (uint8_t k = 0; k < 8; k++) {
        M3 dk; discrete(dk, k); float e = 0; for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) e = fmaxf(e, fabsf(dk[i][j] - t[i][j]));
        if (e < 1e-3f) return k;
    }
    return b;
}

}} // namespace fc::balign
