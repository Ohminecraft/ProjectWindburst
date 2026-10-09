/*
 * This file is part of Windburst Project
 */

#pragma once
#include "../../config/fast_code.h"

#include <stdint.h>
#include <math.h>
#include "acc_gyro_types.h"

// Portable (no Arduino) helpers used by IMUAccGyro; kept separate so the PC tests can run them.

typedef enum
{
    NONE,
    MPU6050_I2C,
    MPU6500_I2C,
    MPU6000_SPI,
    MPU6500_SPI,
    MPU9520_SPI,
    ICM20601_SPI,
    ICM20602_SPI,
    ICM20602_FAKE_SPI,
    ICM42622P_SPI,
    ICM42686P_SPI,
    ICM42688P_SPI,
    ICM45605_SPI,
    ICM45686_SPI,
    BMI160_SPI,
    BMI270_SPI,
    ACCGYRO_DEVICE_COUNT

} IMUAccGyroDevice;

// Mounting of the chip relative to the board: Betaflight's eight orientations (sensor_align_e), with Betaflight's meaning.
//   CW0   = the chip's own X axis points to the nose, its Y axis to the LEFT, its Z axis UP (out of the top of the package):
//           the way the datasheet draws the axes when the chip lies face up with its X towards the nose
//   CW90  = that chip turned 90 degrees clockwise seen from above (its X now points to the right) ... CW180, CW270
//   FLIP  = the chip is on the underside of the board (turned over about the left-right axis), then turned as above
// A setting that is right in Betaflight for a board is right here.
enum AccGyroAlign : uint8_t { CW0_DEG, CW90_DEG, CW180_DEG, CW270_DEG, CW0_DEG_FLIP, CW90_DEG_FLIP, CW180_DEG_FLIP, CW270_DEG_FLIP };

namespace accgyro {

WB_FAST inline void align_discrete(float v[3], AccGyroAlign a) {
    const float x = v[0], y = v[1], z = v[2];
    switch (a) {
    default:
    case CW0_DEG:        break;
    case CW90_DEG:       v[0] = y;  v[1] = -x; v[2] = z;  break;
    case CW180_DEG:      v[0] = -x; v[1] = -y; v[2] = z;  break;
    case CW270_DEG:      v[0] = -y; v[1] = x;  v[2] = z;  break;
    case CW0_DEG_FLIP:   v[0] = -x; v[1] = y;  v[2] = -z; break;
    case CW90_DEG_FLIP:  v[0] = y;  v[1] = x;  v[2] = -z; break;
    case CW180_DEG_FLIP: v[0] = x;  v[1] = -y; v[2] = -z; break;
    case CW270_DEG_FLIP: v[0] = -y; v[1] = -x; v[2] = -z; break;
    }
}

// Betaflight's board frame - the frame its table above produces - is x forward, y LEFT, z UP: a board at rest reads +1 g on z.
// This firmware's frame is x forward, y RIGHT, z DOWN (the same board at rest reads -1 g on z). Up to firmware 0.5 the table's
// result was taken for the second frame directly, which made "CW0" mean a chip lying on its back.
inline void flu_to_frd(float v[3]) { v[1] = -v[1]; v[2] = -v[2]; }
// Chip axes -> this firmware's board frame, for a chip mounted the way Betaflight calls `a`.
WB_FAST inline void align_chip(float v[3], AccGyroAlign a) { align_discrete(v, a); flu_to_frd(v); }

// Fine board alignment (a few degrees, in THIS firmware's frame, applied after the orientation above): the matrix is built as
// Betaflight's buildRotationMatrix + applyRotationMatrix (which multiplies by the transpose). The level trim uses the same
// rotation; the signs are this firmware's own, they are not meant to be copied from a Betaflight setup.
inline void build_rotation(float m[3][3], float roll_deg, float pitch_deg, float yaw_deg) {
    const float d2r = 0.017453292519943295f;
    const float sx = sinf(roll_deg * d2r), cx = cosf(roll_deg * d2r);
    const float sy = sinf(pitch_deg * d2r), cy = cosf(pitch_deg * d2r);
    const float sz = sinf(yaw_deg * d2r), cz = cosf(yaw_deg * d2r);
    m[0][0] = cz * cy;                m[0][1] = -cy * sz;               m[0][2] = sy;
    m[1][0] = sz * cx + sx * cz * sy; m[1][1] = cz * cx - sx * sz * sy; m[1][2] = -sx * cy;
    m[2][0] = sx * sz - cz * cx * sy; m[2][1] = sx * cz + sz * cx * sy; m[2][2] = cy * cx;
}
WB_FAST inline void apply_rotation(float v[3], const float m[3][3]) {
    const float x = v[0], y = v[1], z = v[2];
    v[0] = m[0][0] * x + m[1][0] * y + m[2][0] * z;
    v[1] = m[0][1] * x + m[1][1] * y + m[2][1] * z;
    v[2] = m[0][2] * x + m[1][2] * y + m[2][2] * z;
}

// Identify the chip from raw ID reads. r00_dummy_second = byte 2 of a 2-byte read of reg 0x00 (BMI270 has a dummy byte first),
// r00_first = a plain 1-byte read of reg 0x00 (BMI160), r72 = reg 0x72 (ICM456xx), r75 = reg 0x75 (MPU family / ICM426xx).
inline IMUAccGyroDevice classify_spi(uint8_t r00_dummy_second, uint8_t r00_first, uint8_t r72, uint8_t r75) {
    if (r00_dummy_second == BMI270_WHO_AM_I) return BMI270_SPI;
    if (r00_first == BMI160_WHO_AM_I) return BMI160_SPI;
    if (r72 == ICM45686_WHO_AM_I) return ICM45686_SPI;
    if (r72 == ICM45605_WHO_AM_I) return ICM45605_SPI;
    switch (r75) {
    case MPU6050_WHO_I_AM:   return MPU6000_SPI;                      // 0x68 (SPI => MPU6000)
    case MPU6500_WHO_I_AM:   return MPU6500_SPI;
    case MPU9250_WHO_I_AM:
    case MPU9255_WHO_I_AM:   return MPU9520_SPI;
    case ICM20601_WHO_AM_I:  return ICM20601_SPI;
    case ICM20602_WHO_AM_I:  return ICM20602_SPI;
    case ICM20602_FAKE_WHO_AM_I: return ICM20602_FAKE_SPI;
    case ICM42622P_WHO_AM_I: return ICM42622P_SPI;
    case ICM42686P_WHO_AM_I: return ICM42686P_SPI;
    case ICM42688P_WHO_AM_I: return ICM42688P_SPI;
    default:                 return NONE;
    }
}
inline IMUAccGyroDevice classify_i2c(uint8_t who) {
    if (who == MPU6050_WHO_I_AM) return MPU6050_I2C;
    if (who == MPU6500_WHO_I_AM) return MPU6500_I2C;
    return NONE;
}

// Gyro bias from running sums. false if too few samples, offset > 30 dps or noise > 2 dps rms (board is not still).
inline bool bias_from_sums(const double s[3], const double s2[3], uint16_t ok, uint16_t wanted, float out[3]) {
    if (ok < wanted / 2 || ok == 0) return false;
    float mean[3];
    for (int k = 0; k < 3; k++) {
        mean[k] = (float)(s[k] / ok);
        const double var = s2[k] / ok - (double)mean[k] * mean[k];
        if (fabs(mean[k]) > 30.0 || var > 4.0) return false;
    }
    for (int k = 0; k < 3; k++) out[k] = mean[k];
    return true;
}

} // namespace accgyro
