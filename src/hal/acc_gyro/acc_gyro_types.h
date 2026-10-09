/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>

#define MPU_WHO_AM_I_CMD        0x75

#define MPU6050_WHO_I_AM    0x68
#define MPU6500_WHO_I_AM    0x70
#define MPU9250_WHO_I_AM    0x71
#define MPU9255_WHO_I_AM    0x73
#define ICM20601_WHO_AM_I   0xAC
#define ICM20602_WHO_AM_I   0x12
#define ICM20602_FAKE_WHO_AM_I 0x2E
#define ICM42605_WHO_AM_I   0x42
#define ICM42622P_WHO_AM_I  0x46
#define ICM42686P_WHO_AM_I  0x44
#define ICM42688P_WHO_AM_I  0x47
#define ICM45686_WHO_AM_I   0xE9
#define ICM45605_WHO_AM_I   0xE5
#define BMI160_WHO_AM_I     0xD1
#define BMI270_WHO_AM_I     0x24

struct IMUAccGyroRawData
{
    float accel_x_raw = 0.0f;
    float accel_y_raw = 0.0f;
    float accel_z_raw = 0.0f;
    float gyro_x_raw = 0.0f;
    float gyro_y_raw = 0.0f;
    float gyro_z_raw = 0.0f;
};

enum GyroFsrConfig {
    INV_FSR_250DPS = 0,
    INV_FSR_500DPS,
    INV_FSR_1000DPS,
    INV_FSR_2000DPS,
    INV_FSR_4000DPS,
    NUM_GYRO_FSR
};

enum ICMHighRangeGyroFsrConfig {
    ICM_HIGH_RANGE_FSR_500DPS = 0,
    ICM_HIGH_RANGE_FSR_1000DPS,
    ICM_HIGH_RANGE_FSR_2000DPS,
    ICM_HIGH_RANGE_FSR_4000DPS,
    NUM_ICM_HIGH_RANGE_GYRO_FSR
};

enum ClockSelConfig {
    INV_CLK_INTERNAL = 0,
    INV_CLK_PLL,
    NUM_CLK
};

enum AccelFsrConfig {
    INV_FSR_2G = 0,
    INV_FSR_4G,
    INV_FSR_8G,
    INV_FSR_16G,
    INV_FSR_32G,
    NUM_ACCEL_FSR
};

enum ICMHighRangeAccelFsrConfig {
    ICM_HIGH_RANGE_FSR_4G = 0,
    ICM_HIGH_RANGE_FSR_8G,
    ICM_HIGH_RANGE_FSR_16G,
    ICM_HIGH_RANGE_FSR_32G,
    NUM_ICM_HIGH_RANGE_ACCEL_FSR
};