#ifndef IMU_SERVICE_H
#define IMU_SERVICE_H

#include "bmi088.h"

#define IMU_SERVICE_POLL_PERIOD_MS 10u

/*
 * Latest BMI088 sample. Each three-element axis array is ordered X, Y, Z.
 * Raw arrays contain signed sensor counts; converted arrays contain SI units.
 * This is an internal service structure, not a serialized USB payload.
 */
typedef struct {
    int16_t accel_raw[3];       /* X, Y, Z accelerometer counts */
    int16_t gyro_raw[3];        /* X, Y, Z gyroscope counts */
    float accel_mps2[3];        /* X, Y, Z acceleration in m/s^2 */
    float gyro_rps[3];          /* X, Y, Z angular velocity in rad/s */
    uint32_t timestamp_ms;      /* HAL tick when this sample was completed */
    uint32_t sample_count;      /* Number of successful paired samples */
    uint32_t poll_error_count;  /* Failed sensor reads since initialization */
    HAL_StatusTypeDef last_status;
    uint8_t initialized;        /* Both BMI088 sensor dies initialized successfully */
    uint8_t sample_valid;       /* At least one successful sample is available */
} ImuSample;

/* Initialize the BMI088 service and retain the latest sample/status. */
HAL_StatusTypeDef ImuService_Init(I2C_HandleTypeDef *i2c);

/* Read accelerometer and gyroscope once; a failed read keeps the last good sample. */
HAL_StatusTypeDef ImuService_Poll(void);

/* Return a read-only view of the latest sample and service diagnostics. */
const ImuSample *ImuService_GetLatestSample(void);

#endif /* IMU_SERVICE_H */
