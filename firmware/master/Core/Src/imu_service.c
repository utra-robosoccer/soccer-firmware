#include "imu_service.h"

#include <string.h>

static BMI088 bmi088;
static ImuSample latest_sample;

HAL_StatusTypeDef ImuService_Init(I2C_HandleTypeDef *i2c)
{
    memset(&latest_sample, 0, sizeof(latest_sample));
    latest_sample.last_status = BMI088_Init(&bmi088, i2c);
    latest_sample.initialized = (uint8_t)(latest_sample.last_status == HAL_OK);
    return latest_sample.last_status;
}

HAL_StatusTypeDef ImuService_Poll(void)
{
    BMI088 reading = bmi088;
    int16_t accel_raw[3];
    int16_t gyro_raw[3];
    HAL_StatusTypeDef status;

    if (latest_sample.initialized == 0u) {
        return latest_sample.last_status;
    }

    status = BMI088_ReadAccelerometer(&reading,
                                      &accel_raw[0], &accel_raw[1], &accel_raw[2]);
    if (status == HAL_OK) {
        status = BMI088_ReadGyroscope(&reading,
                                      &gyro_raw[0], &gyro_raw[1], &gyro_raw[2]);
    }

    latest_sample.last_status = status;
    if (status != HAL_OK) {
        latest_sample.poll_error_count++;
        return status;
    }

    memcpy(latest_sample.accel_raw, accel_raw, sizeof(accel_raw));
    memcpy(latest_sample.gyro_raw, gyro_raw, sizeof(gyro_raw));
    memcpy(latest_sample.accel_mps2, reading.accel_mps2,
           sizeof(latest_sample.accel_mps2));
    memcpy(latest_sample.gyro_rps, reading.gyro_rps,
           sizeof(latest_sample.gyro_rps));
    latest_sample.timestamp_ms = HAL_GetTick();
    latest_sample.sample_count++;
    latest_sample.sample_valid = 1u;
    return HAL_OK;
}

const ImuSample *ImuService_GetLatestSample(void)
{
    return &latest_sample;
}
