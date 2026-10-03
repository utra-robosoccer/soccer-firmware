#ifndef BMI088_H
#define BMI088_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* BMI088 I2C addresses in the shifted format expected by STM32 HAL. */
#define BMI088_GYRO_I2C_ADDRESS  (0x68u << 1u)
#define BMI088_ACCEL_I2C_ADDRESS (0x18u << 1u)
#define BMI088_I2C_TIMEOUT_MS    3u

/* Accelerometer registers. */
#define BMI088_ACCEL_CHIP_ID       0x00u
#define BMI088_ACCEL_DATA          0x12u
#define BMI088_ACCEL_CONFIG        0x40u
#define BMI088_ACCEL_RANGE         0x41u
#define BMI088_ACCEL_INT1_IO_CONF  0x53u
#define BMI088_ACCEL_INT1_MAP      0x58u
#define BMI088_ACCEL_POWER_CONFIG  0x7Cu
#define BMI088_ACCEL_POWER_CONTROL 0x7Du
#define BMI088_ACCEL_SOFT_RESET    0x7Eu

/* Gyroscope registers. */
#define BMI088_GYRO_CHIP_ID        0x00u
#define BMI088_GYRO_DATA           0x02u
#define BMI088_GYRO_RANGE          0x0Fu
#define BMI088_GYRO_BANDWIDTH       0x10u
#define BMI088_GYRO_SOFT_RESET     0x14u
#define BMI088_GYRO_INT_CONTROL    0x15u
#define BMI088_GYRO_INT_IO_CONF    0x16u
#define BMI088_GYRO_INT_MAP        0x18u

typedef struct {
    I2C_HandleTypeDef *i2c;
    float accel_lsb_to_mps2;
    float gyro_lsb_to_rps;
    float accel_mps2[3];
    float gyro_rps[3];
} BMI088;

/* Reset, identify, and configure both BMI088 sensor dies. */
HAL_StatusTypeDef BMI088_Init(BMI088 *imu, I2C_HandleTypeDef *i2c);

/* Read or write one 8-bit register on either sensor die. */
HAL_StatusTypeDef BMI088_ReadAccelRegister(BMI088 *imu, uint8_t reg, uint8_t *data);
HAL_StatusTypeDef BMI088_ReadGyroRegister(BMI088 *imu, uint8_t reg, uint8_t *data);
HAL_StatusTypeDef BMI088_WriteAccelRegister(BMI088 *imu, uint8_t reg, uint8_t data);
HAL_StatusTypeDef BMI088_WriteGyroRegister(BMI088 *imu, uint8_t reg, uint8_t data);

/* Read one sample from each sensor and update the converted values in imu. */
HAL_StatusTypeDef BMI088_ReadAccelerometer(BMI088 *imu,
                                           int16_t *accel_x,
                                           int16_t *accel_y,
                                           int16_t *accel_z);
HAL_StatusTypeDef BMI088_ReadGyroscope(BMI088 *imu,
                                       int16_t *gyro_x,
                                       int16_t *gyro_y,
                                       int16_t *gyro_z);

#endif /* BMI088_H */
