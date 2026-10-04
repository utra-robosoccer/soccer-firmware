#include "bmi088.h"

#include <stddef.h>
#include <string.h>

#define BMI088_ACCEL_EXPECTED_CHIP_ID 0x1Eu
#define BMI088_GYRO_EXPECTED_CHIP_ID  0x0Fu

static HAL_StatusTypeDef read_register(I2C_HandleTypeDef *i2c,
                                       uint16_t address,
                                       uint8_t reg,
                                       uint8_t *data,
                                       uint16_t length)
{
    if (i2c == NULL || data == NULL) {
        return HAL_ERROR;
    }

    return HAL_I2C_Mem_Read(i2c, address, reg, I2C_MEMADD_SIZE_8BIT,
                            data, length, BMI088_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef write_register(I2C_HandleTypeDef *i2c,
                                        uint16_t address,
                                        uint8_t reg,
                                        uint8_t data)
{
    if (i2c == NULL) {
        return HAL_ERROR;
    }

    return HAL_I2C_Mem_Write(i2c, address, reg, I2C_MEMADD_SIZE_8BIT,
                             &data, 1u, BMI088_I2C_TIMEOUT_MS);
}

static int16_t decode_i16_le(const uint8_t *data)
{
    uint16_t raw = (uint16_t)data[0] | ((uint16_t)data[1] << 8u);
    return (int16_t)raw;
}

HAL_StatusTypeDef BMI088_ReadAccelRegister(BMI088 *imu, uint8_t reg, uint8_t *data)
{
    if (imu == NULL) {
        return HAL_ERROR;
    }
    return read_register(imu->i2c, BMI088_ACCEL_I2C_ADDRESS, reg, data, 1u);
}

HAL_StatusTypeDef BMI088_ReadGyroRegister(BMI088 *imu, uint8_t reg, uint8_t *data)
{
    if (imu == NULL) {
        return HAL_ERROR;
    }
    return read_register(imu->i2c, BMI088_GYRO_I2C_ADDRESS, reg, data, 1u);
}

HAL_StatusTypeDef BMI088_WriteAccelRegister(BMI088 *imu, uint8_t reg, uint8_t data)
{
    if (imu == NULL) {
        return HAL_ERROR;
    }
    return write_register(imu->i2c, BMI088_ACCEL_I2C_ADDRESS, reg, data);
}

HAL_StatusTypeDef BMI088_WriteGyroRegister(BMI088 *imu, uint8_t reg, uint8_t data)
{
    if (imu == NULL) {
        return HAL_ERROR;
    }
    return write_register(imu->i2c, BMI088_GYRO_I2C_ADDRESS, reg, data);
}

HAL_StatusTypeDef BMI088_Init(BMI088 *imu, I2C_HandleTypeDef *i2c)
{
    HAL_StatusTypeDef status;
    uint8_t chip_id = 0u;

    if (imu == NULL || i2c == NULL) {
        return HAL_ERROR;
    }

    memset(imu, 0, sizeof(*imu));
    imu->i2c = i2c;

    status = BMI088_WriteAccelRegister(imu, BMI088_ACCEL_SOFT_RESET, 0xB6u);
    if (status != HAL_OK) return status;
    HAL_Delay(50u);

    status = BMI088_ReadAccelRegister(imu, BMI088_ACCEL_CHIP_ID, &chip_id);
    if (status != HAL_OK) return status;
    if (chip_id != BMI088_ACCEL_EXPECTED_CHIP_ID) return HAL_ERROR;
    HAL_Delay(10u);

    status = BMI088_WriteAccelRegister(imu, BMI088_ACCEL_CONFIG, 0xA8u);
    if (status != HAL_OK) return status;
    HAL_Delay(10u);
    status = BMI088_WriteAccelRegister(imu, BMI088_ACCEL_RANGE, 0x00u);
    if (status != HAL_OK) return status;
    HAL_Delay(10u);
    status = BMI088_WriteAccelRegister(imu, BMI088_ACCEL_INT1_IO_CONF, 0x0Au);
    if (status != HAL_OK) return status;
    HAL_Delay(10u);
    status = BMI088_WriteAccelRegister(imu, BMI088_ACCEL_INT1_MAP, 0x04u);
    if (status != HAL_OK) return status;
    HAL_Delay(10u);
    status = BMI088_WriteAccelRegister(imu, BMI088_ACCEL_POWER_CONFIG, 0x00u);
    if (status != HAL_OK) return status;
    HAL_Delay(10u);
    status = BMI088_WriteAccelRegister(imu, BMI088_ACCEL_POWER_CONTROL, 0x04u);
    if (status != HAL_OK) return status;
    HAL_Delay(10u);

    imu->accel_lsb_to_mps2 = (9.81f * 3.0f) / 32768.0f;

    status = BMI088_WriteGyroRegister(imu, BMI088_GYRO_SOFT_RESET, 0xB6u);
    if (status != HAL_OK) return status;
    HAL_Delay(250u);

    status = BMI088_ReadGyroRegister(imu, BMI088_GYRO_CHIP_ID, &chip_id);
    if (status != HAL_OK) return status;
    if (chip_id != BMI088_GYRO_EXPECTED_CHIP_ID) return HAL_ERROR;
    HAL_Delay(10u);

    status = BMI088_WriteGyroRegister(imu, BMI088_GYRO_RANGE, 0x01u);
    if (status != HAL_OK) return status;
    HAL_Delay(10u);
    status = BMI088_WriteGyroRegister(imu, BMI088_GYRO_BANDWIDTH, 0x07u);
    if (status != HAL_OK) return status;
    HAL_Delay(10u);
    status = BMI088_WriteGyroRegister(imu, BMI088_GYRO_INT_CONTROL, 0x80u);
    if (status != HAL_OK) return status;
    HAL_Delay(10u);
    status = BMI088_WriteGyroRegister(imu, BMI088_GYRO_INT_IO_CONF, 0x01u);
    if (status != HAL_OK) return status;
    HAL_Delay(10u);
    status = BMI088_WriteGyroRegister(imu, BMI088_GYRO_INT_MAP, 0x01u);
    if (status != HAL_OK) return status;

    imu->gyro_lsb_to_rps = (0.01745329251f * 1000.0f) / 32768.0f;
    return HAL_OK;
}

HAL_StatusTypeDef BMI088_ReadAccelerometer(BMI088 *imu,
                                           int16_t *accel_x,
                                           int16_t *accel_y,
                                           int16_t *accel_z)
{
    uint8_t data[6];
    HAL_StatusTypeDef status;

    if (imu == NULL || accel_x == NULL || accel_y == NULL || accel_z == NULL) {
        return HAL_ERROR;
    }

    status = read_register(imu->i2c, BMI088_ACCEL_I2C_ADDRESS,
                           BMI088_ACCEL_DATA, data, sizeof(data));
    if (status != HAL_OK) return status;

    *accel_x = decode_i16_le(&data[0]);
    *accel_y = decode_i16_le(&data[2]);
    *accel_z = decode_i16_le(&data[4]);
    imu->accel_mps2[0] = imu->accel_lsb_to_mps2 * (float)*accel_x;
    imu->accel_mps2[1] = imu->accel_lsb_to_mps2 * (float)*accel_y;
    imu->accel_mps2[2] = imu->accel_lsb_to_mps2 * (float)*accel_z;
    return HAL_OK;
}

HAL_StatusTypeDef BMI088_ReadGyroscope(BMI088 *imu,
                                       int16_t *gyro_x,
                                       int16_t *gyro_y,
                                       int16_t *gyro_z)
{
    uint8_t data[6];
    HAL_StatusTypeDef status;

    if (imu == NULL || gyro_x == NULL || gyro_y == NULL || gyro_z == NULL) {
        return HAL_ERROR;
    }

    status = read_register(imu->i2c, BMI088_GYRO_I2C_ADDRESS,
                           BMI088_GYRO_DATA, data, sizeof(data));
    if (status != HAL_OK) return status;

    *gyro_x = decode_i16_le(&data[0]);
    *gyro_y = decode_i16_le(&data[2]);
    *gyro_z = decode_i16_le(&data[4]);
    imu->gyro_rps[0] = imu->gyro_lsb_to_rps * (float)*gyro_x;
    imu->gyro_rps[1] = imu->gyro_lsb_to_rps * (float)*gyro_y;
    imu->gyro_rps[2] = imu->gyro_lsb_to_rps * (float)*gyro_z;
    return HAL_OK;
}
