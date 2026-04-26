#include "BMI088.h"

/*
 *
 * INITIALISATION
 *
 */
uint8_t BMI088_Init(BMI088 *imu, I2C_HandleTypeDef *m_i2c_handle) {

  /* Store interface parameters in struct */
  imu->m_i2c_handle    = m_i2c_handle;

  /* Clear DMA flags */
  imu->readingAcc = 0;
  imu->readingGyr = 0;

  uint8_t status = 0;

  /*
   *
   * ACCELEROMETER
   *
   */

  /* Perform accelerometer soft reset */
  status += BMI088_WriteAccRegister(imu, BMI_ACC_SOFTRESET, 0xB6);
  HAL_Delay(50);

  /* Check chip ID */
  uint8_t chipID = 0;
  status = BMI088_ReadAccRegister(imu, BMI_ACC_CHIP_ID, &chipID);

  // chipID bmi085 is 0x1f, chipID bmi088 is 0x1e
  if (chipID != 0x1e) {
    return 0;
  }
  HAL_Delay(10);

  /* Configure accelerometer  */
  status += BMI088_WriteAccRegister(imu, BMI_ACC_CONF, 0xA8); /* (no oversampling, ODR = 100 Hz, BW = 40 Hz) */
  HAL_Delay(10);

  status += BMI088_WriteAccRegister(imu, BMI_ACC_RANGE, 0x00); /* +- 3g range (for bmi085 +- 2g range)*/
  HAL_Delay(10);

  /* Enable accelerometer data ready interrupt */
  status += BMI088_WriteAccRegister(imu, BMI_INT1_IO_CONF, 0x0A); /* INT1 = push-pull output, active high */
  HAL_Delay(10);

  status += BMI088_WriteAccRegister(imu, BMI_INT1_INT2_MAP_DATA, 0x04);
  HAL_Delay(10);

  /* Put accelerometer into active mode */
  status += BMI088_WriteAccRegister(imu, BMI_ACC_PWR_CONF, 0x00);
  HAL_Delay(10);

  /* Turn accelerometer on */
  status += BMI088_WriteAccRegister(imu, BMI_ACC_PWR_CTRL, 0x04);
  HAL_Delay(10);

  /* Pre-compute accelerometer conversion constant (raw to m/s^2) */
  imu->accConversion = 9.81f / 32768.0f * 2.0f * 1.5f; /* Datasheet page 27 */

  /* Set accelerometer TX buffer for DMA */
  imu->accTxBuf[0] = BMI_ACC_DATA | 0x80;

  /*
   *
   * GYROSCOPE
   *
   */

//  HAL_GPIO_WritePin(imu->csGyrPinBank, imu->csGyrPin, GPIO_PIN_SET);

  /* Perform gyro soft reset */
  status += BMI088_WriteGyrRegister(imu, BMI_GYR_SOFTRESET, 0xB6);
  HAL_Delay(250);

  /* Check chip ID */
  status += BMI088_ReadGyrRegister(imu, BMI_GYR_CHIP_ID, &chipID);

  // chip ID is 0x0F for both BMI088/085
  if (chipID != 0x0F) {
    return 0;
  }
  HAL_Delay(10);

  /* Configure gyroscope */
  status += BMI088_WriteGyrRegister(imu, BMI_GYR_RANGE, 0x01); /* +- 1000 deg/s (bmi085 also +-1000deg/s)*/
  HAL_Delay(10);

  status += BMI088_WriteGyrRegister(imu, BMI_GYR_BANDWIDTH, 0x07); /* ODR = 100 Hz, Filter bandwidth = 32 Hz */
  HAL_Delay(10);

  /* Enable gyroscope data ready interrupt */
  status += BMI088_WriteGyrRegister(imu, BMI_GYR_INT_CTRL, 0x80); /* New data interrupt enabled */
  HAL_Delay(10);

  status += BMI088_WriteGyrRegister(imu, BMI_INT3_INT4_IO_CONF, 0x01); /* INT3 = push-pull, active high */
  HAL_Delay(10);

  status += BMI088_WriteGyrRegister(imu, BMI_INT3_INT4_IO_MAP, 0x01); /* Data ready interrupt mapped to INT3 pin */
  HAL_Delay(10);

  /* Pre-compute gyroscope conversion constant (raw to rad/s) */
  imu->gyrConversion = 0.01745329251f * 1000.0f / 32768.0f; /* Datasheet page 39 */

  /* Set gyroscope TX buffer for DMA */
  imu->gyrTxBuf[0] = BMI_GYR_DATA | 0x80;

  return status;

}

/*
 *
 * LOW-LEVEL REGISTER FUNCTIONS
 *
 */

/* ACCELEROMETER READS ARE DIFFERENT TO GYROSCOPE READS. SEND ONE BYTE ADDRESS, READ ONE DUMMY BYTE, READ TRUE DATA !!! */
uint8_t BMI088_ReadAccRegister(BMI088 *imu, uint8_t regAddr, uint8_t *data) {
  HAL_StatusTypeDef status;

  // Read data from the specified register address
  status = HAL_I2C_Mem_Read(imu->m_i2c_handle, BMI_ACC_I2C_ADDRESS, regAddr, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
  if (status != HAL_OK) {
      return status;
  }

  return HAL_OK;
}

uint8_t BMI088_ReadGyrRegister(BMI088 *imu, uint8_t regAddr, uint8_t *data) {
  HAL_StatusTypeDef status;

  // Read data from the specified register address
  status = HAL_I2C_Mem_Read(imu->m_i2c_handle, BMI_GYR_I2C_ADDRESS, regAddr, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
  if (status != HAL_OK) {
      return status;
  }

  return HAL_OK;
}

uint8_t BMI088_WriteAccRegister(BMI088 *imu, uint8_t regAddr, uint8_t data) {
  HAL_StatusTypeDef status;

  // Write data to the specified register address
  status = HAL_I2C_Mem_Write(imu->m_i2c_handle, BMI_ACC_I2C_ADDRESS, regAddr, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
  if (status != HAL_OK) {
      return status;
  }

  return HAL_OK;
}

uint8_t BMI088_WriteGyrRegister(BMI088 *imu, uint8_t regAddr, uint8_t data) {
  HAL_StatusTypeDef status;

  // Write data to the specified register address
  status = HAL_I2C_Mem_Write(imu->m_i2c_handle, BMI_GYR_I2C_ADDRESS, regAddr, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
  if (status != HAL_OK) {
      return status;
  }

  return HAL_OK;
}



/*
 *
 * POLLING
 *
 */
uint8_t BMI088_ReadAccelerometer(BMI088 *imu, I2C_HandleTypeDef *hi2c, int16_t *accX, int16_t *accY, int16_t *accZ) {
    HAL_StatusTypeDef status;
    uint8_t data[6]; // Data buffer to store gyroscope data

    status = HAL_I2C_Mem_Read(hi2c, BMI_ACC_I2C_ADDRESS, BMI_ACC_DATA, I2C_MEMADD_SIZE_8BIT, data, 6, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        return status;
    }

    /* Form signed 16-bit integers */
    *accX = (int16_t) ((data[1] << 8) | data[0]);
    *accY = (int16_t) ((data[3] << 8) | data[2]);
    *accZ = (int16_t) ((data[5] << 8) | data[4]);

    /* Convert to m/s^2 */
    imu->acc_mps2[0] = imu->accConversion * (*accX);
    imu->acc_mps2[1] = imu->accConversion * (*accY);
    imu->acc_mps2[2] = imu->accConversion * (*accZ);

    return status;
}


// Function to read gyroscope values from BMI088 sensor
HAL_StatusTypeDef BMI088_ReadGyroscope(I2C_HandleTypeDef *hi2c, int16_t *gyroX, int16_t *gyroY, int16_t *gyroZ) {
    HAL_StatusTypeDef status;
    uint8_t data[6]; // Data buffer to store gyroscope data

    status = HAL_I2C_Mem_Read(hi2c, BMI_GYR_I2C_ADDRESS, BMI_GYR_DATA, I2C_MEMADD_SIZE_8BIT, data, 6, HAL_MAX_DELAY);

    if (status != HAL_OK) {
        return status;
    }

    // Parse gyroscope data (each value is 16 bits, little-endian)
    *gyroX = (int16_t)((data[1] << 8) | data[0]);
    *gyroY = (int16_t)((data[3] << 8) | data[2]);
    *gyroZ = (int16_t)((data[5] << 8) | data[4]);

    // do gyro conversion here?

    return status;
}


