/*
 * spi_master.h
 *
 *  Created on: 2026年4月19日
 *      Author: 18701
 */

#ifndef INC_SPI_MASTER_H_
#define INC_SPI_MASTER_H_

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#define NUM_SLAVES            4
#define BYTES_PER_MOTOR       5
#define MAX_TOTAL_MOTORS      16
#define MAX_TXRX_BYTES        (MAX_TOTAL_MOTORS * BYTES_PER_MOTOR)

#define P_MIN   -12.57f
#define P_MAX    12.57f
#define V_MIN   -20.0f
#define V_MAX    20.0f
#define KP_MIN    0.0f
#define KP_MAX 5000.0f
#define KD_MIN    0.0f
#define KD_MAX  100.0f
#define T_MIN   -60.0f
#define T_MAX    60.0f

typedef enum {
    MASTER_MODE_DEBUG = 0,
    MASTER_MODE_REAL = 1
} MasterMode_t;

typedef struct {
    uint8_t valid;
    uint8_t motor_id;
    float position;
    float speed;
    uint32_t tick;
} MotorFeedback_t;

/**
 * @brief  [Initialization Interface] Initializes the SPI motor master module
 * @param  hspi Pointer to the SPI handle
 * @param  huart Pointer to the UART handle for debug prints (pass NULL to disable)
 * @retval None
 */
void MotorMaster_Init(SPI_HandleTypeDef *hspi, UART_HandleTypeDef *huart);

/**
 * @brief  [Configuration Interface] Sets the system operation mode
 * @param  mode Target mode (Debug waveform or Real host commands)
 * @retval None
 */
void MotorMaster_SetMode(MasterMode_t mode);

/**
 * @brief  [Output Interface] Sends a control command to the target motor
 * @param  motor_id Logical ID of the target motor
 * @param  position Target position
 * @param  speed Target speed
 * @retval HAL_OK: Success, HAL_ERROR: ID out of bounds or route not found
 */
HAL_StatusTypeDef MotorMaster_SetCommand(uint8_t motor_id, float position, float speed);

/**
 * @brief  [Input Interface] Gets the latest feedback data of the target motor
 * @param  motor_id Logical ID of the target motor
 * @param  feedback Pointer to the structure used to receive feedback data
 * @retval 1: Success, 0: Invalid data or ID out of bounds
 */
uint8_t MotorMaster_GetFeedback(uint8_t motor_id, MotorFeedback_t *feedback);

/**
 * @brief  [Control Interface] Manually clears the emergency stop state
 * @retval None
 */
void MotorMaster_ClearEStop(void);

/**
 * @brief  [Process Interface] Main loop handler, must be called continuously in main while(1)
 * @retval None
 */
void MotorMaster_ProcessLoop(void);


#endif /* INC_SPI_MASTER_H_ */
