/*
 * spi_master.h
 * Simplified SPI Master for 1-to-1 (Expandable to 1-to-Multi)
 */

#ifndef INC_SPI_MASTER_H_
#define INC_SPI_MASTER_H_

#include "main.h"
#include "usbd_cdc_if.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>

#define P_MIN   -12.57f
#define P_MAX    12.57f
#define V_MIN   -20.0f
#define V_MAX    20.0f

#define MAX_MOTORS_PER_SLAVE  4
#define BYTES_PER_MOTOR       5

typedef enum {
    DEV1 = 1,
    DEV2 = 2,
    DEV3 = 3,
    DEV4 = 4
} SpiDevId;

typedef struct {
    uint8_t motor_id;
    float position;
    float speed;
} MotorCmd;

typedef struct {
    uint8_t valid;
    uint8_t motor_id;
    float position;
    float speed;
} MotorFeedback;

void usb_printf(const char *fmt, ...);

/**
 * @brief  Initializes the SPI master with handles
 */
void MotorMaster_Init(SPI_HandleTypeDef *hspi, UART_HandleTypeDef *huart);

/**
 * @brief  Updates the target command buffer for a specific motor index on the active slave
 */
void MotorMaster_SetCommand(uint8_t motor_index, uint8_t motor_id, float position, float speed);

/**
 * @brief  Retrieves the latest feedback for a given motor index
 */
uint8_t MotorMaster_GetFeedback(uint8_t motor_index, MotorFeedback *feedback);

/**
 * @brief  Main process loop executing the sweep test and SPI transmission
 */
void MotorMaster_ProcessLoop(void);

#endif /* INC_SPI_MASTER_H_ */
