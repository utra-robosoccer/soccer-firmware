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

#define NUM_SLV               2
#define MAX_MOTORS_PER_SLAVE  1
#define BYTES_PER_MOTOR       5
#define USB_BYTES_PER_MOTOR  sizeof(motor_cmd_t)

typedef enum {
    DEV1 = 0,
    DEV2 = 1,
    DEV3 = 2,
    DEV4 = 3
} SpiDevId;

typedef struct __attribute__((packed)) {
    uint8_t motor_id;
    float position;
    float speed;
} motor_cmd_t;

typedef struct {
    SpiDevId slv_id;
    motor_cmd_t* slv_motor_cmds;
    motor_cmd_t* slv_motor_feedbacks;
    uint8_t active_motor_count;
    uint8_t* motorID_lut;
} slv_motor_chain_t;

extern uint8_t new_usb_packet_rx_flag;
extern uint8_t buf_rx_jet2master[NUM_SLV * MAX_MOTORS_PER_SLAVE * USB_BYTES_PER_MOTOR];

void usb_printf(const char *fmt, ...);

/**
 * @brief  Initializes the SPI master with handles and sets up the multi-slave chains
 */
void MotorMaster_Init(SPI_HandleTypeDef *hspi, UART_HandleTypeDef *huart);

/**
 * @brief  Main process loop executing the SPI transmission across ALL slaves
 */
void MotorMaster_ProcessLoop(void);


// USB Data Formatting APIs
void MotorMaster_ParseRxBuffer(void);
void MotorMaster_FormatTxBuffer(void);
#endif /* INC_SPI_MASTER_H_ */
