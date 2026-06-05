#ifndef INC_SPI_MASTER_H_
#define INC_SPI_MASTER_H_

#include "main.h"
#include "usbd_cdc_if.h"
#include "proto_common.h"
#include "usb_tx.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>

#define SPI_CMD_NOP       0x00
#define SPI_CMD_ARM       0x01  /* bits[7:4] = motor index */
#define SPI_CMD_HOLD      0x02
#define SPI_CMD_DISARM    0x03
#define SPI_CMD_GOTO_ZERO 0x04  /* bits[7:4] = motor index */
#define SPI_CMD_MIT       0x05  /* pass-through MIT command, data in bytes [1..] */

#define SPI_CMD_ARM_IDX(idx)       (SPI_CMD_ARM       | ((uint8_t)(idx) << 4u))
#define SPI_CMD_GOTO_ZERO_IDX(idx) (SPI_CMD_GOTO_ZERO | ((uint8_t)(idx) << 4u))

#define NUM_SLV               1
#define MAX_MOTORS_PER_SLAVE  2
#define BYTES_PER_MOTOR       5
#define USB_BYTES_PER_MOTOR  sizeof(motor_cmd_t)

/* SPI packet from slave: 1 header byte + N_MOTORS * SpiMotorTele (13 bytes each) */
#define SPI_TELE_BYTES   13u
#define SPI_RX_PKT_SIZE  (1u + N_MOTORS * SPI_TELE_BYTES)

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

void MotorMaster_SetArmed(uint8_t armed);
void MotorMaster_HandleControlReq(const ControlReq *req, uint16_t req_seq);
void MotorMaster_SetMitCmd(uint8_t idx, float pos, float vel,
                            float kp, float kd, float tau_ff);

extern uint32_t master_link_errors;
extern uint32_t master_rx_frames;
#endif /* INC_SPI_MASTER_H_ */
