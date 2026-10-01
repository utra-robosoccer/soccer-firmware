#ifndef INC_SPI_MASTER_H_
#define INC_SPI_MASTER_H_

#include "main.h"
#include "usbd_cdc_if.h"
#include "proto_common.h"
#include "usb_tx.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>

/* SPI command opcodes (SPI_CMD_*) and index macros are defined once in the
   shared common/include/protocol.h (via proto_common.h) so master and slave
   cannot drift. */

/* NUM_SLAVES, MAX_MOTORS_PER_SLAVE come from system_config.h (via proto_common.h). */
#define NUM_SLV               NUM_SLAVES
#define BYTES_PER_MOTOR       5
#define USB_BYTES_PER_MOTOR  sizeof(motor_cmd_t)

/* One full-duplex transfer per slave carries the CRC-framed telemetry frame
   (protocol.h SPI_TELE_FRAME_SIZE) in the RX direction and the command frame in
   the TX prefix. Per-slave sizes differ; buffers are sized for the widest slave
   and the per-slave length is computed at runtime from slave_motor_counts[]. */
#define SPI_MAX_PKT_SIZE SPI_TELE_FRAME_SIZE(MAX_MOTORS_PER_SLAVE)
#define SPI_PKT_SIZE(n)  SPI_TELE_FRAME_SIZE(n)

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
void MotorMaster_SetMitCmd(uint8_t slave_id, uint8_t idx, float pos, float vel,
                            float kp, float kd, float tau_ff, uint16_t cmd_seq);

extern uint32_t master_link_errors;
extern uint32_t master_rx_frames;
extern uint32_t master_proto_ver_mismatch;  /* host frames rejected on PROTO_VERSION */
#endif /* INC_SPI_MASTER_H_ */
