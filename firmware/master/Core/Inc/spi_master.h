#ifndef INC_SPI_MASTER_H_
#define INC_SPI_MASTER_H_

#include "main.h"
#include "usbd_cdc_if.h"
#include "proto_common.h"
#include "usb_link.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>

/* SPI opcodes (SPI_OP_*) and the fixed frame-size macros (one chain per slave)
   are defined once in the shared common/include/protocol.h (via proto_common.h)
   so master and slave cannot drift. The full-duplex transfer is SPI_XFER_SIZE. */

/* NUM_SLAVES, MAX_MOTORS_PER_SLAVE come from system_config.h (via proto_common.h). */
#define NUM_SLV               NUM_SLAVES
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

/* Gated debug output: ONLY to UART4, never the USB CDC (the CDC carries framed
   protocol messages exclusively — docs/protocol.md). Compiled out by default;
   build with -DMASTER_DEBUG_UART to enable. */
#ifdef MASTER_DEBUG_UART
void debug_printf(const char *fmt, ...);
#else
#define debug_printf(...) ((void)0)
#endif

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

/* USB RX (ring + frame scan) lives in usb_link; this module only supplies the
   frame-dispatch callback passed to usb_rx_process in the main loop. */

extern uint32_t master_link_errors;
extern uint32_t master_proto_ver_mismatch;  /* host frames rejected on PROTO_VERSION */
#endif /* INC_SPI_MASTER_H_ */
