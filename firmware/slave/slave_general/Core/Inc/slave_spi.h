/*
 * slave_spi.h
 *
 *  Created on: 2026年1月21日
 *      Author: 18701
 */

#ifndef INC_SLAVE_SPI_H_
#define INC_SLAVE_SPI_H_

#include "stm32f4xx_hal.h"
#include "motor_chain.h"
#include "proto_common.h"
#include "main.h"

/* SPI command opcodes (SPI_CMD_*) and index macros are defined once in the
   shared common/include/protocol.h (via proto_common.h) so master and slave
   cannot drift. */

/* slave→master telemetry frame (see protocol.h SPI_TELE_FRAME_SIZE):
   [alive_mask u8][echo_seq u8][MotorState × N][slave_debug_rsvd[8]][crc16 u16] */
#define PAYLOAD_LENGTH SPI_TELE_FRAME_SIZE(N_MOTORS)
/* DMA buffers must hold the full frame; round up to a 32-byte multiple. */
#define BUFFER_SIZE    (((PAYLOAD_LENGTH) + 31u) & ~31u)

extern uint8_t* volatile  cmd_inbox_buf;  // completed RX frame — main reads (command in)
extern uint8_t* volatile  tele_stage_buf; // inactive TX frame — main writes (telemetry out)
extern volatile uint8_t data_receive_flag;
extern volatile uint8_t data_tx_ready_flag;
extern volatile uint8_t spi_error_flag;

void spi_dma_init(SPI_HandleTypeDef *hspi);
void spi_write_next_tx_buf(const uint8_t* src_frame, uint8_t* dst);




#endif /* INC_SLAVE_SPI_H_ */
