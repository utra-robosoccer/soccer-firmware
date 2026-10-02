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

/* SPI opcodes (SPI_OP_*) and frame-size macros are defined once in the shared
   common/include/protocol.h (via proto_common.h) so master and slave cannot
   drift. One chain per slave → fixed frame sizes (no per-N arithmetic):
     master→slave: [opcode][spi_seq][cycle_id][cmd_seq][cmd_chain_t][crc16]  (70 B)
     slave→master: [tele_chain_t][crc16]                                     (119 B) */

/* The full-duplex transfer is the larger of the two frames. */
#define PAYLOAD_LENGTH SPI_XFER_SIZE
/* DMA buffers must hold the full frame; round up to a 32-byte multiple. */
#define BUFFER_SIZE    (((PAYLOAD_LENGTH) + 31u) & ~31u)

extern uint8_t* volatile  cmd_inbox_buf;  // completed RX frame — main reads (command in)
extern uint8_t* volatile  tele_stage_buf; // inactive TX frame — main writes (telemetry out)
extern volatile uint8_t data_receive_flag;
extern volatile uint8_t data_tx_ready_flag;
extern volatile uint8_t spi_error_flag;
extern volatile uint8_t spi_resyncs;      // DMA realigns (wraps; host takes deltas)

void spi_dma_init(SPI_HandleTypeDef *hspi);
void spi_write_next_tx_buf(const uint8_t* src_frame, uint8_t* dst);

/* Re-align the SPI-slave DMA after a bad exchange (called from the main loop on a
   command CRC failure). Aborts + re-arms the fixed-length DMA so the NEXT exchange
   starts at byte 0 — but ONLY while NSS (PA4) is high (between exchanges); if NSS
   is low it waits (bounded) for the exchange to end, else skips and retries next
   cycle. Returns 1 if it re-armed (resync done), 0 if it skipped (NSS stuck low). */
uint8_t slave_spi_resync(SPI_HandleTypeDef *hspi);

/* Arm the DMA for the NEXT exchange (swap in a freshly-staged telemetry frame if ready).
   Idempotent per exchange: the first caller this cycle arms, the rest no-op. Called from
   the main loop once every live motor has replied, and from the TX-arm deadline timer ISR
   as the guaranteed fallback — so the fresh reply rides the next exchange (no lag). */
void spi_arm_tx(void);


#endif /* INC_SLAVE_SPI_H_ */
