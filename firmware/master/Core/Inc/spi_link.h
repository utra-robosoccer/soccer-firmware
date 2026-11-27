/* spi_link — the SPI wire to the slaves: chip-selects, the fixed-size full-duplex
 * exchange, and per-slave link health (alive / seq-echo diagnostics / last
 * CRC-valid telemetry).
 *
 * Layering: the app decides WHAT to send (opcode + cmd_slave_t — host-dead
 * override, active command, NOP keepalive); spi_link handles HOW (framing,
 * CRC, health classification).
 */
#ifndef SPI_LINK_H
#define SPI_LINK_H

#include "main.h"
#include "proto_common.h"
#include <stdint.h>

typedef enum {
    DEV1 = 0,
    DEV2 = 1,
    DEV3 = 2,
    DEV4 = 3
} SpiDevId;

/* Per-slave link state, updated by every spi_link_poll. Read-only to callers.
   The wire-facing counters mirror spi_master_stats_t (u16, wrap). */
typedef struct {
    uint8_t      alive;           /* last exchange returned a CRC-valid frame      */
    tele_slave_t tele;            /* last CRC-valid telemetry                      */
    uint16_t     crc_errors;      /* telemetry frames failing CRC                  */
    uint16_t     seq_gaps;        /* command-link stalls (echo froze, see SEQ_STALL) */
    /* link-internal seq/echo diagnostics */
    uint8_t      seq;             /* per-slave SPI link-health seq (sent, increments) */
    uint8_t      prev_echo;
    uint8_t      have_prev_echo;
    uint8_t      echo_stall;
} SpiLinkSlave;

void spi_link_init(SPI_HandleTypeDef *hspi);

/* One exchange with slave s: forward opcode + cmd (NULL allowed only for a
   NOP — the frame then carries a zeroed cmd_slave_t), ingest its telemetry,
   classify link health. cmd_seq must be 0 on a NOP (spi_header_t contract). */
void spi_link_poll(uint8_t s, uint8_t opcode, const cmd_slave_t *cmd,
                   uint16_t cycle_id, uint16_t cmd_seq);

const SpiLinkSlave *spi_link_slave(uint8_t s);   /* s < NUM_SLAVES */

#ifdef SPI_INJECT_TEST
/* Test-only: clock one deliberately short exchange to desync slave 0's DMA. */
extern volatile uint8_t g_spi_inject;
#endif

#endif /* SPI_LINK_H */
