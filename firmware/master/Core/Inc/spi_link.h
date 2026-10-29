/* spi_link — the SPI wire to the slaves: chip-selects, the fixed-size full-duplex
 * exchange, and per-slave link health (alive / version mismatch / seq-echo
 * diagnostics / last CRC-valid telemetry).
 *
 * Layering: the app decides WHAT to send (opcode + chain — host-dead override,
 * active command, NOP keepalive); spi_link handles HOW (framing, CRC, version
 * gate, health classification). The version gate is enforced HERE so no caller
 * can accidentally forward commands to a mismatched slave.
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

/* Per-slave link state, updated by every spi_link_poll. Read-only to callers. */
typedef struct {
    uint8_t      alive;           /* last exchange returned a CRC-valid, matching frame */
    uint8_t      mismatch;        /* version gate: CRC-intact tele, wrong proto/schema  */
    uint8_t      motors_alive;    /* bitmask: drive_state past BOOT/DISCOVERING         */
    tele_chain_t tele;            /* last CRC-valid telemetry chain                     */
    uint32_t     crc_errors;      /* telemetry frames failing CRC                       */
    uint32_t     seq_gaps;        /* command-link stalls (echo froze, see SEQ_STALL)    */
    uint32_t     cmd_crc_errors;  /* relayed from tele_chain_t (slave rejected cmds)    */
    /* link-internal seq/echo diagnostics */
    uint8_t      seq;             /* per-slave SPI link-health seq (sent, increments)   */
    uint8_t      prev_echo;
    uint8_t      have_prev_echo;
    uint8_t      echo_stall;
} SpiLinkSlave;

void spi_link_init(SPI_HandleTypeDef *hspi);

/* One exchange with slave s: forward opcode+chain (subject to the version gate),
   ingest its telemetry, classify link health. chain may be NULL only for a NOP. */
void spi_link_poll(uint8_t s, uint8_t opcode, const cmd_chain_t *chain,
                   uint16_t cycle_id, uint16_t cmd_seq);

const SpiLinkSlave *spi_link_slave(uint8_t s);   /* s < NUM_SLAVES */
uint8_t spi_link_any_mismatch(void);             /* 1 = any slave fails the version gate */

#ifdef SPI_INJECT_TEST
/* Test-only: clock one deliberately short exchange to desync slave 0's DMA. */
extern volatile uint8_t g_spi_inject;
#endif

#endif /* SPI_LINK_H */
