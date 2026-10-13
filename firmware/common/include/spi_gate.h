/*
 * spi_gate.h — SPI version gate (v9.1), pure and host-testable.
 *
 * Both SPI frame directions carry a spi_trailer_t (sender's PROTO_VERSION +
 * PROTO_SCHEMA_HASH) between the payload and the CRC. Each side stamps its own
 * build and refuses a mismatched frame's payload; polling continues, so a
 * reflashed board is re-accepted without a reset on either side
 * (docs/protocol_v9.md §SPI version gate). test_spi_gate.c pins the behavior:
 * match, every single-byte mismatch, and recovery after a match.
 */
#ifndef SPI_GATE_H
#define SPI_GATE_H

#include <stdint.h>
#include "protocol.h"   /* PROTO_VERSION, PROTO_SCHEMA_HASH, DriveReq/DriveState */

#ifdef __cplusplus
extern "C" {
#endif

/* Stamp this build's trailer at `at` (little-endian, sizeof(spi_trailer_t)). */
static inline void spi_trailer_write(uint8_t *at)
{
    at[0] = (uint8_t)PROTO_VERSION;
    at[1] = (uint8_t)(PROTO_SCHEMA_HASH & 0xFFu);
    at[2] = (uint8_t)((PROTO_SCHEMA_HASH >> 8) & 0xFFu);
    at[3] = (uint8_t)((PROTO_SCHEMA_HASH >> 16) & 0xFFu);
    at[4] = (uint8_t)((PROTO_SCHEMA_HASH >> 24) & 0xFFu);
}

/* 1 = the trailer at `at` matches this build (version AND schema hash). */
static inline uint8_t spi_trailer_match(const uint8_t *at)
{
    uint32_t hash = (uint32_t)at[1]
                  | ((uint32_t)at[2] << 8)
                  | ((uint32_t)at[3] << 16)
                  | ((uint32_t)at[4] << 24);
    return (at[0] == (uint8_t)PROTO_VERSION && hash == PROTO_SCHEMA_HASH) ? 1u : 0u;
}

/* Robot-wide enable refusal while ANY slave mismatches: an ENABLE for a motor
 * that is not already armed is rewritten to DISABLE (a no-op in DISABLED, so
 * nothing starts); a motor already ENABLING/ENABLED/STOPPING keeps its no-op
 * ENABLE, so a mid-run mismatch never force-disables running joints. Every
 * other request passes through — DISABLE/STOP/RESET must keep working. */
static inline uint8_t spi_gate_filter_req(uint8_t drive_req, uint8_t drive_state)
{
    if (drive_req != (uint8_t)DRV_REQ_ENABLE) return drive_req;
    if (drive_state == (uint8_t)DRV_ENABLING ||
        drive_state == (uint8_t)DRV_ENABLED ||
        drive_state == (uint8_t)DRV_STOPPING) {
        return drive_req;
    }
    return (uint8_t)DRV_REQ_DISABLE;
}

#ifdef __cplusplus
}
#endif
#endif /* SPI_GATE_H */
