/*
 * spi_proto.h — SPI master↔slave frame codec (pure, host-testable).
 *
 * PROTO_VERSION 3 hierarchy: one chain per slave, so each SPI transfer carries a
 * single cmd_chain_t / tele_chain_t (fixed size). No HAL dependency — operates on
 * byte buffers and the shared wire structs in common/include/protocol.h, so it is
 * unit-tested on the host (firmware/common/test/test_spi_proto.c).
 *
 *   master → slave: [opcode u8][spi_seq u8][cycle_id u16][cmd_seq u16]
 *                   [cmd_chain_t][crc16]
 *   slave → master: [tele_chain_t][crc16]
 */
#ifndef SPI_PROTO_H
#define SPI_PROTO_H

#include <stdint.h>
#include <stddef.h>
#include "proto_common.h"   /* protocol.h wire structs + proto_crc16 + SPI_* macros */

#ifdef __cplusplus
extern "C" {
#endif

/* Decoded master→slave command header (the bytes before the cmd_chain_t). */
typedef struct {
    uint8_t  opcode;    /* SPI_OP_NOP / SPI_OP_ROBOT_CMD                 */
    uint8_t  spi_seq;   /* SPI link-health seq to echo back              */
    uint16_t cycle_id;  /* master poll counter                          */
    uint16_t cmd_seq;   /* host command sequence (one per host tick)     */
} SpiCmdHdr;

/* spi_proto_parse_cmd result. */
typedef enum {
    SPI_PARSE_BAD      = 0,  /* CRC failure / garbage — *hdr and *chain zeroed  */
    SPI_PARSE_OK       = 1,  /* valid frame from a matching build — outputs set */
    SPI_PARSE_MISMATCH = 2,  /* CRC-intact but the master's spi_trailer_t
                                version/schema differ — outputs zeroed, apply
                                nothing (version gate, v9.1)                    */
} SpiParseResult;

/* Verify the command-frame CRC + version trailer, extract the header, and copy
 * out the cmd_chain_t. The caller owns dispatch and error counting. */
uint8_t spi_proto_parse_cmd(const uint8_t *frame, SpiCmdHdr *hdr, cmd_chain_t *chain);

/* Assemble a full slave→master telemetry frame into `frame` (SPI_TELE_FRAME_SIZE
 * bytes): the tele_chain_t followed by a CRC16 over it. `chain` is the fully
 * populated wire struct. */
void spi_proto_build_tele(uint8_t *frame, const tele_chain_t *chain);

#ifdef __cplusplus
}
#endif
#endif /* SPI_PROTO_H */
