/*
 * spi_proto.h — SPI master↔slave frame codec (pure, host-testable).
 *
 * One SPI transfer carries a single spi_cmd_frame_t (master→slave) /
 * spi_tele_frame_t (slave→master); both are fixed-size packed structs from
 * protocol_wire.h with a trailing CRC16. No HAL dependency — operates on byte
 * buffers, so it is unit-tested on the host
 * (firmware/common/test/test_spi_proto.c).
 */
#ifndef SPI_PROTO_H
#define SPI_PROTO_H

#include <stdint.h>
#include <stddef.h>
#include "proto_common.h"   /* protocol.h wire structs + proto_crc16 */

#ifdef __cplusplus
extern "C" {
#endif

/* Verify the command-frame CRC and copy out the header + cmd_slave_t.
 * Returns 1 on a valid frame, 0 on CRC failure / garbage (outputs zeroed —
 * the caller applies nothing, counts the reject, and resyncs). */
uint8_t spi_proto_parse_cmd(const uint8_t *frame, spi_header_t *hdr,
                            cmd_slave_t *cmd);

/* Assemble a full slave→master telemetry frame into `frame`
 * (sizeof(spi_tele_frame_t) = SPI_XFER_SIZE bytes): the tele_slave_t followed
 * by a CRC16 over it. */
void spi_proto_build_tele(uint8_t *frame, const tele_slave_t *tele);

#ifdef __cplusplus
}
#endif
#endif /* SPI_PROTO_H */
