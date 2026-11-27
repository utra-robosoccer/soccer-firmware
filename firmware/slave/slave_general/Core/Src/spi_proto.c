/*
 * spi_proto.c — SPI master↔slave frame codec. See spi_proto.h.
 *
 * Pure byte/struct manipulation, no HAL. The host golden tests
 * (firmware/common/test/test_spi_proto.c) pin the byte layout + CRC.
 */
#include "spi_proto.h"
#include <string.h>

uint8_t spi_proto_parse_cmd(const uint8_t *frame, spi_header_t *hdr,
                            cmd_slave_t *cmd)
{
    if (hdr != NULL) memset(hdr, 0, sizeof(*hdr));
    if (cmd != NULL) memset(cmd, 0, sizeof(*cmd));
    if (frame == NULL || hdr == NULL || cmd == NULL) return 0u;

    const size_t crc_off = offsetof(spi_cmd_frame_t, crc16);
    uint16_t calc = proto_crc16(frame, crc_off);
    uint16_t wire = (uint16_t)(frame[crc_off] |
                    ((uint16_t)frame[crc_off + 1u] << 8));
    if (calc != wire) return 0u;

    /* Packed structs; memcpy keeps the reads unaligned-safe. */
    memcpy(hdr, frame + offsetof(spi_cmd_frame_t, hdr), sizeof(*hdr));
    memcpy(cmd, frame + offsetof(spi_cmd_frame_t, cmd), sizeof(*cmd));
    return 1u;
}

void spi_proto_build_tele(uint8_t *frame, const tele_slave_t *tele)
{
    if (frame == NULL) return;

    memset(frame, 0, sizeof(spi_tele_frame_t));
    if (tele != NULL) memcpy(frame, tele, sizeof(tele_slave_t));

    const size_t crc_off = offsetof(spi_tele_frame_t, crc16);
    uint16_t crc = proto_crc16(frame, crc_off);
    frame[crc_off]      = (uint8_t)(crc & 0xFFu);   /* little-endian */
    frame[crc_off + 1u] = (uint8_t)(crc >> 8);
}
