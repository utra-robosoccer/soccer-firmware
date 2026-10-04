/*
 * spi_proto.c — SPI master↔slave frame codec. See spi_proto.h.
 *
 * Pure byte/struct manipulation, no HAL. The host golden tests
 * (firmware/common/test/test_spi_proto.c) pin the byte layout + CRC.
 */
#include "spi_proto.h"
#include <string.h>

uint8_t spi_proto_parse_cmd(const uint8_t *frame, SpiCmdHdr *hdr, cmd_chain_t *chain)
{
    if (hdr   != NULL) memset(hdr, 0, sizeof(*hdr));
    if (chain != NULL) memset(chain, 0, sizeof(*chain));
    if (frame == NULL || hdr == NULL || chain == NULL) return 0u;

    uint16_t ccrc  = proto_crc16(frame, SPI_CMD_CRC_OFF);
    uint16_t cwire = (uint16_t)(frame[SPI_CMD_CRC_OFF] |
                     ((uint16_t)frame[SPI_CMD_CRC_OFF + 1] << 8));
    if (ccrc != cwire) return 0u;

    hdr->opcode   = frame[0];
    hdr->spi_seq  = frame[1];
    hdr->cycle_id = (uint16_t)(frame[2] | ((uint16_t)frame[3] << 8));
    hdr->cmd_seq  = (uint16_t)(frame[4] | ((uint16_t)frame[5] << 8));
    /* cmd_chain_t is packed; copy it out (unaligned-safe). */
    memcpy(chain, frame + SPI_CMD_HDR_BYTES, sizeof(cmd_chain_t));
    return 1u;
}

void spi_proto_build_tele(uint8_t *frame, const tele_chain_t *chain)
{
    if (frame == NULL) return;

    memset(frame, 0, SPI_TELE_FRAME_SIZE);
    if (chain != NULL) memcpy(frame, chain, sizeof(tele_chain_t));

    uint16_t crc = proto_crc16(frame, SPI_TELE_CRC_OFF);
    frame[SPI_TELE_CRC_OFF]     = (uint8_t)(crc & 0xFFu);   /* little-endian */
    frame[SPI_TELE_CRC_OFF + 1] = (uint8_t)(crc >> 8);
}
