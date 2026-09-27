/*
 * spi_proto.c — SPI master<->slave frame codec. See spi_proto.h.
 *
 * Pure byte/struct manipulation, no HAL. Every layout decision here mirrors the
 * pre-extraction code (main.c telemetry assembly + command parse, and
 * motor_runtime.c pack_tele) exactly; the host golden tests enforce that.
 */
#include "spi_proto.h"
#include <string.h>

/* float -> 16-bit raw over [lo,hi], clamped. Identical to the former static
   f_to_u16() in motor_runtime.c. */
static uint16_t f_to_u16(float x, float lo, float hi)
{
    if (x < lo) x = lo;
    if (x > hi) x = hi;
    return (uint16_t)((x - lo) * 65535.0f / (hi - lo));
}

void spi_proto_encode_atom(MotorState *out, const MotorSample *s)
{
    if (out == NULL || s == NULL) return;

    out->pos_raw     = f_to_u16(s->pos, MOTOR_P_MIN, MOTOR_P_MAX);
    out->vel_raw     = f_to_u16(s->vel, MOTOR_V_MIN, MOTOR_V_MAX);
    out->tau_raw     = f_to_u16(s->tau, MOTOR_T_MIN, MOTOR_T_MAX);
    out->temp_c      = (uint8_t)(s->temp < 0.0f ? 0u : (uint8_t)s->temp);
    out->state       = SPI_STATE_PACK(s->life, s->cause);
    out->motor_fault = s->motor_fault;
    out->cmd_flags   = s->cmd_flags;
    out->fault_word  = s->fault_word;
    out->fb_age      = (s->fb_age_ms > 255u) ? 255u : (uint8_t)s->fb_age_ms;
    out->reserved_v2 = 0u;
}

void spi_proto_build_tele(uint8_t *frame, uint8_t n,
                          uint8_t alive_mask, uint8_t echo_seq,
                          const MotorSample *samples,
                          uint32_t cmd_crc_errors, uint32_t zero_rejects)
{
    if (frame == NULL) return;

    const uint16_t frame_len = SPI_TELE_FRAME_SIZE(n);
    memset(frame, 0, frame_len);          /* zeroes any padding + reserved bytes */

    frame[0] = alive_mask;
    frame[1] = echo_seq;

    MotorState *ms = (MotorState *)&frame[SPI_TELE_HDR_BYTES];
    for (uint8_t i = 0; i < n; i++) {
        if (samples != NULL) spi_proto_encode_atom(&ms[i], &samples[i]);
    }

    /* slave_debug_rsvd[0..3] = cmd_crc_errors (u32 LE); [4..7] = zero_rejects
       (u32 LE). Relayed to the master over the CRC-covered reserved region. */
    uint8_t *dbg = &frame[SPI_TELE_DEBUG_OFF(n)];
    dbg[0] = (uint8_t)(cmd_crc_errors & 0xFFu);
    dbg[1] = (uint8_t)((cmd_crc_errors >> 8) & 0xFFu);
    dbg[2] = (uint8_t)((cmd_crc_errors >> 16) & 0xFFu);
    dbg[3] = (uint8_t)((cmd_crc_errors >> 24) & 0xFFu);
    dbg[4] = (uint8_t)(zero_rejects & 0xFFu);
    dbg[5] = (uint8_t)((zero_rejects >> 8) & 0xFFu);
    dbg[6] = (uint8_t)((zero_rejects >> 16) & 0xFFu);
    dbg[7] = (uint8_t)((zero_rejects >> 24) & 0xFFu);

    uint16_t crc = proto_crc16(frame, (size_t)(frame_len - SPI_TELE_CRC_BYTES));
    frame[frame_len - 2] = (uint8_t)(crc & 0xFFu);   /* little-endian */
    frame[frame_len - 1] = (uint8_t)(crc >> 8);
}

uint8_t spi_proto_parse(const uint8_t *frame, uint8_t n, ParsedCmd *out)
{
    if (out != NULL) memset(out, 0, sizeof(*out));
    if (frame == NULL || out == NULL) return 0u;

    uint16_t ccrc  = proto_crc16(frame, SPI_CMD_CRC_OFF(n));
    uint16_t cwire = (uint16_t)(frame[SPI_CMD_CRC_OFF(n)] |
                     ((uint16_t)frame[SPI_CMD_CRC_OFF(n) + 1] << 8));
    if (ccrc != cwire) return 0u;

    out->opcode    = (uint8_t)(frame[0] & 0x0Fu);
    out->seq       = frame[1];
    out->motor_idx = SPI_CMD_MOTOR_IDX(frame[0]);
    out->n_mit     = n;
    return 1u;
}

void spi_proto_read_mit(const uint8_t *frame, uint8_t i, SpiMitCmd *out)
{
    if (frame == NULL || out == NULL) return;
    memcpy(out, frame + SPI_CMD_HDR_BYTES + (size_t)i * sizeof(SpiMitCmd),
           sizeof(SpiMitCmd));
}
