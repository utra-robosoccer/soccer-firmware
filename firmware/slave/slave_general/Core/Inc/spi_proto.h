/*
 * spi_proto.h — SPI master<->slave frame codec (pure, host-testable).
 *
 * Owns the wire framing that used to be scattered across main.c (telemetry
 * assembly + command parse) and motor_runtime.c (pack_tele). No HAL / hardware
 * dependency: it operates only on byte buffers and the wire structs defined in
 * the single shared common/include/protocol.h, so it compiles and is unit-tested
 * on the host (see firmware/common/test/test_spi_proto.c).
 *
 * The frame layout itself is unchanged — this is a faithful extraction, and the
 * host golden tests require byte-identical output to the pre-extraction code.
 */
#ifndef SPI_PROTO_H
#define SPI_PROTO_H

#include <stdint.h>
#include <stddef.h>
#include "proto_common.h"  /* MotorState, SpiMitCmd, SPI_* macros, proto_crc16,
                              MOTOR_*_MIN/MAX, SPI_STATE_PACK (via protocol.h +
                              motor_config.h). One shared header, no mirroring. */

#ifdef __cplusplus
extern "C" {
#endif

/* Neutral per-motor sample the control layer produces; the codec turns it into
   the 16-byte wire MotorState atom. Keeping this separate from MotorRuntime lets
   the codec stay independent of the control layer (and of HAL). */
typedef struct {
    float    pos;          /* rad   */
    float    vel;          /* rad/s */
    float    tau;          /* Nm    */
    float    temp;         /* deg C */
    uint8_t  life;         /* MotorLifecycle  (low nibble of state)  */
    uint8_t  cause;        /* MotorFaultCause (high nibble of state) */
    uint8_t  motor_fault;  /* packed RS fault bits                   */
    uint8_t  cmd_flags;    /* SPI_CMDFLAG_*                          */
    uint32_t fault_word;   /* latched 0x3022                         */
    uint32_t fb_age_ms;    /* ms since last feedback (codec saturates to u8) */
} MotorSample;

/* Encode one sample into a wire atom. Byte-identical to the former
   motor_runtime_pack_tele(): float->raw scaling, temp clamp, state nibble pack,
   fb_age saturation at 255, reserved_v2 = 0. */
void spi_proto_encode_atom(MotorState *out, const MotorSample *s);

/* Assemble a full slave->master telemetry frame into `frame` (which must be
   SPI_TELE_FRAME_SIZE(n) bytes): [alive_mask][echo_seq][atom x n]
   [cmd_crc_errors u32 LE | zero_rejects u32 LE][crc16]. */
void spi_proto_build_tele(uint8_t *frame, uint8_t n,
                          uint8_t alive_mask, uint8_t echo_seq,
                          const MotorSample *samples,
                          uint32_t cmd_crc_errors, uint32_t zero_rejects);

/* Decoded master->slave command header (no side effects). */
typedef struct {
    uint8_t opcode;     /* cmd low nibble (SPI_CMD_*)                 */
    uint8_t seq;        /* command seq to echo back                  */
    uint8_t motor_idx;  /* cmd high nibble (ARM / GOTO_ZERO target)  */
    uint8_t n_mit;      /* SpiMitCmd slot count carried (== n)       */
} ParsedCmd;

/* Verify the command-frame CRC and extract the header fields. Returns 1 when the
   CRC is valid (and *out is filled), 0 otherwise (*out zeroed). The caller owns
   dispatch and error counting. */
uint8_t spi_proto_parse(const uint8_t *frame, uint8_t n, ParsedCmd *out);

/* Copy MIT slot i out of a command frame into *out (memcpy — unaligned-safe,
   matching the pre-extraction parse). */
void spi_proto_read_mit(const uint8_t *frame, uint8_t i, SpiMitCmd *out);

#ifdef __cplusplus
}
#endif
#endif /* SPI_PROTO_H */
