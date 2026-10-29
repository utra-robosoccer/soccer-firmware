#ifndef PROTOCOL_H
#define PROTOCOL_H

/* Wire protocol — thin wrapper.
 *
 * Every wire LAYOUT (structs, enums, flag bits, fixed-point scales, constants)
 * is generated from firmware/common/protocol.yaml into protocol_gen.h — edit
 * the schema and re-run scripts/gen_protocol.py, never the generated header.
 * This file keeps only the hand-written non-layout code: the fixed-point
 * encode/decode helpers, CRC16-CCITT, and the USB frame builder/scanner. */

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "protocol_gen.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── fixed-point encode/decode (shared; host mirrors the arithmetic) ────────
 * Round-half-away-from-zero, no math.h. Encoders saturate at the type limits and
 * set *sat (if non-NULL) on saturation. */
static inline int16_t proto_f_to_i16(float x, float scale, uint8_t *sat)
{
    float v = x * scale;
    v = (v >= 0.0f) ? (v + 0.5f) : (v - 0.5f);
    if (v >  32767.0f) { if (sat) *sat = 1u; return (int16_t)32767; }
    if (v < -32768.0f) { if (sat) *sat = 1u; return (int16_t)(-32768); }
    return (int16_t)v;
}
static inline uint16_t proto_f_to_u16(float x, float scale, uint8_t *sat)
{
    float v = x * scale + 0.5f;
    if (v > 65535.0f) { if (sat) *sat = 1u; return (uint16_t)65535; }
    if (v < 0.0f)     { return 0u; }   /* gains are non-negative */
    return (uint16_t)v;
}
static inline float proto_i16_to_f(int16_t r, float scale) { return (float)r / scale; }
static inline float proto_u16_to_f(uint16_t r, float scale) { return (float)r / scale; }

/* ── CRC16-CCITT ─────────────────────────────────────────────────────────── */

/* Byte-wise lookup table for CRC-16-CCITT (poly 0x1021, non-reflected, MSB-first).
   Table[b] = the bitwise CRC of the single byte b with a zero register, so the
   byte-wise update below is mathematically identical to the old bit-by-bit loop
   (and to the host's binascii.crc_hqx(..., 0xFFFF)) — the wire format is unchanged.
   `const` ⇒ the 512 B table lives in flash (.rodata), not RAM. Header-only: one
   copy per translation unit that CRCs (~512 B each of 512 KB flash). */
static const uint16_t PROTO_CRC16_TABLE[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0,
};

static inline uint16_t proto_crc16_update(uint16_t crc,
                                           const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; ++i) {
        crc = (uint16_t)((crc << 8u)
            ^ PROTO_CRC16_TABLE[((crc >> 8u) ^ data[i]) & 0xFFu]);
    }
    return crc;
}

static inline uint16_t proto_crc16(const uint8_t *data, size_t len)
{
    return proto_crc16_update(PROTO_CRC_INIT, data, len);
}

/* Compute CRC for a complete message (header.crc16 must be 0) */
static inline uint16_t proto_frame_crc(const void *hdr_raw,
                                        const uint8_t *payload,
                                        uint16_t pay_len)
{
    MsgHeader tmp;
    memcpy(&tmp, hdr_raw, MSG_HEADER_SIZE);
    tmp.crc16 = 0u;
    uint16_t crc = proto_crc16((const uint8_t *)&tmp, MSG_HEADER_SIZE);
    if (pay_len > 0u && payload != NULL) {
        crc = proto_crc16_update(crc, payload, pay_len);
    }
    return crc;
}

/* True for any defined MsgType — a cheap header-plausibility gate for resync. */
static inline uint8_t proto_msg_type_known(uint8_t t)
{
    switch (t) {
        case MSG_PING: case MSG_MASTER_STATUS: case MSG_SLAVE_STATUS:
        case MSG_ROBOT_CMD: case MSG_ROBOT_TELE: return 1u;
        default: return 0u;
    }
}

/* ── Resynchronizing frame scanner ───────────────────────────────────────────
   Finds the next complete, valid frame at buf[0]. Pure (no HAL), so the host
   tests (test_proto_frame) pin its recovery behavior. Header fields (type,
   version, length bound) are checked FIRST and the CRC only when the header is
   plausible, so scanning over garbage is cheap. On any header/CRC failure it
   reports RESYNC with consumed=1 (drop one byte and retry) — it never clears the
   whole buffer, so a single stray/lost byte can't desync it permanently. */
typedef enum {
    PROTO_SCAN_FRAME = 0,   /* a full valid frame is at buf[0]; see *out        */
    PROTO_SCAN_NEED_MORE,   /* plausible header, waiting for the rest of the frame */
    PROTO_SCAN_RESYNC       /* not a frame start; drop out->consumed bytes, retry  */
} ProtoScanStatus;

typedef struct {
    uint8_t        type;        /* FRAME: message type                 */
    uint16_t       pay_len;     /* FRAME: payload length               */
    const uint8_t *payload;     /* FRAME: -> payload within buf         */
    uint16_t       consumed;    /* FRAME: whole frame; RESYNC: bytes to drop */
} ProtoScanResult;

static inline ProtoScanStatus proto_frame_scan(const uint8_t *buf, uint16_t len,
                                               ProtoScanResult *out)
{
    if (len < MSG_HEADER_SIZE) return PROTO_SCAN_NEED_MORE;

    MsgHeader hdr;
    memcpy(&hdr, buf, MSG_HEADER_SIZE);

    /* Header-first plausibility (cheap) — reject garbage without hashing it. */
    if (!proto_msg_type_known((uint8_t)hdr.type) ||
        (uint8_t)(hdr.ver_flags & 0xFFu) != PROTO_VERSION ||
        hdr.len > PROTO_MAX_PAYLOAD) {
        out->consumed = 1u;
        return PROTO_SCAN_RESYNC;
    }

    uint16_t total = (uint16_t)(MSG_HEADER_SIZE + hdr.len);
    if (len < total) return PROTO_SCAN_NEED_MORE;    /* header ok; await the body */

    /* Header plausible and complete → now verify the CRC. */
    if (proto_frame_crc(&hdr, buf + MSG_HEADER_SIZE, hdr.len) != hdr.crc16) {
        out->consumed = 1u;
        return PROTO_SCAN_RESYNC;
    }

    out->type     = (uint8_t)hdr.type;
    out->pay_len  = hdr.len;
    out->payload  = buf + MSG_HEADER_SIZE;
    out->consumed = total;
    return PROTO_SCAN_FRAME;
}

/* Build a framed message into out[].  Returns total bytes written, 0 = error. */
static inline uint16_t proto_build(uint8_t *out, uint16_t cap,
                                    uint16_t type, uint16_t seq,
                                    uint8_t src,  uint8_t dst,
                                    uint32_t ts_ms,
                                    const uint8_t *payload, uint16_t pay_len)
{
    uint16_t total = (uint16_t)(MSG_HEADER_SIZE + pay_len);
    if (total > cap) return 0u;

    MsgHeader hdr;
    memset(&hdr, 0, sizeof(hdr));
    hdr.type      = type;
    hdr.seq       = seq;
    hdr.src       = src;
    hdr.dst       = dst;
    hdr.ts_ms     = ts_ms;
    hdr.len       = pay_len;
    hdr.ver_flags = PROTO_VERSION;   /* low byte = version, high byte reserved 0 */

    memcpy(out, &hdr, MSG_HEADER_SIZE);
    if (pay_len > 0u && payload != NULL) {
        memcpy(out + MSG_HEADER_SIZE, payload, pay_len);
    }
    ((MsgHeader *)out)->crc16 = proto_crc16(out, total);
    return total;
}

#ifdef __cplusplus
}
#endif
#endif /* PROTOCOL_H */
