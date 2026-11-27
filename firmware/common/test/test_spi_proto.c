/* Host golden/oracle tests for the SPI frame codec (spi_proto.c).
 *
 * Fixed-size frames (protocol_wire.h): spi_cmd_frame_t = 64 B
 * {spi_header_t, cmd_slave_t, crc16}, spi_tele_frame_t = 138 B
 * {tele_slave_t, crc16}, both CRC16-CCITT over everything before the crc field,
 * CRC stored little-endian. Asserts:
 *   - spi_proto_build_tele writes the tele_slave_t verbatim + a correct CRC;
 *   - spi_proto_parse_cmd round-trips header + cmd_slave_t on a valid frame,
 *     and rejects a frame with any corrupted CRC-covered byte;
 *   - a frozen golden command frame parses to frozen field values (layout pin).
 * Exit 0 = all pass; nonzero = failure (message on stderr).
 *
 * Build (see host/tests/test_spi_proto.py):
 *   gcc -std=c11 -I firmware/common/include -I firmware/slave/.../Core/Inc \
 *       firmware/common/test/test_spi_proto.c .../spi_proto.c -o test_spi_proto
 */
#include "spi_proto.h"
#include "protocol.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stddef.h>

static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { \
    fprintf(stderr, "FAIL: %s (%s:%d)\n", (msg), __FILE__, __LINE__); failures++; } } while (0)

static uint32_t lcg_state = 0x12345678u;
static uint32_t lcg(void) { lcg_state = lcg_state * 1664525u + 1013904223u; return lcg_state; }

static void fill_random(void *p, size_t n)
{
    uint8_t *b = (uint8_t *)p;
    for (size_t i = 0; i < n; i++) b[i] = (uint8_t)(lcg() & 0xFFu);
}

/* Reference encoder for a command frame: lay the bytes out by offsetof, CRC LE. */
static void ref_build_cmd(uint8_t *frame, const spi_header_t *hdr, const cmd_slave_t *cmd)
{
    memset(frame, 0, sizeof(spi_cmd_frame_t));
    memcpy(frame + offsetof(spi_cmd_frame_t, hdr), hdr, sizeof(*hdr));
    memcpy(frame + offsetof(spi_cmd_frame_t, cmd), cmd, sizeof(*cmd));
    uint16_t crc = proto_crc16(frame, offsetof(spi_cmd_frame_t, crc16));
    frame[offsetof(spi_cmd_frame_t, crc16)]      = (uint8_t)(crc & 0xFFu);
    frame[offsetof(spi_cmd_frame_t, crc16) + 1u] = (uint8_t)(crc >> 8);
}

int main(void)
{
    /* ── telemetry build: verbatim struct + correct CRC, randomized ─────────── */
    for (int it = 0; it < 2000; it++) {
        tele_slave_t t;
        fill_random(&t, sizeof(t));
        uint8_t frame[sizeof(spi_tele_frame_t)];
        memset(frame, 0xEE, sizeof(frame));
        spi_proto_build_tele(frame, &t);

        CHECK(memcmp(frame, &t, sizeof(t)) == 0, "tele payload verbatim");
        uint16_t crc = proto_crc16(frame, offsetof(spi_tele_frame_t, crc16));
        uint16_t got = (uint16_t)(frame[offsetof(spi_tele_frame_t, crc16)]
                     | ((uint16_t)frame[offsetof(spi_tele_frame_t, crc16) + 1u] << 8));
        CHECK(got == crc, "tele CRC little-endian over payload");
    }

    /* ── command parse: round-trip + reject-on-corruption, randomized ───────── */
    for (int it = 0; it < 2000; it++) {
        spi_header_t h;
        cmd_slave_t c;
        fill_random(&h, sizeof(h));
        fill_random(&c, sizeof(c));
        uint8_t frame[sizeof(spi_cmd_frame_t)];
        ref_build_cmd(frame, &h, &c);

        spi_header_t ph;
        cmd_slave_t pc;
        CHECK(spi_proto_parse_cmd(frame, &ph, &pc) == 1u, "valid cmd accepted");
        CHECK(memcmp(&ph, &h, sizeof(h)) == 0, "header round-trip");
        CHECK(memcmp(&pc, &c, sizeof(c)) == 0, "cmd round-trip");

        /* corrupt one CRC-covered byte (or a CRC byte) → rejected */
        uint16_t pos = (uint16_t)(lcg() % sizeof(spi_cmd_frame_t));
        frame[pos] ^= (uint8_t)(1u << (lcg() % 8u));
        CHECK(spi_proto_parse_cmd(frame, &ph, &pc) == 0u, "corrupted cmd rejected");
    }

    /* ── frozen golden command frame: pins the layout itself ────────────────── */
    {
        spi_header_t h;
        memset(&h, 0, sizeof(h));
        h.opcode   = SPI_OP_CMD;
        h.spi_seq  = 0xA5u;
        h.cycle_id = 0xBEEF;
        h.cmd_seq  = 0x1234;

        cmd_slave_t c;
        memset(&c, 0, sizeof(c));
        c.req = SLAVE_REQ_RESET;
        for (uint8_t i = 0; i < MAX_MOTORS_PER_SLAVE; i++) {
            c.motors[i].req        = (uint8_t)(i & 1u);     /* STOP/ENABLE alternating */
            c.motors[i].mit.pos    = (int16_t)(1000 * (i + 1));
            c.motors[i].mit.vel    = (int16_t)(-200 * (i + 1));
            c.motors[i].mit.kp     = (uint16_t)(300u + i);
            c.motors[i].mit.kd     = (uint16_t)(40u + i);
            c.motors[i].mit.tau_ff = (int16_t)(7 * (i + 1));
        }
        uint8_t frame[sizeof(spi_cmd_frame_t)];
        ref_build_cmd(frame, &h, &c);

        /* Spot-check frozen absolute offsets (hdr 6 B, motor stride 11 B). */
        CHECK(frame[0] == SPI_OP_CMD, "golden: opcode at byte 0");
        CHECK(frame[1] == 0xA5u, "golden: seq at byte 1");
        CHECK(frame[2] == 0xEFu && frame[3] == 0xBEu, "golden: cycle_id LE at 2..3");
        CHECK(frame[4] == 0x34u && frame[5] == 0x12u, "golden: cmd_seq LE at 4..5");
        CHECK(frame[6] == SLAVE_REQ_RESET, "golden: slave req at byte 6");
        CHECK(frame[7] == 0u, "golden: motor0 req");
        CHECK(frame[8] == 0xE8u && frame[9] == 0x03u, "golden: motor0 pos=1000 LE");
        CHECK(frame[18] == 1u, "golden: motor1 req at 7+11");

        spi_header_t ph; cmd_slave_t pc;
        CHECK(spi_proto_parse_cmd(frame, &ph, &pc) == 1u, "golden parses");
        CHECK(pc.motors[4].mit.pos == 5000 && pc.motors[4].mit.vel == -1000,
              "golden: motor4 fields");
    }

    if (failures) { fprintf(stderr, "%d failure(s)\n", failures); return 1; }
    printf("OK spi_proto\n");
    return 0;
}
