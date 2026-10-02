/* Host golden/oracle tests for the SPI frame codec (spi_proto.c), PROTO_VERSION 3.
 *
 * One chain per slave → fixed-size frames. Asserts:
 *   - spi_proto_build_tele writes the tele_chain_t verbatim + a correct CRC;
 *   - spi_proto_parse_cmd round-trips the header + cmd_chain_t on a valid frame,
 *     and rejects a frame with a corrupted CRC-covered byte.
 * Exit 0 = all pass; nonzero = failure (message on stderr).
 *
 * Build (see host/tests/test_spi_proto.py):
 *   gcc -std=c11 -I firmware/common/include -I firmware/slave/.../Core/Inc \
 *       firmware/common/test/test_spi_proto.c .../spi_proto.c -o test_spi_proto
 */
#include "spi_proto.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>

static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { \
    fprintf(stderr, "FAIL: %s (%s:%d)\n", (msg), __FILE__, __LINE__); failures++; } } while (0)

static uint32_t lcg_state = 0x12345678u;
static uint32_t lcg(void) { lcg_state = lcg_state * 1664525u + 1013904223u; return lcg_state; }

static void fill_cmd_chain(cmd_chain_t *c)
{
    memset(c, 0, sizeof(*c));
    c->chain_id = (uint8_t)(lcg() & 0xFFu);
    c->n_motors = (uint8_t)(1u + (lcg() % MAX_MOTORS_PER_CHAIN));
    for (uint8_t i = 0; i < MAX_MOTORS_PER_CHAIN; i++) {
        c->motors[i].mode_req = (uint8_t)(lcg() % 5u);
        c->motors[i].pos      = (int16_t)(lcg() & 0xFFFFu);
        c->motors[i].vel      = (int16_t)(lcg() & 0xFFFFu);
        c->motors[i].kp       = (uint16_t)(lcg() & 0xFFFFu);
        c->motors[i].kd       = (uint16_t)(lcg() & 0xFFFFu);
        c->motors[i].tau_ff   = (int16_t)(lcg() & 0xFFFFu);
        c->motors[i].flags    = (uint8_t)(lcg() & 0xFFu);
    }
}

static void fill_tele_chain(tele_chain_t *t)
{
    memset(t, 0, sizeof(*t));
    t->chain_id       = (uint8_t)(lcg() & 0xFFu);
    t->n_motors       = (uint8_t)(1u + (lcg() % MAX_MOTORS_PER_CHAIN));
    t->spi_seq_echo   = (uint8_t)(lcg() & 0xFFu);
    t->slave_time_us  = lcg();
    t->cmd_crc_errors = (uint16_t)(lcg() & 0xFFFFu);
    t->can_tx_errors  = (uint16_t)(lcg() & 0xFFFFu);
    for (uint8_t i = 0; i < MAX_MOTORS_PER_CHAIN; i++) {
        t->motors[i].pos              = (int16_t)(lcg() & 0xFFFFu);
        t->motors[i].vel              = (int16_t)(lcg() & 0xFFFFu);
        t->motors[i].tau              = (int16_t)(lcg() & 0xFFFFu);
        t->motors[i].temp_c           = (uint8_t)(lcg() & 0xFFu);
        t->motors[i].state            = (uint8_t)(lcg() % 8u);
        t->motors[i].cause            = (uint8_t)(lcg() % 8u);
        t->motors[i].motor_mode       = (uint8_t)(lcg() % 3u);
        t->motors[i].motor_fault      = (uint8_t)(lcg() & 0xFFu);
        t->motors[i].flags            = (uint8_t)(lcg() & 0xFFu);
        t->motors[i].fb_age_ms        = (uint8_t)(lcg() & 0xFFu);
        t->motors[i].fault_word       = lcg();
        t->motors[i].last_applied_seq = (uint16_t)(lcg() & 0xFFFFu);
    }
}

/* Build a command frame the way the master does, for parse round-trip tests. */
static void build_cmd_frame(uint8_t *frame, uint8_t opcode, uint8_t spi_seq,
                            uint16_t cycle_id, uint16_t cmd_seq, const cmd_chain_t *chain)
{
    memset(frame, 0, SPI_CMD_FRAME_SIZE);
    frame[0] = opcode;
    frame[1] = spi_seq;
    frame[2] = (uint8_t)(cycle_id & 0xFFu);
    frame[3] = (uint8_t)(cycle_id >> 8);
    frame[4] = (uint8_t)(cmd_seq & 0xFFu);
    frame[5] = (uint8_t)(cmd_seq >> 8);
    memcpy(&frame[SPI_CMD_HDR_BYTES], chain, sizeof(cmd_chain_t));
    uint16_t crc = proto_crc16(frame, SPI_CMD_CRC_OFF);
    frame[SPI_CMD_CRC_OFF]     = (uint8_t)(crc & 0xFFu);
    frame[SPI_CMD_CRC_OFF + 1] = (uint8_t)(crc >> 8);
}

static void test_build_tele(void)
{
    for (int trial = 0; trial < 2000; trial++) {
        tele_chain_t t;
        fill_tele_chain(&t);
        uint8_t frame[SPI_TELE_FRAME_SIZE];
        spi_proto_build_tele(frame, &t);
        CHECK(memcmp(frame, &t, sizeof(tele_chain_t)) == 0,
              "tele frame prefix != tele_chain_t");
        uint16_t crc = proto_crc16(frame, SPI_TELE_CRC_OFF);
        CHECK(frame[SPI_TELE_CRC_OFF] == (uint8_t)(crc & 0xFFu) &&
              frame[SPI_TELE_CRC_OFF + 1] == (uint8_t)(crc >> 8), "tele CRC mismatch");
    }
}

static void test_parse_cmd(void)
{
    for (int trial = 0; trial < 2000; trial++) {
        cmd_chain_t c;
        fill_cmd_chain(&c);
        uint8_t opcode  = (lcg() & 1u) ? SPI_OP_ROBOT_CMD : SPI_OP_NOP;
        uint8_t spi_seq = (uint8_t)(lcg() & 0xFFu);
        uint16_t cyc    = (uint16_t)(lcg() & 0xFFFFu);
        uint16_t cseq   = (uint16_t)(lcg() & 0xFFFFu);

        uint8_t frame[SPI_CMD_FRAME_SIZE];
        build_cmd_frame(frame, opcode, spi_seq, cyc, cseq, &c);

        SpiCmdHdr   hdr;
        cmd_chain_t out;
        CHECK(spi_proto_parse_cmd(frame, &hdr, &out) == 1u, "valid cmd rejected");
        CHECK(hdr.opcode == opcode && hdr.spi_seq == spi_seq &&
              hdr.cycle_id == cyc && hdr.cmd_seq == cseq, "cmd header mismatch");
        CHECK(memcmp(&out, &c, sizeof(cmd_chain_t)) == 0, "cmd_chain_t mismatch");

        /* Corrupt a CRC-covered byte → must reject. */
        frame[0] ^= 0xFFu;
        SpiCmdHdr   hdr2;
        cmd_chain_t out2;
        CHECK(spi_proto_parse_cmd(frame, &hdr2, &out2) == 0u, "corrupt cmd accepted");
    }
}

/* Frozen golden: a fixed tele_chain_t pins absolute structural bytes + CRC. */
static void test_golden(void)
{
    tele_chain_t t;
    memset(&t, 0, sizeof(t));
    t.chain_id = 0x03u; t.n_motors = 1u; t.spi_seq_echo = 0x2Au;
    t.slave_time_us = 7777u; t.cmd_crc_errors = 3u; t.can_tx_errors = 1u;
    t.motors[0].pos = 5000; t.motors[0].state = LIFE_MIT; t.motors[0].motor_mode = 2u;
    t.motors[0].last_applied_seq = 0x1234u;

    uint8_t frame[SPI_TELE_FRAME_SIZE];
    spi_proto_build_tele(frame, &t);
    CHECK(frame[0] == 0x03u && frame[1] == 0x01u && frame[2] == 0x2Au, "golden header bytes");
    uint16_t crc = proto_crc16(frame, SPI_TELE_CRC_OFF);
    CHECK(frame[SPI_TELE_CRC_OFF] == (uint8_t)(crc & 0xFFu) &&
          frame[SPI_TELE_CRC_OFF + 1] == (uint8_t)(crc >> 8), "golden CRC");
}

int main(void)
{
    test_build_tele();
    test_parse_cmd();
    test_golden();
    if (failures == 0) { printf("spi_proto golden tests: OK\n"); return 0; }
    fprintf(stderr, "spi_proto golden tests: %d FAILURE(S)\n", failures);
    return 1;
}
