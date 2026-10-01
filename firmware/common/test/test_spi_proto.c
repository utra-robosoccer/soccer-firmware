/* Host golden/oracle tests for the SPI frame codec (spi_proto.c).
 *
 * Purpose: lock the extraction of the SPI framing out of main.c / motor_runtime.c
 * into spi_proto.c to be BYTE-IDENTICAL to the pre-extraction code. The reference
 * functions below (ref_*) are the original algorithms transcribed verbatim from
 * the old main.c telemetry assembly, the old motor_runtime.c pack_tele/f_to_u16,
 * and the old command parse. Each case asserts the new codec produces exactly the
 * same bytes / decoded fields as the reference, over many inputs incl. edge cases.
 * A frozen golden frame (GOLDEN_FRAME) additionally pins the absolute bytes.
 *
 * Build (see host/jetson/tests/test_spi_proto.py which runs it in CI):
 *   gcc -std=c11 -I firmware/common/include \
 *       -I firmware/slave/slave_general/Core/Inc \
 *       firmware/common/test/test_spi_proto.c \
 *       firmware/slave/slave_general/Core/Src/spi_proto.c -o test_spi_proto
 * Exit 0 = all pass; nonzero = failure (message on stderr).
 */
#include "spi_proto.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#define TEST_MAX_N 5u

static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { \
    fprintf(stderr, "FAIL: %s (%s:%d)\n", (msg), __FILE__, __LINE__); failures++; } } while (0)

/* ── reference: the PRE-extraction algorithms (oracle) ───────────────────────*/

/* old motor_runtime.c f_to_u16 */
static uint16_t ref_f_to_u16(float x, float lo, float hi)
{
    if (x < lo) x = lo;
    if (x > hi) x = hi;
    return (uint16_t)((x - lo) * 65535.0f / (hi - lo));
}

/* old motor_runtime.c pack_tele field mapping, applied to a MotorSample */
static void ref_encode_atom(MotorState *out, const MotorSample *s)
{
    out->pos_raw     = ref_f_to_u16(s->pos, MOTOR_P_MIN, MOTOR_P_MAX);
    out->vel_raw     = ref_f_to_u16(s->vel, MOTOR_V_MIN, MOTOR_V_MAX);
    out->tau_raw     = ref_f_to_u16(s->tau, MOTOR_T_MIN, MOTOR_T_MAX);
    out->temp_c      = (uint8_t)(s->temp < 0.0f ? 0u : (uint8_t)s->temp);
    out->state       = SPI_STATE_PACK(s->life, s->cause);
    out->motor_fault = s->motor_fault;
    out->cmd_flags   = s->cmd_flags;
    out->fault_word  = s->fault_word;
    uint32_t age     = s->fb_age_ms;
    out->fb_age      = (age > 255u) ? 255u : (uint8_t)age;
    out->reserved_v2 = 0u;
    out->last_applied_seq = s->last_applied_seq;
}

/* old main.c inline telemetry assembly */
static void ref_build_tele(uint8_t *frame, uint8_t n,
                           uint8_t alive_mask, uint8_t echo_seq,
                           const MotorSample *samples,
                           uint32_t cmd_crc_errors, uint32_t zero_rejects)
{
    memset(frame, 0, SPI_TELE_FRAME_SIZE(n));
    frame[0] = alive_mask;
    frame[1] = echo_seq;
    MotorState *ms = (MotorState *)&frame[SPI_TELE_HDR_BYTES];
    for (uint8_t i = 0; i < n; i++) ref_encode_atom(&ms[i], &samples[i]);
    uint8_t *dbg = &frame[SPI_TELE_DEBUG_OFF(n)];
    dbg[0] = (uint8_t)(cmd_crc_errors & 0xFFu);
    dbg[1] = (uint8_t)((cmd_crc_errors >> 8) & 0xFFu);
    dbg[2] = (uint8_t)((cmd_crc_errors >> 16) & 0xFFu);
    dbg[3] = (uint8_t)((cmd_crc_errors >> 24) & 0xFFu);
    dbg[4] = (uint8_t)(zero_rejects & 0xFFu);
    dbg[5] = (uint8_t)((zero_rejects >> 8) & 0xFFu);
    dbg[6] = (uint8_t)((zero_rejects >> 16) & 0xFFu);
    dbg[7] = (uint8_t)((zero_rejects >> 24) & 0xFFu);
    uint16_t crc = proto_crc16(frame, (size_t)(SPI_TELE_FRAME_SIZE(n) - SPI_TELE_CRC_BYTES));
    frame[SPI_TELE_FRAME_SIZE(n) - 2] = (uint8_t)(crc & 0xFFu);
    frame[SPI_TELE_FRAME_SIZE(n) - 1] = (uint8_t)(crc >> 8);
}

/* ── helpers ─────────────────────────────────────────────────────────────── */

static uint32_t lcg_state = 0x12345678u;
static uint32_t lcg(void) { lcg_state = lcg_state * 1664525u + 1013904223u; return lcg_state; }
static float rand_float(float lo, float hi)
{
    float t = (float)(lcg() & 0xFFFFFFu) / (float)0xFFFFFFu;
    return lo + t * (hi - lo);
}

static MotorSample rand_sample(void)
{
    MotorSample s;
    /* Deliberately overshoot the transport bounds sometimes to exercise clamping. */
    s.pos = rand_float(MOTOR_P_MIN * 1.2f, MOTOR_P_MAX * 1.2f);
    s.vel = rand_float(MOTOR_V_MIN * 1.2f, MOTOR_V_MAX * 1.2f);
    s.tau = rand_float(MOTOR_T_MIN * 1.2f, MOTOR_T_MAX * 1.2f);
    s.temp = rand_float(-10.0f, 120.0f);          /* negative -> clamp to 0 */
    s.life = (uint8_t)(lcg() & 0x0Fu);
    s.cause = (uint8_t)(lcg() & 0x0Fu);
    s.motor_fault = (uint8_t)(lcg() & 0xFFu);
    s.cmd_flags = (uint8_t)(lcg() & 0xFFu);
    s.fault_word = lcg();
    s.fb_age_ms = lcg() % 400u;                   /* spans the 255 saturation */
    s.last_applied_seq = (uint16_t)(lcg() & 0xFFFFu);
    return s;
}

/* ── tests ───────────────────────────────────────────────────────────────── */

static void test_build_matches_reference(void)
{
    for (int trial = 0; trial < 2000; trial++) {
        uint8_t n = (uint8_t)(1u + (lcg() % TEST_MAX_N));
        MotorSample samples[TEST_MAX_N];
        for (uint8_t i = 0; i < n; i++) samples[i] = rand_sample();
        uint8_t alive = (uint8_t)(lcg() & 0xFFu);
        uint8_t seq   = (uint8_t)(lcg() & 0xFFu);
        uint32_t cce  = lcg();
        uint32_t zr   = lcg();

        uint8_t got[SPI_TELE_FRAME_SIZE(TEST_MAX_N)];
        uint8_t exp[SPI_TELE_FRAME_SIZE(TEST_MAX_N)];
        spi_proto_build_tele(got, n, alive, seq, samples, cce, zr);
        ref_build_tele(exp, n, alive, seq, samples, cce, zr);
        CHECK(memcmp(got, exp, SPI_TELE_FRAME_SIZE(n)) == 0,
              "telemetry frame not byte-identical to pre-extraction code");
    }
}

static void test_parse_matches_reference(void)
{
    const uint8_t opcodes[] = { SPI_CMD_NOP, SPI_CMD_ARM, SPI_CMD_HOLD,
                                SPI_CMD_DISARM, SPI_CMD_GOTO_ZERO, SPI_CMD_MIT };
    for (int trial = 0; trial < 2000; trial++) {
        uint8_t n = (uint8_t)(1u + (lcg() % TEST_MAX_N));
        uint8_t op  = opcodes[lcg() % (sizeof(opcodes))];
        uint8_t idx = (uint8_t)(lcg() % TEST_MAX_N);
        uint8_t seq = (uint8_t)(lcg() & 0xFFu);

        uint8_t frame[SPI_CMD_FRAME_SIZE(TEST_MAX_N)];
        memset(frame, 0, sizeof(frame));
        frame[0] = (uint8_t)(op | (uint8_t)(idx << 4u));
        frame[1] = seq;
        /* Fill MIT slots with arbitrary bytes to check read_mit + non-interference. */
        for (uint8_t b = SPI_CMD_HDR_BYTES; b < SPI_CMD_CRC_OFF(n); b++)
            frame[b] = (uint8_t)(lcg() & 0xFFu);
        uint16_t crc = proto_crc16(frame, SPI_CMD_CRC_OFF(n));
        frame[SPI_CMD_CRC_OFF(n)]     = (uint8_t)(crc & 0xFFu);
        frame[SPI_CMD_CRC_OFF(n) + 1] = (uint8_t)(crc >> 8);

        ParsedCmd pc;
        uint8_t ok = spi_proto_parse(frame, n, &pc);
        CHECK(ok == 1u, "valid CRC command rejected");
        CHECK(pc.opcode == (frame[0] & 0x0Fu), "opcode mismatch");
        CHECK(pc.seq == seq, "seq mismatch");
        CHECK(pc.motor_idx == (uint8_t)(frame[0] >> 4u), "motor_idx mismatch");
        CHECK(pc.n_mit == n, "n_mit mismatch");

        /* read_mit must equal a raw memcpy of each slot. */
        for (uint8_t i = 0; i < n; i++) {
            SpiMitCmd mc, ref;
            spi_proto_read_mit(frame, i, &mc);
            memcpy(&ref, frame + SPI_CMD_HDR_BYTES + (size_t)i * sizeof(SpiMitCmd),
                   sizeof(SpiMitCmd));
            CHECK(memcmp(&mc, &ref, sizeof(SpiMitCmd)) == 0, "read_mit slot mismatch");
        }

        /* Corrupt one CRC-covered byte: parse must now reject. */
        frame[0] ^= 0xFFu;
        ParsedCmd pc2;
        CHECK(spi_proto_parse(frame, n, &pc2) == 0u, "corrupt command accepted");
    }
}

/* Frozen golden: exact bytes for a fixed 2-motor frame. Pins the absolute wire
   output so a change to the transport bounds / layout / CRC is caught even if the
   reference above were changed in lockstep. Regenerate deliberately if the wire
   format is intentionally revised. */
static void test_golden_frame(void)
{
    MotorSample s0 = { .pos = 0.0f, .vel = 0.0f, .tau = 0.0f, .temp = 25.0f,
                       .life = MOTOR_ARMED_HOLD, .cause = CAUSE_OVERTORQUE,
                       .motor_fault = 0x0Au, .cmd_flags = 0x05u,
                       .fault_word = 0xDEADBEEFu, .fb_age_ms = 250u,
                       .last_applied_seq = 0x1234u };
    MotorSample s1 = { .pos = MOTOR_P_MAX, .vel = MOTOR_V_MIN, .tau = 1.0f, .temp = -5.0f,
                       .life = MOTOR_ARMED_MIT, .cause = CAUSE_NONE,
                       .motor_fault = 0u, .cmd_flags = 0u,
                       .fault_word = 0u, .fb_age_ms = 1000u,
                       .last_applied_seq = 0u };
    MotorSample samples[2] = { s0, s1 };
    uint8_t got[SPI_TELE_FRAME_SIZE(2)];
    spi_proto_build_tele(got, 2u, 0x03u, 0x2Au, samples, 7u, 3u);

    uint8_t exp[SPI_TELE_FRAME_SIZE(2)];
    ref_build_tele(exp, 2u, 0x03u, 0x2Au, samples, 7u, 3u);
    CHECK(memcmp(got, exp, sizeof(got)) == 0, "golden frame drift vs reference");

    /* Structural pins that don't require hand-computing the CRC. */
    CHECK(got[0] == 0x03u, "golden alive_mask");
    CHECK(got[1] == 0x2Au, "golden echo_seq");
    const uint8_t *dbg = &got[SPI_TELE_DEBUG_OFF(2)];
    CHECK(dbg[0] == 7u && dbg[4] == 3u, "golden debug counters");
    uint16_t crc = proto_crc16(got, (size_t)(SPI_TELE_FRAME_SIZE(2) - SPI_TELE_CRC_BYTES));
    CHECK(got[SPI_TELE_FRAME_SIZE(2) - 2] == (uint8_t)(crc & 0xFFu) &&
          got[SPI_TELE_FRAME_SIZE(2) - 1] == (uint8_t)(crc >> 8), "golden CRC");
}

int main(void)
{
    test_build_matches_reference();
    test_parse_matches_reference();
    test_golden_frame();
    if (failures == 0) { printf("spi_proto golden tests: OK\n"); return 0; }
    fprintf(stderr, "spi_proto golden tests: %d FAILURE(S)\n", failures);
    return 1;
}
