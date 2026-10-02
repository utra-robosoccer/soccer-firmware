/* Cross-language layout fixture (PROTO_VERSION 3, robot/chain/motor hierarchy).
 *
 * Emits golden hex for a full cmd_robot_t and tele_robot_t built with protocol.h,
 * so host/tests/test_protocol.py can byte-compare against the same values packed
 * in Python and catch C↔Python layout / scale / endianness drift.
 *
 * Build:  gcc -std=c11 -I firmware/common/include \
 *             firmware/common/test/gen_fixture.c -o gen_fixture
 */
#include "protocol.h"
#include <stdio.h>
#include <string.h>

static void print_hex(const char *label, const uint8_t *b, unsigned n)
{
    printf("%s", label);
    for (unsigned i = 0; i < n; i++) printf(" %02x", b[i]);
    printf("\n");
}

int main(void)
{
    /* ── cmd_robot_t: 1 chain, 1 motor, known values ── */
    cmd_robot_t c;
    memset(&c, 0, sizeof(c));
    c.cycle_id = 7u;
    c.cmd_seq  = 42u;
    c.n_chains = 1u;
    c.chains[0].chain_id = 0u;
    c.chains[0].n_motors = 1u;
    cmd_motor_t *m = &c.chains[0].motors[0];
    m->mode_req = REQ_MIT;
    m->pos    = proto_f_to_i16(0.5f,   PROTO_POS_SCALE, NULL);   /* 5000  */
    m->vel    = proto_f_to_i16(-1.25f, PROTO_VEL_SCALE, NULL);   /* -125  */
    m->kp     = proto_f_to_u16(15.0f,  PROTO_KP_SCALE, NULL);    /* 150   */
    m->kd     = proto_f_to_u16(1.0f,   PROTO_KD_SCALE, NULL);    /* 100   */
    m->tau_ff = proto_f_to_i16(0.3f,   PROTO_TAU_SCALE, NULL);   /* 30    */
    m->flags  = CMD_FLAG_VALID;
    print_hex("ROBOTCMD", (const uint8_t *)&c, (unsigned)sizeof(c));

    /* ── tele_robot_t: 1 chain, 1 motor, known values ── */
    tele_robot_t t;
    memset(&t, 0, sizeof(t));
    t.cycle_id         = 9u;
    t.master_time_us   = 123456u;
    t.last_cmd_seq_rx  = 42u;
    t.missed_deadlines = 0u;
    t.n_chains         = 1u;
    t.robot_state      = ROBOT_READY;
    t.chains[0].chain_id       = 0u;
    t.chains[0].n_motors       = 1u;
    t.chains[0].spi_seq_echo   = 0x2Au;
    t.chains[0].slave_time_us  = 7777u;
    t.chains[0].cmd_crc_errors = 3u;
    t.chains[0].can_tx_errors  = 1u;
    tele_motor_t *tm = &t.chains[0].motors[0];
    tm->pos         = proto_f_to_i16(0.5f,   PROTO_POS_SCALE, NULL);  /* 5000 */
    tm->vel         = proto_f_to_i16(-1.25f, PROTO_VEL_SCALE, NULL);  /* -125 */
    tm->tau         = proto_f_to_i16(0.3f,   PROTO_TAU_SCALE, NULL);  /* 30   */
    tm->temp_c      = 42u;
    tm->state       = LIFE_MIT;
    tm->cause       = CAUSE_NONE;
    tm->motor_mode  = 2u;
    tm->motor_fault = 0x0Au;
    tm->flags       = TELE_FLAG_TO_ZERO_ARRIVED;
    tm->fb_age_ms   = 250u;
    tm->fault_word  = 0xDEADBEEFu;
    tm->last_applied_seq = 0x1234u;
    tm->reserved    = 0u;
    print_hex("ROBOTTELE", (const uint8_t *)&t, (unsigned)sizeof(t));

    return 0;
}
