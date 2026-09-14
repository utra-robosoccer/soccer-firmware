/* Cross-language layout fixture.
 *
 * Emits golden hex for a MotorState atom and a full SPI telemetry frame so the
 * Python host library (host/jetson/protocol.py) can byte-compare against it and
 * catch C<->Python layout or CRC drift. Compiled and run by the pytest in
 * host/jetson/tests/test_protocol.py.
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

/* Fixed known values — MUST match the Python side of the test. */
static MotorState make_atom(uint16_t pos_raw, uint8_t life, uint8_t cause)
{
    MotorState m;
    memset(&m, 0, sizeof(m));
    m.pos_raw     = pos_raw;
    m.vel_raw     = 40000u;
    m.tau_raw     = 30000u;
    m.temp_c      = 42u;
    m.state       = SPI_STATE_PACK(life, cause);
    m.motor_fault = 0x0Au;   /* distinct from cmd_flags so a byte swap is caught */
    m.cmd_flags   = SPI_CMDFLAG_CLAMPED_POS | SPI_CMDFLAG_CMD_STALE;  /* 0x05 */
    m.fault_word  = 0xDEADBEEFu;
    m.fb_age      = 250u;
    m.reserved_v2 = 0u;
    return m;
}

int main(void)
{
    MotorState m0 = make_atom(12345u, MOTOR_ARMED_HOLD, CAUSE_OVERTORQUE);
    print_hex("ATOM", (const uint8_t *)&m0, (unsigned)sizeof(m0));

    /* Two-motor telemetry frame, independent of the build's N_MOTORS. */
    uint8_t frame[SPI_TELE_FRAME_SIZE(2)];
    memset(frame, 0, sizeof(frame));
    frame[0] = 0x03u;   /* alive_mask */
    frame[1] = 0x2Au;   /* echo_seq   */
    MotorState *ms = (MotorState *)&frame[SPI_TELE_HDR_BYTES];
    ms[0] = m0;
    ms[1] = make_atom(1000u, MOTOR_ARMED_MIT, CAUSE_NONE);
    uint16_t crc = proto_crc16(frame, (size_t)(sizeof(frame) - SPI_TELE_CRC_BYTES));
    frame[sizeof(frame) - 2] = (uint8_t)(crc & 0xFFu);
    frame[sizeof(frame) - 1] = (uint8_t)(crc >> 8);
    print_hex("FRAME", frame, (unsigned)sizeof(frame));
    return 0;
}
