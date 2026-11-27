/* Harness for the pure RobStride CAN codec (motor_rs_can.h), driven by
 * host/tests/test_motor_rs_can.py.
 *
 * Part 1: golden frame vectors — byte-exact encode checks (incl. the Type-17
 * zero_sta read frame verified live on the bench), decode round-trips for
 * the Type-0/2/17 replies, and the CAN-id-sweep roster checks. Part 2:
 * ext-id pack→unpack round-trips and the
 * 29-bit bound over 20 000 pseudo-random (mode,data,id) triples, printed as
 * "<mode> <data> <id> <ext>" so Python re-checks the wire formula.
 *
 * Built: gcc -std=c11 -Wall -Wextra -Werror -I firmware/common/include ... */
#include <stdint.h>
#include <stdio.h>
#include "protocol_wire.h"   /* RsMode (RS_MODE_RUN) */
#include "motor_rs_can.h"

static uint32_t g_lcg = 0x00C0FFEEu;
static uint32_t lcg_next(void)
{
    g_lcg = (uint32_t)(g_lcg * 1664525u + 1013904223u);
    return g_lcg;
}

static int frame_is(const char *what, RsFrame f, uint32_t ext, const uint8_t exp[8])
{
    if (f.ext_id != ext) {
        fprintf(stderr, "%s: ext %08lX want %08lX\n", what,
                (unsigned long)f.ext_id, (unsigned long)ext);
        return 0;
    }
    for (int i = 0; i < 8; ++i) {
        if (f.data[i] != exp[i]) {
            fprintf(stderr, "%s: byte%d %02X want %02X\n", what, i, f.data[i], exp[i]);
            return 0;
        }
    }
    return 1;
}

static int near(float a, float b, float tol) { float d = a - b; return (d < 0 ? -d : d) < tol; }

static int golden_checks(void)
{
    const uint8_t zeros[8] = {0};
    const uint8_t byte0_1[8] = {0x01, 0, 0, 0, 0, 0, 0, 0};

    if (!frame_is("enable", rs_enable_encode(5, 0xFD), 0x0300FD05u, zeros)) return 0;
    if (!frame_is("disable", rs_disable_encode(5, 0xFD), 0x0400FD05u, zeros)) return 0;
    if (!frame_is("clear_fault", rs_clear_fault_encode(5, 0xFD), 0x0400FD05u, byte0_1)) return 0;

    /* Type-4 version-read variant — the exact frame verified on the bench
     * (reply payload: 00 C4 then 6 version bytes). */
    const uint8_t verread[8] = {0x00, 0xC4, 0, 0, 0, 0, 0, 0};
    if (!frame_is("version_read", rs_version_read_encode(5, 0xFD), 0x0400FD05u, verread)) return 0;
    if (!frame_is("mech_zero", rs_mech_zero_encode(2, 0xFD), 0x0600FD02u, byte0_1)) return 0;

    /* Type-17 read of zero_sta (0x7029) — the exact frame verified on the bench. */
    const uint8_t rd7029[8] = {0x29, 0x70, 0, 0, 0, 0, 0, 0};
    if (!frame_is("param_read", rs_param_read_encode(1, 0xFD, 0x7029), 0x1100FD01u, rd7029)) return 0;

    /* Type-18 run-mode write (reg 0x7005, MIT = 0). */
    const uint8_t runmode[8] = {0x05, 0x70, 0, 0, 0, 0, 0, 0};
    if (!frame_is("runmode", rs_runmode_encode(3, 0xFD, MIT_MODE), 0x1200FD03u, runmode)) return 0;

    /* Type-1 MIT zero command, RS02 ranges: every centered field hits 0x7FFF
     * (floor of mid-scale), gains at range bottom hit 0. Torque in the ext-id. */
    const uint8_t mit0[8] = {0x7F, 0xFF, 0x7F, 0xFF, 0, 0, 0, 0};
    RsFrame mit = rs_mit_encode(&motor_rs_configs[MOTOR_MODEL_RS02], 7,
                                0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    if (!frame_is("mit_zero", mit, 0x017FFF07u, mit0)) return 0;

    /* Type-2 feedback decode: id 7, mode NORMAL, no faults, centered pos/vel/
     * torque, temp 28.0 C; then the wrong-id reject path. */
    motor_t m = {0};
    m.id = 7;
    uint32_t fb_ext = rs_extid_pack(2, (uint16_t)((2u << 14) | 7u), 0xFD);
    const uint8_t fb[8] = {0x7F, 0xFF, 0x7F, 0xFF, 0x7F, 0xFF, 0x01, 0x18};
    if (!rs_decode_feedback(fb_ext, fb, &motor_rs_configs[MOTOR_MODEL_RS02], &m)) {
        fprintf(stderr, "feedback: rejected\n"); return 0;
    }
    if (m.status != RS_MODE_RUN || m.motor_errors.overheat ||
        !near(m.pos, 0.0f, 0.001f) || !near(m.rpm, 0.0f, 0.002f) ||
        !near(m.torq, 0.0f, 0.001f) || !near(m.temperature, 28.0f, 0.001f)) {
        fprintf(stderr, "feedback: fields pos=%f rpm=%f torq=%f temp=%f\n",
                (double)m.pos, (double)m.rpm, (double)m.torq, (double)m.temperature);
        return 0;
    }
    m.id = 8;
    if (rs_decode_feedback(fb_ext, fb, &motor_rs_configs[MOTOR_MODEL_RS02], &m)) {
        fprintf(stderr, "feedback: wrong id accepted\n"); return 0;
    }

    /* Type-0 reply decode: LE MCU UID, reporting id corrects motor->id. */
    motor_t m0 = {0};
    m0.id = 1;
    const uint8_t uid[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    if (!rs_decode_get_id(rs_extid_pack(0, 7, 0xFD), uid, &m0) ||
        m0.id != 7 || m0.mcu_id != 0x0807060504030201ull) {
        fprintf(stderr, "get_id: id=%u uid=%016llX\n", m0.id,
                (unsigned long long)m0.mcu_id);
        return 0;
    }

    /* Type-17 reply decode: float view of bytes 4-7 (1.0f LE). */
    const uint8_t pv[8] = {0x29, 0x70, 0, 0, 0x00, 0x00, 0x80, 0x3F};
    float out = 0.0f;
    if (!rs_decode_single_param(rs_extid_pack(0x11, 1, 0xFD), pv, &out) ||
        !near(out, 1.0f, 1e-6f)) {
        fprintf(stderr, "single_param: out=%f\n", (double)out); return 0;
    }

    /* CAN id sweep roster checks (bitmap edges 1 and 0xFC included). */
    {
        const uint8_t cfg[3] = {1, 7, 0xFC};
        uint8_t seen[32] = {0};
        RsIdSweepResult r = rs_id_sweep_check(seen, cfg, 3);   /* empty bus */
        if (r.found_mask != 0x00u || r.unexpected_id != 0u || r.ok) {
            fprintf(stderr, "sweep empty: mask=%02X unexp=%u ok=%u\n",
                    r.found_mask, r.unexpected_id, r.ok); return 0;
        }

        rs_id_sweep_mark(seen, 1);
        rs_id_sweep_mark(seen, 0xFC);
        r = rs_id_sweep_check(seen, cfg, 3);                   /* 7 missing */
        if (r.found_mask != 0x05u || r.unexpected_id != 0u || r.ok) {
            fprintf(stderr, "sweep missing: mask=%02X unexp=%u ok=%u\n",
                    r.found_mask, r.unexpected_id, r.ok); return 0;
        }

        rs_id_sweep_mark(seen, 7);
        r = rs_id_sweep_check(seen, cfg, 3);                   /* exact match */
        if (r.found_mask != 0x07u || r.unexpected_id != 0u || !r.ok) {
            fprintf(stderr, "sweep ok: mask=%02X unexp=%u ok=%u\n",
                    r.found_mask, r.unexpected_id, r.ok); return 0;
        }

        rs_id_sweep_mark(seen, 5);                             /* extra id on bus */
        r = rs_id_sweep_check(seen, cfg, 3);
        if (r.found_mask != 0x07u || r.unexpected_id != 5u || r.ok) {
            fprintf(stderr, "sweep extra: mask=%02X unexp=%u ok=%u\n",
                    r.found_mask, r.unexpected_id, r.ok); return 0;
        }

        /* Missing AND extra together (one motor on a wrong id looks like this). */
        const uint8_t cfg4[4] = {1, 7, 9, 0xFC};
        r = rs_id_sweep_check(seen, cfg4, 4);                  /* 9 missing, 5 extra */
        if (r.found_mask != 0x0Bu || r.unexpected_id != 5u || r.ok) {
            fprintf(stderr, "sweep wrongid: mask=%02X unexp=%u ok=%u\n",
                    r.found_mask, r.unexpected_id, r.ok); return 0;
        }
    }
    return 1;
}

int main(void)
{
    if (!golden_checks()) {
        return 3;
    }
    for (int i = 0; i < 20000; ++i) {
        uint8_t  mode = (uint8_t)(lcg_next() & 0x1Fu);
        uint16_t data = (uint16_t)(lcg_next() & 0xFFFFu);
        uint8_t  id   = (uint8_t)(lcg_next() & 0xFFu);

        uint32_t ext = rs_extid_pack(mode, data, id);
        if (rs_extid_mode(ext) != mode || rs_extid_data(ext) != data ||
            rs_extid_id(ext) != id) {
            fprintf(stderr, "roundtrip fail i=%d\n", i);
            return 1;
        }
        if ((ext >> 29) != 0u) {               /* must fit the 29-bit ext id */
            fprintf(stderr, "bits>28 set i=%d ext=%08lX\n", i, (unsigned long)ext);
            return 2;
        }
        printf("%u %u %u %08lX\n", mode, data, id, (unsigned long)ext);
    }
    return 0;
}
