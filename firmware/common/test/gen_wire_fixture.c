/* Cross-language wire fixture generator (protocol_wire.h ⇄ host ctypes).
 *
 * Fills every host-visible wire struct FIELD BY FIELD, in declaration order,
 * with values drawn from a shared 32-bit LCG, then prints each struct's raw
 * bytes as hex. host/tests/test_wire_fixture.py replays the identical LCG over
 * its ctypes mirrors (same declaration order) and compares the bytes — any
 * drift in field order, width, offset, padding or endianness between
 * protocol_wire.h and master_link/protocol.py fails the test. Also prints
 * sizeofs, the truncation macros, a built PING frame and a CRC, pinning the
 * frame codec too.
 *
 * Output lines:  NAME hexbytes...   |   SIZEOF name n   |   MACRO name n
 * Build: gcc -std=c11 -Wall -Wextra -Werror -I firmware/common/include ... */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "protocol.h"

static uint32_t lcg_state = 0xDEADBEEFu;
static uint32_t lcg(void) { lcg_state = lcg_state * 1664525u + 1013904223u; return lcg_state; }
static uint8_t  u8v(void)  { return (uint8_t)(lcg() & 0xFFu); }
static uint16_t u16v(void) { return (uint16_t)(lcg() & 0xFFFFu); }
static uint32_t u32v(void) { return lcg(); }
static int16_t  i16v(void) { return (int16_t)(lcg() & 0xFFFFu); }

/* ── per-struct fillers, declaration order, recursing into nested types ───── */
static void fill_rs_mit(rs_mit_t *m)
{ m->pos = i16v(); m->vel = i16v(); m->kp = u16v(); m->kd = u16v(); m->tau_ff = i16v(); }

static void fill_cmd_motor(cmd_motor_t *c) { c->req = u8v(); fill_rs_mit(&c->mit); }

static void fill_cmd_slave(cmd_slave_t *c)
{
    c->req = u8v();
    for (unsigned i = 0; i < MAX_MOTORS_PER_SLAVE; i++) fill_cmd_motor(&c->motors[i]);
}

static void fill_cmd_master(cmd_master_t *c)
{
    c->cycle_id = u16v(); c->cmd_seq = u16v(); c->test = u8v();
    for (unsigned i = 0; i < MAX_SLAVES; i++) fill_cmd_slave(&c->slaves[i]);
}

static void fill_rs_feedback(rs_feedback_t *f)
{
    f->pos = i16v(); f->vel = i16v(); f->tau = i16v(); f->temp = i16v();
    f->mode = u8v(); f->fault_bits = u8v(); f->fault_word = u32v();
    f->time_slave_us = u32v();
}

static void fill_tele_motor(tele_motor_t *t)
{
    t->state = u8v(); t->fault = u8v(); t->flags = u8v();
    t->last_applied_seq = u16v(); fill_rs_feedback(&t->rs);
}

static void fill_boot_result(boot_result_t *b)
{
    b->found_mask = u8v(); b->unexpected_id = u8v();
    b->zero_sta_fail_mask = u8v(); b->version_fail_mask = u8v();
}

static void fill_spi_slave_stats(spi_slave_stats_t *s)
{
    s->cmd_crc_errors = u16v(); s->resyncs = u16v();
    s->tx_arm_fails = u16v(); s->seq_echo = u8v();
}

static void fill_can_stats(can_stats_t *c)
{ c->tx_errors = u16v(); c->tec = u8v(); c->rec = u8v(); }

static void fill_status_slave(status_slave_t *s)
{ fill_boot_result(&s->boot); fill_spi_slave_stats(&s->spi); fill_can_stats(&s->can); }

static void fill_tele_slave(tele_slave_t *t)
{
    t->time_slave_us = u32v(); t->state = u8v(); t->n_motors = u8v();
    fill_status_slave(&t->status);
    for (unsigned i = 0; i < MAX_MOTORS_PER_SLAVE; i++) fill_tele_motor(&t->motors[i]);
}

static void fill_host_stats(host_stats_t *h)
{
    h->lost = u8v(); h->last_cmd_seq_rx = u16v(); h->on_time = u16v();
    h->late = u16v(); h->missing = u16v(); h->duplicate = u16v();
}

static void fill_usb_stats(usb_stats_t *u)
{
    u->rx_frames = u16v(); u->rx_errors = u16v(); u->rx_resyncs = u16v();
    u->rx_discarded_bytes = u16v(); u->tx_drops = u16v();
}

static void fill_spi_master_stats(spi_master_stats_t *s)
{ s->alive = u8v(); s->crc_errors = u16v(); s->seq_gaps = u16v(); }

static void fill_status_master(status_master_t *s)
{
    fill_host_stats(&s->host); fill_usb_stats(&s->usb);
    for (unsigned i = 0; i < MAX_SLAVES; i++) fill_spi_master_stats(&s->spi[i]);
    s->cycle_overruns = u16v();
}

static void fill_imu(imu_t *m)
{
    for (int i = 0; i < 3; i++) m->accel[i] = i16v();
    for (int i = 0; i < 3; i++) m->gyro[i] = i16v();
    m->seq = u8v(); m->flags = u8v(); m->errors = u16v();
}

static void fill_tele_master(tele_master_t *t)
{
    t->cycle_id = u16v(); t->time_master_us = u32v(); t->cmd_seq_active = u16v();
    t->robot_state = u8v(); t->n_slaves = u8v();
    fill_status_master(&t->status); fill_imu(&t->imu);
    for (unsigned i = 0; i < MAX_SLAVES; i++) fill_tele_slave(&t->slaves[i]);
}

static void fill_usb_header(usb_header_t *h)
{ h->type = u16v(); h->seq = u16v(); h->len = u16v(); h->crc16 = u16v(); }

static void dump(const char *name, const void *p, size_t n)
{
    printf("%s ", name);
    const uint8_t *b = (const uint8_t *)p;
    for (size_t i = 0; i < n; i++) printf("%02x", b[i]);
    printf("\n");
}

#define DUMP_FILLED(type, filler) do { \
    type v; memset(&v, 0, sizeof(v)); filler(&v); dump(#type, &v, sizeof(v)); } while (0)

int main(void)
{
    /* sizes + truncation macros first (order-independent lines) */
    printf("SIZEOF rs_mit_t %zu\n", sizeof(rs_mit_t));
    printf("SIZEOF cmd_motor_t %zu\n", sizeof(cmd_motor_t));
    printf("SIZEOF cmd_slave_t %zu\n", sizeof(cmd_slave_t));
    printf("SIZEOF cmd_master_t %zu\n", sizeof(cmd_master_t));
    printf("SIZEOF rs_feedback_t %zu\n", sizeof(rs_feedback_t));
    printf("SIZEOF tele_motor_t %zu\n", sizeof(tele_motor_t));
    printf("SIZEOF boot_result_t %zu\n", sizeof(boot_result_t));
    printf("SIZEOF spi_slave_stats_t %zu\n", sizeof(spi_slave_stats_t));
    printf("SIZEOF can_stats_t %zu\n", sizeof(can_stats_t));
    printf("SIZEOF status_slave_t %zu\n", sizeof(status_slave_t));
    printf("SIZEOF tele_slave_t %zu\n", sizeof(tele_slave_t));
    printf("SIZEOF host_stats_t %zu\n", sizeof(host_stats_t));
    printf("SIZEOF usb_stats_t %zu\n", sizeof(usb_stats_t));
    printf("SIZEOF spi_master_stats_t %zu\n", sizeof(spi_master_stats_t));
    printf("SIZEOF status_master_t %zu\n", sizeof(status_master_t));
    printf("SIZEOF imu_t %zu\n", sizeof(imu_t));
    printf("SIZEOF tele_master_t %zu\n", sizeof(tele_master_t));
    printf("SIZEOF usb_header_t %zu\n", sizeof(usb_header_t));
    for (unsigned n = 0; n <= MAX_SLAVES; n++) {
        printf("MACRO CMD_MASTER_SIZE_%u %zu\n", n, (size_t)CMD_MASTER_SIZE(n));
        printf("MACRO TELE_MASTER_SIZE_%u %zu\n", n, (size_t)TELE_MASTER_SIZE(n));
    }
    printf("MACRO SPI_XFER_SIZE %zu\n", (size_t)SPI_XFER_SIZE);

    /* field-by-field LCG fills — order here must match the Python replay */
    DUMP_FILLED(rs_mit_t, fill_rs_mit);
    DUMP_FILLED(cmd_motor_t, fill_cmd_motor);
    DUMP_FILLED(cmd_slave_t, fill_cmd_slave);
    DUMP_FILLED(cmd_master_t, fill_cmd_master);
    DUMP_FILLED(rs_feedback_t, fill_rs_feedback);
    DUMP_FILLED(tele_motor_t, fill_tele_motor);
    DUMP_FILLED(boot_result_t, fill_boot_result);
    DUMP_FILLED(spi_slave_stats_t, fill_spi_slave_stats);
    DUMP_FILLED(can_stats_t, fill_can_stats);
    DUMP_FILLED(status_slave_t, fill_status_slave);
    DUMP_FILLED(tele_slave_t, fill_tele_slave);
    DUMP_FILLED(host_stats_t, fill_host_stats);
    DUMP_FILLED(usb_stats_t, fill_usb_stats);
    DUMP_FILLED(spi_master_stats_t, fill_spi_master_stats);
    DUMP_FILLED(status_master_t, fill_status_master);
    DUMP_FILLED(imu_t, fill_imu);
    DUMP_FILLED(tele_master_t, fill_tele_master);
    DUMP_FILLED(usb_header_t, fill_usb_header);

    /* frame codec pin: a built PING frame with a 3-byte payload + a raw CRC */
    {
        static const uint8_t pay[3] = { 0x11, 0x22, 0x33 };
        uint8_t frame[64];
        uint16_t n = proto_build(frame, sizeof(frame), USB_MSG_PING, 0xBEEF, pay, 3);
        dump("FRAME_PING", frame, n);
        static const uint8_t crcbuf[5] = { 0x01, 0x02, 0x03, 0x04, 0x05 };
        printf("CRC16 %04x\n", proto_crc16(crcbuf, sizeof(crcbuf)));
    }
    return 0;
}
