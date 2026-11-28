/* master_app — see master_app.h. Policy and orchestration only: every wire byte
 * goes through spi_link (slaves) or usb_link (host). */
#include "master_app.h"
#include "master_cycle.h"
#include "spi_link.h"
#include "usb_link.h"
#include "imu.h"   /* BMI088 IMU on I2C1 (raw sample → tele_master_t.imu) */
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

static UART_HandleTypeDef *master_huart = NULL;

/* The 200 Hz cycle is driven by the TIM2 hardware interrupt (master_cycle): one
 * cycle = poll all slaves then emit tele_master_t (status + IMU nested — there
 * is no separate status message). Rates come from the generated MASTER_POLL_HZ. */

/* ── host-command mailbox (pure logic: common/include/cmd_mailbox.h) ───────
   Double-buffered, apply-at-n+1, latest-wins. USB dispatch posts; the swap at the
   start of every master cycle publishes to the ACTIVE buffer the slave polls read. */
static CmdMailbox  g_mbox;
static uint16_t    master_cycle_id = 0;            /* master cycle counter (per cycle) */

/* Host-death dead-man: the FSM that forces STOP to the slaves when the host
   goes quiet while armed; its inputs live in the mailbox. */
static HostWatchdog  g_host_wd;
static HostLinkAction g_host_action = HOST_LINK_OK;

/* Configured slaves that have been seen alive at least once since boot — a
   slave that has NEVER answered is still booting (ROBOT_BOOT); one that
   answered and then went silent is a failure (ROBOT_DEGRADED). */
static uint8_t g_slave_seen[NUM_SLAVES];

/* ── USB app-level counters (transport counters live in usb_link) ─────────── */
uint32_t master_link_errors = 0;   /* host frames rejected here (bad type/length) */
#ifdef USB_TX_TEST
static volatile uint8_t g_txtest = 0; /* test-only: armed by a 0xB0 cmd.test sentinel */
#endif

static uint16_t tx_seq = 0;

/* The CDC stream carries ONLY framed protocol messages — there is deliberately
   NO printf-to-USB path. Diagnostics go to UART4 (dedicated PA0/PA1) via
   debug_printf, compiled out unless the build defines MASTER_DEBUG_UART.
   NOTE: this uses BLOCKING HAL_UART_Transmit (~87 µs/char budget at 115200),
   so MASTER_DEBUG_UART builds are NOT timing-representative — debug only,
   never a release flag. */
#ifdef MASTER_DEBUG_UART
void debug_printf(const char *fmt, ...)
{
    char buf[128];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n > 0 && master_huart != NULL) {
        if (n > (int)sizeof(buf) - 1) n = (int)sizeof(buf) - 1;
        HAL_UART_Transmit(master_huart, (uint8_t *)buf, (uint16_t)n, 20u);
    }
}
#endif

/* Aggregate robot state from the slaves' telemetry (summary only; drives
   nothing — see the RobotState comments in protocol_wire.h). */
static RobotState compute_robot_state(void)
{
    uint8_t any_boot = 0u, any_blocked = 0u, all_running = 1u, all_ready = 1u;
    for (uint8_t s = 0; s < NUM_SLAVES; s++) {
        const SpiLinkSlave *sl = spi_link_slave(s);
        if (sl->alive) g_slave_seen[s] = 1u;
        if (!g_slave_seen[s]) {              /* never answered: still booting */
            any_boot = 1u; all_running = 0u; all_ready = 0u;
            continue;
        }
        if (!sl->alive) {                    /* answered once, silent now */
            all_running = 0u; all_ready = 0u;
            continue;
        }
        uint8_t st = sl->tele.state;
        if (st == SLAVE_BOOT)    any_boot = 1u;
        if (st == SLAVE_BLOCKED) any_blocked = 1u;
        if (st != SLAVE_READY) { all_running = 0u; all_ready = 0u; }
        uint8_t nm = (sl->tele.n_motors > MAX_MOTORS_PER_SLAVE)
                     ? MAX_MOTORS_PER_SLAVE : sl->tele.n_motors;
        for (uint8_t i = 0; i < nm; i++) {
            const tele_motor_t *m = &sl->tele.motors[i];
            if (m->state != MOTOR_ENABLED)          all_running = 0u;
            if (!(m->flags & MOTOR_FLAG_READY))     all_ready = 0u;
        }
    }
    if (any_boot)    return ROBOT_BOOT;
    if (any_blocked) return ROBOT_BLOCKED;
    if (all_running) return ROBOT_RUNNING;
    if (all_ready)   return ROBOT_READY;
    return ROBOT_DEGRADED;
}

/* Any motor currently armed (ENABLING/ENABLED/STOPPING), per the latest telemetry —
   gates the host-death watchdog so it never trips when everything is already off. */
static uint8_t any_motor_armed(void)
{
    for (uint8_t s = 0; s < NUM_SLAVES; s++) {
        const SpiLinkSlave *sl = spi_link_slave(s);
        if (!sl->alive) continue;
        uint8_t nm = (sl->tele.n_motors > MAX_MOTORS_PER_SLAVE)
                     ? MAX_MOTORS_PER_SLAVE : sl->tele.n_motors;
        for (uint8_t i = 0; i < nm; i++) {
            uint8_t st = sl->tele.motors[i].state;
            if (st == MOTOR_ENABLING || st == MOTOR_ENABLED || st == MOTOR_STOPPING)
                return 1u;
        }
    }
    return 0u;
}

static void fill_imu(imu_t *out)
{
    const ImuSample *s = ImuService_GetLatestSample();
    memset(out, 0, sizeof(*out));
    if (s == NULL) return;
    memcpy(out->accel, s->accel_raw, sizeof(out->accel));
    memcpy(out->gyro,  s->gyro_raw,  sizeof(out->gyro));
    out->seq    = (uint8_t)s->sample_count;
    out->flags  = (uint8_t)((s->initialized  ? IMU_FLAG_INIT_OK : 0u) |
                            (s->sample_valid ? IMU_FLAG_VALID   : 0u));
    out->errors = (uint16_t)s->poll_error_count;
}

/* Assemble + emit this cycle's tele_master_t (truncated to the configured
   slave count). master_time_us is this cycle's real TIM2 µs timestamp
   (stamped by the caller at cycle-service start). */
static void emit_tele(uint32_t master_time_us)
{
    static tele_master_t pay;   /* 613 B — static, off the stack */
    memset(&pay, 0, sizeof(pay));
    pay.cycle_id       = master_cycle_id;
    pay.time_master_us = master_time_us;
    pay.cmd_seq_active = g_mbox.cmd_seq_active;
    pay.robot_state    = (uint8_t)compute_robot_state();
    pay.n_slaves       = NUM_SLAVES;

    pay.status.host.lost            = (uint8_t)(g_host_action != HOST_LINK_OK);
    pay.status.host.last_cmd_seq_rx = g_mbox.cmd_seq_rx;
    pay.status.host.on_time         = g_mbox.cnt_on_time;
    pay.status.host.late            = g_mbox.cnt_late;
    pay.status.host.missing         = g_mbox.cnt_missing;
    pay.status.host.duplicate       = g_mbox.cnt_duplicate;

    pay.status.usb.rx_frames          = (uint16_t)usb_rx_frames;
    pay.status.usb.rx_errors          = (uint16_t)(usb_rx_scan_errors + master_link_errors);
    pay.status.usb.rx_resyncs         = (uint16_t)usb_rx_resyncs;
    pay.status.usb.rx_discarded_bytes = (uint16_t)usb_rx_discarded;
    pay.status.usb.tx_drops           = (uint16_t)usb_tx_drops;

    for (uint8_t s = 0; s < NUM_SLAVES && s < MAX_SLAVES; s++) {
        const SpiLinkSlave *sl = spi_link_slave(s);
        pay.status.spi[s].alive      = sl->alive;
        pay.status.spi[s].crc_errors = sl->crc_errors;
        pay.status.spi[s].seq_gaps   = sl->seq_gaps;
        /* Slave-authored telemetry forwarded untouched; a dead slave's slot
           stays zeroed (status.spi[s].alive = 0 is the signal). */
        if (sl->alive) pay.slaves[s] = sl->tele;
    }
    pay.status.cycle_overruns = (uint16_t)master_cycle_overruns();

    fill_imu(&pay.imu);

    static uint8_t frame[sizeof(usb_header_t) + sizeof(tele_master_t)];
    uint16_t n = proto_build(frame, sizeof(frame), USB_MSG_TELE, tx_seq++,
                             (const uint8_t *)&pay,
                             (uint16_t)TELE_MASTER_SIZE(NUM_SLAVES));
    if (n > 0u) usb_tx_write(frame, n);
}

/* Frame dispatch: usb_link's RX scanner delivers complete frames here (main-loop
   context). WHAT a frame means is decided here, not in the transport. */
static void on_usb_frame(const ProtoScanResult *r)
{
    if (r->type == USB_MSG_CMD) {
        /* Exact-length accept: the host must send CMD_MASTER_SIZE(NUM_SLAVES)
           bytes — a cheap host/firmware configuration cross-check. */
        if (r->pay_len == (uint16_t)CMD_MASTER_SIZE(NUM_SLAVES)) {
            cmd_master_t cmd;
            memset(&cmd, 0, sizeof(cmd));
            memcpy(&cmd, r->payload, r->pay_len);
#ifdef SPI_INJECT_TEST
            if (cmd.test == 0xDEu) { g_spi_inject = 1u; return; } /* sentinel */
#endif
#ifdef USB_TX_TEST
            if (cmd.test == 0xB0u) { g_txtest = 1u; return; }     /* sentinel */
#endif
            cmd_mailbox_post(&g_mbox, &cmd);
        } else {
            master_link_errors++;
        }
    } else if (r->type == USB_MSG_PING) {
        uint8_t frame[sizeof(usb_header_t)];
        uint16_t n = proto_build(frame, sizeof(frame), USB_MSG_PING, r->seq,
                                 NULL, 0u);   /* the reply echoes seq */
        if (n > 0u) usb_tx_write(frame, n);
    }
}

/* Decide what to send slave s this cycle (the active buffer is set only by
   cmd_mailbox_swap at cycle start, same main-loop context — no masking needed):
     - host-death tripped → the master OVERRIDES with STOP for every motor
       (a zeroed cmd_slave_t IS that: SLAVE_REQ_NONE + MOTOR_REQ_STOP + zero mit);
     - else an applied host command → re-send it (level-triggered);
     - else (nothing ever applied) → a NOP keepalive, which clocks telemetry but
       does NOT refresh the slave's command watchdog. cmd_seq is 0 on a NOP. */
static void poll_one_slave(uint8_t s)
{
    static const cmd_slave_t stop_all;   /* zero-initialized = STOP everything */

    if (g_host_action != HOST_LINK_OK) {
        spi_link_poll(s, SPI_OP_CMD, &stop_all, master_cycle_id,
                      g_mbox.cmd_seq_active);
    } else if (g_mbox.active_valid) {
        spi_link_poll(s, SPI_OP_CMD, &g_mbox.active[s], master_cycle_id,
                      g_mbox.cmd_seq_active);
    } else {
        spi_link_poll(s, SPI_OP_NOP, NULL, master_cycle_id, 0u);
    }
}

#ifdef USB_TX_TEST
/* Emit self-describing frames of the packet-boundary lengths {63,64,65,128,129,621}
   through the TX slot ring (the path under test): [A5 A5][len u16][idx u8][pattern…].
   64 and 128 exercise the ZLP; 621 is the max telemetry frame. The host verifies all
   six arrive intact and in order. */
static void usb_tx_emit_test_frames(void)
{
    static const uint16_t lens[6] = { 63u, 64u, 65u, 128u, 129u, 621u };
    static uint8_t f[640];
    for (uint8_t k = 0; k < 6u; k++) {
        uint16_t L = lens[k];
        f[0] = 0xA5u; f[1] = 0xA5u;
        f[2] = (uint8_t)(L & 0xFFu); f[3] = (uint8_t)(L >> 8);
        f[4] = k;
        for (uint16_t i = 5u; i < L; i++) f[i] = (uint8_t)((k * 31u + i) & 0xFFu);
        usb_tx_write(f, L);
    }
}
#endif

void master_app_init(SPI_HandleTypeDef *hspi, UART_HandleTypeDef *huart)
{
    master_huart = huart;
    spi_link_init(hspi);
    cmd_mailbox_reset(&g_mbox);
    host_watchdog_reset(&g_host_wd); g_host_action = HOST_LINK_OK;
    memset(g_slave_seen, 0, sizeof(g_slave_seen));
}

void master_app_loop(void)
{
    static uint32_t next_imu_poll_ms = 0;

    /* Drain the USB RX ring (scan + dispatch host frames) and pump the TX ring
       every iteration — both single-producer here in the main loop. */
    usb_rx_process(on_usb_frame);
    usb_tx_pump_responses();
    usb_tx_pump();

    /* One hardware-timed cycle (TIM2 ISR only flagged us; the work is here): poll
       every slave in order, then immediately emit this cycle's tele_master_t. */
    if (master_cycle_take(NULL)) {
        uint32_t t0 = cycle_us_now();           /* cycle service start (µs) */
        master_cycle_id++;
        cmd_mailbox_swap(&g_mbox, master_cycle_id);  /* apply-at-n+1: newest complete set */
        /* Host-death dead-man: if the host went quiet for too long while armed,
           override what we send the slaves (forced STOP), latched until a fresh
           all-STOP command acknowledges the stop. */
        g_host_action = host_watchdog_step(&g_host_wd, g_mbox.cycles_since_fresh,
                                           any_motor_armed(), g_mbox.fresh_all_stop,
                                           HOST_LOST_CYCLES);
        for (uint8_t s = 0; s < NUM_SLAVES; s++) {
            poll_one_slave(s);
        }
#ifdef USB_TX_TEST
        if (g_txtest) {                          /* emit only the test frames this cycle */
            g_txtest = 0u;
            usb_tx_emit_test_frames();
        } else
#endif
        {
            emit_tele(t0);                       /* stamp time_master_us = cycle start */
        }

        /* Sample the BMI088 at 100 Hz — at the START of the idle window,
           i.e. only right after a cycle's service. ImuService_Poll does two
           BLOCKING I2C1 reads (~0.4 ms at 400 kHz; was ~1.6 ms at 100 kHz); on
           its old free-running ms deadline it could start ANYWHERE in the cycle,
           including just before the next TIM2 tick, pushing every other cycle's
           service late. The late exchange compressed the gap to the next one
           below the slave's TX-arm deadline and collided with its unarmed DMA —
           bench: alternating service deltas and a tele CRC error on ~45% of
           exchanges. Anchored here the poll is long done before the next tick;
           the blocking read MUST fit the cycle (why 500 Hz requires 400 kHz).
           A missing IMU → no-op. */
        uint32_t imu_now_ms = HAL_GetTick();
        if ((int32_t)(imu_now_ms - next_imu_poll_ms) >= 0) {
            next_imu_poll_ms = imu_now_ms + IMU_SERVICE_POLL_PERIOD_MS;
            (void)ImuService_Poll();
        }
    }
}
