/* cmd_mailbox — pure master-side host-command mailbox (HAL-free, host-testable).
 *
 * One-deep, latest-wins: USB dispatch posts the host's cmd_master_t into the
 * PENDING back-buffer at any time between cycles; cmd_mailbox_swap() at the
 * start of every master cycle moves the newest complete set into the ACTIVE
 * buffer that the SPI poll reads. So a command received during cycle n is
 * applied at n+1 (never the same cycle, never a half-written set). With no new
 * command the active set is held and re-sent (level-triggered); a second post
 * before the swap replaces the unread one (cnt_duplicate — a mailbox, not a
 * queue: no backlog, the newest intent wins). The swap also classifies
 * host-loop timing (on-time/late/missing) and maintains cycles_since_fresh +
 * fresh_all_stop, the host-death watchdog's inputs (host_watchdog.h).
 */
#ifndef CMD_MAILBOX_H
#define CMD_MAILBOX_H

#include <stdint.h>
#include <string.h>
#include "protocol.h"
#include "master_config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /* ACTIVE: applied + re-sent each cycle (read by the SPI poll). */
    cmd_slave_t active[NUM_SLAVES];
    uint8_t     active_valid;         /* a host command has ever been applied */
    uint16_t    cmd_seq_active;       /* cmd_seq sent to slaves THIS cycle */

    /* PENDING: filled by USB dispatch, consumed by the swap. */
    cmd_slave_t pend[NUM_SLAVES];
    uint16_t    pend_cmd_seq;
    uint16_t    pend_cycle_echo;      /* cmd.cycle_id of the pending command */
    uint8_t     pend_fresh;           /* a new complete set awaits the swap  */
    uint8_t     pend_all_stop;        /* every motor request in it is STOP   */

    uint16_t    cmd_seq_rx;           /* last cmd_seq received from host */

    /* Host-loop-vs-master-cycle counters, classified at the swap (reported in
       status_master_t.host, wrap; the host reports per-run deltas). */
    uint16_t    cnt_on_time, cnt_late, cnt_missing, cnt_duplicate;

    /* Host-death watchdog inputs (host_watchdog_step). */
    uint32_t    cycles_since_fresh;
    uint8_t     fresh_all_stop;       /* a fresh all-STOP command arrived this cycle */
} CmdMailbox;

static inline void cmd_mailbox_reset(CmdMailbox *m)
{
    memset(m, 0, sizeof(*m));
    m->cmd_seq_active = CMD_SEQ_NONE;
    m->pend_cmd_seq   = CMD_SEQ_NONE;
    m->cmd_seq_rx     = CMD_SEQ_NONE;
}

/* Post one host command into the PENDING back-buffer. The caller has already
   enforced the exact CMD_MASTER_SIZE(NUM_SLAVES) payload length, so every
   configured slave's slot is present (slot index = slave index — there is no
   id on the wire). pend_fresh is set LAST, after the whole set is copied, so a
   swap can never take a half-written set. */
static inline void cmd_mailbox_post(CmdMailbox *m, const cmd_master_t *cmd)
{
    if (m->pend_fresh) m->cnt_duplicate++;   /* overwriting an unapplied set */
    m->cmd_seq_rx      = cmd->cmd_seq;
    m->pend_cmd_seq    = cmd->cmd_seq;
    m->pend_cycle_echo = cmd->cycle_id;      /* the master cycle the host was answering */
    uint8_t all_stop = 1u;
    for (uint8_t s = 0; s < NUM_SLAVES; s++) {
        m->pend[s] = cmd->slaves[s];         /* struct copy */
        for (uint8_t i = 0; i < MAX_MOTORS_PER_SLAVE; i++) {
            if (cmd->slaves[s].motors[i].req != (uint8_t)MOTOR_REQ_STOP)
                all_stop = 0u;
        }
    }
    m->pend_all_stop = all_stop;
    m->pend_fresh = 1u;                      /* publish last: the swap sees a complete set */
}

/* Once at the start of every master cycle, before polling. If a fresh set is
   pending, classify it against the cycle it answered and swap it into the
   active buffer (apply at n+1); otherwise hold the active set and count a
   missing cycle. master_cycle_id has already been incremented for THIS cycle. */
static inline void cmd_mailbox_swap(CmdMailbox *m, uint16_t master_cycle_id)
{
    if (m->pend_fresh) {
        /* latest telemetry the host could have answered = the cycle emitted last =
           master_cycle_id - 1 (it was incremented at this cycle's start). Wrap-safe. */
        int16_t d = (int16_t)((uint16_t)(master_cycle_id - 1u) - m->pend_cycle_echo);
        if (d <= 0) m->cnt_on_time++;   /* answered the freshest cycle (d<0 clamps to on-time) */
        else        m->cnt_late++;      /* answered an older cycle (host lagged d cycles) */
        for (uint8_t s = 0; s < NUM_SLAVES; s++) {
            m->active[s] = m->pend[s];
        }
        m->active_valid       = 1u;
        m->cmd_seq_active     = m->pend_cmd_seq;
        m->cycles_since_fresh = 0u;
        m->fresh_all_stop     = m->pend_all_stop;
        m->pend_fresh = 0u;
    } else {
        m->cnt_missing++;               /* no new command → active set re-sent (hold) */
        if (m->cycles_since_fresh < 0xFFFFFFFFu) m->cycles_since_fresh++;
        m->fresh_all_stop = 0u;
    }
}

#ifdef __cplusplus
}
#endif
#endif /* CMD_MAILBOX_H */
