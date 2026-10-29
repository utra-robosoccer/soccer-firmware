/* cmd_mailbox — pure master-side host-command mailbox (HAL-free, host-testable).
 *
 * One-deep, latest-wins: USB dispatch posts the host's cmd_robot_t into the
 * PENDING back-buffer at any time between cycles; cmd_mailbox_swap() at the
 * start of every master cycle moves the newest complete set into the ACTIVE
 * buffer that the SPI poll reads. So a command received during cycle n is
 * applied at n+1 (never the same cycle, never a half-written set). With no new
 * command the active set is held and re-sent (level-triggered); a second post
 * before the swap replaces the unread one (cnt_duplicate — a mailbox, not a
 * queue: no backlog, the newest intent wins). The swap also classifies
 * host-loop timing (on-time/late/missing) and maintains cycles_since_fresh +
 * fresh_fault_reset, the host-death watchdog's inputs (host_watchdog.h).
 * See docs/architecture.md.
 */
#ifndef CMD_MAILBOX_H
#define CMD_MAILBOX_H

#include <stdint.h>
#include <string.h>
#include "protocol.h"
#include "system_config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /* ACTIVE: applied + re-sent each cycle (read by the SPI poll). */
    cmd_chain_t active[NUM_SLAVES];
    uint8_t     active_valid[NUM_SLAVES];
    uint16_t    cmd_seq_active;       /* cmd_seq sent to slaves THIS cycle */

    /* PENDING: filled by USB dispatch, consumed by the swap. */
    cmd_chain_t pend[NUM_SLAVES];
    uint8_t     pend_valid[NUM_SLAVES];
    uint16_t    pend_cmd_seq;
    uint16_t    pend_cycle_echo;      /* cmd.cycle_id of the pending command */
    uint8_t     pend_fresh;           /* a new complete set awaits the swap  */
    uint8_t     pend_fault_reset;     /* pending command carries a RESET request */

    uint16_t    cmd_seq_rx;           /* last cmd_seq received from host */

    /* Host-loop-vs-master-cycle counters, classified at the swap (reported in
       tele_robot_t, wrap; the host reports per-run deltas). */
    uint16_t    cnt_on_time, cnt_late, cnt_missing, cnt_duplicate;

    /* Host-death watchdog inputs (host_watchdog_step). */
    uint32_t    cycles_since_fresh;
    uint8_t     fresh_fault_reset;    /* a fresh fault-reset command arrived this cycle */
} CmdMailbox;

static inline void cmd_mailbox_reset(CmdMailbox *m)
{
    memset(m, 0, sizeof(*m));
    m->cmd_seq_active = CMD_SEQ_NONE;
    m->pend_cmd_seq   = CMD_SEQ_NONE;
    m->cmd_seq_rx     = CMD_SEQ_NONE;
}

/* Post one host command into the PENDING back-buffer (per-slave, latest wins by
   chain_id). pend_fresh is set LAST, after the whole set is copied, so a swap
   can never take a half-written set. Mode requests are level-triggered: a slave
   absent from this command keeps its stored pending chain. Returns the number
   of chains rejected for a bad chain_id (the caller counts them as link
   errors). */
static inline uint8_t cmd_mailbox_post(CmdMailbox *m, const cmd_robot_t *cmd)
{
    uint8_t rejected = 0u;
    if (m->pend_fresh) m->cnt_duplicate++;   /* overwriting an unapplied set */
    m->cmd_seq_rx      = cmd->cmd_seq;
    m->pend_cmd_seq    = cmd->cmd_seq;
    m->pend_cycle_echo = cmd->cycle_id;      /* the master cycle the host was answering */
    uint8_t n = (cmd->n_chains > MAX_CHAINS) ? MAX_CHAINS : cmd->n_chains;
    uint8_t fr = 0u;                         /* does this command carry a RESET request? */
    for (uint8_t c = 0; c < n; c++) {
        uint8_t s = cmd->chains[c].chain_id;
        if (s >= NUM_SLAVES) { rejected++; continue; }
        m->pend[s]       = cmd->chains[c];   /* struct copy */
        m->pend_valid[s] = 1u;
        uint8_t nm = (cmd->chains[c].n_motors > MAX_MOTORS_PER_SLAVE)
                     ? MAX_MOTORS_PER_SLAVE : cmd->chains[c].n_motors;
        for (uint8_t i = 0; i < nm; i++)
            if (cmd->chains[c].motors[i].drive_req == DRV_REQ_RESET) fr = 1u;
    }
    m->pend_fault_reset = fr;
    m->pend_fresh = 1u;                      /* publish last: the swap sees a complete set */
    return rejected;
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
            if (m->pend_valid[s]) {
                m->active[s]       = m->pend[s];
                m->active_valid[s] = 1u;
            }
        }
        m->cmd_seq_active     = m->pend_cmd_seq;
        m->cycles_since_fresh = 0u;
        m->fresh_fault_reset  = m->pend_fault_reset;
        m->pend_fresh = 0u;
    } else {
        m->cnt_missing++;               /* no new command → active set re-sent (hold) */
        if (m->cycles_since_fresh < 0xFFFFFFFFu) m->cycles_since_fresh++;
        m->fresh_fault_reset = 0u;
    }
}

#ifdef __cplusplus
}
#endif
#endif /* CMD_MAILBOX_H */
