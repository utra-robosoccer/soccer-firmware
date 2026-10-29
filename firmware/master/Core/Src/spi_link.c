/* spi_link — see spi_link.h. Owns hspi1, the CS lines and all per-slave link state;
 * knows nothing about mailboxes, watchdogs or USB. */
#include "spi_link.h"
#include <string.h>

static SPI_HandleTypeDef *master_hspi = NULL;

static SpiLinkSlave g_slave[NUM_SLAVES];

#define SEQ_STALL_POLLS 5u    /* echo frozen this many polls = a gap */

#ifdef SPI_INJECT_TEST
volatile uint8_t g_spi_inject = 0; /* test-only: clock one short SPI exchange to desync the slave */
#endif

static inline void CS_ALL_HIGH(void) {
    HAL_GPIO_WritePin(GPIOC,
        SLAVE_CS_0_Pin | SLAVE_CS_1_Pin | SLAVE_CS_2_Pin | SLAVE_CS_3_Pin,
        GPIO_PIN_SET);
}
static inline void CS_SELECT(SpiDevId dev) {
    CS_ALL_HIGH();
    switch (dev) {
        case DEV1: HAL_GPIO_WritePin(GPIOC, SLAVE_CS_0_Pin, GPIO_PIN_RESET); break;
        case DEV2: HAL_GPIO_WritePin(GPIOC, SLAVE_CS_1_Pin, GPIO_PIN_RESET); break;
        case DEV3: HAL_GPIO_WritePin(GPIOC, SLAVE_CS_2_Pin, GPIO_PIN_RESET); break;
        case DEV4: HAL_GPIO_WritePin(GPIOC, SLAVE_CS_3_Pin, GPIO_PIN_RESET); break;
        default: break;
    }
}

/* ── SPI exchange with one slave (fixed-size frames; one chain per slave) ──
   One full-duplex transfer: the command frame [opcode][spi_seq][cycle_id][cmd_seq]
   [cmd_chain_t][crc] rides in the TX prefix; the CRC-framed tele_chain_t comes
   back in RX. The frame CRC doubles as integrity AND presence check — an absent
   slave clocks back garbage that fails CRC. */
typedef enum {
    SPI_EX_DEAD     = 0,   /* HAL failure or CRC-bad/absent frame               */
    SPI_EX_OK       = 1,   /* valid frame from a matching build — *tele_out set */
    SPI_EX_MISMATCH = 2,   /* CRC-intact frame, wrong proto/schema (version gate) */
} SpiExResult;

static SpiExResult spi_exchange(SpiDevId dev, uint8_t opcode, uint8_t spi_seq_v,
                                uint16_t cycle_id, uint16_t cmd_seq,
                                const cmd_chain_t *chain,
                                tele_chain_t *tele_out)
{
    uint8_t tx[SPI_XFER_SIZE];
    uint8_t rx[SPI_XFER_SIZE];
    memset(tx, 0, SPI_XFER_SIZE);
    tx[0] = opcode;
    tx[1] = spi_seq_v;
    tx[2] = (uint8_t)(cycle_id & 0xFFu);
    tx[3] = (uint8_t)(cycle_id >> 8);
    tx[4] = (uint8_t)(cmd_seq & 0xFFu);
    tx[5] = (uint8_t)(cmd_seq >> 8);
    if (chain != NULL) memcpy(&tx[SPI_CMD_HDR_BYTES], chain, sizeof(cmd_chain_t));
    spi_trailer_write(&tx[SPI_CMD_TRAILER_OFF]);   /* version gate (v9.1) */
    uint16_t ccrc = proto_crc16(tx, SPI_CMD_CRC_OFF);
    tx[SPI_CMD_CRC_OFF]     = (uint8_t)(ccrc & 0xFFu);
    tx[SPI_CMD_CRC_OFF + 1] = (uint8_t)(ccrc >> 8);

    CS_SELECT(dev);
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(master_hspi, tx, rx,
                                                    SPI_XFER_SIZE, HAL_MAX_DELAY);
    CS_ALL_HIGH();
    if (st != HAL_OK) return SPI_EX_DEAD;

    uint16_t rx_crc = (uint16_t)(rx[SPI_TELE_CRC_OFF] |
                      ((uint16_t)rx[SPI_TELE_CRC_OFF + 1] << 8));
    uint16_t calc   = proto_crc16(rx, SPI_TELE_CRC_OFF);
    if (rx_crc != calc) {
        g_slave[(uint8_t)dev].crc_errors++;
        return SPI_EX_DEAD;            /* bad/absent frame → caller marks offline */
    }
    /* CRC-intact but a different build on the other side: refuse the payload. */
    if (!spi_trailer_match(&rx[SPI_TELE_TRAILER_OFF])) return SPI_EX_MISMATCH;

    memcpy(tele_out, rx, sizeof(tele_chain_t));
    g_slave[(uint8_t)dev].cmd_crc_errors = tele_out->cmd_crc_errors;   /* relayed */
    return SPI_EX_OK;
}

void spi_link_init(SPI_HandleTypeDef *hspi)
{
    master_hspi = hspi;
    CS_ALL_HIGH();
    memset(g_slave, 0, sizeof(g_slave));
}

const SpiLinkSlave *spi_link_slave(uint8_t s)
{
    return &g_slave[s];
}

uint8_t spi_link_any_mismatch(void)
{
    for (uint8_t s = 0; s < NUM_SLAVES; s++) {
        if (g_slave[s].mismatch) return 1u;
    }
    return 0u;
}

/* Forward one slave's command chain, then ingest its telemetry. */
void spi_link_poll(uint8_t s, uint8_t opcode, const cmd_chain_t *chain,
                   uint16_t cycle_id, uint16_t cmd_seq)
{
    SpiLinkSlave *sl = &g_slave[s];

#ifdef SPI_INJECT_TEST
    if (g_spi_inject && s == 0u) {
        g_spi_inject = 0u;
        /* Clock ONE deliberately short exchange (PAYLOAD-1 bytes) so the slave's
           fixed-length DMA is left mid-frame → desync. slave_spi_resync must
           recover on the next poll (spi_resyncs +1, CRC back to clean). */
        static uint8_t itx[SPI_XFER_SIZE], irx[SPI_XFER_SIZE];
        memset(itx, 0, sizeof(itx));
        CS_SELECT((SpiDevId)s);
        HAL_SPI_TransmitReceive(master_hspi, itx, irx, SPI_XFER_SIZE - 1u, HAL_MAX_DELAY);
        CS_ALL_HIGH();
        return;   /* skip the normal exchange this cycle */
    }
#endif

    /* Local working copy: the version gate below may rewrite it. */
    cmd_chain_t chain_local;
    if (chain != NULL) chain_local = *chain;
    else {
        memset(&chain_local, 0, sizeof(chain_local));
        chain_local.chain_id = s;
        chain_local.n_motors = slave_motor_counts[s];
    }

    /* Version gate (v9.1): never forward commands to a mismatched slave (poll
       with NOPs so a reflash is detected), and while ANY slave mismatches
       refuse NEW enables robot-wide — an ENABLE for a not-yet-armed motor is
       rewritten to DISABLE (no-op); armed motors are left alone. */
    if (sl->mismatch) {
        opcode = SPI_OP_NOP;
        memset(&chain_local, 0, sizeof(chain_local));
        chain_local.chain_id = s;
        chain_local.n_motors = slave_motor_counts[s];
    } else if (opcode == SPI_OP_ROBOT_CMD && spi_link_any_mismatch()) {
        for (uint8_t i = 0; i < MAX_MOTORS_PER_CHAIN; i++) {
            uint8_t ds = sl->alive ? sl->tele.motors[i].drive_state
                                   : (uint8_t)DRV_BOOT;
            chain_local.motors[i].drive_req =
                spi_gate_filter_req(chain_local.motors[i].drive_req, ds);
        }
    }

    uint8_t seq_to_send = sl->seq++;
    tele_chain_t tele;
    SpiExResult st = spi_exchange((SpiDevId)s, opcode, seq_to_send,
                                  cycle_id, cmd_seq, &chain_local, &tele);

    if (st == SPI_EX_OK) {   /* CRC verified inside spi_exchange = valid + present */
        /* Echo-stall diagnostics on the SPI link-health seq (sustained freeze =
           a command link that stopped confirming). Any advance resets the streak. */
        uint8_t echo = tele.spi_seq_echo;
        if (sl->have_prev_echo) {
            if (echo == sl->prev_echo) {
                if (++sl->echo_stall >= SEQ_STALL_POLLS) { sl->seq_gaps++; sl->echo_stall = 0u; }
            } else {
                sl->echo_stall = 0u;
            }
        }
        sl->prev_echo      = echo;
        sl->have_prev_echo = 1u;

        /* A motor is "alive" once discovery advanced it past BOOT/DISCOVERING. */
        uint8_t alive_mask = 0u;
        uint8_t nm = (tele.n_motors > MAX_MOTORS_PER_SLAVE) ? MAX_MOTORS_PER_SLAVE
                                                            : tele.n_motors;
        for (uint8_t i = 0; i < nm; i++) {
            uint8_t st8 = tele.motors[i].drive_state;
            if (st8 > DRV_ABSENT)
                alive_mask |= (uint8_t)(1u << i);
        }
        sl->alive        = 1u;
        sl->mismatch     = 0u;      /* match (again) → gate reopens, no reset */
        sl->motors_alive = alive_mask;
        sl->tele         = tele;    /* struct copy */
    } else if (st == SPI_EX_MISMATCH) {
        /* Responding slave built against a different wire contract: excluded
           from alive/forwarding/enables, reported via tele_chain_t.link and
           MasterStatus.slave_proto_mismatch. Polling continues (NOPs), so a
           reflashed slave is re-accepted without a master reset. */
        sl->alive          = 0u;
        sl->mismatch       = 1u;
        sl->motors_alive   = 0u;
        sl->have_prev_echo = 0u;
        sl->echo_stall     = 0u;
    } else {
        /* Absent slave / garbage frame — mark offline; reset the seq-echo baseline
           so recovery doesn't false-count. The host command mailbox is left intact
           (level-triggered: it reapplies once the slave is back). */
        sl->alive          = 0u;
        sl->mismatch       = 0u;
        sl->motors_alive   = 0u;
        sl->have_prev_echo = 0u;
        sl->echo_stall     = 0u;
    }
}
