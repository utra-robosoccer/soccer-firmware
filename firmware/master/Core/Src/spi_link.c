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

/* ── SPI exchange with one slave (fixed-size frames) ──
   One full-duplex transfer of SPI_XFER_SIZE bytes: the spi_cmd_frame_t rides
   in the TX prefix (the rest is zero fill); the CRC-framed spi_tele_frame_t
   comes back in RX. The frame CRC doubles as integrity AND presence check —
   an absent slave clocks back garbage that fails CRC. */
typedef enum {
    SPI_EX_DEAD = 0,   /* HAL failure or CRC-bad/absent frame            */
    SPI_EX_OK   = 1,   /* CRC-valid frame — *tele_out set                */
} SpiExResult;

static SpiExResult spi_exchange(SpiDevId dev, uint8_t opcode, uint8_t spi_seq_v,
                                uint16_t cycle_id, uint16_t cmd_seq,
                                const cmd_slave_t *cmd,
                                tele_slave_t *tele_out)
{
    uint8_t tx[SPI_XFER_SIZE];
    uint8_t rx[SPI_XFER_SIZE];
    memset(tx, 0, SPI_XFER_SIZE);

    spi_header_t hdr;
    hdr.opcode   = opcode;
    hdr.spi_seq  = spi_seq_v;
    hdr.cycle_id = cycle_id;
    hdr.cmd_seq  = cmd_seq;
    memcpy(&tx[offsetof(spi_cmd_frame_t, hdr)], &hdr, sizeof(hdr));
    if (cmd != NULL) {
        memcpy(&tx[offsetof(spi_cmd_frame_t, cmd)], cmd, sizeof(cmd_slave_t));
    }
    uint16_t ccrc = proto_crc16(tx, offsetof(spi_cmd_frame_t, crc16));
    tx[offsetof(spi_cmd_frame_t, crc16)]      = (uint8_t)(ccrc & 0xFFu);
    tx[offsetof(spi_cmd_frame_t, crc16) + 1u] = (uint8_t)(ccrc >> 8);

    CS_SELECT(dev);
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(master_hspi, tx, rx,
                                                    SPI_XFER_SIZE, HAL_MAX_DELAY);
    CS_ALL_HIGH();
    if (st != HAL_OK) return SPI_EX_DEAD;

    const uint16_t tcrc_off = (uint16_t)offsetof(spi_tele_frame_t, crc16);
    uint16_t rx_crc = (uint16_t)(rx[tcrc_off] | ((uint16_t)rx[tcrc_off + 1u] << 8));
    uint16_t calc   = proto_crc16(rx, tcrc_off);
    if (rx_crc != calc) {
        g_slave[(uint8_t)dev].crc_errors++;
        return SPI_EX_DEAD;            /* bad/absent frame → caller marks offline */
    }

    memcpy(tele_out, rx, sizeof(tele_slave_t));
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

/* Forward one slave's command, then ingest its telemetry. */
void spi_link_poll(uint8_t s, uint8_t opcode, const cmd_slave_t *cmd,
                   uint16_t cycle_id, uint16_t cmd_seq)
{
    SpiLinkSlave *sl = &g_slave[s];

#ifdef SPI_INJECT_TEST
    if (g_spi_inject && s == 0u) {
        g_spi_inject = 0u;
        /* Clock ONE deliberately short exchange (PAYLOAD-1 bytes) so the slave's
           fixed-length DMA is left mid-frame → desync. slave_spi_resync must
           recover on the next poll (resyncs +1, CRC back to clean). */
        static uint8_t itx[SPI_XFER_SIZE], irx[SPI_XFER_SIZE];
        memset(itx, 0, sizeof(itx));
        CS_SELECT((SpiDevId)s);
        HAL_SPI_TransmitReceive(master_hspi, itx, irx, SPI_XFER_SIZE - 1u, HAL_MAX_DELAY);
        CS_ALL_HIGH();
        return;   /* skip the normal exchange this cycle */
    }
#endif

    uint8_t seq_to_send = sl->seq++;
    tele_slave_t tele;
    SpiExResult st = spi_exchange((SpiDevId)s, opcode, seq_to_send,
                                  cycle_id, cmd_seq, cmd, &tele);

    if (st == SPI_EX_OK) {   /* CRC verified inside spi_exchange = valid + present */
        /* Echo-stall diagnostics on the SPI link-health seq (sustained freeze =
           a command link that stopped confirming). Any advance resets the streak. */
        uint8_t echo = tele.status.spi.seq_echo;
        if (sl->have_prev_echo) {
            if (echo == sl->prev_echo) {
                if (++sl->echo_stall >= SEQ_STALL_POLLS) { sl->seq_gaps++; sl->echo_stall = 0u; }
            } else {
                sl->echo_stall = 0u;
            }
        }
        sl->prev_echo      = echo;
        sl->have_prev_echo = 1u;

        sl->alive = 1u;
        sl->tele  = tele;    /* struct copy */
    } else {
        /* Absent slave / garbage frame — mark offline; reset the seq-echo baseline
           so recovery doesn't false-count. The host command mailbox is left intact
           (level-triggered: it reapplies once the slave is back). */
        sl->alive          = 0u;
        sl->have_prev_echo = 0u;
        sl->echo_stall     = 0u;
    }
}
