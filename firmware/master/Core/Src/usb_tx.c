#include "usb_tx.h"
#include "usbd_cdc_if.h"
#include <string.h>

#define RING_SIZE  8192u   /* must be power-of-two */
#define RING_MASK  (RING_SIZE - 1u)
/* Per-transfer drain chunk = one USB FS packet. Larger multi-packet IN transfers
   raise TX throughput but starve the shared OTG core's multi-packet OUT reception
   (host→master ROBOT_CMD frames then never complete). We don't need the extra TX
   headroom: the master transmits only the POPULATED telemetry prefix (emit_robot_tele),
   so one chain is ~129 B; at 200 Hz that's ~26 KB/s, well under this ~64 KB/s ceiling. */
#define TX_CHUNK   64u

static uint8_t  ring[RING_SIZE];
static uint16_t head = 0;   /* write pointer */
static uint16_t tail = 0;   /* read  pointer */
static uint8_t  staging[TX_CHUNK];

static uint16_t ring_used(void) { return (head - tail) & RING_MASK; }
static uint16_t ring_free(void) { return (RING_SIZE - 1u) - ring_used(); }

void usb_tx_write(const uint8_t *data, uint16_t len)
{
    if (len == 0u || data == NULL) return;
    if (len > ring_free()) return;   /* drop if no space */
    for (uint16_t i = 0u; i < len; i++) {
        ring[head & RING_MASK] = data[i];
        head++;
    }
}

/*
 * Drain one USB packet from the ring.
 *
 * Strategy: peek n bytes into staging WITHOUT advancing tail, then call
 * CDC_Transmit_FS.  Only advance tail on USBD_OK so data is never lost on
 * a busy endpoint.  staging remains valid for the DMA until we overwrite it
 * on the next successful call — which can only happen after TxState clears
 * (CDC_Transmit_FS returns USBD_OK again), so there is no aliasing hazard.
 */
void usb_tx_pump(void)
{
    uint16_t avail = ring_used();
    if (avail == 0u) return;

    uint16_t n = (avail > TX_CHUNK) ? TX_CHUNK : avail;

    /* Peek: copy without consuming */
    uint16_t peek = tail;
    for (uint16_t i = 0u; i < n; i++) {
        staging[i] = ring[peek & RING_MASK];
        peek++;
    }

    if (CDC_Transmit_FS(staging, n) == USBD_OK) {
        tail = peek;   /* consume only on success */
    }
    /* On USBD_BUSY: tail unchanged, data stays in ring for next call */
}

/* ── ISR-context response queue (fix 4: restore SPSC on the ring) ────────────
 * ControlResp / PONG frames are built in USB-ISR context. To keep usb_tx_write a
 * single producer (main only), the ISR enqueues finished frames here and main
 * drains them through usb_tx_write in ProcessLoop. Single producer = the USB
 * ISR, single consumer = main → lock-free, no locks on the ring. */
#define RESP_SLOTS  8u
#define RESP_MAX    40u    /* largest response frame (ControlResp = 16+7 = 23 B) */
static uint8_t          resp_buf[RESP_SLOTS][RESP_MAX];
static uint16_t         resp_len[RESP_SLOTS];
static volatile uint8_t resp_head = 0;   /* USB ISR advances */
static volatile uint8_t resp_tail = 0;   /* main advances    */

/* Called from USB-ISR context (CDC_Receive_FS / HandleControlReq). */
void usb_tx_post_from_isr(const uint8_t *data, uint16_t len)
{
    if (len == 0u || len > RESP_MAX || data == NULL) return;
    uint8_t next = (uint8_t)((resp_head + 1u) % RESP_SLOTS);
    if (next == resp_tail) return;        /* full → drop (responses best-effort) */
    memcpy(resp_buf[resp_head], data, len);
    resp_len[resp_head] = len;
    resp_head = next;                     /* publish after the copy */
}

/* Called from main (ProcessLoop): drain queued responses into the TX ring. */
void usb_tx_pump_responses(void)
{
    while (resp_tail != resp_head) {
        usb_tx_write(resp_buf[resp_tail], resp_len[resp_tail]);
        resp_tail = (uint8_t)((resp_tail + 1u) % RESP_SLOTS);
    }
}

uint8_t usb_tx_busy(void) { return 0u; }  /* not used externally */
void    usb_tx_cplt_cb(void) {}           /* no-op: pump polls TxState */
