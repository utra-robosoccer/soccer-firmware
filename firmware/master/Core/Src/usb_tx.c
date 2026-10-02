#include "usb_tx.h"
#include "usbd_cdc_if.h"
#include <string.h>

/* Frame-aware slot ring. Each slot holds one whole framed message; the pump hands
   one whole frame per CDC_Transmit_FS and the USB stack splits it into 64 B packets.
   One frame per transfer → frames never interleave; a frame queued while a transfer
   is in progress waits here and goes out as soon as it completes. Producer and
   consumer are both main-context (ProcessLoop), so no locking is needed.

   This supersedes the old 64 B-per-call drain, whose only reason was to avoid
   starving multi-packet OUT on the shared OTG-FS core; the USB-RX and SPI resync
   paths now recover any OUT hiccup, and validation watches the resync/error
   counters to confirm whole-frame IN does not starve OUT. */
#define NUM_SLOTS     16u
#define TX_SLOT_MAX   512u   /* ≥ largest frame = MSG_HEADER_SIZE + sizeof(tele_robot_t) = 496 */
#define USB_FS_MPS    64u    /* bulk IN max packet size */

static uint8_t  tx_slot[NUM_SLOTS][TX_SLOT_MAX];
static uint16_t tx_len[NUM_SLOTS];
static uint8_t  head = 0;        /* producer: next free slot    */
static uint8_t  tail = 0;        /* consumer: next slot to send */
static uint8_t  zlp_pending = 0; /* last frame was a 64 B multiple → owe a ZLP */
static uint8_t  zlp_dummy[1];    /* a valid (unread) buffer for the 0-length transfer */
uint32_t        usb_tx_drops = 0;

static inline uint8_t slot_next(uint8_t i) { return (uint8_t)((i + 1u) % NUM_SLOTS); }

void usb_tx_write(const uint8_t *data, uint16_t len)
{
    if (len == 0u || data == NULL || len > TX_SLOT_MAX) return;
    uint8_t n = slot_next(head);
    if (n == tail) { usb_tx_drops++; return; }   /* full → drop the WHOLE frame */
    memcpy(tx_slot[head], data, len);
    tx_len[head] = len;
    head = n;
    usb_tx_pump();   /* kick on enqueue: send now if the endpoint is idle */
}

/*
 * Hand one whole frame to the USB stack per call. CDC_Transmit_FS returns USBD_BUSY
 * while a transfer is in progress (TxState≠0), so queued frames simply wait. A frame
 * whose length is an exact multiple of the 64 B packet size needs a trailing
 * zero-length packet to terminate the bulk transfer; we send it before the next frame.
 * The slot stays valid until its transfer completes (we only advance tail on OK and
 * only reuse a slot after wrapping the whole ring), so there is no aliasing hazard.
 */
void usb_tx_pump(void)
{
    if (zlp_pending) {
        if (CDC_Transmit_FS(zlp_dummy, 0u) == USBD_OK) zlp_pending = 0u;
        return;                                  /* BUSY → data transfer still running */
    }
    if (head == tail) return;                    /* ring empty */

    uint16_t len = tx_len[tail];
    if (CDC_Transmit_FS(tx_slot[tail], len) == USBD_OK) {
        zlp_pending = (uint8_t)((len % USB_FS_MPS) == 0u);
        tail = slot_next(tail);
    }
    /* USBD_BUSY → leave tail; retry next pump */
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
