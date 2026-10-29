#include "usb_link.h"
#include "usbd_cdc_if.h"
#include "proto_common.h"   /* protocol.h (tele_robot_t, MSG_HEADER_SIZE) for the size guard */
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
#define TX_SLOT_MAX   USB_TX_SLOT_MAX  /* schema constant; largest-frame-fits assert
                                          is generated in protocol_gen.h */
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

/* ── USB RX ring: CDC ISR produces, main loop consumes (SPSC, lock-free) ───── */
#define USB_RX_RING_SIZE 1024u                       /* power of two               */
#define USB_RX_RING_MASK (USB_RX_RING_SIZE - 1u)
static volatile uint8_t  usb_rx_ring[USB_RX_RING_SIZE];
static volatile uint16_t usb_rx_head = 0;            /* written by ISR only        */
static volatile uint16_t usb_rx_tail = 0;            /* written by main only       */

uint32_t usb_rx_frames      = 0;
uint32_t usb_rx_scan_errors = 0;
uint32_t usb_rx_resyncs     = 0;
uint32_t usb_rx_discarded   = 0;
uint32_t usb_rx_overflows   = 0;

void usb_rx_from_isr(const uint8_t *buf, uint16_t len)
{
    uint16_t head = usb_rx_head;
    for (uint16_t i = 0; i < len; i++) {
        uint16_t next = (uint16_t)((head + 1u) & USB_RX_RING_MASK);
        if (next == usb_rx_tail) { usb_rx_overflows++; break; }  /* full → drop rest */
        usb_rx_ring[head] = buf[i];
        head = next;
    }
    usb_rx_head = head;
}

/* Drain the RX ring and hand every complete frame to on_frame (main-loop context).
   Uses the resynchronizing proto_frame_scan: junk/misaligned bytes are dropped one
   at a time (counted), so a stray byte or lost packet can't desync us permanently. */
void usb_rx_process(usb_rx_frame_cb on_frame)
{
    static uint8_t  scan[768];     /* ≥ 2 × max frame (16 + 480) for headroom */
    static uint16_t scan_len = 0;
    static uint8_t  in_resync = 0;

    /* Drain ring → scan buffer (consumer side; tail is ours). */
    while (scan_len < sizeof(scan) && usb_rx_tail != usb_rx_head) {
        scan[scan_len++] = usb_rx_ring[usb_rx_tail];
        usb_rx_tail = (uint16_t)((usb_rx_tail + 1u) & USB_RX_RING_MASK);
    }

    uint16_t off = 0;
    while ((uint16_t)(scan_len - off) >= MSG_HEADER_SIZE) {
        ProtoScanResult r;
        ProtoScanStatus st = proto_frame_scan(scan + off, (uint16_t)(scan_len - off), &r);

        if (st == PROTO_SCAN_NEED_MORE) break;         /* plausible header; await body */

        if (st == PROTO_SCAN_RESYNC) {                 /* junk → drop byte(s), count run */
            if (!in_resync) { usb_rx_resyncs++; in_resync = 1u; }
            usb_rx_discarded += r.consumed;
            usb_rx_scan_errors++;
            off += r.consumed;
            continue;
        }

        in_resync = 0u;                                /* PROTO_SCAN_FRAME */
        usb_rx_frames++;
        if (on_frame != NULL) on_frame(&r);
        off += r.consumed;
    }

    if (off > 0u) {                                    /* slide unconsumed tail down */
        scan_len = (uint16_t)(scan_len - off);
        if (scan_len > 0u) memmove(scan, scan + off, scan_len);
    } else if (scan_len == sizeof(scan)) {             /* defensive: never wedge full */
        if (!in_resync) { usb_rx_resyncs++; in_resync = 1u; }
        usb_rx_discarded++;
        scan_len--;
        memmove(scan, scan + 1, scan_len);
    }
}
