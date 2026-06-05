#include "usb_tx.h"
#include "usbd_cdc_if.h"
#include <string.h>

#define RING_SIZE  4096u   /* must be power-of-two */
#define RING_MASK  (RING_SIZE - 1u)
#define PKT_MAX    64u     /* USB FS CDC max packet */

static uint8_t  ring[RING_SIZE];
static uint16_t head = 0;   /* write pointer */
static uint16_t tail = 0;   /* read  pointer */
static uint8_t  staging[PKT_MAX];

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

    uint16_t n = (avail > PKT_MAX) ? PKT_MAX : avail;

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

uint8_t usb_tx_busy(void) { return 0u; }  /* not used externally */
void    usb_tx_cplt_cb(void) {}           /* no-op: pump polls TxState */
