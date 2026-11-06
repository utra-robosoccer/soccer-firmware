#ifndef USB_LINK_H
#define USB_LINK_H

#include <stdint.h>
#include "proto_common.h"   /* ProtoScanResult for the RX frame callback */

/* Frame-aware slot ring behind all USB TX. Each usb_tx_write() enqueues one whole
   framed message; usb_tx_pump() hands one whole frame per CDC_Transmit_FS (the USB
   stack splits it into packets) and never interleaves frames. usb_tx_write() kicks
   the pump on enqueue, so a frame goes out immediately when the endpoint is idle. */

void    usb_tx_write(const uint8_t *data, uint16_t len);        /* MAIN context only */
void    usb_tx_pump(void);   /* call from main loop; no-op if endpoint busy  */
uint8_t usb_tx_busy(void);   /* non-zero if a DMA transfer is in flight      */

extern uint32_t usb_tx_drops; /* whole frames dropped because the slot ring was full */

/* SPSC response path: ISR posts finished response frames; main drains them. */
void    usb_tx_post_from_isr(const uint8_t *data, uint16_t len); /* USB-ISR context */
void    usb_tx_pump_responses(void);                            /* MAIN context     */

/* ── RX: CDC ISR produces into a byte ring; main drains + frame-scans ────────
   usb_rx_process runs the resynchronizing proto_frame_scan over the ring and
   hands each complete frame to on_frame (main context). WHAT a frame means is
   the caller's business — usb_link stops at framing. */
typedef void (*usb_rx_frame_cb)(const ProtoScanResult *r);
void usb_rx_from_isr(const uint8_t *buf, uint16_t len);   /* USB-ISR context */
void usb_rx_process(usb_rx_frame_cb on_frame);            /* MAIN context    */

extern uint32_t usb_rx_frames;      /* complete frames delivered to on_frame      */
extern uint32_t usb_rx_scan_errors; /* resync chunks (host sums into link_errors) */
extern uint32_t usb_rx_resyncs;     /* resync events (contiguous discard runs)    */
extern uint32_t usb_rx_discarded;   /* bytes dropped while resyncing              */
extern uint32_t usb_rx_overflows;   /* RX ring overruns (producer outran consumer) */

#endif /* USB_LINK_H */
