#ifndef USB_TX_H
#define USB_TX_H

#include <stdint.h>

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

#endif /* USB_TX_H */
