#ifndef USB_TX_H
#define USB_TX_H

#include <stdint.h>

/* ~4 KB ring buffer behind all USB TX. Call usb_tx_pump() from the main loop
   or from the USB TX-complete callback to drain it.                          */

void    usb_tx_write(const uint8_t *data, uint16_t len);        /* MAIN context only */
void    usb_tx_pump(void);   /* call from main loop; no-op if endpoint busy  */
uint8_t usb_tx_busy(void);   /* non-zero if a DMA transfer is in flight      */

/* SPSC response path: ISR posts finished response frames; main drains them. */
void    usb_tx_post_from_isr(const uint8_t *data, uint16_t len); /* USB-ISR context */
void    usb_tx_pump_responses(void);                            /* MAIN context     */

#endif /* USB_TX_H */
