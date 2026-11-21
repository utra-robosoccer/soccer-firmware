/* can_link — the CAN wire to the RobStride chain: bus init + accept-all filter,
 * the single bounded-wait transmit path, and link-health counters.
 *
 * Layering (mirrors the master's spi_link): robstride.c decides WHAT a frame
 * says (ext-id + 8-byte payload, the vendor codec); can_link handles HOW it
 * moves (mailboxes, filter, counters). RX delivery stays interrupt-driven:
 * can_link's own FIFO0 callback reads each frame out of the peripheral into
 * an ISR-owned buffer and hands it to the can_link_on_frame hook for the app
 * to dispatch and decode.
 */
#ifndef CAN_LINK_H
#define CAN_LINK_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

/* Configure the accept-all filter, start bxCAN, enable the FIFO0 IRQ. */
HAL_StatusTypeDef can_link_init(void);

/* Queue one 8-byte extended frame. The F446 bxCAN has only 3 TX mailboxes;
 * waits (bounded ~2 ms) for a free one so burst sends are not silently
 * dropped — a full extended frame at 1 Mbps drains in ~110 µs. */
HAL_StatusTypeDef can_link_tx(uint32_t ext_id, const uint8_t msg[8]);

/* RX: the FIFO0 HAL callback lives in can_link.c. It reads each frame out of
 * the peripheral and hands it to this hook, which the app (motor_chain.c)
 * implements — dispatch by comm type, LUT, unpack. Runs in ISR context. */
void can_link_on_frame(uint32_t ext_id, uint8_t data[8]);

/* Link-health counters (can_tx_error_count is reported in chain telemetry). */
extern volatile uint32_t can_tx_ok_count;
extern volatile uint32_t can_tx_error_count;
extern volatile uint32_t can_last_tx_error;

/* Raw RX diagnostics (wire-level, gdb-visible): what the peripheral last
 * delivered, before any protocol interpretation. */
extern volatile uint32_t can_raw_rx_count;
extern volatile uint32_t can_last_ext_id;
extern volatile uint32_t can_last_rx_word0;
extern volatile uint32_t can_last_rx_word1;
extern volatile uint8_t  can_last_dlc;
extern volatile uint8_t  can_last_ide;

#endif /* CAN_LINK_H */
