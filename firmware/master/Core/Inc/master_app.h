/* master_app — the master's application layer: the 200 Hz cycle orchestration
 * (mailbox swap → host-death watchdog → slave polls → telemetry emit), USB frame
 * dispatch, and robot-state aggregation. The wires live in spi_link / usb_link;
 * the pure decision logic in common/include (cmd_mailbox.h, host_watchdog.h). */
#ifndef MASTER_APP_H
#define MASTER_APP_H

#include "main.h"
#include "proto_common.h"

void master_app_init(SPI_HandleTypeDef *hspi, UART_HandleTypeDef *huart);
void master_app_loop(void);   /* call forever from main's while(1) */

/* Gated debug output: ONLY to UART4, never the USB CDC (the CDC carries framed
   protocol messages exclusively — docs/protocol.md). Compiled out by default;
   build with -DMASTER_DEBUG_UART to enable. */
#ifdef MASTER_DEBUG_UART
void debug_printf(const char *fmt, ...);
#else
#define debug_printf(...) ((void)0)
#endif

extern uint32_t master_link_errors;   /* app-level rejects (short/bad host frames) */

#endif /* MASTER_APP_H */
