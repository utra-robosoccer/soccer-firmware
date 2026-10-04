/* tx_arm_timer — bare-metal one-shot TIM3 deadline for the slave SPI TX-arm.
 *
 * The slave defers arming the next exchange's DMA until the motor replies to the command
 * from the last exchange are in (so the fresh reply rides the very next exchange — removes
 * the one-exchange telemetry lag). This timer is the guaranteed fallback: started at each
 * TxRxCplt, it fires TX_ARM_DEADLINE_US later and calls spi_arm_tx() if the main loop has
 * not already armed (e.g. a motor never replied, or the main loop is blocked in arm_enable).
 * Mirrors the master's bare-metal master_cycle.c.
 */
#ifndef TX_ARM_TIMER_H
#define TX_ARM_TIMER_H

#include <stdint.h>

void tx_arm_timer_init(uint32_t deadline_us);  /* configure TIM3 (one-pulse) + NVIC */
void tx_arm_timer_start(void);                 /* (re)start the one-shot — from TxRxCplt */
void tx_arm_timer_cancel(void);                /* stop a pending fire — when armed early  */

#endif /* TX_ARM_TIMER_H */
