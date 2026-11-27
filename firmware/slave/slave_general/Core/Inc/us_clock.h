/* us_clock — bare-metal TIM2 (32-bit, APB1) free-running at 1 MHz: the slave's
 * microsecond timebase. Wraps every ~71.6 min, same contract as the master's
 * cycle timer; consumers take deltas. Read-safe from any context (single
 * 32-bit register load). HAL TIM module unused, same style as tx_arm_timer. */
#ifndef US_CLOCK_H
#define US_CLOCK_H

#include <stdint.h>
#include "stm32f4xx.h"

/* Start TIM2 free-running at 1 MHz. Call once at init, before the CAN bus and
 * SPI start (both ISRs stamp timestamps from it). */
void us_clock_init(void);

/* Monotonic 32-bit µs timestamp. */
static inline uint32_t us_now(void)
{
    return TIM2->CNT;
}

#endif /* US_CLOCK_H */
