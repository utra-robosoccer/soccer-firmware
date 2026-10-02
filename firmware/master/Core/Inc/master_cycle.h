/* master_cycle — hardware-timer-driven 200 Hz cycle clock for the master.
 *
 * TIM2 (32-bit) runs free at 1 MHz as a monotonic µs timestamp, and its CH1
 * output-compare fires once per cycle (period from the generated MASTER_POLL_HZ)
 * on an absolute grid (CCR1 += period → no drift). The ISR does no work: it only
 * sets a "cycle due" flag + records the fire time; all poll/assemble work stays in
 * the main loop (MotorMaster_ProcessLoop). See spi_master.c.
 */
#ifndef MASTER_CYCLE_H
#define MASTER_CYCLE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Start TIM2: 1 MHz free-run + CH1 compare every `period_us` (1e6 / MASTER_POLL_HZ). */
void master_cycle_init(uint32_t period_us);

/* Monotonic 32-bit µs timestamp (TIM2->CNT); wraps ~every 71.6 min. */
uint32_t cycle_us_now(void);

/* If a cycle is pending, clear it and return 1 with *tick_us = the ISR fire time
   (µs); else return 0. Single consumer (main loop). */
uint8_t master_cycle_take(uint32_t *tick_us);

/* Count of cycles whose "due" flag was still set when the next tick fired
   (main loop fell a full period behind) — the new missed_deadlines source. */
uint32_t master_cycle_overruns(void);

#ifdef __cplusplus
}
#endif
#endif /* MASTER_CYCLE_H */
