/* master_cycle.c — see master_cycle.h. Bare-metal TIM2 (HAL TIM module is off). */
#include "master_cycle.h"
#include "stm32f4xx_hal.h"

/* TIM2 is on APB1; with APB1 prescaler ≠ 1 the timer clock is 2×APB1 = SYSCLK =
   72 MHz here, so PSC = 71 → 1 MHz (1 µs tick). */
#define TIM2_PSC_1MHZ 71u

static volatile uint8_t  g_due      = 0u;
static volatile uint32_t g_tick_us  = 0u;
static volatile uint32_t g_overruns = 0u;
static uint32_t          g_period_us = 5000u;

void master_cycle_init(uint32_t period_us)
{
    g_period_us = period_us;

    __HAL_RCC_TIM2_CLK_ENABLE();
    TIM2->CR1  = 0u;
    TIM2->PSC  = TIM2_PSC_1MHZ;
    TIM2->ARR  = 0xFFFFFFFFu;          /* free-running 32-bit µs counter */
    TIM2->EGR  = TIM_EGR_UG;           /* latch PSC/ARR */
    TIM2->SR   = 0u;                   /* clear the UG-raised update flag */
    TIM2->CCR1 = TIM2->CNT + period_us;
    TIM2->DIER = TIM_DIER_CC1IE;       /* compare-1 interrupt only */

    HAL_NVIC_SetPriority(TIM2_IRQn, 2, 0);   /* below SPI1/OTG_FS (prio 0) */
    HAL_NVIC_EnableIRQ(TIM2_IRQn);

    TIM2->CR1 = TIM_CR1_CEN;
}

uint32_t cycle_us_now(void) { return TIM2->CNT; }

uint8_t master_cycle_take(uint32_t *tick_us)
{
    if (!g_due) return 0u;
    __disable_irq();
    uint32_t t = g_tick_us;
    g_due = 0u;
    __enable_irq();
    if (tick_us != 0) *tick_us = t;
    return 1u;
}

uint32_t master_cycle_overruns(void) { return g_overruns; }

/* Overrides the weak startup vector. Sets the flag + fire time; no work here. */
void TIM2_IRQHandler(void)
{
    if (TIM2->SR & TIM_SR_CC1IF) {
        TIM2->SR    = ~TIM_SR_CC1IF;      /* clear (rc_w0: writing 1 elsewhere is a no-op) */
        TIM2->CCR1 += g_period_us;        /* next absolute grid point */
        if (g_due) g_overruns++;          /* previous cycle not serviced yet */
        g_tick_us = TIM2->CNT;            /* actual fire time (µs) */
        g_due = 1u;
    }
}
