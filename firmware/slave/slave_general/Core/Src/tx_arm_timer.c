/* tx_arm_timer.c — see tx_arm_timer.h. Bare-metal TIM3 (HAL TIM module unused). */
#include "tx_arm_timer.h"
#include "slave_spi.h"          /* spi_arm_tx() */
#include "stm32f4xx_hal.h"

void tx_arm_timer_init(uint32_t deadline_us)
{
    if (deadline_us == 0u) deadline_us = 1u;

    __HAL_RCC_TIM3_CLK_ENABLE();
    /* TIM3 is on APB1; its kernel clock is PCLK1, doubled when the APB1 prescaler != 1. */
    uint32_t tim_clk = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0u) tim_clk *= 2u;

    TIM3->CR1  = TIM_CR1_OPM;                        /* one-pulse: CEN clears at update   */
    TIM3->PSC  = (tim_clk / 1000000u) - 1u;          /* 1 MHz → 1 µs tick                 */
    TIM3->ARR  = deadline_us;                        /* update (fire) deadline_us later   */
    TIM3->EGR  = TIM_EGR_UG;                         /* latch PSC/ARR                     */
    TIM3->SR   = 0u;                                 /* clear the UG-raised update flag   */
    TIM3->DIER = TIM_DIER_UIE;                       /* update interrupt                  */

    HAL_NVIC_SetPriority(TIM3_IRQn, 1, 0);           /* below SPI1/DMA (prio 0)           */
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

void tx_arm_timer_start(void)
{
    TIM3->CNT  = 0u;
    TIM3->SR   = 0u;
    TIM3->CR1 |= TIM_CR1_CEN;                        /* OPM clears CEN at the next update */
}

void tx_arm_timer_cancel(void)
{
    TIM3->CR1 &= ~TIM_CR1_CEN;
    TIM3->SR   = 0u;
}

/* Overrides the weak startup vector. Deadline reached → arm the next exchange's DMA with
   whatever telemetry is staged (spi_arm_tx no-ops if the main loop already armed). */
void TIM3_IRQHandler(void)
{
    if (TIM3->SR & TIM_SR_UIF) {
        TIM3->SR = ~TIM_SR_UIF;
        spi_arm_tx();
    }
}
