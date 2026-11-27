/* us_clock.c — see us_clock.h. Bare-metal TIM2 (HAL TIM module unused). */
#include "us_clock.h"
#include "stm32f4xx_hal.h"

void us_clock_init(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();
    /* TIM2 is on APB1; its kernel clock is PCLK1, doubled when the APB1
       prescaler != 1 (same derivation as tx_arm_timer's TIM3). */
    uint32_t tim_clk = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0u) tim_clk *= 2u;

    TIM2->CR1  = 0u;                            /* up-count, free-run          */
    TIM2->PSC  = (tim_clk / 1000000u) - 1u;     /* 1 MHz → 1 µs tick           */
    TIM2->ARR  = 0xFFFFFFFFu;                   /* full 32-bit wrap            */
    TIM2->EGR  = TIM_EGR_UG;                    /* latch PSC/ARR               */
    TIM2->SR   = 0u;                            /* clear the UG-raised flag    */
    TIM2->CNT  = 0u;
    TIM2->CR1 |= TIM_CR1_CEN;
}
