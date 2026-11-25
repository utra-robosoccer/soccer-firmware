/* Renode-only replacement for the slave's SPI-DMA arming primitives.
 *
 * Renode's STM32SPI model is master-only and its stock platform wires no peripheral DMA requests,
 * so the slave's DMA-driven SPI cannot run as-is. The seam is narrow: slave_spi.c talks to the
 * hardware only through HAL_SPI_TransmitReceive_DMA / HAL_SPI_Abort_IT (plus register pokes that
 * are harmless on the model). This file replaces just those two. Everything else in slave_spi.c
 * (ping-pong buffers, late-arm via TIM3, error + resync logic) is the production code, unmodified.
 *
 * "Arming" records the DMA buffers. The C# SPI bridge (sim/renode/peripherals/SpiSlaveBridge.cs) is the
 * other end of the wire: when the master drops CS it reads the armed TX buffer out of this
 * machine's RAM, and when CS rises it writes the master's bytes into the armed RX buffer and
 * raises SIM_SPI_IRQn, whose handler below runs the real HAL_SPI_TxRxCpltCallback.
 *
 * NOT modelled: the DMA engine, byte-level SPI timing, SPI overrun/underrun. (Partial transfers ARE
 * modelled by the bridge: a short exchange leaves the "DMA" part-way.)
 */
#include "main.h"
#include "stm32f4xx_hal.h"

#define SIM_SPI_IRQn SPI3_IRQn          /* unused by the slave; the bridge pulses this line */

extern SPI_HandleTypeDef hspi1;

/* Read by the bridge through ELF symbol addresses; keep them plain and non-static. */
volatile uint32_t sim_spi_armed = 0;    /* 1 while a transfer is armed and waiting for the master */
uint8_t * volatile sim_spi_txp = 0;     /* armed TX buffer (what the master will clock in)        */
uint8_t * volatile sim_spi_rxp = 0;     /* armed RX buffer (where the master's bytes land)        */
volatile uint16_t sim_spi_len = 0;
volatile uint32_t sim_spi_arm_count = 0;
volatile uint32_t sim_spi_done_count = 0;

HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *hspi, const uint8_t *pTxData,
                                              uint8_t *pRxData, uint16_t Size)
{
    if (hspi->State != HAL_SPI_STATE_READY) return HAL_BUSY;     /* same contract as the HAL */
    if (pTxData == NULL || pRxData == NULL || Size == 0u) return HAL_ERROR;
    if (!NVIC_GetEnableIRQ(SIM_SPI_IRQn)) {
        HAL_NVIC_SetPriority(SIM_SPI_IRQn, 0, 0);   /* same level as the real SPI DMA streams */
        HAL_NVIC_EnableIRQ(SIM_SPI_IRQn);
    }
    hspi->State = HAL_SPI_STATE_BUSY_TX_RX;
    sim_spi_txp = (uint8_t *)pTxData;
    sim_spi_rxp = pRxData;
    sim_spi_len = Size;
    sim_spi_arm_count++;
    __DMB();
    sim_spi_armed = 1u;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_SPI_Abort_IT(SPI_HandleTypeDef *hspi)
{
    sim_spi_armed = 0u;
    hspi->State = HAL_SPI_STATE_READY;
    return HAL_OK;
}

/* "DMA transfer complete": what the DMA stream ISR + SPI_DMATransmitReceiveCplt do on hardware. */
void SPI3_IRQHandler(void)
{
    sim_spi_armed = 0u;
    sim_spi_done_count++;
    hspi1.State = HAL_SPI_STATE_READY;
    HAL_SPI_TxRxCpltCallback(&hspi1);
}
