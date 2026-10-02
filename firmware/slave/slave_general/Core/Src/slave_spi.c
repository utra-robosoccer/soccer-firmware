 /*

 * slave_spi.c
 *
 *  Created on: 2026年1月21日
 *      Author: 18701
 */
//IOC config:
// SPI -> Slave mode
// Hardware NSS
// TX RX DMA enabled. Intr enabled. Priority high
//

// For all spi slaves, the spi init should be like this
//static void MX_SPI2_Init(void)
//{
//
//  /* USER CODE BEGIN SPI2_Init 0 */
//
//  /* USER CODE END SPI2_Init 0 */
//
//  /* USER CODE BEGIN SPI2_Init 1 */
//
//  /* USER CODE END SPI2_Init 1 */
//  /* SPI2 parameter configuration*/
//  hspi2.Instance = SPI2;
//  hspi2.Init.Mode = SPI_MODE_SLAVE;
//  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
//  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
//  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
//  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
//  hspi2.Init.NSS = SPI_NSS_HARD_INPUT;
//  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
//  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
//  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
//  hspi2.Init.CRCPolynomial = 10;
//  if (HAL_SPI_Init(&hspi2) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN SPI2_Init 2 */
//
//  /* USER CODE END SPI2_Init 2 */
//
//}

#include "slave_spi.h"
#include "string.h"
#include "cachel1_armv7.h"
#include "../../../../common/include/spi_resync.h"  /* spi_resync_poll — NSS gating */

/* NSS (PA4) read + bound for the resync gate. One exchange is ≤ ~1 ms even at the
   slowest prescaler, so NSS returns high well within this; the bound only guards a
   genuinely stuck-low line (dead master) so the control loop never blocks. */
#define SPI_NSS_HIGH()        ((GPIOA->IDR & GPIO_PIN_4) != 0u)
#define SPI_RESYNC_TIMEOUT_MS 5u

volatile uint8_t spi_resyncs = 0;   /* wraps; host takes deltas */

// SPI DMA ping-pong buffers. One half is clocked by the DMA while the other is
// owned by the main loop; the TxRxCplt ISR swaps roles at the end of each
// transfer. [0]/[1] are the two halves of each direction.
uint8_t spi_rx_pingpong[2][BUFFER_SIZE] = {{0x0}, {0x0}};
uint8_t spi_tx_pingpong[2][BUFFER_SIZE] = {
    {0xff, 0xff, 0xff, 0xff, 0, 0, 0, 0, 0xDE, 0xAD, 0xBE, 0xEF},
    {0, 0, 0, 0, 0xff, 0xff, 0xff, 0xff, 0xDE, 0xAD, 0xBE, 0xEF},
};

// Which half the DMA is actively clocking, vs the half the main loop owns.
uint8_t* volatile spi_tx_active;   // DMA is sending this half
uint8_t* volatile spi_rx_active;   // DMA is filling this half
uint8_t* volatile cmd_inbox_buf;   // completed RX frame — main READS (command in)
uint8_t* volatile tele_stage_buf;  // inactive TX frame — main WRITES (telemetry out)

volatile uint8_t data_receive_flag = 0;
volatile uint8_t data_tx_ready_flag = 0;

volatile uint8_t spi_error_flag = 0;

//Callback functions redefinitions
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	//Handling RX ping-pong: hand the just-filled half to main, keep the other for DMA
	if (spi_rx_active == spi_rx_pingpong[0]){
		spi_rx_active = spi_rx_pingpong[1];
		cmd_inbox_buf = spi_rx_pingpong[0];
	}
	else {
		spi_rx_active = spi_rx_pingpong[0];
		cmd_inbox_buf = spi_rx_pingpong[1];
	}

	data_receive_flag = 1;

	//Handling TX ping-pong (only when main has staged a fresh telemetry frame)
	if (data_tx_ready_flag){
		//***The Tx ready flag is set in the CAN receive intr service routine***
		data_tx_ready_flag = 0;
		if(spi_tx_active == spi_tx_pingpong[0]){
			spi_tx_active  = spi_tx_pingpong[1];
			tele_stage_buf = spi_tx_pingpong[0];
		}
		else {
			spi_tx_active  = spi_tx_pingpong[0];
			tele_stage_buf = spi_tx_pingpong[1];
		}
	}

	HAL_SPI_TransmitReceive_DMA(hspi, spi_tx_active, spi_rx_active, PAYLOAD_LENGTH); //rearm DMA
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
	//if we detected error, we restart dma, since the isr handler has already cleared all flags for us
	spi_error_flag = 1;
	HAL_SPI_Abort_IT(hspi);
	HAL_SPI_TransmitReceive_DMA(hspi, spi_tx_active, spi_rx_active, PAYLOAD_LENGTH);

}

void spi_dma_init(SPI_HandleTypeDef *hspi)
{
	//Critical ping-pong init: DMA on half [0], main owns half [1] each direction
	  spi_tx_active  = spi_tx_pingpong[0];
	  tele_stage_buf = spi_tx_pingpong[1];

	  spi_rx_active  = spi_rx_pingpong[0];
	  cmd_inbox_buf  = spi_rx_pingpong[1];
	  if (HAL_SPI_TransmitReceive_DMA(hspi, spi_tx_active, spi_rx_active, PAYLOAD_LENGTH) != HAL_OK){
	  //Set up DMA here, ready to receive
		  Error_Handler();
	  }
}


// Copy a freshly-built telemetry frame into the inactive TX half (tele_stage_buf),
// then flag it ready so the next TxRxCplt swaps it in.
void spi_write_next_tx_buf(const uint8_t* src_frame, uint8_t* dst)
{
	for(int i = 0; i < PAYLOAD_LENGTH; i ++){
		dst[i] = src_frame[i];
	}
	data_tx_ready_flag = 1; //signal -> ok to send this frame next transfer
}

// Re-align the DMA after a bad exchange: abort + re-arm so the NEXT exchange starts
// at byte 0. Gated on NSS high (between exchanges) so we never re-arm mid-exchange.
uint8_t slave_spi_resync(SPI_HandleTypeDef *hspi)
{
	uint32_t t0 = HAL_GetTick();
	for (;;) {
		SpiResyncAction a = spi_resync_poll((uint8_t)SPI_NSS_HIGH(),
		                                    HAL_GetTick() - t0, SPI_RESYNC_TIMEOUT_MS);
		if (a == SPI_RESYNC_WAIT)    continue;            // mid-exchange: wait for NSS high
		if (a == SPI_RESYNC_TIMEOUT) return 0u;           // stuck low: skip, retry next cycle
		break;                                            // PROCEED: NSS high, safe to re-arm
	}
	/* Slave-safe reset: don't use HAL_SPI_Abort (its BSY wait needs a clock and can
	   hang in slave mode). Disable SPI, abort both DMA streams, flush the RX FIFO and
	   clear OVR so no stale byte offsets the next transfer, then re-arm. The re-armed
	   DMA's byte 0 lands on the next NSS select, re-aligning the stream. */
	__HAL_SPI_DISABLE(hspi);
	if (hspi->hdmarx) HAL_DMA_Abort(hspi->hdmarx);
	if (hspi->hdmatx) HAL_DMA_Abort(hspi->hdmatx);
	while (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_RXNE)) { (void)hspi->Instance->DR; }
	__HAL_SPI_CLEAR_OVRFLAG(hspi);
	hspi->State = HAL_SPI_STATE_READY;                    // let the HAL accept a fresh transfer
	// Re-init the ping-pong to the known starting split (as spi_dma_init).
	spi_tx_active  = spi_tx_pingpong[0];
	tele_stage_buf = spi_tx_pingpong[1];
	spi_rx_active  = spi_rx_pingpong[0];
	cmd_inbox_buf  = spi_rx_pingpong[1];
	data_receive_flag = 0;                                // drop the misaligned frame
	HAL_SPI_TransmitReceive_DMA(hspi, spi_tx_active, spi_rx_active, PAYLOAD_LENGTH);
	spi_resyncs++;                                        // wraps naturally (uint8_t)
	return 1u;
}




