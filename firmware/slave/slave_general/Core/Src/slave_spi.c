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
#include "tx_arm_timer.h"                            /* TX-arm deadline (one-shot TIM3) */

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

/* Late-arm (removes the one-exchange telemetry lag): TxRxCplt no longer arms the next
   exchange — it starts the TX-arm deadline timer, whose ISR calls spi_arm_tx() after the
   motor replies are in. armed_this_cycle makes the arm once-per-cycle; g_hspi is captured at
   init so spi_arm_tx can arm outside the TxRxCplt callback. */
static SPI_HandleTypeDef *g_hspi = 0;
static volatile uint8_t   armed_this_cycle = 0;
volatile uint32_t spi_tx_arm_fails = 0;   /* HAL_SPI_TransmitReceive_DMA arm failures (wraps) */

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

	/* LATE ARM: do NOT arm the next exchange here — the motor's reply to the command just
	   received isn't in yet (it lands ~0.3–2 ms later). Start the deadline timer; its ISR calls
	   spi_arm_tx() at TX_ARM_DEADLINE_US (after the reply window, before the next NSS), so the
	   fresh reply rides the very next exchange (no one-exchange lag). */
	armed_this_cycle = 0u;
	tx_arm_timer_start();
}

/* Arm the DMA for the next exchange with the freshest staged telemetry. Once per cycle:
   the deadline timer ISR arms; resync/error (which claim the cycle) make later callers
   no-op. The TX ping-pong swap happens here (not in TxRxCplt) so A+1 carries this cycle's
   reply. A whole frame is staged before data_tx_ready_flag is set, so the swapped buffer is
   never torn. */
void spi_arm_tx(void)
{
	uint32_t pm = __get_PRIMASK();
	__disable_irq();
	if (armed_this_cycle) { if (!pm) __enable_irq(); return; }
	armed_this_cycle = 1u;
	if (!pm) __enable_irq();

	tx_arm_timer_cancel();                 /* if armed early (future paths) */
	if (data_tx_ready_flag) {
		/* data_tx_ready_flag is set (with a barrier) only after the whole frame incl. CRC
		   is copied, so the swapped half is never torn / half-written. */
		data_tx_ready_flag = 0u;
		if (spi_tx_active == spi_tx_pingpong[0]) {
			spi_tx_active  = spi_tx_pingpong[1];
			tele_stage_buf = spi_tx_pingpong[0];
		} else {
			spi_tx_active  = spi_tx_pingpong[0];
			tele_stage_buf = spi_tx_pingpong[1];
		}
	}
	if (HAL_SPI_TransmitReceive_DMA(g_hspi, spi_tx_active, spi_rx_active, PAYLOAD_LENGTH) != HAL_OK) {
		spi_tx_arm_fails++;
		/* Retry once; a persistent failure surfaces as a CRC error next exchange and the
		   main loop's slave_spi_resync does the full reset + re-arm. */
		g_hspi->State = HAL_SPI_STATE_READY;
		if (HAL_SPI_TransmitReceive_DMA(g_hspi, spi_tx_active, spi_rx_active, PAYLOAD_LENGTH) != HAL_OK)
			spi_tx_arm_fails++;
	}
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
	//if we detected error, we restart dma, since the isr handler has already cleared all flags for us
	spi_error_flag = 1;
	armed_this_cycle = 1u;          // claim the cycle BEFORE touching SPI, so a TIM3 preemption's
	tx_arm_timer_cancel();          // spi_arm_tx no-ops and can't race this re-arm
	HAL_SPI_Abort_IT(hspi);
	if (HAL_SPI_TransmitReceive_DMA(hspi, spi_tx_active, spi_rx_active, PAYLOAD_LENGTH) != HAL_OK)
		spi_tx_arm_fails++;
}

void spi_dma_init(SPI_HandleTypeDef *hspi)
{
	//Critical ping-pong init: DMA on half [0], main owns half [1] each direction
	  g_hspi = hspi;                              // captured for spi_arm_tx (late arm)
	  spi_tx_active  = spi_tx_pingpong[0];
	  tele_stage_buf = spi_tx_pingpong[1];

	  spi_rx_active  = spi_rx_pingpong[0];
	  cmd_inbox_buf  = spi_rx_pingpong[1];
	  armed_this_cycle = 1u;                      // the initial arm below counts as this cycle's
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
	/* Publish AFTER the whole frame (incl. CRC) is written. The barrier stops the compiler/
	   CPU from making the flag visible before the copy, so the deadline ISR that reads the
	   flag and swaps this buffer in can never arm a half-written frame. */
	__DMB();
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
	/* Mask the TX-arm deadline timer: this reset + re-arm and the TIM3 ISR's spi_arm_tx both
	   drive the SPI/DMA, and TIM3 (an ISR) would otherwise preempt this main-loop code. Claim
	   the cycle and cancel the pending fire before touching the peripheral. */
	HAL_NVIC_DisableIRQ(TIM3_IRQn);
	armed_this_cycle = 1u;
	tx_arm_timer_cancel();

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
	if (HAL_SPI_TransmitReceive_DMA(hspi, spi_tx_active, spi_rx_active, PAYLOAD_LENGTH) != HAL_OK)
		spi_tx_arm_fails++;                               // CRC next exchange → resync retries
	spi_resyncs++;                                        // wraps naturally (uint8_t)
	HAL_NVIC_EnableIRQ(TIM3_IRQn);                        // next TxRxCplt restarts the deadline
	return 1u;
}




