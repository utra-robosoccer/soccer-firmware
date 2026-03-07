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

// TX & RX buffer declaration in MEM
uint8_t RxBuffer_A[BUFFER_SIZE] = {0x0};
uint8_t RxBuffer_B[BUFFER_SIZE] = {0x0};

uint8_t TxBuffer_A[BUFFER_SIZE] = {0xff, 0xff, 0xff, 0xff, 0, 0,0,0, 0xDE, 0xAD, 0xBE, 0xEF};
uint8_t TxBuffer_B[BUFFER_SIZE] = {0,0,0,0, 0xff,0xff,0xff,0xff, 0xDE, 0xAD, 0xBE, 0xEF};

// SPI DMA TX & RX Memory addr (swapping)
uint8_t* volatile CurTxBuf;
uint8_t* volatile CurRxBuf;

// CAN Bus motor update mem addr (swapping, should always be different from the SPI TX RX mem addr)
uint8_t* volatile motor_update_buf; //This belongs to the Rx side
uint8_t* volatile motor_tele_buf; //This belongs to the Tx side

volatile uint8_t data_receive_flag = 0;
volatile uint8_t data_tx_ready_flag = 0;

float spd_dbg = 1.0f;
uint8_t random_count = 'A';
volatile uint8_t spi_error_flag = 0;

//Callback functions redefinitions
void spi_dbg_helper()
{
	random_count ++;

	if (random_count > 0xFF){
		random_count = 1;
	}
}
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	//Handling RX Double buffer
	if (CurRxBuf == RxBuffer_A){
		CurRxBuf = RxBuffer_B;
		motor_update_buf = RxBuffer_A;
	}
	else {
		CurRxBuf = RxBuffer_A;
		motor_update_buf = RxBuffer_B;
	}

	data_receive_flag = 1;

	//Handling TX Double buffer
	if (data_tx_ready_flag){
		//Main loop signals a complete motor chain telemetry
		data_tx_ready_flag = 0;
		//***The Tx ready flag is set in the CAN receive intr service routine***
		if(CurTxBuf == TxBuffer_A){
			CurTxBuf = TxBuffer_B;
			motor_tele_buf = TxBuffer_A;
		}
		else {
			CurTxBuf = TxBuffer_A;
			motor_tele_buf = TxBuffer_B;
		}
	}

	spi_dbg_helper();
//	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); //toggle a LED if this callback is triggered
	HAL_SPI_TransmitReceive_DMA(hspi, CurTxBuf, CurRxBuf, PAYLOAD_LENGTH); //rearm DMA
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
	//if we detected error, we restart dma, since the isr handler has already cleared all flags for us
	spi_error_flag = 1;
	HAL_SPI_TransmitReceive_DMA(hspi, CurTxBuf, CurRxBuf, PAYLOAD_LENGTH);

}

void spi_dma_init(SPI_HandleTypeDef *hspi)
{
	//Critical txbuf and rxbuf init
	  CurTxBuf = TxBuffer_A;
	  motor_tele_buf = TxBuffer_B;

	  CurRxBuf = RxBuffer_A;
	  motor_update_buf = RxBuffer_B;
	  if (HAL_SPI_TransmitReceive_DMA(hspi, CurTxBuf, CurRxBuf, PAYLOAD_LENGTH) != HAL_OK){
	  //Set up DMA here, ready to receive
		  Error_Handler();
	  }
}


// SPI TX buf write -> copy newest motor_data to the motor_tele_buf
void spi_write_next_tx_buf(const uint8_t* motor_new_data_buf, uint8_t* motor_tele_buf)
{
	memcpy(motor_tele_buf, motor_new_data_buf, PAYLOAD_LENGTH);
	SCB_CleanDCache_by_Addr(motor_tele_buf, PAYLOAD_LENGTH);
	data_tx_ready_flag = 1; //signal -> ok to send motor_tele in the next frame
}



