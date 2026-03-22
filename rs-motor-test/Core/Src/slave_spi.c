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

uint16_t motor_temp_tele_buf[MAX_MOTOR_COUNT * 2] = {0};

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

// --- Helper Functions ---
static int float_to_uint(float x, float x_min, float x_max, unsigned int bits)
{
    /// Converts a float to an int, given range and number of bits ///
    float span = x_max - x_min;
    if(x < x_min) x = x_min;
    else if(x > x_max) x = x_max;
    return (int) ((x - x_min) * ((float)((1 << bits) / span)));
}

static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

//This function updates all the motors inside the motor chain
//update all motor using motor update buf, wait for feedback, and write feedback to a temper
//using little endian to read from the motor update buffer
/*Buffer is arranged as following:
 *
 * Motor chain id = 1, idx = 0
 * 0x0  LSB POS
 * 0x1  MSB POS
 * 0x2  LSB SPD
 * 0x3  MSB SPD
 *
 * Motor chain id = 2, idx = 1
 * 0x4  LSB POS
 * 0x5  MSB POS
 * 0x6  LSB SPD
 * 0x7  MSB SPD
 *
 * .
 * .
 * .
 *
 * Last motor id = MAX_MOTOR_COUNT, idx = MAX_MOTOR_COUNT - 1
 * 0x<idx * 4>      LSB POS
 * 0x<idx * 4 + 1>	MSB POS
 * 0x<idx * 4 + 2> 	LSB SPD
 * 0x<idx * 4 + 3>  MSB SPD
 *
 */
HAL_StatusTypeDef spi_update_all_motors()
{
	for (int i = 0; i < MAX_MOTOR_COUNT; i ++){
		int16_t raw_new_pos =  motor_update_buf[4*i + 1] << 8 | motor_update_buf[4*i];
		int16_t raw_new_spd =  motor_update_buf[4*i + 3] << 8 | motor_update_buf[4*i + 2];

		float new_pos = uint_to_float(raw_new_pos, P_MIN, P_MAX, 16);
		float new_spd = uint_to_float(raw_new_spd, V_MIN, V_MAX, 16);
		motors[i].set_pos = new_pos;
		motors[i].set_rpm = new_spd;

		//for the rest we leave it as fixed, tbc later
		motors[i].set_kd = 5.0f;
		motors[i].set_kp = 100.0f;
		motors[i].set_torq = 0.0f;
		if(motor_set_mit_dbg(motors[i].id,
		                 motors[i].set_torq,
		                 motors[i].set_pos,
		                 motors[i].set_rpm,
		                 motors[i].set_kp,
		                 motors[i].set_kd) != HAL_OK) return HAL_ERROR;
		//Should correctly update the motor

		motor_temp_tele_buf[2*i] = float_to_uint(motors[i].pos, P_MIN, P_MAX, 16);
		motor_temp_tele_buf[2*i + 1] = float_to_uint(motors[i].rpm, V_MIN, V_MAX, 16);
	}

	spi_write_next_tx_buf((uint8_t*)motor_temp_tele_buf, motor_tele_buf);
	return HAL_OK;
}




