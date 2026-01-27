/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_uart.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "robostride_test.h"
#include "motor_chain.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char uart_msg[100];

#define PAYLOAD_LENGTH 8

// TX & RX buffer declaration in MEM
uint8_t RxBuffer_A[PAYLOAD_LENGTH];
uint8_t RxBuffer_B[PAYLOAD_LENGTH];

uint8_t TxBuffer_A[PAYLOAD_LENGTH] = {0xff, 0xff, 0xff, 0xff, 0, 0,0,0};
uint8_t TxBuffer_B[PAYLOAD_LENGTH] = {0,0,0,0, 0xff,0xff,0xff,0xff};

// SPI DMA TX & RX Memory addr (swapping)
uint8_t* CurTxBuf;
uint8_t* CurRxBuf;

// CAN Bus motor update mem addr (swapping, should always be different from the SPI TX RX mem addr)
uint8_t* motor_update_buf; //This belongs to the Rx side
uint8_t* motor_tele_buf; //This belongs to the Tx side

volatile uint8_t data_receive_flag = 0;
volatile uint8_t data_tx_ready_flag = 0;

float spd_dbg = 1.0f;
uint16_t random_count = 0;

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
	random_count ++;
	if (random_count == 15){
		random_count = 0;
		spd_dbg *= -1;

	}

	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); //toggle a LED if this callback is triggered
	HAL_SPI_TransmitReceive_DMA(&hspi2, CurTxBuf, CurRxBuf, PAYLOAD_LENGTH); //rearm DMA
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi->Instance == SPI2)
	{
		//if we detected error, we restart dma, since the isr handler has already cleared all flags for us
		HAL_SPI_TransmitReceive_DMA(&hspi2, CurTxBuf, CurRxBuf, PAYLOAD_LENGTH);
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  can_rx_flag = 0;

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_CAN1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

  if(can_bus_init() != HAL_OK) return 1;
  if(motor_chain_init() != HAL_OK) return 1;

  //Critical txbuf and rxbuf init
  CurTxBuf = TxBuffer_A;
  motor_tele_buf = TxBuffer_B;

  CurRxBuf = RxBuffer_A;
  motor_update_buf = RxBuffer_B;

  memset(RxBuffer_A, 0, PAYLOAD_LENGTH);
  memset(RxBuffer_B, 0, PAYLOAD_LENGTH);
  memset(TxBuffer_A, 0, PAYLOAD_LENGTH);
  memset(TxBuffer_B, 0, PAYLOAD_LENGTH);

  if (HAL_SPI_TransmitReceive_DMA(&hspi2, CurTxBuf, CurRxBuf, PAYLOAD_LENGTH) != HAL_OK){
	  //Set up DMA here, ready to receive
	  Error_Handler();
  }

//   motor_set_spd(1, 2.0f, 10.0f);
//   motor_set_spd(2, 3.0f, 10.0f);

   uint32_t tick_tele = HAL_GetTick();
   uint32_t tick_inc = HAL_GetTick();
   uint32_t tick_pos_inc = HAL_GetTick();
   float pos_1 = 0;
   float pos_2 = 0;
   float spd_1 = 0;
   float spd_2 = 0;
   float pos_step_1 = 1.2;
   float pos_step_2 = 1.2;
   float step_1 = 0.2;
   float step_2 = 0.1;
//
//   while(motor_check_angle_dbg(&motors[0], 10, spd_1, 5)!= HAL_OK)
//   {
//	   if(HAL_GetTick() - tick_inc >= 1){
//		   tick_inc = HAL_GetTick();
//		   if (spd_1 <= 10){
//			   spd_1 += 0.1;
//		   }
//	   }
//   }
//   spd_1 = 0.0;
//   while(motor_check_angle_dbg(&motors[0], 0, spd_1, 5)!= HAL_OK)
//   {
//   	   if(HAL_GetTick() - tick_inc >= 1){
//   		   tick_inc = HAL_GetTick();
//   		   if (spd_1 >= -5 ){
//   			   spd_1 -= 0.1;
//   		   }
//   	   }
//    }
//   while(motor_check_angle_dbg(&motors[0], 0, -4, 5)!= HAL_OK);
//   while(motor_check_angle_dbg(&motors[0], -10, -5, 5)!= HAL_OK);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	if(HAL_GetTick() - tick_tele >= 1){
//		tick_tele = HAL_GetTick();
//		motor_set_spd(1, spd_1, 10.0f);
//		motor_set_spd(2, spd_2, 10.0f);
////		if (motor_check_angle_dbg(&motors[0], pos_1, spd_1, 10.0f) == HAL_OK){
////			HAL_UART_Transmit(&huart2, (uint8_t*)"motor 1 reaches the configured angle\n", 38, HAL_MAX_DELAY);
////		}
////
////		if (motor_check_angle_dbg(&motors[1], pos_2, spd_2, 10.0f) == HAL_OK){
////			HAL_UART_Transmit(&huart2, (uint8_t*)"motor 2 reaches the configured angle\n", 38, HAL_MAX_DELAY);
////		}
//	}
//
//	if(HAL_GetTick() - tick_inc >= 200){
//		tick_inc = HAL_GetTick();
//		spd_1 += step_1;
//		spd_2 += step_2;
//		if (spd_1 >= 10.0f || spd_1 <= - 10.0f) step_1 *= -1;
//		if (spd_2 >= 10.0f || spd_2 <= - 10.0f) step_2 *= -1;
//
		if (data_receive_flag){
			motor_set_spd_dbg(1, spd_dbg, 10.0f);
			HAL_UART_Transmit(&huart2, motor_update_buf, PAYLOAD_LENGTH, HAL_MAX_DELAY);
			data_receive_flag = 0;
		}

//		if (spd_1 <= 10.0f && spd_1 >= - 10.0f) spd_1 += step_1;
//		if (spd_2 <= 10.0f && spd_2 >= - 10.0f) spd_2 += step_2;
//
//		if (pos_1 >= 12.0f || pos_1 <= - 12.0f) step_1 *= -1;
//		if (pos_2 >= 12.0f || pos_2 <= - 12.0f) step_2 *= -1;
//	}

//	if(HAL_GetTick() - tick_pos_inc >= 1000){
//		tick_pos_inc = HAL_GetTick();
//		pos_1 += 4 * pos_step_1;
//		pos_2 += 2 * pos_step_2;
//
//		if (pos_1 >= 12.0f || pos_1 <= - 12.0f) pos_step_1 *= -1;
//		if (pos_2 >= 12.0f || pos_2 <= - 12.0f) pos_step_2 *= -1;
//	}



  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 2;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_16TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_SLAVE;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
