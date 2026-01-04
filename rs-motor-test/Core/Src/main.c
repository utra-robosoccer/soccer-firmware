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
#include "uart_dbg.h"
#include "cubemars.h"
#include "robostride_test.h"

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

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//These are all necessary definitions to use the can bus
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;

uint32_t TxMailbox;

uint8_t recv_msg[8];

char uart_msg[100];

uint8_t can_rx_flag;

#define MAX_MOTOR_COUNT 10 //this defines the total motors
motor_t motors[MAX_MOTOR_COUNT];

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rs_can_rx_header, rx_data);

    // 1. Cast the Extended ID to your struct to interpret bit fields
    // NOTE: This assumes Little Endian ordering matches the hardware register layout
    exCanIdInfo *rx_id_info = (exCanIdInfo *)&rs_can_rx_header.ExtId;

    uint8_t comm_type = rx_id_info->mode; // Bits 24-28
    uint8_t sender_id = 0;

    // 2. Identify Sender Motor ID
    // For Rx frames (Type 2, 17, 0), the Sender ID is usually in Bits 8-15
    // In exCanIdInfo, 'data' covers Bits 8-23, so we mask the lower 8 bits.
    sender_id = (uint8_t)(rx_id_info->data & 0xFF);

    // 3. Dispatch to specific Unpack Function
    if (sender_id > 0 && sender_id <= MAX_MOTOR_COUNT)
    {
        motor_t *target_motor = &motors[sender_id - 1]; // Map ID 1 -> Index 0

        switch (comm_type)
        {
            case 2: // Motor Feedback
                can_unpack_motor_feedback(target_motor, rx_data);
                break;

            case 17: // Single Parameter Read
            {
                float param_value = 0.0f;
                // Unpack the float value from the buffer
                if (can_unpack_single_param(rx_data, &param_value) == HAL_OK) {
                    // TODO: Assign param_value to specific struct member based on Index?
                    // The Index is in rx_data[0] and rx_data[1].
                    // Example: if (rx_data[0] == 0x1E) target_motor->kp = param_value;
                }
                break;
            }

            case 0: // Get ID Response
                can_unpack_get_id(target_motor, rx_data);
                break;

            default:
                // Handle unknown types or other responses (e.g. Type 1 response is Type 2)
                break;
        }


    }
    can_rx_flag = 1;

}

static void get_fault_string(motor_error_t errors, char* buffer) {
    if (errors.undervoltage) sprintf(buffer, "UnderVolt");
    else if (errors.driver_fault) sprintf(buffer, "DriverFault");
    else if (errors.overheat) sprintf(buffer, "OverHeat");
    else if (errors.encoder_fault) sprintf(buffer, "EncoderFault");
    else if (errors.stall_overload) sprintf(buffer, "Stall");
    else if (errors.uncalibrated) sprintf(buffer, "Uncalibrated");
    else sprintf(buffer, "None");
}

static void get_mode_string(motor_status_t status, char* buffer) {
    switch (status) {
        case RS_MODE_RESET:  sprintf(buffer, "RESET"); break;
        case RS_MODE_CALI:   sprintf(buffer, "CALI"); break;
        case RS_MODE_NORMAL: sprintf(buffer, "MOTOR"); break; // 对应手册中的 Motor Mode
        default:             sprintf(buffer, "UNKNOWN"); break;
    }
}

// 统一打印函数：用于 Step 2, 3, 4 的 Type 2 反馈帧
static void print_motor_feedback(motor_t *m, UART_HandleTypeDef *huart) {
    char uart_buf[256];
    char mode_str[10];
    char fault_str[20];

    get_mode_string(m->status, mode_str);
    get_fault_string(m->motor_errors, fault_str);

    // 格式: Motor: 1 Pos: -0.0033 Spd: -0.0571 Torq: -0.0009 Mode: MOTOR Temp: 23.0000 Fault: None
    int len = sprintf(uart_buf,
        "Motor: %d Pos: %.4f Spd: %.4f Torq: %.4f Mode: %s Temp: %.4f Fault: %s\r\n",
        m->id,
        m->pos,
        m->rpm,
        m->torq,
        mode_str,
        m->temperature,
        fault_str
    );
    HAL_UART_Transmit(huart, (uint8_t*)uart_buf, len, 100);
}

// ---------------------------------------------------------
// 主测试程序
// ---------------------------------------------------------
void RoboStride_Test_Routine(void)
{
    uint8_t target_id = 1;
    uint16_t master_id = 0xFD;
    char uart_buf[256];
    uint32_t tickstart;
    motor_t *m = &motors[target_id - 1];
    m->id = target_id;

    // =========================================================
    // 步骤 1: 获取 Motor ID (Type 0)
    // =========================================================
    can_rx_flag = 0;
    if (can_get_motor_id(target_id, master_id) == HAL_OK) {
        tickstart = HAL_GetTick();
        while (can_rx_flag == 0) {
            if ((HAL_GetTick() - tickstart) > 100) {
                HAL_UART_Transmit(&huart2, (uint8_t*)"Timeout: Get ID\r\n", 17, 100);
                break;
            }
        }
        if (can_rx_flag) {
            // 解析 MCU UID
            uint32_t uid_high = (uint32_t)((m->mcu_id >> 32) & 0xFFFFFFFF);
            uint32_t uid_low  = (uint32_t)(m->mcu_id & 0xFFFFFFFF);

            // 获取实际接收到的 Motor ID
            // 根据手册 ，Type 0 应答帧的 CAN ID Bit 23-8 为 Motor ID
            // 我们的结构体 exCanIdInfo 将 Bit 8-23 映射为 .data 域
            // 因此 (ExtId >> 8) & 0xFF 即为接收到的 Motor ID
            uint8_t received_motor_id = (rs_can_rx_header.ExtId >> 8) & 0xFF;

            int len = sprintf(uart_buf, "Step 1 Get ID -> Received Motor ID: %d, MCU UID: 0x%08lX%08lX\r\n",
                              received_motor_id, uid_high, uid_low);
            HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, len, 100);
        }
    }
    HAL_Delay(50);

    // =========================================================
    // 步骤 2: 设置模式为 MIT_MODE (Type 18)
    // =========================================================
    // Type 18 写入成功后，电机会回复 Type 2 反馈帧 [cite: 691]
    can_rx_flag = 0;
    if (can_change_motor_mode(target_id, master_id, MIT_MODE) == HAL_OK) {
        tickstart = HAL_GetTick();
        while (can_rx_flag == 0) {
            if ((HAL_GetTick() - tickstart) > 100) {
                HAL_UART_Transmit(&huart2, (uint8_t*)"Timeout: Set Mode\r\n", 19, 100);
                break;
            }
        }
        if (can_rx_flag) {
            HAL_UART_Transmit(&huart2, (uint8_t*)"Step 2 Set Mode -> ", 19, 100);
            print_motor_feedback(m, &huart2);
        }
    }
    HAL_Delay(50);

    // =========================================================
    // 步骤 3: 使能电机 (Type 3)
    // =========================================================
    // Type 3 使能成功后，电机会回复 Type 2 反馈帧 [cite: 646]
    can_rx_flag = 0;
    if (can_enable_motor(target_id, master_id) == HAL_OK) {
        tickstart = HAL_GetTick();
        while (can_rx_flag == 0) {
            if ((HAL_GetTick() - tickstart) > 100) {
                HAL_UART_Transmit(&huart2, (uint8_t*)"Timeout: Enable Motor\r\n", 23, 100);
                break;
            }
        }
        if (can_rx_flag) {
             HAL_UART_Transmit(&huart2, (uint8_t*)"Step 3 Enable -> ", 17, 100);
             print_motor_feedback(m, &huart2);
        }
    }
    HAL_Delay(50);

    // =========================================================
    // 步骤 4: 发送 MIT 控制指令 (Type 1)
    // =========================================================
    // 设置: Speed=1.0, Kd=10.0. Type 1 发送后回复 Type 2 反馈帧 [cite: 641]
    can_rx_flag = 0;
    if (can_mit_control_set(target_id, 0.0f, 0.0f, 1.0f, 0.0f, 10.0f) == HAL_OK) {

        tickstart = HAL_GetTick();
        while (can_rx_flag == 0) {
            if ((HAL_GetTick() - tickstart) > 100) {
                HAL_UART_Transmit(&huart2, (uint8_t*)"Timeout: MIT Control\r\n", 22, 100);
                break;
            }
        }

        if (can_rx_flag) {
            HAL_UART_Transmit(&huart2, (uint8_t*)"Step 4 Control -> ", 18, 100);
            print_motor_feedback(m, &huart2);
        }
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
  MX_USART2_UART_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
  CAN_FilterTypeDef sFilterConfig;
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_CAN_Start(&hcan1) != HAL_OK){
     return 1;
   }
   HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
   /*
     Go to the can filter config defined in the CAN 1 init function
     Filter Fifo assignment was assigned to CAN rx fifo 0
     since we have turned on thee RX fifo0 intr, RXfifo msg pending callback will be called once incoming data was stored in the RX FIFO0
     count will increment
   */

   RoboStride_Test_Routine();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
