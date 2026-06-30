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
#include "robostride.h"
#include "motor_chain.h"
#include "slave_spi.h"
#include "motor_runtime.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PHASE1_POLL_PERIOD_MS MOTOR_LOOP_PERIOD_MS   /* 200 Hz CAN control loop */
#define PHASE1_PRINT_PERIOD_MS 500U
#define PHASE1_LED_PULSE_MS 40U
#define PHASE1_STATUS_LED_GPIO_Port GPIOA
#define PHASE1_STATUS_LED_Pin GPIO_PIN_5
#if defined(__GNUC__)
#define PHASE1_UNUSED_FN __attribute__((unused))
#else
#define PHASE1_UNUSED_FN
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */
static uint32_t phase1_next_poll_ms = 0;
static uint32_t phase1_last_print_ms = 0;
static uint32_t phase1_last_feedback_count = 0;
static uint32_t phase1_led_off_ms = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_SPI1_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch) {
    uint8_t c = (uint8_t)ch;
    HAL_UART_Transmit(&huart4, &c, 1, HAL_MAX_DELAY);
    return ch;
}

static int32_t to_milli(float value)
{
    return (int32_t)(value * 1000.0f);
}

static void append_milli(char *dst, size_t dst_len, int32_t milli)
{
    const char *sign = "";
    int32_t whole;
    int32_t frac;

    if (milli < 0) {
        sign = "-";
        milli = -milli;
    }

    whole = milli / 1000;
    frac = milli % 1000;
    snprintf(dst, dst_len, "%s%ld.%03ld", sign, (long)whole, (long)frac);
}

static void phase1_print_motor_state(const motor_t *motor)
{
    char pos[20];
    char vel[20];
    char torq[20];
    char temp[20];
    char line[160];
    uint32_t fault = 0;

    append_milli(pos, sizeof(pos), to_milli(motor->pos));
    append_milli(vel, sizeof(vel), to_milli(motor->rpm));
    append_milli(torq, sizeof(torq), to_milli(motor->torq));
    append_milli(temp, sizeof(temp), to_milli(motor->temperature));

    fault |= (uint32_t)motor->motor_errors.undervoltage << 0;
    fault |= (uint32_t)motor->motor_errors.driver_fault << 1;
    fault |= (uint32_t)motor->motor_errors.overheat << 2;
    fault |= (uint32_t)motor->motor_errors.encoder_fault << 3;
    fault |= (uint32_t)motor->motor_errors.stall_overload << 4;
    fault |= (uint32_t)motor->motor_errors.uncalibrated << 5;

    int len = snprintf(
        line,
        sizeof(line),
        "motor=%u pos=%s rad vel=%s rad/s tau=%s Nm temp=%s C status=%u fault=0x%02lX\r\n",
        (unsigned)motor->id,
        pos,
        vel,
        torq,
        temp,
        (unsigned)motor->status,
        (unsigned long)fault);

    if (len > 0) {
        if ((size_t)len >= sizeof(line)) {
            len = (int)sizeof(line) - 1;
        }
        HAL_UART_Transmit(&huart4, (uint8_t *)line, (uint16_t)len, 50);
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
  HAL_Delay(2500);  /* motors need ~1.5-2 s to boot CAN stack from cold power-on */
  SCnSCB->ACTLR |= SCnSCB_ACTLR_DISDEFWBUF_Msk ;

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_SPI1_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */

  /* Align CAN motor array IDs with motor_configs */
  for (uint8_t i = 0; i < N_MOTORS; i++) {
    memset(&motors[i], 0, sizeof(motors[i]));
    motors[i].id        = motor_configs[i].can_id;
    motors[i].master_id = CAN_MASTER_ID;
  }

  if (can_bus_init() != HAL_OK) {
    Error_Handler();
  }

  spi_dma_init(&hspi1);
  motor_runtime_init();   /* discover motors via CAN, populate alive mask */

  phase1_next_poll_ms   = HAL_GetTick();
  phase1_last_print_ms  = HAL_GetTick();
  phase1_last_feedback_count = can_feedback_count;
  printf("slave: %u/%u motors alive\r\n",
         (unsigned)__builtin_popcount(motor_runtime_motors_alive()),
         (unsigned)N_MOTORS);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();

    /* 200 Hz CAN poll — motor_runtime handles MIT hold vs read-state per motor */
    if ((int32_t)(now - phase1_next_poll_ms) >= 0) {
      phase1_next_poll_ms += PHASE1_POLL_PERIOD_MS;
      motor_runtime_update(now);
    }

    /* On each new CAN feedback: refresh SPI telemetry buffer */
    if (can_feedback_count != phase1_last_feedback_count) {
      phase1_last_feedback_count = can_feedback_count;
      HAL_GPIO_WritePin(PHASE1_STATUS_LED_GPIO_Port, PHASE1_STATUS_LED_Pin, GPIO_PIN_SET);
      phase1_led_off_ms = now + PHASE1_LED_PULSE_MS;

      if ((now - phase1_last_print_ms) >= PHASE1_PRINT_PERIOD_MS) {
        phase1_last_print_ms = now;
        phase1_print_motor_state(&motors[0]);
      }

      uint8_t tele[PAYLOAD_LENGTH] = {0};
      tele[0] = motor_runtime_motors_alive();
      for (uint8_t _i = 0; _i < N_MOTORS; _i++) {
        motor_runtime_pack_tele((SpiMotorTele *)(&tele[1]) + _i, _i);
      }
      spi_write_next_tx_buf(tele, motor_tele_buf);
    }

    if (phase1_led_off_ms != 0U && (int32_t)(now - phase1_led_off_ms) >= 0) {
      HAL_GPIO_WritePin(PHASE1_STATUS_LED_GPIO_Port, PHASE1_STATUS_LED_Pin, GPIO_PIN_RESET);
      phase1_led_off_ms = 0U;
    }

    /* SPI command handler — dispatch to motor_runtime state machine */
    if (data_receive_flag) {
      data_receive_flag = 0;
      uint8_t cmd = motor_update_buf[0];
      /* Any command proves the master link is alive — refresh EVERY motor's
         watchdog. Otherwise a long blocking op on one motor (e.g. zeroing
         several motors in a row, each ~60 ms) lets an already-armed motor's
         watchdog expire mid-sequence and it falls back to IDLE. */
      for (uint8_t _w = 0; _w < N_MOTORS; _w++)
        motor_runtime_refresh_watchdog(_w);
      switch (cmd & 0x0Fu) {
        case SPI_CMD_ARM: {
          uint8_t idx = SPI_CMD_MOTOR_IDX(cmd);
          motor_runtime_arm(idx, 0u);
          break;
        }
        case SPI_CMD_GOTO_ZERO: {
          uint8_t idx = SPI_CMD_MOTOR_IDX(cmd);
          motor_runtime_goto_zero(idx, 0u);
          break;
        }
        case SPI_CMD_MIT: {
          for (uint8_t _i = 0; _i < N_MOTORS; _i++) {
            SpiMitCmd mc;
            memcpy(&mc, motor_update_buf + 1 + _i * (uint8_t)sizeof(SpiMitCmd),
                   sizeof(SpiMitCmd));
            if (mc.valid) {
              /* Clamp to the motor's soft angle limits on the slave side. */
              motor_runtime_apply_mit(_i, mc.pos, mc.vel);
            }
            /* Master is alive — always refresh watchdog regardless of valid flag */
            motor_runtime_refresh_watchdog(_i);
          }
          break;
        }
        case SPI_CMD_HOLD:
          for (uint8_t _i = 0; _i < N_MOTORS; _i++)
            motor_runtime_refresh_watchdog(_i);
          break;
        case SPI_CMD_DISARM:
          for (uint8_t _i = 0; _i < N_MOTORS; _i++)
            motor_runtime_disable(_i, 0u);
          break;
        default:
          break;
      }
    }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  HAL_GPIO_WritePin(PHASE1_STATUS_LED_GPIO_Port, PHASE1_STATUS_LED_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = PHASE1_STATUS_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PHASE1_STATUS_LED_GPIO_Port, &GPIO_InitStruct);
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
