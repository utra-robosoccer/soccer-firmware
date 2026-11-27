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
#include "robstride.h"
#include "can_rx.h"
#include "slave_spi.h"
#include "motor_runtime.h"
#include "tx_arm_timer.h"   /* TX-arm deadline (one-shot TIM3) — removes the telemetry lag */
#include "us_clock.h"       /* TIM2 1 MHz free-run — the slave's µs timebase */
#include "../../../../common/include/slave_service.h"  /* slave_service_due — forward/fallback */
#include "spi_proto.h"     /* SPI frame codec: build telemetry, parse commands */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Compile-time gate for ALL UART4 debug output (TX on pin PA0, 115200 8N1).
   OFF by default: no debug console is wired in the normal build, and
   HAL_UART_Transmit is blocking (~8 ms per line) inside the 200 Hz control loop.
   Set to 1 (e.g. -DSLAVE_UART_DEBUG=1) for bring-up on a UART adapter on PA0. */
#ifndef SLAVE_UART_DEBUG
#define SLAVE_UART_DEBUG 0
#endif
#if SLAVE_UART_DEBUG
#define DBG_PRINTF(...) printf(__VA_ARGS__)
#else
#define DBG_PRINTF(...) ((void)0)
#endif

#define LOOP_POLL_PERIOD_MS MOTOR_LOOP_PERIOD_MS   /* 200 Hz CAN control loop */
#define STATUS_LED_PULSE_MS 40U
#define STATUS_LED_GPIO_Port GPIOA
#define STATUS_LED_Pin GPIO_PIN_5
#if SLAVE_UART_DEBUG
#define DBG_PRINT_PERIOD_MS 500U   /* min ms between debug motor-state prints */
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

#if SLAVE_UART_DEBUG
UART_HandleTypeDef huart4;
#endif

/* USER CODE BEGIN PV */
static uint32_t loop_next_poll_ms = 0;
static uint32_t spi_last_rx_ms    = 0;   /* last completed SPI exchange (dead-link detect) */
#if SLAVE_UART_DEBUG
static uint32_t dbg_last_print_ms = 0;
#endif
static uint32_t last_feedback_count = 0;
static uint32_t status_led_off_ms = 0;
static uint8_t  spi_echo_seq    = 0;   /* SPI link-health seq of the last VALID frame */
static uint32_t cmd_crc_errors  = 0;   /* SPI command frames rejected on CRC */
static uint8_t  g_slave_state   = SLAVE_BOOT;   /* SlaveState (telemetry) */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_SPI1_Init(void);
#if SLAVE_UART_DEBUG
static void MX_UART4_Init(void);
#endif
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch) {
#if SLAVE_UART_DEBUG
    uint8_t c = (uint8_t)ch;
    HAL_UART_Transmit(&huart4, &c, 1, HAL_MAX_DELAY);
#endif
    return ch;
}

#if SLAVE_UART_DEBUG
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

static void dbg_print_motor_state(const motor_t *motor)
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
#endif /* SLAVE_UART_DEBUG */

/* Assemble + stage this slave's tele_slave_t for the next SPI exchange. Called once per
   loop iteration after the service (send) and feedback handling, so it carries the
   freshest mirrored motor state and the cmd_seq confirmation paired from the most recent
   reply — which the master then reads on the very next exchange. Also called once with
   g_slave_state = BOOT before the blocking boot checks and with RESETTING before a
   blocking reset round, so the master keeps reading an honest frame while we block. */
static void stage_telemetry(void)
{
    tele_slave_t tele;
    memset(&tele, 0, sizeof(tele));
    tele.time_slave_us = spi_last_exchange_us;  /* µs stamp of the exchange that delivered
                                                   the command this telemetry answers
                                                   (stamped in the TxRxCplt ISR) */
    tele.state    = g_slave_state;
    tele.n_motors = N_MOTORS;
    motor_runtime_boot_result(&tele.status.boot);
    tele.status.spi.cmd_crc_errors = (uint16_t)cmd_crc_errors;
    tele.status.spi.resyncs        = spi_resyncs;               /* DMA realigns (wraps) */
    tele.status.spi.tx_arm_fails   = (uint16_t)spi_tx_arm_fails;/* TX-arm DMA failures (wraps) */
    tele.status.spi.seq_echo       = spi_echo_seq;
    tele.status.can.tx_errors      = (uint16_t)can_tx_error_count;
    can_link_error_counters(&tele.status.can.tec, &tele.status.can.rec);
    for (uint8_t _i = 0; _i < N_MOTORS; _i++) {
        motor_runtime_sample(&tele.motors[_i], _i);
    }
    uint8_t frame[PAYLOAD_LENGTH];
    spi_proto_build_tele(frame, &tele);
    spi_write_next_tx_buf(frame, tele_stage_buf);
}

/* RESETTING exit: the host dropped SLAVE_REQ_RESET (or stopped commanding at
   all) → settle to READY or BLOCKED on the re-sweep verdict. Motors still
   dirty stay MOTOR_FAULT. */
static void resetting_drop(void)
{
    if (g_slave_state == SLAVE_RESETTING) {
        g_slave_state = motor_runtime_boot_ok() ? SLAVE_READY : SLAVE_BLOCKED;
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
#if SLAVE_UART_DEBUG
  MX_UART4_Init();
#endif
  /* USER CODE BEGIN 2 */

  /* Bind CAN ids into the (private) motors[] before the bus starts, so the RX
     ISR's id lookup can match feedback. */
  can_rx_bind_ids();

  us_clock_init();   /* µs timebase up BEFORE the CAN/SPI ISRs that stamp from it */

  if (can_link_init() != HAL_OK) {
    Error_Handler();
  }

  tx_arm_timer_init(TX_ARM_DEADLINE_US);  /* ready BEFORE the first TxRxCplt starts it */
  spi_dma_init(&hspi1);
  stage_telemetry();      /* pre-stage a SLAVE_BOOT frame: exchanges run from the
                             ISRs while the blocking boot checks sweep the bus,
                             so the master reads an honest BOOT state meanwhile */
  motor_runtime_init();   /* blocking: id sweep + zero_sta + fw-version verify */
  g_slave_state = motor_runtime_boot_ok() ? SLAVE_READY : SLAVE_BLOCKED;
  stage_telemetry();      /* publish the verdict (READY/BLOCKED + boot_result) */

  loop_next_poll_ms   = HAL_GetTick();
  spi_last_rx_ms      = loop_next_poll_ms;
  last_feedback_count = can_feedback_count;

  /* µs-resolution timebase (DWT cycle counter) for forward/fallback service gating.
     Thresholds scale with the generated master cycle, so 200/400 Hz both work. */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0u;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
  const uint32_t cyc_per_us    = SystemCoreClock / 1000000u;
  const uint32_t svc_min_ticks = (MASTER_CYCLE_US / 2u) * cyc_per_us;         /* 0.5 cycle */
  const uint32_t svc_fb_ticks  = ((MASTER_CYCLE_US * 8u) / 5u) * cyc_per_us;  /* 1.6 cycle */
  uint32_t last_send_cyc = 0u;   /* last service of any kind (cycles)       */
  uint32_t last_exch_cyc = 0u;   /* last exchange-driven service (cycles)   */
#if SLAVE_UART_DEBUG
  dbg_last_print_ms   = HAL_GetTick();
#endif
  DBG_PRINTF("slave: %u/%u motors alive\r\n",
         (unsigned)__builtin_popcount(motor_runtime_motors_alive()),
         (unsigned)N_MOTORS);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now     = HAL_GetTick();
    uint32_t now_cyc = DWT->CYCCNT;
    uint8_t  need_stage = 0u;   /* stage telemetry once at iteration end if anything changed */

    /* SPI command handler FIRST — forward-on-command: service the motors immediately
       on every valid (CRC-OK) exchange, so a new command reaches the motor without
       waiting for the slave's own tick. Each exchange is one master cycle. */
    if (data_receive_flag) {
      /* Fix 1 (torn-parse): cmd_inbox_buf is a pointer the SPI ISR reswaps, so
         parsing it in place could split a command across two transfers. Copy the
         current command frame into a main-owned local under a brief IRQ mask
         (COPY, not latch: after a swap the old buffer becomes the DMA's next
         write target), then parse only the local. */
      uint8_t cmd_local[sizeof(spi_cmd_frame_t)];
      __disable_irq();
      memcpy(cmd_local, (const void *)cmd_inbox_buf, sizeof(cmd_local));
      data_receive_flag = 0;
      __enable_irq();
      spi_last_rx_ms = now;   /* link alive: an exchange completed */

      /* Verify the command-frame CRC and extract the header + cmd_slave_t. On
         a CRC failure apply nothing (a corrupt command must not refresh
         watchdogs or move a motor), count it, and resync the DMA — send
         nothing this exchange. */
      spi_header_t hdr;
      cmd_slave_t  cmd;
      if (!spi_proto_parse_cmd(cmd_local, &hdr, &cmd)) {
        cmd_crc_errors++;
        slave_spi_resync(&hspi1);
      } else {
        spi_echo_seq = hdr.spi_seq;   /* echo the SPI link-health seq */
        uint8_t cmd_fresh = (hdr.opcode == SPI_OP_CMD) ? 1u : 0u;

        if (cmd_fresh && cmd.req == SLAVE_REQ_RESET) {
          /* Slave-level reset: accepted only while nothing is armed. Each
             round BLOCKS for seconds (clear + re-sweep + verify) behind a
             pre-staged RESETTING frame; exchanges keep running from the ISRs
             and commands received meanwhile are simply superseded. While the
             request stays asserted, re-run rounds until everything reports
             clean; the exit (→ READY/BLOCKED) happens when it is dropped.
             Per-motor requests in a RESET-bearing command are ignored. */
          if (g_slave_state == SLAVE_RESETTING) {
            if (!motor_runtime_reset_clean()) {
              stage_telemetry();
              motor_runtime_reset_round();
              stage_telemetry();
            }
          } else if (!motor_runtime_any_armed()) {
            g_slave_state = SLAVE_RESETTING;
            stage_telemetry();
            motor_runtime_reset_round();
            stage_telemetry();
          }
          /* armed → ignore the request entirely */
        } else if (cmd_fresh) {
          resetting_drop();   /* a non-RESET command drops an in-progress reset */
          /* Only a real command refreshes the master-link watchdog — a NOP keepalive does
             NOT, so the slave's master-loss ramp still fires if the master falls
             back to keepalives or dies. Level-triggered per-motor requests. */
          for (uint8_t _w = 0; _w < N_MOTORS; _w++)
            motor_runtime_refresh_watchdog(_w);
          for (uint8_t _i = 0; _i < N_MOTORS; _i++) {
            motor_runtime_apply_cmd(_i, &cmd.motors[_i], hdr.cmd_seq);
          }
        } else {
          resetting_drop();   /* NOPs = no host command → the request is gone */
        }
        /* The arm handshake is async now (arm_sm.h) so apply_cmd no longer blocks; the
           time-base refresh stays as cheap insurance that watchdog/fault checks never
           run against a `now` captured before a slow apply. */
        now     = HAL_GetTick();
        now_cyc = DWT->CYCCNT;
        /* Forward service: drive the applied command (cmd_fresh) or, on a NOP, hold/ramp.
           The min-guard prevents a double send. */
        if (slave_service_due(SVC_EXCHANGE, 1u, now_cyc, last_send_cyc, last_exch_cyc,
                              svc_min_ticks, svc_fb_ticks)) {
          motor_runtime_update(now, cmd_fresh);
          last_send_cyc = now_cyc;
          last_exch_cyc = now_cyc;
          need_stage = 1u;
        }
      }
    }

    /* SPI dead-link recovery: the DMA re-arm chain is completion-driven
       (TxRxCplt → TIM3 → spi_arm_tx), so a failed arm — the error callback's
       re-arm losing its race with the abort, or HAL refusing the TIM3 arm —
       leaves the link DEAD: no exchange ever completes again, and the
       CRC-triggered resync below can't fire without a received frame (bench:
       a slave rebooted under a free-running master wedged this way within
       ~250 ms, spi_tx_arm_fails=2, DMA NDTR=0, unrecoverable). Detect it by
       silence: no completed exchange for SPI_DEAD_RESYNC_MS while the DMA is
       not cleanly armed → same NSS-gated resync. A silent master over a
       cleanly armed DMA (host off, master dead) is NOT a dead link — no churn. */
    if (spi_error_flag) {
      /* The error callback / a failed TIM3 arm flagged the link dead: resync
         NOW (clean NSS-gated reset + re-arm) — each exchange spent waiting is
         a lost command. */
      spi_error_flag = 0;
      slave_spi_resync(&hspi1);     /* NSS-gated; a stuck-low NSS skips and retries */
      spi_last_rx_ms = now;
    } else if ((int32_t)(now - spi_last_rx_ms) >= (int32_t)SPI_DEAD_RESYNC_MS) {
      /* Silence backstop for failure modes that raise no flag. */
      uint8_t armed_clean =
          (uint8_t)(hspi1.State == HAL_SPI_STATE_BUSY_TX_RX &&
                    hspi1.hdmarx != NULL &&
                    __HAL_DMA_GET_COUNTER(hspi1.hdmarx) == PAYLOAD_LENGTH);
      if (!armed_clean) {
        slave_spi_resync(&hspi1);
      }
      spi_last_rx_ms = now;         /* rate-limit: re-judge after another window */
    }

    /* Fallback tick — services ONLY when exchanges have stopped (SPI link lost), so
       the motors keep holding until the watchdog trips. In normal streaming every
       exchange forwards, so this never fires (no double send). */
    if ((int32_t)(now - loop_next_poll_ms) >= 0) {
      loop_next_poll_ms += LOOP_POLL_PERIOD_MS;
      if (slave_service_due(SVC_FALLBACK, 1u, now_cyc, last_send_cyc, last_exch_cyc,
                            svc_min_ticks, svc_fb_ticks)) {
        resetting_drop();   /* exchanges stopped entirely → the request is gone */
        motor_runtime_update(now, /*cmd_fresh=*/0u);   /* no command → hold/ramp */
        last_send_cyc = now_cyc;
        need_stage = 1u;
      }
    }

    /* On each new CAN feedback: mirror the reply into motor state + pair cmd_seq NOW
       (not at the next exchange), so the telemetry staged below carries the freshest
       state and this reply's confirmation → read by the master on the next exchange. */
    if (can_feedback_count != last_feedback_count) {
      last_feedback_count = can_feedback_count;
      motor_runtime_on_feedback(now);
      need_stage = 1u;
      HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);
      status_led_off_ms = now + STATUS_LED_PULSE_MS;

#if SLAVE_UART_DEBUG
      if ((now - dbg_last_print_ms) >= DBG_PRINT_PERIOD_MS) {
        dbg_last_print_ms = now;
        motor_t snap0;
        motor_get_snapshot(0, &snap0);          /* coherent read for the debug print */
        dbg_print_motor_state(&snap0);
      }
#endif
    }

    /* Stage telemetry once per iteration, after both the service (send + any state/fault
       change) and feedback (fresh state + confirmation) — so the next exchange carries the
       freshest frame. Gated so an idle spin doesn't rebuild needlessly. */
    if (need_stage) {
      stage_telemetry();
    }
    /* TX-arm is deadline-driven (one-shot TIM3 → spi_arm_tx at TX_ARM_DEADLINE_US, after the
       reply window and before the next exchange). An earlier all-replied arm was tried but
       its reply-jitter-coupled timing skipped last_applied values (superseded); the fixed
       deadline is deterministic and one cycle sooner than the old TxRxCplt arm. */

    if (status_led_off_ms != 0U && (int32_t)(now - status_led_off_ms) >= 0) {
      HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);
      status_led_off_ms = 0U;
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
#if SLAVE_UART_DEBUG
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
#endif /* SLAVE_UART_DEBUG */

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
  HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = STATUS_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STATUS_LED_GPIO_Port, &GPIO_InitStruct);
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
