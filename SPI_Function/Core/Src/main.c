/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    float position;
    float speed;
} MotorCmd;

typedef enum {
    DEV1 = 1,   // PE2
    DEV2 = 2,   // PE4
    DEV3 = 3,   // PE5
    DEV4 = 4    // PE6
} SpiDevId;

typedef struct {
  uint8_t found;
  uint8_t slave_index;
  SpiDevId dev;
} MotorRoute;

typedef struct {
  uint8_t valid;
  uint8_t motor_id;
  uint8_t slave_index;
  float position;
  float speed;
  uint32_t tick;
} MotorCommSnapshot;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUM_SLAVES            4
#define BYTES_PER_MOTOR       4
#define MAX_TOTAL_MOTORS      16
#define MAX_TXRX_BYTES        (MAX_TOTAL_MOTORS * BYTES_PER_MOTOR)

// --- Constants (DO NOT CHANGE) ---
#define P_MIN   -12.57f
#define P_MAX    12.57f
#define V_MIN   -20.0f
#define V_MAX    20.0f
#define KP_MIN    0.0f
#define KP_MAX 5000.0f
#define KD_MIN    0.0f
#define KD_MAX  100.0f
#define T_MIN   -60.0f
#define T_MAX    60.0f

typedef struct {
  uint8_t motor_count_per_slave[NUM_SLAVES];
  uint8_t motor_ids[NUM_SLAVES][MAX_TOTAL_MOTORS];
} SlaveRoutingConfig;

typedef struct {
  uint8_t enabled;
  uint8_t max_consecutive_spi_errors;
  uint32_t control_loop_period_ms;
  uint32_t command_cache_timeout_ms;
} SafetyTuneConfig;

typedef struct {
  uint32_t phase_period_ms;
  uint8_t test_motor_count;
  uint8_t test_motor_ids[MAX_TOTAL_MOTORS];
  float single_step_position;
  float single_step_speed;
  float alternating_low_position;
  float alternating_high_position;
  float alternating_speed_base;
  float alternating_speed_step;
  float sweep_start_position;
  float sweep_position_step;
  float sweep_speed_start;
  float sweep_speed_step;
  float sweep_direction_limit;
} DebugTuneConfig;

typedef struct {
  SlaveRoutingConfig routing;
  SafetyTuneConfig safety;
  DebugTuneConfig debug;
} MasterTuneConfig;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
/* All tunable parameters live here.
   Change this block only when adjusting motor routing, safety, or debug test behavior. */
static const MasterTuneConfig g_master_config = {
  .routing = {
    .motor_count_per_slave = {2U, 1U, 0U, 1U},  /* Recommended: match your real wiring per slave */
    .motor_ids = {
      {2U, 5U},  /* DEV1 motors: recommended logical IDs for this slave */
      {9U},      /* DEV2 motors: recommended logical IDs for this slave */
      {0U},      /* DEV3 unused: keep zeroed if no motors are attached */
      {15U}      /* DEV4 motors: recommended logical IDs for this slave */
    }
  },
  .safety = {
    .enabled = 1U,                    /* Recommended: 1U for normal operation, 0U only for bring-up */
    .max_consecutive_spi_errors = 3U, /* Recommended: 3U */
    .control_loop_period_ms = 1U,     /* Recommended: 1U for fast control loop */
    .command_cache_timeout_ms = 100U  /* Recommended: 100U, increase if host updates slowly */
  },
  .debug = {
    .phase_period_ms = 600U,          /* Recommended: 600U for visible test transitions */
    .test_motor_count = 4U,           /* Recommended: match the number of motors you want to exercise */
    .test_motor_ids = {2U, 5U, 9U, 15U}, /* Recommended: list the logical motor IDs used by test cases */
    .single_step_position = 6.0f,     /* Recommended: 6.0f */
    .single_step_speed = 4.0f,        /* Recommended: 4.0f */
    .alternating_low_position = -4.0f,/* Recommended: -4.0f */
    .alternating_high_position = 4.0f,/* Recommended: 4.0f */
    .alternating_speed_base = 2.0f,   /* Recommended: 2.0f */
    .alternating_speed_step = 0.5f,   /* Recommended: 0.5f */
    .sweep_start_position = -10.0f,   /* Recommended: -10.0f */
    .sweep_position_step = 1.5f,      /* Recommended: 1.5f */
    .sweep_speed_start = 1.0f,        /* Recommended: 1.0f */
    .sweep_speed_step = 0.25f,        /* Recommended: 0.25f */
    .sweep_direction_limit = 10.0f    /* Recommended: 10.0f */
  }
};

typedef enum {
  MASTER_MODE_DEBUG = 0,
  MASTER_MODE_REAL = 1
} MasterMode;

typedef enum {
  CMD_SOURCE_TEST = 0,
  CMD_SOURCE_HOST = 1
} CommandSource;

static volatile MasterMode g_master_mode = MASTER_MODE_DEBUG;

/* Master-side safety reminder:
   - Tune all safety knobs here.
   - Turn safety checks on/off with enabled.
   - command_cache is what real mode actually sends. */
typedef struct {
  uint8_t enabled;
  uint8_t estop_latched;
  uint8_t command_cache_valid;
  uint8_t consecutive_spi_errors;
  uint8_t max_consecutive_spi_errors;
  uint8_t active_command_source;
  uint32_t control_loop_period_ms;
  uint32_t command_cache_timeout_ms;
  uint32_t command_cache_tick;
  uint32_t tx_total_count;
  uint32_t tx_success_count;
  uint32_t tx_fail_count;
  uint32_t last_success_tick;
  uint32_t last_error_tick;
  uint32_t estop_latch_count;
  char last_error_reason[32];
  MotorCommSnapshot motor_comm_snapshot[MAX_TOTAL_MOTORS];
  MotorCmd command_cache[MAX_TOTAL_MOTORS];
} MasterSafetyControl;

static MasterSafetyControl g_master_safety = {
  .enabled = 1U,
  .estop_latched = 0U,
  .command_cache_valid = 0U,
  .consecutive_spi_errors = 0U,
  .max_consecutive_spi_errors = 3U,
  .active_command_source = CMD_SOURCE_TEST,
  .control_loop_period_ms = 1U,
  .command_cache_timeout_ms = 100U,
  .command_cache_tick = 0U,
  .tx_total_count = 0U,
  .tx_success_count = 0U,
  .tx_fail_count = 0U,
  .last_success_tick = 0U,
  .last_error_tick = 0U,
  .estop_latch_count = 0U,
  .last_error_reason = "none",
  .motor_comm_snapshot = {0},
  .command_cache = {0}
};

static MotorCmd g_test_cmd[MAX_TOTAL_MOTORS] = {
    {0.0f, 0.0f},
    {0.0f, 0.0f},
    {0.0f, 0.0f},
    {0.0f, 0.0f}
};

static MotorCmd g_host_cmd[MAX_TOTAL_MOTORS] = {
    {0.0f, 0.0f},
    {0.0f, 0.0f},
    {0.0f, 0.0f},
    {0.0f, 0.0f}
};

static MotorCmd g_motor_cmd[MAX_TOTAL_MOTORS] = {
    {0.0f, 0.0f},
    {0.0f, 0.0f},
    {0.0f, 0.0f},
    {0.0f, 0.0f}
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
static void uart_printf(const char *fmt, ...);
static void uart_dump_bytes(const char *tag, const uint8_t *buf, int n);
static inline void CS_ALL_HIGH(void);
static inline void CS_SELECT(SpiDevId dev);
static float clamp_float(float value, float min_value, float max_value);
static uint16_t get_total_configured_motors(void);
static void pack_one_motor(uint8_t *out, const MotorCmd *cmd);
static uint16_t build_slave_buf(uint8_t slave_index, uint8_t *tx_buf, const MotorCmd *all_motor_cmd);
static HAL_StatusTypeDef spi_send_to_one_slave(SpiDevId dev, uint8_t slave_index, const MotorCmd *all_motor_cmd);
static HAL_StatusTypeDef spi_update_all_slaves_param(const MotorCmd *all_motor_cmd);
static void uart_print_one_motor_rx(uint16_t motor_index, const uint8_t *rx);
/* Resolve a logical motor ID to its slave and slot position. */
static uint8_t find_motor_route_by_id(uint8_t motor_id, MotorRoute *route);
/* Get the logical motor ID stored in one slave slot. */
static uint8_t get_motor_id_by_route(uint8_t slave_index, uint8_t motor_index_in_slave);
/* Get the logical motor ID using the configured global order. */
static uint8_t get_motor_id_by_global_slot(uint16_t global_slot);
/* Get the logical motor ID used by test cases. */
static uint8_t get_debug_test_motor_id(uint8_t test_slot);
/* Apply one received host command and transmit only the target slave. */
static HAL_StatusTypeDef submit_host_command_by_motor_id(uint8_t motor_id, float displacement, float speed);
/* Write a host command directly into the logical motor-ID slot. */
static uint8_t set_host_command_by_motor_id(uint8_t motor_id, float position, float speed);
/* Copy host command table into the bounded/safe cache. */
static void publish_command_cache(const MotorCmd *source);
static void set_command_source(CommandSource source);
static const MotorCmd* get_active_command_source(void);
static void copy_command_table(MotorCmd *target, const MotorCmd *source);
static void set_last_error_reason(const char *reason);
/* Record the latest communication state for one motor. */
static void update_motor_comm_snapshot(uint8_t motor_id, uint8_t slave_index, float position, float speed);
/* Dump the motor communication snapshot table over UART. */
static void dump_motor_comm_snapshots(void);
/* Build a safe fallback command (all motors stop). */
static void build_safe_stop_commands(MotorCmd *target);
/* Latch ESTOP once a safety condition is violated. */
static void latch_emergency_stop(const char *reason);
static void clear_emergency_stop_manual(void);
/* Debug mode test cases runner. */
static void run_debug_test_cycle(float *dir, uint32_t *phase_tick, uint8_t *phase, uint8_t *motor_index);
/* Real mode loop: send cached commands + enforce safety checks. */
static void run_real_master_cycle(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Convert float command to bounded integer field for protocol packing. */
static int float_to_uint(float x, float x_min, float x_max, unsigned int bits)
{
    float span = x_max - x_min;
    if (x < x_min) x = x_min;
    else if (x > x_max) x = x_max;
    return (int)((x - x_min) * ((float)((1U << bits) - 1U) / span));
}

/* Decode integer field back to float for debug visibility. */
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1U << bits) - 1U)) + offset;
}

static float clamp_float(float value, float min_value, float max_value)
{
  if (value < min_value) {
    return min_value;
  }
  if (value > max_value) {
    return max_value;
  }
  return value;
}

/* UART helper for quick runtime reminders and debug traces. */
static void uart_printf(const char *fmt, ...)
{
    char buf[128];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (n < 0) return;
    if (n >= (int)sizeof(buf)) n = sizeof(buf) - 1;

    HAL_UART_Transmit(&huart3, (uint8_t*)buf, (uint16_t)strlen(buf), 100);
}

/* Dump packet bytes in hex format. */
static void uart_dump_bytes(const char *tag, const uint8_t *buf, int n)
{
    uart_printf("%s", tag);
    for (int i = 0; i < n; i++) {
        uart_printf(" %02X", buf[i]);
    }
    uart_printf("\r\n");
}

/* Deassert all slave chip-select lines. */
static inline void CS_ALL_HIGH(void)
{
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6, GPIO_PIN_SET);
}

/* Assert one slave CS at a time for targeted transfer. */
static inline void CS_SELECT(SpiDevId dev)
{
    CS_ALL_HIGH();
    switch (dev) {
        case DEV1: HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET); break;
        case DEV2: HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET); break;
        case DEV3: HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET); break;
        case DEV4: HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_RESET); break;
        default: break;
    }
}

/* Sum current motor allocation across all slaves. */
static uint16_t get_total_configured_motors(void)
{
    uint16_t total = 0;
    for (int i = 0; i < NUM_SLAVES; i++) {
    total += g_master_config.routing.motor_count_per_slave[i];
    }
    return total;
}

/* Build one 4-byte motor command payload (position + speed). */
static void pack_one_motor(uint8_t *out, const MotorCmd *cmd)
{
    uint16_t pos_u16 = (uint16_t)float_to_uint(cmd->position, P_MIN, P_MAX, 16);
    uint16_t spd_u16 = (uint16_t)float_to_uint(cmd->speed, V_MIN, V_MAX, 16);

    out[0] = (uint8_t)(pos_u16 & 0xFF);        // POS LSB
    out[1] = (uint8_t)((pos_u16 >> 8) & 0xFF); // POS MSB
    out[2] = (uint8_t)(spd_u16 & 0xFF);        // SPD LSB
    out[3] = (uint8_t)((spd_u16 >> 8) & 0xFF); // SPD MSB
}

static void uart_print_one_motor_encoding(uint16_t motor_index, const MotorCmd *cmd)
{
    uint16_t pos_u16 = (uint16_t)float_to_uint(cmd->position, P_MIN, P_MAX, 16);
    uint16_t spd_u16 = (uint16_t)float_to_uint(cmd->speed,    V_MIN, V_MAX, 16);

    uart_printf("[MOTOR %d] float: pos=%.3f spd=%.3f\r\n",
                (int)motor_index,
                cmd->position,
                cmd->speed);

    uart_printf("[MOTOR %d] uint16: pos=%u (0x%04X) spd=%u (0x%04X)\r\n",
                (int)motor_index,
                pos_u16, pos_u16,
                spd_u16, spd_u16);

    uart_printf("[MOTOR %d] bytes: %02X %02X %02X %02X\r\n",
                (int)motor_index,
                (uint8_t)(pos_u16 & 0xFF),
                (uint8_t)((pos_u16 >> 8) & 0xFF),
                (uint8_t)(spd_u16 & 0xFF),
                (uint8_t)((spd_u16 >> 8) & 0xFF));
}

static void uart_print_one_motor_rx(uint16_t motor_index, const uint8_t *rx)
{
    uint16_t pos_u16 = (uint16_t)rx[0] | ((uint16_t)rx[1] << 8);
    uint16_t spd_u16 = (uint16_t)rx[2] | ((uint16_t)rx[3] << 8);

    float pos_f = uint_to_float((int)pos_u16, P_MIN, P_MAX, 16);
    float spd_f = uint_to_float((int)spd_u16, V_MIN, V_MAX, 16);

    uart_printf("[RX MOTOR %d] bytes : %02X %02X %02X %02X\r\n",
                (int)motor_index,
                rx[0], rx[1], rx[2], rx[3]);

    uart_printf("[RX MOTOR %d] uint16: pos=%u (0x%04X) spd=%u (0x%04X)\r\n",
                (int)motor_index,
                pos_u16, pos_u16,
                spd_u16, spd_u16);

    uart_printf("[RX MOTOR %d] float : pos=%.3f spd=%.3f\r\n",
                (int)motor_index,
                pos_f,
                spd_f);
}

/* Build per-slave TX payload from the global motor command table. */
static uint16_t build_slave_buf(uint8_t slave_index,
                                uint8_t *tx_buf,
                                const MotorCmd *all_motor_cmd)
{
  uint16_t motor_count = g_master_config.routing.motor_count_per_slave[slave_index];

    for (uint16_t m = 0; m < motor_count; m++) {
    uint8_t motor_id = get_motor_id_by_route(slave_index, (uint8_t)m);

    if (motor_id < MAX_TOTAL_MOTORS) {
      pack_one_motor(&tx_buf[m * BYTES_PER_MOTOR], &all_motor_cmd[motor_id]);
    } else {
      MotorCmd safe_cmd = {0.0f, 0.0f};
      pack_one_motor(&tx_buf[m * BYTES_PER_MOTOR], &safe_cmd);
    }
    }

    return motor_count * BYTES_PER_MOTOR;
}

static uint8_t get_motor_id_by_route(uint8_t slave_index, uint8_t motor_index_in_slave)
{
  if (slave_index >= NUM_SLAVES) {
    return MAX_TOTAL_MOTORS;
  }

  if (motor_index_in_slave >= g_master_config.routing.motor_count_per_slave[slave_index]) {
    return MAX_TOTAL_MOTORS;
  }

  return g_master_config.routing.motor_ids[slave_index][motor_index_in_slave];
}

static uint8_t get_motor_id_by_global_slot(uint16_t global_slot)
{
  uint16_t cursor = 0U;

  for (uint8_t slave_index = 0U; slave_index < NUM_SLAVES; slave_index++) {
    for (uint8_t motor_index_in_slave = 0U; motor_index_in_slave < g_master_config.routing.motor_count_per_slave[slave_index]; motor_index_in_slave++) {
      if (cursor == global_slot) {
        return get_motor_id_by_route(slave_index, motor_index_in_slave);
      }
      cursor++;
    }
  }

  return MAX_TOTAL_MOTORS;
}

static uint8_t get_debug_test_motor_id(uint8_t test_slot)
{
  if (test_slot >= g_master_config.debug.test_motor_count) {
    return MAX_TOTAL_MOTORS;
  }

  return g_master_config.debug.test_motor_ids[test_slot];
}

static uint8_t find_motor_route_by_id(uint8_t motor_id, MotorRoute *route)
{
  if (route == NULL) {
    return 0U;
  }

  memset(route, 0, sizeof(*route));

  for (uint8_t slave_index = 0U; slave_index < NUM_SLAVES; slave_index++) {
    for (uint8_t motor_index_in_slave = 0U; motor_index_in_slave < g_master_config.routing.motor_count_per_slave[slave_index]; motor_index_in_slave++) {
      if (g_master_config.routing.motor_ids[slave_index][motor_index_in_slave] == motor_id) {
        route->found = 1U;
        route->slave_index = slave_index;
        route->dev = (SpiDevId)(DEV1 + slave_index);
        return 1U;
      }
    }
  }

  return 0U;
}

static uint8_t set_host_command_by_motor_id(uint8_t motor_id, float position, float speed)
{
  MotorRoute route;

  if (motor_id >= MAX_TOTAL_MOTORS) {
    return 0U;
  }

  if (find_motor_route_by_id(motor_id, &route) == 0U) {
    return 0U;
  }

  g_host_cmd[motor_id].position = clamp_float(position, P_MIN, P_MAX);
  g_host_cmd[motor_id].speed = clamp_float(speed, V_MIN, V_MAX);

  return 1U;
}

static void update_motor_comm_snapshot(uint8_t motor_id,
                                      uint8_t slave_index,
                                      float position,
                                      float speed)
{
  if (motor_id >= MAX_TOTAL_MOTORS) {
    return;
  }

  g_master_safety.motor_comm_snapshot[motor_id].valid = 1U;
  g_master_safety.motor_comm_snapshot[motor_id].motor_id = motor_id;
  g_master_safety.motor_comm_snapshot[motor_id].slave_index = slave_index;
  g_master_safety.motor_comm_snapshot[motor_id].position = position;
  g_master_safety.motor_comm_snapshot[motor_id].speed = speed;
  g_master_safety.motor_comm_snapshot[motor_id].tick = HAL_GetTick();
}

static void dump_motor_comm_snapshots(void)
{
  uart_printf("\r\n=== Motor Comm Snapshot Dump ===\r\n");

  for (uint8_t motor_id = 0U; motor_id < MAX_TOTAL_MOTORS; motor_id++) {
    const MotorCommSnapshot *snapshot = &g_master_safety.motor_comm_snapshot[motor_id];

    if (snapshot->valid == 0U) {
      continue;
    }

    uart_printf("motor=%u slave=%u pos=%.3f speed=%.3f tick=%lu\r\n",
                (unsigned int)snapshot->motor_id,
                (unsigned int)(snapshot->slave_index + 1U),
                snapshot->position,
                snapshot->speed,
                (unsigned long)snapshot->tick);
  }

  uart_printf("=== End Snapshot Dump ===\r\n");
}

static HAL_StatusTypeDef submit_host_command_by_motor_id(uint8_t motor_id, float displacement, float speed)
{
  MotorRoute route;

  if (find_motor_route_by_id(motor_id, &route) == 0U) {
    uart_printf("[HOST CMD] unknown motor id=%u\r\n", (unsigned int)motor_id);
    return HAL_ERROR;
  }

  if (set_host_command_by_motor_id(motor_id, displacement, speed) == 0U) {
    return HAL_ERROR;
  }

  set_command_source(CMD_SOURCE_HOST);
  publish_command_cache(g_host_cmd);
  update_motor_comm_snapshot(motor_id,
                             route.slave_index,
                             g_master_safety.command_cache[motor_id].position,
                             g_master_safety.command_cache[motor_id].speed);

  uart_printf("[HOST CMD] id=%u -> DEV%u, displacement=%.2f speed=%.2f\r\n",
              (unsigned int)motor_id,
              (unsigned int)(route.slave_index + 1U),
              g_master_safety.command_cache[motor_id].position,
              g_master_safety.command_cache[motor_id].speed);

  return spi_send_to_one_slave(route.dev, route.slave_index, g_master_safety.command_cache);
}

/* Transaction helper for one slave: build, transmit, receive, and print. */
static HAL_StatusTypeDef spi_send_to_one_slave(SpiDevId dev,
                                               uint8_t slave_index,
                                               const MotorCmd *all_motor_cmd)
{
    uint8_t tx_buf[MAX_TXRX_BYTES] = {0};
    uint8_t rx_buf[MAX_TXRX_BYTES] = {0};

    if (dev < DEV1 || dev > DEV4) return HAL_ERROR;
    if (slave_index >= NUM_SLAVES) return HAL_ERROR;

    uint16_t tx_len = build_slave_buf(slave_index, tx_buf, all_motor_cmd);
    if (tx_len == 0) {
        uart_printf("[SPI] dev=%d skip (0 motor)\r\n", (int)dev);
        return HAL_OK;
    }

    uart_printf("[SPI] start dev=%d, motors=%d, bytes=%d\r\n",
                (int)dev,
                (int)g_master_config.routing.motor_count_per_slave[slave_index],
                (int)tx_len);

    for (uint16_t m = 0; m < g_master_config.routing.motor_count_per_slave[slave_index]; m++) {
        uint8_t motor_id = get_motor_id_by_route(slave_index, (uint8_t)m);
      update_motor_comm_snapshot(motor_id,
                                 slave_index,
                                 all_motor_cmd[motor_id].position,
                                 all_motor_cmd[motor_id].speed);
        uart_print_one_motor_encoding(motor_id, &all_motor_cmd[motor_id]);
    }

    uart_dump_bytes("[SPI] TX:", tx_buf, tx_len);

    CS_SELECT(dev);
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(&hspi1,
                                                   tx_buf,
                                                   rx_buf,
                                                   tx_len,
                                                   HAL_MAX_DELAY);
    CS_ALL_HIGH();

    if (st == HAL_OK) {
        uart_dump_bytes("[SPI] RX:", rx_buf, tx_len);

        for (uint16_t m = 0; m < g_master_config.routing.motor_count_per_slave[slave_index]; m++) {
            uint8_t motor_id = get_motor_id_by_route(slave_index, (uint8_t)m);
          update_motor_comm_snapshot(motor_id,
                                     slave_index,
                                     all_motor_cmd[motor_id].position,
                                     all_motor_cmd[motor_id].speed);
            uart_print_one_motor_rx(motor_id, &rx_buf[m * BYTES_PER_MOTOR]);
        }

        HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
    } else {
        uart_printf("[SPI] dev=%d transmit error\r\n", (int)dev);
        HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    }

    return st;
}

/* Master scheduler entry: push one control cycle to all configured slaves. */
static HAL_StatusTypeDef spi_update_all_slaves_param(const MotorCmd *all_motor_cmd)
{
    HAL_StatusTypeDef st;

    st = spi_send_to_one_slave(DEV1, 0, all_motor_cmd);
    if (st != HAL_OK) return st;

    st = spi_send_to_one_slave(DEV2, 1, all_motor_cmd);
    if (st != HAL_OK) return st;

    st = spi_send_to_one_slave(DEV3, 2, all_motor_cmd);
    if (st != HAL_OK) return st;

    st = spi_send_to_one_slave(DEV4, 3, all_motor_cmd);
    if (st != HAL_OK) return st;

    return HAL_OK;
}

static void copy_command_table(MotorCmd *target, const MotorCmd *source)
{
  for (uint16_t i = 0; i < MAX_TOTAL_MOTORS; i++) {
    target[i] = source[i];
  }
}

static void set_last_error_reason(const char *reason)
{
  strncpy(g_master_safety.last_error_reason, reason, sizeof(g_master_safety.last_error_reason) - 1U);
  g_master_safety.last_error_reason[sizeof(g_master_safety.last_error_reason) - 1U] = '\0';
}

static void set_command_source(CommandSource source)
{
  g_master_safety.active_command_source = (uint8_t)source;
}

static const MotorCmd* get_active_command_source(void)
{
  if (g_master_safety.active_command_source == (uint8_t)CMD_SOURCE_HOST) {
    return g_host_cmd;
  }
  return g_test_cmd;
}

static void build_safe_stop_commands(MotorCmd *target)
{
  for (uint16_t i = 0; i < MAX_TOTAL_MOTORS; i++) {
    target[i].position = 0.0f;
    target[i].speed = 0.0f;
  }
}

static void publish_command_cache(const MotorCmd *source)
{
  for (uint16_t i = 0; i < MAX_TOTAL_MOTORS; i++) {
    g_master_safety.command_cache[i].position = clamp_float(source[i].position, P_MIN, P_MAX);
    g_master_safety.command_cache[i].speed = clamp_float(source[i].speed, V_MIN, V_MAX);
  }

  g_master_safety.command_cache_valid = 1U;
  g_master_safety.command_cache_tick = HAL_GetTick();
}

static void latch_emergency_stop(const char *reason)
{
  if (!g_master_safety.estop_latched) {
    uart_printf("[SAFETY] ESTOP latched: %s\r\n", reason);
  }

  g_master_safety.estop_latched = 1U;
  g_master_safety.estop_latch_count++;
  g_master_safety.consecutive_spi_errors = 0U;
  g_master_safety.last_error_tick = HAL_GetTick();
  set_last_error_reason(reason);
  build_safe_stop_commands(g_master_safety.command_cache);
  g_master_safety.command_cache_valid = 1U;
  g_master_safety.command_cache_tick = HAL_GetTick();
}

static void clear_emergency_stop_manual(void)
{
  g_master_safety.estop_latched = 0U;
  g_master_safety.consecutive_spi_errors = 0U;
  g_master_safety.command_cache_valid = 0U;
  set_last_error_reason("manual estop clear");
  uart_printf("[SAFETY] ESTOP manually cleared\r\n");
}

static void run_debug_test_cycle(float *dir, uint32_t *phase_tick, uint8_t *phase, uint8_t *motor_index)
{
  const uint32_t phase_period_ms = g_master_config.debug.phase_period_ms;
  const uint8_t test_motor_count = g_master_config.debug.test_motor_count;

  if (test_motor_count == 0U) {
    uart_printf("[TEST] no debug test motors configured\r\n");
    HAL_Delay(100);
    return;
  }

  if (HAL_GetTick() - *phase_tick >= phase_period_ms) {
    *phase_tick = HAL_GetTick();
    *phase = (uint8_t)((*phase + 1U) % 4U);
    *motor_index = (uint8_t)((*motor_index + 1U) % test_motor_count);
  }

  switch (*phase) {
    case 0:
      build_safe_stop_commands(g_test_cmd);
      uart_printf("[TEST] phase 0: all motors idle\r\n");
      break;

    case 1:
      build_safe_stop_commands(g_test_cmd);
      {
        uint8_t motor_id = get_debug_test_motor_id(*motor_index);
        if (motor_id < MAX_TOTAL_MOTORS) {
          g_test_cmd[motor_id].position = g_master_config.debug.single_step_position * (*dir);
          g_test_cmd[motor_id].speed = g_master_config.debug.single_step_speed;
          uart_printf("[TEST] phase 1: single motor step test on ID %u\r\n", (unsigned int)motor_id);
        }
      }
      break;

    case 2:
      for (uint8_t i = 0; i < test_motor_count; i++) {
        uint8_t motor_id = get_debug_test_motor_id(i);
        if (motor_id < MAX_TOTAL_MOTORS) {
          g_test_cmd[motor_id].position = (i & 1U) ? g_master_config.debug.alternating_low_position : g_master_config.debug.alternating_high_position;
          g_test_cmd[motor_id].speed = g_master_config.debug.alternating_speed_base + ((float)i * g_master_config.debug.alternating_speed_step);
        }
      }
      uart_printf("[TEST] phase 2: alternating multi-motor pattern\r\n");
      break;

    default:
      for (uint8_t i = 0; i < test_motor_count; i++) {
        uint8_t motor_id = get_debug_test_motor_id(i);
        if (motor_id < MAX_TOTAL_MOTORS) {
          g_test_cmd[motor_id].position = g_master_config.debug.sweep_start_position + (g_master_config.debug.sweep_position_step * (float)i);
          g_test_cmd[motor_id].speed = g_master_config.debug.sweep_speed_start + ((float)i * g_master_config.debug.sweep_speed_step);
        }
      }
      uart_printf("[TEST] phase 3: boundary sweep pattern\r\n");
      break;
  }

  {
    uint8_t motor_id = get_debug_test_motor_id(*motor_index);
    if (motor_id < MAX_TOTAL_MOTORS) {
      if (g_test_cmd[motor_id].position >= g_master_config.debug.sweep_direction_limit ||
          g_test_cmd[motor_id].position <= -g_master_config.debug.sweep_direction_limit) {
        *dir = -*dir;
      }

      uart_printf("[TEST] motor ID %u cmd: pos=%.2f spd=%.2f\r\n",
            (unsigned int)motor_id,
            g_test_cmd[motor_id].position,
            g_test_cmd[motor_id].speed);
    }
  }

  set_command_source(CMD_SOURCE_TEST);
  publish_command_cache(g_test_cmd);
  spi_update_all_slaves_param(g_master_safety.command_cache);
  HAL_Delay(1);
}

static void run_real_master_cycle(void)
{
  const uint32_t now = HAL_GetTick();
  const MotorCmd *active_cmd = get_active_command_source();

  if (!g_master_safety.command_cache_valid) {
    publish_command_cache(active_cmd);
  }

  /* If safety is enabled, timeout and SPI error checks can latch ESTOP.
     If safety is disabled, commands are still sent but checks are bypassed. */
  if (g_master_safety.enabled != 0U) {
    if (g_master_safety.estop_latched != 0U) {
      build_safe_stop_commands(g_master_safety.command_cache);
    } else if ((now - g_master_safety.command_cache_tick) > g_master_safety.command_cache_timeout_ms) {
      latch_emergency_stop("command cache timeout");
    }
  }

  if (spi_update_all_slaves_param(g_master_safety.command_cache) == HAL_OK) {
    g_master_safety.tx_total_count++;
    g_master_safety.tx_success_count++;
    g_master_safety.last_success_tick = HAL_GetTick();
    g_master_safety.consecutive_spi_errors = 0U;
  } else {
    g_master_safety.tx_total_count++;
    g_master_safety.tx_fail_count++;
    g_master_safety.last_error_tick = HAL_GetTick();
    set_last_error_reason("spi transmit failed");
    if (g_master_safety.consecutive_spi_errors < 255U) {
      g_master_safety.consecutive_spi_errors++;
    }
    if ((g_master_safety.enabled != 0U) &&
        (g_master_safety.consecutive_spi_errors >= g_master_safety.max_consecutive_spi_errors)) {
      latch_emergency_stop("too many SPI errors");
    }
  }

  HAL_Delay(g_master_safety.control_loop_period_ms);
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  uart_printf("\r\n=== SPI parameterized motor demo ===\r\n");
  uart_printf("Configured motors total = %d\r\n", (int)get_total_configured_motors());
  if (get_total_configured_motors() > MAX_TOTAL_MOTORS) {
    Error_Handler();
  }

  g_master_safety.enabled = g_master_config.safety.enabled;
  g_master_safety.max_consecutive_spi_errors = g_master_config.safety.max_consecutive_spi_errors;
  g_master_safety.control_loop_period_ms = g_master_config.safety.control_loop_period_ms;
  g_master_safety.command_cache_timeout_ms = g_master_config.safety.command_cache_timeout_ms;

  set_command_source(CMD_SOURCE_TEST);
  copy_command_table(g_host_cmd, g_motor_cmd);
  copy_command_table(g_test_cmd, g_motor_cmd);
  publish_command_cache(get_active_command_source());
  float dir = 1.0;
  uint32_t phase_tick = HAL_GetTick();
  uint8_t test_phase = 0;
  uint8_t test_motor_index = 0;
  uint8_t prev_user_btn = 0U;
  uint32_t user_btn_press_tick = 0U;
  uint8_t user_btn_long_dumped = 0U;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    uint8_t user_btn = (HAL_GPIO_ReadPin(USER_Btn_GPIO_Port, USER_Btn_Pin) == GPIO_PIN_SET) ? 1U : 0U;
    if ((user_btn != 0U) && (prev_user_btn == 0U)) {
      user_btn_press_tick = HAL_GetTick();
      user_btn_long_dumped = 0U;
      clear_emergency_stop_manual();
    }
    if ((user_btn != 0U) && (user_btn_long_dumped == 0U) &&
        ((HAL_GetTick() - user_btn_press_tick) >= 1500U)) {
      dump_motor_comm_snapshots();
      user_btn_long_dumped = 1U;
    }
    prev_user_btn = user_btn;

    /* Reminder:
       - DEBUG mode: built-in test patterns to validate path/endpoints.
       - REAL mode : send current command cache with safety policy. */
    if (g_master_mode == MASTER_MODE_DEBUG)
    {
      run_debug_test_cycle(&dir, &phase_tick, &test_phase, &test_motor_index);
    }
    else
    {
      set_command_source(CMD_SOURCE_HOST);
      run_real_master_cycle();
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE4 PE5 PE6 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

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
#ifdef USE_FULL_ASSERT
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
