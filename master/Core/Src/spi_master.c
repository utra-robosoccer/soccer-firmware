/*
 * spi_master.c
 *
 *  Created on: 2026年4月19日
 *      Author: 18701
 */

#include "spi_master.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

typedef struct {
    float position;
    float speed;
} MotorCmd;

typedef enum {
    DEV1 = 1,
    DEV2 = 2,
    DEV3 = 3,
    DEV4 = 4
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

typedef struct {
    uint8_t uart_print_enabled;
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

typedef enum {
    CMD_SOURCE_TEST = 0,
    CMD_SOURCE_HOST = 1
} CommandSource;

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
    uint16_t pending_command_mask;
    uint8_t poll_start_slave_index;
    uint8_t poll_resume_slave_index;
    char last_error_reason[32];
    MotorCommSnapshot motor_comm_snapshot[MAX_TOTAL_MOTORS];
    MotorCmd command_cache[MAX_TOTAL_MOTORS];
} MasterSafetyControl;

static SPI_HandleTypeDef *master_hspi = NULL;
static UART_HandleTypeDef *master_huart = NULL;
static volatile MasterMode_t g_master_mode = MASTER_MODE_DEBUG;

static const MasterTuneConfig g_master_config = {
    .routing = {
        .uart_print_enabled = 1U,
        .motor_count_per_slave = {3U, 0U, 0U, 1U},
        .motor_ids = {
            {2U, 4U, 5U},
            {9U},
            {0U},
            {15U}
        }
    },
    .safety = {
        .enabled = 1U,
        .max_consecutive_spi_errors = 3U,
        .control_loop_period_ms = 1U,
        .command_cache_timeout_ms = 100U
    },
    .debug = {
        .phase_period_ms = 600U,
        .test_motor_count = 3U,
        .test_motor_ids = {2U, 4U, 5U},
        .single_step_position = 6.0f,
        .single_step_speed = 4.0f,
        .alternating_low_position = -4.0f,
        .alternating_high_position = 4.0f,
        .alternating_speed_base = 2.0f,
        .alternating_speed_step = 0.5f,
        .sweep_start_position = -10.0f,
        .sweep_position_step = 1.5f,
        .sweep_speed_start = 1.0f,
        .sweep_speed_step = 0.25f,
        .sweep_direction_limit = 10.0f
    }
};

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
    .pending_command_mask = 0U,
    .poll_start_slave_index = 0U,
    .poll_resume_slave_index = 0U,
    .last_error_reason = "none",
    .motor_comm_snapshot = {0},
    .command_cache = {0}
};

static MotorCmd g_test_cmd[MAX_TOTAL_MOTORS] = {0};
static MotorCmd g_host_cmd[MAX_TOTAL_MOTORS] = {0};

static float dir = 1.0;
static uint32_t phase_tick = 0;
static uint8_t test_phase = 0;
static uint8_t test_motor_index = 0;

static void set_command_source(CommandSource source);
static const MotorCmd* get_active_command_source(void);
static void publish_command_cache(const MotorCmd *source);
static void build_safe_stop_commands(MotorCmd *target);
static void latch_emergency_stop(const char *reason);
static void process_pending_host_commands(void);
static uint8_t select_oldest_slave_for_poll(const uint8_t *polled, uint32_t *priority_tick);
static HAL_StatusTypeDef spi_send_to_one_slave(SpiDevId dev, uint8_t slave_index, const MotorCmd *all_motor_cmd);
static uint8_t get_motor_id_by_route(uint8_t slave_index, uint8_t motor_index_in_slave);
static void update_motor_comm_snapshot_from_feedback(uint8_t motor_id, uint8_t slave_index, const uint8_t *rx);
static uint8_t find_motor_route_by_id(uint8_t motor_id, MotorRoute *route);
static uint8_t set_host_command_by_motor_id(uint8_t motor_id, float position, float speed);
static void queue_host_command_by_motor_id(uint8_t motor_id, float position, float speed);
static void run_debug_test_cycle(float *p_dir, uint32_t *p_phase_tick, uint8_t *p_phase, uint8_t *p_motor_index);
static void run_real_master_cycle(void);

static int float_to_uint(float x, float x_min, float x_max, unsigned int bits) {
    float span = x_max - x_min;
    if (x < x_min) x = x_min;
    else if (x > x_max) x = x_max;
    return (int)((x - x_min) * ((float)((1U << bits) - 1U) / span));
}

static float uint_to_float(int x_int, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1U << bits) - 1U)) + offset;
}

static float clamp_float(float value, float min_value, float max_value) {
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static void uart_printf(const char *fmt, ...) {
    if (!master_huart || g_master_config.routing.uart_print_enabled == 0U) return;
    char buf[128];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n > 0) HAL_UART_Transmit(master_huart, (uint8_t*)buf, (uint16_t)n, 100);
}

static void uart_dump_bytes(const char *tag, const uint8_t *buf, int n) {
    uart_printf("%s", tag);
    for (int i = 0; i < n; i++) uart_printf(" %02X", buf[i]);
    uart_printf("\r\n");
}

static inline void CS_ALL_HIGH(void) {
    HAL_GPIO_WritePin(GPIOC, SLAVE_CS_0_Pin | SLAVE_CS_1_Pin | SLAVE_CS_2_Pin | SLAVE_CS_3_Pin, GPIO_PIN_SET);
}

static inline void CS_SELECT(SpiDevId dev) {
    CS_ALL_HIGH();
    switch (dev) {
        case DEV1: HAL_GPIO_WritePin(GPIOC, SLAVE_CS_0_Pin, GPIO_PIN_RESET); break;
        case DEV2: HAL_GPIO_WritePin(GPIOC, SLAVE_CS_1_Pin, GPIO_PIN_RESET); break;
        case DEV3: HAL_GPIO_WritePin(GPIOC, SLAVE_CS_2_Pin, GPIO_PIN_RESET); break;
        case DEV4: HAL_GPIO_WritePin(GPIOC, SLAVE_CS_3_Pin, GPIO_PIN_RESET); break;
        default: break;
    }
}
static void pack_one_motor(uint8_t motor_id, uint8_t *out, const MotorCmd *cmd) {
    out[0] = motor_id;
    uint16_t pos_u16 = (uint16_t)float_to_uint(cmd->position, P_MIN, P_MAX, 16);
    uint16_t spd_u16 = (uint16_t)float_to_uint(cmd->speed, V_MIN, V_MAX, 16);
    out[1] = (uint8_t)(pos_u16 & 0xFF);
    out[2] = (uint8_t)((pos_u16 >> 8) & 0xFF);
    out[3] = (uint8_t)(spd_u16 & 0xFF);
    out[4] = (uint8_t)((spd_u16 >> 8) & 0xFF);
}

static uint8_t unpack_one_motor_feedback(const uint8_t *rx, uint8_t *motor_id_out, float *position_out, float *speed_out) {
    if (!rx || !motor_id_out || !position_out || !speed_out) return 0U;
    *motor_id_out = rx[0];
    uint16_t pos_u16 = (uint16_t)rx[1] | ((uint16_t)rx[2] << 8);
    uint16_t spd_u16 = (uint16_t)rx[3] | ((uint16_t)rx[4] << 8);
    *position_out = uint_to_float((int)pos_u16, P_MIN, P_MAX, 16);
    *speed_out = uint_to_float((int)spd_u16, V_MIN, V_MAX, 16);
    return 1U;
}

static void uart_print_one_motor_encoding(uint16_t motor_index, const MotorCmd *cmd) {
    uint16_t pos_u16 = (uint16_t)float_to_uint(cmd->position, P_MIN, P_MAX, 16);
    uint16_t spd_u16 = (uint16_t)float_to_uint(cmd->speed, V_MIN, V_MAX, 16);
    uart_printf("[MOTOR %d] bytes: %02X %02X %02X %02X %02X\r\n", (int)motor_index,
                (uint8_t)motor_index, (uint8_t)(pos_u16 & 0xFF), (uint8_t)((pos_u16 >> 8) & 0xFF),
                (uint8_t)(spd_u16 & 0xFF), (uint8_t)((spd_u16 >> 8) & 0xFF));
}

static void uart_print_one_motor_rx(uint16_t motor_index, const uint8_t *rx) {
    uart_printf("[RX MOTOR %d] bytes: %02X %02X %02X %02X %02X\r\n", (int)motor_index, rx[0], rx[1], rx[2], rx[3], rx[4]);
}

static uint16_t build_slave_buf(uint8_t slave_index, uint8_t *tx_buf, const MotorCmd *all_motor_cmd) {
    uint16_t motor_count = g_master_config.routing.motor_count_per_slave[slave_index];
    for (uint16_t m = 0; m < motor_count; m++) {
        uint8_t motor_id = get_motor_id_by_route(slave_index, (uint8_t)m);
        if (motor_id < MAX_TOTAL_MOTORS) {
            pack_one_motor(motor_id, &tx_buf[m * BYTES_PER_MOTOR], &all_motor_cmd[motor_id]);
        } else {
            MotorCmd safe_cmd = {0.0f, 0.0f};
            pack_one_motor(0U, &tx_buf[m * BYTES_PER_MOTOR], &safe_cmd);
        }
    }
    return motor_count * BYTES_PER_MOTOR;
}

static uint8_t get_motor_id_by_route(uint8_t slave_index, uint8_t motor_index_in_slave) {
    if (slave_index >= NUM_SLAVES || motor_index_in_slave >= g_master_config.routing.motor_count_per_slave[slave_index]) {
        return MAX_TOTAL_MOTORS;
    }
    return g_master_config.routing.motor_ids[slave_index][motor_index_in_slave];
}

static uint8_t get_debug_test_motor_id(uint8_t test_slot) {
    if (test_slot >= g_master_config.debug.test_motor_count) return MAX_TOTAL_MOTORS;
    return g_master_config.debug.test_motor_ids[test_slot];
}

static uint8_t select_oldest_slave_for_poll(const uint8_t *polled, uint32_t *priority_tick) {
    uint8_t selected_slave = NUM_SLAVES;
    uint32_t selected_tick = 0U;
    uint8_t selected_has_priority = 0U;

    if (!priority_tick) return NUM_SLAVES;

    for (uint8_t slave_index = 0U; slave_index < NUM_SLAVES; slave_index++) {
        if (polled && polled[slave_index] != 0U) continue;
        uint8_t motor_count = g_master_config.routing.motor_count_per_slave[slave_index];
        if (motor_count == 0U) continue;

        uint32_t slave_tick = 0U;
        uint8_t slave_has_valid_snapshot = 0U;

        for (uint8_t m = 0U; m < motor_count; m++) {
            uint8_t motor_id = get_motor_id_by_route(slave_index, m);
            if (motor_id >= MAX_TOTAL_MOTORS) continue;

            const MotorCommSnapshot *snapshot = &g_master_safety.motor_comm_snapshot[motor_id];
            if (snapshot->valid == 0U) {
                slave_has_valid_snapshot = 0U;
                slave_tick = 0U;
                break;
            }
            if (slave_has_valid_snapshot == 0U || snapshot->tick < slave_tick) {
                slave_tick = snapshot->tick;
            }
            slave_has_valid_snapshot = 1U;
        }

        if (selected_slave == NUM_SLAVES) {
            selected_slave = slave_index;
            selected_tick = slave_tick;
            selected_has_priority = slave_has_valid_snapshot;
            continue;
        }
        if (selected_has_priority != 0U && slave_has_valid_snapshot == 0U) {
            selected_slave = slave_index;
            selected_tick = slave_tick;
            selected_has_priority = 0U;
            continue;
        }
        if (selected_has_priority == slave_has_valid_snapshot && slave_tick < selected_tick) {
            selected_slave = slave_index;
            selected_tick = slave_tick;
            selected_has_priority = slave_has_valid_snapshot;
        }
    }
    *priority_tick = selected_tick;
    return selected_slave;
}

static uint8_t find_motor_route_by_id(uint8_t motor_id, MotorRoute *route) {
    if (!route) return 0U;
    memset(route, 0, sizeof(*route));
    for (uint8_t slave_index = 0U; slave_index < NUM_SLAVES; slave_index++) {
        for (uint8_t m = 0U; m < g_master_config.routing.motor_count_per_slave[slave_index]; m++) {
            if (g_master_config.routing.motor_ids[slave_index][m] == motor_id) {
                route->found = 1U;
                route->slave_index = slave_index;
                route->dev = (SpiDevId)(DEV1 + slave_index);
                return 1U;
            }
        }
    }
    return 0U;
}

static uint8_t set_host_command_by_motor_id(uint8_t motor_id, float position, float speed) {
    MotorRoute route;
    if (motor_id >= MAX_TOTAL_MOTORS || find_motor_route_by_id(motor_id, &route) == 0U) return 0U;
    g_host_cmd[motor_id].position = clamp_float(position, P_MIN, P_MAX);
    g_host_cmd[motor_id].speed = clamp_float(speed, V_MIN, V_MAX);
    return 1U;
}

static void queue_host_command_by_motor_id(uint8_t motor_id, float position, float speed) {
    if (set_host_command_by_motor_id(motor_id, position, speed) != 0U) {
        g_master_safety.pending_command_mask |= (uint16_t)(1U << motor_id);
    }
}

static void update_motor_comm_snapshot_from_feedback(uint8_t motor_id, uint8_t slave_index, const uint8_t *rx) {
    if (motor_id >= MAX_TOTAL_MOTORS || !rx) return;

    uint8_t reported_motor_id = motor_id;
    float position = 0.0f;
    float speed = 0.0f;

    if (unpack_one_motor_feedback(rx, &reported_motor_id, &position, &speed) == 0U) return;
    if (reported_motor_id < MAX_TOTAL_MOTORS) motor_id = reported_motor_id;

    g_master_safety.motor_comm_snapshot[motor_id].valid = 1U;
    g_master_safety.motor_comm_snapshot[motor_id].motor_id = motor_id;
    g_master_safety.motor_comm_snapshot[motor_id].slave_index = slave_index;
    g_master_safety.motor_comm_snapshot[motor_id].position = position;
    g_master_safety.motor_comm_snapshot[motor_id].speed = speed;
    g_master_safety.motor_comm_snapshot[motor_id].tick = HAL_GetTick();
    g_master_safety.poll_start_slave_index = slave_index;
}

static HAL_StatusTypeDef spi_send_to_one_slave(SpiDevId dev, uint8_t slave_index, const MotorCmd *all_motor_cmd) {
    uint8_t tx_buf[MAX_TXRX_BYTES] = {0};
    uint8_t rx_buf[MAX_TXRX_BYTES] = {0};

    if (dev < DEV1 || dev > DEV4 || slave_index >= NUM_SLAVES) return HAL_ERROR;

    uint16_t tx_len = build_slave_buf(slave_index, tx_buf, all_motor_cmd);
    if (tx_len == 0) return HAL_OK;

    for (uint16_t m = 0; m < g_master_config.routing.motor_count_per_slave[slave_index]; m++) {
        uart_print_one_motor_encoding(get_motor_id_by_route(slave_index, (uint8_t)m), &all_motor_cmd[get_motor_id_by_route(slave_index, (uint8_t)m)]);
    }

    CS_SELECT(dev);
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(master_hspi, tx_buf, rx_buf, tx_len, HAL_MAX_DELAY);
    CS_ALL_HIGH();

    if (st == HAL_OK) {
        uart_dump_bytes("[SPI] RX:", rx_buf, tx_len);
        for (uint16_t m = 0; m < g_master_config.routing.motor_count_per_slave[slave_index]; m++) {
            uint8_t motor_id = get_motor_id_by_route(slave_index, (uint8_t)m);
            update_motor_comm_snapshot_from_feedback(motor_id, slave_index, &rx_buf[m * BYTES_PER_MOTOR]);
            uart_print_one_motor_rx(motor_id, &rx_buf[m * BYTES_PER_MOTOR]);
        }
    } else {
        uart_printf("[SPI] dev=%d transmit error\r\n", (int)dev);
    }

    return st;
}

static HAL_StatusTypeDef spi_update_all_slaves_param(const MotorCmd *all_motor_cmd) {
    HAL_StatusTypeDef st;
    uint8_t polled[NUM_SLAVES] = {0U};

    for (uint8_t step = 0U; step < NUM_SLAVES; step++) {
        uint32_t selected_tick = 0U;
        uint8_t slave_index = select_oldest_slave_for_poll(polled, &selected_tick);

        if (slave_index >= NUM_SLAVES) {
            g_master_safety.poll_resume_slave_index = g_master_safety.poll_start_slave_index;
            return HAL_OK;
        }
        if (g_master_safety.pending_command_mask != 0U) {
            g_master_safety.poll_resume_slave_index = slave_index;
            return HAL_BUSY;
        }

        st = spi_send_to_one_slave((SpiDevId)(DEV1 + slave_index), slave_index, all_motor_cmd);
        if (st != HAL_OK) {
            g_master_safety.poll_resume_slave_index = slave_index;
            return st;
        }

        polled[slave_index] = 1U;
        g_master_safety.poll_start_slave_index = slave_index;
    }

    g_master_safety.poll_resume_slave_index = g_master_safety.poll_start_slave_index;
    return HAL_OK;
}

static void process_pending_host_commands(void) {
    uint16_t pending_mask = g_master_safety.pending_command_mask;
    if (pending_mask == 0U) return;

    set_command_source(CMD_SOURCE_HOST);
    publish_command_cache(g_host_cmd);

    for (uint8_t slave_index = 0U; slave_index < NUM_SLAVES; slave_index++) {
        uint8_t motor_count = g_master_config.routing.motor_count_per_slave[slave_index];
        uint16_t slave_mask = 0U;

        for (uint8_t m = 0U; m < motor_count; m++) {
            uint8_t motor_id = get_motor_id_by_route(slave_index, m);
            if (motor_id < MAX_TOTAL_MOTORS) slave_mask |= (uint16_t)(1U << motor_id);
        }

        if ((pending_mask & slave_mask) != 0U) {
            if (spi_send_to_one_slave((SpiDevId)(DEV1 + slave_index), slave_index, g_master_safety.command_cache) == HAL_OK) {
                g_master_safety.pending_command_mask &= (uint16_t)~slave_mask;
            }
        }
    }
}

static void set_command_source(CommandSource source) {
    g_master_safety.active_command_source = (uint8_t)source;
}

static const MotorCmd* get_active_command_source(void) {
    if (g_master_safety.active_command_source == (uint8_t)CMD_SOURCE_HOST) return g_host_cmd;
    return g_test_cmd;
}

static void build_safe_stop_commands(MotorCmd *target) {
    for (uint16_t i = 0; i < MAX_TOTAL_MOTORS; i++) {
        target[i].position = 0.0f;
        target[i].speed = 0.0f;
    }
}

static void publish_command_cache(const MotorCmd *source) {
    for (uint16_t i = 0; i < MAX_TOTAL_MOTORS; i++) {
        g_master_safety.command_cache[i].position = clamp_float(source[i].position, P_MIN, P_MAX);
        g_master_safety.command_cache[i].speed = clamp_float(source[i].speed, V_MIN, V_MAX);
    }
    g_master_safety.command_cache_valid = 1U;
    g_master_safety.command_cache_tick = HAL_GetTick();
}

static void latch_emergency_stop(const char *reason) {
    if (!g_master_safety.estop_latched) uart_printf("[SAFETY] ESTOP latched: %s\r\n", reason);
    g_master_safety.estop_latched = 1U;
    g_master_safety.estop_latch_count++;
    g_master_safety.consecutive_spi_errors = 0U;
    g_master_safety.last_error_tick = HAL_GetTick();
    strncpy(g_master_safety.last_error_reason, reason, sizeof(g_master_safety.last_error_reason) - 1U);
    build_safe_stop_commands(g_master_safety.command_cache);
    g_master_safety.command_cache_valid = 1U;
    g_master_safety.command_cache_tick = HAL_GetTick();
}

static void run_debug_test_cycle(float *p_dir, uint32_t *p_phase_tick, uint8_t *p_phase, uint8_t *p_motor_index) {
    const uint32_t phase_period_ms = g_master_config.debug.phase_period_ms;
    const uint8_t test_motor_count = g_master_config.debug.test_motor_count;

    if (test_motor_count == 0U) {
        HAL_Delay(100);
        return;
    }

    if (HAL_GetTick() - *p_phase_tick >= phase_period_ms) {
        *p_phase_tick = HAL_GetTick();
        *p_phase = (uint8_t)((*p_phase + 1U) % 4U);
        *p_motor_index = (uint8_t)((*p_motor_index + 1U) % test_motor_count);
    }

    switch (*p_phase) {
        case 0:
            build_safe_stop_commands(g_test_cmd);
            break;
        case 1:
            build_safe_stop_commands(g_test_cmd);
            {
                uint8_t motor_id = get_debug_test_motor_id(*p_motor_index);
                if (motor_id < MAX_TOTAL_MOTORS) {
                    g_test_cmd[motor_id].position = g_master_config.debug.single_step_position * (*p_dir);
                    g_test_cmd[motor_id].speed = g_master_config.debug.single_step_speed;
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
            break;
        default:
            for (uint8_t i = 0; i < test_motor_count; i++) {
                uint8_t motor_id = get_debug_test_motor_id(i);
                if (motor_id < MAX_TOTAL_MOTORS) {
                    g_test_cmd[motor_id].position = g_master_config.debug.sweep_start_position + (g_master_config.debug.sweep_position_step * (float)i);
                    g_test_cmd[motor_id].speed = g_master_config.debug.sweep_speed_start + ((float)i * g_master_config.debug.sweep_speed_step);
                }
            }
            break;
    }

    uint8_t motor_id = get_debug_test_motor_id(*p_motor_index);
    if (motor_id < MAX_TOTAL_MOTORS) {
        if (g_test_cmd[motor_id].position >= g_master_config.debug.sweep_direction_limit ||
            g_test_cmd[motor_id].position <= -g_master_config.debug.sweep_direction_limit) {
            *p_dir = -*p_dir;
        }
    }

    set_command_source(CMD_SOURCE_TEST);
    publish_command_cache(g_test_cmd);
    spi_update_all_slaves_param(g_master_safety.command_cache);
    HAL_Delay(1);
}

static void run_real_master_cycle(void) {
    const uint32_t now = HAL_GetTick();

    process_pending_host_commands();

    if (!g_master_safety.command_cache_valid) publish_command_cache(get_active_command_source());

    if (g_master_safety.enabled != 0U) {
        if (g_master_safety.estop_latched != 0U) {
            build_safe_stop_commands(g_master_safety.command_cache);
        } else if ((now - g_master_safety.command_cache_tick) > g_master_safety.command_cache_timeout_ms) {
            latch_emergency_stop("command cache timeout");
        }
    }

    HAL_StatusTypeDef poll_status = spi_update_all_slaves_param(g_master_safety.command_cache);
    if (poll_status == HAL_BUSY) {
        process_pending_host_commands();
        g_master_safety.poll_start_slave_index = g_master_safety.poll_resume_slave_index;
        poll_status = spi_update_all_slaves_param(g_master_safety.command_cache);
    }

    if (poll_status == HAL_OK) {
        g_master_safety.tx_total_count++;
        g_master_safety.tx_success_count++;
        g_master_safety.last_success_tick = HAL_GetTick();
        g_master_safety.consecutive_spi_errors = 0U;
    } else if (poll_status != HAL_BUSY) {
        g_master_safety.tx_total_count++;
        g_master_safety.tx_fail_count++;
        g_master_safety.last_error_tick = HAL_GetTick();
        if (g_master_safety.consecutive_spi_errors < 255U) g_master_safety.consecutive_spi_errors++;
        if (g_master_safety.enabled != 0U && g_master_safety.consecutive_spi_errors >= g_master_safety.max_consecutive_spi_errors) {
            latch_emergency_stop("too many SPI errors");
        }
    }

    if (g_master_safety.pending_command_mask != 0U) {
        process_pending_host_commands();
        g_master_safety.poll_start_slave_index = g_master_safety.poll_resume_slave_index;
    }

    HAL_Delay(g_master_safety.control_loop_period_ms);
}

void MotorMaster_Init(SPI_HandleTypeDef *hspi, UART_HandleTypeDef *huart) {
    master_hspi = hspi;
    master_huart = huart;

    CS_ALL_HIGH();
    memset(g_host_cmd, 0, sizeof(g_host_cmd));

    g_master_safety.enabled = g_master_config.safety.enabled;
    g_master_safety.max_consecutive_spi_errors = g_master_config.safety.max_consecutive_spi_errors;
    g_master_safety.control_loop_period_ms = g_master_config.safety.control_loop_period_ms;
    g_master_safety.command_cache_timeout_ms = g_master_config.safety.command_cache_timeout_ms;

    set_command_source(CMD_SOURCE_TEST);
    publish_command_cache(get_active_command_source());

    phase_tick = HAL_GetTick();
}

void MotorMaster_SetMode(MasterMode_t mode) {
    g_master_mode = mode;
}

HAL_StatusTypeDef MotorMaster_SetCommand(uint8_t motor_id, float position, float speed) {
    MotorRoute route;
    if (motor_id >= MAX_TOTAL_MOTORS || find_motor_route_by_id(motor_id, &route) == 0U) return HAL_ERROR;
    queue_host_command_by_motor_id(motor_id, position, speed);
    return HAL_OK;
}

uint8_t MotorMaster_GetFeedback(uint8_t motor_id, MotorFeedback_t *feedback) {
    if (motor_id >= MAX_TOTAL_MOTORS || !feedback) return 0U;

    if (g_master_safety.motor_comm_snapshot[motor_id].valid) {
        feedback->valid = g_master_safety.motor_comm_snapshot[motor_id].valid;
        feedback->motor_id = g_master_safety.motor_comm_snapshot[motor_id].motor_id;
        feedback->position = g_master_safety.motor_comm_snapshot[motor_id].position;
        feedback->speed = g_master_safety.motor_comm_snapshot[motor_id].speed;
        feedback->tick = g_master_safety.motor_comm_snapshot[motor_id].tick;
        return 1U;
    }
    return 0U;
}

void MotorMaster_ClearEStop(void) {
    g_master_safety.estop_latched = 0U;
    g_master_safety.consecutive_spi_errors = 0U;
    g_master_safety.command_cache_valid = 0U;
    uart_printf("[SAFETY] ESTOP manually cleared\r\n");
}

void MotorMaster_ProcessLoop(void) {
    if (g_master_mode == MASTER_MODE_DEBUG) {
        run_debug_test_cycle(&dir, &phase_tick, &test_phase, &test_motor_index);
    } else {
        set_command_source(CMD_SOURCE_HOST);
        run_real_master_cycle();
    }
}


