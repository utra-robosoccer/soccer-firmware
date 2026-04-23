/*
 * spi_master.c
 * Simplified SPI Master for 1-to-1 (Expandable to 1-to-Multi)
 */

#include "spi_master.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

static SPI_HandleTypeDef *master_hspi = NULL;
static UART_HandleTypeDef *master_huart = NULL;

// Buffers for the current 1-to-1 test (1 Slave, up to MAX_MOTORS_PER_SLAVE)
static MotorCmd g_motor_cmds[MAX_MOTORS_PER_SLAVE] = {0};
static MotorFeedback g_motor_feedbacks[MAX_MOTORS_PER_SLAVE] = {0};
static uint8_t g_active_motor_count = 4; // Testing with 3 motors on DEV1
uint8_t motorID_lut[MAX_MOTORS_PER_SLAVE] = {3,2,5,4};

// Sweep test variables
static float sweep_pos = 0.0f;
static float sweep_dir = 1.0f;

/* --- Helpers: Math --- */

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

/* --- Helpers: UART Debug --- */

static void uart_printf(const char *fmt, ...) {
    if (!master_huart) return;
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

void usb_printf(const char *fmt, ...)
{
    // Buffer to hold the formatted string
    char buf[256];
    va_list ap;

    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    // Prevent buffer overflow issues
    if (n < 0) return;
    if (n >= (int)sizeof(buf)) n = sizeof(buf) - 1;

    // Send the data over USB CDC
    // CDC_Transmit_FS expects a uint8_t pointer and the length
    CDC_Transmit_FS((uint8_t*)buf, (uint16_t)n);
}

/* --- Helpers: SPI Hardware --- */

static inline void CS_ALL_HIGH(void) {
    // Assuming pins are defined in main.h matching your setup
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

/* --- Helpers: Frame Packing/Unpacking --- */

static void pack_one_motor(uint8_t *out, const MotorCmd *cmd) {
    out[0] = cmd->motor_id;
    uint16_t pos_u16 = (uint16_t)float_to_uint(cmd->position, P_MIN, P_MAX, 16);
    uint16_t spd_u16 = (uint16_t)float_to_uint(cmd->speed, V_MIN, V_MAX, 16);
    out[1] = (uint8_t)(pos_u16 & 0xFF);
    out[2] = (uint8_t)((pos_u16 >> 8) & 0xFF);
    out[3] = (uint8_t)(spd_u16 & 0xFF);
    out[4] = (uint8_t)((spd_u16 >> 8) & 0xFF);
}

static void unpack_one_motor_feedback(const uint8_t *rx, MotorFeedback *fb) {
    fb->motor_id = rx[0];
    uint16_t pos_u16 = (uint16_t)rx[1] | ((uint16_t)rx[2] << 8);
    uint16_t spd_u16 = (uint16_t)rx[3] | ((uint16_t)rx[4] << 8);
    fb->position = uint_to_float((int)pos_u16, P_MIN, P_MAX, 16);
    fb->speed = uint_to_float((int)spd_u16, V_MIN, V_MAX, 16);
    fb->valid = 1;
}

/* --- Core SPI Transfer --- */

/**
 * @brief Transmits commands to a single slave and reads back feedback.
 * To expand to 1-to-Multi slaves, simply call this function sequentially
 * for DEV1, DEV2, etc., passing the appropriate subsets of motor commands.
 */
static HAL_StatusTypeDef spi_send_to_one_slave(SpiDevId dev, uint8_t num_motors, MotorCmd *cmds, MotorFeedback *feedbacks) {
    uint8_t tx_buf[MAX_MOTORS_PER_SLAVE * BYTES_PER_MOTOR] = {0};
    uint8_t rx_buf[MAX_MOTORS_PER_SLAVE * BYTES_PER_MOTOR] = {0};

    if (num_motors > MAX_MOTORS_PER_SLAVE || num_motors == 0) return HAL_ERROR;

    // Pack
    for (uint8_t i = 0; i < num_motors; i++) {
        pack_one_motor(&tx_buf[i * BYTES_PER_MOTOR], &cmds[i]);
    }

    // Transmit
    CS_SELECT(dev);
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(master_hspi, tx_buf, rx_buf, num_motors * BYTES_PER_MOTOR, HAL_MAX_DELAY);
    CS_ALL_HIGH();

    // Unpack
    if (st == HAL_OK) {
        for (uint8_t i = 0; i < num_motors; i++) {
            unpack_one_motor_feedback(&rx_buf[i * BYTES_PER_MOTOR], &feedbacks[i]);
        }
    } else {
        usb_printf("[SPI] DEV%d Transmit Error\r\n", (int)dev);
    }

    return st;
}

/* --- Public API --- */

void MotorMaster_Init(SPI_HandleTypeDef *hspi, UART_HandleTypeDef *huart) {
    master_hspi = hspi;
    master_huart = huart;
    CS_ALL_HIGH();

    // Init test setup
    for (uint8_t i = 0; i < g_active_motor_count; i++) {
        g_motor_cmds[i].motor_id = motorID_lut[i];
        g_motor_cmds[i].position = 0.0f;
        g_motor_cmds[i].speed = 1.0f;
    }

    usb_printf("\r\n=== SPI Master Initialized ===\r\n");
}

void MotorMaster_SetCommand(uint8_t motor_index, uint8_t motor_id, float position, float speed) {
    if (motor_index < MAX_MOTORS_PER_SLAVE) {
        g_motor_cmds[motor_index].motor_id = motor_id;
        g_motor_cmds[motor_index].position = position;
        g_motor_cmds[motor_index].speed = speed;
    }
}

uint8_t MotorMaster_GetFeedback(uint8_t motor_index, MotorFeedback *feedback) {
    if (motor_index < MAX_MOTORS_PER_SLAVE && g_motor_feedbacks[motor_index].valid) {
        *feedback = g_motor_feedbacks[motor_index];
        return 1;
    }
    return 0;
}

void MotorMaster_ProcessLoop(void) {
    // 1. Update Sweep Test logic
    sweep_pos += 0.1f * sweep_dir;
    if (sweep_pos >= 10.0f || sweep_pos <= -10.0f) {
        sweep_dir *= -1.0f;
    }

    // Apply sweep to all active motors in this test
    for (uint8_t i = 0; i < g_active_motor_count; i++) {
        g_motor_cmds[i].position = sweep_pos;
        g_motor_cmds[i].speed = 2.5f;
    }

    // 2. Execute SPI Transfer for DEV1
    HAL_StatusTypeDef st = spi_send_to_one_slave(DEV1, g_active_motor_count, g_motor_cmds, g_motor_feedbacks);

    // 3. Optional Debugging
    if (st == HAL_OK) {
        usb_printf("[CMD] Pos: %.2f | [FB] ID<%d>: %.2f, ID<%d>: %.2f, ID<%d>: %.2f\r\n",
                    sweep_pos,
					g_motor_feedbacks[0].motor_id,
                    g_motor_feedbacks[0].position,
					g_motor_feedbacks[1].motor_id,
                    g_motor_feedbacks[1].position,
					g_motor_feedbacks[2].motor_id,
                    g_motor_feedbacks[2].position);
    }

    HAL_Delay(1);
}
