/*
 * spi_master.c
 */

#include "spi_master.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

static SPI_HandleTypeDef *master_hspi = NULL;
static UART_HandleTypeDef *master_huart = NULL;

spi_dev_t slave_devices[4] = {
    { .spi_device_idx = DEV1, .active_motor_count = 4, .spi_motor_ids = {3, 2, 5, 4} },
    { .spi_device_idx = DEV2, .active_motor_count = 4, .spi_motor_ids = {11, 12, 13, 14} },
    { .spi_device_idx = DEV3, .active_motor_count = 4, .spi_motor_ids = {21, 22, 23, 24} },
    { .spi_device_idx = DEV4, .active_motor_count = 4, .spi_motor_ids = {31, 32, 33, 34} }
};
static uint8_t dma_tx_buf[MAX_MOTORS_PER_SLAVE * BYTES_PER_MOTOR];
static uint8_t dma_rx_buf[MAX_MOTORS_PER_SLAVE * BYTES_PER_MOTOR];

uint8_t usb_buffer[256];
volatile uint32_t usb_buf_len = 0;
volatile uint8_t usb_rx_flag = 0;

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

void usb_dump_bytes(const char *tag, const uint8_t *buf, int n)
{
    usb_printf("%s", tag);
    for (int i = 0; i < n; i++) {
        usb_printf(" %02X", buf[i]);
    }
    usb_printf("\r\n");
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
 * call this function sequentially
 * for DEV1, DEV2, etc., passing the appropriate subsets of motor commands.
 */
static HAL_StatusTypeDef spi_send_to_one_slave(spi_dev_t *slave)
{
    if (slave->active_motor_count > MAX_MOTORS_PER_SLAVE || slave->active_motor_count == 0) {
        return HAL_ERROR;
    }

    for (uint8_t i = 0; i < slave->active_motor_count; i++) {
        pack_one_motor(&dma_tx_buf[i * BYTES_PER_MOTOR], &slave->motor_cmds[i]);
    }

    CS_SELECT(slave->spi_device_idx);

    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive_DMA(master_hspi, dma_tx_buf, dma_rx_buf, slave->active_motor_count * BYTES_PER_MOTOR);

    while (HAL_SPI_GetState(master_hspi) != HAL_SPI_STATE_READY);

    CS_ALL_HIGH();

    if (st == HAL_OK) {
        for (uint8_t i = 0; i < slave->active_motor_count; i++) {
            unpack_one_motor_feedback(&dma_rx_buf[i * BYTES_PER_MOTOR], &slave->motor_feedbacks[i]);
        }
    } else {
//        usb_printf("[SPI] DEV%d Transmit Error\r\n", (int)slave->spi_device_idx);
    }

    return st;
}

/* --- Public API --- */

void MotorMaster_Init(SPI_HandleTypeDef *hspi, UART_HandleTypeDef *huart)
{
    master_hspi = hspi;
    master_huart = huart;
    CS_ALL_HIGH();

    for (uint8_t s = 0; s < 4; s++) {
        for (uint8_t m = 0; m < slave_devices[s].active_motor_count; m++) {
            uint8_t current_id = slave_devices[s].spi_motor_ids[m];

            slave_devices[s].motor_cmds[m].motor_id = current_id;
            slave_devices[s].motor_cmds[m].position = 0.0f;
            slave_devices[s].motor_cmds[m].speed = 0.0f;

            slave_devices[s].motor_feedbacks[m].motor_id = 0;
            slave_devices[s].motor_feedbacks[m].position = 0.0f;
            slave_devices[s].motor_feedbacks[m].speed = 0.0f;
            slave_devices[s].motor_feedbacks[m].valid = 0;
        }
    }
}
// set command and get feedback need rework, skip packing incoming usb data into customized struct
// directly packing using the motor buffer
void MotorMaster_SetCommand(SpiDevId dev, uint8_t motor_index, float position, float speed)
{
    uint8_t slave_idx = (uint8_t)dev - 1;
    if (slave_idx < 4 && motor_index < slave_devices[slave_idx].active_motor_count) {
        slave_devices[slave_idx].motor_cmds[motor_index].position = position;
        slave_devices[slave_idx].motor_cmds[motor_index].speed = speed;
    }
}

uint8_t MotorMaster_GetFeedback(SpiDevId dev, uint8_t motor_index, MotorFeedback *feedback)
{
    uint8_t slave_idx = (uint8_t)dev - 1;
    if (slave_idx < 4 && motor_index < slave_devices[slave_idx].active_motor_count) {
        if (slave_devices[slave_idx].motor_feedbacks[motor_index].valid) {
            *feedback = slave_devices[slave_idx].motor_feedbacks[motor_index];
            return 1;
        }
    }
    return 0;
}

void MotorMaster_ProcessLoop(void)
{
    sweep_pos += 0.1f * sweep_dir;
    if (sweep_pos >= 10.0f || sweep_pos <= -10.0f) {
        sweep_dir *= -1.0f;
    }

    for (uint8_t s = 0; s < 4; s++) {
        spi_dev_t *current_slave = &slave_devices[s];

        for (uint8_t i = 0; i < current_slave->active_motor_count; i++) {
            current_slave->motor_cmds[i].position = sweep_pos;
            current_slave->motor_cmds[i].speed = 2.5f;
        }

        HAL_StatusTypeDef st = spi_send_to_one_slave(current_slave);

        if (st == HAL_OK && current_slave->spi_device_idx == DEV1) {
            usb_printf("[DEV1] CMD: %.2f | FB0<%d>: %.2f | FB1<%d>: %.2f\r\n",
                        sweep_pos,
                        current_slave->motor_feedbacks[0].motor_id,
                        current_slave->motor_feedbacks[0].position,
                        current_slave->motor_feedbacks[1].motor_id,
                        current_slave->motor_feedbacks[1].position);
        }
    }

    HAL_Delay(1);
}
