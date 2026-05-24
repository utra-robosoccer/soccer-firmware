#include "spi_master.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

static SPI_HandleTypeDef *master_hspi = NULL;
static UART_HandleTypeDef *master_huart = NULL;

uint8_t new_usb_packet_rx_flag = 0;

uint8_t buf_rx_jet2master[NUM_SLV * MAX_MOTORS_PER_SLAVE * USB_BYTES_PER_MOTOR];
uint8_t buf_tx_master2jet[NUM_SLV * MAX_MOTORS_PER_SLAVE * USB_BYTES_PER_MOTOR];

motor_cmd_t motor_cmds[MAX_MOTORS_PER_SLAVE * NUM_SLV];
motor_cmd_t motor_feedbacks[MAX_MOTORS_PER_SLAVE * NUM_SLV];

uint8_t motorID_lut[NUM_SLV][MAX_MOTORS_PER_SLAVE] = {
    {1,2},
//    {5, 6, 7, 8, 0},
//    {9, 10, 11, 12, 0},
//    {13, 14, 15, 16, 0}
};

slv_motor_chain_t slaves[NUM_SLV];

/* --- Helpers --- */
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

void usb_printf(const char *fmt, ...) {
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (n < 0) return;
    if (n >= (int)sizeof(buf)) n = sizeof(buf) - 1;
    CDC_Transmit_FS((uint8_t*)buf, (uint16_t)n);
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

static void pack_one_motor(uint8_t *out, const motor_cmd_t *cmd) {
    out[0] = cmd->motor_id;
    uint16_t pos_u16 = (uint16_t)float_to_uint(cmd->position, P_MIN, P_MAX, 16);
    uint16_t spd_u16 = (uint16_t)float_to_uint(cmd->speed, V_MIN, V_MAX, 16);
    out[1] = (uint8_t)(pos_u16 & 0xFF);
    out[2] = (uint8_t)((pos_u16 >> 8) & 0xFF);
    out[3] = (uint8_t)(spd_u16 & 0xFF);
    out[4] = (uint8_t)((spd_u16 >> 8) & 0xFF);
}

static void unpack_one_motor_feedback(const uint8_t *rx, motor_cmd_t *fb) {
    fb->motor_id = rx[0];
    uint16_t pos_u16 = (uint16_t)rx[1] | ((uint16_t)rx[2] << 8);
    uint16_t spd_u16 = (uint16_t)rx[3] | ((uint16_t)rx[4] << 8);
    fb->position = uint_to_float((int)pos_u16, P_MIN, P_MAX, 16);
    fb->speed = uint_to_float((int)spd_u16, V_MIN, V_MAX, 16);
    // valid flag removed
}

/* --- Core SPI Transfer --- */
static HAL_StatusTypeDef spi_send_to_one_slave(SpiDevId dev, uint8_t num_motors, motor_cmd_t *cmds, motor_cmd_t *feedbacks) {
    uint8_t tx_buf[MAX_MOTORS_PER_SLAVE * BYTES_PER_MOTOR] = {0};
    uint8_t rx_buf[MAX_MOTORS_PER_SLAVE * BYTES_PER_MOTOR] = {0};

    if (num_motors > MAX_MOTORS_PER_SLAVE || num_motors == 0) return HAL_ERROR;

    for (uint8_t i = 0; i < num_motors; i++) {
        pack_one_motor(&tx_buf[i * BYTES_PER_MOTOR], &cmds[i]);
    }

    CS_SELECT(dev);
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(master_hspi, tx_buf, rx_buf, num_motors * BYTES_PER_MOTOR, HAL_MAX_DELAY);
    CS_ALL_HIGH();

    if (st == HAL_OK) {
        for (uint8_t i = 0; i < num_motors; i++) {
            unpack_one_motor_feedback(&rx_buf[i * BYTES_PER_MOTOR], &feedbacks[i]);
        }
    }

    return st;
}

/* --- Public API --- */
void MotorMaster_Init(SPI_HandleTypeDef *hspi, UART_HandleTypeDef *huart) {
    master_hspi = hspi;
    master_huart = huart;
    CS_ALL_HIGH();

    for (uint8_t i = 0; i < NUM_SLV; i++) {
        slaves[i].slv_id = (SpiDevId)i;
        slaves[i].slv_motor_cmds = &motor_cmds[i * MAX_MOTORS_PER_SLAVE];
        slaves[i].slv_motor_feedbacks = &motor_feedbacks[i * MAX_MOTORS_PER_SLAVE];
        slaves[i].active_motor_count = 2;
        slaves[i].motorID_lut = motorID_lut[i];

        for (uint8_t j = 0; j < slaves[i].active_motor_count; j++) {
            slaves[i].slv_motor_cmds[j].motor_id = slaves[i].motorID_lut[j];
            slaves[i].slv_motor_cmds[j].position = 0.0f;
            slaves[i].slv_motor_cmds[j].speed = 1.0f;
        }
    }
}

void MotorMaster_ParseRxBuffer(void) {
    // 极其暴力的零拷贝 cast：直接把 USB 接收到的 byte buffer 当作 struct 数组来用
    motor_cmd_t* rx_cmds = (motor_cmd_t*)buf_rx_jet2master;

    for (int i = 0; i < NUM_SLV * MAX_MOTORS_PER_SLAVE; i++) {
        motor_cmds[i] = rx_cmds[i]; // 直接结构体赋值
    }
}

void MotorMaster_FormatTxBuffer(void) {
    motor_cmd_t* tx_cmds = (motor_cmd_t*)buf_tx_master2jet;

    for (int i = 0; i < NUM_SLV * MAX_MOTORS_PER_SLAVE; i++) {
        tx_cmds[i] = motor_feedbacks[i];
    }
}

void MotorMaster_ProcessLoop(void) {
    if (new_usb_packet_rx_flag) {

        MotorMaster_ParseRxBuffer();

        for (uint8_t i = 0; i < NUM_SLV; i++) {
            spi_send_to_one_slave(slaves[i].slv_id,
                                  slaves[i].active_motor_count,
                                  slaves[i].slv_motor_cmds,
                                  slaves[i].slv_motor_feedbacks);
        }

        MotorMaster_FormatTxBuffer();
        CDC_Transmit_FS(buf_tx_master2jet, sizeof(buf_tx_master2jet));

        new_usb_packet_rx_flag = 0;
    }
}
