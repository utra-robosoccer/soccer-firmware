/*
 * motor_chain.h
 *
 *  Created on: Jan 4, 2026
 *
 */

#ifndef INC_MOTOR_CHAIN_H_
#define INC_MOTOR_CHAIN_H_

#include <robostride.h>
#include <stdint.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_can.h"
#include "main.h"
#include "proto_common.h"   /* N_MOTORS, motor_configs[] (generated) */

#define MAX_MOTOR_COUNT N_MOTORS //Total motors on the CAN bus — driven by the generated motor_config.h
extern motor_t motors[MAX_MOTOR_COUNT];
extern volatile uint8_t can_rx_flag;
extern volatile uint32_t can_raw_rx_count;
extern volatile uint32_t can_feedback_count;
extern volatile uint8_t can_feedback_motor_id;
extern volatile uint32_t can_type0_count;
extern volatile uint32_t can_type2_count;
extern volatile uint32_t can_type17_count;
extern volatile uint32_t can_unknown_type_count;
extern volatile uint32_t can_no_lut_count;
extern volatile uint32_t can_unpack_error_count;
extern volatile uint32_t can_last_ext_id;
extern volatile uint32_t can_last_rx_word0;
extern volatile uint32_t can_last_rx_word1;
extern volatile uint8_t can_last_type;
extern volatile uint8_t can_last_sender_id;
extern volatile uint8_t can_last_dlc;
extern volatile uint8_t can_last_ide;
extern uint8_t rx_data[8];

//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
motor_t* get_motor_by_id(uint8_t id);
HAL_StatusTypeDef motor_chain_init(void);
HAL_StatusTypeDef motor_chain_init_dbg(UART_HandleTypeDef *huart);

HAL_StatusTypeDef motor_chain_go_zeropos(void);
HAL_StatusTypeDef motor_chain_go_zeropos_dbg(UART_HandleTypeDef *huart);

HAL_StatusTypeDef motor_set_spd(uint8_t target_id, float spd, float pd);
HAL_StatusTypeDef motor_set_spd_dbg(uint8_t target_id, float spd, float pd, UART_HandleTypeDef *huart);

HAL_StatusTypeDef motor_set_mit(uint8_t target_id, float torq, float pos, float spd, float kp, float kd);
HAL_StatusTypeDef motor_set_mit_dbg(uint8_t target_id, float torq, float pos, float spd, float kp, float kd, UART_HandleTypeDef *huart);

HAL_StatusTypeDef motor_check_angle(motor_t *motor, float pos, float spd, float pd);
HAL_StatusTypeDef motor_check_angle_dbg(motor_t *motor, float pos, float spd, float pd, UART_HandleTypeDef *huart);



#endif /* INC_MOTOR_CHAIN_H_ */
