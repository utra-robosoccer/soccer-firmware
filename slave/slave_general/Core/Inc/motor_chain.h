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

#define MAX_MOTOR_COUNT 4 //This macro changes depending on the total number of motors connected to the CAN BUS
extern motor_t motors[MAX_MOTOR_COUNT];
extern uint8_t can_rx_flag;
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
