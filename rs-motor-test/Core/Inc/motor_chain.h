/*
 * motor_chain.h
 *
 *  Created on: Jan 4, 2026
 *
 */

#ifndef INC_MOTOR_CHAIN_H_
#define INC_MOTOR_CHAIN_H_

#include <stdint.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_can.h"
#include "robostride_test.h"
#include "main.h"

#define MAX_MOTOR_COUNT 10 //This macro changes depending on the total number of motors connected to the CAN BUS
extern motor_t motors[MAX_MOTOR_COUNT];

HAL_StatusTypeDef motor_chain_init();
HAL_StatusTypeDef motor_set_spd (uint8_t target_id, float spd, float pd);


#endif /* INC_MOTOR_CHAIN_H_ */
