/*
 * slave_spi.h
 *
 *  Created on: 2026年1月21日
 *      Author: 18701
 */

#ifndef INC_SLAVE_SPI_H_
#define INC_SLAVE_SPI_H_

#include "stm32f4xx_hal.h"
#include "main.h"

#define PAYLOAD_LENGTH 8
#define BUFFER_SIZE 32

extern uint8_t* volatile  motor_update_buf; //This belongs to the Rx side
extern uint8_t* volatile  motor_tele_buf; //This belongs to the Tx side
extern volatile uint8_t data_receive_flag;
extern volatile uint8_t data_tx_ready_flag;
extern volatile uint8_t spi_error_flag;

extern uint8_t random_count; // just for spi dbg


void spi_dma_init(SPI_HandleTypeDef *hspi);
void spi_write_next_tx_buf(const uint8_t* motor_new_data_buf, uint8_t* motor_tele_buf);




#endif /* INC_SLAVE_SPI_H_ */
