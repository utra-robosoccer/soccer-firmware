#include "motor_chain.h"

motor_t motors[MAX_MOTOR_COUNT]; //This is the motor chain array

// For FIFO Intr Callback function
uint8_t can_rx_flag = 0;
uint8_t rx_data[8];

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rs_can_rx_header, rx_data);

    exCanIdInfo *rx_id_info = (exCanIdInfo *)&rs_can_rx_header.ExtId;

    uint8_t comm_type = rx_id_info->mode; // Bits 24-28
    uint8_t sender_id = 0;

    sender_id = (uint8_t)(rx_id_info->data & 0xFF);

    if (sender_id > 0 && sender_id <= MAX_MOTOR_COUNT)
    {
        motor_t *target_motor = &motors[sender_id - 1]; // Map ID 1 -> Index 0

        switch (comm_type)
        {
            case 2: // Motor Feedback
                can_unpack_motor_feedback(target_motor, rx_data);
                break;

            case 17: // Single Parameter Read
            {
                float param_value = 0.0f;
                // Unpack the float value from the buffer
                if (can_unpack_single_param(rx_data, &param_value) == HAL_OK) {

                }
                break;
            }

            case 0: // Get ID Response
                can_unpack_get_id(target_motor, rx_data);
                break;

            default:
                // Handle unknown types or other responses (e.g. Type 1 response is Type 2)
                break;
        }

        memcpy(motor_tele_buf, rx_data, sizeof(rx_data));
        data_tx_ready_flag = 1;
    }
    can_rx_flag = 1;

}

// ---------------------------------------------------------
// Print Function Helpers
// ---------------------------------------------------------
static void get_fault_string(motor_error_t errors, char* buffer) {
    if (errors.undervoltage) sprintf(buffer, "UnderVolt");
    else if (errors.driver_fault) sprintf(buffer, "DriverFault");
    else if (errors.overheat) sprintf(buffer, "OverHeat");
    else if (errors.encoder_fault) sprintf(buffer, "EncoderFault");
    else if (errors.stall_overload) sprintf(buffer, "Stall");
    else if (errors.uncalibrated) sprintf(buffer, "Uncalibrated");
    else sprintf(buffer, "None");
}

static void get_mode_string(motor_status_t status, char* buffer) {
    switch (status) {
        case RS_MODE_RESET:  sprintf(buffer, "RESET"); break;
        case RS_MODE_CALI:   sprintf(buffer, "CALI"); break;
        case RS_MODE_NORMAL: sprintf(buffer, "MOTOR"); break;
        default:             sprintf(buffer, "UNKNOWN"); break;
    }
}

// ---------------------------------------------------------
// Unified Print Function
// ---------------------------------------------------------
static void print_unified_can_response(UART_HandleTypeDef *huart, uint8_t type, motor_t *m, uint8_t *rx_data, uint32_t ext_id) {
    char uart_buf[256];
    int len = 0;

    switch (type) {
        // --- Type 0: Get ID Response ---
        case 0: {
            uint8_t received_id = (ext_id >> 8) & 0xFF; // Extract Motor ID from ExtID Bits 8-15

            uint64_t uid = 0;
            for(int i=0; i<8; i++) uid |= ((uint64_t)rx_data[i] << (8*i));
            uint32_t uid_high = (uint32_t)((uid >> 32) & 0xFFFFFFFF);
            uint32_t uid_low  = (uint32_t)(uid & 0xFFFFFFFF);

            len = sprintf(uart_buf, "[Type 0 ID] MotorID: %d, UID: 0x%08lX%08lX\r\n",
                          received_id, uid_high, uid_low);
            break;
        }

        // --- Type 2: Feedback Frame ---
        case 2: {
            char mode_str[10];
            char fault_str[20];
            get_mode_string(m->status, mode_str);
            get_fault_string(m->motor_errors, fault_str);

            len = sprintf(uart_buf, "Motor: %d Pos: %.4f Spd: %.4f Torq: %.4f Mode: %s Temp: %.4f Fault: %s\r\n",
                m->id, m->pos, m->rpm, m->torq, mode_str, m->temperature, fault_str);
            break;
        }

        // --- Type 17: Read Single Parameter Response ---
        case 17: {
            uint16_t index = rx_data[0] | (rx_data[1] << 8);
            float value = 0.0f;
            uint32_t raw_val = 0;

            raw_val |= rx_data[4];
            raw_val |= (uint32_t)rx_data[5] << 8;
            raw_val |= (uint32_t)rx_data[6] << 16;
            raw_val |= (uint32_t)rx_data[7] << 24;
            memcpy(&value, &raw_val, sizeof(float));

            len = sprintf(uart_buf, "[Type 17 Param] ID: %d Index: 0x%04X Value: %.4f\r\n",
                          m->id, index, value);
            break;
        }
        default: break;
    }

    if (len > 0) {
        HAL_UART_Transmit(huart, (uint8_t*)uart_buf, len, 100);
    }
}

// ---------------------------------------------------------
// Motor Chain Control functions
// ---------------------------------------------------------

HAL_StatusTypeDef motor_chain_init()
{
	uint32_t tickstart;
	for (int target_id = 1; target_id <= MAX_MOTOR_COUNT; target_id++){
		can_rx_flag = 0;
		if (can_get_motor_id(target_id, CAN_MASTER_ID) == HAL_OK) {
		        tickstart = HAL_GetTick();
		        while (can_rx_flag == 0) {
		            if ((HAL_GetTick() - tickstart) > 100) {
		                HAL_UART_Transmit(&huart2, (uint8_t*)"Timeout: Get ID\r\n", 17, 100);
		                break;
		            }
		        }
		        if (can_rx_flag) {
		            print_unified_can_response(&huart2, 0, &motors[target_id - 1], rx_data, rs_can_rx_header.ExtId);
		        }
		    } else {return HAL_ERROR;}
		 HAL_Delay(50);

		 //Set Motor Mode to MIT
		 can_rx_flag = 0;
		 if (can_change_motor_mode(target_id, CAN_MASTER_ID, MIT_MODE) == HAL_OK) {
			 tickstart = HAL_GetTick();
			 while (can_rx_flag == 0) {
				 if ((HAL_GetTick() - tickstart) > 100) {
					 HAL_UART_Transmit(&huart2, (uint8_t*)"Timeout: Set Mode\r\n", 19, 100);
					 break;
				 }
			 }
			 if (can_rx_flag) {
				 HAL_UART_Transmit(&huart2, (uint8_t*)"Set Mode\r\n", 10, 100);
				 print_unified_can_response(&huart2, 2, &motors[target_id - 1], rx_data, rs_can_rx_header.ExtId);
			 }
		 } else {return HAL_ERROR;}
		 HAL_Delay(50);

		 //Enable Motor
		 can_rx_flag = 0;
		 if (can_enable_motor(target_id, CAN_MASTER_ID) == HAL_OK) {
			 tickstart = HAL_GetTick();
			 while (can_rx_flag == 0) {
				 if ((HAL_GetTick() - tickstart) > 100) {
					 HAL_UART_Transmit(&huart2, (uint8_t*)"Timeout: Enable Motor\r\n", 23, 100);
					 break;
				 }
			 }
			 if (can_rx_flag) {
				 HAL_UART_Transmit(&huart2, (uint8_t*)"Enable\r\n", 8, 100);
				 print_unified_can_response(&huart2, 2, &motors[target_id - 1], rx_data, rs_can_rx_header.ExtId);
			 }
		 } else {return HAL_ERROR;}
		 HAL_Delay(50);

	}

	return HAL_OK;
}

HAL_StatusTypeDef motor_chain_go_zeropos ()
{
	uint32_t tickstart;
	can_rx_flag = 0;

	float direction;

	for (int target_id = 1; target_id < MAX_MOTOR_COUNT; target_id ++){
		if(motors[target_id - 1].pos > 0){
			direction = -1;
		}
		else if (motors[target_id - 1].pos < 0){
			direction = 1;
		} else {continue;} //motor is @zero pos

		if (can_mit_control_set(target_id, 0.0f, 0.0f, 5.0 * direction, 100.0f, 5.0f) == HAL_OK) {
			tickstart = HAL_GetTick();
			while (can_rx_flag == 0) {
				if ((HAL_GetTick() - tickstart) > 100) {
					return HAL_TIMEOUT;
				}
			}
		} else {return HAL_ERROR;}
	}

	HAL_Delay(500); //Letting all the motors react
	return HAL_OK;
}

HAL_StatusTypeDef motor_chain_go_zeropos_dbg ()
{
	uint32_t tickstart;
	can_rx_flag = 0;

	float direction;

	for (int target_id = 1; target_id < MAX_MOTOR_COUNT; target_id ++){
		if(motors[target_id - 1].pos > 0){
			direction = -1;
		}
		else if (motors[target_id - 1].pos < 0){
			direction = 1;
		} else {continue;} //motor is @zero pos

		if (can_mit_control_set(target_id, 0.0f, 0.0f, 5.0 * direction, 100.0f, 5.0f) == HAL_OK) {
			tickstart = HAL_GetTick();
			while (can_rx_flag == 0) {
				if ((HAL_GetTick() - tickstart) > 100) {
					return HAL_TIMEOUT;
				}
			}
		} else {return HAL_ERROR;}

		if (can_rx_flag) {
			print_unified_can_response(&huart2, 2, &motors[target_id - 1], rx_data, rs_can_rx_header.ExtId);
		}
	}

	HAL_Delay(500); //Letting all the motors react
	return HAL_OK;
}

HAL_StatusTypeDef motor_set_spd (uint8_t target_id, float spd, float pd)
{
	uint32_t tickstart;
	can_rx_flag = 0;
	if (can_mit_control_set(target_id, 0.0f, 0.0f, spd, 0.0f, pd) == HAL_OK) {

		tickstart = HAL_GetTick();
		while (can_rx_flag == 0) {
			if ((HAL_GetTick() - tickstart) > 100) {
				return HAL_TIMEOUT;
			}
		}
	} else {return HAL_ERROR;}

	return HAL_OK;
}

HAL_StatusTypeDef motor_set_spd_dbg (uint8_t target_id, float spd, float pd)
{
	uint32_t tickstart;
	can_rx_flag = 0;
	if (can_mit_control_set(target_id, 0.0f, 0.0f, spd, 0.0f, pd) == HAL_OK) {

		tickstart = HAL_GetTick();
		while (can_rx_flag == 0) {
			if ((HAL_GetTick() - tickstart) > 10) {
				HAL_UART_Transmit(&huart2, (uint8_t*)"Timeout: MIT Control\r\n", 22, 100);
				break;
			}
		}

		if (can_rx_flag) {
			print_unified_can_response(&huart2, 2, &motors[target_id - 1], rx_data, rs_can_rx_header.ExtId);
		}
	} else {return HAL_ERROR;}

	return HAL_OK;
}

HAL_StatusTypeDef motor_set_mit (uint8_t target_id, float torq, float pos, float spd, float kp, float kd)
{
	uint32_t tickstart;
	can_rx_flag = 0;
	if (can_mit_control_set(target_id, torq, pos, spd, kp, kd) == HAL_OK) {

		tickstart = HAL_GetTick();
		while (can_rx_flag == 0) {
			if ((HAL_GetTick() - tickstart) > 100) {
				return HAL_TIMEOUT;
			}
		}

	} else {return HAL_ERROR;}

	return HAL_OK;
}


HAL_StatusTypeDef motor_set_mit_dbg (uint8_t target_id, float torq, float pos, float spd, float kp, float kd)
{
	uint32_t tickstart;
	can_rx_flag = 0;
	if (can_mit_control_set(target_id, torq, pos, spd, kp, kd) == HAL_OK) {

		tickstart = HAL_GetTick();
		while (can_rx_flag == 0) {
			if ((HAL_GetTick() - tickstart) > 100) {
				HAL_UART_Transmit(&huart2, (uint8_t*)"Timeout: MIT Control\r\n", 22, 100);
				break;
			}
		}

		if (can_rx_flag) {
			print_unified_can_response(&huart2, 2, &motors[target_id - 1], rx_data, rs_can_rx_header.ExtId);
		}
	} else {return HAL_ERROR;}

	return HAL_OK;
}

HAL_StatusTypeDef motor_check_angle (motor_t *motor, float pos, float spd, float pd)
{
	uint32_t tickstart;
	can_rx_flag = 0;

	if(motor_set_spd(motor->id, spd, pd) == HAL_TIMEOUT) return HAL_TIMEOUT;

	if ((spd < 0 && motor->pos < pos) || (spd > 0 && motor->pos > pos)){
		motor_set_spd(motor->id, 0, 0); //stop the motor
		return HAL_OK;
	}

	return HAL_BUSY;
}

HAL_StatusTypeDef motor_check_angle_dbg (motor_t *motor, float pos, float spd, float pd)
{
	uint32_t tickstart;
	can_rx_flag = 0;

	if ((spd < 0 && motor->pos < pos) || (spd > 0 && motor->pos > pos)){
		motor_set_spd_dbg(motor->id, 0, pd); //stop the motor
		return HAL_OK;
	}

	if(motor_set_spd_dbg(motor->id, spd, pd) == HAL_TIMEOUT) return HAL_TIMEOUT;

	return HAL_BUSY;
}
