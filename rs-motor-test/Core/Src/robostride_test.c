#include "robostride_test.h"


CAN_TxHeaderTypeDef rs_can_tx_header = {
    .StdId = 0x0,
    .ExtId = 0xff, // dummy value
    .IDE   = CAN_ID_EXT,
    .RTR   = CAN_RTR_DATA,
    .DLC   = 8
};

CAN_RxHeaderTypeDef rs_can_rx_header;

// Expand the .extid to self defined struct. defined in .h file
#define txCanIdEx (*((exCanIdInfo*)&(rs_can_tx_header.ExtId)))
#define rxCanIdEx (*((exCanIdInfo*)&(rs_can_rx_header.ExtId)))

// --- Helper Functions ---
static int float_to_uint(float x, float x_min, float x_max, unsigned int bits)
{  
    /// Converts a float to an int, given range and number of bits ///
    float span = x_max - x_min;  
    if(x < x_min) x = x_min; 
    else if(x > x_max) x = x_max; 
    return (int) ((x - x_min) * ((float)((1 << bits) / span)));
} 

static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

// ============================================================================
// COMM TYPE 0: Get Device ID
// ============================================================================
HAL_StatusTypeDef can_get_motor_id(uint8_t id, uint16_t master_id)
{
    txCanIdEx.mode = 0; // Type 0
    txCanIdEx.id = id;  // Target Motor ID
    txCanIdEx.data = master_id; // Bits 8-23: Master ID
    txCanIdEx.res = 0;

    uint8_t msg[8] = {0}; // Data is empty (0)
    rs_can_tx_header.DLC = 8;

    return HAL_CAN_AddTxMessage(&hcan1, &rs_can_tx_header, msg, &TxMailbox);
}

HAL_StatusTypeDef can_unpack_get_id(motor_t* motor, uint8_t* recv_buf)
{
    // Rx Header Check: Mode should be 0.
    if (rxCanIdEx.mode != 0) return HAL_ERROR;

    uint8_t reported_id = (uint8_t)(rxCanIdEx.data & 0xFF);

    if (reported_id != motor->id) {
       motor -> id = reported_id;
    }

    // Payload: 64-bit MCU unique identifier
    uint64_t uid = 0;
    for(int i=0; i<8; i++){
        uid |= ((uint64_t)recv_buf[i] << (8 * i));
    }
    motor->mcu_id = uid;

    return HAL_OK;
}

// ============================================================================
// COMM TYPE 1: MIT Control Mode
// ============================================================================
HAL_StatusTypeDef can_mit_control_set(uint8_t id, float torque, float MechPosition, float speed, float kp, float kd)
{
    uint8_t msg[8];

    txCanIdEx.mode = 1; // Type 1
    txCanIdEx.id = id;  // Target ID
    txCanIdEx.data = float_to_uint(torque, T_MIN, T_MAX, 16);
    txCanIdEx.res = 0;

    rs_can_tx_header.DLC = 8;

    msg[0] = float_to_uint(MechPosition, P_MIN, P_MAX, 16) >> 8;
    msg[1] = float_to_uint(MechPosition, P_MIN, P_MAX, 16) & 0xFF;
    msg[2] = float_to_uint(speed, V_MIN, V_MAX, 16) >> 8;
    msg[3] = float_to_uint(speed, V_MIN, V_MAX, 16) & 0xFF;
    msg[4] = float_to_uint(kp, KP_MIN, KP_MAX, 16) >> 8;
    msg[5] = float_to_uint(kp, KP_MIN, KP_MAX, 16) & 0xFF;
    msg[6] = float_to_uint(kd, KD_MIN, KD_MAX, 16) >> 8;
    msg[7] = float_to_uint(kd, KD_MIN, KD_MAX, 16) & 0xFF;

    return HAL_CAN_AddTxMessage(&hcan1, &rs_can_tx_header, msg, &TxMailbox);
}

// ============================================================================
// COMM TYPE 2: Feedback (Rx Side)
// ============================================================================
HAL_StatusTypeDef can_unpack_motor_feedback(motor_t* motor, uint8_t* recv_buf)
{
    // 1. Verify Header Mode is 2
    if (rxCanIdEx.mode != 2) return HAL_ERROR;

    // 2. Parse the .data field (CAN bits 8-23)
    uint16_t data_field = rxCanIdEx.data;

    // Bit 8-15 of CAN ID corresponds to bits 0-7 of data_field -> Motor ID
    uint8_t feedback_id = data_field & RS_FB_DATA_ID_MASK;

    // Check if this packet belongs to the motor struct passed in
    if (feedback_id != motor->id){
        return HAL_ERROR;
    }

    // Bit 22-23 of CAN ID corresponds to bits 14-15 of data_field -> Mode
    uint8_t mode_val = (data_field & RS_FB_DATA_MODE_MASK) >> 14;
    motor->status = (motor_status_t)mode_val;

    // Bit 16-21 of CAN ID corresponds to bits 8-13 of data_field -> Faults
    uint8_t fault_byte = (data_field & RS_FB_DATA_FAULT_MASK) >> 8;

    motor->motor_errors.undervoltage   = (fault_byte >> FAULT_BIT_UNDERVOLTAGE)   & 0x01;
    motor->motor_errors.driver_fault   = (fault_byte >> FAULT_BIT_DRIVER_FAULT)   & 0x01;
    motor->motor_errors.overheat       = (fault_byte >> FAULT_BIT_OVERHEAT)       & 0x01;
    motor->motor_errors.encoder_fault  = (fault_byte >> FAULT_BIT_ENCODER_FAULT)  & 0x01;
    motor->motor_errors.stall_overload = (fault_byte >> FAULT_BIT_STALL_OVERLOAD) & 0x01;
    motor->motor_errors.uncalibrated   = (fault_byte >> FAULT_BIT_UNCALIBRATED)   & 0x01;

    // 3. Unpack Data Payload
    // Byte 0-1: Position
    motor->pos = uint_to_float(recv_buf[0] << 8 | recv_buf[1], P_MIN, P_MAX, 16); // Note: Previous code was [1]<<8|0, Datasheet 643 says "High byte first" -> [0]<<8|[1]

    // Byte 2-3: Speed
    motor->rpm = uint_to_float(recv_buf[2] << 8 | recv_buf[3], V_MIN, V_MAX, 16);

    // Byte 4-5: Torque
    motor->torq = uint_to_float(recv_buf[4] << 8 | recv_buf[5], T_MIN, T_MAX, 16);

    // Byte 6-7: Temperature
    motor->temperature = (float)(recv_buf[6] << 8 | recv_buf[7]) / 10.0;

    return HAL_OK;
}

// ============================================================================
// COMM TYPE 3: Enable Motor
// ============================================================================
HAL_StatusTypeDef can_enable_motor(uint8_t id, uint16_t master_id)
{
    uint8_t msg[8] = {0};
    txCanIdEx.mode = 3;
    txCanIdEx.id = id;
    txCanIdEx.res = 0;
    txCanIdEx.data = master_id;

    rs_can_tx_header.DLC = 8;
    return HAL_CAN_AddTxMessage(&hcan1, &rs_can_tx_header, msg, &TxMailbox);
}

// ============================================================================
// COMM TYPE 4: Disable Motor
// ============================================================================
HAL_StatusTypeDef can_disable_motor(uint8_t id, uint16_t master_id)
{
    txCanIdEx.mode = 4;
    txCanIdEx.id = id;
    txCanIdEx.data = master_id;
    txCanIdEx.res = 0;

    uint8_t msg[8] = {0x0};
    rs_can_tx_header.DLC = 8;

    return HAL_CAN_AddTxMessage(&hcan1, &rs_can_tx_header, msg, &TxMailbox);
}

// ============================================================================
// COMM TYPE 6: Set Mechanical Zero
// ============================================================================
HAL_StatusTypeDef can_set_mech_zero(uint8_t id, uint16_t master_id)
{
    txCanIdEx.mode = 0x6;
    txCanIdEx.data = master_id;
    txCanIdEx.id = id;
    txCanIdEx.res = 0;

    uint8_t msg[8] = {0};
    msg[0] = 0x1;

    rs_can_tx_header.DLC = 8;
    return HAL_CAN_AddTxMessage(&hcan1, &rs_can_tx_header, msg, &TxMailbox);
}

// ============================================================================
// COMM TYPE 7: Set CAN ID
// ============================================================================
HAL_StatusTypeDef can_set_motor_can_id(uint8_t id, uint16_t master_id, uint8_t new_motor_id)
{
    txCanIdEx.mode = 0x7;
    txCanIdEx.id = id; // Target current ID
    txCanIdEx.res = 0;
    txCanIdEx.data = (new_motor_id << 8) | (master_id & 0xFF);

    uint8_t msg[8] = {0};
    rs_can_tx_header.DLC = 8;
    return HAL_CAN_AddTxMessage(&hcan1, &rs_can_tx_header, msg, &TxMailbox);
}

// ============================================================================
// COMM TYPE 17: Read Single Parameter
// ============================================================================
HAL_StatusTypeDef can_read_single_param(uint8_t id, uint16_t master_id, uint16_t index)
{
    txCanIdEx.mode = 0x11; // 17
    txCanIdEx.id = id;
    txCanIdEx.data = master_id;
    txCanIdEx.res = 0;

    uint8_t msg[8] = {0};
    msg[0] = index & 0xFF;
    msg[1] = (index >> 8) & 0xFF;

    rs_can_tx_header.DLC = 8;
    return HAL_CAN_AddTxMessage(&hcan1, &rs_can_tx_header, msg, &TxMailbox);
}

HAL_StatusTypeDef can_unpack_single_param(uint8_t* recv_buf, float* result_val)
{
    if (rxCanIdEx.mode != 0x11) return HAL_ERROR;

    uint32_t raw_val = 0;
    raw_val |= recv_buf[4];
    raw_val |= (uint32_t)recv_buf[5] << 8;
    raw_val |= (uint32_t)recv_buf[6] << 16;
    raw_val |= (uint32_t)recv_buf[7] << 24;

    memcpy(result_val, &raw_val, sizeof(float));
    return HAL_OK;
}

// ============================================================================
// COMM TYPE 18: Write Single Parameter / Change Mode
// ============================================================================
HAL_StatusTypeDef can_change_motor_mode(uint8_t id, uint16_t master_id, rs_runmode_t rs_runmode)
{
    uint16_t reg_idx = 0x7005; // Index for run mode
    txCanIdEx.mode = 0x12; // Type 18
    txCanIdEx.id = id;
    txCanIdEx.data = master_id;
    txCanIdEx.res = 0;

    uint8_t msg[8] = {0x0};

    rs_can_tx_header.DLC = 8;

    // Index (Byte 0-1)
    msg[0] = reg_idx & 0xff;
    msg[1] = reg_idx >> 8;

    // Data (Byte 4-7)
    msg[4] = rs_runmode & 0xff;

    return HAL_CAN_AddTxMessage(&hcan1, &rs_can_tx_header, msg, &TxMailbox);
}
