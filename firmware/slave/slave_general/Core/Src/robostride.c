#include "robostride.h"
#include "proto_common.h"   /* motor_can_range_by_id() — per-model V/T ranges */


CAN_TxHeaderTypeDef rs_can_tx_header = {
    .StdId = 0x0,
    .ExtId = 0xff, // dummy value
    .IDE   = CAN_ID_EXT,
    .RTR   = CAN_RTR_DATA,
    .DLC   = 8
};

CAN_RxHeaderTypeDef rs_can_rx_header;

uint32_t TxMailbox;
volatile uint32_t can_tx_ok_count = 0;
volatile uint32_t can_tx_error_count = 0;
volatile uint32_t can_last_tx_error = 0;

// Expand the .extid to self defined struct. defined in .h file
#define txCanIdEx (*((exCanIdInfo*)&(rs_can_tx_header.ExtId)))
#define rxCanIdEx (*((exCanIdInfo*)&(rs_can_rx_header.ExtId)))

// --- Helper Functions ---
static uint16_t float_to_uint(float x, float x_min, float x_max, unsigned int bits)
{  
    /// Converts a float to an int, given range and number of bits ///
    float span = x_max - x_min;  
    if(x < x_min) x = x_min; 
    else if(x > x_max) x = x_max; 
    return (uint16_t)((x - x_min) * (float)((1U << bits) - 1U) / span);
} 

static float uint_to_float(uint32_t x_int, float x_min, float x_max, int bits)
{
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1U << bits) - 1U)) + offset;
}

/* Single transmit path for every CAN frame. The F446 bxCAN has only 3 TX
 * mailboxes; motor_runtime_update() bursts one frame per motor each 10ms tick,
 * so with >3 motors the later frames (e.g. CAN ids 4,5) were silently dropped
 * for lack of a free mailbox. Wait (bounded ~2ms) for a mailbox to free before
 * queueing. A full 8-byte extended frame at 1 Mbps drains in ~110us, so the
 * wait is effectively a few hundred microseconds even when all 3 are busy. */
static HAL_StatusTypeDef can_tx(uint8_t *msg)
{
    uint32_t t0 = HAL_GetTick();
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0u) {
        if ((HAL_GetTick() - t0) > 1u) {
            break;
        }
    }

    HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(&hcan1, &rs_can_tx_header, msg, &TxMailbox);
    if (status == HAL_OK) {
        can_tx_ok_count++;
    } else {
        can_tx_error_count++;
        can_last_tx_error = HAL_CAN_GetError(&hcan1);
    }
    return status;
}

// ============================================================================
// CAN Bus Init
// ============================================================================
static void ext_filter_words(uint32_t ext_id, uint32_t ext_mask, CAN_FilterTypeDef *filter)
{
    uint32_t id_word = ((ext_id & 0x1FFFFFFFU) << 3) | CAN_ID_EXT;
    uint32_t mask_word = ((ext_mask & 0x1FFFFFFFU) << 3) | CAN_ID_EXT;

    filter->FilterIdHigh = (uint16_t)(id_word >> 16);
    filter->FilterIdLow = (uint16_t)(id_word & 0xFFFFU);
    filter->FilterMaskIdHigh = (uint16_t)(mask_word >> 16);
    filter->FilterMaskIdLow = (uint16_t)(mask_word & 0xFFFFU);
}

static HAL_StatusTypeDef can_start_with_filter(CAN_FilterTypeDef *filter)
{
    if (HAL_CAN_ConfigFilter(&hcan1, filter) != HAL_OK) {
        return HAL_ERROR;
    }

    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        return HAL_ERROR;
    }

    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef can_bus_init(void)
{
    // Config CAN Filter to receive all incoming frames.
    CAN_FilterTypeDef sFilterConfig = {0};
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    return can_start_with_filter(&sFilterConfig);
}

HAL_StatusTypeDef can_bus_init_read_state_filter(uint8_t motor_id, uint16_t master_id)
{
    CAN_FilterTypeDef sFilterConfig = {0};
    uint32_t ext_id = (2UL << 24) | ((uint32_t)motor_id << 8) | (master_id & 0xFFU);
    uint32_t ext_mask = (0x1FUL << 24) | (0x00FFUL << 8) | 0xFFUL;

    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;
    ext_filter_words(ext_id, ext_mask, &sFilterConfig);

    return can_start_with_filter(&sFilterConfig);
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

    return can_tx(msg);
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

    /* Velocity and torque ranges are model-specific (RS00 vs RS02); position,
     * Kp and Kd are identical across models. Fall back to the RS02 globals if
     * the id is not in this slave's config. */
    const MotorCanRange *r = motor_can_range_by_id(id);
    float v_min = r ? r->v_min : V_MIN;
    float v_max = r ? r->v_max : V_MAX;
    float t_min = r ? r->t_min : T_MIN;
    float t_max = r ? r->t_max : T_MAX;

    txCanIdEx.mode = 1; // Type 1
    txCanIdEx.id = id;  // Target ID
    txCanIdEx.data = float_to_uint(torque, t_min, t_max, 16);
    txCanIdEx.res = 0;

    rs_can_tx_header.DLC = 8;

    msg[0] = float_to_uint(MechPosition, P_MIN, P_MAX, 16) >> 8;
    msg[1] = float_to_uint(MechPosition, P_MIN, P_MAX, 16) & 0xFF;
    msg[2] = float_to_uint(speed, v_min, v_max, 16) >> 8;
    msg[3] = float_to_uint(speed, v_min, v_max, 16) & 0xFF;
    msg[4] = float_to_uint(kp, KP_MIN, KP_MAX, 16) >> 8;
    msg[5] = float_to_uint(kp, KP_MIN, KP_MAX, 16) & 0xFF;
    msg[6] = float_to_uint(kd, KD_MIN, KD_MAX, 16) >> 8;
    msg[7] = float_to_uint(kd, KD_MIN, KD_MAX, 16) & 0xFF;

    return can_tx(msg);
}

HAL_StatusTypeDef can_read_motor_state(uint8_t id)
{
    return can_mit_control_set(id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
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

    // Velocity and torque ranges are model-specific; look up by this motor's id.
    const MotorCanRange *r = motor_can_range_by_id(motor->id);
    float v_min = r ? r->v_min : V_MIN;
    float v_max = r ? r->v_max : V_MAX;
    float t_min = r ? r->t_min : T_MIN;
    float t_max = r ? r->t_max : T_MAX;

    // 3. Unpack Data Payload
    // Byte 0-1: Position
    motor->pos = uint_to_float(recv_buf[0] << 8 | recv_buf[1], P_MIN, P_MAX, 16); // Note: Previous code was [1]<<8|0, Datasheet 643 says "High byte first" -> [0]<<8|[1]

    // Byte 2-3: Speed
    motor->rpm = uint_to_float(recv_buf[2] << 8 | recv_buf[3], v_min, v_max, 16);

    // Byte 4-5: Torque
    motor->torq = uint_to_float(recv_buf[4] << 8 | recv_buf[5], t_min, t_max, 16);

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
    return can_tx(msg);
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

    return can_tx(msg);
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
    return can_tx(msg);
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
    return can_tx(msg);
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
    return can_tx(msg);
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

    return can_tx(msg);
}
