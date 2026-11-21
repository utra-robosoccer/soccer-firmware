/* robstride — thin send wrappers: pure codec (common/motor_rs_can.h) in,
 * can_link transport out. The MIT wrapper resolves this slave's per-model
 * vendor ranges; everything else is a straight encode-and-send. */
#include "robstride.h"
#include "proto_common.h"   /* motor_rs_config_by_id() — per-model RS vendor ranges */

static HAL_StatusTypeDef rs_send(RsFrame f)
{
    return can_link_tx(f.ext_id, f.data);
}

HAL_StatusTypeDef rs_get_motor_id(uint8_t id, uint16_t master_id)
{
    return rs_send(rs_get_id_encode(id, master_id));
}

HAL_StatusTypeDef rs_mit_control_set(uint8_t id, float torque, float MechPosition, float speed, float kp, float kd)
{
    return rs_send(rs_mit_encode(motor_rs_config_by_id(id), id,
                                 torque, MechPosition, speed, kp, kd));
}

HAL_StatusTypeDef rs_read_motor_state(uint8_t id)
{
    return rs_mit_control_set(id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

HAL_StatusTypeDef rs_enable_motor(uint8_t id, uint16_t master_id)
{
    return rs_send(rs_enable_encode(id, master_id));
}

HAL_StatusTypeDef rs_disable_motor(uint8_t id, uint16_t master_id)
{
    return rs_send(rs_disable_encode(id, master_id));
}

HAL_StatusTypeDef rs_clear_fault(uint8_t id, uint16_t master_id)
{
    return rs_send(rs_clear_fault_encode(id, master_id));
}

HAL_StatusTypeDef rs_set_mech_zero(uint8_t id, uint16_t master_id)
{
    return rs_send(rs_mech_zero_encode(id, master_id));
}

HAL_StatusTypeDef rs_read_single_param(uint8_t id, uint16_t master_id, uint16_t index)
{
    return rs_send(rs_param_read_encode(id, master_id, index));
}

HAL_StatusTypeDef rs_change_motor_mode(uint8_t id, uint16_t master_id, rs_runmode_t rs_runmode)
{
    return rs_send(rs_runmode_encode(id, master_id, rs_runmode));
}
