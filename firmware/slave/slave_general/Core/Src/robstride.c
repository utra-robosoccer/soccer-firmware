/* robstride — thin send wrappers: pure codec (common/motor_rs_can.h) in,
 * can_link transport out. The MIT wrapper resolves this slave's per-model
 * vendor ranges; everything else is a straight encode-and-send. */
#include "robstride.h"
#include "proto_common.h"   /* motor_rs_config_by_id() — per-model RS vendor ranges */

static void rs_send(RsFrame f)
{
    (void)can_link_tx(f.ext_id, f.data);
}

void rs_get_motor_id(uint8_t id, uint16_t master_id)
{
    rs_send(rs_get_id_encode(id, master_id));
}

void rs_mit_control_set(uint8_t id, float torque, float MechPosition, float speed, float kp, float kd)
{
    rs_send(rs_mit_encode(motor_rs_config_by_id(id), id,
                          torque, MechPosition, speed, kp, kd));
}

void rs_enable_motor(uint8_t id, uint16_t master_id)
{
    rs_send(rs_enable_encode(id, master_id));
}

void rs_disable_motor(uint8_t id, uint16_t master_id)
{
    rs_send(rs_disable_encode(id, master_id));
}

void rs_clear_fault(uint8_t id, uint16_t master_id)
{
    rs_send(rs_clear_fault_encode(id, master_id));
}

void rs_set_mech_zero(uint8_t id, uint16_t master_id)
{
    rs_send(rs_mech_zero_encode(id, master_id));
}

void rs_read_single_param(uint8_t id, uint16_t master_id, uint16_t index)
{
    rs_send(rs_param_read_encode(id, master_id, index));
}

void rs_change_motor_mode(uint8_t id, uint16_t master_id, rs_runmode_t rs_runmode)
{
    rs_send(rs_runmode_encode(id, master_id, rs_runmode));
}
