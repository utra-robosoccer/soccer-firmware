/*
 * motor_chain.h
 *
 *  Created on: Jan 4, 2026
 *
 */

#ifndef INC_MOTOR_CHAIN_H_
#define INC_MOTOR_CHAIN_H_

#include "../../../../common/include/motor_types.h"  /* motor_t domain model */
#include <stdint.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_can.h"
#include "main.h"
#include "proto_common.h"   /* N_MOTORS, motor_user_configs[] (generated) */

#define MAX_MOTOR_COUNT N_MOTORS //Total motors on the CAN bus — driven by the generated motor_user_config.h
/* motors[] is private to motor_chain.c (owned by the CAN RX ISR). Main-loop code
   must NOT read it directly — use motor_get_snapshot() for a coherent read and
   motor_set_fault_word() for the one field it writes. */
/* Protocol-level RX diagnostics (wire-level ones — raw count, last ext-id/
   payload words — live in can_link.h). Updated from can_link_on_frame. */
extern volatile uint8_t can_rx_flag;
extern volatile uint32_t can_feedback_count;
extern volatile uint8_t can_feedback_motor_id;
extern volatile uint32_t can_type0_count;
extern volatile uint32_t can_type2_count;
extern volatile uint32_t can_type17_count;
extern volatile uint32_t can_unknown_type_count;
extern volatile uint32_t can_no_lut_count;
extern volatile uint32_t can_unpack_error_count;
extern volatile uint8_t can_last_type;
extern volatile uint8_t can_last_sender_id;

/* Coherent per-motor read for main-loop code: copies motors[idx] under a brief
   critical section so a multi-field read can't be split across two CAN feedback
   cycles. This is the ONLY read path into motors[] from outside the CAN ISR. */
void motor_get_snapshot(uint8_t idx, motor_t *out);

/* Write path for the one motor_t field main-loop code updates: the 0x3022
   fault-word sentinel/clear. A single 32-bit store is atomic w.r.t. the ISR. */
void motor_set_fault_word(uint8_t idx, uint32_t val);

/* Bind motors[i].id / master_id from motor_user_configs[] before the CAN bus starts,
   so the RX ISR's id lookup can match incoming feedback. Call once at init. */
void motor_chain_bind_ids(void);

/* Coherent copy of the CAN-id-sweep bitmap (bit per id that answered a Type-0
   probe; ISR-owned). The boot sweep reads it once after its reply window. */
void motor_chain_sweep_snapshot(uint8_t out[32]);

#endif /* INC_MOTOR_CHAIN_H_ */
