/* can_rx — what the slave receives from the CAN bus: the protocol-aware RX
 * dispatch (can_link_on_frame: comm type → roster LUT → decode) and the
 * ISR-owned last-feedback cache motors[], read by the main loop only through
 * the snapshot accessors below. can_link stays protocol-blind transport. */

#ifndef CAN_RX_H
#define CAN_RX_H

#include "../../../../common/include/motor_types.h"  /* motor_t domain model */
#include <stdint.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_can.h"
#include "main.h"
#include "proto_common.h"   /* N_MOTORS, motor_user_configs[] (generated) */

#define MAX_MOTOR_COUNT N_MOTORS //Total motors on the CAN bus — driven by the generated motor_user_config.h
/* motors[] is private to can_rx.c (owned by the CAN RX ISR). Main-loop code
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
void can_rx_bind_ids(void);

/* Coherent copy of the CAN-id-sweep bitmap (bit per id that answered a Type-0
   probe; ISR-owned). The boot sweep reads it once after its reply window. */
void can_rx_sweep_snapshot(uint8_t out[32]);

/* zero_sta (0x7029) boot capture: 1 once motor idx's Type-17 reply arrived,
   raw value written to *raw_out. ISR-owned; single-load reads, no masking. */
uint8_t can_rx_zero_sta(uint8_t idx, uint32_t *raw_out);

/* fw-version boot capture, routed by WINDOW: arm opens motor idx's one-reply
   window (the next comm-type-2 frame from it is captured as the version reply,
   not decoded as feedback), can_rx_fw_ver returns 1 once captured and copies
   the raw 8-byte payload (00,C4 echo + version bytes). disarm_all MUST run
   when the boot check ends, before normal polling starts. */
void can_rx_fw_ver_arm(uint8_t idx);
void can_rx_fw_ver_disarm_all(void);
uint8_t can_rx_fw_ver(uint8_t idx, uint8_t out[8]);

#endif /* CAN_RX_H */
