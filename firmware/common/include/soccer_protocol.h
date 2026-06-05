#ifndef SOCCER_PROTOCOL_H
#define SOCCER_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SOCCER_PROTOCOL_VERSION 1u
#define SOCCER_FRAME_DELIMITER 0x00u
#define SOCCER_MAX_PAYLOAD_LEN 256u

#if defined(__GNUC__)
#define SOCCER_PACKED __attribute__((packed))
#else
#define SOCCER_PACKED
#endif

typedef enum {
    SOCCER_MSG_PING = 1,
    SOCCER_MSG_DISCOVER = 2,
    SOCCER_MSG_MOTOR_STATE_REQ = 3,
    SOCCER_MSG_ARM_HOLD = 4,
    SOCCER_MSG_DISABLE = 5,
    SOCCER_MSG_MOTOR_STATE = 6,
} SoccerMsgType;

typedef enum {
    SOCCER_SOURCE_JETSON = 1,
    SOCCER_SOURCE_MASTER = 2,
    SOCCER_SOURCE_SLAVE = 3,
    SOCCER_SOURCE_BROADCAST = 255,
} SoccerNodeId;

typedef enum {
    SOCCER_MOTOR_UNDISCOVERED = 0,
    SOCCER_MOTOR_DISCOVERING = 1,
    SOCCER_MOTOR_IDLE = 2,
    SOCCER_MOTOR_ARMING_HOLD = 3,
    SOCCER_MOTOR_ARMED_HOLD = 4,
    SOCCER_MOTOR_MIT_CONTROL = 5,
    SOCCER_MOTOR_ZEROING = 6,
    SOCCER_MOTOR_DISABLING = 7,
    SOCCER_MOTOR_DISABLED = 8,
    SOCCER_MOTOR_FAULT = 9,
} SoccerMotorLifecycleState;

typedef struct SOCCER_PACKED {
    uint16_t type;
    uint16_t seq;
    uint8_t source;
    uint8_t target;
    uint32_t ts_us;
    uint16_t len;
    uint16_t flags;
    uint16_t crc;
} SoccerMsgHeader;

typedef struct SOCCER_PACKED {
    float pos;
    float vel;
    float kp;
    float kd;
    float tau;
} SoccerMotorCmd;

typedef struct SOCCER_PACKED {
    float pos;
    float vel;
    float tau;
    float temp;
    uint32_t fault;
    uint16_t last_cmd_seq;
} SoccerMotorState;

#define SOCCER_MSG_HEADER_SIZE 16u
#define SOCCER_MOTOR_CMD_SIZE 20u
#define SOCCER_MOTOR_STATE_SIZE 22u

#if defined(__cplusplus)
static_assert(sizeof(SoccerMsgHeader) == SOCCER_MSG_HEADER_SIZE, "SoccerMsgHeader size mismatch");
static_assert(sizeof(SoccerMotorCmd) == SOCCER_MOTOR_CMD_SIZE, "SoccerMotorCmd size mismatch");
static_assert(sizeof(SoccerMotorState) == SOCCER_MOTOR_STATE_SIZE, "SoccerMotorState size mismatch");
#elif defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(SoccerMsgHeader) == SOCCER_MSG_HEADER_SIZE, "SoccerMsgHeader size mismatch");
_Static_assert(sizeof(SoccerMotorCmd) == SOCCER_MOTOR_CMD_SIZE, "SoccerMotorCmd size mismatch");
_Static_assert(sizeof(SoccerMotorState) == SOCCER_MOTOR_STATE_SIZE, "SoccerMotorState size mismatch");
#endif

#ifdef __cplusplus
}
#endif

#endif
