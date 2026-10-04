#ifndef ROBOSTRIDE_ID_H
#define ROBOSTRIDE_ID_H
#include <stdint.h>

/* RobStride extended (29-bit) CAN arbitration-ID layout:
 *   bits  0- 7 : motor id          (Tx target id / Rx reporting id)
 *   bits  8-23 : 16-bit data field  (master id, packed torque, or Rx status word)
 *   bits 24-28 : communication type ("mode", 5 bits)
 *   bits 29-31 : reserved (0)
 *
 * Built and parsed with explicit shifts + masks. This replaces the previous
 * `*(exCanIdInfo*)&header.ExtId` pointer cast, which (a) violated strict aliasing
 * — undefined at -O2, where the compiler may drop the field stores and corrupt
 * the arbitration id — and (b) relied on the implementation-defined ordering of
 * C bitfields. Pure (no HAL), so it is host-tested: host/tests/test_robostride_id.py. */
static inline uint32_t rs_extid_pack(uint8_t mode, uint16_t data, uint8_t id)
{
    return ((uint32_t)(mode & 0x1Fu) << 24) |
           ((uint32_t)data           << 8)  |
            (uint32_t)id;
}

static inline uint8_t  rs_extid_mode(uint32_t ext) { return (uint8_t)((ext >> 24) & 0x1Fu); }
static inline uint16_t rs_extid_data(uint32_t ext) { return (uint16_t)((ext >> 8)  & 0xFFFFu); }
static inline uint8_t  rs_extid_id(uint32_t ext)   { return (uint8_t)( ext         & 0xFFu); }

#endif /* ROBOSTRIDE_ID_H */
