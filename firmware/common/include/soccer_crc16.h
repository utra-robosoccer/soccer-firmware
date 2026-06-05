#ifndef SOCCER_CRC16_H
#define SOCCER_CRC16_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SOCCER_CRC16_CCITT_INIT 0xFFFFu
#define SOCCER_CRC16_CCITT_POLY 0x1021u

static inline uint16_t soccer_crc16_ccitt_update(uint16_t crc, const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t bit = 0; bit < 8; ++bit) {
            if ((crc & 0x8000u) != 0u) {
                crc = (uint16_t)((crc << 1) ^ SOCCER_CRC16_CCITT_POLY);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

static inline uint16_t soccer_crc16_ccitt(const uint8_t *data, size_t len)
{
    return soccer_crc16_ccitt_update(SOCCER_CRC16_CCITT_INIT, data, len);
}

#ifdef __cplusplus
}
#endif

#endif
