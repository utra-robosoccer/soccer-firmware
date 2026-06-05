#ifndef SOCCER_COBS_H
#define SOCCER_COBS_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

static inline size_t soccer_cobs_max_encoded_len(size_t decoded_len)
{
    return decoded_len + (decoded_len / 254u) + 1u;
}

static inline int soccer_cobs_encode(
    const uint8_t *input,
    size_t input_len,
    uint8_t *output,
    size_t output_cap,
    size_t *output_len)
{
    size_t read_index = 0u;
    size_t write_index = 1u;
    size_t code_index = 0u;
    uint8_t code = 1u;

    if (output == 0 || output_len == 0 || output_cap == 0u) {
        return -1;
    }

    while (read_index < input_len) {
        if (input[read_index] == 0u) {
            output[code_index] = code;
            code_index = write_index++;
            if (write_index > output_cap) {
                return -1;
            }
            code = 1u;
            read_index++;
        } else {
            if (write_index >= output_cap) {
                return -1;
            }
            output[write_index++] = input[read_index++];
            code++;
            if (code == 0xFFu) {
                output[code_index] = code;
                code_index = write_index++;
                if (write_index > output_cap) {
                    return -1;
                }
                code = 1u;
            }
        }
    }

    output[code_index] = code;
    *output_len = write_index;
    return 0;
}

static inline int soccer_cobs_decode(
    const uint8_t *input,
    size_t input_len,
    uint8_t *output,
    size_t output_cap,
    size_t *output_len)
{
    size_t read_index = 0u;
    size_t write_index = 0u;

    if (output == 0 || output_len == 0) {
        return -1;
    }

    while (read_index < input_len) {
        uint8_t code = input[read_index++];
        if (code == 0u) {
            return -1;
        }

        size_t copy_len = (size_t)code - 1u;
        if ((read_index + copy_len) > input_len || (write_index + copy_len) > output_cap) {
            return -1;
        }

        for (size_t i = 0u; i < copy_len; ++i) {
            output[write_index++] = input[read_index++];
        }

        if (code != 0xFFu && read_index < input_len) {
            if (write_index >= output_cap) {
                return -1;
            }
            output[write_index++] = 0u;
        }
    }

    *output_len = write_index;
    return 0;
}

#ifdef __cplusplus
}
#endif

#endif
