/*
 * @file    utils.c
 * @brief   Implements utility functions
 * @author  github.com/printabot
 * @date    2025-07-13
 * License: CC BY-NC-SA 4.0. See LICENSE.
 * Disclaimer: Provided as is, no warranty of any kind.
 */

#include "../Inc/utils.h"


uint8_t crc8(const uint8_t *data, const size_t len)
{
	/* CRC-8 polinomio x⁸ + x² + x + 1 = 0x07 */
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
            crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
    }
    return (crc);
}

int32_t bytes_to_int32(const uint8_t *data){
	/* Little endian */
	return (int32_t)((data[3] << 24) | (data[2] << 16) | (data[1] << 8) | data[0]);
}

uint16_t bytes_to_uint16(const uint8_t *data){
	/* Little endian */
	return (uint16_t)((data[1] << 8) | data[0]);
}

bool is_opposite_direction(int32_t a, int32_t b) {
    return (((a ^ b) < 0) && (a != 0) && (b != 0));
}

// Calcula la raíz cuadrada entera de un uint64_t
uint32_t isqrt64(const uint64_t x)
{
    uint64_t op = x;
    uint64_t res = 0;
    uint64_t one = (uint64_t)1 << 62;

    while (one > op) one >>= 2;
    while (one != 0) {
        if (op >= res + one) {
            op -= res + one;
            res = (res >> 1) + one;
        } else {
            res >>= 1;
        }
        one >>= 2;
    }
    return (uint32_t)res;
}

