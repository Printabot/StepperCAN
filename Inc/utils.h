/*
 * @file    utils.h
 * @brief   Header for utils.c
 * @author  github.com/printabot
 * @date    2025-07-13
 * License: CC BY-NC-SA 4.0. See LICENSE.
 * Disclaimer: Provided as is, no warranty of any kind.
 */

#ifndef INC_UTILS_H_
#define INC_UTILS_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "common.h"

/* Short delays */
#define NOP1()  __asm volatile ("nop" ::: "memory")
#define NOP2()  do { NOP1(); NOP1(); } while (0)
#define NOP3()  do { NOP2(); NOP1(); } while (0)
#define NOP4()  do { NOP3(); NOP1(); } while (0)
#define NOP5()  do { NOP4(); NOP1(); } while (0)
#define NOP6()  do { NOP5(); NOP1(); } while (0)
#define NOP7()  do { NOP6(); NOP1(); } while (0)
#define NOP8()  do { NOP7(); NOP1(); } while (0)
#define NOP9()  do { NOP8(); NOP1(); } while (0)
#define NOP10() do { NOP9(); NOP1(); } while (0)

void delay_us(const uint32_t us);
uint8_t crc8(const uint8_t *data, const size_t len);
uint32_t isqrt64(const uint64_t x);

uint16_t bytes_to_uint16(const uint8_t *data);
int32_t bytes_to_int32(const uint8_t *data);
bool is_opposite_direction(int32_t a, int32_t b);



#endif /* INC_UTILS_H_ */
