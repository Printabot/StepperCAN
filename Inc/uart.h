/*
 * @file    uart.h
 * @brief   Header for uart.c
 * @author  github.com/printabot
 * @date    2025-07-13
 * License: CC BY-NC-SA 4.0. See LICENSE.
 * Disclaimer: Provided as is, no warranty of any kind.
 */

#ifndef INC_UART_H_
#define INC_UART_H_

#include "common.h"
#include "../tmc_drivers/Inc/tmc2209.h"

#define BAUD_DELAY_US			(uint32_t)52 /* 19200 bps. 1/19200 = 52us/bit */
#define UART_RX_BUFFER_SIZE 	32u

void init_tmc_drivers(void);
void loop_uart_rx_process(void);
void send_uart_bitbang_byte(uint8_t data, GPIO_TypeDef *GPIOx, uint32_t PinMask);
void UART_rx_callback();

size_t cobs_decode(const uint8_t *input, size_t length, uint8_t *output);
size_t cobs_encode(const uint8_t *input, size_t length, uint8_t *output);

void tmc_uart_write (trinamic_motor_t *motor, TMC_uart_tx_datagram_t *datagram);

#endif /* INC_UART_H_ */
