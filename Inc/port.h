/*
 * @file    port.h
 * @brief   Header for port.h. Additionally defines the GPIO access for writing and reading
 * @author  github.com/printabot
 * @date    2025-07-13
 * License: CC BY-NC-SA 4.0. See LICENSE.
 * Disclaimer: Provided as is, no warranty of any kind.
 */

#ifndef STEPPERCAN_PORT_H_
#define STEPPERCAN_PORT_H_

#include "common.h"

/* GPIO control. Use LL or direct register access (Avoid using HAL or similar high level functions) */
/* Define the functions for low level GPIO control according to your system */
#define SET_GPIO(port, pin)				LL_GPIO_SetOutputPin((port), (pin))
#define RESET_GPIO(port, pin)			LL_GPIO_ResetOutputPin((port), (pin))
#define TOGGLE_GPIO(port, pin)			LL_GPIO_TogglePin((port), (pin))
#define IS_GPIO_SET(port, pin)			LL_GPIO_IsOutputPinSet((port), (pin))
#define GPIO_STRUCT						GPIO_TypeDef	/* <------ GPIO_TypeDef is used in STM32 chips */

#define delay_ms(value)					HAL_Delay(value)
#define get_ticks(value)				HAL_GetTick(value)

void send_CAN_frame(const uint16_t Id, const uint8_t *data, const uint8_t DLC);
void clear_WDT(void);
void enable_STEP_ISR(void);
void enable_DDA_ISR(void);
void start_CAN(void);
void start_continue_uart_rx(void);
void send_uart_buffer(uint8_t *data, uint16_t length);
void delay_us(const uint32_t us);

#endif /* STEPPERCAN_PORT_H_ */
