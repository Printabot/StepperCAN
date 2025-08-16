/*
 * @file    board_config.h
 * @brief   Custom board configuration
 * @author  github.com/printabot
 * @date    2025-07-13
 * License: CC BY-NC-SA 4.0. See LICENSE.
 * Disclaimer: Provided as is, no warranty of any kind.
 */

#ifndef BOARD_CONFIG_H_
#define BOARD_CONFIG_H_

#include <stdbool.h>
#include "common.h"

/* --------------------------------------------------------- */
/* Board configuration */
/* --------------------------------------------------------- */
/* Select the number of drivers installed */
#define MOTOR0_ENABLED					false
#define MOTOR1_ENABLED					false
#define MOTOR2_ENABLED					true
#define MOTOR3_ENABLED					true
#define MOTOR4_ENABLED					false
#define MOTOR5_ENABLED					false

/* CAN */
#define CAN_ENABLED						true
#define	CAN_CONTINUOUS_TELEMETRY		true
#define ID_INTERVAL_MS					100
#define REQUEST_ID						0x100
#define RESPONSE_ID						0x101
#define M0_ID							0x110
#define M1_ID							0x111
#define M2_ID							0x112
#define M3_ID							0x113
#define M4_ID							0x114

/* BOARD UART */
#define UART_ENABLED					true
#define UART_BPS						115200

/* Define the UART speed for drivers. E.g. 52us -> 19200bps (software emulated) */
#define UART_PERIOD_FOR_DRIVERS			52

/* Step generetion period USED in the ISR (this value can´t define the ISR period) */
#define TIMER_ISR_PERIOD_US      		10

/* MOTOR0 configuration */
/* --------------------------------------------------------- */
#define M0_DRIVER_TYPE					_TMC2209
#define M0_UART_ENABLED					true
#define M0_TWO_STEPS_PER_CYCLE			true
#define M0_MICRO_STEPS_RESOLUTION		32
#define M0_CURRENT_MA					200
#define M0_HOLDING_CURRENT_PERC			30
#define M0_RSHUNT_MOHM					110
/* Default values */
#define M0_DEFAULT_ACCEL				10000
#define M0_DEFAULT_DECEL				10000
#define M0_DEFAULT_EMERGENCY_DECEL		300000
#define M0_DEFAULT_SPEED_POSITION_MODE	1000
/* Safety limit values */
#define M0_POSITION_LIMIT_P				1000000
#define M0_POSITION_LIMIT_N				-1000000
#define M0_SPEED_LIMIT					100000
#define M0_ACCEL_LIMIT					100000
#define M0_DECEL_LIMIT					M0_DEFAULT_EMERGENCY_DECEL
/* GPIO asignation */
#define M0_EN_GPIO_PORT					E1_EN_GPIO_Port
#define M0_EN_PIN						E1_EN_Pin
#define M0_DIR_GPIO_PORT				E1_DIR_GPIO_Port
#define M0_DIR_PIN						E1_DIR_Pin
#define M0_STEP_GPIO_PORT				E1_STEP_GPIO_Port
#define M0_STEP_PIN						E1_STEP_Pin
#define M0_SOFT_UART_TX_GPIO_PORT		E1_UART_GPIO_Port
#define M0_SOFT_UART_TX_PIN				E1_UART_Pin


/* MOTOR1 configuration */
/* --------------------------------------------------------- */
#define M1_DRIVER_TYPE					_TMC2209
#define M1_UART_ENABLED					true
#define M1_TWO_STEPS_PER_CYCLE			true
#define M1_MICRO_STEPS_RESOLUTION		32
#define M1_CURRENT_MA					200
#define M1_HOLDING_CURRENT_PERC			30
#define M1_RSHUNT_MOHM					110
/* Default values */
#define M1_DEFAULT_ACCEL				10000
#define M1_DEFAULT_DECEL				10000
#define M1_DEFAULT_EMERGENCY_DECEL		300000
#define M1_DEFAULT_SPEED_POSITION_MODE	1000
/* Safety limit values */
#define M1_POSITION_LIMIT_P				1000000
#define M1_POSITION_LIMIT_N				-1000000
#define M1_SPEED_LIMIT					100000
#define M1_ACCEL_LIMIT					100000
#define M1_DECEL_LIMIT					M1_DEFAULT_EMERGENCY_DECEL
/* GPIO asignation */
#define M1_EN_GPIO_PORT					E0_EN_GPIO_Port
#define M1_EN_PIN						E0_EN_Pin
#define M1_DIR_GPIO_PORT				E0_DIR_GPIO_Port
#define M1_DIR_PIN						E0_DIR_Pin
#define M1_STEP_GPIO_PORT				E0_STEP_GPIO_Port
#define M1_STEP_PIN						E0_STEP_Pin
#define M1_SOFT_UART_TX_GPIO_PORT		E0_UART_GPIO_Port
#define M1_SOFT_UART_TX_PIN				E0_UART_Pin

/* MOTOR2 configuration */
/* --------------------------------------------------------- */
#define M2_DRIVER_TYPE					_TMC2209
#define M2_UART_ENABLED					true
#define M2_TWO_STEPS_PER_CYCLE			true
#define M2_MICRO_STEPS_RESOLUTION		32
#define M2_CURRENT_MA					200
#define M2_HOLDING_CURRENT_PERC			30
#define M2_RSHUNT_MOHM					110
/* Default values */
#define M2_DEFAULT_ACCEL				10000
#define M2_DEFAULT_DECEL				10000
#define M2_DEFAULT_EMERGENCY_DECEL		300000
#define M2_DEFAULT_SPEED_POSITION_MODE	1000
/* Safety limit values */
#define M2_POSITION_LIMIT_P				1000000
#define M2_POSITION_LIMIT_N				-1000000
#define M2_SPEED_LIMIT					100000
#define M2_ACCEL_LIMIT					100000
#define M2_DECEL_LIMIT					M2_DEFAULT_EMERGENCY_DECEL
/* GPIO asignation */
#define M2_EN_GPIO_PORT					Z_EN_GPIO_Port
#define M2_EN_PIN						Z_EN_Pin
#define M2_DIR_GPIO_PORT				Z_DIR_GPIO_Port
#define M2_DIR_PIN						Z_DIR_Pin
#define M2_STEP_GPIO_PORT				Z_STEP_GPIO_Port
#define M2_STEP_PIN						Z_STEP_Pin
#define M2_SOFT_UART_TX_GPIO_PORT		Z_UART_GPIO_Port
#define M2_SOFT_UART_TX_PIN				Z_UART_Pin

/* MOTOR3 configuration */
/* --------------------------------------------------------- */
#define M3_DRIVER_TYPE					_TMC2209
#define M3_UART_ENABLED					true
#define M3_TWO_STEPS_PER_CYCLE			true
#define M3_MICRO_STEPS_RESOLUTION		32
#define M3_CURRENT_MA					200
#define M3_HOLDING_CURRENT_PERC			30
#define M3_RSHUNT_MOHM					110
/* Default values */
#define M3_DEFAULT_ACCEL				10000
#define M3_DEFAULT_DECEL				10000
#define M3_DEFAULT_EMERGENCY_DECEL		300000
#define M3_DEFAULT_SPEED_POSITION_MODE	1000
/* Safety limit values */
#define M3_POSITION_LIMIT_P				1000000
#define M3_POSITION_LIMIT_N				-1000000
#define M3_SPEED_LIMIT					100000
#define M3_ACCEL_LIMIT					100000
#define M3_DECEL_LIMIT					M3_DEFAULT_EMERGENCY_DECEL
/* GPIO asignation */
#define M3_EN_GPIO_PORT					Y_EN_GPIO_Port
#define M3_EN_PIN						Y_EN_Pin
#define M3_DIR_GPIO_PORT				Y_DIR_GPIO_Port
#define M3_DIR_PIN						Y_DIR_Pin
#define M3_STEP_GPIO_PORT				Y_STEP_GPIO_Port
#define M3_STEP_PIN						Y_STEP_Pin
#define M3_SOFT_UART_TX_GPIO_PORT		Y_UART_GPIO_Port
#define M3_SOFT_UART_TX_PIN				Y_UART_Pin

/* MOTOR4 configuration */
/* --------------------------------------------------------- */
#define M4_DRIVER_TYPE					_TMC2209
#define M4_UART_ENABLED					true
#define M4_TWO_STEPS_PER_CYCLE			true
#define M4_MICRO_STEPS_RESOLUTION		32
#define M4_CURRENT_MA					200
#define M4_HOLDING_CURRENT_PERC			30
#define M4_RSHUNT_MOHM					110
/* Default values */
#define M4_DEFAULT_ACCEL				10000
#define M4_DEFAULT_DECEL				10000
#define M4_DEFAULT_EMERGENCY_DECEL		300000
#define M4_DEFAULT_SPEED_POSITION_MODE	1000
/* Safety limit values */
#define M4_POSITION_LIMIT_P				1000000
#define M4_POSITION_LIMIT_N				-1000000
#define M4_SPEED_LIMIT					100000
#define M4_ACCEL_LIMIT					100000
#define M4_DECEL_LIMIT					M4_DEFAULT_EMERGENCY_DECEL
/* GPIO asignation */
#define M4_EN_GPIO_PORT					X_EN_GPIO_Port
#define M4_EN_PIN						X_EN_Pin
#define M4_DIR_GPIO_PORT				X_DIR_GPIO_Port
#define M4_DIR_PIN						X_DIR_Pin
#define M4_STEP_GPIO_PORT				X_STEP_GPIO_Port
#define M4_STEP_PIN						X_STEP_Pin
#define M4_SOFT_UART_TX_GPIO_PORT		X_UART_GPIO_Port
#define M4_SOFT_UART_TX_PIN				X_UART_Pin

#endif /* BOARD_CONFIG_H_ */
