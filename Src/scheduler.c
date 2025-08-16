/*
 * @file    scheduler.c
 * @brief   Implements basic task scheduling
 * @author  github.com/printabot
 * @date    2025-07-13
 * License: CC BY-NC-SA 4.0. See LICENSE.
 * Disclaimer: Provided as is, no warranty of any kind.
 */

#include "../Inc/scheduler.h"
#include "../Inc/board_config.h"
#include "../Inc/controller.h"
#include "../Inc/uart.h"
#include "../Inc/can.h"
#include "../Inc/port.h"

uint32_t task1_ticks = 0;
uint32_t task2_ticks = 0;
uint32_t task3_ticks = 0;
uint32_t task4_ticks = 0;

void motor_control_init(){

	init_driver_controller();

	delay_ms(50);
	clear_WDT();
	delay_ms(50);
	clear_WDT();

#if UART_ENABLED == true
	start_continue_uart_rx();
#endif

	start_CAN();
	init_tmc_drivers();
	enable_DDA_ISR();
	enable_STEP_ISR();

	uint32_t now = get_ticks();
	task1_ticks = now + TASK1_TICKS;
	task2_ticks = now + TASK2_TICKS;
	task3_ticks = now + TASK3_TICKS;
	task4_ticks = now + TASK4_TICKS;
}

void do_motor_control_tasks(){

	uint32_t now = get_ticks();
	/* --------------------------------------------------------- */
	if(now >= task1_ticks){
		task1_ticks = now + TASK1_TICKS;
#if CAN_ENABLED == true && CAN_CONTINUOUS_TELEMETRY == true
		send_periodic_data_via_CAN();
#endif
	}
	/* --------------------------------------------------------- */
	if(now >= task2_ticks){
		task2_ticks = now + TASK2_TICKS;
#if UART_ENABLED == true
		loop_uart_rx_process();
#endif
	}
	/* --------------------------------------------------------- */
	if(now >= task3_ticks){
		task3_ticks = now + TASK3_TICKS;
		/* Do some task here */
	}
	/* --------------------------------------------------------- */
	if(now >= task4_ticks){
		task4_ticks = now + TASK4_TICKS;
		clear_WDT();
	}
	/* --------------------------------------------------------- */
}
