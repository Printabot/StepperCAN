/*
 * @file    scheduler.h
 * @brief   Header for scheduler.c
 * @author  github.com/printabot
 * @date    2025-07-13
 * License: CC BY-NC-SA 4.0. See LICENSE.
 * Disclaimer: Provided as is, no warranty of any kind.
 */

#ifndef INC_SCHEDULER_H_
#define INC_SCHEDULER_H_

#include "common.h"

#define COUNT_ENABLED_MOTORS ((MOTOR0_ENABLED ? 1 : 0) + \
                             (MOTOR1_ENABLED ? 1 : 0) + \
                             (MOTOR2_ENABLED ? 1 : 0) + \
                             (MOTOR3_ENABLED ? 1 : 0) + \
                             (MOTOR4_ENABLED ? 1 : 0) + \
                             (MOTOR5_ENABLED ? 1 : 0))

#define TASK1_TICKS 		(uint32_t)ID_INTERVAL_MS/(uint32_t)5
#define TASK2_TICKS 		(uint32_t)1
#define TASK3_TICKS 		(uint32_t)10	/* 10 ms */
#define TASK4_TICKS 		(uint32_t)50	/* 50 ms */

void motor_control_init(void);
void do_motor_control_tasks(void);

#endif /* INC_SCHEDULER_H_ */
