/*
 * @file    core.h
 * @brief   Header for core.c
 * @author  github.com/printabot
 * @date    2025-07-13
 * License: CC BY-NC-SA 4.0. See LICENSE.
 * Disclaimer: Provided as is, no warranty of any kind.
 */


#ifndef CORE_H_
#define CORE_H_

#include <stdint.h>
#include "controller.h"

ret_status set_ramp_dda(volatile MotorData_t* motor, int32_t new_velocity, uint32_t ramp);

#endif /* CORE_H_ */
