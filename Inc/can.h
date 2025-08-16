/*
 * @file    can.h
 * @brief   Header can.c
 * @author  github.com/printabot
 * @date    2025-07-13
 * License: CC BY-NC-SA 4.0. See LICENSE.
 * Disclaimer: Provided as is, no warranty of any kind.
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_

#include "common.h"

void send_periodic_data_via_CAN(void);
void CAN_rx_callback(uint16_t Id, uint8_t *data, uint8_t DLC);
void send_data_response(uint32_t Id, int32_t valueA, int32_t valueB);

#endif /* INC_CAN_H_ */
