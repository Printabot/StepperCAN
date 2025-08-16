/*
 * @file    can.c
 * @brief   Implements functions for sending an receive CAN commands
 * @author  github.com/printabot
 * @date    2025-07-13
 * License: CC BY-NC-SA 4.0. See LICENSE.
 * Disclaimer: Provided as is, no warranty of any kind.
 */
#include "../Inc/can.h"
#include "../Inc/board_config.h"
#include "../Inc/controller.h"
#include "../Inc/version.h"
#include "../Inc/port.h"

static void prepare_tx_data(uint8_t *data, int32_t valueA, int32_t valueB);
static void prepare_command_response(ret_status res, uint8_t command, int32_t aux_data);


/**
  * @brief Send telemetry data via CAN of each enabled motor
  * @param None
  * @retval None
  */
void send_periodic_data_via_CAN(){

	static uint8_t motor_selector = 0;
	static uint8_t TxData[8] = {0};

	int32_t steps;
	int32_t vel;

	switch(motor_selector){
		/* --------------------------------------------------------- */
		case 0:
#if MOTOR0_ENABLED == true
			steps = get_motor0_steps();
			vel = get_motor0_current_velocity();
			prepare_tx_data(TxData, steps, vel);
			send_CAN_frame(M0_ID, TxData, 8u);
#endif
			motor_selector = 1;
			break;
		/* --------------------------------------------------------- */
		case 1:
#if MOTOR1_ENABLED == true
			steps = get_motor1_steps();
			vel = get_motor1_current_velocity();
			prepare_tx_data(TxData, steps, vel);
			send_CAN_frame(M1_ID, TxData, 8u);
#endif
			motor_selector = 2;
			break;
		/* --------------------------------------------------------- */
		case 2:
#if MOTOR2_ENABLED == true
			steps = get_motor2_steps();
			vel = get_motor2_current_velocity();
			prepare_tx_data(TxData, steps, vel);
			send_CAN_frame(M2_ID, TxData, 8u);
#endif
			motor_selector = 3;
			break;
		/* --------------------------------------------------------- */
		case 3:
#if MOTOR3_ENABLED == true
			steps = get_motor3_steps();
			vel = get_motor3_current_velocity();
			prepare_tx_data(TxData, steps, vel);
			send_CAN_frame(M3_ID, TxData, 8u);
#endif
			motor_selector = 4;
			break;
		/* --------------------------------------------------------- */
		case 4:
#if MOTOR4_ENABLED == true
			steps = get_motor4_steps();
			vel = get_motor4_current_velocity();
			prepare_tx_data(TxData, steps, vel);
			send_CAN_frame(M4_ID, TxData, 8u);
#endif
			motor_selector = 0;
			break;
		/* --------------------------------------------------------- */
		default: break;
	}
}

/**
  * @brief Process an incoming frame via CAN
  *
  * @param Id 		CAN Id
  * @param data 	pointer to received data
  * @param DLC 		Size
  *
  * @retval None
  */
void CAN_rx_callback(uint16_t Id, uint8_t *data, uint8_t DLC){

	ret_status res;
	int32_t aux_data;

	switch(Id){

		case REQUEST_ID:
			res = process_command(data, DLC);
			aux_data = get_aux_data_response();
			prepare_command_response(res, data[0], aux_data);
			break;

		default:
			break;
	}
}

/**
  * @brief Create the CAN response
  *
  * @param res 			Response of the last command
  * @param command 		Last command
  * @param aux_data	 	Auxiliary data response according the last command
  *
  * @retval None
  */
static void __attribute__((unused)) prepare_command_response(ret_status res, uint8_t command, int32_t aux_data){

	uint8_t TxData[8] = {0};

	TxData[0] = (uint8_t)res;
	TxData[1] = command;
	TxData[2] = (uint8_t)aux_data;
	TxData[3] = (uint8_t)(aux_data >> 8u);
	TxData[4] = (uint8_t)(aux_data >> 16u);
	TxData[5] = (uint8_t)(aux_data >> 24u);
	send_CAN_frame(RESPONSE_ID, TxData, 8u);
}

/**
  * @brief Convert two int32 to 8 bytes for CAN
  *
  * @param data 	pointer to output data
  * @param DLC 		Size
  *
  * @retval None
  */
static void __attribute__((unused)) prepare_tx_data(uint8_t *data, int32_t valueA, int32_t valueB){

	data[0] = (uint8_t)valueA;
	data[1] = (uint8_t)(valueA >> 8u);
	data[2] = (uint8_t)(valueA >> 16u);
	data[3] = (uint8_t)(valueA >> 24u);
	data[4] = (uint8_t)valueB;
	data[5] = (uint8_t)(valueB >> 8u);
	data[6] = (uint8_t)(valueB >> 16u);
	data[7] = (uint8_t)(valueB >> 24u);
}

/**
  * @brief Send position and speed of the requested motor. Could be used for other values
  *
  * @param Id 	CAN id
  * @param valueA Position
  * @param valueB Speed
  *
  * @retval None
  */
void send_data_response(uint32_t Id, int32_t valueA, int32_t valueB){
	uint8_t TxData[8] = {0};
	prepare_tx_data(TxData, valueA, valueB);
	send_CAN_frame(Id, TxData, 8u);
}

