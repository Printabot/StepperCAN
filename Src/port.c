/*
 * @file    port.c
 * @brief   Implements port functions for specific architecture
 * @author  github.com/printabot
 * @date    2025-07-13
 * License: CC BY-NC-SA 4.0. See LICENSE.
 * Disclaimer: Provided as is, no warranty of any kind.
 */

#include "../Inc/port.h"
#include "../Inc/can.h"
#include "../Inc/uart.h"

extern uint8_t uart_rx_byte;

/* Declare the objects needed for your system peripherals, like WDT, CAN, UART, timers, etc. */
CAN_TxHeaderTypeDef TxHeader;
extern CAN_HandleTypeDef hcan;
extern IWDG_HandleTypeDef hiwdg;
extern UART_HandleTypeDef huart1;

/* Put inside this function the code to send CAN messages according to your system */
void send_CAN_frame(const uint16_t Id, const uint8_t *data, const uint8_t DLC){

	TxHeader.StdId = Id;
	TxHeader.ExtId = 0;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.DLC = DLC;
	TxHeader.TransmitGlobalTime = DISABLE;
	static uint32_t TxMailbox = 0;

	(void)HAL_CAN_AddTxMessage(&hcan, &TxHeader, data, &TxMailbox);
}

/* Put here your callback function for incoming CAN messages and then call CAN_rx_callback with captured Id, data and DLC*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK)
    {
    	CAN_rx_callback(rxHeader.StdId, rxData, rxHeader.DLC);
    }
}

/* Put here your callback function for incoming UART messages and then call UART_rx_callback */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1){
    	UART_rx_callback();
    }
}

/* Put inside this function the code to clear WDT if used */
void clear_WDT(){
	if(hiwdg.Instance != NULL){
		(void)HAL_IWDG_Refresh(&hiwdg);
	}
}

/* Put inside this function the code to enable the timer used to generate the STEP pulses by ISR every 10us (up to 2 motors) or 20us (up to 4 motors).
 * Avoid using high level functions (like HAL) for this timer */
void enable_STEP_ISR(){
	LL_TIM_SetCounter(TIM2, 0);
	LL_TIM_EnableIT_UPDATE(TIM2);
	LL_TIM_ClearFlag_UPDATE(TIM2);
	LL_TIM_EnableCounter(TIM2);
}

/* Put inside this function the code to enable the timer used to control the velocity by ISR every 1ms*/
void enable_DDA_ISR(){
	LL_TIM_SetCounter(TIM7, 0);
	LL_TIM_EnableIT_UPDATE(TIM7);
	LL_TIM_ClearFlag_UPDATE(TIM7);
	LL_TIM_EnableCounter(TIM7);
}

/* Put inside this function the code to add filters, additional configuration and start the CAN */
void start_CAN(){

	CAN_FilterTypeDef sFilterConfig;

	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;

	if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
	{
	  Error_Handler();
	}

	HAL_CAN_Start(&hcan);
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/* Put inside this function the code to activate the UART Rx of 1 byte using interruption */
void start_continue_uart_rx(){
	HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
}

/* Put inside this function the code to send data via UART using interruption */
void send_uart_buffer(uint8_t *data, uint16_t length)
{
	HAL_UART_Transmit_IT(&huart1, data, length);
}

/* Put inside this function the code to generate delays in microseconds to generate the UART frame to the TMC drivers */
void delay_us(const uint32_t us)
{
    LL_TIM_SetCounter(TIM6, 0);
    LL_TIM_EnableCounter(TIM6);
    while (LL_TIM_GetCounter(TIM6) < us);
    LL_TIM_DisableCounter(TIM6);
}

