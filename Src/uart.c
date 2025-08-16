/*
 * @file    uart.c
 * @brief   Implements functions for UART communication
 * @author  github.com/printabot
 * @date    2025-07-13
 * License: CC BY-NC-SA 4.0. See LICENSE.
 * Disclaimer: Provided as is, no warranty of any kind.
 */

#include "../Inc/uart.h"
#include "../Inc/controller.h"
#include "../Inc/utils.h"
#include "../Inc/port.h"
#include "../Inc/board_config.h"

TMC2209_t drv0;
TMC2209_t drv1;
TMC2209_t drv2;
TMC2209_t drv3;
TMC2209_t drv4;

static void prepare_UART_command_response(ret_status res, uint8_t command, int32_t aux_data);

volatile size_t uart_rx_index = 0;
volatile uint8_t uart_frame_ready = 0;

uint8_t uart_rx_byte;
uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];

void loop_uart_rx_process(void)
{
	ret_status res;
	int32_t aux_data;
	uint8_t decoded[UART_RX_BUFFER_SIZE*2];
	size_t decoded_len;

	if (1u == uart_frame_ready) {
		size_t len = uart_rx_index - 1;
        if ((len > 0) && (len < UART_RX_BUFFER_SIZE)){
            decoded_len = cobs_decode(uart_rx_buffer, len, decoded);
            res = process_command(decoded, decoded_len);
            aux_data = get_aux_data_response();
            prepare_UART_command_response(res, decoded[0], aux_data);
        }
        uart_rx_index = 0;
        uart_frame_ready = 0;
    }
}

/**
  * @brief Transmits a single byte via software UART (bit-banging)
  *
  * @param data      Byte to be transmitted (LSB first)
  * @param GPIOx     GPIO port to use for transmission
  * @param PinMask   Bitmask of the specific pin to use as TX output
  *
  * @note This function implements a software-based UART transmission
  *       without using hardware peripherals. Timing is handled by
  *       delay_us() function.
  *
  * @retval None
  */
void send_uart_bitbang_byte(uint8_t data, GPIO_TypeDef *GPIOx, uint32_t PinMask)
{
	SET_GPIO(GPIOx, PinMask);
	/* Optional delay */
	delay_us(2 * BAUD_DELAY_US);
	// Start bit
	RESET_GPIO(GPIOx, PinMask);
	delay_us(BAUD_DELAY_US);
	/* Send 8 data bits (LSB first) */
	for (int i = 0; i < 8; i++)
	{
		if (data & 0x01)
			SET_GPIO(GPIOx, PinMask);
		else
			RESET_GPIO(GPIOx, PinMask);

		delay_us(BAUD_DELAY_US);
		data >>= 1;
	}
	/* Stop bit */
	SET_GPIO(GPIOx, PinMask);
	delay_us(BAUD_DELAY_US);
}

/**
  * @brief  UART Receive Interrupt Callback
  *
  * @note   This function is called when a byte is received through the UART peripheral.
  *
  * @retval None
  */
void UART_rx_callback(){
	if(0 == uart_frame_ready){
		if (uart_rx_index < UART_RX_BUFFER_SIZE) {
			uart_rx_buffer[uart_rx_index++] = uart_rx_byte;
			if (uart_rx_byte == 0x00) {
				uart_frame_ready = 1u;    /* EOL */
			}
		} else {
			/* Buffer overflow */
			uart_rx_index = 0;
		}
	}
	/* Start again the reception data */
	start_continue_uart_rx();
}

/**
  * @brief Encode data using Consistent Overhead Byte Stuffing (COBS)
  *
  * @param input     Pointer to the input data to be encoded
  * @param length    Length of the input data to be encoded
  * @param output    Pointer to the buffer where COBS-encoded data will be stored
  *
  * @retval size_t   The length of the COBS-encoded data
  */
size_t cobs_encode(const uint8_t *input, size_t length, uint8_t *output)
{
    size_t read_index = 0;
    size_t write_index = 1;
    size_t code_index = 0;
    uint8_t code = 1;

    while (read_index < length) {
        if (input[read_index] == 0) {
            output[code_index] = code;
            code_index = write_index++;
            code = 1;
        } else {
            output[write_index++] = input[read_index];
            code++;
            if (code == 0xFF) {
                output[code_index] = code;
                code_index = write_index++;
                code = 1;
            }
        }
        read_index++;
    }
    output[code_index] = code;
    return write_index;
}

/**
  * @brief Decode data using Consistent Overhead Byte Stuffing (COBS)
  *
  * @param input     Pointer to the COBS-encoded input data
  * @param length    Length of the COBS-encoded input data
  * @param output    Pointer to the buffer where decoded data will be stored
  *
  * @retval size_t   The length of the decoded data
  */
size_t cobs_decode(const uint8_t *input, size_t length, uint8_t *output)
{
    size_t read_index = 0;
    size_t write_index = 0;
    uint8_t code, i;

    while (read_index < length) {
        code = input[read_index++];
        for (i = 1; i < code; i++)
            output[write_index++] = input[read_index++];
        if (code != 0xFF && read_index < length)
            output[write_index++] = 0;
    }
    return write_index;
}

/**
  * @brief Create the UART response using Consistent Overhead Byte Stuffing (COBS)
  *
  * @param res 			Response of the last command
  * @param command 		Last command
  * @param aux_data	 	Auxiliary data response according the last command
  *
  * @retval None
  */
static void __attribute__((unused)) prepare_UART_command_response(ret_status res, uint8_t command, int32_t aux_data){

	uint8_t raw_data[UART_RX_BUFFER_SIZE];
	uint8_t coded_data[UART_RX_BUFFER_SIZE*2];
	uint8_t len = 0;

	raw_data[0] = (uint8_t)res;
	raw_data[1] = command;

	if(aux_data > 0){
		raw_data[2] = (uint8_t)aux_data;
		raw_data[3] = (uint8_t)(aux_data >> 8u);
		raw_data[4] = (uint8_t)(aux_data >> 16u);
		raw_data[5] = (uint8_t)(aux_data >> 24u);
		len = 6;
	}else{
		len = 2;
	}
	len = (uint8_t)cobs_encode(raw_data, len, coded_data);
	coded_data[len] = 0;
	len++;
	send_uart_buffer(coded_data, len);
}

/**
  * @brief Send a datagram for a specific driver via UART
  *
  * @param trinamic_motor_t			Pointer to get the motor id
  * @param TMC_uart_tx_datagram_t 	Pointer to content to send
  *
  * @retval None
  */
void tmc_uart_write (trinamic_motor_t *motor, TMC_uart_tx_datagram_t *datagram)
{

	GPIO_STRUCT *GPIOx = NULL;
	uint32_t PinMask = 0;

	if(motor->id > LIBRARY_MOTORS){	/* Too many motors */
		return;
	}

	switch (motor->id){

		case 0:
			GPIOx = M0_SOFT_UART_TX_GPIO_PORT;
			PinMask = M0_SOFT_UART_TX_PIN;
			break;

		case 1:
			GPIOx = M1_SOFT_UART_TX_GPIO_PORT;
			PinMask = M1_SOFT_UART_TX_PIN;
			break;

		case 2:
			GPIOx = M2_SOFT_UART_TX_GPIO_PORT;
			PinMask = M2_SOFT_UART_TX_PIN;
			break;

		case 3:
			GPIOx = M3_SOFT_UART_TX_GPIO_PORT;
			PinMask = M3_SOFT_UART_TX_PIN;
			break;

		case 4:
			GPIOx = M4_SOFT_UART_TX_GPIO_PORT;
			PinMask = M4_SOFT_UART_TX_PIN;
			break;

		default:
			return;
			break;
	}

	for (uint8_t i = 0; i < 8u; i++) {
		send_uart_bitbang_byte(datagram->data[i], GPIOx, PinMask);
	    delay_us(BAUD_DELAY_US); /* Optional */
	}

}

/**
  * @brief Init and configure the TMC drivers via soft UART
  * @param None
  * @retval None
  */
void init_tmc_drivers(){

	set_default_values(&drv0);
	set_default_values(&drv1);
	set_default_values(&drv2);
	set_default_values(&drv3);
	set_default_values(&drv4);

	drv0.config.motor.id = 0u;
	drv1.config.motor.id = 1u;
	drv2.config.motor.id = 2u;
	drv3.config.motor.id = 3u;
	drv4.config.motor.id = 4u;

#if MOTOR0_ENABLED == true && M0_UART_ENABLED == true
	TMC2209_config(&drv0, (uint8_t)M0_MICRO_STEPS_RESOLUTION, (uint16_t)M0_CURRENT_MA, (uint8_t)M0_HOLDING_CURRENT_PERC, (bool)M0_TWO_STEPS_PER_CYCLE);
#endif

#if MOTOR1_ENABLED == true && M1_UART_ENABLED == true
	TMC2209_config(&drv1, (uint8_t)M1_MICRO_STEPS_RESOLUTION, (uint16_t)M1_CURRENT_MA, (uint8_t)M1_HOLDING_CURRENT_PERC, (bool)M1_TWO_STEPS_PER_CYCLE);
#endif

#if MOTOR2_ENABLED == true && M2_UART_ENABLED == true
	TMC2209_config(&drv2, (uint8_t)M2_MICRO_STEPS_RESOLUTION, (uint16_t)M2_CURRENT_MA, (uint8_t)M2_HOLDING_CURRENT_PERC, (bool)M2_TWO_STEPS_PER_CYCLE);
#endif

#if MOTOR3_ENABLED == true && M3_UART_ENABLED == true
	TMC2209_config(&drv3, (uint8_t)M3_MICRO_STEPS_RESOLUTION, (uint16_t)M3_CURRENT_MA, (uint8_t)M3_HOLDING_CURRENT_PERC, (bool)M3_TWO_STEPS_PER_CYCLE);
#endif

#if MOTOR4_ENABLED == true && M4_UART_ENABLED == true
	TMC2209_config(&drv4, (uint8_t)M4_MICRO_STEPS_RESOLUTION, (uint16_t)M4_CURRENT_MA, (uint8_t)M4_HOLDING_CURRENT_PERC, (bool)M4_TWO_STEPS_PER_CYCLE);
#endif

}

