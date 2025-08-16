/*
 * @file    core.c
 * @brief   Implements the core functions for the stepper motor control
 * @author  github.com/printabot
 * @date    2025-07-13
 * License: CC BY-NC-SA 4.0. See LICENSE.
 * Disclaimer: Provided as is, no warranty of any kind.
 */

#include "../Inc/core.h"
#include "../Inc/port.h"
#include "../Inc/board_config.h"

static void m0_process_step_isr(void);
static void m1_process_step_isr(void);
static void m2_process_step_isr(void);
static void m3_process_step_isr(void);
static void m4_process_step_isr(void);

volatile static int32_t motor0_phase_accum = 0;
volatile static int32_t motor1_phase_accum = 0;
volatile static int32_t motor2_phase_accum = 0;
volatile static int32_t motor3_phase_accum = 0;
volatile static int32_t motor4_phase_accum = 0;

volatile MotorData_t motor0;
volatile MotorData_t motor1;
volatile MotorData_t motor2;
volatile MotorData_t motor3;
volatile MotorData_t motor4;

static void motion_planner(volatile MotorData_t* motor);
static void update_ramp_dda(volatile MotorData_t* motor);

/**
  * @brief	Set the values needed for the acceleration and deceleration phase using DDA
  *
  * @param motor			Pointer to MotorData struct
  * @param new_velocity 	Target velocity, signed
  * @param ramp				Value in ms to reach velocity linearly
  *
  * @retval ret_status		Operation status code
  */
ret_status set_ramp_dda(volatile MotorData_t* motor, int32_t new_velocity, uint32_t ramp)
{
	/* Avoid zero division */
	if(ramp == 0){
		return (STATUS_INVALID_ARG);
	}
	/* Ramp too large */
	if(ramp > INT32_MAX){
		return (STATUS_INVALID_ARG);
	}
	motor->delta_dda = new_velocity - motor->current_velocity;
	/* Same speed, nothing to do */
	if(motor->delta_dda == 0){
		return (STATUS_NO_OP);
	}
	int64_t temp = (int64_t)motor->delta_dda * (int64_t)SCALE;
	/* Check limits */
	if((temp > INT32_MAX) || (temp < INT32_MIN)){
		return (STATUS_INVALID_ARG);
	}
	motor->increment_scaled_dda = (int32_t)temp / (int32_t)ramp;
	/* Ramp to large respect delta_dda */
	if(motor->increment_scaled_dda == 0){
		return (STATUS_INVALID_ARG);
	}
	motor->acum_error_dda = 0;
    motor->target_velocity = new_velocity;

    return (STATUS_OK);
}

/**
  * @brief Update in real time the target velocity for each motor. This function is called by ISR @ 1ms
  *
  * @param motor 	Pointer to MotorData
  *
  * @retval 		None
  */
static void __attribute__((unused)) update_ramp_dda(volatile MotorData_t* motor)
{
    int32_t cv = motor->current_velocity;
    if (cv != motor->target_velocity)
    {
        motor->acum_error_dda += motor->increment_scaled_dda;
        int32_t cambio = motor->acum_error_dda / SCALE;
        if (cambio != 0)
        {
        	cv += cambio;
            motor->acum_error_dda -= cambio * SCALE;
            /* Limits */
            if ((motor->delta_dda > 0 && cv > motor->target_velocity) ||
                (motor->delta_dda < 0 && cv < motor->target_velocity))
            {
            	cv = motor->target_velocity;
            }
        }
        /* This part is used to guarantee position exactitude when executing a position command
         * if update_ramp_dda sets the target velocity before completing all required steps */
        if(motor->working_phase == WAITING_FINAL_POSITION){
        	if(cv == motor->target_velocity){
        		if(motor->last_velocity > 0){
       				if(motor->current_position < motor->target_position && motor->target_velocity == 0){
       					cv = -motor->increment_scaled_dda /  SCALE;
       				}
       		 	}else{
       		 		if(motor->current_position > motor->target_position && motor->target_velocity == 0){
       		 			cv = -motor->increment_scaled_dda /  SCALE;
       		 		}
        		}
        	}
        }
        motor->current_velocity = cv;
    }
}

/**
  * @brief	Motion planner for single commands, used for position commands and dual velocity commands (opposite direction)
  *
  * @param motor 	Pointer to MotorData
  *
  * @retval 		None
  */
static void __attribute__((unused)) motion_planner(volatile MotorData_t* motor){

	if(motor->working_status == EXECUTING_POSITION_COMMAND){

		switch (motor->working_phase) {

			case WAITING_ZERO_CROSS_POSITION:
				if((motor->target_pos_reached != 0) || (motor->current_velocity == 0)){
					(void)set_ramp_dda(motor, motor->pending_velocity, (uint32_t)motor->t_accel_ms);
					motor->target_position = motor->aux_pos2;
					motor->working_phase = WAITING_FINAL_SEGMENT;
					motor->target_pos_reached = 0;
				}
				break;

			case WAITING_FINAL_SEGMENT:
				if(motor->target_pos_reached != 0){
					(void)set_ramp_dda(motor, motor->final_velocity, (uint32_t)motor->t_decel_ms);
					motor->target_position = motor->planner_final_position;
					motor->working_phase = WAITING_FINAL_POSITION;
					motor->target_pos_reached = 0;
				}
				break;

			case WAITING_FINAL_POSITION:
				if(motor->target_pos_reached != 0){
					/* Ensures the motor does not exceed the target position because of small truncation inaccuracies */
					motor->current_velocity = motor->target_velocity;
					motor->working_phase = MOTION_IDLE;
					motor->working_status = IDLE;
					motor->target_pos_reached = 0;
				}
				break;

			case MOTION_IDLE:
				motor->working_status = IDLE;
				break;

			default: break;

		}/* End switch */
	}else if(motor->working_status == EXECUTING_DUAL_VELOCITY_COMMAND){

		if(motor->current_velocity == 0){
			(void)set_ramp_dda(motor, motor->pending_velocity, (uint32_t)motor->t_accel_ms);
			motor->working_phase = EXECUTING_VELOCITY_COMMAND;
		}

	}else{
		/* Do nothing */
	}
}

/**
  * @brief Generate step pulses for all enabled motors. It should be called from a high priority ISR
  * @param None
  * @retval None
  */
void process_step_isr(){

	/* Motor Timing Notes (STM32F072 @ 48MHz, Cortex-M0)
	 *
	 * - Typical processing time per motor: ~2.7µs (measured via GPIO toggle/oscilloscope)
	 * - Interrupt period should be configured to:
	 *   1) Process all required motors
	 *   2) Keep total ISR processing under 50% of the period (recommended)
	 *      (%CPU load ≈ [#motors × time per motor] / [ISR period])
	 *
	 * Trade-off considerations:
	 * - Shorter interrupt period (e.g. 10µs):
	 *   + Supports higher step rates (up to 50K steps/s @ 1 step/cycle or 100K steps/s @ 2 step/cycle)
	 *   + Allows high-speed operation at lower microstepping (e.g. 1875 rpm @ 3200 steps/rev, 2 step/cycle)
	 *   + Enables smooth low-speed motion at high microstepping (e.g. 117 rpm @ 51200 steps/rev, 2 step/ycle)
	 *   - Limits number of simultaneous motors (e.g. 2 motors @ 10µs, 50% load)
	 *
	 * - Longer interrupt period (e.g. 50µs):
	 *   - Limits max step rate and speed
	 *   - May cause jerkier motion at low microstepping (e.g. 400 steps/rev)
	 *   + Supports more motors per MCU (e.g. up to 8 motors)
	 *
	 * Example configurations (CPU load approx.):
	 * - 10µs period: 2 motors (~50% load)
	 * - 15µs period: 3 motors (~48% load)
	 * - 20µs period: 4 motors (~48% load)
	 *
	 * Adjust period/motor count to match your application's performance and multitasking needs.
	 *
	 */
#if MOTOR0_ENABLED == 1
	m0_process_step_isr();
#endif

#if MOTOR1_ENABLED == 1
	m1_process_step_isr();
#endif

#if MOTOR2_ENABLED == 1
	m2_process_step_isr();
#endif

#if MOTOR3_ENABLED == 1
	m3_process_step_isr();
#endif

#if MOTOR4_ENABLED == 1
	m4_process_step_isr();
#endif
}

/**
  * @brief Update the speed of each motor in real time based on ramps or accelerations
  * @param None
  * @retval None
  */
void update_motor_data_isr(){

#if MOTOR0_ENABLED == 1
	motion_planner(&motor0);
	update_ramp_dda(&motor0);
#endif

#if MOTOR1_ENABLED == 1
	motion_planner(&motor1);
	update_ramp_dda(&motor1);
#endif

#if MOTOR2_ENABLED == 1
	motion_planner(&motor2);
	update_ramp_dda(&motor2);
#endif

#if MOTOR3_ENABLED == 1
	motion_planner(&motor3);
	update_ramp_dda(&motor3);
#endif

#if MOTOR4_ENABLED == 1
	motion_planner(&motor4);
	update_ramp_dda(&motor4);
#endif
}

/**
  * @brief Generate step pulses for motor 0
  * Keep __attribute__((unused)) to suppress static function warnings
  * @param None
  * @retval None
  */
static void __attribute__((unused)) m0_process_step_isr(){

	/* ================== MOTOR 0 ================== */
	int32_t abs_velocity = (motor0.current_velocity >= 0) ? motor0.current_velocity : -motor0.current_velocity;

#if M0_TWO_STEPS_PER_CYCLE == true
	motor0_phase_accum += abs_velocity * (int32_t)TIMER_ISR_PERIOD_US;
#else
	motor0_phase_accum += abs_velocity * ((int32_t)TIMER_ISR_PERIOD_US * (int32_t)2);
#endif

	if (motor0.current_velocity >= 0){
		SET_GPIO(M0_DIR_GPIO_PORT, M0_DIR_PIN);
	}else{
		RESET_GPIO(M0_DIR_GPIO_PORT, M0_DIR_PIN);
	}

	if (motor0_phase_accum >= PHASE_SCALE)
	{
		motor0_phase_accum -= PHASE_SCALE;
		TOGGLE_GPIO(M0_STEP_GPIO_PORT, M0_STEP_PIN);

#if M0_TWO_STEPS_PER_CYCLE == true
		if(1u == IS_GPIO_SET(M0_DIR_GPIO_PORT, M0_DIR_PIN)){
			motor0.current_position++;
		}else{
			motor0.current_position--;
		}
#else
		NOP1();
		if(1u == IS_GPIO_SET(M0_STEP_GPIO_PORT, M0_STEP_PIN)){
			if(1u == IS_GPIO_SET(M0_DIR_GPIO_PORT, M0_DIR_PIN)){
				motor0.current_position++;
			}else{
				motor0.current_position--;
			}
		}
#endif
		if(motor0.target_position == motor0.current_position){
			motor0.target_pos_reached = 1;
		}
	}
}

/**
  * @brief Generate step pulses for motor 1
  * Keep __attribute__((unused)) to suppress static function warnings
  * @param None
  * @retval None
  */
static void __attribute__((unused)) m1_process_step_isr(){

	/* ================== MOTOR 1 ================== */
	int32_t abs_velocity = (motor1.current_velocity >= 0) ? motor1.current_velocity : -motor1.current_velocity;

#if M1_TWO_STEPS_PER_CYCLE == true
	motor1_phase_accum += abs_velocity * (int32_t)TIMER_ISR_PERIOD_US;
#else
	motor1_phase_accum += abs_velocity * ((int32_t)TIMER_ISR_PERIOD_US * (int32_t)2);
#endif

	if (motor1.current_velocity >= 0){
		SET_GPIO(M1_DIR_GPIO_PORT, M1_DIR_PIN);
	}else{
		RESET_GPIO(M1_DIR_GPIO_PORT, M1_DIR_PIN);
	}

	if (motor1_phase_accum >= PHASE_SCALE)
	{
		motor1_phase_accum -= PHASE_SCALE;
		TOGGLE_GPIO(M1_STEP_GPIO_PORT, M1_STEP_PIN);

#if M1_TWO_STEPS_PER_CYCLE == true
		if(1u == IS_GPIO_SET(M1_DIR_GPIO_PORT, M1_DIR_PIN)){
			motor1.current_position++;
		}else{
			motor1.current_position--;
		}
#else
		NOP1();
		if(1u == IS_GPIO_SET(M1_STEP_GPIO_PORT, M1_STEP_PIN)){
			if(1u == IS_GPIO_SET(M1_DIR_GPIO_PORT, M1_DIR_PIN)){
				motor1.current_position++;
			}else{
				motor1.current_position--;
			}
		}
#endif
		if(motor1.target_position == motor1.current_position){
			motor1.target_pos_reached = 1;
		}
	}
}

/**
  * @brief Generate step pulses for motor 2
  * Keep __attribute__((unused)) to suppress static function warnings
  * @param None
  * @retval None
  */
static void __attribute__((unused)) m2_process_step_isr(){

	/* ================== MOTOR 2 ================== */
	int32_t abs_velocity = (motor2.current_velocity >= 0) ? motor2.current_velocity : -motor2.current_velocity;

#if M2_TWO_STEPS_PER_CYCLE == true
	motor2_phase_accum += abs_velocity * (int32_t)TIMER_ISR_PERIOD_US;
#else
	motor2_phase_accum += abs_velocity * ((int32_t)TIMER_ISR_PERIOD_US * (int32_t)2);
#endif

	if (motor2.current_velocity >= 0){
		SET_GPIO(M2_DIR_GPIO_PORT, M2_DIR_PIN);
	}else{
		RESET_GPIO(M2_DIR_GPIO_PORT, M2_DIR_PIN);
	}

	if (motor2_phase_accum >= PHASE_SCALE)
	{
		motor2_phase_accum -= PHASE_SCALE;
		TOGGLE_GPIO(M2_STEP_GPIO_PORT, M2_STEP_PIN);

#if M2_TWO_STEPS_PER_CYCLE == true
		if(1u == IS_GPIO_SET(M2_DIR_GPIO_PORT, M2_DIR_PIN)){
			motor2.current_position++;
		}else{
			motor2.current_position--;
		}
#else
		NOP1();
		if(1u == IS_GPIO_SET(M2_STEP_GPIO_PORT, M2_STEP_PIN)){
			if(1u == IS_GPIO_SET(M2_DIR_GPIO_PORT, M2_DIR_PIN)){
				motor2.current_position++;
			}else{
				motor2.current_position--;
			}
		}
#endif
		if(motor2.target_position == motor2.current_position){
			motor2.target_pos_reached = 1;
		}
	}
}

/**
  * @brief Generate step pulses for motor 3
  * Keep __attribute__((unused)) to suppress static function warnings
  * @param None
  * @retval None
  */
static void __attribute__((unused)) m3_process_step_isr(){

	/* ================== MOTOR 3 ================== */
	int32_t abs_velocity = (motor3.current_velocity >= 0) ? motor3.current_velocity : -motor3.current_velocity;

#if M3_TWO_STEPS_PER_CYCLE == true
	motor3_phase_accum += abs_velocity * (int32_t)TIMER_ISR_PERIOD_US;
#else
	motor3_phase_accum += abs_velocity * ((int32_t)TIMER_ISR_PERIOD_US * (int32_t)2);
#endif

	if (motor3.current_velocity >= 0){
		SET_GPIO(M3_DIR_GPIO_PORT, M3_DIR_PIN);
	}else{
		RESET_GPIO(M3_DIR_GPIO_PORT, M3_DIR_PIN);
	}

	if (motor3_phase_accum >= PHASE_SCALE)
	{
		motor3_phase_accum -= PHASE_SCALE;
		TOGGLE_GPIO(M3_STEP_GPIO_PORT, M3_STEP_PIN);

#if M3_TWO_STEPS_PER_CYCLE == true
		if(1u == IS_GPIO_SET(M3_DIR_GPIO_PORT, M3_DIR_PIN)){
			motor3.current_position++;
		}else{
			motor3.current_position--;
		}
#else
		NOP1();
		if(1u == IS_GPIO_SET(M3_STEP_GPIO_PORT, M3_STEP_PIN)){
			if(1u == IS_GPIO_SET(M3_DIR_GPIO_PORT, M3_DIR_PIN)){
				motor3.current_position++;
			}else{
				motor3.current_position--;
			}
		}
#endif
		if(motor3.target_position == motor3.current_position){
			motor3.target_pos_reached = 1;
		}
	}
}

/**
  * @brief Generate step pulses for motor 4
  * Keep __attribute__((unused)) to suppress static function warnings
  * @param None
  * @retval None
  */
static void __attribute__((unused)) m4_process_step_isr(){

	/* ================== MOTOR 4 ================== */
	int32_t abs_velocity = (motor4.current_velocity >= 0) ? motor4.current_velocity : -motor4.current_velocity;

#if M4_TWO_STEPS_PER_CYCLE == true
	motor4_phase_accum += abs_velocity * (int32_t)TIMER_ISR_PERIOD_US;
#else
	motor4_phase_accum += abs_velocity * ((int32_t)TIMER_ISR_PERIOD_US * (int32_t)2);
#endif

	if (motor4.current_velocity >= 0){
		SET_GPIO(M4_DIR_GPIO_PORT, M4_DIR_PIN);
	}else{
		RESET_GPIO(M4_DIR_GPIO_PORT, M4_DIR_PIN);
	}

	if (motor4_phase_accum >= PHASE_SCALE)
	{
		motor4_phase_accum -= PHASE_SCALE;
		TOGGLE_GPIO(M4_STEP_GPIO_PORT, M4_STEP_PIN);

#if M4_TWO_STEPS_PER_CYCLE == true
		if(1u == IS_GPIO_SET(M4_DIR_GPIO_PORT, M4_DIR_PIN)){
			motor4.current_position++;
		}else{
			motor4.current_position--;
		}
#else
		NOP1();
		if(1u == IS_GPIO_SET(M4_STEP_GPIO_PORT, M4_STEP_PIN)){
			if(1u == IS_GPIO_SET(M4_DIR_GPIO_PORT, M4_DIR_PIN)){
				motor4.current_position++;
			}else{
				motor4.current_position--;
			}
		}
#endif
		if(motor4.target_position == motor4.current_position){
			motor4.target_pos_reached = 1;
		}
	}
}
