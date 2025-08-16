/*
 * @file    controller.h
 * @brief   Header controller.c
 * @author  github.com/printabot
 * @date    2025-07-13
 * License: CC BY-NC-SA 4.0. See LICENSE.
 * Disclaimer: Provided as is, no warranty of any kind.
 */

#ifndef INC_CONTROLLER_H_
#define INC_CONTROLLER_H_

#include <stdint.h>
#include "common.h"

/* MCU limits */
#define FW_MAX_SPEED						(int32_t)200000
#define FW_MAX_ACCEL_DECEL					(int32_t)200000

#define LIBRARY_MOTORS						5

/* Commands */
#define ENABLE_DISABLE_MOTOR				1		/* Enable or disable the drivers */
#define SET_MOTOR_VELOCITY_TIME_RAMP		2		/* Set a motor velocity that accelerates according a time ramp */
#define SET_MOTOR_VELOCITY_ACCEL_RAMP		3		/* Set a motor velocity that accelerates according the acceleration value */
#define SET_MOTOR_POSITION					4		/* Move the motor to a requested position and speed */
#define SET_SPEED_POSITION_MODE				6		/* Set the speed if a large value is needed  */
#define SET_ACCEL							7
#define SET_DECEL							8
#define SET_EMERGENCY_DECEL					9
#define SET_POSITION_MODE					11
#define CLEAR_POSITION_COUNTER				12
#define COMMAND_FLOW_CONTROL				13
#define SET_SYNCHRONIZED_POSITION			14
#define SET_SYNCHRONIZED_SPEED				15
#define START_SYNCHRONIZED_POSITION			16
#define SET_MOTOR_POSITION_TIME				17
#define GET_POSITION_AND_SPEED				18

/* Functional modes */
#define IDLE								0
#define EXECUTING_VELOCITY_COMMAND			1		/* Same direction velocity command */
#define EXECUTING_DUAL_VELOCITY_COMMAND		2		/* Opposite direction. It has to decelerate first and the accelerate */
#define EXECUTING_POSITION_COMMAND			3		/* Arbitrary position command */

/* Motion phases */
#define MOTION_IDLE							0
#define WAITING_FINAL_SEGMENT				1
#define WAITING_FINAL_POSITION				2
#define WAITING_ZERO_CROSS_POSITION			3

#define ABSOLUTE_POSITION					0
#define RELATIVE_POSITION					1

/* Supported driver list */
#define	_GENERIC							0
#define _TMC2209							1

#define SCALE 								1000  				/* Thousandth precision	*/
#define PHASE_SCALE         				(int32_t)1000000    /* Equivalent to "1.0f" in accumulator */

#define FALSE 								0
#define TRUE  								1


typedef enum {
    STATUS_OK = 0,
	STATUS_NO_OP,
	STATUS_BUSY,
    STATUS_INVALID_ARG,
	STATUS_INVALID_MOTOR,
	STATUS_OUT_OF_RANGE,
	STATUS_UNKNOWN_ERROR,
	STATUS_FW_FAULT,
} ret_status;

/**
 * Motor data struct
 *
 * All members explicitly use int32_t/uint32_t for cross-M-core performance:
 * - M0/M0+/M1: Atomic 32-bit access avoids shifts/masks (Thumb ISA limits).
 * - M3/M4/M7: Enables burst transfers (LDM/STM) and cache-line optimizations.
 * - M23/M33: Maintains compatibility with TrustZone attr alignment (if used).
 * Tradeoffs:
 * - Memory overhead vs. speed (justified for reg-heavy/critical paths).
 * - Padding may still occur if structs are nested/packed differently.
 */
typedef struct {

	/* Position */
	int32_t current_position;			/* Current position in steps */
	int32_t target_position;
	int32_t target_pos_reached;
	int32_t aux_pos2;
	int32_t planner_final_position;
	int32_t position_mode;				/* Absolute or relative */

	/* Velocity */
	int32_t current_velocity;			/* Current velocity in steps/s. This value is the velocity command used by the STEP generator */
    int32_t target_velocity;			/* Target velocity when executing any acceleration phase */
    int32_t final_velocity;
	int32_t pending_velocity;
	int32_t last_velocity;
	int32_t speed_position_mode;		/* User speed value when using position commands on trapezoidal/S-curve profiles */

	/* Accel/decel */
    int32_t acceleration;				/* User acceleration value in steps/s^2 */
    int32_t deceleration;				/* User deceleration value in steps/s^2 */
    int32_t emergency_deceleration;		/* User deceleration value in steps/s^2 for fastest stop */
	int32_t t_accel_ms;					/* Time in ms for acceleration phase */
    int32_t t_decel_ms;					/* Time in ms for deceleration phase */

    /* States */
    int32_t working_status;				/* Idle, executing velocity command, executing velocity command */
    int32_t working_phase;

    /* DDA */
    int32_t delta_dda;
    int32_t increment_scaled_dda;
    int32_t acum_error_dda;
    int32_t time_ramp;

    /* Position synchronization */
    int32_t target_sync_pos;
    int32_t abs_distance;
    int32_t t_total_ms;

	/* Safety limits */
	int32_t position_limit_positive;
	int32_t position_limit_negative;
	int32_t speed_limit;
	int32_t acceleration_limit;
	int32_t deceleration_limit;

	/* Aux data */
	int32_t CAN_Id;

} MotorData_t;

void Update_Rampa_Tiempo_DDA(volatile MotorData_t* motor);

void init_driver_controller(void);
void set_speed_pos_mode(uint8_t  motor, int32_t speed);
void process_step_isr(void);
void update_motor_data_isr(void);

volatile MotorData_t *get_motor_data_struct(uint8_t motor);

ret_status enable_all_motors();
ret_status disable_all_motors();
ret_status enable_single_motor(const uint8_t motor);
ret_status disable_single_motor(const uint8_t motor);

ret_status set_accel_decel_param(volatile MotorData_t *m, const int32_t value, const uint8_t type);
ret_status set_synchronized_position(volatile MotorData_t *motor, int32_t position);
ret_status set_motor_position(volatile MotorData_t *motor, int32_t target_position, int32_t speed);
ret_status set_motor_position_time(volatile MotorData_t *motor, int32_t new_position, int32_t time);
ret_status set_motor_velocity_according_time_ramp(volatile MotorData_t *motor, int32_t velocity, uint32_t ramp);
ret_status set_motor_velocity_according_accel_decel(volatile MotorData_t *motor, int32_t velocity);
ret_status process_command(const uint8_t *rxData, const uint8_t DLC);

int32_t get_motor0_steps();
int32_t get_motor1_steps();
int32_t get_motor2_steps();
int32_t get_motor3_steps();
int32_t get_motor4_steps();

int32_t get_motor0_current_velocity();
int32_t get_motor1_current_velocity();
int32_t get_motor2_current_velocity();
int32_t get_motor3_current_velocity();
int32_t get_motor4_current_velocity();

int32_t get_aux_data_response(void);

#endif /* INC_CONTROLLER_H_ */
