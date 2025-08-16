/*
 * @file    controller.c
 * @brief   Implements functions for speed, position and command parsing
 * @author  github.com/printabot
 * @date    2025-07-13
 * License: CC BY-NC-SA 4.0. See LICENSE.
 * Disclaimer: Provided as is, no warranty of any kind.
 */

#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include "../Inc/scheduler.h"
#include "../Inc/controller.h"
#include "../Inc/utils.h"
#include "../Inc/core.h"
#include "../Inc/board_config.h"
#include "../Inc/port.h"
#include "../Inc/can.h"
#include "../tmc_drivers/Inc/tmc2209.h"

extern volatile MotorData_t motor0;
extern volatile MotorData_t motor1;
extern volatile MotorData_t motor2;
extern volatile MotorData_t motor3;
extern volatile MotorData_t motor4;

static ret_status abs_distance_to_target_velocity(const int32_t vi, const int32_t vf, const int32_t accel, const int32_t decel, int32_t *result);
static ret_status get_deceleration_value(uint32_t vi, uint32_t vf, uint32_t d, int32_t *result);

static int32_t global_synchronized_speed = 0;
static int32_t travel_time_sync = 0;

/**
 * @brief Computes the maximum achievable speed for a triangular motion profile.
 *
 * @param a1     	Positive acceleration (steps/s^2).
 * @param a2     	Positive deceleration (steps/s^2).
 * @param vi     	Initial velocity (steps/s).
 * @param vf     	Final velocity (steps/s).
 * @param  d     	Total distance (steps).
 * @param result 	Pointer to store the computed vmax (steps/s).
 *
 * @return ret_status	Operation status code
 */
static ret_status vmax_general(int32_t a1, int32_t a2, int32_t vi, int32_t vf,  int32_t d, int32_t *result){

	if((a1 == 0) || (a2 == 0) || (d <= 0)){
		return (STATUS_INVALID_ARG);
	}
	if((abs(vi) > FW_MAX_SPEED) || (abs(vf) > FW_MAX_SPEED)){
		return (STATUS_OUT_OF_RANGE);
	}
	if((a1 > FW_MAX_ACCEL_DECEL) || (a2 > FW_MAX_ACCEL_DECEL)){
		return (STATUS_OUT_OF_RANGE);
	}

    int64_t num = ((int64_t)2 * (int64_t)a1 * (int64_t)a2 * (int64_t)d) + ((int64_t)a2 * (int64_t)vi * (int64_t)vi) + ((int64_t)a1 * (int64_t)vf * (int64_t)vf);
    int64_t denom = (int64_t)a1 + (int64_t)a2;
    int64_t vmax2 = num / denom;

    if((num % denom) != 0){
    	vmax2++;		/* Round away from zero */
    }
    *result = (int32_t)isqrt64((uint64_t)vmax2);
    return (STATUS_OK);
}

/**
 * @brief Get a new deceleration value when the motor has to reduce its speed over a short distance
 *
 * @param vi    	Initial velocity (steps/s).
 * @param vf    	Final velocity (steps/s).
 * @param  d    	Total distance (steps).
 * @param result	Pointer to store the deceleration value (steps/s^2).
 *
 * @return ret_status	Operation status code
 */
static ret_status get_deceleration_value(uint32_t vi, uint32_t vf, uint32_t d, int32_t *result)
{
	if(d == 0){
		return (STATUS_INVALID_ARG);	/* Avoid division by zero */
	}
    int64_t num = (int64_t)vi * vi - (int64_t)vf * vf;
    int64_t denom = 2 * (int64_t)d;
    *result = (int32_t)(num / denom);
    return (STATUS_OK);
}

/**
 * @brief Computes the acceleration or deceleration time according the speed difference in the same direction
 *
 * @param  vi     		Initial velocity (steps/s).
 * @param  vf     		Final velocity (steps/s).
 * @param  accel  		Positive acceleration (steps/s^2).
 * @param  decel  		Positive deceleration (steps/s^2).
 * @param  result 		Pointer to store the computed time (ms).
 *
 * @return ret_status	Operation status code
 */
static ret_status time_to_target_speed_same_dir(const int32_t vi, const int32_t vf, const int32_t accel, const int32_t decel, int32_t *result)
{
	if((accel == 0) || (decel == 0)){
		return (STATUS_INVALID_ARG);	/* Avoid division by zero */
	}
	if(vi == vf){
		*result = 0;
		return (STATUS_OK);
	}

	int64_t temp = (int64_t)SCALE * (int64_t)abs(vf - vi);
	int64_t denom = 0;
	if(abs(vf) > abs(vi)){	/* Accelerating */
		denom = (int64_t)abs(accel);
	}else{			/* Decelerating */
		denom = (int64_t)abs(decel);
	}
	int64_t time = temp / denom;
	if((temp % denom) != 0){
		time++;		/* Round away from zero */
	}
	/* Safety limits */
	if(time == 0){
    	time = 1;
    }
    if(time > INT32_MAX){
		return (STATUS_OUT_OF_RANGE);
	}
    *result = (int32_t)time;
    return (STATUS_OK);
}

/**
  * @brief Calculate the acceleration or deceleration distance according the velocity difference
  *
  * @param vi 			Initial velocity in steps/seg
  * @param vf 			Final velocity in steps/seg
  * @param accel 		acceleration in steps/seg^2
  * @param result   	Pointer to store the computed distance
  *
  * @retval ret_status	Operation status code
  */
static ret_status abs_distance_to_target_velocity(const int32_t vi, const int32_t vf, const int32_t accel, const int32_t decel, int32_t *result)
{
	if((accel == 0) || (decel == 0)){
		return (STATUS_INVALID_ARG);	/* Avoid division by zero */
	}
    if(vi == vf){
    	return (STATUS_NO_OP);
    }

    int64_t denom_accel = (int64_t)2 * (int64_t)abs(accel);
    int64_t denom_decel = (int64_t)2 * (int64_t)abs(decel);
    int64_t distance = 0;

    /* Opposite direction */
    if (is_opposite_direction(vi, vf) == true) {

    	/* Deceleration phase */
    	int64_t temp = (0*0 - (int64_t)vi * (int64_t)vi);
        int64_t d1 = temp / denom_decel;
        if((temp % denom_decel) != 0){
        	d1--;		/* Round away from zero */
        }
        d1 *= (int64_t)-1;

        /* Acceleration phase */
        temp = ((int64_t)vf * (int64_t)vf - 0*0);
        int64_t d2 = temp / denom_accel;
        if((temp % denom_accel) != 0){
           	d2++;		/* Round away from zero */
        }
        distance = d1 + d2;

    } else {
    /* Same direction */
    	int64_t temp = ((int64_t)vf * (int64_t)vf) - ((int64_t)vi * (int64_t)vi);
    	if(abs(vf) > abs(vi)){	/* Accelerating */
    		distance = temp / denom_accel;
    		if((temp % denom_accel) != 0){
    			distance++;		/* Round away from zero */
    		}
    	}else{			/* Decelerating */
    		distance = temp / denom_decel;
    		if((temp % denom_decel) != 0){
    		   	distance--;		/* Round away from zero */
     		}
    	}
    }

    /* Safety limits */
    if(distance > INT32_MAX){
    	return (STATUS_OUT_OF_RANGE);
    }
    else if(distance < INT32_MIN){
    	return (STATUS_OUT_OF_RANGE);
    }else{
    	/* Do nothing */
    }
    *result = (int32_t)abs(distance);
    return (STATUS_OK);
}

/**
  * @brief Calculate the maximum speed required to travel a distance in the given time
  *
  * @param delta_pos 		Initial velocity in steps/seg
  * @param T_total_ms 		Final velocity in steps/seg
  * @param A_acc 			acceleration in steps/seg^2
  * @param A_dec 			acceleration in steps/seg^2
  * @param result 			Pointer to store the computed time (ms).
  *
  * @retval ret_status	Operation status code
  */
static ret_status calculate_vmax_for_time(int32_t delta_pos, int32_t T_total_ms, int32_t A_acc, int32_t A_dec, int32_t *result) {

    if ((delta_pos <= 0) || (T_total_ms <= 0) || (A_acc <= 0) || (A_dec <= 0)){
    	return (STATUS_INVALID_ARG);
    }

    int64_t inv_sum = (int64_t)1000000000 / ((int64_t)2*(int64_t)A_acc) + (int64_t)1000000000 / ((int64_t)2*(int64_t)A_dec);
    int64_t temp = ((int64_t)delta_pos * (int64_t)1000000000) / (int64_t)(inv_sum);
    int64_t Vm_max_possible = (int32_t)isqrt64((uint64_t)temp);
    int64_t T_min = Vm_max_possible * 1000000 / A_acc + Vm_max_possible * 1000000 / A_dec;
    T_min = T_min / (int64_t)1000;

    if (T_total_ms < T_min) {
    	return (STATUS_OUT_OF_RANGE);
    }
    if (T_total_ms == T_min) {
    	*result = Vm_max_possible;  /* Tringular profile */
    	return (STATUS_OK);
    }

    int64_t k = inv_sum;
    int64_t a = k / (int64_t)1000;
    int64_t b = (int64_t)-T_total_ms;
    int64_t c = (int64_t)delta_pos;

    int64_t discriminant = b*b - 4*a*c;
    if (discriminant < 0) {
    	return (STATUS_OUT_OF_RANGE);
    }

    int64_t sqrt_d = isqrt64(discriminant * (int64_t)1024);

    /* Two solutions */
    int64_t Vmax1 = ( -b*(int64_t)32 + sqrt_d) * (int64_t)1000 / ( (int64_t)2*a);
    int64_t Vmax2 = ( -b*(int64_t)32 - sqrt_d) * (int64_t)1000 / ( (int64_t)2*a);
    Vmax1 /= (int64_t)32;
    Vmax2 /= (int64_t)32;

    int64_t candidates[2] = {Vmax2, Vmax1};

    for (int i = 0; i < 2; i++) {
        int64_t Vm = candidates[i];
        if (Vm <= 0) {
        	continue;
        }
        int64_t D_acc = (Vm * Vm) / ((int64_t)2 * A_acc);
        int64_t D_dec = (Vm * Vm) / ((int64_t)2 * A_dec);
        int64_t D_ramps = D_acc + D_dec;
        if (D_ramps <= delta_pos) {
        	*result = Vm;
        	return (STATUS_OK);
        }
    }
    /* Fallback: triangular profile*/
    *result = Vm_max_possible;
    return (STATUS_OK);
}

/**
  * @brief Start a synchronized motion
  *
  * @param None
  *
  * @retval ret_status	Operation status code
  */
ret_status start_synchronized_position(){

	volatile MotorData_t *m[LIBRARY_MOTORS] = {NULL};

	m[0] = &motor0;
	m[1] = &motor1;
	m[2] = &motor2;
	m[3] = &motor3;
	m[4] = &motor4;

	int32_t master = m[0]->abs_distance;
	int32_t index = 0;
	/* Find the largest distance */
	for(int32_t i = 0; i < LIBRARY_MOTORS; i++){
		if(m[i]->abs_distance > master){
			master = m[i]->abs_distance;
			index = i;
		}
	}
	if((master > 0) && (global_synchronized_speed > 0)){

		if(m[index]->current_velocity != 0){
			return (STATUS_BUSY);	/* Case Vi != 0 not implemented yet */
		}

		ret_status res = set_motor_position(m[index], m[index]->target_sync_pos, global_synchronized_speed);
		if(res != STATUS_OK){
			return (res);
		}
		int32_t t_total = m[index]->t_total_ms;
		travel_time_sync = t_total;

		int32_t vmax = 0;
		for(int32_t i = 0; i < LIBRARY_MOTORS; i++){
			if((i != index) && (m[i]->abs_distance > 0)){

				if(m[i]->current_velocity != 0){
					return (STATUS_BUSY);	/* Case Vi != 0 not implemented yet */
				}
				res = calculate_vmax_for_time(m[i]->abs_distance, t_total, m[i]->acceleration, m[i]->deceleration, &vmax);
				if(res != STATUS_OK){
					return (res);
				}
				res = set_motor_position(m[i], m[i]->target_sync_pos, vmax);
				if(res != STATUS_OK){
					return (res);
				}
			}
		}
		return (STATUS_OK);
	}else{
		return (STATUS_INVALID_ARG);
	}
}

/**
  * @brief Set the target position for a single motor in synchronized mode
  *
  * @param motor			Pointer to the MotorData_t structure containing motor parameters
  * @param target_position 	Target position
  *
  * @retval ret_status		Operation status code
  */
ret_status set_synchronized_position(volatile MotorData_t *motor, int32_t position){

	motor->abs_distance = abs(position - motor->current_position);
	motor->target_sync_pos = position;

	return (STATUS_OK);
}

/**
  * @brief Set the speed limit in synchronized mode
  *
  * @param speed 	Speed limit
  *
  * @retval ret_status	Operation status code
  */
ret_status set_vmax_synchronized_position(int32_t speed){

	if(speed > 0){
		global_synchronized_speed = speed;
		return STATUS_OK;
	}else{
		return STATUS_INVALID_ARG;
	}
}

/**
  * @brief Set the target position and time for a single motor
  *
  * @param motor 			Pointer to the MotorData_t structure containing motor parameters
  * @param target_position 	Target position in steps per second (step/s)
  * @param time 			Travel time in ms
  *
  * @retval ret_status	Operation status code
  */
ret_status set_motor_position_time(volatile MotorData_t *motor, int32_t new_position, int32_t time){

	if(motor->current_velocity != 0){
		return (STATUS_BUSY);	/* Case Vi != 0 not implemented yet */
	}
	int32_t target_position = new_position;
	int32_t vmax = 0;
	int32_t abs_distance = abs(target_position - motor->current_position);
	ret_status res = calculate_vmax_for_time(abs_distance, time, motor->acceleration, motor->deceleration, &vmax);
	if(res != STATUS_OK){
		return (res);
	}
	if(motor->position_mode == RELATIVE_POSITION){
		target_position += motor->current_position;
	}
	return (set_motor_position(motor, target_position, vmax));
}

/**
  * @brief Set the target position and speed for a single motor
  *
  * @param motor 			Pointer to the MotorData_t structure containing motor parameters
  * @param target_position  Target position in steps per second (step/s)
  * @param speed			Limit speed in steps/s. This speed must be positive for motion consistency
  *
  * @retval ret_status	Operation status code
  */
ret_status set_motor_position(volatile MotorData_t *motor, int32_t new_position, int32_t speed){

	int32_t current_position = motor->current_position;
	int32_t temp_current_velocity = motor->current_velocity;
	motor->final_velocity = 0;	/* Single trajectory */

	ret_status res = STATUS_OK;

	int32_t target_vel = abs(speed);	/* The value must be positive at beginning for movement consistency */
	int32_t target_position = new_position;

	if(motor->position_mode == RELATIVE_POSITION){
		target_position += current_position;
	}

	if(target_vel == 0){
		return (STATUS_NO_OP);
	}
	if(motor->deceleration == 0){
		return (STATUS_INVALID_ARG);
	}
	if(motor->acceleration == 0){
		return (STATUS_INVALID_ARG);
	}

	int32_t delta_pos_abs = abs(target_position - current_position);
	if(delta_pos_abs == 0){
		return (STATUS_NO_OP);
	}

	if(target_position < current_position){
		target_vel *= (int32_t)-1;
	}
	motor->last_velocity = target_vel;

	int32_t t_const = 0;
	int32_t temp_distanceA = 0;
	int32_t temp_distanceB = 0;
	int32_t temp_time = 0;
	bool opposite_direction = false;
	motor->t_total_ms = 0;

	int32_t tempFv = 0;
	/* Check if motor->final_velocity implies a change of direction */
	if(is_opposite_direction(target_vel, motor->final_velocity) == true){
		tempFv = 0;
	}else{
		tempFv = motor->final_velocity;
	}

	opposite_direction = is_opposite_direction(target_vel, motor->current_velocity);
	if(opposite_direction == true){
		res = abs_distance_to_target_velocity(temp_current_velocity, 0, motor->acceleration, motor->deceleration, &temp_distanceA);
		if(res != STATUS_OK){
			return res;
		}
		if(temp_current_velocity > 0){
			motor->target_position = current_position + temp_distanceA;
		}else{
			motor->target_position = current_position - temp_distanceA;
		}
		res = time_to_target_speed_same_dir(temp_current_velocity, 0, motor->acceleration, motor->deceleration, &temp_time);
		if(res != STATUS_OK){
			return res;
		}
		res = set_ramp_dda(motor, 0, temp_time);
		if(res != STATUS_OK){
			return res;
		}
		motor->t_total_ms += temp_time;
		temp_current_velocity = 0;
		delta_pos_abs = abs(motor->target_position - target_position);	/* New delta position is required */
	}

	res = abs_distance_to_target_velocity(temp_current_velocity, target_vel, motor->acceleration, motor->deceleration, &temp_distanceA);
	if(res != STATUS_OK){
		return res;
	}
	res = abs_distance_to_target_velocity(target_vel, tempFv, motor->acceleration, motor->deceleration, &temp_distanceB);
	if(res != STATUS_OK){
		return res;
	}
	int32_t accel_decel_distance = temp_distanceA + temp_distanceB;

	if(accel_decel_distance >= delta_pos_abs){
		/* Triangular profile */
		int32_t new_vmax = 0;
		res = vmax_general(motor->acceleration, motor->deceleration, temp_current_velocity, tempFv, delta_pos_abs, &new_vmax);
		if(res != STATUS_OK){
			return res;
		}
		if(target_vel < 0){
			new_vmax *= -1;
		}
		/* Half profile */
		if(abs(new_vmax) <= abs(temp_current_velocity)){

			res = abs_distance_to_target_velocity(temp_current_velocity, tempFv, motor->acceleration, motor->deceleration, &temp_distanceB);
			if(res != STATUS_OK){
				return res;
			}
			if(temp_distanceB <= delta_pos_abs){
				res = time_to_target_speed_same_dir(temp_current_velocity, tempFv, motor->acceleration, motor->deceleration, &temp_time);
				if(res != STATUS_OK){
					return res;
				}
				motor->t_accel_ms = 0;
				motor->t_decel_ms = temp_time;
				motor->target_position = target_position;
				res = set_ramp_dda(motor, tempFv, motor->t_decel_ms);
				if(res != STATUS_OK){
					return res;
				}
				motor->working_phase = WAITING_FINAL_POSITION;
			}else{
				/* Emergency half profile */
				int32_t new_decel = 0;
				res = get_deceleration_value(temp_current_velocity, tempFv, delta_pos_abs, &new_decel);
				if(res != STATUS_OK){
					return res;
				}
				if(new_decel > motor->emergency_deceleration){
					new_decel = motor->emergency_deceleration;
				}
				res = time_to_target_speed_same_dir(temp_current_velocity, tempFv, motor->acceleration, new_decel, &temp_time);
				if(res != STATUS_OK){
					return res;
				}
				motor->t_accel_ms = 0;
				motor->t_decel_ms = temp_time;
				res = set_ramp_dda(motor, tempFv, motor->t_decel_ms);
				if(res != STATUS_OK){
					return res;
				}
				motor->target_position = target_position;
				motor->working_phase = WAITING_FINAL_POSITION;
			}
		}else{
			/* Regular triangular profile */
			res = time_to_target_speed_same_dir(temp_current_velocity, new_vmax, motor->acceleration, motor->deceleration, &temp_time);
			if(res != STATUS_OK){
				return res;
			}
			motor->t_accel_ms = temp_time;
			res = time_to_target_speed_same_dir(new_vmax, tempFv, motor->acceleration, motor->deceleration, &temp_time);
			if(res != STATUS_OK){
				return res;
			}
			motor->t_decel_ms = temp_time;
			res = abs_distance_to_target_velocity(new_vmax, tempFv, motor->acceleration, motor->deceleration, &temp_distanceB);
			if(res != STATUS_OK){
				return res;
			}
			if(new_vmax > 0){
				motor->aux_pos2 = target_position - temp_distanceB;
			}else{
				motor->aux_pos2 = target_position + temp_distanceB;
			}
			if(opposite_direction == false){
				motor->target_position = motor->aux_pos2;
				if(current_position == motor->target_position){
					if(new_vmax > 0){
						motor->target_position++;
					}else{
						motor->target_position--;
					}
				}
				res = set_ramp_dda(motor, new_vmax, motor->t_accel_ms);
				if(res != STATUS_OK){
					return res;
				}
				motor->working_phase = WAITING_FINAL_SEGMENT;
			}else{
				motor->pending_velocity = new_vmax;
				motor->working_phase = WAITING_ZERO_CROSS_POSITION;
			}
		}
	}else{
		/* Trapezoidal profile */
		t_const = (delta_pos_abs - accel_decel_distance) * (int32_t)SCALE / abs(target_vel);	/* Time at constant speed */

		res = time_to_target_speed_same_dir(temp_current_velocity, target_vel, motor->acceleration, motor->deceleration, &temp_time);
		if(res != STATUS_OK){
			return res;
		}
		motor->t_accel_ms = temp_time;
		res = time_to_target_speed_same_dir(target_vel, tempFv, motor->acceleration, motor->deceleration, &temp_time);
		if(res != STATUS_OK){
			return res;
		}
		motor->t_decel_ms = temp_time;
		res = abs_distance_to_target_velocity(target_vel, tempFv, motor->acceleration, motor->deceleration, &temp_distanceB);
		if(res != STATUS_OK){
			return res;
		}
		if(target_vel > 0){
			motor->aux_pos2 = target_position - temp_distanceB;
		}else{
			motor->aux_pos2 = target_position + temp_distanceB;
		}
		motor->target_position = motor->aux_pos2;
		if(opposite_direction == false){
			res = set_ramp_dda(motor, target_vel, motor->t_accel_ms);
			if(res != STATUS_OK){
				return res;
			}
			motor->working_phase = WAITING_FINAL_SEGMENT;
		}else{
			motor->pending_velocity = target_vel;
			motor->working_phase = WAITING_ZERO_CROSS_POSITION;
		}
	}
	motor->t_total_ms += motor->t_accel_ms;
	motor->t_total_ms += motor->t_decel_ms;
	motor->t_total_ms += t_const;
	motor->planner_final_position = target_position;
	motor->target_pos_reached = 0;
	motor->working_status = EXECUTING_POSITION_COMMAND;

	return res;
}

/**
  * @brief Set the target velocity for a motor with configurable ramp
  *
  * @param motor 		Pointer to the MotorData_t structure containing motor parameters
  * @param velocity 	Target velocity in steps per second (steps/s)
  * @param ramp 		Time in milliseconds (ms) to reach the target velocity
  *
  * @retval ret_status	Operation status code
  */
ret_status set_motor_velocity_according_time_ramp(volatile MotorData_t *motor, int32_t velocity, uint32_t ramp){

	ret_status res = set_ramp_dda(motor, velocity, ramp);
	return (res);
}

/**
  * @brief Sets the target velocity for a motor according its acceleration or deceleration value
  *
  * @param motor 		Pointer to the MotorData_t structure containing motor parameters
  * @param velocity 	Target velocity in steps per second (steps/s)
  *
  * @retval ret_status	Operation status code
  */
ret_status set_motor_velocity_according_accel_decel(volatile MotorData_t *motor, int32_t velocity){

	ret_status res = STATUS_INVALID_ARG;
	int32_t temp = 0;

	if(is_opposite_direction(velocity, motor->current_velocity) == true){
		res = time_to_target_speed_same_dir(motor->current_velocity, 0, motor->acceleration, motor->deceleration, &temp);
		if(res == STATUS_OK){
			res = set_motor_velocity_according_time_ramp(motor, 0, (uint32_t)temp);
			if(res == STATUS_OK){
				res = time_to_target_speed_same_dir(0, velocity, motor->acceleration, motor->deceleration, &temp);
				if(res == STATUS_OK){
					motor->t_accel_ms = temp;
					motor->pending_velocity = velocity;
					motor->working_status = EXECUTING_DUAL_VELOCITY_COMMAND;
				}
			}
		}
	}else{
		res = time_to_target_speed_same_dir(motor->current_velocity, velocity, motor->acceleration, motor->deceleration, &temp);
		if(res == STATUS_OK){
			res = set_motor_velocity_according_time_ramp(motor, velocity, (uint32_t)temp);
			if(res == STATUS_OK){
				motor->working_status = EXECUTING_VELOCITY_COMMAND;
			}
		}
	}
	return (res);
}

/**
  * @brief Set acceleration and deceleration values
  *
  * @param motor	Pointer to the MotorData_t structure containing motor parameters
  * @param value 	Value to set
  * @param type 	Type selector, acceleration, deceleration, or emergency deceleration
  *
  * @retval ret_status	Operation status code
  */
ret_status set_accel_decel_param(volatile MotorData_t *m, const int32_t value, const uint8_t type){

	ret_status res = STATUS_OK;

	int32_t val = abs(value);	/* The values must be positive for motion consistency */

	if(m != NULL){
		if(val != 0){
			switch (type){
				/* ------------------------------------ */
				case SET_ACCEL:
					if(val > m->acceleration_limit){
						val = m->acceleration_limit;
					}
					m->acceleration = val;
					break;
				/* ------------------------------------ */
				case SET_DECEL:
					if(val > m->deceleration_limit){
						val = m->deceleration_limit;
					}
					m->deceleration = val;
					break;
				/* ------------------------------------ */
				case SET_EMERGENCY_DECEL:
					if(val > m->deceleration_limit){
						val = m->deceleration_limit;
					}
					m->emergency_deceleration = val;
					break;
				/* ------------------------------------ */
				default:
					return (STATUS_FW_FAULT);
					break;
			}
			res = STATUS_OK;
		}else{
			res = STATUS_INVALID_ARG;
		}
	}else{
		res = STATUS_INVALID_MOTOR;
	}

	return (res);
}

/**
  * @brief Process user commands received via CAN or UART
  *
  * @param msg 		pointer to the command
  * @param len 		length of the command
  *
  * @retval ret_status	Operation status code
  */
ret_status process_command(const uint8_t* msg, const uint8_t len) {

	uint16_t tempU16;
	int32_t tempS32;

	ret_status res = STATUS_OK;

	uint8_t aux = msg[2];
	uint8_t motor = msg[1];
	volatile MotorData_t *m = get_motor_data_struct(motor);

	/* Command selector */
    switch (msg[0]) {

    	/* Enable or disable the drivers */
    	case ENABLE_DISABLE_MOTOR:
    		if(0xFFu == motor){		/* All motors */
    			if(TRUE == aux){
    				res = enable_all_motors();
				}else if(FALSE == aux){
					res = disable_all_motors();
				}else{
					res = STATUS_INVALID_ARG;
				}
    		}else{					/* Single motor */
				if(TRUE == aux){
					res = enable_single_motor(motor);
				}else if(FALSE == aux){
					res = disable_single_motor(motor);
				}else{
					res = STATUS_INVALID_ARG;
				}
    		}
			break;

        /* Set a motor velocity that accelerates according a time ramp */
    	case SET_MOTOR_VELOCITY_TIME_RAMP:
            if(m != NULL){
            	/* Target velocity, steps/s, signed 32bit */
                tempS32 = bytes_to_int32(msg+2);
				/* Time ramp, ms, unsigned 16bit */
				tempU16 = bytes_to_uint16(msg+6);
				res = set_motor_velocity_according_time_ramp(m, tempS32, (uint32_t)tempU16);
            }else{
            	res = STATUS_INVALID_MOTOR;
            }
            break;

        /* Set a motor velocity that accelerates according the acceleration value */
        case SET_MOTOR_VELOCITY_ACCEL_RAMP:
			if(m != NULL){
				/* Target velocity, steps/s, signed 32bit */
				tempS32 = bytes_to_int32(msg+2);
				res = set_motor_velocity_according_accel_decel(m, tempS32);
			}else{
				res = STATUS_INVALID_MOTOR;
			}
            break;

        /* Move the motor to a requested position and speed. The position could be absolute or relative according to SET_POSITION_MODE command */
        /* If a bigger speed is needed than UINT16_MAX steps/s, set the speed to 0 and the controller will use
         *  the default speed according to SET_SPEED_POSITION_MODE command */
        case SET_MOTOR_POSITION:
        	if(m != NULL){
				/* Target position in steps, signed 32bit */
				tempS32 = bytes_to_int32(msg+2);
				/* Speed limit in steps/s, unsigned 16bit. This value is positive for consistency */
				tempU16 = bytes_to_uint16(msg+6);
				if(tempU16 == 0){
					res = set_motor_position(m, tempS32, abs(m->speed_position_mode));
				}else{
					res = set_motor_position(m, tempS32, (int32_t)tempU16);
				}
			}else{
        	   	res = STATUS_INVALID_MOTOR;
        	}
        	break;

		/* Move the motor to a requested position and time. The position could be absolute or relative according to SET_POSITION_MODE command */
		case SET_MOTOR_POSITION_TIME:
			if(m != NULL){
				/* Target position in steps, signed 32bit */
				tempS32 = bytes_to_int32(msg+2);
				/* Time in ms, unsigned 16bit */
				tempU16 = bytes_to_uint16(msg+6);
				if(tempU16 > 0){
					res = set_motor_position_time(m, tempS32, (int32_t)tempU16);
				}
			}else{
				res = STATUS_INVALID_MOTOR;
			}
			break;
		/* Set a synchronized position for a single motor */
        case SET_SYNCHRONIZED_POSITION:
        	if(m != NULL){
        		tempS32 = bytes_to_int32(msg+2);
        		res = set_synchronized_position(m, tempS32);
			}else{
				res = STATUS_INVALID_MOTOR;
			}
        	break;

        /* Set the speed for a synchronized movement */
        case SET_SYNCHRONIZED_SPEED:
        	tempS32 = bytes_to_int32(msg+1);
        	res = set_vmax_synchronized_position(tempS32);
        	break;

        /* Start a synchronized movement according the requested motor positions and speed limit */
        case START_SYNCHRONIZED_POSITION:
        	if(msg[1] == 1){
        		res = start_synchronized_position();
        	}else{
        		return STATUS_INVALID_ARG;
        	}
        	break;

		/* Set the maximum speed when moving a single motor to a specific position */
		case SET_SPEED_POSITION_MODE:
			if(m != NULL){
				tempS32 = bytes_to_int32(msg+2);
				if(tempS32 != 0){
					m->speed_position_mode = abs(tempS32);	/* The speed must be positive for consistency */
					res = STATUS_OK;
				}else{
					res = STATUS_INVALID_ARG;
				}
			}else{
				res = STATUS_INVALID_MOTOR;
			}
			break;

		/* Set the acceleration value */
        case SET_ACCEL:
        	res = set_accel_decel_param(m, bytes_to_int32(msg+2), SET_ACCEL);
        	break;

		/* Set the deceleration value */
		case SET_DECEL:
			res = set_accel_decel_param(m, bytes_to_int32(msg+2), SET_DECEL);
			break;

		/* Set the emergency deceleration value */
		case SET_EMERGENCY_DECEL:
			res = set_accel_decel_param(m, bytes_to_int32(msg+2), SET_EMERGENCY_DECEL);
			break;

		/* Set absolute or relative position mode */
        case SET_POSITION_MODE:
        	if(m != NULL){
        		if((aux == ABSOLUTE_POSITION) || (aux == RELATIVE_POSITION)){
        			m->position_mode = aux;
        			res = STATUS_OK;
        		}else{
        			res = STATUS_INVALID_ARG;
        		}
			}else{
				res = STATUS_INVALID_MOTOR;
            }
        	break;

        /* Clear the position counter */
        case CLEAR_POSITION_COUNTER:
        	if(m != NULL){
				m->current_position = 0;
			}else{
				res = STATUS_INVALID_MOTOR;
            }
        	break;

        	/* Get position and speed */
        case GET_POSITION_AND_SPEED:
        	if(m != NULL){
        		send_data_response(m->CAN_Id, m->current_position, m->current_velocity);
        		res = STATUS_OK;
			}else{
				res = STATUS_INVALID_MOTOR;
			}
        	break;

        default:
        	break;
    }

    return (res);
}

/**
  * @brief Get the struct pointer according the motor number
  *
  * @param motor	motor id
  *
  * @retval *MotorData_t	Resulting pointer, NULL if not exists
  */
volatile MotorData_t *get_motor_data_struct(const uint8_t motor){

	volatile MotorData_t *m = NULL;

	switch (motor){
		case 0: m = &motor0; break;
		case 1: m = &motor1; break;
		case 2: m = &motor2; break;
		case 3: m = &motor3; break;
		case 4: m = &motor4; break;
		default: break;
	}

	return (m);
}

/**
  * @brief Enable a single motor base on its index, from 0 to 4
  *
  * @param motor 	motor id
  *
  * @retval ret_status	Operation status code
  */
ret_status enable_single_motor(const uint8_t motor){

	ret_status ret = STATUS_OK;

	if(motor >= LIBRARY_MOTORS){
		return (STATUS_INVALID_ARG);
	}

	switch (motor){

		case 0u:
			#if MOTOR0_ENABLED == 1
			LL_GPIO_ResetOutputPin(M0_EN_GPIO_PORT, M0_EN_PIN);
			#else
			ret = STATUS_INVALID_ARG;
			#endif
			break;

		case 1u:
			#if MOTOR1_ENABLED == 1
			LL_GPIO_ResetOutputPin(M1_EN_GPIO_PORT, M1_EN_PIN);
			#else
			ret = STATUS_INVALID_ARG;
			#endif
			break;

		case 2u:
			#if MOTOR2_ENABLED == 1
			LL_GPIO_ResetOutputPin(M2_EN_GPIO_PORT, M2_EN_PIN);
			#else
			ret = STATUS_INVALID_ARG;
			#endif
			break;

		case 3u:
			#if MOTOR3_ENABLED == 1
			LL_GPIO_ResetOutputPin(M3_EN_GPIO_PORT, M3_EN_PIN);
			#else
			ret = STATUS_INVALID_ARG;
			#endif
			break;

		case 4u:
			#if MOTOR4_ENABLED == 1
			LL_GPIO_ResetOutputPin(M4_EN_GPIO_PORT, M4_EN_PIN);
			#else
			ret = STATUS_INVALID_ARG;
			#endif
			break;

		default: ret = STATUS_INVALID_ARG; break;
	}
	return (ret);
}

/**
  * @brief disable a single motor base on its index, from 0 to 4
  *
  * @param motor	Motor id
  *
  * @retval ret_status	Operation status code
  */
ret_status disable_single_motor(const uint8_t motor){

	ret_status ret = STATUS_OK;

	if(motor >= LIBRARY_MOTORS){
		return (STATUS_INVALID_MOTOR);
	}

	switch (motor){

		case 0u:
			#if MOTOR0_ENABLED == true
			LL_GPIO_SetOutputPin(M0_EN_GPIO_PORT, M0_EN_PIN);
			#else
			ret = STATUS_INVALID_MOTOR;
			#endif
			break;

		case 1u:
			#if MOTOR1_ENABLED == true
			LL_GPIO_SetOutputPin(M1_EN_GPIO_PORT, M1_EN_PIN);
			#else
			ret = STATUS_INVALID_MOTOR;
			#endif
			break;

		case 2u:
			#if MOTOR2_ENABLED == true
			LL_GPIO_SetOutputPin(M2_EN_GPIO_PORT, M2_EN_PIN);
			#else
			ret = STATUS_INVALID_MOTOR;
			#endif
			break;

		case 3u:
			#if MOTOR3_ENABLED == true
			LL_GPIO_SetOutputPin(M3_EN_GPIO_PORT, M3_EN_PIN);
			#else
			ret = STATUS_INVALID_MOTOR;
			#endif
			break;

		case 4u:
			#if MOTOR4_ENABLED == true
			LL_GPIO_SetOutputPin(M4_EN_GPIO_PORT, M4_EN_PIN);
			#else
			ret = STATUS_INVALID_MOTOR;
			#endif
			break;

		default: ret = STATUS_INVALID_MOTOR; break;
	}
	return (ret);
}

/**
  * @brief Enable all active motors
  *
  * @param None
  *
  * @retval ret_status	Operation status code
  */
ret_status enable_all_motors(){

	ret_status ret = STATUS_INVALID_MOTOR;

#if MOTOR0_ENABLED == true
	RESET_GPIO(M0_EN_GPIO_PORT, M0_EN_PIN);
	ret = STATUS_OK;
#endif

#if MOTOR1_ENABLED == true
	RESET_GPIO(M1_EN_GPIO_PORT, M1_EN_PIN);
	ret = STATUS_OK;
#endif

#if MOTOR2_ENABLED == true
	RESET_GPIO(M2_EN_GPIO_PORT, M2_EN_PIN);
	ret = STATUS_OK;
#endif

#if MOTOR3_ENABLED == true
	RESET_GPIO(M3_EN_GPIO_PORT, M3_EN_PIN);
	ret = STATUS_OK;
#endif

#if MOTOR4_ENABLED == true
	RESET_GPIO(M4_EN_GPIO_PORT, M4_EN_PIN);
	ret = STATUS_OK;
#endif

	return ret;
}

/**
  * @brief Disable all active motors
  *
  * @param None
  *
  * @retval ret_status	Operation status code
  */
ret_status disable_all_motors(){

	ret_status ret = STATUS_INVALID_MOTOR;

#if MOTOR0_ENABLED == true
	LL_GPIO_SetOutputPin(M0_EN_GPIO_PORT, M0_EN_PIN);
	ret = STATUS_OK;
#endif

#if MOTOR1_ENABLED == true
	LL_GPIO_SetOutputPin(M1_EN_GPIO_PORT, M1_EN_PIN);
	ret = STATUS_OK;
#endif

#if MOTOR2_ENABLED == true
	LL_GPIO_SetOutputPin(M2_EN_GPIO_PORT, M2_EN_PIN);
	ret = STATUS_OK;
#endif

#if MOTOR3_ENABLED == true
	LL_GPIO_SetOutputPin(M3_EN_GPIO_PORT, M3_EN_PIN);
	ret = STATUS_OK;
#endif

#if MOTOR4_ENABLED == true
	LL_GPIO_SetOutputPin(M4_EN_GPIO_PORT, M4_EN_PIN);
	ret = STATUS_OK;
#endif

	return ret;
}

/**
  * @brief Get the absolute steps of motor0
  * @param None
  * @retval Absolute steps
  */
int32_t get_motor0_steps(){
	return (motor0.current_position);
}

/**
  * @brief Get the absolute steps of motor1
  * @param None
  * @retval Absolute steps
  */
int32_t get_motor1_steps(){
	return (motor1.current_position);
}

/**
  * @brief Get the absolute steps of motor2
  * @param None
  * @retval Absolute steps
  */
int32_t get_motor2_steps(){
	return (motor2.current_position);
}

/**
  * @brief Get the absolute steps of motor3
  * @param None
  * @retval Absolute steps
  */
int32_t get_motor3_steps(){
	return (motor3.current_position);
}

/**
  * @brief Get the absolute steps of motor4
  * @param None
  * @retval Absolute steps
  */
int32_t get_motor4_steps(){
	return (motor4.current_position);
}

/**
  * @brief Get the current velocity of motor0
  * @param None
  * @retval Velocity un steps/s
  */
int32_t get_motor0_current_velocity(){
	return (motor0.current_velocity);
}

/**
  * @brief Get the current velocity of motor1
  * @param None
  * @retval Velocity un steps/s
  */
int32_t get_motor1_current_velocity(){
	return (motor1.current_velocity);
}

/**
  * @brief Get the current velocity of motor2
  * @param None
  * @retval Velocity un steps/s
  */
int32_t get_motor2_current_velocity(){
	return (motor2.current_velocity);
}

/**
  * @brief Get the current velocity of motor3
  * @param None
  * @retval Velocity un steps/s
  */
int32_t get_motor3_current_velocity(){
	return (motor3.current_velocity);
}

/**
  * @brief Get the current velocity of motor4
  * @param None
  * @retval Velocity un steps/s
  */
int32_t get_motor4_current_velocity(){
	return (motor4.current_velocity);
}

/**
  * @brief Init data struc for each motor
  * @param None
  * @retval None
  */
int32_t get_aux_data_response(){
	return travel_time_sync;
}

/**
  * @brief Init data struc for each motor
  * @param None
  * @retval None
  */
void init_driver_controller(){

	memset((uint8_t*)&motor0, 0, sizeof(motor0));
	memset((uint8_t*)&motor1, 0, sizeof(motor1));
	memset((uint8_t*)&motor2, 0, sizeof(motor2));
	memset((uint8_t*)&motor3, 0, sizeof(motor3));
	memset((uint8_t*)&motor4, 0, sizeof(motor4));

	/* Default values */
	motor0.acceleration = (int32_t)M0_DEFAULT_ACCEL;
	motor1.acceleration = (int32_t)M1_DEFAULT_ACCEL;
	motor2.acceleration = (int32_t)M2_DEFAULT_ACCEL;
	motor3.acceleration = (int32_t)M3_DEFAULT_ACCEL;
	motor4.acceleration = (int32_t)M4_DEFAULT_ACCEL;

	motor0.deceleration = (int32_t)M0_DEFAULT_DECEL;
	motor1.deceleration = (int32_t)M1_DEFAULT_DECEL;
	motor2.deceleration = (int32_t)M2_DEFAULT_DECEL;
	motor3.deceleration = (int32_t)M3_DEFAULT_DECEL;
	motor4.deceleration = (int32_t)M4_DEFAULT_DECEL;

	motor0.emergency_deceleration = (int32_t)M0_DEFAULT_EMERGENCY_DECEL;
	motor1.emergency_deceleration = (int32_t)M1_DEFAULT_EMERGENCY_DECEL;
	motor2.emergency_deceleration = (int32_t)M2_DEFAULT_EMERGENCY_DECEL;
	motor3.emergency_deceleration = (int32_t)M3_DEFAULT_EMERGENCY_DECEL;
	motor4.emergency_deceleration = (int32_t)M4_DEFAULT_EMERGENCY_DECEL;

	motor0.speed_position_mode = (int32_t)M0_DEFAULT_SPEED_POSITION_MODE;
	motor1.speed_position_mode = (int32_t)M1_DEFAULT_SPEED_POSITION_MODE;
	motor2.speed_position_mode = (int32_t)M2_DEFAULT_SPEED_POSITION_MODE;
	motor3.speed_position_mode = (int32_t)M3_DEFAULT_SPEED_POSITION_MODE;
	motor4.speed_position_mode = (int32_t)M4_DEFAULT_SPEED_POSITION_MODE;

	motor0.position_limit_positive = (int32_t)M0_POSITION_LIMIT_P;
	motor1.position_limit_positive = (int32_t)M1_POSITION_LIMIT_P;
	motor2.position_limit_positive = (int32_t)M2_POSITION_LIMIT_P;
	motor3.position_limit_positive = (int32_t)M3_POSITION_LIMIT_P;
	motor4.position_limit_positive = (int32_t)M4_POSITION_LIMIT_P;

	motor0.position_limit_negative = (int32_t)M0_POSITION_LIMIT_N;
	motor1.position_limit_negative = (int32_t)M1_POSITION_LIMIT_N;
	motor2.position_limit_negative = (int32_t)M2_POSITION_LIMIT_N;
	motor3.position_limit_negative = (int32_t)M3_POSITION_LIMIT_N;
	motor4.position_limit_negative = (int32_t)M4_POSITION_LIMIT_N;

	motor0.speed_limit = (int32_t)M0_SPEED_LIMIT;
	motor1.speed_limit = (int32_t)M1_SPEED_LIMIT;
	motor2.speed_limit = (int32_t)M2_SPEED_LIMIT;
	motor3.speed_limit = (int32_t)M3_SPEED_LIMIT;
	motor4.speed_limit = (int32_t)M4_SPEED_LIMIT;

	motor0.acceleration_limit = (int32_t)M0_ACCEL_LIMIT;
	motor1.acceleration_limit = (int32_t)M1_ACCEL_LIMIT;
	motor2.acceleration_limit = (int32_t)M2_ACCEL_LIMIT;
	motor3.acceleration_limit = (int32_t)M3_ACCEL_LIMIT;
	motor4.acceleration_limit = (int32_t)M4_ACCEL_LIMIT;

	motor0.deceleration_limit = (int32_t)M0_DECEL_LIMIT;
	motor1.deceleration_limit = (int32_t)M1_DECEL_LIMIT;
	motor2.deceleration_limit = (int32_t)M2_DECEL_LIMIT;
	motor3.deceleration_limit = (int32_t)M3_DECEL_LIMIT;
	motor4.deceleration_limit = (int32_t)M4_DECEL_LIMIT;

	motor0.CAN_Id = (int32_t)M0_ID;
	motor1.CAN_Id = (int32_t)M1_ID;
	motor2.CAN_Id = (int32_t)M2_ID;
	motor3.CAN_Id = (int32_t)M3_ID;
	motor4.CAN_Id = (int32_t)M4_ID;
}
