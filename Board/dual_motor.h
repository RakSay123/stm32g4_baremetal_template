#pragma once

#include "rotary_encoder/rotary_encoder.h"
#include "dc_motor/dc_motor.h"
#include "motor_controller/motor_controller.h"

/* Second motor (TB6612FNG channel A) for dual motor control.
 * Motor 1 stays in device_instances.c; everything for motor 2 lives here.
 */

typedef enum {
	DUAL_MOTOR_OK,
	DUAL_MOTOR_ERR
} DUAL_MOTOR_Status_t;

/* Connects motor 2 to the shared TB6612FNG motor driver.
 * Call after tb6612fng_motor_driver_bind() and before motor_controller_init() on motor 2.
 */
DUAL_MOTOR_Status_t dual_motor_init(void);

ROTARY_ENCODER_t* board_get_motor2_encoder(void);

DC_MOTOR_t* board_get_motor2(void);

MOTOR_CONTROLLER_t* board_get_motor2_controller(void);
