#pragma once

#include "dc_motor/dc_motor.h"
#include "rotary_encoder/rotary_encoder.h"
#include "pid/pid.h"
#include <stdbool.h>

typedef enum {
	MOTOR_CONTROLLER_OK,
	MOTOR_CONTROLLER_ERR,
} MOTOR_CONTROLLER_Status_t;

typedef struct {
	DC_MOTOR_t *motor;
	ROTARY_ENCODER_t *encoder;
	PID_t pid;

	float target_rpm;
	float measured_rpm;
	float error;
	float control_output;

	bool enabled;
} MOTOR_CONTROLLER_t;

MOTOR_CONTROLLER_Status_t motor_controller_init(MOTOR_CONTROLLER_t *controller, float kp, float ki, float kd, float output_min, float output_max);

MOTOR_CONTROLLER_Status_t motor_controller_set_rpm(MOTOR_CONTROLLER_t *controller, float target_rpm);

MOTOR_CONTROLLER_Status_t motor_controller_set_rev_per_sec(MOTOR_CONTROLLER_t *controller, float target_rev_per_sec);

MOTOR_CONTROLLER_Status_t motor_controller_update(MOTOR_CONTROLLER_t *controller, uint32_t current_ms, float dt_seconds);

MOTOR_CONTROLLER_Status_t motor_controller_disable(MOTOR_CONTROLLER_t *controller);

MOTOR_CONTROLLER_Status_t motor_controller_enable(MOTOR_CONTROLLER_t *controller);
