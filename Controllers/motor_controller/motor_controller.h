#pragma once

#include "dc_motor/dc_motor.h"
#include "rotary_encoder/rotary_encoder.h"
#include "pid/pid.h"
#include "motor_feedforward/motor_feedforward.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum {
    MOTOR_CONTROLLER_OK,
    MOTOR_CONTROLLER_ERR,
} MOTOR_CONTROLLER_Status_t;

typedef enum {
    MOTOR_CONTROLLER_FAULT_NONE = 0,
    MOTOR_CONTROLLER_FAULT_DIRECTION,
    MOTOR_CONTROLLER_FAULT_STALL,
    MOTOR_CONTROLLER_FAULT_ENCODER,
    MOTOR_CONTROLLER_FAULT_ACTUATOR
} MOTOR_CONTROLLER_Fault_t;

typedef struct {
    DC_MOTOR_t *motor;
    ROTARY_ENCODER_t *encoder;
    PID_t pid;

    float target_rpm;
    float measured_rpm;
    float error;
    float control_output;
    float feedforward_output;

    bool feedforward_enabled;
    bool enabled;

    MOTOR_CONTROLLER_Fault_t fault;
	uint32_t direction_fault_ms;
	uint32_t stall_fault_ms;
} MOTOR_CONTROLLER_t;

MOTOR_CONTROLLER_Status_t motor_controller_init(MOTOR_CONTROLLER_t *controller, float kp, float ki, float kd, float output_min, float output_max);

MOTOR_CONTROLLER_Status_t motor_controller_set_rpm(MOTOR_CONTROLLER_t *controller, float target_rpm);

MOTOR_CONTROLLER_Status_t motor_controller_set_rev_per_sec(MOTOR_CONTROLLER_t *controller, float target_rev_per_sec);

MOTOR_CONTROLLER_Status_t motor_controller_update(MOTOR_CONTROLLER_t *controller, uint32_t current_ms, float dt_seconds);

MOTOR_CONTROLLER_Status_t motor_controller_set_feedforward(MOTOR_CONTROLLER_t *controller, bool enabled);

MOTOR_CONTROLLER_Status_t motor_controller_disable(MOTOR_CONTROLLER_t *controller);

MOTOR_CONTROLLER_Status_t motor_controller_enable(MOTOR_CONTROLLER_t *controller);
