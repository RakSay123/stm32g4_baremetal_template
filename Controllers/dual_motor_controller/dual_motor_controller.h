#pragma once

#include "motor_controller/motor_controller.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum {
    DUAL_MOTOR_CONTROLLER_OK,
    DUAL_MOTOR_CONTROLLER_ERR,
} DUAL_MOTOR_CONTROLLER_Status_t;

typedef struct {
    MOTOR_CONTROLLER_t *motor1;
    MOTOR_CONTROLLER_t *motor2;
} DUAL_MOTOR_CONTROLLER_t;

DUAL_MOTOR_CONTROLLER_Status_t dual_motor_controller_init(DUAL_MOTOR_CONTROLLER_t *dual, MOTOR_CONTROLLER_t *motor1, MOTOR_CONTROLLER_t *motor2);

DUAL_MOTOR_CONTROLLER_Status_t dual_motor_controller_reset(DUAL_MOTOR_CONTROLLER_t *dual, uint32_t current_ms);

DUAL_MOTOR_CONTROLLER_Status_t dual_motor_controller_set_rpm(DUAL_MOTOR_CONTROLLER_t *dual, float target_rpm);

DUAL_MOTOR_CONTROLLER_Status_t dual_motor_controller_set_pwm(DUAL_MOTOR_CONTROLLER_t *dual, uint16_t pwm);

DUAL_MOTOR_CONTROLLER_Status_t dual_motor_controller_update(DUAL_MOTOR_CONTROLLER_t *dual, uint32_t current_ms, float dt_seconds);

DUAL_MOTOR_CONTROLLER_Status_t dual_motor_controller_update_open_loop(DUAL_MOTOR_CONTROLLER_t *dual, uint32_t current_ms, uint16_t pwm);

DUAL_MOTOR_CONTROLLER_Status_t dual_motor_controller_set_gains(DUAL_MOTOR_CONTROLLER_t *dual, float kp, float ki, float kd);

DUAL_MOTOR_CONTROLLER_Status_t dual_motor_controller_set_feedforward(DUAL_MOTOR_CONTROLLER_t *dual, bool enabled);

DUAL_MOTOR_CONTROLLER_Status_t dual_motor_controller_enable(DUAL_MOTOR_CONTROLLER_t *dual);

DUAL_MOTOR_CONTROLLER_Status_t dual_motor_controller_disable(DUAL_MOTOR_CONTROLLER_t *dual);
