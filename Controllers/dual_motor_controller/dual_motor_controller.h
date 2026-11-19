#pragma once

#include "motor_controller/motor_controller.h"
#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    DUAL_MOTOR_CONTROLLER_OK,
    DUAL_MOTOR_CONTROLLER_ERR
} DUAL_MOTOR_CONTROLLER_Status_t;

typedef struct
{
    MOTOR_CONTROLLER_t *left;
    MOTOR_CONTROLLER_t *right;

    bool enabled;
} DUAL_MOTOR_CONTROLLER_t;

DUAL_MOTOR_CONTROLLER_Status_t dual_motor_controller_init(DUAL_MOTOR_CONTROLLER_t *dual, MOTOR_CONTROLLER_t *left, MOTOR_CONTROLLER_t *right);

DUAL_MOTOR_CONTROLLER_Status_t dual_motor_controller_set_rpm(DUAL_MOTOR_CONTROLLER_t *dual, float left_rpm, float right_rpm);

DUAL_MOTOR_CONTROLLER_Status_t dual_motor_controller_update(DUAL_MOTOR_CONTROLLER_t *dual, uint32_t current_ms, float dt_seconds);

DUAL_MOTOR_CONTROLLER_Status_t dual_motor_controller_set_feedforward(DUAL_MOTOR_CONTROLLER_t *dual, bool enabled);

DUAL_MOTOR_CONTROLLER_Status_t dual_motor_controller_enable(DUAL_MOTOR_CONTROLLER_t *dual);

DUAL_MOTOR_CONTROLLER_Status_t dual_motor_controller_disable(DUAL_MOTOR_CONTROLLER_t *dual);
