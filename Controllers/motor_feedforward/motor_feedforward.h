#pragma once

typedef enum {
    MOTOR_FEEDFORWARD_OK,
    MOTOR_FEEDFORWARD_ERR
} MOTOR_FEEDFORWARD_Status_t;

MOTOR_FEEDFORWARD_Status_t motor_feedforward_get_pwm(float target_rpm, float *output_pwm);
