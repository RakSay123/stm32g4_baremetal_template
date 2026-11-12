#include "dual_motor_controller.h"
#include <math.h>
#include <stddef.h>

static bool dual_motor_controller_is_valid(const DUAL_MOTOR_CONTROLLER_t *dual)
{
    return dual != NULL && dual->motor1 != NULL && dual->motor2 != NULL;
}

static MOTOR_CONTROLLER_Status_t dual_motor_controller_reset_one(MOTOR_CONTROLLER_t *controller, uint32_t current_ms)
{
    if (controller->encoder == NULL) return MOTOR_CONTROLLER_ERR;
    if (pid_reset(&controller->pid) != PID_OK || rotary_encoder_set_count_zero(controller->encoder) != ROTARY_ENCODER_OK) return MOTOR_CONTROLLER_ERR;

    controller->encoder->previous_update_ms = current_ms;
    controller->measured_rpm = 0.0f;
    controller->error = 0.0f;
    controller->control_output = 0.0f;
    controller->feedforward_output = 0.0f;
    return MOTOR_CONTROLLER_OK;
}

static MOTOR_CONTROLLER_Status_t dual_motor_controller_set_pwm_one(MOTOR_CONTROLLER_t *controller, uint16_t pwm)
{
    if (controller->motor == NULL) return MOTOR_CONTROLLER_ERR;
    if (dc_motor_set_speed_and_direction(controller->motor, MOTOR_DRIVER_FORWARD, pwm) != DC_MOTOR_OK) return MOTOR_CONTROLLER_ERR;

    controller->control_output = (float)pwm;
    return MOTOR_CONTROLLER_OK;
}

static MOTOR_CONTROLLER_Status_t dual_motor_controller_update_open_loop_one(MOTOR_CONTROLLER_t *controller, uint32_t current_ms, uint16_t pwm)
{
    if (controller->encoder == NULL) return MOTOR_CONTROLLER_ERR;
    if (rotary_encoder_update_distance(controller->encoder) != ROTARY_ENCODER_OK || rotary_encoder_update_velocity(controller->encoder, current_ms) != ROTARY_ENCODER_OK) return MOTOR_CONTROLLER_ERR;

    const float measured_rpm = rotary_encoder_get_rpm(controller->encoder);
    if (!isfinite(measured_rpm)) return MOTOR_CONTROLLER_ERR;

    controller->measured_rpm = measured_rpm;
    controller->target_rpm = 0.0f;
    controller->error = -measured_rpm;
    controller->control_output = (float)pwm;
    return MOTOR_CONTROLLER_OK;
}

static MOTOR_CONTROLLER_Status_t dual_motor_controller_set_gains_one(MOTOR_CONTROLLER_t *controller, float kp, float ki, float kd)
{
    controller->pid.kp = kp;
    controller->pid.ki = ki;
    controller->pid.kd = kd;
    return pid_reset(&controller->pid) == PID_OK ? MOTOR_CONTROLLER_OK : MOTOR_CONTROLLER_ERR;
}

DUAL_MOTOR_CONTROLLER_Status_t dual_motor_controller_init(DUAL_MOTOR_CONTROLLER_t *dual, MOTOR_CONTROLLER_t *motor1, MOTOR_CONTROLLER_t *motor2)
{
    if (dual == NULL || motor1 == NULL || motor2 == NULL || motor1 == motor2) return DUAL_MOTOR_CONTROLLER_ERR;

    dual->motor1 = motor1;
    dual->motor2 = motor2;
    return DUAL_MOTOR_CONTROLLER_OK;
}

DUAL_MOTOR_CONTROLLER_Status_t dual_motor_controller_reset(DUAL_MOTOR_CONTROLLER_t *dual, uint32_t current_ms)
{
    if (!dual_motor_controller_is_valid(dual)) return DUAL_MOTOR_CONTROLLER_ERR;
    if (dual_motor_controller_reset_one(dual->motor1, current_ms) != MOTOR_CONTROLLER_OK || dual_motor_controller_reset_one(dual->motor2, current_ms) != MOTOR_CONTROLLER_OK) return DUAL_MOTOR_CONTROLLER_ERR;

    return DUAL_MOTOR_CONTROLLER_OK;
}

DUAL_MOTOR_CONTROLLER_Status_t dual_motor_controller_set_rpm(DUAL_MOTOR_CONTROLLER_t *dual, float target_rpm)
{
    if (!dual_motor_controller_is_valid(dual)) return DUAL_MOTOR_CONTROLLER_ERR;
    if (motor_controller_set_rpm(dual->motor1, target_rpm) != MOTOR_CONTROLLER_OK || motor_controller_set_rpm(dual->motor2, target_rpm) != MOTOR_CONTROLLER_OK) return DUAL_MOTOR_CONTROLLER_ERR;

    return DUAL_MOTOR_CONTROLLER_OK;
}

DUAL_MOTOR_CONTROLLER_Status_t dual_motor_controller_set_pwm(DUAL_MOTOR_CONTROLLER_t *dual, uint16_t pwm)
{
    if (!dual_motor_controller_is_valid(dual)) return DUAL_MOTOR_CONTROLLER_ERR;
    if (dual_motor_controller_set_pwm_one(dual->motor1, pwm) != MOTOR_CONTROLLER_OK || dual_motor_controller_set_pwm_one(dual->motor2, pwm) != MOTOR_CONTROLLER_OK) return DUAL_MOTOR_CONTROLLER_ERR;

    return DUAL_MOTOR_CONTROLLER_OK;
}

DUAL_MOTOR_CONTROLLER_Status_t dual_motor_controller_update(DUAL_MOTOR_CONTROLLER_t *dual, uint32_t current_ms, float dt_seconds)
{
    if (!dual_motor_controller_is_valid(dual)) return DUAL_MOTOR_CONTROLLER_ERR;
    if (motor_controller_update(dual->motor1, current_ms, dt_seconds) != MOTOR_CONTROLLER_OK || motor_controller_update(dual->motor2, current_ms, dt_seconds) != MOTOR_CONTROLLER_OK) return DUAL_MOTOR_CONTROLLER_ERR;

    return DUAL_MOTOR_CONTROLLER_OK;
}

DUAL_MOTOR_CONTROLLER_Status_t dual_motor_controller_update_open_loop(DUAL_MOTOR_CONTROLLER_t *dual, uint32_t current_ms, uint16_t pwm)
{
    if (!dual_motor_controller_is_valid(dual)) return DUAL_MOTOR_CONTROLLER_ERR;
    if (dual_motor_controller_update_open_loop_one(dual->motor1, current_ms, pwm) != MOTOR_CONTROLLER_OK || dual_motor_controller_update_open_loop_one(dual->motor2, current_ms, pwm) != MOTOR_CONTROLLER_OK) return DUAL_MOTOR_CONTROLLER_ERR;

    return DUAL_MOTOR_CONTROLLER_OK;
}

DUAL_MOTOR_CONTROLLER_Status_t dual_motor_controller_set_gains(DUAL_MOTOR_CONTROLLER_t *dual, float kp, float ki, float kd)
{
    if (!dual_motor_controller_is_valid(dual)) return DUAL_MOTOR_CONTROLLER_ERR;
    if (!isfinite(kp) || !isfinite(ki) || !isfinite(kd) || kp < 0.0f || ki < 0.0f || kd < 0.0f) return DUAL_MOTOR_CONTROLLER_ERR;
    if (dual_motor_controller_set_gains_one(dual->motor1, kp, ki, kd) != MOTOR_CONTROLLER_OK || dual_motor_controller_set_gains_one(dual->motor2, kp, ki, kd) != MOTOR_CONTROLLER_OK) return DUAL_MOTOR_CONTROLLER_ERR;

    return DUAL_MOTOR_CONTROLLER_OK;
}

DUAL_MOTOR_CONTROLLER_Status_t dual_motor_controller_set_feedforward(DUAL_MOTOR_CONTROLLER_t *dual, bool enabled)
{
    if (!dual_motor_controller_is_valid(dual)) return DUAL_MOTOR_CONTROLLER_ERR;

    const bool motor1_previous = dual->motor1->feedforward_enabled;
    if (motor_controller_set_feedforward(dual->motor1, enabled) != MOTOR_CONTROLLER_OK) return DUAL_MOTOR_CONTROLLER_ERR;
    if (motor_controller_set_feedforward(dual->motor2, enabled) != MOTOR_CONTROLLER_OK)
    {
        motor_controller_set_feedforward(dual->motor1, motor1_previous);
        return DUAL_MOTOR_CONTROLLER_ERR;
    }

    return DUAL_MOTOR_CONTROLLER_OK;
}

DUAL_MOTOR_CONTROLLER_Status_t dual_motor_controller_enable(DUAL_MOTOR_CONTROLLER_t *dual)
{
    if (!dual_motor_controller_is_valid(dual)) return DUAL_MOTOR_CONTROLLER_ERR;
    if (motor_controller_enable(dual->motor1) != MOTOR_CONTROLLER_OK || motor_controller_enable(dual->motor2) != MOTOR_CONTROLLER_OK) return DUAL_MOTOR_CONTROLLER_ERR;

    return DUAL_MOTOR_CONTROLLER_OK;
}

DUAL_MOTOR_CONTROLLER_Status_t dual_motor_controller_disable(DUAL_MOTOR_CONTROLLER_t *dual)
{
    if (!dual_motor_controller_is_valid(dual)) return DUAL_MOTOR_CONTROLLER_ERR;

    const MOTOR_CONTROLLER_Status_t motor1_status = motor_controller_disable(dual->motor1);
    const MOTOR_CONTROLLER_Status_t motor2_status = motor_controller_disable(dual->motor2);
    return (motor1_status == MOTOR_CONTROLLER_OK && motor2_status == MOTOR_CONTROLLER_OK) ? DUAL_MOTOR_CONTROLLER_OK : DUAL_MOTOR_CONTROLLER_ERR;
}
