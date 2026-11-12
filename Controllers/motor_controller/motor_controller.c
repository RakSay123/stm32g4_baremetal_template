#include "motor_controller.h"
#include "motor_feedforward/motor_feedforward.h"
#include <math.h>
#include <stdint.h>
#include <stddef.h>

MOTOR_CONTROLLER_Status_t motor_controller_init(MOTOR_CONTROLLER_t *controller, float kp, float ki, float kd, float output_min, float output_max)
{
    if (controller == NULL || controller->motor == NULL || controller->encoder == NULL) return MOTOR_CONTROLLER_ERR;
    if (!isfinite(output_min) || !isfinite(output_max) || output_min < 0.0f || output_max > 65535.0f || output_min >= output_max) return MOTOR_CONTROLLER_ERR;

    controller->enabled = false;
    controller->feedforward_enabled = false;
    controller->target_rpm = 0.0f;
    controller->measured_rpm = 0.0f;
    controller->error = 0.0f;
    controller->control_output = 0.0f;
    controller->feedforward_output = 0.0f;

    if (dc_motor_init(controller->motor) != DC_MOTOR_OK || rotary_encoder_init(controller->encoder) != ROTARY_ENCODER_OK || pid_init(&controller->pid, kp, ki, kd, output_min, output_max) != PID_OK) return MOTOR_CONTROLLER_ERR;

    controller->enabled = true;
    return MOTOR_CONTROLLER_OK;
}

MOTOR_CONTROLLER_Status_t motor_controller_set_rpm(MOTOR_CONTROLLER_t *controller,
                                                    float target_rpm)
{
    if (controller == NULL || !isfinite(target_rpm) || target_rpm < 0.0f) return MOTOR_CONTROLLER_ERR;
    if (controller->feedforward_enabled)
    {
        float ignored;
        if (motor_feedforward_get_pwm(controller->ff_lut, target_rpm, &ignored) != MOTOR_FEEDFORWARD_OK) return MOTOR_CONTROLLER_ERR;
    }
    controller->target_rpm = target_rpm;
    return MOTOR_CONTROLLER_OK;
}

MOTOR_CONTROLLER_Status_t motor_controller_set_rev_per_sec(MOTOR_CONTROLLER_t *controller, float target_rev_per_sec)
{
    return motor_controller_set_rpm(controller, target_rev_per_sec * 60.0f);
}

MOTOR_CONTROLLER_Status_t motor_controller_set_feedforward(MOTOR_CONTROLLER_t *controller, bool enabled)
{
    if (controller == NULL) return MOTOR_CONTROLLER_ERR;

    if (enabled && controller->ff_lut == NULL) return MOTOR_CONTROLLER_ERR;

    if (enabled && controller->target_rpm > 0.0f)
    {
        float ignored;
        if (motor_feedforward_get_pwm(controller->ff_lut, controller->target_rpm, &ignored) != MOTOR_FEEDFORWARD_OK) return MOTOR_CONTROLLER_ERR;
    }
    if (controller->feedforward_enabled != enabled)
    {
        if (pid_reset(&controller->pid) != PID_OK) return MOTOR_CONTROLLER_ERR;
        controller->feedforward_enabled = enabled;
        controller->feedforward_output = 0.0f;
    }
    return MOTOR_CONTROLLER_OK;
}

MOTOR_CONTROLLER_Status_t motor_controller_update(MOTOR_CONTROLLER_t *controller, uint32_t current_ms, float dt_seconds)
{
    if (controller == NULL || controller->motor == NULL || controller->encoder == NULL) return MOTOR_CONTROLLER_ERR;
    if (!controller->enabled) return MOTOR_CONTROLLER_OK;
    if (!isfinite(dt_seconds) || dt_seconds <= 0.0f) return MOTOR_CONTROLLER_ERR;

    if (rotary_encoder_update_distance(controller->encoder) != ROTARY_ENCODER_OK || rotary_encoder_update_velocity(controller->encoder, current_ms) != ROTARY_ENCODER_OK) return MOTOR_CONTROLLER_ERR;

    const float measured_rpm = rotary_encoder_get_rpm(controller->encoder);
    if (!isfinite(measured_rpm)) return MOTOR_CONTROLLER_ERR;

    controller->measured_rpm = measured_rpm;
    controller->error = controller->target_rpm - measured_rpm;

    if (controller->target_rpm == 0.0f) {
        if (dc_motor_brake(controller->motor) != DC_MOTOR_OK || pid_reset(&controller->pid) != PID_OK) return MOTOR_CONTROLLER_ERR;
        controller->feedforward_output = 0.0f;
        controller->control_output = 0.0f;
        return MOTOR_CONTROLLER_OK;
    }

    float ff_pwm = 0.0f;
    if (controller->feedforward_enabled && motor_feedforward_get_pwm(controller->ff_lut, controller->target_rpm, &ff_pwm) != MOTOR_FEEDFORWARD_OK) return MOTOR_CONTROLLER_ERR;

    float combined_pwm = 0.0f;
    if (pid_update_with_bias(&controller->pid, &combined_pwm, controller->target_rpm, measured_rpm, dt_seconds, ff_pwm) != PID_OK) return MOTOR_CONTROLLER_ERR;
    if (!isfinite(combined_pwm) || combined_pwm < 0.0f || combined_pwm > 65535.0f) return MOTOR_CONTROLLER_ERR;

    const uint16_t pwm = (uint16_t)combined_pwm;
    if (dc_motor_set_speed_and_direction(controller->motor, MOTOR_DRIVER_FORWARD, pwm) != DC_MOTOR_OK) return MOTOR_CONTROLLER_ERR;

    controller->feedforward_output = ff_pwm;
    controller->control_output = (float)pwm;
    return MOTOR_CONTROLLER_OK;
}

MOTOR_CONTROLLER_Status_t motor_controller_disable(MOTOR_CONTROLLER_t *controller)
{
    if (controller == NULL || controller->motor == NULL) return MOTOR_CONTROLLER_ERR;
    controller->enabled = false;
    controller->control_output = 0.0f;
    controller->feedforward_output = 0.0f;

    return dc_motor_brake(controller->motor) == DC_MOTOR_OK ? MOTOR_CONTROLLER_OK : MOTOR_CONTROLLER_ERR;
}

MOTOR_CONTROLLER_Status_t motor_controller_enable(MOTOR_CONTROLLER_t *controller)
{
    if (controller == NULL) return MOTOR_CONTROLLER_ERR;
    controller->enabled = true;
    return MOTOR_CONTROLLER_OK;
}
