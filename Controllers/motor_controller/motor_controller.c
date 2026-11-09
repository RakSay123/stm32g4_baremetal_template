#include "motor_controller.h"
#include "uart/uart.h"

MOTOR_CONTROLLER_Status_t motor_controller_init(MOTOR_CONTROLLER_t *controller, float kp, float ki, float kd, float output_min, float output_max)
{
	MOTOR_CONTROLLER_Status_t status = MOTOR_CONTROLLER_OK;

	if (dc_motor_init(controller->motor) != DC_MOTOR_OK) status = MOTOR_CONTROLLER_ERR;
	if (rotary_encoder_init(controller->encoder) != ROTARY_ENCODER_OK) status = MOTOR_CONTROLLER_ERR;
	if (pid_init(&controller->pid, kp, ki, kd, output_min, output_max) != PID_OK) status = MOTOR_CONTROLLER_ERR;

	controller->target_rpm = 0.0f;
	controller->enabled = true;

	return status;
}


MOTOR_CONTROLLER_Status_t motor_controller_set_rpm(MOTOR_CONTROLLER_t *controller, float target_rpm)
{
	controller->target_rpm = target_rpm;

	return MOTOR_CONTROLLER_OK;
}

MOTOR_CONTROLLER_Status_t motor_controller_set_rev_per_sec(MOTOR_CONTROLLER_t *controller, float target_rev_per_sec)
{
	return motor_controller_set_rpm(controller, target_rev_per_sec * 60);
}

MOTOR_CONTROLLER_Status_t motor_controller_update(MOTOR_CONTROLLER_t *controller, uint32_t current_ms, float dt_seconds)
{
	if (controller == NULL) return MOTOR_CONTROLLER_ERR;
	if (!controller->enabled) return MOTOR_CONTROLLER_OK;

	if (rotary_encoder_update_distance(controller->encoder) != ROTARY_ENCODER_OK) return MOTOR_CONTROLLER_ERR;
	if (rotary_encoder_update_velocity(controller->encoder, current_ms) != ROTARY_ENCODER_OK) return MOTOR_CONTROLLER_ERR;

	float measured_rpm = rotary_encoder_get_rpm(controller->encoder);
	float control_output;
	if (pid_update(&controller->pid, &control_output, controller->target_rpm, measured_rpm, dt_seconds) != PID_OK) return MOTOR_CONTROLLER_ERR;

	dc_motor_set_speed_and_direction(controller->motor, MOTOR_DRIVER_FORWARD, control_output);

	controller->measured_rpm = measured_rpm;
	controller->error = controller->target_rpm - measured_rpm;
	controller->control_output = control_output;

	uart_write_int(USART2, current_ms);
	uart_write_str(USART2, ",");
	uart_write_float(USART2, controller->target_rpm);
	uart_write_str(USART2, ",");
	uart_write_float(USART2, controller->measured_rpm);
	uart_write_str(USART2, ",");
	uart_write_float(USART2, controller->error);
	uart_write_str(USART2, ",");
	uart_write_float(USART2, controller->control_output);
	uart_write_line(USART2, "");


	return MOTOR_CONTROLLER_OK;
}

MOTOR_CONTROLLER_Status_t motor_controller_disable(MOTOR_CONTROLLER_t *controller)
{
	controller->enabled = false;

	return MOTOR_CONTROLLER_OK;
}

MOTOR_CONTROLLER_Status_t motor_controller_enable(MOTOR_CONTROLLER_t *controller)
{
	controller->enabled = true;

	return MOTOR_CONTROLLER_OK;
}
