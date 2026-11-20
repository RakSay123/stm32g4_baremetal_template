#include "dc_motor.h"

DC_MOTOR_Status_t dc_motor_init(DC_MOTOR_t *motor)
{
	if (motor == NULL || motor->motor_driver == NULL || motor->motor_driver->ops == NULL) return DC_MOTOR_ERR;

	return DC_MOTOR_OK;
}

DC_MOTOR_Status_t dc_motor_set_speed(DC_MOTOR_t *motor, uint16_t speed)
{
	if (motor_driver_set_output(motor->motor_driver, motor->driver_channel, (uint32_t)speed) != MOTOR_DRIVER_OK) return DC_MOTOR_ERR;

	return DC_MOTOR_OK;
}

DC_MOTOR_Status_t dc_motor_set_direction(DC_MOTOR_t *motor, MOTOR_DRIVER_Direction_t direction)
{
	if (motor_driver_set_direction(motor->motor_driver, motor->driver_channel, direction) != MOTOR_DRIVER_OK) return DC_MOTOR_ERR;

	return DC_MOTOR_OK;
}

DC_MOTOR_Status_t dc_motor_set_speed_and_direction(DC_MOTOR_t *motor, MOTOR_DRIVER_Direction_t direction, uint16_t speed)
{
	if (dc_motor_set_direction(motor, direction) != DC_MOTOR_OK) return DC_MOTOR_ERR;
	if (dc_motor_set_speed(motor, speed) != DC_MOTOR_OK) return DC_MOTOR_ERR;

	return DC_MOTOR_OK;
}

DC_MOTOR_Status_t dc_motor_brake(DC_MOTOR_t *motor)
{
	if (motor_driver_brake(motor->motor_driver, motor->driver_channel) != MOTOR_DRIVER_OK) return DC_MOTOR_ERR;

	return DC_MOTOR_OK;
}

DC_MOTOR_Status_t dc_motor_coast(DC_MOTOR_t *motor)
{
	if (motor_driver_coast(motor->motor_driver, motor->driver_channel) != MOTOR_DRIVER_OK) return DC_MOTOR_ERR;

	return DC_MOTOR_OK;
}
