#include "tb6612fng_motor_driver.h"
#include <stddef.h>

static MOTOR_DRIVER_Status_t tb6612fng_motor_driver_set_direction(void *context, uint8_t channel, MOTOR_DRIVER_Direction_t direction)
{
	TB6612FNG_t *tb6612fng = (TB6612FNG_t *)context;

	TB6612FNG_Status_t status = tb6612fng_set_direction(tb6612fng, (TB6612FNG_Channel_t)channel, (TB6612FNG_Direction_t)direction);

	return (status == TB6612FNG_OK) ? MOTOR_DRIVER_OK : MOTOR_DRIVER_ERR;
}

static MOTOR_DRIVER_Status_t tb6612fng_motor_driver_set_output(void *context, uint8_t channel, uint32_t output)
{
	TB6612FNG_t *tb6612fng = (TB6612FNG_t *)context;

	TB6612FNG_Status_t status = tb6612fng_set_duty_cycle(tb6612fng, (TB6612FNG_Channel_t)channel, output);

	return (status == TB6612FNG_OK) ? MOTOR_DRIVER_OK : MOTOR_DRIVER_ERR;
}

static MOTOR_DRIVER_Status_t tb6612fng_motor_driver_brake(void *context, uint8_t channel)
{
	TB6612FNG_t *tb6612fng = (TB6612FNG_t *)context;

	TB6612FNG_Status_t status = tb6612fng_brake(tb6612fng, (TB6612FNG_Channel_t)channel);

	return (status == TB6612FNG_OK) ? MOTOR_DRIVER_OK : MOTOR_DRIVER_ERR;
}

static MOTOR_DRIVER_Status_t tb6612fng_motor_driver_coast(void *context, uint8_t channel)
{
	TB6612FNG_t *tb6612fng = (TB6612FNG_t *)context;

	TB6612FNG_Status_t status = tb6612fng_coast(tb6612fng, (TB6612FNG_Channel_t)channel);

	return (status == TB6612FNG_OK) ? MOTOR_DRIVER_OK : MOTOR_DRIVER_ERR;;
}

MOTOR_DRIVER_Status_t tb6612fng_motor_driver_bind(MOTOR_DRIVER_t *motor_driver, TB6612FNG_t *tb6612fng)
{
	if (motor_driver == NULL || tb6612fng == NULL) return MOTOR_DRIVER_ERR;

	static const MOTOR_DRIVER_Ops_t tb6612fng_motor_driver_ops = {
		.set_direction = tb6612fng_motor_driver_set_direction,
		.set_output = tb6612fng_motor_driver_set_output,
		.brake = tb6612fng_motor_driver_brake,
		.coast = tb6612fng_motor_driver_coast
	};

	motor_driver->context = tb6612fng;
	motor_driver->ops = &tb6612fng_motor_driver_ops;

	return MOTOR_DRIVER_OK;
}
