#include "motor_driver.h"
#include <stddef.h>

MOTOR_DRIVER_Status_t motor_driver_set_direction(MOTOR_DRIVER_t *driver, uint8_t channel, MOTOR_DRIVER_Direction_t direction)
{
	if (driver == NULL || driver->context == NULL || driver->ops == NULL || driver->ops->set_direction == NULL) return MOTOR_DRIVER_ERR;

	return driver->ops->set_direction(driver->context, channel, direction);
}

MOTOR_DRIVER_Status_t motor_driver_set_output(MOTOR_DRIVER_t *driver, uint8_t channel, uint32_t output)
{
	if (driver == NULL || driver->context == NULL || driver->ops == NULL || driver->ops->set_output == NULL) return MOTOR_DRIVER_ERR;

	return driver->ops->set_output(driver->context, channel, output);
}

MOTOR_DRIVER_Status_t motor_driver_brake(MOTOR_DRIVER_t *driver, uint8_t channel)
{
	if (driver == NULL || driver->context == NULL || driver->ops == NULL || driver->ops->brake == NULL) return MOTOR_DRIVER_ERR;

	return driver->ops->brake(driver->context, channel);
}

MOTOR_DRIVER_Status_t motor_driver_coast(MOTOR_DRIVER_t *driver, uint8_t channel)
{
	if (driver == NULL || driver->context == NULL || driver->ops == NULL || driver->ops->coast == NULL) return MOTOR_DRIVER_ERR;

	return driver->ops->coast(driver->context, channel);
}
