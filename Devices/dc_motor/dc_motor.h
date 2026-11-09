#pragma once

#include <stddef.h>
#include "motor_driver/motor_driver.h"

typedef enum {
	DC_MOTOR_OK,
	DC_MOTOR_ERR
} DC_MOTOR_Status_t;

typedef struct {
	MOTOR_DRIVER_t *motor_driver;
	uint8_t driver_channel;
} DC_MOTOR_t;

#include "dc_motor.h"

DC_MOTOR_Status_t dc_motor_init(DC_MOTOR_t *motor);

DC_MOTOR_Status_t dc_motor_set_speed(DC_MOTOR_t *motor, uint16_t speed);

DC_MOTOR_Status_t dc_motor_set_direction(DC_MOTOR_t *motor, MOTOR_DRIVER_Direction_t direction);

DC_MOTOR_Status_t dc_motor_set_speed_and_direction(DC_MOTOR_t *motor, MOTOR_DRIVER_Direction_t direction, uint16_t speed);

DC_MOTOR_Status_t dc_motor_brake(DC_MOTOR_t *motor);

DC_MOTOR_Status_t dc_motor_coast(DC_MOTOR_t *motor);
