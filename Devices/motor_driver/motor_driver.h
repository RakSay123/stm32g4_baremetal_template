#pragma once

#include <stdint.h>

typedef enum {
	MOTOR_DRIVER_OK,
	MOTOR_DRIVER_ERR
} MOTOR_DRIVER_Status_t;

typedef enum {
	MOTOR_DRIVER_FORWARD,
	MOTOR_DRIVER_REVERSE
} MOTOR_DRIVER_Direction_t;

typedef struct {
	MOTOR_DRIVER_Status_t (*set_direction)(void* context, uint8_t channel, MOTOR_DRIVER_Direction_t direction);
	MOTOR_DRIVER_Status_t (*set_output)(void* context, uint8_t channel, uint32_t output);
	MOTOR_DRIVER_Status_t (*brake)(void *context, uint8_t channel);
	MOTOR_DRIVER_Status_t (*coast)(void *context, uint8_t channel);
} MOTOR_DRIVER_Ops_t;

typedef struct {
	void *context;
	const MOTOR_DRIVER_Ops_t *ops;
} MOTOR_DRIVER_t;

MOTOR_DRIVER_Status_t motor_driver_set_direction(MOTOR_DRIVER_t *driver, uint8_t channel, MOTOR_DRIVER_Direction_t direction);

MOTOR_DRIVER_Status_t motor_driver_set_output(MOTOR_DRIVER_t *driver, uint8_t channel, uint32_t output);

MOTOR_DRIVER_Status_t motor_driver_brake(MOTOR_DRIVER_t *driver, uint8_t channel);

MOTOR_DRIVER_Status_t motor_driver_coast(MOTOR_DRIVER_t *driver, uint8_t channel);
