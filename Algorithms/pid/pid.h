#pragma once

#include <stddef.h>

typedef enum {
	PID_OK,
	PID_ERR,
} PID_Status_t;

typedef struct {
	float kp;
	float ki;
	float kd;

	float integral;

	float output_min;
	float output_max;
} PID_t;

PID_Status_t pid_init(PID_t *pid, float kp, float ki, float kd, float output_min, float output_max);

PID_Status_t pid_reset(PID_t *pid);

PID_Status_t pid_update(PID_t *pid, float *output, float setpoint, float measurement, float dt_seconds);
