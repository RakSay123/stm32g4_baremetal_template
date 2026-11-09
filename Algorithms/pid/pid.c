#include "pid.h"

PID_Status_t pid_init(PID_t *pid, float kp, float ki, float kd, float output_min, float output_max)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->output_min = output_min;
	pid->output_max = output_max;

	pid->integral = 0.0f;

	return PID_OK;
}

PID_Status_t pid_reset(PID_t *pid)
{
	return PID_OK;
}

PID_Status_t pid_update(PID_t *pid, float *output, float setpoint, float measurement, float dt_seconds)
{
	float error = setpoint - measurement;

	float candidate_integral = pid->integral + error * dt_seconds;
	float candidate_p_term = pid->kp * error;
	float candidate_i_term = pid->ki * candidate_integral;
	float candidate_output = candidate_p_term + candidate_i_term;

	if (candidate_output > pid->output_max)
	{
		if (error < 0) pid->integral = candidate_integral;
	}
	else if (candidate_output < pid->output_min)
	{
		if (error > 0) pid->integral = candidate_integral;
	}
	else
	{
		pid->integral = candidate_integral;
	}

	float p_term = pid->kp * error;
	float i_term = pid->ki * pid->integral;

	*output = p_term + i_term;

	if (*output > pid->output_max) *output = pid->output_max;
	else if (*output < pid->output_min) *output = pid->output_min;

	return PID_OK;
}
