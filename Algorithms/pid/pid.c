#include "pid.h"
#include <math.h>

#define PID_DEFAULT_D_FILTER_TAU 0.10f

static float pid_clamp(float value, float minimum, float maximum)
{
    if (value > maximum) return maximum;
    if (value < minimum) return minimum;
    return value;
}

PID_Status_t pid_init(PID_t *pid, float kp, float ki, float kd, float output_min, float output_max)
{
    if (pid == NULL) return PID_ERR;
    if (!isfinite(kp) || !isfinite(ki) || !isfinite(kd) || !isfinite(output_min) || !isfinite(output_max)) return PID_ERR;
    if (kp < 0.0f || ki < 0.0f || kd < 0.0f ||  output_min >= output_max) return PID_ERR;

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->output_min = output_min;
    pid->output_max = output_max;
    pid->derivative_filter_tau = PID_DEFAULT_D_FILTER_TAU;
    return pid_reset(pid);
}

PID_Status_t pid_reset(PID_t *pid)
{
    if (pid == NULL) return PID_ERR;
    pid->integral = 0.0f;
    pid->previous_measurement = 0.0f;
    pid->filtered_derivative = 0.0f;
    pid->has_previous_measurement = false;
    return PID_OK;
}

PID_Status_t pid_set_derivative_filter(PID_t *pid, float tau_seconds)
{
    if (pid == NULL || !isfinite(tau_seconds) || tau_seconds < 0.0f)
        return PID_ERR;
    pid->derivative_filter_tau = tau_seconds;
    return PID_OK;
}

PID_Status_t pid_update(PID_t *pid, float *output, float setpoint, float measurement, float dt_seconds)
{
    return pid_update_with_bias(pid, output, setpoint, measurement, dt_seconds, 0.0f);
}

PID_Status_t pid_update_with_bias(PID_t *pid, float *output, float setpoint, float measurement, float dt_seconds, float bias)
{
    if (pid == NULL || output == NULL) return PID_ERR;
    if (!isfinite(setpoint) || !isfinite(measurement) || !isfinite(dt_seconds) || dt_seconds <= 0.0f || !isfinite(bias)) return PID_ERR;

    const float error = setpoint - measurement;
    const float p_term = pid->kp * error;
    float d_term = 0.0f;

    if (pid->kd != 0.0f) {
        if (pid->has_previous_measurement) {
            const float raw_derivative = (measurement - pid->previous_measurement) / dt_seconds;
            const float alpha = dt_seconds / (pid->derivative_filter_tau + dt_seconds);
            pid->filtered_derivative += alpha * (raw_derivative - pid->filtered_derivative);
        }
        d_term = -pid->kd * pid->filtered_derivative;
    }

    pid->previous_measurement = measurement;
    pid->has_previous_measurement = true;

    const float candidate_integral = pid->integral + error * dt_seconds;
    const float candidate_output = bias + p_term + pid->ki * candidate_integral + d_term;

    if (!isfinite(candidate_output)) return PID_ERR;

    if (candidate_output > pid->output_max)
    {
        if (error < 0.0f) pid->integral = candidate_integral;
    }
    else if (candidate_output < pid->output_min)
    {
        if (error > 0.0f) pid->integral = candidate_integral;
    }
    else
    {
        pid->integral = candidate_integral;
    }

    const float combined_output = bias + p_term + pid->ki * pid->integral + d_term;
    if (!isfinite(combined_output)) return PID_ERR;

    *output = pid_clamp(combined_output, pid->output_min, pid->output_max);
    return PID_OK;
}
