#pragma once

#include <stddef.h>
#include <stdbool.h>

typedef enum {
    PID_OK,
    PID_ERR
} PID_Status_t;

typedef struct {
    float kp;
    float ki;
    float kd;

    float integral;
    float previous_measurement;
    float filtered_derivative;
    bool has_previous_measurement;

    float derivative_filter_tau;

    float output_min;
    float output_max;
} PID_t;

PID_Status_t pid_init(PID_t *pid, float kp, float ki, float kd,
                      float output_min, float output_max);
PID_Status_t pid_reset(PID_t *pid);
PID_Status_t pid_set_derivative_filter(PID_t *pid, float tau_seconds);

/* Existing API: exactly the same as pid_update_with_bias(..., 0.0f). */
PID_Status_t pid_update(PID_t *pid, float *output,
                        float setpoint, float measurement, float dt_seconds);

/* Additive actuator bias/feedforward in the same units as output.
 * Anti-windup considers bias + P + I + D against the FULL actuator limits.
 * output is the FINAL, saturated actuator command (not just PID correction).
 */
PID_Status_t pid_update_with_bias(PID_t *pid, float *output,
                                  float setpoint, float measurement,
                                  float dt_seconds, float bias);
