#pragma once

/* This module uses the measured LUT in motor_ff_lut.h.
 * This is calibrated for ONE motor / supply voltage / load condition.
 * If you change motor or battery conditions, re-characterize the LUT.
 */
typedef enum {
    MOTOR_FEEDFORWARD_OK,
    MOTOR_FEEDFORWARD_ERR
} MOTOR_FEEDFORWARD_Status_t;

/* Look up raw PWM for a target RPM using piecewise linear interpolation.
 * Zero RPM -> zero PWM.
 * Nonzero speeds below the lowest reliable LUT point (the 100-PWM row)
 * are rejected: the 75-PWM point was intermittent in the experiment.
 * Values above the top of the measured LUT are also rejected.
 * No extrapolation and no unsupported startup-boost logic.
 */
MOTOR_FEEDFORWARD_Status_t motor_feedforward_get_pwm(float target_rpm, float *output_pwm);
