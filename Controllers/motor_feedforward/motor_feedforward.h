#pragma once

#include <stdint.h>

/* Each motor has its OWN measured LUT (motor_ff_lut.h for motor 1,
 * motor2_ff_lut.h for motor 2). A LUT is calibrated for one motor /
 * supply voltage / load condition. If you change motor or battery
 * conditions, re-characterize that motor's LUT.
 */
typedef enum {
    MOTOR_FEEDFORWARD_OK,
    MOTOR_FEEDFORWARD_ERR
} MOTOR_FEEDFORWARD_Status_t;

/* One measured point: running at this raw PWM gave this RPM. */
typedef struct { float rpm; uint16_t pwm; } MotorFFPoint;

/* One motor's LUT: its measured points and how many there are. */
typedef struct {
    const MotorFFPoint *points;
    uint32_t count;
} MOTOR_FF_LUT_t;

/* Look up raw PWM for a target RPM in this motor's LUT using piecewise
 * linear interpolation.
 * Zero RPM -> zero PWM.
 * Nonzero speeds below the lowest reliable LUT point (the 100-PWM row)
 * are rejected: the 75-PWM point was intermittent in the experiment.
 * Values above the top of the measured LUT are also rejected.
 * No extrapolation and no unsupported startup-boost logic.
 */
MOTOR_FEEDFORWARD_Status_t motor_feedforward_get_pwm(const MOTOR_FF_LUT_t *lut, float target_rpm, float *output_pwm);
