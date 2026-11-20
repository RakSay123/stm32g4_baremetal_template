#include "motor_feedforward.h"
#include "motor_ff_lut.h"
#include <math.h>
#include <stddef.h>
#include <stdint.h>


#define MOTOR_FF_FIRST_RELIABLE_INDEX 1U

MOTOR_FEEDFORWARD_Status_t motor_feedforward_get_pwm(float target_rpm, float *output_pwm)
{
    if (output_pwm == NULL || !isfinite(target_rpm) || target_rpm < 0.0f) return MOTOR_FEEDFORWARD_ERR;

    if (target_rpm == 0.0f)
    {
        *output_pwm = 0.0f;
        return MOTOR_FEEDFORWARD_OK;
    }

    if (motor_ff_lut_count <= MOTOR_FF_FIRST_RELIABLE_INDEX) return MOTOR_FEEDFORWARD_ERR;

    if (target_rpm < motor_ff_lut[MOTOR_FF_FIRST_RELIABLE_INDEX].rpm || target_rpm > motor_ff_lut[motor_ff_lut_count - 1U].rpm) return MOTOR_FEEDFORWARD_ERR;

    if (target_rpm == motor_ff_lut[MOTOR_FF_FIRST_RELIABLE_INDEX].rpm)
    {
        *output_pwm = (float)motor_ff_lut[MOTOR_FF_FIRST_RELIABLE_INDEX].pwm;
        return MOTOR_FEEDFORWARD_OK;
    }

    for (uint32_t i = MOTOR_FF_FIRST_RELIABLE_INDEX + 1U; i < motor_ff_lut_count; ++i)
    {
        const float low_rpm  = motor_ff_lut[i - 1U].rpm;
        const float high_rpm = motor_ff_lut[i].rpm;
        if (!(high_rpm > low_rpm)) return MOTOR_FEEDFORWARD_ERR;
        if (target_rpm <= high_rpm)
        {
            const float fraction = (target_rpm - low_rpm) / (high_rpm - low_rpm);
            *output_pwm = (float)motor_ff_lut[i - 1U].pwm + fraction * ((float)motor_ff_lut[i].pwm - (float)motor_ff_lut[i - 1U].pwm);
            return MOTOR_FEEDFORWARD_OK;
        }
    }

    return MOTOR_FEEDFORWARD_ERR;
}
