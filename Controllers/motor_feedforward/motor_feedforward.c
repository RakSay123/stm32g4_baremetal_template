#include "motor_feedforward.h"
#include <math.h>
#include <stddef.h>
#include <stdint.h>


#define MOTOR_FF_FIRST_RELIABLE_INDEX 1U

MOTOR_FEEDFORWARD_Status_t motor_feedforward_get_pwm(const MOTOR_FF_LUT_t *lut,
                                                    float target_rpm,
                                                    float *output_pwm)
{
    if (lut == NULL || lut->points == NULL) return MOTOR_FEEDFORWARD_ERR;
    if (output_pwm == NULL || !isfinite(target_rpm) || target_rpm < 0.0f) return MOTOR_FEEDFORWARD_ERR;

    if (target_rpm == 0.0f)
    {
        *output_pwm = 0.0f;
        return MOTOR_FEEDFORWARD_OK;
    }

    const MotorFFPoint *points = lut->points;
    const uint32_t count = lut->count;

    if (count <= MOTOR_FF_FIRST_RELIABLE_INDEX) return MOTOR_FEEDFORWARD_ERR;

    if (target_rpm < points[MOTOR_FF_FIRST_RELIABLE_INDEX].rpm || target_rpm > points[count - 1U].rpm) return MOTOR_FEEDFORWARD_ERR;

    if (target_rpm == points[MOTOR_FF_FIRST_RELIABLE_INDEX].rpm)
    {
        *output_pwm = (float)points[MOTOR_FF_FIRST_RELIABLE_INDEX].pwm;
        return MOTOR_FEEDFORWARD_OK;
    }

    for (uint32_t i = MOTOR_FF_FIRST_RELIABLE_INDEX + 1U; i < count; ++i)
    {
        const float low_rpm  = points[i - 1U].rpm;
        const float high_rpm = points[i].rpm;
        if (!(high_rpm > low_rpm)) return MOTOR_FEEDFORWARD_ERR;
        if (target_rpm <= high_rpm)
        {
            const float fraction = (target_rpm - low_rpm) / (high_rpm - low_rpm);
            *output_pwm = (float)points[i - 1U].pwm + fraction * ((float)points[i].pwm - (float)points[i - 1U].pwm);
            return MOTOR_FEEDFORWARD_OK;
        }
    }

    return MOTOR_FEEDFORWARD_ERR;
}
