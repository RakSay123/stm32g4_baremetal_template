/* PROVISIONAL open-loop feedforward LUT; validate before use. */
#pragma once
#include <stdint.h>

typedef struct { float rpm; uint16_t pwm; } MotorFFPoint;
static const MotorFFPoint motor_ff_lut[9] = {
    { 24.52f, 150 },
    { 36.26f, 200 },
    { 94.48f, 250 },
    { 117.72f, 300 },
    { 140.36f, 350 },
    { 162.91f, 400 },
    { 185.85f, 450 },
    { 208.95f, 500 },
    { 231.69f, 550 },
};
static const uint32_t motor_ff_lut_count = 9U;
