/* PROVISIONAL open-loop feedforward LUT; validate before use. */
#pragma once
#include <stdint.h>

typedef struct { float rpm; uint16_t pwm; } MotorFFPoint;
static const MotorFFPoint motor_ff_lut[7] = {
    { 15.24f, 75 },
    { 27.24f, 100 },
    { 38.76f, 125 },
    { 50.24f, 150 },
    { 61.57f, 175 },
    { 72.76f, 200 },
    { 83.76f, 225 },
};
static const uint32_t motor_ff_lut_count = 7U;
