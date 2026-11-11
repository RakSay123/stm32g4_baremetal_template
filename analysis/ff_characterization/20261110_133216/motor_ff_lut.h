/* PROVISIONAL open-loop feedforward LUT; validate before use. */
#pragma once
#include <stdint.h>

typedef struct { float rpm; uint16_t pwm; } MotorFFPoint;
static const MotorFFPoint motor_ff_lut[17] = {
    { 52.62f, 150 },
    { 64.77f, 175 },
    { 76.52f, 200 },
    { 88.19f, 225 },
    { 100.05f, 250 },
    { 111.09f, 275 },
    { 123.10f, 300 },
    { 135.00f, 325 },
    { 146.86f, 350 },
    { 158.85f, 375 },
    { 170.86f, 400 },
    { 182.71f, 425 },
    { 194.62f, 450 },
    { 207.19f, 475 },
    { 219.10f, 500 },
    { 231.85f, 525 },
    { 244.95f, 550 },
};
static const uint32_t motor_ff_lut_count = 17U;
