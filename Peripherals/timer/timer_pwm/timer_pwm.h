#pragma once

#include <stdint.h>
#include "timer/timer.h"

typedef enum {
    TIM_PWM1 = 0,
    TIM_PWM2 = 1
} TIM_PWM_Mode_t;

typedef struct {
    TIM_TypeDef *TIMx;
    uint8_t channel;
    uint32_t duty_cycle;
    TIM_PWM_Mode_t pwm_mode;
} TIM_PWM_Config_t;

TIM_Status_t timer_pwm_init(TIM_PWM_Config_t *cfg);

TIM_Status_t timer_pwm_set_duty_cycle(TIM_PWM_Config_t *cfg, uint32_t percentage);

TIM_Status_t timer_pwm_set_compare_value( TIM_PWM_Config_t *cfg, uint32_t compare_value);
