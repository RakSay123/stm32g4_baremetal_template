#pragma once

#include <stddef.h>
#include "gpio/gpio.h"
#include "timer/timer_pwm/timer_pwm.h"

typedef enum {
	TB6612FNG_OK,
	TB6612FNG_INIT_ERR,
	TB6612FNG_ERR,
} TB6612FNG_Status_t;

typedef enum {
	TB6612FNG_CHA,
	TB6612FNG_CHB
} TB6612FNG_Channel_t;

typedef enum {
	TB6612FNG_DIRECTION_CW,
	TB6612FNG_DIRECTION_CCW
} TB6612FNG_Direction_t;

typedef struct {
	GPIO_Pin_t *in1;
	GPIO_Pin_t *in2;

	GPIO_Pin_t *pwm_pin;
	GPIO_AF_t pwm_af;
	TIM_PWM_Config_t *pwm;
} TB6612FNG_Channel_Config_t;

typedef struct {
	GPIO_Pin_t *stby;

	TB6612FNG_Channel_Config_t *channel_a;
	TB6612FNG_Channel_Config_t *channel_b;
} TB6612FNG_t;

TB6612FNG_Status_t tb6612fng_init(TB6612FNG_t *tb6612fng);

TB6612FNG_Status_t tb6612fng_set_direction(TB6612FNG_t *tb6612fng, TB6612FNG_Channel_t channel, TB6612FNG_Direction_t direction);

TB6612FNG_Status_t tb6612fng_set_duty_cycle(TB6612FNG_t *tb6612fng, TB6612FNG_Channel_t channel, uint16_t duty_cycle);

TB6612FNG_Status_t tb6612fng_brake(TB6612FNG_t *tb6612fng, TB6612FNG_Channel_t channel);

TB6612FNG_Status_t tb6612fng_coast(TB6612FNG_t *tb6612fng, TB6612FNG_Channel_t channel);

TB6612FNG_Status_t tb6612fng_enable(TB6612FNG_t *tb6612fng);

TB6612FNG_Status_t tb6612fng_disable(TB6612FNG_t *tb6612fng);

TB6612FNG_Status_t tb6612fng_disable_pwm(TB6612FNG_t *tb6612fng, TB6612FNG_Channel_t channel);
