#pragma once

#include "timer/timer.h"
#include "gpio/gpio.h"

typedef enum {
	TIM_ENCODER_MODE_TI2_EDGES,			// SMS = 001
	TIM_ENCODER_MODE_TI1_EDGES,			// SMS = 010
	TIM_ENCODER_MODE_TI1_TI2_EDGES, 	// SMS = 011
} TIM_ENCODER_Mode_t;

typedef enum {
    TIM_ENCODER_IC_PSC_NONE = 0U,
    TIM_ENCODER_IC_PSC_DIV2 = 1U,
    TIM_ENCODER_IC_PSC_DIV4 = 2U,
    TIM_ENCODER_IC_PSC_DIV8 = 3U
} TIM_ENCODER_Input_Prescaler_t;

typedef enum {
    TIM_ENCODER_FILTER_NONE = 0U,

    TIM_ENCODER_FILTER_FCK_INT_N2 = 1U,
    TIM_ENCODER_FILTER_FCK_INT_N4 = 2U,
    TIM_ENCODER_FILTER_FCK_INT_N8 = 3U,

    TIM_ENCODER_FILTER_FDTS_DIV2_N6 = 4U,
    TIM_ENCODER_FILTER_FDTS_DIV2_N8 = 5U,

    TIM_ENCODER_FILTER_FDTS_DIV4_N6 = 6U,
    TIM_ENCODER_FILTER_FDTS_DIV4_N8 = 7U,

    TIM_ENCODER_FILTER_FDTS_DIV8_N6 = 8U,
    TIM_ENCODER_FILTER_FDTS_DIV8_N8 = 9U,

    TIM_ENCODER_FILTER_FDTS_DIV16_N5 = 10U,
    TIM_ENCODER_FILTER_FDTS_DIV16_N6 = 11U,
    TIM_ENCODER_FILTER_FDTS_DIV16_N8 = 12U,

    TIM_ENCODER_FILTER_FDTS_DIV32_N5 = 13U,
    TIM_ENCODER_FILTER_FDTS_DIV32_N6 = 14U,
    TIM_ENCODER_FILTER_FDTS_DIV32_N8 = 15U
} TIM_ENCODER_Filter_t;

typedef enum {
	TIM_ENCODER_POLARITY_NORMAL = 0U,
	TIM_ENCODER_POLARITY_INVERTED = 1U
} TIM_ENCODER_Polarity_t;

typedef enum {
	TIM_ENCODER_DIRECTION_UP,
	TIM_ENCODER_DIRECTION_DOWN,
} TIM_ENCODER_Direction_t;

typedef struct {
	TIM_TypeDef *TIMx;

	GPIO_TypeDef *channel_a_port;
	uint8_t channel_a_pin;
	GPIO_AF_t channel_a_AF;

	GPIO_TypeDef *channel_b_port;
	uint8_t channel_b_pin;
	GPIO_AF_t channel_b_AF;

	TIM_ENCODER_Mode_t mode;

	TIM_ENCODER_Filter_t channel_a_filter;
	TIM_ENCODER_Filter_t channel_b_filter;

	TIM_ENCODER_Input_Prescaler_t channel_a_psc;
	TIM_ENCODER_Input_Prescaler_t channel_b_psc;

	TIM_ENCODER_Polarity_t channel_a_polarity;
	TIM_ENCODER_Polarity_t channel_b_polarity;

} TIM_ENCODER_Config_t;

TIM_Status_t timer_encoder_init(TIM_ENCODER_Config_t *cfg);

TIM_Status_t timer_encoder_get_count(TIM_ENCODER_Config_t *encoder, uint32_t *count);

TIM_Status_t timer_encoder_set_count(TIM_ENCODER_Config_t *encoder, uint32_t count);

TIM_Status_t timer_encoder_reset_count(TIM_ENCODER_Config_t *encoder);

TIM_Status_t timer_encoder_get_direction(TIM_ENCODER_Config_t *encoder, TIM_ENCODER_Direction_t *direction);
