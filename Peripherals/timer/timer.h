#pragma once

#include <stdint.h>
#include <stddef.h>
#include "stm32g474xx.h"

typedef enum
{
	TIM_OK = 0,
	TIM_ERROR_NULL,
	TIM_ERROR_INVALID_INSTANCE,
	TIM_ERROR_INVALID_CONFIG
} TIM_Status_t;

typedef enum
{
	TIM_INTERRUPTS_DISABLED = 0,
	TIM_INTERRUPTS_ENABLED = 1
} TIM_Config_Interrupts_t;

typedef struct
{
	TIM_TypeDef *TIMx;

	uint32_t psc;
	uint32_t arr;
	uint32_t cnt;

	TIM_Config_Interrupts_t interrupt_status;

} TIM_Config_t;

TIM_Status_t timer_init(const TIM_Config_t *cfg);

TIM_Status_t timer_delay_s(TIM_TypeDef *TIMx, uint32_t s);

uint8_t timer_update_pending(TIM_TypeDef *TIMx);

TIM_Status_t timer_clear_update_flag(TIM_TypeDef *TIMx);
