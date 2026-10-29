#pragma once

#include <stddef.h>
#include "stm32g474xx.h"

typedef enum {
	ADC_OK = 0,
	ADC_ERROR_NULL,
	ADC_ERROR_INVALID_CONFIG,
	ADC_ERROR_UNSUPPORTED,
	ADC_ERROR_BUSY,
	ADC_ERROR
} ADC_Status_t;

typedef enum {
	ADC_CLK_MODE_ASYNC = 0,
	ADC_CLK_MODE_SYNC_HCLK_DIV_1 = 1,
	ADC_CLK_MODE_SYNC_HCLK_DIV_2 = 2,
	ADC_CLK_MODE_SYNC_HCLK_DIV_4 = 3
} ADC_Clock_Mode_t;

typedef enum {
	ADC_SMP_CYC_2_POINT_5 = 0,
	ADC_SMP_CYC_6_POINT_5 = 1,
	ADC_SMP_CYC_12_POINT_5 = 2,
	ADC_SMP_CYC_24_POINT_5 = 3,
	ADC_SMP_CYC_47_POINT_5 = 4,
	ADC_SMP_CYC_92_POINT_5 = 5,
	ADC_SMP_CYC_247_POINT_5 = 6,
	ADC_SMP_CYC_640_POINT_5 = 7,

} ADC_Sample_Cycles_t;

typedef struct {
	uint8_t channel;
	ADC_Sample_Cycles_t sample_time;
} ADC_Channel_Config_t;

typedef struct {
	ADC_TypeDef *instance;
	ADC_Clock_Mode_t clk_mode;

	uint8_t seq_len;

	const ADC_Channel_Config_t *channels;
} ADC_Config_t;

ADC_Status_t adc_init(const ADC_Config_t *cfg);

ADC_Status_t adc_read(ADC_TypeDef *ADCx, uint16_t *result);
