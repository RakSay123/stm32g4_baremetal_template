#include "adc.h"

static ADC_Common_TypeDef *get_adc_common(ADC_TypeDef *ADCx)
{
	if (ADCx == ADC1 || ADCx == ADC2) return ADC12_COMMON;
	else if (ADCx == ADC3 || ADCx == ADC4 || ADCx == ADC5) return ADC345_COMMON;
	else return NULL;
}

static void adc_startup_delay(void)
{
    for (volatile uint32_t i = 0; i < 100000U; i++)
    {
        __NOP();
    }
}

static ADC_Status_t adc_enable_clock(ADC_TypeDef *ADCx)
{
	if (ADCx == NULL) return ADC_ERROR_NULL;

	ADC_Common_TypeDef *ADCx_COMMON = get_adc_common(ADCx);
	if (ADCx_COMMON == NULL) return ADC_ERROR_INVALID_CONFIG;

	if (ADCx_COMMON == ADC12_COMMON) RCC->AHB2ENR |= RCC_AHB2ENR_ADC12EN;
	else if (ADCx_COMMON == ADC345_COMMON) RCC->AHB2ENR |= RCC_AHB2ENR_ADC345EN;
	else return ADC_ERROR_INVALID_CONFIG;

	return ADC_OK;
}

static ADC_Status_t adc_configure_clock(const ADC_Config_t *cfg)
{
	if (cfg == NULL || cfg->instance == NULL) return ADC_ERROR_NULL;
	if ((unsigned)cfg->clk_mode > ADC_CLK_MODE_SYNC_HCLK_DIV_4) return ADC_ERROR_INVALID_CONFIG;
	if (cfg->clk_mode == ADC_CLK_MODE_ASYNC) return ADC_ERROR_UNSUPPORTED;

	ADC_Common_TypeDef *common = get_adc_common(cfg->instance);
	if (common == NULL) return ADC_ERROR_INVALID_CONFIG;

	uint32_t busy_mask = ADC_CR_ADEN | ADC_CR_ADDIS | ADC_CR_ADSTART | ADC_CR_JADSTART | ADC_CR_ADSTP | ADC_CR_ADCAL;

	if (common == ADC12_COMMON)
	{
		if ((ADC1->CR & busy_mask) || (ADC2->CR & busy_mask)) return ADC_ERROR_BUSY;
	}
	else if (common == ADC345_COMMON)
	{
		if ((ADC3->CR & busy_mask) || (ADC4->CR & busy_mask) || (ADC5->CR & busy_mask)) return ADC_ERROR_BUSY;
	}
	else
	{
		return ADC_ERROR_INVALID_CONFIG;
	}

	common->CCR = (common->CCR & ~ADC_CCR_CKMODE) | ((uint32_t)cfg->clk_mode << ADC_CCR_CKMODE_Pos);

	return ADC_OK;
}

static ADC_Status_t adc_power_up(ADC_TypeDef *ADCx)
{
	if (ADCx == NULL) return ADC_ERROR_NULL;

	if (get_adc_common(ADCx) == NULL) return ADC_ERROR_INVALID_CONFIG;

	uint32_t cannot_clear_deeppwd = ADC_CR_ADEN | ADC_CR_ADDIS | ADC_CR_ADSTART | ADC_CR_JADSTART | ADC_CR_ADSTP | ADC_CR_JADSTP | ADC_CR_ADCAL;
	uint32_t cannot_set_advregen = ADC_CR_ADEN | ADC_CR_ADDIS | ADC_CR_ADSTART | ADC_CR_JADSTART | ADC_CR_ADSTP | ADC_CR_ADCAL;

	if (ADCx->CR & cannot_clear_deeppwd) return ADC_ERROR_BUSY;
	ADCx->CR &= ~ADC_CR_DEEPPWD;

	if (ADCx->CR & cannot_set_advregen) return ADC_ERROR_BUSY;
	ADCx->CR |= ADC_CR_ADVREGEN;

	// TEMPORARY: Conservative ADC regulator startup delay.
	// Replace with a clock-aware timing mechanism.
	adc_startup_delay();

	return ADC_OK;
}

static ADC_Status_t adc_calibrate(ADC_TypeDef *ADCx)
{
	if (ADCx == NULL) return ADC_ERROR_NULL;

	const uint32_t busy_mask = ADC_CR_ADCAL | ADC_CR_JADSTART | ADC_CR_JADSTP | ADC_CR_ADSTART | ADC_CR_ADSTP | ADC_CR_ADDIS | ADC_CR_ADEN;

	if (ADCx->CR & busy_mask) return ADC_ERROR_BUSY;

	ADCx->CR &= ~ADC_CR_ADCALDIF;
	ADCx->CR |= ADC_CR_ADCAL;

	while (ADCx->CR & ADC_CR_ADCAL);

	return ADC_OK;
}

static ADC_Status_t adc_config_sequence(const ADC_Config_t *cfg)
{
	if (cfg == NULL || cfg->instance == NULL || cfg->channels == NULL) return ADC_ERROR_NULL;
	if (cfg->seq_len < 1 || cfg->seq_len > 16) return ADC_ERROR_INVALID_CONFIG;
	if (cfg->channels[0].channel > 18) return ADC_ERROR_INVALID_CONFIG;
	if (cfg->seq_len != 1) return ADC_ERROR_UNSUPPORTED;

	cfg->instance->SQR1 = (cfg->instance->SQR1 & ~ADC_SQR1_L) | ((uint32_t)(cfg->seq_len - 1U) << ADC_SQR1_L_Pos); // Sets length to be 1
	cfg->instance->SQR1 = (cfg->instance->SQR1 & ~ADC_SQR1_SQ1) | (cfg->channels->channel << ADC_SQR1_SQ1_Pos); // Sets channel to be rank 1, hardcoded, generalize later

	return ADC_OK;
}

static ADC_Status_t adc_config_sampling(const ADC_Config_t *cfg)
{
	if (cfg == NULL || cfg->instance == NULL || cfg->channels == NULL) return ADC_ERROR_NULL;

	uint8_t channel = cfg->channels[0].channel;
	ADC_Sample_Cycles_t sample_time = cfg->channels[0].sample_time;

	if (channel > 18 || (unsigned)sample_time > ADC_SMP_CYC_640_POINT_5) return ADC_ERROR_INVALID_CONFIG;

	uint32_t busy_mask = ADC_CR_ADSTART | ADC_CR_JADSTART;
	if (cfg->instance->CR & busy_mask) return ADC_ERROR_BUSY;

	uint32_t position;
	if (channel < 10)
	{
		position = 3U * channel;
		cfg->instance->SMPR1 = (cfg->instance->SMPR1 & ~(7U << position)) | ((uint32_t)sample_time << position);
	}
	else
	{
		position = 3U * (channel - 10);
		cfg->instance->SMPR2 = (cfg->instance->SMPR2 & ~(7U << position)) | ((uint32_t)sample_time << position);
	}

	return ADC_OK;
}

static ADC_Status_t adc_enable(ADC_TypeDef *ADCx)
{
	if (ADCx == NULL) return ADC_ERROR_NULL;

	if ((ADCx->CR & ADC_CR_DEEPPWD) || !(ADCx->CR & ADC_CR_ADVREGEN)) return ADC_ERROR_BUSY;

	const uint32_t busy_mask = ADC_CR_ADEN | ADC_CR_ADDIS | ADC_CR_ADCAL;
	if (ADCx->CR & busy_mask) return ADC_ERROR_BUSY;

	adc_startup_delay();

	ADCx->ISR = ADC_ISR_ADRDY;
	ADCx->CR |= ADC_CR_ADEN;

	while (!(ADCx->ISR & ADC_ISR_ADRDY)) __NOP();

	ADCx->ISR = ADC_ISR_ADRDY;

	return ADC_OK;
}

ADC_Status_t adc_init(const ADC_Config_t *cfg)
{
	if (cfg == NULL) return ADC_ERROR_NULL;

	ADC_Status_t status;

	status = adc_enable_clock(cfg->instance);
	if (status != ADC_OK) return status;

	status = adc_configure_clock(cfg);
	if (status != ADC_OK) return status;

	status = adc_power_up(cfg->instance);
	if (status != ADC_OK) return status;

	status = adc_calibrate(cfg->instance);
	if (status != ADC_OK) return status;

	status = adc_config_sequence(cfg);
	if (status != ADC_OK) return status;

	status = adc_config_sampling(cfg);
	if (status != ADC_OK) return status;

	status = adc_enable(cfg->instance);
	if (status != ADC_OK) return status;

	return status;
}

ADC_Status_t adc_read(ADC_TypeDef *ADCx, uint16_t *result)
{
	if (ADCx == NULL || result == NULL) return ADC_ERROR_NULL;
	if (get_adc_common(ADCx) == NULL) return ADC_ERROR_INVALID_CONFIG;

	if (!(ADCx->CR & ADC_CR_ADEN)) return ADC_ERROR_BUSY;

	if (ADCx->CR & (ADC_CR_ADSTART | ADC_CR_JADSTART)) return ADC_ERROR_BUSY;

	ADCx->ISR = ADC_ISR_EOC | ADC_ISR_EOS;

	ADCx->CR |= ADC_CR_ADSTART;

	while (!(ADCx->ISR & ADC_ISR_EOC)) __NOP();

	*result = (uint16_t)ADCx->DR;

	return ADC_OK;
}
