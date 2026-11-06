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
	if (get_adc_common(cfg->instance) == NULL) return ADC_ERROR_INVALID_CONFIG;
	if (cfg->seq_len < 1 || cfg->seq_len > 16) return ADC_ERROR_INVALID_CONFIG;

	ADC_TypeDef *ADCx = cfg->instance;

	uint32_t busy_mask = ADC_CR_ADEN | ADC_CR_ADSTART | ADC_CR_JADSTART | ADC_CR_ADDIS;
	if (ADCx->CR & busy_mask) return ADC_ERROR_BUSY;

	uint32_t sqr1 = (uint32_t)(cfg->seq_len - 1U);
	uint32_t sqr2 = 0;
	uint32_t sqr3 = 0;
	uint32_t sqr4 = 0;

	for (uint8_t i = 0; i < cfg->seq_len; i++)
	{
		uint32_t channel = cfg->channels[i].channel;
		uint32_t rank = i + 1U;

		if (channel > 18U) return ADC_ERROR_INVALID_CONFIG;

		if (rank <= 4U)
		{
			uint32_t position = 6U * rank;
			sqr1 |= channel << position;
		}
		else if (rank <= 9U)
		{
			uint32_t position = 6U * (rank - 5U);
			sqr2 |= channel << position;
		}
		else if (rank <= 14U)
		{
			uint32_t position = 6U * (rank - 10U);
			sqr3 |= channel << position;
		}
		else
		{
			uint32_t position = 6U * (rank - 15U);
			sqr4 |= channel << position;
		}
	}

	ADCx->SQR1 = sqr1;
	ADCx->SQR2 = sqr2;
	ADCx->SQR3 = sqr3;
	ADCx->SQR4 = sqr4;

	return ADC_OK;
}

static ADC_Status_t adc_config_sampling(const ADC_Config_t *cfg)
{
	if (cfg == NULL || cfg->instance == NULL || cfg->channels == NULL) return ADC_ERROR_NULL;
	if (get_adc_common(cfg->instance) == NULL) return ADC_ERROR_INVALID_CONFIG;
	if (cfg->seq_len < 1 || cfg->seq_len > 16) return ADC_ERROR_INVALID_CONFIG;

	ADC_TypeDef *ADCx = cfg->instance;

	if (ADCx->CR & (ADC_CR_ADSTART | ADC_CR_JADSTART)) return ADC_ERROR_BUSY;

	uint32_t smpr1 = ADCx->SMPR1;
	uint32_t smpr2 = ADCx->SMPR2;

	for (uint8_t i = 0; i < cfg->seq_len; i++)
	{
		uint8_t channel = cfg->channels[i].channel;
		ADC_Sample_Cycles_t sample_time = cfg->channels[i].sample_time;

		if (channel > 18 || (unsigned)sample_time > ADC_SMP_CYC_640_POINT_5) return ADC_ERROR_INVALID_CONFIG;

		uint32_t position;
		if (channel < 10)
		{
			position = 3U * channel;
			smpr1 = (smpr1 & ~(7U << position)) | ((uint32_t)sample_time << position);
		}
		else
		{
			position = 3U * (channel - 10);
			smpr2 = (smpr2 & ~(7U << position)) | ((uint32_t)sample_time << position);
		}
	}

	ADCx->SMPR1 = smpr1;
	ADCx->SMPR2 = smpr2;

	return ADC_OK;
}

static ADC_Status_t adc_configure_mode(ADC_TypeDef *ADCx)
{
    if (ADCx == NULL) return ADC_ERROR_NULL;
    if (get_adc_common(ADCx) == NULL) return ADC_ERROR_INVALID_CONFIG;
    if (ADCx->CR & ADC_CR_ADEN) return ADC_ERROR_BUSY;

    ADCx->CFGR &= ~(ADC_CFGR_CONT | ADC_CFGR_EXTEN   | ADC_CFGR_DMAEN   | ADC_CFGR_DMACFG  | ADC_CFGR_AUTDLY  | ADC_CFGR_OVRMOD  | ADC_CFGR_RES);

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

	status = adc_configure_mode(cfg->instance);
	if (status != ADC_OK) return status;

	status = adc_enable(cfg->instance);
	if (status != ADC_OK) return status;

	return status;
}

ADC_Status_t adc_start(ADC_TypeDef *ADCx)
{
	if (ADCx == NULL) return ADC_ERROR_NULL;
	if (get_adc_common(ADCx) == NULL) return ADC_ERROR_INVALID_CONFIG;
	if (!(ADCx->CR & ADC_CR_ADEN)) return ADC_ERROR_BUSY;
	if (ADCx->CR & (ADC_CR_ADSTART | ADC_CR_JADSTART | ADC_CR_ADSTP | ADC_CR_JADSTP | ADC_CR_ADDIS)) return ADC_ERROR_BUSY;
	if (ADCx->ISR & ADC_ISR_OVR) return ADC_ERROR_OVERRUN;

	ADCx->ISR = ADC_ISR_EOC | ADC_ISR_EOS;

	ADCx->CR |= ADC_CR_ADSTART;

	return ADC_OK;
}

bool adc_poll(ADC_TypeDef *ADCx)
{
	if (ADCx == NULL) return false;
	if (get_adc_common(ADCx) == NULL) return false;

	return (ADCx->ISR & ADC_ISR_EOC) != 0U;
}

ADC_Status_t adc_get_result(ADC_TypeDef *ADCx, uint16_t *result)
{
	if (ADCx == NULL || result == NULL) return ADC_ERROR_NULL;
	if (get_adc_common(ADCx) == NULL) return ADC_ERROR_INVALID_CONFIG;
	if (!(ADCx->CR & ADC_CR_ADEN)) return ADC_ERROR_BUSY;
	if (ADCx->ISR & ADC_ISR_OVR) return ADC_ERROR_OVERRUN;
	if (!(ADCx->ISR & ADC_ISR_EOC)) return ADC_PENDING;
	if (ADCx->CR & (ADC_CR_ADSTART | ADC_CR_JADSTART)) return ADC_ERROR_BUSY;

	*result = (uint16_t)ADCx->DR;

	return ADC_OK;
}

ADC_Status_t adc_disable(ADC_TypeDef *ADCx)
{
    if (ADCx == NULL) return ADC_ERROR_NULL;

    if (get_adc_common(ADCx) == NULL) return ADC_ERROR_INVALID_CONFIG;

    if (!(ADCx->CR & ADC_CR_ADEN)) return ADC_OK;

    uint32_t busy_mask = ADC_CR_ADSTART | ADC_CR_JADSTART | ADC_CR_ADSTP | ADC_CR_JADSTP | ADC_CR_ADCAL | ADC_CR_ADDIS;
    if (ADCx->CR & busy_mask) return ADC_ERROR_BUSY;

    ADCx->CR |= ADC_CR_ADDIS;

    while (ADCx->CR & ADC_CR_ADEN) __NOP();

    return ADC_OK;
}

ADC_Status_t adc_read(ADC_TypeDef *ADCx, uint16_t *result)
{
    if (result == NULL) return ADC_ERROR_NULL;

    ADC_Status_t status = adc_start(ADCx);
    if (status != ADC_OK) return status;

    while (!adc_poll(ADCx)) __NOP();

    return adc_get_result(ADCx, result);
}
