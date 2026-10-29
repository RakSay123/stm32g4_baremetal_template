#include "timer.h"

static TIM_Status_t timer_enable_clock(TIM_TypeDef *TIMx)
{
	if (TIMx == NULL) return TIM_ERROR_NULL;

	if (TIMx == TIM1) RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
	else if (TIMx == TIM2) RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
	else if (TIMx == TIM3) RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN;
	else if (TIMx == TIM4) RCC->APB1ENR1 |= RCC_APB1ENR1_TIM4EN;
	else if (TIMx == TIM5) RCC->APB1ENR1 |= RCC_APB1ENR1_TIM5EN;
	else if (TIMx == TIM6) RCC->APB1ENR1 |= RCC_APB1ENR1_TIM6EN;
	else if (TIMx == TIM7) RCC->APB1ENR1 |= RCC_APB1ENR1_TIM7EN;
	else if (TIMx == TIM8) RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;
	else if (TIMx == TIM15) RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;
	else if (TIMx == TIM16) RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;
	else if (TIMx == TIM17) RCC->APB2ENR |= RCC_APB2ENR_TIM17EN;
	else if (TIMx == TIM20) RCC->APB2ENR |= RCC_APB2ENR_TIM20EN;
	else return TIM_ERROR_INVALID_INSTANCE;

	// Read back so the peripheral clock is active before the first register access
	(void)RCC->APB1ENR1;
	(void)RCC->APB2ENR;

	return TIM_OK;
}

static TIM_Status_t timer_get_irq(TIM_TypeDef *TIMx, IRQn_Type *irq)
{
	if (TIMx == NULL || irq == NULL) return TIM_ERROR_NULL;

	if (TIMx == TIM1) *irq = TIM1_UP_TIM16_IRQn;
	else if (TIMx == TIM2) *irq = TIM2_IRQn;
	else if (TIMx == TIM3) *irq = TIM3_IRQn;
	else if (TIMx == TIM4) *irq = TIM4_IRQn;
	else if (TIMx == TIM5) *irq = TIM5_IRQn;
	else if (TIMx == TIM6) *irq = TIM6_DAC_IRQn;
	else if (TIMx == TIM7) *irq = TIM7_DAC_IRQn;
	else if (TIMx == TIM8) *irq = TIM8_UP_IRQn;
	else if (TIMx == TIM15) *irq = TIM1_BRK_TIM15_IRQn;
	else if (TIMx == TIM16) *irq = TIM1_UP_TIM16_IRQn;
	else if (TIMx == TIM17) *irq = TIM1_TRG_COM_TIM17_IRQn;
	else if (TIMx == TIM20) *irq = TIM20_UP_IRQn;
	else return TIM_ERROR_INVALID_INSTANCE;

	return TIM_OK;
}

static TIM_Status_t timer_nvic_enable(TIM_TypeDef *TIMx)
{
	IRQn_Type irq;

	TIM_Status_t status = timer_get_irq(TIMx, &irq);
	if (status != TIM_OK) return status;

	NVIC_EnableIRQ(irq);

	return TIM_OK;
}

TIM_Status_t timer_init(const TIM_Config_t *cfg)
{
	if (cfg == NULL || cfg->TIMx == NULL) return TIM_ERROR_NULL;

	TIM_TypeDef *TIMx = cfg->TIMx;

	if (cfg->psc > 0xFFFFU) return TIM_ERROR_INVALID_CONFIG;

	if (!IS_TIM_32B_COUNTER_INSTANCE(TIMx) && cfg->arr > 0xFFFFU) return TIM_ERROR_INVALID_CONFIG;

	if (cfg->cnt > cfg->arr) return TIM_ERROR_INVALID_CONFIG;

	TIM_Status_t status;

	status = timer_enable_clock(TIMx);
	if (status != TIM_OK) return status;

	// Stops the counter and clears any leftover mode bits (OPM, DIR, CMS...)
	TIMx->CR1 = 0;

	TIMx->PSC = cfg->psc;
	TIMx->ARR = cfg->arr;

	// UG loads the buffered PSC/ARR into the active registers, but also sets UIF
	TIMx->EGR = TIM_EGR_UG;
	TIMx->SR = ~TIM_SR_UIF;

	TIMx->CNT = cfg->cnt;

	if (cfg->interrupt_status == TIM_INTERRUPTS_ENABLED)
	{
		TIMx->DIER |= TIM_DIER_UIE;

		status = timer_nvic_enable(TIMx);
		if (status != TIM_OK) return status;
	}
	else
	{
		TIMx->DIER &= ~TIM_DIER_UIE;
	}

	TIMx->CR1 |= TIM_CR1_CEN;

	return TIM_OK;
}

static TIM_Status_t timer_wait(TIM_TypeDef *TIMx)
{
	while (!(TIMx->SR & TIM_SR_UIF));

	TIMx->SR = ~TIM_SR_UIF;

	return TIM_OK;
}

TIM_Status_t timer_delay_s(TIM_TypeDef *TIMx, uint32_t s)
{
	if (TIMx == NULL) return TIM_ERROR_NULL;

	// Polling would hang if the counter is stopped or an ISR is clearing UIF
	if (!(TIMx->CR1 & TIM_CR1_CEN) || (TIMx->DIER & TIM_DIER_UIE)) return TIM_ERROR_INVALID_CONFIG;

	for (uint32_t i = 0; i < s; i++)
	{
		TIM_Status_t status = timer_wait(TIMx);
		if (status != TIM_OK) return status;
	}

	return TIM_OK;
}

uint8_t timer_update_pending(TIM_TypeDef *TIMx)
{
	if (TIMx == NULL) return 0U;

	return (TIMx->SR & TIM_SR_UIF) ? 1U : 0U;
}

TIM_Status_t timer_clear_update_flag(TIM_TypeDef *TIMx)
{
	if (TIMx == NULL) return TIM_ERROR_NULL;

	// SR bits are rc_w0: writing 1 has no effect, so only UIF is cleared
	TIMx->SR = ~TIM_SR_UIF;

	return TIM_OK;
}
