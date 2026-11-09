#include "timer.h"

static TIM_Status_t timer_enable_clock(TIM_TypeDef *TIMx)
{
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
    else return TIM_ERR_INVALID;

    return TIM_OK;
}

static IRQn_Type timer_get_irq(TIM_TypeDef *TIMx)
{
    if (TIMx == TIM1)  return TIM1_UP_TIM16_IRQn;
    if (TIMx == TIM2)  return TIM2_IRQn;
    if (TIMx == TIM3)  return TIM3_IRQn;
    if (TIMx == TIM4)  return TIM4_IRQn;
    if (TIMx == TIM5)  return TIM5_IRQn;
    if (TIMx == TIM6)  return TIM6_DAC_IRQn;
    if (TIMx == TIM7)  return TIM7_DAC_IRQn;
    if (TIMx == TIM8)  return TIM8_UP_IRQn;
    if (TIMx == TIM15) return TIM1_BRK_TIM15_IRQn;
    if (TIMx == TIM16) return TIM1_UP_TIM16_IRQn;
    if (TIMx == TIM17) return TIM1_TRG_COM_TIM17_IRQn;
    if (TIMx == TIM20) return TIM20_UP_IRQn;

    return (IRQn_Type)-1;
}

static TIM_Status_t timer_NVIC_interrupt_enable(TIM_TypeDef *TIMx)
{
    IRQn_Type irq = timer_get_irq(TIMx);

    if (irq == (IRQn_Type)-1) return TIM_ERR_INVALID;

    NVIC_EnableIRQ(irq);

    return TIM_OK;
}

TIM_Status_t timer_init(const TIM_Config_t *cfg)
{
    if (cfg == NULL || cfg->TIMx == NULL) return TIM_ERR_INVALID;

    if (cfg->psc > 0xFFFFU) return TIM_ERR_INVALID;

    if (cfg->interrupt_status != TIM_INTERRUPTS_DISABLED && cfg->interrupt_status != TIM_INTERRUPTS_ENABLED) return TIM_ERR_INVALID;

    if (cfg->TIMx != TIM2 && cfg->TIMx != TIM5)
    {
        if (cfg->arr > 0xFFFFU || cfg->cnt > 0xFFFFU) return TIM_ERR_INVALID;
    }

    TIM_Status_t status = timer_enable_clock(cfg->TIMx);
    if (status != TIM_OK) return status;

    cfg->TIMx->CR1 &= ~TIM_CR1_CEN;
    cfg->TIMx->PSC = cfg->psc;
    cfg->TIMx->ARR = cfg->arr;
    cfg->TIMx->EGR = TIM_EGR_UG;
    cfg->TIMx->SR &= ~TIM_SR_UIF;
    cfg->TIMx->CNT = cfg->cnt;

    if (cfg->interrupt_status == TIM_INTERRUPTS_ENABLED)
    {
        cfg->TIMx->DIER |= TIM_DIER_UIE;

        status = timer_NVIC_interrupt_enable(cfg->TIMx);
        if (status != TIM_OK)
        {
            cfg->TIMx->DIER &= ~TIM_DIER_UIE;
            return status;
        }
    }
    else cfg->TIMx->DIER &= ~TIM_DIER_UIE;

    cfg->TIMx->CR1 |= TIM_CR1_CEN;

    return TIM_OK;
}

static TIM_Status_t timer_wait(TIM_TypeDef *TIMx)
{
    if (TIMx == NULL) return TIM_ERR_INVALID;

    while (!(TIMx->SR & TIM_SR_UIF));

    TIMx->SR &= ~TIM_SR_UIF;

    return TIM_OK;
}

TIM_Status_t timer_delay_s(TIM_TypeDef *TIMx, uint32_t s)
{
    if (TIMx == NULL) return TIM_ERR_INVALID;

    for (uint32_t i = 0; i < s; i++)
    {
        TIM_Status_t status = timer_wait(TIMx);

        if (status != TIM_OK) return status;
    }

    return TIM_OK;
}

TIM_Status_t timer_get_update_flag(TIM_TypeDef *TIMx)
{
    if (TIMx == NULL) return TIM_ERR_INVALID;

    if (TIMx->SR & TIM_SR_UIF) return TIM_OK;

    return TIM_ERR;
}

TIM_Status_t timer_clear_update_flag(TIM_TypeDef *TIMx)
{
    if (TIMx == NULL) return TIM_ERR_INVALID;

    TIMx->SR &= ~TIM_SR_UIF;

    return TIM_OK;
}
