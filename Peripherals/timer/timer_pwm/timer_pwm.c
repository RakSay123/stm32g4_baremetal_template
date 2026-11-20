#include "timer_pwm.h"

#include <stddef.h>
#include <stdint.h>


static TIM_Status_t timer_pwm_validate(const TIM_PWM_Config_t *cfg)
{
    if (cfg == NULL || cfg->TIMx == NULL) return TIM_ERR_INVALID;
    if (cfg->channel < 1U || cfg->channel > 4U) return TIM_ERR_INVALID;
    if (cfg->pwm_mode != TIM_PWM1 && cfg->pwm_mode != TIM_PWM2) return TIM_ERR_INVALID;

    TIM_TypeDef *TIMx = cfg->TIMx;

    if (TIMx == TIM1 || TIMx == TIM2 || TIMx == TIM3 || TIMx == TIM4 || TIMx == TIM5 || TIMx == TIM8 || TIMx == TIM20) return TIM_OK;
    if (TIMx == TIM15 && cfg->channel <= 2U) return TIM_OK;
    if ((TIMx == TIM16 || TIMx == TIM17) && cfg->channel == 1U) return TIM_OK;

    return TIM_ERR_INVALID;
}

static volatile uint32_t *timer_pwm_get_ccr( TIM_TypeDef *TIMx, uint8_t channel)
{
    switch (channel)
    {
        case 1: return &TIMx->CCR1;
        case 2: return &TIMx->CCR2;
        case 3: return &TIMx->CCR3;
        case 4: return &TIMx->CCR4;
        default: return NULL;
    }
}

static TIM_Status_t timer_pwm_config_channel(TIM_PWM_Config_t *cfg)
{
    TIM_TypeDef *TIMx = cfg->TIMx;
    uint8_t channel = cfg->channel;

    volatile uint32_t *ccmr;

    if (channel <= 2U) ccmr = &TIMx->CCMR1;
    else ccmr = &TIMx->CCMR2;

    uint32_t shift = ((channel - 1U) % 2U) * 8U;
    uint32_t ccxs_mask = 0x3U << shift;
    uint32_t ocm_mask = (0x7U << (shift + 4U)) | (1U << (shift + 16U));
    uint32_t preload_bit = 1U << (shift + 3U);
    uint32_t mode = (cfg->pwm_mode == TIM_PWM1) ? 6U : 7U;

    uint32_t enable_bit = 1U << ((channel - 1U) * 4U);
    TIMx->CCER &= ~enable_bit;

    uint32_t reg = *ccmr;
    reg &= ~(ccxs_mask | ocm_mask);
    reg |= mode << (shift + 4U);
    reg |= preload_bit;
    *ccmr = reg;

    uint32_t polarity_bit = 1U << (((channel - 1U) * 4U) + 1U);
    TIMx->CCER &= ~polarity_bit;

    return TIM_OK;
}

static void timer_pwm_enable_main_output(TIM_TypeDef *TIMx)
{
    if (TIMx == TIM1 || TIMx == TIM8 || TIMx == TIM15 || TIMx == TIM16 || TIMx == TIM17 || TIMx == TIM20) TIMx->BDTR |= TIM_BDTR_MOE;
}


static TIM_Status_t timer_pwm_write_compare( TIM_PWM_Config_t *cfg, uint32_t compare_value)
{
    volatile uint32_t *ccr = timer_pwm_get_ccr(cfg->TIMx, cfg->channel);
    if (ccr == NULL) return TIM_ERR_INVALID;

    uint32_t arr = cfg->TIMx->ARR;
    uint64_t period = (uint64_t)arr + 1ULL;

    if ((uint64_t)compare_value > period) compare_value = (uint32_t)period;

    *ccr = compare_value;

    return TIM_OK;
}

static TIM_Status_t timer_pwm_configure_duty_cycle(TIM_PWM_Config_t *cfg)
{
    if (cfg->duty_cycle > 100U) cfg->duty_cycle = 100U;

    uint64_t period = (uint64_t)cfg->TIMx->ARR + 1ULL;
    uint64_t compare = (period * cfg->duty_cycle) / 100ULL;

    if (compare > UINT32_MAX) compare = UINT32_MAX;

    return timer_pwm_write_compare(cfg, (uint32_t)compare);
}

TIM_Status_t timer_pwm_init(TIM_PWM_Config_t *cfg)
{
    TIM_Status_t status = timer_pwm_validate(cfg);
    if (status != TIM_OK) return status;

    TIM_TypeDef *TIMx = cfg->TIMx;
    uint32_t was_enabled = TIMx->CR1 & TIM_CR1_CEN;
    TIMx->CR1 &= ~TIM_CR1_CEN;

    status = timer_pwm_config_channel(cfg);
    if (status != TIM_OK) goto restore_counter;

    status = timer_pwm_configure_duty_cycle(cfg);
    if (status != TIM_OK) goto restore_counter;

    TIMx->EGR = TIM_EGR_UG;
    TIMx->SR &= ~TIM_SR_UIF;

    timer_pwm_enable_main_output(TIMx);

    TIMx->CCER |= 1U << ((cfg->channel - 1U) * 4U);

restore_counter:
    if (was_enabled) TIMx->CR1 |= TIM_CR1_CEN;
    return status;
}

TIM_Status_t timer_pwm_set_duty_cycle(TIM_PWM_Config_t *cfg, uint32_t percentage)
{
    TIM_Status_t status = timer_pwm_validate(cfg);
    if (status != TIM_OK) return status;

    cfg->duty_cycle = percentage;

    return timer_pwm_configure_duty_cycle(cfg);
}

TIM_Status_t timer_pwm_set_compare_value(TIM_PWM_Config_t *cfg, uint32_t compare_value)
{
    TIM_Status_t status = timer_pwm_validate(cfg);
    if (status != TIM_OK) return status;

    return timer_pwm_write_compare(cfg, compare_value);
}
