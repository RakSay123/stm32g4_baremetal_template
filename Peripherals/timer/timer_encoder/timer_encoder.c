#include <stddef.h>
#include <stdint.h>

#include "timer/timer_encoder/timer_encoder.h"

static TIM_Status_t timer_encoder_validate(const TIM_ENCODER_Config_t *cfg)
{
    if (cfg == NULL || cfg->TIMx == NULL) return TIM_ERR_INVALID;
    if (cfg->channel_a_port == NULL || cfg->channel_b_port == NULL) return TIM_ERR_INVALID;
    if (cfg->channel_a_pin > 15U || cfg->channel_b_pin > 15U) return TIM_ERR_INVALID;
    if (cfg->TIMx != TIM1 && cfg->TIMx != TIM2 && cfg->TIMx != TIM3 && cfg->TIMx != TIM4 && cfg->TIMx != TIM5 && cfg->TIMx != TIM8 && cfg->TIMx != TIM20) return TIM_ERR_INVALID;
    if ((uint32_t)cfg->mode > TIM_ENCODER_MODE_TI1_TI2_EDGES) return TIM_ERR_INVALID;
    if ((uint32_t)cfg->channel_a_filter > 15U || (uint32_t)cfg->channel_b_filter > 15U) return TIM_ERR_INVALID;
    if ((uint32_t)cfg->channel_a_psc > TIM_ENCODER_IC_PSC_DIV8 || (uint32_t)cfg->channel_b_psc > TIM_ENCODER_IC_PSC_DIV8) return TIM_ERR_INVALID;
    if ((uint32_t)cfg->channel_a_polarity > TIM_ENCODER_POLARITY_INVERTED || (uint32_t)cfg->channel_b_polarity > TIM_ENCODER_POLARITY_INVERTED) return TIM_ERR_INVALID;

    return TIM_OK;
}

static TIM_Status_t timer_encoder_gpio_init( const TIM_ENCODER_Config_t *cfg)
{
    GPIO_Config_t ch_a = {
        .port = cfg->channel_a_port,
        .pin = cfg->channel_a_pin,
        .mode = GPIO_MODE_AF,
        .otype = GPIO_PUSH_PULL,
        .speed = GPIO_SPEED_HIGH,
        .pull = GPIO_PULL_UP,
        .alternate = cfg->channel_a_AF
    };

    GPIO_Config_t ch_b = {
        .port = cfg->channel_b_port,
        .pin = cfg->channel_b_pin,
        .mode = GPIO_MODE_AF,
        .otype = GPIO_PUSH_PULL,
        .speed = GPIO_SPEED_HIGH,
        .pull = GPIO_PULL_UP,
        .alternate = cfg->channel_b_AF
    };

    if (gpio_init(&ch_a) != GPIO_OK) return TIM_ERR;
    if (gpio_init(&ch_b) != GPIO_OK) return TIM_ERR;

    return TIM_OK;
}

static void timer_encoder_config_inputs(TIM_TypeDef *TIMx)
{
    TIMx->CCMR1 &= ~(
        TIM_CCMR1_CC1S_Msk |
        TIM_CCMR1_CC2S_Msk
    );

    TIMx->CCMR1 |= (
        TIM_CCMR1_CC1S_0 |
        TIM_CCMR1_CC2S_0
    );
}

static void timer_encoder_config_filters(const TIM_ENCODER_Config_t *cfg)
{
    TIM_TypeDef *TIMx = cfg->TIMx;
    TIMx->CCMR1 &= ~(TIM_CCMR1_IC1F_Msk | TIM_CCMR1_IC2F_Msk);
    TIMx->CCMR1 |= ( ((uint32_t)cfg->channel_a_filter << 4U) | ((uint32_t)cfg->channel_b_filter << 12U));
}

static void timer_encoder_config_prescalers(const TIM_ENCODER_Config_t *cfg)
{
    TIM_TypeDef *TIMx = cfg->TIMx;
    TIMx->CCMR1 &= ~(TIM_CCMR1_IC1PSC_Msk | TIM_CCMR1_IC2PSC_Msk);
    TIMx->CCMR1 |= (((uint32_t)cfg->channel_a_psc << 2U) | ((uint32_t)cfg->channel_b_psc << 10U));
}

static void timer_encoder_config_polarity(const TIM_ENCODER_Config_t *cfg)
{
    TIM_TypeDef *TIMx = cfg->TIMx;

    TIMx->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC1NP | TIM_CCER_CC2P | TIM_CCER_CC2NP);

    if (cfg->channel_a_polarity == TIM_ENCODER_POLARITY_INVERTED) TIMx->CCER |= TIM_CCER_CC1P;

    if (cfg->channel_b_polarity == TIM_ENCODER_POLARITY_INVERTED) TIMx->CCER |= TIM_CCER_CC2P;
}

static void timer_encoder_config_mode(const TIM_ENCODER_Config_t *cfg)
{
    TIM_TypeDef *TIMx = cfg->TIMx;

    TIMx->SMCR &= ~(TIM_SMCR_SMS_Msk);

    switch (cfg->mode)
    {
        case TIM_ENCODER_MODE_TI2_EDGES:
            TIMx->SMCR |= 1U;
            break;

        case TIM_ENCODER_MODE_TI1_EDGES:
            TIMx->SMCR |= 2U;
            break;

        case TIM_ENCODER_MODE_TI1_TI2_EDGES:
            TIMx->SMCR |= 3U;
            break;
    }
}

TIM_Status_t timer_encoder_init(TIM_ENCODER_Config_t *cfg)
{
    TIM_Status_t status = timer_encoder_validate(cfg);

    if (status != TIM_OK) return status;

    status = timer_encoder_gpio_init(cfg);

    if (status != TIM_OK) return status;

    TIM_TypeDef *TIMx = cfg->TIMx;

    TIMx->CR1 &= ~TIM_CR1_CEN;
    TIMx->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC2E);

    timer_encoder_config_inputs(TIMx);
    timer_encoder_config_filters(cfg);
    timer_encoder_config_prescalers(cfg);
    timer_encoder_config_polarity(cfg);
    timer_encoder_config_mode(cfg);

    TIMx->CNT = 0U;
    TIMx->SR = 0U;
    TIMx->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E);
    TIMx->CR1 |= TIM_CR1_CEN;

    return TIM_OK;
}

TIM_Status_t timer_encoder_get_count(TIM_ENCODER_Config_t *encoder, uint32_t *count)
{
    if (encoder == NULL || encoder->TIMx == NULL || count == NULL) return TIM_ERR_INVALID;

    *count = encoder->TIMx->CNT;

    return TIM_OK;
}

TIM_Status_t timer_encoder_set_count(TIM_ENCODER_Config_t *encoder, uint32_t count)
{
    if (encoder == NULL || encoder->TIMx == NULL) return TIM_ERR_INVALID;
    if (count > encoder->TIMx->ARR) return TIM_ERR_INVALID;

    encoder->TIMx->CNT = count;

    return TIM_OK;
}

TIM_Status_t timer_encoder_reset_count(TIM_ENCODER_Config_t *encoder)
{
    return timer_encoder_set_count(encoder, 0U);
}

TIM_Status_t timer_encoder_get_direction(TIM_ENCODER_Config_t *encoder, TIM_ENCODER_Direction_t *direction)
{
    if (encoder == NULL || encoder->TIMx == NULL || direction == NULL) return TIM_ERR_INVALID;

    if (encoder->TIMx->CR1 & TIM_CR1_DIR) *direction = TIM_ENCODER_DIRECTION_DOWN;
    else *direction = TIM_ENCODER_DIRECTION_UP;

    return TIM_OK;
}
