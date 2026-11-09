#include "rotary_encoder.h"
#include <stddef.h>

#define PI_F 3.14159265f

static uint32_t rotary_encoder_get_decode_multiplier(const TIM_ENCODER_Config_t *encoder_cfg)
{
	switch (encoder_cfg->mode)
	{
		case TIM_ENCODER_MODE_TI2_EDGES:
		case TIM_ENCODER_MODE_TI1_EDGES:
			return 2U;

		case TIM_ENCODER_MODE_TI1_TI2_EDGES:
			return 4U;

		default:
			return 0U;
	}
}

ROTARY_ENCODER_Status_t rotary_encoder_init(ROTARY_ENCODER_t *cfg)
{
	if (cfg == NULL || cfg->encoder_cfg == NULL) return ROTARY_ENCODER_ERR;

	if (timer_encoder_init(cfg->encoder_cfg) != TIM_OK) return ROTARY_ENCODER_ERR;

	uint32_t initial_count = 0U;

	if (timer_encoder_get_count(cfg->encoder_cfg, &initial_count) != TIM_OK) return ROTARY_ENCODER_ERR;


	if (timer_encoder_get_direction(cfg->encoder_cfg, &cfg->direction) != TIM_OK) return ROTARY_ENCODER_ERR;

	cfg->counts_per_revolution = cfg->pulses_per_revolution * rotary_encoder_get_decode_multiplier(cfg->encoder_cfg);

	cfg->raw_count = (uint16_t)initial_count;
	cfg->previous_raw_count = cfg->raw_count;
	cfg->delta_count = 0;
	cfg->total_count = 0;

	cfg->revolutions = 0.0f;
	cfg->cumulative_angle_degrees = 0.0f;
	cfg->normalized_angle_degrees = 0.0f;

	cfg->displacement_mm = 0.0f;
	cfg->total_distance_mm = 0.0f;

	cfg->previous_velocity_count = 0U;
	cfg->velocity_delta_count = 0U;

	cfg->previous_update_ms = 0U;

	cfg->revolutions_per_second = 0U;
	cfg->rpm = 0U;
	cfg->degrees_per_second = 0U;
	cfg->radians_per_second = 0U;
	cfg->linear_velocity_mm_per_second = 0U;
	cfg->top_speed_mm_per_second = 0U;

	cfg->motion = ROTARY_ENCODER_STOPPED;

	cfg->consecutive_zero_samples = 0U;

	return ROTARY_ENCODER_OK;
}

ROTARY_ENCODER_Status_t rotary_encoder_update_distance(ROTARY_ENCODER_t *encoder)
{
	if (encoder == NULL || encoder->encoder_cfg == NULL || encoder->counts_per_revolution == 0U) return ROTARY_ENCODER_ERR;

	if (timer_encoder_get_direction(encoder->encoder_cfg, &encoder->direction) != TIM_OK) return ROTARY_ENCODER_ERR;

	uint32_t hardware_count = 0U;
	if (timer_encoder_get_count(encoder->encoder_cfg, &hardware_count) != TIM_OK) return ROTARY_ENCODER_ERR;

	encoder->previous_raw_count = encoder->raw_count;
	encoder->raw_count = (uint16_t)hardware_count;

	encoder->delta_count = (int16_t)(encoder->raw_count - encoder->previous_raw_count);
	encoder->total_count += encoder->delta_count;

	encoder->revolutions = (float)encoder->total_count / (float)encoder->counts_per_revolution;

	encoder->cumulative_angle_degrees = (float)encoder->revolutions * 360.0f;

	int32_t normalized_count = encoder->total_count % (int32_t)encoder->counts_per_revolution;
	if (normalized_count < 0) normalized_count += (int32_t)encoder->counts_per_revolution;
	encoder->normalized_angle_degrees = ((float)normalized_count * 360.0f) / (float)encoder->counts_per_revolution;

	encoder->displacement_mm = encoder->revolutions * 2.0f * PI_F * encoder->radius_mm;

	float delta_revolutions = (float)encoder->delta_count / (float)encoder->counts_per_revolution;

	float delta_distance_mm = delta_revolutions * 2.0f * PI_F * encoder->radius_mm;
	if (delta_distance_mm < 0.0f) delta_distance_mm = -delta_distance_mm;
	encoder->total_distance_mm += delta_distance_mm;

	return ROTARY_ENCODER_OK;
}

static void rotary_encoder_update_motion(ROTARY_ENCODER_t *encoder)
{
	if (encoder->velocity_delta_count > 0)
	{
		encoder->motion = ROTARY_ENCODER_UP;
		encoder->consecutive_zero_samples = 0U;
	}
	else if (encoder->velocity_delta_count < 0)
	{
		encoder->motion = ROTARY_ENCODER_DOWN;
		encoder->consecutive_zero_samples = 0U;
	}
	else
	{
		if (encoder->consecutive_zero_samples < encoder->stopped_sample_threshold) encoder->consecutive_zero_samples++;
		if (encoder->consecutive_zero_samples >= encoder->stopped_sample_threshold) encoder->motion = ROTARY_ENCODER_STOPPED;
	}
}

ROTARY_ENCODER_Status_t rotary_encoder_update_velocity(ROTARY_ENCODER_t *encoder, uint32_t now_ms)
{
	if (encoder == NULL || encoder->encoder_cfg == NULL || encoder->counts_per_revolution == 0U) return ROTARY_ENCODER_ERR;

	encoder->sample_period_ms = now_ms - encoder->previous_update_ms;
	encoder->previous_update_ms = now_ms;

	if (encoder->sample_period_ms == 0U	) return ROTARY_ENCODER_OK;

	encoder->velocity_delta_count = encoder->total_count - encoder->previous_velocity_count;

	float elapsed_seconds = (float)encoder->sample_period_ms / 1000.0f;
	float delta_revolutions = (float)encoder->velocity_delta_count / (float)encoder->counts_per_revolution;
	encoder->revolutions_per_second = delta_revolutions / elapsed_seconds;

	encoder->rpm = encoder->revolutions_per_second * 60.0f;
	encoder->degrees_per_second = encoder->revolutions_per_second * 360.0f;
	encoder->radians_per_second = encoder->revolutions_per_second * 2.0f * PI_F;
	encoder->linear_velocity_mm_per_second = encoder->revolutions_per_second * 2.0f * PI_F * encoder->radius_mm;

	float current_speed = (encoder->linear_velocity_mm_per_second > 0) ? encoder->linear_velocity_mm_per_second: -encoder->linear_velocity_mm_per_second;
	if (current_speed > encoder->top_speed_mm_per_second) encoder->top_speed_mm_per_second = current_speed;

	rotary_encoder_update_motion(encoder);

	encoder->previous_velocity_count = encoder->total_count;

	return ROTARY_ENCODER_OK;
}

ROTARY_ENCODER_Status_t rotary_encoder_set_count(ROTARY_ENCODER_t *encoder, uint32_t cnt)
{
	if (timer_encoder_set_count(encoder->encoder_cfg, cnt) == TIM_OK) return ROTARY_ENCODER_OK;

	return ROTARY_ENCODER_ERR;
}

ROTARY_ENCODER_Status_t rotary_encoder_set_count_median(ROTARY_ENCODER_t *encoder)
{
	uint32_t median = encoder->encoder_cfg->TIMx->ARR / 2;
	return rotary_encoder_set_count(encoder, median);
}

ROTARY_ENCODER_Status_t rotary_encoder_set_count_zero(ROTARY_ENCODER_t *encoder)
{
	if (encoder == NULL || encoder->encoder_cfg == NULL) return ROTARY_ENCODER_ERR;

	if (timer_encoder_set_count(encoder->encoder_cfg, 0U) != TIM_OK) return ROTARY_ENCODER_ERR;

	encoder->raw_count = 0U;
	encoder->previous_raw_count = 0U;
	encoder->delta_count = 0;
	encoder->total_count = 0;

	encoder->revolutions = 0.0f;
	encoder->cumulative_angle_degrees = 0.0f;
	encoder->normalized_angle_degrees = 0.0f;

	encoder->displacement_mm = 0.0f;
	encoder->total_distance_mm = 0.0f;

	encoder->previous_raw_count = 0U;
	encoder->previous_update_ms = 0U;

	encoder->revolutions_per_second = 0U;
	encoder->rpm = 0U;
	encoder->degrees_per_second = 0U;
	encoder->radians_per_second = 0U;
	encoder->linear_velocity_mm_per_second = 0U;

	encoder->motion = ROTARY_ENCODER_STOPPED;

	return ROTARY_ENCODER_OK;
}

TIM_ENCODER_Direction_t rotary_encoder_get_direction(ROTARY_ENCODER_t *encoder)
{
	return encoder->direction;
}

ROTARY_ENCODER_Motion_t rotary_encoder_get_motion(ROTARY_ENCODER_t *encoder)
{
	if (encoder == NULL) return ROTARY_ENCODER_STOPPED;
	return encoder->motion;
}

int32_t rotary_encoder_get_total_count(ROTARY_ENCODER_t *encoder)
{
	return (int32_t)encoder->total_count;
}

float rotary_encoder_get_revolutions(ROTARY_ENCODER_t *encoder)
{
	return (float)encoder->revolutions;
}

float rotary_encoder_get_cumulative_angle_degrees(ROTARY_ENCODER_t *encoder)
{
	return (float)encoder->cumulative_angle_degrees;
}

float rotary_encoder_get_normalized_angle_degrees(ROTARY_ENCODER_t *encoder)
{
	return (float)encoder->normalized_angle_degrees;
}

float rotary_encoder_get_displacement_mm(ROTARY_ENCODER_t *encoder)
{
	return (float)encoder->displacement_mm;
}

float rotary_encoder_get_total_distance_mm(ROTARY_ENCODER_t *encoder)
{
	return (float)encoder->total_distance_mm;
}

float rotary_encoder_get_revolutions_per_second(ROTARY_ENCODER_t *encoder)
{
	return (float)encoder->revolutions_per_second;
}

float rotary_encoder_get_rpm(ROTARY_ENCODER_t *encoder)
{
	return (float)encoder->rpm;
}

float rotary_encoder_get_degrees_per_second(ROTARY_ENCODER_t *encoder)
{
	return (float)encoder->degrees_per_second;
}

float rotary_encoder_get_radians_per_second(ROTARY_ENCODER_t *encoder)
{
	return (float)encoder->radians_per_second;
}

float rotary_encoder_get_linear_velocity_mm_per_second(ROTARY_ENCODER_t *encoder)
{
	return (float)encoder->linear_velocity_mm_per_second;
}

float rotary_encoder_get_top_speed_mm_per_second(ROTARY_ENCODER_t *encoder)
{
	return (float)encoder->top_speed_mm_per_second;
}
