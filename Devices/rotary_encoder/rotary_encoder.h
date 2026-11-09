#pragma once

#include "timer/timer_encoder/timer_encoder.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum {
	ROTARY_ENCODER_OK,
	ROTARY_ENCODER_ERR
} ROTARY_ENCODER_Status_t;

typedef enum {
	ROTARY_ENCODER_STOPPED,
	ROTARY_ENCODER_UP,
	ROTARY_ENCODER_DOWN,
} ROTARY_ENCODER_Motion_t;

typedef struct {
	TIM_ENCODER_Config_t *encoder_cfg;

	float radius_mm;

	uint32_t pulses_per_revolution;
	uint32_t counts_per_revolution;

	uint16_t raw_count;
	uint16_t previous_raw_count;
	int16_t delta_count;
	int32_t total_count;

	float revolutions;
	float cumulative_angle_degrees;
	float normalized_angle_degrees;

	float displacement_mm;
	float total_distance_mm;

	int32_t previous_velocity_count;
	int32_t velocity_delta_count;

	uint32_t previous_update_ms;
	uint32_t sample_period_ms;

	float revolutions_per_second;
	float rpm;
	float degrees_per_second;
	float radians_per_second;
	float linear_velocity_mm_per_second;
	float top_speed_mm_per_second;

	TIM_ENCODER_Direction_t direction;
	ROTARY_ENCODER_Motion_t motion;

	uint8_t consecutive_zero_samples;
	uint8_t stopped_sample_threshold;
} ROTARY_ENCODER_t;

ROTARY_ENCODER_Status_t rotary_encoder_init(ROTARY_ENCODER_t *cfg);

ROTARY_ENCODER_Status_t rotary_encoder_update_distance(ROTARY_ENCODER_t *encoder);

ROTARY_ENCODER_Status_t rotary_encoder_update_velocity(ROTARY_ENCODER_t *encoder, uint32_t now_ms);

ROTARY_ENCODER_Status_t rotary_encoder_set_count(ROTARY_ENCODER_t *encoder, uint32_t cnt);

ROTARY_ENCODER_Status_t rotary_encoder_set_count_median(ROTARY_ENCODER_t *encoder);

ROTARY_ENCODER_Status_t rotary_encoder_set_count_zero(ROTARY_ENCODER_t *encoder);

TIM_ENCODER_Direction_t rotary_encoder_get_direction(ROTARY_ENCODER_t *encoder);

ROTARY_ENCODER_Motion_t rotary_encoder_get_motion(ROTARY_ENCODER_t *encoder);

int32_t rotary_encoder_get_total_count(ROTARY_ENCODER_t *encoder);

float rotary_encoder_get_revolutions(ROTARY_ENCODER_t *encoder);

float rotary_encoder_get_cumulative_angle_degrees(ROTARY_ENCODER_t *encoder);

float rotary_encoder_get_normalized_angle_degrees(ROTARY_ENCODER_t *encoder);

float rotary_encoder_get_displacement_mm(ROTARY_ENCODER_t *encoder);

float rotary_encoder_get_total_distance_mm(ROTARY_ENCODER_t *encoder);

float rotary_encoder_get_revolutions_per_second(ROTARY_ENCODER_t *encoder);

float rotary_encoder_get_rpm(ROTARY_ENCODER_t *encoder);

float rotary_encoder_get_degrees_per_second(ROTARY_ENCODER_t *encoder);

float rotary_encoder_get_radians_per_second(ROTARY_ENCODER_t *encoder);

float rotary_encoder_get_linear_velocity_mm_per_second(ROTARY_ENCODER_t *encoder);

float rotary_encoder_get_top_speed_mm_per_second(ROTARY_ENCODER_t *encoder);
