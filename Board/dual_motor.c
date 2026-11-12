#include "dual_motor.h"

#include <stddef.h>

#include "device_instances.h"
#include "tb6612fng/tb6612fng.h"
#include "motor_feedforward/motor_feedforward.h"
#include "motor_feedforward/motor2_ff_lut.h"

static TIM_ENCODER_Config_t tim3_encoder_cfg = {
	.TIMx = TIM3,

	.channel_a_port = GPIOA,
	.channel_a_pin = 6,
	.channel_a_AF = GPIO_AF2,

	.channel_b_port = GPIOA,
	.channel_b_pin = 7,
	.channel_b_AF = GPIO_AF2,

	.mode = TIM_ENCODER_MODE_TI1_TI2_EDGES,

	.channel_a_filter = TIM_ENCODER_FILTER_NONE,
	.channel_b_filter = TIM_ENCODER_FILTER_NONE,

	.channel_a_psc = TIM_ENCODER_IC_PSC_NONE,
	.channel_b_psc = TIM_ENCODER_IC_PSC_NONE,

	.channel_a_polarity = TIM_ENCODER_POLARITY_NORMAL,
	.channel_b_polarity = TIM_ENCODER_POLARITY_NORMAL
};

static ROTARY_ENCODER_t motor2_encoder = {
	.encoder_cfg = &tim3_encoder_cfg,

	.radius_mm = 1.0f,

	.pulses_per_revolution = 210U,
	.counts_per_revolution = 840U,

	.raw_count = 0U,
	.previous_raw_count = 0U,
	.delta_count = 0U,
	.total_count = 0U,

	.revolutions = 0U,
	.cumulative_angle_degrees = 0U,
	.normalized_angle_degrees = 0U,

	.displacement_mm = 0U,
	.total_distance_mm = 0U,

	.previous_update_ms = 0U,
	.sample_period_ms = 0U,

	.revolutions_per_second = 0U,
	.rpm = 0U,
	.degrees_per_second = 0U,
	.radians_per_second = 0U,
	.linear_velocity_mm_per_second = 0U,

	.direction = TIM_ENCODER_DIRECTION_UP,
	.motion = ROTARY_ENCODER_STOPPED,

	.consecutive_zero_samples = 0U,
	.stopped_sample_threshold = 5U,
};

/* motor_driver is set in dual_motor_init(): it lives in device_instances.c. */
static DC_MOTOR_t motor2 = {
	.motor_driver = NULL,
	.driver_channel = TB6612FNG_CHA
};

static const MOTOR_FF_LUT_t motor2_ff = {
	.points = motor_ff_lut,
	.count = sizeof(motor_ff_lut) / sizeof(motor_ff_lut[0])
};

static MOTOR_CONTROLLER_t motor2_controller = {
	.encoder = &motor2_encoder,
	.motor = &motor2,
	.ff_lut = &motor2_ff,
};

static DUAL_MOTOR_CONTROLLER_t dual_motor_controller;

DUAL_MOTOR_Status_t dual_motor_init(void)
{
	MOTOR_DRIVER_t *motor_driver = board_get_motor_driver();
	if (motor_driver == NULL) return DUAL_MOTOR_ERR;

	motor2.motor_driver = motor_driver;
	return DUAL_MOTOR_OK;
}

ROTARY_ENCODER_t* board_get_motor2_encoder(void)
{
	return &motor2_encoder;
}

DC_MOTOR_t* board_get_motor2(void)
{
	return &motor2;
}

MOTOR_CONTROLLER_t* board_get_motor2_controller(void)
{
	return &motor2_controller;
}

DUAL_MOTOR_CONTROLLER_t* board_get_dual_motor_controller(void)
{
	return &dual_motor_controller;
}
