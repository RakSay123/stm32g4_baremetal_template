#include "board_config.h"
#include "device_instances.h"
#include "tb6612fng/tb6612fng_motor_driver.h"
#include "motor_feedforward/motor_feedforward.h"
#include "motor_feedforward/motor_ff_lut.h"

static LED_t status_led = {
	.port = GPIOC,
	.pin = 6,
};

static TIM_ENCODER_Config_t tim2_encoder_cfg = {
	.TIMx = TIM2,

	.channel_a_port = GPIOA,
	.channel_a_pin = 0,
	.channel_a_AF = GPIO_AF1,

	.channel_b_port = GPIOA,
	.channel_b_pin = 1,
	.channel_b_AF = GPIO_AF1,

	.mode = TIM_ENCODER_MODE_TI1_TI2_EDGES,

	.channel_a_filter = TIM_ENCODER_FILTER_NONE,
	.channel_b_filter = TIM_ENCODER_FILTER_NONE,

	.channel_a_psc = TIM_ENCODER_IC_PSC_NONE,
	.channel_b_psc = TIM_ENCODER_IC_PSC_NONE,

	.channel_a_polarity = TIM_ENCODER_POLARITY_NORMAL,
	.channel_b_polarity = TIM_ENCODER_POLARITY_NORMAL
};

static ROTARY_ENCODER_t rotary_encoder_cfg = {
	.encoder_cfg = &tim2_encoder_cfg,

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

static GPIO_Pin_t ain1_pin = {
	.port = GPIOC,
	.pin = 7
};

static GPIO_Pin_t ain2_pin = {
	.port = GPIOA,
	.pin = 9
};

static GPIO_Pin_t pwma_pin = {
	.port = GPIOD,
	.pin = 0
};

static TIM_PWM_Config_t pwma_cfg = {
	.TIMx = TIM1,
	.channel = 2,
	.duty_cycle = 0,
	.pwm_mode = TIM_PWM1
};

static GPIO_Pin_t bin1_pin = {
	.port = GPIOB,
	.pin = 14
};

static GPIO_Pin_t bin2_pin = {
	.port = GPIOB,
	.pin = 15
};

static GPIO_Pin_t pwmb_pin = {
	.port = GPIOA,
	.pin = 8
};

static TIM_PWM_Config_t pwmb_cfg = {
	.TIMx = TIM1,
	.channel = 1,
	.duty_cycle = 0,
	.pwm_mode = TIM_PWM1
};

static GPIO_Pin_t tb6612fng_stby_pin = {
	.port = GPIOB,
	.pin = 11
};

static TB6612FNG_Channel_Config_t tb6612fng_cha = {
	.in1 = &ain1_pin,
	.in2 = &ain2_pin,

	.pwm_pin = &pwma_pin,
	.pwm_af = GPIO_AF2,
	.pwm = &pwma_cfg
};

static TB6612FNG_Channel_Config_t tb6612fng_chb = {
	.in1 = &bin1_pin,
	.in2 = &bin2_pin,

	.pwm_pin = &pwmb_pin,
	.pwm_af = GPIO_AF6,
	.pwm = &pwmb_cfg
};

static TB6612FNG_t tb6612fng = {
	.stby = &tb6612fng_stby_pin,
	.channel_a = &tb6612fng_cha,
	.channel_b = &tb6612fng_chb
};

static MOTOR_DRIVER_t motor_driver;

static DC_MOTOR_t motor = {
	.motor_driver = &motor_driver,
	.driver_channel = TB6612FNG_CHB
};

static const MOTOR_FF_LUT_t motor_ff = {
	.points = motor_ff_lut,
	.count = sizeof(motor_ff_lut) / sizeof(motor_ff_lut[0])
};

static MOTOR_CONTROLLER_t motor_controller = {
	.encoder = &rotary_encoder_cfg,
	.motor = &motor,
	.ff_lut = &motor_ff,
};

static const GPIO_AF_Pin_t USART2_TX = {
	.port = GPIOA,
	.pin = 2,
	.alt = GPIO_AF7
};

static const GPIO_AF_Pin_t USART2_RX = {
	.port = GPIOA,
	.pin = 3,
	.alt = GPIO_AF7
};

static const UART_Config_t uart_cfg = {
	.USARTx = USART2,
	.peripheral_clock_hz = BOARD_FCLK_HZ,
	.baud_rate = BOARD_DEBUG_UART_BAUD,
};

static CONSOLE_t console = {
	.uart_cfg = &uart_cfg,
	.tx = &USART2_TX,
	.rx = &USART2_RX
};

LED_t* board_get_status_led(void)
{
	return &status_led;
}

ROTARY_ENCODER_t* board_get_motor_encoder(void)
{
	return &rotary_encoder_cfg;
}

TB6612FNG_t* board_get_tb6612fng(void)
{
	return &tb6612fng;
}

MOTOR_DRIVER_t* board_get_motor_driver(void)
{
	return &motor_driver;
}

DC_MOTOR_t* board_get_motor(void)
{
	return &motor;
}

MOTOR_CONTROLLER_t* board_get_motor_controller(void)
{
	return &motor_controller;
}

CONSOLE_t* board_get_console(void)
{
	return &console;
}
