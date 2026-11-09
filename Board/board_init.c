#include "board_init.h"

#include <stdint.h>
#include <stddef.h>

#include "board_config.h"
#include "device_instances.h"


#include "gpio/gpio.h"
#include "systick/systick.h"
#include "timer/timer.h"
#include "timer/timer_pwm/timer_pwm.h"
#include "systick/systick.h"
#include "uart/uart.h"

#include "led/led.h"
#include "motor_driver/motor_driver.h"
#include "tb6612fng/tb6612fng.h"
#include "tb6612fng/tb6612fng_motor_driver.h"
#include "dc_motor/dc_motor.h"
#include "motor_controller/motor_controller.h"
#include "console/console.h"

static SYSTICK_Config_t systick_cfg = {
	.clock_hz = BOARD_FCLK_HZ,
	.timebase = SYSTICK_TIMEBASE_1MS
};

static TIM_Config_t tim1_cfg = {
	.TIMx = TIM1,
	.psc = BOARD_TIM1_PSC,
	.arr = BOARD_TIM1_ARR,
	.cnt = BOARD_TIM1_CNT,
	.interrupt_status = TIM_INTERRUPTS_DISABLED
};

static TIM_Config_t tim2_cfg = {
	.TIMx = TIM2,
	.psc = BOARD_TIM2_PSC,
	.arr = BOARD_TIM2_ARR,
	.cnt = BOARD_TIM2_CNT,
	.interrupt_status = TIM_INTERRUPTS_DISABLED
};

BOARD_Status_t board_init(void)
{
	LED_t *status_led = board_get_status_led();
	ROTARY_ENCODER_t *motor_encoder = board_get_motor_encoder();
	TB6612FNG_t *tb6612fng = board_get_tb6612fng();
	MOTOR_DRIVER_t *motor_driver = board_get_motor_driver();
	DC_MOTOR_t *motor = board_get_motor();
	MOTOR_CONTROLLER_t *motor_controller = board_get_motor_controller();
	CONSOLE_t *console = board_get_console();


	if (status_led == NULL) return BOARD_STATUS_LED_ERROR;
	if (tb6612fng == NULL) return BOARD_STATUS_TB6612FNG_ERROR;
	if (motor_driver == NULL) return BOARD_STATUS_MOTOR_DRIVER_ERROR;
	if (motor == NULL) return BOARD_STATUS_DC_MOTOR_ERROR;
	if (console == NULL) return BOARD_STATUS_CONSOLE_ERROR;

	if (systick_init(&systick_cfg) != SYSTICK_OK) return BOARD_STATUS_SYSTICK_ERROR;
	if (timer_init(&tim1_cfg) != TIM_OK) return BOARD_STATUS_TIM1_ERROR;
	if (timer_init(&tim2_cfg) != TIM_OK) return BOARD_STATUS_TIM2_ERROR;

	if (led_init(status_led) != LED_OK) return BOARD_STATUS_LED_ERROR;
	if (rotary_encoder_init(motor_encoder) != ROTARY_ENCODER_OK) return BOARD_STATUS_ENCODER_ERROR;  // should be commented out when motor_controller_init properly initializes this
	if (tb6612fng_init(tb6612fng) != TB6612FNG_OK) return BOARD_STATUS_TB6612FNG_ERROR;
	if (tb6612fng_motor_driver_bind(motor_driver, tb6612fng) != MOTOR_DRIVER_OK) return BOARD_STATUS_MOTOR_DRIVER_ERROR;
	if (dc_motor_init(motor) != DC_MOTOR_OK) return BOARD_STATUS_DC_MOTOR_ERROR;                     // should be commented out when motor_controller_init properly initializes this
	if (motor_controller_init(motor_controller, KP, KI, KD, 0.0f, (float)tb6612fng->channel_a->pwm->TIMx->ARR) != MOTOR_CONTROLLER_OK) return BOARD_STATUS_MOTOR_CONTROLLER_ERROR;
	if (console_init(console) != CONSOLE_OK) return BOARD_STATUS_CONSOLE_ERROR;

	return BOARD_STATUS_OK;
}
