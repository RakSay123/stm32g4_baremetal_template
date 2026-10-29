#include "board_init.h"

#include <stdint.h>
#include <stddef.h>

#include "board_config.h"
#include "device_instances.h"

#include "gpio/gpio.h"
#include "systick/systick.h"
#include "uart/uart.h"
#include "adc/adc.h"

#include "led/led.h"
#include "console/console.h"

BOARD_Status_t board_init(void)
{
	LED_t *status_led = board_get_status_led();
	if (status_led == NULL) return BOARD_STATUS_LED_ERROR;

	CONSOLE_t *console = board_get_console();
	if (console == NULL) return BOARD_STATUS_CONSOLE_ERROR;

	GPIO_Config_t adc_gpio = {
		.port = GPIOB,
		.pin = 1,
		.mode = GPIO_MODE_ANALOG
	};

	SYSTICK_Config_t systick_cfg = {
		.clock_hz = BOARD_FCLK_HZ,
		.timebase = SYSTICK_TIMEBASE_1MS
	};

	ADC_Channel_Config_t adc_channels[] = {
		{
			.channel = 12,
			.sample_time = ADC_SMP_CYC_47_POINT_5
		}
	};

	ADC_Config_t adc1_cfg = {
		.instance = ADC1,
		.clk_mode = ADC_CLK_MODE_SYNC_HCLK_DIV_1,
		.seq_len = 1,
		.channels = adc_channels
	};

	if (gpio_init(&adc_gpio) != GPIO_OK) return BOARD_STATUS_GPIO_ERROR;

	if (systick_init(&systick_cfg) != SYSTICK_OK) return BOARD_STATUS_SYSTICK_ERROR;

	if (adc_init(&adc1_cfg) != ADC_OK) return BOARD_STATUS_ADC_ERROR;

	if (led_init(status_led) != LED_OK) return BOARD_STATUS_LED_ERROR;

	if (console_init(console) != CONSOLE_OK) return BOARD_STATUS_CONSOLE_ERROR;


	return BOARD_STATUS_OK;
}
