#include "board_init.h"

#include <stdint.h>
#include <stddef.h>

#include "board_config.h"
#include "device_instances.h"

#include "gpio/gpio.h"
#include "systick/systick.h"
#include "uart/uart.h"

#include "led/led.h"
#include "console/console.h"

BOARD_Status_t board_init(void)
{
	LED_t *status_led = board_get_status_led();
	if (status_led == NULL) return BOARD_STATUS_LED_ERROR;

	CONSOLE_t *console = board_get_console();
	if (console == NULL) return BOARD_STATUS_CONSOLE_ERROR;

	SYSTICK_Config_t systick_cfg = {
		.clock_hz = BOARD_FCLK_HZ,
		.timebase = SYSTICK_TIMEBASE_1MS
	};

	if (systick_init(&systick_cfg) != SYSTICK_OK) return BOARD_STATUS_SYSTICK_ERROR;

	if (led_init(status_led) != LED_OK) return BOARD_STATUS_LED_ERROR;

	if (console_init(console) != CONSOLE_OK) return BOARD_STATUS_CONSOLE_ERROR;

	return BOARD_STATUS_OK;
}
