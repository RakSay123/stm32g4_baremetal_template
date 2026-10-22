#include "app.h"

#include "app_config.h"
#include "device_instances.h"

#include "systick/systick.h"
#include "uart/uart.h"
#include "led/led.h"

static LED_t *led;

static uint32_t previous_toggle_ms;
static uint32_t previous_print_ms;

static void app_update_status_led(uint32_t current_ms)
{
	if (current_ms - previous_toggle_ms < APP_STATUS_LED_PERIOD_MS) return;
	previous_toggle_ms = current_ms;

	led_toggle(led);
}

void app_update_print(uint32_t current_ms)
{
	if (current_ms - previous_print_ms < APP_STATUS_PRINT_PERIOD_MS) return;
	previous_print_ms = current_ms;

	uart_write_string(USART2, "Hello from stm32g474CEU6\r\n");
}

void app_init(void)
{
	led = board_get_status_led();

	uint32_t current_ms = systick_get_ms();
	previous_toggle_ms = current_ms;
	previous_print_ms = current_ms;
}

void app_update(void)
{
	uint32_t current_ms = systick_get_ms();

	app_update_status_led(current_ms);
	app_update_print(current_ms);
}
