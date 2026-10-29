#include <stdint.h>
#include <math.h>

#include "app.h"

#include "app_config.h"
#include "device_instances.h"

#include "systick/systick.h"
#include "adc/adc.h"

#include "console/console.h"
#include "led/led.h"

static LED_t *led;
static CONSOLE_t *console;

static uint32_t previous_toggle_ms;
static uint32_t previous_print_ms;

static void app_update_status_led(uint32_t current_ms)
{
	if (current_ms - previous_toggle_ms < APP_STATUS_LED_PERIOD_MS) return;
	previous_toggle_ms = current_ms;

	led_toggle(led);
}

static uint16_t app_get_adc_reading()
{
	uint16_t adc_code = 0;
	ADC_Status_t adc_status = adc_read(ADC1, &adc_code);
	if (adc_status != ADC_OK)
	{
		console_println(console, "[APP] ADC read failed!");
		return 0;
	}

	return adc_code;
}

static float app_get_adc_voltage(uint16_t adc_code)
{
	float voltage = ((float)adc_code / ADC_MAX_CODE) * ADC_VREF_VOLTAGE;
	return voltage;
}

static void app_update_print(uint32_t current_ms)
{
	if (current_ms - previous_print_ms < APP_STATUS_PRINT_PERIOD_MS) return;
	previous_print_ms = current_ms;

	uint16_t adc_code = app_get_adc_reading();
	float voltage = app_get_adc_voltage(adc_code);

	console_print(console, "[APP] ADC Code: ");
	console_print_int(console, adc_code);

	console_print(console, " | Voltage: ");
	console_print_float(console, voltage, 3);
	console_println(console, " V");
}

void app_init(void)
{
	led = board_get_status_led();
	console = board_get_console();

	console_println(console, "\r\n------------ ADC_TEST ------------");

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
