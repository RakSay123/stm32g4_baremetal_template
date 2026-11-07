#include <stdint.h>
#include <stdbool.h>
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

static bool adc_waiting = false;
static uint32_t previous_adc_ms;
static uint16_t latest_adc_code;
static bool adc_result_valid = false;

static void app_update_status_led(uint32_t current_ms)
{
	if (current_ms - previous_toggle_ms < APP_STATUS_LED_PERIOD_MS) return;
	previous_toggle_ms = current_ms;

	led_toggle(led);
}

static void app_update_adc(uint32_t current_ms)
{
    if (!adc_waiting)
    {
        if (current_ms - previous_adc_ms < APP_STATUS_PRINT_PERIOD_MS) return;

        ADC_Status_t status = adc_start(ADC1);

        if (status != ADC_OK)
        {
            console_println(console, "[APP] ADC start failed!");
            return;
        }

        adc_waiting = true;
        previous_adc_ms = current_ms;
        return;
    }

    if (!adc_poll(ADC1)) return;

    uint16_t adc_code;
    ADC_Status_t status = adc_get_result(ADC1, &adc_code);

    adc_waiting = false;

    if (status != ADC_OK)
    {
        console_println(console, "[APP] ADC result failed!");
        return;
    }

    latest_adc_code = adc_code;
    adc_result_valid = true;
}

static float app_get_adc_voltage(uint16_t adc_code)
{
	float voltage = ((float)adc_code / ADC_MAX_CODE) * ADC_VREF_VOLTAGE;
	return voltage;
}

static void app_update_print(uint32_t current_ms)
{
    (void)current_ms;

    if (!adc_result_valid)
        return;

    adc_result_valid = false;

    float voltage = app_get_adc_voltage(latest_adc_code);

    console_print(console, "[APP] ADC Code: ");
    console_print_int(console, latest_adc_code);

    console_print(console, " | Voltage: ");
    console_print_float(console, voltage, 3);
    console_println(console, " V");
}

static void app_update_console_input(void)
{
    char character;

    CONSOLE_Status_t status =
        console_read_char(console, &character);

    if (status == CONSOLE_NO_DATA)
        return;

    if (status != CONSOLE_OK)
    {
        console_println(console, "[APP] UART RX error!");
        return;
    }

    switch (character)
    {
        case 'h':
            console_println(console, "[APP] Hello from STM32!");
            break;

        case 'a':
            console_println(console, "[APP] ADC reading requested.");

            // We'll hook this into your ADC state machine later.
            break;

        case 'l':
            led_toggle(led);
            console_println(console, "[APP] LED toggled!");
            break;

        default:
            console_println(console, "[APP] Unknown command.");
            break;
    }
}

void app_init(void)
{
	led = board_get_status_led();
	console = board_get_console();

	console_println(console, "\r\n------------ ADC_TEST ------------");

	uint32_t current_ms = systick_get_ms();
	previous_toggle_ms = current_ms;
	previous_print_ms = current_ms;
	previous_adc_ms = current_ms;

	adc_waiting = false;
	adc_result_valid = false;
}

void app_update(void)
{
    uint32_t current_ms = systick_get_ms();

    app_update_status_led(current_ms);
    app_update_adc(current_ms);
    app_update_print(current_ms);

    app_update_console_input();

    console_process(console);
}
