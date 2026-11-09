#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "app.h"
#include "app_config.h"
#include "device_instances.h"

#include "systick/systick.h"
#include "led/led.h"
#include "dc_motor/dc_motor.h"
#include "rotary_encoder/rotary_encoder.h"
#include "motor_controller/motor_controller.h"
#include "console/console.h"

static LED_t *led;
static DC_MOTOR_t *dc_motor;
static ROTARY_ENCODER_t *encoder;
static MOTOR_CONTROLLER_t *controller;
static CONSOLE_t *console;

static uint32_t previous_led_toggle_ms;
static uint32_t previous_controller_update_ms;
static uint32_t experiment_start_ms;

static bool app_ready = false;
static bool experiment_finished = false;

static void app_stop_experiment(void)
{
    motor_controller_disable(controller);
    dc_motor_brake(dc_motor);
    led_off(led);

    experiment_finished = true;
}

static void app_update_status_led(uint32_t current_ms)
{
    if (current_ms - previous_led_toggle_ms < APP_STATUS_LED_PERIOD_MS) return;

    previous_led_toggle_ms = current_ms;
    led_toggle(led);
}

static void app_log_controller(uint32_t current_ms)
{
    console_print_uint(console, current_ms - experiment_start_ms);
    console_print(console, ",");

    console_print_float(console, controller->target_rpm, 2);
    console_print(console, ",");

    console_print_float(console, controller->measured_rpm, 2);
    console_print(console, ",");

    console_print_float(console, controller->error, 2);
    console_print(console, ",");

    console_print_float(console, controller->control_output, 2);
    console_println(console, "");
}

static void app_controller_update(uint32_t current_ms)
{
    uint32_t elapsed_ms = current_ms - previous_controller_update_ms;

    if (elapsed_ms < APP_CONTROL_PERIOD_MS) return;

    previous_controller_update_ms = current_ms;

    float dt_seconds = (float)elapsed_ms / 1000.0f;

    if (motor_controller_update(controller, current_ms, dt_seconds) != MOTOR_CONTROLLER_OK)
	{
    	app_stop_experiment();
    	return;
	}

    app_log_controller(current_ms);
}

void app_init(void)
{
    app_ready = false;
    experiment_finished = false;

    led = board_get_status_led();
    dc_motor = board_get_motor();
    encoder = board_get_motor_encoder();
    controller = board_get_motor_controller();
    console = board_get_console();

    if (led == NULL || dc_motor == NULL || encoder == NULL || controller == NULL || console == NULL) return;

    console_println(console, "\r\n------------ PID_TEST ------------");

    motor_controller_disable(controller);
    dc_motor_brake(dc_motor);

    systick_delay_s(2);

    if (rotary_encoder_set_count_zero(encoder) != ROTARY_ENCODER_OK) return;

    if (motor_controller_set_rpm(controller, APP_TARGET_RPM) != MOTOR_CONTROLLER_OK) return;

    uint32_t current_ms = systick_get_ms();

    previous_led_toggle_ms = current_ms;
    previous_controller_update_ms = current_ms;
    experiment_start_ms = current_ms;

    encoder->previous_update_ms = current_ms;

    console_println(console, "time_ms,target_rpm,measured_rpm,error,pwm");

    app_ready = true;
    motor_controller_enable(controller);
}

void app_update(void)
{
    if (!app_ready) return;

    uint32_t current_ms = systick_get_ms();

    if (!experiment_finished)
    {
    	if (current_ms - experiment_start_ms >= APP_EXPERIMENT_DURATION_MS)
		{
			app_stop_experiment();
		}
    	else
    	{
    	    app_update_status_led(current_ms);
    	    app_controller_update(current_ms);
    	}
    }

    console_process(console);
}
