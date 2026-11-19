#include <stdint.h>
#include <stdbool.h>

#include "app.h"
#include "app_config.h"
#include "device_instances.h"

#include "systick/systick.h"
#include "console/console.h"
#include "motor_controller/motor_controller.h"
#include "dc_motor/dc_motor.h"
#include "motor_driver/motor_driver.h"
#include "dual_motor_controller/dual_motor_controller.h"

static LED_t *led;
//static MOTOR_CONTROLLER_t *left_motor;
//static MOTOR_CONTROLLER_t *right_motor;
static DUAL_MOTOR_CONTROLLER_t *dual;
static CONSOLE_t *console;

static uint32_t previous_toggle_ms;
static uint32_t previous_control_ms;
static uint32_t previous_print_ms;
static uint32_t test_start_ms;

static bool test_running = false;

static void app_stop_motors(void)
{
    if (dual != NULL) dual_motor_controller_disable(dual);
}

static void app_update_status_led(uint32_t current_ms)
{
	if (current_ms - previous_toggle_ms < APP_STATUS_LED_PERIOD_MS) return;
	previous_toggle_ms = current_ms;

	led_toggle(led);
}

static void app_update_motors(uint32_t current_ms)
{
    uint32_t elapsed_ms = current_ms - previous_control_ms;
    if (elapsed_ms < APP_CONTROL_PERIOD_MS) return;
    previous_control_ms = current_ms;
    float dt_seconds = (float)elapsed_ms / 1000.0f;

    if (dual_motor_controller_update(dual, current_ms, dt_seconds) != DUAL_MOTOR_CONTROLLER_OK)
    {
        app_stop_motors();
        console_println(console, "[APP] Left motor error!");
        return;
    }
}

static void app_update_print(uint32_t current_ms)
{
    if (current_ms - previous_print_ms < APP_PRINT_PERIOD_MS) return;
    previous_print_ms = current_ms;

    console_print(console, "[APP] Left PWM: ");
	console_print_float(console, dual->left->control_output, 0);

    console_print(console, " | Left RPM: ");
    console_print_float(console, dual->left->measured_rpm, 2);

    console_print(console, " | Right PWM: ");
	console_print_float(console, dual->right->control_output, 0);

    console_print(console, " | Right RPM: ");
    console_print_float(console, dual->right->measured_rpm, 2);

    console_println(console, "");
}

void app_init(void)
{
	led = board_get_status_led();
//    left_motor = board_get_left_motor_controller();
//    right_motor = board_get_right_motor_controller();
    dual = board_get_dual_motor_controller();
    console = board_get_console();

    if (dual == NULL || console == NULL) return;

    app_stop_motors();

    console_println(console, "\r\n------ DUAL MOTOR TEST ------");

    if (dual_motor_controller_set_feedforward(dual, true) != DUAL_MOTOR_CONTROLLER_OK)
    {
        console_println(console, "[APP] Feedforward setup failed!");
        return;
    }

    if (dual_motor_controller_set_rpm(dual, 180, 300) != DUAL_MOTOR_CONTROLLER_OK)
    {
        console_println(console, "[APP] Target RPM setup failed!");
        return;
    }

    uint32_t current_ms = systick_get_ms();

    previous_toggle_ms = current_ms;
    previous_control_ms = current_ms;
    previous_print_ms = current_ms;
    test_start_ms = current_ms;

    if (dual_motor_controller_enable(dual) != DUAL_MOTOR_CONTROLLER_OK)
    {
        app_stop_motors();
        console_println(console, "[APP] Motor enable failed!");
        return;
    }

    test_running = true;

    console_println(console, "[APP] Running both motors at 180 RPM.");
}

void app_update(void)
{
    uint32_t current_ms = systick_get_ms();

    if (test_running)
    {


    	if (current_ms - test_start_ms >= APP_TEST_DURATION_MS)
    	{
    		app_stop_motors();
    		console_println(console, "[APP] Test complete.");
    		test_running = false;
    	}
    	else
    	{
    		app_update_motors(current_ms);

    		if (test_running) app_update_print(current_ms);
    	}
    }

    app_update_status_led(current_ms);
    console_process(console);
}
