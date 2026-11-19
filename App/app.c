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
static DUAL_MOTOR_CONTROLLER_t *dual;
static CONSOLE_t *console;

static uint32_t previous_toggle_ms;
static uint32_t previous_control_ms;
static uint32_t previous_print_ms;
static uint32_t test_start_ms;

static bool test_running = false;

static const float rpm_steps[] = { 0.0f, 90.0f, 180.0f, 90.0f, 0.0f, -90.0f, -180.0f, -90.0f };
static uint32_t previous_step_index;

static void app_stop_motors(void)
{
    if (dual != NULL) dual_motor_controller_disable(dual);
}

static void app_update_rpm_targets(uint32_t current_ms)
{
    uint32_t elapsed_ms = current_ms - test_start_ms;

    uint32_t step_index = (elapsed_ms / APP_STEP_PERIOD_MS) % APP_STEP_COUNT;

    if (step_index == previous_step_index) return;

    previous_step_index = step_index;

    float target_rpm = rpm_steps[step_index];

    if (dual_motor_controller_set_rpm(dual, target_rpm, target_rpm) != DUAL_MOTOR_CONTROLLER_OK)
    {
        app_stop_motors();
        test_running = false;
        console_println(console, "[APP] RPM step failed!");
        return;
    }

    console_print(console, "[APP] New target RPM: ");
    console_print_float(console, target_rpm, 2);
    console_println(console, "");
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
        test_running = false;
        console_println(console, "[APP] Motor error!");
        return;
    }
}

static void app_update_print(uint32_t current_ms)
{
    if (current_ms - previous_print_ms < APP_PRINT_PERIOD_MS)
        return;

    previous_print_ms = current_ms;

    console_print(console, "[APP] t=");
    console_print_float(console, (float)(current_ms - test_start_ms) / 1000.0f, 2);

    console_print(console, " | L Target: ");
    console_print_float(console, dual->left->target_rpm, 2);

    console_print(console, " | L RPM: ");
    console_print_float(console, dual->left->measured_rpm, 2);

    console_print(console, " | L PWM: ");
    console_print_float(console, dual->left->control_output, 0);

    console_print(console, " | R Target: ");
    console_print_float(console, dual->right->target_rpm, 2);

    console_print(console, " | R RPM: ");
    console_print_float(console, dual->right->measured_rpm, 2);

    console_print(console, " | R PWM: ");
    console_print_float(console, dual->right->control_output, 0);

    console_println(console, "");
}

void app_init(void)
{
	led = board_get_status_led();
    dual = board_get_dual_motor_controller();
    console = board_get_console();
    if (led == NULL || dual == NULL || console == NULL) return;

    app_stop_motors();

    console_println(console, "\r\n------ DUAL MOTOR TEST ------");

    if (dual_motor_controller_set_feedforward(dual, false) != DUAL_MOTOR_CONTROLLER_OK)
    {
        console_println(console, "[APP] Feedforward setup failed!");
        return;
    }

    if (dual_motor_controller_set_rpm(dual, 0.0f, 0.0f) != DUAL_MOTOR_CONTROLLER_OK)
    {
        console_println(console, "[APP] Target RPM setup failed!");
        return;
    }

    uint32_t current_ms = systick_get_ms();

    previous_toggle_ms = current_ms;
    previous_control_ms = current_ms;
    previous_print_ms = current_ms;
    test_start_ms = current_ms;
    previous_step_index = 0U;

    if (dual_motor_controller_enable(dual) != DUAL_MOTOR_CONTROLLER_OK)
    {
        app_stop_motors();
        console_println(console, "[APP] Motor enable failed!");
        return;
    }

    test_running = true;

    console_println(console, "[APP] Staircase test: ");
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
    		app_update_rpm_targets(current_ms);
    		if (test_running)
			{
    			app_update_motors(current_ms);
    			app_update_print(current_ms);
			}
    	}
    }

    app_update_status_led(current_ms);
    console_process(console);
}
