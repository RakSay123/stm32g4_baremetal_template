#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <math.h>

#include "app.h"
#include "app_config.h"
#include "device_instances.h"

#include "systick/systick.h"
#include "console/console.h"
#include "motor_controller/motor_controller.h"
#include "dual_motor_controller/dual_motor_controller.h"

typedef struct
{
    const char *name;
    float left_rpm;
    float right_rpm;
    uint32_t duration_ms;
} APP_TestPhase_t;

static const APP_TestPhase_t test_phases[] =
{
    { "Initial idle",          0.0f,    0.0f,  800U },
    { "Left forward",         60.0f,    0.0f, 1400U },
    { "Left stop",             0.0f,    0.0f,  700U },
    { "Left reverse",        -60.0f,    0.0f, 1400U },
    { "Left stop",             0.0f,    0.0f,  700U },
    { "Right forward",         0.0f,   60.0f, 1400U },
    { "Right stop",            0.0f,    0.0f,  700U },
    { "Right reverse",         0.0f,  -60.0f, 1400U },
    { "Right stop",            0.0f,    0.0f,  700U },
    { "Both forward",         90.0f,   90.0f, 1400U },
    { "Both reverse",        -90.0f,  -90.0f, 1400U },
    { "Both stop",             0.0f,    0.0f,  700U },
    { "Counter-rotate A",     70.0f,  -70.0f, 1400U },
    { "Counter-rotate B",    -70.0f,   70.0f, 1400U },
    { "Both stop",             0.0f,    0.0f,  700U },
    { "Asymmetric forward A", 60.0f,  120.0f, 1400U },
    { "Asymmetric forward B",120.0f,   60.0f, 1400U },
    { "Asymmetric reverse A",-60.0f, -120.0f, 1400U },
    { "Asymmetric reverse B",-120.0f, -60.0f, 1400U },
    { "Final stop",            0.0f,    0.0f, 1000U }
};

#define APP_PHASE_COUNT (sizeof(test_phases) / sizeof(test_phases[0]))

static LED_t *led;
static DUAL_MOTOR_CONTROLLER_t *dual;
static CONSOLE_t *console;

static uint32_t previous_toggle_ms;
static uint32_t previous_control_ms;
static uint32_t previous_print_ms;
static uint32_t phase_start_ms;

static uint32_t phase_index;
static uint32_t phase_samples;
static uint32_t left_pass_samples;
static uint32_t right_pass_samples;

static uint32_t left_wrong_samples;
static uint32_t right_wrong_samples;

static uint32_t passed_phases;
static bool test_running;

static void app_stop_motors(void)
{
    if (dual != NULL) dual_motor_controller_disable(dual);
}

static void app_fail(const char *reason)
{
    app_stop_motors();
    test_running = false;

    console_print(console, "[FAIL] ");
    console_println(console, reason);
}

static bool app_wheel_passes(float target, float measured)
{
    if (!isfinite(measured)) return false;

    if (target == 0.0f) return fabsf(measured) < APP_IDLE_RPM_TOLERANCE;

    const bool correct_direction = (target > 0.0f && measured > 0.0f) || (target < 0.0f && measured < 0.0f);

    const bool sufficient_speed = fabsf(measured) >= APP_MIN_SPEED_FRACTION * fabsf(target);

    return correct_direction && sufficient_speed;
}

static bool app_wheel_wrong_direction(float target, float measured)
{
    if (target == 0.0f) return false;

    if (fabsf(measured) < APP_WRONG_RPM_THRESHOLD) return false;

    return (target > 0.0f && measured < 0.0f) || (target < 0.0f && measured > 0.0f);
}

static bool app_start_phase(uint32_t index, uint32_t current_ms)
{
    const APP_TestPhase_t *phase = &test_phases[index];

    phase_index = index;
    phase_start_ms = current_ms;

    phase_samples = 0U;
    left_pass_samples = 0U;
    right_pass_samples = 0U;

    left_wrong_samples = 0U;
    right_wrong_samples = 0U;

    if (dual_motor_controller_set_rpm(dual, phase->left_rpm, phase->right_rpm) != DUAL_MOTOR_CONTROLLER_OK)
    {
        app_fail("Unable to set RPM targets");
        return false;
    }

    console_print(console, "[PHASE] ");
    console_print(console, phase->name);

    console_print(console, " | L=");
    console_print_float(console, phase->left_rpm, 0);

    console_print(console, " | R=");
    console_print_float(console, phase->right_rpm, 0);

    console_println(console, "");

    return true;
}

static bool app_finish_phase(void)
{
    const float left_fraction = phase_samples > 0U ? (float)left_pass_samples / (float)phase_samples : 0.0f;
    const float right_fraction = phase_samples > 0U ? (float)right_pass_samples / (float)phase_samples : 0.0f;

    const bool passed = phase_samples > 0U && left_fraction >= APP_PASS_FRACTION && right_fraction >= APP_PASS_FRACTION;

    console_print(console, passed ? "[PASS] " : "[FAIL] ");
    console_print(console, test_phases[phase_index].name);

    console_print(console, " | L pass: ");
    console_print_float(console, left_fraction * 100.0f, 1);

    console_print(console, "% | R pass: ");
    console_print_float(console, right_fraction * 100.0f, 1);
    console_println(console, "%");

    if (passed) passed_phases++;

    return passed;
}

static void app_update_motors(uint32_t current_ms)
{
    const uint32_t elapsed_ms = current_ms - previous_control_ms;

    if (elapsed_ms < APP_CONTROL_PERIOD_MS) return;

    previous_control_ms = current_ms;

    const float dt_seconds = (float)elapsed_ms / 1000.0f;

    if (dual_motor_controller_update(dual, current_ms, dt_seconds) != DUAL_MOTOR_CONTROLLER_OK)
    {
        app_fail("Motor controller update failed");
        return;
    }

    const uint32_t phase_elapsed = current_ms - phase_start_ms;

    if (phase_elapsed < APP_SETTLE_TIME_MS) return;

    const APP_TestPhase_t *phase = &test_phases[phase_index];

    const float left_measured = dual->left->measured_rpm;

    const float right_measured = dual->right->measured_rpm;

    if (!isfinite(left_measured) || !isfinite(right_measured))
    {
        app_fail("Nonfinite encoder measurement");
        return;
    }

    phase_samples++;

    if (app_wheel_passes(phase->left_rpm, left_measured)) left_pass_samples++;

    if (app_wheel_passes(phase->right_rpm, right_measured)) right_pass_samples++;

    if (app_wheel_wrong_direction(phase->left_rpm, left_measured)) left_wrong_samples++;
    else left_wrong_samples = 0U;

    if (app_wheel_wrong_direction(phase->right_rpm, right_measured)) right_wrong_samples++;
    else right_wrong_samples = 0U;

    if (left_wrong_samples >= APP_WRONG_SAMPLE_LIMIT || right_wrong_samples >= APP_WRONG_SAMPLE_LIMIT) app_fail("Sustained wrong motor direction");
}

static void app_update_print(uint32_t current_ms)
{
    if (current_ms - previous_print_ms < APP_PRINT_PERIOD_MS) return;

    previous_print_ms = current_ms;

    console_print(console, "[DATA] L=");
    console_print_float(console, dual->left->measured_rpm, 2);

    console_print(console, " PWM=");
    console_print_float(console, dual->left->control_output, 0);

    console_print(console, " | R=");
    console_print_float(console, dual->right->measured_rpm, 2);

    console_print(console, " PWM=");
    console_print_float(console, dual->right->control_output, 0);

    console_println(console, "");
}

static void app_update_status_led(uint32_t current_ms)
{
    if (current_ms - previous_toggle_ms < APP_STATUS_LED_PERIOD_MS) return;

    previous_toggle_ms = current_ms;
    led_toggle(led);
}

void app_init(void)
{
    led = board_get_status_led();
    dual = board_get_dual_motor_controller();
    console = board_get_console();

    if (led == NULL || dual == NULL || console == NULL) return;

    app_stop_motors();

    console_println(console, "\r\n------ DIFFERENTIAL DRIVE READINESS TEST ------");

    if (dual_motor_controller_set_feedforward(dual, false) != DUAL_MOTOR_CONTROLLER_OK)
    {
        console_println(console, "[FAIL] Feedforward setup");
        return;
    }

    const uint32_t current_ms = systick_get_ms();

    previous_toggle_ms = current_ms;
    previous_control_ms = current_ms;
    previous_print_ms = current_ms;

    phase_index = 0U;
    passed_phases = 0U;
    test_running = false;

    if (dual_motor_controller_enable(dual) != DUAL_MOTOR_CONTROLLER_OK)
    {
        app_stop_motors();
        console_println(console, "[FAIL] Motor enable");
        return;
    }

    test_running = true;
    app_start_phase(0U, current_ms);
}

void app_update(void)
{
    const uint32_t current_ms = systick_get_ms();

    if (test_running)
    {
        const APP_TestPhase_t *phase = &test_phases[phase_index];

        if (current_ms - phase_start_ms >= phase->duration_ms)
        {
            if (!app_finish_phase()) app_fail("Phase acceptance criteria failed");
            else if (phase_index + 1U >= APP_PHASE_COUNT)
            {
                app_stop_motors();
                test_running = false;

                console_print(console, "[RESULT] Passed ");
                console_print_float(console, (float)passed_phases, 0);

                console_print(console, " / ");
                console_print_float(console, (float)APP_PHASE_COUNT, 0);

                console_println(console, " phases. TEST COMPLETE.");
            }
            else app_start_phase(phase_index + 1U, current_ms);
        }

        if (test_running)
        {
            app_update_motors(current_ms);
            if (test_running) app_update_print(current_ms);
        }
    }

    if (led != NULL) app_update_status_led(current_ms);

    if (console != NULL) console_process(console);
}
