#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "app.h"
#include "app_config.h"
#include "device_instances.h"
#include "dual_motor.h"

#include "systick/systick.h"
#include "led/led.h"
#include "dc_motor/dc_motor.h"
#include "rotary_encoder/rotary_encoder.h"
#include "motor_controller/motor_controller.h"
#include "dual_motor_controller/dual_motor_controller.h"
#include "console/console.h"

typedef enum {
    APP_IDLE = 0,
    APP_RUNNING,
    APP_CLOSING
} APP_State_t;

typedef enum {
    APP_MODE_PID = 0,
    APP_MODE_PWM
} APP_Mode_t;

static LED_t *led;
static MOTOR_CONTROLLER_t *controller;
static DUAL_MOTOR_CONTROLLER_t *motors;
static CONSOLE_t *console;

static APP_State_t app_state = APP_IDLE;
static APP_Mode_t app_mode = APP_MODE_PID;
static uint16_t experiment_pwm = 0U;
static bool app_ready = false;
static bool have_previous_stop = false;
static bool deferred_busy = false;
static const char *pending_end_reason = NULL;

static uint32_t previous_led_toggle_ms;
static uint32_t previous_controller_update_ms;
static uint32_t experiment_start_ms;
static uint32_t previous_stop_ms;
static uint32_t experiment_duration_ms = APP_EXPERIMENT_DURATION_MS;
static float experiment_target_rpm = APP_TARGET_RPM;

static char command_buffer[APP_COMMAND_BUFFER_SIZE];
static size_t command_length = 0U;
static bool discard_overlong_line = false;

static bool app_append_fixed_2(char *buffer, size_t capacity, size_t *used, float value)
{
    if (!isfinite(value) || value > 100000.0f || value < -100000.0f || *used >= capacity) return false;

    bool negative = value < 0.0f;
    float magnitude = negative ? -value : value;
    uint32_t scaled = (uint32_t)(magnitude * 100.0f + 0.5f);
    unsigned long whole = (unsigned long)(scaled / 100U);
    unsigned long fraction = (unsigned long)(scaled % 100U);

    int written = snprintf(buffer + *used, capacity - *used,
                           "%s%lu.%02lu", negative ? "-" : "", whole, fraction);
    if (written < 0 || (size_t)written >= capacity - *used) return false;
    *used += (size_t)written;
    return true;
}

static bool app_log_controller(uint32_t current_ms)
{
    char line[112];
    int written = snprintf(line, sizeof(line), "%lu,",
                           (unsigned long)(current_ms - experiment_start_ms));
    if (written < 0 || (size_t)written >= sizeof(line)) return false;
    size_t used = (size_t)written;

    const float fields[4] = {
        controller->target_rpm,
        controller->measured_rpm,
        controller->error,
        controller->control_output
    };

    for (size_t i = 0U; i < 4U; ++i) {
        if (!app_append_fixed_2(line, sizeof(line), &used, fields[i])) return false;
        if (used + 2U >= sizeof(line)) return false;
        line[used++] = (i == 3U) ? '\r' : ',';
    }
    line[used++] = '\n';
    line[used] = '\0';

    return console_print(console, line) == CONSOLE_OK;
}

static void app_stop_experiment(const char *reason)
{
    dual_motor_controller_disable(motors);
    led_off(led);

    if (app_state == APP_RUNNING) {
        app_state = APP_CLOSING;
        pending_end_reason = reason;
        previous_stop_ms = systick_get_ms();
        have_previous_stop = true;
    }
}

static void app_flush_end_marker(void)
{
    if (app_state != APP_CLOSING || pending_end_reason == NULL) return;

    char end_line[48];
    int length = snprintf(end_line, sizeof(end_line), "END %s\r\n", pending_end_reason);
    if (length < 0 || (size_t)length >= sizeof(end_line)) return;

    if (console_print(console, end_line) != CONSOLE_OK) return;

    pending_end_reason = NULL;
    app_state = APP_IDLE;

}

static void app_update_status_led(uint32_t current_ms)
{
    if ((uint32_t)(current_ms - previous_led_toggle_ms) < APP_STATUS_LED_PERIOD_MS) return;
    previous_led_toggle_ms = current_ms;
    led_toggle(led);
}

static void app_update_motor(uint32_t current_ms)
{
    uint32_t elapsed_ms = (uint32_t)(current_ms - previous_controller_update_ms);
    if (elapsed_ms < APP_CONTROL_PERIOD_MS) return;

    previous_controller_update_ms = current_ms;
    float dt_seconds = (float)elapsed_ms / 1000.0f;

    if (app_mode == APP_MODE_PID) {
        if (dual_motor_controller_update(motors, current_ms, dt_seconds) != DUAL_MOTOR_CONTROLLER_OK) {
            app_stop_experiment("ERROR MOTOR");
            return;
        }
    } else {
        if (dual_motor_controller_update_open_loop(motors, current_ms, experiment_pwm) != DUAL_MOTOR_CONTROLLER_OK) {
            app_stop_experiment("ERROR ENCODER");
            return;
        }
    }

    if (!app_log_controller(current_ms)) {
        app_stop_experiment("ERROR LOG");
    }
}

static bool app_parse_float(const char *token, float *result)
{
    if (token == NULL || result == NULL) return false;
    char *end = NULL;
    float value = strtof(token, &end);
    if (end == token || *end != '\0' || !isfinite(value)) return false;
    *result = value;
    return true;
}

static bool app_parse_uint(const char *token, uint32_t *result)
{
    if (token == NULL || result == NULL || *token == '-' || *token == '+') return false;
    char *end = NULL;
    unsigned long value = strtoul(token, &end, 10);
    if (end == token || *end != '\0' || value > UINT32_MAX) return false;
    *result = (uint32_t)value;
    return true;
}

static void app_print_status(void)
{
    console_println(console, "STATUS IDLE");
    console_print(console, "PID ");
    console_print_float(console, controller->pid.kp, 5U);
    console_print(console, " ");
    console_print_float(console, controller->pid.ki, 5U);
    console_print(console, " ");
    console_print_float(console, controller->pid.kd, 5U);
    console_println(console, "");
    console_println(console, controller->feedforward_enabled ? "FF ON" : "FF OFF");
    console_print(console, "DEFAULT_RUN ");
    console_print_float(console, experiment_target_rpm, 2U);
    console_print(console, " ");
    console_print_uint(console, experiment_duration_ms);
    console_println(console, "");
}

static void app_print_help(void)
{
    console_println(console, "COMMANDS:");
    console_println(console, "  PID <kp> <ki> <kd>  set gains while idle");
    console_println(console, "  FF ON | FF OFF     enable/disable feedforward while idle");
    console_println(console, "  RUN <rpm> <ms>     run one experiment");
    console_println(console, "  RUN                reuse last run settings");
    console_println(console, "  PWM <0..999> <ms>  open-loop fixed PWM experiment");
    console_println(console, "  STATUS             show settings");
    console_println(console, "  STOP               stop motor");
    console_println(console, "  HELP               show commands");
}

static bool app_start_experiment(float target_rpm, uint32_t duration_ms)
{
    uint32_t now = systick_get_ms();
    if (have_previous_stop &&
        (uint32_t)(now - previous_stop_ms) < APP_RESTART_COOLDOWN_MS) {
        console_println(console, "ERR COOLDOWN");
        return false;
    }

    if (dual_motor_controller_disable(motors) != DUAL_MOTOR_CONTROLLER_OK ||
        dual_motor_controller_reset(motors, now) != DUAL_MOTOR_CONTROLLER_OK ||
        dual_motor_controller_set_rpm(motors, target_rpm) != DUAL_MOTOR_CONTROLLER_OK) {
        console_println(console, "ERR START");
        return false;
    }

    app_mode = APP_MODE_PID;
    motors->motor1->error = target_rpm;
    motors->motor2->error = target_rpm;

    // Consistent epoch for velocity estimation and the CSV timestamps.
    previous_controller_update_ms = now;
    previous_led_toggle_ms = now;
    experiment_start_ms = now;
    experiment_duration_ms = duration_ms;
    experiment_target_rpm = target_rpm;
    deferred_busy = false;

    console_println(console, "OK RUN");
    console_println(console, "BEGIN");
    console_println(console, "time_ms,target_rpm,measured_rpm,error,pwm");

    app_state = APP_RUNNING;
    dual_motor_controller_enable(motors);
    return true;
}

// Open-loop characterization command. PWM is in the same raw units (0..999)
// as controller->control_output and dc_motor_set_speed_and_direction().
// A RUN experiment remains completely separate from this mode.
static bool app_start_pwm_experiment(uint16_t pwm, uint32_t duration_ms)
{
    uint32_t now = systick_get_ms();
    if (have_previous_stop &&
        (uint32_t)(now - previous_stop_ms) < APP_RESTART_COOLDOWN_MS) {
        console_println(console, "ERR COOLDOWN");
        return false;
    }

    // Do NOT call motor_controller_update() during PWM testing.
    // Reset PID and encoder to make transitions between modes deterministic.
    if (dual_motor_controller_disable(motors) != DUAL_MOTOR_CONTROLLER_OK ||
        dual_motor_controller_reset(motors, now) != DUAL_MOTOR_CONTROLLER_OK ||
        dual_motor_controller_set_rpm(motors, 0.0f) != DUAL_MOTOR_CONTROLLER_OK) {
        dual_motor_controller_disable(motors);
        console_println(console, "ERR START");
        return false;
    }

    app_mode = APP_MODE_PWM;
    experiment_pwm = pwm;
    motors->motor1->control_output = (float)pwm;
    motors->motor2->control_output = (float)pwm;

    previous_controller_update_ms = now;
    previous_led_toggle_ms = now;
    experiment_start_ms = now;
    experiment_duration_ms = duration_ms;
    deferred_busy = false;

    // Begin the bounded run before issuing the motor command, so the
    // application timer never starts *after* the motor is powered.
    app_state = APP_RUNNING;
    if (dual_motor_controller_set_pwm(motors, pwm) != DUAL_MOTOR_CONTROLLER_OK) {
        app_stop_experiment("ERROR MOTOR");
        // Nothing is streamed yet; return to idle safely instead.
        app_state = APP_IDLE;
        pending_end_reason = NULL;
        console_println(console, "ERR START");
        return false;
    }

    console_println(console, "OK PWM");
    console_println(console, "BEGIN");
    console_println(console, "time_ms,target_rpm,measured_rpm,error,pwm");
    return true;
}

static void app_handle_command(char *line)
{
    char *cmd = strtok(line, " \t");
    if (cmd == NULL) return;

    if (app_state != APP_IDLE) {
        if (app_state == APP_RUNNING && strcmp(cmd, "STOP") == 0 && strtok(NULL, " \t") == NULL) {
            app_stop_experiment("STOP");
        } else {
            deferred_busy = true;
        }
        return;
    }

    if (strcmp(cmd, "HELP") == 0 && strtok(NULL, " \t") == NULL) {
        app_print_help();
        return;
    }

    if (strcmp(cmd, "STATUS") == 0 && strtok(NULL, " \t") == NULL) {
        app_print_status();
        return;
    }

    if (strcmp(cmd, "STOP") == 0 && strtok(NULL, " \t") == NULL) {
        dual_motor_controller_disable(motors);
        console_println(console, "OK STOP IDLE");
        return;
    }

    if (strcmp(cmd, "FF") == 0) {
        char *mode = strtok(NULL, " \t");
        char *extra = strtok(NULL, " \t");
        if (mode == NULL || extra != NULL ||
            (strcmp(mode, "ON") != 0 && strcmp(mode, "OFF") != 0)) {
            console_println(console, "ERR FF: expected FF ON or FF OFF");
            return;
        }

        bool enabled = strcmp(mode, "ON") == 0;
        if (dual_motor_controller_set_feedforward(motors, enabled) != DUAL_MOTOR_CONTROLLER_OK) {
            console_println(console, "ERR FF: requested target outside characterized RPM range");
            return;
        }
        console_println(console, enabled ? "OK FF ON" : "OK FF OFF");
        return;
    }

    if (strcmp(cmd, "PID") == 0) {
        char *arg_kp = strtok(NULL, " \t");
        char *arg_ki = strtok(NULL, " \t");
        char *arg_kd = strtok(NULL, " \t");
        char *extra = strtok(NULL, " \t");
        float kp, ki, kd;

        if (extra != NULL || !app_parse_float(arg_kp, &kp) ||
            !app_parse_float(arg_ki, &ki) || !app_parse_float(arg_kd, &kd) ||
            kp < 0.0f || kp > APP_MAX_KP ||
            ki < 0.0f || ki > APP_MAX_KI ||
            kd < 0.0f || kd > APP_MAX_KD) {
            console_println(console, "ERR PID: expected PID <kp> <ki> <kd> (check bounds)");
            return;
        }

        // Gains updated atomically after validating the complete command.
        if (dual_motor_controller_set_gains(motors, kp, ki, kd) != DUAL_MOTOR_CONTROLLER_OK) {
            console_println(console, "ERR PID");
            return;
        }
        console_println(console, "OK PID");
        return;
    }

    if (strcmp(cmd, "RUN") == 0) {
        char *arg_rpm = strtok(NULL, " \t");
        char *arg_ms = strtok(NULL, " \t");
        char *extra = strtok(NULL, " \t");
        float rpm = experiment_target_rpm;
        uint32_t duration_ms = experiment_duration_ms;

        if ((arg_rpm == NULL && arg_ms != NULL) ||
            (arg_rpm != NULL && arg_ms == NULL) || extra != NULL ||
            (arg_rpm != NULL && !app_parse_float(arg_rpm, &rpm)) ||
            (arg_ms != NULL && !app_parse_uint(arg_ms, &duration_ms)) ||
            rpm < APP_MIN_TARGET_RPM || rpm > APP_MAX_TARGET_RPM ||
            duration_ms < APP_MIN_EXPERIMENT_MS ||
            duration_ms > APP_MAX_EXPERIMENT_MS) {
            console_println(console, "ERR RUN: expected RUN <rpm> <ms> (check bounds)");
            return;
        }
        app_start_experiment(rpm, duration_ms);
        return;
    }

    if (strcmp(cmd, "PWM") == 0) {
        char *arg_pwm = strtok(NULL, " \t");
        char *arg_ms = strtok(NULL, " \t");
        char *extra = strtok(NULL, " \t");
        uint32_t pwm;
        uint32_t duration_ms;

        if (extra != NULL || !app_parse_uint(arg_pwm, &pwm) ||
            !app_parse_uint(arg_ms, &duration_ms) || pwm > 999U ||
            duration_ms < APP_MIN_EXPERIMENT_MS ||
            duration_ms > APP_MAX_EXPERIMENT_MS) {
            console_println(console, "ERR PWM: expected PWM <0..999> <ms> (check bounds)");
            return;
        }

        app_start_pwm_experiment((uint16_t)pwm, duration_ms);
        return;
    }

    console_println(console, "ERR UNKNOWN_COMMAND (try HELP)");
}

static void app_process_rx(void)
{
    for (uint32_t i = 0U; i < APP_RX_BYTES_PER_UPDATE; ++i) {
        char ch;
        CONSOLE_Status_t status = console_read_char(console, &ch);
        if (status == CONSOLE_NO_DATA) return;
        if (status != CONSOLE_OK) {
            if (app_state == APP_RUNNING) app_stop_experiment("ERROR UART_RX");
            else if (app_state == APP_IDLE) console_println(console, "ERR UART_RX");
            command_length = 0U;
            discard_overlong_line = false;
            return;
        }

        if (ch == '\r' || ch == '\n') {
            if (discard_overlong_line) {
                if (app_state != APP_IDLE) deferred_busy = true;
                else console_println(console, "ERR LINE_TOO_LONG");
            } else if (command_length != 0U) {
                command_buffer[command_length] = '\0';
                app_handle_command(command_buffer);
            }
            command_length = 0U;
            discard_overlong_line = false;
        } else if (ch == '\b' || (uint8_t)ch == 127U) {
            if (!discard_overlong_line && command_length > 0U) --command_length;
        } else if ((uint8_t)ch >= 32U && (uint8_t)ch <= 126U) {
            if (!discard_overlong_line) {
                if (command_length + 1U < sizeof(command_buffer)) {
                    command_buffer[command_length++] = ch;
                } else {
                    command_length = 0U;
                    discard_overlong_line = true;
                }
            }
        }
    }
}

void app_init(void)
{
    app_ready = false;
    app_state = APP_IDLE;

    led = board_get_status_led();
    controller = board_get_motor_controller();
    motors = board_get_dual_motor_controller();
    console = board_get_console();

    if (led == NULL || controller == NULL || motors == NULL || console == NULL) return;

    if (dual_motor_controller_disable(motors) != DUAL_MOTOR_CONTROLLER_OK) return;
    led_off(led);

    app_ready = true;
    console_set_echo(console, true);
    console_println(console, "READY PID_CONSOLE");
    console_println(console, "Type HELP for commands");
}

void app_update(void)
{
    if (!app_ready) return;

    app_process_rx();

    uint32_t now = systick_get_ms();
    if (app_state == APP_RUNNING) {
        if ((uint32_t)(now - experiment_start_ms) >= experiment_duration_ms) {
            app_stop_experiment("DONE");
        } else {
            app_update_status_led(now);
            app_update_motor(now);
        }
    }

    app_flush_end_marker();
    if (app_state == APP_IDLE && deferred_busy &&
        console_println(console, "ERR BUSY (command ignored during RUN)") == CONSOLE_OK) {
        deferred_busy = false;
    }

    for (uint32_t i = 0U; i < APP_TX_SERVICE_CALLS; ++i) {
        console_process(console);
    }
}
