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

#include "systick/systick.h"
#include "led/led.h"
#include "dc_motor/dc_motor.h"
#include "rotary_encoder/rotary_encoder.h"
#include "motor_controller/motor_controller.h"
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
static DC_MOTOR_t *dc_motor;
static ROTARY_ENCODER_t *encoder;
static MOTOR_CONTROLLER_t *controller;
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

    int written = snprintf(buffer + *used, capacity - *used, "%s%lu.%02lu", negative ? "-" : "", whole, fraction);
    if (written < 0 || (size_t)written >= capacity - *used) return false;
    *used += (size_t)written;
    return true;
}

static bool app_log_controller(uint32_t current_ms)
{
    char line[112];
    int written = snprintf(line, sizeof(line), "%lu,", (unsigned long)(current_ms - experiment_start_ms));
    if (written < 0 || (size_t)written >= sizeof(line)) return false;
    size_t used = (size_t)written;

    const float fields[4] = {
        controller->target_rpm,
        controller->measured_rpm,
        controller->error,
        controller->control_output
    };

    for (size_t i = 0U; i < 4U; ++i)
    {
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
    motor_controller_disable(controller);
    dc_motor_brake(dc_motor);
    led_off(led);

    if (app_state == APP_RUNNING)
    {
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

    if (app_mode == APP_MODE_PID)
    {
        if (motor_controller_update(controller, current_ms, dt_seconds) != MOTOR_CONTROLLER_OK)
        {
            app_stop_experiment("ERROR MOTOR");
            return;
        }
    }
    else
    {
        if (rotary_encoder_update_distance(encoder) != ROTARY_ENCODER_OK || rotary_encoder_update_velocity(encoder, current_ms) != ROTARY_ENCODER_OK)
        {
            app_stop_experiment("ERROR ENCODER");
            return;
        }
        controller->measured_rpm = rotary_encoder_get_rpm(encoder);
        controller->target_rpm = 0.0f; // Sentinel: no closed-loop target.
        controller->error = -controller->measured_rpm;
        controller->control_output = (float)experiment_pwm;
    }

    if (!app_log_controller(current_ms)) app_stop_experiment("ERROR LOG");
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
    if (have_previous_stop && (uint32_t)(now - previous_stop_ms) < APP_RESTART_COOLDOWN_MS)
    {
        console_println(console, "ERR COOLDOWN");
        return false;
    }

    motor_controller_disable(controller);
    if (dc_motor_brake(dc_motor) != DC_MOTOR_OK || pid_reset(&controller->pid) != PID_OK || rotary_encoder_set_count_zero(encoder) != ROTARY_ENCODER_OK || motor_controller_set_rpm(controller, target_rpm) != MOTOR_CONTROLLER_OK)
    {
        console_println(console, "ERR START");
        return false;
    }

    app_mode = APP_MODE_PID;
    controller->measured_rpm = 0.0f;
    controller->error = target_rpm;
    controller->control_output = 0.0f;

    encoder->previous_update_ms = now;
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
    motor_controller_enable(controller);
    return true;
}

static bool app_start_pwm_experiment(uint16_t pwm, uint32_t duration_ms)
{
    uint32_t now = systick_get_ms();
    if (have_previous_stop && (uint32_t)(now - previous_stop_ms) < APP_RESTART_COOLDOWN_MS)
    {
        console_println(console, "ERR COOLDOWN");
        return false;
    }

    motor_controller_disable(controller);
    if (dc_motor_brake(dc_motor) != DC_MOTOR_OK || pid_reset(&controller->pid) != PID_OK || rotary_encoder_set_count_zero(encoder) != ROTARY_ENCODER_OK || motor_controller_set_rpm(controller, 0.0f) != MOTOR_CONTROLLER_OK)
    {
        dc_motor_brake(dc_motor);
        console_println(console, "ERR START");
        return false;
    }

    app_mode = APP_MODE_PWM;
    experiment_pwm = pwm;
    controller->measured_rpm = 0.0f;
    controller->error = 0.0f;
    controller->control_output = (float)pwm;

    encoder->previous_update_ms = now;
    previous_controller_update_ms = now;
    previous_led_toggle_ms = now;
    experiment_start_ms = now;
    experiment_duration_ms = duration_ms;
    deferred_busy = false;

    app_state = APP_RUNNING;
    if (dc_motor_set_speed_and_direction(dc_motor, MOTOR_DRIVER_FORWARD, (float)pwm) != DC_MOTOR_OK)
    {
        app_stop_experiment("ERROR MOTOR");

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

    if (app_state != APP_IDLE)
    {
        if (app_state == APP_RUNNING && strcmp(cmd, "STOP") == 0 && strtok(NULL, " \t") == NULL) app_stop_experiment("STOP");
        else deferred_busy = true;
        return;
    }

    if (strcmp(cmd, "HELP") == 0 && strtok(NULL, " \t") == NULL)
    {
        app_print_help();
        return;
    }

    if (strcmp(cmd, "STATUS") == 0 && strtok(NULL, " \t") == NULL)
    {
        app_print_status();
        return;
    }

    if (strcmp(cmd, "STOP") == 0 && strtok(NULL, " \t") == NULL)
    {
        motor_controller_disable(controller);
        dc_motor_brake(dc_motor);
        console_println(console, "OK STOP IDLE");
        return;
    }

    if (strcmp(cmd, "FF") == 0)
    {
        char *mode = strtok(NULL, " \t");
        char *extra = strtok(NULL, " \t");
        if (mode == NULL || extra != NULL || (strcmp(mode, "ON") != 0 && strcmp(mode, "OFF") != 0))
        {
            console_println(console, "ERR FF: expected FF ON or FF OFF");
            return;
        }

        bool enabled = strcmp(mode, "ON") == 0;
        if (motor_controller_set_feedforward(controller, enabled) != MOTOR_CONTROLLER_OK)
        {
            console_println(console, "ERR FF: requested target outside characterized RPM range");
            return;
        }
        console_println(console, enabled ? "OK FF ON" : "OK FF OFF");
        return;
    }

    if (strcmp(cmd, "PID") == 0)
    {
        char *arg_kp = strtok(NULL, " \t");
        char *arg_ki = strtok(NULL, " \t");
        char *arg_kd = strtok(NULL, " \t");
        char *extra = strtok(NULL, " \t");
        float kp, ki, kd;

        if (extra != NULL || !app_parse_float(arg_kp, &kp) || !app_parse_float(arg_ki, &ki) || !app_parse_float(arg_kd, &kd) || kp < 0.0f || kp > APP_MAX_KP || ki < 0.0f || ki > APP_MAX_KI || kd < 0.0f || kd > APP_MAX_KD)
        {
            console_println(console, "ERR PID: expected PID <kp> <ki> <kd> (check bounds)");
            return;
        }

        controller->pid.kp = kp;
        controller->pid.ki = ki;
        controller->pid.kd = kd;
        pid_reset(&controller->pid);
        console_println(console, "OK PID");
        return;
    }

    if (strcmp(cmd, "RUN") == 0)
    {
        char *arg_rpm = strtok(NULL, " \t");
        char *arg_ms = strtok(NULL, " \t");
        char *extra = strtok(NULL, " \t");
        float rpm = experiment_target_rpm;
        uint32_t duration_ms = experiment_duration_ms;

        if ((arg_rpm == NULL && arg_ms != NULL) || (arg_rpm != NULL && arg_ms == NULL) || extra != NULL || (arg_rpm != NULL && !app_parse_float(arg_rpm, &rpm)) || (arg_ms != NULL && !app_parse_uint(arg_ms, &duration_ms)) || rpm < APP_MIN_TARGET_RPM || rpm > APP_MAX_TARGET_RPM || duration_ms < APP_MIN_EXPERIMENT_MS || duration_ms > APP_MAX_EXPERIMENT_MS)
        {
            console_println(console, "ERR RUN: expected RUN <rpm> <ms> (check bounds)");
            return;
        }
        app_start_experiment(rpm, duration_ms);
        return;
    }

    if (strcmp(cmd, "PWM") == 0)
    {
        char *arg_pwm = strtok(NULL, " \t");
        char *arg_ms = strtok(NULL, " \t");
        char *extra = strtok(NULL, " \t");
        uint32_t pwm;
        uint32_t duration_ms;

        if (extra != NULL || !app_parse_uint(arg_pwm, &pwm) || !app_parse_uint(arg_ms, &duration_ms) || pwm > 999U || duration_ms < APP_MIN_EXPERIMENT_MS || duration_ms > APP_MAX_EXPERIMENT_MS)
        {
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
    for (uint32_t i = 0U; i < APP_RX_BYTES_PER_UPDATE; ++i)
    {
        char ch;
        CONSOLE_Status_t status = console_read_char(console, &ch);
        if (status == CONSOLE_NO_DATA) return;
        if (status != CONSOLE_OK)
        {
            if (app_state == APP_RUNNING) app_stop_experiment("ERROR UART_RX");
            else if (app_state == APP_IDLE) console_println(console, "ERR UART_RX");
            command_length = 0U;
            discard_overlong_line = false;
            return;
        }

        if (ch == '\r' || ch == '\n')
        {
            if (discard_overlong_line)
            {
                if (app_state != APP_IDLE) deferred_busy = true;
                else console_println(console, "ERR LINE_TOO_LONG");
            }
            else if (command_length != 0U)
            {
                command_buffer[command_length] = '\0';
                app_handle_command(command_buffer);
            }
            command_length = 0U;
            discard_overlong_line = false;
        }
        else if (ch == '\b' || (uint8_t)ch == 127U)
        {
            if (!discard_overlong_line && command_length > 0U) --command_length;
        }
        else if ((uint8_t)ch >= 32U && (uint8_t)ch <= 126U)
        {
            if (!discard_overlong_line)
            {
                if (command_length + 1U < sizeof(command_buffer)) command_buffer[command_length++] = ch;
                else
                {
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
    dc_motor = board_get_motor();
    encoder = board_get_motor_encoder();
    controller = board_get_motor_controller();
    console = board_get_console();

    if (led == NULL || dc_motor == NULL || encoder == NULL || controller == NULL || console == NULL) return;

    motor_controller_disable(controller);
    if (dc_motor_brake(dc_motor) != DC_MOTOR_OK) return;
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
    if (app_state == APP_RUNNING)
    {
        if ((uint32_t)(now - experiment_start_ms) >= experiment_duration_ms) app_stop_experiment("DONE");
        else
        {
            app_update_status_led(now);
            app_update_motor(now);
        }
    }

    app_flush_end_marker();
    if (app_state == APP_IDLE && deferred_busy && console_println(console, "ERR BUSY (command ignored during RUN)") == CONSOLE_OK) deferred_busy = false;

    for (uint32_t i = 0U; i < APP_TX_SERVICE_CALLS; ++i) console_process(console);
}
