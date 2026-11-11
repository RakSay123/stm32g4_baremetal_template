#pragma once

#define APP_STATUS_LED_PERIOD_MS       500U
#define APP_CONTROL_PERIOD_MS          20U

#define APP_TARGET_RPM                 180.0f
#define APP_EXPERIMENT_DURATION_MS     10000U

#define APP_MIN_TARGET_RPM             30.0f
#define APP_MAX_TARGET_RPM             450.0f
#define APP_MIN_EXPERIMENT_MS          1000U
#define APP_MAX_EXPERIMENT_MS          15000U
#define APP_RESTART_COOLDOWN_MS        1000U

#define APP_MAX_KP                     20.0f
#define APP_MAX_KI                     40.0f
#define APP_MAX_KD                     0.5f

#define APP_COMMAND_BUFFER_SIZE        80U
#define APP_RX_BYTES_PER_UPDATE        16U
#define APP_TX_SERVICE_CALLS           8U
