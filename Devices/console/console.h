#pragma once

#include <stddef.h>
#include "uart/uart.h"
#include "gpio/gpio.h"

typedef enum {
	CONSOLE_OK = 0,
	CONSOLE_ERROR_NULL,
	CONSOLE_ERROR_INVALID_CONFIG,
	CONSOLE_ERROR_INVALID_UART,
	CONSOLE_ERROR_INVALID_GPIO
} CONSOLE_Status_t;

typedef struct {
	const UART_Config_t *uart_cfg;
	const GPIO_AF_Pin_t *tx;
	const GPIO_AF_Pin_t *rx;
} CONSOLE_t;

CONSOLE_Status_t console_init(CONSOLE_t *console);

CONSOLE_Status_t console_print(CONSOLE_t *console, const char *str);

CONSOLE_Status_t console_println(CONSOLE_t *console, const char *str);

CONSOLE_Status_t console_print_int(CONSOLE_t *console, int32_t value);

CONSOLE_Status_t console_print_uint(CONSOLE_t *console, uint32_t value);

CONSOLE_Status_t console_print_hex(CONSOLE_t *console, uint32_t value);

CONSOLE_Status_t console_print_float(CONSOLE_t *console, float value, uint8_t decimals);
