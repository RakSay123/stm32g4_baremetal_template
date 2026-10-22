#pragma once

#include "board_config.h"

#include "gpio/gpio.h"
#include "uart/uart.h"

#include "led/led.h"
#include "console/console.h"

LED_t* board_get_status_led(void);

CONSOLE_t* board_get_console(void);
