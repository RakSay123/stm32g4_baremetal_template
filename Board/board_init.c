#include "board_init.h"

#include <stdint.h>
#include <stddef.h>

#include "board_config.h"
#include "device_instances.h"

#include "gpio/gpio.h"
#include "systick/systick.h"
#include "uart/uart.h"

#include "led/led.h"

BOARD_Status_t board_init(void)
{
	LED_t *status_led = board_get_status_led();
	if (status_led == NULL) return BOARD_STATUS_LED_ERROR;

	GPIO_Config_t USART2_TX = {
		.port = GPIOA,
		.pin = 2,
		.mode = GPIO_MODE_AF,
		.otype = GPIO_PUSH_PULL,
		.speed = GPIO_SPEED_HIGH,
		.pull = GPIO_NO_PULL,
		.alternate = GPIO_AF7
	};

	GPIO_Config_t USART2_RX = {
		.port = GPIOA,
		.pin = 3,
		.mode = GPIO_MODE_AF,
		.otype = GPIO_PUSH_PULL,
		.speed = GPIO_SPEED_HIGH,
		.pull = GPIO_NO_PULL,
		.alternate = GPIO_AF7
	};

	SYSTICK_Config_t systick_cfg = {
		.clock_hz = BOARD_FCLK_HZ,
		.timebase = SYSTICK_TIMEBASE_1MS
	};

	UART_Config_t uart_cfg = {
		.USARTx = USART2,
		.baud_rate = BAUD_RATE,
		.peripheral_clock_hz = BOARD_FCLK_HZ
	};

	if (gpio_init(&USART2_TX) != GPIO_OK) return BOARD_STATUS_GPIO_ERROR;

	if (gpio_init(&USART2_RX) != GPIO_OK) return BOARD_STATUS_GPIO_ERROR;

	if (systick_init(&systick_cfg) != SYSTICK_OK) return BOARD_STATUS_SYSTICK_ERROR;

	if (uart_init(&uart_cfg) != UART_OK) return BOARD_STATUS_UART_ERROR;

	if (led_init(status_led) != LED_OK) return BOARD_STATUS_LED_ERROR;

	return BOARD_STATUS_OK;
}
