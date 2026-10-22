#include "device_instances.h"

static LED_t status_led = {
	.port = GPIOC,
	.pin = 6,
};

static const GPIO_AF_Pin_t USART2_TX = {
	.port = GPIOA,
	.pin = 2,
	.alt = GPIO_AF7
};

static const GPIO_AF_Pin_t USART2_RX = {
	.port = GPIOA,
	.pin = 3,
	.alt = GPIO_AF7
};

static const UART_Config_t uart_cfg = {
	.USARTx = USART2,
	.baud_rate = BAUD_RATE,
	.peripheral_clock_hz = BOARD_FCLK_HZ
};

static CONSOLE_t console = {
	.uart_cfg = &uart_cfg,
	.tx = &USART2_TX,
	.rx = &USART2_RX
};

LED_t* board_get_status_led(void)
{
	return &status_led;
}

CONSOLE_t* board_get_console(void)
{
	return &console;
}
