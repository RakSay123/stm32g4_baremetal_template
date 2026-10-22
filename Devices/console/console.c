#include <stdint.h>
#include <math.h>
#include "console.h"

CONSOLE_Status_t console_init(CONSOLE_t *console)
{
	if (console == NULL) return CONSOLE_ERROR_NULL;
	if (console->uart_cfg == NULL) return CONSOLE_ERROR_INVALID_UART;
	if (console->tx == NULL || console->rx == NULL) return CONSOLE_ERROR_INVALID_GPIO;

	GPIO_Config_t tx_pin = {
		.port = console->tx->port,
		.pin = console->tx->pin,
		.mode = GPIO_MODE_AF,
		.otype = GPIO_PUSH_PULL,
		.speed = GPIO_SPEED_HIGH,
		.pull = GPIO_NO_PULL,
		.alternate = console->tx->alt
	};

	GPIO_Config_t rx_pin = {
		.port = console->rx->port,
		.pin = console->rx->pin,
		.mode = GPIO_MODE_AF,
		.otype = GPIO_PUSH_PULL,
		.speed = GPIO_SPEED_HIGH,
		.pull = GPIO_NO_PULL,
		.alternate = console->rx->alt
	};

	if ((gpio_init(&tx_pin) != GPIO_OK) || (gpio_init(&rx_pin) != GPIO_OK)) return CONSOLE_ERROR_INVALID_GPIO;

	if (uart_init(console->uart_cfg) != UART_OK) return CONSOLE_ERROR_INVALID_UART;

	return CONSOLE_OK;
}

CONSOLE_Status_t console_print(CONSOLE_t *console, const char *str)
{
	if (console == NULL || str == NULL) return CONSOLE_ERROR_NULL;

	if (console->uart_cfg == NULL || console->uart_cfg->USARTx == NULL) return CONSOLE_ERROR_INVALID_UART;

	UART_Status_t status = uart_write_string(console->uart_cfg->USARTx, str);

	if (status != UART_OK) return CONSOLE_ERROR_INVALID_UART;

	return CONSOLE_OK;
}

CONSOLE_Status_t console_println(CONSOLE_t *console, const char *str)
{
	CONSOLE_Status_t status;

	status = console_print(console, str);
	if (status != CONSOLE_OK) return status;

	status = console_print(console, "\r\n");
	if (status != CONSOLE_OK) return status;

	return status;
}

static void console_u32_to_string(uint32_t value, char *buffer)
{
    char reversed[10];
    uint8_t count = 0;
    uint8_t i = 0;

    do
    {
        reversed[count++] = '0' + (value % 10U);
        value /= 10U;
    } while (value != 0U);

    while (count > 0U)
    {
        buffer[i++] = reversed[--count];
    }

    buffer[i] = '\0';
}

CONSOLE_Status_t console_print_uint(CONSOLE_t *console, uint32_t value)
{
    char buffer[11];

    console_u32_to_string(value, buffer);

    return console_print(console, buffer);
}

CONSOLE_Status_t console_print_int(CONSOLE_t *console, int32_t value)
{
    char buffer[12];
    uint32_t magnitude;
    uint8_t offset = 0;

    if (value < 0)
    {
        buffer[0] = '-';
        offset = 1;

        magnitude = (uint32_t)(-(int64_t)value);
    }
    else
    {
        magnitude = (uint32_t)value;
    }

    console_u32_to_string(magnitude, &buffer[offset]);

    return console_print(console, buffer);
}

CONSOLE_Status_t console_print_hex(CONSOLE_t *console, uint32_t value)
{
    static const char digits[] = "0123456789ABCDEF";
    char buffer[11];

    buffer[0] = '0';
    buffer[1] = 'x';

    for (uint8_t i = 0; i < 8U; i++)
    {
        uint8_t shift = (uint8_t)(28U - 4U * i);
        buffer[2U + i] = digits[(value >> shift) & 0xFU];
    }

    buffer[10] = '\0';

    return console_print(console, buffer);
}

CONSOLE_Status_t console_print_float(CONSOLE_t *console, float value, uint8_t decimals)
{
    if (console == NULL) return CONSOLE_ERROR_NULL;

    if (decimals > 6U) return CONSOLE_ERROR_INVALID_CONFIG;

    if (isnan(value)) return console_print(console, "nan");

    if (isinf(value)) return console_print(console, signbit(value) ? "-inf" : "inf");

    double magnitude = fabs((double)value);

    if (magnitude >= 4294967295.0) return CONSOLE_ERROR_INVALID_CONFIG;

    static const uint32_t powers_of_ten[] = {
        1U, 10U, 100U, 1000U,
        10000U, 100000U, 1000000U
    };

    uint32_t scale = powers_of_ten[decimals];

    uint32_t integer_part = (uint32_t)magnitude;

    double fraction = magnitude - (double)integer_part;
    uint32_t fractional_part = (uint32_t)(fraction * (double)scale + 0.5);

    if (fractional_part >= scale)
    {
        fractional_part = 0U;
        integer_part++;
    }

    char buffer[24];
    uint8_t index = 0;

    if (signbit(value))
    {
        buffer[index++] = '-';
    }

    char integer_buffer[11];
    console_u32_to_string(integer_part, integer_buffer);

    for (uint8_t i = 0; integer_buffer[i] != '\0'; i++)
    {
        buffer[index++] = integer_buffer[i];
    }

    if (decimals > 0U)
    {
        buffer[index++] = '.';

        uint32_t divisor = scale / 10U;

        for (uint8_t i = 0; i < decimals; i++)
        {
            buffer[index++] = '0' + (char)((fractional_part / divisor) % 10U);

            divisor /= 10U;
        }
    }

    buffer[index] = '\0';

    return console_print(console, buffer);
}
