#pragma once

#include <stdint.h>
#include <stddef.h>
#include "stm32g474xx.h"

typedef enum
{
    UART_OK = 0,
    UART_ERROR_NULL,
    UART_ERROR_INVALID_INSTANCE,
    UART_ERROR_INVALID_CONFIG,
    UART_ERROR_TIMEOUT,
    UART_ERROR_HARDWARE
} UART_Status_t;

typedef struct
{
    USART_TypeDef *USARTx;

    uint32_t peripheral_clock_hz;
    uint32_t baud_rate;

} UART_Config_t;

UART_Status_t uart_init(const UART_Config_t *cfg);

UART_Status_t uart_write_byte(USART_TypeDef *USARTx, uint8_t data);

UART_Status_t uart_write_buffer(USART_TypeDef *USARTx, const uint8_t *buffer, size_t length);

UART_Status_t uart_write_string(USART_TypeDef *USARTx, const char *str);

UART_Status_t uart_read_byte(USART_TypeDef *USARTx, uint8_t *data);

UART_Status_t uart_wait_tx_complete(USART_TypeDef *USARTx);
