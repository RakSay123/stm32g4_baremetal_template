#include "uart.h"

static UART_Status_t uart_enable_clock(USART_TypeDef *USARTx)
{
	if (USARTx == NULL) return UART_ERROR_NULL;

	if (USARTx == USART1) RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	else if (USARTx == USART2) RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
	else if (USARTx == USART3) RCC->APB1ENR1 |= RCC_APB1ENR1_USART3EN;
	else if (USARTx == UART4) RCC->APB1ENR1 |= RCC_APB1ENR1_UART4EN;
	else if (USARTx == UART5) RCC->APB1ENR1 |= RCC_APB1ENR1_UART5EN;
	else return UART_ERROR_INVALID_INSTANCE;

	return UART_OK;
}

static UART_Status_t uart_config_baud(const UART_Config_t *cfg)
{
	if (cfg == NULL || cfg->USARTx == NULL) return UART_ERROR_NULL;

	if (cfg->peripheral_clock_hz == 0U || cfg->baud_rate == 0U) return UART_ERROR_INVALID_CONFIG;

	uint32_t brr = (uint32_t)(((uint64_t)cfg->peripheral_clock_hz + cfg->baud_rate / 2U) / cfg->baud_rate);

	if (brr < 16U || brr > 0xFFFFU) return UART_ERROR_INVALID_CONFIG;

	cfg->USARTx->BRR = brr;

	return UART_OK;
}

static UART_Status_t uart_enable(const UART_Config_t *cfg)
{
	if (cfg == NULL || cfg->USARTx == NULL) return UART_ERROR_NULL;

	USART_TypeDef *USARTx = cfg->USARTx;

	USARTx->CR1 |= USART_CR1_TE | USART_CR1_RE;
	USARTx->CR1 |= USART_CR1_UE;

	return UART_OK;
}

UART_Status_t uart_init(const UART_Config_t *cfg)
{
	if (cfg == NULL || cfg->USARTx == NULL) return UART_ERROR_NULL;

	if (cfg->baud_rate == 0U || cfg->peripheral_clock_hz == 0U) return UART_ERROR_INVALID_CONFIG;

	UART_Status_t status;

	status = uart_enable_clock(cfg->USARTx);
	if (status != UART_OK) return status;

	USART_TypeDef *USARTx = cfg->USARTx;

	USARTx->CR1 = 0;
	USARTx->CR2 = 0;
	USARTx->CR3 = 0;
	USARTx->PRESC = 0;

	status = uart_config_baud(cfg);
	if (status != UART_OK) return status;

	status = uart_enable(cfg);
	if (status != UART_OK) return status;

	return status;
}

UART_Status_t uart_write_byte(USART_TypeDef *USARTx, uint8_t data)
{
	if (USARTx == NULL) return UART_ERROR_NULL;

	while (!(USARTx->ISR & USART_ISR_TXE_TXFNF));

	USARTx->TDR = data;

	return UART_OK;
}

UART_Status_t uart_write_buffer(USART_TypeDef *USARTx, const uint8_t *buffer, size_t length)
{
	return UART_OK;
}

UART_Status_t uart_write_string(USART_TypeDef *USARTx, const char *str)
{
	if (USARTx == NULL || str == NULL) return UART_ERROR_NULL;

	while (*str != '\0')
	{
		UART_Status_t status = uart_write_byte(USARTx, (uint8_t)*str);
		if (status != UART_OK) return status;

		str++;
	}

	return UART_OK;
}

UART_Status_t uart_read_byte(USART_TypeDef *USARTx, uint8_t *data)
{
	return UART_OK;
}

UART_Status_t uart_wait_tx_complete(USART_TypeDef *USARTx)
{
	return UART_OK;
}
