#include "uart.h"

#define UART_TX_BUFFER_SIZE 512U

static uint8_t tx_buffer[UART_TX_BUFFER_SIZE];
static uint16_t tx_head = 0;
static uint16_t tx_tail = 0;
static USART_TypeDef *tx_instance = NULL;

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

	tx_instance = cfg->USARTx;
	tx_head = 0;
	tx_tail = 0;

	return status;
}

static uint16_t uart_next_index(uint16_t index)
{
    return (uint16_t)((index + 1U) % UART_TX_BUFFER_SIZE);
}

UART_Status_t uart_write_byte(USART_TypeDef *USARTx, uint8_t data)
{
    if (USARTx == NULL) return UART_ERROR_NULL;

    if (USARTx != tx_instance) return UART_ERROR_INVALID_INSTANCE;

    uint16_t next = uart_next_index(tx_head);

    if (next == tx_tail) return UART_ERROR_BUFFER_FULL;

    tx_buffer[tx_head] = data;
    tx_head = next;

    return UART_OK;
}

UART_Status_t uart_write_buffer(USART_TypeDef *USARTx, const uint8_t *buffer, size_t length)
{
    if (USARTx == NULL || buffer == NULL) return UART_ERROR_NULL;

    if (USARTx != tx_instance) return UART_ERROR_INVALID_INSTANCE;

    uint16_t used;

    if (tx_head >= tx_tail) used = tx_head - tx_tail;
    else used = UART_TX_BUFFER_SIZE - tx_tail + tx_head;

    size_t free_space = UART_TX_BUFFER_SIZE - 1U - used;

    if (length > free_space) return UART_ERROR_BUFFER_FULL;

    for (size_t i = 0; i < length; i++)
    {
        tx_buffer[tx_head] = buffer[i];
        tx_head = uart_next_index(tx_head);
    }

    return UART_OK;
}

UART_Status_t uart_write_string(USART_TypeDef *USARTx, const char *str)
{
    if (USARTx == NULL || str == NULL) return UART_ERROR_NULL;

    size_t length = 0;

    while (str[length] != '\0') length++;

    return uart_write_buffer(USARTx, (const uint8_t *)str, length);
}

UART_Status_t uart_process_tx(USART_TypeDef *USARTx)
{
    if (USARTx == NULL) return UART_ERROR_NULL;
    if (USARTx != tx_instance) return UART_ERROR_INVALID_INSTANCE;
    if (tx_head == tx_tail) return UART_OK;
    if (!(USARTx->ISR & USART_ISR_TXE_TXFNF)) return UART_OK;

    USARTx->TDR = tx_buffer[tx_tail];
    tx_tail = uart_next_index(tx_tail);

    return UART_OK;
}

UART_Status_t uart_read_byte(USART_TypeDef *USARTx, uint8_t *data)
{
    if (USARTx == NULL || data == NULL) return UART_ERROR_NULL;

    uint32_t isr = USARTx->ISR;

    if (isr & USART_ISR_ORE)
    {
        USARTx->ICR = USART_ICR_ORECF;
        return UART_ERROR_HARDWARE;
    }

    if (isr & (USART_ISR_FE | USART_ISR_NE | USART_ISR_PE))
    {
        (void)USARTx->RDR;

        USARTx->ICR = USART_ICR_FECF | USART_ICR_NECF | USART_ICR_PECF;

        return UART_ERROR_HARDWARE;
    }

    if (!(isr & USART_ISR_RXNE_RXFNE)) return UART_NO_DATA;

    *data = (uint8_t)USARTx->RDR;

    return UART_OK;
}

UART_Status_t uart_wait_tx_complete(USART_TypeDef *USARTx)
{
    if (USARTx == NULL)
        return UART_ERROR_NULL;

    if (USARTx != tx_instance)
        return UART_ERROR_INVALID_INSTANCE;

    while (tx_head != tx_tail) uart_process_tx(USARTx);

    while (!(USARTx->ISR & USART_ISR_TC)) __NOP();

    return UART_OK;
}
