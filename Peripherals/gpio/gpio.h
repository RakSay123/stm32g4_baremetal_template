#pragma once

#include <stdint.h>
#include "stm32g474xx.h"

typedef enum {
	GPIO_OK,
	GPIO_ERR
} GPIO_Status_t;

typedef enum {
	GPIO_MODE_INPUT = 0,
	GPIO_MODE_OUTPUT = 1,
	GPIO_MODE_AF = 2,
	GPIO_MODE_ANALOG = 3
} GPIO_Mode_t;

typedef enum {
	GPIO_PUSH_PULL = 0,
	GPIO_OPEN_DRAIN = 1
} GPIO_OType_t;

typedef enum {
	GPIO_SPEED_VERY_LOW = 0,
	GPIO_SPEED_LOW = 1,
	GPIO_SPEED_HIGH = 2,
	GPIO_SPEED_VERY_HIGH = 3
} GPIO_Speed_t;

typedef enum {
	GPIO_NO_PULL = 0,
	GPIO_PULL_UP = 1,
	GPIO_PULL_DOWN = 2
} GPIO_Pull_t;

typedef enum {
	GPIO_AF0 = 0,
	GPIO_AF1 = 1,
	GPIO_AF2 = 2,
	GPIO_AF3 = 3,
	GPIO_AF4 = 4,
	GPIO_AF5 = 5,
	GPIO_AF6 = 6,
	GPIO_AF7 = 7,
	GPIO_AF8 = 8,
	GPIO_AF9 = 9,
	GPIO_AF10 = 10,
	GPIO_AF11 = 11,
	GPIO_AF12 = 12,
	GPIO_AF13 = 13,
	GPIO_AF14 = 14,
	GPIO_AF15 = 15,
} GPIO_AF_t;

typedef enum {
	GPIO_LOW = 0,
	GPIO_HIGH = 1
} GPIO_State_t;

typedef struct {
	GPIO_TypeDef *port;
	uint8_t pin;
	GPIO_Mode_t mode;
	GPIO_OType_t otype;
	GPIO_Speed_t speed;
	GPIO_Pull_t pull;
	GPIO_AF_t alternate;
} GPIO_Config_t;

typedef struct {
	GPIO_TypeDef *port;
	uint8_t pin;
} GPIO_Pin_t;

typedef struct {
	GPIO_TypeDef *port;
	uint8_t pin;
	GPIO_AF_t alt;
} GPIO_AF_Pin_t;

GPIO_Status_t gpio_enable_clock(GPIO_TypeDef *GPIOx);

GPIO_Status_t gpio_mode_config(GPIO_TypeDef *GPIOx, uint8_t pin, GPIO_Mode_t mode);

GPIO_Status_t gpio_output_type_config(GPIO_TypeDef *GPIOx, uint8_t pin, GPIO_OType_t otype);

GPIO_Status_t gpio_speed_config(GPIO_TypeDef *GPIOx, uint8_t pin, GPIO_Speed_t speed);

GPIO_Status_t gpio_pull_config(GPIO_TypeDef *GPIOx, uint8_t pin, GPIO_Pull_t pull);

GPIO_Status_t gpio_alternate_function_config(GPIO_TypeDef *GPIOx, uint8_t pin, GPIO_AF_t alternate);

GPIO_Status_t gpio_init(GPIO_Config_t *cfg);

GPIO_Status_t gpio_write(GPIO_Pin_t *gpio, GPIO_State_t state);

GPIO_Status_t gpio_toggle(GPIO_Pin_t *gpio);

GPIO_Status_t gpio_toggle_pin(GPIO_Config_t *gpio);

GPIO_State_t gpio_read(GPIO_Pin_t *gpio);

GPIO_State_t gpio_read_pin(GPIO_Config_t *cfg);
