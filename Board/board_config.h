#pragma once

#define BOARD_FCLK_HZ                 16000000U
#define BOARD_SYSTICK_HZ              1000U

#define BOARD_DEBUG_UART_BAUD		  115200

#define BOARD_TIM2_PSC				  1 - 1
#define BOARD_TIM2_ARR                0xFFFFU
#define BOARD_TIM2_CNT                0

#define BOARD_TIM1_PSC				 (1U - 1U)
#define BOARD_TIM1_ARR               (999U - 1U)
#define BOARD_TIM1_CNT               0U


#define KP 							  7.0f
#define KI 							  8.0f
#define KD 						  	  0.0f
