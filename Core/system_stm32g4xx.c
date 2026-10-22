#include "stm32g4xx.h"

void SystemInit(void)
{
    /* Enable CP10 and CP11: full FPU access */
    SCB->CPACR |= (0xFUL << 20);

    __DSB();
    __ISB();
}
