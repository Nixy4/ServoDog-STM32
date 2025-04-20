#ifndef __STM32_DELAY_H__
#define __STM32_DELAY_H__

#include "stm32f4xx_hal.h"

void delay_us(uint32_t us);
void delay_ms(uint32_t ms);

#endif // !__STM32_DELAY_H__