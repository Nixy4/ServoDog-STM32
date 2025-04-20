#ifndef WS2812_H
#define WS2812_H

#include "stm32f4xx_hal.h"

#define COLOR_R  0Xff00
#define COLOR_G  0XFF0000
#define COLOR_B  0XFF


static void ws2812_WriteBit0(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
static void ws2812_WriteBit1(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

void ws2812_Write24Bit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin,uint32_t data);
//更新显示 5个 LED 
void show_16GRB(uint32_t GRB[5]);
#endif
