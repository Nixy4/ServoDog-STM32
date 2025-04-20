#ifndef __STM32_DEBUG_H__ 
#define __STM32_DEBUG_H__

//! STM32 Handler
#include "stm32f4xx_hal.h"

//! C Header
#include "stdio.h"

//! FreeRTOS Header
#define PRINTF_FREERTOS_ENABLE 0
#if PRINTF_FREERTOS_ENABLE
#include "FreeRTOS.h"
#include "task.h"
void freertos_printf(const char* fmt,...);
#endif

//! Public Function
void hal_printf_init(UART_HandleTypeDef* huart, uint32_t baudrate);
int fputc(int ch,FILE* f);

// #define NONE         "/033[m"
// #define RED          "/033[0;32;31m"
// #define LIGHT_RED    "/033[1;31m"
// #define GREEN        "/033[0;32;32m"
// #define LIGHT_GREEN  "/033[1;32m"
// #define BLUE         "/033[0;32;34m"
// #define LIGHT_BLUE   "/033[1;34m"
// #define DARY_GRAY    "/033[1;30m"
// #define CYAN         "/033[0;36m"
// #define LIGHT_CYAN   "/033[1;36m"
// #define PURPLE       "/033[0;35m"
// #define LIGHT_PURPLE "/033[1;35m"
// #define BROWN        "/033[0;33m"
// #define YELLOW       "/033[1;33m"
// #define LIGHT_GRAY   "/033[0;37m"
// #define WHITE        "/033[1;37m"

#endif //!__STM32_DEBUG_H__
