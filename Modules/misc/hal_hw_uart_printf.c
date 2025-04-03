#include "hal_hw_uart_printf.h"

#include "string.h"
#include "stdarg.h"

static UART_HandleTypeDef* huartx = NULL;

void hal_printf_init(UART_HandleTypeDef* huart, uint32_t baudrate)
{
	huartx = huart;
	huartx->Init.BaudRate = baudrate;
}

#if PRINTF_FREERTOS_ENABLE
void freertos_printf(const char* fmt,...)
{
	char buf[512];
	va_list args;
	va_start(args,fmt);
	vsprintf(buf,fmt,args);
	va_end(args);

	BaseType_t vxx, isInsideISR;
	isInsideISR = xPortIsInsideInterrupt();
	if( isInsideISR )
	{
		vxx = portSET_INTERRUPT_MASK_FROM_ISR();
		HAL_UART_Transmit(huartx,(uint8_t*)buf,strlen(buf),10000);
		portCLEAR_INTERRUPT_MASK_FROM_ISR(vxx);
	}
	else
	{
		taskENTER_CRITICAL();
		HAL_UART_Transmit(huartx,(uint8_t*)buf,strlen(buf),10000);
		taskEXIT_CRITICAL();
	}
}
#else
int fputc(int ch,FILE* f)
{
	HAL_UART_Transmit(huartx,(uint8_t*)&ch,1,1);
	return ch;
}
#endif

