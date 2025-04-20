#include "cmsis_systick_delay.h"

void delay_us(uint32_t us)
{
  __IO uint32_t curr;
	__IO uint32_t past=SysTick->VAL;
	const uint32_t _1ms=SysTick->LOAD+1;
	const uint32_t dest=(us-((us>0)?1:0))*_1ms/1000;
	uint32_t elapsed=0;
	do{
		curr=SysTick->VAL;
		elapsed+=(curr<past)?(past-curr):(past+_1ms-curr);
		past=curr;
	}while(elapsed<dest);
}

void delay_ms(uint32_t ms)
{
  for(uint32_t i=0;i<ms;i++) {
		delay_us(1000);
	}
}

void delay_ns(uint32_t ns)
{
	__IO uint32_t curr;
	__IO uint32_t past=SysTick->VAL;
	const uint32_t _1ms=SysTick->LOAD+1;
	const uint32_t dest=(ns-((ns>0)?1:0))*_1ms/1000000;
	uint32_t elapsed=0;
	do{
		curr=SysTick->VAL;
		elapsed+=(curr<past)?(past-curr):(past+_1ms-curr);
		past=curr;
	}while(elapsed<dest);
}