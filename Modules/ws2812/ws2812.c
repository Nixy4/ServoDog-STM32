#include "ws2812.h"

//高低电平延时 时间   CNTX 总时间刻度  CNT1 高电平占刻度 
//该值受 时钟速度 影响  168M STM32F407VE  适配 CNTX = 35  CNT1 = 10 ;
#define CNTX  35			//总时间刻度
#define CNT1  10      //数值0高电平持续时间

#define lop141() {\
	__asm("NOP");\
	__asm("NOP");\
	__asm("NOP");\
	__asm("NOP");\
	__asm("NOP");\
	__asm("NOP");\
	__asm("NOP");\
	__asm("NOP");\
	__asm("NOP");\
	__asm("NOP");\


// 
// 
// 

/**
 * FCLK = 168Mhz
 * 一个指令周期 = 1s/168Mhz = 5.9525ns
 * 
 * for(cnt=0; cnt < CNT1; cnt ++); O3 优化后的汇编代码
 * 
 *       	MOV      r0,#0			; 1指令周期
 * |L0.4|
 *       	ADD      r0,r0,#1   ; 1指令周期
 *       	CMP      r0,#0x23   ; 1指令周期
 *       	MOVGE    r0,#0			; 1指令周期
 *       	BLT      |L0.4|     ; 1指令周期
 * 
 * CNT1 >> (4 * 10 + 1 ) * 5.9525ns        = 244.0525ns
 * CNTX - CNT1 >> (4 * 25 + 1 ) * 5.9525ns = 601.2025
 * CNT1 >> (4* 35 + 1) * 5.9525ns          = 841.3525ns
 */

//写入 数值0  高电平持续 	220ns~380ns  低电平持续 580ns~1.6μs
static void ws2812_WriteBit0(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	int cnt = 0;
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
	for(cnt=0; cnt < CNT1; cnt ++);
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
	for(cnt=0; cnt < CNTX - CNT1; cnt ++);
}

//写入 数值1 高电平持续 580ns~1.6μs  低电平持续 220ns~420ns
static void ws2812_WriteBit1(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	int cnt = 0;
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
	for(cnt=0; cnt < CNTX - CNT1; cnt ++);
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
	for(cnt=0; cnt < CNT1; cnt ++);
}

void ws2812_Write24Bit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin,uint32_t data)
{
	int i = 23;
	//高位先发，按照 GRB 的顺序发送数据。
	for(i = 23 ;i >= 0; i--)
	{
		if(data & (1 << i))  ws2812_WriteBit1(GPIOx, GPIO_Pin);
		else  ws2812_WriteBit0(GPIOx, GPIO_Pin);
	}
}

//更新显示 5个 LED 
void show_16GRB(uint32_t GRB[5])
{
	int i = 0;
//	taskENTER_CRITICAL();
	for(;i < 5;i++)
		ws2812_Write24Bit(GPIOE, GPIO_PIN_5, GRB[i]);
//	taskEXIT_CRITICAL();
}



