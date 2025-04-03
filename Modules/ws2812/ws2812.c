#include "ws2812.h"

//�ߵ͵�ƽ��ʱ ʱ��   CNTX ��ʱ��̶�  CNT1 �ߵ�ƽռ�̶� 
//��ֵ�� ʱ���ٶ� Ӱ��  168M STM32F407VE  ���� CNTX = 35  CNT1 = 10 ;
#define CNTX  35			//��ʱ��̶�
#define CNT1  10      //��ֵ0�ߵ�ƽ����ʱ��

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
 * һ��ָ������ = 1s/168Mhz = 5.9525ns
 * 
 * for(cnt=0; cnt < CNT1; cnt ++); O3 �Ż���Ļ�����
 * 
 *       	MOV      r0,#0			; 1ָ������
 * |L0.4|
 *       	ADD      r0,r0,#1   ; 1ָ������
 *       	CMP      r0,#0x23   ; 1ָ������
 *       	MOVGE    r0,#0			; 1ָ������
 *       	BLT      |L0.4|     ; 1ָ������
 * 
 * CNT1 >> (4 * 10 + 1 ) * 5.9525ns        = 244.0525ns
 * CNTX - CNT1 >> (4 * 25 + 1 ) * 5.9525ns = 601.2025
 * CNT1 >> (4* 35 + 1) * 5.9525ns          = 841.3525ns
 */

//д�� ��ֵ0  �ߵ�ƽ���� 	220ns~380ns  �͵�ƽ���� 580ns~1.6��s
static void ws2812_WriteBit0(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	int cnt = 0;
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
	for(cnt=0; cnt < CNT1; cnt ++);
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
	for(cnt=0; cnt < CNTX - CNT1; cnt ++);
}

//д�� ��ֵ1 �ߵ�ƽ���� 580ns~1.6��s  �͵�ƽ���� 220ns~420ns
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
	//��λ�ȷ������� GRB ��˳�������ݡ�
	for(i = 23 ;i >= 0; i--)
	{
		if(data & (1 << i))  ws2812_WriteBit1(GPIOx, GPIO_Pin);
		else  ws2812_WriteBit0(GPIOx, GPIO_Pin);
	}
}

//������ʾ 5�� LED 
void show_16GRB(uint32_t GRB[5])
{
	int i = 0;
//	taskENTER_CRITICAL();
	for(;i < 5;i++)
		ws2812_Write24Bit(GPIOE, GPIO_PIN_5, GRB[i]);
//	taskEXIT_CRITICAL();
}



