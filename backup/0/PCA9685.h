#ifndef __PCA9685_H
#define __PCA9685_H

#include "stm32f4xx_hal.h"
extern I2C_HandleTypeDef hi2c2;
//IIC2 PB11 ->SDA  PB12 ->SCL

//��ʼ�� pca9685 �������Ƶ��,��ʼ����Ƕ�
int PCA9685_init(float hz, uint16_t angle);

//�޸�pwm���Ƶ��
void PCA9685_setFreq(float freq);

//�޸�ͨ��ռ�ձ� 
//num: ͨ���� 0-15
//on����ߵ�ƽλ��  off����͵�ƽλ�� ��Χ0-4096
void PCA9685_setPWM(uint8_t num,uint32_t on,uint32_t off);

//���ö�Ӧͨ��ת��ָ���Ƕ�
void PCA9685_setAngle(uint8_t num,uint16_t angle);

//��������ͨ���ĽǶ�
void PCA9685_setAllAngle(uint16_t angle);

//TargetAngle: Ŀ��Ƕ�  ����0-7 
//CurrentAngle: ��ǰ�Ƕ� ����0-7 
//Step: �i�� 	����0-7 
int AngleMove(const uint8_t *TargetAngle, uint8_t *CurrentAngle,const uint8_t *Step);

#endif
