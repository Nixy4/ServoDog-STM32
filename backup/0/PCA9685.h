#ifndef __PCA9685_H
#define __PCA9685_H

#include "stm32f4xx_hal.h"
extern I2C_HandleTypeDef hi2c2;
//IIC2 PB11 ->SDA  PB12 ->SCL

//初始化 pca9685 设置输出频率,初始舵机角度
int PCA9685_init(float hz, uint16_t angle);

//修改pwm输出频率
void PCA9685_setFreq(float freq);

//修改通道占空比 
//num: 通道号 0-15
//on输出高电平位置  off输出低电平位置 范围0-4096
void PCA9685_setPWM(uint8_t num,uint32_t on,uint32_t off);

//设置对应通道转到指定角度
void PCA9685_setAngle(uint8_t num,uint16_t angle);

//设置所有通道的角度
void PCA9685_setAllAngle(uint16_t angle);

//TargetAngle: 目标角度  数组0-7 
//CurrentAngle: 当前角度 数组0-7 
//Step: 歩长 	数组0-7 
int AngleMove(const uint8_t *TargetAngle, uint8_t *CurrentAngle,const uint8_t *Step);

#endif
