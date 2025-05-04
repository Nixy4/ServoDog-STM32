#pragma once

#include "easing.h"
#include "base.h"
// /*R = Right L = Left F = Front B = Back T = Thighs C = Calves */

#define SERVO_MS_PER_DEGREE     2.f
#define SERVO_DEFUALT_CALC     _easing_calc_InOutBounce

// 舵机对象 F4 = 168Mhz  FPU浮点运算
typedef struct
{
  uint32_t          id;
  easing_calc_fn    lpfnCalc;   //计算回调函数

  QUAD_TYPE            fStart;     //起始值
  QUAD_TYPE            fStop;      //结束值
  QUAD_TYPE            fDelta;     //增量
  uint32_t          uMs;        //经过时间
  QUAD_TYPE            fStep;      //步长
  uint32_t          uMsIndex;   //索引
  QUAD_TYPE            fCurr;      //当前值
  QUAD_TYPE            fOffset;    //偏移量
  volatile uint32_t uLastTick;  //最后一次时间

}servo_t;

servo_t* servo_create(uint8_t id, easing_calc_fn calc, QUAD_TYPE offset);

int servo_init(servo_t* s, uint8_t id, easing_calc_fn calc, QUAD_TYPE offset);

void servo_turn_absolute(servo_t* s, QUAD_TYPE start, QUAD_TYPE stop, QUAD_TYPE ms);
void servo_turn_relative(servo_t* s, QUAD_TYPE distance, QUAD_TYPE ms);
void servo_turn_target(servo_t* s, QUAD_TYPE target, QUAD_TYPE ms);
int servo_turn_update(servo_t* s);

void servo_turn_absolute_block(servo_t* s, QUAD_TYPE start, QUAD_TYPE stop, QUAD_TYPE ms);
void servo_turn_relative_block(servo_t* s, QUAD_TYPE distance, QUAD_TYPE ms);
void servo_turn_target_block(servo_t* s, QUAD_TYPE target, QUAD_TYPE ms);

void servo_set_calc(servo_t* s, easing_calc_fn calc);
void servo_set_angle(servo_t* s, QUAD_TYPE angle, bool auto_delay);