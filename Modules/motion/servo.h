#pragma once

#include "easing.h"

// /*R = Right L = Left F = Front B = Back T = Thighs C = Calves */

// #define SERVO_LEDX_RFT 0
// #define SERVO_LEDX_RBT 1
// #define SERVO_LEDX_LBT 2
// #define SERVO_LEDX_LFT 3
// #define SERVO_LEDX_RBC 4
// #define SERVO_LEDX_RFC 5
// #define SERVO_LEDX_LBC 6
// #define SERVO_LEDX_LFC 7

// #define SERVO_ID_RFT SERVO_LEDX_RFT
// #define SERVO_ID_RFC SERVO_LEDX_RFC
// #define SERVO_ID_RBC SERVO_LEDX_RBC
// #define SERVO_ID_RBT SERVO_LEDX_RBT

// #define SERVO_ID_LFT SERVO_LEDX_LFT
// #define SERVO_ID_LFC SERVO_LEDX_LFC
// #define SERVO_ID_LBC SERVO_LEDX_LBC
// #define SERVO_ID_LBT SERVO_LEDX_LBT
// #define SERVO_ID_MAX 8
// #define SERVO_ID_ERR 0xFFFFFFFF

#define SERVO_MS_PER_DEGREE     2.f
#define SERVO_DEFUALT_CALC     _easing_calc_InOutBounce

typedef struct
{
  uint32_t          id;
  easing_calc_fn    lpfnCalc;   //计算回调函数
  double            fStart;     //起始值
  double            fStop;      //结束值
  double            fDelta;     //增量
  uint32_t          uMs;        //经过时间
  double            fStep;      //步长
  uint32_t          uMsIndex;   //索引
  double            fCurr;      //当前值
  double            fOffset;    //偏移量
  volatile uint32_t uLastTick;  //最后一次时间                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   
}servo_t;

servo_t* servo_create(uint8_t id, easing_calc_fn calc, double offset);
int servo_init(servo_t* s, uint8_t id, easing_calc_fn calc, double offset);

void servo_turn_absolute(servo_t* s, double start, double stop, double ms);
void servo_turn_relative(servo_t* s, double distance, double ms);
void servo_turn_target(servo_t* s, double target, double ms);
void servo_turn_absolute_block(servo_t* s, double start, double stop, double ms);
void servo_turn_relative_block(servo_t* s, double distance, double ms);
void servo_turn_target_block(servo_t* s, double target, double ms);
int servo_turn_update(servo_t* s);

void servo_set_calc(servo_t* s, easing_calc_fn calc);
void servo_set_angle(servo_t* s, double angle, bool auto_delay);