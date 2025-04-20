/*
The MNUI code is licensed under the terms of the GPL v3 license.
Copyright (C) 2024 Nixy·Gan

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef __EASING_H__
#define __EASING_H__

#include "stdint.h"
#include "stdbool.h"
#include "math.h"

#include "stm32f4xx_hal.h"
#define easing_get_tick()  HAL_GetTick()

//! 工作模式
typedef uint8_t easing_mode_t;
/**
 * EASING_MODE_BITCNT       = 0b 0000 0100
 * EASING_MODE_MASK         = 0b 0000 0111
 * 
 * EASING_TIMES_SINGLE      = 0b 0000 0000
 * EASING_TIMES_MANYTIMES   = 0b 0000 0001
 * EASING_TIMES_INFINITE    = 0b 0000 0010
 * 
 * EASING_TIMES_SET         = 0b 0000 0100
 * 
 * EASING_DIR_FORWARD       = 0b 0000 0000
 * EASING_DIR_REVERSE       = 0b 0000 0100
 * EASING_DIR_BACKANDFORTH  = 0b 0000 1000
 * 
 * EASING_MODE_DEFAULT      = 0b 0000 0000
 * 
 * EASING_MODE_NTIMES(n)    = 0b 0000 0001 | 0b 0000 0100 << n
 * EASING_MODE_NTIMES(0)    = 0b 0000 0101
 * EASING_MODE_NTIMES(1)    = 0b 0000 1001
 * 
*/
#define EASING_MODE_BITCNT      (4U)
#define EASING_MODE_MASK        ((1U << EASING_MODE_BITCNT) - 1U)

#define EASING_TIMES_SINGLE     (0U << 0U)                                             // 单次(default)
#define EASING_TIMES_MANYTIMES  (1U << 0U)                                             // 多次
#define EASING_TIMES_INFINITE   (1U << 1U)                                             // 循环

#define EASING_TIMES_SET        (EASING_MODE_BITCNT)

#define EASING_DIR_FORWARD      (0U << 0U)                                             // 正向(default)
#define EASING_DIR_REVERSE      (1U << 2U)                                             // 反向
#define EASING_DIR_BACKANDFORTH (1U << 3U)                                             // 往返

#define EASING_MODE_DEFAULT     EASING_TIMES_SINGLE | EASING_DIR_FORWARD
#define EASING_MODE_NTIMES(n)   EASING_TIMES_MANYTIMES | (n << EASING_TIMES_SET)
#define EASING_INIT(mode,calc,frame) \
  {mode,calc,0,0,0,0,0,0,(frame<2)?2:frame,0.f,0,0,mode&EASING_DIR_REVERSE,0}

//! 计算函数
typedef float (*easing_calc_fn)(const float t);

float _easing_calc_Linear(const float t);       // linear t

float _easing_calc_InQuad(const float t);       // quadratic t^2
float _easing_calc_OutQuad(const float t);
float _easing_calc_InOutQuad(const float t);

float _easing_calc_InCubic(const float t);      // cubic t^3
float _easing_calc_OutCubic(const float t);
float _easing_calc_InOutCubic(const float t);

float _easing_calc_InQuart(const float t);      // quartic t^4
float _easing_calc_OutQuart(const float t);
float _easing_calc_InOutQuart(const float t);

float _easing_calc_InQuint(const float t);      // quintic t^5
float _easing_calc_OutQuint(const float t);
float _easing_calc_InOutQuint(const float t);

float _easing_calc_InSine(const float t);       // sinusoidal 正弦 sin(t)
float _easing_calc_OutSine(const float t);
float _easing_calc_InOutSine(const float t);

float _easing_calc_InExpo(const float t);       // exponential 指数 2^t
float _easing_calc_OutExpo(const float t);
float _easing_calc_InOutExpo(const float t);

float _easing_calc_InCirc(const float t);       // circular 圆形
float _easing_calc_OutCirc(const float t);
float _easing_calc_InOutCirc(const float t);

float _easing_calc_InBack(const float t);       // elastic 衰减三次幂 (s+1)t^3 - st^2
float _easing_calc_OutBack(const float t);
float _easing_calc_InOutBack(const float t);

float _easing_calc_InElastic(const float t);    // elastic 衰减正弦
float _easing_calc_OutElastic(const float t);
float _easing_calc_InOutElastic(const float t);

float _easing_calc_InBounce(const float t);     // back 衰减反弹
float _easing_calc_OutBounce(const float t);
float _easing_calc_InOutBounce(const float t);

//! 结构体
typedef struct _easing_struct
{
  easing_mode_t  xMode;       //移动模式
  easing_calc_fn lpfnCalc;    //计算回调函数
  float          fStart;      //起始值
  float          fStop;       //结束值
  float          fCurr;       //当前值
  float          fOffset;     //偏移量
  float          fDelta;      //增量
  float          fStep;       //步长
  uint16_t       uFrameCnt;   //帧计数
  uint16_t       uFrameIdx;   //帧索引
  bool           bDirection;  //方向 
  int16_t        sTimes;      //次数
  uint32_t       uMs;         //经过时间
  uint16_t       uInterval;   //最小时间间隔
}easing;

//! 功能API
void easing_init( easing* e, easing_mode_t xMode,easing_calc_fn lpfnCalc,
    uint16_t uFrameCnt, float fOffset, uint16_t uInterval);

void easing_start_absolute(easing* pEasing,float fStart,float fStop);
void easing_start_relative(easing* pEasing,float nDistance);
void easing_start_target(easing* pEasing,float fTarget);

void easing_update(easing* pEasing);

bool easing_isok(easing* pEasing);
void easing_stop(easing* pEasing, float fCurr);

float easing_get_float(easing* pEasing);
int easing_get_int(easing* pEasing);

void easing_set_frame_cnt(easing* pEasing, uint16_t uFrameCnt);
uint16_t easing_get_frame_cnt(easing* pEasing);

#endif