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


#include "easing.h"

#define PI 3.1415926

static const float DH  = 1. / 22.;
static const float D1  = 1. / 11.;
static const float D2  = 2. / 11.;
static const float D3  = 3. / 11.;
// static const float D4  = 4. / 11.;
static const float D5  = 5. / 11.;
static const float D7  = 7. / 11.;
static const float IH  = 1. / (1. / 22.);
static const float I1  = 1. / (1. / 11.);
static const float I2  = 1. / (2. / 11.);
static const float I4D = 1. / (4. / 11.) / (4. / 11.);
// static const float IH  = 1. / DH;
// static const float I1  = 1. / D1;
// static const float I2  = 1. / D2;
// static const float I4D = 1. / D4 / D4;

float _easing_calc_InBounce(const float t)
{
	float s;
	if (t < D1) {
		s = t - DH;
		s = DH - s * s * IH;
	} else if (t < D3) {
		s = t - D2;
		s = D1 - s * s * I1;
	} else if (t < D7) {
		s = t - D5;
		s = D2 - s * s * I2;
	} else {
		s = t - 1;
		s = 1 - s * s * I4D;
	}
	return s;
}

float _easing_calc_OutBounce(const float t)
{
	return 1.f - _easing_calc_InBounce(1.f - t);
}

float _easing_calc_InOutBounce(const float t)
{
	return (t < 0.5f) ? _easing_calc_InBounce(t * 2.0f) * 0.5f : 1 - _easing_calc_InBounce(2.0f - t * 2.0f) * 0.5f;
}

float _easing_calc_InCirc(const float t)
{
	return 1.0f - sqrtf(1.0f - t * t);
}

float _easing_calc_OutCirc(const float t)
{
	return 1.f - _easing_calc_InCirc(1.f - t);
}

float _easing_calc_InOutCirc(const float t)
{
	return (t < 0.5f) ? _easing_calc_InCirc(t * 2.0f) * 0.5f : 1 - _easing_calc_InCirc(2.0f - t * 2.0f) * 0.5f;
}

float _easing_calc_InCubic(const float t)
{
	return t * t * t;
}

float _easing_calc_OutCubic(const float t)
{
	return 1.f - _easing_calc_InCubic(1.f - t);
}

float _easing_calc_InOutCubic(const float t)
{
	return (t < 0.5f) ? _easing_calc_InCubic(t * 2.0f) * 0.5f : 1 - _easing_calc_InCubic(2.0f - t * 2.0f) * 0.5f;
}

float _easing_calc_OutElastic(const float t)
{
	float s = 1 - t;
	return 1 - powf(s, 8) + sinf(t * t * 6.f * (float)PI) * s * s;
}

float _easing_calc_InElastic(const float t)
{
	return 1.0f - _easing_calc_OutElastic(1.0f - t);
}

float _easing_calc_InOutElastic(const float t)
{
	return (t < 0.5f) ? _easing_calc_InElastic(t * 2.0f) * 0.5f : 1 - _easing_calc_InElastic(2.0f- t * 2.0f) * 0.5f;
}

float _easing_calc_InExpo(const float t)
{
	return powf(2, 10 * (t - 1));
}

float _easing_calc_OutExpo(const float t)
{
	return 1.0f - powf(2, -10 * t);
}

float _easing_calc_InOutExpo(const float t)
{
	return (t < 0.5f) ? _easing_calc_InExpo(t * 2.0f) * 0.5f : 1 - _easing_calc_InExpo(2.0f - t * 2.0f) * 0.5f;
}

float _easing_calc_Linear(const float t)
{
	return t;
}

float _easing_calc_InQuad(const float t)
{
	return t * t;
}

float _easing_calc_OutQuad(const float t)
{
	return 1.f - _easing_calc_InQuad(1.f - t);
}

float _easing_calc_InOutQuad(const float t)
{
	return (t < 0.5f) ? _easing_calc_InQuad(t * 2.0f) * 0.5f : 1 - _easing_calc_InQuad(2.0f - t * 2.0f) * 0.5f;
}

float _easing_calc_InQuart(const float t)
{
	return t * t * t * t;
}

float _easing_calc_OutQuart(const float t)
{
	return 1.f - _easing_calc_InQuart(1.f - t);
}

float _easing_calc_InOutQuart(const float t)
{
	return (t < 0.5f) ? _easing_calc_InQuart(t * 2.0f) * 0.5f : 1 - _easing_calc_InQuart(2.0f - t * 2.0f) * 0.5f;
}

float _easing_calc_InQuint(const float t)
{
	return t * t * t * t * t;
}

float _easing_calc_OutQuint(const float t)
{
	return 1.f - _easing_calc_InQuint(1.f - t);
}

float _easing_calc_InOutQuint(const float t)
{
	return (t < 0.5f) ? _easing_calc_InQuint(t * 2.0f) * 0.5f : 1 - _easing_calc_InQuint(2.0f - t * 2.0f) * 0.5f;
}

float _easing_calc_InSine(const float t)
{
	return 1.0f - cosf(t * (PI / 2));
}

float _easing_calc_OutSine(const float t)
{
	return 1.f - _easing_calc_InSine(1.f - t);
}

float _easing_calc_InOutSine(const float t)
{
	return (t < 0.5f) ? _easing_calc_InSine(t * 2.0f) * 0.5f : 1 - _easing_calc_InSine(2.0f - t * 2.0f) * 0.5f;
}

float _easing_calc_InBack(const float t)
{
	return 3 * t * t * t - 2 * t * t;
}

float _easing_calc_OutBack(const float t)
{
	return 1.f - _easing_calc_InBack(1.f - t);
}

float _easing_calc_InOutBack(const float t)
{
	return (t < 0.5f) ? _easing_calc_InBack(t * 2.0f) * 0.5f : 1 - _easing_calc_InBack(2.0f - t * 2.0f) * 0.5f;
}

/*---------------------------------------------------------------------------------------*/

void easing_init( easing* e, easing_mode_t xMode,easing_calc_fn lpfnCalc,
	uint16_t uFrameCnt, float fOffset, uint16_t uInterval)
{
	e->xMode        = xMode;
	e->lpfnCalc     = lpfnCalc  ?  lpfnCalc : _easing_calc_Linear;
	e->fStart       = 0;
	e->fStop        = 0;
	e->fDelta       = 0;
	e->fCurr        = 0;
	e->fOffset      = fOffset;
	e->uFrameIdx    = 0;
	e->uFrameCnt    = (uFrameCnt < 2) ? 2 : uFrameCnt;
	e->fStep        = 0.f;
	e->uInterval    = uInterval;
	e->uMs          = 0;
	e->bDirection   = xMode & EASING_DIR_REVERSE;
	e->sTimes       = 0;
}

// 开始某个效果,以绝对值形式
void easing_start_absolute(easing* pEasing,float fStart,float fStop)
{
	pEasing->fCurr	= fStart;
  pEasing->fStart = fStart;
  pEasing->fStop  = fStop;
  pEasing->fDelta = fStop - fStart;

  pEasing->uFrameIdx = 0;  // first frame is fStart
  pEasing->fStep = 0.f;

  pEasing->bDirection = pEasing->xMode & EASING_DIR_REVERSE;

  if (pEasing->xMode & EASING_TIMES_INFINITE) 
  {
	pEasing->sTimes = -1;
  } 
  else 
  {
	pEasing->sTimes = (pEasing->xMode & EASING_TIMES_MANYTIMES) ?
					  (pEasing->xMode >> EASING_TIMES_SET) : 1;
	if (pEasing->xMode & EASING_DIR_BACKANDFORTH) pEasing->sTimes *= 2;
  }

  // 获取节拍时刻
  #ifdef easing_get_tick
  pEasing->uMs = easing_get_tick();
  #endif
}

// 开始某个效果,以绝相对值形式
void easing_start_relative(easing* pEasing,float nDistance)
{
  easing_start_absolute(
	pEasing,
	#if 1
	pEasing->fCurr,   // from current pos
	#else
	easing->fStop,    // from stop pos
	#endif
	pEasing->fStop + nDistance);
}

void easing_start_target(easing* pEasing,float fTarget)
{
  if( pEasing->fCurr != fTarget )
		easing_start_absolute(pEasing, pEasing->fCurr, fTarget);
}

// 更新某个效果
void easing_update(easing* pEasing)
{
  // isok
  if (pEasing->sTimes == 0) return;

  // 节拍计算
  #ifdef easing_get_tick
  if (pEasing->uInterval > 0) 
  {
		if (easing_get_tick() < pEasing->uMs)
		{
			return;
		}
		else
		{
			pEasing->uMs = easing_get_tick() + pEasing->uInterval;
		} 
  }
  #endif

  // 下一帧
  ++pEasing->uFrameIdx;

  if (pEasing->uFrameIdx > pEasing->uFrameCnt) 
  {
		if (pEasing->xMode & EASING_DIR_BACKANDFORTH) //如果使能了往返效果
		{
			// 改变方向
			pEasing->bDirection = !pEasing->bDirection;
			// 跳过第一帧
			pEasing->uFrameIdx = 2;
		} 
		else 
		{
			// at first frame
			pEasing->uFrameIdx = 1;
		}
  }

  if (pEasing->uFrameIdx == pEasing->uFrameCnt) //如果帧索引==帧计数(上限)
  {
		// 最后一帧
		pEasing->fStep = 1.f;
		pEasing->fCurr  = pEasing->bDirection ? pEasing->fStart : pEasing->fStop;
		// 减少时间
		if (!(pEasing->xMode & EASING_TIMES_INFINITE))//如果没有使能往返
		{
			if(--pEasing->sTimes){return;}
		}
  } 
  else // 不是最后一帧
  {
	pEasing->fStep = (float)(pEasing->uFrameIdx - 1) / (pEasing->uFrameCnt - 1);

	// 计算位置
	pEasing->fCurr =  
			  pEasing->bDirection ?
			  (pEasing->fStop - pEasing->fDelta * pEasing->lpfnCalc(pEasing->fStep)):
			  (pEasing->fStart + pEasing->fDelta * pEasing->lpfnCalc(pEasing->fStep));
  }
}

// 判断效果是否完成
inline bool easing_isok(easing* pEasing)
{
	return pEasing->sTimes == 0;
}

// 停止效果
inline void easing_stop(easing* pEasing, float fCurr)
{
	pEasing->sTimes = 0;
	pEasing->fCurr  = fCurr;
}

// 获取效果当前位置
inline float easing_get_float(easing* pEasing)
{
	return pEasing->fCurr + pEasing->fOffset;
}

inline int easing_get_int(easing* pEasing)
{
	// float v = easing_get_float(pEasing);
	// int ret = (int)v;
  // if( (v - (int)v) > 0.5f ) return ret+1; else return ret;
	return (int)(easing_get_float(pEasing)+0.5f);
}

inline void easing_set_frame_cnt(easing* pEasing, uint16_t uFrameCnt)
{
	pEasing->uFrameCnt = uFrameCnt;
}

inline uint16_t easing_get_frame_cnt(easing* pEasing)
{
	return pEasing->uFrameCnt;
}