#pragma once
#include "stdint.h"
#include "stdbool.h"

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
