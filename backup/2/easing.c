#include "stddef.h"
#include "stdint.h"
#include "stdbool.h"
#include "stdlib.h"
#include "math.h"
#include "easing.h"

static const float PI = 3.1415926;

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