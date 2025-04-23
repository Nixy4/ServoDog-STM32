#include "quadruped_def.h"

#define TAG "Easing Base"

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

float _easing_base_InBounce(const float t)
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

float _easing_base_OutBounce(const float t)
{
	return 1.f - _easing_base_InBounce(1.f - t);
}

float _easing_base_InOutBounce(const float t)
{
	return (t < 0.5f) ? _easing_base_InBounce(t * 2.0f) * 0.5f : 1 - _easing_base_InBounce(2.0f - t * 2.0f) * 0.5f;
}

float _easing_base_InCirc(const float t)
{
	return 1.0f - sqrtf(1.0f - t * t);
}

float _easing_base_OutCirc(const float t)
{
	return 1.f - _easing_base_InCirc(1.f - t);
}

float _easing_base_InOutCirc(const float t)
{
	return (t < 0.5f) ? _easing_base_InCirc(t * 2.0f) * 0.5f : 1 - _easing_base_InCirc(2.0f - t * 2.0f) * 0.5f;
}

float _easing_base_InCubic(const float t)
{
	return t * t * t;
}

float _easing_base_OutCubic(const float t)
{
	return 1.f - _easing_base_InCubic(1.f - t);
}

float _easing_base_InOutCubic(const float t)
{
	return (t < 0.5f) ? _easing_base_InCubic(t * 2.0f) * 0.5f : 1 - _easing_base_InCubic(2.0f - t * 2.0f) * 0.5f;
}

float _easing_base_OutElastic(const float t)
{
	float s = 1 - t;
	return 1 - powf(s, 8) + sinf(t * t * 6.f * (float)PI) * s * s;
}

float _easing_base_InElastic(const float t)
{
	return 1.0f - _easing_base_OutElastic(1.0f - t);
}

float _easing_base_InOutElastic(const float t)
{
	return (t < 0.5f) ? _easing_base_InElastic(t * 2.0f) * 0.5f : 1 - _easing_base_InElastic(2.0f- t * 2.0f) * 0.5f;
}

float _easing_base_InExpo(const float t)
{
	return powf(2, 10 * (t - 1));
}

float _easing_base_OutExpo(const float t)
{
	return 1.0f - powf(2, -10 * t);
}

float _easing_base_InOutExpo(const float t)
{
	return (t < 0.5f) ? _easing_base_InExpo(t * 2.0f) * 0.5f : 1 - _easing_base_InExpo(2.0f - t * 2.0f) * 0.5f;
}

float _easing_base_Linear(const float t)
{
	return t;
}

float _easing_base_InQuad(const float t)
{
	return t * t;
}

float _easing_base_OutQuad(const float t)
{
	return 1.f - _easing_base_InQuad(1.f - t);
}

float _easing_base_InOutQuad(const float t)
{
	return (t < 0.5f) ? _easing_base_InQuad(t * 2.0f) * 0.5f : 1 - _easing_base_InQuad(2.0f - t * 2.0f) * 0.5f;
}

float _easing_base_InQuart(const float t)
{
	return t * t * t * t;
}

float _easing_base_OutQuart(const float t)
{
	return 1.f - _easing_base_InQuart(1.f - t);
}

float _easing_base_InOutQuart(const float t)
{
	return (t < 0.5f) ? _easing_base_InQuart(t * 2.0f) * 0.5f : 1 - _easing_base_InQuart(2.0f - t * 2.0f) * 0.5f;
}

float _easing_base_InQuint(const float t)
{
	return t * t * t * t * t;
}

float _easing_base_OutQuint(const float t)
{
	return 1.f - _easing_base_InQuint(1.f - t);
}

float _easing_base_InOutQuint(const float t)
{
	return (t < 0.5f) ? _easing_base_InQuint(t * 2.0f) * 0.5f : 1 - _easing_base_InQuint(2.0f - t * 2.0f) * 0.5f;
}

float _easing_base_InSine(const float t)
{
	return 1.0f - cosf(t * (PI / 2));
}

float _easing_base_OutSine(const float t)
{
	return 1.f - _easing_base_InSine(1.f - t);
}

float _easing_base_InOutSine(const float t)
{
	return (t < 0.5f) ? _easing_base_InSine(t * 2.0f) * 0.5f : 1 - _easing_base_InSine(2.0f - t * 2.0f) * 0.5f;
}

float _easing_base_InBack(const float t)
{
	return 3 * t * t * t - 2 * t * t;
}

float _easing_base_OutBack(const float t)
{
	return 1.f - _easing_base_InBack(1.f - t);
}

float _easing_base_InOutBack(const float t)
{
	return (t < 0.5f) ? _easing_base_InBack(t * 2.0f) * 0.5f : 1 - _easing_base_InBack(2.0f - t * 2.0f) * 0.5f;
}