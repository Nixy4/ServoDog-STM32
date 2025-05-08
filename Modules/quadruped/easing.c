#include "./define.h"

static const quad_fp DH  = 1. / 22.;
static const quad_fp D1  = 1. / 11.;
static const quad_fp D2  = 2. / 11.;
static const quad_fp D3  = 3. / 11.;
// static const quad_fp D4  = 4. / 11.;
static const quad_fp D5  = 5. / 11.;
static const quad_fp D7  = 7. / 11.;
static const quad_fp IH  = 1. / (1. / 22.);
static const quad_fp I1  = 1. / (1. / 11.);
static const quad_fp I2  = 1. / (2. / 11.);
static const quad_fp I4D = 1. / (4. / 11.) / (4. / 11.);
// static const quad_fp IH  = 1. / DH;
// static const quad_fp I1  = 1. / D1;
// static const quad_fp I2  = 1. / D2;
// static const quad_fp I4D = 1. / D4 / D4;

quad_fp _easing_calc_InBounce(const quad_fp t)
{
	quad_fp s;
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

quad_fp _easing_calc_OutBounce(const quad_fp t)
{
	return 1.f - _easing_calc_InBounce(1.f - t);
}

quad_fp _easing_calc_InOutBounce(const quad_fp t)
{
	return (t < 0.5f) ? _easing_calc_InBounce(t * 2.0f) * 0.5f : 1 - _easing_calc_InBounce(2.0f - t * 2.0f) * 0.5f;
}

quad_fp _easing_calc_InCirc(const quad_fp t)
{
	return 1.0f - sqrtf(1.0f - t * t);
}

quad_fp _easing_calc_OutCirc(const quad_fp t)
{
	return 1.f - _easing_calc_InCirc(1.f - t);
}

quad_fp _easing_calc_InOutCirc(const quad_fp t)
{
	return (t < 0.5f) ? _easing_calc_InCirc(t * 2.0f) * 0.5f : 1 - _easing_calc_InCirc(2.0f - t * 2.0f) * 0.5f;
}

quad_fp _easing_calc_InCubic(const quad_fp t)
{
	return t * t * t;
}

quad_fp _easing_calc_OutCubic(const quad_fp t)
{
	return 1.f - _easing_calc_InCubic(1.f - t);
}

quad_fp _easing_calc_InOutCubic(const quad_fp t)
{
	return (t < 0.5f) ? _easing_calc_InCubic(t * 2.0f) * 0.5f : 1 - _easing_calc_InCubic(2.0f - t * 2.0f) * 0.5f;
}

quad_fp _easing_calc_OutElastic(const quad_fp t)
{
	quad_fp s = 1 - t;
	return 1 - powf(s, 8) + sinf(t * t * 6.f * (quad_fp)PI) * s * s;
}

quad_fp _easing_calc_InElastic(const quad_fp t)
{
	return 1.0f - _easing_calc_OutElastic(1.0f - t);
}

quad_fp _easing_calc_InOutElastic(const quad_fp t)
{
	return (t < 0.5f) ? _easing_calc_InElastic(t * 2.0f) * 0.5f : 1 - _easing_calc_InElastic(2.0f- t * 2.0f) * 0.5f;
}

quad_fp _easing_calc_InExpo(const quad_fp t)
{
	return powf(2, 10 * (t - 1));
}

quad_fp _easing_calc_OutExpo(const quad_fp t)
{
	return 1.0f - powf(2, -10 * t);
}

quad_fp _easing_calc_InOutExpo(const quad_fp t)
{
	return (t < 0.5f) ? _easing_calc_InExpo(t * 2.0f) * 0.5f : 1 - _easing_calc_InExpo(2.0f - t * 2.0f) * 0.5f;
}

quad_fp _easing_calc_Linear(const quad_fp t)
{
	return t;
}

quad_fp _easing_calc_InQuad(const quad_fp t)
{
	return t * t;
}

quad_fp _easing_calc_OutQuad(const quad_fp t)
{
	return 1.f - _easing_calc_InQuad(1.f - t);
}

quad_fp _easing_calc_InOutQuad(const quad_fp t)
{
	return (t < 0.5f) ? _easing_calc_InQuad(t * 2.0f) * 0.5f : 1 - _easing_calc_InQuad(2.0f - t * 2.0f) * 0.5f;
}

quad_fp _easing_calc_InQuart(const quad_fp t)
{
	return t * t * t * t;
}

quad_fp _easing_calc_OutQuart(const quad_fp t)
{
	return 1.f - _easing_calc_InQuart(1.f - t);
}

quad_fp _easing_calc_InOutQuart(const quad_fp t)
{
	return (t < 0.5f) ? _easing_calc_InQuart(t * 2.0f) * 0.5f : 1 - _easing_calc_InQuart(2.0f - t * 2.0f) * 0.5f;
}

quad_fp _easing_calc_InQuint(const quad_fp t)
{
	return t * t * t * t * t;
}

quad_fp _easing_calc_OutQuint(const quad_fp t)
{
	return 1.f - _easing_calc_InQuint(1.f - t);
}

quad_fp _easing_calc_InOutQuint(const quad_fp t)
{
	return (t < 0.5f) ? _easing_calc_InQuint(t * 2.0f) * 0.5f : 1 - _easing_calc_InQuint(2.0f - t * 2.0f) * 0.5f;
}

quad_fp _easing_calc_InSine(const quad_fp t)
{
	return 1.0f - cosf(t * (PI / 2));
}

quad_fp _easing_calc_OutSine(const quad_fp t)
{
	return 1.f - _easing_calc_InSine(1.f - t);
}

quad_fp _easing_calc_InOutSine(const quad_fp t)
{
	return (t < 0.5f) ? _easing_calc_InSine(t * 2.0f) * 0.5f : 1 - _easing_calc_InSine(2.0f - t * 2.0f) * 0.5f;
}

quad_fp _easing_calc_InBack(const quad_fp t)
{
	return 3 * t * t * t - 2 * t * t;
}

quad_fp _easing_calc_OutBack(const quad_fp t)
{
	return 1.f - _easing_calc_InBack(1.f - t);
}

quad_fp _easing_calc_InOutBack(const quad_fp t)
{
	return (t < 0.5f) ? _easing_calc_InBack(t * 2.0f) * 0.5f : 1 - _easing_calc_InBack(2.0f - t * 2.0f) * 0.5f;
}