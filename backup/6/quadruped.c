#include "quadruped.h"
#include "stm32_hal_hw_i2c_pca9685.h"
#include "elog.h"
#include "math.h"

#define ____PRIVATE____
#define TAG "QUADRUPED"

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

static const float PI = 3.141592653589793f;
static const float DPI = 6.283185307179586f;
static const float HPI = 1.570796326794897f;

static const float L1 = 80.0f;
static const float L2 = 65.0f;
static const float L3 = 20.0f;
// static const float _L1 = 75.0f;
// static const float L4 = 32.0f;
static const float L5 = 81.54140052758476f; // sqrt(_L1 * _L1 + L4 * L4);
static const float L8 = 15.0f;
static const float L9 = 73.0f;
// static const float R14 = PI / 2;
static const float R15 = 0.4032814817188361f; // atan(L4 / _L1);

static Leg legs[4];

#define LEG_PTR(id) &legs[id]
#define LEG_PTR_RF  &legs[LEG_ID_RF]
#define LEG_PTR_RB  &legs[LEG_ID_RB]
#define LEG_PTR_LB  &legs[LEG_ID_LB]
#define LEG_PTR_LF  &legs[LEG_ID_LF]

#define AMIRRORID(id,x) (id < LEG_ID_LEFT_START ? x : (180.f - x))
#define AMIRRORR(x)     (x)
#define AMIRRORL(x)     (180.f - x)
#define ALIMITT(x)      (x > 180.f ? 180.f : (x < 0.f ? 0.f : x))
#define ALIMITS(x)      (x > 120.f ? 120.f : (x < 0.f ? 0.f : x))
#define AOFFSET(x,off)  (x + off)
#define RADIANS(x)      ((x) * 0.01745329251994329547f)
#define DEGREES(x)      ((x) * 57.29577951308232286465f)

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

static void inverse_kinematics0(Leg* leg, float X, float Z)
{
  float L6, L7, R12, R17, R35, RX7, T, K, RS1, RS2;
  L7 = sqrt(pow(X, 2) + pow(Z, 2));
  R17 = acos((pow(L1, 2) + pow(L7, 2) - pow(L2, 2)) / (2 * L1 * L7));
  T = (pow(L1, 2) + pow(L2, 2) - pow(L7, 2)) / (2 * L1 * L2);
  R12 = acos(T);
  if(X==0) RX7=HPI; else K= Z / X;
  if(X>0&&Z>0) RX7=atan(K);
  else if(X<0&&Z>0) RX7=PI+atan(K);
  else if(X<0&&Z<0) RX7=PI+atan(K);
  else {elog_e(TAG, "RX7");return;}
  RS1 = RX7 - R17;
  R35 = PI - R12 - R15;
  L6 = sqrt(pow(L3, 2) + pow(L5, 2) - 2 * L3 * L5 * cos(R35));
  if(L6 > L8 + L9) {elog_e(TAG, "L6");return;}
  T = (L6 * L6 + L8 * L8 - L9 * L9) / (2 * L6 * L8);
  RS2 = acos(T);
  leg->AS1 = DEGREES(RS1);
  leg->AS2 = DEGREES(RS2);
  leg->RS1 = RS1;
  leg->RS2 = RS2;
  leg->L6 = L6;
  leg->L7 = L7;
  leg->R12 = R12;
  leg->R17 = R17;
  leg->R35 = R35;
  leg->R7X = RX7;
  leg->KCoord.x = X;
  leg->KCoord.z = Z;
  return;
}

static float I1_XZ_to_L7(float X, float Z) {
	return sqrt(pow(X, 2) + pow(Z, 2));
}

static float I2_L7_to_R17(float L7) {
	return acos((pow(L1, 2) + pow(L7, 2) - pow(L2, 2)) / (2 * L1 * L7));
}

static float I3_L7_to_R12(float L7) {
	float T = (pow(L1, 2) + pow(L2, 2) - pow(L7, 2)) / (2 * L1 * L2);
	return acos(T);
}

static float I4_XZ_to_RX7(float X, float Z) {

	if (X ==0) {
		return HPI;
	}

	float K = Z / X;
	if (X >= 0 && Z >= 0) {
		return atan(K);
	} else if (X < 0 && Z >= 0) {
		return PI + atan(K);
	} else if (X < 0 && Z < 0) {
		return PI + atan(K);
	}
	return 0.0;
}

static float I5_R17R7X_to_RS1(float R17, float R7X) {
	return R7X - R17;
}

static float I6_R12_to_R35(float R12) {
	return PI - R12 - R15;
}

static float I7_R35_to_L6(float R35) {
	float L6 = sqrt(pow(L3, 2) + pow(L5, 2) - 2 * L3 * L5 * cos(R35));
	if (L6 > L8 + L9) {
		elog_e(TAG, "L6[%f] > L8 + L9[%f]\n", L6, L8 + L9);
	}
	return L6;
}

static float I8_L6_to_RS2(float L6) {
	float T = (L6 * L6 + L8 * L8 - L9 * L9) / (2 * L6 * L8);
	return acos(T);
}

static void inverse_kinematics1(Leg* leg, float X, float Z)
{
  //运动学计算
  float L6, L7, R17, R12, R35, RX7, RS1, RS2;
  L7 = I1_XZ_to_L7(X, Z);
  R17 = I2_L7_to_R17(L7);
  R12 = I3_L7_to_R12(L7);
  RX7 = I4_XZ_to_RX7(X, Z);
  RS1 = I5_R17R7X_to_RS1(R17, RX7);//!
  R35 = I6_R12_to_R35(R12);
  L6 = I7_R35_to_L6(R35);
  RS2 = I8_L6_to_RS2(L6);//!

  //存储运动学数据
  leg->AS1 = DEGREES(RS1);
  leg->AS2 = DEGREES(RS2);
  leg->RS1 = RS1;
  leg->RS2 = RS2;
  leg->L6 = L6;
  leg->L7 = L7;
  leg->R12 = R12;
  leg->R17 = R17;
  leg->R35 = R35;
  leg->R7X = RX7;
  leg->KCoord.x = X;
  leg->KCoord.z = Z;
}



#define ____PUBLIC____

int leg_init(LegID id, const LegConfig* cfg)
{ 

  Leg* leg = LEG_PTR(id);

  //! Hardware !//
  leg->pcaChannel1 = cfg->pcaChannel1;
  leg->pcaChannel2 = cfg->pcaChannel1;
  leg->offset1 = cfg->offset1;
  leg->offset2 = cfg->offset2;
  leg->angle1 = cfg->angle1;
  leg->angle2 = cfg->angle2;
  pca9685_set_angle(leg->pcaChannel1, leg->angle1);
  pca9685_set_angle(leg->pcaChannel2, leg->angle2);  

  //! Easing Angle !//
  leg->ea_function1 = cfg->ea_function1 != NULL ? cfg->ea_function1 : _easing_calc_Linear;
  leg->ea_function2 = cfg->ea_function2 != NULL ? cfg->ea_function2 : _easing_calc_Linear;
  leg->ea_start1 = 0.f;
  leg->ea_start2 = 0.f;
  leg->ea_stop1 = 0.f;
  leg->ea_stop2 = 0.f;
  leg->ea_delta1 = 0.f;
  leg->ea_delta2 = 0.f;
  leg->ea_step1 = 0.f;
  leg->ea_step2 = 0.f;
  leg->ea_elapsed1 = 0.f;
  leg->ea_elapsed2 = 0.f;
  leg->ea_index1 = 0.f;
  leg->ea_index2 = 0.f;
  leg->ea_lastTick1 = 0.f;
  leg->ea_lastTick2 = 0.f;
  leg->ea_current1 = 0.f;
  leg->ea_current2 = 0.f;

  //! Easing Coord !//
  leg->ec_function = cfg->ec_function != NULL ? cfg->ec_function : _easing_calc_Linear;
  leg->ec_start = (Coord){0.f, 0.f};
  leg->ec_stop = (Coord){0.f, 0.f};
  leg->ec_delta = (Coord){0.f, 0.f};
  leg->ec_step = (Coord){0.f, 0.f};
  leg->ec_elapsed = (Coord){0.f, 0.f};
  leg->ec_index = (Coord){0.f, 0.f};
  leg->ec_lastTick = (Coord){0.f, 0.f};
  leg->ec_current = (Coord){0.f, 0.f};
  
  //! Kinematics !//
  leg->AS1 = 0.f;
  leg->AS2 = 0.f;
  leg->RS1 = 0.f;
  leg->RS2 = 0.f;
  leg->L6 = 0.f;
  leg->L7 = 0.f;
  leg->R12 = 0.f;
  leg->R13 = 0.f;
  leg->R17 = 0.f;
  leg->R35 = 0.f;
  leg->R7X = 0.f;
  leg->KCoord = (Coord){0.f, 0.f};
  
  return 0;
}

void leg_set_angle(LegID id, float a1, float a2)
{
  Leg* leg = LEG_PTR(id);
  //! Hardware !//
  leg->angle1 = a1;
  leg->angle2 = a2;
  if(id < LEG_ID_LEFT_START) {
    a1 = AMIRRORR(a1);
    a2 = AMIRRORR(a2);
  } else {
    a1 = AMIRRORL(a1);
    a2 = AMIRRORL(a2);
  }
  a1 = AOFFSET(a1, leg->offset1);
  a2 = AOFFSET(a2, leg->offset2);
  pca9685_set_angle(leg->pcaChannel1, a1);
  pca9685_set_angle(leg->pcaChannel2, a2);
}

void leg_set_coord(LegID id, float x, float z)
{
  Leg* leg = LEG_PTR(id);
  inverse_kinematics0(leg, x, z);
  leg_set_angle(id, leg->AS1, leg->AS2);
}

void leg_ea_absolute(LegID id, float a1_start, float a1_stop, float a2_start, float a2_stop)
{

}

