#include "quadruped.h"
#include "stm32_hal_hw_i2c_pca9685.h"
#include "elog.h"
#include "math.h"
#include "string.h"

#include "EasingCoordWalk.h"

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

// static const float PI = 3.141592653589793f;
// static const float DPI = 6.283185307179586f;
// static const float HPI = 1.570796326794897f;

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

static void inverse_keinematics0(Leg* leg, float X, float Z)
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
  leg->IKINE.AS1     = DEGREES(RS1);
  leg->IKINE.AS2     = DEGREES(RS2);
  leg->IKINE.RS1     = RS1;
  leg->IKINE.RS2     = RS2;
  leg->IKINE.L6      = L6;
  leg->IKINE.L7      = L7;
  leg->IKINE.R12     = R12;
  leg->IKINE.R17     = R17;
  leg->IKINE.R35     = R35;
  leg->IKINE.R7X     = RX7;
  leg->IKINE.COORD.X = X;
  leg->IKINE.COORD.Z = Z;
  return;
}

static Coord default_ec_function(EasingCoord* ec)
{
  return (Coord) {ec->current.X, ec->current.Z};
}



static void ea_init(EasingAngle* ea, const EasingAngleConfig* cfg)
{
  // memset(ea, 0, sizeof(EasingAngle));
  ea->mode = cfg->mode;
  ea->function = cfg->function != NULL ? cfg->function : _easing_calc_Linear;
  ea->frameCount = cfg->frameCount;
  ea->interval = cfg->interval;
}

static void ec_init(EasingCoord* ec, const EasingCoordConfig* cfg)
{
  // memset(ec, 0, sizeof(EasingCoord));
  ec->mode = cfg->mode;
  ec->function = cfg->function != NULL ? cfg->function : default_ec_function;
  ec->frameCount = cfg->frameCount;
  ec->interval = cfg->interval;
}

int leg_init(LegID id, const LegConfig* cfg)
{ 
  Leg* leg = LEG_PTR(id);

  //! Clear !//
  memset(leg, 0, sizeof(Leg));

  //! Hardware !//
  leg->pcaChannel1 = cfg->pcaChannel1;
  leg->pcaChannel2 = cfg->pcaChannel2;
  leg->offset1 = cfg->offset1;
  leg->offset2 = cfg->offset2;
  leg->angle1 = cfg->angle1;
  leg->angle2 = cfg->angle2;
  pca9685_set_angle(leg->pcaChannel1, leg->angle1);
  pca9685_set_angle(leg->pcaChannel2, leg->angle2);  

  //! Easing Angle !//
  ea_init(&leg->ea1, &cfg->ea1_config);
  ea_init(&leg->ea2, &cfg->ea2_config);
  
  //! Easing Coord !//
  ec_init(&leg->ec, &cfg->ec_config);
  
  //! Kinematics !//

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
  inverse_keinematics0(leg, x, z);
  leg_set_angle(id, leg->IKINE.AS1, leg->IKINE.AS2);
}

void leg_ea_absolute_ptr(EasingAngle* ea, float start, float stop)
{
  ea->start = start;
  ea->stop = stop;
  ea->delta = stop - start;
  ea->current = start;
  ea->frameIndex = 0;
  ea->step = 0.f;
  ea->direction = ea->mode & EASING_DIR_REVERSE;
  if (ea->mode & EASING_TIMES_INFINITE) {
    ea->times = -1;
  } else {
    ea->times = (ea->mode & EASING_TIMES_MANYTIMES) ? (ea->mode >> EASING_TIMES_SET) : 1;
    if (ea->mode & EASING_DIR_BACKANDFORTH) ea->times *= 2;
  }
  ea->elapsedTick = 0;
  ea->lastTick = 0;
}

void leg_ea_relative_ptr(EasingAngle* ea, float distance)
{
  leg_ea_absolute_ptr(ea, ea->current, ea->current + distance);
}

void leg_ea_target_ptr(EasingAngle* ea, float target)
{
  leg_ea_absolute_ptr(ea, ea->current, target);
}

void leg_ea_absolute(LegID id, float a1_start, float a1_stop, float a2_start, float a2_stop)
{
  Leg* leg = LEG_PTR(id);
  if( a1_start != a1_stop)
  {
    leg_ea_absolute_ptr(&leg->ea1, a1_start, a1_stop);
  }
  if( a2_start != a2_stop)
  {
    leg_ea_absolute_ptr(&leg->ea2, a2_start, a2_stop);
  }
}

void leg_ea_relative(LegID id, float distance1, float distance2)
{
  Leg* leg = LEG_PTR(id);
  if( distance1 != 0.f)
  {
    leg_ea_relative_ptr(&leg->ea1, distance1);
  }
  if( distance2 != 0.f)
  {
    leg_ea_relative_ptr(&leg->ea2, distance2);
  }
}

void leg_ea_target(LegID id, float target1, float target2)
{
  Leg* leg = LEG_PTR(id);
  if( target1 != leg->ea1.stop)
  {
    leg_ea_target_ptr(&leg->ea1, target1);
  }
  if( target2 != leg->ea2.stop)
  {
    leg_ea_target_ptr(&leg->ea2, target2);
  }
}

bool leg_ea_is_complete(LegID id)
{
  Leg* leg = LEG_PTR(id);
  if( leg->ea1.times == 0 && leg->ea2.times == 0)
  {
    return true;
  }
  return false;
}

bool leg_ea_update_ptr(EasingAngle* ea)
{
  if (ea->times == 0) return false;

#if defined(EASING_GET_TICK)

  if(ea->interval > 0)
  {
    uint32_t currentTick = EASING_GET_TICK();
    if(currentTick < ea->elapsedTick) {
      return true;
    } else {
      ea->elapsedTick = currentTick + ea->interval;
    }
  }

#endif

  ea->frameIndex++;

  if(ea->frameIndex > ea->frameCount) 
  {
    if (ea->mode & EASING_DIR_BACKANDFORTH) 
    {
      ea->direction = !ea->direction;
      ea->frameIndex = 2;
    } else {
      ea->frameIndex = 1;
    }
  }

  if (ea->frameIndex == ea->frameCount) 
  {
    ea->step = 1.f;
    ea->current = ea->direction ? ea->start : ea->stop;
    if (!(ea->mode & EASING_TIMES_INFINITE)) {
      if(ea->times--) {return true;}
    }
  } else {
    ea->step = (float)(ea->frameIndex - 1) / (ea->frameCount - 1);
    ea->current = ea->direction ? 
      (ea->stop - ea->delta * ea->function(ea->step)) : (ea->start + ea->delta * ea->function(ea->step));
  }
  return true;
}

bool leg_ea_update(LegID id)
{
  Leg* leg = LEG_PTR(id);
  bool ret = false;
  if( leg->ea1.times != 0)
  {
    ret|=leg_ea_update_ptr(&leg->ea1);
    pca9685_set_angle(leg->pcaChannel1, leg->ea1.current);
  }
  if( leg->ea2.times != 0)
  {
    ret|=leg_ea_update_ptr(&leg->ea2);
    pca9685_set_angle(leg->pcaChannel2, leg->ea2.current);
  }
  return ret;
}

void leg_ec_absolute_ptr(EasingCoord* ec, Coord start, Coord stop)
{
  ec->start = start;
  ec->stop = stop;
  ec->delta.X = stop.X - start.X;
  ec->delta.Z = stop.Z - start.Z;
  ec->current = start;
  ec->frameIndex = 0;
  ec->step = (Coord) {0.f, 0.f};
  ec->direction = ec->mode & EASING_DIR_REVERSE;
  if (ec->mode & EASING_TIMES_INFINITE) {
    ec->times = -1;
  } else {
    ec->times = (ec->mode & EASING_TIMES_MANYTIMES) ? (ec->mode >> EASING_TIMES_SET) : 1;
    if (ec->mode & EASING_DIR_BACKANDFORTH) ec->times *= 2;
  }
  ec->elapsedTick = 0;
  ec->lastTick = 0;
}

void leg_ec_relative_ptr(EasingCoord* ec, Coord distance)
{
  leg_ec_absolute_ptr(ec, ec->current, (Coord) {ec->current.X + distance.X, ec->current.Z + distance.Z});
}

void leg_ec_target_ptr(EasingCoord* ec, Coord target)
{
  leg_ec_absolute_ptr(ec, ec->current, target);
}

void leg_ec_absolute(LegID id, float x_start, float z_start, float x_stop, float z_stop)
{
  Leg* leg = LEG_PTR(id);
  if( x_start != x_stop || z_start != z_stop)
  {
    leg_ec_absolute_ptr(&leg->ec, (Coord) {x_start, z_start}, (Coord) {x_stop, z_stop});
  }
}

void leg_ec_relative(LegID id, float x_distance, float z_distance)
{
  Leg* leg = LEG_PTR(id);
  if( x_distance != 0.f || z_distance != 0.f)
  {
    leg_ec_relative_ptr(&leg->ec, (Coord) {x_distance, z_distance});
  }
}

void leg_ec_target(LegID id, float x_target, float z_target)
{
  Leg* leg = LEG_PTR(id);
  if( x_target != leg->ec.stop.X || z_target != leg->ec.stop.Z)
  {
    leg_ec_target_ptr(&leg->ec, (Coord) {x_target, z_target});
  }
}

bool leg_ec_is_complete(LegID id)
{
  Leg* leg = LEG_PTR(id);
  if( leg->ec.times == 0)
  {
    return true;
  }
  return false;
}

bool leg_ec_update_ptr(EasingCoord* ec)
{
  if (ec->times == 0) return false;

#if defined(EASING_GET_TICK)
  if(ec->interval > 0)
  {
    uint32_t currentTick = EASING_GET_TICK();
    if(currentTick < ec->elapsedTick) {
      return true;
    } else {
      ec->elapsedTick = currentTick + ec->interval;
    }
  }
#endif

  ec->frameIndex++;

  if(ec->frameIndex > ec->frameCount) 
  {
    if (ec->mode & EASING_DIR_BACKANDFORTH) 
    {
      ec->direction = !ec->direction;
      ec->frameIndex = 2;
    } else {
      ec->frameIndex = 1;
    }
  }

  if (ec->frameIndex == ec->frameCount) 
  {
    ec->step.X = 1.f;
    ec->step.Z = 1.f;
    ec->current.X = ec->direction ? ec->start.X : ec->stop.X;
    ec->current.Z = ec->direction ? ec->start.Z : ec->stop.Z;
    if (!(ec->mode & EASING_TIMES_INFINITE)) {
      if(ec->times--) {return true;}
    }
  } else {
    ec->step.X = (float)(ec->frameIndex - 1) / (ec->frameCount - 1);
    ec->step.Z = (float)(ec->frameIndex - 1) / (ec->frameCount - 1);
    Coord e = ec->function(ec);
    ec->current.X = ec->direction ? (ec->stop.X - ec->delta.X * e.X) : (ec->start.X + ec->delta.X * e.X);
    ec->current.Z = ec->direction ? (ec->stop.Z - ec->delta.Z * e.Z) : (ec->start.Z + ec->delta.Z * e.Z);
  }
  return true;
}

bool leg_ec_update(LegID id)
{
  Leg* leg = LEG_PTR(id);
  bool ret = false;
  if( leg->ec.times != 0)
  {
    ret|=leg_ec_update_ptr(&leg->ec);
    leg_set_coord(id, leg->ec.current.X, leg->ec.current.Z);
  }
  return ret;
}




