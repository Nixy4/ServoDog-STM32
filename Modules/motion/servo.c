#include "stddef.h"
#include "string.h"
#include "stdlib.h"
#include "pac9685.h"
#include "easing.h"
#include "servo.h"
#include "elog.h"
// #include "aeabi.h"

static const char* TAG = "SERVO";

//angular velocity
#define ANGULAR_VELOCITY_MAX 600.f
#define ANGULAR_VELOCITY_MIN 1.f

servo_t* servo_create(uint8_t id, easing_calc_fn calc, double offset)
{  
  servo_t* e = (servo_t*)malloc(sizeof(servo_t));
  if (e == NULL) return NULL;
  memset(e, 0, sizeof(servo_t));

  e->id               = id;
	e->lpfnCalc         = calc  ?  calc : _easing_calc_Linear;
	e->fStart           = 0;
	e->fStop            = 0;
	e->fDelta           = 0;
	e->uMs              = 0;
	e->fStep            = 0;
  e->uMsIndex         = 0;
	e->fCurr            = 0;
	e->fOffset          = offset;

  return e;
}

int servo_init(servo_t* s, uint8_t id, easing_calc_fn calc, double offset)
{
  if(s == NULL) {
    elog_e(TAG, "servo is null");
    return -1;
  }
  s->id               = id;
  s->lpfnCalc         = calc  ?  calc : _easing_calc_Linear;
  s->fStart           = 0;
  s->fStop            = 0;
  s->fDelta           = 0;
  s->uMs              = 0;
  s->fStep            = 0;
  s->uMsIndex         = 0;
  s->fCurr            = 0;
  s->fOffset          = offset;
  return 0;
}

void servo_turn_absolute(servo_t* s, double start, double stop, double ms)
{
  //参数预处理
  start = start < 0.f ? 0.f : start;
  start = start > 180.f ? 180.f : start;
  stop  = stop  < 0.f ? 0.f : stop;
  stop  = stop  > 180.f ? 180.f : stop;

  if(start == stop) {
    return;
  }
  
  s->fStart   = start;
  s->fStop    = stop;
  s->fDelta   = stop  - start;
  double ms_min = fabs( s->fDelta )* SERVO_MS_PER_DEGREE;
  ms    = ms < ms_min ? ms_min : ms;
  s->uMs      = (uint32_t)(ms+0.5f);
  s->uMsIndex = 0;
  s->fStep    = 0.f;
}

void servo_turn_relative(servo_t* s, double distance, double ms)
{
  double stop = s->fCurr + distance;
  servo_turn_absolute(s, s->fCurr, stop, ms);
}

void servo_turn_target(servo_t* s, double target, double ms)
{
  servo_turn_absolute(s, s->fCurr, target, ms);
}

void servo_turn_absolute_block(servo_t* s, double start, double stop, double ms)
{
  servo_turn_absolute(s, start, stop, ms);
  while(servo_turn_update(s) != 0) {
    HAL_Delay(1);
  }
}

void servo_turn_relative_block(servo_t* s, double distance, double ms)
{
  servo_turn_relative(s, distance, ms);
  while(servo_turn_update(s) != 0) {
    HAL_Delay(1);
  }
}

void servo_turn_target_block(servo_t* s, double target, double ms)
{
  servo_turn_target(s, target, ms);
  while(servo_turn_update(s) != 0) {
    HAL_Delay(1);
  }
}

int servo_turn_update(servo_t* s)
{
  if(HAL_GetTick() == s->uLastTick) {
    return 2;
  }

  if(s->uMs == 0) {
    return 0;
  }

  s->uMsIndex++;

  if(s->uMsIndex > s->uMs) {
    s->uMsIndex = 0;
    s->uMs = 0;
    return 0;
  }
  
  if(s->uMsIndex == s->uMs) {
    //最后1ms
    s->fStep = 1.f;
    s->fCurr = s->fStop;
  } else {
    //不是最后1ms
    s->fStep = (double)(s->uMsIndex-1) / (double)(s->uMs-1);
    // s->fStep = __aeabi_ul2f(s->uMsIndex-1) / __aeabi_ul2f(s->uMs-1); //!
    s->fCurr = s->fStart + s->fDelta * s->lpfnCalc(s->fStep);
  }
  pca9685_set_angle(s->id, s->fCurr);
  s->uLastTick = HAL_GetTick();
  return 1;
}

void servo_set_calc(servo_t* s, easing_calc_fn calc)
{
  if(s == NULL) {
    elog_e(TAG, "servo is null");
    return;
  }
  if(calc == NULL) {
    elog_e(TAG, "calc is null");
    return;
  }
  s->lpfnCalc = calc;
}

void servo_set_angle(servo_t* s, double angle, bool auto_delay)
{
  if(s == NULL) {
    elog_e(TAG, "servo is null");
    return;
  }
  if(angle < 0.f) {
    angle = 0.f;
  }
  if(angle > 180.f) {
    angle = 180.f;
  }

  s->fCurr = angle;
  pca9685_set_angle(s->id, s->fCurr);

  if(auto_delay) {
    double delay = angle * SERVO_MS_PER_DEGREE + 0.5f;
    HAL_Delay((uint32_t)delay);
  }
}
