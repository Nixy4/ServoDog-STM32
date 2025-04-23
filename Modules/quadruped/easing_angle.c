#include "quadruped_def.h"

#define TAG "Easing Angle"

void easing_angle_init(EasingAngle* ea, const EasingAngleConfig* cfg)
{
  ea->mode = cfg->mode;
  ea->function = cfg->function != NULL ? cfg->function : _easing_base_Linear;
  ea->frameCount = cfg->frameCount;
  ea->interval = cfg->interval;
}

void easing_angle_absolute(EasingAngle* ea, float start, float stop, uint16_t frames)
{
  ea->start = start;
  ea->stop = stop;
  ea->delta = stop - start;
  ea->current = start;
  ea->frameIndex = 0;
  ea->frameCount = frames ? frames : EASING_FRAME_COUNT_DEFUALT;
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

void easing_angle_relative(EasingAngle* ea, float distance, uint16_t frames)
{
  easing_angle_absolute(ea, ea->current, ea->current + distance, frames);
}

void easing_angle_target(EasingAngle* ea, float target, uint16_t frames)
{
  easing_angle_absolute(ea, ea->current, target, frames);
}

bool easing_angle_update(EasingAngle* ea)
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

  if(ea->frameIndex > ea->frameCount) //往返
  {
    if (ea->mode & EASING_DIR_BACKANDFORTH) 
    {
      ea->direction = !ea->direction;
      ea->frameIndex = 2;
    } else {
      ea->frameIndex = 1;
    }
  }

  if (ea->frameIndex == ea->frameCount)  //最后一帧
  {
    ea->step = 1.f;
    ea->current = ea->direction ? ea->start : ea->stop;
    if (!(ea->mode & EASING_TIMES_INFINITE)) {
      if(ea->times--) {
        return true;
      }
    }
  } else { //非最后一帧
    ea->step = (float)(ea->frameIndex - 1) / (ea->frameCount - 1);
    ea->current = ea->direction ? 
      (ea->stop - ea->delta * ea->function(ea->step)) : (ea->start + ea->delta * ea->function(ea->step));
  }
  return true;
}