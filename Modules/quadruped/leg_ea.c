#include "quad_def.h"

#define TAG "Easing Angle"

void leg_ea_init_ptr(EasingAngle* ea, const EasingAngleConfig* cfg)
{
  // memset(ea, 0, sizeof(EasingAngle));
  ea->mode = cfg->mode;
  ea->function = cfg->function != NULL ? cfg->function : _easing_base_Linear;
  ea->frameCount = cfg->frameCount;
  ea->interval = cfg->interval;
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
  Leg* leg = leg_ptr(id);
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
  Leg* leg = leg_ptr(id);
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
  Leg* leg = leg_ptr(id);
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
  Leg* leg = leg_ptr(id);
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
  Leg* leg = leg_ptr(id);
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