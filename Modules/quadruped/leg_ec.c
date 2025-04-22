#include "quadruped_def.h"

#define TAG "Easing Coord"

Coord _easing_coord_zero(EasingCoord* ec)
{
  return (Coord) {ec->current.X, ec->current.Z};
}

void elog_d_ec_ptr(EasingCoord* ec)
{  
  elog_d(TAG, "| %10s | %20.10f |","mode",ec->mode);
  elog_d(TAG, "| %10s | %20.10f |","start.X",ec->start.X);
  elog_d(TAG, "| %10s | %20.10f |","start.Z",ec->start.Z);
  elog_d(TAG, "| %10s | %20.10f |","stop.X",ec->stop.X);
  elog_d(TAG, "| %10s | %20.10f |","stop.Z",ec->stop.Z);
  elog_d(TAG, "| %10s | %20.10f |","delta.X",ec->delta.X);
  elog_d(TAG, "| %10s | %20.10f |","delta.Z",ec->delta.Z);
  elog_d(TAG, "| %10s | %20.10f |","step.X",ec->step.X);
  elog_d(TAG, "| %10s | %20.10f |","step.Z",ec->step.Z);
  elog_d(TAG, "| %10s | %20.10f |","current.X",ec->current.X);
  elog_d(TAG, "| %10s | %20.10f |","current.Z",ec->current.Z);
  elog_d(TAG, "| %10s | %20u |","frameIndex",ec->frameIndex);
  elog_d(TAG, "| %10s | %20u |","frameCount",ec->frameCount);
  elog_d(TAG, "| %10s | %20u |","elapsedTick",ec->elapsedTick);
  elog_d(TAG, "| %10s | %20u |","lastTick",ec->lastTick);
  elog_d(TAG, "| %10s | %20u |","direction",ec->direction);
  elog_d(TAG, "| %10s | %20d |","times",ec->times);
  elog_d(TAG, "| %10s | %20u |","interval",ec->interval);
}

void leg_ec_init_ptr(EasingCoord* ec, const EasingCoordConfig* cfg)
{
  ec->mode = cfg->mode;
  ec->function = cfg->function != NULL ? cfg->function : _easing_coord_zero;
  ec->frameCount = cfg->frameCount;
  ec->interval = cfg->interval;
  ec->customData = cfg->customData;
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
  Leg* leg = leg_ptr(id);
  if( x_start != x_stop || z_start != z_stop)
  {
    leg_ec_absolute_ptr(&leg->ec, (Coord) {x_start, z_start}, (Coord) {x_stop, z_stop});
  }
}

void leg_ec_relative(LegID id, float x_distance, float z_distance)
{
  Leg* leg = leg_ptr(id);
  if( x_distance != 0.f || z_distance != 0.f)
  {
    leg_ec_relative_ptr(&leg->ec, (Coord) {x_distance, z_distance});
  }
}

void leg_ec_target(LegID id, float x_target, float z_target)
{
  Leg* leg = leg_ptr(id);
  if( x_target != leg->ec.stop.X || z_target != leg->ec.stop.Z)
  {
    leg_ec_target_ptr(&leg->ec, (Coord) {x_target, z_target});
  }
}

bool leg_ec_is_complete(LegID id)
{
  Leg* leg = leg_ptr(id);
  if( leg->ec.times == 0)
  {
    return true;
  }
  return false;
}

bool leg_ec_update_ptr(EasingCoord* ec)
{
  // elog_d_ec_ptr(ec);

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
  Leg* leg = leg_ptr(id);
  bool ret = false;
  if( leg->ec.times != 0)
  {
    ret|=leg_ec_update_ptr(&leg->ec);
    leg_set_coord(id, leg->ec.current.X, leg->ec.current.Z);
  }
  return ret;
}