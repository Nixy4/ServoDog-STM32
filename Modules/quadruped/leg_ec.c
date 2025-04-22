#include "quadruped_def.h"

#define TAG "Easing Coord"

Coord _easing_coord_Linear(EasingCoord* ec)
{
  Coord K = {0.f, 0.f};
  K.X = _easing_base_Linear(ec->step.X);
  K.Z = _easing_base_Linear(ec->step.Z);
  return K;
}

static void elog_d_ec(EasingCoord* ec)
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

void _leg_ec_init(EasingCoord* ec, const EasingCoordConfig* cfg)
{
  ec->mode = cfg->mode;
  ec->function = cfg->function != NULL ? cfg->function : _easing_coord_Linear;
  ec->current = (Coord) {fksp_start.COORD.X, fksp_start.COORD.Z};
  ec->frameCount = cfg->frameCount;
  ec->interval = cfg->interval;
  ec->customData = cfg->customData;
}

void _leg_ec_absolute(EasingCoord* ec, Coord start, Coord stop, uint16_t frames)
{
  elog_i(TAG, "Param | %10s : %20.10f , %20.10f | %20.10f , %20.10f | %10u |", "start", start.X, start.Z, stop.X, stop.Z, frames);

  ec->start = start;
  ec->stop = stop;
  ec->delta.X = stop.X - start.X;
  ec->delta.Z = stop.Z - start.Z;
  ec->current = start;
  ec->frameIndex = 0;
  ec->frameCount = frames ? frames : EASING_FRAME_COUNT_DEFUALT;
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

void _leg_ec_relative(EasingCoord* ec, Coord distance, uint16_t frames)
{
  _leg_ec_absolute(ec, ec->current, (Coord) {ec->current.X + distance.X, ec->current.Z + distance.Z}, frames);
}

void _leg_ec_target(EasingCoord* ec, Coord target, uint16_t frames)
{
  _leg_ec_absolute(ec, ec->current, target, frames);
}

void leg_ec_absolute(LegID id, float x_start, float z_start, float x_stop, float z_stop, uint16_t frames)
{
  Leg* leg = leg_ptr(id);
  if( x_start != x_stop || z_start != z_stop)
  {
    _leg_ec_absolute(&leg->ec, (Coord) {x_start, z_start}, (Coord) {x_stop, z_stop} ,frames);
  }
}

void leg_ec_relative(LegID id, float x_distance, float z_distance, uint16_t frames)
{
  Leg* leg = leg_ptr(id);
  if( x_distance != 0.f || z_distance != 0.f)
  {
    _leg_ec_relative(&leg->ec, (Coord) {x_distance, z_distance}, frames);
  }
}

void leg_ec_target(LegID id, float x_target, float z_target, uint16_t frames)
{
  Leg* leg = leg_ptr(id);
  if( x_target != leg->ec.stop.X || z_target != leg->ec.stop.Z)
  {
    _leg_ec_target(&leg->ec, (Coord) {x_target, z_target}, frames);
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

bool _leg_ec_update(EasingCoord* ec)
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
    ec->step = (Coord) {1.f, 1.f};
    ec->current = ec->direction ? ec->start : ec->stop;
    if (!(ec->mode & EASING_TIMES_INFINITE)) {
      if(ec->times--) {return true;}
    }
  } else {
    ec->step.X = (float)(ec->frameIndex - 1) / (ec->frameCount - 1);
    ec->step.Z = (float)(ec->frameIndex - 1) / (ec->frameCount - 1);
    Coord K = ec->function(ec);
    ec->current.X = ec->direction ? (ec->stop.X - ec->delta.X * K.X) : (ec->start.X + ec->delta.X * K.X);
    ec->current.Z = ec->direction ? (ec->stop.Z - ec->delta.Z * K.Z) : (ec->start.Z + ec->delta.Z * K.Z);
  }
  return true;
}

bool leg_ec_update(LegID id)
{
  Leg* leg = leg_ptr(id);
  bool ret = false;
  if( leg->ec.times != 0)
  {
    ret|=_leg_ec_update(&leg->ec);
    leg_set_coord(id, leg->ec.current.X, leg->ec.current.Z);
  }
  return ret;
}