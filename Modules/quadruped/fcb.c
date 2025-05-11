#include "quadruped.h"
#include "string.h"

inline void fcb_init(quad_fcb* fcb, fcb_mode mode, uint32_t interval)
{
  memset(fcb, 0, sizeof(quad_fcb));
  fcb->mode = mode;
  fcb->interval = interval;
}

inline void fcb_start(quad_fcb* fcb, uint32_t count)
{
  fcb->count = count;
  fcb->index = 0;
  fcb->last_tick = 0;
}

inline void fcb_next(quad_fcb* fcb)
{
  ++fcb->index;
}

bool fcb_skip(quad_fcb* fcb)
{
  switch (fcb->mode) {
    case FCB_MODE_NONE:
      return false;
    case FCB_MODE_TICK:
      if( HAL_GetTick()-fcb->last_tick > fcb->interval) {
        fcb->last_tick = HAL_GetTick();
        return false;
      } else {
        return true;
      }
    case FCB_MODE_MS:
      delay_ms(fcb->interval);
      return false;
    case FCB_MODE_US:
      delay_us(fcb->interval);
      return false;
    default:
      return false;
  }
}

inline bool fcb_complete(quad_fcb* fcb)
{
  if(fcb->count == 0) {
    return true;
  } else if (fcb->index > fcb->count) {
    return true;
  } else {
    return false;
  }
}

inline bool fcb_last(quad_fcb* fcb)
{
  if (fcb->index == fcb->count) {
    return true;
  } else {
    return false;
  }
}

inline bool fcb_median(quad_fcb* fcb)
{
  if (fcb->index == fcb->count / 2) {
    return true;
  } else {
    return false;
  }
}

inline quad_fp fcb_percentage(quad_fcb* fcb)
{
  return ((quad_fp)fcb->index / (quad_fp)fcb->count);
}

inline uint32_t fcb_current(quad_fcb* fcb)
{
  return fcb->index;
}

inline uint32_t fcb_count(quad_fcb* fcb)
{
  return fcb->count;
}