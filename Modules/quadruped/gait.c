#include "gait.h"

#define TAG "GAIT"

int gait_equation(gait_t* g)
{
  gait_data_t data = {0};

  QUAD_TYPE T, Kx, Kz, x_swing,z_swing, x_support,z_support;
  QUAD_TYPE t = g->t;
  QUAD_TYPE period = g->peried;
  QUAD_TYPE swingTime = g->swingTime;
  QUAD_TYPE swingWidth = g->swingWidth;
  QUAD_TYPE swingHeight = g->swingHeight;
  
  //* 计算 *//
  if( t <= swingTime ) 
  {
    T = _2PI * t / swingTime;
    Kx = (T-sin(T)) / _2PI;
    Kz = (1-cos(T)) / 2.f;
    x_swing = swingWidth * Kx;
    z_swing = swingHeight * Kz;
    x_support = swingWidth -swingWidth * Kx;
    z_support = 0;
  } 
  else if( t >swingTime && t < period ) 
  {
    T = _2PI * (t - swingTime) / swingTime;
    Kx = (T-sin(T)) / _2PI;
    Kz = (1-cos(T)) / 2.f;
    x_swing = swingWidth -swingWidth * Kx;
    z_swing = 0;
    x_support = swingWidth * Kx;
    z_support = swingHeight * Kz;
  }
  else 
  {
    g->data = data;
    return 0;
  }

  //* 同步数据 *//
  data.swingCoord.x   = -x_swing    + g->x_base;
  data.swingCoord.z   = -z_swing    + g->z_base;
  data.supportCoord.x = -x_support  + g->x_base;
  data.supportCoord.z = -z_support  + g->z_base;

  g->data = data;
  
  g->rfCoord = data.swingCoord;
  g->rbCoord = data.supportCoord;
  g->lfCoord = data.supportCoord;
  g->lbCoord = data.swingCoord;

  //* 更新t *//
  g->t += g->t_delta;
  if(g->t >= g->peried) {
    g->t = 0;
    return 0;
  }
  return 1;
}

int gait_update(gait_t* g)
{
  int gflag = gait_equation(g);
  leg_set_coord(g->rfPtr, g->rfCoord.x, g->rfCoord.z);
  leg_set_coord(g->rbPtr, g->rbCoord.x, g->rbCoord.z);
  leg_set_coord(g->lfPtr, g->lfCoord.x, g->lfCoord.z);
  leg_set_coord(g->lbPtr, g->lbCoord.x, g->lbCoord.z);
  return gflag;
}