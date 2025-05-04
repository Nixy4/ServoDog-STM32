#include "walk.h"

//TODO:Walk参数

walk_t walk_gait = 
{
  .peried      = WALK_PERIOD,
  .swingDuty   = WALK_SWING_DUTY,
  .swingTime   = WALK_SWING_TIME,
  .swingWidth  = WALK_SWING_WIDTH,
  .swingHeight = WALK_SWING_HEIGHT,
  .x_base      = WALK_X_BASE,
  .z_base      = WALK_Z_BASE,
  .t_delta     = 1,
  .rfPtr       = &rf,
  .rbPtr       = &rb,
  .lfPtr       = &lf,
  .lbPtr       = &lb,
  .t           = 0,
};

void walk_config(quad_float period, quad_float swing_duty, quad_float swing_width, quad_float swing_height, quad_float z_base)
{
  walk_gait.peried      = period;
  walk_gait.swingDuty   = swing_duty;
  walk_gait.swingTime   = period * swing_duty;
  walk_gait.swingWidth  = swing_width;
  walk_gait.swingHeight = swing_height;
  walk_gait.x_base      = swing_width / 2.f;
  walk_gait.z_base      = z_base;
}

void walk0(quad_float ms)
{
  leg_move_target(&rf, walk_gait.x_base, walk_gait.z_base, ms);
  leg_move_target(&rb, walk_gait.x_base, walk_gait.z_base, ms);
  leg_move_target(&lf, walk_gait.x_base, walk_gait.z_base, ms);
  leg_move_target(&lb, walk_gait.x_base, walk_gait.z_base, ms);
  quad_update_block();
}

int walk1_equation(walk_t* g)
{
  walk_coord_t data = {0};

  quad_float T, Kx, Kz, x_swing,z_swing, x_support,z_support;
  quad_float t = g->t;
  // quad_float period = g->peried;
  quad_float swingTime = g->swingTime;
  quad_float swingWidth = g->swingWidth;
  quad_float swingHeight = g->swingHeight;
  
  //* 计算 *//
  if( t <= swingTime ) 
  {
    T = DOUBLE_PI * t / swingTime;
    Kx = (T-sin(T)) / DOUBLE_PI;
    Kz = (1-cos(T)) / 2.f;
    x_swing = swingWidth * Kx;
    z_swing = swingHeight * Kz;
    x_support = swingWidth -swingWidth * Kx;
    z_support = 0;
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
  
  g->rfCoord = (coord_t){g->x_base, g->z_base};
  g->lfCoord = data.swingCoord;
  g->rbCoord = data.swingCoord;
  g->lbCoord = (coord_t){g->x_base, g->z_base};

  //* 更新t *//
  g->t += g->t_delta;
  if(g->t >= g->swingTime) {
    g->t = 0;
    return 0;
  }
  return 1;
}

int walk1_update(walk_t* g)
{
  int gflag = walk1_equation(g);
  leg_set_coord(g->rfPtr, g->rfCoord.x, g->rfCoord.z);
  leg_set_coord(g->rbPtr, g->rbCoord.x, g->rbCoord.z);
  leg_set_coord(g->lfPtr, g->lfCoord.x, g->lfCoord.z);
  leg_set_coord(g->lbPtr, g->lbCoord.x, g->lbCoord.z);
  return gflag;
}

int walk2_equation(walk_t* g)
{
  walk_coord_t data = {0};

  quad_float T, Kx, Kz, x_swing,z_swing, x_support,z_support;
  quad_float t = g->t;
  quad_float period = g->peried;
  quad_float swingTime = g->swingTime;
  quad_float swingWidth = g->swingWidth;
  quad_float swingHeight = g->swingHeight;
  
  //* 计算 *//
  if( t <= swingTime ) 
  {
    T = DOUBLE_PI * t / swingTime;
    Kx = (T-sin(T)) / DOUBLE_PI;
    Kz = (1-cos(T)) / 2.f;
    x_swing = swingWidth * Kx;
    z_swing = swingHeight * Kz;
    x_support = swingWidth -swingWidth * Kx;
    z_support = 0;
  } 
  else if( t >swingTime && t < period ) 
  {
    T = DOUBLE_PI * (t - swingTime) / swingTime;
    Kx = (T-sin(T)) / DOUBLE_PI;
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

int walk2_update(walk_t* g)
{
  int gflag = walk2_equation(g);
  leg_set_coord(g->rfPtr, g->rfCoord.x, g->rfCoord.z);
  leg_set_coord(g->rbPtr, g->rbCoord.x, g->rbCoord.z);
  leg_set_coord(g->lfPtr, g->lfCoord.x, g->lfCoord.z);
  leg_set_coord(g->lbPtr, g->lbCoord.x, g->lbCoord.z);
  return gflag;
}

void walk1()
{
  while(walk1_update(&walk_gait) != 0);
}

void walk2(int steps)
{
  for(int i = 0; i < steps; i++)
  {
    while(walk2_update(&walk_gait) != 0);
  }
}

void walk(int steps)
{
  walk0(walk_gait.peried*2);
  walk1();
  walk2(steps);
  walk0(walk_gait.peried*2);
}