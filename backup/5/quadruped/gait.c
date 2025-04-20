// #include "gait_def.h"
// #define TAG "GAIT"

int gait_equation(gait_t* g)
{
  gait_data_t data = {0};

  double T, Kx, Kz, x_swing,z_swing, x_support,z_support;
  double t = g->t;
  double period = g->periedTick;
  double swingTime = g->swingTime;
  double swingWidth = g->swingWidth;
  double swingHeight = g->swingHeight;
  
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

    // elog_d(TAG," SWING t:%f, st:%f, T=%f, Kx:%f, Kz:%f, x_swing:%f, x_support:%f, z:%f", t, swingTime, T, Kx, Kz, x_swing, x_support, z);
  } 
  else if( t >swingTime && t < period ) 
  {
    T = _2PI * (t - swingTime) / swingTime;

    Kx = (T-sin(T)) / _2PI;
    Kz = (1-cos(T)) / 2.f;

    x_swing = swingWidth - swingWidth * Kx;
    z_swing = 0;

    x_support = swingWidth * Kx;
    z_support = swingHeight * Kz;
    // elog_d(TAG," SUPPORT t:%f, st:%f, T=%f, Kx:%f, Kz:%f, x_swing:%f, x_support:%f, z:%f", t, swingTime, T, Kx, Kz, x_swing, x_support, z);
  }
  else 
  {
    g->data = data;
    return 0;
  }

  //* 同步数据 *//
  data.swingCoord.x   = -x_swing + g->xBase;
  data.swingCoord.z   = -z_swing + g->zBase;
  data.supportCoord.x = -x_support + g->xBase;
  data.supportCoord.z = -z_support + g->zBase;

  // g->data = data;
  
  g->rfCoord = data.swingCoord;
  g->rbCoord = data.supportCoord;
  g->lfCoord = data.supportCoord;
  g->lbCoord = data.swingCoord;

  //* 更新t *//
  g->t += g->deltaTick;
  if(g->t >= g->periedTick) {
    g->t = 0;
    return 0;
  }
  return 1;
}

// int gait_update(gait_t* g)
// {
//   int gflag = gait_equation(g);
//   leg_set_coord(g->rfPtr, g->rfCoord.x, g->rfCoord.z);
//   leg_set_coord(g->rbPtr, g->rbCoord.x, g->rbCoord.z);
//   leg_set_coord(g->lfPtr, g->lfCoord.x, g->lfCoord.z);
//   leg_set_coord(g->lbPtr, g->lbCoord.x, g->lbCoord.z);
//   return gflag;
// }