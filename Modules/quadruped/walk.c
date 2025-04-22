#include "walk.h"

extern Leg legs[4];

void walk_init(WalkData* walk_data,uint32_t period, float swingWidth, float swingHeight)
{
  walk_data->period = period;
  walk_data->swingWidth = swingWidth;
  walk_data->swingHeight = swingHeight;
  walk_data->swingTick = period / 2;
  walk_data->x_base = 0;
  walk_data->z_base = 0;
  walk_data->x = 0;
  walk_data->z = 0;
}

void walk_one_step(WalkData* walk_data)
{
  float T, Kx, Kz, x1 , z1, x2, z2;
  float period = walk_data->period;
  float swingTick = walk_data->swingTick;
  float swingWidth = walk_data->swingWidth;
  float swingHeight = walk_data->swingHeight;  
  //* 计算 *//
  for(uint32_t t=0; t<walk_data->period; t++)
  {
    if( t <= swingTick ) 
    {
      T  = HPI * t / swingTick;
      Kx = (T-sin(T)) / HPI;
      Kz = (1-cos(T)) / 2.f;

      x1 = swingWidth * Kx;
      z1 = swingHeight * Kz;

      x2 = swingWidth - swingWidth * Kx;
      z2 = 0;

    }
    else
    {
      T  = HPI * (t - swingTick) / swingTick;
      Kx = (T-sin(T)) / HPI;
      Kz = (1-cos(T)) / 2.f;

      x1 = swingWidth - swingWidth * Kx;
      z1 = 0;    

      x2 = swingWidth * Kx;
      z2 = swingHeight * Kz;  
    }

    leg_set_coord(&legs[LEG_ID_RF], x1, z1);
    leg_set_coord(&legs[LEG_ID_RB], x2, z2);
    leg_set_coord(&legs[LEG_ID_LF], x2, z2);
    leg_set_coord(&legs[LEG_ID_LB], x1, z1);
    HAL_Delay(1);//延时一毫秒
  }
}

void walk_n_step(WalkData* walk_data, int n)
{
  for(int i=0; i<n; i++)
  {
    walk_one_step(walk_data);
  }
}