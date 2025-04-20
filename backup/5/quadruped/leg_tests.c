#include "leg.h"

void leg_special_point_test(leg_t* leg)
{
  leg_set_coord(leg,x_min.X,x_min.Z);
  HAL_Delay(1000);
  leg_set_coord(leg,x_max.X,x_max.Z);
  HAL_Delay(1000);
  leg_set_coord(leg,z_min.X,z_min.Z);
  HAL_Delay(1000);
  leg_set_coord(leg,z_max.X,z_max.Z);
  HAL_Delay(1000);
  leg_set_coord(leg,start.X,start.Z);
  HAL_Delay(1000);
  leg_set_coord(leg,end.X,end.Z);
  HAL_Delay(1000);
}

void legs_special_point_test(leg_t* leg[4])
{
  leg_special_point_test(leg[0]);
  leg_special_point_test(leg[1]);
  leg_special_point_test(leg[2]);
  leg_special_point_test(leg[3]);
}