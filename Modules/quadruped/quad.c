#include "quad.h"

Leg legs[4];

void quad_init(void)
{
  pca9685_set_freq(50);
  leg_init( &legs[LEG_ID_RF], LEG_ID_RF, 0, 5, 0, 0);
  leg_init( &legs[LEG_ID_RB], LEG_ID_RB, 1, 4, 0, 0);
  leg_init( &legs[LEG_ID_LF], LEG_ID_LF, 3, 7, 0, 0);
  leg_init( &legs[LEG_ID_LB], LEG_ID_LB, 2, 6, 0, 0);
}

void quad_stand()
{
  leg_set_angle(&legs[LEG_ID_RF], x0_z_max.AS1, x0_z_max.AS2);
  leg_set_angle(&legs[LEG_ID_RB], x0_z_max.AS1, x0_z_max.AS2);
  leg_set_angle(&legs[LEG_ID_LF], x0_z_max.AS1, x0_z_max.AS2);
  leg_set_angle(&legs[LEG_ID_LB], x0_z_max.AS1, x0_z_max.AS2);
}

void quad_fall0()
{
  leg_set_angle(&legs[LEG_ID_RF], 0, 0);
  leg_set_angle(&legs[LEG_ID_RB], 0, 0);
  leg_set_angle(&legs[LEG_ID_LF], 0, 0);
  leg_set_angle(&legs[LEG_ID_LB], 0, 0);
}

void quad_fall1()
{
  leg_set_angle(&legs[LEG_ID_RF], 45, 0);
  leg_set_angle(&legs[LEG_ID_RB], 45, 0);
  leg_set_angle(&legs[LEG_ID_LF], 45, 0);
  leg_set_angle(&legs[LEG_ID_LB], 45, 0);
}