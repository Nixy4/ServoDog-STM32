#include "quadruped.h"

#define TAG "Fixed"

void fixed_set_angle_zero()
{
  led_set_angle(&leg_rf, 0, 0, false);
  led_set_angle(&leg_lf, 0, 0, false);
  led_set_angle(&leg_rb, 0, 0, false);
  led_set_angle(&leg_lb, 0, 0, false);
  
  leg_rf.kine = ksp_start;
  leg_lf.kine = ksp_start;
  leg_rb.kine = ksp_start;
  leg_lb.kine = ksp_start;
}

void fixed_set_angle_mid()
{
  led_set_angle(&leg_rf, 90, 90, false);
  led_set_angle(&leg_lf, 90, 90, false);
  led_set_angle(&leg_rb, 90, 90, false);
  led_set_angle(&leg_lb, 90, 90, false);
}

void fixed_stand_by_coord0()
{
  leg_set_coord(&leg_rf, 0, CONFIG_X0_Z_MAX);
  leg_set_coord(&leg_lf, 0, CONFIG_X0_Z_MAX);
  leg_set_coord(&leg_rb, 0, CONFIG_X0_Z_MAX);
  leg_set_coord(&leg_lb, 0, CONFIG_X0_Z_MAX);
}

void fixed_stand_by_coord1()
{
  leg_set_coord(&leg_rf, ksp_x0_z_max.X, ksp_x0_z_max.Z);
  leg_set_coord(&leg_lf, ksp_x0_z_max.X, ksp_x0_z_max.Z);
  leg_set_coord(&leg_rb, ksp_x0_z_max.X, ksp_x0_z_max.Z);
  leg_set_coord(&leg_lb, ksp_x0_z_max.X, ksp_x0_z_max.Z);
}

void fixed_stand_by_acb(uint32_t frame_count)
{
  quad_kine kine = {0};
  kine_inverse(&kine,0,CONFIG_X0_Z_MAX);
  elog_i(TAG,"kien >> AS1:%.2f AS2:%.2f",kine.AS1,kine.AS2);

  leg_acb_target(&leg_rf, &leg_rf.tacb, kine.AS1, frame_count);
  leg_acb_target(&leg_rf, &leg_rf.sacb, kine.AS2, frame_count);

  leg_acb_target(&leg_lf, &leg_lf.tacb, kine.AS1, frame_count);
  leg_acb_target(&leg_lf ,&leg_lf.sacb, kine.AS2, frame_count);

  leg_acb_target(&leg_rb, &leg_rb.tacb, kine.AS1, frame_count);
  leg_acb_target(&leg_rb, &leg_rb.sacb, kine.AS2, frame_count);

  leg_acb_target(&leg_lb, &leg_lb.tacb, kine.AS1, frame_count);
  leg_acb_target(&leg_lb, &leg_lb.sacb, kine.AS2, frame_count);

  leg_acb_update_all_block();

  leg_rf.kine = kine;
  leg_lf.kine = kine;
  leg_rb.kine = kine;
  leg_lb.kine = kine;
}

void fixed_stand_by_ccb(uint32_t frame_count)
{
  quad_coord c = {0,CONFIG_X0_Z_MAX};
  leg_ccb_target(&leg_rf, &leg_rf.ccb, c, frame_count);
  leg_ccb_target(&leg_lf, &leg_lf.ccb, c, frame_count);
  leg_ccb_target(&leg_rb, &leg_rb.ccb, c, frame_count);
  leg_ccb_target(&leg_lb, &leg_lb.ccb, c, frame_count);
  leg_ccb_update_all_block();
}