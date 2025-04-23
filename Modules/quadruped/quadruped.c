#include "quadruped_def.h"

void quad_init(
  Quadruped* quad, 
  const LegConfig* rf_cfg, 
  const LegConfig* rb_cfg, 
  const LegConfig* lf_cfg, 
  const LegConfig* lb_cfg,
  const GaitConfig* gait_cfg
  )
{
  leg_init(&quad->rf, rf_cfg);
  leg_init(&quad->rb, rb_cfg);
  leg_init(&quad->lf, lf_cfg);
  leg_init(&quad->lb, lb_cfg);
  gait_init(&quad->gait, gait_cfg);
}

void quad_gait_start(Quadruped* quad, uint16_t frames, int16_t times)
{
  gait_start(&quad->gait, frames, times+2);
}

bool quad_gait_update(Quadruped* quad)
{
  if( gait_update(&quad->gait) )
  {
    leg_set_coord(&quad->rf, quad->gait.current.orientation.rf);
    leg_set_coord(&quad->rb, quad->gait.current.orientation.rb);
    leg_set_coord(&quad->lf, quad->gait.current.orientation.lf);
    leg_set_coord(&quad->lb, quad->gait.current.orientation.lb);
    return true;
  }
  return false;
}