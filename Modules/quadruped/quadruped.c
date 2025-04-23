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

bool quad_gait_update(Quadruped* quad)
{
  bool flag = false;
  flag |= gait_update(&quad->gait);
  kinematics_inverse(&quad->rf, quad->gait.current.orientation.rf);
  kinematics_inverse(&quad->rb, quad->gait.current.orientation.rb);
  quad->lf.IKINE = quad->rb.IKINE;
  quad->lb.IKINE = quad->rf.IKINE;
  leg_set_angle(&quad->rf, quad->rf.IKINE.AS1, quad->rf.IKINE.AS2);
  leg_set_angle(&quad->rb, quad->rb.IKINE.AS1, quad->rb.IKINE.AS2);
  leg_set_angle(&quad->lf, quad->lf.IKINE.AS1, quad->lf.IKINE.AS2);
  leg_set_angle(&quad->lb, quad->lb.IKINE.AS1, quad->lb.IKINE.AS2);
  return flag;
}