#include "quadruped_def.h"
#include "string.h"

#define TAG "Leg"

static Leg legs[4];

inline Leg* leg_ptr(LegID id)
{
  return &legs[id];
}

int leg_init(LegID id, const LegConfig* cfg)
{ 
  Leg* leg = leg_ptr(id);

  //! Clear Memory !//
  memset(leg, 0, sizeof(Leg));

  //! Hardware !//
  leg->pcaChannel1 = cfg->pcaChannel1;
  leg->pcaChannel2 = cfg->pcaChannel2;
  leg->start1 = cfg->start1;
  leg->start2 = cfg->start2;
  leg->angle1 = cfg->angle1;
  leg->angle2 = cfg->angle2;
  leg_set_angle(id, cfg->angle1, cfg->angle2);

  //! Easing Angle !//
  leg_ea_init(&leg->ea1, &cfg->ea1_config);
  leg_ea_init(&leg->ea2, &cfg->ea2_config);
  
  //! Easing Coord !//
  _leg_ec_init(&leg->ec, &cfg->ec_config);
  
  elog_i(TAG, "Leg %d Init Success", id);
  return 0;
}

void leg_set_angle(LegID id, float a1, float a2)
{
  Leg* leg = leg_ptr(id);
  //! Hardware !//
  a1 = ALIMITT(a1);
  a2 = ALIMITT(a2);
  leg->angle1 = a1;
  leg->angle2 = a2;
  if(id < LEG_ID_LEFT_START) {
    a1 = AMIRRORR(a1);
    a2 = AMIRRORR(a2);
  } else {
    a1 = AMIRRORL(a1);
    a2 = AMIRRORL(a2);
  }
  a1 = AOFFSET(a1, leg->start1);
  a2 = AOFFSET(a2, leg->start2);
  pca9685_set_angle(leg->pcaChannel1, a1);
  pca9685_set_angle(leg->pcaChannel2, a2);
}

void leg_set_coord(LegID id, float x, float z)
{
  Leg* leg = leg_ptr(id);
  leg_inverse_kinematics(leg, x, z);
  leg_set_angle(id, leg->IKINE.AS1, leg->IKINE.AS2);
}