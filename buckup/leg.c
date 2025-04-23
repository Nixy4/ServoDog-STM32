#include "quadruped_def.h"
#include "string.h"

#define TAG "Leg"

static Leg legs[4];

extern void _leg_eangle_init(EasingAngle* ea, const EasingAngleConfig* cfg);
extern void _leg_ecoord_init(EasingCoord* ec, const EasingCoordConfig* cfg);

inline Leg* leg_ptr(Leg* leg)
{
  return &legs[id];
}

int leg_init(Leg* leg, const LegConfig* cfg)
{ 

  //! Clear Memory !//
  memset(leg, 0, sizeof(Leg));

  //! Hardware !//
  leg->pcaChannel1 = cfg->pcaChannel1;
  leg->pcaChannel2 = cfg->pcaChannel2;
  leg->offset1 = cfg->offset1;
  leg->offset2 = cfg->offset2;
  leg->angle1 = cfg->angle1;
  leg->angle2 = cfg->angle2;
  leg_set_angle(leg, cfg->angle1, cfg->angle2);

  //! Easing Angle !//
  _leg_eangle_init(&leg->ea1, &cfg->ea1_config);
  _leg_eangle_init(&leg->ea2, &cfg->ea2_config);
  
  //! Easing Coord !//
  _leg_ecoord_init(&leg->ec, &cfg->ec_config);
  
  elog_i(TAG, "Leg %d Init Success", id);
  return 0;
}

void _leg_set_angle(Leg* leg, float a1, float a2)
{
  pca9685_set_angle(leg->pcaChannel1, a1);
  pca9685_set_angle(leg->pcaChannel2, a2);
}

void _leg_set_coord(Leg* leg, float x, float z)
{
  kinematics_inverse(leg, x, z);
  _leg_set_angle(leg, leg->IKINE.AS1, leg->IKINE.AS2);
}

void leg_set_angle(Leg* leg, float a1, float a2)
{
  
  a1 = ALIMITT(a1);
  a2 = ALIMITT(a2); 
  leg->angle1 = a1;
  leg->angle2 = a2;
  a1 = AOFFSET(a1, leg->offset1);
  a2 = AOFFSET(a2, leg->offset2);
  if(id < LEG_ID_LEFT_START) {
    a1 = AMIRRORR(a1);
    a2 = AMIRRORR(a2);
  } else {
    a1 = AMIRRORL(a1);
    a2 = AMIRRORL(a2);
  }
  _leg_set_angle(leg, a1, a2);
}

void leg_set_coord(Leg* leg, float x, float z)
{
  
  _leg_set_coord(leg, x, z);
}