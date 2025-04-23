#include "quadruped_def.h"
#include "string.h"

#define TAG "Leg"

void leg_set_angle(Leg* leg, float a1, float a2)
{
  //! Hardware !//
  a1 = ALIMITT(a1);  // 0 >> 180
  a2 = ALIMITS(a2);
  //存入偏移和镜像之前的参数
  leg->angle1 = a1;  // 
  leg->angle2 = a2;
  //偏移
  a1 = AOFFSET(a1, leg->offset1);
  a2 = AOFFSET(a2, leg->offset2);
  //镜像
  if(leg->id >= LEG_ID_LEFT_START) {
    a1 = AMIRRORL(a1); 
    a2 = AMIRRORL(a2);
  }
  //设置
  pca9685_set_angle(leg->pcaChannel1, a1);
  pca9685_set_angle(leg->pcaChannel2, a2);
}

void leg_set_coord(Leg* leg, Coord coord)
{
  kinematics_inverse(leg, coord);
  leg_set_angle(leg, leg->IKINE.AS1, leg->IKINE.AS2);
}

void leg_init(Leg* leg, const LegConfig* cfg)
{ 
  //! Clear Memory !//
  memset(leg, 0, sizeof(Leg));

  //! Hardware !//
  leg->id          = cfg->id;
  leg->pcaChannel1 = cfg->pcaChannel1;
  leg->pcaChannel2 = cfg->pcaChannel2;
  leg->offset1     = cfg->offset1;
  leg->offset2     = cfg->offset2;
  leg->angle1      = cfg->angle1;
  leg->angle2      = cfg->angle2;
  leg_set_angle(leg, cfg->angle1, cfg->angle2);

  //! Easing Angle !//
  easing_angle_init(&leg->ea1, &cfg->ea1_config);
  easing_angle_init(&leg->ea2, &cfg->ea2_config);
  
  //! Easing Coord !//
  easing_coord_init(&leg->ec, &cfg->ec_config);
}

void leg_eangle_absolute(Leg* leg, float start1, float stop1, float start2, float stop2, uint16_t frames)
{
  if( start1 != stop1)
  {
    easing_angle_absolute(&leg->ea1, start1, stop1, frames);
  }
  if( start2 != stop2)
  {
    easing_angle_absolute(&leg->ea2, start2, stop2, frames);
  }
}

void leg_eangle_relative(Leg* leg, float distance1, float distance2, uint16_t frames)
{
  if( distance1 != 0.f)
  {
    easing_angle_relative(&leg->ea1, distance1, frames);
  }
  if( distance2 != 0.f)
  {
    easing_angle_relative(&leg->ea2, distance2, frames);
  }
}

void leg_eangle_target(Leg* leg, float target1, float target2, uint16_t frames)
{
  if( target1 != leg->ea1.stop)
  {
    easing_angle_target(&leg->ea1, target1, frames);
  }
  if( target2 != leg->ea2.stop)
  {
    easing_angle_target(&leg->ea2, target2, frames);
  }
}

bool leg_eangle_update(Leg* leg)
{
  bool flag = false;
  flag |= easing_angle_update(&leg->ea1);
  flag |= easing_angle_update(&leg->ea2);
  leg_set_angle(leg, leg->ea1.current, leg->ea2.current);
  return flag;
}

void leg_eangle_update_block(Leg* leg)
{
  while(leg_eangle_update(leg));
}

void leg_ecoord_absolute(Leg* leg, Coord start, Coord stop, uint16_t frames)
{
  if( start.X != stop.X || start.Z != stop.Z)
  {
    easing_coord_absolute(&leg->ec, start, stop ,frames);
  }
}

void leg_ecoord_relative(Leg* leg, Coord distance, uint16_t frames)
{
  if( distance.X != 0.f || distance.Z != 0.f)
  {
    easing_coord_relative(&leg->ec, distance , frames);
  }
}

void leg_ecoord_target(Leg* leg, Coord target, uint16_t frames)
{
  if( target.X != leg->ec.stop.X || target.Z != leg->ec.stop.Z)
  {
    easing_coord_target(&leg->ec, target, frames);
  }
}

bool leg_ecoord_update(Leg* leg)
{
  bool flag = false;
  flag |= easing_coord_update(&leg->ec);
  leg_set_coord(leg, leg->ec.current);
  return flag;
}

void leg_ecoord_update_block(Leg* leg)
{
  while(leg_ecoord_update(leg));
}