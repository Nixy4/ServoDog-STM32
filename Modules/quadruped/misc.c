#include "quadruped.h"

#if CONFIG_CONST_TYPE == 0

inline quad_fp radians(quad_fp degree) 
{ 
  return degree * CONFIG_DEGREE_TO_RADIAN; 
}

inline quad_fp degrees(quad_fp radian) 
{
  return radian * CONFIG_RADIAN_TO_DEGREE;
}

inline quad_coord coord_mapping(quad_coord c, quad_coord base) 
{ 
  c.X = -c.X + base.X;
  c.Z = -c.Z + base.Z;
  return c;
}

#endif