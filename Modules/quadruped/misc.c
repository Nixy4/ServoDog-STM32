#include "quadruped.h"

#if CONFIG_CACULATE_TYPE == 0

inline quad_fp radians(quad_fp degree) { return degree * CONFIG_DEGREE_TO_RADIAN; }
inline quad_fp degrees(quad_fp radian) { return radian * CONFIG_RADIAN_TO_DEGREE; }

#endif