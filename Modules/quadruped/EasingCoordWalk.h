#pragma once
#include "quadruped.h"

typedef struct
{
  float swingDuty;
} EasingCoordWalkCustomData;

Coord _easing_coord_walk_first(EasingCoord* ec);
Coord _easing_coord_walk_second(EasingCoord* ec);

extern EasingCoordWalkCustomData gEasingCoordWalkCustomData;

// extern EasingCoordConfig walk_first;
// extern EasingCoordConfig walk_second;

#define EC_CONFIG_WALK_FIRST() {EASING_MODE_DEFAULT|EASING_MODE_NTIMES(10),_easing_coord_walk_first,500,0,&gEasingCoordWalkCustomData}
#define EC_CONFIG_WALK_SECOND() {EASING_MODE_DEFAULT|EASING_MODE_NTIMES(10),_easing_coord_walk_second,500,0,&gEasingCoordWalkCustomData}
