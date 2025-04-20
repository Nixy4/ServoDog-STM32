#pragma once

#include "servo.h"
#include "kinematics.h"
#include "leg.h"

typedef struct
{
  
  leg_t right_front;
  leg_t right_back;
  leg_t left_front;
  leg_t left_back;

  
} quadruped_t;