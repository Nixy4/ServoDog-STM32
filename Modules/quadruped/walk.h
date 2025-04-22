#pragma once

#include "leg.h"
#include "stm32f4xx_hal.h"

typedef struct
{
  uint32_t period;
  uint32_t tick;
  uint32_t swingDuty;
  uint32_t swingTick;
  float swingWidth;
  float swingHeight;
  float x_base;
  float z_base;
  float x;
  float z;
  
} WalkData;

void walk_init(WalkData* walk_data,uint32_t period, float swingWidth, float swingHeight);
void walk_one_step(WalkData* walk_data);
void walk_n_step(WalkData* walk_data, int n);