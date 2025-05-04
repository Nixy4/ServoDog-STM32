#pragma once
#include "leg.h"

void quad_init();
void quad_reset();
int quad_update();
void quad_update_block();
void quad_stand0(quad_float ms);
void quad_fall0(quad_float ms);
void quad_fall1(quad_float ms);
void quad_stand1(quad_float ms);

extern leg_t rf;
extern leg_t rb;
extern leg_t lf;
extern leg_t lb;