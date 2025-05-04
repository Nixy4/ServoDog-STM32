
#include "quad_config.h"

#if QUADRUPED_NUMBER == 0
//!大腿组
const QUAD_TYPE L1 = 79.0f;
// const QUAD_TYPE _L1 = 72.0f;
// const QUAD_TYPE L1_ = 7.0f;
const QUAD_TYPE L2 = 68.5f;
const QUAD_TYPE L3 = 20.0f;
// const QUAD_TYPE L4 = 32.0f;
const QUAD_TYPE L5 = 78.7908624143688314f; // sqrt(_L1 * _L1 + L4 * L4);
// const QUAD_TYPE R14 = PI / 2;
const QUAD_TYPE R15 = 0.4182243295792291f; // atan(L4 / _L1);
//!小腿组
const QUAD_TYPE L8 = 12.0f;
const QUAD_TYPE L9 = 77.0f;
#define THIGH_GLOBAL_OFFSET -5 

const leg_init_t rf_cfg = {
  .type = LEG_TYPE_RF,
  .thighServoId = 0,
  .shankServoId = 1,
  .thighOffset = 5+THIGH_GLOBAL_OFFSET,
  .shankOffset = 0,
  .thighAngle = 0,
  .shankAngle = 0,
};

const leg_init_t lf_cfg = {
  .type = LEG_TYPE_LF,
  .thighServoId = 2,
  .shankServoId = 3,
  .thighOffset = 5+THIGH_GLOBAL_OFFSET,
  .shankOffset = 10,
  .thighAngle = 0,
  .shankAngle = 0,
};

const leg_init_t rb_cfg = {
  .type = LEG_TYPE_RB,
  .thighServoId = 4,
  .shankServoId = 5,
  .thighOffset = 0+THIGH_GLOBAL_OFFSET,
  .shankOffset = -3,
  .thighAngle = 0,
  .shankAngle = 0,
};

const leg_init_t lb_cfg = {
  .type = LEG_TYPE_LB,
  .thighServoId = 6,
  .shankServoId = 7,
  .thighOffset = 0+THIGH_GLOBAL_OFFSET,
  .shankOffset = 1,
  .thighAngle = 0,
  .shankAngle = 0,
};

#elif QUADRUPED_NUMBER == 1

//!大腿组
const QUAD_TYPE L1 = 79.0f;
// const QUAD_TYPE _L1 = 72.0f;
// const QUAD_TYPE L1_ = 7.0f;
const QUAD_TYPE L2 = 68.5f;
const QUAD_TYPE L3 = 20.0f;
// const QUAD_TYPE L4 = 32.0f;
const QUAD_TYPE L5 = 78.7908624143688314f; // sqrt(_L1 * _L1 + L4 * L4);
const QUAD_TYPE R15 = 0.4182243295792291f; // atan(L4 / _L1);
// const QUAD_TYPE R14 = PI / 2;
//!小腿组
const QUAD_TYPE L8 = 14.5000000000000000f;
const QUAD_TYPE L9 = 77.0000000000000000f;

#define THIGH_GLOBAL_OFFSET -2

const leg_init_t rf_cfg = {
  .type = LEG_TYPE_RF,
  .thighServoId = 1,
  .shankServoId = 0,
  .thighOffset = 2+THIGH_GLOBAL_OFFSET,
  .shankOffset = 8,
  .thighAngle = 0,
  .shankAngle = 0,
};

const leg_init_t lf_cfg = {
  .type = LEG_TYPE_LF,
  .thighServoId = 3,
  .shankServoId = 2,
  .thighOffset = 5+THIGH_GLOBAL_OFFSET,
  .shankOffset = 8,
  .thighAngle = 0,
  .shankAngle = 0,
};

const leg_init_t rb_cfg = {
  .type = LEG_TYPE_RB,
  .thighServoId = 5,
  .shankServoId = 4,
  .thighOffset = 5+THIGH_GLOBAL_OFFSET,
  .shankOffset = 7,
  .thighAngle = 0,
  .shankAngle = 0,
};

const leg_init_t lb_cfg = {
  .type = LEG_TYPE_LB,
  .thighServoId = 7,
  .shankServoId = 6,
  .thighOffset = 5+THIGH_GLOBAL_OFFSET,
  .shankOffset = 4,
  .thighAngle = 0,
  .shankAngle = 0,
};

#endif