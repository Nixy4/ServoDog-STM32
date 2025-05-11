#pragma once

//! 设备参数配置
#include "quad_config.h"
//! HAL
#include "stm32f4xx_hal.h"
//! BSP
#include "key.h"
//! Middleware
#include "elog.h"
//! C STD
#include "string.h"
#include "math.h"

#if CONFIG_FLOAT_TYPE == 0
  typedef float quad_fp;
#else
  typedef double quad_fp;
#endif

typedef enum leg_index 
{
  LEG_START = 0U,
  LEG_RF = LEG_START,
  LEG_LF,
  LEG_RB,
  LEG_LB,
  LEG_END = LEG_LB,
  LEG_COUNT,
} leg_index;

typedef enum servo_index
{
  SERVO_START = 0U,
  SERVO_RF_T = SERVO_START,
  SERVO_RF_S,
  SERVO_LF_T,
  SERVO_LF_S,
  SERVO_RB_T,
  SERVO_RB_S,
  SERVO_LB_T,
  SERVO_LB_S,
  SERVO_END = SERVO_LB_S,
  SERVO_COUNT,
} servo_index;

typedef enum fcb_mode
{
  FCB_MODE_NONE = 0U,
  FCB_MODE_TICK,
  FCB_MODE_MS,
  FCB_MODE_US,
} fcb_mode;

typedef struct quad_angle
{
  quad_fp thigh;
  quad_fp shank;
} quad_angle;

typedef struct quad_coord
{
  quad_fp X;
  quad_fp Z;
} quad_coord;

typedef volatile struct quad_kine
{
  quad_fp AS1,AS2;
  quad_fp RS1,RS2;
  quad_fp L6,L7;
  quad_fp R12,R13;
  quad_fp R17,R35;
  quad_fp R7X;
  quad_fp X,Z;
} quad_kine;

typedef struct quad_fcb_ctrlblock
{
  fcb_mode mode;
  uint32_t interval;

  uint32_t count;
  volatile uint32_t index;
  volatile uint32_t last_tick;
} quad_fcb;

typedef struct quad_angle_ctrlblock
{
  quad_fp (*calc)(quad_fp i);
  quad_fp start;
  quad_fp end;
  quad_fp delta;
  quad_fp current;
} quad_acb;

typedef struct quad_coord_ctrlblock
{
  quad_fp (*calc_x)(quad_fp i);
  quad_fp (*calc_z)(quad_fp i);
  quad_fp start_x;
  quad_fp start_z;
  quad_fp end_x;
  quad_fp end_z;
  quad_fp delta_x;
  quad_fp delta_z;
  quad_fp current_x;
  quad_fp current_z;
} quad_ccb;

#if CONFIG_CONST_TYPE == 0
  quad_fp radians(quad_fp degree);
  quad_fp degrees(quad_fp radian);
  quad_coord coord_mapping(quad_coord c, quad_coord base);
#else
  #define radians(degree) ((degree) * CONFIG_DEGREE_TO_RADIAN)
  #define degrees(radian) ((radian) * CONFIG_RADIAN_TO_DEGREE)
  #define coord_mapping(c, base) ((quad_coord){-(c).X+(base).X,-(c).Z+(base).Z,})
#endif

void delay_ns(uint32_t ns);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);

quad_fp _easing_calc_Linear(const quad_fp t);       // linear t

quad_fp _easing_calc_InQuad(const quad_fp t);       // quadratic t^2
quad_fp _easing_calc_OutQuad(const quad_fp t);
quad_fp _easing_calc_InOutQuad(const quad_fp t);

quad_fp _easing_calc_InCubic(const quad_fp t);      // cubic t^3
quad_fp _easing_calc_OutCubic(const quad_fp t);
quad_fp _easing_calc_InOutCubic(const quad_fp t);

quad_fp _easing_calc_InQuart(const quad_fp t);      // quartic t^4
quad_fp _easing_calc_OutQuart(const quad_fp t);
quad_fp _easing_calc_InOutQuart(const quad_fp t);

quad_fp _easing_calc_InQuint(const quad_fp t);      // quintic t^5
quad_fp _easing_calc_OutQuint(const quad_fp t);
quad_fp _easing_calc_InOutQuint(const quad_fp t);

quad_fp _easing_calc_InSine(const quad_fp t);       // sinusoidal 正弦 sin(t)
quad_fp _easing_calc_OutSine(const quad_fp t);
quad_fp _easing_calc_InOutSine(const quad_fp t);

quad_fp _easing_calc_InExpo(const quad_fp t);       // exponential 指数 2^t
quad_fp _easing_calc_OutExpo(const quad_fp t);
quad_fp _easing_calc_InOutExpo(const quad_fp t);

quad_fp _easing_calc_InCirc(const quad_fp t);       // circular 圆形
quad_fp _easing_calc_OutCirc(const quad_fp t);
quad_fp _easing_calc_InOutCirc(const quad_fp t);

quad_fp _easing_calc_InBack(const quad_fp t);       // elastic 衰减三次幂 (s+1)t^3 - st^2
quad_fp _easing_calc_OutBack(const quad_fp t);
quad_fp _easing_calc_InOutBack(const quad_fp t);

quad_fp _easing_calc_InElastic(const quad_fp t);    // elastic 衰减正弦
quad_fp _easing_calc_OutElastic(const quad_fp t);
quad_fp _easing_calc_InOutElastic(const quad_fp t);

quad_fp _easing_calc_InBounce(const quad_fp t);     // back 衰减反弹
quad_fp _easing_calc_OutBounce(const quad_fp t);
quad_fp _easing_calc_InOutBounce(const quad_fp t);

//! Frame Control Block
void fcb_init(quad_fcb* fcb, fcb_mode mode, uint32_t interval);
void fcb_start(quad_fcb* frame, uint32_t count);
void fcb_next(quad_fcb* fcb);
bool fcb_skip(quad_fcb* fcb);
bool fcb_complete(quad_fcb* frame);
bool fcb_median(quad_fcb* fcb);
bool fcb_last(quad_fcb* frame);
quad_fp fcb_percentage(quad_fcb* fcb);
uint32_t fcb_current(quad_fcb* fcb);
uint32_t fcb_count(quad_fcb* fcb);

//! Kinematics
void kine_forward(quad_kine* kine, quad_fp AS1, quad_fp AS2);
void kine_inverse(quad_kine* kine, quad_fp X, quad_fp Z);
void kine_elog_d(quad_kine* kine);
bool kine_compare(const quad_kine* kine1, const quad_kine* kine2);
void kine_sptest(void);
void kine_fptest(void);

//! Servo Control
//* Misc
quad_fp servo_angle_mirror(quad_fp angle);
quad_fp servo_angle_offset(quad_fp angle, quad_fp offset);
quad_fp servo_angle_limit_thigh(quad_fp angle);
quad_fp servo_angle_limit_shank(quad_fp angle);
//* Set
void servo_set_freq(quad_fp freq);
void servo_set_angle(servo_index channel, quad_fp angle);
void servo_set_angle_sync(quad_fp angles[SERVO_COUNT]);

//! Leg Control
//* Misc
quad_coord leg_coord_offset(leg_index leg, quad_coord coord);

//* Set
void leg_set_angle(leg_index leg, quad_fp angle_thigh, quad_fp angle_shank, bool kine_update);
void leg_set_coord(leg_index leg, quad_coord coord); //! Coord offset

//* Angle Control Block
void leg_acb_init(servo_index index, quad_fp (*calc)(quad_fp));

bool leg_acb_update(servo_index index, bool kine_update);
void leg_acb_absolute(servo_index index, quad_fp start, quad_fp end, uint32_t fcb_count);
void leg_acb_relative(servo_index index, quad_fp delta, uint32_t fcb_count);
void leg_acb_target(servo_index index, quad_fp target, uint32_t fcb_count);

void leg_acb_update_blocking(servo_index index, bool kine_update);
void leg_acb_absolute_blocking(servo_index index, quad_fp start, quad_fp end, uint32_t fcb_count);
void leg_acb_relative_blocking(servo_index index, quad_fp delta, uint32_t fcb_count);
void leg_acb_target_blocking(servo_index index, quad_fp target, uint32_t fcb_count);

//* Coord Control Block
void leg_ccb_init(leg_index index, quad_fp (*calc_x)(quad_fp), quad_fp (*calc_z)(quad_fp));
bool leg_ccb_update(leg_index index);   //! Coord Offset
void leg_ccb_absolute(leg_index index, quad_coord start, quad_coord end, uint32_t fcb_count);
void leg_ccb_relative(leg_index index, quad_coord delta, uint32_t fcb_count);
void leg_ccb_target(leg_index index, quad_coord target, uint32_t fcb_count);
void leg_ccb_update_blocking(leg_index index);  //! Coord Offset
void leg_ccb_absolute_blocking(leg_index index, quad_coord start, quad_coord end, uint32_t fcb_count);
void leg_ccb_relative_blocking(leg_index index, quad_coord delta, uint32_t fcb_count);
void leg_ccb_target_blocking(leg_index index, quad_coord target, uint32_t fcb_count);

//! Leg Sync Control
//* Leg Set
void leg_sync_set_angle(quad_fp angle_thigh, quad_fp angle_shank, bool kine_update);
void leg_sync_set_coord(quad_coord coord); //! Coord Offset
void leg_sync_set_angle2(quad_fp angles[SERVO_COUNT], bool kine_update);
void leg_sync_set_coord2(quad_coord coords[LEG_COUNT]); //! Coord Offset

//* Angle Control Block
void leg_sync_acb_init(quad_fp (*calc)(quad_fp));
void leg_sync_acb_init2(quad_fp (*calc[SERVO_COUNT])(quad_fp));

bool leg_sync_acb_update(bool kine_update);
void leg_sync_acb_absolute(quad_fp start, quad_fp end, uint32_t fcb_count);
void leg_sync_acb_absolute2(quad_fp start[SERVO_COUNT], quad_fp end[SERVO_COUNT], uint32_t fcb_count);
void leg_sync_acb_relative(quad_fp delta, uint32_t fcb_count);
void leg_sync_acb_relative2(quad_fp delta[SERVO_COUNT], uint32_t fcb_count);
void leg_sync_acb_target(quad_fp target, uint32_t fcb_count);
void leg_sync_acb_target2(quad_fp target[SERVO_COUNT], uint32_t fcb_count);

void leg_sync_acb_update_blocking(bool kine_update);
void leg_sync_acb_absolute_blocking(quad_fp start, quad_fp end, uint32_t fcb_count);
void leg_sync_acb_absolute2_blocking(quad_fp start[SERVO_COUNT], quad_fp end[SERVO_COUNT], uint32_t fcb_count);
void leg_sync_acb_relative_blocking(quad_fp delta, uint32_t fcb_count);
void leg_sync_acb_relative2_blocking(quad_fp delta[SERVO_COUNT], uint32_t fcb_count);
void leg_sync_acb_target_blocking(quad_fp target, uint32_t fcb_count);
void leg_sync_acb_target2_blocking(quad_fp target[SERVO_COUNT], uint32_t fcb_count);

//* Coord Control Block
void leg_sync_ccb_init(quad_fp (*calc_x)(quad_fp), quad_fp (*calc_z)(quad_fp));
void leg_sync_ccb_init2(quad_fp (*calc_x[LEG_COUNT])(quad_fp), quad_fp (*calc_z[LEG_COUNT])(quad_fp));

bool leg_sync_ccb_update(void); //! Coord Offset
void leg_sync_ccb_absolute(quad_coord start, quad_coord end, uint32_t fcb_count);
void leg_sync_ccb_absolute2(quad_coord start[LEG_COUNT], quad_coord end[LEG_COUNT], uint32_t fcb_count);
void leg_sync_ccb_relative(quad_coord delta, uint32_t fcb_count);
void leg_sync_ccb_relative2(quad_coord delta[LEG_COUNT], uint32_t fcb_count);
void leg_sync_ccb_target(quad_coord target, uint32_t fcb_count);
void leg_sync_ccb_target2(quad_coord target[LEG_COUNT], uint32_t fcb_count);

void leg_sync_ccb_update_blocking(void);
void leg_sync_ccb_absolute_blocking(quad_coord start, quad_coord end, uint32_t fcb_count);
void leg_sync_ccb_absolute2_blocking(quad_coord start[LEG_COUNT], quad_coord end[LEG_COUNT], uint32_t fcb_count);
void leg_sync_ccb_relative_blocking(quad_coord delta, uint32_t fcb_count);
void leg_sync_ccb_relative2_blocking(quad_coord delta[LEG_COUNT], uint32_t fcb_count);
void leg_sync_ccb_target_blocking(quad_coord target, uint32_t fcb_count);
void leg_sync_ccb_target2_blocking(quad_coord target[LEG_COUNT], uint32_t fcb_count);

//! Quadruped Control
//* Base
void quad_init(void);
//* Fixed
void quad_fixed_stand0(void);
void quad_fixed_fall0(void);
//* Trot
void quad_tort_init(
    quad_fp swing_width, quad_fp swing_height, quad_fp cuf_off_percentage, quad_coord base, fcb_mode mode, uint32_t frame_count, uint32_t frame_interval);
void quad_tort(uint32_t step_count);
//* Step In Place
void quad_sip_init(
  quad_fp swing_height, quad_fp cuf_off_percentage, quad_coord base, fcb_mode mode, uint32_t frame_count, uint32_t frame_interval);
void quad_sip(uint32_t step_count);
void quad_sip2(uint32_t step_count, uint32_t frame_count);
void quad_sip3(uint32_t step_count);

//!常量 v_const.c
#if CONFIG_CONST_TYPE == 0
  //PI
  extern const quad_fp PI;
  extern const quad_fp DPI;
  extern const quad_fp HPI;
  //大腿组
  extern const quad_fp L1;
  extern const quad_fp L2;
  extern const quad_fp L3;
  extern const quad_fp L5;
  extern const quad_fp R15;
  //小腿组
  extern const quad_fp L8;
  extern const quad_fp L9;
#else
  //PI
  #define PI  CONFIG_PI
  #define DPI CONFIG_DPI
  #define HPI CONFIG_HPI
  //大腿组
  #define L1  CONFIG_L1
  #define L2  CONFIG_L2
  #define L3  CONFIG_L3
  #define L5  CONFIG_L5
  #define R15 CONFIG_R15
  //小腿组
  #define L8  CONFIG_L8
  #define L9  CONFIG_L9
#endif

//!核心变量全局 v_kernel.c
extern quad_kine _kine[LEG_COUNT];

extern quad_fcb _acb_fcb[SERVO_COUNT];
extern quad_fcb _ccb_fcb[LEG_COUNT];
extern quad_acb _acb[SERVO_COUNT];
extern quad_ccb _ccb[LEG_COUNT];

extern quad_fcb _sync_acb_fcb;
extern quad_fcb _sync_ccb_fcb;
extern quad_acb _sync_acb;
extern quad_ccb _sync_ccb;

//!运动学特殊点 v_kine_sp.c
extern const quad_kine kfs_x_min;
extern const quad_kine kfs_x_max;
extern const quad_kine kfs_z_min;
extern const quad_kine kfs_z_max;
extern const quad_kine kfs_x0_z_min;
extern const quad_kine kfs_x0_z_max;
extern const quad_kine kfs_start;
extern const quad_kine kfs_end;

//!运动学前向点 v_kine_fp.c
extern const quad_coord kf[181][121];

//!用户特殊坐标 v_user_sc.c
extern const quad_coord cc_stand0;
extern const quad_coord cc_stand1;
extern const quad_coord cc_start;
extern const quad_coord cc_end;