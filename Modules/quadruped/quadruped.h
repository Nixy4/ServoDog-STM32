#pragma once

#include "config.h"
#include "stm32f4xx_hal.h"
#include "elog.h"
#include "math.h"
#include "aeabi.h"

#if CONFIG_FLOAT_TYPE == 0
  typedef float quad_fp;
#else
  typedef double quad_fp;
#endif

#if CONFIG_CACULATE_TYPE == 0
  quad_fp radians(quad_fp degree);
  quad_fp degrees(quad_fp radian);
#else
  #define radians(degree) ((degree) * CONFIG_DEGREE_TO_RADIAN)
  #define degrees(radian) ((radian) * CONFIG_RADIAN_TO_DEGREE)
#endif

typedef struct quad_servo
{
  uint8_t channel;
  quad_fp offset;
} quad_servo, quad_servo_cfg;

typedef struct quad_agnle
{
  quad_fp AS1;
  quad_fp AS2;
} quad_agnle;

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

typedef enum quad_types 
{
  LEG_RF = 0,
  LEG_LF,
  LEG_RB,
  LEG_LB,
  ACB_THIGH,
  ACB_SHANK,
} quad_types;

typedef struct quad_frame
{
  uint32_t count;
  uint32_t interval;

  volatile uint32_t index;
  volatile uint32_t elapsed_tick;
  volatile uint32_t last_tick;

} quad_frame;

typedef struct quad_angle_ctrlblock
{
  quad_types type;

  quad_fp (*calc)(quad_fp i);
  quad_frame frame;

  quad_fp start;
  quad_fp end;
  quad_fp delta;

} quad_acb;

typedef struct quad_coord_ctrlblock
{
  quad_fp (*calc_x)(quad_fp i);
  quad_fp (*calc_z)(quad_fp i);
  quad_frame frame;

  quad_fp start_x;
  quad_fp start_z;
  quad_fp end_x;
  quad_fp end_z;
  quad_fp delta_x;
  quad_fp delta_z;

} quad_ccb;

typedef struct quad_leg
{
  uint32_t is_init;

  quad_types type;

  quad_servo thigh_servo;
  quad_servo shank_servo;

  quad_kine kine;

  quad_acb tacb;
  quad_acb sacb;

  quad_ccb ccb;

} quad_leg;

typedef struct quad_frame_cfg
{
  uint32_t interval;
} quad_frame_cfg;

typedef struct quad_acb_cfg
{
  quad_fp (*calc)(quad_fp);
  quad_frame_cfg frame_cfg;
} quad_acb_cfg;

typedef struct quad_ccb_cfg
{
  quad_fp (*calc_x)(quad_fp);
  quad_fp (*calc_z)(quad_fp);
  quad_frame_cfg frame_cfg;
} quad_ccb_cfg;

typedef struct quad_leg_cfg
{
  quad_types type;

  quad_servo_cfg thigh_servo_cfg;
  quad_servo_cfg shank_servo_cfg;

  quad_acb_cfg* tacb_cfg;
  quad_acb_cfg* sacb_cfg;

  quad_ccb_cfg* ccb_cfg;

} quad_leg_cfg;

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

void servo_set_freq(quad_fp freq);
void servo_set_angle(quad_servo* servo, quad_fp angle);

void kine_elog_d(quad_kine* kine);
bool kine_compare(const quad_kine* kine1, const quad_kine* kine2);
void kine_forward(quad_kine* kine, quad_fp AS1, quad_fp AS2);
void kine_inverse(quad_kine* kine, quad_fp X, quad_fp Z);
void kine_sptest(void);
void kine_fptest(void);

void leg_init(quad_leg* leg, quad_leg_cfg cfg);
void led_set_angle(quad_leg* leg, quad_fp thigh_angle, quad_fp shank_angle, bool fowward);
void leg_set_coord(quad_leg* leg, quad_fp X, quad_fp Z);

void leg_acb_abslute(quad_leg* leg, quad_acb* acb, quad_fp start, quad_fp end, uint32_t frame_count);
void leg_acb_relative(quad_leg* leg, quad_acb* acb, quad_fp delta, uint32_t frame_count);
void leg_acb_target(quad_leg* leg, quad_acb* acb, quad_fp target, uint32_t frame_count);
bool leg_acb_update(quad_leg* leg, quad_acb* acb);
void leg_acb_update_block(quad_leg* leg, quad_acb* acb);
bool leg_acb_update_all(void);
void leg_acb_update_all_block(void);

void leg_ccb_abslute(quad_leg* leg, quad_ccb* ccb, quad_coord start, quad_coord end, uint32_t frame_count);
void leg_ccb_relative(quad_leg* leg, quad_ccb* ccb, quad_coord delta, uint32_t frame_count);
void leg_ccb_target(quad_leg* leg, quad_ccb* ccb, quad_coord target, uint32_t frame_count);
bool leg_ccb_update(quad_leg* leg, quad_ccb* ccb);
void leg_ccb_update_block(quad_leg* leg, quad_ccb* ccb);
bool leg_ccb_update_all(void);
void leg_ccb_update_all_block(void);

void sync_servo_set_angle(quad_fp angles[8]);

void sync_leg_set_angle(quad_agnle rfa, quad_agnle lfa, quad_agnle rba, quad_agnle lba);
void sync_leg_set_coord(quad_coord rfc, quad_coord lfc, quad_coord rbc, quad_coord lbc);

void sync_leg_acb_start(uint32_t frame_count);
bool sync_leg_acb_update();
void sync_leg_acb_updata_block();

void sync_leg_ccb_start(uint32_t frame_count);
bool sync_leg_ccb_update();
void sync_leg_ccb_update_block();

void fixed_set_angle_zero(void);
void fixed_set_angle_mid(void);

void fixed_stand_by_coord0(void);
void fixed_stand_by_coord1(void);
void fixed_stand_by_acb(uint32_t frame_count);
void fixed_stand_by_ccb(uint32_t frame_count);

void walk_init(quad_fp swing_width, quad_fp swing_height, quad_fp swing_duty);
void walk(uint32_t frame_count, uint32_t steps);

void sil_init(quad_fp swing_height, quad_fp swing_duty);
void sil(uint32_t frame_count, uint32_t times);
void sync_sil(uint32_t frame_count, uint32_t times);

//! 四条腿
extern quad_leg leg_rf;
extern quad_leg leg_lf;
extern quad_leg leg_rb;
extern quad_leg leg_lb;

#if CONFIG_CACULATE_TYPE == 0
  //!PI
  extern const quad_fp PI;
  extern const quad_fp DPI;
  extern const quad_fp HPI;
  //!大腿组
  extern const quad_fp L1;
  extern const quad_fp L2;
  extern const quad_fp L3;
  extern const quad_fp L5;
  extern const quad_fp R15;
  //!小腿组
  extern const quad_fp L8;
  extern const quad_fp L9;
#else
  //!PI
  #define PI  CONFIG_PI
  #define DPI CONFIG_DPI
  #define HPI CONFIG_HPI
  //!大腿组
  #define L1  CONFIG_L1
  #define L2  CONFIG_L2
  #define L3  CONFIG_L3
  #define L5  CONFIG_L5
  #define R15 CONFIG_R15
  //!小腿组
  #define L8  CONFIG_L8
  #define L9  CONFIG_L9
#endif

//!特殊点
extern const quad_kine ksp_x_min;
extern const quad_kine ksp_x_max;
extern const quad_kine ksp_z_min;
extern const quad_kine ksp_z_max;
extern const quad_kine ksp_x0_z_min;
extern const quad_kine ksp_x0_z_max;
extern const quad_kine ksp_start;
extern const quad_kine ksp_end;

//!运动学前向点
extern const quad_coord kfp[181][121];