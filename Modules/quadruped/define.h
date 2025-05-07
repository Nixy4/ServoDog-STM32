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

void servo_set_freq(quad_fp freq);
void servo_set_angle(quad_servo* servo, quad_fp angle);

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

void kine_elog_d(quad_kine* kine);
bool kine_compare(const quad_kine* kine1, const quad_kine* kine2);
void kine_forward(quad_kine* kine, quad_fp AS1, quad_fp AS2);
void kine_inverse(quad_kine* kine, quad_fp X, quad_fp Z);
void kine_sptest(void);
void kine_fptest(void);

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
  quad_fp (*calc)(quad_fp i);
  quad_frame frame;

  quad_fp start;
  quad_fp end;
  quad_fp delta;
  quad_fp current;

} quad_angle_ctrlblock;

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
  volatile quad_fp current_x;
  volatile quad_fp current_z;

} quad_coord_ctrlblock;

typedef enum quad_leg_id 
{
  LEG_ID_RF = 0,
  LEG_ID_LF,
  LEG_ID_RB,
  LEG_ID_LB,
} quad_leg_id;

typedef struct quad_leg
{
  quad_leg_id id;

  quad_servo thigh_servo;
  quad_servo shank_servo;

  volatile quad_kine kine;

  quad_angle_ctrlblock tacb;
  quad_angle_ctrlblock sacb;

  quad_coord_ctrlblock ccb;

} quad_leg;

typedef struct quad_frame_cfg
{
  uint32_t interval;
} quad_frame_cfg;

typedef struct quad_acb_cfg
{
  quad_fp (*calc)(quad_fp i);
  quad_frame_cfg frame_cfg;
} quad_acb_cfg;

typedef struct quad_ccb_cfg
{
  quad_fp (*calc_x)(quad_fp i);
  quad_fp (*calc_z)(quad_fp i);
  quad_frame_cfg frame_cfg;
} quad_ccb_cfg;



typedef struct quad_leg_cfg
{
  quad_leg_id id;

  quad_servo_cfg thigh_servo_cfg;
  quad_servo_cfg shank_servo_cfg;

  quad_acb_cfg tacb_cfg;
  quad_acb_cfg sacb_cfg;

  quad_ccb_cfg ccb_cfg;

} quad_leg_cfg;

#if CONFIG_CACULATE_TYPE == 0

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

//!前向点
extern const quad_coord kfp[181][121];