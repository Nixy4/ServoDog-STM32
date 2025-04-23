#pragma once

#include "stdint.h"
#include "stdbool.h"
#include "math.h"

#include "elog.h"

#define PI 3.141592653589793f
#define DPI 6.283185307179586f
#define HPI 1.570796326794897f

#define L1 80.0f
#define L2 65.0f
#define L3 20.0f
#define L5 81.54140052758476f
#define L8 15.0f
#define L9 73.0f
#define R15 0.4032814817188361f

#include "stm32f4xx_hal.h"
#define EASING_GET_TICK()       HAL_GetTick()

#include "pca9685.h"

#define EASING_MODE_BITCNT      (4U)
#define EASING_MODE_MASK        ((1U << EASING_MODE_BITCNT) - 1U)

#define EASING_TIMES_SINGLE     (0U << 0U)
#define EASING_TIMES_MANYTIMES  (1U << 0U)
#define EASING_TIMES_INFINITE   (1U << 1U)

#define EASING_TIMES_SET        (EASING_MODE_BITCNT)

#define EASING_DIR_FORWARD      (0U << 0U)
#define EASING_DIR_REVERSE      (1U << 2U)
#define EASING_DIR_BACKANDFORTH (1U << 3U)

#define EASING_MODE_DEFAULT     EASING_TIMES_SINGLE | EASING_DIR_FORWARD
#define EASING_MODE_NTIMES(n)   EASING_TIMES_MANYTIMES | (n << EASING_TIMES_SET)

#define EASING_FRAME_COUNT_DEFUALT  5U

#define AMIRRORID(id,x) (id < LEG_ID_LEFT_START ? x : (180.f - x))
#define AMIRRORR(x)     (x)
#define AMIRRORL(x)     (180.f - x)
#define ALIMITT(x)      (x > 180.f ? 180.f : (x < 0.f ? 0.f : x))
#define ALIMITS(x)      (x > 120.f ? 120.f : (x < 0.f ? 0.f : x))
#define AOFFSET(x,off)  (x + off)
#define RADIANS(x)      ((x) * 0.01745329251994329547f)
#define DEGREES(x)      ((x) * 57.29577951308232286465f)

typedef uint8_t EasingMode;

typedef enum
{
  LEG_ID_START = 0U, 
  LEG_ID_RIGHT_START = LEG_ID_START,
  LEG_ID_RF = LEG_ID_START,
  LEG_ID_RB,
  LEG_ID_LEFT_START,
  LEG_ID_LB = LEG_ID_LEFT_START,
  LEG_ID_LF,
  LEG_ID_MAX,
  LEG_ID_END = LEG_ID_MAX,
} LegID;

typedef struct Coord
{
  float X;
  float Z;
} Coord;

typedef union QuadCoord
{
  Coord id[4];
  struct
  {
    Coord rf;
    Coord rb;
    Coord lf;
    Coord lb;
  } orientation;
} QuadCoord;

typedef struct
{
  float AS1, AS2;
  float RS1, RS2;
  float L6, L7;
  float R12, R13, R17, R35, R7X;
  Coord COORD;
} KinematicsData;

typedef struct EasingAngle EasingAngle;
typedef struct EasingAngleConfig EasingAngleConfig;
typedef float (*EasingAngleFunciton)(float);

typedef struct EasingCoord EasingCoord;
typedef struct EasingCoordConfig EasingCoordConfig;
typedef Coord (*EasingCoordFunciton)(EasingCoord*);

typedef struct Gait Gait;
typedef struct GaitConfig GaitConfig;
typedef QuadCoord (*GaitFunciton)(Gait*);

typedef struct LegConfig LegConfig;
typedef struct Leg Leg;

struct EasingAngle
{
  EasingMode mode;
  EasingAngleFunciton function;
  float start;
  float stop;
  float delta;
  float step;
  float current;
  uint16_t frameIndex;
  uint16_t frameCount;
  uint32_t elapsedTick;
  uint32_t lastTick;
  bool direction;
  int16_t times;
  uint16_t interval;
} ;

struct EasingAngleConfig
{
  EasingMode mode;
  EasingAngleFunciton function;
  uint16_t frameCount;
  uint32_t interval;
} ;

struct EasingCoord
{
  EasingMode mode;
  EasingCoordFunciton function;
  Coord start;
  Coord stop;
  Coord delta;
  Coord step;
  Coord current;
  uint16_t frameIndex;
  uint16_t frameCount;
  uint32_t elapsedTick;
  uint32_t lastTick;
  bool direction;
  int16_t times;
  uint16_t interval;
  void* customData;
} ;

struct EasingCoordConfig
{
  EasingMode mode;
  EasingCoordFunciton function;
  uint16_t frameCount;
  uint32_t interval;
  void* customData;
} ;

struct LegConfig
{
  //! Hardware !//
  int pcaChannel1 , pcaChannel2;
  float offset1, offset2;
  float angle1, angle2;

  //! Easing Angle !//
  EasingAngleConfig ea1_config;
  EasingAngleConfig ea2_config;

  //! Easing Coord !//
  EasingCoordConfig ec_config;
} ;

struct Leg
{
  //! Hardware !//
  uint8_t pcaChannel1 , pcaChannel2;
  float offset1, offset2;
  float angle1, angle2;

  //! KinematicsData !//
  KinematicsData IKINE;

  //! Easing Angle !//
  EasingAngle ea1;
  EasingAngle ea2;
  
  //! Easing Coord !//
  EasingCoord ec;
} ;

struct Gait
{
  GaitFunciton startFunc;
  GaitFunciton periodFunc;
  GaitFunciton stopFunc;

  float swingWidth;
  float swingHeight;
  float swingDuty;

  Coord offset;
  QuadCoord current;

  float frameCount;
  float frameIndex;
  float frameInverval;

  int16_t times;
} ;

struct GaitConfig
{
  GaitFunciton startFunc;
  GaitFunciton periodFunc;
  GaitFunciton stopFunc;

  float swingWidth;
  float swingHeight;
  float swingDuty;

  Coord offset;

  float frameCount;
  float frameInverval;
} ;

void _elog_d_ikine(KinematicsData* ikine);
void kinematics_inverse(Leg* leg, float X, float Z);

float _easing_base_Linear(const float t);       // linear t

float _easing_base_InQuad(const float t);       // quadratic t^2
float _easing_base_OutQuad(const float t);
float _easing_base_InOutQuad(const float t);

float _easing_base_InCubic(const float t);      // cubic t^3
float _easing_base_OutCubic(const float t);
float _easing_base_InOutCubic(const float t);

float _easing_base_InQuart(const float t);      // quartic t^4
float _easing_base_OutQuart(const float t);
float _easing_base_InOutQuart(const float t);

float _easing_base_InQuint(const float t);      // quintic t^5
float _easing_base_OutQuint(const float t);
float _easing_base_InOutQuint(const float t);

float _easing_base_InSine(const float t);       // sinusoidal 正弦 sin(t)
float _easing_base_OutSine(const float t);
float _easing_base_InOutSine(const float t);

float _easing_base_InExpo(const float t);       // exponential 指数 2^t
float _easing_base_OutExpo(const float t);
float _easing_base_InOutExpo(const float t);

float _easing_base_InCirc(const float t);       // circular 圆形
float _easing_base_OutCirc(const float t);
float _easing_base_InOutCirc(const float t);

float _easing_base_InBack(const float t);       // elastic 衰减三次幂 (s+1)t^3 - st^2
float _easing_base_OutBack(const float t);
float _easing_base_InOutBack(const float t);

float _easing_base_InElastic(const float t);    // elastic 衰减正弦
float _easing_base_OutElastic(const float t);
float _easing_base_InOutElastic(const float t);

float _easing_base_InBounce(const float t);     // back 衰减反弹
float _easing_base_OutBounce(const float t);
float _easing_base_InOutBounce(const float t);

Coord _easing_coord_Linear(EasingCoord* ec);
 
#define EASING_ANGLE_CONFIG_DEFAULT() {\
  .mode = EASING_MODE_DEFAULT,\
  .function = _easing_base_Linear,\
  .frameCount = 500,\
  .interval = 0\
}

#define EASING_COORD_CONFIG_DEFAULT() {\
  .mode = EASING_MODE_DEFAULT,\
  .function = _easing_coord_Linear,\
  .frameCount = 500,\
  .interval = 0,\
  .customData = NULL\
}

#define EASING_GAIT_WALK_FIRST() {\
  .function = _easing_gait_walk_first,\
  .swingWidth = 40.0f,\
  .swingHeight = 45.0f,\
  .swingDuty = 0.5f,\
  .offset = {0.0f, 115.0f},\
  .frameCount = 500,\
  .frameInverval = 0,\
  .loopCount = 10\
}

#define EASING_GAIT_WALK_SECOND() {\
  .function = _easing_gait_walk_second,\
  .swingWidth = 40.0f,\
  .swingHeight = 45.0f,\
  .swingDuty = 0.5f,\
  .offset = {0.0f, 115.0f},\
  .frameCount = 500,\
  .frameInverval = 0,\
  .loopCount = 10\
}

Leg* leg_ptr(Leg* leg);
int  leg_init(Leg* leg, const LegConfig* cfg);
void leg_set_angle(Leg* leg, float a1, float a2);
void leg_set_coord(Leg* leg, float x, float z);

void leg_eangle_absolute(Leg* leg, float a1_start, float a1_stop, float a2_start, float a2_stop, uint16_t frames);
void leg_eangle_relative(Leg* leg, float distance1, float distance2, uint16_t frames);
void leg_eangle_target(Leg* leg, float target1, float target2, uint16_t frames);
bool leg_eangle_update(Leg* leg);
void leg_eangle_absolute_block(Leg* leg, float a1_start, float a1_stop, float a2_start, float a2_stop, uint16_t frames);
void leg_eangle_relative_block(Leg* leg, float distance1, float distance2, uint16_t frames);
void leg_eangle_target_block(Leg* leg, float target1, float target2, uint16_t frames);
void leg_eangle_update_block(Leg* leg);
void leg_eangle_update_all_block();

void leg_ecoord_absolute(Leg* leg, float x_start, float z_start, float x_stop, float z_stop, uint16_t frames);
void leg_ecoord_relative(Leg* leg, float x_distance, float z_distance, uint16_t frames);
void leg_ecoord_target(Leg* leg, float x_target, float z_target, uint16_t frames);
bool leg_ecoord_update(Leg* leg);
void leg_ecoord_absolute_block(Leg* leg, float x_start, float z_start, float x_stop, float z_stop, uint16_t frames);
void leg_ecoord_relative_block(Leg* leg, float x_distance, float z_distance, uint16_t frames);
void leg_ecoord_target_block(Leg* leg, float x_target, float z_target, uint16_t frames);
void leg_ecoord_update_block(Leg* leg);
void leg_ecoord_update_all_block(void);

void quad_standup0(uint16_t frames);
void quad_falldown0(uint16_t frames);
void quad_falldown1(uint16_t frames);

QuadCoord _quad_gait_start_walk(Gait* qg);
QuadCoord _quad_gait_period_walk(Gait* qg);
QuadCoord _quad_gait_stop_walk(Gait* qg);

void quadruped_gait_init(Gait* eg, const GaitConfig* cfg);
bool quadruped_gait_update(Gait* eg);
void quadruped_gait_operation0(Gait* eg, uint16_t frames);
void quadruped_gait_operation1(Gait* eg, uint16_t frames, int16_t times);
void quadruped_gait_operation0_block(Gait* eg, uint16_t frames);
void quadruped_gait_operation1_block(Gait* eg, uint16_t frames, int16_t times);

extern const KinematicsData kfsp_x_min;
extern const KinematicsData kfsp_x_max;
extern const KinematicsData kfsp_z_min;
extern const KinematicsData kfsp_z_max;
extern const KinematicsData kfsp_x0_z_min;
extern const KinematicsData kfsp_x0_z_max;
extern const KinematicsData kfsp_start;
extern const KinematicsData kfsp_stop;