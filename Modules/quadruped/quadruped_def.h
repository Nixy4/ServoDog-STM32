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

typedef uint16_t EasingMode;

typedef enum
{
  LEG_ID_RIGHT_START = 0U,
  LEG_ID_RF = LEG_ID_RIGHT_START,
  LEG_ID_RB,
  LEG_ID_LEFT_START,
  LEG_ID_LB = LEG_ID_LEFT_START,
  LEG_ID_LF,
  LEG_ID_MAX
} LegID;

struct Coord
{
  float X;
  float Z;
};
typedef struct Coord Coord;

typedef struct
{
  float AS1, AS2;
  float RS1, RS2;
  float L6, L7;
  float R12, R13, R17, R35, R7X;
  Coord COORD;
} KinematicsData;

typedef struct
{
  Coord legsCoord[4];
} GaitData;

typedef struct EasingAngle EasingAngle;
typedef struct EasingAngleConfig EasingAngleConfig;
typedef float (*EasingAngleFunciton)(float v);

typedef struct EasingCoord EasingCoord;
typedef struct EasingCoordConfig EasingCoordConfig;
typedef Coord (*EasingCoordFunciton)(EasingCoord* ec);

typedef struct EasingGait EasingGait;
typedef struct EasingGaitConfig EasingGaitConfig;
typedef GaitData (*EasingGaitFunciton)(EasingGait* eg);

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
  float start1, start2;
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
  float start1, start2;
  float angle1, angle2;

  //! KinematicsData !//
  KinematicsData IKINE;

  //! Easing Angle !//
  EasingAngle ea1;
  EasingAngle ea2;
  
  //! Easing Coord !//
  EasingCoord ec;
} ;

struct EasingGait
{
  EasingGaitFunciton function;
  float swingWidth;
  float swingHeight;
  float swingDuty;
  Coord offset;
  Coord current;
  uint32_t frameCount;
  uint32_t frameIndex;
  uint32_t frameInverval;
  uint32_t loopCount;
} ;

struct EasingGaitConfig
{
  EasingGaitFunciton function;
  float swingWidth;
  float swingHeight;
  float swingDuty;
  Coord offset;
  uint32_t frameCount;
  uint32_t frameInverval;
  uint32_t loopCount;
} ;

void leg_inverse_kinematics(Leg* leg, float X, float Z);

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

Coord _easing_gait_walk_first(EasingGait* eg);
Coord _easing_gait_walk_second(EasingGait* eg);
 
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

Leg* leg_ptr(LegID id);
int  leg_init(LegID id, const LegConfig* cfg);
void leg_set_angle(LegID id, float a1, float a2);
void leg_set_coord(LegID id, float x, float z);

void leg_ea_init(EasingAngle* ea, const EasingAngleConfig* cfg);
void leg_ea_absolute(LegID id, float a1_start, float a1_stop, float a2_start, float a2_stop, uint16_t frames);
void leg_ea_relative(LegID id, float distance1, float distance2, uint16_t frames);
void leg_ea_target(LegID id, float target1, float target2, uint16_t frames);
bool leg_ea_is_complete(LegID id);
bool leg_ea_update(LegID id);

void _leg_ec_init(EasingCoord* ec, const EasingCoordConfig* cfg);
void leg_ec_absolute(LegID id, float x_start, float z_start, float x_stop, float z_stop, uint16_t frames);
void leg_ec_relative(LegID id, float x_distance, float z_distance, uint16_t frames);
void leg_ec_target(LegID id, float x_target, float z_target, uint16_t frames);
bool leg_ec_is_complete(LegID id);
bool leg_ec_update(LegID id);

extern const KinematicsData fksp_x_min;
extern const KinematicsData fksp_x_max;
extern const KinematicsData fksp_z_min;
extern const KinematicsData fksp_z_max;
extern const KinematicsData fksp_x0_z_min;
extern const KinematicsData fksp_x0_z_max;
extern const KinematicsData fksp_start;
extern const KinematicsData fksp_stop;