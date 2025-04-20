#pragma once

#include "stdint.h"
#include "stdbool.h"

#define PI 3.141592653589793f
#define DPI 6.283185307179586f
#define HPI 1.570796326794897f

typedef struct
{
  float X,Z;
} Coord;

#include "stm32f4xx_hal.h"
#define EASING_GET_TICK()       HAL_GetTick()

typedef uint16_t EasingMode;

typedef struct EasingAngleStruct EasingAngle;
typedef struct EasingCoordStruct EasingCoord;
typedef float (*EasingAngleFunciton)(float v);
typedef Coord (*EasingCoordFunciton)(EasingCoord* ec);

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

#define DEFAULT_EA_CONFIG() {EASING_MODE_DEFAULT,_easing_calc_Linear,60,0}
#define DEFAULT_EC_CONFIG() {EASING_MODE_DEFAULT,default_ec_function,60,0}

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

struct EasingAngleStruct
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

struct EasingCoordStruct
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

typedef struct
{
  float AS1, AS2;
  float RS1, RS2;
  float L6, L7;
  float R12, R13, R17, R35, R7X;
  Coord COORD;
} Kinematics;

typedef struct
{
  //! Hardware !//
  uint8_t pcaChannel1 , pcaChannel2;
  float offset1, offset2;
  float angle1, angle2;

  //! Easing Angle !//
  EasingAngle ea1;
  EasingAngle ea2;
  
  //! Easing Coord !//
  EasingCoord ec;

  //! Kinematics !//
  Kinematics IKINE;
  
} Leg;

typedef struct
{
  EasingMode mode;
  EasingAngleFunciton function;
  uint16_t frameCount;
  uint32_t interval;
} EasingAngleConfig;

typedef struct
{
  EasingMode mode;
  EasingCoordFunciton function;
  uint16_t frameCount;
  uint32_t interval;
  void* customData;
} EasingCoordConfig;

typedef struct
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

} LegConfig;

float _easing_calc_Linear(const float t);       // linear t

float _easing_calc_InQuad(const float t);       // quadratic t^2
float _easing_calc_OutQuad(const float t);
float _easing_calc_InOutQuad(const float t);

float _easing_calc_InCubic(const float t);      // cubic t^3
float _easing_calc_OutCubic(const float t);
float _easing_calc_InOutCubic(const float t);

float _easing_calc_InQuart(const float t);      // quartic t^4
float _easing_calc_OutQuart(const float t);
float _easing_calc_InOutQuart(const float t);

float _easing_calc_InQuint(const float t);      // quintic t^5
float _easing_calc_OutQuint(const float t);
float _easing_calc_InOutQuint(const float t);

float _easing_calc_InSine(const float t);       // sinusoidal 正弦 sin(t)
float _easing_calc_OutSine(const float t);
float _easing_calc_InOutSine(const float t);

float _easing_calc_InExpo(const float t);       // exponential 指数 2^t
float _easing_calc_OutExpo(const float t);
float _easing_calc_InOutExpo(const float t);

float _easing_calc_InCirc(const float t);       // circular 圆形
float _easing_calc_OutCirc(const float t);
float _easing_calc_InOutCirc(const float t);

float _easing_calc_InBack(const float t);       // elastic 衰减三次幂 (s+1)t^3 - st^2
float _easing_calc_OutBack(const float t);
float _easing_calc_InOutBack(const float t);

float _easing_calc_InElastic(const float t);    // elastic 衰减正弦
float _easing_calc_OutElastic(const float t);
float _easing_calc_InOutElastic(const float t);

float _easing_calc_InBounce(const float t);     // back 衰减反弹
float _easing_calc_OutBounce(const float t);
float _easing_calc_InOutBounce(const float t);

int leg_init(LegID id, const LegConfig* cfg);

void leg_set_angle(LegID id, float a1, float a2);
void leg_set_coord(LegID id, float x, float z);

void leg_ea_absolute(LegID id, float a1_start, float a1_stop, float a2_start, float a2_stop);
void leg_ea_relative(LegID id, float distance1, float distance2);
void leg_ea_target(LegID id, float target1, float target2);
bool leg_ea_is_complete(LegID id);
bool leg_ea_update(LegID id);

void leg_ec_absolute(LegID id, float x_start, float z_start, float x_stop, float z_stop);
void leg_ec_relative(LegID id, float x_distance, float z_distance);
void leg_ec_target(LegID id, float x_target, float z_target);
bool leg_ec_is_complete(LegID id);
bool leg_ec_update(LegID id);