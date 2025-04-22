#pragma once

//! 头文件做架构设置

#include "pca9685.h"
#include "stdint.h"
#include "math.h"

#define PI  3.141592653589793f
#define DPI 6.283185307179586f
#define HPI 1.570796326794897f

#define L1  80.0f
#define L2  65.0f
#define L3  20.0f
#define L5  81.54140052758476f
#define L8  15.0f
#define L9  73.0f
#define R15 0.4032814817188361f

typedef enum
{
  LEG_ID_RF = 0,
  LEG_ID_RB,
  LEG_ID_LF,
  LEG_ID_LB,
} LegID; 

typedef struct {
  float AS1, AS2;
  float RS1, RS2;
  float L6, L7;
  float R12, R13, R17, R35, R7X;
  float X, Z, KX, KZ;
} KinematicsData;

typedef struct
{
  LegID   id;

  uint8_t thightServoId;
  uint8_t shankServoId;

  //! Hardware !//
  float thightAngle;
  float shankAngle;
  float X;
  float Z;

  //! Kinematics !//
  KinematicsData ikdata;

} Leg ;

void leg_init(Leg *leg,LegID id, 
  uint8_t thightServoId, uint8_t shankServoId, float thightAngle, float shankAngle);
void leg_set_angle(Leg *leg, float thightAngle, float shankAngle);
void leg_set_coord(Leg *leg, float X, float Y);

extern KinematicsData x_min;
extern KinematicsData x_max;
extern KinematicsData z_min;
extern KinematicsData z_max;
extern KinematicsData x0_z_min;
extern KinematicsData x0_z_max;
