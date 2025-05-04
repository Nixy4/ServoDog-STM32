#pragma once
#include "base.h"
#include "servo.h"
#include "kinematics.h"

#define LEG_TYPE_LF 0x00
#define LEG_TYPE_LB 0x01
#define LEG_TYPE_RF 0x10
#define LEG_TYPE_RB 0x11

typedef struct
{
  int type;
  kinematics_data_t data;
  // kinematics_data_t data2;
  servo_t thighServo;
  servo_t shankhServo;
} leg_t;

typedef struct
{
  int type;
  int thighServoId;
  int shankServoId;
  QUAD_TYPE thighOffset;
  QUAD_TYPE shankOffset;
  QUAD_TYPE thighAngle;
  QUAD_TYPE shankAngle;
} leg_init_t;

int leg_init(leg_t* leg, const leg_init_t* cfg);
int leg_set_angle(leg_t* leg, QUAD_TYPE thighAngle, QUAD_TYPE shankAngle, bool isInverse);
int leg_set_coord(leg_t* leg, QUAD_TYPE x, QUAD_TYPE z);

int leg_turn_absolute(leg_t* leg, QUAD_TYPE thighAngle1, QUAD_TYPE thighAngle2, QUAD_TYPE shankAngle1, QUAD_TYPE shankAngle2, QUAD_TYPE ms);
int leg_turn_relative(leg_t* leg, QUAD_TYPE thighAngle, QUAD_TYPE shankAngle, QUAD_TYPE ms);
int leg_turn_target(leg_t* leg, QUAD_TYPE thighAngle, QUAD_TYPE shankAngle, QUAD_TYPE ms);

int leg_move_abselute(leg_t* leg, QUAD_TYPE x1, QUAD_TYPE z1, QUAD_TYPE x2, QUAD_TYPE z2, QUAD_TYPE ms);
int leg_move_relative(leg_t* leg, QUAD_TYPE x, QUAD_TYPE z, QUAD_TYPE step);
int leg_move_target(leg_t* leg, QUAD_TYPE x, QUAD_TYPE z, QUAD_TYPE step);

int leg_update(leg_t* leg);