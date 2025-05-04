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
  quad_float thighOffset;
  quad_float shankOffset;
  quad_float thighAngle;
  quad_float shankAngle;
} leg_init_t;

int leg_init(leg_t* leg, const leg_init_t* cfg);
int leg_set_angle(leg_t* leg, quad_float thighAngle, quad_float shankAngle, bool isInverse);
int leg_set_coord(leg_t* leg, quad_float x, quad_float z);

int leg_turn_absolute(leg_t* leg, quad_float thighAngle1, quad_float thighAngle2, quad_float shankAngle1, quad_float shankAngle2, quad_float ms);
int leg_turn_relative(leg_t* leg, quad_float thighAngle, quad_float shankAngle, quad_float ms);
int leg_turn_target(leg_t* leg, quad_float thighAngle, quad_float shankAngle, quad_float ms);

int leg_move_abselute(leg_t* leg, quad_float x1, quad_float z1, quad_float x2, quad_float z2, quad_float ms);
int leg_move_relative(leg_t* leg, quad_float x, quad_float z, quad_float step);
int leg_move_target(leg_t* leg, quad_float x, quad_float z, quad_float step);

int leg_update(leg_t* leg);