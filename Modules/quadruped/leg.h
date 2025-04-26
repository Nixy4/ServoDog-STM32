#pragma once

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
  double thighAngle;
  double thighOffset;
  int shankServoId;
  double shankAngle;
  double shankOffset;
} leg_init_t;

int leg_init(leg_t* leg, const leg_init_t* cfg);
int leg_set_angle(leg_t* leg, double thighAngle, double shankAngle, bool isInverse);
int leg_set_coord(leg_t* leg, double x, double z);

int leg_turn_absolute(leg_t* leg, double thighAngle1, double thighAngle2, double shankAngle1, double shankAngle2, double ms);
int leg_turn_relative(leg_t* leg, double thighAngle, double shankAngle, double ms);
int leg_turn_target(leg_t* leg, double thighAngle, double shankAngle, double ms);

int leg_move_abselute(leg_t* leg, double x1, double z1, double x2, double z2, double ms);
int leg_move_relative(leg_t* leg, double x, double z, double step);
int leg_move_target(leg_t* leg, double x, double z, double step);

int leg_update(leg_t* leg);