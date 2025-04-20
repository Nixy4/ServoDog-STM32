#pragma once

#include "servo.h"
#include "kinematics.h"

enum
{
  LEG_ID_RF = 0U,
  LEG_ID_RB,
  LEG_ID_LB,
  LEG_ID_LF,
  LEG_ID_MAX
} ;

typedef struct
{
  int id;
  kinematics_data_t data;
  servo_t thighServo;
  servo_t shankhServo;
} leg_t;

typedef struct
{
  int id;
  int thighServoId;
  double thighAngle;
  double thighOffset;
  int shankServoId;
  double shankAngle;
  double shankOffset;
} leg_config_t;

int leg_init(leg_t* leg, const leg_config_t* cfg);
int leg_set_angle(leg_t* leg, double thighAngle, double shankAngle, bool isInverse);
int leg_set_coord(leg_t* leg, double x, double z);

int leg_turn_absolute(leg_t* leg, double thighAngle1, double thighAngle2, double shankAngle1, double shankAngle2, double ms);
int leg_turn_relative(leg_t* leg, double thighAngle, double shankAngle, double ms);
int leg_turn_target(leg_t* leg, double thighAngle, double shankAngle, double ms);

int leg_move_abselute(leg_t* leg, double x1, double z1, double x2, double z2, double ms);
int leg_move_relative(leg_t* leg, double x, double z, double step);
int leg_move_target(leg_t* leg, double x, double z, double step);

int leg_update(leg_t* leg);