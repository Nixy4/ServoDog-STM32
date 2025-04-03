#pragma once

#include "servo.h"

#define LEG_TYPE_LF 0x00
#define LEG_TYPE_LB 0x01
#define LEG_TYPE_RF 0x10
#define LEG_TYPE_RB 0x11



typedef struct
{
  int type;
  double thighAngle;
  double shankAngle;
  double x;
  double z;
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
}leg_config_t;

int leg_init(leg_t* leg, const leg_config_t* cfg);
int leg_set_angle(leg_t* leg, double thighAngle, double shankAngle, bool syncCoord);
int leg_set_coord(leg_t* leg, double x, double z);