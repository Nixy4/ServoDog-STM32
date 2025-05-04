#include "stdio.h"
#include "math.h"
#include "leg.h"
#include "pca9685.h"
#include "servo.h"
#include "easing.h"
#include "elog.h"
#include "kinematics.h"

#define TAG "Leg"

#define RA(x) (x)
#define LA(x) (180.f-x)

static inline quad_float thighAngleLimit(quad_float angle)
{
  if(angle < 0.f){
    angle = 0.f;
  } else if(angle > 180.f){
    angle = 180.f;
  }
  return angle;
}

static inline quad_float shankAngleLimit(quad_float angle)
{
  if(angle < 0.f){
    angle = 0.f;
  } else if(angle > 150.f){
    angle = 150.f;
  }
  return angle;
}

static inline quad_float angleOffset(quad_float angle, quad_float offset)
{
  return angle + offset;
}

static inline quad_float angleMirror(int type, quad_float angle)
{
  if(type == LEG_TYPE_LF || type == LEG_TYPE_LB){
    return LA(angle);
  }else{
    return RA(angle);
  }
}

int leg_init(leg_t* leg, const leg_init_t* cfg)
{
  //! Hardware !//
  leg->type = cfg->type;

  if(servo_init(&leg->thighServo, cfg->thighServoId, _easing_calc_Linear, cfg->thighOffset) != 0){
    return -1;
  }
  if(servo_init(&leg->shankhServo, cfg->shankServoId, _easing_calc_Linear, cfg->shankOffset) != 0){
    return -1;
  }

  leg_set_angle(leg, cfg->thighAngle, cfg->shankAngle, true);

  return 0;
}

int leg_set_angle(leg_t* leg, quad_float thighAngle, quad_float shankAngle, bool isInverse)
{
  //* Kinematics *//
  //如果不是逆解角度
  if(isInverse==false){
    //限制角度范围
    thighAngle = thighAngleLimit(thighAngle);
    shankAngle = shankAngleLimit(shankAngle);
    //计算坐标
    leg->data = forward_kinematics(thighAngle, shankAngle);
  }

  //! Hardware !//
  //角度偏移
  thighAngle = angleOffset(thighAngle, leg->thighServo.fOffset);
  shankAngle = angleOffset(shankAngle, leg->shankhServo.fOffset);
  //角度镜像
  thighAngle = angleMirror(leg->type, thighAngle);
  shankAngle = angleMirror(leg->type, shankAngle);
  //设置舵机角度
  servo_set_angle(&leg->thighServo, thighAngle, false);
  servo_set_angle(&leg->shankhServo, shankAngle, false);
  return 0;
}

int leg_set_coord(leg_t* leg, quad_float x, quad_float z)
{
  leg->data = inverse_kinematics(x, z);
  leg_set_angle(leg, leg->data.AS1, leg->data.AS2, true);
  return 0;
}

int leg_turn_absolute(
  leg_t* leg, quad_float thighAngle1, quad_float thighAngle2, quad_float shankAngle1, quad_float shankAngle2, quad_float ms)
{
  //? Invalid parameters ?//
  if(thighAngle1 == thighAngle2 && shankAngle1 == shankAngle2){
    return 0;
  }

  //* Kinematics *//
  //限制角度范围
  thighAngle1 = thighAngleLimit(thighAngle1);
  thighAngle2 = thighAngleLimit(thighAngle2);
  shankAngle1 = shankAngleLimit(shankAngle1);
  shankAngle2 = shankAngleLimit(shankAngle2);
  //计算坐标
  kinematics_data_t end = forward_kinematics(thighAngle2, shankAngle2);
  //同步终点数据
  leg->data = end;

  //! Hardware !//
  //角度偏移
  thighAngle1 = angleOffset(thighAngle1, leg->thighServo.fOffset);
  thighAngle2 = angleOffset(thighAngle2, leg->thighServo.fOffset);
  shankAngle1 = angleOffset(shankAngle1, leg->shankhServo.fOffset);
  shankAngle2 = angleOffset(shankAngle2, leg->shankhServo.fOffset);
  //角度镜像
  thighAngle1 = angleMirror(leg->type, thighAngle1);
  thighAngle2 = angleMirror(leg->type, thighAngle2);
  shankAngle1 = angleMirror(leg->type, shankAngle1);
  shankAngle2 = angleMirror(leg->type, shankAngle2);
  //设置舵机角度
  servo_turn_absolute(&leg->thighServo, thighAngle1, thighAngle2, ms);
  servo_turn_absolute(&leg->shankhServo, shankAngle1, shankAngle2, ms);
  return 0;
}

int leg_turn_relative(leg_t* leg, quad_float thighAngle, quad_float shankAngle, quad_float ms)
{
  //? Invalid parameters ?//
  if(thighAngle == 0 && shankAngle == 0){
    return 0;
  }
  //计算终点角度
  quad_float thighAngle_ = leg->data.AS1 + thighAngle;
  quad_float shankAngle_ = leg->data.AS2 + shankAngle;

  //* Kinematics *//
  //限制角度范围
  thighAngle = thighAngleLimit(thighAngle_);
  shankAngle = shankAngleLimit(shankAngle_);
  //计算坐标
  kinematics_data_t end = forward_kinematics(thighAngle_, shankAngle_);
  //同步终点数据
  leg->data = end;

  //! Hardware !//
  //角度偏移
  thighAngle = angleOffset(thighAngle, leg->thighServo.fOffset);
  shankAngle = angleOffset(shankAngle, leg->shankhServo.fOffset);
  //角度镜像
  thighAngle = angleMirror(leg->type, thighAngle);
  shankAngle = angleMirror(leg->type, shankAngle);
  //设置舵机角度
  servo_turn_target(&leg->thighServo, thighAngle, ms);
  servo_turn_target(&leg->shankhServo, shankAngle, ms);
  return 0;
}

int leg_turn_target(leg_t* leg, quad_float thighAngle, quad_float shankAngle, quad_float ms)
{
  //? Invalid parameters ?//
  if(thighAngle==leg->data.AS1 && shankAngle==leg->data.AS2){
    return 0;
  }
  
  //* Kinematics *//
  //限制角度范围
  thighAngle = thighAngleLimit(thighAngle);
  shankAngle = shankAngleLimit(shankAngle);
  //计算坐标
  kinematics_data_t end = forward_kinematics(thighAngle, shankAngle);
  //同步终点数据
  leg->data = end;

  //! Hardware !//
  //角度偏移
  thighAngle = angleOffset(thighAngle, leg->thighServo.fOffset);
  shankAngle = angleOffset(shankAngle, leg->shankhServo.fOffset);
  //角度镜像
  thighAngle = angleMirror(leg->type, thighAngle);
  shankAngle = angleMirror(leg->type, shankAngle);
  //设置舵机角度
  servo_turn_target(&leg->thighServo, thighAngle, ms);
  servo_turn_target(&leg->shankhServo, shankAngle, ms);
  return 0;
}

int leg_move_abselute(leg_t* leg, quad_float x1, quad_float z1, quad_float x2, quad_float z2, quad_float ms)
{
  //? Invalid parameters ?//
  if(x1 == x2 && z1 == z2){
    return 0;
  }

  //* Kinematics *//
  kinematics_data_t data1 = inverse_kinematics(x1, z1);
  kinematics_data_t data2 = inverse_kinematics(x2, z2);
  leg->data = data2;

  //! Hardware !//
  quad_float thighAngle1 = data1.AS1;
  quad_float shankAngle1 = data1.AS2;
  quad_float thighAngle2 = data2.AS1;
  quad_float shankAngle2 = data2.AS2;

  if(leg->type==LEG_TYPE_LF || leg->type==LEG_TYPE_LB){
    thighAngle1 = LA(thighAngle1);
    shankAngle1 = LA(shankAngle1);
    thighAngle2 = LA(thighAngle2);
    shankAngle2 = LA(shankAngle2);
  }else{
    thighAngle1 = RA(thighAngle1);
    shankAngle1 = RA(shankAngle1);
    thighAngle2 = RA(thighAngle2);
    shankAngle2 = RA(shankAngle2);
  }
  
  servo_turn_absolute(&leg->thighServo, thighAngle1, thighAngle2, ms);
  servo_turn_absolute(&leg->shankhServo, shankAngle1, shankAngle2, ms);
  return 0;
}

int leg_move_relative(leg_t* leg, quad_float x, quad_float z, quad_float ms)
{
  //? Invalid parameters ?//
  if(x == 0 && z == 0){
    return 0;
  }
  x = leg->data.X + x;
  z = leg->data.Z + z;

  //* Kinematics *//
  kinematics_data_t end = inverse_kinematics(x, z);

  //! Hardware !//
  //运动学角度提取
  quad_float thighAngle = end.AS1;
  quad_float shankAngle = end.AS2;
  //角度偏移
  thighAngle = angleOffset(thighAngle, leg->thighServo.fOffset);
  shankAngle = angleOffset(shankAngle, leg->shankhServo.fOffset);
  //角度镜像
  thighAngle = angleMirror(leg->type, thighAngle);
  shankAngle = angleMirror(leg->type, shankAngle);
  //设置舵机角度
  servo_turn_target(&leg->thighServo, thighAngle, ms);
  servo_turn_target(&leg->shankhServo, shankAngle, ms);
  return 0;
}

int leg_move_target(leg_t* leg, quad_float x, quad_float z, quad_float ms)
{
  //* Kinematics *//
  kinematics_data_t data = inverse_kinematics(x, z);
  leg->data = data;
  quad_float thighAngle = data.AS1;
  quad_float shankAngle = data.AS2;

  //! Hardware !//
  //角度偏移
  thighAngle = angleOffset(thighAngle, leg->thighServo.fOffset);
  shankAngle = angleOffset(shankAngle, leg->shankhServo.fOffset);
  //角度镜像
  thighAngle = angleMirror(leg->type, thighAngle);
  shankAngle = angleMirror(leg->type, shankAngle);
  //设置舵机角度
  servo_turn_target(&leg->thighServo, thighAngle, ms);
  servo_turn_target(&leg->shankhServo, shankAngle, ms);
  return 0;
}

int leg_update(leg_t* leg)
{
  int flag = 0;
  flag |= servo_turn_update(&leg->thighServo);
  flag |= servo_turn_update(&leg->shankhServo);
  return flag;
}