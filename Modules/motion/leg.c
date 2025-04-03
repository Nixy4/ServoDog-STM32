#include "stdio.h"
#include "math.h"
#include "leg.h"
#include "pac9685.h"
#include "servo.h"
#include "easing.h"
#include "elog.h"

const char* TAG = "LEG";

const double  L1   = 80.0f;               //大腿长度 mm
const double  L2   = 65.0f;               //小腿长度 mm 
const double  L3   = 15.0f;               //小腿舵机扭杆长度 mm
const double  L4   = 75.0f;               //拉杆长度 mm
const double  L5   = 20.0f;               //拉杆端点到小腿拐点的距离 mm
const double  L6   = 80.0f;               //小腿舵机到小腿拐点的距离 mm
const double  L7   = 15.0f;               //舵机与扭杆连接点到大腿的距离 mm
const double  L8   = 78.6f;  //大腿舵机到大腿拐点的距离 mm
const double  _2PI = 1.5707963267948966f;
const double  PI   = 3.141592653589793f;
const double  PI2  = 6.283185307179586f;
const double  T21  = 10.806914257755066f;

#define LA(x) (x)
#define RA(x) (180.f-x)

static inline double radians(double degree)
{
  return degree * PI / 180;
}

static inline double degrees(double radians)
{
  return radians * 180.f / PI;
}

static bool forward(double t1, double t2, double* x, double* z)
{
  t1 = radians(t1);
  t2 = radians(t2);
  double v1 = -2*L3*cos(t1);
  double v2 = L3*L3 - L4*L4;
  double v3 = v1*v1-4*v2;
  if(v3<0) return false;
  double v4 = (-v1+sqrt(v3))/2;
  double v5 = acos((v4*v4 -L8*L8 - L5*L5) / (-2*L8*L5))+T21;
  double v6 = sqrt(L2*L2+L1*L1-2*L2*L1*cos(PI-v5));
  double v7 = acos((L1*L1+v6*v6-L2*L2)/(2*L1*v6));
  *x = v6*cos(v7+t1);
  *z = v6*sin(v7+t1);
  return true;
}

static bool inverse(double x, double z, double* t1, double* t2)
{
  double v1 = sqrt(x*x+z*z);
  double v2 = acos((L2*L2-L1*L1-v1*v1)/(-2*L1*v1));
  double v3 = fabs(atan(z/x));
  double v4 = 0;
  if(z>0){
    if(x>0){
      v4 = v3 - v2;
    }else if(x<0){
      v4 = PI - v3 - v2;
    }else{
      v4 = _2PI-v2;
    }
  }else if(z<0){
    v4 = PI - v3 + v2;
  }else{
    v4 = PI - v2;
  }
  // double v5 = sqrt( L8*L8 + L5*L5 - 2*L8*L5 * cos( PI - acos( (v1*v1-L2*L2-L1*L1)/(-2*L1*L2))-T21 ) );
  //     v51 = L8**2       +L5**2       -2*L8*L5*math.cos( math.pi - math.acos( ( v1**2       -L2**2       -L1**2       ) / (-2*L1*L2) ) - T21 )
  double v50 = acos( (pow(v1,2.f) -pow(L2,2.f) -pow(L1,2.f) ) / (-2*L1*L2) );
  double v51 = PI - v50 - T21;
  double v52 = pow(L8,2.f) +pow(L5,2.f) -2*L8*L5*cos(v51) ;
  double v5 = sqrt(v52);
  elog_d(TAG, "v50:%.8f v51:%.8f v52:%.8f v5:%.8f", v50, v51, v52, v5);
  
  double v6 = acos( pow(L4,2.f) - pow(L3,2.f) - pow(v5,2.f) / (-2.f*L3*v5) );

  elog_d(TAG, "x:%.8f z:%.8f v1:%.8f v2:%.8f v3:%.8f v4:%.8f v51:%.8f v5:%.8f v6:%.8f", x, z, v1, v2, v3, v4, v51, v5, v6);

  *t1 = degrees(v4);
  *t2 = degrees(v6);
  return true;
}

int leg_init(leg_t* leg, const leg_config_t* cfg)
{
  leg->type = cfg->type;
  leg->thighAngle = cfg->thighAngle;
  leg->shankAngle = cfg->shankAngle;
  leg->x = 0;
  leg->z = 0;
  if(servo_init(&leg->thighServo, cfg->thighServoId, _easing_calc_Linear, cfg->thighOffset) != 0){
    return -1;
  }
  if(servo_init(&leg->shankhServo, cfg->shankServoId, _easing_calc_Linear, cfg->shankOffset) != 0){
    return -1;
  }
  return 0;
}

int leg_set_angle(leg_t* leg, double thighAngle, double shankAngle, bool syncCoord)
{
  if(syncCoord){
    double x, z;
    if(forward(leg->thighAngle, leg->shankAngle, &x, &z)){
      leg->x = x;
      leg->z = z;
    }else{
      return -2;
    }
  }
  leg->thighAngle = thighAngle;
  leg->shankAngle = shankAngle;
  if(leg->type&0x10) {
    thighAngle = LA(thighAngle);
    shankAngle = LA(shankAngle);
  }else{
    thighAngle = RA(thighAngle);
    shankAngle = RA(shankAngle);   
  }
  servo_set_angle(&leg->thighServo, thighAngle, false);
  servo_set_angle(&leg->shankhServo, shankAngle, false);
  return 0;
}

int leg_set_coord(leg_t* leg, double x, double z)
{
  double thighAngle;
  double shankAngle;
  if(inverse(x,z,&thighAngle,&shankAngle)==false) return -1;

  elog_d(TAG, "type:0x%X x:%.2f z:%.2f thigh:%.2f shanke:%.2f", leg->type, x, z, thighAngle, shankAngle);

  leg->thighAngle = thighAngle;
  leg->shankAngle = shankAngle;  
  leg->x = x;
  leg->z = z;

  if(leg->type&0x10) {
    thighAngle = LA(thighAngle);
    shankAngle = LA(shankAngle);
  }else{
    thighAngle = RA(thighAngle);
    shankAngle = RA(shankAngle);   
  }
  servo_set_angle(&leg->thighServo, thighAngle, false);
  servo_set_angle(&leg->shankhServo, shankAngle, false);
  return 0;
}