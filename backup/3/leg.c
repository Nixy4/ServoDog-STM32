#include "stdio.h"
#include "leg.h"
#include "pac9685.h"
#include "elog.h"

#define R(x) (x)
#define L(x) (180.f-x)

#define LR_BIT    0x80
#define LR_LEFT   0x00
#define LR_RIGHT  0x80

#define FB_BIT    0x40
#define FB_FRONT  0x00
#define FB_BACK   0x40

#define TS_BIT    0x20
#define TS_THIGHS 0x00
#define TS_CALVES 0x20

#define SET_LR_BIT(leg,lr) (leg = (leg & ~LEG_LR_BIT) | lr)
#define SET_FB_BIT(leg,fb) (leg = (leg & ~LEG_FB_BIT) | fb)
#define SET_TS_BIT(leg,ts) (leg = (leg & ~LEG_TS_BIT) | ts)
#define GET_LR_BIT(leg)    (leg & LEG_LR_BIT)
#define GET_FB_BIT(leg)    (leg & LEG_FB_BIT)
#define GET_TS_BIT(leg)    (leg & LEG_TS_BIT)

#define LEG_ID_LFT (LR_LEFT | FB_FRONT | TS_THIGHS)
#define LEG_ID_LFC (LR_LEFT | FB_FRONT | TS_CALVES)
#define LEG_ID_LBT (LR_LEFT | FB_BACK  | TS_THIGHS)
#define LEG_ID_LBC (LR_LEFT | FB_BACK  | TS_CALVES)

#define LEG_ID_RFT (LR_RIGHT| FB_FRONT | TS_THIGHS)
#define LEG_ID_RFC (LR_RIGHT| FB_FRONT | TS_CALVES)
#define LEG_ID_RBT (LR_RIGHT| FB_BACK  | TS_THIGHS)
#define LEG_ID_RBC (LR_RIGHT| FB_BACK  | TS_CALVES)

#define SERVO_ID_ERR

#define TMAX 170
#define TMIN 0
#define CMAX 100
#define CMIN 0

static uint32_t map(uint32_t leg)
{
  switch(leg)
  {
    case LEG_ID_LFT: return SERVO_ID_LFT;
    case LEG_ID_LFC: return SERVO_ID_LFC;
    case LEG_ID_LBT: return SERVO_ID_LBT;
    case LEG_ID_LBC: return SERVO_ID_LBC;
    case LEG_ID_RFT: return SERVO_ID_RFT;
    case LEG_ID_RFC: return SERVO_ID_RFC;
    case LEG_ID_RBT: return SERVO_ID_RBT;
    case LEG_ID_RBC: return SERVO_ID_RBC;
         default   : return 0xFFFFFFFF;
  }
}

typedef struct
{
  double thigh;
  double shank; 
}leg_angle_t;

typedef struct
{
  int x;
  int y;

  servo_t* st;
  servo_t* sc;
}leg_t;

leg_t rf;
leg_t rb;
leg_t lf;
leg_t lb;

void leg_init_all()
{
  pca9685_set_freq(50);
  //*左侧舵机初始化
  pca9685_set_angle(SERVO_LEDX_LFT, 180);
  pca9685_set_angle(SERVO_LEDX_LFC, 180);
  pca9685_set_angle(SERVO_LEDX_LBT, 180);
  pca9685_set_angle(SERVO_LEDX_LBC, 180);

  lf.st = servo_create(SERVO_ID_LFT, _easing_calc_Linear,0);
  lf.sc = servo_create(SERVO_ID_LFC, _easing_calc_Linear,0);
  lb.st = servo_create(SERVO_ID_LBT, _easing_calc_Linear,0);
  lb.sc = servo_create(SERVO_ID_LBC, _easing_calc_Linear,0);

  lf.st->fCurr = 180;
  lf.sc->fCurr = 180;
  lb.st->fCurr = 180;
  lb.sc->fCurr = 180;

  //*右侧舵机初始化
  pca9685_set_angle(SERVO_LEDX_RFT, 0);
  pca9685_set_angle(SERVO_LEDX_RFC, 0);
  pca9685_set_angle(SERVO_LEDX_RBT, 0);
  pca9685_set_angle(SERVO_LEDX_RBC, 0);

  rf.st = servo_create(SERVO_ID_RFT, _easing_calc_Linear,0);
  rf.sc = servo_create(SERVO_ID_RFC, _easing_calc_Linear,0);
  rb.st = servo_create(SERVO_ID_RBT, _easing_calc_Linear,0);
  rb.sc = servo_create(SERVO_ID_RBC, _easing_calc_Linear,0);

  HAL_Delay(500);
}

const double  L1 = 80.0f;    //大腿长度 mm
const double  L2 = 65.0f;    //小腿长度 mm 
const double  L3 = 15.0f;    //小腿舵机扭杆长度 mm
const double  L4 = 75.0f;	   //拉杆长度 mm
const double  L5 = 20.0f;    //拉杆端点到小腿拐点的距离 mm
const double  L6 = 80.0f;    //小腿舵机到小腿拐点的距离 mm
const double  PI = 3.1415926535f;

static int acalc(double x,double y,double *A1, double *A2)
{
  //计算小腿角度 余弦定理
  double a2 = PI - acos((x*x + y*y - L1*L1 - L2*L2) / (-2*L1*L2));
  //计算小腿舵机角度
  double Lx = sqrt(L6*L6+L5*L5 - 2* L6*L5*cos(a2));
  printf("lx=%.2f,a2=%.2f\n",Lx,a2);

  if(Lx > (L3+L4) || Lx < L4-L3)
      return -1;
  double ax = acos((Lx*Lx+L3*L3 - L4*L4)/(2*L3*Lx));

  *A2 = 180*ax/PI;
  
  //计算大腿角度
  double tmp = acos((L1*L1+x*x+y*y-L2*L2)/(2*L1*sqrt(x*x+y*y)));
  double a1;
  if (x>0)
      a1=fabs(atan(y/x))-tmp;
  else if(x < 0)
      a1=PI-fabs(atan(y/x))-tmp;
  else
      a1=PI - 1.5707f - tmp;

  *A1 = 180*a1/PI; 
  return 0;
}

static double thigh_radin(double x, double y)
{
  double fai = acos((x*x + y*y + L1*L1 - L2*L2)/(2*L1*sqrt(x*x + y*x)));
  return fai;
}

int leg_update(leg_t* leg)
{
  return 0 | servo_update(leg->st) | servo_update(leg->sc);
}