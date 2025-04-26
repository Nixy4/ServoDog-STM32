#include "kinematics.h"
#include "leg.h"
#include "elog.h"
#include "base.h"
#include "math.h"

typedef struct 
{
  double peried;
  double swingDuty;
  double swingTime;
  double swingHeight;
  double swingWidth;
  double x_base;
  double z_base;
  double t_delta;
  leg_t* rf;
  leg_t* rb;
  leg_t* lf;
  leg_t* lb;
} gait_config_t;

typedef struct
{
  coord_t swingCoord;
  coord_t supportCoord;
}gait_data_t;

typedef struct
{
  double peried;
  double swingDuty;
  double swingTime;
  double swingWidth;
  double swingHeight;
  double x_base;
  double z_base;
  double t_delta;

  leg_t* rfPtr;
  leg_t* rbPtr;
  leg_t* lfPtr;
  leg_t* lbPtr;

  coord_t rfCoord;
  coord_t rbCoord;
  coord_t lfCoord;
  coord_t lbCoord;

  volatile double t;
  volatile gait_data_t data;
} gait_t;

int gait_equation(gait_t* g);
int gait_update(gait_t* g);
