#include "kinematics.h"
#include "leg.h"
#include "elog.h"
#include "math.h"

typedef struct 
{
  QUAD_TYPE peried;
  QUAD_TYPE swingDuty;
  QUAD_TYPE swingTime;
  QUAD_TYPE swingHeight;
  QUAD_TYPE swingWidth;
  QUAD_TYPE x_base;
  QUAD_TYPE z_base;
  QUAD_TYPE t_delta;
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
  QUAD_TYPE peried;
  QUAD_TYPE swingDuty;
  QUAD_TYPE swingTime;
  QUAD_TYPE swingWidth;
  QUAD_TYPE swingHeight;
  QUAD_TYPE x_base;
  QUAD_TYPE z_base;
  QUAD_TYPE t_delta;

  leg_t* rfPtr;
  leg_t* rbPtr;
  leg_t* lfPtr;
  leg_t* lbPtr;

  coord_t rfCoord;
  coord_t rbCoord;
  coord_t lfCoord;
  coord_t lbCoord;

  volatile QUAD_TYPE t;
  volatile gait_data_t data;
} gait_t;

int gait_equation(gait_t* g);
int gait_update(gait_t* g);
