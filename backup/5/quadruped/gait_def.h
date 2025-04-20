#include "kinematics.h"
#include "leg.h"
#include "elog.h"
#include "math.h"

typedef struct 
{
  double periedTick;
  double deltaTick;
  double swingDuty;
  double swingTime;
  double swingWidth;
  double swingHeight;
  double xBase;
  double zBase;
} gait_config_t, gait_base_t;

typedef struct
{
  coord_t swingCoord;
  coord_t supportCoord;
} gait_data_t;

typedef void* (*gait_equation_fn)(double t);

// typedef struct
// {
//   double periedTick;
//   double swingDuty;
//   double swingTime;
//   double swingWidth;
//   double swingHeight;
//   double xBase;
//   double zBase;
//   double deltaTick;

//   leg_t* rfPtr;
//   leg_t* rbPtr;
//   leg_t* lfPtr;
//   leg_t* lbPtr;

//   coord_t rfCoord;
//   coord_t rbCoord;
//   coord_t lfCoord;
//   coord_t lbCoord;

//   volatile double t;
//   volatile gait_data_t data;
// } gait_t;

// int gait_equation(gait_t* g);
// int gait_update(gait_t* g);
